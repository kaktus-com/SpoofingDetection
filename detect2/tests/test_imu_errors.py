import json
from pathlib import Path
import re
import shutil

import numpy as np
from numpy.testing import assert_allclose
import pytest

from SpoofingDetection.detect2.geometry import quaternion_to_matrix
from SpoofingDetection.detect2.imu_errors import ImuErrors, perturb_imu
from SpoofingDetection.detect2.imu_robustness import Condition, run_sweep
from SpoofingDetection.detect2.motion import ImuSeries, MotionUnavailable, estimate_antenna_displacement, load_imu
from SpoofingDetection.detect2.motion_evaluation import remove_polynomial
from SpoofingDetection.detect2.replay import replay_dataset
from SpoofingDetection.detect2.synthetic.carrier import CarrierConfig
from SpoofingDetection.detect2.synthetic.generate_carrier import generate_datasets
from SpoofingDetection.detect2.synthetic.motions import MotionConfig, generate_motion


def imu_source(scenario='combined', duration=3):
    motion = generate_motion(MotionConfig(scenario, duration_s=duration))
    return ImuSeries(motion.time_s, motion.specific_force_body_m_s2, motion.angular_rate_body_rad_s,
                     motion.quaternion_wxyz, motion.config.lever_arm_body_m, [0, 0, -9.80665], motion.config.rate_hz)


def test_zero_errors_preserve_measurements_and_nonzero_errors_do_not_mutate_source():
    imu = imu_source()
    before = [value.copy() for value in [imu.time_s, imu.specific_force_body_m_s2, imu.quaternion_body_to_enu_wxyz]]
    ideal = perturb_imu(imu, ImuErrors())
    assert_allclose(ideal.specific_force_body_m_s2, before[1], atol=0)
    assert_allclose(ideal.linear_acceleration_enu_m_s2, imu.linear_acceleration_enu_m_s2, atol=1e-14)
    changed = perturb_imu(imu, ImuErrors(.1, (.2, 0, 0), (0, 1, 0), .02))
    for actual, expected in zip([imu.time_s, imu.specific_force_body_m_s2, imu.quaternion_body_to_enu_wxyz], before):
        assert_allclose(actual, expected, atol=0)
    assert not np.shares_memory(changed.specific_force_body_m_s2, imu.specific_force_body_m_s2)


def test_noise_is_repeatable_common_across_levels_and_has_per_sample_sigma():
    imu = imu_source('stationary', duration=50)
    a = perturb_imu(imu, ImuErrors(accel_noise_sigma_m_s2=.05), 123, 'stationary')
    b = perturb_imu(imu, ImuErrors(accel_noise_sigma_m_s2=.1), 123, 'stationary')
    repeat = perturb_imu(imu, ImuErrors(accel_noise_sigma_m_s2=.05), 123, 'stationary')
    assert_allclose(a.specific_force_body_m_s2, repeat.specific_force_body_m_s2, atol=0)
    noise = a.specific_force_body_m_s2-imu.specific_force_body_m_s2
    assert_allclose(b.specific_force_body_m_s2-imu.specific_force_body_m_s2, 2*noise, atol=2e-15)
    assert_allclose(noise.std(axis=0), .05, rtol=.04)
    assert np.max(np.abs(noise.mean(axis=0))) < .002
    different = perturb_imu(imu, ImuErrors(accel_noise_sigma_m_s2=.05), 123, 'another_motion')
    assert not np.allclose(a.specific_force_body_m_s2, different.specific_force_body_m_s2)


def test_body_fixed_tilt_changes_gravity_removal_with_correct_rotation_direction():
    imu = imu_source('stationary')
    angle = np.radians(2.)
    tilt = np.array([[np.cos(angle), 0., np.sin(angle)], [0., 1., 0.], [-np.sin(angle), 0., np.cos(angle)]])
    changed = perturb_imu(imu, ImuErrors(attitude_error_body_rotvec_deg=(0, 2, 0)))
    expected_rotation = quaternion_to_matrix(imu.quaternion_body_to_enu_wxyz)@tilt
    assert_allclose(quaternion_to_matrix(changed.quaternion_body_to_enu_wxyz), expected_rotation, atol=5e-16)
    assert_allclose(np.linalg.norm(changed.linear_acceleration_enu_m_s2, axis=1), 2*9.80665*np.sin(angle/2), atol=4e-15)


def test_constant_body_bias_is_not_generally_removed_during_rotation():
    errors = []
    query = np.arange(20)/20+1
    for scenario in ['three_axis', 'rotation_only']:
        imu = imu_source(scenario)
        biased = perturb_imu(imu, ImuErrors(accel_bias_body_m_s2=(.5, 0, 0)))
        difference = (estimate_antenna_displacement(biased, query).antenna_displacement_m
                      -estimate_antenna_displacement(imu, query).antenna_displacement_m)
        errors.append(np.linalg.norm(remove_polynomial(query, difference))/np.sqrt(len(query)))
    assert errors[0] < 1e-15
    assert errors[1] > 1e-5


def test_time_offset_tags_inputs_without_shifting_carrier_and_requires_coverage():
    imu = imu_source()
    shifted = perturb_imu(imu, ImuErrors(time_offset_s=.02))
    assert_allclose(shifted.time_s, imu.time_s+.02, atol=0)
    assert_allclose(shifted.specific_force_body_m_s2, imu.specific_force_body_m_s2, atol=0)
    with pytest.raises(MotionUnavailable, match='coverage_missing'):
        estimate_antenna_displacement(shifted, np.arange(20)/20)
    estimate_antenna_displacement(shifted, np.arange(20)/20+1)


@pytest.mark.parametrize('kwargs', [{'accel_noise_sigma_m_s2': -.1}, {'time_offset_s': np.nan},
                                    {'accel_bias_body_m_s2': [1, 2]}, {'attitude_error_body_rotvec_deg': [0, np.inf, 0]}])
def test_invalid_error_settings_are_rejected(kwargs):
    with pytest.raises(ValueError):
        ImuErrors(**kwargs)


def make_dataset(tmp_path):
    source = tmp_path/'measurements'
    generate_datasets(source, [MotionConfig(name, duration_s=3) for name in ['stationary', 'combined']],
                      CarrierConfig(), plots=False)
    return source


def conditions():
    return [Condition('ideal', 'ideal', 0, '', ImuErrors()),
            Condition('timing', 'timing', 20, 'ms', ImuErrors(time_offset_s=.02)),
            Condition('large_tilt', 'attitude', 10, 'deg', ImuErrors(attitude_error_body_rotvec_deg=(0, 10, 0)))]


def test_sweep_freezes_every_condition_before_truth_and_keeps_matched_windows(tmp_path, monkeypatch):
    source, output = make_dataset(tmp_path), tmp_path/'sweep'
    original = Path.open
    source_imu = source/'combined/inputs/imu.csv'
    before = source_imu.read_bytes()
    def guard(path, *args, **kwargs):
        if 'truth' in path.parts:
            assert (output/'run.json').is_file()
            for condition in conditions():
                for scenario in ['stationary', 'combined']:
                    for case in ['benign', 'spoofed']:
                        assert (output/condition.name/scenario/case/'windows.csv').is_file()
        return original(path, *args, **kwargs)
    monkeypatch.setattr(Path, 'open', guard)
    rows = run_sweep(source, output, conditions=conditions(), plots=False)
    totals = [row for row in rows if row['scenario'] == 'all']
    assert all(row['total_windows'] == 4 for row in totals)
    ideal = next(row for row in totals if row['condition'] == 'ideal')
    assert ideal['correct_conclusive'] == 2
    assert ideal['inconclusive_windows'] == 2
    assert ideal['decision_changes_from_ideal'] == 0
    assert source_imu.read_bytes() == before
    for condition in conditions():
        run = json.loads((output/condition.name/'run.json').read_text())
        assert run['edge_trim_s'] == 1
        assert run['imu_input_errors']['seed'] == 42
        for scenario in ['stationary', 'combined']:
            saved = load_imu(output/condition.name/'inputs'/scenario/'imu.csv', source/scenario/'inputs/frame.json', 200)
            assert_allclose(saved.time_s, np.arange(601)/200+condition.errors.time_offset_s, atol=1e-15)
        for filename in ['report.html', 'motion_report.html']:
            html = (output/condition.name/filename).read_text()
            assert '__INPUT_NOTE__' not in html
            for link in re.findall(r'href="([^"]+)"', html):
                assert (output/condition.name/link).is_file()
    for link in re.findall(r'href="([^"]+)"', (output/'report.html').read_text()):
        assert (output/link).is_file()


def test_perturbed_prediction_and_sweep_need_no_truth(tmp_path, monkeypatch):
    source = make_dataset(tmp_path)
    for name in ['stationary', 'combined']:
        shutil.rmtree(source/name/'truth')
    original = Path.open
    def guard(path, *args, **kwargs):
        assert 'truth' not in path.parts
        return original(path, *args, **kwargs)
    monkeypatch.setattr(Path, 'open', guard)
    rows = run_sweep(source, tmp_path/'prediction_only', conditions=conditions()[:2], plots=False, evaluate=False)
    assert rows == []
    with pytest.raises(ValueError, match='require IMU'):
        replay_dataset(source, tmp_path/'invalid', oracle=True, imu_errors=ImuErrors(.1))
    with pytest.raises(ValueError, match='no complete'):
        replay_dataset(source, tmp_path/'trimmed', imu=True, trim_s=2.)
