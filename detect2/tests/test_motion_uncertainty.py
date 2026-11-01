from dataclasses import replace
from pathlib import Path
import shutil

import numpy as np
from numpy.testing import assert_allclose
import pytest

from SpoofingDetection.detect2.detector import DetectorConfig
from SpoofingDetection.detect2.imu_errors import ImuErrors, perturb_imu
from SpoofingDetection.detect2.motion import ImuSeries, estimate_antenna_displacement, integrate_linear_acceleration
from SpoofingDetection.detect2.motion_uncertainty import (
    integration_matrix, displacement_covariance, noise_norm_bound, assess_motion_noise, attach_motion_check,
)
from SpoofingDetection.detect2.replay import replay_dataset, run_windows
from SpoofingDetection.detect2.synthetic.carrier import CarrierConfig
from SpoofingDetection.detect2.synthetic.generate_carrier import generate_datasets
from SpoofingDetection.detect2.synthetic.motions import MotionConfig, generate_motion


def source(scenario='combined', sigma=.0):
    motion = generate_motion(MotionConfig(scenario, duration_s=3, rate_hz=211))
    return ImuSeries(motion.time_s, motion.specific_force_body_m_s2, motion.angular_rate_body_rad_s,
                     motion.quaternion_wxyz, motion.config.lever_arm_body_m, [0, 0, -9.80665], 211, sigma)


def test_integration_operator_matches_independent_segment_integration_at_irregular_epochs():
    imu = source()
    query = .013+np.arange(21)/23
    weights = integration_matrix(imu.time_s, query)
    estimate = estimate_antenna_displacement(imu, query)
    assert_allclose(weights@imu.linear_acceleration_enu_m_s2, estimate.imu_displacement_m, atol=3e-15)
    elapsed = query-query[0]
    assert_allclose(weights.sum(axis=1), .5*elapsed**2, atol=2e-16)
    # Analytic acceleration a(t)=t, including a nonzero window origin.
    assert_allclose(weights@imu.time_s, .5*query[0]*elapsed**2+elapsed**3/6, atol=2e-16)


def test_covariance_matches_monte_carlo_and_preserves_boundary_and_epoch_correlations():
    # Independent numerical integration of 2,000 trajectories on a merged grid,
    # rather than sampling from the covariance or using its integration matrix.
    source_t = np.arange(101)/100
    query = .013+np.arange(17)/23
    sigma = .2
    rng = np.random.default_rng(8123)
    raw = sigma*rng.standard_normal((len(source_t), 2000))
    grid = np.unique(np.r_[query, source_t[(source_t > query[0])&(source_t < query[-1])]])
    right = np.searchsorted(source_t, grid)
    left = right-1
    fraction = ((grid-source_t[left])/(source_t[right]-source_t[left]))[:, None]
    acceleration = raw[left]*(1-fraction)+raw[right]*fraction
    positions, _ = integrate_linear_acceleration(grid, acceleration)
    at_query = positions[np.searchsorted(grid, query)]
    covariance, observable = displacement_covariance(source_t, query, sigma)
    p = np.column_stack([np.ones(len(query)), query, query**2])
    q = np.linalg.qr(p, mode='reduced')[0]
    projected = at_query-q@(q.T@at_query)
    for actual, predicted in [(np.cov(at_query), covariance), (np.cov(projected), observable)]:
        assert np.linalg.norm(actual-predicted)/np.linalg.norm(predicted) < .08
    assert covariance[-1, -2]/np.sqrt(covariance[-1, -1]*covariance[-2, -2]) > .9
    assert_allclose(covariance[0], 0., atol=0)
    # Treating interpolated boundary samples as independent is not done here.
    assert np.linalg.matrix_rank(observable, tol=1e-15) <= len(query)-3
    assert_allclose(displacement_covariance(source_t, query, 2*sigma)[1], 4*observable, atol=1e-20)


def test_noise_bound_has_expected_scaling_and_covers_gaussian_noise_draws():
    _, covariance = displacement_covariance(np.arange(201)/200, np.arange(20)/20, .5)
    bound = noise_norm_bound(covariance, .001)
    assert_allclose(noise_norm_bound(4*covariance, .001), 2*bound, rtol=1e-14)
    eigenvalues, vectors = np.linalg.eigh(covariance)
    factor = vectors*np.sqrt(np.maximum(eigenvalues, 0.))
    samples = np.random.default_rng(8124).standard_normal((10000, 20, 3))
    noise = np.einsum('ij,bjk->bik', factor, samples)
    assert np.mean(np.linalg.norm(noise, axis=(1, 2)) > bound) <= .001
    assert noise_norm_bound(np.zeros((20, 20)), .001) == 0


@pytest.mark.parametrize('scenario', ['stationary', 'constant_velocity', 'east_oscillation', 'circle', 'circle_coning'])
def test_degenerate_motion_stays_below_uncertainty_floor_across_seeds(scenario):
    imu = source(scenario)
    times = 1+np.arange(20)/20
    for seed in range(10):
        estimate = estimate_antenna_displacement(perturb_imu(imu, ImuErrors(accel_noise_sigma_m_s2=.5), seed, scenario), times)
        check = assess_motion_noise(estimate, DetectorConfig())
        assert check['applied'] and not check['passed']


@pytest.mark.parametrize('scenario,sigma,passes', [('three_axis', .5, True), ('combined', .5, True),
                                                  ('rotation_only', .05, True), ('rotation_only', .5, False)])
def test_informative_motion_and_conservative_weak_axis_tradeoff(scenario, sigma, passes):
    imu = source(scenario)
    for seed in [42, 43, 44]:
        estimate = estimate_antenna_displacement(perturb_imu(imu, ImuErrors(accel_noise_sigma_m_s2=sigma), seed, scenario), 1+np.arange(20)/20)
        assert assess_motion_noise(estimate, DetectorConfig())['passed'] == passes


def test_replay_gate_matches_paired_ablation_and_never_changes_passed_fits(tmp_path):
    root = tmp_path/'observations'
    generate_datasets(root, [MotionConfig(name, duration_s=3) for name in ['stationary', 'combined']], CarrierConfig(), plots=False)
    errors = ImuErrors(accel_noise_sigma_m_s2=.5)
    ungated, _ = replay_dataset(root, tmp_path/'before', imu=True, imu_errors=errors, trim_s=1,
                               evaluate=False, plots=False, config=DetectorConfig(imu_noise_gate_enabled=False))
    gated, _ = replay_dataset(root, tmp_path/'after', imu=True, imu_errors=errors, trim_s=1, evaluate=False, plots=False)
    for before, after in zip(ungated, gated):
        record = before['records'][0]
        original = record.result.diagnostics.copy()
        check = assess_motion_noise(record.motion_estimate, DetectorConfig())
        paired = attach_motion_check(record.result, check)
        direct = after['records'][0].result
        assert (paired.decision, paired.reason, paired.gamma) == (direct.decision, direct.reason, direct.gamma)
        assert record.result.diagnostics == original
        if check['passed']:
            assert paired.benign is record.result.benign and paired.spoofed is record.result.spoofed
        else:
            assert paired.benign is None and paired.spoofed is None and paired.gamma is None


def test_noisy_gated_predictions_need_no_truth_and_zero_noise_preserves_baseline(tmp_path, monkeypatch):
    root = tmp_path/'observations'
    generate_datasets(root, [MotionConfig('stationary', duration_s=3)], CarrierConfig(), plots=False)
    shutil.rmtree(root/'stationary/truth')
    original = Path.open
    def guard(path, *args, **kwargs):
        assert 'truth' not in path.parts
        return original(path, *args, **kwargs)
    monkeypatch.setattr(Path, 'open', guard)
    runs, _ = replay_dataset(root, tmp_path/'gated', imu=True, imu_errors=ImuErrors(accel_noise_sigma_m_s2=.5),
                              trim_s=1, evaluate=False, plots=False)
    assert all(run['records'][0].result.reason == 'imu_motion_below_noise_floor' for run in runs)
    for run in runs:
        clean_imu = replace(run['imu'], accel_noise_sigma_m_s2=0.)
        # Gate bypass at declared zero noise preserves the entire existing fit path.
        with_gate = run_windows(run['data'], imu=clean_imu)
        without_gate = run_windows(run['data'], DetectorConfig(imu_noise_gate_enabled=False), imu=clean_imu)
        assert [(r.result.decision, r.result.gamma) for r in with_gate] == [(r.result.decision, r.result.gamma) for r in without_gate]


def test_invalid_covariance_noise_and_probability_are_rejected():
    for sigma in [-1, np.nan]:
        with pytest.raises(ValueError):
            displacement_covariance(np.arange(10), np.arange(5), sigma)
    for alpha in [0, 1, np.nan]:
        with pytest.raises(ValueError):
            DetectorConfig(imu_noise_tail_probability=alpha)
    with pytest.raises(ValueError, match='positive semidefinite'):
        noise_norm_bound(np.diag([1., -1.]), .001)
