import csv
from dataclasses import replace
import json
import re

import numpy as np
from numpy.testing import assert_allclose, assert_array_equal
import pytest

from SpoofingDetection.detect2.geometry import GPS_L1_WAVELENGTH_M, LocalFrame
from SpoofingDetection.detect2.synthetic.carrier import (
    CarrierConfig, generate_pair, pair_metrics,
)
from SpoofingDetection.detect2.synthetic.generate_carrier import generate_datasets
from SpoofingDetection.detect2.synthetic.motions import MotionConfig


def test_known_east_motion_has_correct_sign_wavelength_and_directions():
    config = CarrierConfig(noise_sigma_cycles=0, add_trend=False,
                            spoofer_azimuth_deg=90, spoofer_elevation_deg=0)
    pair = generate_pair(MotionConfig('east_oscillation', duration_s=1.), config)
    displacement_east = .05*np.sin(4*np.pi*pair.time_s)
    angles = np.radians(pair.geometry['azimuth_deg'])
    elevations = np.radians(pair.geometry['elevation_deg'])
    for i in range(len(pair.signal_ids)):
        expected = -displacement_east*np.cos(elevations[i])*np.sin(angles[i])/GPS_L1_WAVELENGTH_M
        assert_allclose(pair.cases['benign'].phase_cycles[:, i], expected, atol=2e-16)
        assert_allclose(pair.cases['spoofed'].phase_cycles[:, i],
                        -displacement_east/GPS_L1_WAVELENGTH_M, atol=2e-16)


def test_paired_cases_share_motion_and_trends_but_not_noise():
    pair = generate_pair(MotionConfig('three_axis'))
    benign, spoofed = pair.cases['benign'], pair.cases['spoofed']
    assert_array_equal(benign.slow_phase_cycles, spoofed.slow_phase_cycles)
    assert not np.array_equal(benign.noise_cycles, spoofed.noise_cycles)
    assert np.all((pair.trend_coefficients[0] >= 1e8) & (pair.trend_coefficients[0] < 1.2e8))
    # Every spoofed signal has the SAME motion-induced phase, not the same raw phase.
    assert_allclose(np.ptp(spoofed.geometric_phase_cycles, axis=1), 0., atol=0.)
    assert np.std(spoofed.phase_cycles[0]) > 1e5
    assert np.max(np.ptp(benign.geometric_phase_cycles, axis=1)) > .1
    metrics = pair_metrics(pair)
    assert metrics['displacement_rank'] == 3
    assert metrics['spoofed']['geometric_cross_signal_spread_rms_cycles'] < 1e-16
    assert metrics['benign']['geometric_cross_signal_spread_rms_cycles'] > .01


def test_quadratic_trend_uses_seconds_and_half_acceleration_factor():
    pair = generate_pair(MotionConfig('stationary', duration_s=2.), CarrierConfig(noise_sigma_cycles=0))
    y = pair.cases['benign'].phase_cycles
    dt = 1/pair.config.gnss_rate_hz
    second_derivative = np.diff(y, n=2, axis=0)/dt**2
    assert_allclose(second_derivative, np.broadcast_to(pair.trend_coefficients[2], second_derivative.shape),
                    atol=2e-5, rtol=0)
    assert_array_equal(y, pair.cases['spoofed'].phase_cycles)


def test_noise_stationary_variance_correlation_and_case_independence():
    config = CarrierConfig(noise_sigma_cycles=.03, noise_rho=.7, add_trend=False)
    pair = generate_pair(MotionConfig('stationary', duration_s=20), config)
    stats = pair_metrics(pair)
    for name in ['benign', 'spoofed']:
        assert_allclose(stats[name]['realized_noise_rms_cycles'], .03, rtol=.10)
        assert abs(stats[name]['realized_noise_lag1_correlation']-.7) < .05
    correlation = np.corrcoef(pair.cases['benign'].noise_cycles.ravel(),
                              pair.cases['spoofed'].noise_cycles.ravel())[0, 1]
    assert abs(correlation) < .12


def test_same_seed_reproduces_and_different_seed_changes_realizations():
    motion = MotionConfig('three_axis', duration_s=.5)
    first = generate_pair(motion, CarrierConfig(seed=42))
    generate_pair(MotionConfig('circle', duration_s=.5), CarrierConfig(seed=42))
    repeated = generate_pair(motion, CarrierConfig(seed=42))
    changed = generate_pair(motion, CarrierConfig(seed=43))
    for name in ['benign', 'spoofed']:
        assert_array_equal(first.cases[name].phase_cycles, repeated.cases[name].phase_cycles)
        assert not np.array_equal(first.cases[name].noise_cycles, changed.cases[name].noise_cycles)
    assert not np.array_equal(first.trend_coefficients, changed.trend_coefficients)


def test_noiseless_stationary_control_has_valid_zero_phases_and_no_motion():
    pair = generate_pair(MotionConfig('stationary'), CarrierConfig(noise_sigma_cycles=0, add_trend=False))
    for case in pair.cases.values():
        assert_array_equal(case.phase_cycles, np.zeros_like(case.phase_cycles))
    metrics = pair_metrics(pair)
    assert metrics['displacement_rank'] == 0
    assert metrics['benign']['realized_noise_lag1_correlation'] is None


def test_common_and_noncommensurate_time_grids_evaluate_same_motion():
    pair = generate_pair(MotionConfig('combined', duration_s=1))
    assert_allclose(pair.imu_motion.time_s[::10], pair.time_s, atol=1e-16)
    assert_allclose(pair.imu_motion.antenna_position_m[::10], pair.gnss_motion.antenna_position_m, atol=1e-16)
    # Non-integer rate ratios remain valid; the GNSS grid is NOT rounded onto IMU indices.
    other = generate_pair(MotionConfig('east_oscillation', duration_s=1, rate_hz=211),
                           CarrierConfig(gnss_rate_hz=23, add_trend=False, noise_sigma_cycles=0))
    assert_allclose(other.time_s, np.arange(24)/23, atol=0)
    assert_allclose(other.gnss_motion.body_position_m[:, 0], .05*np.sin(4*np.pi*other.time_s), atol=1e-16)


@pytest.mark.parametrize('kwargs', [dict(gnss_rate_hz=0), dict(noise_sigma_cycles=-1),
                                   dict(noise_rho=1), dict(noise_rho=-.1), dict(seed=-1),
                                   dict(seed=1.5), dict(noise_sigma_cycles=float('nan')),
                                   dict(spoofer_elevation_deg=91), dict(trend_rate_span_cycles_s=-1)])
def test_bad_carrier_configuration(kwargs):
    with pytest.raises(ValueError):
        CarrierConfig(**kwargs)


def test_invalid_rates_are_rejected_before_writing_files(tmp_path):
    target = tmp_path/'not_created'
    with pytest.raises(ValueError):
        generate_datasets(target, [MotionConfig('three_axis')], CarrierConfig(gnss_rate_hz=1), plots=False)
    assert not target.exists()
    with pytest.raises(ValueError):
        generate_pair(MotionConfig('three_axis', rate_hz=20), CarrierConfig(gnss_rate_hz=30))


def test_export_boundary_rows_reproducibility_and_standalone_report(tmp_path):
    motion = MotionConfig('three_axis', duration_s=.5)
    output = tmp_path/'first'
    generate_datasets(output, [motion], frame=LocalFrame(), plots=False)
    folder = output/'three_axis'
    manifest = json.loads((folder/'manifest.json').read_text())
    for key in ['imu', 'satellite_geometry', 'frame']:
        assert (folder/manifest[key]).is_file()
    with (folder/manifest['imu']).open() as stream:
        imu = list(csv.DictReader(stream))
    assert len(imu) == 101
    required = {'time_s', 'signal_id', 'carrier_phase_cycles', 'phase_sigma_cycles',
                'carrier_valid', 'lock_time_s', 'arc_id'}
    for case in ['benign', 'spoofed']:
        with (folder/manifest['carrier_files'][case]).open() as stream:
            reader = csv.DictReader(stream)
            assert set(reader.fieldnames) == required
            rows = list(reader)
        assert len(rows) == 11*8
        assert all(row['carrier_valid'] == '1' for row in rows)
        assert all(row['arc_id'] == '0' for row in rows)
        times = np.array([float(row['time_s']) for row in rows])
        assert_allclose(times.reshape(-1, 8)[:, 0], np.arange(11)/20)
        with (folder/'truth'/f'{case}_components.csv').open() as stream:
            truth = list(csv.DictReader(stream))
        reconstructed = np.array([float(row['slow_phase_cycles'])+float(row['geometric_phase_cycles'])
                                   +float(row['noise_cycles']) for row in truth])
        assert_allclose([float(row['carrier_phase_cycles']) for row in rows], reconstructed, rtol=0, atol=1e-8)
    with (folder/'inputs/satellite_geometry.csv').open() as stream:
        geometry = list(csv.DictReader(stream))
    assert len(geometry) == 8
    assert not any('actual' in key or 'spoof' in key for key in geometry[0])
    with (folder/'truth/labels.csv').open() as stream:
        labels = list(csv.DictReader(stream))
    assert len(labels) == 22
    assert {row['is_spoofed'] for row in labels} == {'0', '1'}
    html = (output/'report.html').read_text()
    assert '<script src=' not in html and '__DATA__' not in html
    payload = json.loads(re.search(r'<script id="carrier-data" type="application/json">(.*?)</script>', html, re.S).group(1))
    assert len(payload['three_axis']['time']) == 11
    for target in re.findall(r'href="([^"]+)"', html):
        assert (output/target).is_file(), target
    # Selecting only this scenario versus a differently ordered set must produce identical observations.
    second = tmp_path/'second'
    generate_datasets(second, [replace(motion, scenario='stationary'), motion], plots=False)
    for case in ['benign', 'spoofed']:
        assert (second/'three_axis'/case/'carrier.csv').read_bytes() == (folder/case/'carrier.csv').read_bytes()
