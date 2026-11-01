from dataclasses import replace

import numpy as np
from numpy.testing import assert_allclose
import pytest
from scipy.spatial.transform import Rotation

from SpoofingDetection.detect2.detector import (
    DetectorConfig, detect_window, rotation_residual_jacobian, solve_unit_direction,
)
from SpoofingDetection.detect2.schema import WindowInput
from SpoofingDetection.detect2.synthetic.carrier import CarrierConfig, generate_pair
from SpoofingDetection.detect2.synthetic.motions import MotionConfig


def example(scenario='three_axis', case='benign', sigma=.01, rho=0.):
    pair = generate_pair(MotionConfig(scenario, duration_s=1),
                          CarrierConfig(noise_sigma_cycles=sigma, noise_rho=rho))
    keep = pair.time_s < 1
    return WindowInput(pair.time_s[keep], pair.cases[case].phase_cycles[keep],
                        pair.gnss_motion.antenna_position_m[keep], pair.geometry['los_enu'],
                        np.full(8, .19029367279836487), sigma, tuple(pair.signal_ids))


@pytest.mark.parametrize('scenario', ['three_axis', 'rotation_only', 'combined'])
@pytest.mark.parametrize('case', ['benign', 'spoofed'])
@pytest.mark.parametrize('sigma', [0., .01])
def test_clean_and_noisy_full_rank_cases(scenario, case, sigma):
    result = detect_window(example(scenario, case, sigma))
    assert result.decision == case
    assert (result.gamma > 0) == (case == 'benign')
    assert result.diagnostics['sigma_floor_applied'] == (sigma == 0)
    assert result.benign.diagnostics['determinant'] == pytest.approx(1., abs=1e-14)
    assert result.spoofed.diagnostics['unit_norm'] == pytest.approx(1., abs=1e-14)
    assert result.benign.diagnostics['agreeing_best_starts'] >= 2
    assert_allclose(result.gamma, result.spoofed.total_cost-result.benign.total_cost, rtol=1e-10, atol=1e-6)
    chosen = result.benign if case == 'benign' else result.spoofed
    if sigma == 0:
        assert np.max(np.abs(chosen.residual_cycles)) < 5e-8
    else:
        assert np.sqrt(np.mean(chosen.residual_cycles**2)) < .015


@pytest.mark.parametrize('case', ['benign', 'spoofed'])
def test_common_frame_rotation_and_polynomial_invariance(case):
    original = example(case=case)
    baseline = detect_window(original)
    rotation = Rotation.from_euler('xyz', [35, -61, 108], degrees=True).as_matrix()
    t = original.time_s[:, None]
    rng = np.random.default_rng(456)
    nuisance = rng.uniform(-1000, 1000, (1, 8)) + t*rng.uniform(-3000, 3000, (1, 8)) + t*t*rng.uniform(-10, 10, (1, 8))
    changed = replace(original, displacement_m=original.displacement_m@rotation.T,
                       phase_cycles=original.phase_cycles+nuisance)
    result = detect_window(changed)
    assert result.decision == case
    assert_allclose(result.gamma, baseline.gamma, atol=.002, rtol=1e-6)
    if case == 'benign':
        fitted = np.array(result.benign.parameters['rotation_nominal_to_displacement'])
        base_fitted = np.array(baseline.benign.parameters['rotation_nominal_to_displacement'])
        assert_allclose(fitted, rotation@base_fitted, atol=2e-6)


@pytest.mark.parametrize('rho', [0., .7])
def test_qr_cost_matches_independent_covariance_weighted_least_squares(rho):
    window = example(case='benign', rho=rho)
    result = detect_window(window, DetectorConfig(noise_rho=rho))
    assert result.decision == 'benign'
    n = len(window.time_s)
    t = window.time_s-window.time_s[0]
    polynomial = np.column_stack([np.ones(n), t, t*t])
    covariance = .01**2 * rho**np.abs(np.arange(n)[:, None]-np.arange(n)[None, :])
    cholesky = np.linalg.cholesky(covariance)
    p_white = np.linalg.solve(cholesky, polynomial)
    d = window.displacement_m-window.displacement_m[0]
    for fit, directions in [
            (result.benign, window.nominal_los@np.array(result.benign.parameters['rotation_nominal_to_displacement']).T),
            (result.spoofed, np.tile(result.spoofed.parameters['shared_direction_displacement_frame'], (8, 1)))]:
        cost = 0.
        for j in range(8):
            motion = -d@directions[j]/window.wavelength_m[j]
            y = window.phase_cycles[:, j]-window.phase_cycles[0, j]-motion
            y_white = np.linalg.solve(cholesky, y)
            beta = np.linalg.lstsq(p_white, y_white, rcond=None)[0]
            residual = y_white-p_white@beta
            cost += .5*residual@residual
        assert_allclose(cost, fit.total_cost, rtol=1e-10, atol=1e-6)


def test_rotation_jacobian_against_central_differences():
    rng = np.random.default_rng(52)
    x = np.array([.3, -.8, 1.2])
    seed = Rotation.from_euler('xyz', [10, 25, 80], degrees=True).as_matrix()
    los = rng.normal(size=(8, 3)); los /= np.linalg.norm(los, axis=1)[:, None]
    matrices, targets = rng.normal(size=(8, 3, 3)), rng.normal(size=(8, 3))
    _, jacobian = rotation_residual_jacobian(x, seed, los, matrices, targets)
    derivative = np.column_stack([
        (rotation_residual_jacobian(x+np.eye(3)[i]*1e-6, seed, los, matrices, targets)[0]
         -rotation_residual_jacobian(x-np.eye(3)[i]*1e-6, seed, los, matrices, targets)[0])/2e-6
        for i in range(3)])
    assert_allclose(jacobian, derivative, atol=1e-8, rtol=1e-8)


@pytest.mark.parametrize('hard_case', [False, True])
def test_unit_direction_global_optimality_certificate(hard_case):
    rng = np.random.default_rng(451)
    matrices = rng.normal(size=(8, 3, 3))
    targets = np.zeros((8, 3)) if hard_case else rng.normal(size=(8, 3))
    direction, diagnostic = solve_unit_direction(matrices, targets)
    assert diagnostic['success']
    assert diagnostic['hard_case'] == hard_case
    assert_allclose(np.linalg.norm(direction), 1., atol=2e-15)
    h = np.einsum('sji,sjk->ik', matrices, matrices)
    g = np.einsum('sji,sj->i', matrices, targets)
    multiplier = diagnostic['multiplier']
    assert np.linalg.eigvalsh(h+multiplier*np.eye(3))[0] > -1e-10
    assert_allclose((h+multiplier*np.eye(3))@direction, g, atol=1e-10)
    candidates = rng.normal(size=(2000, 3)); candidates /= np.linalg.norm(candidates, axis=1)[:, None]
    cost = lambda u: .5*np.sum((np.einsum('sij,j->si', matrices, u)-targets)**2)
    assert cost(direction) <= min(cost(u) for u in candidates)+1e-10


@pytest.mark.parametrize('scenario,reason', [
    ('stationary', 'insufficient_nonpolynomial_motion'),
    ('constant_velocity', 'insufficient_nonpolynomial_motion'),
    ('east_oscillation', 'rank_deficient_motion'),
    ('circle', 'rank_deficient_motion'),
    ('circle_coning', 'rank_deficient_motion'),
])
def test_uninformative_and_planar_controls_are_inconclusive(scenario, reason):
    result = detect_window(example(scenario))
    assert result.decision == 'inconclusive'
    assert result.reason == reason
    assert result.gamma is None


def test_missing_signals_and_phase_arc_resets_are_not_bridged():
    window = example()
    window.carrier_valid[:, 0] = False
    window.arc_id[10:, 1] = 1
    window.lock_time_s[12:, 2] = 0
    result = detect_window(window)
    assert result.decision == 'benign'
    assert len(result.signal_indices) == 5
    assert set(result.diagnostics['excluded_signals']) == {'S01', 'S02', 'S03'}
    window.carrier_valid[:, 3:5] = False
    result = detect_window(window)
    assert result.decision == 'inconclusive'
    assert result.reason == 'insufficient_continuous_signals'


def test_time_gap_geometry_and_model_mismatch_quality_checks():
    window = example()
    window.time_s[-1] += .1
    assert detect_window(window).reason == 'measurement_time_gap'
    window = example()
    window.nominal_los[:] = [0, 0, 1]
    assert detect_window(window).reason == 'ambiguous_nominal_geometry'
    window = example()
    window.phase_cycles[8:, :] += 20
    result = detect_window(window)
    assert result.decision == 'inconclusive'
    assert result.reason in {'neither_model_fits', 'optimizer_not_reliably_converged'}
    assert detect_window(example().subset(np.arange(6))).reason == 'insufficient_samples'
