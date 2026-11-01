"""White-accelerometer covariance and a conservative motion-quality check.

The covariance is conditional on supplied attitude and times. It does not model
bias, attitude uncertainty, timing errors, coloured noise or integration error.
The likelihood fits and their zero gamma threshold are untouched.
"""

from dataclasses import replace

import numpy as np

from .detector import WindowResult, polynomial_basis


def integration_matrix(source_times, measurement_times):
    """Exact p(t)=K a for linearly interpolated sampled ENU acceleration.

    Integrate each original sample's hat function against (t-s) over [t0,t].
    Clipped boundary segments retain both source weights and their correlation.
    Zero initial position/velocity match the normal short-window integrator.
    """
    source = np.asarray(source_times, dtype=float)
    query = np.asarray(measurement_times, dtype=float)
    if (source.ndim != 1 or query.ndim != 1 or len(source) < 2 or len(query) < 2
            or not np.isfinite(source).all() or not np.isfinite(query).all()
            or np.any(np.diff(source) <= 0) or np.any(np.diff(query) <= 0)
            or query[0] < source[0]-1e-10 or query[-1] > source[-1]+1e-10):
        raise ValueError('Integration covariance requires increasing, covered source and measurement times')
    start = np.maximum(query[0], source[:-1])[None, :]
    stop = np.minimum(query[:, None], source[1:][None, :])
    duration = np.maximum(0., stop-start)
    remaining = query[:, None]-start
    offset = start-source[:-1]
    integral = remaining*duration-.5*duration**2
    right_weight = (remaining*offset*duration+.5*(remaining-offset)*duration**2-duration**3/3)/np.diff(source)
    matrix = np.zeros((len(query), len(source)))
    matrix[:, :-1] += integral-right_weight
    matrix[:, 1:] += right_weight
    return matrix


def displacement_covariance(source_times, measurement_times, sigma):
    """Per-ENU-axis temporal covariance, before and after quadratic removal.

    Independent isotropic body noise stays independent/isotropic when rotated by
    deterministic supplied attitude. The three ENU axes therefore share C, with
    zero cross-axis covariance. GNSS epochs are correlated, including boundaries.
    """
    if not np.isfinite(sigma) or sigma < 0:
        raise ValueError('Accelerometer sigma must be finite and nonnegative')
    n = len(measurement_times)
    if sigma == 0:
        return np.zeros((n, n)), np.zeros((n, n))
    weights = sigma*integration_matrix(source_times, measurement_times)
    p, _, _ = polynomial_basis(np.asarray(measurement_times))
    q = np.linalg.qr(p, mode='reduced')[0]
    observable_weights = weights-q@(q.T@weights)
    return weights@weights.T, observable_weights@observable_weights.T


def noise_norm_bound(covariance, tail_probability):
    """Bound ||E||_F (hence ||E||_2) for three independent N(0,C) columns.

    Hsu/Kakade/Zhang (2012), Proposition 1.1, applied to blockdiag(C,C,C):
    B²=3 tr(C)+2 sqrt(3 tr(C²) log(1/alpha))+2 lambda_max(C) log(1/alpha).
    The model probability of exceeding B is at most alpha per window. This is
    not a spoofing false-alarm probability or an across-window guarantee.
    """
    if not np.isfinite(tail_probability) or not 0 < tail_probability < 1:
        raise ValueError('Motion noise tail probability must be between zero and one')
    covariance = np.asarray(covariance, dtype=float)
    if (covariance.ndim != 2 or covariance.shape[0] != covariance.shape[1]
            or not np.isfinite(covariance).all()
            or not np.allclose(covariance, covariance.T, atol=1e-18, rtol=1e-10)):
        raise ValueError('Motion covariance must be finite, square and symmetric')
    eigenvalues = np.linalg.eigvalsh(covariance)
    if eigenvalues[0] < -1e-12*max(np.max(np.abs(eigenvalues)), 1e-30):
        raise ValueError('Motion covariance must be positive semidefinite')
    eigenvalues = np.maximum(0., eigenvalues)
    x = np.log(1/tail_probability)
    squared = 3*eigenvalues.sum()+2*np.sqrt(3*np.sum(eigenvalues**2)*x)+2*eigenvalues[-1]*x
    return float(np.sqrt(squared))


def assess_motion_noise(estimate, config):
    """Lower-bound true excitation using Weyl and reverse triangle inequalities."""
    n = len(estimate.time_s)
    if n < config.min_samples:
        return {'applied': False, 'passed': True, 'status': 'insufficient_samples_for_noise_check'}
    p, _, _ = polynomial_basis(estimate.time_s)
    q = np.linalg.qr(p, mode='reduced')[0]
    d = estimate.antenna_displacement_m
    observed = d-q@(q.T@d)
    singular = np.linalg.svd(observed, compute_uv=False)
    covariance = estimate.observable_covariance_m2
    sigma = estimate.diagnostics['accel_noise_sigma_m_s2']
    bound = noise_norm_bound(covariance, config.imu_noise_tail_probability)
    lower = np.maximum(0., singular-bound)/np.sqrt(n)
    total_lower = max(0., np.linalg.norm(observed)-bound)/np.sqrt(n)
    upper_condition = float((singular[0]+bound)/(singular[-1]-bound)) if singular[-1] > bound else None
    applied = bool(config.imu_noise_gate_enabled and sigma > 0)
    passed = not applied or (lower[-1] >= config.min_axis_rms_m and total_lower >= config.min_motion_rms_m
                             and upper_condition is not None and upper_condition <= config.max_motion_condition)
    return {'applied': applied, 'passed': bool(passed),
            'status': 'checked' if applied else 'disabled' if not config.imu_noise_gate_enabled else 'configured_white_noise_is_zero',
            'accel_noise_sigma_m_s2': sigma, 'tail_probability_per_window': config.imu_noise_tail_probability,
            'observed_axis_rms_m': (singular/np.sqrt(n)).tolist(),
            'observed_total_rms_m': float(np.linalg.norm(observed)/np.sqrt(n)),
            'axis_rms_lower_bound_m': lower.tolist(), 'total_rms_lower_bound_m': float(total_lower),
            'noise_norm_bound_m': bound, 'noise_bound_rms_m': float(bound/np.sqrt(n)),
            'expected_noise_vector_rms_m': float(np.sqrt(max(0., 3*np.trace(covariance))/n)),
            'min_axis_required_m': config.min_axis_rms_m, 'total_rms_required_m': config.min_motion_rms_m,
            'noise_free_motion_condition_upper_bound': upper_condition,
            'model': 'independent isotropic Gaussian acceleration samples; supplied attitude/times fixed; no bias/attitude/timing uncertainty'}


def attach_motion_check(result, check):
    """Attach diagnostics, rejecting before fitting when excitation is uncertain.

    Also supports a paired ablation: apply the identical pre-fit check to a saved
    ungated result without changing its fits or rerunning the likelihood solver.
    """
    diagnostic = {**result.diagnostics, 'imu_motion_noise': check}
    if check['passed']:
        return replace(result, diagnostics=diagnostic)
    return WindowResult(reason='imu_motion_below_noise_floor', diagnostics=diagnostic)
