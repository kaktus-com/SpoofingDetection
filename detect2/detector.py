"""Windowed carrier/displacement likelihood comparison from arXiv:2502.03870v1.

Raw phase is modeled with per-signal quadratic nuisance terms plus motion.
QR removes the nuisance terms. The benign model has ONE common proper rotation;
the spoofed model has ONE common unit LOS direction. Neither fit receives labels
or generated trends. Full-rank three-dimensional motion is required in this first
implementation; lower-rank motion is conservatively inconclusive.
"""

from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import brentq, least_squares
from scipy.spatial.transform import Rotation

from .schema import WindowInput


@dataclass(frozen=True)
class DetectorConfig:
    min_samples: int = 10
    min_signals: int = 4
    sigma_floor_cycles: float = .001
    noise_rho: float = 0.
    max_gap_s: float = .075
    min_motion_rms_m: float = .001
    min_axis_rms_m: float = .0001
    rank_absolute_tolerance_m: float = 1e-8
    rank_relative_tolerance: float = 1e-6
    max_motion_condition: float = 1000.
    max_geometry_condition: float = 1000.
    max_fit_rms_sigma: float = 3.
    max_optimizer_evaluations: int = 250
    tie_absolute: float = 1e-8
    tie_relative: float = 1e-8
    imu_noise_gate_enabled: bool = True
    imu_noise_tail_probability: float = .001

    def __post_init__(self):
        if self.min_samples < 7 or self.min_signals < 3 or self.max_optimizer_evaluations < 1:
            raise ValueError("Need at least 7 samples, 3 signals and a positive optimizer budget")
        positive = [self.sigma_floor_cycles, self.max_gap_s, self.min_motion_rms_m,
                    self.min_axis_rms_m, self.rank_absolute_tolerance_m, self.rank_relative_tolerance,
                    self.max_motion_condition, self.max_geometry_condition, self.max_fit_rms_sigma]
        if not np.isfinite(positive).all() or any(value <= 0 for value in positive):
            raise ValueError("Quality thresholds and sigma floor must be finite and positive")
        if not np.isfinite(self.noise_rho) or not 0 <= self.noise_rho < 1:
            raise ValueError("AR(1) rho must be in [0,1)")
        if not np.isfinite([self.tie_absolute, self.tie_relative]).all() or min(self.tie_absolute, self.tie_relative) < 0:
            raise ValueError("Tie tolerances must be finite and nonnegative")
        if not np.isfinite(self.imu_noise_tail_probability) or not 0 < self.imu_noise_tail_probability < 1:
            raise ValueError('IMU motion noise tail probability must be between zero and one')


@dataclass
class ReducedSignal:
    matrix: np.ndarray
    target: np.ndarray
    r: np.ndarray
    z: np.ndarray
    polynomial: np.ndarray
    motion: np.ndarray
    centered_phase: np.ndarray
    offset: float
    sigma: np.ndarray
    tail_cost: float


@dataclass
class HypothesisFit:
    reduced_cost: float
    total_cost: float
    parameters: dict
    trend_cycles: np.ndarray
    geometric_phase_cycles: np.ndarray
    fitted_phase_cycles: np.ndarray
    residual_cycles: np.ndarray
    normalized_residuals: np.ndarray
    polynomial_coefficients: np.ndarray
    diagnostics: dict


@dataclass
class WindowResult:
    decision: str = "inconclusive"
    reason: str = ""
    gamma: float | None = None
    diagnostics: dict = field(default_factory=dict)
    signal_indices: np.ndarray = field(default_factory=lambda: np.array([], dtype=int))
    benign: HypothesisFit | None = None
    spoofed: HypothesisFit | None = None


def whiten(values, sigma, rho=0.):
    """Whiten marginal-sigma-scaled white or known stationary AR(1) errors."""
    values = np.asarray(values)
    scaled = values/np.asarray(sigma)[:, None] if values.ndim == 2 else values/np.asarray(sigma)
    result = scaled.copy()
    if rho:
        result[1:] = (scaled[1:]-rho*scaled[:-1])/np.sqrt(1-rho*rho)
    return result


def polynomial_basis(time_s):
    center = .5*(time_s[0]+time_s[-1])
    scale = time_s[-1]-time_s[0]
    tau = (time_s-center)/scale
    return np.column_stack((np.ones(len(tau)), tau, .5*tau*tau)), center, scale


def reduce_signal(polynomial, displacement, phase, wavelength, sigma, rho=0.):
    """Full QR of [quadratic, -displacement/lambda], whitened per signal."""
    motion = -displacement/wavelength
    design = whiten(np.column_stack((polynomial, motion)), sigma, rho)
    # Subtracting an OBSERVED offset changes only the free constant nuisance term.
    centered = phase-phase[0]
    q, r = np.linalg.qr(design, mode="complete")
    z = q.T @ whiten(centered, sigma, rho)
    return ReducedSignal(r[3:6, 3:6], z[3:6], r, z, polynomial, motion,
                          centered, float(phase[0]), sigma, .5*float(z[6:]@z[6:]))


def skew(v):
    v = np.asarray(v)
    result = np.zeros(v.shape[:-1]+(3, 3))
    result[..., 0, 1], result[..., 0, 2] = -v[..., 2], v[..., 1]
    result[..., 1, 0], result[..., 1, 2] = v[..., 2], -v[..., 0]
    result[..., 2, 0], result[..., 2, 1] = -v[..., 1], v[..., 0]
    return result


def left_jacobian(rotvec):
    theta = np.linalg.norm(rotvec)
    k = skew(rotvec)
    if theta < 1e-5:
        a, b = .5-theta*theta/24, 1/6-theta*theta/120
    else:
        a, b = (1-np.cos(theta))/theta**2, (theta-np.sin(theta))/theta**3
    return np.eye(3)+a*k+b*(k@k)


def rotation_residual_jacobian(rotvec, seed, nominal_los, matrices, targets):
    rotation = Rotation.from_rotvec(rotvec).as_matrix()@seed
    directions = nominal_los@rotation.T
    residual = np.einsum("sij,sj->si", matrices, directions)-targets
    jac = np.einsum("sij,sjk,kl->sil", matrices, -skew(directions), left_jacobian(rotvec))
    return residual.ravel(), jac.reshape(-1, 3)


def fit_rotation(matrices, targets, nominal_los, max_evaluations=250):
    """Deterministic multistart SO(3) least squares; no global-optimum claim."""
    estimated = np.linalg.solve(matrices, targets[..., None])[..., 0]
    u, _, vt = np.linalg.svd(estimated.T@nominal_los)
    wahba = u@np.diag([1., 1., np.linalg.det(u@vt)])@vt
    seeds = [wahba, *Rotation.create_group("O").as_matrix()]
    solutions = []
    for seed in seeds:
        def fun(x):
            return rotation_residual_jacobian(x, seed, nominal_los, matrices, targets)[0]
        def jac(x):
            return rotation_residual_jacobian(x, seed, nominal_los, matrices, targets)[1]
        result = least_squares(fun, np.zeros(3), jac=jac, method="trf",
                               ftol=1e-10, xtol=1e-10, gtol=1e-10, max_nfev=max_evaluations)
        if result.success and np.isfinite(result.cost):
            solutions.append((float(result.cost), Rotation.from_rotvec(result.x).as_matrix()@seed,
                              int(result.nfev), float(result.optimality)))
    if not solutions:
        return None, {"success": False, "converged_starts": 0, "attempted_starts": len(seeds)}
    solutions.sort(key=lambda item: item[0])
    cost, rotation, evaluations, optimality = solutions[0]
    agreeing = sum(abs(other[0]-cost) <= 1e-7*(1+cost) for other in solutions)
    return rotation, {"success": agreeing >= 2, "converged_starts": len(solutions),
                      "attempted_starts": len(seeds), "agreeing_best_starts": agreeing,
                      "best_reduced_cost": cost, "best_evaluations": evaluations,
                      "best_optimality": optimality, "determinant": float(np.linalg.det(rotation))}


def solve_unit_direction(matrices, targets):
    """Global quadratic minimization on the unit sphere, including the hard case.

    The solution satisfies (H + multiplier*I)u=g, ||u||=1 and H+multiplier*I >=0.
    Eigenvalue scaling makes the secular equation numerically well behaved.
    """
    h = np.einsum("sji,sjk->ik", matrices, matrices)
    g = np.einsum("sji,sj->i", matrices, targets)
    scale = max(np.linalg.norm(h, 2), np.linalg.norm(g), 1e-30)
    eigenvalues, basis = np.linalg.eigh(h/scale)
    projected = basis.T@(g/scale)
    minimum = eigenvalues[0]
    near_min = eigenvalues-minimum < 1e-12
    components = np.zeros(3)
    hard_case = False
    if np.linalg.norm(projected[near_min]) <= 1e-12:
        components[~near_min] = projected[~near_min]/(eigenvalues[~near_min]-minimum)
        norm = np.linalg.norm(components)
        if norm <= 1.+1e-12:
            components[np.flatnonzero(near_min)[0]] = np.sqrt(max(0., 1.-norm*norm))
            multiplier = -minimum
            hard_case = True
    if not hard_case:
        lower = np.nextafter(-minimum, np.inf)
        def secular(value):
            denominator = eigenvalues+value
            with np.errstate(divide="ignore", over="ignore", invalid="ignore"):
                ratio = np.divide(projected, denominator, out=np.zeros_like(projected), where=denominator != 0)
                if np.any((denominator == 0)&(projected != 0)):
                    return np.inf
                return float(ratio@ratio-1.)
        upper = max(1., np.linalg.norm(projected)+abs(minimum))
        while secular(upper) > 0:
            upper *= 2
        multiplier = brentq(secular, lower, upper, xtol=5e-15, rtol=1e-14)
        components = projected/(eigenvalues+multiplier)
    direction = basis@components
    direction /= np.linalg.norm(direction)
    multiplier *= scale
    stationarity = np.linalg.norm((h+multiplier*np.eye(3))@direction-g)/scale
    min_shifted_eigenvalue = float(np.linalg.eigvalsh((h+multiplier*np.eye(3))/scale)[0])
    diagnostic = {"success": bool(stationarity < 1e-7 and min_shifted_eigenvalue >= -1e-9),
                  "hard_case": hard_case, "unit_norm": float(np.linalg.norm(direction)),
                  "relative_kkt_residual": float(stationarity),
                  "min_shifted_eigenvalue_scaled": min_shifted_eigenvalue,
                  "multiplier": float(multiplier)}
    return direction, diagnostic


def reconstruct_fit(signals, directions, parameters, optimizer_diagnostics, rho=0.):
    trends, geometric, fitted, residuals, normalized, coefficients = [], [], [], [], [], []
    reduced_cost, total_cost = 0., 0.
    consistency_errors = []
    for signal, direction in zip(signals, directions):
        beta = np.linalg.solve(signal.r[:3, :3], signal.z[:3]-signal.r[:3, 3:6]@direction)
        trend = signal.polynomial@beta
        motion = signal.motion@direction
        residual = signal.centered_phase-trend-motion
        whitened = whiten(residual, signal.sigma, rho)
        reduced = signal.matrix@direction-signal.target
        red = .5*float(reduced@reduced)
        direct = .5*float(whitened@whitened)
        consistency_errors.append(abs(direct-red-signal.tail_cost))
        reduced_cost += red
        total_cost += direct
        trends.append(trend+signal.offset)
        geometric.append(motion)
        fitted.append(signal.offset+trend+motion)
        residuals.append(residual)
        normalized.append(whitened)
        coefficients.append(beta)
    diag = dict(optimizer_diagnostics)
    diag["qr_vs_direct_cost_error"] = float(max(consistency_errors))
    diag["common_tail_cost"] = float(sum(signal.tail_cost for signal in signals))
    return HypothesisFit(reduced_cost, total_cost, parameters, np.column_stack(trends),
                          np.column_stack(geometric), np.column_stack(fitted), np.column_stack(residuals),
                          np.column_stack(normalized), np.column_stack(coefficients), diag)


def detect_window(window: WindowInput, config=DetectorConfig()):
    n, s = window.phase_cycles.shape
    result = WindowResult(diagnostics={"samples": n, "available_signals": s, "mode": "supplied_displacement"})
    def reject(reason):
        result.reason = reason
        return result
    if n < config.min_samples:
        return reject("insufficient_samples")
    if np.max(np.diff(window.time_s)) > config.max_gap_s+1e-12:
        return reject("measurement_time_gap")
    active, excluded = [], {}
    for j, name in enumerate(window.signal_ids):
        if not (np.all(window.carrier_valid[:, j]) and np.isfinite(window.phase_cycles[:, j]).all()
                and np.isfinite(window.sigma_cycles[:, j]).all() and np.all(window.sigma_cycles[:, j] >= 0)):
            excluded[name] = "invalid_or_missing_measurement"
        elif (not np.isfinite(window.lock_time_s[:, j]).all() or np.any(window.lock_time_s[:, j] < 0)
              or np.any(np.diff(window.lock_time_s[:, j]) < -1e-12)
              or np.any(window.arc_id[:, j] != window.arc_id[0, j])):
            excluded[name] = "phase_arc_or_lock_reset"
        else:
            active.append(j)
    result.signal_indices = np.array(active, dtype=int)
    result.diagnostics.update(used_signals=len(active), excluded_signals=excluded)
    if len(active) < config.min_signals:
        return reject("insufficient_continuous_signals")
    los = window.nominal_los[active]
    geometry_singular = np.linalg.svd(los, compute_uv=False)
    if geometry_singular[-1] <= geometry_singular[0]/config.max_geometry_condition:
        return reject("ambiguous_nominal_geometry")
    p, center, scale = polynomial_basis(window.time_s)
    d = window.displacement_m-window.displacement_m[0]
    q = np.linalg.qr(p, mode="reduced")[0]
    high_motion = d-q@(q.T@d)
    singular = np.linalg.svd(high_motion, compute_uv=False)
    threshold = max(config.rank_absolute_tolerance_m, config.rank_relative_tolerance*singular[0])
    rank = int(np.sum(singular > threshold))
    rms = float(np.linalg.norm(high_motion)/np.sqrt(n))
    axis_rms = float(singular[-1]/np.sqrt(n))
    condition = float(singular[0]/singular[-1]) if rank == 3 else None
    result.diagnostics.update(motion_rank=rank, motion_rms_m=rms, min_axis_rms_m=axis_rms,
                              motion_singular_values_m=singular.tolist(), motion_condition=condition,
                              time_center_s=float(center), time_scale_s=float(scale))
    if rms < config.min_motion_rms_m:
        return reject("insufficient_nonpolynomial_motion")
    if rank < 3:
        return reject("rank_deficient_motion")
    if axis_rms < config.min_axis_rms_m or condition > config.max_motion_condition:
        return reject("weak_or_ill_conditioned_motion")
    sigma = np.maximum(window.sigma_cycles[:, active], config.sigma_floor_cycles)
    result.diagnostics["sigma_floor_applied"] = bool(np.any(window.sigma_cycles[:, active] < config.sigma_floor_cycles))
    signals = [reduce_signal(p, d, window.phase_cycles[:, j], window.wavelength_m[j], sigma[:, k], config.noise_rho)
               for k, j in enumerate(active)]
    matrices = np.array([signal.matrix for signal in signals])
    targets = np.array([signal.target for signal in signals])
    if any(np.linalg.cond(matrix) > 1e8 for matrix in matrices):
        return reject("ill_conditioned_weighted_motion")
    rotation, benign_diag = fit_rotation(matrices, targets, los, config.max_optimizer_evaluations)
    direction, spoofed_diag = solve_unit_direction(matrices, targets)
    if rotation is not None:
        result.benign = reconstruct_fit(signals, los@rotation.T, {"rotation_nominal_to_displacement": rotation.tolist()}, benign_diag, config.noise_rho)
    result.spoofed = reconstruct_fit(signals, np.broadcast_to(direction, (len(active), 3)),
                                     {"shared_direction_displacement_frame": direction.tolist()}, spoofed_diag, config.noise_rho)
    if not benign_diag["success"] or not spoofed_diag["success"]:
        return reject("optimizer_not_reliably_converged")
    for fit in [result.benign, result.spoofed]:
        if fit.diagnostics["qr_vs_direct_cost_error"] > 1e-7*(1+fit.total_cost):
            return reject("qr_reconstruction_mismatch")
    # Eq.16 uses the informative reduced costs. Identical tail terms cancel.
    result.gamma = result.spoofed.reduced_cost-result.benign.reduced_cost
    best = min([result.benign, result.spoofed], key=lambda fit: fit.total_cost)
    parameter_count = 3 if best is result.benign else 2
    dof = n*len(active)-3*len(active)-parameter_count
    best_rms = float(np.sqrt(2*best.total_cost/max(1, dof)))
    result.diagnostics["best_fit_rms_sigma"] = best_rms
    if best_rms > config.max_fit_rms_sigma:
        return reject("neither_model_fits")
    tolerance = config.tie_absolute+config.tie_relative*(1+result.spoofed.reduced_cost+result.benign.reduced_cost)
    if abs(result.gamma) <= tolerance:
        return reject("indistinguishable_model_costs")
    result.decision = "benign" if result.gamma > 0 else "spoofed"
    result.reason = "positive_gamma" if result.gamma > 0 else "negative_gamma"
    return result
