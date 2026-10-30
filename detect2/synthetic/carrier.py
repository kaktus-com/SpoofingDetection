"""Paired carrier-phase observations from analytic motion and fixed directions.

This is a measurement-level model, not an RF/tracking-loop simulator. It uses
range-observable phase signs, a per-signal quadratic nuisance trend and additive
noise. No detector, likelihood fitting or displacement estimation is performed.
"""

from dataclasses import dataclass, replace
import hashlib

import numpy as np

from ..geometry import GPS_L1_WAVELENGTH_M, LocalFrame, az_el_to_enu
from .generate import source_geometry
from .motions import MotionConfig, generate_motion

MODEL_VERSION = "carrier-pair-v1"
CASES = ("benign", "spoofed")


@dataclass(frozen=True)
class CarrierConfig:
    gnss_rate_hz: float = 20.0
    noise_sigma_cycles: float = 0.01
    noise_rho: float = 0.0
    seed: int = 42
    add_trend: bool = True
    trend_rate_span_cycles_s: float = 2500.0
    trend_acceleration_span_cycles_s2: float = 0.2
    spoofer_azimuth_deg: float = 230.0
    spoofer_elevation_deg: float = 15.0

    def __post_init__(self):
        values = [self.gnss_rate_hz, self.noise_sigma_cycles, self.noise_rho,
                  self.trend_rate_span_cycles_s, self.trend_acceleration_span_cycles_s2,
                  self.spoofer_azimuth_deg, self.spoofer_elevation_deg]
        if not np.isfinite(values).all():
            raise ValueError("Carrier configuration values must be finite")
        if self.gnss_rate_hz <= 0 or self.noise_sigma_cycles < 0:
            raise ValueError("GNSS rate must be positive; noise sigma must be nonnegative")
        if not 0 <= self.noise_rho < 1:
            raise ValueError("Noise rho must be in [0, 1)")
        if min(self.trend_rate_span_cycles_s, self.trend_acceleration_span_cycles_s2) < 0:
            raise ValueError("Trend coefficient spans must be nonnegative")
        if not -90 <= self.spoofer_elevation_deg <= 90:
            raise ValueError("Spoofer elevation must lie in [-90, 90] degrees")
        if isinstance(self.seed, bool) or not isinstance(self.seed, (int, np.integer)) or self.seed < 0:
            raise ValueError("Seed must be a nonnegative integer")


@dataclass
class CarrierCase:
    phase_cycles: np.ndarray
    geometric_phase_cycles: np.ndarray
    slow_phase_cycles: np.ndarray
    noise_cycles: np.ndarray
    actual_los_enu: np.ndarray


@dataclass
class CarrierPair:
    config: CarrierConfig
    imu_motion: object
    gnss_motion: object
    geometry: dict
    trend_coefficients: np.ndarray
    cases: dict

    @property
    def time_s(self):
        return self.gnss_motion.time_s

    @property
    def signal_ids(self):
        return self.geometry["names"]


def named_rng(seed, scenario, stream):
    """Stable streams independent of scenario selection/order and other draws."""
    digest = hashlib.sha256(f"{MODEL_VERSION}/{scenario}/{stream}".encode()).digest()
    words = [int.from_bytes(digest[i:i+4], "little") for i in range(0, len(digest), 4)]
    sequence = np.random.SeedSequence([int(seed), *words])
    return np.random.Generator(np.random.PCG64(sequence))


def phase_noise(rng, shape, sigma, rho):
    """Stationary Gaussian AR(1); sigma is the marginal standard deviation.

    At rho=0, draws are independent across epochs and signals. Each paired case
    has a separate named stream. The first sample has stationary variance too.
    """
    noise = rng.normal(0.0, sigma, size=shape)
    if rho:
        innovation_factor = np.sqrt(1.0-rho*rho)
        for i in range(1, len(noise)):
            noise[i] = rho*noise[i-1] + innovation_factor*noise[i]
    return noise


def generate_pair(motion_config, carrier_config=CarrierConfig(), frame=LocalFrame()):
    """Evaluate the SAME analytic motion independently on GNSS and IMU grids.

    No interpolation or rounded sample-index matching is used. Advertised
    satellite geometry is identical in both cases; the true spoofer direction
    is retained only in the generated truth, not the nominal geometry inputs.
    """
    if carrier_config.gnss_rate_hz > motion_config.rate_hz:
        raise ValueError("This dataset requires IMU rate >= GNSS rate")
    gnss_config = replace(motion_config, rate_hz=carrier_config.gnss_rate_hz)
    imu, gnss = generate_motion(motion_config), generate_motion(gnss_config)
    geometry = source_geometry(frame)
    n, s = len(gnss.time_s), len(geometry["names"])
    rng = named_rng(carrier_config.seed, motion_config.scenario, "trends")
    coefficients = np.zeros((3, s))
    if carrier_config.add_trend:
        # The constant represents an arbitrary large phase/ambiguity baseline.
        coefficients[0] = rng.integers(100000000, 120000000, size=s)
        coefficients[1] = rng.uniform(-carrier_config.trend_rate_span_cycles_s,
                                       carrier_config.trend_rate_span_cycles_s, size=s)
        coefficients[2] = rng.uniform(-carrier_config.trend_acceleration_span_cycles_s2,
                                       carrier_config.trend_acceleration_span_cycles_s2, size=s)
    t = gnss.time_s[:, None]
    slow = coefficients[0] + t*coefficients[1] + .5*t*t*coefficients[2]
    displacement = gnss.antenna_position_m-gnss.antenna_position_m[0]
    spoof_direction = az_el_to_enu(np.radians(carrier_config.spoofer_azimuth_deg),
                                   np.radians(carrier_config.spoofer_elevation_deg))
    directions = {"benign": geometry["los_enu"],
                  "spoofed": np.broadcast_to(spoof_direction, (s, 3)).copy()}
    cases = {}
    for case in CASES:
        # Receiver-to-source u and antenna displacement d: delta_range = -u·d.
        geometric = -(displacement @ directions[case].T)/GPS_L1_WAVELENGTH_M
        noise = phase_noise(named_rng(carrier_config.seed, motion_config.scenario, f"noise/{case}"),
                            (n, s), carrier_config.noise_sigma_cycles, carrier_config.noise_rho)
        cases[case] = CarrierCase(slow+geometric+noise, geometric, slow.copy(), noise, directions[case])
    return CarrierPair(carrier_config, imu, gnss, geometry, coefficients, cases)


def pair_metrics(pair):
    displacement = pair.gnss_motion.antenna_position_m-pair.gnss_motion.antenna_position_m[0]
    metrics = {"gnss_epochs": len(pair.time_s), "imu_samples": len(pair.imu_motion.time_s),
               "signals": len(pair.signal_ids),
               "carrier_rows_per_case": len(pair.time_s)*len(pair.signal_ids),
               "displacement_rank": int(np.linalg.matrix_rank(displacement, tol=1e-12)),
               "gnss_rate_hz": pair.config.gnss_rate_hz,
               "imu_rate_hz": pair.imu_motion.config.rate_hz}
    for name, case in pair.cases.items():
        noise = case.noise_cycles
        denominator = np.sqrt(np.sum(noise[1:]**2)*np.sum(noise[:-1]**2))
        metrics[name] = {
            "realized_noise_rms_cycles": float(np.sqrt(np.mean(noise**2))),
            "realized_noise_lag1_correlation": float(np.sum(noise[1:]*noise[:-1])/denominator) if denominator else None,
            "geometric_cross_signal_spread_rms_cycles": float(np.sqrt(np.mean(np.var(case.geometric_phase_cycles, axis=1)))),
            "phase_reconstruction_error_cycles": float(np.max(np.abs(
                case.phase_cycles-case.slow_phase_cycles-case.geometric_phase_cycles-noise))),
        }
    return metrics
