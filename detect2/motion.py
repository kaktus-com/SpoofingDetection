"""Short-window antenna displacement from IMU specific force and supplied attitude.

Initial IMU position/velocity are zero in every window. The unknown initial
velocity creates a linear displacement error, removed by the detector's nuisance
projection. No truth, carrier phase or GNSS position is used by this estimator.
"""

from dataclasses import dataclass
import csv
import json
from pathlib import Path

import numpy as np

from .geometry import quaternion_to_matrix, rotate
from .motion_uncertainty import displacement_covariance


class MotionUnavailable(ValueError):
    """A window cannot be integrated without extrapolation or bridging a gap."""


@dataclass
class ImuSeries:
    time_s: np.ndarray
    specific_force_body_m_s2: np.ndarray
    angular_rate_body_rad_s: np.ndarray
    quaternion_body_to_enu_wxyz: np.ndarray
    lever_arm_body_m: np.ndarray
    gravity_enu_m_s2: np.ndarray
    nominal_rate_hz: float
    accel_noise_sigma_m_s2: float = 0.

    def __post_init__(self):
        self.time_s = np.asarray(self.time_s, dtype=float)
        self.specific_force_body_m_s2 = np.asarray(self.specific_force_body_m_s2, dtype=float)
        self.angular_rate_body_rad_s = np.asarray(self.angular_rate_body_rad_s, dtype=float)
        q = np.asarray(self.quaternion_body_to_enu_wxyz, dtype=float)
        self.lever_arm_body_m = np.asarray(self.lever_arm_body_m, dtype=float)
        self.gravity_enu_m_s2 = np.asarray(self.gravity_enu_m_s2, dtype=float)
        n = len(self.time_s)
        if (n < 2 or self.time_s.shape != (n,) or not np.isfinite(self.time_s).all()
                or np.any(np.diff(self.time_s) <= 0)):
            raise ValueError("IMU times must be finite and strictly increasing")
        for values, shape in [(self.specific_force_body_m_s2, (n, 3)),
                               (self.angular_rate_body_rad_s, (n, 3)), (q, (n, 4)),
                               (self.lever_arm_body_m, (3,)), (self.gravity_enu_m_s2, (3,))]:
            if values.shape != shape or not np.isfinite(values).all():
                raise ValueError("Invalid IMU/calibration array shape or non-finite measurement")
        norm = np.linalg.norm(q, axis=1)
        if np.any(norm < 1e-12):
            raise ValueError("A zero IMU attitude quaternion is invalid")
        self.quaternion_body_to_enu_wxyz = q/norm[:, None]
        self.max_input_quaternion_norm_error = float(np.max(np.abs(norm-1)))
        if not np.isfinite(self.nominal_rate_hz) or self.nominal_rate_hz <= 0:
            raise ValueError("Nominal IMU rate must be positive")
        if not np.isfinite(self.accel_noise_sigma_m_s2) or self.accel_noise_sigma_m_s2 < 0:
            raise ValueError('Accelerometer white-noise sigma must be finite and nonnegative')
        self.linear_acceleration_enu_m_s2 = rotate(
            quaternion_to_matrix(self.quaternion_body_to_enu_wxyz), self.specific_force_body_m_s2) + self.gravity_enu_m_s2


@dataclass
class MotionEstimate:
    time_s: np.ndarray
    antenna_displacement_m: np.ndarray
    imu_displacement_m: np.ndarray
    imu_velocity_m_s: np.ndarray
    lever_displacement_m: np.ndarray
    integration_time_s: np.ndarray
    integration_acceleration_m_s2: np.ndarray
    integration_position_m: np.ndarray
    diagnostics: dict
    displacement_covariance_m2: np.ndarray | None = None
    observable_covariance_m2: np.ndarray | None = None


def load_imu(imu_path, frame_path, nominal_rate_hz, accel_noise_sigma_m_s2=0.):
    frame = json.loads(Path(frame_path).read_text())
    if (frame.get("body_axes") != "Forward, Right, Down" or frame.get("local_axes") != "East, North, Up"
            or frame.get("quaternion") != "Hamilton wxyz, active FRD body to ENU"):
        raise ValueError("IMU adapter requires declared FRD force/gyro and Hamilton body-to-ENU attitude")
    # The CSV fields explicitly contain BODY measurements, already calibrated
    # into FRD. Do not apply sensor-to-body calibration a second time.
    keys = ["time_s", *[f"specific_force_body_{axis}_m_s2" for axis in ["forward", "right", "down"]],
            *[f"angular_rate_body_{axis}_rad_s" for axis in ["forward", "right", "down"]],
            *[f"q_body_to_enu_{axis}" for axis in ["w", "x", "y", "z"]]]
    with Path(imu_path).open(newline="") as stream:
        reader = csv.DictReader(stream)
        if set(keys)-set(reader.fieldnames or []):
            raise ValueError("IMU CSV is missing required measured fields")
        values = np.array([[float(row[key]) for key in keys] for row in reader])
    if values.ndim != 2 or values.shape[0] < 2:
        raise ValueError("IMU CSV requires at least two samples")
    return ImuSeries(values[:, 0], values[:, 1:4], values[:, 4:7], values[:, 7:11],
                     frame["lever_arm_body_m"], frame["gravity_enu_m_s2"], nominal_rate_hz, accel_noise_sigma_m_s2)


def quaternion_at(times, quaternions, query):
    """Shortest-path Hamilton quaternion SLERP, including sign-equivalent inputs."""
    right = np.searchsorted(times, query, side="left")
    right = np.clip(right, 0, len(times)-1)
    exact = np.abs(times[right]-query) < 1e-12
    left = np.where(exact, right, np.maximum(0, right-1))
    dt = times[right]-times[left]
    fraction = np.divide(query-times[left], dt, out=np.zeros(len(query)), where=dt > 0)
    q0, q1 = quaternions[left], quaternions[right].copy()
    dot = np.sum(q0*q1, axis=1)
    q1[dot < 0] *= -1
    dot = np.clip(np.abs(dot), 0, 1)
    angle = np.arccos(dot)
    sine = np.sin(angle)
    normal = sine > 1e-7
    first = 1-fraction
    second = fraction.copy()
    first[normal] = np.sin((1-fraction[normal])*angle[normal])/sine[normal]
    second[normal] = np.sin(fraction[normal]*angle[normal])/sine[normal]
    q = first[:, None]*q0+second[:, None]*q1
    return q/np.linalg.norm(q, axis=1)[:, None]


def integrate_linear_acceleration(times, acceleration):
    """Exact integral of piecewise-linear acceleration, with zero initial p/v."""
    dt = np.diff(times)[:, None]
    velocity = np.zeros_like(acceleration)
    position = np.zeros_like(acceleration)
    velocity[1:] = np.cumsum(.5*dt*(acceleration[:-1]+acceleration[1:]), axis=0)
    position[1:] = np.cumsum(velocity[:-1]*dt + dt*dt*(2*acceleration[:-1]+acceleration[1:])/6, axis=0)
    return position, velocity


def estimate_antenna_displacement(imu: ImuSeries, measurement_times):
    query = np.asarray(measurement_times, dtype=float)
    if query.ndim != 1 or len(query) < 2 or not np.isfinite(query).all() or np.any(np.diff(query) <= 0):
        raise MotionUnavailable("imu_insufficient_epoch_span")
    tolerance = 1e-10
    if query[0] < imu.time_s[0]-tolerance or query[-1] > imu.time_s[-1]+tolerance:
        raise MotionUnavailable("imu_coverage_missing")
    query = np.clip(query, imu.time_s[0], imu.time_s[-1])
    left = max(0, np.searchsorted(imu.time_s, query[0], side="right")-1)
    right = min(len(imu.time_s)-1, np.searchsorted(imu.time_s, query[-1], side="left"))
    source_t = imu.time_s[left:right+1]
    if len(source_t) < 2:
        raise MotionUnavailable("imu_insufficient_epoch_span")
    max_gap = float(np.max(np.diff(source_t)))
    if max_gap > 1.5/imu.nominal_rate_hz+tolerance:
        raise MotionUnavailable("imu_time_gap")
    interior = imu.time_s[(imu.time_s > query[0]+1e-12)&(imu.time_s < query[-1]-1e-12)]
    grid = np.concatenate(([query[0]], interior, [query[-1]]))
    acceleration = np.column_stack([np.interp(grid, imu.time_s, imu.linear_acceleration_enu_m_s2[:, axis]) for axis in range(3)])
    position, velocity = integrate_linear_acceleration(grid, acceleration)
    # Evaluate the integrated piecewise cubic INSIDE each source interval; do not
    # linearly interpolate position or round GNSS epochs to an IMU index.
    interval = np.clip(np.searchsorted(grid, query, side="right")-1, 0, len(grid)-2)
    h = (grid[interval+1]-grid[interval])[:, None]
    tau = (query-grid[interval])[:, None]
    slope = (acceleration[interval+1]-acceleration[interval])/h
    at_query = position[interval]+velocity[interval]*tau+.5*acceleration[interval]*tau*tau+slope*tau**3/6
    velocity_query = velocity[interval]+acceleration[interval]*tau+.5*slope*tau*tau
    rotations = quaternion_to_matrix(quaternion_at(imu.time_s, imu.quaternion_body_to_enu_wxyz, query))
    lever = rotate(rotations, imu.lever_arm_body_m)
    lever_displacement = lever-lever[0]
    antenna = at_query-at_query[0]+lever_displacement
    covariance, observable_covariance = displacement_covariance(source_t, query, imu.accel_noise_sigma_m_s2)
    diagnostics = {"method": "piecewise-linear acceleration integrated exactly per segment; quaternion SLERP",
                   "attitude_source": "supplied body-to-ENU quaternion, not an estimated AHRS",
                   "initial_position_m": [0., 0., 0.], "initial_velocity_m_s": [0., 0., 0.],
                   "window_start_s": float(query[0]), "window_last_epoch_s": float(query[-1]),
                   "integration_samples": len(grid), "source_max_gap_s": max_gap,
                   "accel_noise_sigma_m_s2": float(imu.accel_noise_sigma_m_s2),
                   "uncertainty_model": 'per-axis temporal covariance of independent isotropic white acceleration samples; conditioned on supplied attitude/times; no bias/attitude/timing/integration-error covariance',
                   "quaternion_norm_error_before_normalization": imu.max_input_quaternion_norm_error,
                   "drift_policy": "zero initialization per window; no truth velocity, bias correction or high-pass filter; polynomial nuisance projection handles low-order drift"}
    return MotionEstimate(query, antenna, at_query-at_query[0], velocity_query, lever_displacement,
                           grid, acceleration, position, diagnostics, covariance, observable_covariance)
