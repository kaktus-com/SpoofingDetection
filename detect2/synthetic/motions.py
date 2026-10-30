"""Analytic rigid-body motion, generated independently of any detector.

The body origin is the IMU. Its position is ENU; body axes are FRD. Derivatives
are analytic, including rotational velocity/acceleration at the antenna offset.
"""

from dataclasses import dataclass, field

import numpy as np

from ..geometry import body_to_enu_matrix, coordinates, rotate, rpy_to_quaternion

GRAVITY_ENU_M_S2 = np.array([0., 0., -9.80665])
DEFAULT_LEVER_ARM_M = (0.10, 0.05, -0.20)
SCENARIOS = {
    "stationary": "Stationary control",
    "constant_velocity": "Straight line toward East",
    "east_oscillation": "East–West sinusoidal translation",
    "circle": "Horizontal circular translation",
    "circle_coning": "Circular translation with inward-tilted antenna pointing",
    "three_axis": "Three-axis sinusoidal translation",
    "rotation_only": "Fixed IMU origin, rotating offset antenna",
    "combined": "Three-axis translation and rotation",
}


@dataclass(frozen=True)
class MotionConfig:
    scenario: str
    duration_s: float = 6.0
    rate_hz: float = 200.0
    lever_arm_body_m: tuple = DEFAULT_LEVER_ARM_M
    amplitude_scale: float = 1.0
    frequency_scale: float = 1.0
    cone_tilt_deg: float = 45.0

    def __post_init__(self):
        if self.scenario not in SCENARIOS:
            raise ValueError(f"Unknown scenario: {self.scenario}")
        settings = [self.duration_s, self.rate_hz, self.amplitude_scale, self.frequency_scale]
        if not np.isfinite(settings).all() or any(v <= 0 for v in settings):
            raise ValueError("Duration, rate and scales must be finite and positive")
        if self.duration_s * self.rate_hz < 4:
            raise ValueError("At least five motion samples are required")
        if self.rate_hz <= 2 * 2.5 * self.frequency_scale:
            raise ValueError("Sample rate must exceed twice the highest motion frequency")
        if not np.isfinite(self.cone_tilt_deg) or not 0 <= self.cone_tilt_deg <= 90:
            raise ValueError("Cone tilt must be between 0 and 90 degrees from vertical")
        if coordinates(self.lever_arm_body_m).shape != (3,):
            raise ValueError("Antenna offset must be one FRD vector")


@dataclass
class Motion:
    config: MotionConfig
    time_s: np.ndarray
    body_position_m: np.ndarray
    body_velocity_m_s: np.ndarray
    body_acceleration_m_s2: np.ndarray
    rpy_rad: np.ndarray
    quaternion_wxyz: np.ndarray
    body_to_enu: np.ndarray
    angular_rate_body_rad_s: np.ndarray
    angular_acceleration_body_rad_s2: np.ndarray
    antenna_position_m: np.ndarray
    antenna_velocity_m_s: np.ndarray
    antenna_acceleration_m_s2: np.ndarray
    specific_force_body_m_s2: np.ndarray
    parameters: dict = field(default_factory=dict)

    @property
    def antenna_boresight_enu(self):
        """Unit pointing axis of an antenna mounted along negative body Down."""
        return -self.body_to_enu[..., :, 2]

    @property
    def body_forward_enu(self):
        return self.body_to_enu[..., :, 0]


def harmonic(time_s, amplitude, frequency_hz, phase=0.):
    """Position starts at zero; velocity and acceleration are its derivatives."""
    omega = 2 * np.pi * np.asarray(frequency_hz)
    angle = time_s[:, None] * omega + np.asarray(phase)
    amplitude = np.asarray(amplitude)
    return (amplitude * (np.sin(angle) - np.sin(phase)),
            amplitude * omega * np.cos(angle),
            -amplitude * omega**2 * np.sin(angle))


def body_angular_kinematics(rpy, rates, accelerations):
    """Euler-angle derivatives → FRD angular velocity and its body derivative."""
    r, p, _ = rpy.T
    rd, pd, yd = rates.T
    rdd, pdd, ydd = accelerations.T
    sr, cr, sp, cp = np.sin(r), np.cos(r), np.sin(p), np.cos(p)
    omega = np.column_stack((rd - yd * sp,
                             pd * cr + yd * sr * cp,
                             -pd * sr + yd * cr * cp))
    alpha = np.column_stack((rdd - ydd * sp - yd * cp * pd,
                             pdd * cr - pd * sr * rd + ydd * sr * cp
                             + yd * cr * rd * cp - yd * sr * sp * pd,
                             -pdd * sr - pd * cr * rd + ydd * cr * cp
                             - yd * sr * rd * cp - yd * cr * sp * pd))
    return omega, alpha


def generate_motion(config):
    """Generate samples at exactly 1/rate_hz, never beyond the requested duration."""
    t = np.arange(int(np.floor(config.duration_s * config.rate_hz + 1e-9)) + 1) / config.rate_hz
    n = len(t)
    pos, vel, acc, rpy, rates, accelerations = [np.zeros((n, 3)) for _ in range(6)]
    scale, freq = config.amplitude_scale, config.frequency_scale
    parameters = {}
    if config.scenario == "constant_velocity":
        vel[:, 0] = 0.25 * scale
        pos = t[:, None] * vel
        parameters["velocity_enu_m_s"] = [0.25 * scale, 0., 0.]
    elif config.scenario == "east_oscillation":
        pos, vel, acc = harmonic(t, [0.05 * scale, 0., 0.], [2. * freq, 0., 0.])
        parameters.update(amplitude_enu_m=[0.05 * scale, 0., 0.], frequency_hz=[2. * freq, 0., 0.])
    elif config.scenario in {"circle", "circle_coning"}:
        radius, omega = 0.1 * scale, 2 * np.pi * freq
        angle = omega * t
        pos = np.column_stack((radius * (np.cos(angle) - 1), radius * np.sin(angle), np.zeros(n)))
        vel = np.column_stack((-radius * omega * np.sin(angle), radius * omega * np.cos(angle), np.zeros(n)))
        acc = np.column_stack((-radius * omega**2 * np.cos(angle), -radius * omega**2 * np.sin(angle), np.zeros(n)))
        parameters.update(radius_m=radius, frequency_hz=freq)
        if config.scenario == "circle_coning":
            # Counterclockwise ENU orbit: NED yaw decreases. The antenna's
            # -Down axis leans radially inward and stays normal to body velocity.
            rpy[:, 1] = np.radians(config.cone_tilt_deg)
            rpy[:, 2] = np.pi / 2 - angle
            rates[:, 2] = -omega
            parameters.update(tilt_from_vertical_deg=config.cone_tilt_deg,
                              antenna_boresight_body=[0., 0., -1.],
                              pointing="radially inward and upward",
                              orbit_direction="counterclockwise viewed from above",
                              attitude_model="prescribed kinematics, not force-balanced flight dynamics")
    elif config.scenario in {"three_axis", "combined"}:
        amplitudes = scale * np.array([0.04, 0.03, 0.02])
        frequencies = freq * np.array([1.25, 1.75, 2.5])
        phases = np.array([0., np.pi / 4, np.pi / 3])
        pos, vel, acc = harmonic(t, amplitudes, frequencies, phases)
        parameters.update(amplitude_enu_m=amplitudes.tolist(), frequency_hz=frequencies.tolist(),
                          phase_rad=phases.tolist())
    if config.scenario in {"rotation_only", "combined"}:
        # Amplitude scaling changes translation, not the declared attitude profile.
        amplitudes = np.radians([15., 10., 20.])
        frequencies = freq * np.array([1.5, 1., 0.5])
        rpy, rates, accelerations = harmonic(t, amplitudes, frequencies)
        parameters.update(attitude_amplitude_deg=[15., 10., 20.],
                          attitude_frequency_hz=frequencies.tolist())
    rotation = body_to_enu_matrix(rpy)
    omega, alpha = body_angular_kinematics(rpy, rates, accelerations)
    lever = np.asarray(config.lever_arm_body_m)
    lever_velocity = np.cross(omega, lever)
    lever_acceleration = np.cross(alpha, lever) + np.cross(omega, lever_velocity)
    return Motion(config, t, pos, vel, acc, rpy, rpy_to_quaternion(rpy), rotation,
                  omega, alpha, pos + rotate(rotation, lever),
                  vel + rotate(rotation, lever_velocity),
                  acc + rotate(rotation, lever_acceleration),
                  rotate(np.swapaxes(rotation, -1, -2), acc - GRAVITY_ENU_M_S2), parameters)
