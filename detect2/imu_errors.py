"""Reproducible input errors for controlled IMU integration experiments.

These are engineering stress settings, not a model of any particular sensor.
Only measured inputs are used; no truth is accessed here.
"""

from dataclasses import asdict, dataclass, replace
import csv
import hashlib

import numpy as np

from .geometry import quaternion_multiply


@dataclass(frozen=True)
class ImuErrors:
    accel_noise_sigma_m_s2: float = 0.
    accel_bias_body_m_s2: tuple = (0., 0., 0.)
    attitude_error_body_rotvec_deg: tuple = (0., 0., 0.)
    time_offset_s: float = 0.

    def __post_init__(self):
        for name in ['accel_bias_body_m_s2', 'attitude_error_body_rotvec_deg']:
            value = np.asarray(getattr(self, name), dtype=float)
            if value.shape != (3,) or not np.isfinite(value).all():
                raise ValueError(f'{name} must contain three finite body-axis values')
            object.__setattr__(self, name, tuple(float(x) for x in value))
        if (not np.isfinite([self.accel_noise_sigma_m_s2, self.time_offset_s]).all()
                or self.accel_noise_sigma_m_s2 < 0):
            raise ValueError('IMU noise sigma must be nonnegative and all error settings finite')


def perturb_imu(imu, errors: ImuErrors, seed=42, stream='default'):
    """Add body-frame force errors, a body-fixed attitude error and clock offset.

    The same seed/stream gives the same unit noise for every error level. Both
    carrier cases must share this one perturbed IMU. A positive time offset tags
    the original sample at t with t+offset, without shifting the carrier clock.
    """
    if not isinstance(seed, (int, np.integer)) or seed < 0:
        raise ValueError('IMU experiment seed must be a nonnegative integer')
    digest = np.frombuffer(hashlib.sha256(stream.encode('utf-8')).digest(), dtype='<u4')
    rng = np.random.Generator(np.random.PCG64(np.random.SeedSequence([int(seed), *digest.tolist()])))
    force = (imu.specific_force_body_m_s2 + np.asarray(errors.accel_bias_body_m_s2)
             + errors.accel_noise_sigma_m_s2*rng.standard_normal(imu.specific_force_body_m_s2.shape))
    rotvec = np.radians(errors.attitude_error_body_rotvec_deg)
    angle = np.linalg.norm(rotvec)
    error_q = np.r_[np.cos(angle/2), .5*np.sinc(angle/(2*np.pi))*rotvec]
    attitude = quaternion_multiply(imu.quaternion_body_to_enu_wxyz, error_q)
    return replace(imu, time_s=imu.time_s.copy()+errors.time_offset_s,
                   specific_force_body_m_s2=force,
                   angular_rate_body_rad_s=imu.angular_rate_body_rad_s.copy(),
                   quaternion_body_to_enu_wxyz=attitude,
                   lever_arm_body_m=imu.lever_arm_body_m.copy(),
                   gravity_enu_m_s2=imu.gravity_enu_m_s2.copy(),
                   accel_noise_sigma_m_s2=float(np.hypot(imu.accel_noise_sigma_m_s2, errors.accel_noise_sigma_m_s2)))


def export_imu(path, imu):
    """Save the actual perturbed measurements in the normal measured-input schema."""
    fields = ['time_s', *[f'specific_force_body_{axis}_m_s2' for axis in ['forward', 'right', 'down']],
              *[f'angular_rate_body_{axis}_rad_s' for axis in ['forward', 'right', 'down']],
              *[f'q_body_to_enu_{axis}' for axis in ['w', 'x', 'y', 'z']]]
    values = np.column_stack((imu.time_s, imu.specific_force_body_m_s2, imu.angular_rate_body_rad_s,
                              imu.quaternion_body_to_enu_wxyz))
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(fields)
        writer.writerows(values.tolist())


def error_metadata(errors, seed):
    return {'settings': asdict(errors), 'seed': int(seed),
            'noise_definition': 'independent Gaussian standard deviation per axis per IMU sample; not a spectral noise density',
            'attitude_definition': 'R_input @ Exp(body rotation vector); supplied attitude, not gyro integration',
            'timing_definition': 'tag original IMU sample at t as t+offset; carrier timestamps unchanged',
            'pairing': 'same IMU realization for both carrier cases; common unit noise across sigma levels',
            'scope': 'engineering sensitivity experiment, not calibrated hardware or field performance'}
