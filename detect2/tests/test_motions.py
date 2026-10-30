import numpy as np
import pytest
from numpy.testing import assert_allclose

from SpoofingDetection.detect2.geometry import quaternion_multiply, rotate
from SpoofingDetection.detect2.synthetic.motions import (
    GRAVITY_ENU_M_S2, MotionConfig, SCENARIOS, generate_motion,
)


def test_stationary_gravity_and_rigid_offset():
    m = generate_motion(MotionConfig('stationary'))
    assert_allclose(m.antenna_position_m, np.tile([.05, .10, .20], (len(m.time_s), 1)))
    assert_allclose(m.body_acceleration_m_s2, 0.)
    assert_allclose(m.specific_force_body_m_s2, np.tile([0, 0, -9.80665], (len(m.time_s), 1)))


def test_circle_analytic_radius_speed_and_acceleration():
    m = generate_motion(MotionConfig('circle'))
    assert_allclose(np.linalg.norm(m.body_position_m + [.1, 0, 0], axis=1), .1, atol=1e-15)
    assert_allclose(np.linalg.norm(m.body_velocity_m_s, axis=1), .1 * 2*np.pi, atol=1e-15)
    assert_allclose(np.linalg.norm(m.body_acceleration_m_s2, axis=1), .1 * (2*np.pi)**2, atol=1e-14)


@pytest.mark.parametrize('tilt_deg', [0., 45., 70., 90.])
def test_circular_cone_tilt_radial_pointing_and_tangent_normal(tilt_deg):
    m = generate_motion(MotionConfig('circle_coning', duration_s=1., cone_tilt_deg=tilt_deg))
    theta = 2*np.pi*m.time_s
    beta = np.radians(tilt_deg)
    expected = np.column_stack((-np.sin(beta)*np.cos(theta),
                                 -np.sin(beta)*np.sin(theta),
                                 np.full(len(theta), np.cos(beta))))
    assert_allclose(m.antenna_boresight_enu, expected, atol=6e-16)
    assert_allclose(np.linalg.norm(m.antenna_boresight_enu, axis=1), 1., atol=5e-16)
    assert_allclose(np.sum(m.antenna_boresight_enu*m.body_velocity_m_s, axis=1), 0., atol=2e-16)
    assert_allclose(np.sum(m.antenna_boresight_enu*m.body_forward_enu, axis=1), 0., atol=2e-16)
    assert_allclose(np.linalg.norm(m.body_position_m+[.1, 0, 0], axis=1), .1, atol=1e-15)
    assert_allclose(m.body_to_enu[0], m.body_to_enu[-1], atol=4e-16)
    # With constant pitch during the orbit, antenna height stays constant too.
    assert_allclose(m.antenna_position_m[:, 2], m.antenna_position_m[0, 2], atol=1e-16)


@pytest.mark.parametrize('scenario', list(SCENARIOS))
def test_motion_derivatives_and_gravity_reconstruction(scenario):
    m = generate_motion(MotionConfig(scenario, duration_s=.5, rate_hz=4000))
    dt = 1 / m.config.rate_hz
    inside = slice(3, -3)
    for p, v, a in [(m.body_position_m, m.body_velocity_m_s, m.body_acceleration_m_s2),
                    (m.antenna_position_m, m.antenna_velocity_m_s, m.antenna_acceleration_m_s2)]:
        assert_allclose(np.gradient(p, dt, axis=0)[inside], v[inside], atol=4e-6, rtol=1e-5)
        assert_allclose(np.gradient(v, dt, axis=0)[inside], a[inside], atol=8e-5, rtol=1e-5)
    assert_allclose(rotate(m.body_to_enu, m.specific_force_body_m_s2) + GRAVITY_ENU_M_S2,
                    m.body_acceleration_m_s2, atol=1e-14)
    assert_allclose(np.linalg.norm(m.antenna_position_m-m.body_position_m, axis=1),
                    np.linalg.norm(m.config.lever_arm_body_m), atol=2e-16)


@pytest.mark.parametrize('scenario', ['rotation_only', 'circle_coning'])
def test_gyro_matches_quaternion_derivative(scenario):
    m = generate_motion(MotionConfig(scenario, duration_s=1, rate_hz=4000))
    qdot = np.gradient(m.quaternion_wxyz, 1/m.config.rate_hz, axis=0)
    omega_quat = np.column_stack((np.zeros(len(m.time_s)), m.angular_rate_body_rad_s))
    assert_allclose(qdot[2:-2], .5 * quaternion_multiply(m.quaternion_wxyz, omega_quat)[2:-2],
                    atol=3e-6, rtol=1e-5)
    assert_allclose(np.gradient(m.angular_rate_body_rad_s, 1/m.config.rate_hz, axis=0)[2:-2],
                    m.angular_acceleration_body_rad_s2[2:-2], atol=4e-5, rtol=1e-5)


def test_rotation_at_zero_offset_does_not_translate_antenna():
    m = generate_motion(MotionConfig('rotation_only', lever_arm_body_m=(0, 0, 0)))
    assert_allclose(m.antenna_position_m, 0.)
    assert_allclose(m.antenna_velocity_m_s, 0.)
    assert_allclose(m.antenna_acceleration_m_s2, 0.)


def test_oscillation_frequency_amplitude_and_analytic_derivative():
    m = generate_motion(MotionConfig('east_oscillation', duration_s=2))
    assert_allclose(m.body_position_m[:, 0], .05 * np.sin(4*np.pi*m.time_s), atol=1e-16)
    assert_allclose(np.max(m.body_position_m[:, 0]), .05, atol=1e-16)
    assert_allclose(np.diff(m.time_s), 1/200, atol=3e-16)


def test_timebase_and_bad_configuration():
    m = generate_motion(MotionConfig('stationary', duration_s=1.003, rate_hz=200))
    assert m.time_s[-1] <= 1.003
    assert len(m.time_s) == 201
    for kwargs in [dict(rate_hz=0), dict(duration_s=-1), dict(frequency_scale=0),
                   dict(rate_hz=4), dict(lever_arm_body_m=(1, 2)), dict(amplitude_scale=float('nan')),
                   dict(cone_tilt_deg=-1), dict(cone_tilt_deg=91), dict(cone_tilt_deg=float('nan'))]:
        with pytest.raises(ValueError):
            MotionConfig('stationary', **kwargs)
