from dataclasses import replace

import numpy as np
from numpy.testing import assert_allclose
import pytest

from SpoofingDetection.detect2.geometry import rotate, quaternion_to_matrix
from SpoofingDetection.detect2.motion import (
    ImuSeries, MotionUnavailable, estimate_antenna_displacement,
    integrate_linear_acceleration, quaternion_at,
)
from SpoofingDetection.detect2.motion_evaluation import remove_polynomial
from SpoofingDetection.detect2.synthetic.motions import MotionConfig, generate_motion


def measured_imu(motion):
    return ImuSeries(motion.time_s, motion.specific_force_body_m_s2,
                      motion.angular_rate_body_rad_s, motion.quaternion_wxyz,
                      motion.config.lever_arm_body_m, [0, 0, -9.80665], motion.config.rate_hz)


def test_piecewise_linear_acceleration_has_exact_analytic_integrals():
    t = np.array([0., .01, .035, .09, .2, .31])
    constant, slope = np.array([.2, -.5, 1.]), np.array([2., -.1, .4])
    a = constant+t[:, None]*slope
    p, v = integrate_linear_acceleration(t, a)
    assert_allclose(v, t[:, None]*constant+.5*t[:, None]**2*slope, atol=2e-16)
    assert_allclose(p, .5*t[:, None]**2*constant+t[:, None]**3*slope/6, atol=2e-17)


@pytest.mark.parametrize('scenario', ['stationary', 'constant_velocity', 'east_oscillation', 'circle',
                                      'three_axis', 'rotation_only', 'combined', 'circle_coning'])
def test_ideal_imu_recovers_observable_antenna_motion_without_initial_velocity(scenario):
    source = generate_motion(MotionConfig(scenario, duration_s=2))
    imu = measured_imu(source)
    query = np.arange(20)/20+1.
    estimate = estimate_antenna_displacement(imu, query)
    indices = np.round(query*200).astype(int)
    truth = source.antenna_position_m[indices]-source.antenna_position_m[indices[0]]
    error = estimate.antenna_displacement_m-truth
    observable = remove_polynomial(query, error)
    assert np.linalg.norm(observable)/np.sqrt(len(query)) < 2e-5
    assert_allclose(estimate.imu_velocity_m_s[0], 0., atol=1e-15)
    assert_allclose(estimate.antenna_displacement_m[0], 0., atol=1e-15)
    assert estimate.diagnostics['initial_velocity_m_s'] == [0., 0., 0.]
    if scenario == 'constant_velocity':
        assert_allclose(estimate.antenna_displacement_m, 0., atol=1e-14)
        assert np.linalg.norm(error[-1]) > .2
    if scenario == 'rotation_only':
        assert_allclose(estimate.antenna_displacement_m, truth, atol=3e-15)


def test_slerp_and_noncommensurate_rates_do_not_round_epochs():
    source = generate_motion(MotionConfig('combined', duration_s=1, rate_hz=211))
    target = generate_motion(MotionConfig('combined', duration_s=1, rate_hz=23))
    query = target.time_s
    estimate = estimate_antenna_displacement(measured_imu(source), query)
    true = target.antenna_position_m-target.antenna_position_m[0]
    assert np.linalg.norm(remove_polynomial(query, estimate.antenna_displacement_m-true))/np.sqrt(len(query)) < 3e-5
    # Alternating quaternion signs must represent exactly the same rotations.
    signed = source.quaternion_wxyz.copy(); signed[::2] *= -1
    q = quaternion_at(source.time_s, signed, query)
    assert_allclose(quaternion_to_matrix(q), quaternion_to_matrix(quaternion_at(source.time_s, source.quaternion_wxyz, query)), atol=1e-14)


def test_constant_enu_acceleration_bias_is_quadratic_and_removed_by_detector_basis():
    source = generate_motion(MotionConfig('three_axis', duration_s=1))
    imu = measured_imu(source)
    bias_enu = np.array([.03, -.02, .01])
    bias_body = rotate(np.swapaxes(quaternion_to_matrix(source.quaternion_wxyz), -1, -2), bias_enu)
    biased = replace(imu, specific_force_body_m_s2=imu.specific_force_body_m_s2+bias_body)
    query = np.arange(20)/20
    first = estimate_antenna_displacement(imu, query)
    second = estimate_antenna_displacement(biased, query)
    difference = second.antenna_displacement_m-first.antenna_displacement_m
    assert_allclose(difference, .5*query[:, None]**2*bias_enu, atol=4e-16)
    assert_allclose(remove_polynomial(query, difference), 0., atol=5e-16)


def test_no_extrapolation_or_bridging_imu_gaps():
    source = generate_motion(MotionConfig('three_axis', duration_s=1))
    imu = measured_imu(source)
    with pytest.raises(MotionUnavailable, match='coverage_missing'):
        estimate_antenna_displacement(imu, np.array([-.001, .9]))
    with pytest.raises(MotionUnavailable, match='coverage_missing'):
        estimate_antenna_displacement(imu, np.array([.1, 1.001]))
    keep = np.ones(len(source.time_s), dtype=bool); keep[50] = False
    broken = ImuSeries(imu.time_s[keep], imu.specific_force_body_m_s2[keep], imu.angular_rate_body_rad_s[keep],
                        imu.quaternion_body_to_enu_wxyz[keep], imu.lever_arm_body_m, imu.gravity_enu_m_s2, 200)
    with pytest.raises(MotionUnavailable, match='imu_time_gap'):
        estimate_antenna_displacement(broken, np.arange(20)/20)
    # A gap outside the queried window does not invalidate an unrelated window.
    estimate_antenna_displacement(broken, np.arange(10)/20+.5)


def test_zero_lever_arm_rotation_has_no_displacement_and_invalid_quaternions_fail():
    source = generate_motion(MotionConfig('rotation_only', duration_s=1, lever_arm_body_m=(0, 0, 0)))
    imu = measured_imu(source)
    result = estimate_antenna_displacement(imu, np.arange(20)/20)
    assert_allclose(result.antenna_displacement_m, 0., atol=2e-15)
    bad = imu.quaternion_body_to_enu_wxyz.copy(); bad[5] = 0
    with pytest.raises(ValueError, match='zero IMU attitude'):
        replace(imu, quaternion_body_to_enu_wxyz=bad)
