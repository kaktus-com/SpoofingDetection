import numpy as np
import pytest
from numpy.testing import assert_allclose
from scipy.spatial.transform import Rotation

from SpoofingDetection.detect2.geometry import (
    ENU_FROM_NED, GPS_L1_WAVELENGTH_M, LocalFrame, WGS84_A_M, WGS84_B_M,
    az_el_to_enu, body_to_enu_matrix, ecef_to_geodetic, enu_to_az_el,
    enu_to_ned, finite_range_change, geodetic_to_ecef, ned_to_enu,
    projected_range_change, quaternion_to_matrix, quaternion_to_rpy,
    rotate, rpy_to_quaternion,
)


def test_wgs84_axis_anchors():
    points = geodetic_to_ecef(np.radians([0., 0., 90., -90.]),
                              np.radians([0., 90., 0., 0.]), 0.)
    expected = [[WGS84_A_M, 0, 0], [0, WGS84_A_M, 0],
                [0, 0, WGS84_B_M], [0, 0, -WGS84_B_M]]
    assert_allclose(points, expected, atol=1e-8)


def test_geodetic_roundtrip_random_surface_and_space():
    rng = np.random.default_rng(724)
    llh = np.column_stack((rng.uniform(-89.99, 89.99, 300),
                           rng.uniform(-180, 180, 300), rng.uniform(-500, 3e7, 300)))
    llh[:, :2] = np.radians(llh[:, :2])
    xyz = geodetic_to_ecef(*llh.T)
    restored = ecef_to_geodetic(xyz)
    assert_allclose(restored[:, :2], llh[:, :2], rtol=0, atol=2e-14)
    assert_allclose(restored[:, 2], llh[:, 2], rtol=0, atol=3e-8)
    assert_allclose(geodetic_to_ecef(*restored.T), xyz, rtol=0, atol=3e-8)


def test_poles_dateline_and_invalid_geodetic():
    assert_allclose(ecef_to_geodetic([0, 0, WGS84_B_M + 123]), [np.pi/2, 0, 123], atol=1e-8)
    for lat in [-90, 0, 90]:
        xyz = geodetic_to_ecef(np.radians(lat), np.pi, 10)
        result = ecef_to_geodetic(xyz)
        assert_allclose(geodetic_to_ecef(*result), xyz, atol=1e-8)
    with pytest.raises(ValueError):
        ecef_to_geodetic([0, 0, 0])
    with pytest.raises(ValueError):
        geodetic_to_ecef(np.pi, 0, 0)


def test_local_basis_and_points_versus_vectors():
    frame = LocalFrame(0, 0, 0)
    assert_allclose(frame.ecef_vector_to_enu([1, 2, 3]), [2, 3, 1], atol=1e-15)
    assert_allclose(frame.ecef_to_enu(frame.origin_ecef_m), [0, 0, 0])
    assert_allclose(frame.enu_to_ecef([1, 2, 3]), [WGS84_A_M+3, 1, 2])
    for frame in [LocalFrame(), LocalFrame(-45, 179.999, 200), LocalFrame(90, 30, 0)]:
        r = frame.enu_from_ecef
        assert_allclose(r @ r.T, np.eye(3), atol=3e-16)
        assert_allclose(np.linalg.det(r), 1., atol=3e-16)
        points = np.array([[.01, -.02, .03], [100, -20, 40]])
        assert_allclose(frame.ecef_to_enu(frame.enu_to_ecef(points)), points, atol=2e-9)
        assert_allclose(frame.ecef_vector_to_enu(frame.enu_vector_to_ecef(points)), points, atol=1e-12)


def test_ned_and_body_compass_conventions():
    assert_allclose(enu_to_ned([1, 2, 3]), [2, 1, -3])
    assert_allclose(ned_to_enu(enu_to_ned([1, 2, 3])), [1, 2, 3])
    level = body_to_enu_matrix([0, 0, 0])
    assert_allclose(rotate(level, [1, 0, 0]), [0, 1, 0])
    assert_allclose(rotate(level, [0, 1, 0]), [1, 0, 0])
    assert_allclose(rotate(level, [0, 0, 1]), [0, 0, -1])
    assert_allclose(rotate(body_to_enu_matrix([0, 0, np.pi/2]), [1, 0, 0]), [1, 0, 0], atol=1e-15)
    assert_allclose(rotate(body_to_enu_matrix([0, np.pi/2, 0]), [1, 0, 0]), [0, 0, 1], atol=1e-15)


def test_quaternions_against_independent_rotation_library():
    rng = np.random.default_rng(912)
    rpy = rng.uniform(-1.2, 1.2, (80, 3))
    reference = np.einsum('ij,njk->nik', ENU_FROM_NED,
                          Rotation.from_euler('ZYX', rpy[:, ::-1]).as_matrix())
    q = rpy_to_quaternion(rpy)
    assert_allclose(body_to_enu_matrix(rpy), reference, atol=1e-15)
    assert_allclose(quaternion_to_matrix(q), reference, atol=1e-15)
    assert_allclose(Rotation.from_quat(q[:, [1, 2, 3, 0]]).as_matrix(), reference, atol=1e-15)
    assert_allclose(quaternion_to_rpy(q), rpy, atol=2e-15)
    assert_allclose(quaternion_to_matrix(-q), reference, atol=1e-15)
    with pytest.raises(ValueError):
        quaternion_to_matrix([0, 0, 0, 0])


@pytest.mark.parametrize('pitch', [-np.pi/2, np.pi/2])
def test_gimbal_lock_retains_orientation(pitch):
    rpy = np.array([.4, pitch, -.7])
    restored = quaternion_to_rpy(rpy_to_quaternion(rpy))
    assert_allclose(body_to_enu_matrix(restored), body_to_enu_matrix(rpy), atol=4e-8)


def test_sky_cardinals_and_direction_roundtrip():
    directions = az_el_to_enu(np.radians([0, 90, 180, 270, 0]), np.radians([0, 0, 0, 0, 90]))
    assert_allclose(directions, [[0, 1, 0], [1, 0, 0], [0, -1, 0], [-1, 0, 0], [0, 0, 1]], atol=1e-15)
    angles = np.radians([[12, 30], [120, 70], [300, -20]])
    assert_allclose(enu_to_az_el(az_el_to_enu(*angles.T)), angles, atol=1e-15)
    with pytest.raises(ValueError):
        enu_to_az_el([0, 0, 0])


def test_range_sign_scaling_and_stable_finite_range():
    assert_allclose(projected_range_change([0, 1, 0], [[0, 1, 0]]), [-1.])
    assert_allclose(projected_range_change([0, 0, 1], [[0, 1, 0]]), [0.])
    assert_allclose(finite_range_change([0, 1, 0], [[0, 2e7, 0]]), [-1.], atol=1e-15)
    # Perpendicular displacement has a tiny positive second-order range change.
    assert_allclose(finite_range_change([1, 0, 0], [[0, 2e7, 0]]), [2.5e-8], rtol=1e-14)
    assert_allclose(GPS_L1_WAVELENGTH_M, .19029367279836487)
    source = np.array([[10, 15, 20], [-20, 30, 10]])
    motion = np.array([[.1, .2, -.1], [1, 0, 0]])
    direct = np.linalg.norm(source - motion[:, None, :], axis=-1) - np.linalg.norm(source, axis=-1)
    assert_allclose(finite_range_change(motion, source), direct, atol=1e-14)
    with pytest.raises(ValueError):
        projected_range_change(motion, [[2, 0, 0]])
