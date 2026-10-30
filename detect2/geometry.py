"""Explicit WGS-84, local-frame and body-frame conversions.

Angles in the numerical API are radians; distances are metres. Arrays use a
final coordinate dimension, e.g. (n, 3). Rotations multiply column vectors:
v_target = R_target_from_source @ v_source. Points also need a translation.
See CONVENTIONS.md for definitions, equations and primary references.
"""

from dataclasses import dataclass

import numpy as np

WGS84_A_M = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_B_M = WGS84_A_M * (1.0 - WGS84_F)
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)
ENU_FROM_NED = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
GPS_L1_WAVELENGTH_M = 299792458.0 / 1575420000.0


def coordinates(value, size=3):
    """Validate a finite array with a final coordinate dimension."""
    value = np.asarray(value, dtype=float)
    if value.ndim == 0 or value.shape[-1] != size or not np.all(np.isfinite(value)):
        raise ValueError(f"Expected finite coordinates with final dimension {size}")
    return value


def rotate(matrix, vectors):
    """Apply a matrix to column vectors, allowing batch broadcasting."""
    return np.einsum("...ij,...j->...i", matrix, coordinates(vectors))


def geodetic_to_ecef(latitude_rad, longitude_rad, height_m):
    """WGS-84 ellipsoidal latitude/longitude/height → ECEF point(s)."""
    lat, lon, height = np.broadcast_arrays(latitude_rad, longitude_rad, height_m)
    if not np.all(np.isfinite([lat, lon, height])) or np.any(np.abs(lat) > np.pi / 2):
        raise ValueError("Latitude must be within ±pi/2 and all inputs finite")
    n = WGS84_A_M / np.sqrt(1.0 - WGS84_E2 * np.sin(lat) ** 2)
    return np.stack(((n + height) * np.cos(lat) * np.cos(lon),
                     (n + height) * np.cos(lat) * np.sin(lon),
                     ((1.0 - WGS84_E2) * n + height) * np.sin(lat)), axis=-1)


def ecef_to_geodetic(ecef_m):
    """ECEF → [latitude rad, longitude rad, ellipsoidal height m].

    Fixed-point latitude iteration, with a stable height projection at poles.
    Intended for surface/air/space positions, not the interior of the Earth.
    Longitude at an exact pole is defined as zero.
    """
    xyz = coordinates(ecef_m)
    x, y, z = np.moveaxis(xyz, -1, 0)
    p = np.hypot(x, y)
    if np.any(np.linalg.norm(xyz, axis=-1) < 0.5 * WGS84_B_M):
        raise ValueError("Geodetic inverse is not defined here for deep-Earth points")
    lat = np.arctan2(z, p * (1.0 - WGS84_E2))
    for _ in range(20):
        n = WGS84_A_M / np.sqrt(1.0 - WGS84_E2 * np.sin(lat) ** 2)
        updated = np.arctan2(z + WGS84_E2 * n * np.sin(lat), p)
        converged = np.all(np.abs(updated - lat) < 1e-14)
        lat = updated
        if converged:
            break
    else:
        raise ValueError("ECEF-to-geodetic latitude iteration did not converge")
    height = p * np.cos(lat) + z * np.sin(lat) - WGS84_A_M * np.sqrt(
        1.0 - WGS84_E2 * np.sin(lat) ** 2)
    lon = np.where(p < 1e-9, 0.0, np.arctan2(y, x))
    return np.stack((lat, lon, height), axis=-1)


def ecef_to_enu_matrix(latitude_rad, longitude_rad):
    """Rows are local East, North, Up unit vectors expressed in ECEF."""
    lat, lon = float(latitude_rad), float(longitude_rad)
    if not np.isfinite([lat, lon]).all() or abs(lat) > np.pi / 2:
        raise ValueError("Invalid local-frame origin angles")
    sl, cl, sp, cp = np.sin(lon), np.cos(lon), np.sin(lat), np.cos(lat)
    return np.array([[-sl, cl, 0.], [-sp * cl, -sp * sl, cp],
                     [cp * cl, cp * sl, sp]])


@dataclass(frozen=True)
class LocalFrame:
    """Fixed ENU tangent frame at a WGS-84 origin; height is ellipsoidal."""

    latitude_deg: float = 59.3526
    longitude_deg: float = 18.0728
    height_m: float = 30.0

    def __post_init__(self):
        if (not np.isfinite([self.latitude_deg, self.longitude_deg, self.height_m]).all()
                or abs(self.latitude_deg) > 90 or abs(self.longitude_deg) > 180
                or self.height_m < -10000):
            raise ValueError("Invalid surface/air/space local-frame origin")

    @property
    def origin_ecef_m(self):
        return geodetic_to_ecef(np.radians(self.latitude_deg),
                                np.radians(self.longitude_deg), self.height_m)

    @property
    def enu_from_ecef(self):
        return ecef_to_enu_matrix(np.radians(self.latitude_deg),
                                  np.radians(self.longitude_deg))

    def ecef_to_enu(self, point_m):
        return rotate(self.enu_from_ecef, coordinates(point_m) - self.origin_ecef_m)

    def enu_to_ecef(self, point_m):
        return self.origin_ecef_m + rotate(self.enu_from_ecef.T, point_m)

    def ecef_vector_to_enu(self, vector):
        return rotate(self.enu_from_ecef, vector)

    def enu_vector_to_ecef(self, vector):
        return rotate(self.enu_from_ecef.T, vector)


def enu_to_ned(vector):
    return rotate(ENU_FROM_NED.T, vector)


def ned_to_enu(vector):
    return rotate(ENU_FROM_NED, vector)


def body_to_enu_matrix(rpy_rad):
    """FRD body → ENU using NED aerospace roll/pitch/yaw.

    R_ENU_body = R_ENU_NED Rz(yaw) Ry(pitch) Rx(roll).
    Zero angles: forward North, right East, down Down. Yaw +pi/2 faces East.
    """
    roll, pitch, yaw = np.moveaxis(coordinates(rpy_rad), -1, 0)
    sr, cr, sp, cp = np.sin(roll), np.cos(roll), np.sin(pitch), np.cos(pitch)
    sy, cy = np.sin(yaw), np.cos(yaw)
    row0 = np.stack((cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy), -1)
    row1 = np.stack((cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy), -1)
    row2 = np.stack((-sp, sr * cp, cr * cp), -1)
    return np.einsum("ij,...jk->...ik", ENU_FROM_NED,
                     np.stack((row0, row1, row2), axis=-2))


def quaternion_multiply(left_wxyz, right_wxyz):
    """Hamilton product; composition of active rotations, right first."""
    a, b = np.broadcast_arrays(coordinates(left_wxyz, 4), coordinates(right_wxyz, 4))
    scalar = a[..., :1] * b[..., :1] - np.sum(a[..., 1:] * b[..., 1:], -1, keepdims=True)
    vector = (a[..., :1] * b[..., 1:] + b[..., :1] * a[..., 1:]
              + np.cross(a[..., 1:], b[..., 1:]))
    return np.concatenate((scalar, vector), axis=-1)


def rpy_to_quaternion(rpy_rad):
    """Hamilton [w,x,y,z] quaternion that rotates FRD body vectors to ENU."""
    r, p, y = np.moveaxis(coordinates(rpy_rad) / 2.0, -1, 0)
    sr, cr, sp, cp, sy, cy = np.sin(r), np.cos(r), np.sin(p), np.cos(p), np.sin(y), np.cos(y)
    q_ned_body = np.stack((cr * cp * cy + sr * sp * sy,
                           sr * cp * cy - cr * sp * sy,
                           cr * sp * cy + sr * cp * sy,
                           cr * cp * sy - sr * sp * cy), axis=-1)
    return quaternion_multiply([0., 2 ** -0.5, 2 ** -0.5, 0.], q_ned_body)


def quaternion_to_matrix(q_wxyz):
    q = coordinates(q_wxyz, 4)
    norm = np.linalg.norm(q, axis=-1, keepdims=True)
    if np.any(norm < 1e-12):
        raise ValueError("A zero quaternion does not define a rotation")
    w, x, y, z = np.moveaxis(q / norm, -1, 0)
    return np.stack((
        np.stack((1 - 2 * (y*y + z*z), 2 * (x*y - w*z), 2 * (x*z + w*y)), -1),
        np.stack((2 * (x*y + w*z), 1 - 2 * (x*x + z*z), 2 * (y*z - w*x)), -1),
        np.stack((2 * (x*z - w*y), 2 * (y*z + w*x), 1 - 2 * (x*x + y*y)), -1),
    ), axis=-2)


def quaternion_to_rpy(q_wxyz):
    """Return canonical NED roll/pitch/yaw; choose roll=0 at gimbal lock."""
    m = np.einsum("ij,...jk->...ik", ENU_FROM_NED.T, quaternion_to_matrix(q_wxyz))
    pitch = np.arcsin(np.clip(-m[..., 2, 0], -1., 1.))
    singular = np.abs(np.cos(pitch)) < 1e-7
    roll = np.where(singular, 0., np.arctan2(m[..., 2, 1], m[..., 2, 2]))
    yaw = np.where(singular, np.arctan2(-m[..., 0, 1], m[..., 1, 1]),
                   np.arctan2(m[..., 1, 0], m[..., 0, 0]))
    return np.stack((roll, pitch, yaw), axis=-1)


def az_el_to_enu(azimuth_rad, elevation_rad):
    """Receiver→source LOS. Azimuth clockwise from North, elevation above horizon."""
    az, el = np.broadcast_arrays(azimuth_rad, elevation_rad)
    if not np.isfinite([az, el]).all() or np.any(np.abs(el) > np.pi / 2):
        raise ValueError("Invalid azimuth/elevation")
    return np.stack((np.cos(el) * np.sin(az), np.cos(el) * np.cos(az), np.sin(el)), -1)


def enu_to_az_el(vectors):
    v = coordinates(vectors)
    norm = np.linalg.norm(v, axis=-1)
    if np.any(norm < 1e-12):
        raise ValueError("A zero vector has no direction")
    az = np.mod(np.arctan2(v[..., 0], v[..., 1]), 2 * np.pi)
    el = np.arctan2(v[..., 2], np.hypot(v[..., 0], v[..., 1]))
    return np.stack((az, el), axis=-1)


def projected_range_change(displacement_m, los_unit):
    """First-order range change: moving toward a transmitter shortens range."""
    los = coordinates(los_unit)
    if los.ndim != 2 or not np.allclose(np.linalg.norm(los, axis=1), 1., atol=1e-12, rtol=0):
        raise ValueError("LOS must be an (sources, 3) array of unit vectors")
    return -np.einsum("...j,sj->...s", coordinates(displacement_m), los)


def finite_range_change(displacement_m, transmitter_from_reference_m):
    """Exact range differences to fixed sources, without subtracting large ranges.

    Source vectors originate at the receiver's reference position. Rationalize
    |s-d|-|s| as (-2 s·d + d·d)/(|s-d|+|s|) to preserve sub-mm changes.
    """
    d, s = coordinates(displacement_m), coordinates(transmitter_from_reference_m)
    if s.ndim != 2 or np.any(np.linalg.norm(s, axis=-1) < 1e-12):
        raise ValueError("Source vectors must be nonzero, shape (sources, 3)")
    numerator = -2 * np.einsum("...j,sj->...s", d, s) + np.sum(d*d, -1)[..., None]
    denominator = np.linalg.norm(s - d[..., None, :], axis=-1) + np.linalg.norm(s, axis=-1)
    return numerator / denominator
