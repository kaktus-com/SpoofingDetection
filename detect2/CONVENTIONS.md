# Geometry and motion conventions

All geometry and motion code in this directory is implemented independently of
the older `torej` scripts. The current output is analytic truth and geometric
projections. No receiver observations or spoofing classifications are simulated.

## Units and array layout

Numerical geometry functions use radians, metres and seconds. Public names
identify the exceptional degree-valued origin configuration. CSV headers include
units. Vectors are stored with their coordinates on the last axis: `(samples, 3)`.
Matrices act on column vectors: `v_target = R_target_from_source @ v_source`.
The exported `site.json` gives the actual origin and ECEF-to-ENU rotation matrix.

## WGS-84 geodetic and ECEF

Geodetic latitude is the angle of the ellipsoid normal, not geocentric latitude.
Longitude is positive East. Height is above the WGS-84 ellipsoid; no MSL/geoid
conversion is performed. The default height of 30 m is an illustrative simulation
setting, not a claim about a flight or terrain elevation.

The WGS-84 semi-major axis is `a = 6378137 m`; inverse flattening is
`1/f = 298.257223563`. Set `e² = f(2-f)` and `N = a/sqrt(1-e² sin²(latitude))`.
Forward ECEF coordinates are:

```text
X = (N + h) cos(latitude) cos(longitude)
Y = (N + h) cos(latitude) sin(longitude)
Z = (N(1-e²) + h) sin(latitude)
```

The inverse iterates latitude and computes height by projection along the
ellipsoid normal. It handles poles without dividing by `cos(latitude)`, and
rejects deep-Earth positions outside the implemented domain. Longitude at an
exact pole is defined as zero; physical ECEF position remains unambiguous.

References: [NGA WGS-84 parameters](https://earth-info.nga.mil/index.php?action=wgs84&dir=wgs84),
[ESA ellipsoidal/Cartesian conversion](https://gssc.esa.int/navipedia/index.php/Ellipsoidal_and_Cartesian_Coordinates_Conversion).

## Local ENU and NED

The fixed tangent frame is **East, North, Up** at the configured origin. Up is
normal to the reference ellipsoid. A point conversion subtracts the ECEF origin
before rotating; a free vector conversion only rotates. These are separate
methods on `LocalFrame` to avoid silently treating positions as directions.

For origin latitude `φ` and longitude `λ`:

```text
                 [ -sin λ           cos λ          0     ]
R_ENU_from_ECEF = [ -sin φ cos λ     -sin φ sin λ    cos φ ]
                 [  cos φ cos λ      cos φ sin λ    sin φ ]

p_ENU  = R_ENU_from_ECEF (p_ECEF - origin_ECEF)
p_ECEF = origin_ECEF + transpose(R_ENU_from_ECEF) p_ENU

[North, East, Down] = [ENU_North, ENU_East, -ENU_Up]
```

All frames are right-handed. The fixed local frame is appropriate for the short,
small trajectories here; it is not a moving navigation frame on a long flight.
Reference: [ESA ECEF/ENU transformations](https://gssc.esa.int/navipedia/index.php/Transformations_between_ECEF_and_ENU_coordinates).

## Body attitude and quaternions

Body axes are **Forward, Right, Down (FRD)**. Roll, pitch and yaw are aerospace
angles relative to NED. Positive yaw turns from North toward East; positive pitch
raises the nose; positive roll lowers the right side. Define:

```text
R_NED_from_body = Rz(yaw) Ry(pitch) Rx(roll)
R_ENU_from_body = R_ENU_from_NED R_NED_from_body
```

At zero angles, body Forward is North, Right is East, and Down is negative Up.
The body-to-ENU matrix is therefore not identity at zero roll/pitch/yaw.

Quaternions use Hamilton multiplication, `[w, x, y, z]` order, and actively
rotate FRD body vectors to ENU. The zero-Euler quaternion is
`[0, 1/sqrt(2), 1/sqrt(2), 0]`, up to its equivalent overall negative sign.
This differs from a body-to-NED quaternion. Quaternion inversion/conjugation
is necessary when reversing rotation direction.

The inverse Euler conversion uses canonical pitch in `[-pi/2, pi/2]`; at gimbal
lock it chooses roll zero and an equivalent yaw. Quaternions carry the complete
orientation even when individual Euler angles are nonunique.

Reference for PX4 axis conventions:
[PX4 reference frames](https://docs.px4.io/main/en/ros/external_position_estimation#reference-frames).

## IMU origin and antenna offset

The body origin is the IMU. `lever_arm_body_m` points **from the IMU to the
antenna**, expressed in FRD. The default is `[0.10, 0.05, -0.20] m`: 10 cm
forward, 5 cm right, 20 cm above the IMU. It is a configurable example.

```text
p_antenna = p_IMU + R_ENU_from_body lever
v_antenna = v_IMU + R_ENU_from_body (omega_body × lever)
a_antenna = a_IMU + R_ENU_from_body
                       (alpha_body × lever + omega_body × (omega_body × lever))
```

Angular velocity is computed from Euler-angle derivatives; it is not generally
the vector `[roll_rate, pitch_rate, yaw_rate]`. Analytic angular acceleration is
the derivative of that velocity expressed in body coordinates.

With a nonzero offset, rotation alone moves the antenna. With a zero offset,
rotation alone does not translate its phase center. Antenna phase-center variation
and polarization effects are outside this geometric foundation.

The antenna pointing/boresight axis is modeled as negative body Down, independent
of the antenna offset vector. `circle_coning` combines a circular IMU path with
constant pitch (45° by default), zero roll and yaw following the orbit. This
keeps boresight tilted inward at a constant angle from vertical. Direction
endpoints form a circle on the unit sphere; rays from a shared origin form the
cone. The body Forward axis remains orthogonal to boresight. Both unit vectors
and boresight's angle from vertical are exported in the truth CSVs.

## Ideal accelerometer

Use constant local gravity `g_ENU = [0, 0, -9.80665] m/s²`. Accelerometer specific
force at the IMU is:

```text
f_body = transpose(R_ENU_from_body) (a_IMU - g_ENU)
a_IMU  = R_ENU_from_body f_body + g_ENU
```

Thus a level stationary FRD sensor reads `[0, 0, -9.80665] m/s²`. The plotted
specific force is ideal and unfiltered, not an inertial displacement estimate.
Earth rotation, Coriolis, gravity variation, sensor bias and sensor noise are
not included at this stage. All kinematic derivatives are analytic.

## Source geometry, range and phase signs

Azimuth is clockwise from North; elevation is above the horizon. Receiver-to-source
unit directions in ENU are:

```text
u = [cos(elevation) sin(azimuth),
     cos(elevation) cos(azimuth),
     sin(elevation)]
```

The eight fixed source directions named S01–S08 are illustrative, not a real
constellation. Fixed sources are placed 20,200 km from the site for finite-range
checks, not as an orbital model. A shared example direction is also plotted.

For antenna displacement `d` from its own initial position and receiver-to-source
direction `u` at that initial antenna position:

```text
range change ≈ -dot(u, d)
geometric phase contribution = range change / wavelength
```

Moving toward a source reduces range. GPS L1 wavelength is calculated as
`299792458 / 1575420000 m`. These phase contributions omit clock terms, integer
ambiguities, satellite dynamics, propagation effects and receiver noise. They do
not yet implement the paper's complete carrier model. A receiver's accumulated
beat-phase convention must be checked independently before adapting real data;
the sign here is explicitly the range-observable sign.

Finite-source range changes are also computed using the stable expression
`(-2 s·d + d·d) / (|s-d| + |s|)`, where `s` is the source vector at the reference
antenna position. This avoids subtracting two large, nearly equal distances.

## Verification and plots

Tests use known geodetic/basis anchors, random round trips, an independent
SciPy rotation implementation, compass cases, analytic circle dynamics,
finite-difference kinematic/quaternion derivatives, gravity reconstruction,
zero-offset rotation and exact distance checks. Round-trip residuals shown in the
plots are numerical closure checks, not estimates of physical sensor accuracy.

Every motion has equal-scale 3D/top-view paths, body/antenna position-velocity-
acceleration plots, attitude/quaternion/ideal-IMU plots, coordinate conversion
plots and geometric range/phase plots. The standalone HTML includes an animated
3D view with scenario selection, time scrubber, camera controls and numerical
readouts. PNGs and a multipage PDF provide full-resolution scientific artifacts.
