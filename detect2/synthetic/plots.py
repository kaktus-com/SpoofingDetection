"""Publication-style plots of the motion/geometry foundation, without a detector."""

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from ..geometry import (ENU_FROM_NED, WGS84_A_M, WGS84_B_M,
                        body_to_enu_matrix, enu_to_ned)

COLORS = ["#2563a6", "#d56824", "#238563"]
AXES = ["East", "North", "Up"]
PLOT_NAMES = {
    "path": "Body and antenna paths",
    "kinematics": "Position, velocity and acceleration",
    "attitude_imu": "Attitude, quaternion, angular rate and ideal accelerometer",
    "conversions": "ECEF, ENU, NED and geodetic conversions",
    "projections": "Direction-dependent range and phase contributions",
}


def setup_style():
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9,
                         "axes.titlesize": 10, "axes.labelsize": 9,
                         "axes.spines.top": False, "axes.spines.right": False,
                         "axes.grid": True, "grid.alpha": .2,
                         "figure.dpi": 110, "savefig.dpi": 150,
                         "legend.fontsize": 8, "lines.linewidth": 1.3})


def components(ax, t, values, labels=AXES):
    for i, label in enumerate(labels):
        ax.plot(t, values[:, i], color=COLORS[i % len(COLORS)], label=label)
    ax.legend(loc="upper right", ncol=3)
    ax.set_xlabel("Time (s)")


def equal_3d(ax, points, minimum=.1):
    lo, hi = np.min(points, axis=0), np.max(points, axis=0)
    center = (lo + hi) / 2
    radius = max(np.max(hi-lo) * .6, minimum)
    ax.set_xlim(center[0]-radius, center[0]+radius)
    ax.set_ylim(center[1]-radius, center[1]+radius)
    ax.set_zlim(center[2]-radius, center[2]+radius)
    ax.set_box_aspect([1, 1, 1])
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_zlabel("Up (m)")
    ax.view_init(elev=25, azim=-55)


def path_plot(motion):
    m = motion
    fig = plt.figure(figsize=(11, 5), layout="constrained")
    ax = fig.add_subplot(121, projection="3d")
    ax.plot(*m.body_position_m.T, "--", color="#475569", label="IMU/body origin")
    ax.plot(*m.antenna_position_m.T, color="#8b3bb0", label="Antenna")
    for k in np.linspace(0, len(m.time_s)-1, 6, dtype=int):
        ax.plot(*np.stack([m.body_position_m[k], m.antenna_position_m[k]]).T,
                color="#64748b", alpha=.4, linewidth=.8)
    ax.scatter(*m.body_position_m[0], color="#475569", s=20)
    ax.scatter(*m.antenna_position_m[0], color="#8b3bb0", s=20)
    equal_3d(ax, np.concatenate([m.body_position_m, m.antenna_position_m]))
    ax.legend(loc="upper left")
    ax.set_title("3D path; rods connect the IMU to the antenna")
    ax = fig.add_subplot(122)
    ax.plot(*m.body_position_m[:, :2].T, "--", color="#475569", label="IMU/body origin")
    ax.plot(*m.antenna_position_m[:, :2].T, color="#8b3bb0", label="Antenna")
    ax.scatter(*m.antenna_position_m[0, :2], color="#8b3bb0", label="Start", s=25)
    ax.set(xlabel="East (m)", ylabel="North (m)", title="Top view (equal distance scale)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend()
    fig.suptitle(f"{m.config.scenario}: body and antenna movement")
    return fig


def kinematics_plot(m):
    fig, axes = plt.subplots(3, 2, figsize=(11, 9), sharex=True, sharey="row", layout="constrained")
    pairs = [(m.body_position_m, m.antenna_position_m, "Position (m)"),
             (m.body_velocity_m_s, m.antenna_velocity_m_s, "Velocity (m/s)"),
             (m.body_acceleration_m_s2, m.antenna_acceleration_m_s2, "Acceleration (m/s²)")]
    for row, (body, antenna, units) in enumerate(pairs):
        for col, data in enumerate([body, antenna]):
            components(axes[row, col], m.time_s, data)
            axes[row, col].set_ylabel(units)
    axes[0, 0].set_title("IMU/body origin — ENU")
    axes[0, 1].set_title("Antenna — ENU (includes rotation about the IMU)")
    fig.suptitle(f"{m.config.scenario}: analytic kinematics; matching row scales")
    return fig


def attitude_plot(m):
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), layout="constrained")
    components(axes[0, 0], m.time_s, np.degrees(m.rpy_rad), ["Roll", "Pitch", "Yaw"])
    axes[0, 0].set(title="Aerospace attitude relative to NED", ylabel="Angle (deg)")
    for i, label in enumerate(["w", "x", "y", "z"]):
        axes[0, 1].plot(m.time_s, m.quaternion_wxyz[:, i], label=label)
    axes[0, 1].legend(ncol=4)
    axes[0, 1].set(title="Body → ENU Hamilton quaternion [w,x,y,z]",
                    ylabel="Quaternion component", xlabel="Time (s)")
    components(axes[1, 0], m.time_s, m.angular_rate_body_rad_s, ["Forward", "Right", "Down"])
    axes[1, 0].set(title="Body angular velocity (not Euler-angle derivatives)", ylabel="Angular rate (rad/s)")
    components(axes[1, 1], m.time_s, m.specific_force_body_m_s2, ["Forward", "Right", "Down"])
    axes[1, 1].set(title="Ideal accelerometer at the IMU; gravity included", ylabel="Specific force (m/s²)")
    fig.suptitle(f"{m.config.scenario}: attitude and ideal inertial quantities")
    return fig


def conversion_plot(m, frame, ecef, llh, errors):
    fig, axes = plt.subplots(3, 2, figsize=(11, 9), layout="constrained")
    components(axes[0, 0], m.time_s, ecef-frame.origin_ecef_m, ["X", "Y", "Z"])
    axes[0, 0].set(title="ECEF displacement from the site origin", ylabel="ECEF Δcoordinate (m)")
    components(axes[0, 1], m.time_s, m.antenna_position_m)
    axes[0, 1].set(title="Local ENU position", ylabel="ENU coordinate (m)")
    components(axes[1, 0], m.time_s, enu_to_ned(m.antenna_position_m), ["North", "East", "Down"])
    axes[1, 0].set(title="Same position in local NED", ylabel="NED coordinate (m)")
    axes[1, 1].plot(m.time_s, (np.degrees(llh[:, 0])-frame.latitude_deg)*1e6, label="Latitude")
    lon_delta = (np.degrees(llh[:, 1])-frame.longitude_deg+180) % 360-180
    axes[1, 1].plot(m.time_s, lon_delta*1e6, label="Longitude")
    axes[1, 1].legend()
    axes[1, 1].set(title="Angular offsets from the site", ylabel="Offset (µdeg)", xlabel="Time (s)")
    axes[2, 0].plot(m.time_s, llh[:, 2])
    axes[2, 0].ticklabel_format(useOffset=False, axis="y")
    axes[2, 0].set(title="WGS-84 ellipsoidal height (not MSL)", ylabel="Height (m)", xlabel="Time (s)")
    for label, error in errors.items():
        axes[2, 1].plot(m.time_s, error*1e9, label=label)
    axes[2, 1].legend()
    axes[2, 1].set(title="Numerical round-trip closure", ylabel="Error norm (nm)", xlabel="Time (s)")
    fig.suptitle(f"{m.config.scenario}: coordinate conversions of the antenna position")
    return fig


def projection_plot(m, geometry, projection):
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), layout="constrained")
    for i, name in enumerate(geometry["names"]):
        axes[0, 0].plot(m.time_s, projection["linear_range_m"][:, i]*1000, label=name)
        axes[0, 1].plot(m.time_s, projection["linear_phase_cycles"][:, i], label=name)
        axes[1, 1].plot(m.time_s, projection["linearization_error_m"][:, i]*1e9)
    axes[0, 0].legend(ncol=4)
    axes[0, 0].set(title="Different directions: geometric range change", ylabel="Range change (mm)")
    axes[0, 1].set(title="Same geometric contribution / GPS L1 wavelength", ylabel="Phase contribution (cycles)")
    axes[1, 0].plot(m.time_s, projection["shared_range_m"]*1000, color="#8b3bb0")
    axes[1, 0].set(title="One shared direction: all equal-wavelength signals coincide", ylabel="Range change (mm)")
    axes[1, 1].set(title="Finite-source range minus first-order projection", ylabel="Approximation error (nm)")
    for ax in axes.flat:
        ax.set_xlabel("Time (s)")
    fig.suptitle(f"{m.config.scenario}: geometric contributions only; no receiver measurements")
    return fig


def orientation_cone_plot(m):
    """Separate the translating antenna position from its unit pointing axis."""
    fig = plt.figure(figsize=(12, 9), layout="constrained")
    boresight, forward = m.antenna_boresight_enu, m.body_forward_enu
    tilt = np.radians(m.config.cone_tilt_deg)
    radius = m.parameters["radius_m"]
    ax = fig.add_subplot(221, projection="3d")
    ax.plot(*m.body_position_m.T, "--", color="#475569", label="IMU orbit")
    ax.plot(*m.antenna_position_m.T, color="#8b3bb0", label="Offset antenna orbit")
    # Sample one orbit; evenly sampling six whole turns can alias to two arrows.
    orbit_end = min(len(m.time_s)-1, int(round(m.config.rate_hz/m.parameters["frequency_hz"])))
    for k in np.unique(np.linspace(0, orbit_end, 13, dtype=int)[:-1]):
        ax.quiver(*m.antenna_position_m[k], *boresight[k],
                  length=.8*radius, color="#8b3bb0", alpha=.6)
    center = np.array([-radius, 0., 0.])
    ax.scatter(*center, color="#475569", marker="x", label="Body orbit center")
    equal_3d(ax, np.concatenate([m.body_position_m, m.antenna_position_m,
                                 m.antenna_position_m+.8*radius*boresight]), minimum=radius)
    ax.legend(loc="upper left", fontsize=7)
    ax.set_title("Actual circular movement; arrows show antenna pointing")

    ax = fig.add_subplot(222, projection="3d")
    phi, fraction = np.meshgrid(np.linspace(0, 2*np.pi, 70), np.linspace(0, 1, 10))
    ax.plot_surface(fraction*np.sin(tilt)*np.cos(phi), fraction*np.sin(tilt)*np.sin(phi),
                    fraction*np.cos(tilt), color="#8b3bb0", alpha=.12, linewidth=0)
    ax.plot(*boresight.T, color="#8b3bb0", label="Antenna pointing endpoints")
    ax.plot([0, 0], [0, 0], [0, 1.], "--", color="#475569", label="Vertical")
    for direction, label, color in [(boresight[0], "Antenna Up (−body Down)", "#8b3bb0"),
                                     (forward[0], "Body Forward at same time", "#d56824")]:
        ax.quiver(0, 0, 0, *direction, color=color, length=1, label=label)
    arc = np.linspace(0, tilt, 30)
    ax.plot(-.5*np.sin(arc), np.zeros(len(arc)), .5*np.cos(arc), color="#c13c3c")
    ax.text(-.55*np.sin(tilt/2), 0, .55*np.cos(tilt/2), f"{m.config.cone_tilt_deg:g}°", color="#c13c3c")
    ax.set(xlabel="East unit component", ylabel="North unit component", zlabel="Up unit component",
           xlim=(-1.1, 1.1), ylim=(-1.1, 1.1), zlim=(-.1, 1.1),
           title="Pointing vectors relocated to one origin: cone sweep")
    ax.set_box_aspect([2.2, 2.2, 1.2])
    ax.view_init(elev=25, azim=-55)
    ax.legend(loc="upper left", fontsize=7)

    ax = fig.add_subplot(223)
    components(ax, m.time_s, boresight)
    ax.set(title="Antenna pointing axis in ENU (unit vector)", ylabel="Direction component")
    ax = fig.add_subplot(224)
    antenna_tilt = np.degrees(np.arccos(np.clip(boresight[:, 2], -1, 1)))
    forward_tilt = np.degrees(np.arccos(np.clip(forward[:, 2], -1, 1)))
    separation = np.degrees(np.arccos(np.clip(np.sum(boresight*forward, axis=1), -1, 1)))
    ax.plot(m.time_s, antenna_tilt, color="#8b3bb0", label="Antenna axis / vertical")
    ax.plot(m.time_s, forward_tilt, "--", color="#d56824", label="Body Forward / vertical")
    ax.plot(m.time_s, separation, ":", color="#475569", label="Antenna axis / Body Forward")
    ax.set(title="Fixed tilt; perpendicular body axes", xlabel="Time (s)", ylabel="Angle (deg)", ylim=(-5, 100))
    ax.legend()
    fig.suptitle(f"{m.config.scenario}: {m.config.cone_tilt_deg:g}° cone half-angle from vertical")
    return fig


def frames_plot(frame):
    # Explicit outer margins keep projected 3D tick labels inside the export.
    fig = plt.figure(figsize=(14, 6))
    fig.subplots_adjust(left=.02, right=.94, bottom=.12, top=.80, wspace=.16)
    ax = fig.add_subplot(121, projection="3d")
    lon, lat = np.meshgrid(np.linspace(0, 2*np.pi, 40), np.linspace(-np.pi/2, np.pi/2, 22))
    ax.plot_wireframe(WGS84_A_M/1000*np.cos(lat)*np.cos(lon),
                      WGS84_A_M/1000*np.cos(lat)*np.sin(lon), WGS84_B_M/1000*np.sin(lat),
                      color="#94a3b8", alpha=.35, linewidth=.5)
    site = frame.origin_ecef_m/1000
    ax.scatter(*site, color="#8b3bb0", s=25)
    ax.text(*site, " Site", fontsize=8)
    for i, (name, color) in enumerate(zip(AXES, COLORS)):
        direction = frame.enu_from_ecef.T[:, i]
        ax.quiver(*site, *direction, length=2000, color=color)
        ax.text(*(site+direction*2300), name, color=color, fontsize=8)
    ax.set(xlabel="ECEF X (km)", ylabel="ECEF Y (km)", zlabel="ECEF Z (km)",
           title="WGS-84 and the site tangent basis\nDirection arrows enlarged for visibility")
    ax.set_box_aspect([1, 1, 1])
    for setter in [ax.set_xlim, ax.set_ylim, ax.set_zlim]:
        setter(-8500, 8500)
    ax.view_init(elev=24, azim=35)
    ax = fig.add_subplot(122, projection="3d")
    tripods = [(np.array([-2., 0, 0]), np.eye(3), AXES, "ENU"),
               (np.zeros(3), ENU_FROM_NED, ["North", "East", "Down"], "NED"),
               (np.array([2., 0, 0]), body_to_enu_matrix([0, 0, 0]),
                ["Forward", "Right", "Down"], "Level FRD, yaw=0")]
    for origin, basis, names, label in tripods:
        for i, name in enumerate(names):
            direction = basis[:, i]
            ax.quiver(*origin, *direction, color=COLORS[i], length=.8)
            ax.text(*(origin+direction*.95), name, fontsize=8)
        ax.text(*(origin+[0, 0, -.6]), label, fontsize=8)
    ax.set(xlabel="East", ylabel="North", zlabel="Up", xlim=(-3.2, 3.4),
           ylim=(-.5, 1.8), zlim=(-1.4, 1.4),
           title="ENU, NED and body axes at zero attitude\nDisplay origins separated; bases have unit scale")
    ax.set_box_aspect([2.5, 1, 1])
    ax.view_init(elev=23, azim=-62)
    fig.suptitle("Coordinate conventions: latitude is geodetic; heights are ellipsoidal")
    return fig


def sky_plot(geometry):
    fig = plt.figure(figsize=(11, 5), layout="constrained")
    ax = fig.add_subplot(121, projection="polar")
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)
    ax.set_ylim(0, 90)
    ax.set_yticks([0, 30, 60, 90], ["90°", "60°", "30°", "0°"])
    for i, name in enumerate(geometry["names"]):
        az, el = geometry["azimuth_deg"][i], geometry["elevation_deg"][i]
        ax.scatter(np.radians(az), 90-el)
        ax.annotate(name, (np.radians(az), 90-el), xytext=(5, 5), textcoords="offset points")
    ax.scatter(np.radians(230), 75, color="#8b3bb0", marker="x", s=50, label="Shared direction")
    ax.set_title("Sky plot: elevation labels; azimuth from North")
    ax.legend(loc="lower right", bbox_to_anchor=(1.2, -.1))
    ax = fig.add_subplot(122, projection="3d")
    for i, name in enumerate(geometry["names"]):
        v = geometry["los_enu"][i]
        ax.quiver(0, 0, 0, *v, length=1)
        ax.text(*v*1.1, name, fontsize=8)
    ax.quiver(0, 0, 0, *geometry["shared_los_enu"], color="#8b3bb0", length=1)
    equal_3d(ax, np.array([[-1, -1, -1], [1, 1, 1]]))
    ax.set(xlabel="East unit component", ylabel="North unit component", zlabel="Up unit component",
           title="Receiver → source unit directions")
    fig.suptitle("Illustrative source geometry — not real satellite ephemerides")
    return fig


def overview_plot(motions):
    fig = plt.figure(figsize=(12, 10), layout="constrained")
    for i, m in enumerate(motions):
        ax = fig.add_subplot(3, 3, i+1, projection="3d")
        ax.plot(*m.body_position_m.T, "--", color="#475569", label="IMU")
        ax.plot(*m.antenna_position_m.T, color="#8b3bb0", label="Antenna")
        equal_3d(ax, np.concatenate([m.body_position_m, m.antenna_position_m]))
        ax.set_title(m.config.scenario)
        if i == 0:
            ax.legend()
    fig.suptitle("Motion controls — each panel uses equal 3D axis scales, independently fitted")
    return fig
