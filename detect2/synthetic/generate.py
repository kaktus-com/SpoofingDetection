"""Generate analytic motion truth, coordinate checks and a visual report.

Run from the repository root:
    python3 -m SpoofingDetection.detect2.synthetic.generate

This foundation does not simulate receiver tracking loops, noisy IMU samples or
spoofing decisions. Range/phase outputs are explicitly geometric contributions.
"""

import argparse
from contextlib import nullcontext
import csv
from dataclasses import asdict
import json
from pathlib import Path

import numpy as np

from ..geometry import (GPS_L1_WAVELENGTH_M, LocalFrame, az_el_to_enu,
                        ecef_to_geodetic, enu_to_ned, finite_range_change,
                        geodetic_to_ecef, projected_range_change)
from .motions import DEFAULT_LEVER_ARM_M, MotionConfig, SCENARIOS, generate_motion


def write_csv(path, columns):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        writer.writerows(zip(*columns.values()))


def write_json(path, data):
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")


def source_geometry(frame):
    az = np.array([15., 60., 110., 160., 210., 260., 305., 345.])
    el = np.array([25., 65., 35., 55., 30., 70., 40., 60.])
    los = az_el_to_enu(np.radians(az), np.radians(el))
    return {"names": [f"S{i+1:02d}" for i in range(len(az))],
            "azimuth_deg": az, "elevation_deg": el, "los_enu": los,
            "los_ecef": frame.enu_vector_to_ecef(los),
            "source_enu_m": los * 20200000.,
            "shared_los_enu": az_el_to_enu(np.radians(230.), np.radians(15.)),
            "illustrative_range_m": 20200000.}


def project_motion(motion, geometry):
    # Sources are fixed at known ENU points; LOS is evaluated at antenna t=0.
    d = motion.antenna_position_m-motion.antenna_position_m[0]
    source_vectors = geometry["source_enu_m"]-motion.antenna_position_m[0]
    los = source_vectors/np.linalg.norm(source_vectors, axis=1)[:, None]
    linear = projected_range_change(d, los)
    exact = finite_range_change(d, source_vectors)
    shared = projected_range_change(d, geometry["shared_los_enu"][None, :])[:, 0]
    return {"linear_range_m": linear, "exact_range_m": exact,
            "linear_phase_cycles": linear/GPS_L1_WAVELENGTH_M,
            "linearization_error_m": exact-linear, "shared_range_m": shared}


def motion_coordinates(m, frame):
    ecef = frame.enu_to_ecef(m.antenna_position_m)
    llh = ecef_to_geodetic(ecef)
    errors = {"ENU → ECEF → ENU": np.linalg.norm(frame.ecef_to_enu(ecef)-m.antenna_position_m, axis=1),
              "ECEF → geodetic → ECEF": np.linalg.norm(geodetic_to_ecef(*llh.T)-ecef, axis=1)}
    return ecef, llh, errors


def export_motion(output, m, frame, geometry):
    folder = output/m.config.scenario
    folder.mkdir(parents=True, exist_ok=True)
    ecef, llh, errors = motion_coordinates(m, frame)
    projection = project_motion(m, geometry)
    columns = {"time_s": m.time_s}
    vectors = [
        ("imu_position_enu", m.body_position_m, ["east_m", "north_m", "up_m"]),
        ("imu_velocity_enu", m.body_velocity_m_s, ["east_m_s", "north_m_s", "up_m_s"]),
        ("imu_acceleration_enu", m.body_acceleration_m_s2, ["east_m_s2", "north_m_s2", "up_m_s2"]),
        ("antenna_position_enu", m.antenna_position_m, ["east_m", "north_m", "up_m"]),
        ("antenna_velocity_enu", m.antenna_velocity_m_s, ["east_m_s", "north_m_s", "up_m_s"]),
        ("antenna_acceleration_enu", m.antenna_acceleration_m_s2, ["east_m_s2", "north_m_s2", "up_m_s2"]),
        ("attitude", m.rpy_rad, ["roll_rad", "pitch_rad", "yaw_rad"]),
        ("q_body_to_enu", m.quaternion_wxyz, ["w", "x", "y", "z"]),
        ("antenna_boresight_enu", m.antenna_boresight_enu, ["east", "north", "up"]),
        ("body_forward_enu", m.body_forward_enu, ["east", "north", "up"]),
        ("angular_rate_body", m.angular_rate_body_rad_s, ["forward_rad_s", "right_rad_s", "down_rad_s"]),
        ("angular_acceleration_body", m.angular_acceleration_body_rad_s2, ["forward_rad_s2", "right_rad_s2", "down_rad_s2"]),
        ("ideal_specific_force_body", m.specific_force_body_m_s2, ["forward_m_s2", "right_m_s2", "down_m_s2"]),
        ("antenna_position_ned", enu_to_ned(m.antenna_position_m), ["north_m", "east_m", "down_m"]),
        ("antenna_ecef", ecef, ["x_m", "y_m", "z_m"]),
        ("antenna_geodetic", np.column_stack((np.degrees(llh[:, :2]), llh[:, 2])),
         ["latitude_deg", "longitude_deg", "ellipsoidal_height_m"]),
    ]
    for prefix, values, labels in vectors:
        for i, label in enumerate(labels):
            columns[f"{prefix}_{label}"] = values[:, i]
    columns["antenna_tilt_from_vertical_deg"] = np.degrees(
        np.arccos(np.clip(m.antenna_boresight_enu[:, 2], -1., 1.)))
    write_csv(folder/"truth.csv", columns)
    columns = {"time_s": m.time_s, "shared_direction_range_change_m": projection["shared_range_m"]}
    for i, name in enumerate(geometry["names"]):
        for key in ["linear_range_m", "exact_range_m", "linear_phase_cycles", "linearization_error_m"]:
            columns[f"{name}_{key}"] = projection[key][:, i]
    write_csv(folder/"geometric_projections.csv", columns)
    metrics = {"samples": len(m.time_s), "actual_duration_s": float(m.time_s[-1]),
               "max_antenna_speed_m_s": float(np.max(np.linalg.norm(m.antenna_velocity_m_s, axis=1))),
               "max_antenna_acceleration_m_s2": float(np.max(np.linalg.norm(m.antenna_acceleration_m_s2, axis=1))),
               "max_enu_ecef_roundtrip_m": float(np.max(next(iter(errors.values())))),
               "max_geodetic_ecef_roundtrip_m": float(np.max(list(errors.values())[1])),
               "max_quaternion_norm_error": float(np.max(np.abs(np.linalg.norm(m.quaternion_wxyz, axis=1)-1))),
               "max_range_linearization_error_m": float(np.max(np.abs(projection["linearization_error_m"])))}
    write_json(folder/"config.json", {"motion": asdict(m.config), "analytic_parameters": m.parameters,
                                     "origin": asdict(frame), "provenance": "analytic synthetic truth",
                                     "model": "fixed tangent frame; uniform gravity; no Earth-rate terms",
                                     "phase_convention": "range contribution / wavelength, not accumulated beat-phase sign",
                                     "wavelength_m": GPS_L1_WAVELENGTH_M, "metrics": metrics})
    return ecef, llh, errors, projection, metrics


def generate(output, scenarios, frame, duration_s=6., rate_hz=200.,
             lever_arm_body_m=DEFAULT_LEVER_ARM_M, amplitude_scale=1., frequency_scale=1., plots=True,
             cone_tilt_deg=45.):
    # Validate all configurations before creating output files.
    configs = [MotionConfig(name, duration_s, rate_hz, tuple(lever_arm_body_m),
                             amplitude_scale, frequency_scale, cone_tilt_deg) for name in scenarios]
    motions = [generate_motion(config) for config in configs]
    if not motions:
        raise ValueError("Choose at least one scenario")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    geometry = source_geometry(frame)
    columns = {"source_id": geometry["names"], "azimuth_deg": geometry["azimuth_deg"],
               "elevation_deg": geometry["elevation_deg"]}
    for key in ["los_enu", "los_ecef", "source_enu_m"]:
        for i, axis in enumerate(["east", "north", "up"] if "enu" in key else ["x", "y", "z"]):
            columns[f"{key}_{axis}"] = geometry[key][:, i]
    write_csv(output/"illustrative_geometry.csv", columns)
    write_json(output/"site.json", {"origin": asdict(frame), "origin_ecef_m": frame.origin_ecef_m.tolist(),
                                   "enu_from_ecef": frame.enu_from_ecef.tolist(),
                                   "source_range_from_site_m": geometry["illustrative_range_m"],
                                   "geometry_provenance": "illustrative fixed sources, not satellite ephemerides",
                                   "shared_direction_azimuth_deg": 230., "shared_direction_elevation_deg": 15.})
    global_figures, scenario_figures, all_metrics = [], {}, {}
    if plots:
        from matplotlib.backends.backend_pdf import PdfPages
        from . import plots as plotter
        plotter.setup_style()
    with PdfPages(output/"motion_geometry.pdf") if plots else nullcontext(None) as pdf:
        def save(fig, path):
            import matplotlib.pyplot as plt
            fig.savefig(output/path)
            pdf.savefig(fig)
            plt.close(fig)
            return str(path)

        if plots:
            for name, fig in [("coordinate_frames", plotter.frames_plot(frame)),
                              ("source_geometry", plotter.sky_plot(geometry)),
                              ("motion_overview", plotter.overview_plot(motions))]:
                global_figures.append(save(fig, Path(name+".png")))
        for m in motions:
            ecef, llh, errors, projection, metrics = export_motion(output, m, frame, geometry)
            all_metrics[m.config.scenario] = metrics
            scenario_figures[m.config.scenario] = []
            if plots:
                factories = {"path": lambda: plotter.path_plot(m),
                             "kinematics": lambda: plotter.kinematics_plot(m),
                             "attitude_imu": lambda: plotter.attitude_plot(m),
                             "conversions": lambda: plotter.conversion_plot(m, frame, ecef, llh, errors),
                             "projections": lambda: plotter.projection_plot(m, geometry, projection)}
                if m.config.scenario == "circle_coning":
                    factories["orientation_cone"] = lambda: plotter.orientation_cone_plot(m)
                for name, factory in factories.items():
                    scenario_figures[m.config.scenario].append(save(factory(), Path(m.config.scenario)/(name+".png")))
            print(f"{m.config.scenario}: {len(m.time_s)} samples; ENU round-trip "
                  f"{metrics['max_enu_ecef_roundtrip_m']:.2e} m", flush=True)
    write_json(output/"metrics.json", all_metrics)
    from .report import write_report
    write_report(output, motions, frame, global_figures, scenario_figures, plots)
    print(f"Report: {(output/'report.html').resolve()}", flush=True)
    return all_metrics


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).resolve().parent/"data"/"motion_geometry")
    parser.add_argument("--scenario", choices=["all", *SCENARIOS], default="all")
    parser.add_argument("--duration-s", type=float, default=6.)
    parser.add_argument("--rate-hz", type=float, default=200.)
    parser.add_argument("--origin", type=float, nargs=3, metavar=("LAT_DEG", "LON_DEG", "ELLIPSOID_HEIGHT_M"),
                        default=(59.3526, 18.0728, 30.))
    parser.add_argument("--lever-arm", type=float, nargs=3, metavar=("FORWARD_M", "RIGHT_M", "DOWN_M"),
                        default=DEFAULT_LEVER_ARM_M)
    parser.add_argument("--amplitude-scale", type=float, default=1.)
    parser.add_argument("--frequency-scale", type=float, default=1.)
    parser.add_argument("--cone-tilt-deg", type=float, default=45.,
                        help="circle_coning antenna tilt from vertical, between 0 and 90 degrees")
    parser.add_argument("--no-plots", action="store_true", help="export data and interactive view without static plots/PDF")
    args = parser.parse_args(argv)
    try:
        generate(args.output_dir, list(SCENARIOS) if args.scenario == "all" else [args.scenario],
                 LocalFrame(*args.origin), args.duration_s, args.rate_hz, args.lever_arm,
                 args.amplitude_scale, args.frequency_scale, not args.no_plots, args.cone_tilt_deg)
    except ValueError as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
