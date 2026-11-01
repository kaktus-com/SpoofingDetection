"""Generate paired benign/spoofed carrier observations, truth and comparison plots.

Run from the repository root:
    python3 -m SpoofingDetection.detect2.synthetic.generate_carrier
"""

import argparse
from contextlib import nullcontext
from dataclasses import asdict
from pathlib import Path

import numpy as np

from ..geometry import GPS_L1_WAVELENGTH_M, LocalFrame
from .carrier import CASES, MODEL_VERSION, CarrierConfig, generate_pair, pair_metrics
from .generate import write_csv, write_json
from .motions import DEFAULT_LEVER_ARM_M, MotionConfig, SCENARIOS


def vector_columns(columns, prefix, values, labels):
    for i, label in enumerate(labels):
        columns[f"{prefix}_{label}"] = values[:, i]


def export_pair(folder, pair, frame):
    """Export measured fields apart from all latent components and labels."""
    folder.mkdir(parents=True, exist_ok=True)
    (folder/"inputs").mkdir(exist_ok=True)
    (folder/"truth").mkdir(exist_ok=True)
    imu, gnss = pair.imu_motion, pair.gnss_motion
    columns = {"time_s": imu.time_s}
    vector_columns(columns, "specific_force_body", imu.specific_force_body_m_s2,
                   ["forward_m_s2", "right_m_s2", "down_m_s2"])
    vector_columns(columns, "angular_rate_body", imu.angular_rate_body_rad_s,
                   ["forward_rad_s", "right_rad_s", "down_rad_s"])
    vector_columns(columns, "q_body_to_enu", imu.quaternion_wxyz, ["w", "x", "y", "z"])
    write_csv(folder/"inputs"/"imu.csv", columns)
    s, n = len(pair.signal_ids), len(pair.time_s)
    geometry = {"signal_id": pair.signal_ids, "constellation": ["GPS"]*s,
                "band": ["L1"]*s, "wavelength_m": np.full(s, GPS_L1_WAVELENGTH_M)}
    vector_columns(geometry, "nominal_los_enu", pair.geometry["los_enu"], ["east", "north", "up"])
    write_csv(folder/"inputs"/"satellite_geometry.csv", geometry)
    write_json(folder/"inputs"/"frame.json", {
        "origin_wgs84": asdict(frame), "enu_from_ecef": frame.enu_from_ecef.tolist(),
        "body_axes": "Forward, Right, Down", "local_axes": "East, North, Up",
        "quaternion": "Hamilton wxyz, active FRD body to ENU",
        "lever_arm_body_m": list(imu.config.lever_arm_body_m),
        "sensor_to_body_rotation": np.eye(3).tolist(),
        "gravity_enu_m_s2": [0., 0., -9.80665],
        "timebase": "common relative measurement seconds; no latency, jitter or clock offsets",
    })
    for filename, motion in [("motion_imu.csv", imu), ("motion_gnss.csv", gnss)]:
        truth = {"time_s": motion.time_s}
        for prefix, values in [("imu_position_enu", motion.body_position_m),
                               ("antenna_position_enu", motion.antenna_position_m),
                               ("antenna_displacement_enu", motion.antenna_position_m-motion.antenna_position_m[0])]:
            vector_columns(truth, prefix, values, ["east_m", "north_m", "up_m"])
        vector_columns(truth, "antenna_velocity_enu", motion.antenna_velocity_m_s,
                       ["east_m_s", "north_m_s", "up_m_s"])
        vector_columns(truth, "antenna_acceleration_enu", motion.antenna_acceleration_m_s2,
                       ["east_m_s2", "north_m_s2", "up_m_s2"])
        write_csv(folder/"truth"/filename, truth)
    trend = {"signal_id": pair.signal_ids,
             "beta0_cycles": pair.trend_coefficients[0],
             "beta1_cycles_s": pair.trend_coefficients[1],
             "beta2_cycles_s2": pair.trend_coefficients[2]}
    write_csv(folder/"truth"/"phase_trends.csv", trend)
    times = np.repeat(pair.time_s, s)
    ids = np.tile(pair.signal_ids, n)
    for name, case in pair.cases.items():
        # Both cases refer to the SAME nominal geometry and ideal inertial input.
        measured = {"time_s": times, "signal_id": ids,
                    "carrier_phase_cycles": case.phase_cycles.ravel(),
                    "phase_sigma_cycles": np.full(n*s, pair.config.noise_sigma_cycles),
                    "carrier_valid": np.ones(n*s, dtype=int),
                    "lock_time_s": times, "arc_id": np.zeros(n*s, dtype=int)}
        write_csv(folder/name/"carrier.csv", measured)
        components = {"time_s": times, "signal_id": ids,
                      "geometric_phase_cycles": case.geometric_phase_cycles.ravel(),
                      "slow_phase_cycles": case.slow_phase_cycles.ravel(),
                      "noise_cycles": case.noise_cycles.ravel(),
                      "noiseless_phase_cycles": (case.slow_phase_cycles+case.geometric_phase_cycles).ravel()}
        vector_columns(components, "actual_los_enu", np.tile(case.actual_los_enu, (n, 1)), ["east", "north", "up"])
        write_csv(folder/"truth"/f"{name}_components.csv", components)
    write_csv(folder/"truth"/"labels.csv", {
        "time_s": np.tile(pair.time_s, len(CASES)),
        "case": np.repeat(CASES, n),
        "is_spoofed": np.repeat([0, 1], n),
    })
    metrics = pair_metrics(pair)
    write_json(folder/"truth"/"metrics.json", metrics)
    write_json(folder/"truth"/"generation.json", {
        "model_version": MODEL_VERSION, "numpy_version": np.__version__,
        "motion": asdict(imu.config), "analytic_motion_parameters": imu.parameters,
        "carrier": asdict(pair.config),
        "rng": "PCG64 with SHA-256-named SeedSequence streams; independent per case",
        "phase_model": "beta0 + beta1*t + 0.5*beta2*t^2 - dot(actual_los, antenna_displacement)/wavelength + noise",
        "geometry_model": "fixed far-field receiver-to-source ENU directions; illustrative, not real ephemerides",
        "noise_model": "stationary Gaussian AR(1); rho=0 is white; sigma is marginal cycles stddev",
        "imu_model": "ideal specific force, angular rate and exact attitude; no IMU sensor noise or integration",
        "phase_continuity": "all samples valid; no gaps or cycle slips; one continuous arc per signal",
        "endpoint_policy": "samples at exact 1/rate intervals <= requested duration, including t=0",
    })
    manifest = {
        "schema_version": 1, "provenance": "synthetic measurement-level observations",
        "scenario": imu.config.scenario, "timebase": "shared relative seconds, t=0 origin",
        "gnss_rate_hz": pair.config.gnss_rate_hz, "imu_rate_hz": imu.config.rate_hz,
        "imu": "inputs/imu.csv", "imu_quality": "ideal measurements and exact supplied attitude",
        "satellite_geometry": "inputs/satellite_geometry.csv", "frame": "inputs/frame.json",
        "carrier_files": {name: f"{name}/carrier.csv" for name in CASES},
        "phase_units": "cycles, range-observable sign; not receiver beat-phase convention",
        "phase_noise_sigma_cycles": pair.config.noise_sigma_cycles,
        "phase_noise_lag1_rho": pair.config.noise_rho,
        "signal_identity": "S01–S08 are illustrative GPS L1 channels, not real satellite ephemerides",
        "input_boundary": "only imu, satellite_geometry, frame and the selected carrier file are measured inputs; truth/ is evaluation only",
    }
    write_json(folder/"manifest.json", manifest)
    return metrics


def generate_datasets(output, motion_configs, carrier_config=CarrierConfig(), frame=LocalFrame(), plots=True):
    # Evaluate/validate all requested scenarios before mutating output files.
    pairs = [generate_pair(config, carrier_config, frame) for config in motion_configs]
    if not pairs:
        raise ValueError("Choose at least one motion")
    names = [pair.imu_motion.config.scenario for pair in pairs]
    if len(set(names)) != len(names):
        raise ValueError("Each motion scenario may occur only once in an output dataset")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    figures, metrics = {}, {}
    if plots:
        from matplotlib.backends.backend_pdf import PdfPages
        from . import carrier_plots as plotter
        plotter.setup_style()
    with PdfPages(output/"carrier_observations.pdf") if plots else nullcontext(None) as pdf:
        for pair in pairs:
            name = pair.imu_motion.config.scenario
            metrics[name] = export_pair(output/name, pair, frame)
            figures[name] = []
            if plots:
                import matplotlib.pyplot as plt
                factories = {"observations": plotter.observations_plot,
                             "components": plotter.components_plot,
                             "between_signals": plotter.between_signals_plot,
                             "motion_and_sampling": plotter.motion_plot,
                             "noise": plotter.noise_plot}
                for plot_name, factory in factories.items():
                    path = Path(name)/"plots"/(plot_name+".png")
                    (output/path).parent.mkdir(parents=True, exist_ok=True)
                    fig = factory(pair)
                    fig.savefig(output/path)
                    pdf.savefig(fig)
                    plt.close(fig)
                    figures[name].append(str(path))
            print(f"{name}: {len(pair.time_s)} GNSS epochs × {len(pair.signal_ids)} signals per case; "
                  f"{len(pair.imu_motion.time_s)} shared IMU samples", flush=True)
    write_json(output/"manifest.json", {
        "schema_version": 1, "model_version": MODEL_VERSION,
        "description": "paired synthetic carrier observations; no detector decisions computed",
        "scenarios": {name: f"{name}/manifest.json" for name in names},
        "gnss_rate_hz": carrier_config.gnss_rate_hz,
        "evaluation_boundary": "labels, components, latent directions, coefficients and generation settings are under each scenario's truth/ directory",
    })
    from .carrier_report import write_carrier_report
    write_carrier_report(output, pairs, figures, plots)
    print(f"Report: {(output/'report.html').resolve()}", flush=True)
    return metrics


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).resolve().parent/"data"/"carrier_phase")
    parser.add_argument("--scenario", choices=["all", *SCENARIOS], default="all")
    parser.add_argument("--duration-s", type=float, default=6.)
    parser.add_argument("--imu-rate-hz", type=float, default=200.)
    parser.add_argument("--gnss-rate-hz", type=float, default=20.)
    parser.add_argument("--origin", type=float, nargs=3, metavar=("LAT_DEG", "LON_DEG", "ELLIPSOID_HEIGHT_M"), default=(59.3526, 18.0728, 30.))
    parser.add_argument("--lever-arm", type=float, nargs=3, metavar=("FORWARD_M", "RIGHT_M", "DOWN_M"), default=DEFAULT_LEVER_ARM_M)
    parser.add_argument("--amplitude-scale", type=float, default=1.)
    parser.add_argument("--frequency-scale", type=float, default=1.)
    parser.add_argument("--cone-tilt-deg", type=float, default=45.)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--noise-sigma-cycles", type=float, default=.01)
    parser.add_argument("--noise-rho", type=float, default=0., help="stationary AR(1) lag-1 correlation; zero gives white noise")
    parser.add_argument("--no-trend", action="store_true", help="disable all quadratic nuisance terms, including the constant baseline")
    parser.add_argument("--trend-rate-span-cycles-s", type=float, default=2500.)
    parser.add_argument("--trend-acceleration-span-cycles-s2", type=float, default=.2)
    parser.add_argument("--spoofer-direction-deg", type=float, nargs=2, metavar=("AZIMUTH", "ELEVATION"), default=(230., 15.))
    parser.add_argument("--no-plots", action="store_true", help="export data and interactive traces without static PNG/PDF figures")
    args = parser.parse_args(argv)
    names = list(SCENARIOS) if args.scenario == "all" else [args.scenario]
    try:
        motions = [MotionConfig(name, args.duration_s, args.imu_rate_hz, tuple(args.lever_arm),
                                 args.amplitude_scale, args.frequency_scale, args.cone_tilt_deg) for name in names]
        carrier = CarrierConfig(args.gnss_rate_hz, args.noise_sigma_cycles, args.noise_rho, args.seed,
                                not args.no_trend, args.trend_rate_span_cycles_s,
                                args.trend_acceleration_span_cycles_s2, *args.spoofer_direction_deg)
        generate_datasets(args.output_dir, motions, carrier, LocalFrame(*args.origin), not args.no_plots)
    except ValueError as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
