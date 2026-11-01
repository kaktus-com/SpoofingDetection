"""Offline detector replay with explicit IMU or oracle displacement.

Predictions are written before optional truth evaluation. IMU prediction reads
no truth files; oracle prediction explicitly uses GNSS-epoch truth displacement.
"""

import argparse
import csv
from dataclasses import asdict, dataclass, replace
import json
from pathlib import Path

import numpy as np

from .detector import DetectorConfig, WindowResult, detect_window
from .schema import CarrierSeries, WindowInput, load_carrier, load_inputs
from .motion import MotionEstimate, MotionUnavailable, estimate_antenna_displacement, load_imu
from .imu_errors import ImuErrors, error_metadata, export_imu, perturb_imu
from .motion_uncertainty import assess_motion_noise, attach_motion_check


@dataclass
class WindowRecord:
    index: int
    start_s: float
    end_s: float
    data: WindowInput | CarrierSeries
    result: WindowResult
    motion_estimate: MotionEstimate | None = None


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+"\n")


def write_rows(path, rows, fields=None):
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(rows[0]) if rows else list(fields or [])
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def run_windows(data, config=DetectorConfig(), window_s=1., step_s=1., nominal_interval_s=.05, imu=None):
    if isinstance(data, CarrierSeries) and imu is None:
        raise ValueError("Carrier-only measurements require an IMU estimator or explicit oracle displacement")
    if not np.isfinite([window_s, step_s, nominal_interval_s]).all() or min(window_s, step_s, nominal_interval_s) <= 0:
        raise ValueError("Window, step and nominal sample interval must be finite and positive")
    available_end = data.time_s[-1]+nominal_interval_s
    count = max(0, int(np.floor((available_end-data.time_s[0]-window_s+1e-9)/step_s))+1)
    records = []
    for index in range(count):
        start = data.time_s[0]+index*step_s
        end = start+window_s
        indices = np.flatnonzero((data.time_s >= start-1e-10)&(data.time_s < end-1e-10))
        if not len(indices):
            continue
        window = data.subset(indices)
        estimate, result, motion_check = None, None, None
        if imu is not None:
            try:
                estimate = estimate_antenna_displacement(imu, window.time_s)
                window = (window.with_displacement(estimate.antenna_displacement_m) if isinstance(window, CarrierSeries)
                          else replace(window, displacement_m=estimate.antenna_displacement_m))
                motion_check = assess_motion_noise(estimate, config)
                if not motion_check['passed']:
                    result = WindowResult(diagnostics={'samples': len(window.time_s), 'available_signals': len(window.signal_ids)})
            except MotionUnavailable as exc:
                result = WindowResult(reason=str(exc), diagnostics={"samples": len(window.time_s)})
        if result is None:
            result = detect_window(window, config)
        if motion_check is not None:
            result = attach_motion_check(result, motion_check)
        result.diagnostics["mode"] = "imu_displacement" if imu is not None else "oracle_displacement"
        records.append(WindowRecord(index, float(start), float(end), window, result, estimate))
    if not records:
        records.append(WindowRecord(0, float(data.time_s[0]), float(data.time_s[0]+window_s), data,
                                     WindowResult(reason="incomplete_window", diagnostics={"samples": len(data.time_s), "mode": "imu_displacement" if imu is not None else "oracle_displacement"})))
    return records


def export_records(folder, records, mode="oracle"):
    summaries, details, fitted = [], [], []
    fit_fields = ["window_index", "time_s", "signal_id", "observed_phase_cycles",
                  "benign_fitted_phase_cycles", "spoofed_fitted_phase_cycles",
                  "benign_residual_cycles", "spoofed_residual_cycles",
                  "benign_whitened_residual", "spoofed_whitened_residual"]
    for record in records:
        r, diag = record.result, record.result.diagnostics
        row = {"window_index": record.index, "start_s": record.start_s, "end_s": record.end_s,
               "displacement_mode": mode, "decision": r.decision, "reason": r.reason,
               "gamma": r.gamma, "samples": len(record.data.time_s),
               "used_signals": len(r.signal_indices),
               "motion_rank": diag.get("motion_rank"), "motion_rms_m": diag.get("motion_rms_m"),
               "min_axis_rms_m": diag.get("min_axis_rms_m"), "motion_condition": diag.get("motion_condition"),
               "best_fit_rms_sigma": diag.get("best_fit_rms_sigma"),
               "sigma_floor_applied": diag.get("sigma_floor_applied", False)}
        check = diag.get('imu_motion_noise', {})
        row.update(imu_noise_gate_applied=check.get('applied'), imu_noise_gate_passed=check.get('passed'),
                   noise_bound_rms_m=check.get('noise_bound_rms_m'),
                   expected_noise_vector_rms_m=check.get('expected_noise_vector_rms_m'),
                   min_axis_rms_lower_bound_m=check.get('axis_rms_lower_bound_m', [None])[-1])
        fits = {}
        for name in ["benign", "spoofed"]:
            fit = getattr(r, name)
            row[f"{name}_reduced_cost"] = fit.reduced_cost if fit else None
            row[f"{name}_total_cost"] = fit.total_cost if fit else None
            fits[name] = None if fit is None else {
                "parameters": fit.parameters, "reduced_cost": fit.reduced_cost,
                "total_cost": fit.total_cost, "diagnostics": fit.diagnostics,
                "polynomial_coefficients_centered_scaled_time": fit.polynomial_coefficients.tolist()}
        summaries.append(row)
        details.append({"window_index": record.index, "signal_ids": [record.data.signal_ids[i] for i in r.signal_indices],
                        "diagnostics": diag, "fits": fits,
                        "motion_estimation": record.motion_estimate.diagnostics if record.motion_estimate else None,
                        "motion_uncertainty": None if record.motion_estimate is None else {
                            'raw_temporal_covariance_per_enu_axis_m2': record.motion_estimate.displacement_covariance_m2.tolist(),
                            'observable_temporal_covariance_per_enu_axis_m2': record.motion_estimate.observable_covariance_m2.tolist()}})
        if r.benign is not None and r.spoofed is not None:
            for i, time in enumerate(record.data.time_s):
                for k, j in enumerate(r.signal_indices):
                    fitted.append(dict(zip(fit_fields, [record.index, float(time), record.data.signal_ids[j],
                        float(record.data.phase_cycles[i, j]), float(r.benign.fitted_phase_cycles[i, k]),
                        float(r.spoofed.fitted_phase_cycles[i, k]), float(r.benign.residual_cycles[i, k]),
                        float(r.spoofed.residual_cycles[i, k]), float(r.benign.normalized_residuals[i, k]),
                        float(r.spoofed.normalized_residuals[i, k])])) )
    write_rows(folder/"windows.csv", summaries)
    write_rows(folder/"fits.csv", fitted, fit_fields)
    write_json(folder/"window_details.json", details)
    if mode == "imu":
        motion_rows = []
        fields = ["window_index", "time_s"]
        for prefix, unit in [("antenna_displacement", "m"), ("imu_displacement", "m"),
                              ("imu_velocity", "m_s"), ("lever_displacement", "m")]:
            fields += [f"estimated_{prefix}_enu_{axis}_{unit}" for axis in ["east", "north", "up"]]
        fields += ['raw_white_noise_sigma_per_axis_m', 'observable_white_noise_sigma_per_axis_m']
        for record in records:
            estimate = record.motion_estimate
            if estimate is None:
                continue
            values = np.column_stack((estimate.antenna_displacement_m, estimate.imu_displacement_m,
                                       estimate.imu_velocity_m_s, estimate.lever_displacement_m,
                                       np.sqrt(np.maximum(0., np.diag(estimate.displacement_covariance_m2))),
                                       np.sqrt(np.maximum(0., np.diag(estimate.observable_covariance_m2)))))
            for time, row in zip(estimate.time_s, values):
                motion_rows.append(dict(zip(fields, [record.index, float(time), *row.tolist()])))
        write_rows(folder/"estimated_displacement.csv", motion_rows, fields)


def replay_dataset(data_dir, output_dir, oracle=False, scenarios=None, cases=None,
                    config=DetectorConfig(), window_s=1., step_s=1., evaluate=True,
                    plots=True, noise_rho_override=None, imu=False, compare_oracle=True,
                    imu_errors=None, imu_seed=42, trim_s=0., accel_noise_sigma_override=None):
    if bool(oracle) == bool(imu):
        raise ValueError("Choose exactly one explicit displacement mode: oracle=True or imu=True")
    mode = "imu" if imu else "oracle"
    if imu_errors is not None and not imu:
        raise ValueError('IMU input errors require IMU displacement mode')
    if not np.isfinite(trim_s) or trim_s < 0:
        raise ValueError('Edge trim must be finite and nonnegative')
    data_dir, output_dir = Path(data_dir).resolve(), Path(output_dir).resolve()
    if output_dir == data_dir or data_dir in output_dir.parents:
        raise ValueError("Write detector results outside the observation dataset")
    manifest = json.loads((data_dir/"manifest.json").read_text())
    scenario_files = manifest.get("scenarios", {})
    names = list(scenario_files) if scenarios is None else list(scenarios)
    if not names or any(name not in scenario_files for name in names):
        raise ValueError(f"Choose scenarios from {sorted(scenario_files)}")
    jobs = []
    for name in names:
        scenario_manifest = data_dir/scenario_files[name]
        info = json.loads(scenario_manifest.read_text())
        folder = scenario_manifest.parent
        selected = list(info["carrier_files"]) if cases is None else list(cases)
        if not selected or any(case not in info["carrier_files"] for case in selected):
            raise ValueError(f"Unknown observation case for {name}")
        rate = float(info["gnss_rate_hz"])
        if not np.isfinite(rate) or rate <= 0:
            raise ValueError("Nominal GNSS rate must be positive")
        interval = 1./rate
        rho = float(info.get("phase_noise_lag1_rho", 0.)) if noise_rho_override is None else noise_rho_override
        run_config = replace(config, noise_rho=rho, max_gap_s=1.5*interval)
        imu_path = folder/info["imu"] if imu else None
        frame_path = folder/info["frame"] if imu else None
        accel_sigma = float(info.get('imu_accel_noise_sigma_m_s2', 0.)) if accel_noise_sigma_override is None else accel_noise_sigma_override
        inertial = load_imu(imu_path, frame_path, float(info["imu_rate_hz"]), accel_sigma) if imu else None
        if imu_errors is not None:
            inertial = perturb_imu(inertial, imu_errors, imu_seed, name)
            export_imu(output_dir/'inputs'/name/'imu.csv', inertial)
        for case in selected:
            carrier = folder/info["carrier_files"][case]
            geometry = folder/info["satellite_geometry"]
            oracle_path = folder/"truth"/"motion_gnss.csv"
            data = load_carrier(carrier, geometry) if imu else load_inputs(carrier, geometry, oracle_path)
            if trim_s:
                indices = np.flatnonzero((data.time_s >= data.time_s[0]+trim_s-1e-10)
                                         & (data.time_s < data.time_s[-1]-trim_s-1e-10))
                if len(indices) < 2 or data.time_s[indices[-1]]-data.time_s[indices[0]]+interval < window_s-1e-9:
                    raise ValueError('Edge trim leaves no complete detector window')
                data = data.subset(indices)
            jobs.append({"scenario": name, "case": case, "data": data, "config": run_config,
                         "folder": folder, "interval": interval, "mode": mode, "imu": inertial,
                         "imu_errors": imu_errors,
                         "prediction_inputs": [str(scenario_manifest), str(carrier), str(geometry),
                                               *([str(imu_path), str(frame_path)] if imu else [str(oracle_path)])]})
    output_dir.mkdir(parents=True, exist_ok=True)
    runs = []
    for job in jobs:
        records = run_windows(job["data"], job["config"], window_s, step_s, job["interval"], job["imu"])
        target = output_dir/job["scenario"]/job["case"]
        export_records(target, records, mode)
        runs.append({**job, "records": records, "output": target})
        counts = {name: sum(record.result.decision == name for record in records) for name in ["benign", "spoofed", "inconclusive"]}
        print(f"{job['scenario']}/{job['case']}: {counts}", flush=True)
    # Freeze all predictions before accessing labels for evaluation or plots.
    write_json(output_dir/"run.json", {
        "mode": f"{mode}_displacement", "data_dir": str(data_dir),
        "window_s": window_s, "step_s": step_s,
        "edge_trim_s": trim_s,
        "imu_input_errors": None if imu_errors is None else error_metadata(imu_errors, imu_seed),
        "imu_accel_noise_sigma_override_m_s2": accel_noise_sigma_override,
        "hypotheses": "proper common SO(3) rotation vs common unit LOS; zero gamma threshold",
        "quality_policy": "full-rank 3D, continuity, movement, fit and optimization checks",
        "runs": [{"scenario": run["scenario"], "case": run["case"], "config": asdict(run["config"]),
                   "prediction_inputs": run["prediction_inputs"],
                   "effective_accel_white_noise_sigma_m_s2": float(run['imu'].accel_noise_sigma_m_s2) if imu else None,
                   "perturbed_imu": str(output_dir/'inputs'/run['scenario']/'imu.csv') if imu_errors is not None else None} for run in runs],
    })
    evaluation = {"status": "not_requested", "windows": [], "summary": {}}
    if evaluate:
        from .evaluation import evaluate_predictions
        evaluation = evaluate_predictions(runs)
        write_json(output_dir/"evaluation.json", evaluation)
        write_rows(output_dir/"evaluation.csv", evaluation["windows"],
                    ["scenario", "case", "window_index", "truth", "decision", "correct"])
    motion_comparison = None
    if imu and evaluate and compare_oracle:
        from .motion_evaluation import compare_imu_with_oracle
        motion_comparison = compare_imu_with_oracle(runs)
        write_json(output_dir/"motion_comparison.json", motion_comparison)
        write_rows(output_dir/"motion_comparison.csv", motion_comparison["windows"])
    figures = {name: [] for name in names}
    if plots:
        from matplotlib.backends.backend_pdf import PdfPages
        from . import detector_plots as plotter
        import matplotlib.pyplot as plt
        plotter.setup_style()
        with PdfPages(output_dir/"detector_results.pdf") as pdf:
            for name in names:
                selected = [run for run in runs if run["scenario"] == name]
                artifacts = [(Path(name)/"decisions.png", plotter.decisions_plot(name, selected, evaluation))]
                if motion_comparison and any(run.get("motion_comparison") for run in selected):
                    from . import motion_plots
                    artifacts += [(Path(name)/"motion_estimation.png", motion_plots.displacement_plot(name, selected)),
                                  (Path(name)/"imu_vs_oracle.png", motion_plots.decisions_comparison_plot(name, selected))]
                for run in selected:
                    if any(record.result.benign is not None and record.result.spoofed is not None for record in run["records"]):
                        artifacts += [(Path(name)/run["case"]/"fits.png", plotter.fits_plot(run)),
                                      (Path(name)/run["case"]/"residuals.png", plotter.residuals_plot(run))]
                for path, fig in artifacts:
                    (output_dir/path).parent.mkdir(parents=True, exist_ok=True)
                    fig.savefig(output_dir/path)
                    pdf.savefig(fig)
                    plt.close(fig)
                    figures[name].append(str(path))
    from .detector_report import write_detector_report
    write_detector_report(output_dir, runs, evaluation, figures, plots, motion_comparison)
    print(f"Report: {output_dir/'report.html'}", flush=True)
    return runs, evaluation


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    base = Path(__file__).resolve().parent
    parser.add_argument("--data-dir", type=Path, default=base/"synthetic/data/carrier_phase")
    parser.add_argument("--output-dir", type=Path)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--oracle", action="store_true",
                        help="explicitly permit GNSS-epoch truth displacement; not IMU-estimated movement")
    group.add_argument("--imu", action="store_true", help="estimate movement from specific force and supplied attitude")
    parser.add_argument("--scenario", nargs="+", help="default: all scenarios in the dataset")
    parser.add_argument("--case", nargs="+", help="default: all cases, without using names for prediction")
    parser.add_argument("--window-s", type=float, default=1.)
    parser.add_argument("--step-s", type=float, default=1.)
    parser.add_argument("--sigma-floor-cycles", type=float, default=.001)
    parser.add_argument("--noise-rho", type=float, help="override the public input covariance setting")
    parser.add_argument("--no-evaluate", action="store_true")
    parser.add_argument("--no-plots", action="store_true")
    parser.add_argument("--no-oracle-compare", action="store_true", help="skip post-prediction oracle comparison in IMU mode")
    parser.add_argument('--imu-errors', type=Path, help='JSON ImuErrors settings for a controlled input-error experiment')
    parser.add_argument('--imu-seed', type=int, default=42)
    parser.add_argument('--trim-s', type=float, default=0., help='trim carrier epochs at both recording edges, in seconds')
    parser.add_argument('--accel-noise-sigma-m-s2', type=float, help='configured original per-axis, per-sample accelerometer white-noise sigma; injected noise adds in quadrature')
    parser.add_argument('--no-imu-noise-gate', action='store_true', help='ablation: disable the additional IMU uncertainty quality check')
    parser.add_argument('--imu-noise-tail-probability', type=float, default=.001)
    args = parser.parse_args(argv)
    try:
        errors = None
        if args.imu_errors is not None:
            errors = ImuErrors(**json.loads(args.imu_errors.read_text()))
        output_dir = args.output_dir or base/"results"/("imu_perturbed" if errors is not None else "imu" if args.imu else "oracle")
        replay_dataset(args.data_dir, output_dir, args.oracle, args.scenario, args.case,
                         DetectorConfig(sigma_floor_cycles=args.sigma_floor_cycles,
                                        imu_noise_gate_enabled=not args.no_imu_noise_gate,
                                        imu_noise_tail_probability=args.imu_noise_tail_probability),
                         args.window_s, args.step_s, not args.no_evaluate, not args.no_plots, args.noise_rho,
                         args.imu, not args.no_oracle_compare, errors, args.imu_seed, args.trim_s,
                         args.accel_noise_sigma_m_s2)
    except (ValueError, OSError, KeyError, TypeError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
