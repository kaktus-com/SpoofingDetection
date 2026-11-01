"""Post-prediction comparison with truth displacement and oracle detector fits.

This module is invoked only after IMU predictions and their provenance have
been saved. Its reference displacement/directions never reach IMU estimation.
"""

import csv
from dataclasses import replace

import numpy as np

from .detector import detect_window, polynomial_basis
from .schema import load_oracle_displacement


def remove_polynomial(times, values):
    p, _, _ = polynomial_basis(times)
    q = np.linalg.qr(p, mode="reduced")[0]
    return values-q@(q.T@values)


def reference_directions(run):
    """Actual propagation directions for EVALUATION only; nominal fallback."""
    path = run["folder"]/"truth"/f"{run['case']}_components.csv"
    if not path.exists():
        return None, "nominal_geometry_response"
    directions = {}
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        keys = ["actual_los_enu_east", "actual_los_enu_north", "actual_los_enu_up"]
        if set(["time_s", "signal_id", *keys])-set(reader.fieldnames or []):
            return None, "nominal_geometry_response"
        for row in reader:
            directions[float(row["time_s"]), row["signal_id"]] = [float(row[key]) for key in keys]
    return directions, "actual_propagation_directions_from_held_out_truth"


def compare_imu_with_oracle(runs):
    rows, unavailable = [], []
    for run in runs:
        run["motion_comparison"] = {}
        oracle_path = run["folder"]/"truth"/"motion_gnss.csv"
        if not oracle_path.exists():
            unavailable.append({"scenario": run["scenario"], "case": run["case"], "reason": "oracle_displacement_unavailable"})
            continue
        oracle = load_oracle_displacement(oracle_path, run["data"].time_s)
        actual_directions, reference_name = reference_directions(run)
        for record in run["records"]:
            estimate = record.motion_estimate
            if estimate is None:
                unavailable.append({"scenario": run["scenario"], "case": run["case"],
                                    "window_index": record.index, "reason": "imu_estimate_unavailable"})
                continue
            times = record.data.time_s
            indices = np.searchsorted(run["data"].time_s, times)
            reference = oracle[indices]-oracle[indices[0]]
            estimated = estimate.antenna_displacement_m
            difference = estimated-reference
            reference_hp = remove_polynomial(times, reference)
            estimated_hp = remove_polynomial(times, estimated)
            observable_error = estimated_hp-reference_hp
            if actual_directions is not None and all((float(t), name) in actual_directions for t in times for name in record.data.signal_ids):
                los = np.array([[actual_directions[float(t), name] for name in record.data.signal_ids] for t in times])
                phase_reference_name = reference_name
            else:
                los = np.broadcast_to(record.data.nominal_los, (len(times), *record.data.nominal_los.shape))
                phase_reference_name = "nominal_geometry_response"
            if not np.isfinite(los).all() or not np.allclose(np.linalg.norm(los, axis=2), 1., atol=1e-8):
                raise ValueError("Held-out phase comparison directions must be finite unit vectors")
            reference_phase = -np.einsum("ni,nsi->ns", reference, los)/record.data.wavelength_m
            estimated_phase = -np.einsum("ni,nsi->ns", estimated, los)/record.data.wavelength_m
            reference_phase_hp = remove_polynomial(times, reference_phase)
            estimated_phase_hp = remove_polynomial(times, estimated_phase)
            phase_error_hp = estimated_phase_hp-reference_phase_hp
            baseline = detect_window(replace(record.data, displacement_m=reference), run["config"])
            imu_result = record.result
            gamma_delta = imu_result.gamma-baseline.gamma if imu_result.gamma is not None and baseline.gamma is not None else None
            row = {"scenario": run["scenario"], "case": run["case"], "window_index": record.index,
                   "start_s": record.start_s, "end_s": record.end_s,
                   "raw_displacement_vector_rmse_m": float(np.linalg.norm(difference)/np.sqrt(len(times))),
                   "observable_displacement_vector_rmse_m": float(np.linalg.norm(observable_error)/np.sqrt(len(times))),
                   "observable_displacement_max_error_m": float(np.max(np.linalg.norm(observable_error, axis=1))),
                   "raw_phase_rmse_cycles": float(np.sqrt(np.mean((estimated_phase-reference_phase)**2))),
                   "observable_phase_rmse_cycles": float(np.sqrt(np.mean(phase_error_hp**2))),
                   "phase_reference": phase_reference_name, "oracle_decision": baseline.decision,
                   "imu_decision": imu_result.decision, "oracle_gamma": baseline.gamma,
                   "imu_gamma": imu_result.gamma, "gamma_difference": gamma_delta,
                   "decision_agrees": baseline.decision == imu_result.decision}
            rows.append(row)
            run["motion_comparison"][record.index] = {
                "metrics": row, "oracle_displacement_m": reference.tolist(),
                "estimated_displacement_m": estimated.tolist(),
                "oracle_observable_displacement_m": reference_hp.tolist(),
                "estimated_observable_displacement_m": estimated_hp.tolist(),
                "oracle_observable_phase_cycles": reference_phase_hp.tolist(),
                "estimated_observable_phase_cycles": estimated_phase_hp.tolist(),
                "observable_phase_error_cycles": phase_error_hp.tolist(),
            }
    summary = {"compared_windows": len(rows), "unavailable_comparisons": len(unavailable),
               "decision_agreements": sum(row["decision_agrees"] for row in rows),
               "conclusive_in_both": sum(row["imu_decision"] != "inconclusive" and row["oracle_decision"] != "inconclusive" for row in rows),
               "max_observable_displacement_vector_rmse_m": max((row["observable_displacement_vector_rmse_m"] for row in rows), default=None),
               "max_observable_phase_rmse_cycles": max((row["observable_phase_rmse_cycles"] for row in rows), default=None),
               "max_absolute_gamma_difference": max((abs(row["gamma_difference"]) for row in rows if row["gamma_difference"] is not None), default=None),
               "interpretation": "held-out oracle comparison AFTER frozen IMU predictions; polynomial projection removes unobservable initial position/velocity drift"}
    return {"status": "post_prediction_oracle_comparison", "summary": summary, "windows": rows, "unavailable": unavailable}
