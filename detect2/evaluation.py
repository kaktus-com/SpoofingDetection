"""Label evaluation AFTER predictions have been computed and written."""

import csv

import numpy as np


def evaluate_predictions(runs):
    windows = []
    for run in runs:
        path = run["folder"]/"truth"/"labels.csv"
        labels = {}
        if path.exists():
            with path.open(newline="") as stream:
                for row in csv.DictReader(stream):
                    if row["case"] == run["case"]:
                        time, value = float(row["time_s"]), int(row["is_spoofed"])
                        if not np.isfinite(time) or value not in (0, 1) or time in labels:
                            raise ValueError("Evaluation labels require unique finite times and binary values")
                        labels[time] = value
        for record in run["records"]:
            values = [labels.get(float(time)) for time in record.data.time_s]
            if not values or any(value is None for value in values):
                truth = "unavailable"
            elif len(set(values)) != 1:
                truth = "mixed"
            else:
                truth = "spoofed" if values[0] else "benign"
            decision = record.result.decision
            correct = decision == truth if truth in ("benign", "spoofed") and decision != "inconclusive" else None
            windows.append({"scenario": run["scenario"], "case": run["case"], "window_index": record.index,
                            "truth": truth, "decision": decision, "correct": correct})
    labeled = [row for row in windows if row["truth"] in ("benign", "spoofed")]
    conclusive = [row for row in labeled if row["decision"] != "inconclusive"]
    summary = {"total_windows": len(windows), "labeled_windows": len(labeled),
               "conclusive_labeled_windows": len(conclusive),
               "inconclusive_labeled_windows": len(labeled)-len(conclusive),
               "correct_conclusive": sum(row["correct"] for row in conclusive),
               "false_alarms": sum(row["truth"] == "benign" and row["decision"] == "spoofed" for row in conclusive),
               "missed_detections": sum(row["truth"] == "spoofed" and row["decision"] == "benign" for row in conclusive),
               "conclusive_coverage": len(conclusive)/len(labeled) if labeled else None,
               "accuracy_on_conclusive": sum(row["correct"] for row in conclusive)/len(conclusive) if conclusive else None,
               "displacement_modes": sorted({run.get("mode", "oracle") for run in runs}),
               "interpretation": "synthetic detector validation only; displacement source is recorded separately; inconclusive coverage is reported separately"}
    return {"status": "evaluated_after_prediction", "summary": summary, "windows": windows}
