"""Portable report of detector outputs, with labels attached only for evaluation."""

import base64
from html import escape
import json
from pathlib import Path


def write_detector_report(output, runs, evaluation, figures, has_plots, motion_comparison=None):
    mode = runs[0].get("mode", "oracle")
    labels = {(row["scenario"], row["case"], row["window_index"]): row["truth"] for row in evaluation.get("windows", [])}
    data = {}
    for run in runs:
        scenario, case = run["scenario"], run["case"]
        cases = data.setdefault(scenario, {})
        cases[case] = []
        for record in run["records"]:
            r = record.result
            has_fits = r.benign is not None and r.spoofed is not None
            indices = r.signal_indices if len(r.signal_indices) else range(len(record.data.signal_ids))
            payload = {"index": record.index, "start": record.start_s, "end": record.end_s,
                       "decision": r.decision, "reason": r.reason, "gamma": r.gamma,
                       "truth": labels.get((scenario, case, record.index), "unavailable"),
                       "time": record.data.time_s.tolist(),
                       "signals": [record.data.signal_ids[j] for j in indices], "hasFits": has_fits,
                       "diagnostics": r.diagnostics, "displacementMode": mode}
            if has_fits:
                observed = record.data.phase_cycles[:, r.signal_indices]
                payload["observed"] = (observed-observed[0]).tolist()
                for name in ["benign", "spoofed"]:
                    fit = getattr(r, name)
                    payload[name] = {"phase": (fit.fitted_phase_cycles-observed[0]).tolist(),
                                     "motionObserved": (observed-fit.trend_cycles).tolist(),
                                     "motion": fit.geometric_phase_cycles.tolist(),
                                     "residual": fit.residual_cycles.tolist(),
                                     "whitened": fit.normalized_residuals.tolist(),
                                     "reducedCost": fit.reduced_cost, "totalCost": fit.total_cost}
            cases[case].append(payload)
    sections = []
    for name, cases in data.items():
        links = []
        for case in cases:
            for filename, label in [("windows.csv", "Window decisions/costs"), ("fits.csv", "Fitted phase/residuals"),
                                     ("window_details.json", "Parameters and diagnostics")]:
                links.append(f'<a href="{name}/{case}/{filename}">{escape(case)}: {label}</a>')
            if mode == "imu":
                links.append(f'<a href="{name}/{case}/estimated_displacement.csv">{escape(case)}: estimated displacement</a>')
        images = []
        for path in figures[name]:
            encoded = base64.b64encode((output/path).read_bytes()).decode("ascii")
            images.append(f'<img loading="lazy" src="data:image/png;base64,{encoded}" alt="{escape(path)}">')
        sections.append(f'<section class="scenario" data-scenario="{name}"><h2>{escape(name)}</h2><p>'
                        +" · ".join(links)+"</p>"+"".join(images)+"</section>")
    summary = evaluation.get("summary", {})
    if summary:
        text = (f"{summary['total_windows']} windows; {summary['conclusive_labeled_windows']} conclusive labeled windows; "
                f"{summary['correct_conclusive']} correct; {summary['inconclusive_labeled_windows']} inconclusive; "
                f"{summary['false_alarms']} false alarms; {summary['missed_detections']} missed detections.")
    else:
        text = "Label evaluation was not requested."
    links = ['<a href="run.json">Run settings and input provenance</a>']
    if evaluation.get("status") == "evaluated_after_prediction":
        links += ['<a href="evaluation.json">Evaluation summary</a>', '<a href="evaluation.csv">Labels versus frozen predictions</a>']
    if has_plots:
        links.append('<a href="detector_results.pdf">All figures as PDF</a>')
    if motion_comparison is not None:
        links += ['<a href="motion_report.html">IMU versus oracle motion and phase</a>',
                  '<a href="motion_comparison.json">Held-out motion comparison metrics</a>']
    title = "IMU-displacement detector" if mode == "imu" else "Oracle-displacement detector"
    note = ("Movement is estimated from IMU specific force and supplied attitude, with zero initial velocity in each window. "
            "No truth displacement or velocity is used before predictions are saved. Any injected input errors are recorded in run settings. "
            "These synthetic results do not establish real-sensor performance, attitude estimation or full navigation." if mode == "imu" else
            "Movement is supplied as known synthetic antenna displacement. This does not demonstrate IMU integration or real-world performance.")
    template = Path(__file__).with_name("detector_report_template.html").read_text()
    for marker, value in {"__DATA__": json.dumps(data, allow_nan=False), "__SUMMARY__": escape(text),
                           "__LINKS__": " · ".join(links), "__SECTIONS__": "".join(sections),
                           "__MODE_TITLE__": title, "__MODE_NOTE__": escape(note)}.items():
        template = template.replace(marker, value)
    (output/"report.html").write_text(template)
    if motion_comparison is not None:
        from .motion_report import write_motion_report
        write_motion_report(output, runs, motion_comparison, figures)
