"""Portable comparison report for paired synthetic carrier observations."""

import base64
from html import escape
import json
from pathlib import Path

from .carrier import CASES, pair_metrics


def write_carrier_report(output, pairs, figures, has_plots):
    data, sections, options = {}, [], []
    names = [pair.imu_motion.config.scenario for pair in pairs]
    preferred = "three_axis" if "three_axis" in names else names[0]
    for pair in pairs:
        name = pair.imu_motion.config.scenario
        data[name] = {"time": pair.time_s.tolist(), "signals": pair.signal_ids,
                      "sigma": pair.config.noise_sigma_cycles, "rho": pair.config.noise_rho,
                      "gnssRate": pair.config.gnss_rate_hz, "imuRate": pair.imu_motion.config.rate_hz,
                      "metrics": pair_metrics(pair), "cases": {}}
        for case_name in CASES:
            case = pair.cases[case_name]
            data[name]["cases"][case_name] = {
                "relative": (case.phase_cycles-case.phase_cycles[0]).tolist(),
                "withoutTrend": (case.phase_cycles-case.slow_phase_cycles).tolist(),
                "geometric": case.geometric_phase_cycles.tolist(),
            }
        pictures = []
        for path in figures[name]:
            encoded = base64.b64encode((output/path).read_bytes()).decode("ascii")
            pictures.append(f'<img loading="lazy" src="data:image/png;base64,{encoded}" alt="{escape(path)}">')
        options.append(f'<option value="{name}" {"selected" if name == preferred else ""}>{escape(name)}</option>')
        links = [("manifest.json", "Input manifest"), ("benign/carrier.csv", "Benign carrier CSV"),
                 ("spoofed/carrier.csv", "Spoofed carrier CSV"), ("inputs/imu.csv", "Shared ideal IMU"),
                 ("inputs/satellite_geometry.csv", "Nominal geometry"), ("truth/labels.csv", "Truth labels"),
                 ("truth/generation.json", "Generation settings"), ("truth/metrics.json", "Truth diagnostics")]
        anchor = " · ".join(f'<a href="{name}/{target}">{label}</a>' for target, label in links)
        sections.append(f'<section class="scenario" data-scenario="{name}"><h2>{escape(name)}</h2>'
                        f'<p>{anchor}</p>{"".join(pictures)}</section>')
    template = Path(__file__).with_name("carrier_report_template.html").read_text()
    replacements = {"__DATA__": json.dumps(data, allow_nan=False), "__OPTIONS__": "".join(options),
                    "__SECTIONS__": "".join(sections),
                    "__PDF__": '<a href="carrier_observations.pdf">All figures as PDF</a> · ' if has_plots else ""}
    for key, value in replacements.items():
        template = template.replace(key, value)
    (output/"report.html").write_text(template)
