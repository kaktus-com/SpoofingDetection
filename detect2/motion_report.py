"""Interactive held-out comparison of IMU motion, phase response and oracle runs."""

import base64
from html import escape
import json
from pathlib import Path
import numpy as np


def write_motion_report(output, runs, comparison, figures):
    data = {}
    for run in runs:
        if not run.get('motion_comparison'):
            continue
        windows = []
        for record in run['records']:
            reference = run['motion_comparison'].get(record.index)
            if reference is not None:
                windows.append({'index': record.index, 'start': record.start_s, 'end': record.end_s,
                                'time': record.data.time_s.tolist(), 'signals': list(record.data.signal_ids),
                                'raw_white_noise_sigma_m': np.sqrt(np.maximum(0., np.diag(record.motion_estimate.displacement_covariance_m2))).tolist(),
                                'observable_white_noise_sigma_m': np.sqrt(np.maximum(0., np.diag(record.motion_estimate.observable_covariance_m2))).tolist(),
                                'motion_noise_check': record.result.diagnostics.get('imu_motion_noise'), **reference})
        data.setdefault(run['scenario'], {})[run['case']] = windows
    if not data:
        (output/'motion_report.html').write_text('<!doctype html><html lang="en"><meta charset="utf-8"><title>Motion comparison</title><p>No comparable motion windows were available. IMU predictions and failure reasons remain saved separately.</p><a href="report.html">Detector report</a></html>')
        return
    sections = []
    for name in data:
        images = []
        for path in figures[name]:
            if Path(path).name not in {'motion_estimation.png', 'imu_vs_oracle.png'}:
                continue
            encoded = base64.b64encode((output/path).read_bytes()).decode('ascii')
            images.append(f'<img loading="lazy" src="data:image/png;base64,{encoded}" alt="{escape(path)}">')
        sections.append(f'<section class="scenario" data-scenario="{name}"><h2>{escape(name)}</h2>{"".join(images)}</section>')
    summary = comparison['summary']
    text = (f"{summary['compared_windows']} compared windows; {summary['decision_agreements']} matching IMU/oracle decisions; "
            f"{summary['conclusive_in_both']} conclusive in both modes. Maximum observable displacement RMSE: "
            f"{summary['max_observable_displacement_vector_rmse_m']*1e6:.4g} micrometres.")
    template = Path(__file__).with_name('motion_report_template.html').read_text()
    input_note = ('Controlled errors were injected into the supplied IMU/attitude inputs; settings and seed are recorded in run.json. '
                  'Attitude is still supplied rather than estimated. Known antenna offset and common nominal timebase are assumed. '
                  'These are engineering sensitivity experiments, not measured hardware performance.'
                  if any(run.get('imu_errors') is not None for run in runs) else
                  'Ideal synthetic sensor inputs, exact supplied attitude, known antenna offset and common timestamps are used in this baseline. '
                  'Attitude estimation and measured sensor/timing performance remain future work.')
    for marker, value in {'__DATA__': json.dumps(data, allow_nan=False), '__SUMMARY__': escape(text),
                           '__SECTIONS__': ''.join(sections), '__INPUT_NOTE__': escape(input_note)}.items():
        template = template.replace(marker, value)
    (output/'motion_report.html').write_text(template)
