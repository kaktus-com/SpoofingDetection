"""Paired multi-seed decision coverage and motion/noise-floor figures."""

import base64
from html import escape
import json
from pathlib import Path

import numpy as np


def outcome_plot(scenario, rows):
    from .detector_plots import plt
    selected = [row for row in rows if row['scenario'] == scenario]
    fig, axes = plt.subplots(1, 2, figsize=(12, 5), sharey=True, layout='constrained')
    fields = [('correct_conclusive', 'Correct conclusive', '#238563'), ('inconclusive', 'Inconclusive', '#94a3b8'),
              ('false_alarms', 'False alarm', '#c13c3c'), ('missed_detections', 'Missed detection', '#d56824')]
    for ax, mode in zip(axes, ['ungated', 'gated']):
        subset = sorted([row for row in selected if row['mode'] == mode], key=lambda row: row['sigma_m_s2'])
        x = np.arange(len(subset))
        bottom = np.zeros(len(subset))
        for key, label, color in fields:
            heights = np.array([100*row[key]/row['labeled_windows'] if row['labeled_windows'] else np.nan for row in subset])
            ax.bar(x, heights, bottom=bottom, label=label, color=color)
            bottom += heights
        ax.set(title=mode.title(), xticks=x,
               xticklabels=[f"{row['sigma_m_s2']:g}\nn={row['windows']}" for row in subset],
               xlabel='Accelerometer sigma (m/s² per axis per sample)', ylim=(0, 105))
        ax.set_ylabel('Share of labeled windows (%)')
    axes[1].legend(loc='upper center', ncol=2, fontsize=8)
    fig.suptitle(f'{scenario}: paired decisions across IMU noise seeds\nOne ideal reference; the same carrier data and likelihood fits in both modes')
    return fig


def motion_floor_plot(windows):
    from .detector_plots import plt
    names = list(dict.fromkeys(row['scenario'] for row in windows))
    fig, axes = plt.subplots(int(np.ceil(len(names)/2)), 2, figsize=(12, 3.1*int(np.ceil(len(names)/2))), squeeze=False, layout='constrained')
    for ax, name in zip(axes.flat, names):
        selected = [row for row in windows if row['scenario'] == name]
        # Both carrier cases share their IMU. Plot each physical estimate once.
        first_case = selected[0]['case']
        selected = [row for row in selected if row['case'] == first_case and row['noise_bound_rms_m'] is not None]
        for rejected, color, label in [(False, '#238563', 'Noise check passed/bypassed'), (True, '#c13c3c', 'Noise check rejected')]:
            points = [row for row in selected if row['noise_gate_rejected'] == rejected]
            ax.scatter([row['sigma_m_s2'] for row in points], [row['observed_min_axis_rms_m']*1000 for row in points],
                       s=18, alpha=.4, color=color, label=label)
        sigmas = sorted({row['sigma_m_s2'] for row in selected})
        floor = [max(row['noise_bound_rms_m']+row['required_min_axis_rms_m'] for row in selected if row['sigma_m_s2'] == sigma)*1000 for sigma in sigmas]
        ax.plot(sigmas, floor, '--', color='#475569', label='Noise bound + weakest-axis threshold')
        ax.set(title=name, xlabel='Accelerometer sigma (m/s² per sample)', ylabel='Observed weakest-axis RMS (mm)')
        ax.set_ylim(bottom=0)
    for ax in list(axes.flat)[len(names):]:
        ax.set_visible(False)
    axes.flat[0].legend(fontsize=7)
    fig.suptitle('Observed excitation versus propagated white-noise uncertainty\nNoise gate also checks total RMS and condition; green does not imply a conclusive detector decision')
    return fig


def write_report(output, rows, windows, plots=True):
    output = Path(output)
    meta = json.loads((output/'run.json').read_text())
    names = list(dict.fromkeys(row['scenario'] for row in rows))
    figures = []
    if plots and rows:
        from matplotlib.backends.backend_pdf import PdfPages
        from .detector_plots import plt, setup_style
        setup_style()
        (output/'plots').mkdir(exist_ok=True)
        with PdfPages(output/'imu_noise_validation.pdf') as pdf:
            fig = motion_floor_plot(windows)
            path = Path('plots/motion_noise_floor.png')
            fig.savefig(output/path); pdf.savefig(fig); plt.close(fig); figures.append(path)
            for name in names:
                fig = outcome_plot(name, rows)
                path = Path('plots')/(name+'_outcomes.png')
                fig.savefig(output/path); pdf.savefig(fig); plt.close(fig); figures.append(path)
    table = []
    for row in rows:
        cells = [row['scenario'], f"{row['sigma_m_s2']:g}", row['mode'], row['seed_count'], row['windows'],
                 row['correct_conclusive'], row['inconclusive'], row['false_alarms'], row['missed_detections']]
        table.append('<tr>'+''.join('<td>'+escape(str(value))+'</td>' for value in cells)+'</tr>')
    summary = []
    for mode in ['ungated', 'gated']:
        selected = [row for row in rows if row['mode'] == mode]
        if selected:
            summary.append(f"{mode.title()}: {sum(r['windows'] for r in selected)} windows, "
                           f"{sum(r['false_alarms'] for r in selected)} false alarms, {sum(r['missed_detections'] for r in selected)} missed detections, "
                           f"{sum(r['inconclusive'] for r in selected)} inconclusive.")
    links = ['<a href="run.json">Settings and assumptions</a>']
    if rows:
        links += ['<a href="summary.json">Summary JSON</a>', '<a href="summary.csv">Summary CSV</a>', '<a href="windows.csv">Paired windows and noise checks</a>']
    if figures:
        links.append('<a href="imu_noise_validation.pdf">All figures as PDF</a>')
    conditions = []
    for condition in meta['conditions']:
        name = condition['name']
        conditions.append(f'<li>{escape(name)}: <a href="{name}/ungated/report.html">ungated fits</a> · <a href="{name}/gated/report.html">gated fits</a></li>')
    images = []
    for path in figures:
        encoded = base64.b64encode((output/path).read_bytes()).decode('ascii')
        images.append(f'<img loading="lazy" src="data:image/png;base64,{encoded}" alt="{escape(path.stem)}">')
    document = '''<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>detect2 · IMU noise gate validation</title><style>:root{font:16px/1.55 system-ui,sans-serif;color:#243247;background:#f3f5f8}body{max-width:1300px;margin:auto;padding:30px 22px}h1{font-size:2rem;letter-spacing:-.04em}a{color:#255fa0}.card{background:white;border:1px solid #dce2ea;border-radius:14px;padding:20px;margin:22px 0}.note{border-left:3px solid #d56824;padding:2px 15px;color:#52627a}.scroll{overflow:auto}table{border-collapse:collapse;width:100%;font-size:.85rem}th,td{text-align:right;padding:8px;border-bottom:1px solid #e5e9ef}th:first-child,td:first-child{text-align:left}img{width:100%;height:auto;margin:12px 0}p{max-width:105ch}</style></head><body>
<h1>IMU motion-noise gate: paired validation</h1>
<p>Configured accelerometer white noise is propagated through the exact integration and quadratic-removal operators. The additional quality check requires reliable full-3D excitation above a conservative noise bound. Likelihood fits and the zero gamma threshold are unchanged.</p>
<p class="note">The same carrier observations and IMU realization are used for each before/after pair. Both carrier cases share their IMU. Only the IMU noise seed varies; carrier noise and motion are fixed. Attitude remains supplied. This is conditional synthetic validation, not a field detection probability.</p>
<p>__SUMMARY__</p><p>__LINKS__</p>
<div class="card scroll"><h2>Coverage and errors</h2><p>Zero-noise reference evaluated once; other levels use the listed seeds. Inconclusive is a separate outcome. Rejecting uncertain movement can reduce coverage, including rotation-only motion at higher noise.</p>
<table><thead><tr><th>Motion</th><th>Sigma</th><th>Mode</th><th>Seeds</th><th>Windows</th><th>Correct conclusive</th><th>Inconclusive</th><th>False alarms</th><th>Missed</th></tr></thead><tbody>__TABLE__</tbody></table></div>
<div class="card"><h2>Motion against its noise floor and paired outcomes</h2>__IMAGES__</div>
<details class="card"><summary>Per-condition interactive fits and saved predictions</summary><ul>__CONDITIONS__</ul></details>
<p class="note">The bound covers independent isotropic Gaussian accelerometer noise conditional on supplied attitude and timestamps. Bias, attitude uncertainty, clock error, numerical integration error and coloured sensor noise are excluded. The tail bound is per window and is not a spoofing false-alarm rate.</p></body></html>'''
    for marker, value in {'__SUMMARY__': escape(' '.join(summary) if rows else 'Prediction only; truth was not accessed.'),
                           '__LINKS__': ' · '.join(links), '__TABLE__': ''.join(table), '__IMAGES__': ''.join(images),
                           '__CONDITIONS__': ''.join(conditions)}.items():
        document = document.replace(marker, value)
    (output/'report.html').write_text(document)
