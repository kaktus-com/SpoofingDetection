"""Scientific sensitivity plots and an offline report for IMU input errors."""

import base64
from html import escape
import json
from pathlib import Path

import numpy as np

FAMILIES = {'accel_noise': 'Accelerometer white noise', 'body_bias': 'Constant body-Forward bias',
            'attitude': 'Supplied-attitude error about body Right', 'timing': 'IMU timestamp offset'}


def noise_floor_plot(scenario, rows):
    from .detector_plots import plt
    selected = sorted([row for row in rows if row['scenario'] == scenario and row['family'] in {'ideal', 'accel_noise'}], key=lambda row: row['value'])
    x = [row['value'] for row in selected]
    bound = np.array([row['noise_bound_rms_max_mm'] for row in selected])
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5), layout='constrained')
    for ax, prefix, required, title in [
        (axes[0], 'observed_min_axis', 'min_axis_rms_required_mm', 'Weakest motion axis after quadratic removal'),
        (axes[1], 'observed_total', 'total_rms_required_mm', 'Total motion after quadratic removal')]:
        low = np.array([row[prefix+'_min_mm'] for row in selected])
        high = np.array([row[prefix+'_max_mm'] for row in selected])
        threshold = np.array([row[required] for row in selected])
        ax.fill_between(x, low, high, alpha=.2, color='#8b3bb0', label='Observed range across windows')
        ax.plot(x, low, 'o-', color='#8b3bb0', label='Smallest observed RMS')
        ax.plot(x, bound, '--', color='#d56824', label='Noise norm bound / sqrt(N)')
        ax.plot(x, bound+threshold, ':', color='#c13c3c', label='Bound + original RMS threshold')
        ax.set(xlabel='Accelerometer sigma (m/s² per axis per sample)', ylabel='RMS displacement (mm)', title=title)
        ax.set_ylim(0, max(1e-3, np.max(np.r_[high, bound+threshold])*1.15))
        ax.legend(fontsize=7)
    fig.suptitle(f'{scenario}: observed movement versus configured white-noise uncertainty\nConservative per-window bound; both RMS requirements and the condition check must pass')
    return fig


def sensitivity_plot(scenario, family, rows):
    from .detector_plots import plt
    selected = sorted([row for row in rows if row['scenario'] == scenario and row['family'] == family], key=lambda row: row['value'])
    ideal = next(row for row in rows if row['scenario'] == scenario and row['family'] == 'ideal')
    x = [row['value'] for row in selected]
    number = lambda row, key: row[key] if row[key] is not None else np.nan
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), layout='constrained')
    for ax, key, title, unit in [
        (axes[0, 0], 'max_observable_displacement_rmse_um', 'Worst-window displacement RMSE after quadratic removal', 'µm'),
        (axes[0, 1], 'max_observable_phase_rmse_millicycles', 'Worst-window motion-phase RMSE after quadratic removal', 'millicycles'),
        (axes[1, 1], 'max_relative_gamma_difference_percent', 'Largest relative gamma change from oracle', '%')]:
        ax.plot(x, [number(row, key) for row in selected], 'o-', color='#8b3bb0')
        if ideal[key] is not None:
            ax.axhline(ideal[key], ls='--', color='#475569', label='Ideal IMU reference')
            ax.legend()
        if all(row[key] is None for row in selected):
            ax.text(.5, .5, 'No comparable conclusive fits', ha='center', va='center', transform=ax.transAxes)
        ax.set(title=title, ylabel=unit)
        # RMSE and absolute relative gamma differences are nonnegative. Keep
        # zero visible so roundoff-sized variations do not look like a trend.
        finite = [value for value in [number(row, key) for row in selected]+[number(ideal, key)] if np.isfinite(value)]
        ax.set_ylim(0., max(1e-9, 1.12*max(finite, default=0.)))
    ax = axes[1, 0]
    for key, label, color in [('conclusive_windows', 'Conclusive', '#238563'),
                               ('inconclusive_windows', 'Inconclusive', '#8b3bb0'),
                               ('false_alarms', 'False alarms', '#c13c3c'),
                               ('missed_detections', 'Missed detections', '#d56824')]:
        ax.plot(x, [row[key] for row in selected], 'o-', label=label, color=color)
    ax.set(title=f'Frozen decisions ({ideal["total_windows"]} paired windows per level)', ylabel='Window count', ylim=(-.3, ideal['total_windows']+.5))
    ax.legend(ncol=2)
    for ax in axes.flat:
        ax.set_xlabel(selected[0]['unit'])
    fig.suptitle(f'{scenario}: {FAMILIES[family]}\nSame carrier observations and thresholds; supplied attitude; one reproducible IMU noise realization')
    return fig


def overview_plot(conditions, rows, scenarios):
    from .detector_plots import plt
    fig, axes = plt.subplots(1, 3, figsize=(15, max(7, .35*len(conditions)+2)), layout='constrained')
    lookup = {(row['condition'], row['scenario']): row for row in rows}
    for ax, key, title, cmap, maximum in [
        (axes[0], 'conclusive_coverage_percent', 'Conclusive coverage (%)', 'YlGn', 100),
        (axes[1], 'false_alarms', 'False alarms (window count)', 'OrRd', None),
        (axes[2], 'missed_detections', 'Missed detections (window count)', 'OrRd', None)]:
        values = np.array([[lookup[c.name, s][key] if lookup[c.name, s][key] is not None else np.nan
                            for s in scenarios] for c in conditions])
        im = ax.imshow(values, aspect='auto', cmap=cmap, vmin=0,
                       vmax=maximum or max(1, np.nanmax(values)), interpolation='nearest')
        ax.set(xticks=range(len(scenarios)), xticklabels=scenarios,
               yticks=range(len(conditions)), yticklabels=[c.name for c in conditions], title=title)
        ax.tick_params(axis='x', rotation=60)
        ax.grid(False)
        for i in range(len(conditions)):
            for j in range(len(scenarios)):
                value = values[i, j]
                ax.text(j, i, '—' if not np.isfinite(value) else f'{value:g}', ha='center', va='center', fontsize=7,
                        color='white' if value > im.norm.vmax*.6 else '#243247')
        fig.colorbar(im, ax=ax, fraction=.035, pad=.02)
    fig.suptitle('Own IMU integration: controlled-error sensitivity\nInconclusive windows remain separate from wrong decisions; engineering levels, one seed')
    return fig


def write_report(output, conditions, rows, plots=True):
    output = Path(output)
    scenarios = list(dict.fromkeys(row['scenario'] for row in rows if row['scenario'] != 'all'))
    figures = []
    if plots and rows:
        from matplotlib.backends.backend_pdf import PdfPages
        from .detector_plots import plt, setup_style
        setup_style()
        (output/'plots').mkdir(exist_ok=True)
        with PdfPages(output/'imu_robustness.pdf') as pdf:
            jobs = [('overview', 'all', 'all', overview_plot(conditions, rows, scenarios))]
            for name, scenario, family, fig in jobs:
                path = Path('plots')/(name+'.png')
                fig.savefig(output/path); pdf.savefig(fig); plt.close(fig)
                figures.append((path, scenario, family))
            for scenario in scenarios:
                for family in FAMILIES:
                    if not any(row['scenario'] == scenario and row['family'] == family for row in rows):
                        continue
                    fig = sensitivity_plot(scenario, family, rows)
                    path = Path('plots')/(scenario+'_'+family+'.png')
                    fig.savefig(output/path); pdf.savefig(fig); plt.close(fig)
                    figures.append((path, scenario, family))
                if any(row['scenario'] == scenario and row.get('noise_bound_rms_max_mm') is not None for row in rows):
                    fig = noise_floor_plot(scenario, rows)
                    path = Path('plots')/(scenario+'_noise_floor.png')
                    fig.savefig(output/path); pdf.savefig(fig); plt.close(fig)
                    figures.append((path, scenario, 'noise_floor'))
    totals = [row for row in rows if row['scenario'] == 'all']
    failures = [row for row in totals if row['false_alarms'] or row['missed_detections']]
    findings = ('Wrong classifications occur in this sweep: '
                +'; '.join(f"{row['condition']}: {row['false_alarms']} false alarms, {row['missed_detections']} missed detections" for row in failures)+'. '
                'The current fit treats estimated displacement as known and does not propagate IMU uncertainty. '
                'Added sensor noise can also make a weak-motion control pass the rank gate. Integration and quality handling need further work before real-flight use.'
                if failures else 'No wrong conclusive classifications were observed in these evaluated conditions. '
                'Inconclusive coverage remains separate, and this single-seed sweep does not establish detection probabilities.' if totals else
                'Only predictions have been saved; correctness and motion error have not been evaluated.')
    table_rows = []
    for condition in conditions:
        row = next((row for row in totals if row['condition'] == condition.name), None)
        columns = ([row['conclusive_windows'], row['correct_conclusive'], row['inconclusive_windows'],
                    row['false_alarms'], row['missed_detections'], row['decision_changes_from_ideal'],
                    f"{row['max_observable_displacement_rmse_um']:.3g}" if row['max_observable_displacement_rmse_um'] is not None else '—']
                   if row else ['—']*7)
        table_rows.append('<tr><td><a href="'+condition.name+'/report.html">'+escape(condition.name)+'</a></td>'
                          +''.join('<td>'+escape(str(value))+'</td>' for value in columns)+'</tr>')
    images = []
    for path, scenario, family in figures:
        encoded = base64.b64encode((output/path).read_bytes()).decode('ascii')
        images.append(f'<section class="figure" data-scenario="{escape(scenario)}" data-family="{escape(family)}">'
                      f'<img loading="lazy" src="data:image/png;base64,{encoded}" alt="{escape(path.stem)}"></section>')
    links = ['<a href="run.json">Settings, seed and prediction boundary</a>']
    if rows:
        links += ['<a href="summary.json">Summary JSON</a>', '<a href="summary.csv">Summary CSV</a>']
    if figures:
        links.append('<a href="imu_robustness.pdf">All figures as PDF</a>')
    selectors = ('<label>Motion <select id="scenario"><option value="all">All motions</option>'
                 +''.join(f'<option value="{escape(name)}">{escape(name)}</option>' for name in scenarios)
                 +'</select></label><label>Error <select id="family"><option value="all">All error types</option>'
                 +''.join(f'<option value="{name}">{escape(label)}</option>' for name, label in {**FAMILIES, 'noise_floor': 'Motion versus noise floor'}.items())+'</select></label>')
    settings = json.loads((output/'run.json').read_text())
    gate_note = ('The additional IMU uncertainty gate is enabled. Its Gaussian noise-norm exceedance bound is '
                 f"{settings['imu_noise_tail_probability']:g} per window, conditional on the configured white-noise model. "
                 'This is not a spoofing false-alarm probability. Bias, attitude and timing errors remain outside that covariance.'
                 if settings.get('imu_noise_gate_enabled') else 'The additional IMU uncertainty gate is disabled for this comparison.')
    template = Path(__file__).with_name('imu_robustness_report_template.html').read_text()
    for key, value in {'__LINKS__': ' · '.join(links), '__ROWS__': ''.join(table_rows),
                       '__FIGURES__': ''.join(images), '__SELECTORS__': selectors,
                       '__FINDINGS__': escape(findings),
                       '__GATE_NOTE__': escape(gate_note),
                       '__STATUS__': ('Evaluated after every condition saved its predictions.' if rows else 'Prediction only: no truth evaluation was requested.')}.items():
        template = template.replace(key, value)
    (output/'report.html').write_text(template)
