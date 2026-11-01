"""Held-out motion/phase and oracle-decision comparisons after IMU prediction."""

import numpy as np

from .detector_plots import plt

COLORS = ['#2563a6', '#d56824', '#238563']
AXES = ['East', 'North', 'Up']


def displacement_plot(name, runs):
    run = next(run for run in runs if run.get('motion_comparison'))
    index, comparison = next(iter(run['motion_comparison'].items()))
    record = next(record for record in run['records'] if record.index == index)
    estimate = record.motion_estimate
    t = record.data.time_s
    fig, axes = plt.subplots(3, 2, figsize=(12, 10), layout='constrained')
    for row, keys in enumerate([('oracle_displacement_m', 'estimated_displacement_m'),
                                ('oracle_observable_displacement_m', 'estimated_observable_displacement_m')]):
        ax = axes[0, row]
        for i, label in enumerate(AXES):
            ax.plot(t, np.array(comparison[keys[0]])[:, i], color=COLORS[i], label=label+' truth')
            ax.plot(t, np.array(comparison[keys[1]])[:, i], '--', color=COLORS[i])
        ax.set(xlabel='Measurement time (s)', ylabel='Antenna displacement ENU (m)',
               title='Raw: unknown initial velocity gives linear drift' if row == 0 else 'After quadratic removal: motion used by detector')
        ax.legend(ncol=3)
    true_phase = np.array(comparison['oracle_observable_phase_cycles'])
    estimated_phase = np.array(comparison['estimated_observable_phase_cycles'])
    error = np.array(comparison['observable_phase_error_cycles'])
    for j, signal in enumerate(record.data.signal_ids):
        color = plt.get_cmap('tab10')(j)
        axes[1, 0].plot(t, true_phase[:, j], color=color, alpha=.6)
        axes[1, 0].plot(t, estimated_phase[:, j], '--', color=color, label=signal)
        axes[1, 1].plot(t, error[:, j]*1000, color=color)
    axes[1, 0].set(title=f'{run["case"]}: true and estimated observable motion phase', ylabel='Phase (cycles)', xlabel='Measurement time (s)')
    axes[1, 0].legend(ncol=4)
    axes[1, 1].set(title='Estimated − true phase after polynomial removal', ylabel='Phase error (millicycles)', xlabel='Measurement time (s)')
    for i, axis in enumerate(AXES):
        axes[2, 0].plot(estimate.integration_time_s, estimate.integration_acceleration_m_s2[:, i], color=COLORS[i], label=axis)
        axes[2, 1].plot(t, estimate.lever_displacement_m[:, i], color=COLORS[i], label=axis)
    axes[2, 0].set(title='Measured acceleration rotated to ENU, gravity removed', ylabel='Linear acceleration (m/s²)', xlabel='IMU time (s)')
    axes[2, 1].set(title='Antenna displacement due to rotation of the body offset', ylabel='Lever displacement ENU (m)', xlabel='Measurement time (s)')
    axes[2, 0].legend(ncol=3); axes[2, 1].legend(ncol=3)
    error_um = comparison['metrics']['observable_displacement_vector_rmse_m']*1e6
    fig.suptitle(f'{name}: window [{record.start_s:g}, {record.end_s:g}) s — observable motion RMSE {error_um:.3g} µm\n'
                 'Solid: held-out truth; dashed: zero-velocity-initialized IMU estimate. Truth was read after prediction.')
    return fig


def decisions_comparison_plot(name, runs):
    fig, axes = plt.subplots(3, len(runs), figsize=(6*len(runs), 9), squeeze=False,
                              sharex='col', layout='constrained')
    state = {'benign': 1, 'inconclusive': 0, 'spoofed': -1}
    for col, run in enumerate(runs):
        records = run['records']
        t = [.5*(record.start_s+record.end_s) for record in records]
        comparisons = run.get('motion_comparison', {})
        metrics = [comparisons.get(record.index, {}).get('metrics', {}) for record in records]
        number = lambda row, key: row.get(key) if row.get(key) is not None else np.nan
        imu_gamma = [number(row, 'imu_gamma') for row in metrics]
        oracle_gamma = [number(row, 'oracle_gamma') for row in metrics]
        axes[0, col].plot(t, oracle_gamma, '--', color='#475569', label='Oracle')
        axes[0, col].plot(t, imu_gamma, 'o', color='#8b3bb0', label='IMU')
        axes[0, col].axhline(0, color='#94a3b8', linewidth=.8)
        axes[0, col].set(title=f'{run["case"]}: gamma from the unchanged detector', ylabel='Gamma')
        axes[0, col].legend()
        axes[1, col].plot(t, [number(row, 'gamma_difference') for row in metrics], 'o-', color='#8b3bb0')
        axes[1, col].set(title='IMU gamma − oracle gamma', ylabel='Gamma difference')
        axes[2, col].plot(t, [state.get(row.get('oracle_decision'), np.nan) for row in metrics], '--', color='#475569', label='Oracle')
        axes[2, col].plot(t, [state[record.result.decision] for record in records], 'o', color='#8b3bb0', label='IMU')
        axes[2, col].set(yticks=[-1, 0, 1], yticklabels=['Spoofed', 'Inconclusive', 'Benign'],
                        ylim=(-1.3, 1.3), xlabel='Window midpoint (s)')
        axes[2, col].legend()
        if all(record.result.gamma is None for record in records):
            for row in [0, 1]:
                axes[row, col].text(.5, .5, 'Inconclusive: no reliable full-3D fit', ha='center', va='center', transform=axes[row, col].transAxes)
    fig.suptitle(f'{name}: IMU predictions compared with a post-prediction oracle replay\n'
                 'The oracle result never initializes or alters the IMU estimate or prediction')
    return fig
