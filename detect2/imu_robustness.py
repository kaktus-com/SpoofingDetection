"""Controlled IMU error sweeps, preserving the ideal integration baseline.

All conditions use the same carrier measurements, windows and detector settings.
Every condition's predictions are saved before any truth evaluation starts.
"""

import argparse
from collections import Counter
from contextlib import redirect_stdout
from dataclasses import asdict, dataclass
import io
import json
from pathlib import Path
import re

import numpy as np

from .detector_report import write_detector_report
from .detector import DetectorConfig
from .evaluation import evaluate_predictions
from .imu_errors import ImuErrors
from .motion_evaluation import compare_imu_with_oracle
from .replay import replay_dataset, write_json, write_rows


@dataclass(frozen=True)
class Condition:
    name: str
    family: str
    value: float
    unit: str
    errors: ImuErrors

    def __post_init__(self):
        if not re.fullmatch(r'[a-z0-9_]+', self.name) or not np.isfinite(self.value):
            raise ValueError('Condition requires a safe identifier and finite sweep value')


def default_conditions():
    result = [Condition('ideal', 'ideal', 0., '', ImuErrors())]
    for value, label in [(.01, '0p01'), (.05, '0p05'), (.2, '0p2'), (.5, '0p5')]:
        result.append(Condition('accel_noise_'+label, 'accel_noise', value, 'm/s² per sample',
                                ImuErrors(accel_noise_sigma_m_s2=value)))
        result.append(Condition('body_bias_'+label, 'body_bias', value, 'm/s² forward',
                                ImuErrors(accel_bias_body_m_s2=(value, 0., 0.))))
    for value, label in [(.1, '0p1'), (.5, '0p5'), (1., '1'), (3., '3')]:
        result.append(Condition('attitude_'+label+'deg', 'attitude', value, 'degrees about body Right',
                                ImuErrors(attitude_error_body_rotvec_deg=(0., value, 0.))))
    for value in [-50, -20, -5, 5, 20, 50]:
        result.append(Condition('timing_'+('minus_' if value < 0 else 'plus_')+str(abs(value))+'ms',
                                'timing', value, 'ms', ImuErrors(time_offset_s=value/1000)))
    result.append(Condition('combined_errors', 'combined', 1., '', ImuErrors(
        accel_noise_sigma_m_s2=.05, accel_bias_body_m_s2=(.05, 0., 0.),
        attitude_error_body_rotvec_deg=(0., .5, 0.), time_offset_s=.02)))
    return result


def summarize(condition, scenario, runs, evaluation, comparison, baseline):
    labels = [row for row in evaluation['windows'] if scenario == 'all' or row['scenario'] == scenario]
    labeled = [row for row in labels if row['truth'] in ('benign', 'spoofed')]
    conclusive = [row for row in labeled if row['decision'] != 'inconclusive']
    motion = [row for row in comparison['windows'] if scenario == 'all' or row['scenario'] == scenario]
    selected = [run for run in runs if scenario == 'all' or run['scenario'] == scenario]
    pairs = [(record.result, baseline[(run['scenario'], run['case'], record.index)])
             for run in selected for record in run['records']]
    maximum = lambda key, scale=1.: max((row[key]*scale for row in motion if row[key] is not None), default=None)
    relative = [100*abs(row['gamma_difference']/row['oracle_gamma']) for row in motion
                if row['gamma_difference'] is not None and row['oracle_gamma'] is not None and abs(row['oracle_gamma']) > 1e-8]
    reasons = Counter(record.result.reason for run in selected for record in run['records']
                      if record.result.decision == 'inconclusive')
    checks = [record.result.diagnostics.get('imu_motion_noise', {}) for run in selected for record in run['records']]
    checks = [check for check in checks if 'observed_axis_rms_m' in check]
    noise_metrics = {
        'noise_gate_rejections': sum(record.result.reason == 'imu_motion_below_noise_floor' for run in selected for record in run['records']),
        'noise_bound_rms_max_mm': max((check['noise_bound_rms_m']*1000 for check in checks), default=None),
        'min_axis_rms_required_mm': max((check['min_axis_required_m']*1000 for check in checks), default=None),
        'total_rms_required_mm': max((check['total_rms_required_m']*1000 for check in checks), default=None),
    }
    for prefix, values in [('observed_min_axis', [check['observed_axis_rms_m'][-1]*1000 for check in checks]),
                            ('observed_total', [check['observed_total_rms_m']*1000 for check in checks]),
                            ('lower_bound_min_axis', [check['axis_rms_lower_bound_m'][-1]*1000 for check in checks])]:
        noise_metrics[prefix+'_min_mm'] = min(values, default=None)
        noise_metrics[prefix+'_max_mm'] = max(values, default=None)
    return {'condition': condition.name, 'family': condition.family, 'value': condition.value,
            'unit': condition.unit, 'scenario': scenario, 'total_windows': len(labels),
            'labeled_windows': len(labeled), 'conclusive_windows': len(conclusive),
            'correct_conclusive': sum(row['correct'] for row in conclusive),
            'inconclusive_windows': sum(row['decision'] == 'inconclusive' for row in labels),
            'false_alarms': sum(row['truth'] == 'benign' and row['decision'] == 'spoofed' for row in conclusive),
            'missed_detections': sum(row['truth'] == 'spoofed' and row['decision'] == 'benign' for row in conclusive),
            'conclusive_coverage_percent': 100*len(conclusive)/len(labeled) if labeled else None,
            'accuracy_on_conclusive_percent': 100*sum(row['correct'] for row in conclusive)/len(conclusive) if conclusive else None,
            'decision_changes_from_ideal': sum(now.decision != before.decision for now, before in pairs),
            'lost_conclusive_from_ideal': sum(now.decision == 'inconclusive' and before.decision != 'inconclusive' for now, before in pairs),
            'gained_conclusive_from_ideal': sum(now.decision != 'inconclusive' and before.decision == 'inconclusive' for now, before in pairs),
            'max_observable_displacement_rmse_um': maximum('observable_displacement_vector_rmse_m', 1e6),
            'max_observable_phase_rmse_millicycles': maximum('observable_phase_rmse_cycles', 1000),
            'max_absolute_gamma_difference': max((abs(row['gamma_difference']) for row in motion if row['gamma_difference'] is not None), default=None),
            'max_relative_gamma_difference_percent': max(relative, default=None),
            'inconclusive_reasons': json.dumps(dict(sorted(reasons.items())), sort_keys=True), **noise_metrics}


def run_sweep(data_dir, output_dir, conditions=None, scenarios=None, seed=42, trim_s=1., plots=True, evaluate=True,
              noise_gate_enabled=True, noise_tail_probability=.001):
    source, output = Path(data_dir).resolve(), Path(output_dir).resolve()
    if source == output or source in output.parents:
        raise ValueError('Write robustness results outside the measurement dataset')
    conditions = list(default_conditions() if conditions is None else conditions)
    if not conditions or len({c.name for c in conditions}) != len(conditions):
        raise ValueError('Choose at least one condition with unique names')
    if conditions[0].errors != ImuErrors() or conditions[0].name != 'ideal':
        raise ValueError('The first condition must be the unchanged ideal reference')
    output.mkdir(parents=True, exist_ok=True)
    batches = []
    for condition in conditions:
        print(f'Predicting {condition.name}', flush=True)
        target = output/condition.name
        log = io.StringIO()
        with redirect_stdout(log):
            runs, _ = replay_dataset(source, target, scenarios=scenarios, imu=True,
                                     evaluate=False, plots=False, imu_errors=condition.errors,
                                     imu_seed=seed, trim_s=trim_s,
                                     config=DetectorConfig(imu_noise_gate_enabled=noise_gate_enabled,
                                                           imu_noise_tail_probability=noise_tail_probability))
        (target/'replay.log').write_text(log.getvalue())
        write_json(target/'imu_errors.json', asdict(condition.errors))
        batches.append((condition, target, runs))
    # Nothing under truth/ is opened until ALL conditions have saved predictions.
    write_json(output/'run.json', {
        'mode': 'own_short_window_imu_integration_sensitivity', 'data_dir': str(source),
        'seed': seed, 'edge_trim_s': trim_s, 'scenarios': list(dict.fromkeys(run['scenario'] for run in batches[0][2])),
        'conditions': [asdict(condition) for condition in conditions],
        'imu_noise_gate_enabled': noise_gate_enabled, 'imu_noise_tail_probability': noise_tail_probability,
        'prediction_boundary': 'all conditions exported before any truth evaluation; no truth velocity or displacement initializes integration',
        'pairing': 'unchanged carrier observations and detector thresholds; same IMU for paired cases; common unit noise across sigma levels',
        'limitations': 'one engineering seed, supplied attitude, selected synthetic motions; no hardware accuracy, detection probability or gyro-attitude-estimator claim',
    })
    rows = []
    if evaluate:
        baseline = {(run['scenario'], run['case'], record.index): record.result
                    for run in batches[0][2] for record in run['records']}
        for condition, target, runs in batches:
            print(f'Evaluating {condition.name}', flush=True)
            evaluation = evaluate_predictions(runs)
            comparison = compare_imu_with_oracle(runs)
            write_json(target/'evaluation.json', evaluation)
            write_rows(target/'evaluation.csv', evaluation['windows'])
            write_json(target/'motion_comparison.json', comparison)
            write_rows(target/'motion_comparison.csv', comparison['windows'])
            names = list(dict.fromkeys(run['scenario'] for run in runs))
            write_detector_report(target, runs, evaluation, {name: [] for name in names}, False, comparison)
            for scenario in ['all', *names]:
                rows.append(summarize(condition, scenario, runs, evaluation, comparison, baseline))
        write_json(output/'summary.json', {'status': 'evaluated_after_all_predictions', 'rows': rows})
        write_rows(output/'summary.csv', rows)
    from .imu_robustness_report import write_report
    write_report(output, conditions, rows, plots=plots)
    print(f'Report: {output/"report.html"}', flush=True)
    return rows


def main(argv=None):
    base = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-dir', type=Path, default=base/'synthetic/data/carrier_phase')
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--scenario', nargs='+', help='default: all eight controls')
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--trim-s', type=float, default=1., help='exclude recording edges for matched interior windows')
    parser.add_argument('--no-plots', action='store_true')
    parser.add_argument('--no-evaluate', action='store_true', help='save predictions without reading any truth')
    parser.add_argument('--no-imu-noise-gate', action='store_true')
    parser.add_argument('--imu-noise-tail-probability', type=float, default=.001)
    args = parser.parse_args(argv)
    try:
        output = args.output_dir or base/'results'/('imu_robustness_ungated' if args.no_imu_noise_gate else 'imu_robustness_gated')
        run_sweep(args.data_dir, output, scenarios=args.scenario, seed=args.seed,
                  trim_s=args.trim_s, plots=not args.no_plots, evaluate=not args.no_evaluate,
                  noise_gate_enabled=not args.no_imu_noise_gate, noise_tail_probability=args.imu_noise_tail_probability)
    except (ValueError, OSError, KeyError) as exc:
        parser.error(str(exc))


if __name__ == '__main__':
    main()
