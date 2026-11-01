"""Paired before/after validation of the IMU uncertainty gate across seeds.

An ungated likelihood fit is reused only when the pre-fit uncertainty check
passes; rejected copies contain no fits or gamma. All predictions for both
modes are exported before labels are opened. No thresholds are tuned here.
"""

import argparse
from contextlib import redirect_stdout
from dataclasses import asdict, replace
import io
import json
from pathlib import Path

from .detector import DetectorConfig
from .detector_report import write_detector_report
from .evaluation import evaluate_predictions
from .imu_errors import ImuErrors
from .motion_uncertainty import assess_motion_noise, attach_motion_check
from .replay import export_records, replay_dataset, write_json, write_rows


def gated_copy(runs, output, config):
    """The same quality check as live replay, with immutable paired fit reuse."""
    copied = []
    for run in runs:
        run_config = replace(run['config'], imu_noise_gate_enabled=config.imu_noise_gate_enabled,
                             imu_noise_tail_probability=config.imu_noise_tail_probability)
        records = []
        for record in run['records']:
            if record.motion_estimate is None:
                result = replace(record.result, diagnostics=dict(record.result.diagnostics))
            else:
                check = assess_motion_noise(record.motion_estimate, run_config)
                result = attach_motion_check(record.result, check)
            records.append(replace(record, result=result))
        target = output/run['scenario']/run['case']
        export_records(target, records, 'imu')
        copied.append({**run, 'records': records, 'config': run_config, 'output': target})
    return copied


def aggregate_rows(windows):
    keys = list(dict.fromkeys((row['sigma_m_s2'], row['scenario']) for row in windows))
    rows = []
    for sigma, scenario in keys:
        selected = [row for row in windows if row['sigma_m_s2'] == sigma and row['scenario'] == scenario]
        for mode in ['ungated', 'gated']:
            known = [row for row in selected if row['truth'] in ('benign', 'spoofed')]
            conclusive = [row for row in known if row[mode+'_decision'] != 'inconclusive']
            rows.append({'sigma_m_s2': sigma, 'scenario': scenario, 'mode': mode,
                         'seed_count': len({row['seed'] for row in selected}), 'windows': len(selected),
                         'labeled_windows': len(known), 'conclusive': len(conclusive),
                         'correct_conclusive': sum(row[mode+'_decision'] == row['truth'] for row in conclusive),
                         'inconclusive': sum(row[mode+'_decision'] == 'inconclusive' for row in selected),
                         'false_alarms': sum(row['truth'] == 'benign' and row[mode+'_decision'] == 'spoofed' for row in known),
                         'missed_detections': sum(row['truth'] == 'spoofed' and row[mode+'_decision'] == 'benign' for row in known),
                         'coverage_percent': 100*len(conclusive)/len(known) if known else None,
                         'noise_gate_rejections': sum(row['noise_gate_rejected'] for row in selected) if mode == 'gated' else 0})
    return rows


def run_validation(data_dir, output_dir, seeds=tuple(range(42, 52)), sigmas=(.05, .2, .5),
                   scenarios=None, trim_s=1., plots=True, evaluate=True, tail_probability=.001):
    import numpy as np
    source, output = Path(data_dir).resolve(), Path(output_dir).resolve()
    if source == output or source in output.parents:
        raise ValueError('Write noise validation outside the measurement dataset')
    seeds, sigmas = list(seeds), list(sigmas)
    if (not seeds or len(set(seeds)) != len(seeds) or any(not isinstance(s, (int, np.integer)) or s < 0 for s in seeds)
            or not sigmas or len(set(sigmas)) != len(sigmas) or any(not np.isfinite(s) or s <= 0 for s in sigmas)):
        raise ValueError('Choose distinct nonnegative seeds and distinct positive finite sigmas')
    config = DetectorConfig(imu_noise_gate_enabled=True, imu_noise_tail_probability=tail_probability)
    ungated_config = replace(config, imu_noise_gate_enabled=False)
    batches = []
    conditions = [('ideal', seeds[0], 0.)]
    conditions += [(f'seed_{seed}_sigma_{sigma:g}'.replace('.', 'p'), seed, sigma) for sigma in sigmas for seed in seeds]
    for name, seed, sigma in conditions:
        print(f'Predicting paired {name}', flush=True)
        root = output/name
        log = io.StringIO()
        with redirect_stdout(log):
            before, _ = replay_dataset(source, root/'ungated', imu=True, evaluate=False, plots=False,
                                       scenarios=scenarios, config=ungated_config, trim_s=trim_s,
                                       imu_errors=ImuErrors(accel_noise_sigma_m_s2=sigma), imu_seed=seed)
        (root/'ungated/replay.log').write_text(log.getvalue())
        after = gated_copy(before, root/'gated', config)
        meta = json.loads((root/'ungated/run.json').read_text())
        meta['paired_quality_check'] = 'identical pre-fit check; passed fits reused unchanged; rejected fits/gamma absent'
        for item in meta['runs']:
            item['config'].update(imu_noise_gate_enabled=True, imu_noise_tail_probability=tail_probability)
        write_json(root/'gated/run.json', meta)
        # A usable prediction-only report also exists when truth is absent.
        names = list(dict.fromkeys(run['scenario'] for run in after))
        write_detector_report(root/'gated', after, {'status': 'not_requested'}, {n: [] for n in names}, False)
        batches.append((name, seed, sigma, before, after))
    write_json(output/'run.json', {
        'mode': 'paired_imu_noise_gate_validation', 'data_dir': str(source), 'seeds': seeds, 'sigmas_m_s2': sigmas,
        'conditions': [dict(name=n, seed=s, sigma_m_s2=x) for n, s, x in conditions],
        'edge_trim_s': trim_s, 'gated_config': asdict(config), 'ungated_config': asdict(ungated_config),
        'prediction_boundary': 'all paired predictions exported before any truth access; source measurements and old results are preserved',
        'pairing': 'same displacement and carrier observations in both modes; same IMU for both carrier cases; ideal sigma=0 evaluated once',
        'limitations': 'only IMU white-noise seed varies; carrier data and motions fixed; supplied attitude; alpha is a per-window model bound, not a detection error rate',
    })
    windows = []
    if evaluate:
        for name, seed, sigma, before, after in batches:
            print(f'Evaluating paired {name}', flush=True)
            for mode, runs in [('ungated', before), ('gated', after)]:
                target = output/name/mode
                evaluation = evaluate_predictions(runs)
                write_json(target/'evaluation.json', evaluation)
                write_rows(target/'evaluation.csv', evaluation['windows'])
                names = list(dict.fromkeys(run['scenario'] for run in runs))
                write_detector_report(target, runs, evaluation, {n: [] for n in names}, False)
                if mode == 'ungated':
                    labels = {(row['scenario'], row['case'], row['window_index']): row['truth'] for row in evaluation['windows']}
            for raw_run, checked_run in zip(before, after):
                for raw, checked in zip(raw_run['records'], checked_run['records']):
                    gate = checked.result.diagnostics.get('imu_motion_noise', {})
                    axis = gate.get('observed_axis_rms_m', [None])[-1]
                    windows.append({'condition': name, 'seed': seed, 'sigma_m_s2': sigma,
                                    'scenario': raw_run['scenario'], 'case': raw_run['case'], 'window_index': raw.index,
                                    'start_s': raw.start_s, 'end_s': raw.end_s,
                                    'truth': labels[raw_run['scenario'], raw_run['case'], raw.index],
                                    'ungated_decision': raw.result.decision, 'gated_decision': checked.result.decision,
                                    'ungated_reason': raw.result.reason, 'gated_reason': checked.result.reason,
                                    'ungated_gamma': raw.result.gamma, 'gated_gamma': checked.result.gamma,
                                    'noise_gate_rejected': checked.result.reason == 'imu_motion_below_noise_floor',
                                    'observed_min_axis_rms_m': axis,
                                    'noise_bound_rms_m': gate.get('noise_bound_rms_m'),
                                    'required_min_axis_rms_m': gate.get('min_axis_required_m')})
        rows = aggregate_rows(windows)
        write_json(output/'summary.json', {'status': 'evaluated_after_all_paired_predictions', 'rows': rows})
        write_rows(output/'summary.csv', rows)
        write_rows(output/'windows.csv', windows)
    else:
        rows = []
    from .imu_noise_validation_report import write_report
    write_report(output, rows, windows, plots)
    print(f'Report: {output/"report.html"}', flush=True)
    return rows, windows


def main(argv=None):
    base = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-dir', type=Path, default=base/'synthetic/data/carrier_phase')
    parser.add_argument('--output-dir', type=Path, default=base/'results/imu_noise_validation')
    parser.add_argument('--seeds', nargs='+', type=int, default=list(range(42, 52)))
    parser.add_argument('--sigmas', nargs='+', type=float, default=[.05, .2, .5])
    parser.add_argument('--scenario', nargs='+')
    parser.add_argument('--trim-s', type=float, default=1.)
    parser.add_argument('--tail-probability', type=float, default=.001)
    parser.add_argument('--no-plots', action='store_true')
    parser.add_argument('--no-evaluate', action='store_true')
    args = parser.parse_args(argv)
    try:
        run_validation(args.data_dir, args.output_dir, args.seeds, args.sigmas, args.scenario,
                       args.trim_s, not args.no_plots, not args.no_evaluate, args.tail_probability)
    except (ValueError, OSError, KeyError) as exc:
        parser.error(str(exc))


if __name__ == '__main__':
    main()
