import csv
import json
from pathlib import Path
import re
import shutil

import numpy as np
from numpy.testing import assert_allclose

from SpoofingDetection.detect2 import motion_evaluation
from SpoofingDetection.detect2.replay import replay_dataset
from SpoofingDetection.detect2.synthetic.carrier import CarrierConfig
from SpoofingDetection.detect2.synthetic.generate_carrier import generate_datasets
from SpoofingDetection.detect2.synthetic.motions import MotionConfig


def dataset(tmp_path, scenario='three_axis', sigma=.01):
    root = tmp_path/'observations'
    generate_datasets(root, [MotionConfig(scenario, duration_s=1)], CarrierConfig(noise_sigma_cycles=sigma), plots=False)
    return root


def test_imu_predictions_work_without_any_truth_directory(tmp_path, monkeypatch):
    source = dataset(tmp_path)
    shutil.rmtree(source/'three_axis/truth')
    original = Path.open
    def guard(path, *args, **kwargs):
        assert 'truth' not in path.parts, f'IMU prediction tried to read {path}'
        return original(path, *args, **kwargs)
    monkeypatch.setattr(Path, 'open', guard)
    output = tmp_path/'results'
    runs, evaluation = replay_dataset(source, output, imu=True, evaluate=False, plots=False)
    assert evaluation['status'] == 'not_requested'
    assert [run['records'][0].result.decision for run in runs] == ['benign', 'spoofed']
    meta = json.loads((output/'run.json').read_text())
    assert meta['mode'] == 'imu_displacement'
    for run in meta['runs']:
        assert not any('truth' in Path(path).parts for path in run['prediction_inputs'])
    for run in runs:
        record = run['records'][0]
        assert record.result.diagnostics['mode'] == 'imu_displacement'
        assert record.motion_estimate.diagnostics['initial_velocity_m_s'] == [0., 0., 0.]
        with (run['output']/'windows.csv').open() as stream:
            assert list(csv.DictReader(stream))[0]['displacement_mode'] == 'imu'
        with (run['output']/'estimated_displacement.csv').open() as stream:
            rows = list(csv.DictReader(stream))
        assert len(rows) == 20


def test_truth_comparison_happens_after_freeze_and_does_not_alter_estimates(tmp_path, monkeypatch):
    source = dataset(tmp_path)
    output = tmp_path/'results'
    original_open = Path.open
    truth_reads = []
    def guard(path, *args, **kwargs):
        if 'truth' in path.parts:
            assert (output/'run.json').is_file()
            assert (output/'three_axis/benign/windows.csv').is_file()
            assert (output/'three_axis/spoofed/windows.csv').is_file()
            truth_reads.append(path.name)
        return original_open(path, *args, **kwargs)
    monkeypatch.setattr(Path, 'open', guard)
    compare = motion_evaluation.compare_imu_with_oracle
    def verify_frozen(runs):
        arrays = [run['records'][0].motion_estimate.antenna_displacement_m.copy() for run in runs]
        predictions = [(run['records'][0].result.decision, run['records'][0].result.gamma) for run in runs]
        value = compare(runs)
        for run, before, prediction in zip(runs, arrays, predictions):
            assert_allclose(run['records'][0].motion_estimate.antenna_displacement_m, before, atol=0)
            assert (run['records'][0].result.decision, run['records'][0].result.gamma) == prediction
        return value
    monkeypatch.setattr(motion_evaluation, 'compare_imu_with_oracle', verify_frozen)
    runs, evaluation = replay_dataset(source, output, imu=True, plots=False)
    assert evaluation['summary']['correct_conclusive'] == 2
    assert {'labels.csv', 'motion_gnss.csv', 'benign_components.csv', 'spoofed_components.csv'} <= set(truth_reads)
    comparison = json.loads((output/'motion_comparison.json').read_text())
    assert comparison['summary']['decision_agreements'] == 2
    assert comparison['summary']['max_observable_displacement_vector_rmse_m'] < 2e-5
    assert comparison['windows'][0]['raw_displacement_vector_rmse_m'] > .1
    assert comparison['windows'][0]['phase_reference'] == 'actual_propagation_directions_from_held_out_truth'
    for filename in ['report.html', 'motion_report.html']:
        html = (output/filename).read_text()
        assert '__DATA__' not in html
        for target in re.findall(r'href="([^"]+)"', html):
            assert (output/target).is_file(), target


def test_missing_imu_sample_is_inconclusive_without_interpolation(tmp_path):
    source = dataset(tmp_path)
    path = source/'three_axis/inputs/imu.csv'
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    rows = [row for row in rows if float(row['time_s']) != .25]
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0]); writer.writeheader(); writer.writerows(rows)
    runs, evaluation = replay_dataset(source, tmp_path/'results', imu=True, plots=False)
    for run in runs:
        assert run['records'][0].result.reason == 'imu_time_gap'
        assert run['records'][0].result.decision == 'inconclusive'
        assert run['records'][0].motion_estimate is None
    assert evaluation['summary']['inconclusive_labeled_windows'] == 2


def test_noiseless_imu_baseline_with_large_unknown_trends(tmp_path):
    source = dataset(tmp_path, 'combined', sigma=0.)
    output = tmp_path/'results'
    runs, evaluation = replay_dataset(source, output, imu=True, plots=False)
    assert evaluation['summary']['correct_conclusive'] == 2
    comparison = json.loads((output/'motion_comparison.json').read_text())
    assert comparison['summary']['conclusive_in_both'] == 2
    for row in comparison['windows']:
        assert abs(row['gamma_difference']/row['oracle_gamma']) < .01
