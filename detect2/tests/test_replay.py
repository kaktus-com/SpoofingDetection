import csv
import json
from pathlib import Path
import re

import numpy as np
from numpy.testing import assert_allclose
import pytest

from SpoofingDetection.detect2 import evaluation, schema
from SpoofingDetection.detect2.replay import replay_dataset, run_windows
from SpoofingDetection.detect2.schema import load_inputs
from SpoofingDetection.detect2.synthetic.carrier import CarrierConfig
from SpoofingDetection.detect2.synthetic.generate_carrier import generate_datasets
from SpoofingDetection.detect2.synthetic.motions import MotionConfig


def dataset(tmp_path, scenario='three_axis', sigma=.01):
    folder = tmp_path/'observations'
    generate_datasets(folder, [MotionConfig(scenario, duration_s=1)], CarrierConfig(noise_sigma_cycles=sigma), plots=False)
    return folder


def test_loader_reads_only_measured_inputs_and_explicit_displacement(tmp_path, monkeypatch):
    root = dataset(tmp_path)/'three_axis'
    opened = []
    original = schema._rows
    def spy(path, required):
        opened.append(Path(path).relative_to(root).as_posix())
        return original(path, required)
    monkeypatch.setattr(schema, '_rows', spy)
    data = load_inputs(root/'benign/carrier.csv', root/'inputs/satellite_geometry.csv', root/'truth/motion_gnss.csv')
    assert set(opened) == {'benign/carrier.csv', 'inputs/satellite_geometry.csv', 'truth/motion_gnss.csv'}
    assert data.phase_cycles.shape == (21, 8)
    records = run_windows(data)
    assert len(records) == 1
    assert len(records[0].data.time_s) == 20
    assert records[0].data.time_s[-1] == pytest.approx(.95)
    assert records[0].result.decision == 'benign'


def test_predictions_are_frozen_before_labels_and_do_not_depend_on_them(tmp_path, monkeypatch):
    source = dataset(tmp_path)
    output = tmp_path/'results'
    original = evaluation.evaluate_predictions
    def after_prediction(runs):
        assert (output/'run.json').is_file()
        assert (output/'three_axis/benign/windows.csv').is_file()
        return original(runs)
    monkeypatch.setattr(evaluation, 'evaluate_predictions', after_prediction)
    runs, measured = replay_dataset(source, output, oracle=True, cases=['benign'], plots=False)
    assert measured['summary']['correct_conclusive'] == 1
    before = (output/'three_axis/benign/windows.csv').read_bytes()
    label_path = source/'three_axis/truth/labels.csv'
    with label_path.open() as stream:
        rows = list(csv.DictReader(stream))
    for row in rows:
        row['is_spoofed'] = '1' if row['is_spoofed'] == '0' else '0'
    with label_path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0]); writer.writeheader(); writer.writerows(rows)
    _, changed = replay_dataset(source, output, oracle=True, cases=['benign'], plots=False)
    assert changed['summary']['correct_conclusive'] == 0
    assert changed['summary']['missed_detections'] == 1
    assert (output/'three_axis/benign/windows.csv').read_bytes() == before
    html = (output/'report.html').read_text()
    payload = json.loads(re.search(r'<script id="detector-data" type="application/json">(.*?)</script>', html, re.S).group(1))
    assert payload['three_axis']['benign'][0]['decision'] == 'benign'
    assert payload['three_axis']['benign'][0]['truth'] == 'spoofed'
    for target in re.findall(r'href="([^"]+)"', html):
        assert (output/target).is_file(), target


def test_unneeded_truth_files_and_case_names_do_not_affect_prediction(tmp_path):
    source = dataset(tmp_path)
    scenario = source/'three_axis'
    for path in (scenario/'truth').glob('*'):
        if path.name != 'motion_gnss.csv':
            path.unlink()
    manifest_path = scenario/'manifest.json'
    manifest = json.loads(manifest_path.read_text())
    manifest['carrier_files'] = {'unlabeled_recording': manifest['carrier_files']['benign']}
    manifest_path.write_text(json.dumps(manifest))
    output = tmp_path/'results'
    runs, measured = replay_dataset(source, output, oracle=True, evaluate=False, plots=False)
    assert runs[0]['records'][0].result.decision == 'benign'
    assert measured['status'] == 'not_requested'
    assert not (output/'evaluation.json').exists()
    with pytest.raises(ValueError):
        replay_dataset(source, tmp_path/'forbidden', oracle=False, plots=False)
    with pytest.raises(ValueError):
        replay_dataset(source, source/'overwritten', oracle=True, plots=False)


def test_loader_rejects_duplicate_epochs_and_misaligned_oracle(tmp_path):
    root = dataset(tmp_path)/'three_axis'
    carrier, geometry, oracle = root/'benign/carrier.csv', root/'inputs/satellite_geometry.csv', root/'truth/motion_gnss.csv'
    lines = carrier.read_text().splitlines()
    carrier.write_text('\n'.join(lines+[lines[-1]])+'\n')
    with pytest.raises(ValueError, match='increase strictly'):
        load_inputs(carrier, geometry, oracle)
    carrier.write_text('\n'.join(lines)+'\n')
    with oracle.open() as stream:
        rows = list(csv.DictReader(stream))
    rows[0]['time_s'] = '.001'
    with oracle.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0]); writer.writeheader(); writer.writerows(rows)
    with pytest.raises(ValueError, match='does not match'):
        load_inputs(carrier, geometry, oracle)


def test_stationary_evaluation_retains_inconclusive_coverage(tmp_path):
    source = dataset(tmp_path, 'stationary')
    runs, result = replay_dataset(source, tmp_path/'results', oracle=True, plots=False)
    assert result['summary']['total_windows'] == 2
    assert result['summary']['inconclusive_labeled_windows'] == 2
    assert result['summary']['conclusive_coverage'] == 0
    assert result['summary']['accuracy_on_conclusive'] is None
