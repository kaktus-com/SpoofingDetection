import csv
import json
from pathlib import Path
import re
import shutil

from SpoofingDetection.detect2.imu_noise_validation import run_validation
from SpoofingDetection.detect2.synthetic.carrier import CarrierConfig
from SpoofingDetection.detect2.synthetic.generate_carrier import generate_datasets
from SpoofingDetection.detect2.synthetic.motions import MotionConfig


def make_source(tmp_path):
    root = tmp_path/'observations'
    generate_datasets(root, [MotionConfig(name, duration_s=3) for name in ['stationary', 'combined']],
                      CarrierConfig(noise_rho=.4), plots=False)
    return root


def test_paired_validation_freezes_every_mode_before_truth_and_preserves_likelihood(tmp_path, monkeypatch):
    root, output = make_source(tmp_path), tmp_path/'validation'
    before = (root/'combined/inputs/imu.csv').read_bytes()
    original_open = Path.open
    names = ['ideal', 'seed_42_sigma_0p5', 'seed_43_sigma_0p5']
    def guard(path, *args, **kwargs):
        if 'truth' in path.parts:
            assert (output/'run.json').is_file()
            for name in names:
                for mode in ['gated', 'ungated']:
                    for scenario in ['stationary', 'combined']:
                        for case in ['benign', 'spoofed']:
                            assert (output/name/mode/scenario/case/'windows.csv').is_file()
        return original_open(path, *args, **kwargs)
    monkeypatch.setattr(Path, 'open', guard)
    rows, windows = run_validation(root, output, seeds=[42, 43], sigmas=[.5], plots=False)
    assert len(windows) == 12
    assert (root/'combined/inputs/imu.csv').read_bytes() == before
    assert sum(row['false_alarms']+row['missed_detections'] for row in rows if row['mode'] == 'gated') == 0
    assert all(row['gated_decision'] == 'inconclusive' for row in windows if row['scenario'] == 'stationary')
    for row in windows:
        if row['gated_decision'] != 'inconclusive':
            assert row['gated_gamma'] == row['ungated_gamma']
        if row['noise_gate_rejected']:
            assert row['gated_gamma'] is None
    for name in names:
        meta = json.loads((output/name/'gated/run.json').read_text())
        assert all(item['config']['noise_rho'] == .4 for item in meta['runs'])
        assert all(Path(item['perturbed_imu']).is_file() for item in meta['runs'])
        for mode in ['gated', 'ungated']:
            html = (output/name/mode/'report.html').read_text()
            for target in re.findall(r'href="([^"]+)"', html):
                assert (output/name/mode/target).is_file()
    for target in re.findall(r'href="([^"]+)"', (output/'report.html').read_text()):
        assert (output/target).is_file()


def test_paired_prediction_only_works_without_truth(tmp_path, monkeypatch):
    source = make_source(tmp_path)
    for scenario in ['stationary', 'combined']:
        shutil.rmtree(source/scenario/'truth')
    original_open = Path.open
    def guard(path, *args, **kwargs):
        assert 'truth' not in path.parts
        return original_open(path, *args, **kwargs)
    monkeypatch.setattr(Path, 'open', guard)
    rows, windows = run_validation(source, tmp_path/'predictions', seeds=[43], sigmas=[.5], evaluate=False, plots=False)
    assert rows == windows == []
