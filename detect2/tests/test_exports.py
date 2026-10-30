import csv
import json
import re

import numpy as np
from numpy.testing import assert_allclose

from SpoofingDetection.detect2.geometry import LocalFrame
from SpoofingDetection.detect2.synthetic.generate import generate
from SpoofingDetection.detect2.synthetic.motions import SCENARIOS


def test_all_scenario_exports_have_consistent_units_and_offline_report(tmp_path):
    metrics = generate(tmp_path, list(SCENARIOS), LocalFrame(0, 0, 10),
                       duration_s=.1, rate_hz=100, plots=False)
    assert set(metrics) == set(SCENARIOS)
    for name in SCENARIOS:
        with (tmp_path/name/'truth.csv').open() as stream:
            rows = list(csv.DictReader(stream))
        assert len(rows) == 11
        time = [float(row['time_s']) for row in rows]
        assert_allclose(time, np.arange(11)/100)
        # At this equatorial site, local Up maps directly to ECEF X.
        height_offset = .3/np.sqrt(2) if name == 'circle_coning' else .2
        assert_allclose(float(rows[0]['antenna_ecef_x_m']), 6378137+10+height_offset, atol=1e-9)
        assert_allclose(float(rows[0]['antenna_position_ned_down_m']), -height_offset)
        assert_allclose(float(rows[0]['antenna_tilt_from_vertical_deg']), 45 if name == 'circle_coning' else 0)
        config = json.loads((tmp_path/name/'config.json').read_text())
        assert config['origin']['height_m'] == 10
        assert metrics[name]['max_enu_ecef_roundtrip_m'] < 2e-9
        with (tmp_path/name/'geometric_projections.csv').open() as stream:
            projections = list(csv.DictReader(stream))
        assert_allclose([float(value) for key, value in projections[0].items() if key != 'time_s'], 0.)
    html = (tmp_path/'report.html').read_text()
    assert '__DATA__' not in html and '__OPTIONS__' not in html
    assert '<script src=' not in html
    payload = re.search(r'<script id="motion-data" type="application/json">(.*?)</script>', html, re.S)
    data = json.loads(payload.group(1))
    assert set(data) == set(SCENARIOS)
    assert len(data['combined']['rotation']) == 11
    assert 'boresight' in data['circle_coning'] and 'forward' in data['circle_coning']
    for target in re.findall(r'href="([^"]+)"', html):
        assert (tmp_path/target).is_file(), target
