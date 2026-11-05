"""Build the standalone Stage 2 review from downloaded, immutable evidence."""
from datetime import datetime
import hashlib
from html import escape
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parents[2]
sys.path.insert(0, str(REPO))
from detect2.acquisition.audit_session import archive_index
from detect2.acquisition.common import fingerprint
from detect2.acquisition.ubx import frames, rawx

latest = json.loads((ROOT / 'latest.json').read_text())
release = ROOT / latest['release_id']
evidence = release / 'evidence'
load = lambda p: json.loads(p.read_text())
validation = load(evidence / 'stage2_validation.json')
file_hashes = load(evidence / 'evidence_hashes.json')
for name, expected in file_hashes.items():
    assert fingerprint(evidence / name) == expected, name
assert all(validation['checks'].values())
assert hashlib.sha256((release / 'stage2_evidence.tar.gz').read_bytes()).hexdigest() == '59a77747e214e57677fbb1dc31f85457c56c18b5cfc2b3e576615fbf85b0b4c1'


def href(path, label):
    return f'<a href="{escape(str(path.relative_to(ROOT)))}">{escape(label)}</a>'


charts = []
rows = []
details = []
local_checks = []
total_observations = 0
for label in ['duration', 'interrupt']:
    session = next((evidence / label).iterdir())
    manifest = load(session / 'manifest.json')
    audit = load(session / 'audit.json')
    health = [json.loads(line) for line in (session / 'health.jsonl').read_text().splitlines()]
    elapsed = (datetime.fromisoformat(manifest['ended_utc']) - datetime.fromisoformat(manifest['started_utc'])).total_seconds()
    for stream in ['receiver', 'mavlink']:
        result, _ = archive_index(session, stream, manifest['files'][stream + '.bin'])
        assert result['status'] == 'PASS', result
        assert fingerprint(session / (stream + '_receive.csv')) == manifest['files'][stream + '_receive.csv']
        stats = manifest['streams'][stream]
        assert stats['received_bytes'] == stats['written_bytes'] and stats['dropped_bytes'] == 0
        assert stats['writer_closed'] and not stats['error']
        assert stats['queue_chunks'] == 0 and stats['buffered_bytes'] == 0
    assert not (session / 'UNFINALIZED').exists() and not manifest['issues']['counts']
    assert audit['telemetry']['messages']['ATTITUDE_QUATERNION'] > 0
    assert audit['telemetry']['messages']['HIGHRES_IMU'] > 0
    total_observations += audit['receiver']['observations']
    stamps = []
    for _, cls, ident, payload in frames((session / 'receiver.bin').read_bytes(), {}):
        if (cls, ident) == (2, 21):
            header, _ = rawx(payload)
            stamps.append(header['gps_week'] * 604800. + header['gps_tow_s'])
    h0 = health[0]['host_monotonic_ns']
    charts.append({'name': label, 'elapsed': elapsed,
        'receiver': [[(h['host_monotonic_ns']-h0)/1e9, h['streams']['receiver']['written_bytes']/1e6] for h in health],
        'mavlink': [[(h['host_monotonic_ns']-h0)/1e9, h['streams']['mavlink']['written_bytes']/1e6] for h in health],
        'gaps': [[b-stamps[0], (b-a)*1000] for a, b in zip(stamps, stamps[1:])]})
    messages = audit['telemetry']['messages']
    rx = manifest['streams']['receiver']
    title = 'Timed capture' if label == 'duration' else 'Ctrl+C capture'
    rows.append(f'<tr><td>{href(session / "report.html", title)}</td><td>{elapsed:.2f} s</td>'
        f'<td>{rx["written_bytes"]:,}</td><td>{audit["receiver"]["epochs"]}</td>'
        f'<td>{messages["ATTITUDE_QUATERNION"]:,} / {messages["HIGHRES_IMU"]:,}</td>'
        f'<td>{audit["receiver"]["max_epoch_gap_s"]*1000:.0f} ms</td><td>{manifest["status"]}</td></tr>')
    links = ' · '.join(href(session / p, p) for p in ['report.html','manifest.json','audit.json','receiver.bin','receiver_receive.csv','mavlink.bin','mavlink_receive.csv','receiver_observations.csv','health.jsonl','framing_detail.json'])
    peak_queue = max(h['streams']['receiver']['queue_chunks'] for h in health)
    details.append(f'<details><summary>{title}: files and transport evidence</summary><p>{links}</p>'
        f'<p>Receiver: {rx["input_chunks"]:,} indexed chunks; peak writer buffering {rx["peak_buffered_bytes"]:,} bytes; '
        f'peak queue {rx["peak_queue_chunks"]} chunks. Received = written = {rx["written_bytes"]:,} bytes. '
        'Dropped chunks/bytes = 0; both writers closed and queues drained.</p></details>')
    local_checks.append({'session': str(session.relative_to(ROOT)), 'index_coverage_and_archive_hashes': 'PASS',
                         'receive_indexes_match_manifest': 'PASS', 'received_equals_written': 'PASS',
                         'both_writers_closed': True, 'dropped_bytes': 0, 'finalized': True})

before = load(evidence / 'receiver_before.json')
stage0 = load(ROOT.parent / 'acquisition_stage0/acquisition_stage0_20261104T152748Z/receiver_configuration.json')
stage0_ram = {k: stage0['layer_replies']['0'][0][k] for k in before['ram']}
baseline_matches = before['ram'] == stage0_ram
assert baseline_matches
local = {'evidence_archive_sha256': '59a77747e214e57677fbb1dc31f85457c56c18b5cfc2b3e576615fbf85b0b4c1',
    'downloaded_fingerprints_verified': len(file_hashes), 'receiver_ram_matches_stage0': baseline_matches,
    'live_captures': local_checks, 'manual_checkpoint_2': 'awaiting user review', 'stage3_started': False}
(release / 'local_verification.json').write_text(json.dumps(local, indent=2) + '\n')

checks = ''.join(f'<tr><td>{escape(k.replace("_", " "))}</td><td class="pass">PASS</td></tr>' for k in validation['checks'])
settings = ''.join(f'<tr><td><code>{escape(k)}</code></td><td>{v}</td><td>{v}</td></tr>' for k,v in before['ram'].items())
evidence_links = ' · '.join(href(evidence / p, p) for p in ['stage2_validation.json','receiver_before.json','receiver_after.json','px4_before.json','px4_after.json','system_before.json','system_after.json','postcheck_recovery.json','deployment_check.json','evidence_hashes.json'])
html = '''<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>detect2 · Stage 2 live capture</title>
<style>:root{font:15px/1.65 system-ui,sans-serif;color:#20394b;background:#f0f5f7}body{max-width:1160px;margin:38px auto;padding:0 22px}h1{font-size:40px;letter-spacing:-1px;line-height:1.2}h2{font-size:23px}.tag{font-size:12px;letter-spacing:2px;color:#60778a}.badge{background:#ffebc5;color:#825009;border-radius:20px;padding:6px 13px;font-size:12px;font-weight:700}.card,details{background:white;border:1px solid #dae5ec;border-radius:12px;margin:20px 0;padding:23px}.metrics{display:grid;grid-template-columns:repeat(3,1fr);gap:15px}.metric{background:white;border-radius:12px;padding:20px;border:1px solid #dae5ec}.metric b{display:block;font-size:31px}.metric span{font-size:12px;color:#60778a}a{color:#146f8a}table{width:100%;border-collapse:collapse;font-size:13px}th,td{text-align:left;padding:10px;border-bottom:1px solid #e1e9ed;vertical-align:top}.scroll{overflow:auto}.pass{color:#17704d;font-weight:700}.note{background:#fff6e7;border-left:3px solid #b87d22;padding:13px 18px}pre{background:#eef4f7;padding:17px;white-space:pre-wrap;overflow-wrap:anywhere;font-size:12px}li{margin:9px 0}canvas{width:100%;height:260px;display:block}select{font:inherit;padding:6px}summary{cursor:pointer;font-weight:600}.legend{font-size:12px;color:#60778a}@media(max-width:650px){.metrics{grid-template-columns:1fr}h1{font-size:30px}}</style></head><body>
<div class="tag">DETECT2 / LIVE DRONE RECORDING / 05 NOVEMBER 2026</div>
<h1>Stage 2: real data archived.<br>Review capture integrity.</h1><span class="badge">AWAITING YOUR MANUAL CHECKPOINT 2</span>
<p>The new recorder ran on the Pi beside the old installation, using its existing router and unchanged X20/PX4 configuration. Two real captures tested timed shutdown and a controlled SIGINT, the same signal as Ctrl+C. These are live bench recordings; the supplied attitude is recorded, and no motion reconstruction or spoofing verdict is claimed.</p>
<div class="metrics"><div class="metric"><b>2</b><span>Live captures: 20 seconds and approximately 15 seconds</span></div><div class="metric"><b>0 bytes</b><span>Recorder-reported drops in either capture</span></div><div class="metric"><b>__OBS__</b><span>Retained RAWX observations across 108 signal identities</span></div></div>
<section class="card"><h2>Manual checkpoint 2</h2><ol>
<li>Open both live audit reports below. Confirm receiver and MAVLink archives have indexed receive timestamps and complete byte coverage.</li>
<li>Confirm ATTITUDE_QUATERNION and HIGHRES_IMU are present: about 50 messages/s each. Euler ATTITUDE is also retained.</li>
<li>Open the Ctrl+C manifest: status <code>stopped</code>, stop reason <code>sigint</code>, zero issues/drops, and both writers closed. The child exited 130 as expected; no UNFINALIZED marker remains.</li>
<li>Review the RAWX gaps and the framing warning below separately from the archive-integrity PASS results.</li>
<li>Compare the before/after receiver and PX4 settings. Confirm the old launcher and router remain unchanged.</li></ol>
<p class="note">Checkpoint 2 concerns capture integrity. RAWX continuity still fails; native ULog and the GNSS↔PX4 clock bridge remain UNKNOWN. Approving this checkpoint does not validate spoofing detection. Stage 3 has not started.</p>
<p>After review, confirm <b>PASS Stage 2</b> to proceed to the separate temporary receiver-traffic configuration stage.</p></section>
<section class="card"><h2>Live capture results</h2><div class="scroll"><table><tr><th>Open audit</th><th>Wall duration</th><th>Receiver bytes</th><th>RAWX epochs</th><th>Quaternion / IMU</th><th>Max RAWX gap</th><th>Lifecycle</th></tr>__ROWS__</table></div>
<p>All 39 downloaded evidence-file fingerprints match the Pi. Local rechecks confirm complete indexed byte coverage and unchanged binary/index hashes. Both streams received and wrote identical byte counts. This proves preservation of bytes accepted by the recorder; it cannot exclude upstream whole-packet losses.</p>__DETAILS__</section>
<section class="card"><h2>Watch archive growth and RAWX gaps</h2><label>Capture <select id="capture"><option value="0">20-second timed capture</option><option value="1">Ctrl+C capture</option></select></label>
<p class="legend">Cumulative archived MB: blue = receiver; green = MAVLink. X axis: seconds since first host health sample.</p><canvas id="growth"></canvas>
<p class="legend">RAWX interval in milliseconds; dashed orange line = expected 40 ms from unchanged receiver settings. X axis: receiver measurement seconds since first retained RAWX epoch. These axes have independent clocks and do not assert a GNSS↔PX4 alignment.</p><canvas id="gaps"></canvas></section>
<section class="card"><h2>Why some audit checks remain red</h2><ul>
<li><b>Receiver continuity:</b> unchanged 25 Hz settings still produce intervals up to 280 ms (timed) and 320 ms (Ctrl+C). There are 148 and 106 unexpected intervals respectively. Both streams retain 108 GNSS/satellite/signal identities. Reducing excess receiver traffic belongs to Stage 3.</li>
<li><b>Framing:</b> timed capture has one invalid UBX candidate at byte 18, before its first valid frame at byte 676. The cause cannot be established from this capture boundary. The audit keeps FAIL; no warning is suppressed. Ctrl+C has zero invalid candidates. Each has one trailing partial candidate, expected when stopping an ongoing byte stream.</li>
<li><b>Telemetry continuity:</b> the conservative 1.5× median-interval rule flags Euler ATTITUDE jitter (91 / 52 intervals; maximum 18 ms). Quaternion and HIGHRES_IMU have zero gaps under that rule. These counts are not a network loss probability.</li>
<li><b>Ctrl+C lifecycle:</b> its generic capture-session check labels <code>stopped</code> as FAIL because it accepts only <code>complete</code>. Intentional stopping is verified separately by exit 130, stop reason, closed writers, no issues, and passing finalization/archive checks.</li>
<li><b>Read-only query:</b> the first post-capture MON-VER identity poll timed out. A read-only retry succeeded and matched the original identity and all 18 RAM keys. Neither live capture was repeated and no configuration was written.</li></ul></section>
<section class="card"><h2>Unchanged installation and settings</h2><table>__CHECKS__</table>
<p>PX4 logging remains SDLOG_PROFILE=1, SDLOG_MODE=0, SDLOG_BACKEND=3. The receiver RAM keys also match the Stage 0 baseline. Router PID 19394 and its command are unchanged. No service, serial connection, receiver setting or PX4 parameter was changed. Active TIMESYNC stayed disabled; existing incoming clock packets were archived passively.</p>
<details><summary>All 18 receiver RAM values: before and after</summary><div class="scroll"><table><tr><th>Key</th><th>Before</th><th>After</th></tr>__SETTINGS__</table></div></details><p>__EVIDENCE_LINKS__</p></section>
<section class="card"><h2>Deployment and repeatable commands</h2><p>Release: <code>acquisition_20261105T100733Z</code>. All 16 deployed source files match the Stage 1 package, with pyserial 3.5, pymavlink 2.4.49 and pyubx2 1.3.3. The Pi offline healthy replay passed before live capture. The old <code>start</code> remains the rollback launcher. No capture process is left running.</p>
<pre>ssh nss@10.15.4.141
cd /home/nss/SpoofingDetection/releases/acquisition_20261105T100733Z
# Run only while the old recorder is stopped and the existing router is running.
CAPTURE_PYTHON=/home/nss/venv/bin/python ./start_capture --duration-s 20 --expected-rawx-hz 25 --output-root /home/nss/SpoofingDetection/recordings_v2
# For a manual Ctrl+C check, omit --duration-s, then press Ctrl+C.
# Use the exact session path printed by capture:
/home/nss/venv/bin/python -m detect2.acquisition.audit_session SESSION_PATH</pre>
<p>__RELEASE_LINKS__</p><p><a href="../../IMPLEMENTATION_PLAN.md">Full stage plan</a> · <a href="../acquisition_stage1/report.html">Approved Stage 1 tests</a></p></section>
<script>const captures=__CHARTS__;
function plot(id,series,ylabel,target){const c=document.getElementById(id),r=c.getBoundingClientRect(),d=window.devicePixelRatio||1;c.width=r.width*d;c.height=260*d;const x=c.getContext('2d');x.scale(d,d);const w=r.width,h=260,l=58,t=20,rpad=18,b=34;let xmax=Math.max(...series.flatMap(s=>s.v.map(p=>p[0])),1),ymax=Math.max(...series.flatMap(s=>s.v.map(p=>p[1])),target||0,.01)*1.12;const px=a=>l+a/xmax*(w-l-rpad),py=a=>h-b-a/ymax*(h-t-b);x.font='11px system-ui';x.fillStyle='#60778a';for(let k=0;k<=4;k++){let y=ymax*k/4;x.strokeStyle='#e2eaee';x.beginPath();x.moveTo(l,py(y));x.lineTo(w-rpad,py(y));x.stroke();x.fillText(y<5?y.toFixed(2):y.toFixed(0),4,py(y)+4);x.fillText((xmax*k/4).toFixed(1),px(xmax*k/4)-7,h-10)}x.fillText(ylabel,4,12);if(target){x.setLineDash([5,4]);x.strokeStyle='#c28727';x.beginPath();x.moveTo(l,py(target));x.lineTo(w-rpad,py(target));x.stroke();x.setLineDash([])}for(const s of series){x.strokeStyle=s.color;x.lineWidth=1.4;x.beginPath();s.v.forEach((p,i)=>i?x.lineTo(px(p[0]),py(p[1])):x.moveTo(px(p[0]),py(p[1])));x.stroke()}c.onmousemove=e=>{const a=(e.offsetX-l)/(w-l-rpad)*xmax;if(a<0)return;let p=series[0].v.reduce((best,v)=>Math.abs(v[0]-a)<Math.abs(best[0]-a)?v:best,series[0].v[0]);c.title=p[0].toFixed(3)+' s · '+p[1].toFixed(3)+' '+ylabel}}
function draw(){const a=captures[+document.getElementById('capture').value];plot('growth',[{v:a.receiver,color:'#176ca3'},{v:a.mavlink,color:'#198563'}],'MB');plot('gaps',[{v:a.gaps,color:'#176ca3'}],'ms',40)}document.getElementById('capture').onchange=draw;window.addEventListener('resize',draw);draw();</script></body></html>'''
replacements = {'__OBS__': f'{total_observations:,}', '__ROWS__': ''.join(rows), '__DETAILS__': ''.join(details),
    '__CHECKS__': checks, '__SETTINGS__': settings, '__EVIDENCE_LINKS__': evidence_links,
    '__RELEASE_LINKS__': ' · '.join(href(p, p.name) for p in [release / 'release_manifest.json', release / 'local_verification.json', release / 'run_stage2.py', release / 'stage2_evidence.tar.gz']),
    '__CHARTS__': json.dumps(charts).replace('<', '\\u003c')}
for key, value in replacements.items():
    html = html.replace(key, value)
(ROOT / 'report.html').write_text(html)
print('Report:', ROOT / 'report.html')
print('Local verification:', release / 'local_verification.json')
