"""Review live Stage 3 recordings, transport headroom and unchanged controls."""
from collections import Counter
from datetime import datetime
from html import escape
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parents[2]))
from detect2.acquisition.audit_session import archive_index
from detect2.acquisition.common import fingerprint
from detect2.acquisition.ubx import frames, rawx

load = lambda p: json.loads(p.read_text())
latest = load(ROOT/'latest.json')
release = ROOT/latest['release_id']
evidence = release/'evidence'
files = load(evidence/'evidence_hashes.json')
for name, expected in files.items():
    assert fingerprint(evidence/name) == expected, name
validation = load(evidence/'stage3_validation.json')
before = load(evidence/'receiver_before.json')
after = load(evidence/'receiver_after.json')


def link(p, text=None):
    return f'<a href="{escape(str(p.relative_to(ROOT)))}">{escape(text or p.name)}</a>'


def inspect(session, baud, label, health_path=True):
    m = load(session/'manifest.json'); a = load(session/'audit.json')
    for name in ['receiver','mavlink']:
        check, _ = archive_index(session,name,m['files'][name+'.bin'])
        assert check['status'] == 'PASS', check
        assert fingerprint(session/(name+'_receive.csv')) == m['files'][name+'_receive.csv']
    health = [json.loads(line) for line in (session/'health.jsonl').read_text().splitlines()]
    duration = (datetime.fromisoformat(m['ended_utc'])-datetime.fromisoformat(m['started_utc'])).total_seconds()
    # Steady host-receive window excludes startup and shutdown health rows.
    middle = health[2:-1]
    dt = (middle[-1]['host_monotonic_ns']-middle[0]['host_monotonic_ns'])/1e9
    bps = (middle[-1]['streams']['receiver']['received_bytes']-middle[0]['streams']['receiver']['received_bytes'])/dt
    counts = Counter(); sizes = Counter(); epochs = []; offsets = []; signals = Counter()
    for offset, cls, ident, payload in frames((session/'receiver.bin').read_bytes(), {}):
        key = f'{cls:02x}-{ident:02x}'; counts[key] += 1; sizes[key] += 8+len(payload)
        if (cls,ident) == (2,21):
            h, obs = rawx(payload); epochs.append(h['gps_week']*604800.+h['gps_tow_s']); offsets.append(offset)
            signals.update(str(o['gnss_id']) for o in obs)
    h0 = health[0]['host_monotonic_ns']
    series = {'name':label,'growth': [[(h['host_monotonic_ns']-h0)/1e9,h['streams']['receiver']['written_bytes']/1e6] for h in health],
              'gaps': [[b-epochs[0],(b-c)*1000] for c,b in zip(epochs,epochs[1:])],
              'throughput': [[(b['host_monotonic_ns']-h0)/1e9,(b['streams']['receiver']['received_bytes']-c['streams']['receiver']['received_bytes'])/((b['host_monotonic_ns']-c['host_monotonic_ns'])/1e9)/1000] for c,b in zip(health,health[1:]) if b['host_monotonic_ns']>c['host_monotonic_ns']]}
    comms = load(session/'receiver_comms.json') if (session/'receiver_comms.json').exists() else []
    uart = [p for row in comms for p in row['ports'] if p['portId']==256]
    summary = {'label':label,'baud':baud,'duration_s':duration,'rawx_epochs':len(epochs),
               'unexpected_rawx_intervals':a['checks']['receiver_epoch_continuity']['unexpected_intervals'],
               'max_rawx_gap_ms':a['receiver']['max_epoch_gap_s']*1000,'steady_receiver_bytes_per_s':bps,
               'uart_capacity_bytes_per_s':baud/10,'uart_utilization':bps/(baud/10),
               'signal_observation_identities':len(a['receiver']['signal_counts']),
               'observation_count':a['receiver']['observations'], 'rawx_diagnostics':a['receiver']['diagnostics'],
               'invalid_carrier_flag_observations':a['receiver']['invalid_carrier_flag_observations'],
               'packet_counts':dict(counts),'packet_bytes':dict(sizes),'observation_counts_by_gnss_id':dict(signals),
               'recorder_dropped_bytes':sum(s['dropped_bytes'] for s in m['streams'].values()),
               'receiver_peak_writer_buffer_bytes':m['streams']['receiver']['peak_buffered_bytes'],
               'uart1_health_samples':len(uart),'uart1_max_tx_usage_percent':max((p['txUsage'] for p in uart),default=None),
               'uart1_max_tx_pending_bytes':max((p['txPending'] for p in uart),default=None),
               'uart1_skipped_counter_change':uart[-1]['skipped']-uart[0]['skipped'] if uart else None,
               'uart1_overrun_counter_change':uart[-1]['overrunErrs']-uart[0]['overrunErrs'] if uart else None,
               'session':str(session),'status':m['status'],'stream_integrity':all(s['writer_closed'] and s['received_bytes']==s['written_bytes'] and s['dropped_bytes']==0 and not s['error'] for s in m['streams'].values()),
               'low_rate_packets_present':all(counts[k]>0 for k in ['01-07','01-35','0a-36','02-13','0d-01'])}
    assert m['status']=='complete' and not m['issues']['counts'] and summary['stream_integrity']
    assert not (session/'UNFINALIZED').exists()
    return summary,series,a


summaries=[]; charts=[]; audits=[]; sessions=[]
for name,baud,label in [('reduced_921600',921600,'Final messages · 921600 baud'),('reduced_2000000',2000000,'Final profile · 2 Mbaud · 1 Hz health')]:
    session=next((evidence/name).iterdir());sessions.append(session)
    summary,series,audit=inspect(session,baud,label);summaries.append(summary);charts.append(series);audits.append(audit)
main=summaries[-1]
flash_before=load(evidence/'flash_before.json');flash_after=load(evidence/'flash_after.json')
flash_unchanged=flash_before['status']=='PASS' and flash_after['status']=='PASS' and flash_before['values']==flash_after['values']
acceptance={'all_configuration_checks_pass':all(validation['checks'].values()),'flash_touched_key_values_unchanged':flash_unchanged,
    'two_minute_capture_completed':main['duration_s']>=120,'archive_integrity':main['stream_integrity'],
    'rawx_continuity':audits[-1]['checks']['receiver_epoch_continuity']['status']=='PASS',
    'receiver_frame_validation':audits[-1]['checks']['receiver_frame_validation']['status']=='PASS',
    'uart_headroom_at_least_20_percent':main['uart_utilization']<=.8,
    'uart_health_present':main['uart1_health_samples']>=100,
    'uart_tx_usage_below_80_percent':main['uart1_max_tx_usage_percent'] is not None and main['uart1_max_tx_usage_percent']<80,
    'uart_no_new_skipped_bytes':main['uart1_skipped_counter_change']==0,
    'uart_no_new_overruns':main['uart1_overrun_counter_change']==0,
    'navigation_health_subframe_timepulse_packets_present':main['low_rate_packets_present']}
summary={'stage':3,'results':summaries,'automated_acceptance':acceptance,'all_traffic_checks_pass':all(acceptance.values()),
         'manual_checkpoint_3':'awaiting user review','current_receiver_baud':2000000,'persistent_writes':False,
         'constellation_changes_needed':False if all(acceptance.values()) else 'review evidence',
         'gnss_to_px4_clock':'UNKNOWN','native_ulog':'UNKNOWN','ready_for_validated_detector':False}
assert after['ram']['CFG_MSGOUT_UBX_MON_COMMS_UART1']==1
assert after['ram']['CFG_UART1_BAUDRATE']==2000000
assert all(after['ram'][k]==v for k,v in load(ROOT.parents[1]/'acquisition/x20_recording_profile.json')['settings'].items())
(release/'review_summary.json').write_text(json.dumps(summary,indent=2)+'\n')

table=''.join(f'<tr><td>{link(p/"report.html",s["label"])}</td><td>{s["duration_s"]:.1f} s</td><td>{s["rawx_epochs"]:,}</td><td>{s["unexpected_rawx_intervals"]}</td><td>{s["max_rawx_gap_ms"]:.2f} ms</td><td>{s["steady_receiver_bytes_per_s"]/1000:.1f} kB/s</td><td>{s["uart_utilization"]*100:.1f}%</td><td>{s["recorder_dropped_bytes"]}</td></tr>' for s,p in zip(summaries,sessions))
checkrows=''.join(f'<tr><td>{escape(k.replace("_"," "))}</td><td class="{str(v).lower()}">{"PASS" if v else "REVIEW"}</td></tr>' for k,v in acceptance.items())
settings=''.join(f'<tr><td><code>{k}</code></td><td>{before["ram"][k]}</td><td>{after["ram"][k]}</td></tr>' for k in before['ram'])
detail=''.join('<details><summary>'+escape(s['label'])+' — raw data and diagnostics</summary><p>'+' · '.join(link(p/n) for n in ['manifest.json','audit.json','report.html','receiver.bin','receiver_receive.csv','mavlink.bin','mavlink_receive.csv','receiver_observations.csv','receiver_comms.json','health.jsonl'])+'</p></details>' for s,p in zip(summaries,sessions))
links=' · '.join(link(evidence/n) for n in ['reviewed_diff.json','profile_transaction.json','baud_transaction.json','receiver_before.json','receiver_after.json','flash_before.json','flash_after.json','px4_before.json','px4_after.json','system_before.json','system_after.json','stage3_validation.json','evidence_hashes.json'])
html='''<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>detect2 · Stage 3 receiver traffic</title><style>:root{font:15px/1.65 system-ui,sans-serif;color:#223b4c;background:#eff5f7}body{max-width:1180px;margin:35px auto;padding:0 22px}h1{font-size:39px;line-height:1.2;letter-spacing:-1px}h2{font-size:23px}.tag{font-size:12px;letter-spacing:2px;color:#617b8d}.badge{background:#fff0cd;color:#805818;padding:6px 13px;border-radius:20px;font-size:12px;font-weight:700}.card,details{background:white;border:1px solid #d9e5ec;border-radius:12px;padding:24px;margin:20px 0}.metrics{display:grid;grid-template-columns:repeat(3,1fr);gap:14px}.metric{background:white;border:1px solid #d9e5ec;border-radius:12px;padding:20px}.metric b{display:block;font-size:31px}.metric span{font-size:12px;color:#617b8d}a{color:#146f8a}table{width:100%;border-collapse:collapse;font-size:13px}th,td{text-align:left;padding:9px;border-bottom:1px solid #e2e9ed;vertical-align:top}.scroll{overflow:auto}.true{color:#187447;font-weight:700}.false{color:#a15e16;font-weight:700}.note{background:#fff7e9;border-left:3px solid #bb8536;padding:14px 18px}pre{background:#f0f5f7;padding:17px;white-space:pre-wrap;overflow-wrap:anywhere;font-size:12px}summary{cursor:pointer;font-weight:600}li{margin:9px 0}canvas{width:100%;height:240px;display:block}select{padding:6px;font:inherit}.legend{color:#617b8d;font-size:12px}@media(max-width:650px){.metrics{grid-template-columns:1fr}}</style></head><body>
<div class="tag">DETECT2 / TEMPORARY RECEIVER CONFIGURATION / 05 NOVEMBER 2026</div><h1>Stage 3: receiver traffic and 2 Mbaud.</h1><span class="badge">AWAITING MANUAL CHECKPOINT 3</span>
<p>The authorized changes applied the reviewed message profile and switched X20 UART1 plus the FT230X host connection from 921600 to 2000000 baud. The message-profile and baud transactions were journaled and verified by readback. All seven constellation-enable flags match the baseline. No flash/BBR writes, PX4 parameter changes or router restarts were issued.</p>
<div class="metrics"><div class="metric"><b>__GAPS__</b><span>Unexpected RAWX intervals in the two-minute test</span></div><div class="metric"><b>__USAGE__%</b><span>Measured steady UART capacity used at 2 Mbaud</span></div><div class="metric"><b>0 bytes</b><span>Recorder-reported drops in both final tests</span></div></div>
<section class="card"><h2>Manual checkpoint 3</h2><ol><li>Open the two-minute audit. Inspect the RAWX intervals: target 50 ms, with zero missing/repeated epochs and zero clock resets.</li><li>Compare the exact profile and baud transaction readbacks. Confirm the active UART baud is 2000000 and the measurement interval is 50 ms.</li><li>Review throughput and MON-COMMS backlog below. Check headroom and zero growth in skipped/overrun counters; historical peak counters may retain earlier saturation.</li><li>Confirm that observations from every enabled constellation remain archived, including original phase/quality flags, and that navigation, health, subframe and timepulse packets are present.</li><li>Verify the capture command and rollback procedure below. Review before approving persistent settings or continuing to PX4 native logging.</li></ol><p class="note">The receiver currently uses temporary RAM settings at <b>2 Mbaud</b>. Use the explicit command below. The unchanged legacy launcher assumes 921600 baud and needs the documented rollback first. Persistent settings and Stage 4 have not been applied.</p></section>
<section class="card"><h2>Measured comparison</h2><div class="scroll"><table><tr><th>Open live audit</th><th>Duration</th><th>RAWX epochs</th><th>Unexpected intervals</th><th>Max interval</th><th>Steady receiver traffic</th><th>UART usage</th><th>Dropped bytes</th></tr>__TABLE__</table></div><p>Before these changes, Stage 2 had gaps up to 280–320 ms. The first test changes messages/rate only; the second also changes baud. Both final tests use MON-COMMS at 1 Hz; the earlier attempt exposed and corrected the initial base-cycle assumption. Thus any improvement already seen at 921600 is evidence for the reduced-message profile, rather than proof that increasing baud alone fixed the original stream.</p>__DETAIL__</section>
<section class="card"><h2>Traffic and measurement intervals</h2><label>Capture <select id="choice"><option value="0">Reduced messages at 921600</option><option value="1" selected>Final 2 Mbaud · 1 Hz health</option></select></label><p class="legend">Receiver throughput (kB/s), indexed by host receive time. Dashed line: 80% of the selected 8N1 UART capacity.</p><canvas id="throughput"></canvas><p class="legend">RAWX intervals (ms) against receiver elapsed measurement time. Dashed line: expected 50 ms. This separate clock axis does not establish GNSS↔PX4 alignment.</p><canvas id="gaps"></canvas></section>
<section class="card"><h2>Automated traffic acceptance</h2><table>__CHECKS__</table><p>UART1 health: __HEALTH__. Capture integrity verifies archived bytes and their indexes; it cannot by itself exclude upstream whole-packet loss. Native ULog and the GNSS↔PX4 measurement-clock bridge remain UNKNOWN, so this is not a validated spoofing result.</p><p>The final capture retained __INVALID_FLAGS__ observations with an invalid carrier flag; these are available for downstream quality rejection and are not silently filtered. Valid transport does not imply every phase measurement is usable. The telemetry audit can still flag Euler ATTITUDE timestamp jitter. Read the individual quaternion/IMU stream checks separately from RAWX continuity.</p></section>
<section class="card"><h2>Earlier failed attempt and recorder correction</h2><p>The initial 128-chunk limit overflowed during a 2 Mbaud repeat: 21,200 receiver bytes and 10,271 MAVLink bytes were explicitly rejected. The session reported failed and automatic rollback restored all original RAM keys and 921600 baud. This failed capture is retained and is not counted as a passing test.</p><p>The revised recorder allows 4096 chunks per stream, retains the 8 MiB byte cap, and records those limits in the manifest. A paused-writer burst test was added; all 21 acquisition/baud tests passed before this release. Both final captures below use the revised recorder and corrected 1 Hz MON-COMMS profile. This short acceptance test does not guarantee loss-free operation during longer filesystem stalls.</p><p>__FAILURE_LINKS__</p></section><section class="card"><h2>Constellations and configuration</h2><p>__CONSTELLATIONS__</p><p>Message cleanup disables NAV-SIG, RXM-MEASX and duplicate high-rate navigation outputs, retains RAWX at 20 Hz, and enables lower-rate navigation/communication health plus SFRBX/TIM-TP output. Exact values are below. TIM-TP packet presence confirms output availability; physical PPS wiring and its timing accuracy are not verified here.</p><details><summary>RAM before / after, including all constellation-enable flags</summary><div class="scroll"><table><tr><th>Key</th><th>Before</th><th>After</th></tr>__SETTINGS__</table></div></details><p>__LINKS__</p></section>
<section class="card"><h2>Run and restore</h2><pre>ssh nss@10.15.4.141
cd /home/nss/SpoofingDetection/releases/acquisition_20261105T103442Z
# Current temporary 2 Mbaud profile; existing router must already be running.
CAPTURE_PYTHON=/home/nss/venv/bin/python ./start_capture --gps-baud 2000000 --expected-rawx-hz 20 --duration-s 120 --output-root /home/nss/SpoofingDetection/recordings_v2
# Audit the exact session printed by capture:
/home/nss/venv/bin/python -m detect2.acquisition.audit_session SESSION_PATH

# Stop any recorder before rollback. Restore the original message profile
# and 921600 baud, verifying the same receiver and final readback:
/home/nss/venv/bin/python run_stage3.py restore</pre><p>No recorder is left running after the tests. Do not rely on a power cycle to select a profile: RAM-only changes are volatile, and startup/persistent settings may differ. Explicitly verify baud/profile before another capture.</p><p>__ARTIFACTS__</p><p>Hardware references: <a href="https://content.u-blox.com/sites/default/files/documents/ZED-X20P_IntegrationManual_UBXDOC-963802114-12901.pdf">u-blox X20 UART configuration</a> · <a href="https://ftdichip.com/products/ft230xs/">FT230X up to 3 Mbaud</a>. The live identity/readback and continuity test validate this installed HPG 2.00 setup beyond the documentation claim.</p><p><a href="../../IMPLEMENTATION_PLAN.md">Stage plan</a> · <a href="../acquisition_stage2/report.html">Stage 2 baseline</a></p></section>
<script>const datasets=__CHARTS__;function plot(id,points,target,unit){const c=document.getElementById(id),w=c.getBoundingClientRect().width,d=devicePixelRatio||1,h=240;c.width=w*d;c.height=h*d;const q=c.getContext('2d');q.scale(d,d);const l=58,b=32,t=20,r=15,xmax=Math.max(...points.map(p=>p[0]),1),ymax=Math.max(...points.map(p=>p[1]),target)*1.15;const px=x=>l+x/xmax*(w-l-r),py=y=>h-b-y/ymax*(h-b-t);q.font='11px system-ui';q.fillStyle='#60798c';q.fillText(unit,4,12);for(let i=0;i<=4;i++){const y=ymax*i/4;q.strokeStyle='#e0e9ed';q.beginPath();q.moveTo(l,py(y));q.lineTo(w-r,py(y));q.stroke();q.fillText(y.toFixed(1),4,py(y)+4);q.fillText((xmax*i/4).toFixed(0),px(xmax*i/4)-6,h-8)}q.setLineDash([5,4]);q.strokeStyle='#bf8635';q.beginPath();q.moveTo(l,py(target));q.lineTo(w-r,py(target));q.stroke();q.setLineDash([]);q.strokeStyle='#176f9d';q.lineWidth=1.3;q.beginPath();points.forEach((p,i)=>i?q.lineTo(px(p[0]),py(p[1])):q.moveTo(px(p[0]),py(p[1])));q.stroke();c.onmousemove=e=>{let a=(e.offsetX-l)/(w-l-r)*xmax;const nearest=points.reduce((v,p)=>Math.abs(p[0]-a)<Math.abs(v[0]-a)?p:v,points[0]);c.title=nearest[0].toFixed(3)+' s · '+nearest[1].toFixed(3)+' '+unit}}function draw(){let i=+document.getElementById('choice').value,a=datasets[i];plot('throughput',a.throughput,(i?200:92.16)*.8,'kB/s');plot('gaps',a.gaps,50,'ms')}document.getElementById('choice').onchange=draw;window.addEventListener('resize',draw);draw();</script></body></html>'''
replacements={'__INVALID_FLAGS__':str(main['invalid_carrier_flag_observations']),'__GAPS__':str(main['unexpected_rawx_intervals']),'__USAGE__':f'{main["uart_utilization"]*100:.1f}',
    '__TABLE__':table,'__DETAIL__':detail,'__CHECKS__':checkrows,'__SETTINGS__':settings,'__LINKS__':links,
    '__HEALTH__':f'{main["uart1_health_samples"]} samples, maximum current TX usage {main["uart1_max_tx_usage_percent"]}%, maximum pending {main["uart1_max_tx_pending_bytes"]} bytes, skipped-counter change {main["uart1_skipped_counter_change"]}, overrun-counter change {main["uart1_overrun_counter_change"]}',
    '__CONSTELLATIONS__':('The two-minute test meets all automated traffic targets with the existing constellations enabled. No constellation reduction is needed for this configuration and observed sky view.' if all(acceptance.values()) else 'Constellation-enable flags were preserved. Some traffic targets need review; inspect the checks before deciding whether a reduced-constellation fallback is needed.'),
    '__FAILURE_LINKS__': ' · '.join(link(p) for p in [(ROOT/'acquisition_20261105T102353Z/previous_evidence/failed_capture/report.html'), (ROOT/'acquisition_20261105T102353Z/previous_evidence/rollback_readback.json'), (ROOT/'acquisition_20261105T102353Z/previous_evidence/previous_attempt_summary.json')]),
    '__ARTIFACTS__':' · '.join(link(p) for p in [release/'review_summary.json',release/'local_verification.json',release/'release_manifest.json',release/'test_result.txt',release/'run_stage3.py',release/'stage3_evidence.tar.gz']),
    '__CHARTS__':json.dumps(charts).replace('<','\\u003c')}
for key,value in replacements.items():html=html.replace(key,value)
(ROOT/'report.html').write_text(html)
print(json.dumps({'automated_acceptance':acceptance,'main_result':main},indent=2))
print('HTML report:',ROOT/'report.html')
