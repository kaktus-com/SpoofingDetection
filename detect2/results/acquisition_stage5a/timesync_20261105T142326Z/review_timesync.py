"""Rebuild the local Stage 5A review from the hashed drone evidence snapshot."""
from pathlib import Path
import csv
from html import escape
import json
import sys

ROOT=Path(__file__).resolve().parent
REPO=ROOT.parents[3]
sys.path.insert(0,str(REPO))
from detect2.acquisition.audit_session import audit as raw_audit
from detect2.acquisition.clock_report import report as clock_report
from detect2.acquisition.runtime_report import report as recording_report
from detect2.acquisition.common import fingerprint
from detect2.acquisition.ulog_audit import audit as native_audit
from detect2.acquisition.timesync import message_target
from pymavlink.dialects.v20 import common as mav
import numpy as np
from pyulog import ULog


def load(p):return json.loads(p.read_text())


def tx_check(folder):
    data=(folder/'mavlink_tx.bin').read_bytes();offset=0;count=0
    with (folder/'mavlink_tx.csv').open() as f:
        for r in csv.DictReader(f):
            assert int(r['byte_offset'])==offset
            end=offset+int(r['length']);msg=mav.MAVLink(None).parse_char(data[offset:end])
            assert msg.get_type()=='TIMESYNC' and (msg.get_srcSystem(),msg.get_srcComponent())==(245,191)
            assert message_target(msg)==(1,1)
            assert msg.tc1==int(r['tc1_ns']) and msg.ts1==int(r['ts1_ns'])
            count+=1;offset=end
    assert offset==len(data)
    events=[json.loads(l) for l in (folder/'clock_sync.jsonl').read_text().splitlines()]
    requests={r['ts1_ns'] for r in events if r.get('event')=='request_sent'}
    accepted=[r for r in events if r.get('accepted')]
    assert all(r['ts1_ns'] in requests for r in accepted)
    assert len({r['ts1_ns'] for r in accepted})==len(accepted)
    return {'status':'PASS','transmitted_frames':count,'bytes':len(data),'unique_matched_replies':len(accepted)}


evidence=ROOT/'evidence'
hashes=load(evidence/'evidence_hashes.json')
for name,expected in hashes.items():assert fingerprint(evidence/name)==expected,name
transport=load(evidence/'transport_pause.json')
results=[]
for kind in ['stationary','pause']:
    folders=list((evidence/kind).glob('*/manifest.json'));assert len(folders)==1
    folder=folders[0].parent;manifest=load(folder/'manifest.json');runtime=load(folder/'runtime_manifest.json')
    recording_report(folder)
    raw=load(folder/'audit.json');clock=load(folder/'timesync/summary.json')
    native_path=folder/runtime['native_file'];assert fingerprint(native_path)==runtime['native_fingerprint']
    native=native_audit(native_path,ROOT/'native_audits'/folder.name)
    tx=tx_check(folder)
    imu=native['topics']['sensor_combined:0'];att=native['topics']['vehicle_attitude:0']
    log=ULog(str(native_path));gap_events=[]
    for topic in ['sensor_combined','vehicle_attitude']:
        d=next(t.data for t in log.data_list if t.name==topic and t.multi_id==0)
        field='timestamp_sample' if 'timestamp_sample' in d else 'timestamp'
        ts=np.asarray(d[field],dtype=np.int64);delta=np.diff(ts);threshold=1.5*np.median(delta)+1
        for i in np.flatnonzero(delta>threshold):
            gap_events.append({'topic':topic,'source_time_field':field,'start_px4_us':int(ts[i]),
                'end_px4_us':int(ts[i+1]),'interval_ms':int(delta[i])/1000})
    checks={'raw_archives_intact':all(raw['checks'][n+'_archive']['status']=='PASS' for n in ['receiver','mavlink']),
        'no_recorder_queue_loss':all(s['dropped_bytes']==0 for s in manifest['streams'].values()),
        'complete_session':runtime['status']=='complete' and manifest['status']=='complete',
        'gnss_continuous_20hz':raw['checks']['receiver_epoch_continuity']['status']=='PASS',
        'addressed_unique_matched_replies':clock['addressing']=={'addressed':clock['accepted_replies']} and clock['accepted_replies']>0,
        'native_parsed':native['checks']['native_file_parsed'],
        'native_no_dropouts':not native['logged_dropouts'],
        'native_imu_continuous':imu['large_gaps']==0 and imu['backwards_timestamps']==0,
        'native_attitude_continuous':att['large_gaps']==0 and att['backwards_timestamps']==0}
    if kind=='stationary':
        checks.update(three_minute_capture=(clock['segments'][0]['duration_s']>=179 and runtime['recording_ready']),
                      single_clock_segment=len(clock['segments'])==1)
    else:
        start,end=transport['pause_start_host_ns'],transport['pause_end_host_ns']
        with (folder/'receiver_receive.csv').open() as f:
            during=[r for r in csv.DictReader(f) if start<=int(r['host_monotonic_ns'])<=end]
        receiver_during={'chunks':len(during),'bytes':sum(int(r['length']) for r in during),'pause_s':(end-start)/1e9}
        checks.update(explicit_transport_pause=transport['status']=='complete',new_clock_segment=len(clock['segments'])==2,
                      receiver_bytes_during_pause=receiver_during['bytes']>0,
                      segment_gap_event=any(e.get('reason')=='accepted_reply_gap' for e in clock['segment_events']))
    results.append({'kind':kind,'session':folder.name,'checks':checks,'clock':clock,'transmit_archive':tx,
        'rawx_epochs':raw['receiver']['epochs'],'native_imu_hz':imu['average_hz'],'native_attitude_hz':att['average_hz'],
        'native_imu_gaps':imu['large_gaps'],'native_attitude_gaps':att['large_gaps'],'native_dropouts':native['logged_dropouts'],
        'native_orientation_change_deg':native.get('max_orientation_change_deg'),'native_gap_events':gap_events,
        'receiver_during_pause':receiver_during if kind=='pause' else None,
        'mavlink_source_gaps':raw['telemetry'].get('source_gap_checks',{}),
        'native_imu_provenance':imu.get('provenance_and_reset_fields',{}),
        'native_attitude_provenance':att.get('provenance_and_reset_fields',{}),
        'archive_issues':manifest['issues'],'runtime_errors':runtime['errors']})
timing_checks=['raw_archives_intact','no_recorder_queue_loss','complete_session','gnss_continuous_20hz',
    'addressed_unique_matched_replies','three_minute_capture','single_clock_segment','explicit_transport_pause',
    'new_clock_segment','receiver_bytes_during_pause','segment_gap_event']
summary={'evidence_files_verified':len(hashes),'runs':results,'transport_test':transport,
    'all_stage5a_checks_pass':all(all(r['checks'].values()) for r in results),
    'stage5a_clock_and_receiver_checks_pass':all(v for r in results for k,v in r['checks'].items() if k in timing_checks),
    'additional_native_quality_review':'Pause-test ULog contains two ~10 ms sample intervals in both IMU and attitude. Zero logger dropouts. Preserve/exclude marked windows; do not label entire recording gap-free.',
    'pi_px4_mapping':'estimated_not_independently_verified','gnss_to_px4_clock':'UNKNOWN',
    'physical_usb_reconnect_tested':False,'existing_flight_recordings_modified':False,
    'manual_checkpoint':'Review Stage 5A before proceeding to 5B; no PPS wiring changes.'}
(ROOT/'review_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
cards=[]
for r in results:
    folder=evidence/r['kind']/r['session'];rel=str(folder.relative_to(ROOT))
    rows=''.join('<tr><td>'+escape(k.replace('_',' '))+'</td><td class="'+('pass' if v else 'review')+'">'+('PASS' if v else 'REVIEW')+'</td></tr>' for k,v in r['checks'].items())
    segment_rows=''.join('<tr>'+''.join('<td>'+escape(str(s.get(k,'—')))+'</td>' for k in ['segment_id','accepted_samples','duration_s','rtt_median_ms','rtt_p95_ms','drift_ppm','held_out_abs_residual_p95_ms'])+'</tr>' for s in r['clock']['segments'])
    cards.append(f'''<section><h2>{'Three-minute stationary test' if r['kind']=='stationary' else 'Controlled MAVLink pause / resume'}</h2><p><code>{r['session']}</code></p>
<p>{r['rawx_epochs']} RAWX epochs · native IMU {r['native_imu_hz']:.2f} Hz · attitude {r['native_attitude_hz']:.2f} Hz · native sample gaps {r['native_imu_gaps']} / {r['native_attitude_gaps']} · logger dropouts {len(r['native_dropouts'])}.</p>
<p>Maximum supplied-attitude change: {r['native_orientation_change_deg']:.3f} degrees. Native gap events are retained below; a zero logger-dropout count does not guarantee every sensor sample exists.</p><details><summary>Exact native sample intervals requiring review</summary><pre>{escape(json.dumps(r['native_gap_events'],indent=2))}</pre></details>
<p><a href="{rel}/timesync/report.html">Interactive timing plots and mapping</a> · <a href="{rel}/recording_report.html">Combined recording report</a> · <a href="{rel}/report.html">Raw GNSS/telemetry audit</a> · <a href="native_audits/{r['session']}/report.html">Native IMU/attitude plots</a></p>
<div class="scroll"><table><tr><th>Segment</th><th>Replies</th><th>Span s</th><th>Median RTT ms</th><th>p95 RTT ms</th><th>Drift ppm</th><th>Held-out p95 ms</th></tr>{segment_rows}</table></div><details><summary>Individual checks</summary><table>{rows}</table></details></section>''')
page='''<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Stage 5A timing review</title><style>body{font:15px/1.65 system-ui;max-width:1150px;margin:30px auto;padding:20px;background:#edf4f7;color:#213c4c}h1{font-size:38px}section{background:#fff;border:1px solid #dce5e9;border-radius:14px;padding:24px;margin:22px 0}.note{background:#fff1d3;padding:18px;border-radius:9px}a{color:#0b7292}td,th{padding:10px;border-bottom:1px solid #dde6e9;text-align:left}.pass{color:#15815d}.review{color:#b65f08}pre{white-space:pre-wrap;overflow-wrap:anywhere}.scroll{overflow-x:auto}</style></head><body><h1>Stage 5A · Pi ↔ PX4 time bridge</h1><p>5 November 2026 · Bench tests on the same Pi, Cube and X20 as the three recorded sessions.</p><p class="note">Clock bridge estimated; GNSS epoch alignment is still UNKNOWN. These new exchanges cannot retroactively synchronize the earlier flights. RTT/2 and held-out fit scatter are not independent proof of absolute timing accuracy.</p>CARDS
<section><h2>What the interruption test establishes</h2><p>A 1.5-second local UDP pause drops both directions without buffering. The receiver remains connected and continues recording. Requests abandoned at the clock boundary cannot be accepted in the next segment. The native logger continues independently. This tests transport pause/resume, not a physical USB disconnect or an autopilot reboot. The normal 2.5-second required-stream watchdog is unchanged.</p><pre>PAUSE</pre></section>
<section><h2>Scope and next manual checkpoint</h2><p>No aircraft control commands, router restart, persistent receiver write, PPS wiring or changes to the earlier flight data. Runtime receiver/native logging setup follows the existing tested launcher.</p><p>Review both timing plots, the visible interruption boundary and continuous GNSS/native samples. Stage 5B separately establishes GNSS week/TOW to the Pi/PX4 timebase and an independently checked timing error budget. Stop here for manual review before 5B.</p><p><a href="review_summary.json">Full check summary</a> · <a href="evidence/evidence_hashes.json">Original evidence fingerprints</a> · <a href="review_timesync.py">Rebuild script</a> · <a href="final_deployment_manifest.json">Final release source hashes</a></p></section></body></html>'''
page=page.replace('CARDS',''.join(cards)).replace('PAUSE',escape(json.dumps(transport,indent=2)))
(ROOT/'report.html').write_text(page)
print(json.dumps({'checks_pass':summary['all_stage5a_checks_pass'],'report':str(ROOT/'report.html'),'runs':[{'kind':r['kind'],'checks':r['checks'],'segments':r['clock']['segments']} for r in results]},indent=2))
