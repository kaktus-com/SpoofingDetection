"""Build the Stage 4 manual checkpoint from actual downloaded native data."""
from html import escape
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parents[2]))
from detect2.acquisition.common import fingerprint
from detect2.acquisition.audit_session import archive_index
from detect2.acquisition.ulog_audit import audit as audit_ulog

load=lambda p:json.loads(p.read_text())
latest=load(ROOT/'latest.json');release=ROOT/latest['release_id'];evidence=release/'evidence'
hashes=load(evidence/'evidence_hashes.json')
for name,expected in hashes.items():assert fingerprint(evidence/name)==expected,name
native=audit_ulog(evidence/'native.ulg',release/'native_audit')
session=next((evidence/'capture').iterdir());manifest=load(session/'manifest.json');raw=load(session/'audit.json')
for name in ['receiver','mavlink']:
    result,_=archive_index(session,name,manifest['files'][name+'.bin']);assert result['status']=='PASS',result
    assert fingerprint(session/(name+'_receive.csv'))==manifest['files'][name+'_receive.csv']
before=load(evidence/'px4_before.json');after=load(evidence/'px4_after.json')
system_before=load(evidence/'system_before.json');system_after=load(evidence/'system_after.json')
receiver_before=load(evidence/'receiver_before.json');receiver_after=load(evidence/'receiver_after.json')
confirmation=load(release/'manual_motion_confirmation.json') if (release/'manual_motion_confirmation.json').exists() else {'status':'awaiting user reply','confirmed_motion':False}
checks={'px4_identity_unchanged':before['identity']==after['identity'],
        'logging_parameter_readback':after['parameters']=={'SDLOG_PROFILE':17,'SDLOG_MODE':2,'SDLOG_BACKEND':3},
        'receiver_ram_and_identity_unchanged':receiver_before['ram']==receiver_after['ram'] and receiver_before['identity']==receiver_after['identity'],
        'legacy_scripts_unchanged':system_before['legacy_files']==system_after['legacy_files'],
        'router_process_unchanged':system_before['router_processes']==system_after['router_processes'],
        'raw_session_completed_without_drops':manifest['status']=='complete' and all(s['writer_closed'] and s['received_bytes']==s['written_bytes'] and s['dropped_bytes']==0 for s in manifest['streams'].values()),
        'rawx_continuity':raw['checks']['receiver_epoch_continuity']['status']=='PASS'}
summary={'stage':4,'checks':checks,'native_logging_checks':native['checks'],
         'manual_motion':confirmation,'max_native_orientation_change_deg':native.get('max_orientation_change_deg'),
         'all_native_logging_checks_pass':native['all_native_logging_checks_pass'],
         'logger_restart_scope':'logger module only','whole_autopilot_reboot_performed':False,
         'boot_logging_after_autopilot_restart':'UNKNOWN','receiver_profile_persisted':False,
         'gnss_to_px4_clock':'UNKNOWN','manual_checkpoint_4':'awaiting user review',
         'stage5_started':False,'logger_runtime_buffer_kib':128,'startup_buffer_kib':8,'startup_override_applied':False,'ready_for_validated_detector':False}
(release/'review_summary.json').write_text(json.dumps(summary,indent=2)+'\n')


def link(path,label=None):return f'<a href="{escape(str(path.relative_to(ROOT)))}">{escape(label or path.name)}</a>'


imu=native['topics'].get('sensor_combined:0',{});att=native['topics'].get('vehicle_attitude:0',{})
rows=''.join(f'<tr><td>{escape(k.replace("_"," "))}</td><td class="{str(v).lower()}">{"PASS" if v else "REVIEW"}</td></tr>' for k,v in {**checks,**native['checks']}.items())
files=' · '.join(link(evidence/n) for n in ['px4_before.json','px4_transaction.json','px4_after_apply.json','px4_after.json','native_logger.json','download.json','firmware.txt','logger_recording.txt','logger_after_recording.txt','logger_final_status.txt','receiver_before.json','receiver_after.json','system_before.json','system_after.json','logger_left_running_status.txt','runtime_final_state.json','startup_config_readonly_probe.txt','evidence_hashes.json'])
html='''<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>detect2 · Stage 4 native PX4 logging</title><style>:root{font:15px/1.65 system-ui,sans-serif;color:#233b4b;background:#eff5f7}body{max-width:1150px;margin:36px auto;padding:0 22px}h1{font-size:39px;line-height:1.2;letter-spacing:-1px}h2{font-size:23px}.card{padding:24px;background:white;border:1px solid #d9e5ec;border-radius:12px;margin:22px 0}.badge{font-size:12px;font-weight:700;background:#fff0cb;color:#835b1b;padding:6px 13px;border-radius:20px}.metrics{display:grid;grid-template-columns:repeat(3,1fr);gap:14px}.metric{background:white;border:1px solid #d9e5ec;padding:20px;border-radius:12px}.metric b{font-size:32px;display:block}.metric span{font-size:12px;color:#60798c}a{color:#176f94}table{width:100%;border-collapse:collapse;font-size:13px}th,td{text-align:left;padding:9px;border-bottom:1px solid #e1e9ed}.true{color:#187047;font-weight:700}.false{color:#a25d19;font-weight:700}pre{white-space:pre-wrap;overflow-wrap:anywhere;background:#f0f5f7;padding:18px;font-size:12px}.note{background:#fff7e8;border-left:3px solid #bb842d;padding:13px 18px}li{margin:10px 0}@media(max-width:650px){.metrics{grid-template-columns:1fr}}</style></head><body><h1>Stage 4: native PX4 logging.</h1><span class="badge">AWAITING MANUAL CHECKPOINT 4</span>
<p>The drone's actual native SD-card ULog was recorded and downloaded through the existing MAVLink router. Logging parameters changed from profile 1/mode 0 to profile 17/mode 2; backend 3 was retained. Identity and readback were checked. The logger module was restarted with mode/backend flags and a tested 128 KiB runtime buffer. The firmware startup default remains 8 KiB; the flight controller and router were not restarted.</p>
<div class="metrics"><div class="metric"><b>__IMU_HZ__ Hz</b><span>Actual native sensor_combined sample rate</span></div><div class="metric"><b>__ATT_HZ__ Hz</b><span>Actual supplied vehicle_attitude rate</span></div><div class="metric"><b>__DROPOUTS__</b><span>ULog-reported dropouts</span></div></div>
<section class="card"><h2>Manual checkpoint 4</h2><ol><li>Open the native ULog report and inspect actual IMU/attitude rates, unique timestamps and interval plots. Red REVIEW checks remain unresolved; requested rates alone are not a pass.</li><li>Inspect the orientation-change plot and confirm it corresponds to your manual movement, with stationary data before and after.</li><li>Confirm profile 17, mode 2 and backend 3 in both parameter readback and the ULog's own initial parameters.</li><li>Review sensor-selection IDs, accelerometer/gyro calibration counters, quaternion reset information and ULog dropouts in the native audit JSON.</li><li>Review the restart scope: only the logger module was restarted. Actual automatic logging after a whole-autopilot reboot remains UNKNOWN and needs a controlled boot check.</li></ol><p class="note">External GNSS↔PX4 timing remains UNKNOWN. The X20 profile remains RAM-only at 2 Mbaud. PX4 logging parameters have readback, but the 128 KiB buffer is runtime-only. Its startup override and actual log-from-boot behavior remain unverified. This stage does not validate a spoofing decision.</p></section>
<section class="card"><h2>Open the measured results</h2><p>__REPORT_LINKS__</p><p>Native sensor_combined: __IMU_SAMPLES__ samples, __IMU_GAPS__ intervals above the conservative gap rule. Supplied attitude: __ATT_SAMPLES__ samples, __ATT_GAPS__ intervals above the gap rule. Measurement timing uses timestamp_sample when present. Publication-time jitter is retained separately in the native audit and must not be mistaken for missing supplied attitude samples.</p><p>Manual movement: __CONFIRMATION__. Maximum orientation change from the first supplied quaternion: __ROTATION__°. The movement cue was issued using host time; no GNSS/ULog alignment is inferred.</p><p>Raw GNSS and MAVLink recording ran alongside the native logger for 90 seconds. All raw observations remain archived, including invalid quality flags. Native acceleration is not replaced with a GNSS/PX4 position solution, and no attitude is derived here.</p></section>
<section class="card"><h2>Automated checks</h2><table>__CHECKS__</table></section>
<section class="card"><h2>What the repeat fixed, and what still needs review</h2><p>The first native log, with an 8 KiB buffer, contains 85 dropout records totaling 17.839 seconds. Its simultaneous raw recording logged a real FTDI disconnect/reconnect; the receiver restarted at 921600 and its old 25 Hz profile. Those artifacts are preserved below. After you secured the connections, the approved 20 Hz/2 Mbaud RAM settings were reapplied with readback.</p><p>The repeat used a memory-checked 128 KiB logger buffer: zero ULog dropout records, maximum reported buffer use about 38 KiB, and a clean simultaneous raw capture. Both tests retain the real measurements; they have different durations and are not a controlled SD-card benchmark.</p><p>The final native IMU rate is about 189.55 Hz, below the audit's strict 190 Hz cutoff. No IMU intervals exceed its gap rule. Supplied attitude measurement time (timestamp_sample) is continuous, with zero gaps, duplicates or backwards timestamps. Its message/publication timestamp has two intervals just above 1.5× median (maximum 7.948 ms). Both diagnostics remain visible; publication jitter is not classified as lost measurement samples. The native IMU rate check remains REVIEW.</p><p>__PREVIOUS_LINKS__</p></section><section class="card"><h2>Configuration and provenance</h2><p>The firmware reports custom branch <code>more-configts</code>, hash <code>cb411db5b54a98442ab974292c81b74c057bdf25</code>. The local PX4 source matches that hash; its logger files are unchanged. Profile 17 combines default topics and bit 4 full-rate topics. The native ULog retains sensor_combined, supplied attitude, sensor selection/status and available reset/calibration provenance. Native IMU and MAVLink HIGHRES_IMU can differ in corrections, so neither is corrected twice by this report.</p><p>__FILES__</p><p>__ARTIFACTS__</p><p>Protocol references: <a href="https://docs.px4.io/main/en/dev_log/logging">PX4 logging</a> · <a href="https://mavlink.io/en/services/parameter.html">MAVLink parameter encoding/readback</a>.</p></section>
<section class="card"><h2>Concrete startup-buffer proposal — not applied</h2><p>The read-only NSH check confirms there is no existing <code>/fs/microsd/etc/config.txt</code>. The matching rcS source sets LOGGER_BUF=8, then sources that user configuration before starting the logger. The proposed new file contains:</p><pre># detect2: tested native logger buffer, KiB
set LOGGER_BUF 128</pre><p>__PROPOSAL_LINKS__</p><p>Review this small startup change before installation and a controlled reboot. The runtime test validates 128 KiB for this bench session; it does not validate startup allocation or automatic logging after a reboot.</p></section><section class="card"><h2>Rollback and remaining checks</h2><pre>ssh nss@10.15.4.141
cd /home/nss/SpoofingDetection/releases/acquisition_20261105T110249Z
# Restore saved PX4 logging profile/mode and restart the logger in original mode:
/home/nss/venv/bin/python run_stage4.py restore</pre><p>The tested ULog was closed and downloaded. The raw recorder is stopped. Native SD logging has been restarted and left running with profile 17/mode 2 and the 128 KiB runtime buffer. Do not treat the unchanged receiver RAM profile as persistent startup configuration. A later controlled full-autopilot restart must verify log-from-boot behavior; a receiver restart is a separate operation.</p><p><a href="../../IMPLEMENTATION_PLAN.md">Stage plan</a> · <a href="../acquisition_stage3/report.html">Approved Stage 3 receiver checks</a></p></section></body></html>'''
replacements={'__IMU_HZ__':f'{imu.get("average_hz",0):.1f}','__ATT_HZ__':f'{att.get("average_hz",0):.1f}',
    '__DROPOUTS__':str(len(native['logged_dropouts'])),'__IMU_SAMPLES__':str(imu.get('samples')),
    '__ATT_SAMPLES__':str(att.get('samples')),'__IMU_GAPS__':str(imu.get('large_gaps')),'__ATT_GAPS__':str(att.get('large_gaps')),
    '__ROTATION__':f'{native.get("max_orientation_change_deg",0):.1f}','__CONFIRMATION__':escape(confirmation['status']),
    '__CHECKS__':rows,'__FILES__':files,'__REPORT_LINKS__':' · '.join([link(release/'native_audit/report.html','Native ULog rates, intervals and orientation plots'),link(session/'report.html','Simultaneous raw GNSS/MAVLink audit'),link(evidence/'native.ulg','Original SD-card ULog')]),
    '__PREVIOUS_LINKS__': ' · '.join(link(p) for p in [(ROOT/'acquisition_20261105T105214Z/native_audit/report.html'),(ROOT/'acquisition_20261105T105214Z/previous_evidence/failed_raw_capture/report.html'),(ROOT/'acquisition_20261105T105214Z/previous_evidence/kernel_usb_log.txt'),(ROOT/'acquisition_20261105T105214Z/previous_evidence/receiver_reapply_transaction.json')]),
    '__PROPOSAL_LINKS__': ' · '.join(link(p) for p in [release/'proposed_logger_config.txt',release/'startup_override_proposal.json',evidence/'startup_config_readonly_probe.txt']),
    '__ARTIFACTS__':' · '.join(link(p) for p in [release/'review_summary.json',release/'local_verification.json',release/'local_audit_source.py',release/'release_manifest.json',release/'test_result.txt',release/'run_stage4.py'])}
for key,value in replacements.items():html=html.replace(key,value)
(ROOT/'report.html').write_text(html)
print('NATIVE CHECKS',json.dumps(native['checks'],indent=2))
print('CONTROL CHECKS',json.dumps(checks,indent=2))
print('Report:',ROOT/'report.html')
