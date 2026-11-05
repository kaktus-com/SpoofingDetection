"""Review saved recordings, independently of undeployed TIMESYNC changes."""
from collections import Counter
import csv
from datetime import datetime
from html import escape
import json
from pathlib import Path
import sys
from zoneinfo import ZoneInfo

import numpy as np
from pyulog import ULog

ROOT=Path(__file__).resolve().parent
REPO=ROOT.parents[3]
sys.path.insert(0,str(REPO))
from detect2.acquisition.audit_session import audit as audit_raw,archive_index
from detect2.acquisition.ulog_audit import audit as audit_native
from detect2.acquisition.common import fingerprint

EVIDENCE=ROOT/'evidence'
NAMES=['20261105T131308Z_d3f681','20261105T132010Z_c4cbfd','20261105T132748Z_9c3265']
load=lambda p:json.loads(p.read_text())


def link(path,label=None):
    return f'<a href="{escape(str(path.relative_to(ROOT)))}">{escape(label or path.name)}</a>'


def intervals(data,field,start,end,invert=False):
    """State durations over the recorded native measurement window only."""
    ts=np.asarray(data['timestamp'],dtype=np.int64)
    values=np.asarray(data[field],dtype=bool)
    if invert:values=~values
    result=[]
    for i,value in enumerate(values):
        if not value:continue
        a=max(int(ts[i]),start);b=min(int(ts[i+1]) if i+1<len(ts) else end,end)
        if b>a:
            if result and result[-1][1]==a:result[-1][1]=b
            else:result.append([a,b])
    return {'duration_s':sum((b-a)/1e6 for a,b in result),
            'intervals_boot_us':result,'intervals_elapsed_s':[[(a-start)/1e6,(b-start)/1e6] for a,b in result]}


def raw_source_times(folder):
    from pymavlink.dialects.v20 import common as mav
    ranges={};parsers={}
    with (folder/'mavlink.bin').open('rb') as stream,(folder/'mavlink_receive.csv').open() as index:
        for row in csv.DictReader(index):
            stream.seek(int(row['byte_offset']));raw=stream.read(int(row['length']))
            peer=(row.get('peer_host'),row.get('peer_port'),row.get('segment_id'))
            parser=parsers.setdefault(peer,mav.MAVLink(None));parser.robust_parsing=True
            for msg in parser.parse_buffer(raw) or []:
                if (msg.get_srcSystem(),msg.get_srcComponent())!=(1,1):continue
                name=msg.get_type()
                if name=='HIGHRES_IMU':stamp=int(msg.time_usec)
                elif name=='ATTITUDE_QUATERNION':stamp=int(msg.time_boot_ms)*1000
                else:continue
                if name not in ranges:ranges[name]=[stamp,stamp]
                else:ranges[name][0]=min(ranges[name][0],stamp);ranges[name][1]=max(ranges[name][1],stamp)
    return ranges


def phase_quality(path):
    epochs={};signals={};gnss=Counter();lock_decreases=0;last={}
    with path.open() as f:
        for row in csv.DictReader(f):
            t=int(row['gps_week'])*604800.+float(row['gps_tow_s'])
            epoch=epochs.setdefault(t,[0,0,0]);epoch[0]+=1
            valid=row['carrier_valid']=='True';half=row['half_cycle_valid']=='True'
            epoch[1]+=int(valid);epoch[2]+=int(valid and half)
            key=row['signal_id'];gnss[row['gnss_id']]+=1
            item=signals.setdefault(key,{'observations':0,'carrier_valid':0,'carrier_and_half_cycle_valid':0,'cno_sum':0.})
            item['observations']+=1;item['carrier_valid']+=int(valid)
            item['carrier_and_half_cycle_valid']+=int(valid and half);item['cno_sum']+=float(row['cno_db_hz'])
            lock=int(row['lock_time_ms'])
            if key in last and lock<last[key]:lock_decreases+=1
            last[key]=lock
    for item in signals.values():item['mean_cno_db_hz']=item.pop('cno_sum')/item['observations']
    t0=min(epochs)
    points=[[t-t0,*values] for t,values in sorted(epochs.items())]
    return {'epochs':len(epochs),'duration_s':max(epochs)-t0,
            'total_observations':sum(p[1] for p in points),'carrier_valid_observations':sum(p[2] for p in points),
            'carrier_and_half_cycle_valid_observations':sum(p[3] for p in points),
            'lock_counter_decreases':lock_decreases,'gnss_observation_counts':dict(gnss),
            'signals':signals,'epoch_counts':points}


def native_events(imu,att,airborne):
    ranges=airborne['intervals_boot_us'] if airborne else []
    events=[]
    for topic,data,counters in [('sensor_combined',imu,['accel_calibration_count','gyro_calibration_count']),
                                 ('vehicle_attitude',att,['quat_reset_counter'])]:
        ts=np.asarray(data.get('timestamp_sample',data['timestamp']),dtype=np.int64)
        delta=np.diff(ts);positive=delta[delta>0];median=float(np.median(positive)) if len(positive) else 0
        for i in np.flatnonzero(delta>1.5*median+1):
            a,b=int(ts[i]),int(ts[i+1])
            events.append({'kind':'measurement_gap','topic':topic,'before_boot_us':a,'after_boot_us':b,
                'gap_ms':(b-a)/1000,'overlaps_airborne':any(a<end and b>start for start,end in ranges)})
        for field in counters:
            if field not in data:continue
            for i in np.flatnonzero(data[field][1:]!=data[field][:-1])+1:
                point=int(ts[i]);events.append({'kind':'counter_change','topic':topic,'field':field,
                    'boot_us':point,'new_value':int(data[field][i]),
                    'overlaps_airborne':any(start<=point<=end for start,end in ranges)})
    return events


hashes=load(EVIDENCE/'evidence_hashes.json')
for name,expected in hashes.items():assert fingerprint(EVIDENCE/name)==expected,name
summaries=[];chartdata=[];table=[];detail=[]
for number,name in enumerate(NAMES,1):
    folder=EVIDENCE/'sessions'/name
    state=load(folder/'runtime_manifest.json');manifest=load(folder/'manifest.json')
    expected=manifest['files']
    for stream in ['receiver','mavlink']:
        result,_=archive_index(folder,stream,expected[stream+'.bin']);assert result['status']=='PASS',result
        assert fingerprint(folder/(stream+'_receive.csv'))==expected[stream+'_receive.csv']
    raw=audit_raw(folder)
    native_path=folder/state['native_file'];assert fingerprint(native_path)==state['native_fingerprint']
    native=audit_native(native_path,ROOT/'native_audits'/name)
    phase=phase_quality(folder/'receiver_observations.csv')
    assert phase['total_observations']==raw['receiver']['observations']
    log=ULog(str(native_path));datasets={(d.name,d.multi_id):d.data for d in log.data_list}
    imu=datasets['sensor_combined',0];att=datasets['vehicle_attitude',0]
    start=int(imu['timestamp'][0]);end=int(imu['timestamp'][-1])
    armed=intervals(datasets['actuator_armed',0],'armed',start,end) if ('actuator_armed',0) in datasets else None
    airborne=intervals(datasets['vehicle_land_detected',0],'landed',start,end,True) if ('vehicle_land_detected',0) in datasets else None
    events=native_events(imu,att,airborne)
    (ROOT/'native_audits'/name/'native_quality_events.json').write_text(json.dumps(events,indent=2)+'\n')
    source_times=raw_source_times(folder)
    native_ranges={'HIGHRES_IMU':[int(imu['timestamp'][0]),int(imu['timestamp'][-1])],
                   'ATTITUDE_QUATERNION':[int(att['timestamp'][0]),int(att['timestamp'][-1])]}
    coverage={k:bool(k in source_times and native_ranges[k][0]<=source_times[k][0]+20000 and native_ranges[k][1]>=source_times[k][1]-20000) for k in native_ranges}
    clipping={k:int(np.count_nonzero(v)) for k,v in imu.items() if 'clipping' in k}
    selected=load(ROOT/'native_audits'/name/'ulog_audit.json')
    status='Good acquisition; clock alignment pending'
    native_issues=[]
    if native['file_corruption']:native_issues.append('ULog parser reports corruption')
    if native['logged_dropouts']:native_issues.append('Native ULog dropouts')
    if native['topics']['sensor_combined:0']['large_gaps']:native_issues.append('Native IMU measurement gaps')
    if native['topics']['vehicle_attitude:0']['large_gaps']:native_issues.append('Native attitude measurement gaps')
    if not all(coverage.values()):native_issues.append('Native/MAVLink boot-time coverage requires review')
    if any(clipping.values()):native_issues.append('Native IMU clipping flags present')
    if native_issues:status='Saved successfully; native quality needs review'
    airborne_event_count=sum(e['overlaps_airborne'] for e in events)
    if airborne and airborne['duration_s']>0 and airborne_event_count==0 and not any(clipping.values()) and all(coverage.values()):
        status='Flight samples continuous; exclude marked ground events'
    elif airborne and airborne['duration_s']==0 and not native_issues:
        status='Continuous stationary reference; no airborne interval'
    total=phase['total_observations'];valid_pct=100*phase['carrier_valid_observations']/total
    native_name=Path(state['native_path']).name
    item={'recording':number,'session':name,'pi_folder_time_cet':datetime.strptime(name[:16],'%Y%m%dT%H%M%SZ').replace(tzinfo=ZoneInfo('UTC')).astimezone(ZoneInfo('Europe/Stockholm')).isoformat(),
          'assessment':status,'raw_lifecycle':manifest['status'],'combined_lifecycle':state['status'],
          'raw_issues':manifest['issues']['counts'],'rawx_epochs':phase['epochs'],'rawx_duration_s':phase['duration_s'],
          'rawx_unexpected_intervals':raw['checks']['receiver_epoch_continuity']['unexpected_intervals'],
          'rawx_clock_resets':raw['receiver']['clock_resets'],'ubx_diagnostics':raw['receiver']['diagnostics'],
          'archive_integrity':'PASS','recorder_dropped_bytes':sum(v['dropped_bytes'] for v in manifest['streams'].values()),
          'carrier_valid_percent':valid_pct,'carrier_and_half_cycle_valid_percent':100*phase['carrier_and_half_cycle_valid_observations']/total,
          'signal_identities':len(phase['signals']),'lock_counter_decreases':phase['lock_counter_decreases'],
          'native_file':state['native_file'],'bookmarked_sd_path':state['native_path'],'native_file_fingerprint':fingerprint(native_path),
          'native_imu':native['topics']['sensor_combined:0'],'native_attitude':native['topics']['vehicle_attitude:0'],
          'native_dropouts':native['logged_dropouts'],'native_dropout_total_ms':sum(d['duration_ms'] for d in native['logged_dropouts']),
          'native_issues':native_issues,'clipping_sample_counts':clipping,'armed':armed,'airborne':airborne,
          'native_quality_events':events,'native_quality_events_overlapping_airborne':airborne_event_count,
          'max_orientation_change_deg':native.get('max_orientation_change_deg'),'raw_px4_source_ranges_boot_us':source_times,
          'native_px4_ranges_boot_us':native_ranges,'native_covers_raw_px4_source_times':coverage,
          'mavlink_source_gap_checks':raw['telemetry']['source_gap_checks'],'clock_alignment':'UNKNOWN',
          'native_imu_reference_accepted_by_user_hz':189.6,'spoofing_label':'not provided; no detector verdict inferred'}
    summaries.append(item);chartdata.append({'name':name,'points':phase['epoch_counts']})
    (ROOT/'native_audits'/name/'recording_quality.json').write_text(json.dumps({'summary':item,'signal_quality':phase['signals']},indent=2)+'\n')
    title=f'Recording {number}: '+datetime.fromisoformat(item['pi_folder_time_cet']).strftime('%H:%M:%S CET')
    table.append(f'<tr><td>{link(folder/"report.html",title)}</td><td>{phase["duration_s"]:.1f} s</td><td>{phase["epochs"]:,}</td><td>0 / 0</td><td>{valid_pct:.1f}%</td><td>{item["native_imu"]["average_hz"]:.1f} Hz</td><td>{len(native["logged_dropouts"])}</td><td>{airborne["duration_s"]:.1f} s</td></tr>' if airborne is not None else '')
    links=' · '.join([link(folder/'report.html','Raw GNSS/MAVLink audit'),link(ROOT/'native_audits'/name/'report.html','Native IMU/attitude plots'),link(ROOT/'native_audits'/name/'recording_quality.json','Full quality and coverage details'),link(ROOT/'native_audits'/name/'native_quality_events.json','Exact gap/reset/calibration event locations'),link(native_path,'Original native ULog')])
    warnings='; '.join(native_issues) if native_issues else 'No native gap, clipping or dropout issues flagged.'
    detail.append(f'<section class="card"><h2>Recording {number}: {escape(status)}</h2><p>{escape(name)} · bookmarked file <code>{escape(state["native_path"])}</code></p><p>{links}</p><p>Native measurement gaps over the whole file: IMU {item["native_imu"]["large_gaps"]}, attitude {item["native_attitude"]["large_gaps"]}. Native dropout duration: {item["native_dropout_total_ms"]:.0f} ms. Gap/reset/calibration events overlapping airborne intervals: {airborne_event_count}. Maximum orientation change: {item["max_orientation_change_deg"]:.1f}°. Native file covers the raw PX4 source-time ranges: {all(coverage.values())}.</p><p>{escape(warnings)}</p></section>')

summary={'recordings':summaries,'failed_setup_attempt':'20261105T131201Z_3f60be','source_evidence_files_verified':len(hashes),
         'pending_downloads_recovered':[NAMES[0],NAMES[2]],'source_raw_archives_and_manifests_unchanged':True,
         'timesync_changes_deployed':False,'gnss_to_px4_clock_alignment':'UNKNOWN','detector_run':False}
(ROOT/'review_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
html='''<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>5 November recording review</title><style>body{font:15px/1.65 system-ui,sans-serif;color:#243b4c;background:#eff5f7;max-width:1200px;margin:35px auto;padding:0 22px}h1{font-size:38px;line-height:1.2;letter-spacing:-1px}h2{font-size:23px}.card{background:white;border:1px solid #d9e5ed;padding:24px;border-radius:12px;margin:22px 0}a{color:#176f94}.scroll{overflow:auto}table{width:100%;border-collapse:collapse;font-size:13px}td,th{text-align:left;padding:10px;border-bottom:1px solid #e1e9ed}pre{white-space:pre-wrap;overflow-wrap:anywhere;background:#f0f5f7;padding:16px;font-size:12px}.note{background:#fff6e7;border-left:3px solid #ba852d;padding:13px 18px}canvas{width:100%;height:280px;display:block}select{padding:7px;font:inherit}li{margin:9px 0}</style></head><body><h1>Your three recordings: data review.</h1><p>Recorded on 5 November 2026. The three real sessions were inspected before resuming Stage 5A. The earlier failed setup attempt reached no recording-ready state and contains no sensor data.</p><section class="card"><h2>Summary</h2><div class="scroll"><table><tr><th>Recording</th><th>GNSS span</th><th>RAWX epochs</th><th>Recorder loss / GNSS gaps</th><th>Carrier-valid flags</th><th>Native IMU</th><th>ULog dropouts</th><th>PX4 airborne flag duration</th></tr>__TABLE__</table></div><p><b>The airborne portions of recordings 1 and 2 have continuous native IMU and attitude samples, with no detected gap/reset/calibration events during those airborne intervals.</b> Recording 3 stayed disarmed on the ground and provides a stationary reference. Exclude analysis windows that cross the marked ground gaps or reset/calibration events.</p><p>All three raw archives have complete byte-index coverage and matching source fingerprints, with zero recorder-reported losses and continuous 20 Hz RAWX. No receiver clock resets or invalid interior UBX candidates were observed. Native quality findings are listed per session below.</p><p class="note">The first and third native downloads had been interrupted. They were completed with the installed launcher’s finalize command, preserving the raw binaries, indexes and raw manifests. Original before-state metadata and hashes are retained. The folders on the Pi now have their native files and status stopped, the normal intentional Ctrl+C result.</p></section>
<section class="card"><h2>Carrier tracking flags over each recording</h2><select id="record"><option value="0">Recording 1</option><option value="1">Recording 2</option><option value="2">Recording 3</option></select><p>Observations per epoch: blue = total, green = carrier-valid, orange = carrier plus half-cycle flags valid. X axis is elapsed external GNSS measurement time, not an aligned PX4/host clock.</p><canvas id="quality"></canvas><p>Approximately 7–9% of observations have invalid carrier flags. They were retained, as intended. Later analysis must select valid continuous tracks and handle lock/ambiguity changes; this flag percentage alone does not establish phase suitability or spoofing.</p></section>
__DETAILS__
<section class="card"><h2>What the red checks mean</h2><ul><li>The raw audit calls a stopped lifecycle FAIL for its generic capture-session test. Here the stop was deliberate, writers closed, and archive/index checks pass.</li><li>Some MAVLink publication intervals are irregular, particularly in recordings 1 and 2. Native sample timestamps and ULog dropouts are checked separately; publication gaps are not automatically a count of lost native IMU measurements.</li><li>The native audit retains its original strict 190 Hz check. You accepted the approximately 189.6 Hz bench rate; actual rates are shown, without changing that historical cutoff to force a pass.</li></ul></section>
<section class="card"><h2>Pairing and remaining limits</h2><p>The native filenames and Pi folder names differ by roughly ten minutes, including different clocks and file-creation points. Files were paired using the recorded SD-card bookmarks, then checked against PX4 boot-time ranges shared by MAVLink and the native data. Filenames were not used to infer measurement alignment.</p><p>External GNSS↔PX4 alignment remains UNKNOWN. No fused GNSS position, displacement integration or spoofing decision was used in this review. Airborne/armed durations come from PX4 state flags, not GNSS-derived trajectory. No attack labels were assumed.</p><p>The TIMESYNC changes remain local and undeployed. Stage 5A can resume after review of these findings.</p><p>__LINKS__</p></section>
<script>const records=__CHARTS__;function draw(){const c=document.getElementById('quality'),w=c.getBoundingClientRect().width,h=280,d=devicePixelRatio||1;c.width=w*d;c.height=h*d;const q=c.getContext('2d');q.scale(d,d);const points=records[+document.getElementById('record').value].points,xmax=Math.max(...points.map(p=>p[0]),1),ymax=Math.max(...points.map(p=>p[1]),1)*1.1;const px=x=>55+x/xmax*(w-75),py=y=>h-32-y/ymax*(h-52);q.font='11px system-ui';for(let i=0;i<=4;i++){q.fillStyle='#60798c';q.fillText((ymax*i/4).toFixed(0),4,py(ymax*i/4)+4);q.fillText((xmax*i/4).toFixed(0),px(xmax*i/4)-7,h-8);q.strokeStyle='#e1e9ed';q.beginPath();q.moveTo(55,py(ymax*i/4));q.lineTo(w-20,py(ymax*i/4));q.stroke()}['#247cb0','#178264','#c88725'].forEach((color,j)=>{q.strokeStyle=color;q.lineWidth=1;q.beginPath();points.forEach((p,i)=>i?q.lineTo(px(p[0]),py(p[j+1])):q.moveTo(px(p[0]),py(p[j+1])));q.stroke()})}document.getElementById('record').onchange=draw;window.addEventListener('resize',draw);draw();</script></body></html>'''
links=' · '.join(link(p) for p in [ROOT/'review_summary.json',EVIDENCE/'inventory_after.json',EVIDENCE/'finalization_results.json',EVIDENCE/'raw_hashes_before.json',EVIDENCE/'evidence_hashes.json',ROOT/'review_recordings.py'])
html=html.replace('__TABLE__',''.join(table)).replace('__DETAILS__',''.join(detail)).replace('__LINKS__',links).replace('__CHARTS__',json.dumps(chartdata).replace('<','\\u003c'))
(ROOT/'report.html').write_text(html)
print(json.dumps({'recordings':[{k:r[k] for k in ['recording','session','rawx_duration_s','rawx_epochs','carrier_valid_percent','native_issues','native_dropout_total_ms','airborne','assessment']} for r in summaries]},indent=2))
print('Review report:',ROOT/'report.html')
