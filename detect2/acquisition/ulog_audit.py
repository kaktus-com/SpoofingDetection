"""Native ULog sample continuity and supplied-attitude audit (no integration)."""
import argparse
from html import escape
import json
from pathlib import Path

import numpy as np

from .common import atomic_json, fingerprint


def timing_summary(values):
    stamp = np.asarray(values, dtype=np.int64)
    delta = np.diff(stamp)
    positive = delta[delta > 0]
    median = float(np.median(positive)) if len(positive) else None
    return {'samples': len(stamp), 'distinct_timestamps': len(np.unique(stamp)),
            'duration_s': float((stamp[-1]-stamp[0])/1e6) if len(stamp)>1 else 0,
            'average_hz': (len(stamp)-1)*1e6/(stamp[-1]-stamp[0]) if len(stamp)>1 and stamp[-1]>stamp[0] else None,
            'median_interval_us': median, 'max_interval_us': int(delta.max()) if len(delta) else None,
            'duplicate_timestamps': int(np.count_nonzero(delta == 0)),
            'backwards_timestamps': int(np.count_nonzero(delta < 0)),
            'zero_timestamps': int(np.count_nonzero(stamp == 0)),
            'large_gaps': int(np.count_nonzero(delta > 1.5*median+1)) if median else None}


def topic_summary(data):
    # Use supplied measurement time when available. Keep the message timestamp
    # diagnostics too: publication jitter must not masquerade as sample loss.
    source = 'timestamp_sample' if 'timestamp_sample' in data else 'timestamp'
    fields = {}
    for name, values in data.items():
        if any(k in name for k in ['reset_counter','calibration_count','device_id','timestamp_sample','clipping']):
            a = np.asarray(values)
            fields[name] = {'first': a[0].item() if len(a) else None,
                            'last': a[-1].item() if len(a) else None,
                            'changes': int(np.count_nonzero(a[1:] != a[:-1]))}
    return {**timing_summary(data[source]),'timestamp_source':source,
            'publication_timing':timing_summary(data['timestamp']),
            'provenance_and_reset_fields': fields, 'field_names': list(data)}


def interval_points(stamp):
    if len(stamp)<2:return []
    delta=np.diff(stamp);positive=delta[delta>0]
    selected=set(range(1,len(stamp),max(1,len(stamp)//1600)))
    if len(positive):
        selected.update((np.flatnonzero((delta<=0)|(delta>1.5*np.median(positive)+1))+1).tolist())
    selected.add(int(np.argmax(delta))+1)
    return [[float((stamp[i]-stamp[0])/1e6),float(delta[i-1]/1000)] for i in sorted(selected)]


def audit(path, output):
    from pyulog import ULog
    path, output = Path(path), Path(output)
    output.mkdir(parents=True, exist_ok=True)
    log = ULog(str(path))
    topics = {f'{d.name}:{d.multi_id}': topic_summary(d.data) for d in log.data_list if 'timestamp' in d.data}
    imu = topics.get('sensor_combined:0')
    attitude = topics.get('vehicle_attitude:0')
    parameters = {k: log.initial_parameters.get(k) for k in ['SDLOG_PROFILE','SDLOG_MODE','SDLOG_BACKEND']}
    checks = {'native_file_parsed': not log.file_corruption,
              'logging_parameters': parameters == {'SDLOG_PROFILE':17,'SDLOG_MODE':2,'SDLOG_BACKEND':3},
              'native_imu_at_least_190_hz': bool(imu and imu['average_hz'] and imu['average_hz']>=190),
              'supplied_attitude_at_least_100_hz': bool(attitude and attitude['average_hz'] and attitude['average_hz']>=100),
              'no_logged_dropouts': len(log.dropouts)==0,
              'sensor_selection_present': 'sensor_selection:0' in topics,
              'imu_calibration_counters_present': bool(imu and all(k in imu['field_names'] for k in ['accel_calibration_count','gyro_calibration_count'])),
              'attitude_reset_fields_present': bool(attitude and all(k in attitude['field_names'] for k in ['quat_reset_counter','delta_q_reset[0]']))}
    for name, t in [('imu',imu),('attitude',attitude)]:
        checks[name+'_timestamp_continuity'] = bool(t and t['large_gaps']==0 and t['duplicate_timestamps']==0 and t['backwards_timestamps']==0 and t['zero_timestamps']==0)
    dropouts=[{'timestamp':d.timestamp,'duration_ms':d.duration} for d in log.dropouts]
    info={k:v for k,v in log.msg_info_dict.items() if k in ['ver_sw','ver_hw','sys_name','ver_sw_release','ver_os','sys_uuid','time_ref_utc']}
    result={'file':fingerprint(path),'filename':path.name,'checks':checks,'topics':topics,
            'parameters':parameters,'logged_dropouts':dropouts,'file_corruption':log.file_corruption,
            'firmware_info':info,'changed_parameters':log.changed_parameters,
            'all_native_logging_checks_pass':all(checks.values()),'gnss_to_px4_clock':'UNKNOWN',
            'controlled_motion_manually_confirmed':False,'ready_for_validated_detector':False}
    atomic_json(output/'ulog_audit.json',result)
    # Plot against ULog boot time only. No GNSS/host alignment is inferred.
    curves=[]
    for name in ['sensor_combined','vehicle_attitude']:
        dataset=next((d for d in log.data_list if d.name==name and d.multi_id==0),None)
        if dataset is None:continue
        d=dataset.data;source='timestamp_sample' if 'timestamp_sample' in d else 'timestamp';ts=np.asarray(d[source],dtype=np.int64)
        curves.append({'name':name+' measurement intervals ('+source+')','intervals':interval_points(ts)})
        if source!='timestamp':
            pub=np.asarray(d['timestamp'],dtype=np.int64)
            curves.append({'name':name+' publication intervals (timestamp)','intervals':interval_points(pub)})
    if attitude:
        d=next(d.data for d in log.data_list if d.name=='vehicle_attitude' and d.multi_id==0)
        q=np.column_stack([d[f'q[{i}]'] for i in range(4)]).astype(float)
        dot=np.clip(np.abs(np.sum(q*q[0],axis=1)),0,1)
        rotation=np.degrees(2*np.arccos(dot))
        ts=np.asarray(d.get('timestamp_sample',d['timestamp']),dtype=np.int64);step=max(1,len(ts)//1600)
        result['max_orientation_change_deg']=float(np.max(rotation))
        curves.append({'name':'orientation change from first supplied quaternion','intervals':[[float((ts[i]-ts[0])/1e6),float(rotation[i])] for i in range(0,len(ts),step)]})
        atomic_json(output/'ulog_audit.json',result)
    rows=''.join(f'<tr><td>{escape(k.replace("_"," "))}</td><td>{"PASS" if v else "REVIEW"}</td></tr>' for k,v in checks.items())
    topicrows=''.join(f'<tr><td>{escape(k)}</td><td>{v["samples"]}</td><td>{v["average_hz"]:.1f}</td><td>{v["max_interval_us"]}</td><td>{v["large_gaps"]}</td></tr>' for k,v in topics.items() if v['average_hz'] and any(s in k for s in ['sensor_combined','vehicle_attitude','vehicle_imu','sensor_selection','sensors_status_imu']))
    html='''<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Native PX4 ULog audit</title><style>body{font:15px/1.6 system-ui;color:#243b4a;background:#f0f5f7;max-width:1150px;margin:35px auto;padding:0 20px}section{background:white;border:1px solid #dae5ed;padding:24px;border-radius:12px;margin:20px 0}table{width:100%;border-collapse:collapse;font-size:13px}td,th{text-align:left;padding:8px;border-bottom:1px solid #e2e9ed}pre{white-space:pre-wrap;overflow-wrap:anywhere}canvas{width:100%;height:230px}select{padding:8px}a{color:#176f95}</style></head><body><h1>Native SD-card ULog</h1><p>__FILE__ · supplied PX4 attitude and native sensor data. No displacement integration or spoofing classification.</p><section><h2>Checks</h2><table>__CHECKS__</table></section><section><h2>Rates and continuity</h2><p>Measurement timing uses timestamp_sample when present; otherwise it uses timestamp. Original message/publication timing remains in the JSON and a separate plot. Large gaps exceed 1.5× the median positive interval plus 1 µs; a publication flag is not automatically a missing measurement.</p><table><tr><th>Topic</th><th>Samples</th><th>Average Hz</th><th>Max interval µs</th><th>Measurement gaps</th></tr>__TOPICS__</table><select id="series"></select><canvas id="plot"></canvas></section><section><h2>Provenance and limits</h2><p><a href="ulog_audit.json">Full topic fields, sensor IDs, reset/calibration counters, parameters, firmware and dropout details</a></p><p>Clock alignment with external GNSS remains UNKNOWN. Native IMU data and HIGHRES_IMU may differ in corrections; this report does not apply calibration or bias corrections. Orientation changes in the plot use supplied PX4 quaternions.</p><pre>__INFO__</pre></section><script>const curves=__CURVES__;const select=document.getElementById('series');curves.forEach((s,i)=>{const o=document.createElement('option');o.value=i;o.textContent=s.name;select.appendChild(o)});function draw(){const c=document.getElementById('plot'),w=c.getBoundingClientRect().width,h=230,d=devicePixelRatio||1;c.width=w*d;c.height=h*d;const ctx=c.getContext('2d');ctx.scale(d,d);const p=curves[+select.value].intervals,xmax=Math.max(...p.map(v=>v[0]),1),ymax=Math.max(...p.map(v=>v[1]),1)*1.1;ctx.font='11px system-ui';for(let i=0;i<=4;i++){let y=h-30-i/4*(h-50);ctx.fillStyle='#60798c';ctx.fillText((ymax*i/4).toFixed(2),3,y+4);ctx.fillText((xmax*i/4).toFixed(1),55+i/4*(w-75),h-8);ctx.strokeStyle='#e0e8ec';ctx.beginPath();ctx.moveTo(55,y);ctx.lineTo(w-15,y);ctx.stroke()}ctx.strokeStyle='#176f9d';ctx.beginPath();p.forEach((v,i)=>{const x=55+v[0]/xmax*(w-75),y=h-30-v[1]/ymax*(h-50);i?ctx.lineTo(x,y):ctx.moveTo(x,y)});ctx.stroke()}select.onchange=draw;window.addEventListener('resize',draw);if(curves.length)draw();</script></body></html>'''
    for key,value in {'__FILE__':escape(path.name),'__CHECKS__':rows,'__TOPICS__':topicrows,
                      '__INFO__':escape(json.dumps({'parameters':parameters,'firmware':info,'dropouts':dropouts},indent=2)),
                      '__CURVES__':json.dumps(curves).replace('<','\\u003c')}.items():html=html.replace(key,value)
    (output/'report.html').write_text(html)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('ulog',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();result=audit(a.ulog,a.output)
    print(json.dumps(result['checks'],indent=2));print('Report:',a.output/'report.html')


if __name__=='__main__':main()
