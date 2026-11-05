"""Offline Pi/PX4 clock bridge; source timestamps are never rewritten.

Fits use low-RTT samples and a chronological holdout. Neither their residuals
nor RTT symmetry independently establish absolute accuracy or GNSS timing.
"""
import argparse
from collections import Counter
import csv
from html import escape
import json
from pathlib import Path
import statistics


def percentile(values, p):
    values=sorted(values)
    if not values:return None
    at=(len(values)-1)*p;low=int(at);high=min(low+1,len(values)-1)
    return values[low]+(values[high]-values[low])*(at-low)


def fit_line(points):
    xbar=statistics.mean(x for x,y in points);ybar=statistics.mean(y for x,y in points)
    denominator=sum((x-xbar)**2 for x,y in points)
    slope=sum((x-xbar)*(y-ybar) for x,y in points)/denominator if denominator else 0.
    return ybar-slope*xbar,slope


def map_host_to_px4(summary, host_ns, segment_id):
    """Evaluate an estimated bridge inside one observed segment only."""
    segment=next((s for s in summary['segments'] if s['segment_id']==segment_id),None)
    if segment is None or segment['mapping_status']!='estimated_not_independently_verified':
        raise ValueError('No usable clock estimate for this segment')
    if not segment['valid_host_min_ns']<=host_ns<=segment['valid_host_max_ns']:
        raise ValueError('Clock extrapolation across an unobserved interval is forbidden')
    return round(host_ns+segment['offset_at_origin_ns']+
        segment['offset_slope_ns_per_s']*((host_ns-segment['origin_host_ns'])/1e9))


def summarize_segment(segment, rows):
    origin=rows[0]['host_midpoint_ns']
    bins={}
    for row in rows:
        key=(row['host_midpoint_ns']-origin)//1_000_000_000
        if key not in bins or row['round_trip_ns']<bins[key]['round_trip_ns']:bins[key]=row
    selected=[bins[k] for k in sorted(bins)]
    result={'segment_id':segment,'accepted_samples':len(rows),'selected_low_rtt_bins':len(selected),
        'valid_host_min_ns':rows[0]['host_midpoint_ns'],'valid_host_max_ns':rows[-1]['host_midpoint_ns'],
        'duration_s':(rows[-1]['host_midpoint_ns']-origin)/1e9,
        'rtt_median_ms':statistics.median(r['round_trip_ns'] for r in rows)/1e6,
        'rtt_p95_ms':percentile([r['round_trip_ns']/1e6 for r in rows],.95),
        'rtt_max_ms':max(r['round_trip_ns'] for r in rows)/1e6,
        'mapping_status':'insufficient_samples','independent_accuracy_verified':False}
    if len(selected)<6:return result,[]
    split=max(3,int(len(selected)*2/3));train=selected[:split];held=selected[split:]
    points=[((r['host_midpoint_ns']-origin)/1e9,r['remote_minus_host_ns']) for r in train]
    intercept,slope=fit_line(points)
    residuals=[y-(intercept+slope*x) for x,y in points]
    middle=statistics.median(residuals);mad=statistics.median(abs(r-middle) for r in residuals)
    threshold=max(1000.,6*1.4826*mad)
    keep=[abs(r-middle)<=threshold for r in residuals]
    if sum(keep)>=3:intercept,slope=fit_line([p for p,k in zip(points,keep) if k])
    diagnostics=[]
    selected_ids={r['ts1_ns']:i for i,r in enumerate(selected)}
    for r in rows:
        index=selected_ids.get(r['ts1_ns'])
        role='unselected' if index is None else 'held_out' if index>=split else 'train' if keep[index] else 'train_outlier'
        residual=r['remote_minus_host_ns']-intercept-slope*((r['host_midpoint_ns']-origin)/1e9)
        diagnostics.append({'segment_id':segment,'host_midpoint_ns':r['host_midpoint_ns'],
            'elapsed_s':(r['host_midpoint_ns']-origin)/1e9,'role':role,'residual_ns':residual,
            'round_trip_ns':r['round_trip_ns'],'ts1_ns':r['ts1_ns']})
    held_residual=[abs(d['residual_ns'])/1e6 for d in diagnostics if d['role']=='held_out']
    result.update(mapping_status='estimated_not_independently_verified',origin_host_ns=origin,
        offset_at_origin_ns=intercept,offset_slope_ns_per_s=slope,drift_ppm=slope/1000,
        training_bins=len(train),training_outlier_bins=keep.count(False),held_out_bins=len(held),
        held_out_abs_residual_p95_ms=percentile(held_residual,.95),
        held_out_abs_residual_max_ms=max(held_residual),robust_training_threshold_ns=threshold,
        formula='PX4_ns = host_ns + offset_at_origin_ns + offset_slope_ns_per_s * ((host_ns - origin_host_ns) / 1e9)',
        extrapolation_allowed=False)
    return result,diagnostics


def report(folder):
    folder=Path(folder);rows=[json.loads(l) for l in (folder/'clock_sync.jsonl').read_text().splitlines()]
    accepted=[r for r in rows if r.get('event')=='reply' and r.get('accepted')]
    rejected=[r for r in rows if r.get('event')=='reply' and not r.get('accepted')]
    segments=[];diagnostics=[]
    for sid in sorted({r['segment_id'] for r in accepted}):
        summary,points=summarize_segment(sid,[r for r in accepted if r['segment_id']==sid])
        segments.append(summary);diagnostics+=points
    out=folder/'timesync';out.mkdir(exist_ok=True)
    requests=sum(r.get('event')=='request_sent' for r in rows)
    summary={'schema_version':1,'accepted_replies':len(accepted),'requests_sent':requests,
        'rejected_reply_reasons':dict(Counter(r['reason'] for r in rejected)),
        'expired_requests':sum(r.get('event')=='request_expired' for r in rows),
        'abandoned_at_stop':sum(r.get('event')=='request_abandoned_at_stop' for r in rows),
        'addressing':dict(Counter(r['addressing'] for r in accepted)),
        'segment_events':[r for r in rows if r.get('event')=='clock_segment_start' or r.get('clock_discontinuity')],
        'segments':segments,'gnss_to_px4_clock':'UNKNOWN','absolute_accuracy_verified':False,
        'method':'Lowest RTT per 1-second bin; first 2/3 training with 6-MAD rejection (1 us floor); last 1/3 held out. No fit crosses segments.'}
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    with (out/'fit_diagnostics.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=['segment_id','host_midpoint_ns','elapsed_s','role','residual_ns','round_trip_ns','ts1_ns'])
        writer.writeheader();writer.writerows(diagnostics)
    base=min((r['host_midpoint_ns'] for r in accepted),default=0)
    baseline=accepted[0]['remote_minus_host_ns'] if accepted else 0
    data={'samples':[[ (r['host_midpoint_ns']-base)/1e9,r['round_trip_ns']/1e6,
        (r['remote_minus_host_ns']-baseline)/1e6,r['segment_id']] for r in accepted],
        'fits':[[ (d['host_midpoint_ns']-base)/1e9,d['residual_ns']/1e6,d['role'],d['segment_id']] for d in diagnostics],
        'rejections':[[ (r['host_monotonic_ns']-base)/1e9,r.get('round_trip_ns',0)/1e6,r['reason']] for r in rejected],
        'events':[[ (r['host_monotonic_ns']-base)/1e9,r.get('reason','clock_discontinuity')] for r in summary['segment_events']]}
    table=''.join('<tr>'+''.join('<td>'+escape(str(s.get(k,'—')))+'</td>' for k in
        ['segment_id','accepted_samples','duration_s','rtt_median_ms','rtt_p95_ms','drift_ppm','held_out_abs_residual_p95_ms'])+'</tr>' for s in segments)
    page='''<!doctype html><html><head><meta charset="utf-8"><title>Pi ↔ PX4 TIMESYNC</title><style>
body{font:15px/1.6 system-ui;max-width:1150px;margin:30px auto;padding:20px;background:#eef4f7;color:#203c4c}section{background:white;padding:22px;margin:20px 0;border-radius:12px}canvas{width:100%;height:270px}td,th{padding:9px;text-align:left;border-bottom:1px solid #ddd}pre{white-space:pre-wrap;overflow-wrap:anywhere}.note{background:#fff2d5;padding:16px}a{color:#096786}</style></head><body>
<h1>Pi ↔ PX4 clock bridge</h1><p>SESSION</p><p class="note">Estimated clock bridge only. GNSS ↔ PX4 remains UNKNOWN. RTT/2 is a transport proxy; fitted residuals are not an independent accuracy measurement. Archived source timestamps remain unchanged.</p>
<section><h2>Exchanges and clock segments</h2><p>ACCEPTED accepted replies / REQUESTS requests. Rejected reasons: REJECTIONS.</p><table><tr><th>Segment</th><th>Accepted</th><th>Span s</th><th>RTT median ms</th><th>RTT p95 ms</th><th>Drift ppm</th><th>Held-out p95 ms</th></tr>TABLE</table><p>Use each mapping only within its host-time interval. Pause/reconnect and clock-jump boundaries invalidate the previous mapping across the boundary.</p></section>
<section><h2>Round trip (ms)</h2><canvas id="rtt"></canvas><h2>Offset change (ms, relative to first accepted exchange)</h2><canvas id="offset"></canvas><h2>Affine-fit residual (ms)</h2><canvas id="fit"></canvas><p>Hover to inspect samples. Purple: held out; orange: training outliers; red: rejected exchanges or segment boundaries. Every accepted exchange is plotted.</p><pre id="hover">Hover over a plot.</pre></section>
<section><h2>Method and files</h2><p>METHOD</p><p>Clock mapping relates Pi monotonic nanoseconds to PX4 monotonic boot nanoseconds. PX4 message timestamps may refer to publication time or sensor sampling time; use the documented message field. Drift is remote-minus-host offset slope.</p><p><a href="summary.json">Mapping and summary JSON</a> · <a href="fit_diagnostics.csv">All fit diagnostics</a> · <a href="../clock_sync.jsonl">Original clock exchanges</a> · <a href="../mavlink_tx.csv">Transmit index</a> · <a href="../recording_report.html">Recording report</a> · <a href="https://mavlink.io/en/services/timesync.html">MAVLink TIMESYNC protocol</a></p><pre>EVENTS</pre></section>
<script>const data=DATA;
function plot(id,points,field){const c=document.getElementById(id),ctx=c.getContext('2d');let hit=[];
function draw(){const w=c.clientWidth,h=270,dpr=devicePixelRatio||1;c.width=w*dpr;c.height=h*dpr;ctx.scale(dpr,dpr);ctx.clearRect(0,0,w,h);if(!points.length)return;
const xs=points.map(p=>p[0]),ys=points.map(p=>p[field]);let xmin=Math.min(...xs),xmax=Math.max(...xs),ymin=Math.min(...ys),ymax=Math.max(...ys);if(xmax===xmin)xmax++;if(ymax===ymin)ymax++;const pad=(ymax-ymin)*.08;ymin-=pad;ymax+=pad;
const X=x=>55+(x-xmin)/(xmax-xmin)*(w-75),Y=y=>h-35-(y-ymin)/(ymax-ymin)*(h-55);ctx.font='12px system-ui';ctx.fillStyle='#456';ctx.strokeStyle='#dce4e9';
for(let i=0;i<5;i++){const y=ymin+(ymax-ymin)*i/4;ctx.beginPath();ctx.moveTo(55,Y(y));ctx.lineTo(w-20,Y(y));ctx.stroke();ctx.fillText(y.toFixed(3),1,Y(y)+4);ctx.fillText((xmin+(xmax-xmin)*i/4).toFixed(1)+'s',X(xmin+(xmax-xmin)*i/4)-12,h-10)}
for(const e of data.events){ctx.strokeStyle='#c55';ctx.beginPath();ctx.moveTo(X(e[0]),15);ctx.lineTo(X(e[0]),h-35);ctx.stroke()}
hit=points.map(p=>[X(p[0]),Y(p[field]),p]);for(const p of hit){ctx.fillStyle=p[2][2]==='held_out'?'#9562ad':p[2][2]==='train_outlier'?'#d58d15':p[2][2]==='rejected'?'#d33':'#147b99';ctx.beginPath();ctx.arc(p[0],p[1],2,0,Math.PI*2);ctx.fill()}}
c.addEventListener('mousemove',e=>{let best=null,dist=Infinity;for(const p of hit){let d=(p[0]-e.offsetX)**2+(p[1]-e.offsetY)**2;if(d<dist){dist=d;best=p}}if(best)document.getElementById('hover').textContent=id+' '+JSON.stringify(best[2])});draw();window.addEventListener('resize',draw)}
plot('rtt',data.samples.concat(data.rejections.map(p=>[p[0],p[1],'rejected',p[2]])),1);plot('offset',data.samples,2);plot('fit',data.fits,1);
</script></body></html>'''
    substitutions={'SESSION':escape(folder.name),'ACCEPTED':str(len(accepted)),'REQUESTS':str(requests),
        'REJECTIONS':escape(str(summary['rejected_reply_reasons'])),'TABLE':table,'METHOD':escape(summary['method']),
        'EVENTS':escape(json.dumps(summary['segment_events'],indent=2)),'DATA':json.dumps(data).replace('</','<\\/')}
    for key,value in substitutions.items():page=page.replace(key,value)
    (out/'report.html').write_text(page)
    return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('folder',type=Path)
    args=parser.parse_args();print(json.dumps(report(args.folder),indent=2))
