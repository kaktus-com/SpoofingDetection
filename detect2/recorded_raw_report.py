"""Standalone HTML/PDF review for conditional raw-recording detector runs."""
import base64
from collections import Counter
from html import escape
import json
from pathlib import Path

import numpy as np

from .recorded_flight import clean_json, write_json


def report(output,summaries):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages
    output=Path(output);cards=[];interactive={};summary_rows=[]
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
    with PdfPages(output/'november_detector_inspection.pdf') as pdf:
        for number,run in enumerate(summaries,1):
            primary=next(v for v in run['variants'] if v['offset_s']==0);rows=primary['rows'];name=run['session']
            t=np.array([r['gps_elapsed_start_s'] for r in rows])
            motion=np.array([r['observed_motion_rms_m'] if r['observed_motion_rms_m'] is not None else np.nan for r in rows])*1000
            bound=np.array([r['motion_noise_bound_rms_m'] if r['motion_noise_bound_rms_m'] is not None else np.nan for r in rows])*1000
            gamma=np.array([r['gamma'] if r['gamma'] is not None else np.nan for r in rows])
            proxy=next((v for v in run['variants'] if v['noise_model']=='ground_proxy_sensitivity' and v['offset_s']==0),None)
            fig,axes=plt.subplots(4,1,figsize=(11.7,9.5),sharex=True,constrained_layout=True)
            fig.suptitle(f'Recording {number}: {name}\nConditional replay — GNSS/PX4 timing unverified')
            for i,r in enumerate(rows):
                if r['airborne']:
                    for ax in axes:ax.axvspan(t[i],t[i]+1,color='#d9eadf',alpha=.65,lw=0)
            axes[0].plot(t,motion,label='QR-residual displacement RMS',color='#147d91')
            axes[0].plot(t,[np.nan if r['min_axis_rms_m'] is None else r['min_axis_rms_m']*1000 for r in rows],label='Weakest-axis RMS',color='#7851a9')
            axes[0].plot(t,bound,label='Configured acceleration-noise bound',color='#cc8927');axes[0].set_ylabel('Motion (mm)');axes[0].legend(loc='upper right')
            if proxy:
                axes[0].plot(t,[np.nan if r['motion_noise_bound_rms_m'] is None else r['motion_noise_bound_rms_m']*1000 for r in proxy['rows']],ls='--',color='#9b6b9b',label='Stationary-proxy sensitivity bound');axes[0].legend(loc='upper right')
            axes[1].plot(t,[r['continuous_quality_signals'] for r in rows],color='#147d91')
            axes[1].axhline(4,color='#bb6651',ls='--',label='Minimum 4 signals');axes[1].set_ylabel('Continuous\nGPS L1 signals');axes[1].legend()
            axes[2].plot(t,gamma,'.',ms=3,label='Primary');axes[2].axhline(0,color='#777',lw=.8);axes[2].set_ylabel('Conditional γ')
            if proxy:
                axes[2].plot(t,[np.nan if r['gamma'] is None else r['gamma'] for r in proxy['rows']],'.',ms=3,color='#b88029',label='Stationary-proxy sensitivity');axes[2].legend()
            if not np.isfinite(gamma).any():axes[2].text(.98,.95,'Primary: no likelihood fits',transform=axes[2].transAxes,va='top',ha='right')
            codes={'benign':1,'inconclusive':0,'spoofed':-1}
            axes[3].scatter(t,[codes[r['conditional_decision']] for r in rows],s=8,color='#147d91')
            if proxy:axes[3].scatter(t,[codes[r['conditional_decision']] for r in proxy['rows']],s=10,marker='x',color='#b88029')
            axes[3].set_yticks([-1,0,1],['Spoofed model','Inconclusive','Benign model']);axes[3].set_xlabel('GNSS elapsed seconds');axes[3].set_ylim(-1.4,1.4)
            for ax in axes:ax.grid(alpha=.18)
            image_path=output/name/'detector_timeline.png';fig.savefig(image_path,dpi=150);pdf.savefig(fig);plt.close(fig)
            image='<img alt="Motion, phase quality, gamma and conditional decision timeline" src="data:image/png;base64,'+base64.b64encode(image_path.read_bytes()).decode()+'">'
            example=run.get('example');example_html=''
            if example is not None:
                tt=np.asarray(example['elapsed_s']);dd=np.asarray(example['motion_mm']);yy=np.asarray(example['measured_phase_residual_cycles']);pred=np.asarray(example['nominal_motion_projection_cycles'])
                fig,axes=plt.subplots(2,1,figsize=(11.7,7),sharex=True,constrained_layout=True)
                fig.suptitle(f'Recording {number}, window {example["window_index"]}: highest primary motion RMS')
                for j,axis in enumerate(['East','North','Up']):axes[0].plot(tt,dd[:,j],label=axis)
                axes[0].set_ylabel('QR-residual antenna\ndisplacement (mm)');axes[0].legend()
                for j,signal in enumerate(example['signals']):
                    line,=axes[1].plot(tt,yy[:,j],label=signal+' measured')
                    axes[1].plot(tt,pred[:,j],'--',color=line.get_color(),label=signal+' nominal projection')
                axes[1].set_ylabel('QR-residual phase (cycles)');axes[1].set_xlabel('Window elapsed seconds');axes[1].legend(ncol=2)
                for ax in axes:ax.grid(alpha=.18)
                p=output/name/'example_window.png';fig.savefig(p,dpi=150);pdf.savefig(fig);plt.close(fig)
                example_html='<details><summary>Highest-motion window: displacement and phase</summary><p>Selected using IMU motion only. Dashed curves use receiver sky directions directly; they are reference projections, not the detector’s fitted common rotation. Outcome: '+escape(example['conditional_reason'])+'.</p><img alt="Selected window motion and phase" src="data:image/png;base64,'+base64.b64encode(p.read_bytes()).decode()+'"></details>'
            variant_rows=''
            for variant in run['variants']:
                d=variant['conditional_decisions'];path=name+'/'+variant['folder']
                variant_rows+=f'<tr><td>{variant["offset_s"]:+.3f} s</td><td>{variant["noise_model"]}: {variant["accel_noise_sigma_m_s2"]:.4f}</td><td>{variant["windows"]}</td><td>{d.get("benign",0)}</td><td>{d.get("spoofed",0)}</td><td>{d.get("inconclusive",0)}</td><td>{variant["gamma_available"]}</td><td><a href="{path}/windows.csv">CSV</a> · <a href="{path}/window_details.json">Fit details</a> · <a href="{path}/estimated_displacement.csv">Displacement</a></td></tr>'
                interactive[f'Recording {number}, {variant["noise_model"]}, {variant["offset_s"]:+.3f} s']=variant['rows']
            reasons=''.join('<tr><td>'+escape(reason)+'</td><td>'+str(count)+'</td></tr>' for reason,count in sorted(primary['conditional_reasons'].items(),key=lambda x:-x[1]))
            airborne_rows=[r for r in rows if r['airborne']];airborne_counts=dict(Counter(r['conditional_decision'] for r in airborne_rows))
            comparable=[r for r in (airborne_rows or rows) if r['min_axis_rms_m'] is not None]
            axis_note=''
            if comparable:
                axis_median=float(np.median([r['min_axis_rms_m'] for r in comparable]))*1000
                bound_median=float(np.median([r['motion_noise_bound_rms_m'] for r in comparable]))*1000
                axis_note=f'<p>Median weakest-axis displacement RMS ({"airborne" if airborne_rows else "all integrated"} windows): <b>{axis_median:.3f} mm</b>, versus the primary uncertainty bound <b>{bound_median:.3f} mm</b>. This bound comes from an assumed noise level, not an in-flight calibration.</p>'
            proxy_reasons=''
            if proxy:
                proxy_reasons='<details><summary>Stationary-proxy sensitivity reasons at zero offset</summary><table>'+''.join('<tr><td>'+escape(k)+'</td><td>'+str(v)+'</td></tr>' for k,v in sorted(proxy['conditional_reasons'].items(),key=lambda x:-x[1]))+'</table></details>'
            cards.append(f'''<section><h2>Recording {number}</h2><p><code>{name}</code> · {run['duration_s']:.1f} seconds · {len(run['selected_signals'])} GPS L1 identities over the recording. {len(airborne_rows)} primary windows have an airborne midpoint; green plot shading marks these candidate intervals.</p>
<p>Primary conditional outputs: {escape(str(primary['conditional_decisions']))}. Airborne subset: {escape(str(airborne_counts))}. <b>Validated conclusive decisions: 0.</b></p>{axis_note}{image}
<h3>Timing and noise sensitivity — primary result unchanged</h3><p>The stationary proxy is an optimistic diagnostic, not calibrated flight noise. Its orange plot markers and conditional outputs must not replace the primary result.</p><div class="scroll"><table><tr><th>Offset</th><th>Noise assumption m/s²</th><th>Windows</th><th>Benign model</th><th>Spoofed model</th><th>Inconclusive</th><th>γ available</th><th>Exports</th></tr>{variant_rows}</table></div>
<details open><summary>Primary rejection / decision reasons</summary><table>{reasons}</table></details>{proxy_reasons}{example_html}<p><a href="{name}/run.json">Session assumptions, provenance and complete results</a> · <a href="{name}/windows_all_offsets.csv">All offsets CSV</a></p></section>''')
            summary_rows.append({'session':name,'primary_conditional_decisions':primary['conditional_decisions'],
                'primary_reasons':primary['conditional_reasons'],'primary_gamma_available':primary['gamma_available'],
                'primary_windows':primary['windows'],'primary_airborne_conditional_decisions':airborne_counts,
                'validated_conclusive_decisions':0,'variants':[{k:v for k,v in a.items() if k!='rows'} for a in run['variants']],
                'clock':run['clock'],'source_unchanged':run['source_unchanged']})
    result={'status':'EXPLORATORY_UNVERIFIED_CLOCK_AND_FIELD_ERROR_MODEL','recordings':summary_rows,
        'validated_conclusive_decisions':0,'timing_selected_from_detector':False,'later_timesync_extrapolated':False,
        'primary_offset_s':0.,'window_s':1.,'step_s':1.,'accel_white_noise_sigma_assumption_m_s2':1.,
        'report':'report.html','pdf':'november_detector_inspection.pdf'}
    primary_counts=Counter();proxy_counts=Counter();primary_reasons=Counter()
    for run in summaries:
        for variant in run['variants']:
            if variant['offset_s']!=0:continue
            if variant['noise_model']=='configured':
                primary_counts.update(variant['conditional_decisions']);primary_reasons.update(variant['conditional_reasons'])
            else:proxy_counts.update(variant['conditional_decisions'])
    result.update(primary_conditional_decisions=dict(primary_counts),primary_reasons=dict(primary_reasons),
        stationary_proxy_zero_offset_conditional_decisions=dict(proxy_counts))
    write_json(output/'summary.json',result)
    headline=f'''<section><h2>What this replay found</h2><p><b>Primary run: {primary_counts.get('inconclusive',0)} of {sum(primary_counts.values())} windows are inconclusive.</b> {primary_reasons.get('imu_motion_below_noise_floor',0)} windows fail the motion-noise gate; other exclusions are native gaps and reset/calibration transitions. The 1 m/s² assumption does not permit a likelihood comparison in these windows.</p><p>The separately labelled stationary-proxy sensitivity at zero offset gives {proxy_counts.get('benign',0)} benign-model preferences, {proxy_counts.get('spoofed',0)} spoofed-model preferences and {proxy_counts.get('inconclusive',0)} inconclusive outputs. <b>These preferences are not verified spoofing detections.</b> Fixed ±50 ms runs show how provisional timing affects these numbers.</p></section>'''
    data=json.dumps(clean_json(interactive),allow_nan=False).replace('<','\\u003c')
    page='''<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>November detector replay</title><style>
body{font:15px/1.65 system-ui;background:#eef4f7;color:#203d4d;max-width:1170px;margin:25px auto;padding:20px}h1{font-size:38px;line-height:1.15}section{background:white;border:1px solid #d9e5ea;border-radius:14px;margin:22px 0;padding:24px}a{color:#08769b}img{width:100%;height:auto}table{border-collapse:collapse;width:100%}td,th{padding:9px;border-bottom:1px solid #dce5e9;text-align:left}.note{background:#fff0d0;padding:18px;border-radius:10px}.scroll{overflow:auto}details{margin:15px 0}summary{cursor:pointer;font-weight:600}select,input{font:inherit;padding:8px;margin:8px}pre{white-space:pre-wrap;overflow-wrap:anywhere}</style></head><body>
<h1>How the detector behaves on the November recordings</h1><p>5 November 2026 · Three original flight/reference recordings · RAWX + receiver sky geometry + native PX4 attitude/IMU</p>
<p class="note"><b>Exploratory replay, not a validated spoofing verdict.</b> These recordings lack PPS edges and active TIMESYNC exchanges. Alignment uses independent receive-time envelopes with an assumed clock rate. All final validated decisions remain inconclusive. Conditional benign/spoofed outputs and γ below describe the numerical detector only. They cannot establish an alarm rate or prove a clean flight.</p>
HEADLINE
<section><h2>What was run</h2><p>Unmodified detector quality thresholds and hypothesis fits, 1-second non-overlapping windows, GPS L1 C/A only (one signal per satellite), 2 cm forward antenna offset and PX4-supplied attitude. Displacement comes from our accelerometer integration; no fused PX4 displacement or GPS motion enters it.</p><p>RAWX phase/half-cycle validity, lock counters, half-cycle-subtraction changes, invalid uncertainty codes and missing samples control signal continuity. Receiver-reported NAV-SAT directions require healthy, ephemeris-source flags and age ≤2 seconds. They are approximate nominal geometry, not independently validated orbits. The recorded standard deviation scales the fit but does not calibrate real in-flight noise.</p><p>Primary acceleration white-noise assumption: 1 m/s² per sample, with the existing uncertainty gate enabled. Attitude, vibration, bias and timing errors remain outside that model. The 0 ms offset is primary; ±50 ms are fixed sensitivity checks. No timing or threshold is optimized using detector outcomes or flight labels. Windows crossing native attitude/calibration transitions are rejected.</p></section>
CARDS
<section><h2>Inspect an individual window</h2><label>Recording / offset <select id="variant"></select></label><label>Window <input id="window" type="number" min="0" value="0"></label><pre id="details"></pre><p>Observed motion and noise bounds are meters in the JSON/CSV. Conditional output is separate from validated_decision. Detailed fit parameters and excluded signals are available in each export.</p></section>
<section><h2>What must happen before a field conclusion</h2><p>Record verified X20 pulse epochs against PX4 PPS edge timestamps; validate the clock mapping and latency budget. Calibrate the field IMU/phase error assumptions and validate nominal geometry. Retain quality exclusions; then rerun without exploratory clock assumptions.</p><p><a href="summary.json">Overall summary JSON</a> · <a href="november_detector_inspection.pdf">All figures PDF</a> · <a href="../../RECORDED_RAW.md">Method and reproduction</a> · <a href="../../CONTINUATION.md">Progress and next tasks</a></p></section>
<script>const runs=DATA;const select=document.getElementById('variant'),win=document.getElementById('window');for(const name of Object.keys(runs)){const o=document.createElement('option');o.value=name;o.textContent=name;select.appendChild(o)}select.selectedIndex=1;function show(){const rows=runs[select.value];const index=Math.min(rows.length-1,Math.max(0,Math.trunc(Number(win.value)||0)));win.max=rows.length-1;document.getElementById('details').textContent=JSON.stringify(rows[index],null,2)}select.onchange=show;win.oninput=show;show();</script></body></html>'''
    (output/'report.html').write_text(page.replace('HEADLINE',headline).replace('CARDS',''.join(cards)).replace('DATA',data))
    print('HTML report:',output/'report.html',flush=True)
    return result
