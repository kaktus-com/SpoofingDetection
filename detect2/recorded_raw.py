"""Conditional detector replay of archived RAWX/NAV-SAT and native PX4 IMU.

This adapter never validates the missing GNSS/PX4 clock bridge. It runs the
existing detector with an explicitly provisional arrival-time anchor and fixed
sensitivity offsets. Source packets/timestamps remain unchanged.
"""
import argparse
from collections import Counter
from dataclasses import asdict, replace
import csv
import json
from pathlib import Path
import struct

import numpy as np

from .acquisition.audit_session import archive_index
from .acquisition.common import fingerprint
from .acquisition.ubx import frames, rawx
from .detector import DetectorConfig, WindowResult
from .recorded_flight import load_px4_imu, write_json, write_csv
from .replay import run_windows, WindowRecord, export_records
from .schema import CarrierSeries

WAVELENGTH=299792458./1575420000.  # GPS gnssId=0/sigId=0: L1 C/A.
DEFAULT_SOURCE=Path(__file__).parent/'results/recorded_nov05/review_20261105T135817Z/evidence/sessions'
DEFAULT_OUTPUT=Path(__file__).parent/'results/november_detector'
SESSIONS=('20261105T131308Z_d3f681','20261105T132010Z_c4cbfd','20261105T132748Z_9c3265')


def nav_sat(payload):
    if len(payload)<8:raise ValueError('Short NAV-SAT')
    tow,version,count,_=struct.unpack_from('<IBBH',payload)
    if version!=1 or len(payload)!=8+count*12:raise ValueError('Unexpected NAV-SAT version/length')
    rows={}
    for i in range(count):
        gnss,sv,cno,elev,azim,residual,flags=struct.unpack_from('<BBBbhhI',payload,8+12*i)
        health=(flags>>4)&3;orbit=(flags>>8)&7
        if gnss!=0 or not -90<=elev<=90 or not 0<=azim<=360 or health!=1 or orbit!=1:
            continue
        az,el=np.radians([azim,elev])
        rows[f'0:{sv}:0']=np.array([np.sin(az)*np.cos(el),np.cos(az)*np.cos(el),np.sin(el)])
    return tow/1000.,rows


def carrier_quality(epochs):
    """One frequency per SV; keep invalid/missing samples and explicit arc breaks."""
    ids=tuple(sorted({r['signal_id'] for e in epochs for r in e['rows']
                      if r['gnss_id']==0 and r['signal_code']==0}))
    lookup={s:j for j,s in enumerate(ids)};shape=(len(epochs),len(ids))
    phase,sigma,locks=[np.full(shape,np.nan) for _ in range(3)]
    valid=np.zeros(shape,dtype=bool);arcs=np.zeros(shape,dtype=int)
    previous={};arc=Counter()
    for i,e in enumerate(epochs):
        seen=set()
        for r in e['rows']:
            name=r['signal_id']
            if name not in lookup:continue
            if name in seen:raise ValueError('Duplicate signal within RAWX epoch')
            seen.add(name);j=lookup[name];code=r['carrier_std_encoding']&15
            ok=(r['carrier_valid'] and r['half_cycle_valid'] and code!=15 and
                np.isfinite(r['carrier_phase_cycles']) and not e['header']['clock_reset'])
            old=previous.get(name)
            if old and (not old['ok'] or not ok or i!=old['index']+1 or
                        r['lock_time_ms']<old['lock'] or r['sub_half_cycle']!=old['sub'] or
                        e['gps_s']-old['gps_s']>.075):arc[name]+=1
            phase[i,j]=r['carrier_phase_cycles'];sigma[i,j]=.004*code if code!=15 else np.nan
            locks[i,j]=r['lock_time_ms']/1000.;valid[i,j]=ok;arcs[i,j]=arc[name]
            previous[name]={'index':i,'ok':ok,'lock':r['lock_time_ms'],'sub':r['sub_half_cycle'],'gps_s':e['gps_s']}
    return ids,phase,sigma,locks,valid,arcs


def read_receiver(folder):
    manifest=json.loads((folder/'manifest.json').read_text())
    result,index=archive_index(folder,'receiver',manifest['files']['receiver.bin'])
    if result['status']!='PASS':raise ValueError(result)
    assert fingerprint(folder/'receiver_receive.csv')==manifest['files']['receiver_receive.csv']
    ends=np.array([int(r['byte_offset'])+int(r['length']) for r in index],dtype=np.int64)
    hosts=np.array([int(r['host_monotonic_ns']) for r in index],dtype=np.int64)
    epochs=[];sky=[];diag={}
    for offset,cls,ident,payload in frames((folder/'receiver.bin').read_bytes(),diag):
        if (cls,ident)==(2,21):
            h,rows=rawx(payload)
            if h['version']!=1:raise ValueError('Unexpected RAWX version')
            position=np.searchsorted(ends,offset+len(payload)+8,side='left')
            if position>=len(index):raise ValueError('RAWX frame has no complete receive chunk')
            epochs.append({'header':h,'rows':rows,'gps_s':h['gps_week']*604800.+h['gps_tow_s'],
                           'host_s':hosts[position]/1e9})
        elif (cls,ident)==(1,53):sky.append(nav_sat(payload))
    if diag.get('invalid_candidates') or not epochs or not sky:raise ValueError('Receiver framing/required messages unavailable')
    gps=np.array([e['gps_s'] for e in epochs])
    if np.any(np.diff(gps)<=0) or any(e['header']['clock_reset'] for e in epochs):
        raise ValueError('RAWX reset segments require separate analysis')
    return epochs,sky,carrier_quality(epochs),diag


def gps_elapsed(epochs):
    # Subtract week/TOW separately to retain sub-microsecond epoch precision.
    first=epochs[0]['header']
    return np.array([(e['header']['gps_week']-first['gps_week'])*604800.+
                     e['header']['gps_tow_s']-first['gps_tow_s'] for e in epochs])


def arrival_anchor(folder,epochs):
    """Independent receive-time envelopes, never a phase-fitted clock estimate."""
    from pymavlink.dialects.v20 import common as mav
    manifest=json.loads((folder/'manifest.json').read_text())
    status,index=archive_index(folder,'mavlink',manifest['files']['mavlink.bin'])
    if status['status']!='PASS':raise ValueError(status)
    assert fingerprint(folder/'mavlink_receive.csv')==manifest['files']['mavlink_receive.csv']
    parsers={};offsets=[];stamps=[]
    with (folder/'mavlink.bin').open('rb') as stream:
        for row in index:
            stream.seek(int(row['byte_offset']));data=stream.read(int(row['length']))
            peer=(row.get('peer_host'),row.get('peer_port'),row.get('segment_id'))
            parser=parsers.setdefault(peer,mav.MAVLink(None));parser.robust_parsing=True
            for msg in parser.parse_buffer(data) or []:
                if msg.get_type()=='HIGHRES_IMU' and (msg.get_srcSystem(),msg.get_srcComponent())==(1,1):
                    boot=msg.time_usec/1e6
                    offsets.append(int(row['host_monotonic_ns'])/1e9-boot);stamps.append(boot)
    if not offsets or np.any(np.diff(stamps)<0):raise ValueError('Missing/reset PX4 source clock')
    elapsed=gps_elapsed(epochs)
    host_minus_elapsed=np.array([e['host_s'] for e in epochs])-elapsed
    native_to_host=float(np.min(offsets));gnss_to_host=float(np.min(host_minus_elapsed))
    return {'anchor_boot_s':gnss_to_host-native_to_host,'host_minus_px4_min_s':native_to_host,
        'host_minus_gnss_elapsed_min_s':gnss_to_host,'gnss_receive_delay_range_s':float(np.ptp(host_minus_elapsed)),
        'px4_receive_offset_range_s':float(np.ptp(offsets)),'px4_messages':len(offsets),
        'formula':'candidate_PX4_s = anchor_boot_s + GPS_elapsed_s + fixed_offset_s',
        'status':'UNVERIFIED','rate_assumed_s_per_s':1.,'phase_used_for_alignment':False,
        'later_timesync_extrapolated':False,'absolute_transport_latency_known':False}


def native_boundaries(path):
    from pyulog import ULog
    log=ULog(str(path),message_name_filter_list=['sensor_combined','vehicle_attitude','vehicle_land_detected'])
    boundaries=[];land=None
    for ds in log.data_list:
        if ds.multi_id!=0:continue
        d=ds.data
        if ds.name=='vehicle_land_detected':land=d;continue
        ts=d['timestamp_sample'] if ds.name=='vehicle_attitude' else d['timestamp']
        for key in ['quat_reset_counter','accel_calibration_count','gyro_calibration_count']:
            if key in d:
                for i in np.flatnonzero(d[key][1:]!=d[key][:-1])+1:boundaries.append(float(ts[i])/1e6)
    return boundaries,land


def sky_for_window(sky,tow,ids,max_age_s=2.):
    # Use only already-published sky epochs. A future sky row is not substituted.
    candidates=[s for s in sky if 0<=tow-s[0]<=max_age_s]
    if not candidates:return None,np.zeros(len(ids),dtype=bool),None
    selected=max(candidates,key=lambda s:s[0]);covered=np.array([s in selected[1] for s in ids])
    los=np.array([selected[1].get(s,np.array([0.,0.,1.])) for s in ids])
    return los,covered,tow-selected[0]


def replay_one(folder,output,offsets=(-.05,0.,.05),lever=(.02,0.,0.),noise_sigma=1.):
    folder,output=Path(folder),Path(output);output.mkdir(parents=True,exist_ok=True)
    runtime=json.loads((folder/'runtime_manifest.json').read_text());native=folder/runtime['native_file']
    if fingerprint(native)!=runtime['native_fingerprint']:raise ValueError('Native ULog fingerprint mismatch')
    sources={n:fingerprint(folder/n) for n in ['receiver.bin','receiver_receive.csv','mavlink.bin','mavlink_receive.csv','manifest.json',runtime['native_file']]}
    epochs,sky,quality,framing=read_receiver(folder);ids,phase,sigma,locks,valid,arcs=quality
    clock=arrival_anchor(folder,epochs);imu,imu_meta=load_px4_imu(native,lever)
    boundaries,land=native_boundaries(native)
    elapsed=gps_elapsed(epochs);results=[];all_rows=[];example=None
    config=DetectorConfig();count=int(np.floor((elapsed[-1]+.05)/1.+1e-8))
    proxy=imu_meta.get('ground_noise_proxy_axis_m_s2')
    jobs=[(float(offset),float(noise_sigma),'configured') for offset in offsets]
    if proxy is not None and np.isfinite(proxy).all() and min(proxy)>0:
        jobs.extend((float(offset),float(max(proxy)),'ground_proxy_sensitivity') for offset in offsets)
    for offset,assumed_sigma,noise_name in jobs:
        imu=replace(imu,accel_noise_sigma_m_s2=assumed_sigma);records=[];context=[]
        for i in range(count):
            indices=np.flatnonzero((elapsed>=i-1e-7)&(elapsed<i+1-1e-7))
            if not len(indices):continue
            time_s=clock['anchor_boot_s']+elapsed[indices]+offset
            tow=epochs[0]['header']['gps_tow_s']+float(np.mean(elapsed[indices]))
            los,covered,age=sky_for_window(sky,tow,ids)
            base=CarrierSeries(time_s,phase[indices],np.tile([0.,0.,1.],(len(ids),1)) if los is None else los,
                np.full(len(ids),WAVELENGTH),sigma[indices],ids,valid[indices]&covered[None,:],locks[indices],arcs[indices])
            if los is None:r=WindowRecord(i,time_s[0],time_s[-1]+.05,base,WindowResult(reason='receiver_geometry_unavailable'))
            elif any(time_s[0]<=b<=time_s[-1] for b in boundaries):
                r=WindowRecord(i,time_s[0],time_s[-1]+.05,base,WindowResult(reason='px4_attitude_or_calibration_transition'))
            else:
                r=run_windows(base,config,imu=imu)[0];r.index=i
            landed=None
            if land is not None:
                j=np.searchsorted(land['timestamp']/1e6,float(np.mean(time_s)),side='right')-1
                if j>=0:landed=bool(land['landed'][j])
            check=r.result.diagnostics.get('imu_motion_noise',{})
            row={'session':folder.name,'window_index':i,'gps_elapsed_start_s':float(elapsed[indices[0]]),
                'offset_s':offset,'candidate_boot_start_s':float(time_s[0]),'samples':len(indices),
                'noise_model':noise_name,'accel_noise_sigma_m_s2':assumed_sigma,
                'airborne':None if landed is None else not landed,'sky_age_s':age,
                'continuous_quality_signals':int(np.count_nonzero(np.all(base.carrier_valid,axis=0)&
                    np.all(np.isfinite(base.sigma_cycles),axis=0)&np.all(np.diff(base.lock_time_s,axis=0)>=0,axis=0)&
                    np.all(base.arc_id==base.arc_id[0],axis=0))),
                'conditional_decision':r.result.decision,'conditional_reason':r.result.reason,
                'gamma':r.result.gamma,'used_signals':len(r.result.signal_indices),
                'observed_motion_rms_m':check.get('observed_total_rms_m',r.result.diagnostics.get('motion_rms_m')),
                'min_axis_rms_m':(check.get('observed_axis_rms_m') or [None])[-1],
                'motion_noise_bound_rms_m':check.get('noise_bound_rms_m'),
                'best_fit_rms_sigma':r.result.diagnostics.get('best_fit_rms_sigma'),
                'validated_decision':'inconclusive','validation_reason':'GNSS_PX4_clock_and_field_error_model_unverified'}
            records.append(r);context.append(row)
        variant=output/(('ground_proxy_' if noise_name!='configured' else '')+'offset_'+format(offset,'+.3f').replace('.','p'))
        variant.mkdir(exist_ok=True);write_csv(variant/'windows.csv',context)
        export_records(variant,records,mode='imu')
        # Keep the existing core exports explicitly conditional; summary CSV above
        # has separate validated and conditional fields for downstream readers.
        (variant/'windows.csv').rename(variant/'conditional_core_windows.csv')
        write_csv(variant/'windows.csv',context)
        decisions=Counter(r['conditional_decision'] for r in context);reasons=Counter(r['conditional_reason'] for r in context)
        results.append({'offset_s':offset,'noise_model':noise_name,'accel_noise_sigma_m_s2':assumed_sigma,
            'folder':variant.name,'windows':len(records),'conditional_decisions':dict(decisions),
            'conditional_reasons':dict(reasons),'gamma_available':sum(r['gamma'] is not None for r in context),
            'validated_conclusive_decisions':0,'rows':context})
        all_rows.extend(context)
        if offset==0 and noise_name=='configured' and any(r.motion_estimate is not None for r in records):
            from .detector import polynomial_basis
            choices=[(row,r) for row,r in zip(context,records) if r.motion_estimate is not None]
            row,r=max(choices,key=lambda pair:pair[0]['observed_motion_rms_m'] or 0.)
            q=np.linalg.qr(polynomial_basis(r.data.time_s)[0],mode='reduced')[0]
            d=r.motion_estimate.antenna_displacement_m;d=d-q@(q.T@d)
            js=[j for j in range(len(ids)) if np.all(r.data.carrier_valid[:,j]) and np.isfinite(r.data.phase_cycles[:,j]).all()][:3]
            measured=r.data.phase_cycles[:,js]-r.data.phase_cycles[0,js]
            measured=measured-q@(q.T@measured)
            example={'window_index':r.index,'selection':'maximum primary-offset nonpolynomial motion RMS; independent of phase fit/decision',
                'elapsed_s':r.data.time_s-r.data.time_s[0],'motion_mm':d*1000,
                'signals':[ids[j] for j in js],'measured_phase_residual_cycles':measured,
                'nominal_motion_projection_cycles':-d@r.data.nominal_los[js].T/WAVELENGTH,
                'conditional_reason':r.result.reason}
        print(folder.name,noise_name,'sigma',assumed_sigma,'offset',offset,'windows',len(records),'decisions',dict(decisions),'reasons',dict(reasons),flush=True)
    source_unchanged=all(fingerprint(folder/n)==v for n,v in sources.items())
    if not source_unchanged:raise ValueError('Source changed during replay')
    summary={'session':folder.name,'source':str(folder.resolve()),'source_fingerprints':sources,'source_unchanged':source_unchanged,
        'geometry':'receiver NAV-SAT azimuth/elevation; GPS ephemeris-source and healthy flags required; not independently reconstructed or validated',
        'selected_signals':ids,'frequency_hz':1575420000.,'wavelength_m':WAVELENGTH,
        'rawx_epochs':len(epochs),'duration_s':float(elapsed[-1]),'framing':framing,'clock':clock,
        'detector_config':asdict(config),'complete_windows':count,'partial_tail_s':float(elapsed[-1]+.05-count),
        'code_fingerprints':{name:fingerprint(Path(__file__).parent/name) for name in
            ['recorded_raw.py','recorded_raw_report.py','detector.py','motion.py','motion_uncertainty.py','recorded_flight.py']},
        'imu':imu_meta,'lever_arm_body_m':list(lever),'accel_white_noise_sigma_assumption_m_s2':noise_sigma,
        'motion_model':'independent Gaussian acceleration samples; attitude/timing/colour/bias uncertainty uncalibrated',
        'ground_proxy_sensitivity':'Maximum axis stationary adjacent-force MAD / 0.9538725524; independent of phase/decisions; not calibrated in-flight noise and not the primary result',
        'label':'unknown; land status used for plot stratification only','phase_weights':'0.004 * RAWX cpStd cycles; 15 invalid; 0.001 floor in detector; field covariance uncalibrated',
        'gps_position_or_displacement_used':False,'primary_offset_s':0.,'fixed_offsets_s':list(offsets),
        'timing_optimized_from_phase_or_detector':False,'variants':results,'example':example,'validated_conclusive_decisions':0}
    write_json(output/'run.json',summary);write_csv(output/'windows_all_offsets.csv',all_rows)
    return summary


def run(source=DEFAULT_SOURCE,output=DEFAULT_OUTPUT,offsets=(-.05,0.,.05)):
    source,output=Path(source).resolve(),Path(output).resolve()
    if output==source or source in output.parents:raise ValueError('Output must be outside the source recordings')
    if not np.isfinite(offsets).all() or len(set(offsets))!=len(offsets) or 0. not in offsets:
        raise ValueError('Choose finite unique offsets including the fixed primary offset 0')
    summaries=[replay_one(source/name,output/name,offsets) for name in SESSIONS]
    from .recorded_raw_report import report
    report(output,summaries)
    return summaries


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source-root',type=Path,default=DEFAULT_SOURCE)
    p.add_argument('--output-dir',type=Path,default=DEFAULT_OUTPUT);p.add_argument('--offsets-s',nargs='+',type=float,default=[-.05,0.,.05])
    a=p.parse_args();run(a.source_root,a.output_dir,a.offsets_s)
