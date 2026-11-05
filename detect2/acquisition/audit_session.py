"""Audit raw archive integrity separately from measurement/clock readiness."""
import argparse
from collections import Counter
import csv
from html import escape
import json
import math
import mmap
from pathlib import Path
import struct

from .common import atomic_json,fingerprint
from .ubx import frames,rawx,NAMES


def archive_index(folder,name,expected):
    path=folder/(name+'.bin');index=folder/(name+'_receive.csv')
    result={'status':'FAIL','errors':[],'rows':0,'bytes':path.stat().st_size if path.exists() else None}
    if not path.exists() or not index.exists():
        result['errors'].append('archive or index missing');return result,[]
    try:
        with index.open() as f:rows=list(csv.DictReader(f))
        position=0;previous_seq=-1;last_time=None
        for row in rows:
            seq=int(row['sequence']);offset=int(row['byte_offset']);length=int(row['length']);host=int(row['host_monotonic_ns'])
            if offset!=position or length<0:result['errors'].append('index coverage discontinuity')
            if seq!=previous_seq+1:result['errors'].append('input sequence gap')
            if last_time is not None and host<last_time:result['errors'].append('host timestamps run backwards')
            previous_seq=seq;last_time=host;position=offset+length
        if position!=path.stat().st_size:result['errors'].append('unindexed or missing archive bytes')
        actual=fingerprint(path);result['sha256']=actual['sha256'];result['rows']=len(rows)
        if expected is None:result['errors'].append('final source fingerprint unavailable')
        elif actual!=expected:result['errors'].append('archive fingerprint mismatch')
        result['status']='PASS' if not result['errors'] else 'FAIL'
        return result,rows
    except (OSError,ValueError,KeyError) as error:
        result['errors'].append(str(error));return result,[]


def inspect_receiver(folder):
    path=folder/'receiver.bin';diag={};epochs=[];signals=Counter();types=Counter();invalid_flags=0;resets=0
    csv_path=folder/'receiver_observations.csv'
    with csv_path.open('w',newline='') as out:
        writer=None
        if not path.exists() or not path.stat().st_size:
            return {'status':'UNKNOWN','reason':'no receiver bytes','epochs':0,'observations':0},epochs
        with path.open('rb') as stream,mmap.mmap(stream.fileno(),0,access=mmap.ACCESS_READ) as data:
            for offset,cls,ident,payload in frames(data,diag):
                name=NAMES.get((cls,ident),f'UNKNOWN_{cls:02x}_{ident:02x}');types[name]+=1
                if (cls,ident)!=(2,21):continue
                try:header,observations=rawx(payload)
                except ValueError:
                    diag['malformed_rawx']=diag.get('malformed_rawx',0)+1;continue
                absolute=header['gps_week']*604800.+header['gps_tow_s']
                if not math.isfinite(absolute):diag['nonfinite_epoch']=diag.get('nonfinite_epoch',0)+1;continue
                header['absolute_gps_s']=absolute;header['byte_offset']=offset;epochs.append(header)
                resets+=int(header['clock_reset'])
                for row in observations:
                    row['byte_offset']=offset
                    if writer is None:writer=csv.DictWriter(out,fieldnames=list(row));writer.writeheader()
                    writer.writerow(row);signals[row['signal_id']]+=1
                    invalid_flags+=int(not row['carrier_valid'])
    gaps=[];backwards=0
    for left,right in zip(epochs[:-1],epochs[1:]):
        if right['clock_reset']:continue
        delta=right['absolute_gps_s']-left['absolute_gps_s']
        if delta<=0:backwards+=1
        else:gaps.append(delta)
    sorted_gaps=sorted(gaps)
    nominal=sorted_gaps[len(gaps)//2] if gaps else None
    # Expected rate is declared by capture/replay, never inferred from a damaged stream.
    return {'status':'PASS' if epochs and not diag.get('invalid_candidates') and not diag.get('malformed_rawx') else 'FAIL',
        'epochs':len(epochs),'observations':sum(signals.values()),'signal_counts':dict(signals),
        'packet_counts':dict(types),'invalid_carrier_flag_observations':invalid_flags,'clock_resets':resets,
        'backwards_or_repeated_epochs':backwards,'median_epoch_interval_s':nominal,
        'max_epoch_gap_s':max(gaps) if gaps else None,'diagnostics':diag,
        'note':'Invalid flags retained; continuity at resets is segmented, not repaired.'},epochs


def inspect_mavlink(folder,index):
    from pymavlink.dialects.v20 import common as mav
    counts=Counter();stamps={};timelines={};bad=0;parsers={}
    path=folder/'mavlink.bin'
    if not path.exists():return {'status':'UNKNOWN','messages':{}}
    with path.open('rb') as stream:
        for row in index:
            stream.seek(int(row['byte_offset']));data=stream.read(int(row['length']))
            peer=(row.get('peer_host',''),row.get('peer_port',''),row.get('segment_id','0'))
            parser=parsers.setdefault(peer,mav.MAVLink(None));parser.robust_parsing=True
            try:messages=parser.parse_buffer(data) or []
            except Exception:bad+=1;continue
            for msg in messages:
                name=msg.get_type();counts[name]+=1
                if name=='BAD_DATA':bad+=1
                for field in ['time_boot_ms','time_usec']:
                    if hasattr(msg,field):
                        value=getattr(msg,field);stamps.setdefault(name,set()).add(value)
                        timelines.setdefault((msg.get_srcSystem(),msg.get_srcComponent(),name),[]).append(value/(1000 if field=='time_boot_ms' else 1e6));break
    source_gaps={}
    for key,times in timelines.items():
        if key[2] not in ['ATTITUDE','ATTITUDE_QUATERNION','HIGHRES_IMU'] or len(times)<3:continue
        delta=[b-a for a,b in zip(times[:-1],times[1:])];positive=sorted(d for d in delta if d>0)
        nominal=positive[len(positive)//2] if positive else None
        source_gaps[':'.join(map(str,key))]={'median_interval_s':nominal,'max_interval_s':max(delta),
            'large_gaps':sum(d>1.5*nominal+1e-6 for d in delta) if nominal else 0,
            'backwards_timestamps':sum(d<0 for d in delta)}
    return {'status':'PASS' if counts and not bad else 'FAIL' if bad else 'UNKNOWN',
        'messages':dict(counts),'bad_or_undecodable_messages':bad,
        'distinct_source_timestamps':{k:len(v) for k,v in stamps.items()},'source_gap_checks':source_gaps,
        'note':'Unknown messages remain in original datagrams. Sequence gaps are not automatically equated with link loss through a router.'}


def audit(folder):
    folder=Path(folder)
    try:manifest=json.loads((folder/'manifest.json').read_text())
    except (OSError,ValueError) as error:manifest={'status':'unknown','files':{},'issues':{'counts':{'manifest_error':str(error)}}}
    checks={};indexes={}
    for name in ['receiver','mavlink']:
        checks[name+'_archive'],indexes[name]=archive_index(folder,name,manifest.get('files',{}).get(name+'.bin'))
        index_path=folder/(name+'_receive.csv');expected=manifest.get('files',{}).get(index_path.name)
        if expected is None or not index_path.exists() or fingerprint(index_path)!=expected:
            checks[name+'_archive']['status']='FAIL';checks[name+'_archive']['errors'].append('index fingerprint mismatch or unavailable')
    receiver,epochs=inspect_receiver(folder)
    telemetry=inspect_mavlink(folder,indexes['mavlink']) if checks['mavlink_archive']['status']=='PASS' else {'status':'UNKNOWN','reason':'MAVLink index/archive integrity failed; decoding skipped'}
    expected_hz=manifest.get('metadata',{}).get('expected_rawx_hz')
    bad_intervals=None
    if expected_hz and len(epochs)>1:
        bad_intervals=sum(not r['clock_reset'] and abs((r['absolute_gps_s']-l['absolute_gps_s'])-1/expected_hz)>.002
                          for l,r in zip(epochs[:-1],epochs[1:]))
    checks['receiver_epoch_continuity']={'status':'UNKNOWN' if bad_intervals is None else 'FAIL' if bad_intervals or receiver.get('clock_resets') else 'PASS',
        'expected_hz':expected_hz,'unexpected_intervals':bad_intervals,'clock_resets':receiver.get('clock_resets',0)}
    checks['finalized']={'status':'PASS' if manifest.get('status') not in ['recording','unknown'] and not (folder/'UNFINALIZED').exists() else 'FAIL'}
    checks['capture_session']={'status':'PASS' if manifest.get('status')=='complete' else 'FAIL','recorded_status':manifest.get('status'),
        'issues':manifest.get('issues',{}).get('counts',{})}
    checks['receiver_rawx_present']={'status':'PASS' if receiver['epochs'] else 'UNKNOWN'}
    checks['receiver_frame_validation']={'status':receiver['status'],'diagnostics':receiver.get('diagnostics',{})}
    checks['mavlink_decoding']={'status':telemetry['status']}
    gaps=telemetry.get('source_gap_checks',{})
    checks['mavlink_source_continuity']={'status':'FAIL' if any(g['large_gaps'] or g['backwards_timestamps'] for g in gaps.values()) else 'PASS' if gaps else 'UNKNOWN',
        'streams':gaps,'interpretation':'Source timestamp gaps observed; a router/changed stream rate may also explain gaps. This is not a network loss probability.'}
    checks['gnss_to_px4_clock']={'status':'UNKNOWN','reason':'No independently verified pulse/measurement-clock mapping in this stage.'}
    checks['native_ulog']={'status':'UNKNOWN','reason':'Native SD-card ULog is attached and validated in a later stage.'}
    replay=manifest.get('replay_sources',{})
    for name,source in replay.items():
        checks[name+'_replay_bytes']={'status':'PASS' if fingerprint(folder/(name+'.bin'))==source['fingerprint'] else 'FAIL',
                                     'source':source['path'],'source_sha256':source['fingerprint']['sha256']}
    result={'schema_version':1,'capture_status':manifest.get('status'),'checks':checks,'receiver':receiver,
            'telemetry':telemetry,'ready_for_validated_detector':False,
            'interpretation':'PASS concerns an individual check only. UNKNOWN never counts as PASS; this stage does not validate flight timing or detection.'}
    atomic_json(folder/'audit.json',result)
    rows=''.join('<tr><td>'+escape(k.replace('_',' '))+'</td><td class="'+v['status'].lower()+'">'+v['status']+'</td><td>'+escape(json.dumps({a:b for a,b in v.items() if a!='status'}))+'</td></tr>' for k,v in checks.items())
    issue_text=json.dumps(manifest.get('issues',{}),indent=2)
    epoch_data=[{'elapsed':e['absolute_gps_s']-epochs[0]['absolute_gps_s'],'reset':e['clock_reset']} for e in epochs]
    template=Path(__file__).with_name('audit_template.html').read_text()
    replacements={'__SESSION__':escape(folder.name),'__STATUS__':escape(manifest.get('status','UNKNOWN')),'__MODE__':escape(manifest.get('mode','UNKNOWN')),
                  '__TABLE__':rows,'__RECEIVER__':escape(json.dumps(receiver,indent=2)),
                  '__TELEMETRY__':escape(json.dumps(telemetry,indent=2)),'__ISSUES__':escape(issue_text),
                  '__EPOCHS__':json.dumps(epoch_data,allow_nan=False).replace('<','\\u003c'),
                  '__INJECTIONS__':escape(json.dumps(manifest.get('metadata',{}).get('replay_injections',{}),indent=2))}
    for marker,value in replacements.items():template=template.replace(marker,value)
    (folder/'report.html').write_text(template)
    for name,value in checks.items():print(value['status'],name)
    print('HTML report:',(folder/'report.html').resolve())
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('session',type=Path)
    audit(p.parse_args().session)


if __name__=='__main__':main()
