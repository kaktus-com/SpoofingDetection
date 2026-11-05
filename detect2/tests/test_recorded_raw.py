import json
import struct

import numpy as np
from numpy.testing import assert_allclose
import pytest

from SpoofingDetection.detect2.acquisition.capture import Session
from SpoofingDetection.detect2.acquisition.common import fingerprint
from SpoofingDetection.detect2.acquisition.replay import synthetic_inputs
from SpoofingDetection.detect2.acquisition.ubx import frames, packet, rawx
from SpoofingDetection.detect2.recorded_raw import (nav_sat, carrier_quality, gps_elapsed,
    read_receiver, arrival_anchor, sky_for_window, run)
from SpoofingDetection.detect2 import recorded_raw
from SpoofingDetection.detect2.motion import ImuSeries


def sky_payload(entries=((0,3,45,0,0,0,0x110),),tow=221000000):
    return struct.pack('<IBBH',tow,1,len(entries),0)+b''.join(struct.pack('<BBBbhhI',*e) for e in entries)


def epoch(t,**updates):
    receiver,_=synthetic_inputs(False)
    header,rows=rawx(next(p for _,c,i,p in frames(receiver,{}) if (c,i)==(2,21)))
    header['gps_tow_s']=221000+t*.05
    row=next(r for r in rows if r['signal_id']=='0:3:0');row.update(lock_time_ms=1000+t*50,**updates)
    return {'header':header,'rows':[row],'gps_s':header['gps_week']*604800+header['gps_tow_s'],'host_s':500+t*.05}


def test_navsat_axes_health_orbit_and_malformed_packets():
    entries=[(0,3,45,0,0,0,0x110),(0,4,45,0,90,0,0x110),(0,5,45,90,0,0,0x110),
             (0,6,45,0,0,0,0x120),(0,7,45,0,0,0,0x210),(0,8,45,91,0,0,0x110)]
    tow,rows=nav_sat(sky_payload(entries))
    assert tow==221000 and set(rows)=={'0:3:0','0:4:0','0:5:0'}
    assert_allclose(rows['0:3:0'],[0,1,0],atol=1e-15)
    assert_allclose(rows['0:4:0'],[1,0,0],atol=1e-15)
    assert_allclose(rows['0:5:0'],[0,0,1],atol=1e-15)
    with pytest.raises(ValueError):nav_sat(sky_payload()[:-1])


def test_rawx_quality_keeps_invalid_sigma_and_resets_half_cycle_arcs():
    epochs=[epoch(0),epoch(1,carrier_std_encoding=15),epoch(2),epoch(3,sub_half_cycle=False)]
    ids,phase,sigma,lock,valid,arcs=carrier_quality(epochs)
    assert ids==('0:3:0',) and phase.shape==(4,1)
    assert_allclose(sigma[[0,2,3],0],.012)
    assert np.isnan(sigma[1,0]) and valid[:,0].tolist()==[True,False,True,True]
    assert arcs[1,0]>arcs[0,0] and arcs[2,0]>arcs[1,0] and arcs[3,0]>arcs[2,0]
    assert_allclose(gps_elapsed(epochs),[0,.05,.1,.15],atol=1e-10)


def test_missing_epoch_and_lock_decrease_start_new_arcs():
    a,b=epoch(0),epoch(2);b['rows'][0]['lock_time_ms']=10
    _,_,_,_,valid,arcs=carrier_quality([a,b])
    assert valid.all() and arcs[1,0]>arcs[0,0]


def test_geometry_does_not_replace_missing_history_with_future_rows():
    sky=[nav_sat(sky_payload(tow=221001000))]
    assert sky_for_window(sky,221000.,('0:3:0',))[0] is None
    los,covered,age=sky_for_window(sky,221001.5,('0:3:0','0:9:0'))
    assert covered.tolist()==[True,False] and age==.5
    assert sky_for_window(sky,221004.,('0:3:0',))[0] is None


def test_frame_receive_time_is_the_chunk_completing_the_frame(tmp_path):
    receiver,_=synthetic_inputs(False)
    rawx_frame=next(packet(c,i,p) for _,c,i,p in frames(receiver,{}) if (c,i)==(2,21))
    data=rawx_frame+packet(1,53,sky_payload())
    session=Session(tmp_path,expected_streams=('receiver',))
    session.writers['receiver'].submit(data[:10],1_000_000_000)
    session.writers['receiver'].submit(data[10:],2_000_000_000)
    session.finish()
    epochs,sky,quality,diag=read_receiver(session.path)
    assert epochs[0]['host_s']==2. and diag['valid_frames']==2
    assert len(sky)==1


def test_arrival_anchor_is_independent_of_phase_and_future_clock_files(tmp_path):
    _,datagrams=synthetic_inputs(False)
    session=Session(tmp_path,expected_streams=('mavlink',))
    session.writers['mavlink'].submit(datagrams[0],2_000_000_000,peer=('127.0.0.1',14551))
    session.writers['mavlink'].submit(datagrams[1],2_050_000_000,peer=('127.0.0.1',14551))
    session.finish();epochs=[epoch(0),epoch(1)]
    first=arrival_anchor(session.path,epochs)
    (session.path/'clock_sync.jsonl').write_text('not used and deliberately invalid')
    for e in epochs:e['rows'][0]['carrier_phase_cycles']=-1e99
    second=arrival_anchor(session.path,epochs)
    assert first==second and first['anchor_boot_s']==pytest.approx(498.)
    assert first['phase_used_for_alignment'] is False and first['status']=='UNVERIFIED'


def test_replay_refuses_writing_inside_source(tmp_path):
    with pytest.raises(ValueError,match='outside'):run(tmp_path,tmp_path/'results')


def test_receiver_reset_is_not_silently_mapped_across_a_single_anchor(tmp_path):
    raw,_=synthetic_inputs(True)
    session=Session(tmp_path,expected_streams=('receiver',))
    session.writers['receiver'].submit(raw+packet(1,53,sky_payload()),1_000_000_000)
    session.finish()
    with pytest.raises(ValueError,match='reset segments'):read_receiver(session.path)


def test_conditional_replay_keeps_validation_blocked_and_source_unchanged(tmp_path,monkeypatch):
    raw,datagrams=synthetic_inputs(False);session=Session(tmp_path/'source')
    session.writers['receiver'].submit(packet(1,53,sky_payload())+raw,100_000_000_000)
    for i,data in enumerate(datagrams):session.writers['mavlink'].submit(data,100_000_000_000+i*50_000_000,peer=('127.0.0.1',14551))
    session.finish();folder=session.path;native=folder/'native.ulg';native.write_bytes(b'fixture')
    (folder/'runtime_manifest.json').write_text(json.dumps({'native_file':'native.ulg','native_fingerprint':fingerprint(native)}))
    t=np.linspace(-3,3,1201)
    imu=ImuSeries(t,np.tile([0,0,9.80665],(len(t),1)),np.zeros((len(t),3)),np.tile([1,0,0,0],(len(t),1)),[.02,0,0],[0,0,-9.80665],200.)
    monkeypatch.setattr(recorded_raw,'load_px4_imu',lambda *a:(imu,{'ground_noise_proxy_axis_m_s2':[.01,.02,.03]}))
    monkeypatch.setattr(recorded_raw,'native_boundaries',lambda p:([],None))
    summary=recorded_raw.replay_one(folder,tmp_path/'output')
    assert summary['source_unchanged'] and summary['validated_conclusive_decisions']==0
    assert not summary['timing_optimized_from_phase_or_detector'] and not summary['clock']['later_timesync_extrapolated']
    assert len(summary['variants'])==6
    for variant in summary['variants']:
        assert variant['windows']==2
        assert all(r['validated_decision']=='inconclusive' for r in variant['rows'])
        assert variant['accel_noise_sigma_m_s2']==(1. if variant['noise_model']=='configured' else .03)
    assert 'NaN' not in (tmp_path/'output/run.json').read_text()
