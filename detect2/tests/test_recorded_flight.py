"""Recorded time/frame/unknown-quality boundaries, without real-data fixtures."""

import csv
from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np
from numpy.testing import assert_allclose
import pytest

from SpoofingDetection.detect2.recorded_flight import (
    ULOG_TOPICS, clock_diagnostics, load_recorded_carrier, phase_windows,
    supplied_attitude_on_accel, load_px4_imu, run,
)
from SpoofingDetection.detect2.geometry import quaternion_multiply, quaternion_to_matrix


def carrier_csv(path, times=None):
    times = np.arange(40)/25 if times is None else np.asarray(times)
    fields=['gps_time','carrier_sync_time','imu_sync_time','mavlink_time_boot_ms',
            'roll','pitch','yaw','carrier_0_3_7','carrier_0_4_7','carrier_0_6_7','carrier_0_9_7',
            'detrended_0_3_7']
    with path.open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
        for t in times:
            row={'gps_time':221000+t,'carrier_sync_time':100+1.4*t,
                 'imu_sync_time':99.98+1.4*t,'mavlink_time_boot_ms':(10+1.4*t)*1000,
                 'roll':0,'pitch':0,'yaw':0,'detrended_0_3_7':999}
            row.update({f'carrier_0_{prn}_7':1e8+20*t+3*t*t+prn*.001*t**3 for prn in [3,4,6,9]})
            writer.writerow(row)


def fake_px4(monkeypatch):
    times=10+np.arange(601)/200
    attitude=10+np.arange(61)/20
    acc={'timestamp':np.round(times*1e6).astype(np.int64),
         'accelerometer_timestamp_relative':np.zeros(len(times)),
         'accelerometer_clipping':np.zeros(len(times))}
    for j in range(3):
        acc[f'accelerometer_m_s2[{j}]']=np.full(len(times),-9.80665 if j==2 else 0.)
        acc[f'gyro_rad[{j}]']=np.zeros(len(times))
    att={'timestamp_sample':np.round(attitude*1e6).astype(np.int64),
         'quat_reset_counter':np.zeros(len(attitude),dtype=int)}
    for j in range(4):att[f'q[{j}]']=np.full(len(attitude),1. if j==0 else 0.)
    calls=[]
    def load(path,message_name_filter_list):
        calls.append(tuple(message_name_filter_list))
        assert set(message_name_filter_list)==set(ULOG_TOPICS)
        # No info/UTC dictionary or GNSS topic exists, so accidental reads fail.
        return SimpleNamespace(data_list=[SimpleNamespace(name=name,multi_id=0,data=d)
                                          for name,d in [('sensor_combined',acc),('vehicle_attitude',att)]])
    monkeypatch.setitem(sys.modules,'pyulog',SimpleNamespace(ULog=load))
    return calls


def test_clock_proxy_adds_arrival_offset_and_does_not_call_delay_synchronization(tmp_path):
    path=tmp_path/'data.csv';carrier_csv(path)
    data=load_recorded_carrier(path);check=clock_diagnostics(data)
    assert check['anchor_boot_s']==pytest.approx(10.02)
    assert check['receive_delay_growth_s']==pytest.approx(.4*data.relative_time[-1])
    assert check['status']=='unverified' and not check['gps_derived_ulog_utc_used']
    assert data.phase[0,0]==1e8
    assert data.phase[1,0]!=999


def test_phase_projection_uses_real_irregular_epochs_without_filling_gaps(tmp_path):
    path=tmp_path/'data.csv';carrier_csv(path,[0,.04,.12,.16,.28,.32,.4,.6,.72,.84])
    data=load_recorded_carrier(path)
    windows=phase_windows(data)
    assert len(windows)==1 and windows[0]['samples']==10
    assert not windows[0]['availability_pass']
    p=windows[0]['phase_residual_cycles'][:,0]
    t=data.relative_time
    # Quadratic-null residual must be orthogonal to all polynomial terms.
    assert_allclose(np.column_stack([np.ones(len(t)),t,t*t]).T@p,0,atol=1e-12)


def test_conflicting_duplicate_epochs_are_rejected(tmp_path):
    path=tmp_path/'data.csv';carrier_csv(path)
    lines=path.read_text().splitlines()
    row=lines[1].split(',');row[7]='123'
    path.write_text('\n'.join(lines+[','.join(row)])+'\n')
    with pytest.raises(ValueError,match='Conflicting'):
        load_recorded_carrier(path)


def test_slerp_masks_reset_gap_and_coverage_and_converts_body_frame():
    times=np.array([0.,.05,.1,.3])
    q=np.tile([1.,0,0,0],(4,1))
    enu=quaternion_multiply([0,np.sqrt(.5),np.sqrt(.5),0],q)
    query=np.array([-.01,0,.025,.075,.1,.2,.3,.4])
    supplied,valid=supplied_attitude_on_accel(query,times,enu,[0,0,1,1])
    assert valid.tolist()==[False,True,True,False,True,False,True,False]
    assert_allclose(quaternion_to_matrix(supplied[valid])@np.array([1.,0,0]),
                    np.tile([0.,1.,0.],(valid.sum(),1)),atol=1e-15)


def test_imu_adapter_uses_supplied_attitude_without_gnss_or_double_board_rotation(monkeypatch):
    calls=fake_px4(monkeypatch)
    imu,meta=load_px4_imu('unused.ulg',[.02,0,0])
    assert calls==[ULOG_TOPICS]
    assert_allclose(imu.linear_acceleration_enu_m_s2,0,atol=4e-15)
    assert meta['lever_arm_body_m'].tolist()==[.02,0,0]
    assert not meta['gyro_attitude_estimation'] and not meta['gps_position_or_utc_header_used']


def test_end_to_end_inspection_keeps_unknown_quality_blank_and_inputs_immutable(tmp_path,monkeypatch):
    fake_px4(monkeypatch)
    folder=tmp_path/'20260707_152258';folder.mkdir()
    source=folder/'synced_measurements.csv';carrier_csv(source)
    ulog=tmp_path/'flight.ulg';ulog.write_bytes(b'test')
    original=source.read_bytes()
    out=tmp_path/'report'
    meta=run(folder,ulog,out,lever_arm=[.02,0,0],plots=False,expected_case='benign')
    assert source.read_bytes()==original
    assert meta['conclusive_decisions']==0 and meta['geometry'] is None
    assert not meta['labels']['used_for_alignment_or_model_fit']
    with (out/'adapted_inputs/carrier.csv').open() as stream:
        saved=list(csv.DictReader(stream))
    assert saved and all(row['receiver_valid']==row['lock_time_s']==row['sigma_cycles']=='' for row in saved)
    report=(out/'report.html').read_text()
    assert 'NaN' not in report and '__DATA__' not in report and 'Infinity' not in report
    assert 'SATELLITE_GEOMETRY_MISSING' not in report  # no hidden manufactured geometry
