"""Acquisition failure boundaries, byte integrity, clocks and RAM transactions."""
import csv
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time

import pytest

from SpoofingDetection.detect2.acquisition.common import ArchiveWriter,Issues
from SpoofingDetection.detect2.acquisition.capture import Session
from SpoofingDetection.detect2.acquisition.replay import replay,synthetic_inputs
from SpoofingDetection.detect2.acquisition.audit_session import audit
from SpoofingDetection.detect2.acquisition.serial_capture import SerialCapture
from SpoofingDetection.detect2.acquisition.mavlink_capture import MavlinkCapture
from SpoofingDetection.detect2.acquisition.timesync import TimeSyncTracker,TimeSyncObserver
from SpoofingDetection.detect2.acquisition.receiver_config import snapshot,apply_ram,restore_ram,profile_values
from SpoofingDetection.detect2.acquisition.ubx import frames,rawx


def wait_for(predicate,timeout=3):
    until=time.monotonic()+timeout
    while time.monotonic()<until:
        if predicate():return
        time.sleep(.005)
    raise AssertionError('condition not reached before deadline')


def test_replay_preserves_flags_all_bands_unknown_messages_and_exposes_injected_gaps(tmp_path):
    receiver,datagrams=synthetic_inputs()
    p=replay(tmp_path,receiver,datagrams,expected_hz=20,injections={'clock_reset':True,'mavlink_interruption':True})
    a=json.loads((p/'audit.json').read_text())
    assert (p/'receiver.bin').read_bytes()==receiver
    assert (p/'mavlink.bin').read_bytes()==b''.join(datagrams)
    assert a['checks']['receiver_replay_bytes']['status']=='PASS'
    assert a['receiver']['epochs']==39 and a['receiver']['observations']==156
    assert a['receiver']['invalid_carrier_flag_observations']==1
    assert a['receiver']['clock_resets']==1
    assert a['receiver']['packet_counts']['UNKNOWN_99_55']==1
    assert len(a['receiver']['signal_counts'])==4
    assert a['checks']['receiver_epoch_continuity']['status']=='FAIL'
    assert a['checks']['mavlink_source_continuity']['status']=='FAIL'
    assert a['checks']['gnss_to_px4_clock']['status']=='UNKNOWN'
    assert a['checks']['native_ulog']['status']=='UNKNOWN'
    assert a['telemetry']['messages']['ATTITUDE_QUATERNION']==29
    assert a['telemetry']['messages']['HIGHRES_IMU']==29
    with (p/'receiver_observations.csv').open() as f:rows=list(csv.DictReader(f))
    assert len(rows)==156 and any(r['carrier_valid']=='False' for r in rows)
    assert not a['ready_for_validated_detector']


def test_audit_detects_tampering_with_archive_and_index(tmp_path):
    receiver,dgrams=synthetic_inputs();p=replay(tmp_path,receiver,dgrams,expected_hz=20)
    with (p/'receiver.bin').open('ab') as f:f.write(b'extra unindexed bytes')
    with (p/'mavlink_receive.csv').open('a') as f:f.write('100,999,1,100,0,127.0.0.1,1\n')
    a=audit(p)
    assert a['checks']['receiver_archive']['status']=='FAIL'
    assert a['checks']['mavlink_archive']['status']=='FAIL'


def test_slow_but_bounded_writer_drains_on_close(tmp_path):
    issues=Issues();w=ArchiveWriter(tmp_path,'receiver',issues,max_chunks=8,max_bytes=2048,write_delay_s=.01)
    chunks=[bytes([i])*80 for i in range(6)]
    for i,c in enumerate(chunks):assert w.submit(c,i)
    assert w.close()['written_bytes']==480
    assert (tmp_path/'receiver.bin').read_bytes()==b''.join(chunks)
    assert issues.snapshot()['counts']=={}


def test_overflow_marks_failure_and_counts_every_lost_chunk(tmp_path):
    s=Session(tmp_path,queue_chunks=1,queue_bytes=120,write_delay_s=.02,expected_streams=('receiver',))
    for i in range(80):s.writers['receiver'].submit(b'x'*100,i)
    m=s.finish()
    w=m['streams']['receiver']
    assert m['status']=='failed' and w['dropped_chunks']>0
    assert w['received_bytes']==w['written_bytes']+w['dropped_bytes']
    assert w['peak_buffered_bytes']<=120
    assert m['issues']['counts']['archive_rejected_input']==w['dropped_chunks']


def test_default_buffer_preserves_small_packet_burst_during_writer_pause(tmp_path):
    release = threading.Event()
    entered = threading.Event()
    class PausedFile:
        def __init__(self,path,mode):self.file=open(path,mode)
        def write(self,data):
            entered.set()
            if not release.wait(3):raise TimeoutError('Test writer was not released')
            return self.file.write(data)
        def flush(self):self.file.flush()
        def fileno(self):return self.file.fileno()
        def close(self):self.file.close()
    s=Session(tmp_path,expected_streams=('receiver',),opener=PausedFile)
    writer=s.writers['receiver']
    chunks=[bytes([i%256])*64 for i in range(600)]
    accepted=[]
    try:
        accepted.append(writer.submit(chunks[0],0))
        assert entered.wait(1)
        for i,chunk in enumerate(chunks[1:],1):accepted.append(writer.submit(chunk,i))
    finally:
        release.set()
        manifest=s.finish()
    assert all(accepted)
    assert manifest['status']=='complete' and not manifest['issues']['counts']
    assert (s.path/'receiver.bin').read_bytes()==b''.join(chunks)
    assert manifest['streams']['receiver']['peak_buffered_bytes']<=manifest['buffer_limits_per_stream']['max_bytes']


def test_disk_write_failure_cannot_finalize_as_success(tmp_path):
    class Broken:
        def __init__(self,path,mode):self.f=open(path,mode)
        def write(self,data):raise OSError('simulated disk full')
        def flush(self):self.f.flush()
        def fileno(self):return self.f.fileno()
        def close(self):self.f.close()
    s=Session(tmp_path,expected_streams=('receiver',),opener=Broken)
    s.writers['receiver'].submit(b'not written',1)
    m=s.finish()
    assert m['status']=='failed'
    assert m['issues']['counts']['archive_write_error']==1
    assert 'simulated disk full' in m['streams']['receiver']['error']
    assert audit(s.path)['checks']['capture_session']['status']=='FAIL'


def test_missing_mavlink_does_not_stop_serial_acquisition(tmp_path):
    receiver,_=synthetic_inputs()
    class Serial:
        def __init__(self,*args,**kwargs):self.remaining=receiver
        @property
        def in_waiting(self):return len(self.remaining)
        def read(self,n):
            result=self.remaining[:n];self.remaining=self.remaining[n:]
            if not result:time.sleep(.005)
            return result
        def close(self):pass
    s=Session(tmp_path);worker=SerialCapture('fake',921600,s.writers['receiver'],s.stop,s.issues,Serial)
    s.workers=[worker];worker.start()
    wait_for(lambda:s.writers['receiver'].stats()['written_bytes']==len(receiver))
    m=s.finish()
    assert (s.path/'receiver.bin').read_bytes()==receiver
    assert m['missing_streams']==['mavlink'] and m['status']=='incomplete'


def test_timesync_matches_source_token_target_and_duplicate_rules():
    t=TimeSyncTracker();token=t.request(1_000_000_000)
    assert t.response((9,1),5,token,1_002_000_000)['reason']=='wrong_source'
    assert t.response((1,1),5,token,1_002_000_000,(4,5))['reason']=='wrong_target'
    assert t.response((1,1),5,token+10,1_002_000_000)['reason']=='unmatched_or_duplicate_token'
    r=t.response((1,1),501_000_000,token,1_002_000_000)
    assert r['accepted'] and r['remote_minus_host_ns']==-500_000_000
    assert r['half_round_trip_proxy_ns']==1_000_000
    assert t.response((1,1),5,token,1_002_000_000)['reason']=='unmatched_or_duplicate_token'


def test_timesync_rtt_expiry_and_reboot_start_new_segment():
    t=TimeSyncTracker();token=t.request(1_000_000_000)
    assert not t.response((1,1),5,token,1_030_000_000)['accepted']
    token=t.request(2_000_000_000);r=t.response((1,1),1_001_000_000,token,2_002_000_000)
    assert r['accepted']
    token=t.request(3_000_000_000);r=t.response((1,1),50_000_000,token,3_002_000_000)
    assert r['segment_id']==1 and r['clock_discontinuity'] and r['state']=='warming_up'
    old=t.request(4_000_000_000);t.request(7_000_000_000)
    assert old in t.expired and old not in t.pending


class Backend:
    def __init__(self):self.values=profile_values();self.values['CFG_RATE_MEAS']=40;self.calls=[];self.bad_read=False
    def identity(self):return {'model':'fake X20','transport_device':'fake-unique-port'}
    def read(self,keys):
        values={k:self.values[k] for k in keys}
        if self.bad_read:self.bad_read=False;values['CFG_RATE_MEAS']=999
        return values
    def set_ram(self,values):self.calls.append(dict(values));self.values.update(values)


def test_receiver_transaction_only_applies_ram_after_baseline_and_restores(tmp_path):
    b=Backend();desired=profile_values();before=snapshot(b,list(desired));j=tmp_path/'transaction.json'
    r=apply_ram(b,before,desired,j)
    assert r['status']=='applied_and_verified' and r['layer']=='RAM only'
    r2=restore_ram(b,r,tmp_path/'restore.json')
    assert r2['status']=='restored_and_verified' and b.values==before['ram']
    assert len(b.calls)==2


def test_stale_receiver_baseline_never_writes(tmp_path):
    b=Backend();desired=profile_values();before=snapshot(b,list(desired));b.values['CFG_RATE_MEAS']=100
    with pytest.raises(RuntimeError,match='Stale'):apply_ram(b,before,desired,tmp_path/'j.json')
    assert b.calls==[] and not (tmp_path/'j.json').exists()


def test_failed_readback_rolls_back_to_exact_saved_values(tmp_path):
    class BadRead(Backend):
        def set_ram(self,values):
            super().set_ram(values)
            if len(self.calls)==1:self.bad_read=True
    b=BadRead();desired=profile_values();before=snapshot(b,list(desired));j=tmp_path/'j.json'
    with pytest.raises(RuntimeError,match='rolled_back'):apply_ram(b,before,desired,j)
    assert b.values==before['ram'] and json.loads(j.read_text())['status']=='rolled_back'


def test_profile_rejects_unreviewed_or_destructive_keys(tmp_path):
    p=tmp_path/'p.json'
    for settings in [{'CFG_UART1_BAUDRATE':9600},{'CFG_MSGOUT_UBX_RXM_RAWX_UART1':0},{'CFG_RATE_MEAS':1}]:
        p.write_text(json.dumps({'settings':settings}))
        with pytest.raises(ValueError):profile_values(p)


def test_udp_datagram_boundaries_and_empty_datagram_are_preserved(tmp_path):
    s=Session(tmp_path,expected_streams=('mavlink',))
    worker=MavlinkCapture(('127.0.0.1',0),s.writers['mavlink'],s.stop,s.issues)
    s.workers=[worker];worker.start()
    wait_for(lambda:worker.sock is not None and worker.sock.getsockname()[1]>0)
    source=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    data=[b'',b'unknown opaque datagram',b'\0\xff\x01']
    for d in data:source.sendto(d,worker.sock.getsockname())
    wait_for(lambda:s.writers['mavlink'].stats()['input_chunks']==3)
    source.close();m=s.finish()
    with (s.path/'mavlink_receive.csv').open() as f:rows=list(csv.DictReader(f))
    assert [int(r['length']) for r in rows]==[len(d) for d in data]
    assert (s.path/'mavlink.bin').read_bytes()==b''.join(data)
    assert m['status']=='complete'


def test_enabled_timesync_roundtrip_uses_only_local_fake_peer(tmp_path):
    from pymavlink.dialects.v20 import common as mav
    s=Session(tmp_path,expected_streams=('mavlink',))
    worker=MavlinkCapture(('127.0.0.1',0),s.writers['mavlink'],s.stop,s.issues,
        lambda sock:TimeSyncObserver(s.path,sock,s.stop,s.issues,active=True))
    s.workers=[worker];worker.start();wait_for(lambda:worker.sock is not None and worker.sock.getsockname()[1]>0)
    peer=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);peer.bind(('127.0.0.1',0));peer.settimeout(.3)
    encoder=mav.MAVLink(None,srcSystem=1,srcComponent=1);decoder=mav.MAVLink(None)
    peer.sendto(mav.MAVLink_heartbeat_message(2,12,0,0,3,3).pack(encoder),worker.sock.getsockname())
    end=time.monotonic()+.65
    while time.monotonic()<end:
        try:raw,address=peer.recvfrom(65535)
        except socket.timeout:continue
        for msg in decoder.parse_buffer(raw) or []:
            if msg.get_type()=='TIMESYNC' and msg.tc1==0:
                peer.sendto(mav.MAVLink_timesync_message(time.monotonic_ns()-500_000_000,msg.ts1).pack(encoder),address)
    peer.close();s.finish()
    rows=[json.loads(line) for line in (s.path/'clock_sync.jsonl').read_text().splitlines()]
    accepted=[r for r in rows if r.get('accepted')]
    assert accepted and all(abs(r['remote_minus_host_ns']+500_000_000)<r['half_round_trip_proxy_ns']+2_000_000 for r in accepted)


def test_sigint_finalizes_live_session_without_requiring_drone(tmp_path):
    import pty
    master,slave=pty.openpty();device=os.ttyname(slave)
    root=Path(__file__).resolve().parents[2]
    sock=socket.socket();sock.bind(('127.0.0.1',0));port=sock.getsockname()[1];sock.close()
    command=[sys.executable,'-m','detect2.acquisition.capture','--gps-device',device,
             '--mavlink-bind',f'127.0.0.1:{port}','--output-root',str(tmp_path)]
    proc=subprocess.Popen(command,cwd=root,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    try:
        wait_for(lambda:bool(list(tmp_path.glob('*/manifest.json'))))
        path=next(tmp_path.iterdir());time.sleep(.15)
        for i in range(5):os.write(master,b'raw-test-'+bytes([i]));time.sleep(.02)
        def wrote():
            try:return json.loads((path/'health.jsonl').read_text().splitlines()[-1])['streams']['receiver']['written_bytes']>0
            except (OSError,ValueError,IndexError):return False
        wait_for(wrote)
        proc.send_signal(signal.SIGINT);stdout,stderr=proc.communicate(timeout=5)
        assert proc.returncode==130,(stdout,stderr)
        m=json.loads((path/'manifest.json').read_text())
        assert m['stop_reason']=='sigint' and not (path/'UNFINALIZED').exists()
        assert m['streams']['receiver']['writer_closed'] and m['streams']['receiver']['written_bytes']>0
    finally:
        if proc.poll() is None:proc.kill();proc.wait()
        os.close(master);os.close(slave)
