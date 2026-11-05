import csv
import json
import socket
import threading
import time
import pytest

from pymavlink.dialects.v20 import common as mav

from SpoofingDetection.detect2.acquisition.clock_report import report, summarize_segment, map_host_to_px4
from SpoofingDetection.detect2.acquisition.common import Issues
from SpoofingDetection.detect2.acquisition.timesync import TimeSyncObserver, message_target, pack_timesync


def test_extension_targets_survive_legacy_sdk_and_crc_check():
    encoder=mav.MAVLink(None,srcSystem=245,srcComponent=191)
    raw=pack_timesync(encoder,0,123456789,(1,1))
    msg=mav.MAVLink(None).parse_char(raw)
    assert msg.get_type()=='TIMESYNC' and msg.ts1==123456789
    assert message_target(msg)==(1,1)
    legacy=mav.MAVLink_timesync_message(42,123).pack(encoder)
    assert message_target(mav.MAVLink(None).parse_char(legacy))==(0,0)


def test_mapping_holdout_recovers_known_drift_and_keeps_outliers():
    rows=[];base=9_000_000_000_000
    for i in range(300):
        ns=base+i*100_000_000
        rows.append(dict(host_midpoint_ns=ns,remote_minus_host_ns=-8_000_000_000+2000*i,
                         round_trip_ns=1_000_000+i%10*1000,ts1_ns=ns))
    # 20 ppm: offset increases by 2,000 ns per 100 ms.
    rows[50]['remote_minus_host_ns']+=5_000_000
    s,points=summarize_segment(0,rows)
    assert abs(s['drift_ppm']-20)<1e-6
    assert s['held_out_abs_residual_p95_ms']<1e-6
    assert s['training_outlier_bins']==1
    assert len(points)==len(rows) and any(p['role']=='train_outlier' for p in points)
    assert not s['extrapolation_allowed'] and not s['independent_accuracy_verified']
    summary={'segments':[s]}
    assert map_host_to_px4(summary,base+10_000_000_000,0)==base+10_000_000_000-8_000_000_000+200_000
    with pytest.raises(ValueError):map_host_to_px4(summary,base-1,0)
    with pytest.raises(ValueError):map_host_to_px4(summary,base,1)


def test_observer_pause_starts_segment_and_archives_exact_transmit_bytes(tmp_path):
    host=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);host.bind(('127.0.0.1',0))
    peer=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);peer.bind(('127.0.0.1',0));peer.settimeout(.05)
    stop=threading.Event();issues=Issues()
    observer=TimeSyncObserver(tmp_path,host,stop,issues,active=True,gap_ns=200_000_000)
    encoder=mav.MAVLink(None,srcSystem=1,srcComponent=1)
    observer.start();observer.observe(mav.MAVLink_heartbeat_message(2,12,0,0,3,3).pack(encoder),time.monotonic_ns(),peer.getsockname())
    start=time.monotonic();late=None
    try:
        while time.monotonic()-start<1.2:
            try:raw,_=peer.recvfrom(65535)
            except socket.timeout:continue
            msg=mav.MAVLink(None).parse_char(raw)
            if msg.tc1!=0:continue
            elapsed=time.monotonic()-start
            if .2<elapsed<.65:
                if late is None:late=msg.ts1
                continue
            if late is not None:
                observer.observe(pack_timesync(encoder,time.monotonic_ns()-500_000_000,late,(245,191)),time.monotonic_ns(),peer.getsockname());late=None
            observer.observe(pack_timesync(encoder,time.monotonic_ns()-500_000_000,msg.ts1,(245,191)),time.monotonic_ns(),peer.getsockname())
    finally:
        stop.set();observer.finished.set();observer.join(2);host.close();peer.close()
    assert not observer.is_alive() and not issues.snapshot()['counts']
    events=[json.loads(l) for l in (tmp_path/'clock_sync.jsonl').read_text().splitlines()]
    assert {r['segment_id'] for r in events if r.get('accepted')}=={0,1}
    assert any(r.get('reason')=='accepted_reply_gap' for r in events)
    assert any(r.get('reason')=='unmatched_or_duplicate_token' for r in events)
    raw=(tmp_path/'mavlink_tx.bin').read_bytes();offset=0
    with (tmp_path/'mavlink_tx.csv').open() as f:
        for r in csv.DictReader(f):
            assert int(r['byte_offset'])==offset
            end=offset+int(r['length']);msg=mav.MAVLink(None).parse_char(raw[offset:end])
            assert msg.ts1==int(r['ts1_ns']) and message_target(msg)==(1,1)
            offset=end
    assert offset==len(raw)
    summary=report(tmp_path)
    assert summary['addressing']=={'addressed':summary['accepted_replies']}
    assert summary['gnss_to_px4_clock']=='UNKNOWN'
