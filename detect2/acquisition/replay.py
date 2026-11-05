"""Deterministic, offline inputs through the same raw archive writer."""
import argparse
import csv
import json
from pathlib import Path
import struct
import time
import math
from itertools import cycle

from .capture import Session
from .common import fingerprint,atomic_json
from .ubx import packet


def synthetic_inputs(faults=True):
    from pymavlink.dialects.v20 import common as mav
    receiver=[];datagrams=[];encoder=mav.MAVLink(None,srcSystem=1,srcComponent=1)
    for epoch in range(40):
        # One missing epoch, one receiver clock-reset flag, invalid phase flags,
        # and multiple constellations/frequencies. They must all survive capture.
        if faults and epoch==12:continue
        status=3 if faults and epoch==25 else 1
        header=struct.pack('<dHbBBB2s',221000+epoch*.05,2426,18,4,status,1,b'\0\0')
        observations=b''
        for j,(gnss,sv,sig) in enumerate([(0,3,0),(0,3,7),(2,11,0),(3,7,5)]):
            flags=5 if faults and epoch==8 and j==1 else 15
            observations+=struct.pack('<ddfBBBBHBBBBBB',2e7,1e8+epoch*.01+j,-30.,gnss,sv,sig,0,
                                      epoch*50,45,2,3,4,flags,0)
        receiver.append(packet(2,21,header+observations))
        msg=mav.MAVLink_attitude_quaternion_message(epoch*50,1.,0.,0.,0.,0.,0.,0.)
        raw=bytes(msg.pack(encoder));encoder.seq=(encoder.seq+1)%256
        imu=mav.MAVLink_highres_imu_message(epoch*50_000,0.,0.,-9.80665,0.,0.,0.,0.,0.,0.,1013.,0.,0.,20.,63)
        datagrams.append(raw+bytes(imu.pack(encoder)));encoder.seq=(encoder.seq+1)%256
    receiver.insert(4,packet(0x99,0x55,b'unknown-message-preserved'))
    # A simulated MAVLink interruption excludes middle datagrams but not GNSS.
    if faults:datagrams=[d for i,d in enumerate(datagrams) if not 10<=i<20]
    return b''.join(receiver),datagrams


def replay(root,receiver_bytes,datagrams=(),write_delay_s=0.,queue_chunks=128,queue_bytes=8*1024*1024,
           expected_hz=None,injections=None,source_paths=None,reason='replay_complete'):
    session=Session(root,mode='replay',write_delay_s=write_delay_s,queue_chunks=queue_chunks,queue_bytes=queue_bytes,
                    metadata={'expected_rawx_hz':expected_hz,'replay_injections':injections or {},
                              'timesync_active':False,'gnss_to_px4_clock_verified':False})
    fixture=session.path/'replay_inputs';fixture.mkdir()
    receiver_source=fixture/'receiver.bin';receiver_source.write_bytes(receiver_bytes)
    mav_source=fixture/'mavlink.bin';mav_source.write_bytes(b''.join(datagrams))
    session.manifest['replay_sources']={n:{'path':str(p.relative_to(session.path)),'fingerprint':fingerprint(p)}
        for n,p in [('receiver',receiver_source),('mavlink',mav_source)]}
    session.manifest['metadata']['external_source_paths']=source_paths or {}
    # Different chunk boundaries exercise framing independence. Simulation times
    # are explicit; wall-clock queue age is measured separately by the writer.
    offset=0;clock=1_000_000_000
    for size in cycle([1,7,101,4096,8192]):
        raw=receiver_bytes[offset:offset+size]
        if not raw:break
        if not write_delay_s:
            while session.writers['receiver'].queue.qsize()>queue_chunks//2:time.sleep(.001)
        session.writers['receiver'].submit(raw,clock);offset+=len(raw);clock+=1_000_000
    for i,data in enumerate(datagrams):
        if not write_delay_s:
            while session.writers['mavlink'].queue.qsize()>queue_chunks//2:time.sleep(.001)
        session.writers['mavlink'].submit(data,1_000_000_000+i*50_000_000,peer=('127.0.0.1',14551))
    session.health();session.finish(reason)
    from .audit_session import audit
    audit(session.path)
    return session.path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output-root',type=Path,default=Path('recordings_v2'))
    p.add_argument('--receiver',type=Path,help='Replay saved receiver bytes, without modifying them')
    p.add_argument('--mavlink',type=Path);p.add_argument('--mavlink-index',type=Path)
    p.add_argument('--writer-delay-ms',type=float,default=0);p.add_argument('--queue-chunks',type=int,default=128)
    p.add_argument('--expected-rawx-hz',type=float)
    p.add_argument('--scenario',choices=['faults','healthy','missing-mavlink','overflow'],default='faults')
    a=p.parse_args()
    if a.queue_chunks<1 or not math.isfinite(a.writer_delay_ms) or a.writer_delay_ms<0:p.error('Invalid queue/delay')
    if a.expected_rawx_hz is not None and (not math.isfinite(a.expected_rawx_hz) or a.expected_rawx_hz<=0):p.error('Expected rate must be positive')
    if a.receiver:
        raw=a.receiver.read_bytes();datagrams=[]
        if a.mavlink and a.mavlink_index:
            with a.mavlink.open('rb') as f,a.mavlink_index.open() as ix:
                for row in csv.DictReader(ix):f.seek(int(row['byte_offset']));datagrams.append(f.read(int(row['length'])))
        elif a.mavlink:p.error('Recorded MAVLink replay needs --mavlink-index to preserve boundaries')
        injections={'scenario':'recorded input; no synthetic repair or resampling'}
    else:
        raw,datagrams=synthetic_inputs(faults=a.scenario=='faults')
        injections={'scenario':'synthetic faults','missing_rawx_epoch':12,'invalid_carrier_flag_epoch':8,
                    'clock_reset_epoch':25,'unknown_ubx_message':True,'mavlink_interruption':True}
        if a.scenario!='faults':injections={'scenario':a.scenario}
        if a.scenario=='missing-mavlink':datagrams=[]
        if a.scenario=='overflow':a.writer_delay_ms=max(100,a.writer_delay_ms);a.queue_chunks=1
    path=replay(a.output_root,raw,datagrams,a.writer_delay_ms/1000,a.queue_chunks,
                expected_hz=a.expected_rawx_hz or (20. if not a.receiver else None),injections=injections,
                source_paths={'receiver':str(a.receiver),'mavlink':str(a.mavlink)})
    print('Replay session:',path.resolve())


if __name__=='__main__':main()
