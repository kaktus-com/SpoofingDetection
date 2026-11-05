"""Independent raw GNSS/MAVLink recorder. No receiver or PX4 settings are changed."""
import argparse
from datetime import datetime,timezone
import importlib.metadata
import json
import csv
import math
import os
from pathlib import Path
import platform
import signal
import socket
import threading
import time
import uuid

from .common import ArchiveWriter,Issues,atomic_json,fingerprint

DEFAULT_GPS='/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DQ00KK6P-if00-port0'


class Session:
    def __init__(self, root, mode='live', queue_chunks=4096, queue_bytes=8*1024*1024,
                 write_delay_s=0., expected_streams=('receiver','mavlink'), opener=None, metadata=None):
        name=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex[:6]
        self.path=Path(root)/name;self.path.mkdir(parents=True,exist_ok=False)
        (self.path/'UNFINALIZED').write_text('Capture has not finalized. Audit before use.\n')
        self.issues=Issues();self.stop=threading.Event();self.workers=[]
        self.manifest={'schema_version':1,'mode':mode,'status':'recording','started_utc':datetime.now(timezone.utc).isoformat(),
            'host':socket.gethostname(),'platform':platform.platform(),
            'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip() if Path('/proc/sys/kernel/random/boot_id').exists() else None,
            'expected_streams':list(expected_streams),'configuration_changed':False,
            'buffer_limits_per_stream':{'max_chunks':queue_chunks,'max_bytes':queue_bytes},
            'clock_anchors':[{'host_monotonic_ns':time.monotonic_ns(),'unix_ns':time.time_ns()}],
            'packages':{},'source_hashes':{},'metadata':metadata or {}}
        for package in ['pyserial','pymavlink','pyubx2']:
            try:self.manifest['packages'][package]=importlib.metadata.version(package)
            except importlib.metadata.PackageNotFoundError:self.manifest['packages'][package]=None
        for path in sorted(Path(__file__).parent.glob('*.py')):self.manifest['source_hashes'][path.name]=fingerprint(path)['sha256']
        self.writers={}
        atomic_json(self.path/'manifest.json',self.manifest)
        try:
            for name in ['receiver','mavlink']:
                self.writers[name]=ArchiveWriter(self.path,name,self.issues,max_chunks=queue_chunks,
                    max_bytes=queue_bytes,write_delay_s=write_delay_s,opener=opener)
            self.health_file=(self.path/'health.jsonl').open('w')
        except BaseException:
            for writer in self.writers.values():writer.close()
            raise

    def health(self):
        row={'host_monotonic_ns':time.monotonic_ns(),'unix_ns':time.time_ns(),
             'streams':{n:w.stats() for n,w in self.writers.items()},'issues':self.issues.snapshot()['counts']}
        try:self.health_file.write(json.dumps(row)+'\n');self.health_file.flush()
        except OSError as error:
            self.issues.add('health_write_error',error=str(error));self.stop.set()
        return row

    def finish(self, reason='duration'):
        self.stop.set()
        for worker in self.workers:
            worker.join(3)
            if worker.is_alive():self.issues.add('source_shutdown_timeout',worker=worker.name)
        stats={n:w.close() for n,w in self.writers.items()}
        self.health();self.health_file.close()
        clock_rows=[]
        if (self.path/'clock_sync.jsonl').exists():
            try:clock_rows=[json.loads(line) for line in (self.path/'clock_sync.jsonl').read_text().splitlines()]
            except (OSError,ValueError) as error:self.issues.add('clock_export_error',error=str(error))
        fields=list(dict.fromkeys(['event','host_monotonic_ns','accepted','reason','segment_id',
            'source_system','source_component','ts1_ns','tc1_ns','round_trip_ns','remote_minus_host_ns',
            'filtered_offset_ns','half_round_trip_proxy_ns']+[k for r in clock_rows for k in r]))
        with (self.path/'clock_sync.csv').open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(clock_rows)
        issues=self.issues.snapshot()
        missing=[n for n in self.manifest['expected_streams'] if not stats[n]['written_bytes']]
        failed=bool(issues['counts'])
        status='failed' if failed else 'incomplete' if missing else 'stopped' if reason in ['sigint','sigterm'] else 'complete'
        self.manifest.update(status=status,stop_reason=reason,ended_utc=datetime.now(timezone.utc).isoformat(),
            streams=stats,issues=issues,missing_streams=missing,
            files={p.name:fingerprint(p) for p in self.path.iterdir() if p.is_file() and p.name not in
                   ['manifest.json','UNFINALIZED','runtime_manifest.json','RUNTIME_UNFINALIZED'] and not p.name.endswith('.tmp')})
        self.manifest['clock_anchors'].append({'host_monotonic_ns':time.monotonic_ns(),'unix_ns':time.time_ns()})
        atomic_json(self.path/'manifest.json',self.manifest)
        if all(s['writer_closed'] for s in stats.values()):(self.path/'UNFINALIZED').unlink()
        return self.manifest


def parser():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output-root',type=Path,default=Path('recordings_v2'))
    p.add_argument('--duration-s',type=float)
    p.add_argument('--gps-device',default=DEFAULT_GPS);p.add_argument('--gps-baud',type=int,default=921600)
    p.add_argument('--mavlink-bind',default='127.0.0.1:14551')
    p.add_argument('--expected-rawx-hz',type=float,help='User-declared expected rate for gap auditing; not a receiver configuration command')
    p.add_argument('--queue-chunks',type=int,default=4096);p.add_argument('--queue-bytes',type=int,default=8*1024*1024)
    p.add_argument('--timesync',action='store_true',help='Enable clock requests/replies; passive by default. Use only at the timing stage.')
    return p


def main():
    from .ownership import CaptureLocks
    a=parser().parse_args()
    try:host,port=a.mavlink_bind.rsplit(':',1);bind=(host,int(port))
    except (ValueError,AttributeError):parser().error('MAVLink bind requires HOST:PORT')
    with CaptureLocks(a.gps_device,bind):return capture_main()


def capture_main():
    p=parser();a=p.parse_args()
    if a.duration_s is not None and (not math.isfinite(a.duration_s) or a.duration_s<=0):p.error('duration must be finite and positive')
    if min(a.queue_chunks,a.queue_bytes,a.gps_baud)<=0:p.error('limits and baud must be positive')
    if a.expected_rawx_hz is not None and (not math.isfinite(a.expected_rawx_hz) or a.expected_rawx_hz<=0):p.error('Expected rate must be finite and positive')
    try:host,port=a.mavlink_bind.rsplit(':',1);bind=(host,int(port));assert 0<bind[1]<65536
    except (ValueError,AssertionError):p.error('MAVLink bind requires HOST:PORT')
    from .serial_capture import SerialCapture
    from .mavlink_capture import MavlinkCapture
    from .timesync import TimeSyncObserver
    session=Session(a.output_root,queue_chunks=a.queue_chunks,queue_bytes=a.queue_bytes,
        metadata={'gps_device':a.gps_device,'gps_baud':a.gps_baud,'mavlink_bind':list(bind),
                  'receiver_profile':'not queried; UNKNOWN','timesync_active':a.timesync,'expected_rawx_hz':a.expected_rawx_hz,
                  'timestamps':'original host receive chunks/datagrams; source sensor times unchanged',
                  'gnss_to_px4_clock_verified':False})
    serial=SerialCapture(a.gps_device,a.gps_baud,session.writers['receiver'],session.stop,session.issues)
    observer=(lambda sock:TimeSyncObserver(session.path,sock,session.stop,session.issues,active=True)) if a.timesync else None
    udp=MavlinkCapture(bind,session.writers['mavlink'],session.stop,session.issues,observer)
    session.workers=[serial,udp]
    reason='duration';signal_reason=[]
    def stop(signum,frame):signal_reason.append('sigint' if signum==signal.SIGINT else 'sigterm');session.stop.set()
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    print('Session:',session.path.resolve(),flush=True)
    print('GNSS:',a.gps_device,'baud',a.gps_baud,'| MAVLink:',a.mavlink_bind,flush=True)
    print('Receiver profile: UNKNOWN (not queried) | TIMESYNC:', 'active' if a.timesync else 'disabled/passive',flush=True)
    started=time.monotonic()
    try:
        for worker in session.workers:worker.start()
        while not session.stop.is_set():
            session.health()
            if a.duration_s is not None and time.monotonic()-started>=a.duration_s:break
            session.stop.wait(min(1.,max(.01,a.duration_s-(time.monotonic()-started))) if a.duration_s else 1.)
    except BaseException as error:
        session.issues.add('capture_main_error',error=str(error));reason='error'
    if signal_reason:reason=signal_reason[-1]
    manifest=session.finish(reason)
    print('Capture:',manifest['status'],'| Audit: python -m detect2.acquisition.audit_session',session.path,flush=True)
    code=130 if reason=='sigint' else 143 if reason=='sigterm' else 0 if manifest['status']=='complete' else 2
    raise SystemExit(code)


if __name__=='__main__':main()
