"""Configure at runtime, record original streams and collect the native ULog.

No startup files, receiver flash/BBR writes, router starts, reboot or arming.
PX4 logging parameters may be autosaved by PX4 if correction is necessary.
"""
import argparse
from datetime import datetime,timezone
import json
import math
from pathlib import Path
import re
import signal
import socket
import threading
import time

from .baudrate import BaudBackend,KEY,transition
from .capture import Session,DEFAULT_GPS
from .common import atomic_json,fingerprint
from .live_health import LiveHealth
from .mavlink_capture import MavlinkCapture
from .ownership import CaptureLocks
from .px4_logging import PX4Link,ArmedError,LOG_KEYS,DESIRED,apply_logging
from .receiver_config import profile_values,snapshot,apply_ram
from .serial_capture import SerialCapture

PROFILE=Path(__file__).with_name('runtime_profile.json')
SIGNALS=['CFG_SIGNAL_'+s+'_ENA' for s in ['GPS','GAL','BDS','GLO','QZSS','SBAS','NAVIC']]
START_LOGGER='logger start -b 128 -t -m all -f'


def bind_address(text):
    host,port=text.rsplit(':',1);port=int(port)
    if host!='127.0.0.1' or not 0<port<65536:raise ValueError('Use a loopback router endpoint HOST:PORT')
    return host,port


def probe_endpoint(bind):
    # Prove no older/non-cooperating process owns the endpoint before any writes.
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:sock.bind(bind)


def native_path(status):
    paths=re.findall(r'(/fs/microsd/log/[^\s\x1b]+\.ulg)',status)
    return paths[0] if paths and 'Full File Logging Running' in status else None


def configure_receiver(folder,device,backend_factory=BaudBackend):
    desired=profile_values();keys=list(desired)+[KEY]+SIGNALS;attempts=[]
    for rate in [2000000,921600]:
        backend=None
        try:
            backend=backend_factory(device,rate)
            before=snapshot(backend,keys)
        except Exception as error:
            attempts.append({'baud':rate,'error':str(error)})
            if backend:backend.close()
            continue
        try:
            reported=before['identity']['reported_receiver'];values=set(reported.values())
            if not {'MOD=ZED-X20P','FWVER=HPG 2.00','PROTVER=50.02'}<=values:
                raise RuntimeError('Unexpected receiver identity; no configuration write')
            if before['ram'][KEY]!=rate:raise RuntimeError('Receiver/host baud mismatch; no write')
            atomic_json(folder/'receiver_before.json',before)
            atomic_json(folder/'receiver_probe_attempts.json',attempts)
            if any(before['ram'][k]!=v for k,v in desired.items()):
                apply_ram(backend,before,desired,folder/'receiver_transaction.json')
            if rate!=2000000:transition(backend,rate,2000000,folder/'baud_transaction.json')
            after=snapshot(backend,keys)
            if after['identity']!=before['identity'] or after['ram'][KEY]!=2000000:
                raise RuntimeError('Receiver identity/baud verification failed')
            if any(after['ram'][k]!=v for k,v in desired.items()) or any(after['ram'][k]!=before['ram'][k] for k in SIGNALS):
                raise RuntimeError('Receiver profile or constellation readback mismatch')
            atomic_json(folder/'receiver_after.json',after)
            return before,after
        finally:backend.close()
    atomic_json(folder/'receiver_probe_attempts.json',attempts)
    raise RuntimeError('Receiver did not reply at 2 Mbaud or 921600; no profile was inferred')


def verify_px4(client,profile):
    client.assert_disarmed();identity=client.identity()
    if identity['uid']!=profile['px4_uid']:raise RuntimeError('PX4 UID differs from the reviewed aircraft')
    firmware=client.shell('ver all',3)
    if not re.search(r'PX4 git-hash:\s*'+re.escape(profile['px4_firmware_hash'])+r'\b',firmware):
        raise RuntimeError('PX4 firmware differs; review logger commands before use')
    return identity,firmware


def start_native(client,folder,identity):
    baseline={'identity':identity,'parameters':client.read_params(LOG_KEYS)}
    atomic_json(folder/'px4_before.json',baseline)
    if baseline['parameters']!=DESIRED:apply_logging(client,baseline,folder/'px4_transaction.json')
    previous=client.shell('logger status',3);(folder/'previous_logger.txt').write_text(previous)
    (folder/'logger_stop_before.txt').write_text(client.shell('logger stop',3))
    memory=client.shell('free',3);(folder/'memory_before_logger.txt').write_text(memory)
    line=next((s for s in memory.splitlines() if 'Umem:' in s),'')
    if not line or int(line.split()[4])<196608:raise RuntimeError('Insufficient contiguous MCU memory for reviewed logger buffer')
    (folder/'logger_start.txt').write_text(client.shell(START_LOGGER,3))
    try:
        status=client.shell('logger status',3);(folder/'logger_recording.txt').write_text(status)
        path=native_path(status)
        if not path or '/ 131072 B' not in status:raise RuntimeError('Native logger path or 128 KiB buffer was not verified')
        if client.read_params(LOG_KEYS)!=DESIRED:raise RuntimeError('PX4 logging parameter verification failed')
        atomic_json(folder/'px4_after.json',{'identity':client.identity(),'parameters':DESIRED})
    except Exception:
        try:
            result=client.shell('logger stop',3)
            atomic_json(folder/'failed_native_setup_cleanup.json',{'stop_reply':result})
        except Exception as cleanup_error:
            atomic_json(folder/'failed_native_setup_cleanup.json',{'error':str(cleanup_error),'native_state':'UNKNOWN'})
        raise
    return path


def save_runtime(folder,state):
    state['updated_utc']=datetime.now(timezone.utc).isoformat()
    atomic_json(folder/'runtime_manifest.json',state)


def collect_native(folder,state,client_factory=PX4Link):
    """Never stops an armed vehicle's log or a different/newer recording."""
    client=None
    try:
        client=client_factory('udpin:'+state['mavlink_bind'])
        identity=client.identity()
        if identity!=state['px4_identity']:raise RuntimeError('PX4 identity changed; native collection refused')
        status=client.shell('logger status',3);(folder/'logger_at_stop.txt').write_text(status)
        active=native_path(status)
        if active==state['native_path']:
            (folder/'logger_close.txt').write_text(client.shell('logger stop',3))
            stopped=client.shell('logger status',3);(folder/'logger_closed_status.txt').write_text(stopped)
            if 'not running' not in stopped:raise RuntimeError('Native log was not confirmed closed')
        elif active is not None:
            # The bookmarked log is closed; preserve a newer independent log.
            state['newer_native_log_left_running']=active
        elif 'not running' not in status and 'Not logging' not in status:
            raise RuntimeError('Unrecognized logger state; no successful closure inferred')
        path=folder/'native.ulg';i=0
        while path.exists() or path.with_suffix('.ftp.part').exists():
            i+=1;path=folder/f'native_{i:03}.ulg'
        download=client.download(state['native_path'],path)
        atomic_json(folder/'native_download.json',download)
        state['native_file']=path.name;state['native_fingerprint']=download['fingerprint']
        state['native_collection']='collected';state.pop('native_collection_error',None)
        drops=re.search(r'dropouts:\s*(\d+)',status)
        if active==state['native_path'] and drops and int(drops.group(1)):
            state['native_logger_dropout_count']=int(drops.group(1))
    except (Exception,KeyboardInterrupt) as error:
        state['native_collection']='pending'
        state['native_collection_error']=str(error) or 'Native collection interrupted; retry --finalize'
        return False
    finally:
        if client:client.close()
    return True


def final_status(folder,state):
    raw=json.loads((folder/'manifest.json').read_text())
    if state.get('native_collection')!='collected':state['status']='pending_native'
    elif raw['status']=='failed' or state.get('errors') or state.get('native_logger_dropout_count'):
        state['status']='failed'
    elif not state.get('recording_ready'):state['status']='incomplete'
    elif raw['status']=='complete':state['status']='complete'
    elif raw['status']=='stopped':state['status']='stopped'
    else:state['status']='incomplete'
    state['files']={p.name:fingerprint(p) for p in folder.iterdir() if p.is_file() and p.name not in
                    ['runtime_manifest.json','RUNTIME_UNFINALIZED','recording_report.html','report.html',
                     'audit.json','receiver_observations.csv'] and not p.name.endswith(('.tmp','.ftp.part'))}
    save_runtime(folder,state)
    if state.get('native_collection')=='collected' and not (folder/'UNFINALIZED').exists():
        (folder/'RUNTIME_UNFINALIZED').unlink(missing_ok=True)
    try:
        from .runtime_report import report
        report(folder)
    except Exception as error:
        state['report_error']=str(error);save_runtime(folder,state)
        print('Report generation failed:',str(error),flush=True)
    return 0 if state['status'] in ['complete','stopped'] else 3 if state['status']=='pending_native' else 2


def run(root,device,bind,profile,duration=None,stop=None,reason=None,backend_factory=BaudBackend,
        client_factory=PX4Link,serial_factory=SerialCapture,mavlink_factory=MavlinkCapture,timesync=False):
    stop=stop or threading.Event();reason=reason if reason is not None else []
    session=Session(root,metadata={'gps_device':device,'gps_baud':2000000,'expected_rawx_hz':20,
        'mavlink_bind':list(bind),'timesync_active':timesync,'runtime_launcher':True,
        'serial_reconnect_policy':'stop and mark failed; reconfigure only in a new session',
        'gnss_to_px4_clock_verified':False})
    folder=session.path;(folder/'RUNTIME_UNFINALIZED').write_text('Runtime acquisition has not fully finalized.\n')
    state={'schema_version':1,'status':'preparing','mavlink_bind':f'{bind[0]}:{bind[1]}','profile':profile,
        'errors':[],'recording_ready':False,'native_collection':'not_started','startup_files_changed':False,
        'receiver_persistent_writes':False,'px4_parameter_autosave':'possible if logging parameters need correction',
        'accepted_native_imu_hz':189.6,'gnss_to_px4_clock':'UNKNOWN','timesync_active':timesync}
    save_runtime(folder,state);client=None;live=None
    print('Session:',folder.resolve(),flush=True)
    try:
        client=client_factory('udpin:'+state['mavlink_bind'])
        identity,firmware=verify_px4(client,profile);state['px4_identity']=identity
        (folder/'firmware.txt').write_text(firmware)
        if stop.is_set():raise InterruptedError('Stopped during preparation')
        before,after=configure_receiver(folder,device,backend_factory)
        state['receiver_identity']=after['identity']
        if stop.is_set():raise InterruptedError('Stopped during preparation')
        state['native_path']=start_native(client,folder,identity);save_runtime(folder,state)
        session.manifest['configuration_changed']=True
        session.manifest['metadata']['receiver_profile']='verified: receiver_after.json'
        session.manifest['metadata']['native_log_bookmark']=state['native_path']
        atomic_json(folder/'manifest.json',session.manifest)
        client.close();client=None
        live=LiveHealth(session.stop,session.issues)
        def observer_factory(sock):
            if not timesync:return live
            from .timesync import TimeSyncObserver,CombinedObserver
            return CombinedObserver(live,TimeSyncObserver(folder,sock,session.stop,session.issues,active=True))
        session.workers=[serial_factory(device,2000000,session.writers['receiver'],session.stop,session.issues,
                                      reconnect=False,observer=live.receiver),
                         mavlink_factory(bind,session.writers['mavlink'],session.stop,session.issues,
                                         observer_factory=observer_factory)]
        started=time.monotonic_ns();last_health=0;ready_at=None
        for worker in session.workers:worker.start()
        while not stop.is_set() and not session.stop.is_set():
            now=time.monotonic_ns();health=live.check(started,now)
            if health['ready'] and ready_at is None:
                ready_at=now;state.update(status='recording',recording_ready=True,ready_monotonic_ns=now)
                save_runtime(folder,state)
                print('RECORDING READY | RAWX 20 Hz, 2 Mbaud | quaternion + IMU present | native buffer 128 KiB',flush=True)
            if now-last_health>=1e9:
                session.health();atomic_json(folder/'live_health.json',health);last_health=now
                if session.issues.snapshot()['counts']:session.stop.set()
            if duration is not None and ready_at is not None and (now-ready_at)/1e9>=duration:break
            stop.wait(.05)
        state['live_health']=live.check(started)
    except Exception as error:
        state['errors'].append(str(error))
        session.issues.add('runtime_preparation_or_capture_error',error=str(error))
    finally:
        if client:client.close()
        why=reason[-1] if reason else 'error' if state['errors'] or session.issues.snapshot()['counts'] else 'duration'
        session.finish(why)
    if state.get('native_path'):
        state['status']='collecting_native';save_runtime(folder,state)
        print('Raw capture finalized; collecting native log (requires disarmed PX4).',flush=True)
        collect_native(folder,state,client_factory)
    else:
        state['native_collection']='not_started';state['status']='failed'
        save_runtime(folder,state)
        (folder/'RUNTIME_UNFINALIZED').unlink(missing_ok=True)
        print('Preparation failed; see runtime_manifest.json',flush=True)
        return folder,2
    code=final_status(folder,state)
    print('Runtime:',state['status'],'| Session:',folder.resolve(),flush=True)
    if code==3:print('After disarm: ./start_recording --finalize '+str(folder.resolve()),flush=True)
    return folder,130 if code==0 and reason and reason[-1]=='sigint' else 143 if code==0 and reason and reason[-1]=='sigterm' else code


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output-root',type=Path,default=Path('recordings_v2'))
    p.add_argument('--duration-s',type=float,help='Recording seconds after required streams are ready; default: until Ctrl+C')
    p.add_argument('--gps-device',default=DEFAULT_GPS)
    p.add_argument('--mavlink-bind',default='127.0.0.1:14551')
    p.add_argument('--timesync',action='store_true',help='Estimate Pi/PX4 clocks with addressed TIMESYNC; GNSS clock mapping remains unverified')
    p.add_argument('--finalize',type=Path,help='Collect the bookmarked native log after disarm; no receiver reconfiguration')
    a=p.parse_args()
    if a.duration_s is not None and (not math.isfinite(a.duration_s) or a.duration_s<=0):p.error('Duration must be finite and positive')
    try:bind=bind_address(a.mavlink_bind)
    except ValueError as error:p.error(str(error))
    profile=json.loads(PROFILE.read_text());stop=threading.Event();reason=[]
    def interrupted(signum,frame):
        if reason:raise KeyboardInterrupt
        reason.append('sigint' if signum==signal.SIGINT else 'sigterm')
        stop.set()
    signal.signal(signal.SIGINT,interrupted);signal.signal(signal.SIGTERM,interrupted)
    try:
        if a.finalize:
            folder=a.finalize.resolve();state=json.loads((folder/'runtime_manifest.json').read_text())
            bind=bind_address(state['mavlink_bind'])
            with CaptureLocks(None,bind):
                probe_endpoint(bind)
                if state.get('native_collection')=='collected':
                    path=folder/state['native_file']
                    if fingerprint(path)!=state['native_fingerprint']:raise RuntimeError('Stored native file was modified')
                else:collect_native(folder,state)
                code=final_status(folder,state)
        else:
            with CaptureLocks(a.gps_device,bind):
                probe_endpoint(bind)
                folder,code=run(a.output_root,a.gps_device,bind,profile,a.duration_s,stop,reason,timesync=a.timesync)
    except Exception as error:
        print('Runtime launcher:',str(error),flush=True);code=2
    raise SystemExit(code)


if __name__=='__main__':main()
