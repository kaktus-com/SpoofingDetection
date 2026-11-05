"""Bench-only UDP pause/resume test of the TIMESYNC recorder.

Run only after other recorders stop. Relays the existing 14551 router endpoint
to a separate local recorder port, dropping both directions for a marked pause.
No buffering/replay, router restart, USB removal or aircraft control is used.
"""
import argparse
import json
from pathlib import Path
import signal
import socket
import subprocess
import time


def run(launcher, output, review, pause_s=1.5, duration_s=35):
    if not 1.1<=pause_s<=1.8:raise ValueError('Pause must remain below the 2.5 s acquisition watchdog')
    review=Path(review);review.mkdir(parents=True,exist_ok=True)
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.bind(('127.0.0.1',14551));sock.settimeout(.01)
    child_address=('127.0.0.1',14561);router=None;proc=None;events=[];counts={};ready=None;pause_start=None;pause_end=None
    path=review/'transport_pause.json';started=time.monotonic_ns()
    def save(status):
        tmp=path.with_suffix('.tmp')
        tmp.write_text(json.dumps({'status':status,'test':'controlled UDP transport pause/resume; not a physical reconnect',
            'started_host_ns':started,'events':events,'counts':counts,'pause_start_host_ns':pause_start,
            'pause_end_host_ns':pause_end,'requested_pause_s':pause_s,'router_peer':router,
            'watchdog_unchanged_s':2.5,'buffering':False,'child_exit_code':proc.poll() if proc else None},indent=2)+'\n')
        tmp.replace(path)
    try:
        with (review/'pause_console.log').open('wb') as log:
            proc=subprocess.Popen([str(Path(launcher).resolve()),'--timesync','--duration-s',str(duration_s),
                '--mavlink-bind','127.0.0.1:14561','--output-root',str(output.resolve())],stdout=log,stderr=subprocess.STDOUT)
            save('running')
            while proc.poll() is None:
                now=time.monotonic_ns()
                if now-started>600_000_000_000:raise TimeoutError('Bench process exceeded 10 minutes')
                if ready is None:
                    for p in output.glob('*/runtime_manifest.json'):
                        state=json.loads(p.read_text())
                        if state.get('recording_ready'):ready=state['ready_monotonic_ns']
                if ready and pause_start is None and now-ready>=10_000_000_000:
                    pause_start=now;events.append({'event':'pause_start','host_monotonic_ns':now});save('paused')
                if pause_start and pause_end is None and now-pause_start>=int(pause_s*1e9):
                    pause_end=now;events.append({'event':'pause_end','host_monotonic_ns':now});save('resumed')
                try:data,peer=sock.recvfrom(65535)
                except socket.timeout:continue
                direction='to_router' if peer==child_address else 'to_recorder'
                if direction=='to_recorder':router=peer
                target=router if direction=='to_router' else child_address
                dropping=pause_start is not None and pause_end is None
                key=direction+('_dropped' if dropping else '_forwarded')
                counts[key]=counts.get(key,0)+1
                if target and not dropping:sock.sendto(data,target)
            save('complete' if proc.returncode==0 and pause_end else 'failed')
    except BaseException as error:
        events.append({'event':'harness_error','error':str(error),'host_monotonic_ns':time.monotonic_ns()})
        if proc and proc.poll() is None:proc.send_signal(signal.SIGINT)
        save('failed');raise
    finally:sock.close()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--launcher',type=Path,required=True);p.add_argument('--output-root',type=Path,required=True)
    p.add_argument('--review',type=Path,required=True);a=p.parse_args()
    run(a.launcher,a.output_root,a.review)
