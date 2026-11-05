"""Stage 2 supervisor: read-only pre/post checks and two bounded live captures.

Runs on the Pi with its existing venv. No configuration writes, active clock
sync, service changes or router restarts. Signals only its own capture child.
"""
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import sys
import time

RELEASE = Path('/home/nss/SpoofingDetection/releases/acquisition_20261105T100733Z')
ROOT = Path('/home/nss/SpoofingDetection/recordings_v2_stage2/acquisition_20261105T100733Z')
LEGACY = Path('/home/nss/SpoofingDetection')
GPS = '/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DQ00KK6P-if00-port0'
sys.path.insert(0, str(RELEASE))
from detect2.acquisition.receiver_config import SerialBackend, snapshot, profile_values


def save(name, value):
    (ROOT / name).write_text(json.dumps(value, indent=2) + '\n')
    print(name, flush=True)


def state():
    paths = sorted(set(LEGACY.glob('*.py')) | set(LEGACY.glob('README*')) | {LEGACY / 'start'})
    ps = subprocess.check_output(['ps', '-eo', 'pid,args'], text=True)
    return {
        'legacy_files': {str(p.relative_to(LEGACY)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths if p.is_file()},
        'router_processes': [line.strip() for line in ps.splitlines() if '/usr/bin/mavlink-routerd ' in line],
        'device_links': {str(p): str(p.resolve()) for p in Path('/dev/serial/by-id').iterdir()},
        'free_bytes': __import__('shutil').disk_usage(LEGACY).free,
        'packages': {p: importlib.metadata.version(p) for p in ['pyserial', 'pymavlink', 'pyubx2']},
        'udp_sockets': subprocess.check_output(['ss', '-unlp'], text=True),
    }


def receiver_snapshot():
    backend = SerialBackend(GPS, 921600)
    try:
        return snapshot(backend, list(profile_values()))
    finally:
        backend.close()


def px4_snapshot():
    from pymavlink.dialects.v20 import common as mav
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', 14551))
    sock.settimeout(.3)
    parser = mav.MAVLink(None)
    deadline = time.monotonic() + 12
    requested = False
    params = {}
    heartbeats = []
    keys = ['SDLOG_PROFILE', 'SDLOG_MODE', 'SDLOG_BACKEND']
    try:
        while time.monotonic() < deadline:
            try:
                raw, peer = sock.recvfrom(65535)
            except socket.timeout:
                continue
            for msg in parser.parse_buffer(raw) or []:
                if msg.get_type() == 'HEARTBEAT' and msg.get_srcSystem() == 1 and msg.get_srcComponent() == 1:
                    assert not (msg.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED), 'Autopilot armed; stage stopped'
                    heartbeats.append({'system': 1, 'component': 1, 'base_mode': msg.base_mode, 'armed': False})
                    if not requested:
                        encoder = mav.MAVLink(None, srcSystem=245, srcComponent=191)
                        for key in keys:
                            packet = encoder.param_request_read_encode(1, 1, key.encode(), -1)
                            sock.sendto(packet.pack(encoder), peer)
                        requested = True
                if msg.get_type() == 'PARAM_VALUE' and msg.get_srcSystem() == 1 and msg.get_srcComponent() == 1:
                    key = msg.param_id.rstrip('\0')
                    if key in keys:
                        assert msg.param_type == mav.MAV_PARAM_TYPE_INT32, (key, msg.param_type)
                        params[key] = struct.unpack('<i', struct.pack('<f', msg.param_value))[0]
            if len(params) == len(keys) and heartbeats:
                return {'parameters': params, 'heartbeats': heartbeats, 'configuration_changed': False,
                        'integer_encoding': 'MAVLink bytewise INT32', 'only_commands': 'PARAM_REQUEST_READ'}
        raise TimeoutError(f'Incomplete PX4 snapshot: {params}, heartbeat count {len(heartbeats)}')
    finally:
        sock.close()


def capture(label, duration, interrupt_after=None):
    out = ROOT / label
    command = [str(RELEASE / 'start_capture'), '--output-root', str(out), '--duration-s', str(duration), '--expected-rawx-hz', '25']
    env = dict(os.environ, CAPTURE_PYTHON=sys.executable)
    started = time.monotonic()
    with (ROOT / f'{label}_stdout.txt').open('w') as stdout, (ROOT / f'{label}_stderr.txt').open('w') as stderr:
        child = subprocess.Popen(command, cwd=RELEASE, env=env, stdout=stdout, stderr=stderr)
        try:
            if interrupt_after is not None:
                time.sleep(interrupt_after)
                assert child.poll() is None, 'Capture exited before planned SIGINT'
                child.send_signal(signal.SIGINT)
            code = child.wait(timeout=duration + 15)
        finally:
            if child.poll() is None:
                child.terminate()
                child.wait(timeout=10)
    sessions = list(out.glob('*/manifest.json'))
    assert len(sessions) == 1
    session = sessions[0].parent
    manifest = json.loads(sessions[0].read_text())
    result = {'command': command, 'CAPTURE_PYTHON': sys.executable, 'exit_code': code,
              'elapsed_s': time.monotonic() - started, 'session': str(session),
              'signal': 'SIGINT to own capture child' if interrupt_after is not None else None,
              'expected_status': 'stopped' if interrupt_after is not None else 'complete'}
    save(f'{label}_run.json', result)
    assert code == (130 if interrupt_after is not None else 0), result
    assert manifest['status'] == result['expected_status'], manifest
    assert not (session / 'UNFINALIZED').exists()
    for stats in manifest['streams'].values():
        assert stats['writer_closed'] and stats['dropped_bytes'] == 0 and stats['written_bytes'] > 0, stats
    subprocess.run([sys.executable, '-m', 'detect2.acquisition.audit_session', str(session)], cwd=RELEASE, check=True)
    return result


def main():
    ROOT.mkdir(parents=True, exist_ok=False)
    save('system_before.json', state())
    assert len(json.loads((ROOT / 'system_before.json').read_text())['router_processes']) == 1
    before_rx = receiver_snapshot()
    save('receiver_before.json', before_rx)
    before_px4 = px4_snapshot()
    save('px4_before.json', before_px4)
    normal = capture('duration', 20)
    interrupted = capture('interrupt', 30, interrupt_after=15)
    after_rx = receiver_snapshot()
    save('receiver_after.json', after_rx)
    after_px4 = px4_snapshot()
    save('px4_after.json', after_px4)
    save('system_after.json', state())
    pre = json.loads((ROOT / 'system_before.json').read_text())
    post = json.loads((ROOT / 'system_after.json').read_text())
    release = json.loads((RELEASE / 'release_manifest.json').read_text())
    deployed = {p: hashlib.sha256((RELEASE / p).read_bytes()).hexdigest() for p in release['files']}
    checks = {
        'receiver_ram_unchanged': before_rx['ram'] == after_rx['ram'],
        'receiver_identity_unchanged': before_rx['identity'] == after_rx['identity'],
        'px4_logging_parameters_unchanged': before_px4['parameters'] == after_px4['parameters'],
        'legacy_files_unchanged': pre['legacy_files'] == post['legacy_files'],
        'router_process_unchanged': pre['router_processes'] == post['router_processes'],
        'deployed_hashes_match': deployed == release['files'],
    }
    save('stage2_validation.json', {'checks': checks, 'captures': [normal, interrupted],
          'configuration_writes': False, 'timesync_active': False,
          'checkpoint_2': 'awaiting user review', 'stage3_started': False})
    assert all(checks.values()), checks
    print('STAGE2_DONE', ROOT, flush=True)


if __name__ == '__main__':
    main()
