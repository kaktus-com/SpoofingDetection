"""Authorized Stage 3: RAM profile, 921600 comparison, 2 Mbaud continuity test.

No persistent writes, constellation edits, PX4 changes, or router restarts.
On an operational error, restore the saved message profile and original baud.
"""
import hashlib
from io import BytesIO
import json
import os
from pathlib import Path
import runpy
import subprocess
import sys
import time

RELEASE = Path('/home/nss/SpoofingDetection/releases/acquisition_20261105T103442Z')
ROOT = Path('/home/nss/SpoofingDetection/recordings_v2_stage3/acquisition_20261105T103442Z')
GPS = '/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DQ00KK6P-if00-port0'
sys.path.insert(0, str(RELEASE))
from detect2.acquisition.baudrate import BaudBackend, KEY, transition
from detect2.acquisition.receiver_config import snapshot, apply_ram, restore_ram, profile_values
from detect2.acquisition.ubx import frames
from pyubx2 import UBXReader

PRIOR = runpy.run_path('/home/nss/SpoofingDetection/recordings_v2_stage2/acquisition_20261105T100733Z/run_stage2.py')
SIGNALS = ['CFG_SIGNAL_'+s+'_ENA' for s in ['GPS','GAL','BDS','GLO','QZSS','SBAS','NAVIC']]
KEYS = list(profile_values()) + [KEY] + SIGNALS


def save(name, data):
    (ROOT / name).write_text(json.dumps(data, indent=2) + '\n')
    print(name, flush=True)


def connect(rate):
    return BaudBackend(GPS, rate)


def snap(rate):
    errors = []
    for _ in range(3):
        backend = connect(rate)
        try:
            result = snapshot(backend, KEYS)
            result['host_baud'] = rate
            result['read_only_retry_errors'] = errors
            return result
        except Exception as error:
            errors.append(str(error))
        finally:
            backend.close()
    raise RuntimeError(f'Receiver snapshot failed: {errors}')


def flash_snapshot(rate):
    backend = connect(rate)
    try:
        keys = list(profile_values()) + [KEY]
        reply = backend._query(backend.message.config_poll(2, 0, keys).serialize(),
                               lambda m: m.identity == 'CFG-VALGET' and m.layer == 2)
        result = {k: getattr(reply, k) for k in keys if hasattr(reply, k)}
        return {'status': 'PASS' if set(result) == set(keys) else 'UNKNOWN', 'layer': 'Flash', 'values': result}
    except Exception as error:
        return {'status': 'UNKNOWN', 'error': str(error)}
    finally:
        backend.close()


def monitor_comms(session):
    records = []
    for offset, cls, ident, payload in frames((session/'receiver.bin').read_bytes(), {}):
        if (cls, ident) != (10, 54):
            continue
        from detect2.acquisition.ubx import packet
        _, msg = UBXReader(BytesIO(packet(cls, ident, payload)), quitonerror=0).read()
        ports = []
        for i in range(1, msg.nPorts + 1):
            suffix = f'_{i:02}'
            ports.append({k: getattr(msg, k+suffix) for k in
                          ['portId','txPending','txBytes','txUsage','txPeakUsage','rxPending','rxBytes','rxUsage','overrunErrs','skipped']})
        records.append({'archive_byte_offset': offset, 'ports': ports})
    (session/'receiver_comms.json').write_text(json.dumps(records, indent=2)+'\n')


def capture(label, baud, duration):
    command = [str(RELEASE/'start_capture'), '--output-root', str(ROOT/label),
               '--gps-baud', str(baud), '--duration-s', str(duration), '--expected-rawx-hz', '20']
    env = dict(os.environ, CAPTURE_PYTHON=sys.executable)
    print('CAPTURE_START', label, baud, duration, flush=True)
    with (ROOT/(label+'_stdout.txt')).open('w') as out, (ROOT/(label+'_stderr.txt')).open('w') as err:
        child = subprocess.Popen(command, cwd=RELEASE, env=env, stdout=out, stderr=err)
        try:
            code = child.wait(timeout=duration+25)
        finally:
            if child.poll() is None:
                child.terminate(); child.wait(timeout=10)
    sessions = list((ROOT/label).glob('*/manifest.json'))
    assert len(sessions) == 1
    session = sessions[0].parent
    manifest = json.loads(sessions[0].read_text())
    save(label+'_run.json', {'command': command, 'CAPTURE_PYTHON': sys.executable,
                            'exit_code': code, 'session': str(session), 'baud': baud})
    assert code == 0 and manifest['status'] == 'complete', manifest
    for stats in manifest['streams'].values():
        assert stats['received_bytes'] == stats['written_bytes'] and stats['written_bytes'] > 0
        assert stats['writer_closed'] and not stats['dropped_bytes'] and not stats['error']
    assert not (session/'UNFINALIZED').exists() and not manifest['issues']['counts']
    subprocess.run([sys.executable, '-m', 'detect2.acquisition.audit_session', str(session)], cwd=RELEASE, check=True)
    monitor_comms(session)
    return str(session)


def rollback():
    before = json.loads((ROOT/'receiver_before.json').read_text())
    transaction = json.loads((ROOT/'profile_transaction.json').read_text()) if (ROOT/'profile_transaction.json').exists() else None
    errors = []
    for rate in [2000000, 921600]:
        backend = connect(rate)
        try:
            if backend.identity() != before['identity']:
                raise RuntimeError('Identity mismatch; no rollback write')
            if transaction and transaction['status'] == 'applied_and_verified':
                restore_ram(backend, transaction, ROOT/f'profile_restore_{rate}.json')
            if rate != before['ram'][KEY]:
                transition(backend, rate, before['ram'][KEY], ROOT/f'baud_restore_{rate}.json')
            backend.close()
            after = snap(before['ram'][KEY])
            save('rollback_readback.json', after)
            assert after['ram'] == before['ram']
            return
        except Exception as error:
            errors.append({'baud': rate, 'error': str(error)})
        finally:
            backend.close()
    save('rollback_errors.json', errors)
    raise RuntimeError(f'Rollback not verified: {errors}')


def main():
    if len(sys.argv) > 1 and sys.argv[1] == 'restore':
        rollback(); print('RESTORED_ORIGINAL_PROFILE_AND_BAUD', flush=True); return
    ROOT.mkdir(parents=True, exist_ok=False)
    save('system_before.json', PRIOR['state']())
    save('px4_before.json', PRIOR['px4_snapshot']())  # also verifies disarmed
    before = snap(921600)
    save('receiver_before.json', before)
    save('flash_before.json', flash_snapshot(921600))
    desired = profile_values()
    save('reviewed_diff.json', {k: {'before': before['ram'][k], 'desired': v} for k,v in desired.items() if before['ram'][k] != v})
    backend = connect(921600)
    try:
        apply_ram(backend, before, desired, ROOT/'profile_transaction.json')
    finally:
        backend.close()
    try:
        time.sleep(3)
        save('receiver_profile_921600.json', snap(921600))
        low = capture('reduced_921600', 921600, 30)
        backend = connect(921600)
        try:
            transition(backend, 921600, 2000000, ROOT/'baud_transaction.json')
        finally:
            backend.close()
        time.sleep(3)
        save('receiver_profile_2000000.json', snap(2000000))
        high = capture('reduced_2000000', 2000000, 120)
        after = snap(2000000)
        save('receiver_after.json', after)
        save('flash_after.json', flash_snapshot(2000000))
        save('px4_after.json', PRIOR['px4_snapshot']())
        save('system_after.json', PRIOR['state']())
        pre = json.loads((ROOT/'system_before.json').read_text())
        post = json.loads((ROOT/'system_after.json').read_text())
        checks = {'profile_readback_matches': all(after['ram'][k] == v for k,v in desired.items()),
                  'receiver_baud_is_2000000': after['ram'][KEY] == 2000000,
                  'constellation_enable_flags_unchanged': all(before['ram'][k] == after['ram'][k] for k in SIGNALS),
                  'receiver_identity_unchanged': before['identity'] == after['identity'],
                  'legacy_files_unchanged': pre['legacy_files'] == post['legacy_files'],
                  'router_process_unchanged': pre['router_processes'] == post['router_processes'],
                  'px4_logging_parameters_unchanged': json.loads((ROOT/'px4_before.json').read_text())['parameters'] == json.loads((ROOT/'px4_after.json').read_text())['parameters']}
        save('stage3_validation.json', {'checks': checks, 'sessions': [low, high],
             'configuration_layer': 'RAM only', 'current_host_baud': 2000000,
             'constellation_changes': False, 'persistent_writes': False,
             'manual_checkpoint_3': 'awaiting user review', 'stage4_started': False})
        assert all(checks.values()), checks
        print('STAGE3_DONE', ROOT, flush=True)
    except BaseException as error:
        save('stage3_error.json', {'error': str(error)})
        rollback()
        raise


if __name__ == '__main__':
    main()
