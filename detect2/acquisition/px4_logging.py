"""Disarmed-only PX4 logging parameters, bounded shell and native-log download.

Uses the existing router endpoint. Never opens the Cube serial port, arms the
vehicle, changes flight parameters or writes receiver configuration.
"""
import os
from pathlib import Path
import struct
import socket
import time

from .common import atomic_json, fingerprint

LOG_KEYS = ('SDLOG_PROFILE', 'SDLOG_MODE', 'SDLOG_BACKEND')
DESIRED = {'SDLOG_PROFILE': 17, 'SDLOG_MODE': 2, 'SDLOG_BACKEND': 3}


class ArmedError(RuntimeError):
    """Configuration/native-log shutdown must wait until the vehicle disarms."""


def int32_wire(value):
    return struct.unpack('<f', struct.pack('<i', value))[0]


def wire_int32(value):
    return struct.unpack('<i', struct.pack('<f', value))[0]


def apply_logging(client, baseline, journal):
    """Write only the reviewed logging diff; retain backend and rollback on error."""
    if Path(journal).exists():
        raise ValueError('Journal exists')
    client.assert_disarmed()
    if client.identity() != baseline['identity']:
        raise RuntimeError('PX4 identity mismatch; no write')
    before = client.read_params(LOG_KEYS)
    if before != baseline['parameters'] or before['SDLOG_BACKEND'] != 3:
        raise RuntimeError('Stale or unexpected logging baseline; no write')
    if before['SDLOG_PROFILE'] not in (1,17) or before['SDLOG_MODE'] not in (0,2):
        raise RuntimeError('Unreviewed existing logging profile/mode; no write')
    record = {'status': 'prepared', 'identity': baseline['identity'], 'before': before,
              'desired': DESIRED, 'scope': 'PX4 logging parameters only', 'writes': []}
    atomic_json(journal, record)
    try:
        for key in LOG_KEYS:
            if before[key] != DESIRED[key]:
                client.set_param(key, DESIRED[key])
                record['writes'].append(key)
                atomic_json(journal, record)
        record['after'] = client.read_params(LOG_KEYS)
        if record['after'] != DESIRED:
            raise RuntimeError('Logging readback mismatch')
        record['status'] = 'applied_and_verified'
        atomic_json(journal, record)
        return record
    except Exception as error:
        record['apply_error'] = str(error)
        try:
            client.assert_disarmed()
            if client.identity() != baseline['identity']:
                raise RuntimeError('Identity changed; no rollback write')
            for key in LOG_KEYS:
                if key != 'SDLOG_BACKEND':
                    client.set_param(key, before[key])
            record['rollback_readback'] = client.read_params(LOG_KEYS)
            record['status'] = 'rolled_back' if record['rollback_readback'] == before else 'rollback_failed'
        except Exception as rollback_error:
            record.update(status='rollback_failed', rollback_error=str(rollback_error))
        atomic_json(journal, record)
        raise RuntimeError(f"Logging apply failed: {record['status']}; inspect {journal}") from error


class PX4Link:
    def __init__(self, endpoint='udpin:127.0.0.1:14551'):
        os.environ.setdefault('MAVLINK20', '1')
        from pymavlink import mavutil
        self.protocol = mavutil.mavlink
        self.master = mavutil.mavlink_connection(endpoint, source_system=245, source_component=191,
                                                  dialect='common', robust_parsing=True)
        if isinstance(self.master.port,socket.socket):
            self.master.port.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,0)
        self._identity = None
        try:self.assert_disarmed()
        except BaseException:
            self.master.close();raise

    def close(self):
        self.master.close()

    def receive(self, kind, timeout=5):
        until = time.monotonic() + timeout
        while time.monotonic() < until:
            message = self.master.recv_match(type=kind, blocking=True, timeout=min(.3, max(.01, until-time.monotonic())))
            if message and message.get_srcSystem() == 1 and message.get_srcComponent() == 1:
                return message
        raise TimeoutError(f'No PX4 {kind} reply')

    def assert_disarmed(self):
        # Do not authorize a write from a heartbeat queued during slow setup.
        until=time.monotonic()+.2
        while time.monotonic()<until and self.master.recv_match(blocking=False) is not None:pass
        heartbeat = self.receive('HEARTBEAT', 5)
        if heartbeat.base_mode & 128:
            raise ArmedError('PX4 armed; operation refused')
        return heartbeat.to_dict()

    def identity(self):
        # Cache the actual UID/firmware, not an inferred branch/version label.
        self.master.mav.command_long_send(1, 1, 512, 0, 148, 0, 0, 0, 0, 0, 0)
        version = self.receive('AUTOPILOT_VERSION')
        if not version.capabilities & 16:
            raise RuntimeError('Bytewise parameter capability is not advertised')
        self._identity = {k: getattr(version, k) for k in ['uid', 'flight_sw_version', 'flight_custom_version', 'vendor_id', 'product_id']}
        return self._identity

    def read_params(self, keys):
        if not set(keys) <= set(LOG_KEYS) | {'SDLOG_BOOT_BAT'}:
            raise ValueError('Only logging parameters may be queried by this helper')
        result = {}
        for _ in range(3):
            for key in keys:
                if key not in result:
                    self.master.mav.param_request_read_send(1, 1, key.encode(), -1)
            until = time.monotonic() + 2
            while time.monotonic() < until and len(result) < len(keys):
                try:
                    msg = self.receive('PARAM_VALUE', min(.4, max(.01, until-time.monotonic())))
                except TimeoutError:
                    continue
                name = msg.param_id.rstrip('\0')
                if name in keys:
                    if msg.param_type != 6:
                        raise RuntimeError('Logging parameter has unexpected type')
                    result[name] = wire_int32(msg.param_value)
            if len(result) == len(keys):
                return result
        raise TimeoutError(f'Incomplete logging parameters: {result}')

    def set_param(self, name, value):
        reviewed={'SDLOG_PROFILE':(1,17),'SDLOG_MODE':(0,2)}
        if name not in reviewed or value not in reviewed[name]:
            raise ValueError('Unreviewed parameter/value')
        self.assert_disarmed()
        self.master.mav.param_set_send(1, 1, name.encode(), int32_wire(value), 6)
        until = time.monotonic() + 5
        while time.monotonic() < until:
            msg = self.receive('PARAM_VALUE', max(.01, until-time.monotonic()))
            if msg.param_id.rstrip('\0') == name and msg.param_type == 6 and wire_int32(msg.param_value) == value:
                if self.read_params([name]) == {name: value}:
                    return
        raise TimeoutError(f'No verified readback for {name}')

    def shell(self, command, timeout=5):
        allowed = {'ver all', 'free', 'logger status', 'logger help', 'logger stop',
                   'logger start -b 8 -t -m all -f', 'logger start -b 64 -t -m all -f',
                   'logger start -b 128 -t -m all -f', 'logger off'}
        if command not in allowed:
            raise ValueError('Unreviewed shell command')
        if command.startswith('logger ') and command not in {'logger status','logger help'}:
            self.assert_disarmed()
        output = bytearray()
        try:
            # Clear any old shell response before issuing this operation.
            while self.master.recv_match(type='SERIAL_CONTROL', blocking=False):
                pass
            data = (command+'\n').encode()
            self.master.mav.serial_control_send(10, 2|4|16, 0, 0, len(data), list(data)+[0]*(70-len(data)))
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                try:
                    msg = self.receive('SERIAL_CONTROL', min(.4, max(.01, end-time.monotonic())))
                except TimeoutError:
                    continue
                output.extend(bytes(msg.data[:msg.count]))
        finally:
            self.master.mav.serial_control_send(10, 0, 0, 0, 0, [0]*70)
        return output.decode(errors='replace')

    def download(self, remote_path, local_path):
        """Read-only file payload operation through the installed MAVFTP client."""
        if not remote_path.startswith('/fs/microsd/log/') or not remote_path.endswith('.ulg') or '..' in remote_path.split('/'):
            raise ValueError('Only native SD-card ULog paths may be downloaded')
        local_path = Path(local_path)
        if local_path.exists():
            raise ValueError('Local log already exists')
        self.assert_disarmed()
        from pymavlink import mavftp
        ftp = mavftp.MAVFTP(self.master, 1, 1)
        ftp.temp_filename = str(local_path.with_suffix('.ftp.part'))
        if Path(ftp.temp_filename).exists():
            raise ValueError('Partial transfer already exists; preserve it')
        ftp.ftp_settings.burst_read_size = 239
        result = ftp.cmd_get([remote_path, str(local_path)])
        if result.return_code != 0:
            raise RuntimeError(f'FTP open failed: {result.return_code}')
        # Large native logs can take several short receive windows. A timeout
        # with ongoing progress is not EOF and must not become a valid download.
        previous_total = -1
        for _ in range(10):
            result = ftp.process_ftp_reply('OpenFileRO', timeout=55)
            if ftp.done:
                break
            print('ULog download progress:', ftp.read_total, '/', ftp.remote_file_size, flush=True)
            if ftp.read_total <= previous_total:
                break
            previous_total = ftp.read_total
        # SDK termination clears reached_eof/read_gaps. `done` and get_result
        # are assigned only after its no-gap completion condition is satisfied.
        if result.return_code != 0 or not ftp.done or ftp.get_result is None or not local_path.exists():
            raise RuntimeError(f'FTP download incomplete: {result.return_code}; gaps {ftp.read_gaps}')
        size = ftp.remote_file_size
        if size is None or len(ftp.get_result) != size or local_path.stat().st_size != size:
            raise RuntimeError('Downloaded size differs from advertised file size')
        return {'remote_path': remote_path, 'advertised_bytes': size,
                'local_file': str(local_path), 'fingerprint': fingerprint(local_path),
                'sdk_no_gap_completion': ftp.done, 'completed_bytes': len(ftp.get_result)}
