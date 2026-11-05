"""Explicit RAM-only X20 UART1 transition between 921600 and 2 Mbaud.

Unlike message-rate changes, the ACK may arrive at either baud. Success requires
identity and CFG-VALGET verification at the new host rate. Failure probes only
the two reviewed rates and restores the original rate on the same receiver.
"""
import argparse
import json
from pathlib import Path
import time

from .common import atomic_json
from .receiver_config import SerialBackend

KEY = 'CFG_UART1_BAUDRATE'
RATES = (921600, 2000000)


class BaudBackend(SerialBackend):
    def host_rate(self, rate):
        self.serial.baudrate = rate
        self.serial.reset_input_buffer()
        self.buffer.clear()

    def switch_ram(self, rate):
        # Layer bit 1 = RAM only. Do not wait for an ACK across a baud switch.
        wire = self.message.config_set(1, 0, [(KEY, rate)]).serialize()
        if self.serial.write(wire) != len(wire):
            raise OSError('Short baud configuration write')
        self.serial.flush()
        time.sleep(.2)  # u-blox specifies typically 100 ms before new-rate input.
        self.host_rate(rate)


def transition(backend, original, target, journal):
    if original not in RATES or target not in RATES or original == target:
        raise ValueError('Only distinct reviewed rates 921600 and 2000000 are supported')
    if Path(journal).exists():
        raise ValueError('Journal exists; preserve the previous transaction')
    identity = backend.identity()
    if backend.read([KEY]) != {KEY: original}:
        raise RuntimeError('Receiver baud differs from the declared starting baud; no write')
    record = {'status': 'prepared', 'layer': 'RAM only', 'key': KEY,
              'before': original, 'desired': target, 'identity': identity,
              'recovery_attempts': []}
    atomic_json(journal, record)
    try:
        backend.switch_ram(target)
        if backend.identity() != identity or backend.read([KEY]) != {KEY: target}:
            raise RuntimeError('New-rate identity or baud readback mismatch')
        record.update(status='applied_and_verified', after=target)
        atomic_json(journal, record)
        return record
    except Exception as error:
        record.update(status='recovering', apply_error=str(error))
        atomic_json(journal, record)
        for rate in (target, original):
            attempt = {'probe_host_baud': rate}
            try:
                backend.host_rate(rate)
                if backend.identity() != identity:
                    raise RuntimeError('Identity mismatch; refusing rollback write')
                current = backend.read([KEY])[KEY]
                if current not in RATES or current != rate:
                    raise RuntimeError('Unexpected baud readback; refusing rollback write')
                if current != original:
                    backend.switch_ram(original)
                if backend.identity() != identity or backend.read([KEY]) != {KEY: original}:
                    raise RuntimeError('Rollback verification failed')
                attempt['status'] = 'restored_and_verified'
                record['recovery_attempts'].append(attempt)
                record.update(status='rolled_back', after=original)
                atomic_json(journal, record)
                raise RuntimeError(f'Baud transition failed and rolled_back; inspect {journal}') from error
            except Exception as recovery_error:
                if record['status'] == 'rolled_back':
                    raise
                attempt['error'] = str(recovery_error)
                record['recovery_attempts'].append(attempt)
                atomic_json(journal, record)
        record['status'] = 'recovery_failed'
        atomic_json(journal, record)
        raise RuntimeError(f'Baud recovery_failed; no success inferred; inspect {journal}') from error


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--device', required=True)
    p.add_argument('--from-baud', required=True, type=int, choices=RATES)
    p.add_argument('--to-baud', required=True, type=int, choices=RATES)
    p.add_argument('--journal', required=True, type=Path)
    a = p.parse_args()
    backend = BaudBackend(a.device, a.from_baud)
    try:
        print(json.dumps(transition(backend, a.from_baud, a.to_baud, a.journal), indent=2))
    finally:
        backend.close()


if __name__ == '__main__':
    main()
