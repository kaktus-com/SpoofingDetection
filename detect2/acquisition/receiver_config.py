"""Reviewable receiver snapshots and RAM-only transactions with readback/rollback.

No automatic invocation from capture. Hardware operations belong to Stage 3.
"""
import argparse
from datetime import datetime,timezone
import json
from pathlib import Path
import time

from .common import atomic_json

PROFILE=Path(__file__).with_name('x20_recording_profile.json')


def profile_values(path=PROFILE):
    values=json.loads(Path(path).read_text())['settings']
    validate_values(values)
    return values


def validate_values(values):
    allowed=set(json.loads(PROFILE.read_text())['settings'])
    if not values or not set(values)<=allowed:raise ValueError('Only reviewed acquisition-rate keys are supported')
    for key,value in values.items():
        if isinstance(value,bool) or not isinstance(value,int) or value<0:raise ValueError('Configuration values must be nonnegative integers')
        if 'MSGOUT' in key and value>255:raise ValueError('Message rate out of range')
    if values.get('CFG_RATE_MEAS',50)<40 or values.get('CFG_RATE_NAV',1)<1:raise ValueError('Invalid measurement/navigation interval')
    if values.get('CFG_MSGOUT_UBX_RXM_RAWX_UART1',1)<1:raise ValueError('RAWX output must remain enabled')


def snapshot(backend,keys):
    values=backend.read(keys)
    if set(values)!=set(keys):raise RuntimeError('Incomplete RAM baseline; no configuration write permitted')
    return {'schema_version':1,'utc':datetime.now(timezone.utc).isoformat(),'identity':backend.identity(),
            'ram':values,'configuration_changed':False}


def apply_ram(backend,baseline,desired,journal):
    validate_values(desired)
    if Path(journal).exists():raise ValueError('Transaction journal already exists; choose a new path')
    if baseline['identity']!=backend.identity():raise RuntimeError('Receiver identity differs from baseline')
    keys=list(desired);before=backend.read(keys)
    if any(baseline['ram'].get(k)!=before.get(k) for k in keys):raise RuntimeError('Stale baseline; configuration changed since snapshot')
    if set(before)!=set(keys):raise RuntimeError('Incomplete pre-write readback')
    record={'status':'prepared','layer':'RAM only','identity':baseline['identity'],'before':before,'desired':desired}
    atomic_json(journal,record)
    try:
        backend.set_ram(desired)
        after=backend.read(keys);record['after']=after
        if after!=desired:raise RuntimeError('RAM readback differs from requested profile')
        record['status']='applied_and_verified';atomic_json(journal,record)
        return record
    except Exception as error:
        record['apply_error']=str(error)
        try:
            backend.set_ram(before);restored=backend.read(keys);record['restore_readback']=restored
            record['status']='rolled_back' if restored==before else 'rollback_failed'
        except Exception as restore_error:
            record.update(status='rollback_failed',restore_error=str(restore_error))
        atomic_json(journal,record)
        raise RuntimeError(f"Apply failed: {record['status']}; inspect {journal}") from error


def restore_ram(backend,record,journal):
    if Path(journal).exists():raise ValueError('Restore journal already exists')
    if backend.identity()!=record['identity']:raise RuntimeError('Receiver identity differs')
    before=record['before'];current=backend.read(list(before))
    validate_values(before)
    if set(current)!=set(before):raise RuntimeError('Incomplete current readback; no restore write permitted')
    for key,value in current.items():
        if value not in (before.get(key),record['desired'].get(key)):raise RuntimeError('Unrelated configuration changes; review manually before restoring')
    atomic_json(journal,{'status':'prepared_restore','layer':'RAM only','before':current,'restore':before})
    try:backend.set_ram(before);after=backend.read(list(before))
    except Exception as error:
        atomic_json(journal,{'status':'restore_failed','layer':'RAM only','error':str(error),'restore':before});raise
    result={'status':'restored_and_verified' if after==before else 'restore_failed','layer':'RAM only','after':after}
    atomic_json(journal,result)
    if after!=before:raise RuntimeError('Restore readback failed')
    return result


class SerialBackend:
    def __init__(self,device,baud):
        from serial import Serial
        from pyubx2 import UBXMessage,UBXReader,POLL
        self.message,self.reader,self.poll=UBXMessage,UBXReader,POLL
        self.serial=Serial(device,baud,timeout=.05,exclusive=True)
        self.device=str(Path(device).absolute())
        self.buffer=bytearray()

    def close(self):self.serial.close()

    def _query(self,wire,match):
        from io import BytesIO
        self.serial.write(wire);deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            self.buffer.extend(self.serial.read(min(max(self.serial.in_waiting,1),65535)))
            while True:
                start=self.buffer.find(b'\xb5\x62')
                if start<0:
                    self.buffer=self.buffer[-1:];break
                if start:del self.buffer[:start]
                if len(self.buffer)<6:break
                size=8+int.from_bytes(self.buffer[4:6],'little')
                if len(self.buffer)<size:break
                frame=bytes(self.buffer[:size]);del self.buffer[:size]
                try:_,message=self.reader(BytesIO(frame),quitonerror=0).read()
                except Exception:continue
                if message is not None and match(message):return message
        raise TimeoutError('Receiver query timed out; no successful result inferred')

    def identity(self):
        message=self._query(self.message('MON','MON-VER',self.poll).serialize(),lambda m:m.identity=='MON-VER')
        fields={k:(v.decode('ascii',errors='replace').rstrip('\0') if isinstance(v,bytes) else v)
                for k,v in vars(message).items() if k.startswith(('swVersion','hwVersion','extension_'))}
        return {'transport_device':self.device,'reported_receiver':fields,'unique_receiver_hardware_id':'not queried; physical identity must be checked at the manual gate'}

    def read(self,keys):
        def match(m):
            if m.identity=='ACK-NAK' and m.clsID==6 and m.msgID==0x8b:raise RuntimeError('Receiver rejected RAM query')
            return m.identity=='CFG-VALGET' and m.layer==0
        msg=self._query(self.message.config_poll(0,0,keys).serialize(),match)
        values={k:getattr(msg,k) for k in keys if hasattr(msg,k)}
        if set(values)!=set(keys):raise RuntimeError('Receiver returned incomplete configuration')
        return values

    def set_ram(self,values):
        # Hard-coded layer bit 1 = RAM, transaction 0. No flash/BBR save path.
        wire=self.message.config_set(1,0,list(values.items())).serialize()
        def match(m):return m.identity in ['ACK-ACK','ACK-NAK'] and m.clsID==6 and m.msgID==0x8a
        ack=self._query(wire,match)
        if ack.identity!='ACK-ACK':raise RuntimeError('Receiver rejected RAM configuration')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('action',choices=['inspect','apply-ram','restore-ram'])
    p.add_argument('--device',required=True);p.add_argument('--baud',type=int,default=921600)
    p.add_argument('--profile',type=Path,default=PROFILE)
    p.add_argument('--baseline',type=Path);p.add_argument('--transaction',type=Path)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():p.error('Output already exists; preserve previous snapshots/journals')
    if a.action=='apply-ram' and not a.baseline:p.error('apply-ram requires --baseline')
    if a.action=='restore-ram' and not a.transaction:p.error('restore-ram requires --transaction')
    desired=profile_values(a.profile)
    backend=SerialBackend(a.device,a.baud)
    try:
        if a.action=='inspect':
            result=snapshot(backend,list(desired));result['proposed_diff']={k:{'current':result['ram'][k],'proposed':v} for k,v in desired.items() if result['ram'][k]!=v}
            atomic_json(a.output,result)
        elif a.action=='apply-ram':result=apply_ram(backend,json.loads(a.baseline.read_text()),desired,a.output)
        else:result=restore_ram(backend,json.loads(a.transaction.read_text()),a.output)
        print(json.dumps(result,indent=2))
    finally:backend.close()


if __name__=='__main__':main()
