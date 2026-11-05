"""Read-only Cube PPS research snapshot; no driver/parameter changes.

Run from the release/repository root with its acquisition dependencies installed.
Uses the existing router endpoint, with the normal disarmed/endpoint guards.
The production logging helper's command allowlist is left unchanged.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time

from detect2.acquisition.ownership import CaptureLocks
from detect2.acquisition.px4_logging import PX4Link, wire_int32

PARAMETERS=tuple(f'PWM_AUX_FUNC{i}' for i in range(1,7))+('PWM_AUX_TIM0','PWM_AUX_TIM1',
    'PPS_CAP_ENABLE','PPS_CAP_GPS_ID','CAM_CAP_FBACK','CAM_CAP_MODE','CAM_CAP_EDGE',
    'SENS_EN_LL40LS','FD_EXT_ATS_EN')+tuple(f'PWM_MAIN_FUNC{i}' for i in range(1,9))
COMMANDS=('ver all','pps_capture status','camera_capture status','pwm_out status','pwm_input status','logger status')


def read_parameters(link):
    found={}
    for attempt in range(3):
        for name in PARAMETERS:
            if name not in found:link.master.mav.param_request_read_send(1,1,name.encode(),-1)
        end=time.monotonic()+2
        while time.monotonic()<end and len(found)<len(PARAMETERS):
            try:msg=link.receive('PARAM_VALUE',min(.3,max(.01,end-time.monotonic())))
            except TimeoutError:continue
            name=msg.param_id.rstrip('\0')
            if name in PARAMETERS:
                found[name]=wire_int32(msg.param_value) if msg.param_type==6 else msg.param_value
    return {'values':found,'unanswered':[p for p in PARAMETERS if p not in found]}


def read_status(link, command):
    if command not in COMMANDS:raise ValueError('Status queries only')
    output=bytearray()
    try:
        while link.master.recv_match(type='SERIAL_CONTROL',blocking=False):pass
        data=(command+'\n').encode()
        link.master.mav.serial_control_send(10,2|4|16,0,0,len(data),list(data)+[0]*(70-len(data)))
        end=time.monotonic()+3
        while time.monotonic()<end:
            try:msg=link.receive('SERIAL_CONTROL',min(.3,max(.01,end-time.monotonic())))
            except TimeoutError:continue
            output.extend(bytes(msg.data[:msg.count]))
    finally:link.master.mav.serial_control_send(10,0,0,0,0,[0]*70)
    return output.decode(errors='replace')


def inspect(output):
    with CaptureLocks(None,('127.0.0.1',14551)):
        link=PX4Link()
        try:
            result={'inspected_utc':datetime.now(timezone.utc).isoformat(),'read_only':True,
                'identity':link.identity(),'parameters':read_parameters(link),
                'status':{command:read_status(link,command) for command in COMMANDS}}
        finally:link.close()
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    inspect(parser.parse_args().output)
