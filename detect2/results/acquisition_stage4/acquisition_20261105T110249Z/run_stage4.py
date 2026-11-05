"""Stage 4 native logger bench recording, through the existing router only."""
import json
import os
from pathlib import Path
import re
import runpy
import subprocess
import sys
import time

RELEASE=Path('/home/nss/SpoofingDetection/releases/acquisition_20261105T110249Z')
ROOT=Path('/home/nss/SpoofingDetection/recordings_v2_stage4/acquisition_20261105T110249Z')
sys.path.insert(0,str(RELEASE))
from detect2.acquisition.px4_logging import PX4Link,apply_logging,LOG_KEYS,DESIRED
from detect2.acquisition.common import atomic_json

prior=runpy.run_path('/home/nss/SpoofingDetection/releases/acquisition_20261105T103442Z/run_stage3.py')


def save(name,value):
    atomic_json(ROOT/name,value);print(name,flush=True)


def shell(client,command,name,timeout=4):
    result=client.shell(command,timeout)
    (ROOT/name).write_text(result)
    print(name,result[:250].replace('\n',' | '),flush=True)
    return result


def restore():
    before=json.loads(Path('/home/nss/SpoofingDetection/recordings_v2_stage4/acquisition_20261105T105214Z/px4_before.json').read_text())
    client=PX4Link()
    try:
        assert client.identity()==before['identity']
        current=client.read_params(LOG_KEYS)
        assert all(current[k] in [DESIRED[k],before['parameters'][k]] for k in LOG_KEYS)
        save('rollback_prepared.json',{'before_restore':current,'restore':before['parameters']})
        shell(client,'logger stop','rollback_logger_stop.txt')
        for key in ['SDLOG_PROFILE','SDLOG_MODE']:
            client.set_param(key,before['parameters'][key])
        # Original mode 0: start without -f. This exact baseline start command
        # is separately handled below by a reviewed SERIAL_CONTROL packet.
        command='logger start -b 8 -t -m all\n';data=command.encode()
        client.assert_disarmed()
        client.master.mav.serial_control_send(10,2|4|16,0,0,len(data),list(data)+[0]*(70-len(data)))
        time.sleep(2)
        client.master.mav.serial_control_send(10,0,0,0,0,[0]*70)
        readback=client.read_params(LOG_KEYS)
        save('rollback_readback.json',readback)
        assert readback==before['parameters']
    finally:client.close()


def main():
    if len(sys.argv)>1 and sys.argv[1]=='restore':restore();return
    ROOT.mkdir(parents=True,exist_ok=False)
    save('system_before.json',prior['PRIOR']['state']())
    save('receiver_before.json',prior['snap'](2000000))
    client=PX4Link()
    try:
        baseline={'identity':client.identity(),'parameters':client.read_params(LOG_KEYS),
                  'heartbeat':client.assert_disarmed(),'SDLOG_BOOT_BAT':client.read_params(['SDLOG_BOOT_BAT'])}
        save('px4_before.json',baseline)
        shell(client,'ver all','firmware.txt')
        status=shell(client,'logger status','logger_before.txt')
        assert 'Not logging' in status or 'not running' in status,'Existing logger is active; no interruption permitted'
        apply_logging(client,baseline,ROOT/'px4_transaction.json')
        shell(client,'logger stop','logger_stop.txt')
        memory=shell(client,'free','memory_before_logger.txt');line=next(line for line in memory.splitlines() if 'Umem:' in line);assert int(line.split()[4])>196608,'Insufficient contiguous memory for reviewed buffer'
        shell(client,'logger start -b 128 -t -m all -f','logger_start.txt')
        status=shell(client,'logger status','logger_recording.txt')
        paths=re.findall(r'(/fs/microsd/log/[^\s\x1b]+\.ulg)',status)
        assert paths, f'No active native ULog path: {status}'
        native_path=paths[0]
        save('native_logger.json',{'remote_path':native_path,'restart_scope':'logger module only',
             'flight_controller_rebooted':False,'startup_flags':'logger start -b 128 -t -m all -f','logger_buffer_kib':128,
             'source_commit':'cb411db5b54a98442ab974292c81b74c057bdf25',
             'boot_behavior_independently_verified':False})
        save('px4_after_apply.json',{'identity':client.identity(),'parameters':client.read_params(LOG_KEYS)})
    except BaseException as error:
        save('setup_error.json',{'error':str(error)})
        raise
    finally:client.close()
    command=[str(RELEASE/'start_capture'),'--gps-baud','2000000','--expected-rawx-hz','20',
             '--duration-s','90','--output-root',str(ROOT/'capture')]
    with (ROOT/'capture_stdout.txt').open('w') as out,(ROOT/'capture_stderr.txt').open('w') as err:
        child=subprocess.Popen(command,cwd=RELEASE,env=dict(os.environ,CAPTURE_PYTHON=sys.executable),stdout=out,stderr=err)
        try:
            time.sleep(2)
            assert child.poll() is None
            save('movement_instruction_window.json',{'host_monotonic_ns':time.monotonic_ns(),
                'unix_ns':time.time_ns(),'duration_s':90,'instruction':'stationary, gentle manual orientation changes, stationary',
                'manual_motion_confirmed':False,'gnss_to_px4_clock_verified':False})
            print('READY_FOR_MANUAL_MOVEMENT',flush=True)
            code=child.wait(timeout=110)
        finally:
            if child.poll() is None:child.terminate();child.wait(timeout=10)
    session=next((ROOT/'capture').iterdir())
    manifest=json.loads((session/'manifest.json').read_text())
    save('capture_run.json',{'command':command,'exit_code':code,'session':str(session),'native_path':native_path})
    assert code==0 and manifest['status']=='complete',{'code':code,'status':manifest['status'],'issues':manifest['issues']['counts']}
    client=PX4Link()
    try:
        shell(client,'logger status','logger_after_recording.txt')
        shell(client,'logger stop','logger_close.txt')
        shell(client,'logger status','logger_final_status.txt')
        save('download.json',client.download(native_path,ROOT/'native.ulg'))
        save('px4_after.json',{'identity':client.identity(),'parameters':client.read_params(LOG_KEYS),'heartbeat':client.assert_disarmed()})
    finally:client.close()
    subprocess.run([sys.executable,'-m','detect2.acquisition.audit_session',str(session)],cwd=RELEASE,check=True)
    save('receiver_after.json',prior['snap'](2000000))
    save('system_after.json',prior['PRIOR']['state']())
    print('STAGE4_NATIVE_LOG_DOWNLOADED',ROOT,flush=True)


if __name__=='__main__':main()
