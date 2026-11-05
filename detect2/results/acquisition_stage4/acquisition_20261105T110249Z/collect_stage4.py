"""Collect immutable native/raw evidence; leave the tested logger active."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile

release=Path('/home/nss/SpoofingDetection/releases/acquisition_20261105T110249Z')
root=Path('/home/nss/SpoofingDetection/recordings_v2_stage4/acquisition_20261105T110249Z')
old=Path('/home/nss/SpoofingDetection/recordings_v2_stage4/acquisition_20261105T105214Z')
sys.path.insert(0,str(release))
from detect2.acquisition.px4_logging import PX4Link
from detect2.acquisition.common import atomic_json

assert (root/'native.ulg').exists() and (root/'px4_after.json').exists()
session=next((root/'capture').iterdir())
assert json.loads((session/'manifest.json').read_text())['status']=='complete'
rows=subprocess.check_output(['ps','-eo','pid,args'],text=True).splitlines()
assert not any(' -m detect2.acquisition.capture ' in row for row in rows)
client=PX4Link()
try:
    result=client.shell('logger start -b 128 -t -m all -f',4)
    (root/'logger_left_running_start.txt').write_text(result)
    status=client.shell('logger status',4)
    (root/'logger_left_running_status.txt').write_text(status)
    assert 'Full File Logging Running' in status and '/ 131072 B' in status,status
    atomic_json(root/'runtime_final_state.json',{'native_logger_running':True,'logger_buffer_kib':128,
        'buffer_persisted':False,'receiver_profile_persisted':False,'raw_recorder_running':False,
        'whole_autopilot_rebooted':False,'next_boot_buffer_kib':8,
        'px4_logging_parameters':client.read_params(['SDLOG_PROFILE','SDLOG_MODE','SDLOG_BACKEND'])})
finally:client.close()
for n in ['release_manifest.json','deployment_check.json','run_stage4.py']:(root/n).write_bytes((release/n).read_bytes())
meta=json.loads((root/'release_manifest.json').read_text())
assert all(hashlib.sha256((release/p).read_bytes()).hexdigest()==sha for p,sha in meta['source_files'].items())
hashes={str(p.relative_to(root)):{'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'bytes':p.stat().st_size}
        for p in root.rglob('*') if p.is_file() and p.name!='evidence_hashes.json'}
(root/'evidence_hashes.json').write_text(json.dumps(hashes,indent=2)+'\n')
archive=Path('/tmp/stage4_final_acquisition_20261105T110249Z.tar.gz')
with tarfile.open(archive,'w:gz') as t:t.add(root,arcname='evidence')
print('FINAL_ARCHIVE',archive.stat().st_size,hashlib.sha256(archive.read_bytes()).hexdigest(),flush=True)
failed=next((old/'capture').iterdir())
subprocess.run([sys.executable,'-m','detect2.acquisition.audit_session',str(failed)],cwd=release,check=True)
previous=Path('/home/nss/SpoofingDetection/recordings_v2_stage4/previous_acquisition_20261105T105214Z');previous.mkdir(exist_ok=False)
shutil.copytree(failed,previous/'failed_raw_capture')
for name in ['native_closed.ulg','download_closed.json','px4_before.json','px4_transaction.json','native_logger.json',
             'logger_after_recording.txt','logger_final_status.txt','logger_stop_final.txt','logger_stopped_status.txt',
             'memory_before_buffer_test.txt','receiver_after_disconnect_921600.json','receiver_reapply_transaction.json',
             'receiver_reapply_baud.json','receiver_after_reapply.json','firmware.txt']:
    if (old/name).exists():shutil.copy2(old/name,previous/name)
result=subprocess.run(['journalctl','-k','--since','1 hour ago','--no-pager','-n','150'],capture_output=True,text=True)
(previous/'kernel_usb_log.txt').write_text(result.stdout+'\n'+result.stderr)
hashes={str(p.relative_to(previous)):{'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'bytes':p.stat().st_size}
        for p in previous.rglob('*') if p.is_file()}
(previous/'evidence_hashes.json').write_text(json.dumps(hashes,indent=2)+'\n')
archive=Path('/tmp/stage4_previous_acquisition_20261105T105214Z.tar.gz')
with tarfile.open(archive,'w:gz') as t:t.add(previous,arcname='previous_evidence')
print('PREVIOUS_ARCHIVE',archive.stat().st_size,hashlib.sha256(archive.read_bytes()).hexdigest(),flush=True)
