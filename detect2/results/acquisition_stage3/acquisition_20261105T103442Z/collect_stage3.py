"""After capture, package final evidence and a separate retained failed attempt."""
import hashlib
import json
from pathlib import Path
import runpy
import shutil
import subprocess
import sys
import tarfile

release=Path('/home/nss/SpoofingDetection/releases/acquisition_20261105T103442Z')
root=Path('/home/nss/SpoofingDetection/recordings_v2_stage3/acquisition_20261105T103442Z')
old=Path('/home/nss/SpoofingDetection/recordings_v2_stage3/acquisition_20261105T102353Z')
rows=subprocess.check_output(['ps','-eo','pid,args'],text=True).splitlines()
assert not any(' -m detect2.acquisition.capture ' in row for row in rows)
validation=json.loads((root/'stage3_validation.json').read_text())
assert all(validation['checks'].values())
manifest=json.loads((release/'release_manifest.json').read_text())
assert all(hashlib.sha256((release/p).read_bytes()).hexdigest()==h for p,h in manifest['source_files'].items())
for n in ['release_manifest.json','deployment_check.json','run_stage3.py']:(root/n).write_bytes((release/n).read_bytes())
files={str(p.relative_to(root)):{'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'bytes':p.stat().st_size}
       for p in root.rglob('*') if p.is_file() and p.name!='evidence_hashes.json'}
(root/'evidence_hashes.json').write_text(json.dumps(files,indent=2)+'\n')
for path in root.glob('*/*/audit.json'):
    a=json.loads(path.read_text());print('FINAL',path.parent.name,a['checks']['receiver_epoch_continuity'],flush=True)
archive=Path('/tmp/stage3_final_acquisition_20261105T103442Z.tar.gz')
with tarfile.open(archive,'w:gz') as t:t.add(root,arcname='evidence')
print('FINAL_ARCHIVE',archive.stat().st_size,hashlib.sha256(archive.read_bytes()).hexdigest(),flush=True)
# Audit the earlier failure only after the final capture has released the devices.
failed=next((old/'final_2000000').iterdir())
subprocess.run([sys.executable,'-m','detect2.acquisition.audit_session',str(failed)],cwd=release,check=True)
s=runpy.run_path(str(release/'run_stage3.py'));s['monitor_comms'](failed)
previous=Path('/home/nss/SpoofingDetection/recordings_v2_stage3/previous_acquisition_20261105T102353Z')
previous.mkdir(exist_ok=False)
shutil.copytree(failed,previous/'failed_capture')
for name in ['receiver_before.json','rollback_readback.json','comms_correction_error.json','profile_transaction.json',
             'baud_transaction.json','comms_transaction.json','comms_restore.json','profile_restore_2000000.json',
             'baud_restore_2000000.json','stage3_validation_initial.json','finalize_stage3.py']:
    if (old/name).exists():shutil.copy2(old/name,previous/name)
before=json.loads((old/'receiver_before.json').read_text());after=json.loads((old/'rollback_readback.json').read_text())
assert before['ram']==after['ram']
m=json.loads((failed/'manifest.json').read_text())
(previous/'previous_attempt_summary.json').write_text(json.dumps({'capture_status':m['status'],
    'dropped_bytes_by_stream':{k:v['dropped_bytes'] for k,v in m['streams'].items()},
    'automatic_rollback_verified':True,'restored_baud':after['ram']['CFG_UART1_BAUDRATE'],
    'superseded_by':release.name,'included_in_passing_acceptance':False},indent=2)+'\n')
hashes={str(p.relative_to(previous)):{'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'bytes':p.stat().st_size}
        for p in previous.rglob('*') if p.is_file()}
(previous/'evidence_hashes.json').write_text(json.dumps(hashes,indent=2)+'\n')
archive=Path('/tmp/stage3_previous_failure_20261105T102353Z.tar.gz')
with tarfile.open(archive,'w:gz') as t:t.add(previous,arcname='previous_evidence')
print('PREVIOUS_ARCHIVE',archive.stat().st_size,hashlib.sha256(archive.read_bytes()).hexdigest(),flush=True)
