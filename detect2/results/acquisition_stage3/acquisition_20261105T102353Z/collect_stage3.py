"""Package finalized live evidence; no sensor or configuration commands."""
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile

root=Path('/home/nss/SpoofingDetection/recordings_v2_stage3/acquisition_20261105T102353Z')
release=Path('/home/nss/SpoofingDetection/releases/acquisition_20261105T102353Z')
validation=json.loads((root/'stage3_validation.json').read_text())
assert 'final_profile' in validation and all(validation['checks'].values())
for name in ['release_manifest.json','deployment_check.json','run_stage3.py','finalize_stage3.py',
             'correction_manifest.json','x20_recording_profile_final.json']:
    (root/name).write_bytes((release/name).read_bytes())
correction=json.loads((root/'correction_manifest.json').read_text())
assert hashlib.sha256((root/'finalize_stage3.py').read_bytes()).hexdigest()==correction['correction_file_sha256']
assert hashlib.sha256((root/'x20_recording_profile_final.json').read_bytes()).hexdigest()==correction['final_profile_sha256']
initial=json.loads((root/'release_manifest.json').read_text())
assert all(hashlib.sha256((release/p).read_bytes()).hexdigest()==sha for p,sha in initial['source_files'].items())
rows=subprocess.check_output(['ps','-eo','pid,args'],text=True).splitlines()
assert not any(' -m detect2.acquisition.capture ' in row for row in rows)
print('No recorder remains; original release and corrected profile hashes verified',flush=True)
for p in root.glob('*/*/audit.json'):
    a=json.loads(p.read_text())
    c=json.loads(p.with_name('receiver_comms.json').read_text())
    uart=[port for row in c for port in row['ports'] if port['portId']==256]
    print(p.parent.name,'RAWX',a['receiver']['epochs'],'continuity',a['checks']['receiver_epoch_continuity'],
          'MON-COMMS',len(uart),'pending',max(v['txPending'] for v in uart),flush=True)
files={str(p.relative_to(root)):{'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'bytes':p.stat().st_size}
       for p in root.rglob('*') if p.is_file() and p.name!='evidence_hashes.json'}
(root/'evidence_hashes.json').write_text(json.dumps(files,indent=2)+'\n')
archive=Path('/tmp/stage3_final_acquisition_20261105T102353Z.tar.gz')
with tarfile.open(archive,'w:gz') as t:t.add(root,arcname='evidence')
print('FINAL_ARCHIVE',archive.stat().st_size,hashlib.sha256(archive.read_bytes()).hexdigest(),flush=True)
