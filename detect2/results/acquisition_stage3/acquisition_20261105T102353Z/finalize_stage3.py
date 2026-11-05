"""Correct the observed MON-COMMS base-cycle assumption and repeat acceptance.

Adds one journaled RAM-only correction (MON-COMMS divisor 20 -> 1). The initial
dated release remains unchanged. Restore follows both saved transactions.
"""
import hashlib
import json
from pathlib import Path
import runpy
import sys
import time

RELEASE = Path('/home/nss/SpoofingDetection/releases/acquisition_20261105T102353Z')
s = runpy.run_path(str(RELEASE/'run_stage3.py'))
ROOT = s['ROOT']
load = lambda name: json.loads((ROOT/name).read_text())
save = s['save']


def restore():
    if (ROOT/'comms_transaction.json').exists():
        backend = s['connect'](2000000)
        try:
            s['restore_ram'](backend, load('comms_transaction.json'), ROOT/'comms_restore.json')
        finally:
            backend.close()
    s['rollback']()
    print('RESTORED_ORIGINAL_PROFILE_AND_BAUD', flush=True)


def main():
    if len(sys.argv)>1 and sys.argv[1]=='restore':
        restore(); return
    assert all(load('stage3_validation.json')['checks'].values())
    for name in ['stage3_validation.json','receiver_after.json','flash_after.json']:
        dest=ROOT/(Path(name).stem+'_initial.json')
        assert not dest.exists()
        dest.write_bytes((ROOT/name).read_bytes())
    if (ROOT/'evidence_hashes.json').exists():
        (ROOT/'evidence_hashes_initial.json').write_bytes((ROOT/'evidence_hashes.json').read_bytes())
    before=s['snap'](2000000)
    save('receiver_before_comms_correction.json',before)
    desired=s['profile_values']()
    desired['CFG_MSGOUT_UBX_MON_COMMS_UART1']=1
    save('final_profile.json',{'settings':desired,'expected_rawx_hz':20,'host_baud':2000000,
        'reason':'Initial divisor 20 yielded only 6 MON-COMMS packets in 120 s; divisor 1 targets the observed one-second base cycle.'})
    backend=s['connect'](2000000)
    try:
        s['apply_ram'](backend,before,desired,ROOT/'comms_transaction.json')
    finally:
        backend.close()
    try:
        time.sleep(3)
        final=s['capture']('final_2000000',2000000,120)
        after=s['snap'](2000000)
        save('receiver_after.json',after)
        save('flash_after.json',s['flash_snapshot'](2000000))
        save('px4_after.json',s['PRIOR']['px4_snapshot']())
        save('system_after.json',s['PRIOR']['state']())
        validation=load('stage3_validation_initial.json')
        validation['sessions'].append(final)
        validation['checks']['profile_readback_matches']=all(after['ram'][k]==v for k,v in desired.items())
        initial=load('receiver_before.json');pre=load('system_before.json');post=load('system_after.json')
        validation['checks'].update(
            receiver_baud_is_2000000=after['ram'][s['KEY']]==2000000,
            constellation_enable_flags_unchanged=all(initial['ram'][k]==after['ram'][k] for k in s['SIGNALS']),
            receiver_identity_unchanged=initial['identity']==after['identity'],
            legacy_files_unchanged=pre['legacy_files']==post['legacy_files'],
            router_process_unchanged=pre['router_processes']==post['router_processes'],
            px4_logging_parameters_unchanged=load('px4_before.json')['parameters']==load('px4_after.json')['parameters'])
        validation.update(final_profile='final_profile.json',comms_correction='20 -> 1; one-second base cycle',
            rollback_command='/home/nss/venv/bin/python finalize_stage3.py restore')
        save('stage3_validation.json',validation)
        assert all(validation['checks'].values()),validation
        print('FINAL_STAGE3_DONE',flush=True)
    except BaseException as error:
        save('comms_correction_error.json',{'error':str(error)})
        restore()
        raise


if __name__=='__main__':main()
