import json
from pathlib import Path

import pytest

from SpoofingDetection.detect2.acquisition.px4_logging import PX4Link,apply_logging, int32_wire, wire_int32


class Client:
    def __init__(self):
        self.parameters={'SDLOG_PROFILE':1,'SDLOG_MODE':0,'SDLOG_BACKEND':3}
        self.writes=[];self.armed=False;self.fail=False;self.journal=None
    def assert_disarmed(self):
        if self.armed:raise RuntimeError('armed')
    def identity(self):return {'uid':123,'firmware':'matching-hash'}
    def read_params(self,keys):return {k:self.parameters[k] for k in keys}
    def set_param(self,key,value):
        assert self.journal.exists()
        self.writes.append((key,value))
        if self.fail and key=='SDLOG_MODE' and value==2:raise TimeoutError('lost ACK')
        self.parameters[key]=value
    def baseline(self):return {'identity':self.identity(),'parameters':dict(self.parameters)}


def test_bytewise_int32_survives_wire_roundtrip():
    for value in [-1,0,1,2,17,4095]:assert wire_int32(int32_wire(value))==value
    assert wire_int32(17.0)!=17  # numeric float encoding is wrong for this PX4.


def test_logging_journal_precedes_only_reviewed_parameter_writes(tmp_path):
    c=Client();c.journal=tmp_path/'journal.json'
    result=apply_logging(c,c.baseline(),c.journal)
    assert result['status']=='applied_and_verified'
    assert c.writes==[('SDLOG_PROFILE',17),('SDLOG_MODE',2)]
    assert c.parameters['SDLOG_BACKEND']==3


def test_armed_stale_identity_or_existing_journal_refuses_writes(tmp_path):
    for failure in ['armed','stale','identity','journal']:
        c=Client();c.journal=tmp_path/f'{failure}.json';baseline=c.baseline()
        if failure=='armed':c.armed=True
        if failure=='stale':c.parameters['SDLOG_PROFILE']=5
        if failure=='identity':baseline['identity']={'uid':'other'}
        if failure=='journal':c.journal.write_text('preserve')
        with pytest.raises((RuntimeError,ValueError)):apply_logging(c,baseline,c.journal)
        assert not c.writes


def test_failed_apply_restores_saved_logging_parameters(tmp_path):
    c=Client();c.journal=tmp_path/'journal.json';c.fail=True;baseline=c.baseline()
    with pytest.raises(RuntimeError,match='rolled_back'):apply_logging(c,baseline,c.journal)
    assert c.parameters==baseline['parameters']
    assert json.loads(c.journal.read_text())['rollback_readback']==baseline['parameters']


def test_large_download_continues_progress_and_uses_sdk_completion_not_cleared_eof(tmp_path,monkeypatch):
    from types import SimpleNamespace
    from pymavlink import mavftp
    content=b'ULog actual test bytes'
    class FTP:
        def __init__(self,*args):
            self.ftp_settings=SimpleNamespace(burst_read_size=80)
            self.done=False;self.get_result=None;self.remote_file_size=len(content)
            self.read_total=0;self.read_gaps=[];self.calls=0
        def cmd_get(self,args):self.path=Path(args[1]);return SimpleNamespace(return_code=0)
        def process_ftp_reply(self,*args,**kwargs):
            self.calls+=1
            if self.calls==1:self.read_total=5;return SimpleNamespace(return_code=73)
            self.path.write_bytes(content);self.done=True;self.get_result=content
            self.reached_eof=False  # cleared by the installed SDK on termination
            return SimpleNamespace(return_code=0)
    monkeypatch.setattr(mavftp,'MAVFTP',FTP)
    c=PX4Link.__new__(PX4Link);c.master=object();c.assert_disarmed=lambda:None
    path=tmp_path/'native.ulg'
    result=c.download('/fs/microsd/log/test/native.ulg',path)
    assert path.read_bytes()==content and result['completed_bytes']==len(content)
    assert result['sdk_no_gap_completion']


def test_download_size_mismatch_and_unreviewed_parameters_are_refused(tmp_path,monkeypatch):
    from types import SimpleNamespace
    from pymavlink import mavftp
    class FTP:
        def __init__(self,*args):self.ftp_settings=SimpleNamespace(burst_read_size=80)
        def cmd_get(self,args):self.path=Path(args[1]);return SimpleNamespace(return_code=0)
        def process_ftp_reply(self,*args,**kwargs):
            self.path.write_bytes(b'partial');self.done=True;self.get_result=b'partial';self.remote_file_size=100
            return SimpleNamespace(return_code=0)
    monkeypatch.setattr(mavftp,'MAVFTP',FTP)
    c=PX4Link.__new__(PX4Link);c.master=object();c.assert_disarmed=lambda:None
    with pytest.raises(RuntimeError,match='size differs'):c.download('/fs/microsd/log/test/native.ulg',tmp_path/'native.ulg')
    with pytest.raises(ValueError):c.set_param('SDLOG_MODE',17)
    with pytest.raises(ValueError):c.set_param('COM_ARM_AUTH_REQ',1)
