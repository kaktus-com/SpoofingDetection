import json
from pathlib import Path
import threading
import time

import pytest

from SpoofingDetection.detect2.acquisition import runtime
from SpoofingDetection.detect2.acquisition.baudrate import KEY
from SpoofingDetection.detect2.acquisition.common import Issues,fingerprint
from SpoofingDetection.detect2.acquisition.live_health import LiveHealth
from SpoofingDetection.detect2.acquisition.ownership import CaptureLocks
from SpoofingDetection.detect2.acquisition.px4_logging import ArmedError
from SpoofingDetection.detect2.acquisition.replay import synthetic_inputs
from SpoofingDetection.detect2.acquisition.ubx import frames,packet


PROFILE=json.loads(runtime.PROFILE.read_text())


class ReceiverState:
    def __init__(self,rate=921600):
        self.values={**runtime.profile_values(),KEY:rate,**{k:0 for k in runtime.SIGNALS}}
        self.values['CFG_RATE_MEAS']=40;self.writes=[];self.opened=[];self.wrong_model=False
    def backend(self,device,rate):
        state=self;state.opened.append(rate)
        class Backend:
            def __init__(self):self.host=rate
            def identity(self):
                if self.host!=state.values[KEY]:raise TimeoutError('No decoded reply at this host baud')
                return {'reported_receiver':{'model':'MOD=other' if state.wrong_model else 'MOD=ZED-X20P',
                          'firmware':'FWVER=HPG 2.00','protocol':'PROTVER=50.02'},'transport_device':device}
            def read(self,keys):self.identity();return {k:state.values[k] for k in keys}
            def set_ram(self,values):state.writes.append(dict(values));state.values.update(values)
            def switch_ram(self,new):state.writes.append({KEY:new});state.values[KEY]=new;self.host=new
            def host_rate(self,new):self.host=new
            def close(self):pass
        return Backend()


class ControllerState:
    def __init__(self):
        self.values=dict(runtime.DESIRED);self.active='/fs/microsd/log/prior.ulg';self.calls=[]
        self.uid=PROFILE['px4_uid'];self.armed=False;self.fail_download=False
    def client(self,endpoint):
        state=self
        if state.armed:raise ArmedError('armed: keep native logging')
        class Client:
            def close(self):pass
            def assert_disarmed(self):
                if state.armed:raise ArmedError('armed')
            def identity(self):return {'uid':state.uid,'firmware':PROFILE['px4_firmware_hash']}
            def read_params(self,keys):return {k:state.values[k] for k in keys}
            def set_param(self,k,v):state.values[k]=v
            def shell(self,command,timeout=3):
                state.calls.append(command)
                if command=='ver all':return 'PX4 git-hash: '+PROFILE['px4_firmware_hash']+'\n'
                if command=='free':return 'Umem: 969072 337200 631872 475952 1492 109\n'
                if command=='logger stop':self.assert_disarmed();state.active=None;return 'stopped'
                if command==runtime.START_LOGGER:
                    self.assert_disarmed();state.active='/fs/microsd/log/owned.ulg';return 'started'
                if command=='logger status':
                    return f'Full File Logging Running:\nLog file: {state.active}\ndropouts: 0, max used buffer: 32000 / 131072 B' if state.active else 'module not running'
                raise AssertionError(command)
            def download(self,remote,path):
                self.assert_disarmed()
                if state.fail_download:raise TimeoutError('download interrupted')
                path.write_bytes(b'complete fake native fixture')
                return {'fingerprint':fingerprint(path),'remote_path':remote,'sdk_no_gap_completion':True}
        return Client()


def fake_workers(disconnect=False):
    receiver,datagrams=synthetic_inputs(False)
    rawx=[packet(c,i,p) for _,c,i,p in frames(receiver,{}) if (c,i)==(2,21)]
    class Serial(threading.Thread):
        def __init__(self,device,baud,archive,stop,issues,**kwargs):
            super().__init__();self.archive,self.stop,self.issues=archive,stop,issues;self.observer=kwargs['observer']
            assert kwargs['reconnect'] is False
        def run(self):
            for i,data in enumerate(rawx):
                if self.stop.is_set():return
                now=time.monotonic_ns();self.archive.submit(data,now);self.observer(data,now)
                if disconnect and i==3:
                    self.issues.add('serial_source_error',error='simulated unplug');self.stop.set();return
                self.stop.wait(.05)
    class Mav(threading.Thread):
        def __init__(self,bind,archive,stop,issues,observer_factory):
            super().__init__();self.archive,self.stop=archive,stop;self.observer=observer_factory(None)
        def run(self):
            self.observer.start()
            try:
                for data in datagrams:
                    if self.stop.is_set():return
                    now=time.monotonic_ns();self.archive.submit(data,now,peer=('127.0.0.1',1))
                    self.observer.observe(data,now,('127.0.0.1',1));self.stop.wait(.05)
            finally:self.observer.finished.set();self.observer.join(1)
    return Serial,Mav


def test_runtime_recovers_reset_baud_preserves_constellations_and_collects_one_bundle(tmp_path):
    receiver=ReceiverState();controller=ControllerState();serial,mav=fake_workers()
    path,code=runtime.run(tmp_path,'fake-device',('127.0.0.1',14551),PROFILE,.15,
        backend_factory=receiver.backend,client_factory=controller.client,serial_factory=serial,mavlink_factory=mav)
    state=json.loads((path/'runtime_manifest.json').read_text())
    assert code==0 and state['status']=='complete' and state['recording_ready']
    assert receiver.opened==[2000000,921600] and receiver.values[KEY]==2000000
    assert receiver.values['CFG_RATE_MEAS']==50 and all(receiver.values[k]==0 for k in runtime.SIGNALS)
    assert all(set(w)<=set(runtime.profile_values())|{KEY} for w in receiver.writes)
    assert (path/'native.ulg').exists() and not (path/'RUNTIME_UNFINALIZED').exists()
    assert controller.active is None
    raw=json.loads((path/'manifest.json').read_text())
    assert raw['status']=='complete' and all(v['received_bytes']==v['written_bytes'] for v in raw['streams'].values())
    assert all(fingerprint(path/name)==value for name,value in raw['files'].items())
    assert all(fingerprint(path/name)==value for name,value in state['files'].items())


def test_disconnect_fails_session_and_never_reconfigures_mid_recording(tmp_path):
    receiver=ReceiverState(2000000);controller=ControllerState();serial,mav=fake_workers(True)
    path,code=runtime.run(tmp_path,'fake-device',('127.0.0.1',14551),PROFILE,.8,
        backend_factory=receiver.backend,client_factory=controller.client,serial_factory=serial,mavlink_factory=mav)
    assert code==2 and json.loads((path/'runtime_manifest.json').read_text())['status']=='failed'
    assert receiver.opened==[2000000]
    assert json.loads((path/'manifest.json').read_text())['issues']['counts']['serial_source_error']==1
    assert (path/'native.ulg').exists()


def test_identity_mismatch_never_configures_receiver_or_stops_existing_native_log(tmp_path):
    controller=ControllerState();controller.uid=999;receiver=ReceiverState()
    path,code=runtime.run(tmp_path,'fake',('127.0.0.1',14551),PROFILE,.1,
                          backend_factory=receiver.backend,client_factory=controller.client)
    assert code==2 and not receiver.opened and 'logger stop' not in controller.calls
    assert json.loads((path/'runtime_manifest.json').read_text())['status']=='failed'


def test_wrong_receiver_never_writes_and_setup_failure_is_explicit(tmp_path):
    receiver=ReceiverState(2000000);receiver.wrong_model=True
    with pytest.raises(RuntimeError,match='Unexpected receiver'):
        runtime.configure_receiver(tmp_path,'fake',receiver.backend)
    assert not receiver.writes


def test_armed_cleanup_defers_native_and_later_finalizes_without_touching_receiver(tmp_path):
    controller=ControllerState();controller.active='/fs/microsd/log/owned.ulg';controller.armed=True
    state={'mavlink_bind':'127.0.0.1:14551','px4_identity':{'uid':controller.uid,'firmware':PROFILE['px4_firmware_hash']},
           'native_path':controller.active}
    assert not runtime.collect_native(tmp_path,state,controller.client)
    assert state['native_collection']=='pending' and not controller.calls
    controller.armed=False
    assert runtime.collect_native(tmp_path,state,controller.client)
    assert state['native_collection']=='collected' and controller.active is None


def test_newer_native_log_is_not_stopped_and_failed_download_is_not_complete(tmp_path):
    controller=ControllerState();controller.fail_download=True
    state={'mavlink_bind':'127.0.0.1:14551','px4_identity':{'uid':controller.uid,'firmware':PROFILE['px4_firmware_hash']},
           'native_path':'/fs/microsd/log/older.ulg'}
    assert not runtime.collect_native(tmp_path,state,controller.client)
    assert state['native_collection']=='pending' and 'logger stop' not in controller.calls
    assert state['newer_native_log_left_running']==controller.active


def test_global_locks_exclude_different_output_roots_and_shared_endpoints(tmp_path):
    with CaptureLocks('fake-a',('127.0.0.1',14551),tmp_path):
        with pytest.raises(RuntimeError,match='already owned'):
            with CaptureLocks('fake-b',('127.0.0.1',14551),tmp_path):pass
        with pytest.raises(RuntimeError,match='already owned'):
            with CaptureLocks('fake-a',('127.0.0.1',14552),tmp_path):pass
    with CaptureLocks('fake-a',('127.0.0.1',14551),tmp_path):pass


def test_live_guard_exposes_missing_epochs_clock_reset_and_stale_streams():
    receiver,datagrams=synthetic_inputs(False);packets=[packet(c,i,p) for _,c,i,p in frames(receiver,{}) if (c,i)==(2,21)]
    stop=threading.Event();issues=Issues();live=LiveHealth(stop,issues)
    live.receiver(packets[0],1_000_000_000);live.receiver(packets[2],1_100_000_000)
    assert stop.is_set() and issues.snapshot()['counts']['rawx_interval_discontinuity']==1
    stop=threading.Event();issues=Issues();live=LiveHealth(stop,issues)
    live.observe(datagrams[2],1_100_000_000,('',0));live.observe(datagrams[0],1_200_000_000,('',0))
    assert stop.is_set() and issues.snapshot()['counts']['px4_clock_reset']==1
    stop=threading.Event();issues=Issues();live=LiveHealth(stop,issues)
    live.check(1_000_000_000,10_000_000_000)
    assert stop.is_set() and issues.snapshot()['counts']['required_stream_stale']==1


def test_ctrl_c_finalizes_both_logs_and_returns_interrupt_code(tmp_path):
    receiver=ReceiverState(2000000);controller=ControllerState();serial,mav=fake_workers()
    stop=threading.Event();reason=[]
    def interrupt():reason.append('sigint');stop.set()
    timer=threading.Timer(.3,interrupt);timer.start()
    try:
        path,code=runtime.run(tmp_path,'fake',('127.0.0.1',14551),PROFILE,None,stop,reason,
            backend_factory=receiver.backend,client_factory=controller.client,serial_factory=serial,mavlink_factory=mav)
    finally:timer.join()
    state=json.loads((path/'runtime_manifest.json').read_text())
    assert code==130 and state['status']=='stopped' and state['native_collection']=='collected'
    assert not (path/'UNFINALIZED').exists() and not (path/'RUNTIME_UNFINALIZED').exists()


def test_unverified_native_buffer_is_closed_and_not_reported_ready(tmp_path):
    controller=ControllerState();original=controller.client
    def client_factory(endpoint):
        c=original(endpoint);shell=c.shell
        def altered(command,timeout=3):
            result=shell(command,timeout)
            return result.replace('/ 131072 B','/ 8192 B') if command=='logger status' and controller.active and controller.active.endswith('owned.ulg') else result
        c.shell=altered;return c
    receiver=ReceiverState(2000000)
    path,code=runtime.run(tmp_path,'fake',('127.0.0.1',14551),PROFILE,.1,
        backend_factory=receiver.backend,client_factory=client_factory)
    assert code==2 and controller.active is None
    assert not json.loads((path/'runtime_manifest.json').read_text())['recording_ready']
    assert (path/'failed_native_setup_cleanup.json').exists()


def test_recording_that_ends_armed_keeps_native_active_until_explicit_finalization(tmp_path):
    controller=ControllerState();receiver=ReceiverState(2000000);serial,mav=fake_workers()
    timer=threading.Timer(.18,lambda:setattr(controller,'armed',True));timer.start()
    try:
        path,code=runtime.run(tmp_path,'fake',('127.0.0.1',14551),PROFILE,.25,
            backend_factory=receiver.backend,client_factory=controller.client,serial_factory=serial,mavlink_factory=mav)
    finally:timer.join()
    state=json.loads((path/'runtime_manifest.json').read_text())
    assert code==3 and state['status']=='pending_native' and controller.active.endswith('owned.ulg')
    assert (path/'RUNTIME_UNFINALIZED').exists() and not (path/'UNFINALIZED').exists()
    opened=list(receiver.opened);controller.armed=False
    assert runtime.collect_native(path,state,controller.client)
    assert runtime.final_status(path,state)==0 and receiver.opened==opened
    assert not (path/'RUNTIME_UNFINALIZED').exists()
