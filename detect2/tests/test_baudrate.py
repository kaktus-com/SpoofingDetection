import json

import pytest

from SpoofingDetection.detect2.acquisition.baudrate import KEY, transition


class Receiver:
    def __init__(self):
        self.rate = self.host = 921600
        self.writes = []
        self.fail_target = False
        self.reject_write = False

    def identity(self):
        if self.host != self.rate or (self.fail_target and self.host == 2000000):
            raise TimeoutError('No decoded identity at host baud')
        return {'serial': 'test-device'}

    def read(self, keys):
        self.identity()
        return {KEY: self.rate}

    def host_rate(self, rate):
        self.host = rate

    def switch_ram(self, rate):
        self.writes.append(rate)
        if not self.reject_write:
            self.rate = rate
        self.host = rate


def test_new_rate_requires_identity_and_readback_and_can_restore(tmp_path):
    r = Receiver()
    a = transition(r, 921600, 2000000, tmp_path/'apply.json')
    assert a['status'] == 'applied_and_verified' and r.rate == 2000000
    b = transition(r, 2000000, 921600, tmp_path/'restore.json')
    assert b['status'] == 'applied_and_verified' and r.rate == 921600
    assert r.writes == [2000000, 921600]


def test_rejected_switch_recovers_original_host_rate(tmp_path):
    r = Receiver(); r.reject_write = True
    with pytest.raises(RuntimeError, match='rolled_back'):
        transition(r, 921600, 2000000, tmp_path/'journal.json')
    a = json.loads((tmp_path/'journal.json').read_text())
    assert a['status'] == 'rolled_back' and r.host == r.rate == 921600
    assert r.writes == [2000000]


def test_unreachable_receiver_never_claims_success(tmp_path):
    r = Receiver(); r.fail_target = True
    with pytest.raises(RuntimeError, match='recovery_failed'):
        transition(r, 921600, 2000000, tmp_path/'journal.json')
    assert json.loads((tmp_path/'journal.json').read_text())['status'] == 'recovery_failed'
    assert r.writes == [2000000]


def test_stale_or_unreviewed_rate_and_existing_journal_never_write(tmp_path):
    r = Receiver(); p = tmp_path/'journal.json'
    with pytest.raises(ValueError): transition(r, 921600, 3000000, p)
    r.rate = r.host = 2000000
    with pytest.raises(RuntimeError, match='no write'): transition(r, 921600, 2000000, p)
    p.write_text('preserve')
    with pytest.raises(ValueError, match='exists'): transition(r, 2000000, 921600, p)
    assert r.writes == [] and p.read_text() == 'preserve'


def test_journal_exists_before_switch_and_identity_mismatch_blocks_recovery(tmp_path):
    r = Receiver(); p = tmp_path/'journal.json'
    original = r.switch_ram
    def switch(rate):
        assert json.loads(p.read_text())['status'] == 'prepared'
        original(rate)
        r.identity = lambda: {'serial': 'different-device'}
    r.switch_ram = switch
    with pytest.raises(RuntimeError, match='recovery_failed'):
        transition(r, 921600, 2000000, p)
    assert r.writes == [2000000]
