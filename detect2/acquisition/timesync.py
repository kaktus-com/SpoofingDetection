"""Optional Pi↔PX4 clock estimation. No GNSS synchronization is implied.

Exact request-token/source matching supports installed legacy TIMESYNC fields;
target extensions are checked when present. Offset = remote minus host.
"""
from collections import deque
import json
import queue
import statistics
import csv
import os
import struct
import threading
import time


def pack_timesync(encoder,tc1,ts1,target):
    """Encode the standard MAVLink 2 extensions with the pinned legacy SDK.

    TIMESYNC's CRC-extra is unchanged by extension fields. The installed
    generated class lacks them; its normal packet builder handles CRC/framing.
    """
    from pymavlink.dialects.v20 import common as mav
    message=mav.MAVLink_timesync_message(int(tc1),int(ts1))
    return message._pack(encoder,message.crc_extra,struct.pack('<qqBB',int(tc1),int(ts1),*target))


def message_target(message):
    # get_payload() in the pinned SDK includes four header bytes for decoded
    # MAVLink 2 packets. Extract the actual payload from the original frame.
    raw=bytes(message.get_msgbuf())
    if raw and raw[0]==0xfd:
        payload=raw[10:10+raw[1]]
        return (payload[16] if len(payload)>16 else 0,payload[17] if len(payload)>17 else 0)
    return (0,0)


class TimeSyncTracker:
    def __init__(self, target=(1,1), identity=(245,191), max_rtt_ns=20_000_000,
                 jump_ns=100_000_000, expiry_ns=2_000_000_000):
        self.target,self.identity = target,identity
        self.max_rtt_ns,self.jump_ns,self.expiry_ns = max_rtt_ns,jump_ns,expiry_ns
        self.pending = {}; self.samples = deque(maxlen=31); self.segment = 0; self.last_token = 0
        self.expired=[]

    def new_segment(self):
        abandoned=list(self.pending)
        self.segment+=1;self.samples.clear();self.pending.clear()
        return abandoned

    def request(self, now):
        self.expired=[t for t,s in self.pending.items() if now-s >= self.expiry_ns]
        self.pending = {t:s for t,s in self.pending.items() if now-s < self.expiry_ns}
        token = max(int(now),self.last_token+1); self.last_token = token
        self.pending[token] = int(now)
        return token

    def response(self, source, tc1, ts1, now, target=None):
        row={'host_monotonic_ns':int(now),'source_system':source[0],'source_component':source[1],
             'tc1_ns':int(tc1),'ts1_ns':int(ts1),'segment_id':self.segment,'accepted':False}
        if target is not None:row.update(target_system=target[0],target_component=target[1])
        if source != self.target: return {**row,'reason':'wrong_source'}
        if target and (target[0] not in (0,self.identity[0]) or target[1] not in (0,self.identity[1])):
            return {**row,'reason':'wrong_target'}
        if tc1 <= 0: return {**row,'reason':'remote_request_or_invalid_time'}
        if ts1 not in self.pending: return {**row,'reason':'unmatched_or_duplicate_token'}
        sent=self.pending.pop(ts1); rtt=int(now)-sent
        row['round_trip_ns']=rtt
        row['host_send_monotonic_ns']=sent
        row['host_midpoint_ns']=(sent+int(now))//2
        if rtt < 0 or rtt > self.max_rtt_ns: return {**row,'reason':'round_trip_rejected'}
        offset=int(tc1)-(sent+int(now))//2
        if self.samples and abs(offset-statistics.median(self.samples)) > self.jump_ns:
            abandoned=self.new_segment()
            row['clock_discontinuity']=True
            row['abandoned_request_tokens']=abandoned
        self.samples.append(offset)
        return {**row,'accepted':True,'reason':'accepted','segment_id':self.segment,
                'remote_minus_host_ns':offset,'filtered_offset_ns':int(statistics.median(self.samples)),
                'half_round_trip_proxy_ns':rtt//2,'accepted_samples_in_segment':len(self.samples),
                'state':'warming_up' if len(self.samples)<5 else 'estimated_not_independently_verified'}


class TimeSyncObserver(threading.Thread):
    """Separate bounded decoder. Raw acquisition never waits on this observer."""
    def __init__(self, folder, sock, stop, issues, active=False, target=(1,1), identity=(245,191),
                 gap_ns=1_000_000_000):
        super().__init__(name='timesync-observer',daemon=True)
        self.folder,self.sock,self.stop,self.issues = folder,sock,stop,issues
        self.active,self.target,self.identity = active,target,identity
        self.gap_ns=gap_ns
        self.queue = queue.Queue(64); self.finished = threading.Event()

    def observe(self, data, host_ns, peer):
        try: self.queue.put_nowait((data,host_ns,peer))
        except queue.Full: self.issues.add('timesync_observer_overflow')

    def run(self):
        try:
            from pymavlink.dialects.v20 import common as mav
            parsers={}; tracker=TimeSyncTracker(self.target,self.identity)
            encoder=mav.MAVLink(None,srcSystem=self.identity[0],srcComponent=self.identity[1])
            last_request=0; peer=None;last_accepted=None;gap_active=False;position=0;sequence=0
            with (self.folder/'clock_sync.jsonl').open('w') as log, (self.folder/'mavlink_tx.bin').open('wb') as tx, (self.folder/'mavlink_tx.csv').open('w',newline='') as index:
                writer=csv.DictWriter(index,fieldnames=['sequence','byte_offset','length','host_send_monotonic_ns',
                    'peer_host','peer_port','tc1_ns','ts1_ns','target_system','target_component'])
                writer.writeheader()
                def event(row):log.write(json.dumps(row)+'\n');log.flush()
                def send(tc1,ts1,target):
                    nonlocal position,sequence
                    raw=pack_timesync(encoder,tc1,ts1,target);encoder.seq=(encoder.seq+1)%256
                    sent=time.monotonic_ns()
                    self.sock.sendto(raw,peer);tx.write(raw);tx.flush()
                    writer.writerow(dict(sequence=sequence,byte_offset=position,length=len(raw),host_send_monotonic_ns=sent,
                        peer_host=peer[0],peer_port=peer[1],tc1_ns=tc1,ts1_ns=ts1,target_system=target[0],target_component=target[1]))
                    index.flush();position+=len(raw);sequence+=1
                event({'event':'clock_segment_start','segment_id':0,'reason':'observer_start','host_monotonic_ns':time.monotonic_ns()})
                while not self.finished.is_set() or not self.queue.empty():
                    check_now=time.monotonic_ns()
                    if last_accepted is not None and not gap_active and check_now-last_accepted>self.gap_ns and not self.stop.is_set():
                        abandoned=tracker.new_segment();gap_active=True
                        event({'event':'clock_segment_start','segment_id':tracker.segment,'reason':'accepted_reply_gap',
                            'host_monotonic_ns':check_now,'last_accepted_reply_ns':last_accepted,
                            'abandoned_request_tokens':abandoned,'gap_threshold_ns':self.gap_ns})
                    try: data,now,origin=self.queue.get(timeout=.05)
                    except queue.Empty: data=None
                    if data is not None:
                        if origin not in parsers and len(parsers)>=8:
                            self.issues.add('timesync_peer_limit');self.queue.task_done();continue
                        parser=parsers.setdefault(origin,mav.MAVLink(None));parser.robust_parsing=True
                        try: messages=parser.parse_buffer(data) or []
                        except Exception as error:
                            event({'event':'decode_error','error':str(error)});messages=[]
                        for msg in messages:
                            source=(msg.get_srcSystem(),msg.get_srcComponent())
                            if source == self.target: peer=origin
                            if msg.get_type()!='TIMESYNC':continue
                            addressed=message_target(msg)
                            if msg.tc1==0:
                                valid_target=addressed[0] in (0,self.identity[0]) and addressed[1] in (0,self.identity[1])
                                event({'event':'remote_request','host_monotonic_ns':now,'source':source,'ts1_ns':msg.ts1,
                                    'target_system':addressed[0],'target_component':addressed[1],'reply_allowed':valid_target and source==self.target})
                                if self.active and source==self.target and valid_target and not self.stop.is_set():
                                    send(time.monotonic_ns(),msg.ts1,source)
                            else:
                                row=tracker.response(source,msg.tc1,msg.ts1,now,addressed)
                                row['event']='reply';row['addressing']='addressed' if any(addressed) else 'legacy_broadcast'
                                row['observer_decode_monotonic_ns']=time.monotonic_ns()
                                event(row)
                                if row['accepted']:last_accepted=now;gap_active=False
                        self.queue.task_done()
                    now=time.monotonic_ns()
                    if self.active and peer and not self.stop.is_set() and now-last_request>=100_000_000:
                        token=tracker.request(now)
                        for old in tracker.expired:event({'event':'request_expired','ts1_ns':old,'host_monotonic_ns':now})
                        send(0,token,self.target)
                        event({'event':'request_sent','host_monotonic_ns':now,'ts1_ns':token,'segment_id':tracker.segment});last_request=now
                for token in tracker.pending:event({'event':'request_abandoned_at_stop','ts1_ns':token,'segment_id':tracker.segment})
                for stream in [log,tx,index]:stream.flush();os.fsync(stream.fileno())
        except Exception as error:
            self.issues.add('timesync_observer_error',error=str(error))


class CombinedObserver:
    """Fan out archived datagrams to health checks and the bounded clock worker."""
    def __init__(self,live,clock):
        self.live,self.clock=live,clock
        self.finished=clock.finished
    def start(self):self.clock.start()
    def observe(self,data,host_ns,peer):
        self.live.observe(data,host_ns,peer);self.clock.observe(data,host_ns,peer)
    def join(self,timeout):self.clock.join(timeout)
    def is_alive(self):return self.clock.is_alive()
