"""Small live continuity checks, always after original bytes are archived."""
import struct
import math
import threading
import time


class LiveHealth(threading.Thread):
    def __init__(self,stop,issues):
        super().__init__(name='live-stream-checks',daemon=True)
        from pymavlink.dialects.v20 import common
        self.parser=common.MAVLink(None);self.parser.robust_parsing=True
        self.stop,self.issues=stop,issues
        self.finished=threading.Event();self.lock=threading.Lock()
        self.buffer=bytearray();self.frames=0;self.epochs=0;self.last_epoch=None
        self.last={};self.counts={};self.source_times={}

    def run(self):self.finished.wait()

    def fail(self,kind,**details):
        if not self.stop.is_set():self.issues.add(kind,**details)
        self.stop.set()

    def receiver(self,data,host_ns):
        with self.lock:
            self.buffer.extend(data)
            while True:
                i=self.buffer.find(b'\xb5\x62')
                if i<0:self.buffer=self.buffer[-1:];return
                if i:del self.buffer[:i]
                if len(self.buffer)<6:return
                size=8+int.from_bytes(self.buffer[4:6],'little')
                if len(self.buffer)<size:return
                frame=bytes(self.buffer[:size]);a=b=0
                for value in frame[2:-2]:a=(a+value)&255;b=(b+a)&255
                if frame[-2:]!=bytes([a,b]):
                    del self.buffer[:1]
                    if self.epochs:self.fail('receiver_checksum_discontinuity')
                    continue
                del self.buffer[:size];self.frames+=1
                if frame[2:4]!=b'\x02\x15':continue
                payload=frame[6:-2]
                if len(payload)<16:self.fail('malformed_rawx');continue
                tow,week,leaps,count,status,version=struct.unpack_from('<dHbBBB',payload)
                if len(payload)!=16+32*count:self.fail('malformed_rawx');continue
                epoch=week*604800.+tow
                if not math.isfinite(epoch):self.fail('malformed_rawx_timestamp');continue
                if status&2:self.fail('receiver_clock_reset')
                if self.last_epoch is not None and abs(epoch-self.last_epoch-.05)>.002:
                    self.fail('rawx_interval_discontinuity',interval_s=epoch-self.last_epoch)
                self.last_epoch=epoch;self.epochs+=1;self.last['RAWX']=host_ns

    def observe(self,data,host_ns,peer):
        with self.lock:
            try:messages=self.parser.parse_buffer(data) or []
            except Exception as error:self.fail('mavlink_live_decode_error',error=str(error));return
            for msg in messages:
                if msg.get_type()=='BAD_DATA':self.fail('mavlink_frame_corruption');continue
                if (msg.get_srcSystem(),msg.get_srcComponent())!=(1,1):continue
                name=msg.get_type()
                if name not in ['ATTITUDE_QUATERNION','HIGHRES_IMU']:continue
                stamp=msg.time_boot_ms if name=='ATTITUDE_QUATERNION' else msg.time_usec
                previous=self.source_times.get(name)
                if previous is not None and stamp<previous:self.fail('px4_clock_reset',stream=name)
                self.source_times[name]=stamp;self.last[name]=host_ns
                self.counts[name]=self.counts.get(name,0)+1

    def check(self,started,now=None):
        now=time.monotonic_ns() if now is None else now
        with self.lock:
            ready=self.epochs>=2 and all(self.counts.get(k,0)>=2 for k in ['ATTITUDE_QUATERNION','HIGHRES_IMU'])
            for name in ['RAWX','ATTITUDE_QUATERNION','HIGHRES_IMU']:
                age=(now-self.last.get(name,started))/1e9
                if age>(2.5 if ready else 8):self.fail('required_stream_stale',stream=name,age_s=age)
            return {'ready':ready,'rawx_epochs':self.epochs,'mavlink_messages':dict(self.counts),
                    'last_receive_monotonic_ns':dict(self.last),'last_gps_epoch':self.last_epoch}
