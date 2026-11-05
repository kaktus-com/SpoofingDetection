"""Original serial bytes, independent of decoding and attitude availability."""
import threading
import time


class SerialCapture(threading.Thread):
    def __init__(self, device, baud, archive, stop, issues, factory=None, reconnect=True, observer=None):
        super().__init__(name='serial-acquisition',daemon=True)
        self.device,self.baud,self.archive,self.stop,self.issues = device,baud,archive,stop,issues
        self.factory = factory; self.connections = 0
        self.reconnect,self.observer=reconnect,observer

    def run(self):
        if self.factory is None:
            try:
                from serial import Serial
                self.factory = Serial
            except ImportError as error:
                self.issues.add('serial_dependency_error',error=str(error));return
        while not self.stop.is_set():
            serial = None
            try:
                serial = self.factory(self.device,self.baud,timeout=.05,exclusive=True)
                segment = self.connections; self.connections += 1
                while not self.stop.is_set():
                    count = min(max(serial.in_waiting,1),self.archive.max_chunk_bytes)
                    raw = serial.read(count)
                    if raw:
                        now=time.monotonic_ns()
                        self.archive.submit(raw,now,segment_id=segment)
                        if self.observer:self.observer(raw,now)
            except Exception as error:
                self.issues.add('serial_source_error',device=self.device,error=str(error),segment=self.connections)
                if not self.reconnect:self.stop.set()
            finally:
                if serial is not None:
                    try: serial.close()
                    except Exception: pass
            if not self.stop.is_set(): self.stop.wait(1.)
