"""UDP datagrams archived unchanged, with explicit boundaries and peers."""
import socket
import threading
import time


class MavlinkCapture(threading.Thread):
    def __init__(self, bind, archive, stop, issues, observer_factory=None):
        super().__init__(name='mavlink-acquisition',daemon=True)
        self.bind,self.archive,self.stop,self.issues=bind,archive,stop,issues
        self.observer_factory=observer_factory;self.observer=None;self.sock=None

    def run(self):
        try:
            self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
            # No SO_REUSEADDR: two local recorders must not silently share data.
            self.sock.bind(self.bind);self.sock.settimeout(.1)
            if self.observer_factory:
                self.observer=self.observer_factory(self.sock);self.observer.start()
            while not self.stop.is_set():
                try: raw,ancillary,flags,peer=self.sock.recvmsg(65535)
                except socket.timeout:continue
                now=time.monotonic_ns()
                if flags & socket.MSG_TRUNC:self.issues.add('udp_datagram_truncated')
                self.archive.submit(raw,now,peer=peer)
                if self.observer:self.observer.observe(raw,now,peer)
        except Exception as error:
            self.issues.add('mavlink_source_error',error=str(error))
        finally:
            if self.observer:
                self.observer.finished.set();self.observer.join(3)
                if self.observer.is_alive():self.issues.add('timesync_shutdown_timeout')
            if self.sock:self.sock.close()
