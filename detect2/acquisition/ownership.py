"""Process locks shared by the raw recorder and runtime launcher (Linux/Pi)."""
from contextlib import ExitStack
import fcntl
import hashlib
import os
from pathlib import Path
import tempfile


class CaptureLocks:
    def __init__(self, device, bind, directory=None):
        self.directory=Path(directory or tempfile.gettempdir())
        self.resources=['udp:'+str(bind[0])+':'+str(bind[1])]
        if device:self.resources.append('serial:'+os.path.realpath(device))
        self.stack=ExitStack()

    def __enter__(self):
        try:
            for resource in sorted(self.resources):
                key=hashlib.sha256(resource.encode()).hexdigest()[:24]
                path=self.directory/f'detect2-{os.getuid()}-{key}.lock'
                fd=os.open(path,os.O_CREAT|os.O_RDWR|os.O_NOFOLLOW,0o600)
                stream=self.stack.enter_context(os.fdopen(fd,'r+'))
                try:fcntl.flock(stream,fcntl.LOCK_EX|fcntl.LOCK_NB)
                except BlockingIOError:raise RuntimeError(f'Recording resource already owned: {resource}') from None
                stream.seek(0);stream.truncate();stream.write(str(os.getpid())+'\n');stream.flush()
            return self
        except BaseException:
            self.stack.close();raise

    def __exit__(self,*args):self.stack.close()
