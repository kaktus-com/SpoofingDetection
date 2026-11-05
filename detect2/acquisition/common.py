"""Bounded raw archives and explicit lifecycle/integrity metadata."""
import csv
import hashlib
import json
import os
from pathlib import Path
import queue
import threading
import time


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix+'.tmp')
    with temporary.open('w') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
    os.replace(temporary, path)


def fingerprint(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''): h.update(block)
    return {'bytes': Path(path).stat().st_size, 'sha256': h.hexdigest()}


class Issues:
    def __init__(self):
        self.lock = threading.Lock(); self.counts = {}; self.events = []

    def add(self, kind, **details):
        with self.lock:
            self.counts[kind] = self.counts.get(kind, 0)+1
            if len(self.events) < 200:
                self.events.append({'kind': kind, 'host_monotonic_ns': time.monotonic_ns(), **details})

    def snapshot(self):
        with self.lock: return {'counts': dict(self.counts), 'events': list(self.events)}


class ArchiveWriter:
    """One bounded queue/worker per stream. Overflow never appears as success."""
    FIELDS = ['sequence','byte_offset','length','host_monotonic_ns','segment_id','peer_host','peer_port']

    def __init__(self, folder, name, issues, max_chunks=4096, max_bytes=8*1024*1024,
                 max_chunk_bytes=65535, write_delay_s=0., opener=None):
        if min(max_chunks,max_bytes,max_chunk_bytes) <= 0 or write_delay_s < 0:
            raise ValueError('Archive limits must be positive and delay nonnegative')
        self.folder, self.name, self.issues = Path(folder), name, issues
        self.queue = queue.Queue(max_chunks); self.max_bytes = max_bytes
        self.max_chunk_bytes = max_chunk_bytes; self.write_delay_s = write_delay_s
        self.lock = threading.Lock(); self.closing = threading.Event()
        self.received = self.written = self.sequence = self.dropped_bytes = self.dropped_chunks = 0
        self.buffered_bytes = self.peak_buffered_bytes = self.peak_queue_chunks = 0
        self.oldest_queued_ns = None; self.error = None; self.closed = False
        self.binary = (opener or open)(self.folder/(name+'.bin'), 'xb')
        try: self.index_file = open(self.folder/(name+'_receive.csv'), 'x', newline='')
        except BaseException:
            self.binary.close(); raise
        self.index = csv.DictWriter(self.index_file, fieldnames=self.FIELDS); self.index.writeheader()
        self.worker = threading.Thread(target=self._write, name='archive-'+name, daemon=True)
        self.worker.start()

    def submit(self, data, host_ns, segment_id=0, peer=None):
        data = bytes(data)
        with self.lock:
            sequence = self.sequence; self.sequence += 1; self.received += len(data)
            rejected = (self.closing.is_set() or self.error is not None or
                        len(data) > self.max_chunk_bytes or
                        self.buffered_bytes+len(data) > self.max_bytes)
            if not rejected:
                try:
                    self.queue.put_nowait((sequence,data,int(host_ns),int(segment_id),peer,time.monotonic_ns()))
                except queue.Full: rejected = True
            if rejected:
                self.dropped_chunks += 1; self.dropped_bytes += len(data)
                self.issues.add('archive_rejected_input', stream=self.name, sequence=sequence, bytes=len(data))
                return False
            self.buffered_bytes += len(data)
            self.peak_buffered_bytes = max(self.peak_buffered_bytes,self.buffered_bytes)
            self.peak_queue_chunks = max(self.peak_queue_chunks,self.queue.qsize())
            return True

    def _write(self):
        offset = 0
        try:
            while not self.closing.is_set() or not self.queue.empty():
                try: item = self.queue.get(timeout=.05)
                except queue.Empty: continue
                seq,data,host_ns,segment,peer,enqueued = item
                with self.lock: self.oldest_queued_ns = enqueued
                try:
                    if self.write_delay_s: time.sleep(self.write_delay_s)
                    if self.binary.write(data) != len(data): raise OSError('short archive write')
                    self.index.writerow({'sequence':seq,'byte_offset':offset,'length':len(data),
                        'host_monotonic_ns':host_ns,'segment_id':segment,
                        'peer_host':peer[0] if peer else '', 'peer_port':peer[1] if peer else ''})
                    offset += len(data)
                    with self.lock: self.written += len(data)
                finally:
                    with self.lock:
                        self.buffered_bytes -= len(data); self.oldest_queued_ns = None
                    self.queue.task_done()
            for stream in [self.binary,self.index_file]:
                stream.flush(); os.fsync(stream.fileno())
        except BaseException as error:
            with self.lock: self.error = f'{type(error).__name__}: {error}'
            self.issues.add('archive_write_error',stream=self.name,error=self.error)
        finally:
            for stream in [self.binary,self.index_file]:
                try: stream.close()
                except OSError: pass
            self.closed = True

    def stats(self):
        with self.lock:
            return {'received_bytes':self.received,'written_bytes':self.written,
                'input_chunks':self.sequence,'dropped_bytes':self.dropped_bytes,'dropped_chunks':self.dropped_chunks,
                'buffered_bytes':self.buffered_bytes,'queue_chunks':self.queue.qsize(),
                'peak_buffered_bytes':self.peak_buffered_bytes,'peak_queue_chunks':self.peak_queue_chunks,
                'write_in_progress_age_s':((time.monotonic_ns()-self.oldest_queued_ns)/1e9 if self.oldest_queued_ns else 0.),
                'error':self.error,'writer_closed':self.closed}

    def close(self, timeout=10):
        self.closing.set(); self.worker.join(timeout)
        if self.worker.is_alive(): self.issues.add('archive_shutdown_timeout',stream=self.name)
        return self.stats()
