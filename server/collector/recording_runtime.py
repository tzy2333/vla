"""Bounded recording telemetry and scoped deferral of cyclic garbage collection.

CPython reference counting remains active. Only the cyclic collector is deferred,
for the already time-limited episode, then its original enabled state is restored.
This is not a real-time guarantee and does not change any data-quality threshold.
"""
from collections import deque
import gc
from pathlib import Path
import threading
import time

from .clock import monotonic_ns


def memory_kib():
    try:
        return {line.split(':')[0]: int(line.split()[1])
                for line in Path('/proc/self/status').read_text().splitlines()
                if line.startswith(('VmRSS:', 'VmHWM:'))}
    except OSError:
        return {}


class CaptureTrace:
    def __init__(self, capacity=1600):
        self.events = deque(maxlen=capacity)
        self.lock = threading.Lock()

    def record(self, stage, **fields):
        row = dict(stage=stage, ns=monotonic_ns(), **fields)
        with self.lock:
            self.events.append(row)

    def snapshot(self):
        with self.lock:
            return list(self.events)


class RecordingRuntime:
    _lock = threading.Lock()
    _owner = None

    def __init__(self):
        self.active = False
        self.was_enabled = None
        self.summary = {'version': 1, 'cyclic_gc_policy': 'defer_during_bounded_recording',
                        'reference_counting_active': True}

    def begin(self):
        with self._lock:
            if type(self)._owner is not None:
                raise RuntimeError('Another recording owns the cyclic-GC guard.')
            type(self)._owner = self
            self.was_enabled = gc.isenabled()
            self.active = True
        try:
            before = monotonic_ns()
            if self.was_enabled:
                gc.collect()
                gc.disable()
            self.summary.update(pre_record_gc_ms=(monotonic_ns()-before)/1e6,
                                gc_previously_enabled=self.was_enabled,
                                memory_start_kib=memory_kib())
            self.started_ns = monotonic_ns()
            self.cpu_started = time.process_time()
        except BaseException:
            self.close()
            raise

    def close(self):
        with self._lock:
            if not self.active:
                return
            self.active = False
            type(self)._owner = None
            if self.was_enabled:
                gc.enable()
        if hasattr(self, 'started_ns'):
            self.summary.update(elapsed_seconds=(monotonic_ns()-self.started_ns)/1e9,
                                process_cpu_seconds=time.process_time()-self.cpu_started,
                                memory_end_kib=memory_kib())
        if self.was_enabled:
            before = monotonic_ns()
            gc.collect()
            self.summary['post_record_gc_ms'] = (monotonic_ns()-before)/1e6
