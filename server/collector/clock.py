"""One process-wide clock; retain Linux CLOCK_MONOTONIC used by SDK contracts.

Python <=3.12 on Windows can implement monotonic with a coarse tick counter.
Use QPC there for local validation. Never compare timestamps across boots/hosts.
"""
import os
import time
from pathlib import Path
import uuid

DOMAIN = 'windows_qpc' if os.name == 'nt' else 'host_monotonic'
monotonic_ns = time.perf_counter_ns if os.name == 'nt' else time.monotonic_ns
try:
    SESSION = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
except OSError:
    SESSION = 'process-'+uuid.uuid4().hex


def monotonic():
    return monotonic_ns()/1e9
