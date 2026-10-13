"""Bounded native-frame timing telemetry. Receipt statistics are not calibration."""
from collections import deque
import math
from itertools import product
import threading
import uuid

import numpy as np

from .clock import monotonic_ns


def latest_ready_frame(frames,target_ns,pending=None):
    """Do not prematurely select an older frame while a causal one is encoding.

    Caller holds the camera lock. Pending tuple is (frame_id, alignment_time_ns).
    Real adapters use checked SDK host arrival or receipt; calibrated adapters supply the
    mapped measurement time here as well. Waiting budget remains in the sampler.
    """
    from .timing import frame_time
    if target_ns is None:return frames[-1] if frames else None
    ready=next((frame for frame in reversed(frames) if frame_time(frame)<=target_ns),None)
    if pending is not None and pending[1]<=target_ns and (ready is None or pending[0]>ready.frame_id):
        return None
    return ready


def ready_frame_candidates(frames, target_ns, pending=None):
    """Return ready causal frames even while a newer frame is being encoded.

    Joint selection enforces uniqueness, age, offset and pair skew. A pending
    newer frame must not hide a ready pair that already satisfies those limits.
    If no valid pair exists, the sampler still waits within its normal budget.
    """
    from .timing import frame_time
    return [frame for frame in reversed(frames) if frame_time(frame) <= target_ns]


def freshest_valid_pair(candidates, target_ns, observed_ns, previous, config):
    """Joint selection prevents rejecting a valid unused historical frame pair.

    All original causal, uniqueness, age, alignment and host-receipt-skew limits
    apply. The reserve policy consumes older eligible frames first, retaining
    newer frames for the next grid point when SDK arrival intervals jitter.
    This increases mean image age within the SAME 30 ms configured upper limit.
    """
    from .timing import frame_time
    keys = tuple(candidates)
    if not keys:
        return {}
    eligible = {key: [frame for frame in candidates[key]
                if frame.frame_id > previous[key]
                and 0 <= (observed_ns-frame.received_ns)/1e6 <= config['camera_max_age_ms']
                and 0 <= (target_ns-frame_time(frame))/1e6 <= config['camera_alignment_max_offset_ms']]
                for key in keys}
    best = None
    best_score = None
    for frames in product(*(eligible[key] for key in keys)):
        receipts = [frame.received_ns for frame in frames]
        if (max(receipts)-min(receipts))/1e6 > config.get('camera_pair_max_skew_ms',20):
            continue
        stamps = [frame_time(frame) for frame in frames]
        if config.get('camera_pairing_policy')=='oldest_valid_causal_pair_v2':
            score = (-max(stamps), -sum(stamps))
        else:
            score = (min(stamps), sum(stamps))
        if best_score is None or score > best_score:
            best_score = score
            best = dict(zip(keys, frames))
    return best


def percentiles(values):
    values=[float(v) for v in values if v is not None and math.isfinite(v)]
    if not values:return None
    return dict(zip(('p50','p95','p99','max'),
                    (round(float(v),4) for v in np.percentile(values,[50,95,99,100]))))


def optional_metadata(names, supports, read):
    """Missing optional UVC metadata must never terminate an otherwise valid stream."""
    values={};unavailable={}
    for name in names:
        try:
            if not supports(name):
                unavailable[name]='unsupported_by_current_stream';continue
            value=read(name)
            if value is None or not math.isfinite(value):
                unavailable[name]='invalid_value';continue
            values[name]=value
        except Exception as exc:
            unavailable[name]=type(exc).__name__+': '+str(exc)[:160]
    return {'values':values,'unavailable':unavailable}


class CameraDiagnostics:
    def __init__(self,capacity=2400,window_seconds=30):
        self.rows=deque(maxlen=capacity)
        self.window_ns=round(window_seconds*1e9)
        self.lock=threading.Lock()
        self.session_id=uuid.uuid4().hex
        self.total_frames=0

    def record(self,frame,stages=None):
        row={'frame_id':frame.frame_id,'received_ns':frame.received_ns,
             'ready_ns':frame.ready_ns,'device_timestamp_ms':frame.device_timestamp_ms,
             'sdk_arrival_host_ns':frame.sdk_arrival_host_ns,'timestamp_basis':frame.timestamp_basis,
             'device_clock_domain':frame.device_clock_domain,
             'processing_ms':(frame.ready_ns-frame.received_ns)/1e6,
             'stages_ms':stages or {},'metadata':frame.timing_metadata or {}}
        with self.lock:
            self.rows.append(row);self.total_frames+=1

    def snapshot(self,now_ns=None):
        now_ns=monotonic_ns() if now_ns is None else now_ns
        with self.lock:
            rows=[r for r in self.rows if now_ns-self.window_ns<=r['received_ns']<=now_ns]
            total=self.total_frames
        report={'camera_session':self.session_id,'window_seconds':self.window_ns/1e9,
                'sample_count':len(rows),'total_frames_observed':total,
                'physical_sync_verified':False,'timestamp_basis':'host_receive_unmapped_device_clock'}
        if not rows:return report
        dt=[(b['received_ns']-a['received_ns'])/1e6 for a,b in zip(rows,rows[1:])]
        device_dt=[b['device_timestamp_ms']-a['device_timestamp_ms'] for a,b in zip(rows,rows[1:])
                   if a['device_clock_domain']==b['device_clock_domain']]
        stages=set().union(*(r['stages_ms'] for r in rows))
        observed=(rows[-1]['received_ns']-rows[0]['received_ns'])/1e9
        fields=set().union(*(r['metadata'].get('values',{}) for r in rows))
        report.update(observed_seconds=round(observed,3),
            timestamp_basis=rows[-1]['timestamp_basis'],
            received_fps=round((len(rows)-1)/observed,3) if observed>0 else None,
            latest_age_ms=round((now_ns-rows[-1]['received_ns'])/1e6,3),
            frame_id_gaps=sum(max(0,b['frame_id']-a['frame_id']-1) for a,b in zip(rows,rows[1:])),
            receive_interval_ms=percentiles(dt),device_interval_ms=percentiles(device_dt),
            processing_ms=percentiles(r['processing_ms'] for r in rows),
            stages_ms={s:percentiles(r['stages_ms'].get(s) for r in rows) for s in sorted(stages)},
            metadata_present_counts={f:sum(f in r['metadata'].get('values',{}) for r in rows) for f in sorted(fields)},
            latest=rows[-1])
        return report

    def recent_frames(self,count=12):
        with self.lock:rows=list(self.rows)[-count:]
        return [{k:r[k] for k in ('frame_id','received_ns','ready_ns','sdk_arrival_host_ns','device_timestamp_ms','processing_ms')}
                for r in rows]


class AlignmentDiagnostics:
    """Recent saved rows only, not all native camera frames; reset per episode."""
    def __init__(self,capacity=1500):
        self.rows=deque(maxlen=capacity);self.lock=threading.Lock()

    def add(self,row):
        values={key:abs(im['alignment_offset_ms']) for key,im in row['images'].items()}
        values.update(pair_skew=row['quality']['camera_pair_skew_ms'],
                      interpolation_gap=row['alignment']['joints']['gap_ms'] if row['alignment']['joints'] else None,
                      assembly_lateness=row['quality']['assembly_lateness_ms'],
                      command_send_delay=row['quality']['command_send_delay_ms'])
        with self.lock:self.rows.append(values)

    def snapshot(self):
        with self.lock:rows=list(self.rows)
        return {'sample_count':len(rows),'capacity':self.rows.maxlen,'physical_sync_verified':False,
                'metrics_ms':{k:percentiles(row[k] for row in rows) for k in rows[0]} if rows else {}}
