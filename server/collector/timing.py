"""Explicit clock provenance and bounded interpolation; never invent exposure time."""
from collections import deque
from dataclasses import asdict, dataclass
import math
import threading
import time
from .clock import monotonic, monotonic_ns

import numpy as np


@dataclass(frozen=True)
class ClockMapping:
    """Externally calibrated affine mapping, scoped to a device/host clock session.

    Do not estimate this from arrival times alone: transport offset is unobservable.
    Integer anchors preserve precision; uncertainty and validity are mandatory.
    """
    device_anchor_ns: int
    host_anchor_ns: int
    scale: float
    uncertainty_ns: int
    valid_device_min_ns: int
    valid_device_max_ns: int
    calibration_id: str
    clock_session: str

    def map(self, device_ns, clock_session):
        if (not self.calibration_id or not self.clock_session or
                clock_session != self.clock_session or not math.isfinite(self.scale) or
                self.scale <= 0 or self.uncertainty_ns < 0):
            raise ValueError('Invalid or mismatched clock calibration.')
        if not self.valid_device_min_ns <= device_ns <= self.valid_device_max_ns:
            raise ValueError('Clock calibration expired or device clock reset.')
        return self.host_anchor_ns + round((device_ns-self.device_anchor_ns)*self.scale)


def frame_time(frame):
    if frame.measurement_host_ns is not None:return frame.measurement_host_ns
    return frame.sdk_arrival_host_ns if frame.sdk_arrival_host_ns is not None else frame.received_ns


def slerp_wxyz(a, b, weight):
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    for q in (a, b):
        if not np.isfinite(q).all() or abs(np.linalg.norm(q)-1) > 1e-4:
            raise ValueError('Interpolation requires unit quaternions.')
    dot = float(np.dot(a, b))
    if dot < 0:
        b, dot = -b, -dot
    dot = float(np.clip(dot, -1, 1))
    if dot > .9995:
        q = a + weight*(b-a)
    else:
        theta = math.acos(dot)
        q = (math.sin((1-weight)*theta)*a + math.sin(weight*theta)*b)/math.sin(theta)
    return (q/np.linalg.norm(q)).tolist()


class StateHistory:
    """Separate joint/TCP timelines. No extrapolation, clamping or reset bridging."""
    def __init__(self, max_gap_ms=40, maxlen=2000, continuous_joint_indices=()):
        if max_gap_ms <= 0 or maxlen < 2:
            raise ValueError('Invalid history limits.')
        self.max_gap_ns = round(max_gap_ms*1e6)
        self.continuous = tuple(continuous_joint_indices)
        if any(i not in (0,1,2,3,4,6,7,8,9,10) for i in self.continuous):
            raise ValueError('Only explicitly continuous robot joints may unwrap.')
        self.joints, self.tcp = deque(maxlen=maxlen), deque(maxlen=maxlen)
        self.condition = threading.Condition()
        self.error = None
        self.last_device_ns = None
        self.packet_id = 0

    def fail(self, error):
        with self.condition:
            self.error = str(error)
            self.condition.notify_all()

    def add(self, state):
        # Validate/copy outside the history mutex. Numerical work must not block
        # the consumer waiting for a newer packet (or the next feedback read).
        if not state.valid:
            raise ValueError('Invalid feedback; interpolation across a fault is forbidden.')
        now = monotonic_ns()
        if state.received_ns > now or (state.ee_received_ns or state.received_ns) > now:
            raise ValueError('Feedback receipt is in the future.')
        prepared = []
        for queue, values, measured, received, basis in (
            (self.joints, state.state, state.measurement_host_ns, state.received_ns, state.timestamp_basis),
            (self.tcp, state.actual_ee, state.ee_measurement_host_ns,
             state.ee_received_ns or state.received_ns, state.ee_timestamp_basis)):
            if values is None:
                continue
            expected = 12 if queue is self.joints else 16
            if len(values) != expected or not all(math.isfinite(v) for v in values):
                raise ValueError('Invalid feedback vector.')
            for index in ((5,11) if queue is self.joints else (7,15)):
                if not 0 <= values[index] <= 1:
                    raise ValueError('Invalid gripper feedback.')
            if queue is self.tcp:
                for index in (3,11):
                    if abs(math.sqrt(sum(v*v for v in values[index:index+4]))-1)>1e-4:
                        raise ValueError('Interpolation requires unit quaternions.')
            stamp = received if measured is None else measured
            if measured is not None and (not state.clock_mapping_id or
                    state.clock_uncertainty_ns is None or state.clock_uncertainty_ns < 0 or
                    measured > received + state.clock_uncertainty_ns):
                raise ValueError('Mapped feedback lacks valid clock provenance.')
            signature = (basis, state.clock_domain, state.clock_mapping_id, measured is not None)
            prepared.append((queue, dict(t=stamp, values=list(values), received_ns=received,
                                        signature=signature)))
        with self.condition:
            if state.device_ns is not None:
                if self.last_device_ns is not None and state.device_ns < self.last_device_ns:
                    raise ValueError('Robot device clock reset/out of order.')
            updates = []
            for queue, item in prepared:
                if queue:
                    last = queue[-1]
                    if item['t'] < last['t'] or item['signature'] != last['signature']:
                        raise ValueError('Feedback clock changed/out of order; start a new episode.')
                    if item['t'] == last['t']:
                        if item['values'] != last['values']:
                            raise ValueError('Feedback changed without a new measurement timestamp.')
                        continue
                updates.append((queue,item))
            # Commit both timelines atomically only after both have passed.
            if state.device_ns is not None:
                self.last_device_ns = state.device_ns
            self.packet_id += 1
            for queue,item in updates:
                item['packet_id'] = self.packet_id
                queue.append(item)
            self.condition.notify_all()
            return self.packet_id if updates else None

    def _bracket(self, queue, target):
        if not queue or target < queue[0]['t'] or target > queue[-1]['t']:
            raise LookupError('No feedback bracketing the alignment time.')
        # Online targets are near the newest packet: do not scan 2,000 old
        # packets twice per sample while holding the producer's mutex.
        b = queue[-1]
        for a in reversed(queue):
            if a['t'] == target:
                return a,a
            if a['t'] < target:
                return a,b
            b = a
        raise LookupError('No feedback bracketing the alignment time.')

    def _between(self, a, b, target, pose=False):
        gap = b['t']-a['t']
        if gap > self.max_gap_ns:
            raise ValueError('Feedback gap exceeds interpolation budget.')
        weight = (target-a['t'])/gap if gap else 0.
        av, bv = np.array(a['values']), np.array(b['values'])
        delta = bv-av
        if not pose:
            for index in self.continuous:
                delta[index] = (delta[index]+np.pi) % (2*np.pi)-np.pi
        out = (av+weight*delta).tolist()
        if pose:
            for index in (3,11):
                out[index:index+4] = slerp_wxyz(av[index:index+4], bv[index:index+4], weight)
        return out, dict(before_ns=a['t'], after_ns=b['t'], weight=weight,
                         gap_ms=gap/1e6, before_packet_id=a['packet_id'],
                         after_packet_id=b['packet_id'],
                         before_received_ns=a['received_ns'], after_received_ns=b['received_ns'],
                         timestamp_basis=a['signature'][0], calibrated=a['signature'][3])

    def interpolate(self, target_ns, require_ee=True, wait_until=None):
        with self.condition:
            while True:
                if self.error:
                    raise RuntimeError(self.error)
                try:
                    joint_bracket = self._bracket(self.joints,target_ns)
                    tcp_bracket = self._bracket(self.tcp,target_ns) if require_ee else None
                    break
                except LookupError:
                    remaining = (wait_until-monotonic()) if wait_until is not None else 0
                    if remaining <= 0:
                        raise RuntimeError('Feedback does not bracket alignment time; no extrapolation allowed.')
                    self.condition.wait(min(remaining, .01))
        # Bracket entries are immutable snapshots; numerical work can proceed
        # after releasing the lock, even if the bounded deques evict entries.
        state,joints = self._between(*joint_bracket,target_ns)
        ee,tcp = self._between(*tcp_bracket,target_ns,pose=True) if tcp_bracket else (None,None)
        with self.condition:
            if self.error:
                raise RuntimeError(self.error)
        return state,ee,dict(joints=joints,tcp=tcp)


class FeedbackRecorder:
    """Record every observed SDK poll independently of 50 Hz rows.

    A future callback SDK must feed every packet instead of relying on polling.
    No driver thread survives episode finalization.
    """
    def __init__(self, robot, writer, config, trace=None):
        self.robot, self.writer = robot, writer
        self.trace = trace
        self.hz = config['feedback_poll_hz']
        self.history = StateHistory(config['state_interpolation_max_gap_ms'],
                                    continuous_joint_indices=config['continuous_joint_indices'])
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, name='feedback-history', daemon=True)

    def start(self):
        self.thread.start()

    def _run(self):
        deadline = monotonic()
        try:
            while not self.stop_event.is_set():
                if self.trace: self.trace.record('feedback_read_start')
                state = self.robot.read_state()
                if self.trace: self.trace.record('feedback_read_end',received_ns=state.received_ns)
                packet_id = self.history.add(state)
                if self.trace: self.trace.record('feedback_history_added',received_ns=state.received_ns,packet_id=packet_id)
                if packet_id is not None:
                    self.writer.submit_feedback(dict(packet_id=packet_id, **asdict(state)))
                    if self.trace: self.trace.record('feedback_queued',packet_id=packet_id)
                deadline = max(deadline+1/self.hz, monotonic())
                self.stop_event.wait(max(0, deadline-monotonic()))
        except Exception as exc:
            self.history.fail(exc)

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=2)
        if self.thread.is_alive():
            raise RuntimeError('SDK feedback read did not terminate; adapter must use bounded reads.')


TIMING_DEFAULTS = dict(feedback_poll_hz=100, feedback_warmup_ms=60,
    state_interpolation_max_gap_ms=40, alignment_wait_budget_ms=18,
    camera_alignment_max_offset_ms=30, command_send_max_delay_ms=40,
    continuous_joint_indices=[])


def configure_timing(config):
    for key, value in TIMING_DEFAULTS.items():
        config.setdefault(key, value)
    for key in TIMING_DEFAULTS:
        if key != 'continuous_joint_indices':
            value = config[key]
            if isinstance(value, bool) or not isinstance(value, (int,float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'Invalid timing option: {key}')
    if config['feedback_poll_hz'] > 1000:
        raise ValueError('Feedback poll rate exceeds the supported budget.')
    StateHistory(config['state_interpolation_max_gap_ms'],
                 continuous_joint_indices=config['continuous_joint_indices'])
    config['sampling_mode'] = 'fixed_grid_causal_images_interpolated_state'
    config['timing_version'] = 1


def timing_summary(rows):
    def stats(values):
        values=[v for v in values if v is not None]
        if not values:return None
        return {k:round(float(v), 6) for k,v in zip(
            ('p50','p95','p99','max'), np.percentile(values, [50,95,99,100]))}
    aligned = [r for r in rows if 'alignment' in r]
    if not aligned:
        return {'verified':False, 'reason':'Legacy capture without alignment provenance.'}
    return dict(verified=False, alignment_metadata_present=True, physical_sync_verified=False,
        note='Clock calibration and hardware validation are still required.',
        camera_absolute_offset_ms={key:stats([abs(r['images'][key]['alignment_offset_ms']) for r in aligned])
                                   for key in aligned[0]['images']},
        interpolation_gap_ms=stats([r['alignment']['joints']['gap_ms'] for r in aligned if r['alignment']['joints']]),
        tcp_interpolation_gap_ms=stats([r['alignment']['tcp']['gap_ms'] for r in aligned if r['alignment']['tcp']])
            if any(r['alignment']['tcp'] for r in aligned) else None,
        command_send_delay_ms=stats([r['quality']['command_send_delay_ms'] for r in aligned]),
        assembly_lateness_ms=stats([r['quality']['assembly_lateness_ms'] for r in aligned]),
        sample_lateness_ms=stats([r['quality']['sample_lateness_ms'] for r in aligned]))


def validate_alignment(manifest, rows, packets):
    """Reconstruct each aligned state from persisted source packets on export."""
    config = manifest['config']
    by_id = {}
    previous_id = -1
    previous_device = None
    for packet in packets:
        ident = packet['packet_id']
        if ident <= previous_id or not packet['valid']:
            raise ValueError('Invalid feedback archive ordering/validity.')
        previous_id = ident
        device = packet.get('device_ns')
        if device is not None:
            if previous_device is not None and device < previous_device:
                raise ValueError('Device clock reset in feedback archive.')
            previous_device = device
        by_id[ident] = packet
    if len(packets) != manifest.get('feedback_count'):
        raise ValueError('Feedback archive count differs from manifest.')
    origin = rows[0]['target_ns']
    for index, row in enumerate(rows):
        target = origin+round(index*1e9/manifest['fps'])
        if row.get('timing_version') != 1 or row['target_ns'] != target or row['alignment']['target_ns'] != target:
            raise ValueError('Alignment target is not on the fixed dataset grid.')
        if not target <= row['sample_ns'] <= row['assembled_ns'] <= row['command']['requested_ns'] <= row['command']['sent_ns']:
            raise ValueError('Observation/command event ordering is invalid.')
        send_delay = (row['command']['sent_ns']-target)/1e6
        command=row['command']
        accepted,effective=command.get('accepted_ns'),command.get('effective_ns')
        if not command.get('accepted_ok') or (accepted is not None and accepted<command['sent_ns']) or (effective is not None and (accepted is None or effective<accepted)):
            raise ValueError('Invalid command acceptance/effect ordering.')
        if not 0 <= send_delay <= config['command_send_max_delay_ms'] or abs(send_delay-row['quality']['command_send_delay_ms'])>1e-6:
            raise ValueError('Command send delay/provenance invalid.')
        for key, im in row['images'].items():
            stamp = im['measurement_host_ns'] if im['measurement_host_ns'] is not None else im['received_ns']
            if im.get('sdk_arrival_host_ns') is not None:
                from .host_arrival import validate_image_arrival
                arrival=validate_image_arrival(im)
                if im['measurement_host_ns'] is None:stamp=arrival
            elif im['timestamp_basis']=='sdk_host_arrival_unmapped_exposure':
                raise ValueError('SDK host arrival proof missing.')
            if im['measurement_host_ns'] is not None and (not im['clock_mapping_id'] or
                    im['clock_uncertainty_ns'] is None or im['clock_uncertainty_ns'] < 0 or
                    stamp > im['received_ns']+im['clock_uncertainty_ns']):
                raise ValueError('Image measurement time lacks clock provenance.')
            offset = (stamp-target)/1e6
            if not -config['camera_alignment_max_offset_ms']<=offset<=0:
                raise ValueError(f'Image {key} outside causal alignment window.')
            if im['alignment_time_ns']!=stamp or abs(im['alignment_offset_ms']-offset)>1e-6:
                raise ValueError('Image alignment metadata is inconsistent.')
        for name, value_key, mapped_key, receipt_key, basis_key in (
            ('joints','state','measurement_host_ns','received_ns','timestamp_basis'),
            ('tcp','actual_ee','ee_measurement_host_ns','ee_received_ns','ee_timestamp_basis')):
            proof = row['alignment'][name]
            if proof is None:
                if name=='joints' or config.get('require_ee_feedback'):
                    raise ValueError('Missing interpolation provenance.')
                continue
            try:
                a,b = [by_id[proof[k]] for k in ('before_packet_id','after_packet_id')]
            except KeyError as exc:
                raise ValueError('Interpolation source packet missing.') from exc
            stamps=[]
            for p, side in ((a,'before'),(b,'after')):
                received=p.get(receipt_key) or p['received_ns']
                stamp=p.get(mapped_key)
                mapped=stamp is not None
                stamp=stamp if mapped else received
                if mapped and (not p.get('clock_mapping_id') or p.get('clock_uncertainty_ns') is None or
                               p['clock_uncertainty_ns']<0 or stamp>received+p['clock_uncertainty_ns']):
                    raise ValueError('Mapped feedback provenance invalid.')
                if proof[side+'_ns']!=stamp or proof[side+'_received_ns']!=received or received>row['assembled_ns']:
                    raise ValueError('Feedback timestamp provenance mismatch.')
                if proof['timestamp_basis']!=p[basis_key] or proof['calibrated']!=mapped:
                    raise ValueError('Feedback time basis mismatch.')
                stamps.append(stamp)
            if any(a.get(k)!=b.get(k) for k in ('clock_domain','clock_mapping_id')):
                raise ValueError('Interpolation crosses a clock calibration boundary.')
            t0,t1=stamps
            gap=t1-t0
            if not t0<=target<=t1 or gap>config['state_interpolation_max_gap_ms']*1e6:
                raise ValueError('Interpolation bracket/gap invalid.')
            weight=(target-t0)/gap if gap else 0.
            if abs(weight-proof['weight'])>1e-9 or abs(gap/1e6-proof['gap_ms'])>1e-6:
                raise ValueError('Interpolation weight/gap mismatch.')
            av,bv=np.asarray(a[value_key]),np.asarray(b[value_key])
            delta=bv-av
            if name=='joints':
                for j in config['continuous_joint_indices']:
                    delta[j]=(delta[j]+np.pi)%(2*np.pi)-np.pi
            expected=(av+weight*delta).tolist()
            if name=='tcp':
                for j in (3,11):
                    expected[j:j+4]=slerp_wxyz(av[j:j+4],bv[j:j+4],weight)
                actual=row['actual_ee']
                pose=expected[:7]+expected[8:15]
                if not np.allclose(pose,row['observation.ee_pose'],atol=1e-7,rtol=0):
                    raise ValueError('TCP pose differs from archived interpolation.')
            else:
                actual=row['observation.state']
            if not np.allclose(expected,actual,atol=1e-7,rtol=0):
                raise ValueError('State differs from archived interpolation.')
