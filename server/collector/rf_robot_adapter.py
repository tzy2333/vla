"""Real RF observation transport. Only UDP command 20010 exists in this adapter."""
from collections import deque
import copy
import errno
import math
import select
import socket
import struct
import threading
import uuid

import numpy as np

from .adapters import RobotState
from .clock import monotonic, monotonic_ns, DOMAIN
from .rf_kinematics import CandidateKinematics
from .pull_statistics import PullStatistics

OBSERVATION_SCHEMA = 'tabletop_bimanual_raw_joint_observation_2cam_v2'


def decode_observation(data):
    if len(data) != 392 or struct.unpack_from('>I', data)[0] != 20010 or struct.unpack_from('>I', data, 388)[0] != 20010:
        raise ValueError('RF observation packet length/header/trailer is invalid.')
    values = list(struct.unpack_from('>96f', data, 4))
    if not all(math.isfinite(v) for v in values):
        raise ValueError('RF observation contains non-finite values.')
    return values


class RFObservationAdapter:
    is_mock = False
    supports_commands = False

    def __init__(self, config, *, solver=None):
        self.fk = solver or CandidateKinematics()
        if solver is None and config.get('robot_model_override'):
            self.fk.model=copy.deepcopy(config['robot_model_override']['model'])
            self.fk.mapping=copy.deepcopy(config['robot_model_override']['mapping'])
            # Validate the feedback conversion only; never evaluate online FK.
            self.fk.angles_from_observation([0.0]*96)
        self.settings = config.get('rf_robot', {})
        self.end_effector_config=copy.deepcopy(config.get('robot_end_effectors',{}).get('rf_udp_20010',{}))
        self.poll_hz = float(self.settings.get('poll_hz', 50))
        self.timeout_s = float(self.settings.get('timeout_ms', 100))/1000
        self.max_age_ms = config.get('feedback_max_age_ms', 100)
        self.max_gap_ns = round(config.get('state_interpolation_max_gap_ms', 40)*1e6)
        if not 1 <= self.poll_hz <= 100 or not .01 <= self.timeout_s <= .3:
            raise ValueError('Invalid RF read rate or timeout.')
        self.session = uuid.uuid4().hex
        self.condition = threading.Condition()
        self.history = deque(maxlen=600)
        self.stop_event = threading.Event()
        self.error = None
        self.request_count = 0
        self.pull_statistics = PullStatistics()
        self.max_rtt_ms = 0.
        self.max_interval_ms = 0.
        self.recovery_count = 0
        self.last_transport_error = None
        self.thread = threading.Thread(target=self._run, name='rf-real-feedback', daemon=True)
        self.thread.start()

    def calibration_snapshot(self):
        return {'model': copy.deepcopy(self.fk.model), 'mapping': copy.deepcopy(self.fk.mapping),
                'session': self.session, 'source': 'rf_udp_20010', 'commands_enabled': False}

    def describe(self):
        names = ['肩前抬', '肩外摆', '大臂旋转', '肘关节', '腕旋转']
        channels, modules = [], []
        for side, offset, label in [('left', 0, '左臂'), ('right', 6, '右臂')]:
            arm = self.fk.model['arms'][side]
            ids = []
            for j, mid in enumerate(arm['motor_ids']):
                ident = f'{side}_joint_{mid}'
                ids.append(ident)
                raw = arm['obs_indices'][j]
                channels.append(dict(id=ident, label=names[j], index=offset+j, unit='rad',
                                     motor_id=mid, raw_indices=[raw, raw+20, raw+40]))
            modules.append(dict(id=side+'_arm', label=label, channels=ids))
        head_ids=[]
        for index, (name, label) in enumerate([('base_rotation','头部旋转'),('upper_head_raise','头部俯仰')],12):
            joint=self.fk.mapping['neck'][name]; raw=joint['position_obs_index']
            ident='head_joint_'+str(joint['axis_id']); head_ids.append(ident)
            channels.append(dict(id=ident,label=label,index=index,unit='rad',motor_id=joint['axis_id'],
                                 raw_indices=[raw,raw+20,raw+40],angle_basis='encoder',zero_calibrated=False))
        modules.append(dict(id='head',label='头部',channels=head_ids))
        tools=[];profile=getattr(self,'end_effector_config',{})
        for side,label in [('left','左侧'),('right','右侧')]:
            if profile.get(side)=='dexterous_hand':
                indices=self.fk.mapping['dexterous_hand'][side+'_obs_indices']
                tools.append(dict(id=side+'_hand',label=label+'灵巧手',kind='dexterous_hand',
                    note='原始归一化反馈 · 指位映射待确认',channels=[dict(id='raw_'+str(i),label='通道 '+str(j+1),
                    packet_path=['raw_observation',i],unit='normalized') for j,i in enumerate(indices)]))
        grippers=[label for side,label in [('left','左侧'),('right','右侧')] if profile.get(side)=='gripper']
        if grippers:
            tools.insert(0,dict(id='gripper',label=' / '.join(grippers)+'夹爪',kind='gripper',
                note='原始通道 · 左右与开合映射待确认',channels=[dict(id='raw_'+str(i),label='通道 '+name,
                packet_path=['raw_observation',i],unit='normalized') for i,name in [(60,'A'),(61,'B')]]))
        slots=[dict(id=side,label=label,options=[dict(id='none',label='未配置'),dict(id='gripper',label='夹爪'),dict(id='dexterous_hand',label='灵巧手')]) for side,label in [('left','左侧末端'),('right','右侧末端')]]
        return dict(version=1, id='rf_udp_20010', label='RF 双臂机器人',
                    source='rf_udp_20010', channels=channels, modules=modules,
                    end_effectors=tools,end_effector_slots=slots,
                    raw_packet_dimension=96, supports_raw_archive=True,
                    notes='头部为编码器角度，零位待标定。')

    def project_packet(self, packet, channels, retain_raw=False):
        if retain_raw:
            result=copy.deepcopy(packet)
            result['state']=result['state'][:12]
            return result
        keep = ('packet_id', 'session', 'request_ns', 'received_ns', 'device_ns',
                'timestamp_basis', 'clock_domain', 'device_sequence', 'model_id', 'mapping_version')
        result = {k: copy.deepcopy(packet[k]) for k in keep if k in packet}
        indices = sorted({i for c in channels for i in c['raw_indices']})
        result.update(state=[packet['state'][c['index']] for c in channels],
                      channel_ids=[c['id'] for c in channels],
                      raw_channels={str(i): packet['raw_observation'][i] for i in indices},
                      raw_packet_complete=False)
        return result

    def _make_packet(self, values, requested, received, ident):
        converted = self.fk.angles_from_observation(values)
        angles = {s: converted[s]['model_angles_rad'] for s in ('left', 'right')}
        # Unassigned gripper channels are NOT replaced by zeros or guessed sides.
        state = angles['left'] + [None] + angles['right'] + [None]
        state += [values[self.fk.mapping['neck'][name]['position_obs_index']]*2*math.pi
                  for name in ('base_rotation','upper_head_raise')]
        return {'packet_id': ident, 'session': self.session, 'raw_observation': values,
                'request_ns': requested, 'received_ns': received, 'device_ns': None,
                'measurement_host_ns': None, 'timestamp_basis': 'host_receive_unmapped_device_clock',
                'clock_domain': DOMAIN, 'device_sequence': None,
                'packet_id_semantics': 'local_response_counter_not_device_measurement_sequence',
                'state': state, 'ee_pose': None, 'valid': True,
                'joint_angles_rad': angles,
                'joint_motor_ids': {s: self.fk.model['arms'][s]['motor_ids'] for s in ('left', 'right')},
                'model_id': self.fk.model['model_id'], 'mapping_version': self.fk.mapping['mapping_version'],
                'zero_reference_calibrated': all(converted[s]['zero_calibration_verified'] for s in ('left', 'right')),
                'pose_source': 'deferred_offline_CAD_FK', 'fk_computation': 'offline_only',
                'gripper_raw': {'60': values[60], '61': values[61]},
                'hand_raw': {'62:68': values[62:68], '68:74': values[68:74]},
                'gripper_mapping_complete': False, 'training_ready': False}

    def _run(self):
        pending = False
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                # Exclusive ownership: never steal another reader's reply port.
                sock.bind((self.settings.get('bind_host', '0.0.0.0'), int(self.settings.get('receive_port', 3002))))
                sock.connect((self.settings.get('host', '192.168.11.100'), int(self.settings.get('port', 3000))))
                sock.settimeout(self.timeout_s)
                while not self.stop_event.is_set():
                    if select.select([sock], [], [], 0)[0]:
                        raise RuntimeError('Unexpected queued RF reply; sample identity cannot be established.')
                    requested = monotonic_ns()
                    self.pull_statistics.request()
                    pending = True
                    sock.send(struct.pack('>I', 20010))
                    self.request_count += 1
                    late = False
                    try:
                        data = sock.recv(4096)
                    except socket.timeout:
                        self.pull_statistics.result('timeouts')
                        pending = False
                        late = True
                        with self.condition:
                            self.error = 'RF reply timeout; waiting for the outstanding reply before sending another request.'
                            self.last_transport_error = self.error
                            self.condition.notify_all()
                        # There is exactly one outstanding request. Do not issue
                        # another while its delayed response could be misassigned.
                        expires = monotonic()+5
                        while not self.stop_event.is_set():
                            if monotonic() >= expires:
                                raise TimeoutError('Outstanding RF reply never arrived; reader remains stopped.')
                            try:
                                data = sock.recv(4096)
                                break
                            except socket.timeout:
                                continue
                        if self.stop_event.is_set():
                            break
                    received = monotonic_ns()
                    if received-requested > self.timeout_s*1e9:
                        late = True
                    if late:
                        if pending:
                            self.pull_statistics.result('timeouts')
                            pending = False
                        # Consume and discard the old request's reply. A new
                        # session forbids interpolation or recording across loss.
                        decode_observation(data)
                        with self.condition:
                            self.error = 'Discarded late RF reply; resynchronizing receive queue.'
                            self.last_transport_error = self.error
                            self.history.clear()
                            self.session = uuid.uuid4().hex
                            self.recovery_count += 1
                            self.condition.notify_all()
                        quiet_until = monotonic()+.1
                        drain_limit = monotonic()+1
                        while not self.stop_event.is_set() and monotonic() < quiet_until:
                            if monotonic() >= drain_limit:
                                raise RuntimeError('RF receive queue did not become quiet.')
                            if select.select([sock], [], [], .01)[0]:
                                decode_observation(sock.recv(4096))
                                quiet_until = monotonic()+.1
                        continue
                    packet = self._make_packet(decode_observation(data), requested, received, self.request_count)
                    self.pull_statistics.result('successes')
                    pending = False
                    with self.condition:
                        if self.history:
                            self.max_interval_ms = max(self.max_interval_ms, (received-self.history[-1]['received_ns'])/1e6)
                        self.max_rtt_ms = max(self.max_rtt_ms, (received-requested)/1e6)
                        self.history.append(packet)
                        self.error = None
                        self.condition.notify_all()
                    self.stop_event.wait(max(0, requested/1e9+1/self.poll_hz-monotonic()))
        except Exception as exc:
            if pending:
                self.pull_statistics.result('errors')
            if not self.stop_event.is_set():
                # Unrecoverable framing/source/lost-reply failures remain latched.
                with self.condition:
                    self.error = (f"反馈端口 {self.settings.get('receive_port',3002)} 已被其他程序占用，请释放该端口后重连。"
                                  if isinstance(exc,OSError) and exc.errno==errno.EADDRINUSE
                                  else f'{type(exc).__name__}: {exc}')
                    self.last_transport_error = self.error
                    self.condition.notify_all()

    def health_check(self):
        with self.condition:
            last = self.history[-1] if self.history else None
            age = (monotonic_ns()-last['received_ns'])/1e6 if last else None
            healthy = not self.error and not self.stop_event.is_set() and age is not None and 0 <= age <= self.max_age_ms
            return {'healthy': bool(healthy), 'mock': False, 'feedback_valid': bool(healthy),
                    'source': 'rf_udp_20010', 'interface': 'real_observation_only',
                    'model': self.fk.model['model_id'], 'age_ms': age, 'error': self.error,
                    'response_count': self.pull_statistics.lifetime()['successes'],
                    'pull_statistics': self.pull_statistics.lifetime(),
                    'request_count': self.request_count, 'poll_hz': self.poll_hz,
                    'max_rtt_ms': self.max_rtt_ms, 'max_response_interval_ms': self.max_interval_ms,
                    'recovery_count': self.recovery_count, 'last_transport_error': self.last_transport_error,
                    'zero_reference_calibrated': all(a['encoder_to_model'].get('zero_calibration_verified', False) for a in self.fk.model['arms'].values()),
                    'commands_enabled': False, 'gripper_mapping_complete': False,
                    'device_measurement_time_available': False, 'session': self.session}

    def live_snapshot(self):
        with self.condition:
            packet = copy.deepcopy(self.history[-1]) if self.history else None
        health = self.health_check()
        # Stale values are never presented as current feedback.
        return {'packet': packet if health['healthy'] else None, 'health': health,
                'last_received_ns': packet['received_ns'] if packet else None}

    def read_state(self):
        snapshot = self.live_snapshot()
        p = snapshot['packet']
        if p is None:
            return RobotState([None]*12, None, snapshot['last_received_ns'] or 0, None, False)
        return RobotState(list(p['state'][:12]), None, p['received_ns'],
                          None, True, p['received_ns'], clock_domain=DOMAIN)

    def interpolate(self, target_ns, wait_until):
        with self.condition:
            while True:
                if self.error:
                    raise RuntimeError('真实关节反馈中断：'+self.error)
                if self.stop_event.is_set():
                    raise RuntimeError('Real feedback reader stopped.')
                before = next((p for p in reversed(self.history) if p['received_ns'] <= target_ns), None)
                after = next((p for p in self.history if p['received_ns'] >= target_ns), None)
                if before is not None and after is not None:
                    break
                remaining = wait_until-monotonic()
                if remaining <= 0:
                    raise RuntimeError('真实反馈缺少固定采样时刻前后的数据包；未补帧。')
                self.condition.wait(min(.005, remaining))
            gap = after['received_ns']-before['received_ns']
            if gap > self.max_gap_ns or gap < 0 or before['session'] != after['session']:
                raise RuntimeError('真实反馈插值跨度超限或会话发生变化。')
        weight = (target_ns-before['received_ns'])/gap if gap else 0.
        state = []
        for side, start in [('left', 0), ('right', 6)]:
            angles = (np.asarray(before['state'][start:start+5])*(1-weight)
                      + np.asarray(after['state'][start:start+5])*weight)
            state.extend(angles.tolist()+[None])
        state.extend(before['state'][i]*(1-weight)+after['state'][i]*weight for i in (12,13))
        proof = {'before_packet_id': before['packet_id'], 'after_packet_id': after['packet_id'],
                 'before_received_ns': before['received_ns'], 'after_received_ns': after['received_ns'],
                 'before_ns': before['received_ns'], 'after_ns': after['received_ns'],
                 'weight': weight, 'gap_ms': gap/1e6, 'basis': 'host_receive_not_device_measurement'}
        return state, None, proof, (before, after)

    def send_ee_target(self, action):
        raise RuntimeError('当前仅接入真实观测；没有动作发送接口，不会发送运动指令。')

    def close(self):
        self.stop_event.set()
        with self.condition:
            self.condition.notify_all()
        self.thread.join(timeout=self.timeout_s+2)
