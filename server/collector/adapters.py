"""Replaceable hardware contracts and deterministic mock devices.

The mock geometry is a pipeline fixture, not an X7 kinematic model.
"""
from dataclasses import dataclass
import io
import math
import threading
import time
from .clock import monotonic, monotonic_ns, DOMAIN
from typing import Optional, Protocol

import numpy as np
from PIL import Image, ImageDraw

CAMERAS = ('head', 'left_wrist')
STATE_NAMES = [f'left_joint_{i}' for i in range(1, 6)] + ['left_gripper'] + [
    f'right_joint_{i}' for i in range(1, 6)] + ['right_gripper']
ACTION_NAMES = [f'{side}_{name}' for side in ('left', 'right') for name in
                ('ee_x', 'ee_y', 'ee_z', 'ee_qw', 'ee_qx', 'ee_qy', 'ee_qz', 'gripper_target')]
SCHEMA = 'tabletop_bimanual_eef_quat_2cam_v2'


@dataclass
class RobotState:
    state: list[float]
    actual_ee: list[float]
    received_ns: int
    device_ns: Optional[int]
    valid: bool = True
    ee_received_ns: Optional[int] = None
    measurement_host_ns: Optional[int] = None
    ee_measurement_host_ns: Optional[int] = None
    timestamp_basis: str = 'host_receive_unmapped_device_clock'
    ee_timestamp_basis: str = 'host_receive_unmapped_device_clock'
    clock_domain: str = 'unknown_device_clock'
    clock_mapping_id: Optional[str] = None
    clock_uncertainty_ns: Optional[int] = None


@dataclass
class AcceptedCommand:
    command_id: str
    requested: list[float]
    accepted: list[float]
    joint_target: list[float]
    requested_ns: int
    sent_ns: int
    accepted_ok: bool = True
    accepted_ns: Optional[int] = None
    effective_ns: Optional[int] = None
    timing_basis: str = 'host_send_only_effect_unknown'


class RobotAdapter(Protocol):
    is_mock: bool

    def read_state(self) -> RobotState: ...
    def send_ee_target(self, action: list[float]) -> AcceptedCommand: ...
    def health_check(self) -> dict: ...
    def close(self) -> None: ...


class BaseAdapter(Protocol):
    """Reserved independent channel; it is disabled in the tabletop schema.

    Future send_navigation_goal receives a planar map/odom-frame target, NOT a
    raw camera pixel or an uncalibrated 3D point. Navigation owns path execution.
    """
    def read_state(self) -> dict: ...
    def send_navigation_goal(self, x_m: float, y_m: float,
                             yaw_rad: Optional[float], frame: str, goal_id: str) -> dict: ...
    def cancel_goal(self, goal_id: str) -> dict: ...
    def stop(self) -> dict: ...
    def health_check(self) -> dict: ...


class DisabledBase:
    def read_state(self):
        return {'enabled': False, 'mode': 'fixed_base', 'pose': None, 'velocity': None}

    def send_navigation_goal(self, x_m, y_m, yaw_rad=None, frame='map', goal_id=''):
        raise RuntimeError('Base navigation is disabled for the tabletop schema.')

    def cancel_goal(self, goal_id):
        return {'enabled': False, 'cancelled': False, 'goal_id': goal_id}

    def stop(self):
        return {'enabled': False, 'stop_sent': False}

    def health_check(self):
        return {'enabled': False, 'healthy': True, 'mode': 'fixed_base'}


def mock_fk(q):
    """An invertible 5-axis *virtual* fixture: translation 3 + roll/pitch 2."""
    out = []
    for start, y in ((0, .24), (6, -.24)):
        j = q[start:start + 5]
        roll, pitch = j[3], j[4]
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        out += [.36 + .10 * j[0], y + .12 * j[1], .25 + .10 * j[2],
                cr * cp, sr * cp, cr * sp, -sr * sp, q[start + 5]]
    return out


def mock_ik(action):
    q = []
    for start, y in ((0, .24), (8, -.24)):
        x, yy, z, w, qx, qy, qz, g = action[start:start + 8]
        norm = math.sqrt(w*w + qx*qx + qy*qy + qz*qz)
        if norm < 1e-6:
            raise ValueError('Zero quaternion.')
        w, qx, qy, qz = np.array([w, qx, qy, qz]) / norm
        roll = math.atan2(2*(w*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
        pitch = math.asin(float(np.clip(2*(w*qy - qz*qx), -1, 1)))
        yaw = math.atan2(2*(w*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
        if abs(yaw) > .03:
            raise ValueError('The mock 5-axis fixture cannot realize arbitrary yaw.')
        q += list(np.clip([(x-.36)/.10, (yy-y)/.12, (z-.25)/.10, roll, pitch], -1.2, 1.2))
        q += [float(np.clip(g, 0, 1))]
    return q


def virtual_demonstration(seconds):
    q = []
    for side in (0, 1):
        t = seconds + side * .8
        q += [.6*math.sin(t), .5*math.sin(t*.7), .4*math.sin(t*.9),
              .3*math.sin(t*.6), .4*math.sin(t*.5), .5 + .45*math.sin(t)]
    return mock_fk(q)


class MockRobot:
    is_mock = True

    def __init__(self):
        self.lock = threading.Lock()
        self.q = np.zeros(12, dtype=np.float64)
        self.target = self.q.copy()
        self.last = monotonic()
        self.command_seq = 0
        self.feedback_fault = False

    def read_state(self):
        with self.lock:
            now = monotonic()
            # Actual simulation feedback intentionally lags commanded targets.
            self.q += (self.target - self.q) * (1 - math.exp(-6 * (now - self.last)))
            self.last = now
            ts = monotonic_ns()
            return RobotState(self.q.tolist(), mock_fk(self.q.tolist()), ts, ts,
                              not self.feedback_fault, ts, ts, ts,
                              'simulated_measurement', 'simulated_measurement',
                              DOMAIN, 'mock_identity_v1', 0)

    def send_ee_target(self, action):
        a = np.asarray(action, dtype=np.float64)
        if a.shape != (16,) or not np.isfinite(a).all():
            raise ValueError('Expected 16 finite action values.')
        requested_ns = monotonic_ns()
        q = mock_ik(a.tolist())
        with self.lock:
            self.command_seq += 1
            self.target = np.array(q)
            sent = monotonic_ns()
            return AcceptedCommand(f'mock-{self.command_seq:08d}', a.tolist(), mock_fk(q),
                                   q, requested_ns, sent, True, sent, sent, 'simulated_immediate')

    def health_check(self):
        return {'healthy': not self.feedback_fault, 'mock': True, 'feedback_valid': not self.feedback_fault,
                'model': 'virtual_5axis_fixture_not_robot_urdf'}

    def close(self):
        pass


@dataclass(frozen=True)
class CameraFrame:
    key: str
    frame_id: int
    captured_ns: int
    received_ns: int
    jpeg: bytes
    source: str = 'mock'
    serial: Optional[str] = None
    device_timestamp_ms: Optional[float] = None
    timestamp_basis: str = 'host_generated'
    measurement_host_ns: Optional[int] = None
    device_clock_domain: str = 'unknown'
    device_timestamp_semantics: str = 'unknown'
    clock_mapping_id: Optional[str] = None
    clock_uncertainty_ns: Optional[int] = None
    exposure_us: Optional[float] = None
    sensor_timestamp_raw: Optional[float] = None
    ready_ns: Optional[int] = None
    camera_session: Optional[str] = None
    timing_metadata: Optional[dict] = None
    sdk_arrival_host_ns: Optional[int] = None


class CameraAdapter(Protocol):
    def latest(self, key: str, at_ns: Optional[int] = None) -> Optional[CameraFrame]: ...
    def health_check(self) -> dict: ...
    def close(self) -> None: ...


class MockCameras:
    def __init__(self, robot, width=320, height=240, fps=30, keys=CAMERAS):
        from collections import deque
        self.robot, self.width, self.height, self.fps = robot, width, height, fps
        self.keys = keys
        self.lock = threading.Lock()
        self.frames = {k: deque(maxlen=12) for k in self.keys}
        self.faults = set()
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, name='mock-cameras', daemon=True)
        self.thread.start()

    def _draw(self, key, frame_id, state):
        image = Image.new('RGB', (self.width, self.height), '#18283c')
        d = ImageDraw.Draw(image)
        d.rectangle((18, 60, self.width-18, self.height-22), fill='#a18864', outline='#d3bd98')
        d.text((12, 12), f'MOCK | {key} | frame {frame_id}', fill='white')
        d.text((12, 31), 'SIMULATED DATA - NOT ROBOT TRAINING', fill='#ffce76')
        d.rectangle((self.width//2-18, self.height//2-10, self.width//2+18, self.height//2+26),
                    fill='#eb9250')
        for start, color, offset in ((0, '#74d9e7', 0), (8, '#9be4aa', 70)):
            ee = state.actual_ee[start:start+8]
            x = int(self.width*.46 + (ee[0]-.36)*650 + offset)
            y = int(self.height*.65 - (ee[2]-.25)*550)
            if key == 'left_wrist' and start == 8:
                continue
            if key == 'right_wrist' and start == 0:
                continue
            d.line((30+offset*3, self.height-24, x, y), fill=color, width=7)
            gap = int(4 + 12 * (1-ee[7]))
            d.line((x-gap, y-12, x-gap, y+8), fill=color, width=4)
            d.line((x+gap, y-12, x+gap, y+8), fill=color, width=4)
        out = io.BytesIO()
        image.save(out, format='JPEG', quality=85)
        return out.getvalue()

    def _run(self):
        seq, deadline = 0, monotonic()
        while not self.stop_event.is_set():
            seq += 1
            state = self.robot.read_state()
            for key in self.keys:
                if key in self.faults:
                    continue
                captured_ns = state.received_ns
                jpeg = self._draw(key, seq, state)
                frame = CameraFrame(key, seq, captured_ns, monotonic_ns(), jpeg,
                    measurement_host_ns=captured_ns, device_clock_domain=DOMAIN,
                    device_timestamp_semantics='simulated_measurement',
                    clock_mapping_id='mock_identity_v1', clock_uncertainty_ns=0)
                with self.lock:
                    self.frames[key].append(frame)
            deadline += 1/self.fps
            self.stop_event.wait(max(0, deadline - monotonic()))
            if deadline < monotonic() - .1:
                deadline = monotonic()

    def latest(self, key, at_ns=None):
        with self.lock:
            frames = self.frames.get(key, ())
            if at_ns is None:
                return frames[-1] if frames else None
            from .timing import frame_time
            return next((f for f in reversed(frames) if frame_time(f) <= at_ns), None)

    def health_check(self):
        now = monotonic_ns()
        return {k: {'mock': True, 'native_fps':self.fps,'width':self.width,'height':self.height,
                    'healthy': bool(self.latest(k)) and
                    (now-self.latest(k).captured_ns)/1e6 < 100,
                    'age_ms': round((now-self.latest(k).captured_ns)/1e6, 2)
                    if self.latest(k) else None} for k in self.keys}

    def set_fault(self, key, enabled):
        if enabled: self.faults.add(key)
        else: self.faults.discard(key)

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=3)


class MockProvidedSDK:
    """A supplied-SDK contract fixture backed by the virtual 5-axis robot."""
    is_mock = True

    def __init__(self, robot):
        self.robot = robot

    def read_actual_state(self):
        s = self.robot.read_state()
        return {'state': s.state, 'actual_ee': s.actual_ee,
                'ee_pose':s.actual_ee[:7]+s.actual_ee[8:15], 'valid': s.valid,
                'ee_received_monotonic_ns':s.received_ns,
                'received_monotonic_ns': s.received_ns, 'device_timestamp_ns': s.device_ns,
                'measurement_host_ns':s.measurement_host_ns,
                'ee_measurement_host_ns':s.ee_measurement_host_ns,
                'timestamp_basis':s.timestamp_basis, 'ee_timestamp_basis':s.ee_timestamp_basis,
                'clock_domain':s.clock_domain, 'clock_mapping_id':s.clock_mapping_id,
                'clock_uncertainty_ns':s.clock_uncertainty_ns}

    def send_ee_target(self, action):
        c = self.robot.send_ee_target(action)
        return {'accepted_ok': c.accepted_ok, 'accepted_action': c.accepted,
                'joint_target': c.joint_target, 'command_id': c.command_id,
                'sent_monotonic_ns': c.sent_ns, 'accepted_monotonic_ns':c.accepted_ns,
                'effective_monotonic_ns':c.effective_ns, 'timing_basis':c.timing_basis}

    def health_check(self):
        return self.robot.health_check() | {'interface': 'supplied_sdk_fixture'}

    def close(self):
        self.robot.close()
