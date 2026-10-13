"""Transport and recording tests use only isolated localhost devices."""
from collections import deque
import copy
import io
import json
from pathlib import Path
import socket
import struct
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from collector.rf_robot_adapter import RFObservationAdapter, decode_observation, OBSERVATION_SCHEMA
from collector.rf_kinematics import CandidateKinematics
from collector.clock import monotonic_ns, monotonic
from collector.engine import Collector
from collector.adapters import CameraFrame, CAMERAS
from collector.exporter import check_raw


class LocalReplies:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', 0))
        self.sock.settimeout(.05)
        self.port = self.sock.getsockname()[1]
        self.stop = threading.Event()
        self.commands = []
        self.drop = False
        self.bad = False
        self.delay_once = 0
        raw = [0.0]*96
        for arm in CandidateKinematics().model['arms'].values():
            for index, zero in zip(arm['obs_indices'], arm['encoder_to_model']['zero_raw']):
                raw[index] = zero
        self.raw = raw
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        while not self.stop.is_set():
            try:
                data, source = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            self.commands.append(data)
            if not self.drop:
                if self.delay_once:
                    delay, self.delay_once = self.delay_once, 0
                    time.sleep(delay)
                body = b'bad' if self.bad else struct.pack('>I96fI', 20010, *self.raw, 20010)
                self.sock.sendto(body, source)

    def close(self):
        self.stop.set()
        self.thread.join(1)
        self.sock.close()


class FixtureCameras:
    """Test-owned frames, never imported by the production device factory."""
    def __init__(self, robot, config):
        self.lock = threading.Lock()
        self.frames = {k: deque(maxlen=30) for k in CAMERAS}
        self.stop = threading.Event()
        out = io.BytesIO()
        Image.new('RGB', (160, 120), 'grey').save(out, 'JPEG')
        self.jpeg = out.getvalue()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        ident = 0
        while not self.stop.is_set():
            ident += 1
            now = monotonic_ns()
            with self.lock:
                for key in CAMERAS:
                    self.frames[key].append(CameraFrame(key, ident, now, now, self.jpeg,
                        source='localhost_test_fixture', ready_ns=now))
            self.stop.wait(.008)

    def latest(self, key, at_ns=None):
        with self.lock:
            return next((f for f in reversed(self.frames[key]) if at_ns is None or f.received_ns <= at_ns), None)

    def candidates(self, key, at_ns):
        with self.lock:
            return [f for f in reversed(self.frames[key]) if f.received_ns <= at_ns]

    def health_check(self):
        return {k: {'healthy': bool(self.latest(k)), 'source': 'localhost_test_fixture',
                    'height': 120, 'width': 160, 'native_fps': 120, 'mock': True} for k in CAMERAS}

    def close(self):
        self.stop.set()
        self.thread.join(1)


class RealAcquisitionTests(unittest.TestCase):
    def setUp(self):
        self.server = LocalReplies()
        self.config = json.loads((ROOT/'config.json').read_text(encoding='utf-8'))
        self.config['rf_robot'] = {'host': '127.0.0.1', 'port': self.server.port,
                                  'bind_host': '127.0.0.1', 'receive_port': 0,
                                  'poll_hz': 50, 'timeout_ms': 60}
        self.config.update(mode='real', fps=20, minimum_free_mb=1)
        self.adapter = None

    def tearDown(self):
        if self.adapter:
            self.adapter.close()
        self.server.close()

    def connect(self):
        self.adapter = RFObservationAdapter(self.config)
        end = monotonic()+2
        while not self.adapter.health_check()['healthy'] and monotonic() < end:
            time.sleep(.01)
        self.assertTrue(self.adapter.health_check()['healthy'], self.adapter.error)
        return self.adapter

    def test_live_real_data_and_no_command_capability(self):
        a = self.connect()
        packet = a.live_snapshot()['packet']
        self.assertIsNone(packet['ee_pose'])
        self.assertEqual(packet['fk_computation'], 'offline_only')
        self.assertEqual(len(packet['joint_angles_rad']['left']), 5)
        self.assertIsNone(packet['state'][5])
        self.assertIsNone(packet['state'][11])
        self.assertEqual(packet['raw_observation'], self.server.raw)
        self.assertFalse(a.is_mock)
        self.assertIsNone(packet['device_ns'])
        with self.assertRaises(RuntimeError):
            a.send_ee_target([0]*16)
        self.assertTrue(all(c == struct.pack('>I', 20010) for c in self.server.commands))

    def test_cache_reads_preserve_original_receipt_time(self):
        a = self.connect()
        self.server.drop = True
        time.sleep(.025)
        first = a.live_snapshot()['last_received_ns']
        for _ in range(20):
            self.assertEqual(a.read_state().received_ns, first)

    def test_timeout_latches_and_no_delayed_reply_is_relabelled(self):
        a = self.connect()
        self.server.drop = True
        time.sleep(.18)
        n = len(self.server.commands)
        self.server.drop = False
        time.sleep(.09)
        self.assertEqual(n, len(self.server.commands))
        self.assertFalse(a.health_check()['healthy'])
        self.assertIsNone(a.live_snapshot()['packet'])
        with self.assertRaises(RuntimeError):
            a.interpolate(monotonic_ns(), monotonic()+.1)

    def test_malformed_packet_latches(self):
        a = self.connect()
        self.server.bad = True
        time.sleep(.09)
        self.assertIn('ValueError', a.error)
        self.assertIsNone(a.live_snapshot()['packet'])

    def test_late_reply_is_discarded_before_live_recovery(self):
        a = self.connect()
        old_session = a.session
        self.server.delay_once = .16
        time.sleep(.1)
        self.assertFalse(a.health_check()['healthy'])
        requests = len(self.server.commands)
        time.sleep(.02)
        self.assertEqual(len(self.server.commands), requests)
        end = monotonic()+2
        while (not a.health_check()['healthy'] or a.session == old_session) and monotonic() < end:
            time.sleep(.01)
        self.assertTrue(a.health_check()['healthy'], a.error)
        self.assertNotEqual(a.session, old_session)
        self.assertEqual(a.recovery_count, 1)
        self.assertTrue(all(p['session'] == a.session for p in a.history))
        self.assertLess(a.max_rtt_ms, 60)

    def test_packet_framing_and_nonfinite_values(self):
        body = struct.pack('>I96fI', 20010, *self.server.raw, 20010)
        self.assertEqual(len(decode_observation(body)), 96)
        invalid = [body[:-1], body+b'x', struct.pack('>I', 1)+body[4:], body[:-4]+struct.pack('>I', 1)]
        values = list(self.server.raw)
        values[3] = float('nan')
        invalid.append(struct.pack('>I96fI', 20010, *values, 20010))
        for data in invalid:
            with self.assertRaises(ValueError):
                decode_observation(data)

    def test_interpolation_has_bounded_sources_and_never_computes_fk(self):
        a = self.connect()
        time.sleep(.07)
        with a.condition:
            before, after = list(a.history)[-2:]
        target = (before['received_ns']+after['received_ns'])//2
        with patch.object(a.fk, 'forward', side_effect=AssertionError('Online FK forbidden')):
            with patch.object(a.fk, 'forward_from_observation', side_effect=AssertionError('Online FK forbidden')):
                state, pose, proof, packets = a.interpolate(target, monotonic()+.05)
                a._make_packet(self.server.raw, target, target+1, 999)
        self.assertIsNone(pose)
        self.assertIsNone(state[5])
        self.assertEqual([p['packet_id'] for p in packets], [proof['before_packet_id'], proof['after_packet_id']])
        self.assertLessEqual(proof['gap_ms'], 40)
        a.max_gap_ns = 1
        with self.assertRaises(RuntimeError):
            a.interpolate(target, monotonic()+.05)

    def test_simulation_modes_are_rejected_before_opening_devices(self):
        for mode in ('mock', 'hybrid'):
            with self.assertRaisesRegex(ValueError, 'Only real'):
                Collector(dict(self.config, mode=mode), ROOT)

    def test_record_replay_preserves_real_source_and_missing_action(self):
        # This test verifies the full writer with localhost RF packets and test
        # images, never labels these fixtures as hardware acceptance evidence.
        # Allow test-process scheduling on a Jetson concurrently running cameras;
        # the actual 40ms production gap and rejection are tested separately.
        self.config.update(alignment_wait_budget_ms=100, state_interpolation_max_gap_ms=100,
                           sample_lateness_max_ms=150)
        with tempfile.TemporaryDirectory() as directory:
            with patch('collector.orbbec_camera.RealCameras', FixtureCameras):
                app = Collector(self.config, directory)
            try:
                end = monotonic()+2
                while not app.robot.health_check()['healthy'] and monotonic() < end:
                    time.sleep(.01)
                with self.assertRaises(ValueError):
                    app.start('forbidden simulated capture', 'test', purpose='test')
                with self.assertRaises(ValueError):
                    app.start('missing actions', 'test', purpose='training')
                ep = app.start('local transport test', 'test', fps=20)['episode_id']
                time.sleep(.55)
                if app.phase == 'RECORDING':
                    app.stop()
                self.assertEqual(app.phase, 'REVIEW', app.last_error)
                row = app.replay(ep, 0)
                self.assertIsNone(row['action'])
                self.assertIsNone(row['command'])
                self.assertFalse(row['mock'])
                self.assertIsNone(row['observation.ee_pose'])
                self.assertEqual(row['fk_computation'], 'offline_only')
                self.assertEqual(row['state_valid_mask'], [True]*5+[False]+[True]*5+[False])
                path = app.episode_path(ep)
                manifest = json.loads((path/'manifest.json').read_text(encoding='utf-8'))
                self.assertEqual(manifest['schema_version'], OBSERVATION_SCHEMA)
                self.assertFalse(manifest['training_ready'])
                packets = [json.loads(line) for line in (path/'feedback.jsonl').read_text().splitlines()]
                self.assertTrue(packets)
                self.assertTrue(all(len(p['raw_observation']) == 96 for p in packets))
                self.assertIn(row['alignment']['joints']['before_packet_id'], {p['packet_id'] for p in packets})
                with self.assertRaises(ValueError):
                    check_raw(path)
                with self.assertRaises(ValueError):
                    app.export([ep])
            finally:
                app.close()


if __name__ == '__main__':
    unittest.main()
