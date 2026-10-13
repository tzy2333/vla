"""Isolated contract, selection, persistence and telemetry tests; no real devices."""
import copy
from collections import deque
import importlib.metadata
import io
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from types import SimpleNamespace

from PIL import Image
from collector.adapters import CameraFrame
from collector.clock import monotonic, monotonic_ns
from collector.engine import Collector
from collector.platform_contract import selection, validate_descriptor, camera_bindings
from collector.pull_statistics import PullStatistics
from collector.rf_robot_adapter import RFObservationAdapter
from collector.platform_management import summarize
from collector.segments import SegmentAnnotations
from test_rf_real_acquisition import LocalReplies
from monitor import Feed

ROOT=Path(__file__).resolve().parents[1]


class TestCameras:
    def __init__(self,robot,config):
        self.keys=list(camera_bindings(config));self.bad=set(config.get('fixture_bad_cameras',[]))
        self.frames={k:deque(maxlen=40) for k in self.keys};self.lock=threading.Lock();self.stop=threading.Event()
        out=io.BytesIO();Image.new('RGB',(16,12),'grey').save(out,'JPEG');self.jpeg=out.getvalue()
        self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()
    def run(self):
        seq=0
        while not self.stop.is_set():
            seq+=1;now=monotonic_ns()
            with self.lock:
                for k in self.keys:
                    if k not in self.bad:self.frames[k].append(CameraFrame(k,seq,now,now,self.jpeg,source='isolated_test',ready_ns=now))
            self.stop.wait(.008)
    def latest(self,key,at_ns=None):
        with self.lock:return next((f for f in reversed(self.frames[key]) if at_ns is None or f.received_ns<=at_ns),None)
    def candidates(self,key,at_ns):
        with self.lock:return [f for f in reversed(self.frames[key]) if f.received_ns<=at_ns]
    def health_check(self):
        return {k:dict(healthy=k not in self.bad and bool(self.latest(k)),source='isolated_test',mock=True,width=16,height=12,native_fps=120) for k in self.keys}
    def close(self):self.stop.set();self.thread.join(1)


class ModularCaptureTests(unittest.TestCase):
    def setUp(self):
        self.server=LocalReplies();self.tmp=tempfile.TemporaryDirectory()
        self.config=json.loads((ROOT/'config.json').read_text(encoding='utf-8'))
        self.config.update(data_dir='data',minimum_free_mb=1,fps=10,alignment_wait_budget_ms=150,
                           state_interpolation_max_gap_ms=150,sample_lateness_max_ms=200)
        self.config['rf_robot']=dict(host='127.0.0.1',port=self.server.port,bind_host='127.0.0.1',receive_port=0,poll_hz=50,timeout_ms=60)
        self.app=None
    def tearDown(self):
        if self.app:self.app.close()
        self.server.close();self.tmp.cleanup()
    def connect(self,expect_robot=True):
        with patch('collector.orbbec_camera.RealCameras',TestCameras):self.app=Collector(copy.deepcopy(self.config),self.tmp.name)
        deadline=monotonic()+2
        while expect_robot and not self.app.robot.health_check()['healthy'] and monotonic()<deadline:time.sleep(.01)
        time.sleep(.05)
        return self.app
    def record(self,chosen):
        ep=self.app.start('isolated modular test','test',fps=10,capture_selection=chosen)['episode_id']
        time.sleep(.39)
        if self.app.phase=='RECORDING':self.app.stop()
        self.assertEqual(self.app.phase,'REVIEW',self.app.last_error)
        path=self.app.episode_path(ep)
        return path,json.loads((path/'manifest.json').read_text(encoding='utf-8')),self.app.replay(ep,0)
    def test_left_arm_and_wrist_do_not_persist_other_modules(self):
        self.config['fixture_bad_cameras']=['head'];self.connect()
        path,m,row=self.record(dict(modules=['left_arm'],cameras=['left_wrist']))
        self.assertEqual(m['camera_keys'],['left_wrist']);self.assertFalse((path/'cameras/head').exists())
        self.assertEqual(len(row['observation.state']),5);self.assertEqual(row['state_valid_mask'],[True]*5)
        packets=[json.loads(x) for x in (path/'feedback.jsonl').read_text().splitlines()]
        for packet in packets:
            self.assertNotIn('raw_observation',packet)
            self.assertEqual(len(packet['state']),5)
            self.assertTrue(all(c.startswith('left_') for c in packet['channel_ids']))
            self.assertTrue(all(int(i)%2==1 and int(i)<50 for i in packet['raw_channels']))
        self.assertIsNotNone(m['pull_statistics']);self.assertGreater(m['pull_statistics']['attempts'],0)
    def test_subtask_mark_stop_and_next_recording_keep_pending_review(self):
        app=self.connect();annotations=SegmentAnnotations(app)
        chosen=dict(modules=['head'],cameras=[])
        ep=app.start('isolated segmented capture','test',fps=10,capture_selection=chosen)['episode_id']
        time.sleep(.25)
        snapshot=annotations.read(ep)
        doc=annotations.mutate(dict(episode_id=ep,revision=snapshot['revision'],operation='mark',label='抓取'))
        boundary=doc['marks'][-1]['frame'];self.assertGreater(boundary,0)
        time.sleep(.25);app.stop()
        self.assertEqual(app.phase,'REVIEW',app.last_error)
        doc=annotations.read(ep)
        self.assertEqual(len(doc['segments']),2)
        self.assertEqual(doc['segments'][0]['end_frame_exclusive'],boundary)
        self.assertEqual(sum(s['frame_count'] for s in doc['segments']),app.frames)
        before=(app.episode_path(ep)/'manifest.json').read_bytes()
        app.prepare_next(ep)
        next_ep=app.start('isolated segmented capture','test',fps=10,capture_selection=chosen)['episode_id']
        time.sleep(.2);app.stop()
        self.assertNotEqual(next_ep,ep)
        self.assertEqual((app.episode_path(ep)/'manifest.json').read_bytes(),before)
        self.assertEqual(annotations.read(next_ep)['marks'][0]['label'],'')

    def test_head_channels_are_live_and_can_be_recorded_independently(self):
        import math
        self.server.raw[14]=.125;self.server.raw[15]=-.25
        app=self.connect()
        self.assertEqual(app.status()['state_dim'],12)
        self.assertAlmostEqual(app.status()['state'][-2],math.pi/4)
        path,m,row=self.record(dict(modules=['head'],cameras=[]))
        self.assertEqual(len(row['observation.state']),2)
        self.assertEqual(m['capture_selection']['modules'],['head'])
        self.assertAlmostEqual(row['observation.state'][1],-math.pi/2)
        for line in (path/'feedback.jsonl').read_text().splitlines():
            packet=json.loads(line)
            self.assertEqual(set(packet['raw_channels']),{'14','15','34','35','54','55'})
        self.assertTrue(all(c['angle_basis']=='encoder' and not c['zero_calibrated'] for c in m['state_channels']))

    def test_camera_release_state_survives_service_restart(self):
        app=self.connect();app.release_cameras()
        saved=json.loads(app.device_state_path.read_text())
        self.assertEqual(set(saved['released_cameras']),{'head','left_wrist'})
        app.close();self.app=None
        with patch('collector.orbbec_camera.OrbbecRGB',side_effect=AssertionError('must stay released')):
            self.app=Collector(copy.deepcopy(self.config),self.tmp.name)
        self.assertTrue(all(c['error']=='相机已释放' for c in self.app.health()['cameras'].values()))
    def test_cameras_only_continue_without_robot_feedback(self):
        self.server.drop=True;self.connect(expect_robot=False)
        path,m,row=self.record(dict(modules=[],cameras=['head']))
        self.assertEqual(row['observation.state'],[]);self.assertIsNone(row['alignment']['joints'])
        self.assertEqual((path/'feedback.jsonl').read_text(),'');self.assertIsNone(m['pull_statistics'])
        self.assertNotIn('robot',m['device_sources'])
        self.assertEqual(summarize([m])['total'],1)
    def test_joints_only_ignore_failed_cameras(self):
        self.config['fixture_bad_cameras']=['head','left_wrist'];self.connect()
        path,m,row=self.record(dict(modules=['right_arm'],cameras=[]))
        self.assertEqual(row['images'],{});self.assertEqual(m['camera_keys'],[])
        self.assertFalse((path/'cameras').exists());self.assertEqual(len(row['observation.state']),5)
    def test_three_cameras_are_written_with_separate_frames(self):
        self.config['wrist_cameras']['right_wrist']=dict(serial='isolated-right',width=16,height=12,fps=120)
        self.connect();path,m,row=self.record(dict(modules=['left_arm','right_arm'],cameras=['right_wrist','head','left_wrist']))
        self.assertEqual(m['camera_keys'],['head','left_wrist','right_wrist'])
        self.assertEqual(len(row['observation.state']),10)
        for k in m['camera_keys']:
            files=list((path/'cameras'/k).glob('*.jpg'));self.assertEqual(len(files),m['frame_count'])
            with Image.open(files[0]) as im:self.assertEqual(im.size,(16,12))
    def test_invalid_selection_does_not_start_writer(self):
        app=self.connect()
        for choice in [dict(modules=[],cameras=[]),dict(modules=['unknown'],cameras=[]),dict(modules=[],cameras=['right_wrist']),dict(modules=['left_arm','left_arm'],cameras=[])]:
            with self.assertRaises(ValueError):app.start('bad','test',capture_selection=choice)
        self.assertIsNone(app.writer)
    def test_seven_joint_plugin_runs_without_rf_layout_in_engine(self):
        class SevenJointAdapter(RFObservationAdapter):
            def describe(self):
                d=super().describe();d.update(id='test_seven',source='isolated_seven',label='Seven joints')
                d['channels']=d['channels'][:7];d['modules']=[dict(id='arm',label='Arm',channels=[c['id'] for c in d['channels']])]
                return d
        self.config['robot_adapter']='test_seven'
        entry=SimpleNamespace(load=lambda:SevenJointAdapter)
        with patch('collector.platform_contract.importlib.metadata.entry_points',return_value=[entry]):self.connect()
        path,m,row=self.record(dict(modules=['arm'],cameras=[]))
        self.assertEqual(len(row['observation.state']),7);self.assertEqual(self.app.status()['state_dim'],7)
        self.assertEqual(m['robot_adapter_id'],'test_seven')
    def test_capture_config_snapshot_is_immutable(self):
        app=self.connect();_,m,_=self.record(dict(modules=['left_arm'],cameras=[]))
        before=copy.deepcopy(app.writer.manifest['config']);app.config['fps']=50
        self.assertEqual(app.writer.manifest['config'],before)
    def test_camera_release_keeps_robot_alive_and_blocks_active_recording(self):
        app=self.connect();original=app.cameras
        app.start('release guard','test',fps=10,capture_selection=dict(modules=['left_arm'],cameras=[]))
        with self.assertRaises(ValueError):app.release_cameras()
        time.sleep(.2)
        if app.phase=='RECORDING':app.stop()
        result=app.release_cameras();self.assertTrue(result['released']);self.assertTrue(original.stop.is_set())
        self.assertTrue(all(v['error']=='相机已释放' and not v['healthy'] for v in app.health()['cameras'].values()))
        self.assertIsNone(app.cameras.latest('head'));self.assertTrue(app.robot.health_check()['healthy'])


class StatisticsTests(unittest.TestCase):
    def test_request_before_recording_is_excluded_and_timeouts_count_once(self):
        stats=PullStatistics();stats.request();stats.begin();stats.result('successes')
        self.assertEqual(stats.recording()['attempts'],0);self.assertIsNone(stats.recording()['success_rate'])
        stats.request();stats.result('successes');stats.request();stats.result('timeouts')
        stats.request();stats.result('errors');stats.request()
        result=stats.finish();self.assertEqual(result['success_rate'],25)
        self.assertEqual(result['pending'],1);stats.result('successes')
        self.assertEqual(stats.recording(),result)
        stats.begin();self.assertEqual(stats.recording()['attempts'],0)
    def test_unknown_and_duplicate_channel_metadata_rejected(self):
        descriptor=dict(version=1,channels=[dict(id='j',index=0,unit='rad')],modules=[dict(id='arm',channels=['other'])])
        with self.assertRaises(ValueError):validate_descriptor(descriptor)
    def test_terminal_deduplicates_cached_samples_and_never_invents_joints(self):
        feed=Feed();status=dict(phase='IDLE',feedback_valid=False,robot_feedback=None,
                               health=dict(robot=dict(healthy=False,error='offline'),cameras={}))
        first=feed.events(status);self.assertFalse(any(e[0]=='JOINT' for e in first))
        self.assertEqual(feed.events(status),[])
        status['health']['cameras']['head']=dict(healthy=True,frame_id=1,received_ns=99,camera_session='s',width=16,height=12,age_ms=2)
        self.assertEqual(sum(e[0]=='IMAGE' for e in feed.events(status)),1)
        self.assertEqual(feed.events(status),[])


if __name__=='__main__':unittest.main()
