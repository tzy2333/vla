"""Deterministic timing cases plus a hardware-free 50 Hz record/export check."""
import copy
from dataclasses import replace
import json
from pathlib import Path
import sys
import time
import unittest
import uuid
import queue
import threading

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from collector.adapters import RobotState, CameraFrame, mock_fk, CAMERAS
from collector.engine import Collector
from collector.clock import monotonic_ns
from collector.exporter import check_raw, training_sample
from collector.storage import RawWriter
from collector.timing import ClockMapping, StateHistory, frame_time, slerp_wxyz, configure_timing


def packet(t, q=0, ee_t=None):
    values=[q]*12
    values[5]=values[11]=.5
    return RobotState(values,mock_fk(values),t,t,True,ee_t or t)


class InterpolationTests(unittest.TestCase):
    def test_feedback_producer_progresses_during_interpolation_math(self):
        h=StateHistory(); h.add(packet(10)); h.add(packet(30,.5))
        computing=threading.Event(); release=threading.Event(); inserted=threading.Event()
        original=h._between
        def slow(*args,**kwargs):
            computing.set()
            self.assertTrue(release.wait(2))
            return original(*args,**kwargs)
        h._between=slow
        results=[]
        reader=threading.Thread(target=lambda:results.append(h.interpolate(20)))
        def insert():
            h.add(packet(40,.6)); inserted.set()
        producer=threading.Thread(target=insert)
        reader.start()
        try:
            self.assertTrue(computing.wait(1))
            producer.start()
            self.assertTrue(inserted.wait(1),'Interpolation math blocked new feedback')
        finally:
            release.set(); reader.join(3)
            if producer.ident is not None: producer.join(3)
        self.assertEqual(results[0][2]['joints']['before_ns'],10)
        self.assertEqual(results[0][2]['joints']['after_ns'],30)
        self.assertAlmostEqual(results[0][0][0],.25)

    def test_invalid_tcp_does_not_partially_update_joint_history(self):
        h=StateHistory(); h.add(packet(10))
        invalid=packet(20,.3)
        invalid.actual_ee[3:7]=[0,0,0,0]
        with self.assertRaises(ValueError): h.add(invalid)
        self.assertEqual(h.joints[-1]['t'],10)
        self.assertEqual(h.tcp[-1]['t'],10)

    def test_writer_overflow_never_silently_drops_feedback(self):
        writer=RawWriter.__new__(RawWriter)
        writer.error=None; writer.queue=queue.Queue(maxsize=1)
        writer.queue.put(('feedback',{},None))
        with self.assertRaisesRegex(RuntimeError,'queue full'):
            writer.submit_feedback({'packet_id':2})

    def test_repeated_stale_and_skewed_camera_frames_rejected(self):
        app=Collector.__new__(Collector)
        app.config={'fps':50,'camera_wait_budget_ms':1,'camera_max_age_ms':60,
                    'camera_alignment_max_offset_ms':30,'camera_pair_max_skew_ms':20}
        app.stop_event=threading.Event(); app.frames=0; app.started_ns=monotonic_ns()
        class Cameras:
            def latest(self,key,at_ns): return frames[key]
        app.cameras=Cameras()
        normal={k:CameraFrame(k,1,app.started_ns-1_000_000,app.started_ns-1_000_000,b'') for k in CAMERAS}
        for kind in ('repeated','stale','skew'):
            frames=normal.copy(); previous={k:0 for k in CAMERAS}
            if kind=='repeated': previous['head']=1
            if kind=='stale': frames['head']=replace(frames['head'],received_ns=app.started_ns-100_000_000)
            if kind=='skew': frames['head']=replace(frames['head'],received_ns=app.started_ns-25_000_000)
            with self.assertRaises(RuntimeError): app.capture_group(previous,app.started_ns/1e9-1)

    def test_known_linear_midpoint(self):
        h=StateHistory()
        h.add(packet(96_000_000,.40)); h.add(packet(104_000_000,.48))
        q,ee,proof=h.interpolate(100_000_000)
        self.assertAlmostEqual(q[0],.44)
        self.assertAlmostEqual(ee[0],.404)
        self.assertEqual(proof['joints']['weight'],.5)
        self.assertEqual(proof['joints']['gap_ms'],8)

    def test_slerp_rotation_and_antipodal(self):
        np.testing.assert_allclose(slerp_wxyz([1,0,0,0],[0,0,0,1],.5),
                                   [2**-.5,0,0,2**-.5],atol=1e-12)
        np.testing.assert_allclose(slerp_wxyz([1,0,0,0],[-1,0,0,0],.5),[1,0,0,0])
        with self.assertRaises(ValueError): slerp_wxyz([0,0,0,0],[1,0,0,0],.5)

    def test_no_extrapolation_and_no_long_gap(self):
        h=StateHistory(max_gap_ms=10)
        h.add(packet(10_000_000)); h.add(packet(40_000_000,.5))
        for t in (0,50_000_000):
            with self.assertRaises(RuntimeError): h.interpolate(t)
        with self.assertRaises(ValueError): h.interpolate(20_000_000)

    def test_separate_tcp_timeline(self):
        h=StateHistory()
        h.add(packet(10_000_000,0,5_000_000))
        h.add(packet(30_000_000,1,25_000_000))
        q,ee,proof=h.interpolate(20_000_000)
        self.assertEqual(proof['joints']['weight'],.5)
        self.assertEqual(proof['tcp']['weight'],.75)
        self.assertAlmostEqual(q[0],.5)
        self.assertAlmostEqual(ee[0],.435)

    def test_cached_packet_not_new_measurement(self):
        h=StateHistory(); p=packet(10)
        self.assertEqual(h.add(p),1)
        self.assertIsNone(h.add(p))
        with self.assertRaises(ValueError): h.add(packet(10,.3))

    def test_clock_reset_invalid_future_and_mapping_boundary(self):
        h=StateHistory(); h.add(packet(20))
        with self.assertRaises(ValueError): h.add(packet(10))
        with self.assertRaises(ValueError): StateHistory().add(replace(packet(20),valid=False))
        with self.assertRaises(ValueError): StateHistory().add(packet(monotonic_ns()+10**9))
        h=StateHistory(); h.add(packet(10))
        with self.assertRaises(ValueError): h.add(replace(packet(20),clock_domain='different'))

    def test_mapping_requires_provenance(self):
        with self.assertRaises(ValueError):
            StateHistory().add(replace(packet(100),measurement_host_ns=80))

    def test_only_configured_continuous_joints_unwrap(self):
        h=StateHistory(continuous_joint_indices=[0]); g=StateHistory()
        a,b=packet(10,3.1),packet(20,-3.1)
        for history in (h,g): history.add(a); history.add(b)
        self.assertAlmostEqual(h.interpolate(15)[0][0],np.pi)
        self.assertAlmostEqual(g.interpolate(15)[0][0],0)
        with self.assertRaises(ValueError): StateHistory(continuous_joint_indices=[5])

    def test_affine_clock_mapping_precision_expiry_and_session(self):
        c=ClockMapping(10**18,5*10**17,1.00001,100,10**18,10**18+10**9,'calibration','bootA')
        self.assertEqual(c.map(10**18+100_000,'bootA'),5*10**17+100_001)
        for stamp,session in ((10**18-1,'bootA'),(10**18,'bootB')):
            with self.assertRaises(ValueError): c.map(stamp,session)

    def test_frame_time_does_not_mislabel_receipt_as_exposure(self):
        f=CameraFrame('head',1,100,200,b'')
        self.assertEqual(frame_time(f),200)
        self.assertEqual(frame_time(replace(f,measurement_host_ns=80)),80)


@unittest.skip('Legacy simulation runtime removed; real transport/recording covered by test_rf_real_acquisition.')
class FlowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scratch=ROOT.parents[1]/'.document_qa'/'timing_validation'/uuid.uuid4().hex
        cls.scratch.mkdir(parents=True)

    def setUp(self):
        # A fault in one timing-sensitive integration case must not leak its
        # collector phase/camera threads into unrelated provenance checks.
        config=json.loads((ROOT/'config.json').read_text(encoding='utf-8'))
        config.update(mode='mock',camera_fps=120,width=160,height=120,
                      data_dir='data',minimum_free_mb=1)
        self.collector=Collector(config,self.scratch/self._testMethodName)
        time.sleep(.2)

    def tearDown(self):
        self.collector.close()

    def test_record_export_and_tamper_rejection(self):
        app=self.collector
        ep=app.start('timestamp alignment validation','automated',fps=50)['episode_id']
        time.sleep(1.3)
        if app.phase=='RECORDING': app.stop()
        self.assertEqual(app.phase,'REVIEW',app.last_error)
        self.assertGreater(app.frames,40)
        app.review(ep,True,True)
        path=app.episode_path(ep)
        manifest,rows,quality=check_raw(path)
        self.assertEqual(manifest['capture_version'],5)
        self.assertGreater(manifest['feedback_count'],len(rows))
        targets=[r['target_ns'] for r in rows]
        self.assertTrue(all(b-a==20_000_000 for a,b in zip(targets,targets[1:])))
        for key in CAMERAS:
            self.assertEqual(len({r['images'][key]['frame_id'] for r in rows}),len(rows))
            self.assertTrue(all(r['images'][key]['alignment_offset_ms']<=0 for r in rows))
        self.assertFalse(quality['timing']['physical_sync_verified'])
        self.assertTrue(quality['timing']['verified'])
        exported=app.export([ep])
        sample=training_sample(exported['path'],horizon=50)
        self.assertEqual(sample['actions'].shape,(50,16))
        self.assertEqual(set(sample['images']),set(CAMERAS))
        from training.pi05_adapter import TabletopInputs,TabletopOutputs
        mapped=TabletopInputs()(sample)
        self.assertFalse(bool(mapped['image_mask']['right_wrist_0_rgb']))
        restored=TabletopOutputs()({'actions':np.pad(mapped['actions'],((0,0),(0,16)))})['actions']
        np.testing.assert_allclose(restored,sample['actions'],atol=1e-6)
        sidecar=Path(exported['path'])/'timing'/'episode_000000.jsonl'
        self.assertTrue(sidecar.exists())
        self.assertTrue(sidecar.with_name('episode_000000_feedback.jsonl').exists())
        # Tamper only with this test's fresh scratch capture, then restore it.
        original=(path/'samples.jsonl').read_text(encoding='utf-8')
        for change in ('state','grid','image','source'):
            altered=copy.deepcopy(rows)
            if change=='state': altered[0]['observation.state'][0]+=.1
            if change=='grid': altered[1]['target_ns']+=1000
            if change=='image': altered[0]['images']['head']['alignment_offset_ms']=999
            if change=='source': altered[0]['alignment']['joints']['before_packet_id']=-1
            try:
                (path/'samples.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in altered),encoding='utf-8')
                with self.assertRaises(ValueError): check_raw(path)
            finally:
                (path/'samples.jsonl').write_text(original,encoding='utf-8')
        result=dict(passed=True,frames=len(rows),feedback_count=manifest['feedback_count'],
                    export=exported,quality=quality,hardware_verified=False)
        (self.scratch/'report.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
        print('TIMING_REPORT='+str(self.scratch/'report.json'))

    def test_camera_disconnect_stops_without_repeating_images(self):
        app=self.collector
        ep=app.start('camera fault validation','automated')['episode_id']
        time.sleep(.2)
        app.cameras.set_fault('left_wrist',True)
        try:
            limit=time.perf_counter()+3
            while app.phase=='RECORDING' and time.perf_counter()<limit: time.sleep(.02)
            self.assertEqual(app.phase,'FAULT',app.last_error)
            with self.assertRaises(ValueError): app.review(ep,True,True)
            app.review(ep,False,False)
        finally:
            app.cameras.set_fault('left_wrist',False)
            time.sleep(.1)

    def test_feedback_fault_cannot_be_interpolated_through(self):
        app=self.collector
        ep=app.start('feedback fault validation','automated')['episode_id']
        time.sleep(.2)
        app.simulator.feedback_fault=True
        try:
            limit=time.perf_counter()+3
            while app.phase=='RECORDING' and time.perf_counter()<limit: time.sleep(.02)
            self.assertEqual(app.phase,'FAULT',app.last_error)
            self.assertIn('Invalid feedback',app.last_error)
            with self.assertRaises(ValueError): app.review(ep,True,True)
            app.review(ep,False,False)
        finally:
            app.simulator.feedback_fault=False

    def test_legacy_and_aligned_captures_must_export_separately(self):
        # First validate old-reader compatibility on an isolated copy of a new test episode.
        import shutil
        app=self.collector
        # This case creates its own tiny capture so it is independent of test ordering.
        ep=app.start('legacy compatibility fixture','automated')['episode_id']
        time.sleep(.2)
        if app.phase=='RECORDING': app.stop()
        self.assertEqual(app.phase,'REVIEW',app.last_error)
        app.review(ep,True,True)
        original=app.episode_path(ep)
        legacy=self.scratch/'legacy_fixture'
        shutil.copytree(original,legacy)
        m=json.loads((legacy/'manifest.json').read_text(encoding='utf-8'))
        m['capture_version']=3; m.pop('timing_version',None)
        (legacy/'manifest.json').write_text(json.dumps(m))
        check_raw(legacy)
        from collector.exporter import export_dataset
        with self.assertRaisesRegex(ValueError,'legacy'):
            export_dataset([legacy,original],self.scratch/'unused_export')

    def test_training_still_refused(self):
        with self.assertRaises(ValueError):
            self.collector.start('not real','automated',purpose='training')


if __name__=='__main__':
    result=unittest.main(verbosity=2,exit=False).result
    report=dict(passed=result.wasSuccessful(),tests_run=result.testsRun,
                failures=len(result.failures),errors=len(result.errors),
                hardware_verified=False,simulation_report=str(FlowTests.scratch/'report.json'))
    (ROOT.parents[1]/'.document_qa'/'time_alignment_validation.json').write_text(
        json.dumps(report,indent=2),encoding='utf-8')
    sys.exit(0 if result.wasSuccessful() else 1)
