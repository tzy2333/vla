import sys
from pathlib import Path
import unittest
import threading
from collections import deque

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from collector.adapters import CameraFrame
from collector.camera_diagnostics import CameraDiagnostics, AlignmentDiagnostics, optional_metadata, latest_ready_frame
from collector.clock import monotonic_ns, monotonic
from collector.engine import Collector


class DiagnosticsTests(unittest.TestCase):
    def frame(self,index,receipt):
        return CameraFrame('head',index,receipt,receipt,b'',device_timestamp_ms=receipt/1e6,
            ready_ns=receipt+2_000_000,timing_metadata={'values':{'EXPOSURE':1000},'unavailable':{}})

    def test_native_rate_processing_and_skips(self):
        d=CameraDiagnostics()
        for i,index in enumerate([1,2,4]):d.record(self.frame(index,1_000_000_000+i*20_000_000))
        result=d.snapshot(1_045_000_000)
        self.assertEqual(result['received_fps'],50)
        self.assertEqual(result['frame_id_gaps'],1)
        self.assertEqual(result['processing_ms']['p95'],2)
        self.assertEqual(result['metadata_present_counts']['EXPOSURE'],3)
        self.assertFalse(result['physical_sync_verified'])

    def test_bounded_and_stale_window(self):
        d=CameraDiagnostics(capacity=3,window_seconds=1)
        for i in range(6):d.record(self.frame(i,1_000_000_000+i*20_000_000))
        self.assertEqual(d.snapshot(1_120_000_000)['sample_count'],3)
        self.assertEqual(d.snapshot(3_000_000_000)['sample_count'],0)
        self.assertEqual(d.snapshot(3_000_000_000)['total_frames_observed'],6)

    def test_optional_metadata_failure_is_explicit(self):
        def read(name):
            if name=='error':raise RuntimeError('SDK metadata unavailable')
            return float('nan') if name=='invalid' else 123
        r=optional_metadata(['valid','missing','error','invalid'],lambda name:name!='missing',read)
        self.assertEqual(r['values'],{'valid':123})
        self.assertEqual(set(r['unavailable']),{'missing','error','invalid'})

    def test_sessions_change_on_camera_reopen(self):
        self.assertNotEqual(CameraDiagnostics().session_id,CameraDiagnostics().session_id)

    def test_alignment_bounds_and_reset(self):
        d=AlignmentDiagnostics(capacity=2)
        for i in [1,2,3]:
            d.add({'images':{'head':{'alignment_offset_ms':-i}},
                   'quality':{'camera_pair_skew_ms':i,'assembly_lateness_ms':i,'command_send_delay_ms':i},
                   'alignment':{'joints':{'gap_ms':10}}})
        r=d.snapshot()
        self.assertEqual(r['sample_count'],2)
        self.assertEqual(r['metrics_ms']['head']['p50'],2.5)
        self.assertEqual(AlignmentDiagnostics().snapshot()['sample_count'],0)

    def test_causal_inflight_frame_defers_older_ready_frame(self):
        old=self.frame(10,80_000_000);new=self.frame(11,98_000_000)
        frames=deque([old])
        self.assertIsNone(latest_ready_frame(frames,100_000_000,(11,98_000_000)))
        self.assertIs(latest_ready_frame(frames,None,(11,98_000_000)),old)
        frames.append(new)
        self.assertIs(latest_ready_frame(frames,100_000_000,None),new)

    def test_future_frame_never_replaces_causal_frame(self):
        old=self.frame(10,98_000_000);future=self.frame(11,102_000_000)
        self.assertIs(latest_ready_frame([old],100_000_000,(11,102_000_000)),old)
        self.assertIs(latest_ready_frame([old,future],100_000_000),old)

    def test_sampler_waits_for_inflight_without_moving_target(self):
        app=Collector.__new__(Collector);app.frames=0;app.started_ns=monotonic_ns();app.stop_event=threading.Event()
        app.config={'fps':50,'camera_wait_budget_ms':15,'camera_max_age_ms':60,
                    'camera_alignment_max_offset_ms':30,'camera_pair_max_skew_ms':20}
        old=self.frame(2,app.started_ns-19_000_000);new=self.frame(3,app.started_ns-2_000_000)
        target=app.started_ns
        class Cameras:
            calls=0
            def latest(self,key,at_ns):
                assert at_ns==target
                self.calls+=1
                if self.calls<=2:return latest_ready_frame([old],at_ns,(3,new.received_ns))
                return latest_ready_frame([old,new],at_ns)
        app.cameras=Cameras()
        _,frames,_=app.capture_group({'head':1,'left_wrist':1},monotonic())
        self.assertEqual([f.frame_id for f in frames.values()],[3,3])
        self.assertEqual(app.started_ns,target)

    def test_inflight_frame_does_not_remove_wait_deadline(self):
        app=Collector.__new__(Collector);app.frames=0;app.started_ns=monotonic_ns();app.stop_event=threading.Event()
        app.config={'fps':50,'camera_wait_budget_ms':1,'camera_max_age_ms':60,
                    'camera_alignment_max_offset_ms':30,'camera_pair_max_skew_ms':20}
        class Cameras:
            def latest(self,key,at_ns):return None
        app.cameras=Cameras()
        with self.assertRaises(RuntimeError):app.capture_group({'head':1,'left_wrist':1},monotonic()-.1)

if __name__=='__main__':unittest.main()
