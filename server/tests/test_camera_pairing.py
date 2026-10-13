from dataclasses import replace
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from collector.adapters import CameraFrame
from collector.camera_diagnostics import freshest_valid_pair, ready_frame_candidates


class PairingTests(unittest.TestCase):
    def setUp(self):
        self.target=100_000_000
        self.config={'camera_max_age_ms':60,'camera_alignment_max_offset_ms':30,
                     'camera_pair_max_skew_ms':20}
        self.previous={'head':10,'left_wrist':20}
        self.older=CameraFrame('head',11,81_000_000,81_000_000,b'a',sdk_arrival_host_ns=80_000_000)
        self.latest=CameraFrame('head',12,104_000_000,104_000_000,b'b',sdk_arrival_host_ns=98_000_000)
        self.wrist=CameraFrame('left_wrist',21,83_000_000,83_000_000,b'c',sdk_arrival_host_ns=83_000_000)

    def select(self,head=None,wrist=None,previous=None,observed=105_000_000):
        return freshest_valid_pair({'head':head or [self.latest,self.older],
                                    'left_wrist':wrist or [self.wrist]},
                                   self.target,observed,previous or self.previous,self.config)

    def test_recorded_failure_has_valid_older_unused_pair(self):
        selected=self.select()
        self.assertIs(selected['head'],self.older)
        self.assertIs(selected['left_wrist'],self.wrist)

    def test_prefers_latest_when_both_meet_limits(self):
        wrist=replace(self.wrist,received_ns=88_000_000)
        self.assertIs(self.select(wrist=[wrist])['head'],self.latest)

    def test_never_reuses_old_frame_to_hide_missing_pair(self):
        self.assertIsNone(self.select(previous={'head':11,'left_wrist':20}))

    def test_future_stale_and_overoffset_candidates_rejected(self):
        for frame in (replace(self.older,sdk_arrival_host_ns=101_000_000),
                      replace(self.older,sdk_arrival_host_ns=69_000_000),
                      replace(self.older,received_ns=40_000_000)):
            self.assertIsNone(self.select(head=[frame]))

    def test_candidate_snapshot_keeps_inflight_wait_and_excludes_future(self):
        future=replace(self.latest,sdk_arrival_host_ns=101_000_000)
        self.assertEqual(ready_frame_candidates([self.older,future],self.target),[self.older])
        self.assertEqual(ready_frame_candidates([self.older],self.target,(12,99_000_000)),[])

    def test_reserve_policy_survives_recorded_arrival_jitter_without_reuse(self):
        self.config['camera_pairing_policy']='oldest_valid_causal_pair_v2'
        def frame(key,ident,stamp):
            return CameraFrame(key,ident,stamp,stamp,b'jpeg',sdk_arrival_host_ns=stamp)
        candidates={key:[frame(key,2,99_500_000),frame(key,1,83_000_000)]
                    for key in ('head','left_wrist')}
        first=freshest_valid_pair(candidates,100_000_000,105_000_000,
                                 {'head':0,'left_wrist':0},self.config)
        self.assertEqual([f.frame_id for f in first.values()],[1,1])
        # The next raw frame arrives after the next target, so it cannot be used.
        for key in candidates: candidates[key].insert(0,frame(key,3,121_000_000))
        second=freshest_valid_pair(candidates,120_000_000,125_000_000,
                                  {key:f.frame_id for key,f in first.items()},self.config)
        self.assertEqual([f.frame_id for f in second.values()],[2,2])
        for f in second.values(): self.assertLessEqual(120_000_000-f.sdk_arrival_host_ns,30_000_000)


if __name__=='__main__': unittest.main()
