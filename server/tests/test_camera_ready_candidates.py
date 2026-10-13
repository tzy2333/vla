import unittest
from types import SimpleNamespace

from collector.camera_diagnostics import ready_frame_candidates, freshest_valid_pair


def frame(ident, received, aligned):
    return SimpleNamespace(frame_id=ident, received_ns=received,
                           sdk_arrival_host_ns=aligned, measurement_host_ns=None)


class ReadyCandidatesTests(unittest.TestCase):
    def test_pending_encoding_preserves_actual_failure_ready_pair(self):
        # Reproduce the 848x480 acceptance failure with its actual timestamps.
        target=5254569851372
        observed=5254584938395
        head=frame(3774,5254569102019,5254568867819)
        wrist=frame(3753,5254552789947,5254552427563)
        candidates={'head':ready_frame_candidates([head],target),
                    'left_wrist':ready_frame_candidates([wrist],target,(3754,target-1000))}
        cfg={'camera_max_age_ms':60,'camera_alignment_max_offset_ms':30,
             'camera_pair_max_skew_ms':20,'camera_pairing_policy':'oldest_valid_causal_pair_v2'}
        previous={'head':3772,'left_wrist':3752}
        selected=freshest_valid_pair(candidates,target,observed,previous,cfg)
        self.assertIsNotNone(selected)
        self.assertIs(selected['left_wrist'],wrist)
        # Ready candidates never relax the no-reuse or pair-skew requirements.
        self.assertIsNone(freshest_valid_pair(candidates,target,observed,{'head':3774,'left_wrist':3753},cfg))
        cfg['camera_pair_max_skew_ms']=10
        self.assertIsNone(freshest_valid_pair(candidates,target,observed,previous,cfg))

    def test_future_and_unfinished_frames_are_not_candidates(self):
        ready=frame(1,90,90);future=frame(2,110,110)
        self.assertEqual(ready_frame_candidates([ready,future],100,(3,99)),[ready])
        self.assertEqual(ready_frame_candidates([],100,(3,99)),[])

    def test_old_frame_still_rejected(self):
        old=frame(1,1,1)
        candidates={'head':[old],'left_wrist':[old]}
        cfg={'camera_max_age_ms':60,'camera_alignment_max_offset_ms':30,'camera_pair_max_skew_ms':20}
        self.assertIsNone(freshest_valid_pair(candidates,40_000_000,40_000_001,{'head':0,'left_wrist':0},cfg))


if __name__=='__main__':unittest.main()
