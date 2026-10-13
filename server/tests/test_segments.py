"""Subtask sidecars never alter samples, review states or robot hardware."""
import json
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest

from collector.engine import Collector
from collector.segments import SegmentAnnotations
from collector.storage import atomic_json


class SegmentTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); root = Path(self.tmp.name)
        self.ep = '20261013_120000_1234abcd'; self.path = root / 'raw' / self.ep
        self.path.mkdir(parents=True)
        self.manifest = dict(episode_id=self.ep, frame_count=100, fps=50, status='REVIEW', task='pick', training_ready=False)
        atomic_json(self.path / 'manifest.json', self.manifest)
        (self.path / 'samples.jsonl').write_text('immutable observations', encoding='utf-8')
        self.app = Collector.__new__(Collector)
        self.app.data=root;self.app.raw=root/'raw';self.app.lock=threading.RLock()
        self.app.phase='REVIEW';self.app.writer=None;self.app.thread=None;self.app.last_episode=self.ep
        self.app.stop_event=threading.Event();self.app.frames=100
        self.annotations=SegmentAnnotations(self.app)

    def tearDown(self):self.tmp.cleanup()

    def change(self, operation, **extra):
        return self.annotations.mutate(dict(episode_id=self.ep, revision=self.annotations.read(self.ep)['revision'], operation=operation, **extra))

    def test_ranges_cover_samples_and_survive_restart_without_raw_edits(self):
        before=(self.path/'manifest.json').read_bytes()
        self.change('rename',mark_id='start',label='接近')
        doc=self.change('mark',frame=30,label='抓取')
        self.change('mark',frame=80,label='放置')
        doc=SegmentAnnotations(self.app).read(self.ep)
        self.assertEqual([(s['start_frame'],s['end_frame_exclusive']) for s in doc['segments']],[(0,30),(30,80),(80,100)])
        self.assertEqual(sum(s['frame_count'] for s in doc['segments']),100)
        self.assertEqual(doc['segments'][1]['start_seconds'],.6)
        self.assertEqual((self.path/'manifest.json').read_bytes(),before)
        self.assertEqual((self.path/'samples.jsonl').read_text(),'immutable observations')
        self.assertFalse(doc['training_ready'])

    def test_live_boundary_uses_submitted_not_written_count_and_pending_is_not_exported_as_empty_segment(self):
        self.app.phase='RECORDING';self.app.frames=27
        self.app.writer=SimpleNamespace(episode_id=self.ep,manifest=self.manifest,written=10)
        doc=self.change('mark',label='下一段')
        self.assertEqual(doc['marks'][-1]['frame'],27)
        self.assertTrue(doc['marks'][-1]['pending'])
        self.assertEqual(len(doc['segments']),1)
        self.app.frames=30
        self.assertEqual(self.annotations.read(self.ep)['segments'][-1]['frame_count'],3)
        self.app.stop_event.set()
        with self.assertRaises(ValueError):self.change('mark')

    def test_invalid_boundaries_and_optimistic_conflicts(self):
        for frame in [-1,0,100,True,1.5,'2']:
            with self.assertRaises(ValueError):self.change('mark',frame=frame)
        self.change('mark',frame=20)
        with self.assertRaises(ValueError):self.change('mark',frame=20)
        with self.assertRaises(ValueError):self.annotations.mutate(dict(episode_id=self.ep,revision='0',operation='rename',mark_id='start',label='stale'))
        with self.assertRaises(ValueError):self.annotations.read('../outside')
        with self.assertRaises(ValueError):self.change('rename',mark_id='start',label='a'*161)

    def test_deleting_boundary_merges_without_deleting_frames(self):
        doc=self.change('mark',frame=40,label='抓取')
        doc=self.change('delete',mark_id=doc['marks'][-1]['id'])
        self.assertEqual(len(doc['segments']),1);self.assertEqual(doc['segments'][0]['frame_count'],100)
        with self.assertRaises(ValueError):self.change('delete',mark_id='start')

    def test_prepare_next_preserves_pending_review_and_rejects_wrong_or_busy_episode(self):
        before=(self.path/'manifest.json').read_bytes()
        with self.assertRaises(ValueError):self.app.prepare_next('wrong')
        self.assertEqual(self.app.prepare_next(self.ep)['phase'],'IDLE')
        self.assertEqual((self.path/'manifest.json').read_bytes(),before)
        for phase in ['IDLE','RECORDING','FAULT','EXPORTING']:
            self.app.phase=phase
            with self.assertRaises(ValueError):self.app.prepare_next(self.ep)

    def test_fault_tail_marks_remain_pending_instead_of_inventing_samples(self):
        self.app.phase='RECORDING';self.app.frames=110
        self.app.writer=SimpleNamespace(episode_id=self.ep,manifest=self.manifest)
        self.change('mark',label='未落盘尾段')
        self.app.phase='FAULT'
        doc=self.annotations.read(self.ep)
        self.assertTrue(doc['marks'][-1]['pending'])
        self.assertEqual(sum(s['frame_count'] for s in doc['segments']),100)


if __name__=='__main__':unittest.main()
