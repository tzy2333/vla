"""Isolated fixtures only: no device, movement or production review writes."""
import copy
import json
from pathlib import Path
import tempfile
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import patch
from PIL import Image
from collector.storage import atomic_json
from collector.quality_inspection import DEFAULTS,inspect_episode
from collector.review_workflow import ReviewWorkflow

ID='20261013_180000_1234abcd'

class ReviewTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name);self.raw=self.root/'raw'/ID;self.raw.mkdir(parents=True)
        self.manifest=dict(episode_id=ID,status='REVIEW',frame_count=8,fps=30,mock=False,task='isolated fixture',operator='test',
            schema_version='modular_robot_observation_v1',robot_adapter_id='test_fixture',camera_keys=['head'],camera_shapes={'head':[16,16,3]},
            device_sources={'robot':'test_fixture','head':'test_fixture'},state_channels=[dict(id='q1',index=0)],training_ready=False,fault=None)
        atomic_json(self.raw/'manifest.json',self.manifest);(self.raw/'cameras/head').mkdir(parents=True)
        self.rows=[]
        for i in range(8):
            Image.new('RGB',(16,16),(i*10,20,30)).save(self.raw/f'cameras/head/{i:06d}.jpg')
            self.rows.append(dict(frame_index=i,timestamp=i/30,sample_ns=1000000000+round(i*1e9/30),
                **{'observation.state':[i/100],'action':None,'state_valid_mask':[True]},
                images={'head':dict(path=f'cameras/head/{i:06d}.jpg',frame_id=i,received_ns=1000000000+round(i*1e9/30),alignment_offset_ms=1)}))
        self.write_rows()
        def path(ident):
            if ident!=ID:raise ValueError('bad id')
            return self.raw
        self.app=SimpleNamespace(data=self.root,raw=self.root/'raw',exports=self.root/'datasets',phase='IDLE',lock=threading.RLock(),episode_path=path,episodes=lambda:[copy.deepcopy(self.manifest)])
        self.workflow=ReviewWorkflow(self.app,False)
    def tearDown(self):self.workflow.close();self.temp.cleanup()
    def write_rows(self):(self.raw/'samples.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in self.rows),encoding='utf-8')
    def decide(self,stage='primary',decision='pass',reviewer='a',**extra):
        d=self.workflow.read(ID)
        return self.workflow.mutate(dict(episode_id=ID,revision=d['revision'],stage=stage,decision=decision,reviewer=reviewer,**extra))
    def test_pending_is_retained(self):
        before=(self.raw/'manifest.json').read_bytes();self.workflow.inspect(ID)
        self.assertEqual(self.workflow.snapshot()['summary']['pending'],1)
        self.assertEqual(before,(self.raw/'manifest.json').read_bytes())
    def test_primary_secondary_distinct_and_funnel(self):
        self.workflow.inspect(ID);self.decide(outcome='failure')
        with self.assertRaises(ValueError):self.decide('secondary')
        self.decide('secondary',reviewer='b',outcome='failure');s=self.workflow.snapshot()['summary']
        self.assertEqual((s['primary_passed'],s['secondary_passed'],s['final_delivery'],s['pending']),(1,1,1,0))
        self.assertEqual(s['task_success_rate'],0)
    def test_stale_revision_rejected(self):
        self.workflow.inspect(ID)
        with self.assertRaises(ValueError):self.workflow.mutate(dict(episode_id=ID,revision='0',stage='primary',decision='pass',reviewer='a'))
    def test_no_pass_without_l1(self):
        with self.assertRaises(ValueError):self.decide()
    def test_reject_reason_required(self):
        with self.assertRaises(ValueError):self.decide(decision='reject')
        self.decide(decision='reject',reason='bad annotation');self.assertEqual(self.workflow.snapshot()['summary']['rejected'],1)
    def test_rework_resubmit_history(self):
        self.decide(decision='rework',reason='add annotation');self.decide('resubmit',reason='fixed')
        d=self.workflow.read(ID);self.assertEqual(d['cycle'],1);self.assertEqual(len(d['history']),2);self.assertEqual(d['state'],'pending_primary')
    def test_no_manufactured_kappa(self):
        self.workflow.inspect(ID);self.decide();self.decide('secondary',reviewer='b')
        self.assertIsNone(self.workflow.snapshot()['summary']['kappa'])
        self.decide('blind',reviewer='c',independent=True);s=self.workflow.snapshot()['summary']
        self.assertEqual(s['paired_count'],1);self.assertIsNone(s['kappa']);self.assertEqual(s['agreement_pct'],100)
    def test_blind_disagreement_requires_arbitration(self):
        self.workflow.inspect(ID);self.decide();self.decide('blind','reject',reviewer='b',reason='rubric',independent=True)
        self.assertEqual(self.workflow.read(ID)['state'],'disputed')
        with self.assertRaises(ValueError):self.decide('arbitration',reviewer='a',reason='check')
        self.decide('arbitration',reviewer='c',reason='third reviewer');self.assertEqual(self.workflow.read(ID)['state'],'approved')
    def test_busy_rejects_decisions(self):
        self.app.phase='RECORDING'
        with self.assertRaises(ValueError):self.decide(decision='reject',reason='bad')
    def test_threshold_change_invalidates_delivery(self):
        self.workflow.inspect(ID);self.decide();self.decide('secondary',reviewer='b')
        t=self.workflow.targets();self.workflow.save_targets(dict(revision=t['revision'],values=dict(t['values'],receipt_skew_ms=9)))
        self.assertEqual(self.workflow.snapshot()['summary']['final_delivery'],0)
        with self.assertRaises(ValueError):self.workflow.start_export(dict(episode_ids=[ID]))
    def test_corrupt_image_and_fault_critical(self):
        (self.raw/'cameras/head/000001.jpg').write_bytes(b'broken');r=inspect_episode(self.raw,DEFAULTS)
        self.assertEqual(r['score'],0);self.assertTrue(r['critical_failure']);self.assertEqual(r['status'],'failed')
    def test_critical_failure_veto_even_with_small_batch(self):
        (self.raw/'cameras/head/000001.jpg').write_bytes(b'broken');self.workflow.inspect(ID)
        self.assertEqual(self.workflow.snapshot()['summary']['batch_status'],'不通过（致命项）')
    def test_missing_sensors_unavailable(self):
        r=inspect_episode(self.raw,DEFAULTS);self.assertEqual(r['status'],'passed')
        for k in ('jerk_rms','speed_cv','exposure_sync_ms','action'):self.assertIsNone(r['metrics'][k]['value'])
    def test_repeated_timestamp_rejected(self):
        self.rows[2]['sample_ns']=self.rows[1]['sample_ns'];self.write_rows()
        self.assertEqual(inspect_episode(self.raw,DEFAULTS)['status'],'failed')
    def test_path_traversal_rejected(self):
        self.rows[0]['images']['head']['path']='../../outside.jpg';self.write_rows()
        self.assertTrue(inspect_episode(self.raw,DEFAULTS)['critical_failure'])
    def test_no_training_export_without_actions(self):
        with self.assertRaises(ValueError):self.workflow.start_export(dict(episode_ids=[ID],kind='training'))
    def test_dynamic_observation_export(self):
        from collector.observation_export import export_observations
        import pyarrow.parquet as pq
        self.workflow.inspect(ID);self.decide();self.decide('secondary',reviewer='b')
        result=export_observations([self.raw],self.root/'exports',{ID:self.workflow.read(ID)},verify_official=False)
        root=Path(result['path']);info=json.loads((root/'meta/info.json').read_text())
        self.assertEqual(info['codebase_version'],'v2.1');self.assertFalse(info['training_ready']);self.assertNotIn('action',info['features'])
        table=pq.read_table(root/'data/chunk-000/episode_000000.parquet')
        self.assertEqual(table['next.done'].to_pylist(),[False]*7+[True]);self.assertEqual(table.num_rows,8)
        self.assertEqual(info['features']['observation.state']['shape'],[1])
        self.assertTrue((root/'meta/episodes_stats.jsonl').exists())
    def test_invalid_vector_export_rejected(self):
        from collector.observation_export import export_observations
        self.rows[0]['observation.state']=[None];self.write_rows()
        with self.assertRaises(ValueError):export_observations([self.raw],self.root/'exports',{},verify_official=False)
    def test_camera_only_export_does_not_invent_state(self):
        from collector.observation_export import export_observations
        self.manifest['state_channels']=[];atomic_json(self.raw/'manifest.json',self.manifest)
        for row in self.rows:row['observation.state']=[];row['state_valid_mask']=[]
        self.write_rows()
        result=export_observations([self.raw],self.root/'exports',{ID:self.workflow.read(ID)},verify_official=False)
        info=json.loads((Path(result['path'])/'meta/info.json').read_text())
        self.assertNotIn('observation.state',info['features']);self.assertNotIn('action',info['features'])
    def test_two_camera_export_uses_selected_keys(self):
        from collector.observation_export import export_observations
        self.manifest['camera_keys'].append('wrist');self.manifest['camera_shapes']['wrist']=[16,16,3]
        atomic_json(self.raw/'manifest.json',self.manifest)
        for row in self.rows:row['images']['wrist']=copy.deepcopy(row['images']['head'])
        self.write_rows()
        result=export_observations([self.raw],self.root/'exports',{ID:self.workflow.read(ID)},verify_official=False)
        info=json.loads((Path(result['path'])/'meta/info.json').read_text());self.assertEqual(info['total_videos'],2)
        self.assertIn('observation.images.wrist',info['features']);self.assertNotIn('observation.images.right_wrist',info['features'])
    def test_calibrated_constant_velocity_has_zero_jerk_and_cv(self):
        self.manifest.update(ee_position_calibrated=True,ee_position_unit='m');atomic_json(self.raw/'manifest.json',self.manifest)
        for row in self.rows:row['observation.ee_pose']=[row['timestamp'],0,0,1,0,0,0]
        self.write_rows();r=inspect_episode(self.raw,DEFAULTS)
        self.assertLess(r['metrics']['jerk_rms']['value'],1e-8);self.assertLess(r['metrics']['speed_cv']['value'],1e-8)
    def test_uncalibrated_pose_not_used_for_physical_jerk(self):
        for row in self.rows:row['observation.ee_pose']=[row['timestamp'],0,0,1,0,0,0]
        self.write_rows();r=inspect_episode(self.raw,DEFAULTS)
        self.assertIsNone(r['metrics']['jerk_rms']['value'])
    def test_legacy_review_does_not_become_secondary(self):
        self.manifest['status']='COMMITTED';atomic_json(self.raw/'manifest.json',self.manifest)
        s=self.workflow.snapshot()['summary'];self.assertEqual(s['pending_primary'],1);self.assertEqual(s['secondary_passed'],0)

if __name__=='__main__':unittest.main()
