"""Quality rates use round denominators and observed pipeline results only."""
import copy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

from collector.platform_management import PlatformManagement, summarize
from collector.quality_metrics import QualityMetrics


def episode(index, **fields):
    return dict(episode_id=f'20261013_120000_{index:08x}', mock=False,
                schema_version='modular_robot_observation_v1', robot_adapter_id='other_robot',
                device_sources={}, status='REVIEW', frame_count=100, fps=50, **fields)


class QualityTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        self.quality=QualityMetrics(self.root)
    def tearDown(self):self.tmp.cleanup()
    def report(self, ep, status, run='run1', stage='postprocess'):
        revision=self.quality.report(ep['episode_id'])['revision']
        return self.quality.save_report(dict(episode_id=ep['episode_id'],revision=revision,stage=stage,
                                            status=status,run_id=run,source='test-worker',evidence='report.json'))
    def snapshot(self, rows):return self.quality.snapshot(rows,summarize(rows))

    def test_defaults_persist_and_validate_revision_and_numeric_ranges(self):
        doc=self.quality.thresholds();self.assertEqual(doc['minimum_rounds'],20)
        self.assertEqual(doc['rates']['postprocess_success_rate'],95)
        saved=self.quality.save_thresholds(dict(doc,minimum_rounds=25))
        self.assertEqual(QualityMetrics(self.root).thresholds(),saved)
        with self.assertRaises(ValueError):self.quality.save_thresholds(doc)
        for bad in [True,float('nan'),float('inf'),-1,101,'95']:
            body=copy.deepcopy(saved);body['rates']['completion_rate']=bad
            with self.assertRaises(ValueError):self.quality.save_thresholds(body)
        for bad in [0,True,1.5,1000001]:
            with self.assertRaises(ValueError):self.quality.save_thresholds(dict(saved,minimum_rounds=bad))

    def test_no_results_are_not_success_or_failure(self):
        doc=self.snapshot([])
        self.assertEqual(doc['rates']['postprocess_success_rate']['state'],'unavailable')
        self.assertEqual(doc['rates']['completion_rate']['state'],'no_data')
        self.assertIsNone(doc['rates']['completion_rate']['value'])
        self.assertFalse(doc['pipeline']['upload']['connected'])

    def test_threshold_boundary_and_minimum_use_each_rate_denominator(self):
        rows=[episode(i) for i in range(20)]
        rows[-1].update(status='FAULT',fault='camera')
        rows[0].update(outcome_source='human_review_v1',task_outcome='success')
        rates=self.snapshot(rows)['rates']
        self.assertEqual(rates['completion_rate']['state'],'pass')
        self.assertEqual(rates['completion_rate']['value'],95)
        self.assertEqual(rates['task_success_rate']['state'],'insufficient')
        self.assertEqual(rates['task_success_rate']['denominator'],1)
        rows[-2].update(status='FAULT',fault='camera')
        self.assertEqual(self.snapshot(rows)['rates']['completion_rate']['state'],'below')
        self.assertEqual(self.snapshot(rows[:5])['rates']['completion_rate']['state'],'insufficient')

    def test_processing_retries_count_one_round_and_keep_audit_history(self):
        ep=episode(1);self.report(ep,'failed')
        self.report(ep,'running',run='retry')
        current=self.snapshot([ep]);self.assertIsNone(current['rates']['postprocess_success_rate']['value'])
        self.assertEqual(current['pipeline']['postprocess']['running'],1)
        self.report(ep,'passed',run='retry')
        current=self.snapshot([ep])
        self.assertEqual(current['rates']['postprocess_success_rate']['denominator'],1)
        self.assertEqual(current['pipeline']['postprocess']['failed'],0)
        self.assertEqual(len(self.quality.report(ep['episode_id'])['history']),3)
        same=self.report(ep,'passed',run='retry');self.assertEqual(len(same['history']),3)
        with self.assertRaises(ValueError):self.report(ep,'failed',run='run1')
        with self.assertRaises(ValueError):self.report(ep,'running',run='retry')

    def test_failures_data_errors_and_unreported_are_separate(self):
        rows=[episode(i) for i in range(5)]
        for ep,state in zip(rows,['passed','failed','data_error','running']):self.report(ep,state)
        q=self.snapshot(rows);rate=q['rates']['postprocess_success_rate']
        self.assertEqual(rate['numerator'],1);self.assertEqual(rate['denominator'],3)
        self.assertEqual(q['pipeline']['postprocess']['unreported'],1)
        self.assertIsNone(self.snapshot([rows[-1]])['rates']['postprocess_success_rate']['value'])

    def test_reports_require_provenance_and_stale_edits_do_not_overwrite(self):
        ep=episode(1);doc=self.report(ep,'running')
        body=dict(episode_id=ep['episode_id'],revision='0',stage='postprocess',status='passed',run_id='run1',source='test',evidence='proof')
        with self.assertRaises(ValueError):self.quality.save_report(body)
        body['revision']=doc['revision'];body['evidence']=''
        with self.assertRaises(ValueError):self.quality.save_report(body)
        with self.assertRaises(ValueError):self.quality.report('../../other')

    def test_dashboard_filters_rounds_and_does_not_count_subtask_markers(self):
        rows=[episode(1,task_id='a'),episode(2,task_id='b')]
        rows[1]['episode_id']='20261012_120000_00000002'
        app=SimpleNamespace(data=self.root,episodes=lambda:rows)
        manager=PlatformManagement(app,self.root,self.root/'config.json')
        self.quality=manager.quality;self.report(rows[0],'passed')
        marker=self.root/'annotations'/(rows[0]['episode_id']+'.json');marker.parent.mkdir()
        marker.write_text(json.dumps({'marks':[{}, {}, {}]}))
        result=manager.dashboard(day='2026-10-13',task_id='a')
        self.assertEqual(result['summary']['finalized'],1)
        self.assertEqual(result['quality']['rates']['postprocess_success_rate']['denominator'],1)
        self.assertEqual(manager.dashboard(task_id='b')['quality']['pipeline']['postprocess']['reported'],0)
        with self.assertRaises(ValueError):manager.pipeline_report('20261013_120000_ffffffff')


if __name__=='__main__':unittest.main()
