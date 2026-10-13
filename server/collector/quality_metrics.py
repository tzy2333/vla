"""Round-based quality gates and evidence-bearing external pipeline reports."""
import copy
import datetime
import math
import re
import threading
import uuid

from .storage import atomic_json, load_json


DEFAULT_THRESHOLDS = dict(completion_rate=95., postprocess_success_rate=95., retention_rate=90., task_success_rate=90.)
STAGES = ('upload', 'postprocess', 'screening')
STATES = ('running', 'passed', 'failed', 'data_error')


class QualityMetrics:
    def __init__(self, directory):
        self.directory = directory
        self.threshold_file = directory / 'quality_thresholds.json'
        self.lock = threading.RLock()

    def thresholds(self):
        with self.lock:
            return load_json(self.threshold_file) if self.threshold_file.exists() else dict(
                revision='0', minimum_rounds=20, rates=DEFAULT_THRESHOLDS.copy())

    def save_thresholds(self, body):
        with self.lock:
            if set(body) != {'revision', 'minimum_rounds', 'rates'}:
                raise ValueError('阈值字段不完整或包含未知字段。')
            if body['revision'] != self.thresholds()['revision']:
                raise ValueError('阈值已被修改，请刷新统计后重试。')
            rates = body['rates']; minimum = body['minimum_rounds']
            if not isinstance(rates, dict) or set(rates) != set(DEFAULT_THRESHOLDS):
                raise ValueError('成功率阈值字段无效。')
            if any(type(v) not in (int, float) or not math.isfinite(v) or not 0 <= v <= 100 for v in rates.values()):
                raise ValueError('成功率阈值须为0至100之间的有限数值。')
            if type(minimum) is not int or not 1 <= minimum <= 1000000:
                raise ValueError('最少有效轮次须为1至1000000的整数。')
            result = dict(revision=uuid.uuid4().hex, minimum_rounds=minimum, rates=copy.deepcopy(rates))
            atomic_json(self.threshold_file, result)
            return result

    def report_path(self, episode_id):
        if not isinstance(episode_id, str) or not re.fullmatch(r'[0-9]{8}_[0-9]{6}_[0-9a-f]{8}', episode_id):
            raise ValueError('录制轮次编号无效。')
        return self.directory / 'pipeline_reports' / (episode_id + '.json')

    def report(self, episode_id):
        with self.lock:
            path = self.report_path(episode_id)
            return load_json(path) if path.exists() else dict(episode_id=episode_id, revision='0', stages={}, history=[])

    def save_report(self, body):
        with self.lock:
            if set(body) != {'episode_id', 'revision', 'stage', 'status', 'run_id', 'source', 'evidence'}:
                raise ValueError('处理结果字段不完整或包含未知字段。')
            if body['stage'] not in STAGES or body['status'] not in STATES:
                raise ValueError('未知处理阶段或状态。')
            for key, limit in [('run_id', 100), ('source', 100), ('evidence', 500)]:
                value = body[key]
                if not isinstance(value, str) or not value.strip() or len(value) > limit:
                    raise ValueError('结果需提供有效的执行编号、来源及证据引用。')
            doc = self.report(body['episode_id'])
            if doc['revision'] != body['revision']:
                raise ValueError('处理结果已更新，请重新读取版本后提交。')
            event = {k: body[k] for k in ('stage', 'status', 'run_id', 'source', 'evidence')}
            old = doc['stages'].get(body['stage'])
            if old and all(old[k] == event[k] for k in event):
                return doc
            if old and old['run_id'] == body['run_id'] and old['status'] != 'running':
                raise ValueError('该次处理已结束；重试需使用新的执行编号。')
            prior_runs = {e['run_id'] for e in doc['history'] if e['stage'] == body['stage']}
            if body['run_id'] in prior_runs and old and old['run_id'] != body['run_id']:
                raise ValueError('不能用旧的执行结果覆盖新一轮处理。')
            event['received_at'] = datetime.datetime.now(datetime.timezone.utc).isoformat()
            doc['stages'][body['stage']] = event
            doc['history'].append(event.copy())
            doc['revision'] = uuid.uuid4().hex
            atomic_json(self.report_path(body['episode_id']), doc)
            return doc

    def snapshot(self, episodes, summary):
        with self.lock:
            thresholds = self.thresholds()
            pipeline = {stage: dict(connected=False, reported=0, unreported=len(episodes), **{s: 0 for s in STATES}) for stage in STAGES}
            for ep in episodes:
                for stage, result in self.report(ep['episode_id'])['stages'].items():
                    counts = pipeline[stage]
                    counts['connected'] = True
                    counts['reported'] += 1
                    counts['unreported'] -= 1
                    counts[result['status']] += 1
            pp = pipeline['postprocess']
            fractions = dict(completion_rate=(summary['completed'], summary['finalized'], True),
                             postprocess_success_rate=(pp['passed'], pp['passed']+pp['failed']+pp['data_error'], pp['connected']),
                             retention_rate=(summary['kept'], summary['reviewed'], True),
                             task_success_rate=(summary['successes'], summary['labeled'], True))
            rates = {}
            for name, (numerator, denominator, available) in fractions.items():
                value = 100*numerator/denominator if denominator else None
                state = ('unavailable' if not available else 'no_data' if not denominator else
                         'insufficient' if denominator < thresholds['minimum_rounds'] else
                         'pass' if value >= thresholds['rates'][name] else 'below')
                rates[name] = dict(value=value, numerator=numerator, denominator=denominator,
                                   threshold=thresholds['rates'][name], minimum_rounds=thresholds['minimum_rounds'], state=state)
            return dict(thresholds=thresholds, rates=rates, pipeline=pipeline)
