"""Human subtask boundaries, stored separately from immutable observations."""
import copy
import re
import threading
import time
import uuid

from .storage import atomic_json, load_json


class SegmentAnnotations:
    def __init__(self, app):
        self.app = app
        self.root = app.data / 'annotations'
        self.lock = threading.RLock()

    def context(self, episode_id):
        if not isinstance(episode_id, str) or not re.fullmatch(r'[0-9]{8}_[0-9]{6}_[0-9a-f]{8}', episode_id):
            raise ValueError('请选择有效片段。')
        with self.app.lock:
            writer = self.app.writer
            if writer and writer.episode_id == episode_id and self.app.phase == 'RECORDING':
                return dict(count=self.app.frames, live=True, stopping=self.app.stop_event.is_set(),
                            manifest=copy.deepcopy(writer.manifest))
            path = self.app.episode_path(episode_id)
            manifest = load_json(path / 'manifest.json')
            return dict(count=manifest['frame_count'], live=False, stopping=False, manifest=manifest)

    def load(self, episode_id):
        path = self.root / (episode_id + '.json')
        if path.exists():
            return load_json(path)
        return dict(schema='subtask_segments_v1', episode_id=episode_id, revision='0',
                    marks=[dict(id='start', frame=0, label='', source='episode_start')])

    def view(self, doc, context):
        result = copy.deepcopy(doc)
        count = context['count']; manifest = context['manifest']; fps = manifest['fps']
        marks = result['marks']
        result.update(frame_count=count, fps=fps, recording=context['live'],
                      task=manifest.get('task'), task_id=manifest.get('task_id'),
                      episode_status=manifest.get('status'), training_ready=manifest.get('training_ready', False),
                      frame_interval='[start_frame, end_frame_exclusive)',
                      time_reference='sample_index / fps; not wall time or exposure time',
                      samples_path='raw/' + doc['episode_id'] + '/samples.jsonl',
                      annotation_path='annotations/' + doc['episode_id'] + '.json')
        segments = []
        for i, mark in enumerate(marks):
            start = mark['frame']; end = min(count, marks[i+1]['frame'] if i+1 < len(marks) else count)
            mark['pending'] = start >= count
            if end > start:
                segments.append(dict(id=mark['id'], label=mark['label'], start_frame=start,
                                     end_frame_exclusive=end, frame_count=end-start,
                                     start_seconds=start/fps, end_seconds=end/fps))
        result['segments'] = segments
        return result

    def read(self, episode_id):
        with self.lock:
            context = self.context(episode_id)
            return self.view(self.load(episode_id), context)

    def mutate(self, body):
        with self.lock:
            episode_id = body.get('episode_id'); context = self.context(episode_id)
            if context['stopping']:
                raise ValueError('正在结束录制，请落盘后再标注。')
            doc = self.load(episode_id)
            if body.get('revision') != doc['revision']:
                raise ValueError('标注已更新，请刷新分段后重试。')
            operation = body.get('operation')
            label = body.get('label', '')
            if not isinstance(label, str) or len(label) > 160:
                raise ValueError('子任务名称最多160字。')
            label = label.strip()
            if operation == 'mark':
                if len(doc['marks']) >= 2000:
                    raise ValueError('单个片段最多2000个分段。')
                if context['live']:
                    if body.get('frame') is not None:
                        raise ValueError('录制中分界帧由服务器确定。')
                    frame = context['count']
                    source = 'next_unsubmitted_sample'
                else:
                    frame = body.get('frame'); source = 'replay_frame'
                    if type(frame) is not int or not 0 <= frame < context['count']:
                        raise ValueError('分界帧超出已保存片段。')
                if any(m['frame'] == frame for m in doc['marks']):
                    raise ValueError('当前帧已有分界，可直接修改该段名称。')
                doc['marks'].append(dict(id=uuid.uuid4().hex[:12], frame=frame, label=label, source=source,
                                         marked_at=time.strftime('%Y-%m-%dT%H:%M:%S%z')))
                doc['marks'].sort(key=lambda m: m['frame'])
            elif operation in ('rename', 'delete'):
                mark = next((m for m in doc['marks'] if m['id'] == body.get('mark_id')), None)
                if not mark:
                    raise ValueError('分段不存在，请刷新。')
                if operation == 'rename':
                    mark['label'] = label
                else:
                    if mark['id'] == 'start':
                        raise ValueError('首段不能删除；移除其他分界会合并到前段。')
                    doc['marks'].remove(mark)
            else:
                raise ValueError('未知标注操作。')
            doc['revision'] = uuid.uuid4().hex
            doc['updated_at'] = time.strftime('%Y-%m-%dT%H:%M:%S%z')
            atomic_json(self.root / (episode_id + '.json'), doc)
            return self.view(doc, context)
