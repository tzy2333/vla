"""Bounded asynchronous raw writer, review and conservative crash recovery."""
import json
import copy
import os
from pathlib import Path
import queue
import threading
import time
from .clock import monotonic, monotonic_ns, DOMAIN, SESSION
import uuid

from .adapters import CAMERAS, SCHEMA


def atomic_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.tmp')
    with temporary.open('w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.flush()
        os.fsync(f.fileno())
    os.replace(temporary, path)


def load_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def event(path, kind, **extra):
    with (Path(path)/'events.jsonl').open('a', encoding='utf-8') as f:
        f.write(json.dumps({'event': kind, 'monotonic_ns': monotonic_ns(), **extra},
                           ensure_ascii=False, allow_nan=False) + '\n')
        f.flush()
        os.fsync(f.fileno())


class RawWriter:
    def __init__(self, raw_root, config, task, operator):
        config=copy.deepcopy(config)
        camera_keys=config.get('camera_keys',CAMERAS)
        self.episode_id = time.strftime('%Y%m%d_%H%M%S') + '_' + uuid.uuid4().hex[:8]
        self.path = Path(raw_root)/(self.episode_id+'.inprogress')
        self.path.mkdir(parents=True)
        for key in camera_keys:
            (self.path/'cameras'/key).mkdir(parents=True)
        self.manifest = {'episode_id': self.episode_id, 'status': 'RECORDING',
                         'schema_version': config.get('schema_version',SCHEMA), 'mock': False, 'base_enabled': False,
                         'capture_version':config.get('capture_version',6), 'training_ready':False,
                         'timing_version':config.get('timing_version',1),
                         'time_reference':'host_monotonic_ns_per_boot',
                         'host_clock_domain':DOMAIN, 'host_clock_session':SESSION,
                         'physical_sync_verified':False,
                         'feedback_capture':'real_udp_reply_packets_used_as_interpolation_sources',
                         'camera_keys':list(camera_keys),
                         'capture_selection':config.get('capture_selection'),
                         'state_channels':config.get('state_channels'),
                         'robot_descriptor':config.get('robot_descriptor'),
                         'robot_adapter_id':config.get('robot_adapter_id'),
                         'raw_feedback_scope':config.get('raw_feedback_scope','complete_packet'),
                         'capture_purpose':config.get('capture_purpose','test'),
                         'sampling_mode':config.get('sampling_mode','uniform_sampling'),
                         'sample_interval_s':1/config['fps'],
                         'fps': config['fps'], 'width': config['width'], 'height': config['height'],
                         'device_sources': config.get('device_sources',{'robot':'mock',**{k:'mock' for k in CAMERAS}}),
                         'camera_shapes': config.get('camera_shapes',{k:[config['height'],config['width'],3] for k in CAMERAS}),
                         'task': task, 'operator': operator, 'created_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
                         'task_id':(config.get('task_metadata') or {}).get('task_id'),
                         'task_metadata':config.get('task_metadata'),
                         'task_outcome':'unknown','outcome_source':None,
                         'frame_count': 0, 'fault': None, 'success': None,
                         'calibration_version': 'rf_cad_and_user_zero_20261012',
                         'robot_calibration': config.get('robot_calibration'),
                         'action_available':False,
                         'config': config}
        atomic_json(self.path/'manifest.json', self.manifest)
        event(self.path, 'started', mock=False)
        self.queue = queue.Queue(maxsize=config['writer_queue_size'])
        self.flush_every = config['flush_every_frames']
        self.error = None
        self.written = 0
        self.feedback_written = 0
        self.write_stats = {'queue_high_water':0,'max_queue_wait_ms':0.,'max_item_write_ms':0.,
                            'items_written':0,'queue_capacity':config['writer_queue_size']}
        self.thread = threading.Thread(target=self._run, name='raw-writer', daemon=True)
        self.thread.start()

    def submit(self, row, frames):
        self._enqueue(('sample', row, frames))

    def submit_feedback(self, packet):
        self._enqueue(('feedback', packet, None))

    def _enqueue(self, item):
        if self.error:
            raise RuntimeError('Writer failure: ' + self.error)
        try:
            self.queue.put_nowait((monotonic_ns(),item))
            if hasattr(self,'write_stats'):
                self.write_stats['queue_high_water']=max(self.write_stats['queue_high_water'],self.queue.qsize())
        except queue.Full as e:
            raise RuntimeError('Raw write queue full; recording stopped without silently dropping frames.') from e

    def _run(self):
        try:
            with (self.path/'samples.jsonl').open('w', encoding='utf-8') as samples, \
                    (self.path/'commands.jsonl').open('w', encoding='utf-8') as commands, \
                    (self.path/'feedback.jsonl').open('w', encoding='utf-8') as feedback:
                while True:
                    item = self.queue.get()
                    if item is None:
                        break
                    enqueued_ns,item=item
                    write_started=monotonic_ns()
                    self.write_stats['max_queue_wait_ms']=max(self.write_stats['max_queue_wait_ms'],(write_started-enqueued_ns)/1e6)
                    kind, row, frames = item
                    if kind == 'feedback':
                        feedback.write(json.dumps(row, allow_nan=False) + '\n')
                        self.feedback_written += 1
                        if self.feedback_written % self.flush_every == 0:
                            feedback.flush()
                            os.fsync(feedback.fileno())
                        self._item_written(write_started)
                        continue
                    for key, frame in frames.items():
                        (self.path/row['images'][key]['path']).write_bytes(frame.jpeg)
                    commands.write(json.dumps(row['command'], allow_nan=False) + '\n')
                    samples.write(json.dumps(row, allow_nan=False) + '\n')
                    self.written += 1
                    if self.written % self.flush_every == 0:
                        samples.flush(); commands.flush()
                        os.fsync(samples.fileno()); os.fsync(commands.fileno())
                    self._item_written(write_started)
                samples.flush(); commands.flush()
                os.fsync(samples.fileno()); os.fsync(commands.fileno())
                feedback.flush(); os.fsync(feedback.fileno())
        except Exception as e:
            self.error = f'{type(e).__name__}: {e}'

    def _item_written(self,started_ns):
        self.write_stats['items_written']+=1
        self.write_stats['max_item_write_ms']=max(self.write_stats['max_item_write_ms'],(monotonic_ns()-started_ns)/1e6)

    def finish(self, fault=None):
        while self.thread.is_alive():
            try:
                self.queue.put(None, timeout=.2)
                break
            except queue.Full:
                continue
        self.thread.join(timeout=15)
        if self.thread.is_alive():
            # Do not rename a directory still being written.
            raise RuntimeError('Writer did not finish; inprogress data retained for recovery.')
        fault = fault or self.error
        from .timing import timing_summary
        with (self.path/'samples.jsonl').open(encoding='utf-8') as f:
            rows=[json.loads(line) for line in f]
        timing=timing_summary(rows)
        atomic_json(self.path/'timing_report.json', timing)
        self.manifest.update(frame_count=self.written, feedback_count=self.feedback_written,
                             write_stats=self.write_stats,
                             timing_report=timing, fault=fault,
                             status='FAULT' if fault else 'REVIEW')
        atomic_json(self.path/'manifest.json', self.manifest)
        event(self.path, 'stopped', frames=self.written, fault=fault)
        final = self.path.parent/self.episode_id
        self.path.rename(final)
        self.path = final
        return self.manifest.copy()


def valid_rows(path):
    """Find a contiguous, decodable prefix. Never invent missing samples."""
    from PIL import Image
    rows = []
    sample_file = Path(path)/'samples.jsonl'
    if not sample_file.exists():
        return rows
    with sample_file.open(encoding='utf-8') as f:
        for line in f:
            try:
                row = json.loads(line)
                if row['frame_index'] != len(rows):
                    break
                for key in row['images']:
                    with Image.open(Path(path)/row['images'][key]['path']) as image:
                        image.verify()
                rows.append(row)
            except (ValueError, KeyError, OSError):
                break
    return rows


def recover_inprogress(raw_root):
    recovered = []
    for path in Path(raw_root).glob('*.inprogress'):
        manifest_file = path/'manifest.json'
        if not manifest_file.exists():
            continue
        manifest = load_json(manifest_file)
        rows = valid_rows(path)
        # Original files are retained. Recovery is excluded from exports by default.
        manifest.update(status='RECOVERED', frame_count=len(rows),
                        fault='Process exited before a complete stop; explicit recovery review required.')
        atomic_json(manifest_file, manifest)
        event(path, 'recovered', valid_prefix_frames=len(rows))
        final = path.with_name(path.name.removesuffix('.inprogress'))
        if final.exists():
            raise RuntimeError('Recovery destination exists; source left intact.')
        path.rename(final)
        recovered.append(manifest)
    return recovered
