"""Local, transactional LeRobot v2.1 export and independent readback checks.

This exporter publishes NEW datasets; it never appends to an existing export.
"""
from fractions import Fraction
import io
import json
import shutil
from pathlib import Path
import time
import uuid

import av
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
from PIL import Image

from .adapters import ACTION_NAMES, CAMERAS, SCHEMA, STATE_NAMES
from .storage import atomic_json, load_json
from .timing import validate_alignment, timing_summary

DATA_PATH = 'data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet'
VIDEO_PATH = 'videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4'


def stat(a):
    a = np.asarray(a)
    return {'min': np.min(a, axis=0).tolist(), 'max': np.max(a, axis=0).tolist(),
            'mean': np.mean(a, axis=0).tolist(), 'std': np.std(a, axis=0).tolist(),
            'count': [len(a)]}


def jsonl(path, rows):
    Path(path).write_text(''.join(json.dumps(r, ensure_ascii=False, allow_nan=False)+'\n' for r in rows),
                          encoding='utf-8')


def read_rows(path):
    with (Path(path)/'samples.jsonl').open(encoding='utf-8') as f:
        return [json.loads(line) for line in f if line.strip()]


def check_raw(path):
    m = load_json(Path(path)/'manifest.json')
    if m.get('schema_version')!=SCHEMA or set(m.get('camera_shapes',{}))!=set(CAMERAS):
        raise ValueError('该片段属于旧相机配置，请只导出当前头部和左腕两路配置的片段。')
    if m['status'] != 'COMMITTED' or m.get('fault'):
        raise ValueError('Only reviewed, committed, fault-free episodes may be exported.')
    rows = read_rows(path)
    if len(rows) != m['frame_count'] or not rows:
        raise ValueError('Raw row count differs from manifest or episode is empty.')
    if m.get('capture_version',3)>=4:
        with (Path(path)/'feedback.jsonl').open(encoding='utf-8') as feedback:
            packets=[json.loads(line) for line in feedback]
        validate_alignment(m,rows,packets)
    state = np.array([r['observation.state'] for r in rows], dtype=np.float32)
    action = np.array([r['action'] for r in rows], dtype=np.float32)
    if state.shape != (len(rows),12) or action.shape != (len(rows),16):
        raise ValueError('Invalid state/action dimensions.')
    if not np.isfinite(state).all() or not np.isfinite(action).all():
        raise ValueError('Non-finite numeric value.')
    if m['config'].get('require_ee_feedback'):
        ee=np.array([r.get('observation.ee_pose') for r in rows],dtype=np.float32)
        if ee.shape!=(len(rows),14) or not np.isfinite(ee).all():
            raise ValueError('Actual TCP feedback must contain 14 finite pose values.')
        for s in (3,10):
            if not np.allclose(np.linalg.norm(ee[:,s:s+4],axis=1),1,atol=1e-4):
                raise ValueError('Actual TCP quaternion is not normalized.')
    for s in (3,11):
        if not np.allclose(np.linalg.norm(action[:,s:s+4],axis=1),1,atol=1e-4):
            raise ValueError('Action quaternion is not normalized.')
    for values in (state[:,[5,11]], action[:,[7,15]]):
        if ((values<0)|(values>1)).any():
            raise ValueError('Gripper value outside [0,1].')
    prev = {k: -1 for k in CAMERAS}
    last_ns = -1
    for i,r in enumerate(rows):
        if r['frame_index'] != i or r['sample_ns'] <= last_ns:
            raise ValueError('Non-contiguous index or non-monotonic raw timestamp.')
        last_ns = r['sample_ns']
        if r['state_received_ns'] > r['command']['sent_ns']:
            raise ValueError('Command precedes the actual observation.')
        if not np.allclose(r['action'],r['command']['accepted'],atol=1e-7):
            raise ValueError('Action label is not the accepted target.')
        if m.get('capture_version',3)<4 and r['command']['sent_ns'] - r['sample_ns'] > int(1e9/m['fps']):
            raise ValueError('Command was not sent within its sample interval; effect time is unknown.')
        for key in CAMERAS:
            im = r['images'][key]
            age = (r['sample_ns'] - im['received_ns' if m.get('capture_version',3)>=4 else 'captured_ns'])/1e6
            if not 0 <= age <= m['config']['camera_max_age_ms'] or im['frame_id'] <= prev[key]:
                raise ValueError('Stale/reused/out-of-order camera frame.')
            prev[key] = im['frame_id']
            with Image.open(Path(path)/im['path']) as image:
                shape=m.get('camera_shapes',{}).get(key,[m['height'],m['width'],3])
                if image.size != (shape[1],shape[0]):
                    raise ValueError('Unexpected camera dimensions.')
                image.verify()
        receipts=[r['images'][key]['received_ns'] for key in CAMERAS]
        if (max(receipts)-min(receipts))/1e6>m['config'].get('camera_pair_max_skew_ms',20):
            raise ValueError('Camera host-receipt time difference exceeds the pairing budget.')
    elapsed = np.diff([r['sample_ns'] for r in rows])/1e6
    interval_error = float(np.max(np.abs(elapsed-1000/m['fps']))) if len(elapsed) else 0
    if interval_error > m['config']['sample_lateness_max_ms']:
        raise ValueError('Raw intervals exceed the configured timing budget.')
    return m,rows,{'frames':len(rows),'max_interval_error_ms':round(interval_error,3),
                   'sample_hz_measured':round((len(rows)-1)*1e9/(rows[-1]['sample_ns']-rows[0]['sample_ns']),3) if len(rows)>1 else None,
                   'unique_image_frames':{k:len(set(r['images'][k]['frame_id'] for r in rows)) for k in CAMERAS},
                   'max_camera_pair_skew_ms':round(max((max(r['images'][k]['received_ns'] for k in CAMERAS)-min(r['images'][k]['received_ns'] for k in CAMERAS))/1e6 for r in rows),3),
                   'max_image_age_ms':max(r['images'][k]['age_ms'] for r in rows for k in CAMERAS),
                   'timing':timing_summary(rows) | {'verified':m.get('capture_version',3)>=4}}


def encode_video(raw, rows, key, output, fps, width, height):
    output.parent.mkdir(parents=True,exist_ok=True)
    with av.open(str(output),'w') as c:
        stream = c.add_stream('libx264',rate=fps)
        stream.width,stream.height,stream.pix_fmt = width,height,'yuv420p'
        stream.options = {'crf':'23','preset':'veryfast','threads':'1'}
        for i,r in enumerate(rows):
            with Image.open(raw/r['images'][key]['path']) as im:
                rgb = np.asarray(im.convert('RGB'))
            frame = av.VideoFrame.from_ndarray(rgb,format='rgb24')
            frame.pts,frame.time_base = i,Fraction(1,fps)
            for p in stream.encode(frame): c.mux(p)
        for p in stream.encode(): c.mux(p)


def video_stats(path, expected_frames, fps):
    """Decode every frame, check presentation time and calculate exact channel moments."""
    n, pixels = 0,0
    lo,hi,total,total2 = np.full(3,np.inf),np.full(3,-np.inf),np.zeros(3),np.zeros(3)
    with av.open(str(path)) as c:
        for frame in c.decode(video=0):
            timestamp = float(frame.pts * frame.time_base)
            if abs(timestamp-n/fps) > .5/fps:
                raise ValueError('Video presentation timestamp mismatch.')
            image = frame.to_ndarray(format='rgb24').astype(np.float64)/255
            flat = image.reshape(-1,3)
            lo=np.minimum(lo,flat.min(axis=0)); hi=np.maximum(hi,flat.max(axis=0))
            total += flat.sum(axis=0); total2 += np.square(flat).sum(axis=0)
            pixels += len(flat); n+=1
    if n != expected_frames:
        raise ValueError(f'Video has {n} frames; expected {expected_frames}.')
    mean=total/pixels; std=np.sqrt(np.maximum(0,total2/pixels-mean*mean))
    return {k:v.reshape(3,1,1).tolist() for k,v in
            {'min':lo,'max':hi,'mean':mean,'std':std}.items()} | {'count':[n]}


def export_dataset(episode_paths, output_root):
    episodes = [check_raw(Path(p)) for p in episode_paths]
    if not episodes:
        raise ValueError('No committed episodes selected.')
    fps,width,height = (episodes[0][0][k] for k in ('fps','width','height'))
    first=episodes[0][0]
    if any(m.get('capture_version',3)!=first.get('capture_version',3) for m,_,_ in episodes):
        raise ValueError('Export legacy and time-aligned captures separately.')
    if any(m.get('camera_pairing_policy','latest_per_camera_v0')!=first.get('camera_pairing_policy','latest_per_camera_v0') for m,_,_ in episodes):
        raise ValueError('Export episodes with different image pairing policies separately.')
    camera_shapes=first.get('camera_shapes',{k:[height,width,3] for k in CAMERAS})
    sources=first.get('device_sources',{'robot':'mock',**{k:'mock' for k in CAMERAS}})
    include_ee=bool(first['config'].get('require_ee_feedback'))
    if any(bool(m['config'].get('require_ee_feedback'))!=include_ee for m,_,_ in episodes):
        raise ValueError('Do not mix episodes with different TCP feedback feature contracts.')
    if any((m['fps'],m['width'],m['height'],m['schema_version'],m['mock']) !=
           (fps,width,height,SCHEMA,True) for m,_,_ in episodes):
        raise ValueError('Do not mix schema/calibration/source types in an export.')
    if any(m.get('camera_shapes',{k:[m['height'],m['width'],3] for k in CAMERAS})!=camera_shapes or
           m.get('device_sources',{'robot':'mock',**{k:'mock' for k in CAMERAS}})!=sources
           for m,_,_ in episodes):
        raise ValueError('Select episodes with the same camera sources and resolutions; do not mix all-mock and hybrid episodes.')
    name = 'mock_v21_' + time.strftime('%Y%m%d_%H%M%S') + '_' + uuid.uuid4().hex[:6]
    final = Path(output_root)/name
    temp = final.with_name(name+'.inprogress')
    (temp/'meta').mkdir(parents=True)
    (temp/'timing').mkdir()
    tasks=[]; episode_meta=[]; stats_meta=[]; raw_checks=[]; global_index=0
    features={'observation.state':{'dtype':'float32','shape':[12],'names':STATE_NAMES},
              'action':{'dtype':'float32','shape':[16],'names':ACTION_NAMES}}
    if include_ee:
        features['observation.ee_pose']={'dtype':'float32','shape':[14],
            'names':[f'{s}_actual_ee_{k}' for s in ('left','right') for k in ('x','y','z','qw','qx','qy','qz')]}
    for k in ('timestamp','frame_index','episode_index','index','task_index'):
        features[k]={'dtype':'float32' if k=='timestamp' else 'int64','shape':[1],'names':None}
    for key in CAMERAS:
        features['observation.images.'+key]={'dtype':'video','shape':camera_shapes[key],
            'names':['height','width','channels'],'info':{'video.fps':fps,'video.codec':'h264',
            'video.pix_fmt':'yuv420p','video.is_depth_map':False,'has_audio':False}}
    try:
        for ep,(raw,(m,rows,quality)) in enumerate(zip(episode_paths,episodes)):
            raw=Path(raw); n=len(rows)
            jsonl(temp/f'timing/episode_{ep:06d}.jsonl',[
                {k:v for k,v in r.items() if k not in ('observation.state','observation.ee_pose','actual_ee','action')}
                for r in rows])
            atomic_json(temp/f'timing/episode_{ep:06d}_manifest.json',m)
            if m.get('capture_version',3)>=4:
                shutil.copy2(raw/'feedback.jsonl',temp/f'timing/episode_{ep:06d}_feedback.jsonl')
            if m['task'] not in tasks: tasks.append(m['task'])
            task_index=tasks.index(m['task'])
            arrays={
                'observation.state':pa.array([r['observation.state'] for r in rows],type=pa.list_(pa.float32(),12)),
                'action':pa.array([r['action'] for r in rows],type=pa.list_(pa.float32(),16)),
                'timestamp':pa.array(np.arange(n,dtype=np.float32)/fps,type=pa.float32()),
                'frame_index':pa.array(np.arange(n),type=pa.int64()),
                'episode_index':pa.array([ep]*n,type=pa.int64()),
                'index':pa.array(np.arange(global_index,global_index+n),type=pa.int64()),
                'task_index':pa.array([task_index]*n,type=pa.int64())}
            if include_ee:
                arrays['observation.ee_pose']=pa.array([r['observation.ee_pose'] for r in rows],type=pa.list_(pa.float32(),14))
            dest=temp/DATA_PATH.format(episode_chunk=ep//1000,episode_index=ep)
            dest.parent.mkdir(parents=True,exist_ok=True)
            pq.write_table(pa.table(arrays),dest,compression='zstd')
            # Numeric readback is independent of construction.
            readback=pq.read_table(dest)
            if readback.num_rows != n or readback.schema.field('action').type != pa.list_(pa.float32(),16):
                raise ValueError('Parquet readback failed.')
            es={k:stat(readback[k].to_pylist()) for k in arrays}
            for key in CAMERAS:
                vk='observation.images.'+key
                video=temp/VIDEO_PATH.format(episode_chunk=ep//1000,episode_index=ep,video_key=vk)
                hh,ww,_=camera_shapes[key]
                encode_video(raw,rows,key,video,fps,ww,hh)
                es[vk]=video_stats(video,n,fps)
            # Scalar statistics use a [1] dimension like official LeRobot.
            for k in ('timestamp','frame_index','episode_index','index','task_index'):
                for sk in ('min','max','mean','std'): es[k][sk]=[es[k][sk]]
            episode_meta.append({'episode_index':ep,'tasks':[m['task']],'length':n})
            stats_meta.append({'episode_index':ep,'stats':es})
            raw_checks.append({'source_episode':m['episode_id'],**quality})
            global_index+=n
        info={'codebase_version':'v2.1','robot_type':'mock_bimanual_5axis',
              'total_episodes':len(episodes),'total_frames':global_index,'total_tasks':len(tasks),
              'total_videos':len(episodes)*len(CAMERAS),'total_chunks':(len(episodes)+999)//1000,
              'chunks_size':1000,'fps':fps,'splits':{'train':f'0:{len(episodes)}'},
              'data_path':DATA_PATH,'video_path':VIDEO_PATH,'features':features,
              'mock':True,'device_sources':sources,'schema_version':SCHEMA,'base_enabled':False,
               'capture_version':first.get('capture_version',3),'camera_keys':list(CAMERAS),
               'timing_version':first.get('timing_version',0),
               'physical_sync_verified':False,
               'camera_pairing_policy':first.get('camera_pairing_policy','latest_per_camera_v0'),
               'timing_path':'timing/episode_{episode_index:06d}.jsonl',
              'sampling_mode':first.get('sampling_mode','uniform_sampling'),
              'sample_interval_s':1/fps,'action_horizon':first['config'].get('action_horizon',10),
              'native_camera_fps':first['config'].get('camera_native_fps',{k:first['config']['camera_fps'] for k in CAMERAS})}
        atomic_json(temp/'meta/info.json',info)
        jsonl(temp/'meta/tasks.jsonl',[{'task_index':i,'task':t} for i,t in enumerate(tasks)])
        jsonl(temp/'meta/episodes.jsonl',episode_meta)
        jsonl(temp/'meta/episodes_stats.jsonl',stats_meta)
        import importlib.util
        official_check=None
        if importlib.util.find_spec('lerobot'):
            from .official_validation import validate_with_lerobot
            official_check=validate_with_lerobot(temp,horizon=first['config'].get('action_horizon',10))
        atomic_json(temp/'quality_report.json',{'passed':True,'mock':True,'device_sources':sources,'episodes':raw_checks,
                    'checks':['raw dimensions/provenance/timing','Parquet full readback',
                              'both camera videos fully decoded','video PTS/frame counts'],
                    'official_lerobot_loader_verified':bool(official_check),
                    'official_loader_validation':official_check,
                    'note':'Robot actions/state are simulated. Camera provenance is recorded in device_sources. This validates the pipeline, not physical task execution.'})
        (temp/'README.md').write_text('# PIPELINE TEST DATA\n\nRobot state and actions are simulated. '
            'Camera sources: '+json.dumps(sources)+'. Do not mix with real demonstrations.\n',encoding='utf-8')
        temp.rename(final)
        return {'path':str(final),'name':name,'frames':global_index,'episodes':len(episodes),'mock':True}
    except Exception:
        # Failed staging directories are intentionally retained for diagnosis, never published.
        raise


def training_sample(dataset, episode=0, frame_index=0, horizon=50):
    """Small readback adapter showing the effective Pi inputs/labels (no model training)."""
    root=Path(dataset); info=load_json(root/'meta/info.json'); fps=info['fps']
    table=pq.read_table(root/DATA_PATH.format(episode_chunk=episode//1000,episode_index=episode))
    rows=table.to_pylist()
    if not 0 <= frame_index < len(rows): raise ValueError('Frame out of range.')
    row=rows[frame_index]
    actions=np.array([rows[min(i,len(rows)-1)]['action']
                      for i in range(frame_index,frame_index+horizon)],dtype=np.float32)
    mask=np.arange(frame_index,frame_index+horizon)<len(rows)
    images={}
    for key in CAMERAS:
        path=root/VIDEO_PATH.format(episode_chunk=episode//1000,episode_index=episode,
                                   video_key='observation.images.'+key)
        with av.open(str(path)) as c:
            for i,f in enumerate(c.decode(video=0)):
                if i==frame_index:
                    im=f.to_image(); im.thumbnail((224,224))
                    canvas=Image.new('RGB',(224,224)); canvas.paste(im,((224-im.width)//2,(224-im.height)//2))
                    images[key]=np.asarray(canvas,dtype=np.uint8); break
    tasks=[json.loads(l) for l in (root/'meta/tasks.jsonl').read_text(encoding='utf-8').splitlines()]
    return {'state':np.array(row['observation.state'],dtype=np.float32),'images':images,
            'ee_pose':np.array(row['observation.ee_pose'],dtype=np.float32) if 'observation.ee_pose' in row else None,
            'prompt':next(t['task'] for t in tasks if t['task_index']==row['task_index']),
            'actions':actions,'action_valid_mask':mask}
