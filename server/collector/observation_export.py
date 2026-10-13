"""Dynamic LeRobot v2.1 observation archives, never invented action labels."""
import importlib.metadata
import json
from pathlib import Path
import shutil
import time
import uuid
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
from .exporter import DATA_PATH, VIDEO_PATH, encode_video, video_stats, stat, jsonl, read_rows
from .storage import load_json, atomic_json
from .platform_management import real_episode


def contract(m, rows):
    channels=m.get('state_channels') or []
    return dict(fps=m['fps'],camera_shapes=m.get('camera_shapes',{}),camera_keys=m.get('camera_keys',list(m.get('camera_shapes',{}))),
                channels=channels,state_size=len(rows[0].get('observation.state') or []),
                robot_adapter_id=m.get('robot_adapter_id'),schema_version=m.get('schema_version'),
                robot_calibration=m.get('robot_calibration'),device_sources=m.get('device_sources'),
                capture_selection=m.get('capture_selection'))


def validate_official(root):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    import torch
    if importlib.metadata.version('lerobot')!='0.3.3':raise ValueError('v2.1 验证需固定 LeRobot 0.3.3。')
    info=load_json(root/'meta/info.json')
    ds=LeRobotDataset('local/real_observations',root=root,video_backend='pyav')
    if len(ds)!=info['total_frames']:raise ValueError('官方读取帧数不一致。')
    inspected=0
    for ep in range(info['total_episodes']):
        lo=int(ds.episode_data_index['from'][ep]);hi=int(ds.episode_data_index['to'][ep])
        for index in set((lo,(lo+hi)//2,hi-1)):
            r=ds[index]
            if 'action' in r:raise ValueError('观测档案不得存在虚构动作。')
            for name,f in info['features'].items():
                if f['dtype']=='video':
                    h,w,_=f['shape']
                    if tuple(r[name].shape)!=(3,h,w) or not torch.isfinite(r[name]).all():raise ValueError('官方视频读取不一致。')
                if name=='observation.state' and tuple(r[name].shape)!=tuple(f['shape']):raise ValueError('官方状态维度不一致。')
            if int(r['episode_index'].item())!=ep:raise ValueError('片段索引不一致。')
            if bool(r['next.done'].item())!=(index==hi-1):raise ValueError('结束标记不一致。')
            inspected+=1
    return dict(passed=True,loader_version='0.3.3',format='v2.1',samples_checked=inspected,video_backend='pyav')


def export_observations(paths,output_root,reviews,annotations=None,verify_official=True):
    bundles=[]
    for path in paths:
        raw=Path(path);m=load_json(raw/'manifest.json');rows=read_rows(raw)
        if not real_episode(m) or m.get('fault') or not rows or len(rows)!=m['frame_count']:raise ValueError('片段必须为完整真实观测。')
        if any(r.get('action') is not None for r in rows):raise ValueError('该导出器仅用于无动作目标的观测档案；动作数据需单独契约。')
        c=contract(m,rows)
        if bundles and c!=bundles[0][3]:raise ValueError('相机、关节通道、标定或采样率不同，请分批导出。')
        for i,r in enumerate(rows):
            a=np.asarray(r.get('observation.state') or [],dtype=float)
            if r.get('frame_index')!=i or a.shape!=(c['state_size'],) or not np.isfinite(a).all() or not all(r.get('state_valid_mask',[True]*len(a))):raise ValueError('状态或帧索引无效，不能用零填充缺失状态。')
            for key in c['camera_keys']:
                image=(raw/r['images'][key]['path']).resolve()
                if not image.is_relative_to(raw.resolve()):raise ValueError('图像路径越界。')
        bundles.append((raw,m,rows,c))
    if not bundles:raise ValueError('没有选择片段。')
    output_root=Path(output_root);output_root.mkdir(exist_ok=True,parents=True)
    size=sum(p.stat().st_size for raw,_,_,_ in bundles for p in raw.rglob('*') if p.is_file())
    if shutil.disk_usage(output_root).free < max(512*2**20,size*2):raise ValueError('磁盘空间不足以完成视频及校验，请先扩容。')
    name='observations_v21_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]
    final=output_root/name;temp=output_root/(name+'.inprogress');(temp/'meta').mkdir(parents=True);(temp/'provenance').mkdir()
    c=bundles[0][3];fps=c['fps'];features={}
    if c['state_size']:
        names=[x['id'] for x in c['channels']] if len(c['channels'])==c['state_size'] else None
        features['observation.state']=dict(dtype='float32',shape=[c['state_size']],names=names)
    for k in ('timestamp','frame_index','episode_index','index','task_index'):
        features[k]=dict(dtype='float32' if k=='timestamp' else 'int64',shape=[1],names=None)
    features['next.done']=dict(dtype='bool',shape=[1],names=None)
    for key in c['camera_keys']:
        features['observation.images.'+key]=dict(dtype='video',shape=c['camera_shapes'][key],names=['height','width','channels'],
            info={'video.fps':fps,'video.codec':'h264','video.pix_fmt':'yuv420p','video.is_depth_map':False,'has_audio':False})
    tasks=[];episode_meta=[];stats=[];global_index=0
    for ep,(raw,m,rows,_) in enumerate(bundles):
        n=len(rows);task=m.get('task') or '未命名任务'
        if task not in tasks:tasks.append(task)
        arrays={'timestamp':pa.array(np.arange(n,dtype=np.float32)/fps,type=pa.float32()),
            'frame_index':pa.array(range(n),type=pa.int64()),'episode_index':pa.array([ep]*n,type=pa.int64()),
            'index':pa.array(range(global_index,global_index+n),type=pa.int64()),
            'task_index':pa.array([tasks.index(task)]*n,type=pa.int64()),'next.done':pa.array([False]*(n-1)+[True])}
        if c['state_size']:arrays['observation.state']=pa.array([r['observation.state'] for r in rows],type=pa.list_(pa.float32(),c['state_size']))
        dest=temp/DATA_PATH.format(episode_chunk=ep//1000,episode_index=ep);dest.parent.mkdir(exist_ok=True,parents=True)
        table=pa.table(arrays);pq.write_table(table,dest,compression='zstd');back=pq.read_table(dest)
        if not table.equals(back):raise ValueError('Parquet 完整回读不一致。')
        es={k:stat(np.asarray(back[k].to_pylist(),dtype=float)) for k in arrays}
        for k in arrays:
            if k!='observation.state':
                for s in ('min','max','mean','std'):es[k][s]=[es[k][s]]
        for key in c['camera_keys']:
            vk='observation.images.'+key;video=temp/VIDEO_PATH.format(episode_chunk=ep//1000,episode_index=ep,video_key=vk)
            h,w,_=c['camera_shapes'][key];encode_video(raw,rows,key,video,fps,w,h);es[vk]=video_stats(video,n,fps)
        ident=m['episode_id'];dest=temp/'provenance'/f'episode_{ep:06d}';dest.mkdir()
        atomic_json(dest/'manifest.json',m);atomic_json(dest/'review.json',reviews[ident])
        for source in ('samples.jsonl','feedback.jsonl','events.jsonl','commands.jsonl'):
            if (raw/source).exists():shutil.copy2(raw/source,dest/source)
        if annotations and (Path(annotations)/(ident+'.json')).exists():shutil.copy2(Path(annotations)/(ident+'.json'),dest/'segments.json')
        episode_meta.append(dict(episode_index=ep,tasks=[task],length=n,source_episode_id=ident,
                                 operator=m.get('operator'),task_outcome=reviews[ident].get('outcome','unknown')))
        stats.append(dict(episode_index=ep,stats=es));global_index+=n
    info=dict(codebase_version='v2.1',robot_type=c['robot_adapter_id'] or 'real_observation',total_episodes=len(bundles),
        total_frames=global_index,total_tasks=len(tasks),total_videos=len(bundles)*len(c['camera_keys']),
        total_chunks=(len(bundles)+999)//1000,chunks_size=1000,fps=fps,splits={'train':f'0:{len(bundles)}'},
        data_path=DATA_PATH,video_path=VIDEO_PATH,features=features,mock=False,training_ready=False,
        action_available=False,dataset_purpose='observation_archive',physical_sync_verified=False,
        contract=c,provenance_path='provenance/episode_{episode_index:06d}')
    atomic_json(temp/'meta/info.json',info);jsonl(temp/'meta/tasks.jsonl',[dict(task_index=i,task=t) for i,t in enumerate(tasks)])
    jsonl(temp/'meta/episodes.jsonl',episode_meta);jsonl(temp/'meta/episodes_stats.jsonl',stats)
    validation=validate_official(temp) if verify_official else dict(passed=False,reason='isolated test only')
    atomic_json(temp/'validation.json',dict(parquet_full_readback=True,all_video_frames_decoded=True,official=validation))
    (temp/'README.md').write_text('# Real observations · LeRobot v2.1\n\n真实观测档案，缺少真实 action，training_ready=false。不可直接用于行为克隆训练。\n'
        '每个片段一个 Parquet，每相机每片段一个 MP4；任务、统计采用 v2.1 JSONL 元数据。\n'
        'provenance 保存原始时间证据、标定快照、审核及分段；引用的原始 JPEG 保留在采集服务器。\n'
        '官方加载器固定 LeRobot 0.3.3。物理曝光同步未标定。\n',encoding='utf-8')
    temp.rename(final)
    return dict(path=str(final),name=name,episodes=len(bundles),frames=global_index,format='v2.1',training_ready=False,official_validation=validation)
