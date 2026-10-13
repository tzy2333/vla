"""Offline checks of saved observations; no robot or camera commands."""
import math
from pathlib import Path
import numpy as np
from PIL import Image
from .storage import load_json

DEFAULTS = dict(completeness_pct=98.0, receipt_skew_ms=10.0, jerk_rms=5.0,
                speed_cv=0.6, batch_score=97.0, task_success_pct=90.0,
                human_pass_pct=95.0, kappa=0.8, secondary_sample_pct=30.0,
                failure_min_pct=5.0, failure_max_pct=15.0, minimum_episodes=20)


def unavailable(reason):
    return dict(value=None, status='unavailable', reason=reason)


def inspect_episode(raw, targets, checkpoint=lambda: None):
    import json
    raw = Path(raw); m = load_json(raw/'manifest.json')
    keys = m.get('camera_keys') or list(m.get('camera_shapes', {}))
    checks = []; rows = []; errors = []
    def check(key, passed, detail, critical=True):
        checks.append(dict(key=key, passed=bool(passed), detail=detail, critical=critical))
    with (raw/'samples.jsonl').open(encoding='utf-8') as f:
        for line in f:
            checkpoint()
            try: rows.append(json.loads(line))
            except (ValueError, TypeError): errors.append('无法解析的样本行')
    n = len(rows); expected = m.get('frame_count', 0)
    check('format', not errors and n == expected and n > 0, f'样本 {n} / 声明 {expected}')
    shape = len(m.get('state_channels') or [])
    if not shape and rows: shape = len(rows[0].get('observation.state') or [])
    valid_state = True; indices = True; timestamps = True; good = 0; image_total = n*len(keys)
    prev_time = -1; prev_ids = {}; receipt = []; gaps = []; offsets = []; bad_images = 0
    limit_violations = 0; limits = [c for c in (m.get('state_channels') or []) if c.get('limits')]
    for i, r in enumerate(rows):
        checkpoint()
        indices &= r.get('frame_index') == i
        ts = r.get('sample_ns')
        timestamps &= type(ts) is int and ts > prev_time
        if type(ts) is int: prev_time = ts
        v = r.get('observation.state')
        try:
            a = np.asarray(v, dtype=float)
            valid_state &= a.shape == (shape,) and bool(np.isfinite(a).all())
            mask=r.get('state_valid_mask', [True]*shape)
            if len(mask)!=shape or not all(x is True for x in mask): valid_state = False
            for j,c in enumerate(m.get('state_channels') or []):
                lim=c.get('limits')
                if isinstance(lim,dict) and lim.get('min') is not None and lim.get('max') is not None:
                    limit_violations += int(not lim['min'] <= a[j] <= lim['max'])
        except (ValueError, TypeError, IndexError): valid_state=False
        times=[]
        for key in keys:
            im=(r.get('images') or {}).get(key,{})
            try:
                p=(raw/im['path']).resolve()
                if not p.is_relative_to(raw.resolve()): raise ValueError('image path outside episode')
                with Image.open(p) as image:
                    image.load()
                    expected_shape=m.get('camera_shapes',{}).get(key)
                    if expected_shape and image.size != (expected_shape[1],expected_shape[0]): raise ValueError('shape')
                fid=im['frame_id']
                if type(fid) is not int or fid <= prev_ids.get(key,-1): raise ValueError('reused frame')
                prev_ids[key]=fid;good+=1
            except (KeyError, ValueError, OSError, TypeError): bad_images+=1
            received=im.get('received_ns')
            if type(received) is int: times.append(received)
            offset=im.get('alignment_offset_ms')
            if isinstance(offset,(float,int)) and math.isfinite(offset): offsets.append(abs(offset))
        if len(times)==len(keys) and len(times)>1: receipt.append((max(times)-min(times))/1e6)
        gap=((r.get('alignment') or {}).get('joints') or {}).get('gap_ms')
        if isinstance(gap,(float,int)) and math.isfinite(gap):gaps.append(gap)
    check('indices',indices,'frame_index 从 0 连续递增')
    check('timestamps',timestamps,'采样主机时间严格递增')
    check('state',valid_state,'状态维度、有限数值及有效掩码')
    check('images',bad_images==0, f'解码、尺寸、唯一帧检查：{good}/{image_total}')
    if limits: check('physical_limits',limit_violations==0,f'已声明限位违规 {limit_violations}')
    complete=100*good/(expected*len(keys)) if expected and keys else (100*n/expected if expected else 0)
    complete=min(100,complete)
    check('completeness',complete>=targets['completeness_pct'],f'{complete:.3f}% ≥ {targets["completeness_pct"]}%',False)
    if len(keys)>1:
        check('receipt_timestamps', len(receipt)==n,'每组具有完整接收时间')
        if receipt:check('receipt_skew',max(receipt)<targets['receipt_skew_ms'],f'主机接收差最大 {max(receipt):.3f} ms；目标 < {targets["receipt_skew_ms"]} ms',False)
    check('capture_fault',not m.get('fault') and m.get('status') not in ('FAULT','RECOVERED'),m.get('fault') or m.get('status'))
    metrics=dict(completeness_pct=dict(value=complete,unit='%'),
        receipt_skew_ms=dict(value=max(receipt),unit='ms') if receipt else unavailable('少于两路相机或时间缺失'),
        exposure_sync_ms=unavailable('未完成曝光时钟映射，主机接收差不能代表物理曝光同步'),
        image_offset_ms=dict(value=max(offsets),unit='ms') if offsets else unavailable('缺少图像对齐时间'),
        interpolation_gap_ms=dict(value=max(gaps),unit='ms') if gaps else unavailable('未录制关节插值证据'),
        jerk_rms=unavailable('缺少经物理标定的末端位置（m）'),
        speed_cv=unavailable('缺少经物理标定的末端位置（m）'),
        physical_limits=dict(value=limit_violations) if limits else unavailable('适配器未声明物理限位'),
        action=unavailable('未接入真实控制目标'),depth=unavailable('未接入深度录制'),
        force_torque=unavailable('未接入力/扭矩传感器'),rgb_cameras=dict(value=len(keys),unit='路（3路以上为建议，按实际选择录制）'),
        joint_velocity_torque=unavailable('来源包可保留原始反馈；尚无经物理标定的速度和力矩特征'),
        gripper=unavailable('当前观测契约未提供有效夹爪开合特征'))
    # EE derivatives are only meaningful with an explicit calibrated metres contract.
    if m.get('ee_position_calibrated') is True and m.get('ee_position_unit')=='m' and n>=7:
        try:
            p=np.asarray([r['observation.ee_pose'] for r in rows],dtype=float)
            t=np.asarray([r['timestamp'] for r in rows],dtype=float)
            if p.shape[1] not in (7,14) or not np.isfinite(p).all() or np.any(np.diff(t)<=0):raise ValueError()
            results=[]
            for col in range(0,p.shape[1],7):
                v=np.gradient(p[:,col:col+3],t,axis=0,edge_order=2)
                acceleration=np.gradient(v,t,axis=0,edge_order=2)
                jerk=np.gradient(acceleration,t,axis=0,edge_order=2)[2:-2]
                speed=np.linalg.norm(v[2:-2],axis=1);mean=float(speed.mean())
                results.append((float(np.sqrt(np.mean(np.sum(jerk*jerk,axis=1)))),float(speed.std()/mean) if mean>1e-6 else None))
            metrics['jerk_rms']=dict(value=max(x[0] for x in results),unit='m/s³',method='真实时间间隔二阶中心差分，去除首尾2帧，不平滑；多末端取最大值')
            cvs=[x[1] for x in results]
            metrics['speed_cv']=dict(value=max(cvs),unit='ratio') if all(x is not None for x in cvs) else unavailable('平均速度接近零，CV 无定义')
            check('jerk_rms',metrics['jerk_rms']['value']<targets['jerk_rms'],'末端 RMS Jerk',False)
            if metrics['speed_cv']['value'] is not None:check('speed_cv',metrics['speed_cv']['value']<targets['speed_cv'],'末端速度 CV',False)
        except (ValueError,TypeError,KeyError,IndexError):pass
    fatal=any(not c['passed'] and c['critical'] for c in checks)
    # Explicit project deduction model, not an industry certification.
    score=max(0,100-100*int(fatal)-5*sum(not c['passed'] and not c['critical'] for c in checks))
    return dict(status='passed' if all(c['passed'] for c in checks) else 'failed',
                checks=checks,metrics=metrics,score=score,critical_failure=fatal,
                score_model='项目规则：致命项直接0分，每个非致命未达标项扣5分；缺测项不计分并披露',
                frame_count=n,targets=targets,physical_sync_verified=False)
