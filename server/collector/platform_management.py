"""Task records, honest metrics, versioned settings and explicit operator reset."""
import copy
import datetime
import hashlib
import json
import math
from pathlib import Path
import re
import socket
import struct
import threading
import time
import uuid

from .storage import atomic_json, load_json
from .quality_metrics import QualityMetrics


def now():
    return datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).isoformat(timespec='seconds')


def text(value, limit, label, required=False):
    if not isinstance(value, str):raise ValueError(label+'必须是文本。')
    value=value.strip()
    if len(value)>limit or (required and not value):raise ValueError(label+'为空或过长。')
    return value


def real_episode(ep):
    return ep.get('mock') is False and ((ep.get('device_sources') or {}).get('robot')=='rf_udp_20010'
        or (ep.get('schema_version')=='modular_robot_observation_v1' and bool(ep.get('robot_adapter_id'))))


def summarize(episodes):
    real=[e for e in episodes if real_episode(e)]
    done=[e for e in real if e.get('status') not in ('RECORDING',None)]
    complete=[e for e in done if not e.get('fault') and e.get('status') not in ('RECOVERED','FAULT') and e.get('frame_count',0)>0]
    reviewed=[e for e in done if e.get('status') in ('COMMITTED','DISCARDED')]
    kept=[e for e in reviewed if e['status']=='COMMITTED']
    outcomes=[e for e in done if e.get('outcome_source')=='human_review_v1' and e.get('task_outcome') in ('success','failure')]
    wins=sum(e['task_outcome']=='success' for e in outcomes)
    def ratio(a,b):return round(100*a/b,2) if b else None
    return {'total':len(real),'finalized':len(done),'completed':len(complete),
            'completion_rate':ratio(len(complete),len(done)),
            'reviewed':len(reviewed),'kept':len(kept),'retention_rate':ratio(len(kept),len(reviewed)),
            'pending':sum(e.get('status') in ('REVIEW','FAULT','RECOVERED') for e in done),
            'faults':sum(bool(e.get('fault')) or e.get('status') in ('RECOVERED','FAULT') for e in done),
            'labeled':len(outcomes),'successes':wins,'task_success_rate':ratio(wins,len(outcomes)),
            'frames':sum(e.get('frame_count',0) for e in done),
            'seconds':round(sum(e.get('frame_count',0)/e['fps'] for e in done if e.get('fps',0)>0),2),
            'excluded_legacy_or_simulated':len(episodes)-len(real)}


class PlatformManagement:
    def __init__(self, app, root, config_path):
        self.app=app;self.root=Path(root);self.config_path=Path(config_path)
        self.directory=app.data/'management';self.directory.mkdir(exist_ok=True)
        self.task_file=self.directory/'tasks.json'
        self.lock=threading.RLock()
        self.quality=QualityMetrics(self.directory)

    def _tasks(self):
        return load_json(self.task_file) if self.task_file.exists() else []

    def tasks(self):
        with self.lock:
            episodes=self.app.episodes()
            return {'tasks':[dict(t,metrics=summarize([e for e in episodes if e.get('task_id')==t['task_id']])) for t in self._tasks()]}

    def save_task(self, body):
        allowed={'task_id','revision','name','project','scene','object','operator','target_episodes','status','notes'}
        if set(body)-allowed:raise ValueError('任务包含未知字段。')
        with self.lock:
            tasks=self._tasks();ident=body.get('task_id')
            old=next((t for t in tasks if t['task_id']==ident),None)
            if ident and old is None:raise ValueError('任务不存在。')
            if old and body.get('revision')!=old['revision']:raise ValueError('任务已被其他页面修改，请刷新后重试。')
            target=body.get('target_episodes',100)
            if type(target) is not int or not 1<=target<=1_000_000:raise ValueError('目标片段数必须为1至1000000。')
            state=body.get('status','active')
            if state not in ('active','completed','archived'):raise ValueError('任务状态无效。')
            task={'task_id':ident or uuid.uuid4().hex[:12],'revision':uuid.uuid4().hex,
                  'name':text(body.get('name',''),100,'任务名称',True),
                  'target_episodes':target,'status':state,
                  'created_at':old['created_at'] if old else now(),'updated_at':now()}
            for key,limit in [('project',80),('scene',80),('object',80),('operator',80),('notes',500)]:
                task[key]=text(body.get(key,''),limit,key)
            tasks=[t for t in tasks if t['task_id']!=task['task_id']]+[task]
            atomic_json(self.task_file,tasks)
            return task

    def start_capture(self, body):
        with self.lock:
            task_id=body.get('task_id')
            task=next((t for t in self._tasks() if t['task_id']==task_id),None) if task_id else None
            if task_id and (not task or task['status']!='active'):raise ValueError('请选择进行中的任务。')
        return self.app.start(task['name'] if task else body.get('task',''),body.get('operator',''),
                                  body.get('fps'),body.get('purpose','observation'),task_metadata=task,
                                  capture_selection=body.get('capture_selection'))

    def dashboard(self, day='', task_id=''):
        if day and not re.fullmatch(r'\d{4}-\d{2}-\d{2}',day):raise ValueError('日期格式应为YYYY-MM-DD。')
        episodes=self.app.episodes()
        if day:episodes=[e for e in episodes if e['episode_id'].startswith(day.replace('-',''))]
        if task_id:episodes=[e for e in episodes if e.get('task_id')==task_id]
        groups={}
        for e in episodes:
            if not real_episode(e):continue
            key=e['episode_id'][:8];groups.setdefault(key,[]).append(e)
        reasons={}
        for e in episodes:
            if real_episode(e) and e.get('fault'):reasons[e['fault']]=reasons.get(e['fault'],0)+1
        summary=summarize(episodes)
        return {'summary':summary,'quality':self.quality.snapshot([e for e in episodes if real_episode(e)],summary),
                'daily':[{'day':d,'metrics':summarize(es)} for d,es in sorted(groups.items())[-14:]],
                'fault_reasons':[{'reason':k,'count':v} for k,v in sorted(reasons.items(),key=lambda x:-x[1])],
                'definitions':{'completion':'无故障且有数据的结束片段 / 全部结束片段',
                               'retention':'已保留 / 已保留与已舍弃',
                               'task_success':'人工明确标为成功 / 人工明确标为成功或失败；未标注及旧版标签不计入'},
                'generated_at':now()}

    def pipeline_report(self, episode_id, body=None):
        ep=next((e for e in self.app.episodes() if e['episode_id']==episode_id),None)
        if not ep or not real_episode(ep):
            raise ValueError('只能关联已结束的真实录制轮次。')
        return self.quality.report(episode_id) if body is None else self.quality.save_report(body)

    def _disk_config(self):
        data=self.config_path.read_bytes()
        return json.loads(data),hashlib.sha256(data).hexdigest()

    def settings(self):
        with self.lock:
            from .platform_contract import available_adapters, interface_fields, camera_bindings
            cfg,revision=self._disk_config()
            current=self.app.config
            supported=current.get('robot_adapter','rf_udp_20010')=='rf_udp_20010'
            model=copy.deepcopy(self.app.robot.fk.model) if supported else None
            pending=cfg.get('robot_model_override')
            descriptor=getattr(self.app,'descriptor',{})
            adapters=available_adapters()
            for adapter in adapters:
                adapter['fields']=interface_fields(adapter['id'])
                adapter['values']=copy.deepcopy(cfg.get('rf_robot',{}) if adapter['id']=='rf_udp_20010' else cfg.get('adapter_options',{}).get(adapter['id'],{}))
            return {'revision':revision,'identity':cfg.get('robot_identity',{'name':'机器人','location':''}),
                    'adapters':adapters,'model_supported':supported,
                    'camera_bindings':camera_bindings(cfg),'current_camera_bindings':camera_bindings(current),
                    'end_effector_slots':copy.deepcopy(descriptor.get('end_effector_slots',[])),
                    'end_effector_config':copy.deepcopy(cfg.get('robot_end_effectors',{}).get(current.get('robot_adapter','rf_udp_20010'),{})),
                    'current_adapter':current.get('robot_adapter','rf_udp_20010'),
                    'saved_adapter':cfg.get('robot_adapter','rf_udp_20010'),
                    'current':{k:copy.deepcopy(current.get(k)) for k in ('fps','head_camera_profile','wrist_cameras','camera_bindings')},
                    'saved':{k:cfg.get(k) for k in ('fps','head_camera_profile','wrist_cameras','camera_bindings')},
                    'model':copy.deepcopy(pending['model'] if pending else model),
                    'current_model':model,'model_pending':bool(pending and pending['model']!=model),
                    'camera_pending':any(cfg.get(k)!=current.get(k) for k in ('fps','head_camera_profile','wrist_cameras','camera_bindings','robot_adapter','rf_robot','adapter_options','robot_identity','robot_end_effectors')),
                    'robot_interface':copy.deepcopy(cfg.get('rf_robot',{})) if supported else {},
                    'parameter_write_supported':False,'fk_computation':'offline_only',
                    'controller_parameters':[{'name':n,'code':code,'writable':False,'value':None} for n,code in
                        [('目标位置',20001),('速度',20002),('加速度',20003),('减速度',20004),('限位',2012),('电子齿轮比',2013)]],
                    'control_note':'尚缺完整写入报文、单位及范围；控制器参数暂不可写。'}

    def _idle(self):
        if self.app.phase in ('PREPARING','RECORDING','EXPORTING') or (self.app.writer and not self.app.writer.queue.empty()):
            raise ValueError('请先结束录制并等待写入完成。')

    def save_settings(self, body):
        with self.lock,self.app.lock:
            self._idle()
            cfg,revision=self._disk_config()
            if body.get('revision')!=revision:raise ValueError('设置已变化，请刷新后再保存。')
            group=body.get('group')
            if group=='interface':
                from .platform_contract import available_adapters, validate_interface_options
                if set(body)-{'revision','group','robot_adapter','options'} or body.get('robot_adapter') not in [x['id'] for x in available_adapters()]:
                    raise ValueError('请选择已安装的机器人接口。')
                cfg['robot_adapter']=body['robot_adapter']
                if 'options' in body:
                    options=validate_interface_options(body['robot_adapter'],body['options'])
                    if body['robot_adapter']=='rf_udp_20010':cfg.setdefault('rf_robot',{}).update(options)
                    else:cfg.setdefault('adapter_options',{})[body['robot_adapter']]=options
            elif group=='end_effectors':
                adapter=self.app.config.get('robot_adapter','rf_udp_20010')
                if cfg.get('robot_adapter','rf_udp_20010')!=adapter:raise ValueError('请先应用机器人接口。')
                slots=getattr(self.app,'descriptor',{}).get('end_effector_slots',[])
                choices=body.get('choices')
                if set(body)!={'revision','group','choices'} or not isinstance(choices,dict) or set(choices)!={s['id'] for s in slots}:
                    raise ValueError('末端设备配置不匹配。')
                for slot in slots:
                    if choices[slot['id']] not in [o['id'] for o in slot['options']]:raise ValueError('末端类型无效。')
                cfg.setdefault('robot_end_effectors',{})[adapter]=copy.deepcopy(choices)
            elif group=='cameras':
                from .platform_contract import camera_bindings
                if set(body)!={'revision','group','camera_bindings'} or not isinstance(body['camera_bindings'],dict):
                    raise ValueError('相机配置无效。')
                allowed={'driver','serial','label','width','height','fps','use_sdk_host_arrival','jpeg_backend','sdk_queue_size','stream_format','conversion_threads'}
                for key,binding in body['camera_bindings'].items():
                    if not isinstance(binding,dict) or set(binding)-allowed:raise ValueError('未知相机参数。')
                    if not isinstance(binding.get('serial'),str) or not binding['serial'].strip():raise ValueError('填写相机序列号。')
                    if len(str(binding.get('label','')))>80:raise ValueError('相机名称过长。')
                cfg['camera_bindings']=camera_bindings({'camera_bindings':body['camera_bindings']})
            elif group=='capture':
                if set(body)-{'revision','group','name','location','fps','camera_profile'}:raise ValueError('包含未知设置。')
                cfg['robot_identity']={'name':text(body.get('name',''),80,'机器人名称',True),'location':text(body.get('location',''),120,'场地')}
                rate=body.get('fps')
                if type(rate) is not int or rate not in (5,10,20,30,50):raise ValueError('采集频率无效。')
                profiles={'high50':((1280,800,60),(848,480,60)),
                          'baseline50':((1280,720,60),(424,240,60)),
                          'dual720':((1280,720,30),(1280,720,30))}
                custom=body.get('camera_profile')=='custom'
                if not custom and body.get('camera_profile') not in profiles:raise ValueError('相机配置无效。')
                if not custom:
                    head,wrist=profiles[body['camera_profile']]
                    if rate>min(head[2],wrist[2]):raise ValueError('该分辨率下相机最高30fps，不能保存50组独立图像。')
                    if cfg.get('camera_bindings') is not None:raise ValueError('独立相机配置请使用当前档位。')
                    cfg['head_camera_profile'].update(zip(('width','height','fps'),head))
                    cfg['wrist_cameras']['left_wrist'].update(zip(('width','height','fps'),wrist))
                cfg['fps']=rate
            elif group=='model':
                if self.app.config.get('robot_adapter','rf_udp_20010')!='rf_udp_20010':
                    raise ValueError('该接口不使用 RF 本体模型参数。')
                if set(body)-{'revision','group','tcp_translation_m','arms'}:raise ValueError('包含未知模型字段。')
                pending=cfg.get('robot_model_override')
                model=copy.deepcopy(pending['model'] if pending else self.app.robot.fk.model)
                mapping=copy.deepcopy(pending['mapping'] if pending else self.app.robot.fk.mapping)
                def vector(values,length,limit):
                    if not isinstance(values,list) or len(values)!=length or any(type(x) not in (int,float) or not math.isfinite(x) or abs(x)>limit for x in values):
                        raise ValueError('参数数量、范围或数值格式无效。')
                    return values
                model['tcp_translation_m']=vector(body.get('tcp_translation_m'),3,1)
                arms=body.get('arms',{})
                if set(arms)!={'left','right'}:raise ValueError('需要左右臂完整参数。')
                for side in ('left','right'):
                    a=arms[side];arm=model['arms'][side];conv=arm['encoder_to_model']
                    if set(a)!={'zero_raw','sign','origins_m'}:raise ValueError('关节参数字段不完整。')
                    zeros=vector(a['zero_raw'],5,1);signs=vector(a['sign'],5,1)
                    if any(x not in (-1,1) for x in signs):raise ValueError('方向只允许+1或-1。')
                    if not isinstance(a['origins_m'],list) or len(a['origins_m'])!=5:raise ValueError('需要5个关节原点。')
                    for joint,origin in zip(arm['joints'],a['origins_m']):joint['origin_m']=vector(origin,3,2)
                    changed=zeros!=conv['zero_raw'] or signs!=conv['sign']
                    if changed:
                        conv.update(zero_calibration_verified=False,zero_policy='manual_gui_override',
                                    zero_calibration_scope='manually_edited_not_sampled',manual_edit_at=now())
                    conv.update(zero_raw=zeros,sign=signs)
                    mapping['arm_zero_pose']['encoder_offsets'][side]=copy.deepcopy(zeros)
                    for i,mid in enumerate(arm['motor_ids']):
                        mapping['joint_motion_directions']['by_motor_id'][str(mid)]['sign']=signs[i]*conv['model_sign_for_named_motion'][i]
                model['configuration_revision']=uuid.uuid4().hex
                mapping['configuration_revision']=model['configuration_revision']
                cfg['robot_model_override']={'model':model,'mapping':mapping,'edited_at':now()}
            else:raise ValueError('该类参数尚无写入接口。')
            cfg['mode']='real';cfg['compute_ee_online']=False;cfg['require_ee_feedback']=False
            backups=self.directory/'settings_backups';backups.mkdir(exist_ok=True)
            atomic_json(backups/(uuid.uuid4().hex+'.json'),load_json(self.config_path))
            atomic_json(self.config_path,cfg)
            return {'saved':True,'takes_effect':'after_apply','message':'已保存，待应用。','settings':self.settings()}

    def apply_settings(self, body):
        if set(body)!={'revision'}:raise ValueError('请先刷新设置。')
        with self.lock,self.app.lock:
            self._idle()
            cfg,revision=self._disk_config()
            if body['revision']!=revision:raise ValueError('设置已变化，请刷新后重试。')
            old=self.app.config
            selected=copy.deepcopy(old)
            for key in ('fps','head_camera_profile','wrist_cameras','camera_bindings','robot_adapter','robot_identity','robot_model_override','rf_robot','adapter_options','robot_end_effectors'):
                if key in cfg:selected[key]=copy.deepcopy(cfg[key])
                else:selected.pop(key,None)
            if selected.get('robot_adapter','rf_udp_20010')=='rf_udp_20010':
                from .rf_kinematics import CandidateKinematics
                candidate=CandidateKinematics()
                if selected.get('robot_model_override'):
                    candidate.model=selected['robot_model_override']['model']
                    candidate.mapping=selected['robot_model_override']['mapping']
                candidate.angles_from_observation([0.0]*96)
            from .platform_contract import camera_bindings,available_adapters
            camera_bindings(selected)
            if selected.get('robot_adapter','rf_udp_20010') not in [a['id'] for a in available_adapters()]:
                raise ValueError('机器人接口尚未安装。')
            self.app.cameras.close();self.app.robot.close()
            from .orbbec_camera import RealCameras
            from .unavailable_camera import UnavailableCameras
            from .rf_robot_adapter import RFObservationAdapter
            try:self.app.cameras=RealCameras(None,selected)
            except Exception as exc:self.app.cameras=UnavailableCameras(selected,exc)
            from .platform_contract import create_robot,validate_descriptor
            self.app.robot=create_robot(selected,RFObservationAdapter)
            self.app.descriptor=validate_descriptor(self.app.robot.describe())
            selected['robot_calibration']=self.app.robot.calibration_snapshot()
            h=self.app.cameras.health_check()
            selected['camera_shapes']={k:[v['height'],v['width'],3] for k,v in h.items()}
            selected['camera_native_fps']={k:v['native_fps'] for k,v in h.items()}
            selected['device_sources']={'robot':self.app.descriptor['source'],**{k:v['source'] for k,v in h.items()}}
            self.app.recording_selection=None
            self.app.config=selected
            return {'applied':True,'message':'设置已应用，正在检查真实设备；设备全部就绪后才能录制。','health':self.app.health()}

    def reset_robot(self, body):
        if self.app.config.get('robot_adapter','rf_udp_20010')!='rf_udp_20010':
            raise ValueError('此接口未提供复位功能。')
        if set(body)!={'request_id','confirmed'} or body.get('confirmed') is not True:
            raise ValueError('请明确确认发送厂家复位开机指令。')
        ident=body['request_id']
        if not isinstance(ident,str) or not re.fullmatch(r'[a-f0-9-]{32,36}',ident):raise ValueError('操作编号无效。')
        with self.lock,self.app.lock:
            folder=self.directory/'control_requests';folder.mkdir(exist_ok=True)
            path=folder/(ident+'.json')
            if path.exists():return load_json(path)  # Never retry a UDP motion command.
            self._idle()
            if self.app.phase not in ('IDLE','REVIEW','FAULT'):raise ValueError('当前状态不能复位。')
            if not self.app.robot.health_check()['healthy']:raise ValueError('真实关节反馈未就绪，未发送复位。')
            prior=sorted(folder.glob('*.json'),key=lambda x:x.stat().st_mtime,reverse=True)
            if prior and time.time()-prior[0].stat().st_mtime<10:raise ValueError('刚发送过复位，请先确认机器人状态。')
            target=self.app.config['rf_robot']
            record={'request_id':ident,'code':2001,'name':'厂家复位开机','sent_at':now(),
                    'status':'prepared_unknown','completion_confirmed':False,
                    'message':'尚未确认控制器接收或动作完成，不会自动重发。'}
            atomic_json(path,record)
            try:
                with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
                    sock.settimeout(.5)
                    sent=sock.sendto(struct.pack('!i',2001),(target['host'],target['port']))
                if sent!=4:raise OSError('Incomplete UDP datagram')
                record.update(status='udp_sent_unconfirmed',message='复位指令已发送；接口未提供完成确认，请观察本体。')
            except OSError as e:
                record.update(status='send_error_unconfirmed',message='发送异常，动作是否收到未知；不自动重发：'+str(e))
            atomic_json(path,record)
            return record

    def control_history(self):
        folder=self.directory/'control_requests'
        if not folder.exists():return {'operations':[]}
        files=sorted(folder.glob('*.json'),key=lambda x:x.stat().st_mtime,reverse=True)[:20]
        return {'operations':[load_json(p) for p in files]}
