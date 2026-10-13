import json
import copy
from pathlib import Path
import re
import shutil
import threading
import time
from .clock import monotonic, monotonic_ns

from .adapters import CAMERAS, DisabledBase
from .rf_robot_adapter import RFObservationAdapter, OBSERVATION_SCHEMA as SCHEMA
from .storage import RawWriter, atomic_json, event, load_json, recover_inprogress
from .timing import configure_timing, frame_time
from .camera_diagnostics import AlignmentDiagnostics, freshest_valid_pair
from .recording_runtime import CaptureTrace, RecordingRuntime
from .platform_contract import (SCHEMA as MODULAR_SCHEMA, create_robot, validate_descriptor,
                                selection, selected_channels, project_state, camera_bindings)


class Collector:
    def __init__(self, config, root):
        if config['mode'] != 'real':
            raise ValueError('Only real robot mode is supported; simulation has been removed.')
        if config.get('base_enabled'):
            raise ValueError('Base navigation is disabled for tabletop acquisition.')
        self.config=config
        configure_timing(config)
        self.data=(Path(root)/config['data_dir']).resolve()
        self.raw=self.data/'raw'; self.exports=self.data/'datasets'
        self.raw.mkdir(parents=True,exist_ok=True); self.exports.mkdir(parents=True,exist_ok=True)
        self.device_state_path=self.data/'device_state.json'
        if self.device_state_path.exists():
            config['released_cameras']=load_json(self.device_state_path).get('released_cameras',[])
        self.recovered=recover_inprogress(self.raw)
        self.robot=None
        try:
            from .orbbec_camera import RealCameras
            self.cameras=RealCameras(self.robot,config)
        except Exception as exc:
            from .unavailable_camera import UnavailableCameras
            self.cameras=UnavailableCameras(config,exc)
        # SDK initialization can hold Python execution for seconds. Open the
        # observation socket only after both cameras have finished initializing.
        self.robot=create_robot(config, RFObservationAdapter)
        self.descriptor=validate_descriptor(self.robot.describe())
        camera_health=self.cameras.health_check()
        config['camera_shapes']={k:[h['height'],h['width'],3] for k,h in camera_health.items()}
        config['camera_native_fps']={k:h['native_fps'] for k,h in camera_health.items()}
        config['device_sources']={'robot':self.descriptor['source'], **{k:h['source'] for k,h in camera_health.items()}}
        config['robot_calibration']=self.robot.calibration_snapshot()
        config['schema_version']=SCHEMA
        config['capture_version']=7
        config['require_ee_feedback']=False
        config['compute_ee_online']=False
        self.base=DisabledBase()
        self.lock=threading.RLock()
        self.phase='IDLE'; self.writer=None; self.thread=None
        self.preparation=None; self.preparation_error=None
        self.stop_event=threading.Event()
        self.frames=0; self.last_episode=None; self.last_error=None; self.last_export=None
        self.current_state=None; self.current_action=None; self.metrics={}
        self.started_ns=0
        self.feedback=None
        self.alignment_diagnostics=AlignmentDiagnostics()
        self.capture_failure=None
        self.capture_trace=CaptureTrace()
        self.recording_selection=None
        self.recording_channels=[]
        self.legacy_layout=False

    def capabilities(self):
        bindings=camera_bindings(self.config)
        labels={'head':'头部','left_wrist':'左腕','right_wrist':'右腕'}
        return {'robot':self.descriptor, 'cameras':[
            {'id':k,'label':v.get('label',labels.get(k,k)),'driver':v['driver'],'serial':v['serial']}
            for k,v in bindings.items()],
            'default_selection':selection(self.descriptor,list(bindings)),
            'schema_version':MODULAR_SCHEMA}

    def timing_diagnostics(self):
        camera_reader=getattr(self.cameras,'timing_diagnostics',lambda:{})
        with self.lock:
            phase=self.phase
            episode_id=self.writer.episode_id if self.writer else self.last_episode
            alignment=self.alignment_diagnostics
        return {'physical_sync_verified':False,'phase':phase,'episode_id':episode_id,
                'cameras':camera_reader(),'alignment':alignment.snapshot(),
                'note':'接收与处理耗时诊断；不能据此认定曝光同步或时钟标定已完成。'}

    def health(self):
        cameras=self.cameras.health_check()
        for key,h in cameras.items():
            f=self.cameras.latest(key)
            h.update(frame_id=f.frame_id if f else None, received_ns=f.received_ns if f else None,
                     camera_session=f.camera_session if f else None)
        return {'robot':self.robot.health_check(),'cameras':cameras,
                'base':self.base.health_check(),'disk_free_mb':round(shutil.disk_usage(self.data).free/2**20)}

    def status(self):
        with self.lock:
            feedback=self.robot.live_snapshot()
            packet=feedback['packet']
            actual_pose=packet.get('ee_pose') if packet else None
            return {'preparation':getattr(self,'preparation',None),'preparation_error':getattr(self,'preparation_error',None),'phase':self.phase,'mode':self.config['mode'],'mock':False,'frames':self.frames,
                    'device_sources':self.config['device_sources'],
                    'schema_version':MODULAR_SCHEMA,'camera_keys':list(self.cameras.health_check()),
                    'capabilities':self.capabilities(),
                    'capture_selection':self.recording_selection,
                    'recording_pull_statistics':self.robot.pull_statistics.recording() if self.recording_channels else None,
                    'written_frames':self.writer.written if self.writer else 0,
                    'elapsed_seconds':round((monotonic_ns()-self.started_ns)/1e9,2)
                    if self.phase=='RECORDING' else round(self.frames/self.config['fps'],2),
                    'fps':self.config['fps'],'state_dim':len(self.descriptor['channels']),'action_dim':0,'base_enabled':False,
                    'sample_interval_s':1/self.config['fps'],
                    'action_horizon':self.config.get('action_horizon',10),
                    'sampling_mode':self.config.get('sampling_mode','uniform_sampling'),
                    'timing_version':1, 'physical_sync_verified':False,
                    'capture_version':8, 'training_ready':False, 'fk_computation':'offline_only',
                    'observation_recording_ready':feedback['health']['healthy'],
                    'action_available':False, 'state_valid_mask':[True]*len(self.descriptor['channels']),
                    'state_channels':self.descriptor['channels'],
                    'robot_feedback':packet, 'feedback_valid':feedback['health']['healthy'],
                    'minimum_free_mb':self.config['minimum_free_mb'],
                    'timing_note':'真实关节按接收时间插值；末端位姿在采集后统一计算',
                    'state':project_state(packet['state'],self.descriptor['channels']) if packet else None,'ee_pose':actual_pose,'ee_pose_dim':0,
                    'action':self.current_action,'metrics':self.metrics,
                    'writer_queue':self.writer.queue.qsize() if self.writer else 0,
                    'episode_id':self.writer.episode_id if self.writer else None,
                    'last_episode':self.last_episode,'last_error':self.last_error,
                    'last_export':self.last_export,'data_path':str(self.data),'health':self.health()}

    def _prepare_camera_streams(self, keys):
        """Open only requested offline devices; wait for fresh frames, retry once.

        Caller reserves PREPARING, but does not hold the collector lock while
        an SDK opens. Status stays readable and concurrent mutations reject.
        """
        if not keys or all(self.cameras.health_check()[k]['healthy'] for k in keys):
            return
        if not hasattr(self.cameras,'connect'):
            from .orbbec_camera import RealCameras
            self.cameras.close()
            self.cameras=RealCameras(self.robot,dict(self.config,released_cameras=list(camera_bindings(self.config))))
        touched=[]
        labels={c['id']:c['label'] for c in self.capabilities()['cameras']}
        try:
            for key in keys:
                if self.cameras.health_check()[key]['healthy']:continue
                touched.append(key)
                for attempt in (1,2):
                    with self.lock:self.preparation=dict(camera=key,attempt=attempt,message=('正在连接' if attempt==1 else '正在重试连接')+labels[key]+'相机')
                    self.cameras.connect(key)
                    deadline=monotonic()+5
                    previous=None
                    while monotonic()<deadline:
                        h=self.cameras.health_check()[key];frame=self.cameras.latest(key)
                        if h['healthy'] and frame:
                            identity=(frame.camera_session,frame.frame_id)
                            if previous is not None and identity!=previous:break
                            previous=identity
                        else:previous=None
                        if getattr(self.cameras,'errors',{}).get(key) or (h.get('error') and h.get('age_ms') is None):break
                        time.sleep(.04)
                    else:h=self.cameras.health_check()[key]
                    if h['healthy'] and previous is not None and frame and (frame.camera_session,frame.frame_id)!=previous:break
                    if attempt==2:
                        raise ValueError(labels[key]+'相机连接失败（已重试）：'+str(h.get('error') or '未持续收到新图像')+'。请检查 USB、供电或其他程序占用后重新开始。')
                    self.cameras.release(key);time.sleep(.3)
            # All selected cameras must still be ready after other SDKs finish opening.
            deadline=monotonic()+5
            while not all(self.cameras.health_check()[k]['healthy'] for k in keys):
                if monotonic()>=deadline:raise ValueError('相机连接后未保持就绪，请检查设备连接后重新开始。')
                time.sleep(.04)
            self._save_camera_state(touched,False)
            h=self.cameras.health_check()
            self.config['camera_shapes']={k:[v['height'],v['width'],3] for k,v in h.items()}
        except Exception:
            for key in touched:self.cameras.release(key)
            self._save_camera_state(touched,True)
            raise

    def start(self, task, operator, fps=None, purpose='observation', task_metadata=None, capture_selection=None):
        task=str(task).strip();operator=str(operator).strip()
        with self.lock:
            if self.phase not in ('IDLE','REVIEW'):raise ValueError('当前状态不能开始录制，请先结束正在进行的操作或处理故障。')
            if self.thread and self.thread.is_alive():raise ValueError('上一轮正在落盘，请稍后再开始。')
            if not task or len(task)>300 or len(operator)>80:raise ValueError('请填写任务描述（最多300字），操作者最多80字。')
            if purpose!='observation':raise ValueError('当前仅支持真实观测采集。')
            rate=self.config['fps'] if fps is None else fps
            if type(rate) is not int or rate not in (5,10,20,30,50):raise ValueError('请选择有效的采集频率。')
            h=self.health()
            raw_selection={'modules':['left_arm','right_arm'],'cameras':list(h['cameras'])} if capture_selection is None and self.descriptor['id']=='rf_udp_20010' else capture_selection
            chosen=selection(self.descriptor,list(h['cameras']),raw_selection)
            channels=selected_channels(self.descriptor,chosen)
            if channels and not h['robot']['healthy']:raise ValueError('所选关节反馈未就绪，请先重连关节反馈。')
            if any(h['cameras'][k]['native_fps']<rate for k in chosen['cameras']):raise ValueError('相机帧率低于采集频率；请降低采集频率或提高相机帧率。')
            if h['disk_free_mb']<self.config['minimum_free_mb']:raise ValueError('存储空间不足，不能开始新一轮。')
            previous_phase=self.phase
            if previous_phase=='REVIEW':self.prepare_next(self.last_episode)
            self.phase='PREPARING';self.preparation=dict(message='正在准备所选设备');self.preparation_error=None
        try:
            self._prepare_camera_streams(chosen['cameras'])
            with self.lock:
                self.phase='IDLE'
                result=self._start_ready(task,operator,fps,purpose,task_metadata,capture_selection)
                self.preparation=None
                return result
        except Exception as exc:
            with self.lock:
                if self.phase in ('IDLE','PREPARING'):self.phase=previous_phase
                self.preparation=None;self.preparation_error=str(exc)
            raise

    def _start_ready(self, task, operator, fps=None, purpose='observation', task_metadata=None, capture_selection=None):
        task=str(task).strip(); operator=str(operator).strip()
        if not task or len(task)>300 or len(operator)>80:
            raise ValueError('Task is required (up to 300 characters); operator limit is 80.')
        with self.lock:
            if self.phase!='IDLE': raise ValueError('Review/discard the current episode before starting another.')
            if self.feedback and self.feedback.thread.is_alive():
                raise ValueError('Previous SDK feedback reader has not stopped.')
            if purpose == 'training':
                raise ValueError('真实观测已接入；训练示教还缺操作者目标和夹爪映射，请先录制真实观测。')
            if purpose != 'observation':
                raise ValueError('模拟录制已取消，请选择真实观测采集。')
            h=self.health()
            if capture_selection is None and self.descriptor['id']=='rf_udp_20010':
                legacy_selection={'modules':['left_arm','right_arm'],'cameras':list(h['cameras'])}
            else:legacy_selection=capture_selection
            chosen=selection(self.descriptor,list(h['cameras']),legacy_selection)
            channels=selected_channels(self.descriptor,chosen)
            if fps is not None:
                if type(fps) is not int or fps not in (5,10,20,30,50):
                    raise ValueError('Supported sample rates are 5, 10, 20, 30 and 50 Hz.')
                self.config['fps']=fps
            self.config['capture_purpose']=purpose
            if (channels and not h['robot']['healthy']) or not all(h['cameras'][k]['healthy'] for k in chosen['cameras']):
                raise ValueError('Preflight failed: robot feedback or a camera is unavailable.')
            if any(h['cameras'][k]['native_fps']<self.config['fps'] for k in chosen['cameras']):
                raise ValueError('相机帧率低于采集频率；请降低采集频率或提高相机帧率。')
            if h['disk_free_mb']<self.config['minimum_free_mb']:
                raise ValueError('Insufficient disk space for a new episode.')
            if self.config.get('require_ee_feedback') and self.robot.read_state().actual_ee is None:
                raise ValueError('Actual TCP feedback is required; command targets cannot substitute for it.')
            self.config['robot_calibration']=self.robot.calibration_snapshot()
            self.config['task_metadata']=task_metadata
            self.recording_selection=chosen
            self.recording_channels=channels
            self.legacy_layout=capture_selection is None and self.descriptor['id']=='rf_udp_20010'
            episode_config=copy.deepcopy(self.config)
            episode_config.update(camera_keys=chosen['cameras'],capture_selection=chosen,
                                  robot_descriptor=self.descriptor,state_channels=channels,
                                  schema_version=SCHEMA if self.legacy_layout else MODULAR_SCHEMA,
                                  capture_version=7 if self.legacy_layout else 8,
                                  raw_feedback_scope='complete_packet' if self.legacy_layout else 'selected_channels_only',
                                  robot_adapter_id=self.descriptor['id'])
            for field in ('camera_shapes','camera_native_fps'):
                episode_config[field]={k:v for k,v in episode_config.get(field,{}).items() if k in chosen['cameras']}
            episode_config['device_sources']={k:v for k,v in episode_config['device_sources'].items()
                                               if k in chosen['cameras'] or (k=='robot' and channels)}
            self.writer=RawWriter(self.raw,episode_config,task,operator)
            self.robot.pull_statistics.begin()
            self.alignment_diagnostics=AlignmentDiagnostics()
            self.frames=0; self.last_error=None; self.metrics={}
            self.capture_failure=None
            self.capture_trace=CaptureTrace()
            self.camera_pair_alternative_count=0
            self.started_ns=monotonic_ns(); self.stop_event.clear(); self.phase='RECORDING'
            self.thread=threading.Thread(target=self._record,name='sampler',daemon=True)
            self.thread.start()
            return {'episode_id':self.writer.episode_id,'phase':self.phase}

    def capture_group(self, previous_frames, deadline):
        keys=tuple(previous_frames)
        if not keys:
            return monotonic_ns(),{},0.
        # The target never moves while waiting for an already captured frame
        # to finish encoding. Real devices use verified SDK host arrival when enabled;
        # neither SDK host arrival nor application receipt is an exposure timestamp.
        expires=deadline+self.config.get('camera_wait_budget_ms',15)/1000
        target_ns=self.started_ns+round(self.frames*1e9/self.config['fps'])
        problem='相机新帧未就绪'
        while not self.stop_event.is_set():
            sample_ns=monotonic_ns()
            candidate_reader=getattr(self.cameras,'candidates',None)
            if candidate_reader:
                candidates={key:candidate_reader(key,target_ns) for key in keys}
            else:
                latest={key:self.cameras.latest(key,target_ns) for key in keys}
                candidates={key:[frame] if frame is not None else [] for key,frame in latest.items()}
            frames={key:items[0] if items else None for key,items in candidates.items()}
            sample_ns=monotonic_ns()
            selected=freshest_valid_pair(candidates,target_ns,sample_ns,previous_frames,self.config)
            if selected is not None:
                receipts=[frame.received_ns for frame in selected.values()]
                if any(selected[key] is not frames[key] for key in keys):
                    self.camera_pair_alternative_count=getattr(self,'camera_pair_alternative_count',0)+1
                return sample_ns,selected,(max(receipts)-min(receipts))/1e6
            missing=[key for key,frame in frames.items() if not frame or
                frame.frame_id<=previous_frames[key] or
                not 0<=(sample_ns-frame.received_ns)/1e6<=self.config['camera_max_age_ms'] or
                not 0<=(target_ns-frame_time(frame))/1e6<=self.config['camera_alignment_max_offset_ms']]
            if missing:
                problem='相机新帧缺失或超时：'+', '.join(missing)
            else:
                receipts=[f.received_ns for f in frames.values()]
                skew_ms=(max(receipts)-min(receipts))/1e6
                if skew_ms<=self.config.get('camera_pair_max_skew_ms',20):
                    return sample_ns,frames,skew_ms
                problem='相机接收时间差超限'
            if monotonic()>=expires:
                self.capture_failure={'target_ns':target_ns,'observed_ns':monotonic_ns(),
                    'frame_index':self.frames,'problem':problem,'previous_frames':dict(previous_frames),
                    'selected':{k:({'frame_id':f.frame_id,'received_ns':f.received_ns,'ready_ns':f.ready_ns} if f else None)
                                for k,f in frames.items()},
                    'native_frames':getattr(self.cameras,'failure_trace',lambda:{})()}
                raise RuntimeError(problem+'；请检查取流或降低采集频率。')
            self.stop_event.wait(min(.001,max(0,expires-monotonic())))
        return None

    def _record(self):
        writer=self.writer; fault=None
        runtime=RecordingRuntime()
        previous_frames={k:-1 for k in self.recording_selection['cameras']}
        try:
            runtime.begin()
            saved_packets=set()
            recording_session=self.robot.session
            self.stop_event.wait(self.config['feedback_warmup_ms']/1000)
            self.started_ns=monotonic_ns()
            deadline=self.started_ns/1e9
            while not self.stop_event.is_set():
                self.stop_event.wait(max(0,deadline-monotonic()))
                if self.stop_event.is_set(): break
                sample_ns=monotonic_ns()
                if self.recording_channels and self.robot.session != recording_session:
                    raise RuntimeError('真实反馈发生中断并建立新会话；本段已停止，不跨中断拼接。')
                lateness_ms=max(0,(monotonic()-deadline)*1000)
                if lateness_ms>self.config['sample_lateness_max_ms']:
                    raise RuntimeError('Sampler scheduling budget exceeded.')
                if (sample_ns-self.started_ns)/1e9>self.config['max_episode_seconds']:
                    break
                target_ns=self.started_ns+round(self.frames*1e9/self.config['fps'])
                self.capture_trace.record('camera_wait_start',target_ns=target_ns,frame_index=self.frames)
                group=self.capture_group(previous_frames,deadline)
                self.capture_trace.record('camera_wait_end',target_ns=target_ns)
                if group is None:break
                sample_ns,frames,pair_skew_ms=group
                lateness_ms=max(0,(sample_ns/1e9-deadline)*1000)
                if lateness_ms>self.config['sample_lateness_max_ms']:
                    raise RuntimeError('Sampler scheduling budget exceeded while waiting for camera frames.')
                self.capture_trace.record('interpolation_start',target_ns=target_ns,
                    expires_ns=round((deadline+self.config['alignment_wait_budget_ms']/1000)*1e9))
                if self.recording_channels:
                    values,ee_pose,proof,packets=self.robot.interpolate(target_ns,
                        deadline+self.config['alignment_wait_budget_ms']/1000)
                    if not self.legacy_layout:
                        values=project_state(values,self.recording_channels)
                    else:
                        values=values[:12]
                else:
                    values,ee_pose,proof,packets=[],None,None,()
                if packets and packets[0]['session'] != recording_session:
                    raise RuntimeError('真实反馈会话已变化；不跨中断拼接录制。')
                actual_ee=None
                alignment={'joints':proof,'tcp':None}
                for packet in packets:
                    if packet['packet_id'] not in saved_packets:
                        writer.submit_feedback(self.robot.project_packet(packet,self.recording_channels,self.legacy_layout))
                        saved_packets.add(packet['packet_id'])
                assembled_ns=monotonic_ns()
                self.capture_trace.record('interpolation_end',target_ns=target_ns)
                feedback_age=(assembled_ns-proof['after_received_ns'])/1e6 if proof else None
                ee_age=(assembled_ns-alignment['tcp']['after_received_ns'])/1e6 if alignment['tcp'] else None
                if (feedback_age is not None and not 0<=feedback_age<=self.config['feedback_max_age_ms']) or (ee_age is not None and not 0<=ee_age<=self.config['feedback_max_age_ms']):
                    raise RuntimeError('Robot feedback receipt invalid or stale.')
                ee_pose=actual_ee[:7]+actual_ee[8:15] if actual_ee is not None else None
                alignment.update(target_ns=target_ns, policy='causal_images_linear_real_joints_deferred_CAD_FK',
                                 physical_sync_verified=False)
                image_refs={}
                for key in self.recording_selection['cameras']:
                    frame=frames[key]
                    age=(sample_ns-frame.received_ns)/1e6 if frame else float('inf')
                    if not frame or age>self.config['camera_max_age_ms']:
                        raise RuntimeError(f'Camera {key} missing or stale.')
                    if frame.frame_id<=previous_frames[key]:
                        raise RuntimeError(f'Camera {key} repeated a frame; recording stopped.')
                    previous_frames[key]=frame.frame_id
                    frames[key]=frame
                    image_refs[key]={'path':f'cameras/{key}/{self.frames:06d}.jpg',
                        'frame_id':frame.frame_id,'captured_ns':frame.captured_ns,
                        'received_ns':frame.received_ns,'age_ms':round(age,3)}
                    image_refs[key].update(source=frame.source,serial=frame.serial,
                        device_timestamp_ms=frame.device_timestamp_ms,timestamp_basis=frame.timestamp_basis,
                        measurement_host_ns=frame.measurement_host_ns,
                        sdk_arrival_host_ns=frame.sdk_arrival_host_ns,
                        alignment_time_ns=frame_time(frame),
                        alignment_offset_ms=(frame_time(frame)-target_ns)/1e6,
                        device_clock_domain=frame.device_clock_domain,
                        device_timestamp_semantics=frame.device_timestamp_semantics,
                        clock_mapping_id=frame.clock_mapping_id,clock_uncertainty_ns=frame.clock_uncertainty_ns,
                        exposure_us=frame.exposure_us,sensor_timestamp_raw=frame.sensor_timestamp_raw,
                        ready_ns=frame.ready_ns,camera_session=frame.camera_session,
                        timing_metadata=frame.timing_metadata)
                command_delay=None
                row={'frame_index':self.frames,'timestamp':self.frames/self.config['fps'],
                    'sample_ns':sample_ns,'target_ns':target_ns,'assembled_ns':assembled_ns,
                    'state_received_ns':proof['after_received_ns'] if proof else None,'state_device_ns':None,
                    'observation.state':values,'observation.ee_pose':ee_pose,
                    'actual_ee':actual_ee,
                    'ee_received_ns':alignment['tcp']['after_received_ns'] if alignment['tcp'] else None,
                    'alignment':alignment,'timing_version':1,
                    'action_semantics':'unavailable_observation_only_no_target_is_inferred',
                    'action':None, 'command':None, 'training_ready':False,
                    'state_valid_mask':([True]*5+[False]+[True]*5+[False]) if self.legacy_layout else [True]*len(values),
                    'state_channels':None if self.legacy_layout else self.recording_channels,
                    'robot_feedback_source':self.descriptor['source'] if packets else None,
                    'pose_source':'deferred_offline_CAD_FK', 'fk_computation':'offline_only',
                    'feedback_session':packets[0]['session'] if packets else None,
                    'images':image_refs,'mock':False,'base_enabled':False,
                    'quality':{'feedback_age_ms':feedback_age,'ee_feedback_age_ms':ee_age,
                               'command_send_delay_ms':command_delay,
                               'assembly_lateness_ms':(assembled_ns-target_ns)/1e6,
                               'camera_pair_skew_ms':round(pair_skew_ms,3),
                               'sample_lateness_ms':round(lateness_ms,3)}}
                with self.lock:
                    writer.submit(row,frames)
                    self.frames+=1; self.current_state=values; self.current_action=None
                    self.metrics={'image_age_ms':max((i['age_ms'] for i in image_refs.values()),default=None),
                                  'camera_pair_skew_ms':round(pair_skew_ms,3),
                                  'sample_lateness_ms':round(lateness_ms,3),'feedback_age_ms':feedback_age}
                    self.metrics.update(
                        camera_alignment_offset_ms=max((abs(i['alignment_offset_ms']) for i in image_refs.values()),default=None),
                        interpolation_gap_ms=proof['gap_ms'] if proof else None,
                        command_send_delay_ms=command_delay)
                self.alignment_diagnostics.add(row)
                if self.frames%self.config['fps']==0 and shutil.disk_usage(self.data).free/2**20<self.config['minimum_free_mb']:
                    raise RuntimeError('Disk reserve reached; recording stopped.')
                deadline=(self.started_ns+round(self.frames*1e9/self.config['fps']))/1e9
        except Exception as e:
            fault=f'{type(e).__name__}: {e}'
            self.capture_trace.record('capture_fault',error=fault)
            if self.capture_failure:writer.manifest['capture_failure']=self.capture_failure
        finally:
            statistics=self.robot.pull_statistics.finish()
            writer.manifest['pull_statistics']=statistics if self.recording_channels else None
            if self.feedback:
                try:
                    self.feedback.close()
                    fault=fault or self.feedback.history.error
                except Exception as e:
                    fault=fault or str(e)
            runtime.close()
            writer.manifest['recording_runtime']=runtime.summary
            writer.manifest['camera_pairing_policy']=self.config.get('camera_pairing_policy','latest_per_camera_v0')
            writer.manifest['camera_pair_alternative_count']=self.camera_pair_alternative_count
            if fault:
                writer.manifest['capture_trace']=self.capture_trace.snapshot()
            try:
                result=writer.finish(fault)
                with self.lock:
                    self.frames=result['frame_count']; self.last_episode=result['episode_id']
                    self.last_error=result.get('fault'); self.phase='FAULT' if result.get('fault') else 'REVIEW'
            except Exception as e:
                with self.lock:
                    self.phase='FAULT'; self.last_error=f'{type(e).__name__}: {e}'
            self.stop_event.set()

    def stop(self):
        with self.lock:
            if self.phase!='RECORDING': raise ValueError('No active recording.')
            self.stop_event.set(); thread=self.thread
        thread.join(timeout=20)
        if thread.is_alive(): raise RuntimeError('Recording is still finalizing; wait and inspect status.')
        return self.status()

    def episode_path(self, episode_id):
        if not re.fullmatch(r'[0-9]{8}_[0-9]{6}_[0-9a-f]{8}',str(episode_id)):
            raise ValueError('Invalid episode identifier.')
        path=self.raw/episode_id
        if not path.is_dir(): raise ValueError('Episode not found.')
        return path

    def prepare_next(self, episode_id):
        """Leave review pending, without committing or discarding any episode."""
        with self.lock:
            if self.phase != 'REVIEW' or episode_id != self.last_episode:
                raise ValueError('请先结束当前录制；故障片段需先检查处理。')
            if self.thread and self.thread.is_alive():
                raise ValueError('片段正在落盘，请稍后重试。')
            manifest = load_json(self.episode_path(episode_id) / 'manifest.json')
            if manifest['status'] != 'REVIEW':
                raise ValueError('片段已处理，请刷新。')
            self.phase = 'IDLE'
            return dict(phase=self.phase, pending_review=episode_id)

    def episodes(self):
        items=[]
        for p in sorted(self.raw.iterdir(),reverse=True):
            if p.is_dir() and not p.name.endswith('.inprogress') and (p/'manifest.json').exists():
                m=load_json(p/'manifest.json')
                item={k:m.get(k) for k in ('episode_id','status','task','operator','frame_count','success','fault','mock','schema_version','fps','device_sources','camera_shapes','camera_keys','capture_selection','state_channels','robot_descriptor','robot_adapter_id','pull_statistics','capture_purpose','training_ready','task_id','task_metadata','task_outcome','outcome_source','created_at')}
                item.update(capture_version=m.get('capture_version',3),timing_version=m.get('timing_version',0))
                items.append(item)
        return items

    def review(self, episode_id, keep, success):
        if success is not None and type(success) is not bool:raise ValueError('任务结果只能为成功、失败或未标注。')
        with self.lock:
            if self.phase in ('PREPARING','RECORDING','EXPORTING'): raise ValueError('Collector is busy.')
            path=self.episode_path(episode_id); m=load_json(path/'manifest.json')
            if m['status'] not in ('REVIEW','FAULT','RECOVERED'):
                raise ValueError('Episode has already been reviewed.')
            if keep and (m.get('fault') or m['frame_count']==0 or m['status']!='REVIEW'):
                raise ValueError('Fault/recovered/empty episodes cannot be committed in this prototype.')
            m.update(status='COMMITTED' if keep else 'DISCARDED',success=success,
                     task_outcome='unknown' if success is None else 'success' if success else 'failure',
                     outcome_source='human_review_v1',reviewed_at=time.strftime('%Y-%m-%dT%H:%M:%S%z'))
            atomic_json(path/'manifest.json',m); event(path,'reviewed',status=m['status'],success=m['success'])
            if episode_id==self.last_episode:
                self.phase='IDLE'
                self.last_error=None
            return m

    def replay(self, episode_id,index):
        path=self.episode_path(episode_id); m=load_json(path/'manifest.json')
        if not 0<=index<m['frame_count']: raise ValueError('Frame index out of range.')
        # Seek-free JSONL implementation is sufficient for bounded prototype episodes.
        with (path/'samples.jsonl').open(encoding='utf-8') as f:
            for i,line in enumerate(f):
                if i==index: return json.loads(line)
        raise ValueError('Frame not found.')

    def export(self, episode_ids):
        raise ValueError('当前片段为真实观测，尚无示教动作目标；不能导出为训练数据。原始图像与反馈已保存，可回放审核。')

    def _camera_idle(self, key):
        if self.phase in ('PREPARING','RECORDING','EXPORTING') or (self.writer and self.writer.thread.is_alive()):
            raise ValueError('请先结束录制并等待落盘。')
        keys=list(camera_bindings(self.config))
        if key is not None and (not isinstance(key,str) or key not in keys):
            raise ValueError('未知相机。')
        return keys if key is None else [key]

    def _save_camera_state(self, keys, released):
        desired=set(self.config.get('released_cameras',[]))
        desired.update(keys) if released else desired.difference_update(keys)
        self.config['released_cameras']=sorted(desired)
        atomic_json(self.device_state_path,{'released_cameras':sorted(desired)})

    def reconnect_cameras(self, key=None):
        with self.lock:
            keys=self._camera_idle(key);previous_phase=self.phase
            self.phase='PREPARING';self.preparation=dict(message='正在连接相机');self.preparation_error=None
        try:
            self._prepare_camera_streams(keys)
            return {'connected':True,'message':'相机已就绪','health':self.cameras.health_check()}
        except Exception as exc:
            self.preparation_error=str(exc)
            return {'connected':False,'message':str(exc),'health':self.cameras.health_check()}
        finally:
            with self.lock:self.phase=previous_phase;self.preparation=None

    def reconnect_robot(self):
        with self.lock:
            if self.phase in ('PREPARING','RECORDING','EXPORTING') or (self.writer and self.writer.thread.is_alive()):
                raise ValueError('请先结束录制并等待落盘。')
            if self.robot.health_check()['healthy']:
                return {'connected':True,'message':'关节反馈已在线。'}
            self.robot.close()
            self.robot=create_robot(self.config,RFObservationAdapter)
            self.descriptor=validate_descriptor(self.robot.describe())
            return {'connected':False,'message':'只读连接已重建，等待反馈。'}

    def release_cameras(self, key=None):
        with self.lock:
            keys=self._camera_idle(key)
            if hasattr(self.cameras,'release'):
                for name in keys:self.cameras.release(name)
            elif key is None:
                from .unavailable_camera import UnavailableCameras
                self.cameras.close()
                self.cameras=UnavailableCameras(self.config,'相机已释放')
            else:
                raise ValueError('当前相机管理器不可单独释放，请先连接设备。')
            self._save_camera_state(keys,True)
            return {'released':True,'message':'相机已释放','cameras':keys}

    def close(self):
        self.stop_event.set()
        if self.thread: self.thread.join(timeout=20)
        self.cameras.close(); self.robot.close()
