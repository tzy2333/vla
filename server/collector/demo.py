"""Visible, asynchronous end-to-end pipeline demonstration (no model training)."""
import json
from pathlib import Path
import threading
import time
import numpy as np

from .exporter import check_raw,training_sample
from .adapters import CAMERAS
from .storage import atomic_json,load_json

STEPS=('设备预检','读取状态与实际末端','录制两路画面和目标动作','结束与回放检查',
       '审核并保留','导出LeRobot v2.1','官方加载与视频检查','π0.5输入输出适配','完成')


class DemoRunner:
    def __init__(self,collector):
        self.app=collector;self.lock=threading.Lock();self.thread=None
        self.data={'running':False,'passed':None,'steps':[], 'report':None}
        previous=self.app.data/'latest_demo_report.json'
        if previous.exists():
            report=load_json(previous)
            if (set(report.get('camera_health',{}))==set(CAMERAS) and
                report.get('timing_version')==self.app.config.get('timing_version') and
                report.get('dataset_fps')==self.app.config['fps'] and
                report.get('pi05_action_chunk_shape')==[self.app.config.get('action_horizon',50),16]):
                self.data={'running':False,'passed':report.get('passed'),
                           'steps':report.get('steps',[]), 'report':report}

    def snapshot(self):
        with self.lock:return json.loads(json.dumps(self.data,ensure_ascii=False))

    def step(self,index,state,detail=''):
        with self.lock:
            self.data['steps'][index].update(status=state,detail=detail)
            self.data['current_step']=index

    def start(self,fps=50,seconds=4):
        if type(fps) is not int or fps not in (5,10,20,30,50):raise ValueError('Choose 5, 10, 20, 30 or 50 Hz.')
        if type(seconds) not in (int,float) or not 2<=seconds<=10:raise ValueError('Demo duration must be 2 to 10 seconds.')
        with self.app.lock:
            if self.app.phase!='IDLE':raise ValueError('Review or finish the current episode first.')
        with self.lock:
            if self.data['running']:raise ValueError('A demo is already running.')
            self.data={'running':True,'passed':None,'fps':fps,'seconds':seconds,
                'steps':[{'name':name,'status':'pending','detail':''} for name in STEPS], 'report':None}
        self.thread=threading.Thread(target=self._run,args=(fps,seconds),name='visible-demo',daemon=True)
        self.thread.start()
        return self.snapshot()

    def _run(self,fps,seconds):
        ep=None;current=0
        try:
            self.step(0,'running')
            health=self.app.health()
            if not all(c['healthy'] for c in health['cameras'].values()) or not health['robot']['healthy']:
                raise RuntimeError('Preflight failed.')
            names={'head':'头部','left_wrist':'左腕'}
            sources='；'.join(names[k]+('模拟' if h.get('mock',True) else '真实') for k,h in health['cameras'].items())
            self.step(0,'passed',sources+'；机器人接口模拟；底盘关闭')
            current=1;self.step(1,'running')
            state=self.app.robot.read_state()
            if state.actual_ee is None:raise RuntimeError('Actual end-effector feedback missing.')
            self.step(1,'passed','实际关节/夹爪12维；实际末端位姿14维；独立时间戳')
            current=2;self.step(2,'running',f'{fps}Hz，录制{seconds}秒；每行关联接受目标16维')
            ep=self.app.start('可视化采集流程测试','pipeline_demo',fps)['episode_id']
            time.sleep(seconds)
            result=self.app.status()
            if result['phase']=='RECORDING':result=self.app.stop()
            if result['phase']!='REVIEW':raise RuntimeError(result.get('last_error') or 'Recording did not reach REVIEW.')
            self.step(2,'passed',f'{result["frames"]}帧已持续落盘')
            current=3;self.step(3,'running')
            first=self.app.replay(ep,0);last=self.app.replay(ep,result['frames']-1)
            if len(first['observation.ee_pose'])!=14:raise RuntimeError('TCP feedback shape mismatch.')
            self.step(3,'passed','首尾帧读取成功；两路原始图像与实际/目标位姿可回放')
            current=4;self.step(4,'running')
            self.app.review(ep,True,True)
            self.step(4,'passed','该段标为流程测试并保留；模拟数据不会当作真实示教')
            current=5;self.step(5,'running','编码两路视频；Parquet包含实际末端与目标动作')
            exported=self.app.export([ep]);self.step(5,'passed',exported['name'])
            current=6;self.step(6,'running')
            quality=load_json(Path(exported['path'])/'quality_report.json')
            if not quality.get('official_lerobot_loader_verified'):raise RuntimeError('Official LeRobot loader verification did not run.')
            m,rows,metrics=check_raw(self.app.episode_path(ep))
            self.step(6,'passed',f'官方LeRobot 0.3.3通过；两路视频全部解码；实测{metrics["sample_hz_measured"]}Hz')
            current=7;self.step(7,'running')
            from training.pi05_adapter import TabletopInputs,TabletopOutputs
            horizon=self.app.config.get('action_horizon',50)
            sample=training_sample(exported['path'],horizon=horizon)
            mapped=TabletopInputs()(sample)
            padded=np.pad(mapped['actions'],((0,0),(0,16)))
            restored=TabletopOutputs()({'actions':padded})['actions']
            if not np.allclose(restored,sample['actions'],atol=1e-6):raise RuntimeError('Action adapter round trip failed.')
            if sample['ee_pose'].shape!=(14,):raise RuntimeError('Dataset actual TCP feedback missing.')
            if bool(mapped['image_mask']['right_wrist_0_rgb']):raise RuntimeError('Unused image slot must be masked.')
            if set(sample['images'])!={'head','left_wrist'}:raise RuntimeError('Expected exactly two recorded cameras.')
            self.step(7,'passed',f'state[12]＋2张224×224RGB＋prompt；actions[{horizon},16]→填充[{horizon},32]→恢复16维')
            report={'passed':True,'mode':self.app.config['mode'],'episode_id':ep,'mock_robot':True,
                'timing_version':self.app.config.get('timing_version'),
                'sampling_mode':self.app.config.get('sampling_mode','uniform_sampling'),
                'dataset_fps':fps,'sample_interval_s':1/fps,
                'dataset':exported,'metrics':metrics,'camera_health':health['cameras'],
                'state_shape':[12],'actual_ee_pose_shape':[14],'action_shape':[16],
                'pi05_action_chunk_shape':[horizon,16],'pi05_internal_action_shape':[horizon,32],
                'horizon_seconds':horizon/fps,'official_loader':quality['official_loader_validation'],
                'first_sample':{'state':first['observation.state'],'actual_ee_pose':first['observation.ee_pose'],
                                'accepted_action':first['action'],'requested_action':first['command']['requested']},
                'steps':self.snapshot()['steps'],
                'pi05_adapter_numeric_test_passed':True,'pi05_model_weights_loaded':False,
                'pi05_forward_or_training_run':False,'exposure_clock_calibrated':False,
                'note':'Data pipeline and adapter mapping passed. Pi0.5 weights/tokenizer/training are not loaded on Jetson.'}
            self.step(8,'passed','采集至训练样本适配完成；下一步需要真实示教与GPU训练环境')
            report['steps']=self.snapshot()['steps']
            atomic_json(self.app.data/'latest_demo_report.json',report)
            atomic_json(Path(exported['path'])/'pipeline_demo_report.json',report)
            with self.lock:self.data.update(passed=True,report=report)
        except Exception as e:
            self.step(current,'failed',f'{type(e).__name__}: {e}')
            with self.lock:self.data.update(passed=False,error=str(e))
        finally:
            with self.lock:self.data['running']=False
