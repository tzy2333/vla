"""Bounded SDK inventory in a short-lived process; never starts a stream."""
import copy
import json
import re
import subprocess
import sys
import threading
import time
import uuid
from .platform_contract import camera_bindings
from .storage import atomic_json
from .platform_management import now, text


def enumerate_devices():
    devices=[];errors=[]
    try:
        import pyrealsense2 as rs
        rs_context=rs.context()
        for d in rs_context.query_devices():
            serial=d.get_info(rs.camera_info.serial_number);profiles=[]
            for sensor in d.query_sensors():
                for p in sensor.get_stream_profiles():
                    if p.stream_type()==rs.stream.color and p.format() in (rs.format.rgb8,rs.format.yuyv):
                        v=p.as_video_stream_profile();profiles.append(dict(width=v.width(),height=v.height(),fps=v.fps(),stream_format=str(p.format()).split('.')[-1]))
            devices.append(dict(driver='realsense',serial=serial,label=d.get_info(rs.camera_info.name),profiles=profiles))
    except Exception as exc:errors.append('RealSense: '+str(exc))
    try:
        from pyorbbecsdk import Context, OBSensorType, OBFormat
        ob_context=Context()
        found=ob_context.query_devices()
        for i in range(found.get_count()):
            d=found.get_device_by_index(i);info=d.get_device_info();profiles=[]
            try:
                ps=d.get_sensor(OBSensorType.COLOR_SENSOR).get_stream_profile_list()
                for j in range(ps.get_count()):
                    v=ps.get_stream_profile_by_index(j).as_video_stream_profile()
                    if v.get_format()==OBFormat.MJPG:profiles.append(dict(width=v.get_width(),height=v.get_height(),fps=v.get_fps()))
            except Exception as exc:errors.append('Orbbec 档位读取: '+str(exc))
            devices.append(dict(driver='orbbec',serial=info.get_serial_number(),label=info.get_name(),profiles=profiles))
    except Exception as exc:errors.append('Orbbec: '+str(exc))
    return dict(devices=devices,errors=errors)


class CameraInventory:
    def __init__(self,app,management):
        self.app=app;self.management=management;self.lock=threading.RLock();self.scanning=False
        self.cache=dict(devices=[],errors=[],scanned_at=None);self.last=0;self.thread=None

    def view(self):
        with self.lock:
            result=copy.deepcopy(self.cache);bindings=camera_bindings(self.app.config)
            for device in result['devices']:
                device['bound_as']=next((k for k,b in bindings.items() if b['driver']==device['driver'] and b['serial']==device['serial']),None)
            return dict(result,scanning=self.scanning,paused=self.app.phase in ('PREPARING','RECORDING','EXPORTING'),
                        age_seconds=round(time.monotonic()-self.last,1) if self.last else None)

    def refresh(self):
        with self.app.lock,self.lock:
            if self.scanning or time.monotonic()-self.last<8 or self.app.phase in ('PREPARING','RECORDING','EXPORTING'):return self.view()
            self.scanning=True
        def scan():
            try:
                r=subprocess.run([sys.executable,'-m','collector.camera_inventory','--probe'],
                    cwd=str(self.management.root),capture_output=True,text=True,timeout=8)
                line=next((s for s in r.stdout.splitlines() if s.startswith('INVENTORY_JSON=')),None)
                if r.returncode or not line:raise ValueError('SDK 枚举失败，请检查驱动或使用手动配置。')
                result=json.loads(line.split('=',1)[1]);result['scanned_at']=now()
                with self.lock:self.cache=result;self.last=time.monotonic()
            except Exception as exc:
                with self.lock:self.cache.update(errors=[str(exc)],scanned_at=now());self.last=time.monotonic()
            finally:
                with self.lock:self.scanning=False
        self.thread=threading.Thread(target=scan,daemon=True);self.thread.start();return self.view()

    def bind(self,b):
        # Same lock order as settings writes; never closes robot or existing cameras.
        with self.management.lock,self.app.lock,self.lock:
            self.management._idle()
            if self.scanning or not self.last or time.monotonic()-self.last>45:raise ValueError('请刷新发现列表后再添加。')
            cfg,revision=self.management._disk_config()
            if b.get('revision')!=revision:raise ValueError('设置已变化，请刷新。')
            old=camera_bindings(self.app.config)
            if camera_bindings(cfg)!=old:raise ValueError('已有未应用相机设置，请先应用或还原后添加。')
            device=next((d for d in self.cache['devices'] if d['serial']==b.get('serial') and d['driver']==b.get('driver')),None)
            if device is None:raise ValueError('未发现该相机。')
            key=b.get('key')
            if not isinstance(key,str) or not re.fullmatch(r'[a-z][a-z0-9_]{0,63}',key):raise ValueError('相机标识须以小写字母开头，仅含小写字母、数字和下划线。')
            label=text(b.get('label',''),80,'相机名称',True)
            if key in old:raise ValueError('相机标识已存在。')
            profile=b.get('profile')
            if profile not in device['profiles']:raise ValueError('请选择设备实际支持的档位。')
            if profile['fps']<self.app.config['fps']:raise ValueError('相机帧率低于当前采集频率，请选更高档位或调整采样率。')
            binding=dict(profile,driver=device['driver'],serial=device['serial'],label=label,use_sdk_host_arrival=True)
            desired=camera_bindings({'camera_bindings':dict(old,**{key:binding})})
            if not hasattr(self.app.cameras,'bindings') or not hasattr(self.app.cameras,'errors'):raise ValueError('相机管理器尚未就绪，请应用设置恢复。')
            cfg['camera_bindings']=desired
            atomic_json(self.management.directory/'settings_backups'/(uuid.uuid4().hex+'.json'),self.management._disk_config()[0])
            atomic_json(self.management.config_path,cfg)
            self.app.config['camera_bindings']=copy.deepcopy(desired)
            self.app.cameras.bindings=copy.deepcopy(desired);self.app.cameras.errors[key]='相机已释放'
            self.app.config['camera_shapes'][key]=[profile['height'],profile['width'],3]
            self.app.config['camera_native_fps'][key]=profile['fps']
            self.app.config['device_sources'][key]='orbbec_rgb' if device['driver']=='orbbec' else 'realsense_rgb'
            self.app._save_camera_state([key],True)
            return dict(added=key,message='已添加。勾选录制后，开始下一轮时自动启用。',settings=self.management.settings())


if __name__=='__main__':
    print('INVENTORY_JSON='+json.dumps(enumerate_devices(),ensure_ascii=True))
