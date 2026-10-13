"""Real Orbbec RGB plus serial-bound RealSense or simulated wrist cameras.

The device timestamp is preserved separately. Camera exposure clock calibration
has not been performed: the common host timestamp is RECEIVE time, not exposure.
"""
from collections import deque
import threading
import time
from .clock import monotonic, monotonic_ns

import numpy as np

from .adapters import CameraFrame
from .camera_diagnostics import CameraDiagnostics, optional_metadata, latest_ready_frame
from .host_arrival import HostArrivalBridge, clock_sample


class OrbbecRGB:
    def __init__(self,serial,width=1280,height=720,fps=60,on_update=None,use_sdk_host_arrival=False):
        from pyorbbecsdk import Context,Pipeline,Config,OBSensorType,OBFormat
        self.context=Context()
        devices=self.context.query_devices()
        matches=[devices.get_device_by_index(i) for i in range(devices.get_count())
                 if devices.get_device_by_index(i).get_device_info().get_serial_number()==serial]
        if len(matches)!=1: raise RuntimeError('Configured Orbbec serial number was not found uniquely.')
        self.device=matches[0]; self.serial=serial
        self.on_update=on_update
        self.pipeline=Pipeline(self.device)
        profiles=self.pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        self.profile=profiles.get_video_stream_profile(width,height,OBFormat.MJPG,fps)
        self.width,self.height=self.profile.get_width(),self.profile.get_height()
        self.format=self.profile.get_format()
        if self.format!=OBFormat.MJPG:
            raise RuntimeError('This Orbbec integration expects the verified native MJPG profile.')
        cfg=Config(); cfg.enable_stream(self.profile)
        self.lock=threading.Lock(); self.frames=deque(maxlen=20)
        self.error=None; self.fault=False; self.stop_event=threading.Event()
        self.last_device_stamp=None
        self.pending_frame=None;self.pending_wait_requests=0
        self.diagnostics=CameraDiagnostics()
        self.arrival_bridge=HostArrivalBridge('orbbec_system_timestamp_us',self.diagnostics.session_id) if use_sdk_host_arrival else None
        self.pipeline.start(cfg)
        self.thread=threading.Thread(target=self._run,name='orbbec-rgb',daemon=True)
        self.thread.start()

    def _run(self):
        from pyorbbecsdk import OBFrameMetadataType
        try:
            while not self.stop_event.is_set():
                fs=self.pipeline.wait_for_frames(500)
                if fs is None: continue
                color=fs.get_color_frame()
                if color is None or self.fault: continue
                receipt_ns=monotonic_ns()
                bridge_sample=clock_sample() if self.arrival_bridge else None
                frame_id=color.get_index()
                with self.lock:self.pending_frame=(frame_id,receipt_ns)
                if self.on_update:self.on_update(('pending',self.pending_frame))
                device_us=color.get_timestamp_us()
                stamp=device_us/1000
                if self.last_device_stamp is not None and stamp<=self.last_device_stamp:
                    raise RuntimeError('Orbbec device timestamp reset/repeated/out of order.')
                self.last_device_stamp=stamp
                metadata=optional_metadata(('TIMESTAMP','SENSOR_TIMESTAMP','EXPOSURE','ACTUAL_FRAME_RATE'),
                    lambda name:color.has_metadata(getattr(OBFrameMetadataType,name)),
                    lambda name:color.get_metadata_value(getattr(OBFrameMetadataType,name)))
                metadata.update(source='orbbec_frame_metadata',device_timestamp_us=device_us,
                                units='sdk_native_unverified',sensor_timestamp_semantics='unverified')
                sdk_arrival_ns=None
                if self.arrival_bridge:
                    system_us=color.get_system_timestamp_us()
                    sdk_arrival_ns,proof=self.arrival_bridge.map(system_us,receipt_ns,bridge_sample)
                    metadata.update(system_timestamp_us=system_us,host_arrival_proof=proof)
                    with self.lock:self.pending_frame=(frame_id,sdk_arrival_ns)
                    if self.on_update:self.on_update(('pending',self.pending_frame))
                metadata_done_ns=monotonic_ns()
                jpeg=np.asarray(color.get_data()).view(np.uint8).reshape(-1).tobytes()
                ready_ns=monotonic_ns()
                frame=CameraFrame('head',frame_id,receipt_ns,receipt_ns,jpeg,
                                  source='orbbec_rgb',serial=self.serial,
                                  device_timestamp_ms=stamp,
                                  device_clock_domain='orbbec_device_clock_unverified',
                                  ready_ns=ready_ns,camera_session=self.diagnostics.session_id,timing_metadata=metadata,
                                  sdk_arrival_host_ns=sdk_arrival_ns,
                                  timestamp_basis='sdk_host_arrival_unmapped_exposure' if sdk_arrival_ns is not None else 'host_receive_unmapped_device_clock')
                with self.lock:
                    self.frames.append(frame);self.pending_frame=None
                stages={'metadata':(metadata_done_ns-receipt_ns)/1e6,
                    'jpeg_copy':(ready_ns-metadata_done_ns)/1e6}
                self.diagnostics.record(frame,stages)
                if self.on_update:self.on_update(('frame',frame,stages))
        except Exception as e:
            self.error=f'{type(e).__name__}: {e}'

    def latest(self,at_ns=None):
        with self.lock:
            result=latest_ready_frame(self.frames,at_ns,self.pending_frame)
            if result is None and at_ns is not None and self.pending_frame is not None and self.pending_frame[1]<=at_ns:
                self.pending_wait_requests+=1
            return result

    def health_check(self):
        f=self.latest(); now=monotonic_ns()
        age=(now-f.received_ns)/1e6 if f else None
        return {'healthy':not self.error and age is not None and age<100,
                'mock':False,'source':'orbbec_rgb','serial':self.serial,
                'width':self.width,'height':self.height,'native_fps':self.profile.get_fps(),
                'age_ms':round(age,3) if age is not None else None,
                'timestamp_basis':f.timestamp_basis if f else 'unavailable','error':self.error}

    def supported_profiles(self):
        from pyorbbecsdk import OBSensorType
        profiles=self.pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        output=[]
        for i in range(profiles.get_count()):
            p=profiles.get_stream_profile_by_index(i).as_video_stream_profile()
            output.append({'width':p.get_width(),'height':p.get_height(),
                           'fps':p.get_fps(),'format':str(p.get_format())})
        return output

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=2)
        self.pipeline.stop()


class RealCameras:
    def __init__(self,robot,config):
        from .platform_contract import camera_bindings
        from .isolated_camera import IsolatedCamera
        self.bindings=camera_bindings(config)
        self.isolated=config.get('camera_process_isolation',False)
        self.devices={};self.errors={}
        released=set(config.get('released_cameras',[]))
        for key in self.bindings:
            if key in released:self.errors[key]='相机已释放'
            else:self.connect(key)
        self._aliases()

    def _aliases(self):
        self.head=self.devices.get('head')
        self.real_wrists={k:v for k,v in self.devices.items() if k!='head'}

    def connect(self,key):
        if key not in self.bindings:raise ValueError('未知相机。')
        if key in self.devices:
            if self.devices[key].health_check()['healthy']:return
            self.release(key)
        from .isolated_camera import IsolatedCamera
        settings=dict(self.bindings[key]);driver=settings.pop('driver');settings.pop('label',None)
        try:
            if driver=='orbbec':
                camera=IsolatedCamera('head',settings) if self.isolated else OrbbecRGB(**settings)
            else:
                from .realsense_camera import RealSenseRGB
                camera=IsolatedCamera('realsense',dict(key=key,**settings)) if self.isolated else RealSenseRGB(key,**settings)
            self.devices[key]=camera;self.errors.pop(key,None)
        except Exception as exc:
            self.errors[key]=f'{type(exc).__name__}: {exc}'
        self._aliases()

    def release(self,key):
        if key not in self.bindings:raise ValueError('未知相机。')
        camera=self.devices.get(key)
        if camera is not None:
            camera.close()
            del self.devices[key]
        self.errors[key]='相机已释放'
        self._aliases()

    def latest(self,key,at_ns=None):
        from dataclasses import replace
        camera=self.devices.get(key)
        frame=camera.latest(at_ns) if camera else None
        return replace(frame,key=key) if frame and frame.key!=key else frame

    def candidates(self,key,at_ns):
        from .camera_diagnostics import ready_frame_candidates
        camera=self.devices.get(key)
        if camera is None:
            return []
        with camera.lock:
            frames=ready_frame_candidates(camera.frames,at_ns,camera.pending_frame)
            if not frames and camera.pending_frame is not None and camera.pending_frame[1]<=at_ns:
                camera.pending_wait_requests+=1
            from dataclasses import replace
            return [replace(f,key=key) if f.key!=key else f for f in frames]

    def health_check(self):
        from .unavailable_camera import unavailable_health
        result={}
        for k,v in self.bindings.items():
            device=self.devices.get(k)
            result[k]=device.health_check() if device else unavailable_health(v,self.errors.get(k,'相机正在连接'))
        return result

    def timing_diagnostics(self):
        return {key:dict(camera.diagnostics.snapshot(),pending_wait_requests=camera.pending_wait_requests)
                for key,camera in self.devices.items()}

    def failure_trace(self):
        return {key:camera.diagnostics.recent_frames() for key,camera in self.devices.items()}

    def close(self):
        for key in list(self.devices):self.release(key)
