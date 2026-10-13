"""Serial-bound RealSense RGB; original device time and host receipt are separate."""
from collections import deque
import threading
import time
from .clock import monotonic, monotonic_ns

import numpy as np

from .adapters import CameraFrame
from .camera_diagnostics import CameraDiagnostics, optional_metadata, latest_ready_frame
from .jpeg_encoder import RGBJPEGEncoder, configure_frame_queue, restore_frame_queue
from .host_arrival import HostArrivalBridge, clock_sample


def queued_arrival_expired(raw, sample):
    # SDK initialization of a different camera can suspend this Python thread.
    # Drop only plausible, expired host arrivals; do not remap them as new data.
    return type(raw) is int and 250_000_000 < sample['wall_ns']-raw*1_000_000 <= 5_000_000_000


def yuyv_to_rgb(data,width,height):
    """Unpack the SDK's byte/uint16 view as Y0 U Y1 V; OpenCV emits RGB."""
    import cv2
    raw=np.asanyarray(data)
    if raw.dtype not in (np.dtype('uint8'),np.dtype('uint16')) or raw.nbytes!=height*width*2 or width%2:
        raise ValueError('Invalid packed YUYV frame layout.')
    return cv2.cvtColor(raw.view(np.uint8).reshape(height,width,2),cv2.COLOR_YUV2RGB_YUY2)


class RealSenseRGB:
    def __init__(self, key, serial, width=424, height=240, fps=60, jpeg_backend='simplejpeg', sdk_queue_size=16, on_update=None, stream_format='rgb8', conversion_threads=None, use_sdk_host_arrival=False):
        import pyrealsense2 as rs
        if stream_format not in ('rgb8','yuyv'):raise ValueError('Unsupported RealSense stream format.')
        if conversion_threads is not None and (type(conversion_threads) is not int or not 1<=conversion_threads<=32):
            raise ValueError('Conversion threads must be 1..32.')
        self.stream_format=stream_format
        if stream_format=='yuyv':
            import cv2  # Load the converter before the SDK starts delivering frames.
        self.conversion_threads=conversion_threads
        self.previous_threads=None
        self.key, self.serial = key, str(serial)
        self.on_update=on_update
        self.width, self.height, self.fps = width, height, fps
        self.lock = threading.Lock()
        self.frames = deque(maxlen=30)
        self.error = None
        self.fault = False
        self.stop_event = threading.Event()
        self.last_device_stamp = None
        self.last_domain = None
        self.pending_frame = None
        self.pending_wait_requests = 0
        self.expired_delivery_count = 0
        self.diagnostics = CameraDiagnostics()
        self.arrival_bridge=HostArrivalBridge('realsense_time_of_arrival_ms',self.diagnostics.session_id) if use_sdk_host_arrival else None
        self.encoder=RGBJPEGEncoder(jpeg_backend)
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(self.serial)
        config.enable_stream(rs.stream.color, width, height, getattr(rs.format,stream_format), fps)
        device=config.resolve(self.pipeline).get_device()
        self.queue_settings,self.queue_restore=configure_frame_queue(device,rs,sdk_queue_size)
        # The pipeline's default wait_for_frames queue retains only the latest
        # frameset. A bounded explicit queue keeps arrivals during JPEG work or
        # scheduling jitter; alignment still rejects stale frames, never retimes.
        self.frame_queue=rs.frame_queue(sdk_queue_size,keep_frames=True)
        try:self.pipeline.start(config,self.frame_queue)
        except Exception:
            restore_frame_queue(self.queue_restore,rs);raise
        if stream_format=='yuyv' and conversion_threads is not None:
            import cv2
            self.previous_threads=cv2.getNumThreads()
            cv2.setNumThreads(conversion_threads)
        self.thread = threading.Thread(target=self._run, name=key+'-realsense', daemon=True)
        self.thread.start()

    def _run(self):
        import pyrealsense2 as rs
        try:
            while not self.stop_event.is_set():
                frames = self.frame_queue.wait_for_frame(1000).as_frameset()
                color = frames.get_color_frame()
                if not color or self.fault:
                    continue
                receipt_ns = monotonic_ns()
                bridge_sample=clock_sample() if self.arrival_bridge else None
                system_receipt_ms=time.time_ns()/1e6
                frame_id=color.get_frame_number()
                with self.lock:self.pending_frame=(frame_id,receipt_ns)
                if self.on_update:self.on_update(('pending',self.pending_frame))
                stamp = color.get_timestamp()
                domain = str(color.get_frame_timestamp_domain())
                if self.last_device_stamp is not None and (stamp<=self.last_device_stamp or domain!=self.last_domain):
                    raise RuntimeError('RealSense device clock reset/changed/out of order.')
                self.last_device_stamp, self.last_domain = stamp, domain
                def supports(name):
                    field = getattr(rs.frame_metadata_value, name, None)
                    return field is not None and color.supports_frame_metadata(field)
                metadata=optional_metadata(('actual_exposure','sensor_timestamp','frame_timestamp','time_of_arrival','backend_timestamp'),
                    supports,lambda name:color.get_frame_metadata(getattr(rs.frame_metadata_value,name)))
                metadata['source']='librealsense_frame_metadata'
                metadata['units']={'actual_exposure':'us','sensor_timestamp':'us','frame_timestamp':'us',
                                   'time_of_arrival':'host_epoch_ms','backend_timestamp':'sdk_native'}
                metadata['sensor_timestamp_semantics']='device_calculated_exposure_midpoint_if_present'
                metadata['host_receive_system_ms']=system_receipt_ms
                metadata['jpeg']=self.encoder.settings
                metadata['sdk_queue']=self.queue_settings
                metadata['delivery_queue']={'mode':'explicit_frameset_fifo','capacity':self.frame_queue.capacity()}
                metadata['stream_format']=self.stream_format
                metadata['conversion_threads']=self.conversion_threads
                metadata['rgb_conversion']='opencv_yuyv_bt601_limited_to_rgb' if self.stream_format=='yuyv' else 'sdk_rgb8'
                arrival=metadata['values'].get('time_of_arrival')
                sdk_arrival_ns=None
                if self.arrival_bridge:
                    if queued_arrival_expired(arrival,bridge_sample):
                        self.expired_delivery_count+=1
                        with self.lock:self.pending_frame=None
                        continue
                    sdk_arrival_ns,proof=self.arrival_bridge.map(arrival,receipt_ns,bridge_sample)
                    metadata['host_arrival_proof']=proof
                    with self.lock:self.pending_frame=(frame_id,sdk_arrival_ns)
                    if self.on_update:self.on_update(('pending',self.pending_frame))
                # TIME_OF_ARRIVAL is the SDK system-clock arrival, not exposure or USB start.
                # This wall-clock difference is diagnostic; alignment uses the checked
                # conservative monotonic bridge above, never this floating-point value.
                delivery_ms=system_receipt_ms-arrival if arrival is not None else None
                if delivery_ms is not None and not 0<=delivery_ms<=1000:delivery_ms=None
                metadata_done_ns=monotonic_ns()
                rgb = np.asanyarray(color.get_data())
                if self.stream_format=='yuyv':rgb=yuyv_to_rgb(rgb,self.width,self.height)
                converted_ns=monotonic_ns()
                jpeg_bytes,encoding_stages=self.encoder.encode(rgb)
                ready_ns=monotonic_ns()
                frame = CameraFrame(self.key, frame_id, receipt_ns,
                    receipt_ns, jpeg_bytes, source='realsense_rgb', serial=self.serial,
                    device_timestamp_ms=stamp, device_clock_domain=domain,
                    exposure_us=metadata['values'].get('actual_exposure'),
                    sensor_timestamp_raw=metadata['values'].get('sensor_timestamp'),
                    ready_ns=ready_ns,camera_session=self.diagnostics.session_id,timing_metadata=metadata,
                    sdk_arrival_host_ns=sdk_arrival_ns,
                    timestamp_basis='sdk_host_arrival_unmapped_exposure' if sdk_arrival_ns is not None else 'host_receive_unmapped_device_clock')
                with self.lock:
                    self.frames.append(frame)
                    self.pending_frame=None
                stages={'metadata':(metadata_done_ns-receipt_ns)/1e6,
                        'native_color_conversion':(converted_ns-metadata_done_ns)/1e6,**encoding_stages}
                if delivery_ms is not None:stages['sdk_arrival_to_app']=delivery_ms
                if delivery_ms is not None:stages['sdk_arrival_to_ready']=delivery_ms+(ready_ns-receipt_ns)/1e6
                self.diagnostics.record(frame,stages)
                if self.on_update:self.on_update(('frame',frame,stages))
        except Exception as exc:
            if not self.stop_event.is_set():
                self.error = f'{type(exc).__name__}: {exc}'

    def latest(self, at_ns=None):
        with self.lock:
            result=latest_ready_frame(self.frames,at_ns,self.pending_frame)
            if result is None and at_ns is not None and self.pending_frame is not None and self.pending_frame[1]<=at_ns:
                self.pending_wait_requests+=1
            return result

    def health_check(self):
        frame = self.latest()
        age = (monotonic_ns()-frame.received_ns)/1e6 if frame else None
        return {'healthy':not self.error and age is not None and age<100,
            'mock':False, 'source':'realsense_rgb', 'model':'Intel RealSense',
            'serial':self.serial, 'width':self.width, 'height':self.height,
            'native_fps':self.fps, 'age_ms':round(age,3) if age is not None else None,
            'timestamp_basis':frame.timestamp_basis if frame else 'unavailable', 'error':self.error}

    def close(self):
        import pyrealsense2 as rs
        self.stop_event.set()
        self.thread.join(timeout=2)
        self.pipeline.stop()
        restore_frame_queue(self.queue_restore,rs)
        if self.previous_threads is not None:
            import cv2
            cv2.setNumThreads(self.previous_threads)
