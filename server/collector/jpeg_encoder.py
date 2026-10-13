"""Explicit JPEG settings, RGB input, and separately measurable encode stages."""
import numpy as np
from .clock import monotonic_ns


class RGBJPEGEncoder:
    def __init__(self,backend='simplejpeg'):
        if backend not in ('opencv','simplejpeg'):raise ValueError('Unsupported JPEG encoder.')
        self.backend=backend
        if backend=='simplejpeg':
            import simplejpeg
            self.codec=simplejpeg
        else:
            import cv2
            self.codec=cv2
        self.settings={'backend':backend,'quality':90,'subsampling':'420','fast_dct':False}

    def encode(self,rgb):
        if rgb.dtype!=np.uint8 or rgb.ndim!=3 or rgb.shape[2]!=3:
            raise ValueError('JPEG encoder requires uint8 RGB HxWx3.')
        begin=monotonic_ns()
        if self.backend=='simplejpeg':
            rgb=np.ascontiguousarray(rgb)
            prepared=monotonic_ns()
            result=self.codec.encode_jpeg(rgb,quality=90,colorspace='RGB',colorsubsampling='420',fastdct=False)
        else:
            bgr=self.codec.cvtColor(rgb,self.codec.COLOR_RGB2BGR)
            prepared=monotonic_ns()
            ok,data=self.codec.imencode('.jpg',bgr,[self.codec.IMWRITE_JPEG_QUALITY,90])
            if not ok:raise RuntimeError('JPEG encoding failed.')
            result=data.tobytes()
        end=monotonic_ns()
        return result,{'color_conversion':(prepared-begin)/1e6,'jpeg_encoding':(end-prepared)/1e6}


def configure_frame_queue(device,rs,requested):
    """Limit the SDK frame pool without changing exposure, resolution or firmware."""
    if type(requested) is not int or not 1<=requested<=32:raise ValueError('SDK queue size must be 1..32.')
    results=[];restore=[]
    for sensor in device.query_sensors():
        if not sensor.supports(rs.option.frames_queue_size):continue
        item={'requested':requested}
        try:
            before=sensor.get_option(rs.option.frames_queue_size)
            restore.append((sensor,before))
            sensor.set_option(rs.option.frames_queue_size,requested)
            item.update(previous=before,actual=sensor.get_option(rs.option.frames_queue_size))
        except Exception as exc:item['error']=type(exc).__name__+': '+str(exc)[:160]
        results.append(item)
    return results,restore


def restore_frame_queue(settings,rs):
    for sensor,value in settings:
        try:sensor.set_option(rs.option.frames_queue_size,value)
        except Exception:pass  # Best-effort shutdown only; a disconnected camera cannot accept writes.
