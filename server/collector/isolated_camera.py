"""One SDK owner per spawned process; retain host timestamps across IPC.

Only JPEG bytes and bounded metadata cross the pipe. Pending notifications precede
encoding, so the parent can still wait for a causal frame within its usual budget.
"""
from collections import deque
from dataclasses import replace
import multiprocessing as mp
import os
import threading

from .clock import monotonic_ns
from .camera_diagnostics import CameraDiagnostics, latest_ready_frame


def camera_worker(kind, settings, connection, stop):
    camera = None
    parent_pid = os.getppid()
    send_lock = threading.Lock()
    try:
        def publish(event):
            with send_lock: connection.send(event)
        if kind == 'head':
            from .orbbec_camera import OrbbecRGB
            camera = OrbbecRGB(**settings, on_update=publish)
            publish(('profiles', camera.supported_profiles()))
        else:
            from .realsense_camera import RealSenseRGB
            camera = RealSenseRGB(**settings, on_update=publish)
        while not stop.wait(.05):
            if os.getppid()!=parent_pid: break
            if camera.error:
                raise RuntimeError(camera.error)
    except Exception as exc:
        try: publish(('error', f'{type(exc).__name__}: {exc}'))
        except (BrokenPipeError, EOFError, OSError): pass
    finally:
        if camera is not None: camera.close()
        connection.close()


class IsolatedCamera:
    def __init__(self, kind, settings, worker=camera_worker):
        self.kind, self.settings = kind, dict(settings)
        self.lock = threading.Lock()
        self.frames = deque(maxlen=30)
        self.pending_frame = None
        self.pending_wait_requests = 0
        self.error = None
        self.fault = False
        self.profiles = []
        self.diagnostics = CameraDiagnostics()
        self.closed = threading.Event()
        self.started = threading.Event()
        context = mp.get_context('spawn')
        self.stop = context.Event()
        self.connection, sender = context.Pipe(duplex=False)
        self.process = context.Process(target=worker, args=(kind, settings, sender, self.stop),
                                       name=kind+'-sdk', daemon=True)
        self.process.start()
        sender.close()
        self.reader = threading.Thread(target=self._receive, name=kind+'-ipc', daemon=True)
        self.reader.start()
        if not self.started.wait(12) or self.error:
            error = self.error or 'Camera process startup timed out.'
            self.close()
            raise RuntimeError(error)

    def _accept(self, event):
        kind, *payload = event
        if kind == 'pending':
            with self.lock: self.pending_frame = payload[0]
        elif kind == 'frame':
            frame, stages = payload
            available_ns = monotonic_ns()
            encode_ready_ns = frame.ready_ns
            metadata = dict(frame.timing_metadata or {})
            metadata['transport'] = {'mode':'separate_sdk_process', 'pid':self.process.pid,
                                     'encode_ready_ns':encode_ready_ns}
            frame = replace(frame, ready_ns=available_ns, timing_metadata=metadata,
                            camera_session=frame.camera_session or self.diagnostics.session_id)
            self.diagnostics.session_id=frame.camera_session
            with self.lock:
                if not self.fault: self.frames.append(frame)
                self.pending_frame = None
            self.diagnostics.record(frame, dict(stages, ipc_delivery=(available_ns-encode_ready_ns)/1e6))
            self.started.set()
        elif kind == 'profiles': self.profiles = payload[0]
        elif kind == 'error':
            self.error = payload[0]
            self.started.set()

    def _receive(self):
        try:
            # Drain until EOF even during shutdown: stopping the reader early can
            # leave the SDK worker blocked writing its final JPEG into the pipe.
            while True: self._accept(self.connection.recv())
        except (EOFError, OSError) as exc:
            if not self.closed.is_set():
                self.error = self.error or 'Camera worker disconnected: '+type(exc).__name__
                self.started.set()
        except Exception as exc:
            self.error = f'{type(exc).__name__}: {exc}'
            self.started.set()

    def latest(self, at_ns=None):
        with self.lock:
            result = latest_ready_frame(self.frames, at_ns, self.pending_frame)
            if result is None and at_ns is not None and self.pending_frame is not None and self.pending_frame[1]<=at_ns:
                self.pending_wait_requests += 1
            return result

    def supported_profiles(self): return self.profiles

    def health_check(self):
        frame = self.latest()
        age = (monotonic_ns()-frame.received_ns)/1e6 if frame else None
        return {'healthy':not self.error and self.process.is_alive() and age is not None and age<100,
                'mock':False, 'source':'orbbec_rgb' if self.kind=='head' else 'realsense_rgb',
                'serial':self.settings['serial'], 'width':self.settings['width'],
                'height':self.settings['height'], 'native_fps':self.settings['fps'],
                'age_ms':round(age,3) if age is not None else None,
                'timestamp_basis':'host_receive_unmapped_device_clock',
                'error':self.error, 'process_isolated':True, 'worker_pid':self.process.pid}

    def close(self):
        if self.closed.is_set(): return
        self.closed.set()
        self.stop.set()
        self.process.join(timeout=5)
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(timeout=3)
        self.reader.join(timeout=2)
        self.connection.close()
