"""Explicit missing hardware state, with no generated or cached image fallback."""
from .platform_contract import camera_bindings


def unavailable_health(binding, error):
    return dict(healthy=False, mock=False, source=binding['driver']+'_rgb', error=str(error),
                serial=binding['serial'], width=binding['width'], height=binding['height'],
                native_fps=binding['fps'], age_ms=None, timestamp_basis='unavailable',
                profile_is_configured_not_streaming=True)


class UnavailableCameras:
    def __init__(self, config, error):
        self.bindings=camera_bindings(config);self.error=str(error);self.head=None

    def health_check(self):
        return {k:unavailable_health(v,self.error) for k,v in self.bindings.items()}

    def latest(self,key,at_ns=None):return None
    def candidates(self,key,at_ns):return []
    def timing_diagnostics(self):return {key:{'sample_count':0,'error':self.error} for key in self.bindings}
    def failure_trace(self):return {}
    def close(self):pass
