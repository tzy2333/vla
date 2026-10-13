"""Bridge documented SDK HOST wall-clock arrivals to the host monotonic clock.

This is not a device/exposure calibration. Use a conservative upper bound for
selection, including timestamp quantization and clock-read uncertainty.
"""
import time
from .clock import monotonic_ns

SOURCES={'realsense_time_of_arrival_ms':1_000_000,'orbbec_system_timestamp_us':1_000}


def clock_sample():
    best=None
    for _ in range(3):
        before=monotonic_ns();wall=time.time_ns();after=monotonic_ns()
        sample={'before_ns':before,'wall_ns':wall,'after_ns':after}
        if best is None or after-before<best['after_ns']-best['before_ns']:best=sample
        if after-before<=1_000_000:break
    return best


def recompute_arrival(proof,received_ns):
    if proof.get('version')!=1 or proof.get('selection')!='conservative_arrival_upper_bound':
        raise ValueError('Unsupported SDK arrival proof.')
    unit=SOURCES.get(proof.get('source'))
    if unit is None or not proof.get('session'):raise ValueError('Unknown SDK host clock source.')
    raw=proof['raw'];s=proof['clock_sample']
    if type(raw) is not int or raw<=0:raise ValueError('Invalid SDK host timestamp.')
    before,wall,after=(s[k] for k in ('before_ns','wall_ns','after_ns'))
    if not all(type(v) is int for v in (before,wall,after,received_ns)) or not 0<=after-before<=1_000_000:
        raise ValueError('Host clock sampling interval invalid.')
    offset=(before+after)//2-wall
    if abs(offset-proof['anchor_offset_ns'])>2_000_000:raise ValueError('Host wall clock stepped; reopen cameras.')
    epoch=raw*unit
    if not -unit<=wall-epoch<=250_000_000:raise ValueError('SDK host timestamp outside verified clock domain.')
    upper=min(received_ns,epoch+after-wall+unit)
    if not 0<=received_ns-upper<=250_000_000:raise ValueError('Invalid SDK arrival ordering.')
    return upper


class HostArrivalBridge:
    def __init__(self,source,session):
        self.source,self.session=source,session
        self.anchor_offset_ns=None
        self.last_ns=None

    def map(self,raw,received_ns,sample):
        offset=(sample['before_ns']+sample['after_ns'])//2-sample['wall_ns']
        if self.anchor_offset_ns is None:self.anchor_offset_ns=offset
        proof={'version':1,'source':self.source,'session':self.session,'raw':raw,'clock_sample':sample,
               'anchor_offset_ns':self.anchor_offset_ns,'selection':'conservative_arrival_upper_bound'}
        upper=recompute_arrival(proof,received_ns)
        if self.last_ns is not None and upper<=self.last_ns:raise ValueError('SDK host arrival clock reset or repeated.')
        self.last_ns=upper
        return upper,proof


def validate_image_arrival(image):
    metadata=image['timing_metadata'];proof=metadata['host_arrival_proof']
    if proof.get('session')!=image['camera_session']:raise ValueError('SDK arrival session mismatch.')
    raw=(metadata['values'].get('time_of_arrival') if proof['source']=='realsense_time_of_arrival_ms'
         else metadata.get('system_timestamp_us'))
    if raw!=proof['raw']:raise ValueError('SDK arrival metadata was changed.')
    value=recompute_arrival(proof,image['received_ns'])
    if value!=image['sdk_arrival_host_ns'] or image['timestamp_basis']!='sdk_host_arrival_unmapped_exposure':
        raise ValueError('SDK host arrival provenance mismatch.')
    return value
