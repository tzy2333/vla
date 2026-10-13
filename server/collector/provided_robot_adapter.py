"""Example binding for a future supplied robot SDK. Not enabled by run.py.

The provider must convert units, order, frame/TCP and gripper calibration before
returning this contract. Callback receipt timestamps use this host's monotonic clock.
"""
import time
from .clock import monotonic, monotonic_ns
import numpy as np
from .adapters import AcceptedCommand,RobotState


def finite_vector(value,length,name):
    a=np.asarray(value,dtype=np.float64)
    if a.shape!=(length,) or not np.isfinite(a).all():
        raise ValueError(f'{name} must contain {length} finite values.')
    return a.tolist()


class ProvidedRobotAdapter:
    is_mock=False

    def __init__(self,sdk):
        self.sdk=sdk
        self.is_mock=bool(getattr(sdk,'is_mock',False))

    def read_state(self):
        packet=self.sdk.read_actual_state()
        state=finite_vector(packet['state'],12,'actual state')
        if any(not 0<=state[i]<=1 for i in (5,11)):
            raise ValueError('Actual gripper feedback is not calibrated to [0,1].')
        # A cached feedback packet must retain its ORIGINAL receipt time.
        received_ns=int(packet['received_monotonic_ns'])
        if received_ns>monotonic_ns(): raise ValueError('Feedback clock mismatch.')
        ee=packet.get('actual_ee')
        if packet.get('ee_pose') is not None:
            pose=finite_vector(packet['ee_pose'],14,'actual TCP poses')
            from_pose=pose[:7]+[state[5]]+pose[7:]+[state[11]]
            if ee is not None and not np.allclose(ee,from_pose,atol=1e-6):
                raise ValueError('Actual TCP pose fields disagree.')
            ee=from_pose
        if ee is not None:
            ee=finite_vector(ee,16,'actual TCP/gripper diagnostic')
            for s in (3,11):
                if abs(np.linalg.norm(ee[s:s+4])-1)>1e-4:
                    raise ValueError('Actual TCP quaternion is not normalized.')
        ee_received_ns=int(packet.get('ee_received_monotonic_ns',received_ns)) if ee is not None else None
        if ee_received_ns is not None and ee_received_ns>monotonic_ns():
            raise ValueError('Actual TCP feedback clock mismatch.')
        return RobotState(state,ee,received_ns,packet.get('device_timestamp_ns'),
                          bool(packet['valid']),ee_received_ns,
                          packet.get('measurement_host_ns'), packet.get('ee_measurement_host_ns'),
                          packet.get('timestamp_basis','host_receive_unmapped_device_clock'),
                          packet.get('ee_timestamp_basis','host_receive_unmapped_device_clock'),
                          packet.get('clock_domain','unknown_device_clock'),
                          packet.get('clock_mapping_id'), packet.get('clock_uncertainty_ns'))

    def send_ee_target(self,action):
        requested=finite_vector(action,16,'requested action')
        requested_ns=monotonic_ns()
        packet=self.sdk.send_ee_target(requested)
        if not packet['accepted_ok']: raise RuntimeError('Robot controller rejected the target.')
        accepted=finite_vector(packet['accepted_action'],16,'accepted action')
        for s in (3,11):
            norm=np.linalg.norm(accepted[s:s+4])
            if abs(norm-1)>1e-4: raise ValueError('Accepted TCP quaternion must be normalized.')
        if any(not 0<=accepted[i]<=1 for i in (7,15)):
            raise ValueError('Accepted gripper target outside [0,1].')
        q=packet.get('joint_target')
        if q is not None: q=finite_vector(q,12,'accepted joint target')
        sent_ns=int(packet['sent_monotonic_ns'])
        if not requested_ns<=sent_ns<=monotonic_ns():
            raise ValueError('Command send time is inconsistent with this host clock.')
        accepted_ns=packet.get('accepted_monotonic_ns')
        effective_ns=packet.get('effective_monotonic_ns')
        if accepted_ns is not None and not sent_ns<=accepted_ns<=monotonic_ns():
            raise ValueError('Command acceptance time is inconsistent.')
        if effective_ns is not None and (accepted_ns is None or not accepted_ns<=effective_ns<=monotonic_ns()):
            raise ValueError('Confirmed effective time is inconsistent; planned execution must be separate.')
        return AcceptedCommand(str(packet['command_id']),requested,accepted,q,requested_ns,sent_ns,
                               True,accepted_ns,effective_ns,
                               packet.get('timing_basis','host_send_only_effect_unknown'))

    def health_check(self):
        return self.sdk.health_check()

    def close(self):
        self.sdk.close()
