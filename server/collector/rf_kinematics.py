"""Offline FK of an explicitly provisional CAD model; no robot I/O or control.

Direct inputs use model-axis radians. Normalized observations require an explicit
inverse normalization and an opt-in to provisional FK. No robot I/O or control;
this module is not connected to Collector and cannot certify TCP calibration.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np


DEFAULT_MODEL = Path(__file__).with_name('rf_kinematics_candidate.json')
DEFAULT_MAPPING = Path(__file__).with_name('rf_observation_mapping.json')


def vector(value, count, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (count,) or not np.isfinite(result).all():
        raise ValueError(f'{name} must contain {count} finite values.')
    return result


def axis_rotation(axis, angle):
    axis = vector(axis, 3, 'axis')
    norm = np.linalg.norm(axis)
    if norm < 1e-12 or not np.isfinite(angle):
        raise ValueError('Nonzero axis and finite angle required.')
    x, y, z = axis / norm
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle)*skew + (1-np.cos(angle))*(skew@skew)


def transform(xyz, rpy):
    roll, pitch, yaw = vector(rpy, 3, 'rpy')
    result = np.eye(4)
    result[:3, :3] = (axis_rotation([0, 0, 1], yaw)
                       @ axis_rotation([0, 1, 0], pitch)
                       @ axis_rotation([1, 0, 0], roll))
    result[:3, 3] = vector(xyz, 3, 'translation')
    return result


def quaternion_wxyz(rotation):
    """Convert a proper rotation, including 180 degree orientations."""
    r = np.asarray(rotation, dtype=float)
    if (r.shape != (3, 3) or not np.isfinite(r).all()
            or not np.allclose(r.T@r, np.eye(3), atol=1e-8)
            or not np.isclose(np.linalg.det(r), 1, atol=1e-8)):
        raise ValueError('Expected a proper rotation matrix.')
    trace = np.trace(r)
    if trace > 0:
        scale = 2*np.sqrt(trace+1)
        q = np.array([scale/4, (r[2, 1]-r[1, 2])/scale,
                      (r[0, 2]-r[2, 0])/scale, (r[1, 0]-r[0, 1])/scale])
    else:
        i = int(np.argmax(np.diag(r)))
        j, k = (i+1) % 3, (i+2) % 3
        scale = 2*np.sqrt(max(0, 1+r[i, i]-r[j, j]-r[k, k]))
        q = np.zeros(4)
        q[0] = (r[k, j]-r[j, k])/scale
        q[i+1] = scale/4
        q[j+1] = (r[i, j]+r[j, i])/scale
        q[k+1] = (r[i, k]+r[k, i])/scale
    q /= np.linalg.norm(q)
    if q[0] < 0:
        q = -q
    return q


class CandidateKinematics:
    def __init__(self, path=DEFAULT_MODEL, mapping_path=DEFAULT_MAPPING):
        self.model = json.loads(Path(path).read_text(encoding='utf-8'))
        self.mapping = json.loads(Path(mapping_path).read_text(encoding='utf-8'))
        self._chain_cache = {}
        self._tcp_cache = None
        if self.model['angle_unit'] != 'rad' or self.model['position_unit'] != 'm':
            raise ValueError('This solver requires radians and meters.')

    def _fixed_chain(self, side, arm):
        # Geometry is constant across samples. Rebuild only when a caller edits
        # the model; no repeated RPY matrices or axis normalization at 50Hz.
        key = tuple((tuple(j['origin_m']), tuple(j['origin_rpy_rad']), tuple(j['axis']))
                    for j in arm['joints'])
        cached = self._chain_cache.get(side)
        if cached is None or cached[0] != key:
            chain = []
            for xyz, rpy, axis in key:
                v = vector(axis, 3, 'axis')
                norm = np.linalg.norm(v)
                if norm < 1e-12:
                    raise ValueError('Nonzero joint axis required.')
                x, y, z = v/norm
                skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
                chain.append((transform(xyz, rpy), skew, skew@skew))
            cached = (key, chain)
            self._chain_cache[side] = cached
        tcp_key = (tuple(self.model['tcp_translation_m']), tuple(self.model['tcp_rpy_rad']))
        if self._tcp_cache is None or self._tcp_cache[0] != tcp_key:
            self._tcp_cache = (tcp_key, transform(*tcp_key))
        return cached[1], self._tcp_cache[1]

    def forward(self, side, model_angles_rad):
        if side not in ('left', 'right'):
            raise ValueError('Side must be left or right.')
        angles = vector(model_angles_rad, 5, 'model joint angles')
        arm = self.model['arms'][side]
        if len(arm['joints']) != 5:
            raise ValueError('Expected exactly five arm joints.')
        pose = np.eye(4)
        joint_origins = []
        chain, tcp = self._fixed_chain(side, arm)
        for (fixed, skew, skew_squared), angle in zip(chain, angles):
            pose = pose @ fixed
            joint_origins.append(pose[:3, 3].tolist())
            rotation = np.eye(4)
            rotation[:3, :3] = np.eye(3) + math.sin(angle)*skew + (1-math.cos(angle))*skew_squared
            pose = pose @ rotation
        pose = pose @ tcp
        return {
            'model_id': self.model['model_id'], 'side': side,
            'reference_frame': self.model['reference_frame'],
            'input_semantics': 'model_axis_angles_rad_not_raw_encoder_values',
            'position_unit': 'm', 'quaternion_order': 'wxyz',
            'position_m': pose[:3, 3].tolist(),
            'quaternion_wxyz': quaternion_wxyz(pose[:3, :3]).tolist(),
            'transform': pose.tolist(), 'joint_origins_m': joint_origins,
            'provenance': 'offline_unvalidated_CAD_candidate_forward_kinematics',
            'physical_calibration_verified': False,
            'training_ready': False,
            'assumptions': self.model['assumptions'],
        }

    def angles_from_observation(self, observation, *, conversion_parameters=None):
        """Undo affine normalization, then map user motion signs to model axes.

        Each side requires five positive scale_rad_per_raw values and five
        zero_raw values, in arm_joint_order. zero_raw is the normalized reading
        at MODEL zero, not necessarily the normalized midpoint or encoder zero.
        Unknown parameters are never inferred from value range or static data.
        Caller overrides are provisional parameters, never a calibration claim.
        """
        raw = vector(observation, 96, 'raw observation')
        if conversion_parameters is not None and (
                not isinstance(conversion_parameters, dict)
                or set(conversion_parameters) != {'left', 'right'}):
            raise ValueError('Conversion parameters must contain left and right.')
        result = {}
        for side in ('left', 'right'):
            arm = self.model['arms'][side]
            mapping = self.mapping[side + '_arm']
            profile = arm['encoder_to_model']
            if not isinstance(profile, dict):
                raise RuntimeError('Encoder conversion profile is missing.')
            indices = arm['obs_indices']
            if (arm['motor_ids'] != mapping['axis_ids']
                    or indices != mapping['position_obs_indices']
                    or [j['name'] for j in arm['joints']] != self.mapping['arm_joint_order']
                    or [j['axis'] for j in arm['joints']] != profile['model_axes']
                    or any(type(i) is not int or not 0 <= i < 20 for i in indices)
                    or len(set(indices)) != 5
                    or [self.mapping['axis_ids_by_slot'][i] for i in indices] != arm['motor_ids']):
                raise ValueError('Joint mapping/model axes changed; review conversion profile.')
            raw_signs = vector([
                self.mapping['joint_motion_directions']['by_motor_id'][str(i)]['sign']
                for i in arm['motor_ids']], 5, 'user motion signs')
            model_signs = vector(profile['model_sign_for_named_motion'], 5, 'model motion signs')
            signs = vector(profile['sign'], 5, 'encoder-to-model signs')
            if (not np.isin(raw_signs, [-1, 1]).all()
                    or not np.isin(model_signs, [-1, 1]).all()
                    or not np.array_equal(signs, raw_signs * model_signs)):
                raise ValueError('Conversion signs disagree with named motion definitions.')
            params = profile if conversion_parameters is None else conversion_parameters[side]
            if not isinstance(params, dict):
                raise ValueError('Each arm requires a conversion parameter object.')
            if params.get('scale_rad_per_raw') is None or params.get('zero_raw') is None:
                raise RuntimeError('Normalized feedback requires scale_rad_per_raw and zero_raw for each arm; normalization rule is unknown.')
            scale = vector(params['scale_rad_per_raw'], 5, 'scale_rad_per_raw')
            zero = vector(params['zero_raw'], 5, 'zero_raw')
            if not (scale > 0).all():
                raise ValueError('Normalization scales must be positive; direction is encoded separately.')
            if (raw[indices] < -1).any() or (raw[indices] > 1).any():
                raise ValueError('Joint position exceeds documented normalized range [-1, 1].')
            with np.errstate(over='ignore', invalid='ignore'):
                encoder_angles = vector(raw[indices] * scale, 5, 'encoder angles')
                angles = vector(signs * scale * (raw[indices] - zero), 5, 'converted model angles')
            result[side] = {
                'motor_ids': list(arm['motor_ids']), 'obs_indices': list(indices),
                'raw_joint_values': raw[indices].tolist(),
                'encoder_angles_rad': encoder_angles.tolist(),
                'model_angles_rad': angles.tolist(),
                'sign': signs.tolist(), 'scale_rad_per_raw': scale.tolist(),
                'zero_raw': zero.tolist(), 'conversion_version': profile['conversion_version'],
                'parameter_source': ('caller_supplied_provisional' if conversion_parameters is not None
                                     else 'model_profile'),
                'normalization_rule_documented': (conversion_parameters is None
                                                  and profile.get('normalization_rule_documented', False)),
                'normalization_source': (profile.get('normalization_source')
                                         if conversion_parameters is None else None),
                'zero_policy': (profile.get('zero_policy') if conversion_parameters is None
                                else 'caller_supplied_provisional'),
                'zero_calibration_verified': (conversion_parameters is None
                                             and profile.get('zero_calibration_verified', False)),
                'zero_calibration_scope': (profile.get('zero_calibration_scope')
                                           if conversion_parameters is None else None),
                'zero_calibration_source': (profile.get('zero_calibration_source')
                                            if conversion_parameters is None else None),
                'normalization_verified': False,
                'physical_calibration_verified': False,
            }
        return result

    def forward_from_observation(self, observation, *, conversion_parameters=None,
                                 allow_provisional=False):
        """Return both candidate TCP poses; never label them measured or calibrated.

        This offline function creates no sample timestamp: the acquisition layer
        must retain the actual receive time and unknown device measurement time.
        """
        raw = vector(observation, 96, 'raw observation')
        if allow_provisional is not True:
            raise RuntimeError('Physical calibration is unverified; offline trial requires allow_provisional=True and explicit normalization parameters.')
        converted = self.angles_from_observation(raw, conversion_parameters=conversion_parameters)
        poses = {}
        for side in ('left', 'right'):
            poses[side] = self.forward(side, converted[side]['model_angles_rad'])
            poses[side]['input_semantics'] = 'normalized_feedback_with_explicit_inverse_normalization'
            poses[side]['conversion'] = converted[side]
        return {
            'model_id': self.model['model_id'], 'mapping_version': self.mapping['mapping_version'],
            'reference_frame': self.model['reference_frame'],
            'position_unit': 'm', 'quaternion_order': 'wxyz',
            'raw_observation': raw.tolist(),
            'left': poses['left'], 'right': poses['right'],
            'ee_pose': [value for side in ('left', 'right')
                        for value in poses[side]['position_m'] + poses[side]['quaternion_wxyz']],
            'provenance': 'offline_candidate_FK_from_normalized_observation',
            'normalization_rule_documented': all(converted[s]['normalization_rule_documented']
                                                  for s in ('left', 'right')),
            'zero_reference_calibrated': all(converted[s]['zero_calibration_verified']
                                             for s in ('left', 'right')),
            'normalization_verified': False, 'physical_calibration_verified': False,
            'training_ready': False,
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--left', nargs=5, type=float, default=[0]*5, metavar='RAD')
    parser.add_argument('--right', nargs=5, type=float, default=[0]*5, metavar='RAD')
    parser.add_argument('--observation-json', type=Path, help='JSON array of 96 normalized feedback values; offline only.')
    parser.add_argument('--allow-provisional', action='store_true', help='Explicitly accept uncalibrated candidate FK for offline inspection.')
    args = parser.parse_args()
    solver = CandidateKinematics()
    if args.observation_json:
        observation = json.loads(args.observation_json.read_text(encoding='utf-8'))
        result = solver.forward_from_observation(observation, allow_provisional=args.allow_provisional)
    else:
        result = {side: solver.forward(side, getattr(args, side)) for side in ('left', 'right')}
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
