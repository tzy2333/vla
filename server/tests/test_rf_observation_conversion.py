"""Supplier normalization and independent geometric checks; no robot I/O."""
import copy
import hashlib
import json
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from collector.rf_kinematics import CandidateKinematics


class ObservationConversionTests(unittest.TestCase):
    def setUp(self):
        self.fk = CandidateKinematics()
        # Synthetic geometric cases start from exact raw zero. The production
        # sampled offsets are checked separately against the archived packet data.
        for arm in self.fk.model['arms'].values():
            arm['encoder_to_model']['zero_raw'] = [0]*5
            arm['encoder_to_model']['zero_calibration_verified'] = False

    def pose(self, raw):
        return self.fk.forward_from_observation(raw, allow_provisional=True)

    def test_supplier_quarter_turn_and_one_pulse(self):
        raw = np.zeros(96)
        raw[1] = -.25  # ID12: negative raises forward, 1/4 revolution.
        raw[0] = -1/720000  # ID11: exactly one pulse.
        result = self.fk.angles_from_observation(raw)
        self.assertAlmostEqual(result['left']['encoder_angles_rad'][0], -np.pi/2)
        self.assertAlmostEqual(result['left']['model_angles_rad'][0], np.pi/2)
        self.assertAlmostEqual(result['right']['model_angles_rad'][0], 2*np.pi/720000)
        self.assertTrue(result['left']['normalization_rule_documented'])

    def test_all_ten_slots_and_signs_ignore_other_channels(self):
        raw = np.arange(96, dtype=float)/100
        original = raw.copy()
        result = self.fk.angles_from_observation(raw)
        np.testing.assert_allclose(result['left']['model_angles_rad'],
                                   2*np.pi*np.array([-.01, .03, -.05, -.07, -.09]))
        np.testing.assert_allclose(result['right']['model_angles_rad'],
                                   2*np.pi*np.array([0, -.02, .04, -.06, .08]))
        np.testing.assert_array_equal(raw, original)
        raw[10:] = 0
        self.assertEqual(result, self.fk.angles_from_observation(raw))

    def test_both_shoulders_negative_raw_raise_forward(self):
        raw = [0.0]*96
        raw[0] = raw[1] = -.25
        result = self.pose(raw)
        for side, y in [('left', .2), ('right', -.2)]:
            np.testing.assert_allclose(result[side]['position_m'], [.485, y, 0], atol=1e-12)

    def test_opposite_raw_side_swing_moves_both_arms_outward(self):
        raw = [0.0]*96
        raw[3], raw[2] = .25, -.25
        result = self.pose(raw)
        for side, y in [('left', .685), ('right', -.685)]:
            np.testing.assert_allclose(result[side]['position_m'], [0, y, 0], atol=1e-12)

    def test_negative_raw_elbows_flex_forward(self):
        raw = [0.0]*96
        raw[7] = raw[6] = -.25
        result = self.pose(raw)
        for side, y in [('left', .2), ('right', -.2)]:
            np.testing.assert_allclose(result[side]['position_m'], [.205, y, -.280], atol=1e-12)

    def test_bent_elbow_upper_arm_twist_is_outward(self):
        # Geometry: upper arm down .280m; bent forearm length .205m swings
        # from forward into +Y on the left and -Y on the right.
        raw = [0.0]*96
        raw[7] = raw[6] = -.25
        raw[5], raw[4] = .25, -.25  # User: ID24 positive, ID23 negative.
        result = self.pose(raw)
        for side, y in [('left', .405), ('right', -.405)]:
            np.testing.assert_allclose(result[side]['position_m'], [0, y, -.280], atol=1e-12)

    def test_wrist_outward_marker_rotates_without_moving_tcp(self):
        raw = [0.0]*96
        raw[9], raw[8] = .25, -.25  # User: ID18 positive, ID17 negative.
        result = self.pose(raw)
        for side, y, outward in [('left', .2, 1), ('right', -.2, -1)]:
            np.testing.assert_allclose(result[side]['position_m'], [0, y, -.485], atol=1e-12)
            marker = np.asarray(result[side]['transform'])[:3, 0]
            np.testing.assert_allclose(marker, [0, outward, 0], atol=1e-12)

    def test_explicit_zero_offset_is_subtracted_before_scale_and_sign(self):
        params = {s: {'scale_rad_per_raw': [2*np.pi]*5, 'zero_raw': [.1]*5}
                  for s in ('left', 'right')}
        raw = [.1]*96
        raw[1] = -.15
        result = self.fk.angles_from_observation(raw, conversion_parameters=params)
        np.testing.assert_allclose(result['left']['model_angles_rad'], [np.pi/2, 0, 0, 0, 0], atol=1e-12)
        np.testing.assert_allclose(result['right']['model_angles_rad'], [0]*5, atol=1e-12)
        self.assertFalse(result['left']['normalization_rule_documented'])

    @unittest.skipUnless((Path(__file__).parent/'fixtures/rf_zero_reference_20261012.json').exists(),
                         'Private calibration samples are not distributed in the source release.')
    def test_saved_real_zero_reference_maps_to_model_zero(self):
        fk = CandidateKinematics()
        raw = [0.0]*96
        source = fk.model['arms']['left']['encoder_to_model']['zero_calibration_source']
        data = (Path(__file__).parent/'fixtures/rf_zero_reference_20261012.json').read_bytes()
        self.assertEqual(len(source['sha256']), 64)
        sample = json.loads(data)
        self.assertEqual(sample['received'], 100)
        self.assertFalse(sample['motion_commands_sent'])
        for side in ('left', 'right'):
            arm = fk.model['arms'][side]
            for motor, slot, zero in zip(arm['motor_ids'], arm['obs_indices'], arm['encoder_to_model']['zero_raw']):
                self.assertEqual(zero, sample['joints'][str(motor)]['median'])
                self.assertEqual(zero, np.median([s['obs'][slot] for s in sample['samples']]))
                raw[slot] = zero
        result = fk.forward_from_observation(raw, allow_provisional=True)
        self.assertTrue(result['zero_reference_calibrated'])
        for side, y in [('left', .2), ('right', -.2)]:
            np.testing.assert_allclose(result[side]['conversion']['model_angles_rad'], [0]*5, atol=1e-12)
            np.testing.assert_allclose(result[side]['position_m'], [0, y, -.485], atol=1e-12)
        raw[1] -= .25
        result = fk.forward_from_observation(raw, allow_provisional=True)
        np.testing.assert_allclose(result['left']['position_m'], [.485, .2, 0], atol=1e-12)

    def test_raw_output_pose_order_and_calibration_boundary(self):
        raw = [0.0]*96
        raw[1] = -.125
        with self.assertRaises(RuntimeError):
            self.fk.forward_from_observation(raw)
        result = self.pose(raw)
        self.assertEqual(result['raw_observation'], raw)
        expected = result['left']['position_m'] + result['left']['quaternion_wxyz'] + result['right']['position_m'] + result['right']['quaternion_wxyz']
        self.assertEqual(result['ee_pose'], expected)
        self.assertEqual(len(result['ee_pose']), 14)
        self.assertTrue(result['normalization_rule_documented'])
        self.assertFalse(result['normalization_verified'])
        self.assertFalse(result['physical_calibration_verified'])
        self.assertFalse(result['training_ready'])
        self.assertNotIn('measurement_host_ns', result)
        self.assertNotIn('received_ns', result)
        self.assertFalse(self.fk.mapping['runtime_integration_enabled'])

    def test_bad_observations_and_missing_or_invalid_parameters(self):
        for raw in ([0]*95, [0]*97, [float('nan')]*96, [float('inf')]*96, [1.001]*96):
            with self.subTest(raw_first=raw[0], length=len(raw)), self.assertRaises(ValueError):
                self.pose(raw)
        params = {s: {'scale_rad_per_raw': [2*np.pi]*5, 'zero_raw': [0]*5}
                  for s in ('left', 'right')}
        for bad in (0, -1, float('nan'), float('inf')):
            candidate = copy.deepcopy(params)
            candidate['left']['scale_rad_per_raw'][0] = bad
            with self.subTest(scale=bad), self.assertRaises(ValueError):
                self.fk.angles_from_observation([0]*96, conversion_parameters=candidate)
        with self.assertRaises(ValueError):
            self.fk.angles_from_observation([0]*96, conversion_parameters={'left': params['left']})
        self.fk.model['arms']['left']['encoder_to_model']['scale_rad_per_raw'] = None
        with self.assertRaises(RuntimeError):
            self.pose([0]*96)

    def test_stale_mapping_signs_or_axes_are_rejected(self):
        for change in ('indices', 'sign', 'axes'):
            self.fk = CandidateKinematics()
            arm = self.fk.model['arms']['left']
            if change == 'indices':
                arm['obs_indices'][0] = 0
            elif change == 'sign':
                arm['encoder_to_model']['sign'][2] = 1
            else:
                arm['joints'][2]['axis'] = [0, 0, 1]
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.pose([0]*96)


if __name__ == '__main__':
    unittest.main()
