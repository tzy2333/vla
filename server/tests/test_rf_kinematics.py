"""Analytical FK cases verify the candidate's math, not the physical model."""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from collector.rf_kinematics import CandidateKinematics, axis_rotation, quaternion_wxyz


class CandidateFKTests(unittest.TestCase):
    def setUp(self):
        self.fk = CandidateKinematics()

    def test_zero_pose_uses_shoulder_midpoint_and_gripper_midpoint(self):
        for side, y in [('left', .2), ('right', -.2)]:
            pose = self.fk.forward(side, [0]*5)
            np.testing.assert_allclose(pose['position_m'], [0, y, -.485], atol=1e-12)
            np.testing.assert_allclose(pose['quaternion_wxyz'], [0, 1, 0, 0], atol=1e-12)
            self.assertFalse(pose['physical_calibration_verified'])
            self.assertFalse(pose['training_ready'])

    def test_shoulder_forward_right_angle(self):
        pose = self.fk.forward('left', [np.pi/2, 0, 0, 0, 0])
        np.testing.assert_allclose(pose['position_m'], [.485, .2, 0], atol=1e-12)

    def test_elbow_right_angle(self):
        pose = self.fk.forward('left', [0, 0, 0, np.pi/2, 0])
        np.testing.assert_allclose(pose['position_m'], [.205, .2, -.280], atol=1e-12)

    def test_side_swing_is_outward_on_each_side(self):
        for side, y in [('left', .685), ('right', -.685)]:
            pose = self.fk.forward(side, [0, np.pi/2, 0, 0, 0])
            np.testing.assert_allclose(pose['position_m'], [0, y, 0], atol=1e-12)

    def test_wrist_changes_orientation_not_centerline_position(self):
        a = self.fk.forward('left', [.2, .3, -.4, .5, 0])
        b = self.fk.forward('left', [.2, .3, -.4, .5, .7])
        np.testing.assert_allclose(a['position_m'], b['position_m'], atol=1e-12)
        self.assertFalse(np.allclose(a['quaternion_wxyz'], b['quaternion_wxyz']))

    def test_mirrored_model_geometry_with_equal_model_angles(self):
        angles = [.25, -.35, .6, 1.1, -.2]
        left = self.fk.forward('left', angles)
        right = self.fk.forward('right', angles)
        mirror = np.diag([1, -1, 1])
        np.testing.assert_allclose(right['position_m'], mirror @ left['position_m'], atol=1e-12)
        lrot = np.array(left['transform'])[:3, :3]
        rrot = np.array(right['transform'])[:3, :3]
        np.testing.assert_allclose(rrot, mirror@lrot@mirror, atol=1e-12)

    def test_quaternion_at_pi_and_general_rotations(self):
        for axis, angle in [([1, 0, 0], np.pi), ([0, 1, 0], np.pi),
                            ([0, 0, 1], np.pi), ([1, -2, 3], 2.4)]:
            r = axis_rotation(axis, angle)
            q = quaternion_wxyz(r)
            self.assertAlmostEqual(np.linalg.norm(q), 1)
            w, x, y, z = q
            restored = np.array([
                [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
            ])
            np.testing.assert_allclose(restored, r, atol=1e-12)

    def test_invalid_inputs_and_unverified_raw_conversion_are_rejected(self):
        for angles in ([0]*4, [0, 0, np.nan, 0, 0], [0, 0, np.inf, 0, 0]):
            with self.assertRaises(ValueError):
                self.fk.forward('left', angles)
        with self.assertRaises(ValueError):
            self.fk.forward('unknown', [0]*5)
        with self.assertRaises(RuntimeError):
            self.fk.forward_from_observation([0]*96)


if __name__ == '__main__':
    unittest.main()
