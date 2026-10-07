import math
import unittest

from pc_app.motion_viewer.orientation import relative_matrix, rotation_matrix


class OrientationTests(unittest.TestCase):
    def assert_matrix(self, actual, expected):
        for row, wanted in zip(actual, expected):
            for value, target in zip(row, wanted):
                self.assertAlmostEqual(value, target, places=10)

    def test_identity_and_normalization(self):
        self.assert_matrix(rotation_matrix((1.01, 0, 0, 0)), ((1, 0, 0), (0, 1, 0), (0, 0, 1)))

    def test_positive_90_degree_rotations_match_android_right_handed_axes(self):
        c = math.sqrt(0.5)
        self.assert_matrix(rotation_matrix((c, c, 0, 0)), ((1, 0, 0), (0, 0, -1), (0, 1, 0)))
        self.assert_matrix(rotation_matrix((c, 0, c, 0)), ((0, 0, 1), (0, 1, 0), (-1, 0, 0)))
        self.assert_matrix(rotation_matrix((c, 0, 0, c)), ((0, -1, 0), (1, 0, 0), (0, 0, 1)))

    def test_quaternion_sign_does_not_change_rotation(self):
        self.assert_matrix(rotation_matrix((.5, .5, .5, .5)), rotation_matrix((-.5, -.5, -.5, -.5)))

    def test_reference_uses_inverse_on_left_and_current_becomes_identity(self):
        c = math.sqrt(.5)
        base = (c, c, 0, 0)
        self.assert_matrix(relative_matrix(base, base), rotation_matrix((1, 0, 0, 0)))
        self.assert_matrix(relative_matrix((c, 0, 0, c), base), ((0, -1, 0), (0, 0, 1), (-1, 0, 0)))

    def test_invalid_quaternions_are_rejected(self):
        for value in ((0, 0, 0, 0), (1, 0, 0), (float("nan"), 0, 0, 0)):
            with self.assertRaises(ValueError):
                rotation_matrix(value)
