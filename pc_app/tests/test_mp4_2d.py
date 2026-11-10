"""Regression tests for the MP4-only, sensor-free LEGACY-style 2D engine."""
import importlib.util
import math
import unittest
from unittest.mock import patch

if not all(importlib.util.find_spec(name) for name in ("cv2", "numpy")):
    raise unittest.SkipTest("OpenCV and NumPy required")

import cv2
import numpy as np

from pc_app.motion_viewer.mp4_2d import Video2DEngine, similarity_step
from pc_app.tests.test_pc_odometry import texture


class Video2DTests(unittest.TestCase):
    @staticmethod
    def frame(image, x=0., y=0., angle=0., scale=1.):
        centre = (np.array(image.shape[::-1])-1)/2
        c, s = math.cos(angle), math.sin(angle)
        linear = scale*np.array([[c, -s], [s, c]])
        affine = np.column_stack((linear, centre-linear@centre + [x, y]))
        return cv2.warpAffine(image, affine, image.shape[::-1])

    def test_translation_and_h_over_f_only_scale_the_same_trajectory(self):
        image = texture()
        trajectories = []
        statuses = []
        for height, focal in ((1., .9), (2., .9), (1., 1.8), (.05, 5.), (20., .1), (50., .9), (100., .1)):
            engine = Video2DEngine(height, focal)
            path, accepted = [], []
            for i, (x, y) in enumerate(((0, 0), (-4, 0), (-8, 0), (-8, 4), (-8, 8), (-4, 8))):
                result = engine.process(self.frame(image, x, y), i*.1, i)
                path.append([result["x_m"], result["y_m"]])
                accepted.append(result["status"])
            trajectories.append(np.array(path)/(height/focal))
            statuses.append(accepted)
            self.assertEqual(focal*640, result["focal_px"])
            self.assertEqual(height, result["height_m"])
        for path, status in zip(trajectories[1:], statuses[1:]):
            np.testing.assert_allclose(path, trajectories[0], atol=1e-8)
            self.assertEqual(statuses[0], status)
        self.assertEqual(["INITIALIZING"]+["TRACKING"]*5, statuses[0])
        np.testing.assert_allclose(trajectories[0][-1], [4/640, -8/640], atol=.0002)

    def test_centre_rotation_and_small_zoom_do_not_invent_translation(self):
        image = texture()
        engine = Video2DEngine(1.)
        engine.process(image, 0., 0)
        for i in range(1, 8):
            scale = 1.+.01*math.sin(i)
            result = engine.process(self.frame(image, angle=i*.025, scale=scale), i*.1, i)
            self.assertEqual("TRACKING", result["status"], result)
            self.assertIsNone(result["tilt_deg"])
        self.assertLess(result["distance_m"], .002)
        self.assertAlmostEqual(-7*.025, result["heading_rad"], delta=.002)

    def test_closed_rectangle_returns_to_origin_by_motion_not_reset(self):
        image = texture()
        offsets = [(0, 0)] + [(i*4, 0) for i in range(1, 11)]
        offsets += [(40, i*4) for i in range(1, 11)]
        offsets += [(40-i*4, 40) for i in range(1, 11)]
        offsets += [(0, 40-i*4) for i in range(1, 11)]
        engine = Video2DEngine(1.)
        for i, (x, y) in enumerate(offsets):
            result = engine.process(self.frame(image, x, y), i*.1, i)
            self.assertEqual("INITIALIZING" if i == 0 else "TRACKING", result["status"])
            np.testing.assert_allclose(engine.position, -np.array([x, y])/576, atol=.001)
        self.assertEqual(0, result["segment"])
        self.assertAlmostEqual(160/576, result["distance_m"], delta=.002)
        self.assertLess(np.linalg.norm(engine.position), .001)

    def test_blank_prefix_starts_at_first_textured_frame(self):
        image = texture()
        engine = Video2DEngine(1.)
        engine.process(np.zeros_like(image), 0., 0)
        first = engine.process(image, .1, 1)
        self.assertEqual("INITIALIZING", first["status"])
        result = engine.process(self.frame(image, -4), .2, 2)
        self.assertEqual("TRACKING", result["status"])
        self.assertEqual(.1, result["origin_time_s"])
        self.assertEqual(0, result["segment"])
        self.assertTrue(result["heading_continuous"])

    def test_similarity_uses_centre_flow_not_top_left_translation(self):
        a = np.float32([[x, y] for x in range(60, 600, 60) for y in range(60, 440, 60)])
        angle = .04
        c, s = math.cos(angle), math.sin(angle)
        linear = 1.02*np.array([[c, -s], [s, c]])
        centre = np.array([319.5, 239.5])
        b = (a-centre)@linear.T + centre + [-5., 3.]
        flow, turn, scale, *_ = similarity_step(a, b, (480, 640))
        np.testing.assert_allclose(flow, [-5., 3.], atol=1e-4)
        self.assertAlmostEqual(-angle, turn, places=6)
        self.assertAlmostEqual(1.02, scale, places=6)

    def test_turns_rotate_steps_into_initial_axes(self):
        image = texture()
        engine = Video2DEngine(1.)
        engine.process(image, 0., 0)
        centre = np.array([319.5, 239.5])
        h = np.eye(3)
        expected = np.zeros(2)
        for i in range(1, 10):
            turn = .035
            middle = (i-.5)*turn
            # Exact LEGACY convention: midpoint heading rotates camera step to XY.
            delta = np.array([.012, -.008])
            c, s = math.cos(-middle), math.sin(-middle)
            flow = -576*np.array([[c, -s], [s, c]])@delta
            c, s = math.cos(-turn), math.sin(-turn)
            linear = np.array([[c, -s], [s, c]])
            pair = np.eye(3)
            pair[:2, :2] = linear
            pair[:2, 2] = centre-linear@centre+flow
            h = pair@h
            frame = cv2.warpPerspective(image, h, (640, 480))
            result = engine.process(frame, i*.1, i)
            self.assertEqual("TRACKING", result["status"], result)
            expected += delta
        np.testing.assert_allclose(engine.position, expected, atol=.002)
        self.assertAlmostEqual(9*.035, result["heading_rad"], delta=.003)

    def test_rejection_cannot_pin_reference_or_reset_coordinates(self):
        image = texture()
        engine = Video2DEngine(1.)
        engine.process(image, 0., 0)
        good = engine.process(self.frame(image, -4, 2), .1, 1)
        with patch("pc_app.motion_viewer.mp4_2d.similarity_step", side_effect=ValueError("rejected")):
            for i in range(2, 24):
                result = engine.process(self.frame(image, -4*i, 2*i), i*.1, i)
                self.assertEqual("DEGRADED", result["status"])
                self.assertEqual("TRACKING", result["visual_status"])
                self.assertEqual(i*.1, engine.previous.time_s)
                self.assertEqual(good["x_m"], result["x_m"])
        result = engine.process(self.frame(image, -96, 48), 2.4, 24)
        self.assertEqual("TRACKING", result["status"], result)
        self.assertAlmostEqual(2.3, result["reference_time_s"])
        self.assertTrue(result["recovered"])
        self.assertFalse(result["heading_continuous"])
        self.assertAlmostEqual(8/576, result["x_m"], delta=.001)
        self.assertAlmostEqual(4/576, result["delta_x_m"], delta=.001)

    def test_blank_gap_skips_unknown_motion_and_resumes_without_old_anchor(self):
        image = texture()
        engine = Video2DEngine(1.)
        rows = []
        for i in range(9):
            frame = np.zeros_like(image) if i in (3, 4) else self.frame(image, -4*i)
            rows.append(engine.process(frame, i*.1, i))
        self.assertEqual("INITIALIZING", rows[5]["status"])
        self.assertEqual("TRACKING", rows[6]["status"])
        self.assertTrue(rows[6]["recovered"])
        self.assertEqual(1, rows[6]["segment"])
        self.assertFalse(rows[6]["heading_continuous"])
        for i in (3, 4, 5):
            self.assertEqual(rows[2]["x_m"], rows[i]["x_m"])
        self.assertAlmostEqual(20/576, rows[-1]["x_m"], delta=.001)

    def test_scene_cut_skips_cut_then_tracks_new_scene_with_warning(self):
        image = texture()
        other = cv2.GaussianBlur(np.random.default_rng(456).integers(0, 256, image.shape, dtype=np.uint8), (3, 3), .7)
        engine = Video2DEngine(1.)
        engine.process(image, 0., 0)
        good = engine.process(self.frame(image, -4), .1, 1)
        lost = engine.process(other, .2, 2)
        self.assertEqual("DEGRADED", lost["status"])
        self.assertEqual(good["x_m"], lost["x_m"])
        resumed = engine.process(self.frame(other, -4), .3, 3)
        self.assertEqual("TRACKING", resumed["status"], resumed)
        self.assertEqual(.2, resumed["reference_time_s"])
        self.assertFalse(resumed["heading_continuous"])
        self.assertAlmostEqual(8/576, resumed["x_m"], delta=.001)

    def test_large_zoom_is_rejected_without_waiting_for_original_scale(self):
        image = texture()
        engine = Video2DEngine(1.)
        engine.process(image, 0., 0)
        good = engine.process(self.frame(image, -4), .1, 1)
        lost = engine.process(self.frame(image, -8, scale=1.1), .2, 2)
        self.assertEqual("DEGRADED", lost["status"], lost)
        self.assertEqual(good["x_m"], lost["x_m"])
        result = engine.process(self.frame(image, -12, scale=1.1), .3, 3)
        self.assertEqual("TRACKING", result["status"], result)
        self.assertAlmostEqual(8/576, result["x_m"], delta=.001)

    def test_small_height_oscillations_do_not_require_a_3d_pose(self):
        image = texture()
        engine = Video2DEngine(1.)
        for i in range(20):
            result = engine.process(self.frame(image, -4*i, scale=1+.012*math.sin(i*.5)), i*.1, i)
            self.assertEqual("INITIALIZING" if i == 0 else "TRACKING", result["status"], result)
        self.assertGreater(result["x_m"], .10)
        self.assertTrue(result["heading_continuous"])

    def test_timestamp_gap_resumes_on_next_pair(self):
        image = texture()
        engine = Video2DEngine(1.)
        engine.process(image, 0., 0)
        self.assertEqual("DEGRADED", engine.process(self.frame(image, -4), 2., 1)["status"])
        result = engine.process(self.frame(image, -8), 2.1, 2)
        self.assertEqual("TRACKING", result["status"], result)
        self.assertAlmostEqual(4/576, result["x_m"], delta=.001)

    def test_invalid_time_shape_and_parameters_cannot_change_reference(self):
        image = texture()
        engine = Video2DEngine(1.)
        engine.process(image, 0., 0)
        ref = engine.previous
        for frame, stamp in ((image, 0.), (image, float("nan")), (image[:300], .1)):
            with self.assertRaises(ValueError):
                engine.process(frame, stamp, 1)
            self.assertIs(ref, engine.previous)
        for height, factor in ((0, .9), (.049, .9), (100.001, .9), (float("inf"), .9),
                               (float("nan"), .9), (1, 0), (1, float("inf"))):
            with self.assertRaises(ValueError):
                Video2DEngine(height, factor)

    def test_orb_fallback_uses_only_previous_frame(self):
        image = texture()
        engine = Video2DEngine(1.)
        engine.process(image, 0., 0)
        with patch("pc_app.motion_viewer.mp4_2d.tracked_homography", side_effect=ValueError("LK failed")):
            result = engine.process(self.frame(image, -20, 10), .1, 1)
        self.assertEqual("TRACKING", result["status"], result)
        self.assertEqual("ORB", result["tracking_method"])
        self.assertAlmostEqual(20/576, result["x_m"], delta=.002)
