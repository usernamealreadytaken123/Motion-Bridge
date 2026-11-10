import importlib.util
import unittest
from unittest.mock import patch
if not all(importlib.util.find_spec(name) for name in ("av", "cv2", "numpy")):
    raise unittest.SkipTest("Install pc_app/requirements-gui.txt for MP4 tests")

import cv2
import numpy as np
import hashlib
import json
from pathlib import Path
import tempfile
from threading import Event
import time
import av
from pc_app.motion_viewer.mp4_odometry import (VideoPlanarEngine, assumed_intrinsics, solve_ground_step,
                                             MAX_CHAIN_FRAMES, MAX_CHAIN_SECONDS, MAX_CHAIN_ERROR_PX)
from pc_app.motion_viewer.mp4_worker import Mp4Reader, Mp4Worker, upright_frame, checked_dimensions
from pc_app.motion_viewer.mp4_smoke import make_clip
from pc_app.motion_viewer.odometry_math import camera_basis
from pc_app.tests.test_pc_odometry import quaternion, texture


class VideoEngineTests(unittest.TestCase):
    @staticmethod
    def shifted(image, i):
        return cv2.warpAffine(image, np.float32([[1, 0, -4*i], [0, 1, 2*i]]), (640, 480))

    def test_metric_rejection_advances_visual_frame_and_preserves_chained_turn(self):
        image = texture()
        k = assumed_intrinsics(image.shape)
        initial = camera_basis(quaternion(25, 2, 20), 0, 0)
        n = initial.T @ [0., 0., -1.]
        engine = VideoPlanarEngine(1.)
        engine.process(image, 0., 0)
        def frame(i):
            current = camera_basis(quaternion(25+i*.3, 2+i*.2, 20+i*1.5), 0, 0)
            displacement = np.array([i*.025, -i*.015, 0.])
            h = k @ (current.T @ initial + np.outer(-current.T @ displacement, n)) @ np.linalg.inv(k)
            return cv2.warpPerspective(image, h, (640, 480))
        first = engine.process(frame(1), .1, 1)
        self.assertEqual("TRACKING", first["status"])
        pose_reference = engine.pose_reference
        rotation, axes, position = engine.camera_to_origin.copy(), engine.axes.copy(), engine.position.copy()
        with patch.object(engine, "pose_candidate", side_effect=ValueError("temporary pose rejection")):
            for i in (2, 3):
                result = engine.process(frame(i), i*.1, i)
                self.assertEqual("DEGRADED", result["status"])
                self.assertEqual("TRACKING", result["visual_status"])
                self.assertAlmostEqual((i-1)*.1, result["visual_reference_time_s"])
                self.assertAlmostEqual(i*.1, engine.previous.time_s)
                self.assertIs(pose_reference, engine.pose_reference)
                np.testing.assert_array_equal(rotation, engine.camera_to_origin)
                np.testing.assert_array_equal(axes, engine.axes)
                np.testing.assert_array_equal(position, engine.position)
        result = engine.process(frame(4), .4, 4)
        self.assertEqual("TRACKING", result["status"], result)
        self.assertEqual("CHAIN", result["pose_method"])
        self.assertEqual(3, result["pending_frames"])
        self.assertAlmostEqual(.1, result["reference_time_s"])
        right = np.array([1., 0., 0.])-n[0]*n
        right /= np.linalg.norm(right)
        expected = np.array([right, np.cross(n, right)]) @ initial.T @ [4*.025, -4*.015, 0.]
        np.testing.assert_allclose(engine.position, expected, atol=.005)
        self.assertEqual(1, result["segment"])
        self.assertIs(engine.previous, engine.pose_reference)
        self.assertEqual(0, engine.pending_frames)
        next_result = engine.process(frame(5), .5, 5)
        self.assertEqual("PAIR", next_result["pose_method"])
        self.assertAlmostEqual(.4, next_result["reference_time_s"])

    def test_initial_window_can_restart_only_before_ground_frame_exists(self):
        image = texture()
        engine = VideoPlanarEngine(1.)
        engine.process(image, 0., 0)
        with patch.object(engine, "pose_candidate", side_effect=ValueError("no plane yet")):
            for i in range(1, 18):
                result = engine.process(self.shifted(image, i), i*.1, i)
                self.assertAlmostEqual(i*.1, engine.previous.time_s)
                self.assertEqual(0., result["x_m"])
                if result["initialization_restarted"]:
                    self.assertGreater(result["origin_time_s"], MAX_CHAIN_SECONDS)
                    self.assertIn("неизвестно", result["message"])
        result = engine.process(self.shifted(image, 18), 1.8, 18)
        self.assertEqual("TRACKING", result["status"], result)
        self.assertAlmostEqual(1.6, result["origin_time_s"])
        self.assertAlmostEqual(8/576, result["x_m"], delta=.002)

    def test_chain_limits_do_not_rebase_an_established_pose(self):
        image = texture()
        for dt in (.01, .11):  # exercise frame-count and elapsed-time bounds
            engine = VideoPlanarEngine(1.)
            engine.process(image, 0., 0)
            good = engine.process(self.shifted(image, 1), dt, 1)
            pose_reference = engine.pose_reference
            with patch.object(engine, "pose_candidate", side_effect=ValueError("pose rejected")):
                for i in range(2, MAX_CHAIN_FRAMES+3):
                    result = engine.process(self.shifted(image, i), dt*i, i)
                    self.assertEqual("TRACKING", result["visual_status"])
                    self.assertLessEqual(engine.pending_frames, MAX_CHAIN_FRAMES)
                    self.assertEqual(good["x_m"], result["x_m"])
                    self.assertIs(pose_reference, engine.pose_reference)
            self.assertIsNone(engine.pending_h)
            self.assertEqual(0, engine.initialization_restarts)
            self.assertEqual(0., engine.origin_time_s)
            recovered = engine.process(self.shifted(image, 18), dt*18, 18)
            self.assertEqual("TRACKING", recovered["status"], recovered)
            self.assertEqual("DIRECT", recovered["pose_method"])
            self.assertAlmostEqual(72/576, recovered["x_m"], delta=.003)

    def test_visual_restart_never_attaches_unrelated_scene_to_metric_pose(self):
        image = texture()
        engine = VideoPlanarEngine(1.)
        engine.process(image, 0., 0)
        good = engine.process(self.shifted(image, 1), .1, 1)
        reference = engine.pose_reference
        unrelated = cv2.GaussianBlur(np.random.default_rng(456).integers(0, 256, image.shape, dtype=np.uint8), (3, 3), .7)
        first = engine.process(unrelated, 1., 2)
        self.assertEqual("RESTARTED", first["visual_status"])
        next_frame = engine.process(self.shifted(unrelated, 1), 1.1, 3)
        self.assertEqual("TRACKING", next_frame["visual_status"])
        self.assertEqual("DEGRADED", next_frame["status"])
        self.assertEqual(good["x_m"], next_frame["x_m"])
        self.assertIs(reference, engine.pose_reference)
        self.assertIsNone(engine.pending_h)
        back = engine.process(self.shifted(image, 4), 1.2, 4)
        self.assertEqual("TRACKING", back["status"], back)
        self.assertEqual("DIRECT", back["pose_method"])
        self.assertAlmostEqual(16/576, back["x_m"], delta=.002)

    def test_chain_composition_order_and_error_budget(self):
        image = texture()
        engine = VideoPlanarEngine(1.)
        engine.process(image, 0., 0)
        k = assumed_intrinsics(image.shape)
        a = np.float32([[100., 100.], [500., 100.], [100., 350.], [500., 350.]])
        first = np.array([[1., 0., -30.], [0., 1., 10.], [0., 0., 1.]])
        second = k @ cv2.Rodrigues(np.array([.04, -.02, .2]))[0] @ np.linalg.inv(k)
        for time_s, h in ((.1, first), (.2, second)):
            b = cv2.perspectiveTransform(a[:, None], h).reshape(-1, 2)
            result = engine.advance_chain((h, a, b, 1., .1, "LK"), time_s, k)
        expected = second @ first
        np.testing.assert_allclose(result[0]/result[0][2, 2], expected/expected[2, 2], atol=1e-9)
        engine.pending_error = MAX_CHAIN_ERROR_PX
        self.assertIsNone(engine.advance_chain((np.eye(3), a, a, 1., .1, "LK"), .3, k))
        self.assertIsNone(engine.pending_h)

    def test_failed_composition_cannot_reuse_map_of_previous_image(self):
        image = texture()
        engine = VideoPlanarEngine(1.)
        engine.process(image, 0., 0)
        good = engine.process(self.shifted(image, 1), .1, 1)
        engine.pending_h = np.zeros((3, 3))
        with patch.object(engine, "pose_candidate", side_effect=ValueError("pose rejected")):
            result = engine.process(self.shifted(image, 2), .2, 2)
        self.assertEqual("TRACKING", result["visual_status"])
        self.assertEqual("DEGRADED", result["status"])
        self.assertIsNone(engine.pending_h)
        self.assertEqual(.2, engine.previous.time_s)
        self.assertEqual(.1, engine.pose_reference.time_s)
        self.assertEqual(good["x_m"], result["x_m"])

    def test_pixels_translation_and_height_scale(self):
        image = texture()
        for height in (1., 2.):
            engine = VideoPlanarEngine(height)
            engine.process(image, 0., 0)
            for i in range(1, 6):
                moved = cv2.warpAffine(image, np.float32([[1, 0, -4*i], [0, 1, 2*i]]), (640, 480))
                result = engine.process(moved, i*.1, i)
                self.assertEqual("TRACKING", result["status"], result)
                self.assertEqual(0, result["segment"])
            np.testing.assert_allclose([result["x_m"], result["y_m"]], [height*20/576, -height*10/576], atol=.001)

    def test_tilted_camera_translation_and_rotation(self):
        image = texture()
        k = assumed_intrinsics(image.shape)
        old, new = camera_basis(quaternion(25, 2, 20), 0, 0), camera_basis(quaternion(27, 3, 21), 0, 0)
        n = old.T @ [0., 0., -1.]
        world_delta = np.array([.025, -.015, 0.])
        r, t = new.T @ old, -new.T @ world_delta
        homography = k @ (r+np.outer(t, n)) @ np.linalg.inv(k)
        moved = cv2.warpPerspective(image, homography, (640, 480))
        engine = VideoPlanarEngine(1.)
        engine.process(image, 0., 0)
        result = engine.process(moved, .1, 1)
        self.assertEqual("TRACKING", result["status"], result)
        right = np.array([1., 0., 0.])-n[0]*n; right /= np.linalg.norm(right)
        expected = np.array([right, np.cross(n, right)]) @ old.T @ world_delta
        np.testing.assert_allclose([result["x_m"], result["y_m"]], expected, atol=.003)

    def test_rotation_only_does_not_invent_motion(self):
        image = texture()
        k = assumed_intrinsics(image.shape)
        b0, b1 = camera_basis(quaternion(), 0, 0), camera_basis(quaternion(1.5, 1., .5), 0, 0)
        moved = cv2.warpPerspective(image, k @ b1.T @ b0 @ np.linalg.inv(k), (640, 480))
        engine = VideoPlanarEngine(1.)
        engine.process(image, 0., 0)
        result = engine.process(moved, .1, 1)
        self.assertEqual("INITIALIZING", result["status"], result)
        self.assertEqual(0., result["distance_m"])

    def test_zoom_or_height_change_is_not_planar_translation(self):
        k = assumed_intrinsics((480, 640))
        points = np.array([[100., 100.], [500., 100.], [100., 350.], [500., 350.]])
        homography = k @ np.diag([1., 1., 1.1]) @ np.linalg.inv(k)
        with self.assertRaises(ValueError):
            solve_ground_step(homography, k, points)

    def test_texture_loss_and_time_gap_keep_origin_and_recover_displacement(self):
        image = texture()
        def shifted(i):
            return cv2.warpAffine(image, np.float32([[1, 0, -4*i], [0, 1, 2*i]]), (640, 480))
        engine = VideoPlanarEngine(1.)
        engine.process(image, 0., 0)
        good = engine.process(shifted(1), .1, 1)
        reference, orientation, axes = engine.previous, engine.camera_to_origin.copy(), engine.axes.copy()
        for i in range(2, 5):
            bad = engine.process(np.zeros_like(image), i*.1, i)
            self.assertEqual("DEGRADED", bad["status"])
            self.assertEqual((good["x_m"], good["y_m"]), (bad["x_m"], bad["y_m"]))
            self.assertEqual(good["distance_m"], bad["distance_m"])
            self.assertIs(reference, engine.previous)
            np.testing.assert_array_equal(orientation, engine.camera_to_origin)
            np.testing.assert_array_equal(axes, engine.axes)
        new = engine.process(shifted(5), .5, 5)
        self.assertEqual("TRACKING", new["status"], new)
        self.assertEqual(1, new["segment"])
        self.assertTrue(new["recovered"])
        self.assertEqual(3, new["rejected_frames"])
        np.testing.assert_allclose([new["x_m"], new["y_m"]], [20/576, -10/576], atol=.001)
        gap = engine.process(shifted(6), 2., 6)
        self.assertEqual("TRACKING", gap["status"], gap)
        self.assertEqual(2, gap["segment"])
        np.testing.assert_allclose([gap["x_m"], gap["y_m"]], [24/576, -12/576], atol=.001)

    def test_unrelated_scene_never_rebases_pose(self):
        image = texture()
        engine = VideoPlanarEngine(1.)
        engine.process(image, 0., 0)
        good = engine.process(cv2.warpAffine(image, np.float32([[1, 0, -8], [0, 1, 4]]), (640, 480)), .1, 1)
        unrelated = cv2.GaussianBlur(np.random.default_rng(456).integers(0, 256, image.shape, dtype=np.uint8), (3, 3), .7)
        for i in range(2, 8):
            result = engine.process(unrelated, float(i), i)
            self.assertEqual("DEGRADED", result["status"], result)
            self.assertEqual((good["x_m"], good["y_m"]), (result["x_m"], result["y_m"]))
            self.assertEqual(0, result["segment"])

    def test_descriptor_recovery_after_optical_flow_failure(self):
        image = texture()
        engine = VideoPlanarEngine(1.)
        engine.process(image, 0., 0)
        moved = cv2.warpAffine(image, np.float32([[1, 0, -32], [0, 1, 16]]), (640, 480))
        with patch("pc_app.motion_viewer.mp4_odometry.tracked_homography", side_effect=ValueError("LK failed")):
            result = engine.process(moved, 1., 1)
        self.assertEqual("TRACKING", result["status"], result)
        self.assertEqual("ORB", result["tracking_method"])
        np.testing.assert_allclose([result["x_m"], result["y_m"]], [32/576, -16/576], atol=.003)

    def test_recovery_preserves_ground_axes_during_tilt_and_turn(self):
        image = texture()
        k = assumed_intrinsics(image.shape)
        initial = camera_basis(quaternion(25, 2, 20), 0, 0)
        n = initial.T @ [0., 0., -1.]
        engine = VideoPlanarEngine(1.)
        engine.process(image, 0., 0)
        for i in range(1, 7):
            current = camera_basis(quaternion(25+i*.3, 2+i*.1, 20+i), 0, 0)
            displacement = np.array([i*.025, -i*.015, 0.])
            homography = k @ (current.T @ initial + np.outer(-current.T @ displacement, n)) @ np.linalg.inv(k)
            frame = cv2.warpPerspective(image, homography, (640, 480))
            result = engine.process(np.zeros_like(image) if i in (3, 4) else frame, i*.1, i)
            self.assertEqual("DEGRADED" if i in (3, 4) else "TRACKING", result["status"], result)
        right = np.array([1., 0., 0.])-n[0]*n
        right /= np.linalg.norm(right)
        expected = np.array([right, np.cross(n, right)]) @ initial.T @ displacement
        np.testing.assert_allclose([result["x_m"], result["y_m"]], expected, atol=.005)
        self.assertEqual(1, result["segment"])

    def test_known_plane_rejects_zoom_and_accepts_noisy_motion(self):
        k = assumed_intrinsics((480, 640))
        points = np.float32([[x, y] for x in range(60, 600, 50) for y in range(60, 440, 50)])
        zoom = k @ np.diag([1., 1., 1.1]) @ np.linalg.inv(k)
        zoomed = cv2.perspectiveTransform(points[:, None, :], zoom).reshape(-1, 2)
        with self.assertRaises(ValueError):
            solve_ground_step(zoom, k, points, np.array([0., 0., 1.]), zoomed)
        shifted = points+[-4., 2.]+np.random.default_rng(123).normal(0, .2, points.shape)
        h, _ = cv2.findHomography(points, shifted)
        r, t, n, _ = solve_ground_step(h, k, points, np.array([0., 0., 1.]), shifted)
        np.testing.assert_allclose(-r.T @ t, [4/576, -2/576, 0.], atol=.0005)


class Mp4FileTests(unittest.TestCase):
    def test_actual_mp4_b_frames_vfr_timestamps_and_portrait_rotation(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory)/"portrait.MP4"
            stamps = [500, 600, 800, 900, 1100]
            make_clip(source, times=stamps, rotation=90)
            reader = Mp4Reader(source)
            decoded = list(reader.frames(Event()))
            np.testing.assert_allclose([t for t, _ in decoded], [0., .1, .3, .4, .6], atol=1e-6)
            self.assertEqual((640, 480), decoded[0][1].shape)
            with av.open(str(source)) as container:
                frame = next(container.decode(video=0))
                self.assertTrue(container.streams.video[0].codec_context.has_b_frames)
                np.testing.assert_array_equal(decoded[0][1], np.rot90(frame.to_ndarray(format="gray")))

    def test_high_frame_rate_is_sampled_by_pts_not_nominal_fps(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory)/"high-fps.mp4"
            make_clip(source, times=[500+i*20 for i in range(20)])
            reader = Mp4Reader(source)
            frames = list(reader.frames(Event()))
            np.testing.assert_allclose([t for t, _ in frames], [0., .08, .16, .24, .32], atol=1e-6)
            self.assertEqual(20, reader.decoded)

    def test_2d_worker_uses_all_decoded_frames_and_source_timestamps(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory)/"all-frames.mp4"
            make_clip(source, times=[500+i*20 for i in range(20)])
            worker = Mp4Worker(source, 1., directory)
            worker.start(); worker.join(10)
            self.assertFalse(worker.is_alive())
            self.assertIsNone(worker.update["error"])
            self.assertEqual(20, worker.update["frames"])
            self.assertEqual(20, worker.update["decoded"])
            recording = Path(worker.update["recording"])
            manifest = json.loads((recording/"manifest.json").read_text(encoding="utf-8"))
            self.assertIsNone(manifest["max_analysis_fps"])
            self.assertEqual("all_decoded_frames", manifest["frame_sampling"])
            rows = [json.loads(line) for line in (recording/"results.jsonl").read_text().splitlines()]
            np.testing.assert_allclose([r["time_s"] for r in rows], [i*.02 for i in range(20)], atol=1e-6)
            self.assertEqual(19, worker.update["tracking"])

    def test_worker_preserves_source_records_estimate_and_height_rescales(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory)/"input.mp4"
            make_clip(source)
            digest = hashlib.sha256(source.read_bytes()).digest()
            results = []
            recordings = []
            for height in (1., 2.):
                worker = Mp4Worker(source, height, directory)
                worker.start(); worker.join(10)
                self.assertFalse(worker.is_alive())
                self.assertIsNone(worker.update["error"])
                self.assertEqual(12, worker.update["frames"])
                path = Path(worker.update["recording"])
                manifest = json.loads((path/"manifest.json").read_text(encoding="utf-8"))
                self.assertTrue(manifest["complete"])
                self.assertEqual("ASSUMED", manifest["calibration"])
                rows = [json.loads(line) for line in (path/"results.jsonl").read_text().splitlines()]
                self.assertEqual(12, len(rows))
                self.assertEqual("TRACKING", rows[-1]["status"])
                self.assertEqual(0, rows[-1]["segment"])
                results.append(rows[-1]["x_m"])
                recordings.append(path)
            self.assertAlmostEqual(44/576, results[0], delta=.003)
            self.assertAlmostEqual(2*results[0], results[1], delta=.000001)
            self.assertNotEqual(*recordings)
            self.assertEqual(digest, hashlib.sha256(source.read_bytes()).digest())

    def test_pause_cancel_closes_file_and_marks_partial_result(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory)/"cancel.mp4"
            make_clip(source)
            worker = Mp4Worker(source, 1., directory)
            worker.paused.set()
            worker.start()
            time.sleep(.15)
            self.assertEqual(0, worker.update["frames"])
            worker.stop(); worker.join(5)
            self.assertFalse(worker.is_alive())
            self.assertIsNone(worker.update["error"])
            manifest = json.loads((Path(worker.update["recording"])/"manifest.json").read_text(encoding="utf-8"))
            self.assertFalse(manifest["complete"])
            self.assertTrue(manifest["cancelled"])
            source.rename(source.with_name("closed.mp4"))

    def test_bad_file_mirror_and_dimensions_fail_safely(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory)/"bad.mp4"
            source.write_bytes(b"not a movie")
            worker = Mp4Worker(source, 1., directory)
            worker.start(); worker.join(5)
            self.assertIsNotNone(worker.update["error"])
            self.assertFalse(json.loads((Path(worker.update["recording"])/"manifest.json").read_text())["complete"])
            self.assertEqual(b"not a movie", source.read_bytes())
            source = Path(directory)/"mirror.mp4"
            make_clip(source, count=3, mirrored=True)
            with self.assertRaisesRegex(ValueError, "зеркальная"):
                list(Mp4Reader(source).frames(Event()))
        with self.assertRaises(ValueError):
            checked_dimensions(8000, 8000)
        frame = av.VideoFrame.from_ndarray(texture(), format="gray")
        with self.assertRaises(ValueError):
            upright_frame(frame, 15)

    def test_decoder_downsizes_without_losing_portrait_geometry(self):
        frame = av.VideoFrame(1080, 1920, "gray")
        self.assertEqual((960, 540), upright_frame(frame).shape)
