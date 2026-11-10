"""Offscreen Qt integration tests; skipped when running the stdlib-only environment."""

import importlib.util
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import unittest

HAS_GUI = all(importlib.util.find_spec(name) for name in ("PySide6", "pyqtgraph", "numpy"))
if HAS_GUI:
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtCore import Qt
    from PySide6.QtTest import QTest
    from PySide6.QtWidgets import QApplication
    from pc_app.motion_viewer.gui import MotionViewer

from pc_app.tests.test_history import display_event
from pc_app.tests.test_replay import write_session
from pc_app.motion_viewer.preferences import Preferences


@unittest.skipUnless(HAS_GUI, "Install pc_app/requirements-gui.txt to run Qt integration tests")
class GuiTests(unittest.TestCase):
    @unittest.skipUnless(importlib.util.find_spec("av") and importlib.util.find_spec("cv2"), "MP4 dependencies are required")
    def test_mp4_focal_control_reaches_engine_manifest_and_display(self):
        from pc_app.motion_viewer.mp4_smoke import make_clip
        source = Path(self.directory.name)/"focal.mp4"
        make_clip(source)
        self.window.open_mp4()
        video = self.window.mp4_window
        video.open_video(source)
        video.focal.setValue(1.8)
        self.assertEqual(.05, video.height.minimum())
        self.assertEqual(100., video.height.maximum())
        video.height.setValue(100.)
        video.start()
        worker = video.worker
        self.assertFalse(video.focal.isEnabled())
        self.wait_for(lambda: video.worker is None)
        self.assertIsNone(worker.update["error"])
        self.assertAlmostEqual(100*44/1152, worker.update["result"]["x_m"], delta=.02)
        self.assertIn("1152.0 px", video.metrics.text())
        manifest = json.loads((Path(video.recording)/"manifest.json").read_text(encoding="utf-8"))
        self.assertEqual(1.8, manifest["focal_factor"])
        self.assertEqual(100., manifest["height_m"])
        self.assertEqual(100., worker.update["result"]["height_m"])
        self.assertEqual(1152., manifest["focal_px"])
        self.assertTrue(video.focal.isEnabled())
        summary = worker.update["final_summary"]
        self.assertTrue(video.summary.isVisible())
        self.assertEqual(manifest["final_summary"], summary)
        self.assertTrue(summary["at_video_end"])
        self.assertIn(f"X = {summary['x_m']:+.3f} м", video.summary.text())
        self.assertIn(f"Y = {summary['y_m']:+.3f} м", video.summary.text())
        self.assertEqual(f"Расстояние от (0, 0) по прямой: {summary['distance_from_origin_m']:.3f} м",
                         video.summary.text().splitlines()[1])
        self.assertNotIn("√", video.summary.text())
        self.assertNotIn("не длина пройденного пути", video.summary.text())
        # Reruns and new selections must not show the previous endpoint.
        video.start()
        video.pause()
        self.assertFalse(video.summary.isVisible())
        self.assertEqual("", video.summary.text())
        video.stop()
        self.wait_for(lambda: video.worker is None)
        self.assertIn("расчёт отменён", video.summary.text())
        video.open_video(source)
        self.assertFalse(video.summary.isVisible())
        self.assertEqual("", video.summary.text())
        video.close()

    @unittest.skipUnless(importlib.util.find_spec("av") and importlib.util.find_spec("cv2"), "MP4 dependencies are required")
    def test_mp4_shows_visual_tracking_without_claiming_metric_motion(self):
        from unittest.mock import patch
        from pc_app.motion_viewer.mp4_smoke import make_clip
        source = Path(self.directory.name)/"visual-only.mp4"
        make_clip(source)
        self.window.open_mp4()
        video = self.window.mp4_window
        video.open_video(source)
        with patch("pc_app.motion_viewer.mp4_2d.similarity_step", side_effect=ValueError("2D-модель не согласуется с точками")):
            video.start()
            worker = video.worker
            self.wait_for(lambda: video.worker is None)
        self.assertEqual(11, worker.update["visual_tracking"])
        self.assertEqual(0, worker.update["tracking"])
        self.assertEqual(0., video.marker.xData[0])
        self.assertIn("точки: отслеживаются", video.metrics.text())
        self.assertIn("2D-модель не согласуется", video.metrics.text())
        self.assertIn("наклон не компенсируется", video.metrics.text())
        self.assertGreater(len(worker.update["result"]["tracks"]), 0)
        self.assertTrue(video.summary.isVisible())
        self.assertIn("расстояние недоступны", video.summary.text())
        self.assertIsNone(worker.update["final_summary"]["distance_from_origin_m"])
        video.close()

    @unittest.skipUnless(importlib.util.find_spec("av") and importlib.util.find_spec("cv2"), "MP4 dependencies are required")
    def test_mp4_lost_marker_keeps_last_position(self):
        from pc_app.motion_viewer.mp4_smoke import make_clip
        import numpy as np
        source = Path(self.directory.name)/"lost-at-end.mp4"
        make_clip(source, blank_indices=(8, 9, 10, 11))
        self.window.open_mp4()
        video = self.window.mp4_window
        video.open_video(source)
        video.start()
        worker = video.worker
        self.wait_for(lambda: video.worker is None)
        result = worker.update["result"]
        self.assertEqual("DEGRADED", result["status"])
        self.assertGreater(result["x_m"], .04)
        self.assertEqual(result["x_m"], video.marker.xData[0])
        self.assertEqual(result["y_m"], video.marker.yData[0])
        self.assertEqual("x", video.marker.opts["symbol"])
        self.assertIn("Позиция удерживается", video.metrics.text())
        self.assertIn("последняя рассчитанная позиция", video.status.text())
        self.assertTrue(np.isnan(video.curve.xData[-1]))
        manifest = json.loads((Path(video.recording)/"manifest.json").read_text(encoding="utf-8"))
        self.assertTrue(manifest["tracking_lost_at_end"])
        self.assertEqual("adjacent_frames_keep_position_break_path", manifest["tracking_loss_policy"])
        self.assertEqual("similarity_2d", manifest["algorithm"])
        self.assertFalse(manifest["tilt_compensation"])
        summary = manifest["final_summary"]
        self.assertFalse(summary["at_video_end"])
        self.assertAlmostEqual(.7, summary["point_time_s"])
        self.assertAlmostEqual(1.1, summary["last_processed_time_s"])
        self.assertIn("Финальная точка видео не определена", video.summary.text())
        self.assertIn("t = 0.70 с", video.summary.text())
        self.assertEqual(result["x_m"], summary["x_m"])
        video.close()

    @unittest.skipUnless(importlib.util.find_spec("av") and importlib.util.find_spec("cv2"), "MP4 dependencies are required")
    def test_mp4_failed_decode_shows_unavailable_summary(self):
        source = Path(self.directory.name)/"broken.mp4"
        source.write_bytes(b"not an mp4")
        self.window.open_mp4()
        video = self.window.mp4_window
        video.open_video(source)
        video.start()
        self.wait_for(lambda: video.worker is None)
        self.assertTrue(video.summary.isVisible())
        self.assertIn("с ошибкой", video.summary.text())
        self.assertIn("недоступны", video.summary.text())
        video.close()

    @unittest.skipUnless(importlib.util.find_spec("av") and importlib.util.find_spec("cv2"), "MP4 dependencies are required")
    def test_mp4_import_compute_render_and_parent_close_while_paused(self):
        from pc_app.motion_viewer.mp4_smoke import exercise_mp4, make_clip
        result = exercise_mp4(self.window, self.wait_for, Path(self.directory.name)/"mp4-smoke.json")
        self.assertTrue(result["source_unchanged"])
        self.window.open_mp4()
        video = self.window.mp4_window
        source = Path(self.directory.name)/"pause.mp4"
        make_clip(source, count=40)
        video.open_video(source)
        video.start()
        video.pause()
        self.assertFalse(video.open_button.isEnabled())
        self.assertFalse(video.height.isEnabled())
        self.assertFalse(video.focal.isEnabled())
        self.window.close()
        self.wait_for(lambda: video.worker is None and not self.window.isVisible())

    @unittest.skipUnless(importlib.util.find_spec("cv2"), "OpenCV is needed for PC odometry")
    def test_pc_odometry_tcp_pixels_display_record_recompute_and_close(self):
        from pc_app.motion_viewer.smoke_test import exercise_pc_odometry
        result = exercise_pc_odometry(self.window, self.wait_for, Path(self.directory.name)/"raw-smoke.json")
        self.assertTrue(result["raw_recompute"])
        self.window.close()
        self.wait_for(lambda: not self.window.isVisible())
        self.assertIsNone(self.window.odometry_window.worker)

    @classmethod
    def setUpClass(cls):
        cls.application = QApplication.instance() or QApplication([])
        cls.application.setQuitOnLastWindowClosed(False)

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.window = MotionViewer("127.0.0.1", 5005)
        self.window.record_directory = Path(self.directory.name)
        self.window.show()

    def tearDown(self):
        self.window.close()
        self.wait_for(lambda: self.window.worker is None and self.window.replay_loader is None)
        self.window.close()
        self.window.deleteLater()
        self.application.processEvents()
        self.directory.cleanup()

    def wait_for(self, predicate, timeout=4):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            # Unlike QApplication.exec(), QTest.qWait can retain the GIL on Windows.
            # Give Python's loader/receiver thread execution time between Qt test waits.
            time.sleep(0.005)
            QTest.qWait(20)
        self.assertTrue(predicate(), "GUI condition timed out: " + self.window.error_label.text()
                        + " / " + self.window.replay_label.text())

    def start(self):
        # Select an ephemeral port for tests, leaving the production UI range unchanged.
        self.window.port_spin.setMinimum(0)
        self.window.port_spin.setValue(0)
        QTest.mouseClick(self.window.start_button, Qt.MouseButton.LeftButton)
        self.wait_for(lambda: self.window.worker is not None and self.window.worker._address is not None)
        return self.window.worker._address

    def test_udp_to_plots_recording_stale_camera_stop_and_restart(self):
        address = self.start()
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            for sequence in range(4):
                event = display_event(sequence=sequence, frame_stamp=sequence + 100)
                sender.sendto(json.dumps(event["payload"]).encode(), address)
        self.wait_for(lambda: self.window._snapshot["datagrams"] == 4)
        self.assertFalse(self.window.start_button.isEnabled())
        self.assertTrue(self.window.stop_button.isEnabled())
        self.assertIn("0.420", self.window.pose_label.text())
        self.assertEqual(4, len(self.window.path_curve.xData))
        self.assertEqual("TRACKING", self.window.camera_label.text())
        history = self.window.history.streams[self.window.history.last_key]
        history.frame_updated_ns = time.monotonic_ns() - 3_000_000_000
        self.window.refresh()
        self.assertIn("не обновляются", self.window.camera_label.text())
        self.window.clear_plots()
        self.assertIsNone(self.window.path_curve.xData)
        self.assertEqual(0.42, history.latest["planar_odometry"]["x_m"])
        path = self.window.record_path
        QTest.mouseClick(self.window.stop_button, Qt.MouseButton.LeftButton)
        self.wait_for(lambda: self.window.worker is None)
        self.assertEqual(4, len(path.read_text().splitlines()))
        self.assertTrue(self.window.start_button.isEnabled())
        self.start()
        self.assertNotEqual(path, self.window.record_path)
        self.assertEqual(0, len(self.window.history.streams))
        self.window.close()
        self.wait_for(lambda: self.window.worker is None)
        self.assertFalse(self.window.isVisible())

    def test_epoch_and_session_changes_do_not_connect_unrelated_paths(self):
        self.window.history.observe(display_event(session="one"))
        self.window.history.observe(display_event(session="two", x_m=3))
        self.window.refresh()
        self.assertEqual(3, self.window.stream_combo.count())
        self.assertEqual([3], list(self.window.path_curve.xData))
        self.window.stream_combo.setCurrentIndex(1)
        self.window.refresh()
        self.assertEqual([0.42], list(self.window.path_curve.xData))
        self.window.history.observe(display_event(sequence=1, session="one", epoch=3, x_m=0))
        self.window.refresh()
        self.assertEqual([0], list(self.window.path_curve.xData))
        self.assertTrue(self.window.trajectory_plot.getViewBox().state["yInverted"])

    def test_lidar_over_udp_is_displayed_recorded_and_replayed(self):
        address = self.start()
        event = display_event(height_source="IPHONE_LIDAR", height_valid=True,
                              surface_distance_m=1.2, lidar_depth_m=1.4,
                              lidar_age_ms=50.0, lidar_rtt_ms=20.0, lidar_time_s=10.5)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            sender.sendto(json.dumps(event["payload"]).encode(), address)
        self.wait_for(lambda: "iPhone LiDAR" in self.window.quality_label.text())
        self.assertIn("1.200", self.window.quality_label.text())
        self.assertIn("1.400", self.window.quality_label.text())
        event = display_event(sequence=1, frame_stamp=200, status="DEGRADED",
                              height_source="IPHONE_LIDAR", height_valid=False,
                              lidar_depth_m=None, lidar_age_ms=None, lidar_rtt_ms=None, lidar_time_s=None)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            sender.sendto(json.dumps(event["payload"]).encode(), address)
        self.wait_for(lambda: "нет свежего LiDAR" in self.window.quality_label.text())
        path = self.window.record_path
        self.window.stop_receiving()
        self.wait_for(lambda: self.window.worker is None)
        self.assertIn('"height_source":"IPHONE_LIDAR"', path.read_text().replace(" ", ""))
        self.window.open_recording(path)
        self.wait_for(lambda: self.window.replay_engine is not None and self.window.replay_engine.cursor > 0)
        self.assertIn("iPhone LiDAR", self.window.quality_label.text())
        self.assertIn("1.400", self.window.quality_label.text())

    def test_bad_bind_address_and_occupied_port_recover_controls(self):
        self.window.host_edit.setText("not an address")
        self.window.start_receiving()
        self.assertIsNone(self.window.worker)
        self.assertIn("IPv4", self.window.error_label.text())
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as occupied:
            occupied.bind(("127.0.0.1", 0))
            self.window.host_edit.setText("127.0.0.1")
            self.window.port_spin.setValue(occupied.getsockname()[1])
            self.window.start_receiving()
            self.wait_for(lambda: self.window.worker is None)
            self.assertTrue(self.window.start_button.isEnabled())
            self.assertIn("Error", self.window.error_label.text())

    def test_separate_50hz_simulator_updates_live_gui_without_losing_recording(self):
        address = self.start()
        simulator = subprocess.Popen(
            [sys.executable, "-m", "pc_app.motion_viewer", "simulate", "--duration", "1",
             "--rate", "50", "--port", str(address[1])],
            cwd=Path(__file__).resolve().parents[2], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True,
        )
        try:
            self.wait_for(lambda: self.window._snapshot["datagrams"] == 50)
            stdout, stderr = simulator.communicate(timeout=3)
            self.assertEqual(0, simulator.returncode, stderr)
            self.assertIn("SIMULATED", stdout)
            self.assertIn("СИМУЛЯЦИЯ", self.window.stream_label.text())
            self.assertEqual(50, len(self.window.path_curve.xData), {
                "snapshot": self.window._snapshot,
                "histories": [(key, len(history.trajectory), history.last_sequence)
                              for key, history in self.window.history.streams.items()],
            })
            self.assertEqual(50, len(self.window.sensor_curves["gyroscope"][0].xData))
            self.assertFalse(self.window.sensor_plots["gyroscope"].getAxis("left").autoSIPrefix)
            self.assertEqual(0, self.window._display_dropped)
            self.assertEqual("", self.window.error_label.text())
            self.window.stop_receiving()
            self.wait_for(lambda: self.window.worker is None)
            self.assertEqual(50, len(self.window.record_path.read_text().splitlines()))
        finally:
            if simulator.poll() is None:
                simulator.kill()
                simulator.communicate()

    def test_replay_seek_updates_model_plots_and_paused_recorded_time(self):
        path = Path(self.directory.name) / "replay.jsonl"
        write_session(path, 20)
        original = path.read_bytes()
        self.window.open_recording(path)
        self.wait_for(lambda: self.window.replay_engine is not None and self.window.replay_engine.cursor > 0)
        self.assertFalse(self.window.start_button.isEnabled())
        self.assertEqual((1, 0, 0, 0), self.window.phone_view.quaternion)
        self.window.seek_replay(1.0)
        self.wait_for(lambda: not self.window._seeking)
        self.assertEqual(11, self.window.replay_engine.cursor)
        self.assertEqual(11, len(self.window.path_curve.xData))
        self.assertAlmostEqual(-1, self.window.phone_view.matrix[0, 1])
        self.assertFalse(self.window.phone_view.stale)
        self.window.phone_view.use_current_as_reference()
        self.assertAlmostEqual(1, self.window.phone_view.matrix[0, 0])
        self.window.seek_replay(0)
        self.wait_for(lambda: not self.window._seeking)
        self.assertEqual(1, len(self.window.path_curve.xData))
        self.window.phone_view.reset_reference()
        self.assertAlmostEqual(1, self.window.phone_view.matrix[0, 0])
        self.window.speed_combo.setCurrentIndex(self.window.speed_combo.findData(8.0))
        self.window.toggle_playback()
        self.wait_for(lambda: not self.window.playback_clock.playing)
        self.assertEqual(20, self.window.replay_engine.cursor)
        self.assertIn("ПАУЗА", self.window.status_label.text())
        self.assertEqual(original, path.read_bytes())
        self.window.close_recording()
        self.assertTrue(self.window.start_button.isEnabled())
        self.assertIsNone(self.window.phone_view.quaternion)

    def test_corrupt_replay_does_not_replace_previous_recording(self):
        path = Path(self.directory.name) / "valid.jsonl"
        write_session(path)
        self.window.open_recording(path)
        self.wait_for(lambda: self.window.replay_engine is not None)
        previous = self.window.replay_engine
        broken = path.with_name("broken.jsonl")
        broken.write_text("{bad")
        self.window.open_recording(broken)
        self.wait_for(lambda: self.window.replay_loader is None)
        self.assertIs(previous, self.window.replay_engine)
        self.assertIn("Строка 1", self.window.error_label.text())

    def test_new_recording_always_opens_at_start_and_live_replay_are_exclusive(self):
        first = Path(self.directory.name) / "first.jsonl"
        second = Path(self.directory.name) / "second.jsonl"
        write_session(first, 20)
        write_session(second, 2)
        self.window.open_recording(first)
        self.wait_for(lambda: self.window.replay_engine is not None)
        self.window.seek_replay(1.9)
        self.wait_for(lambda: not self.window._seeking)
        self.window.open_recording(second)
        self.wait_for(lambda: self.window.replay_loader is None)
        self.assertEqual(0, self.window.playback_clock.position)
        self.assertEqual(1, self.window.replay_engine.cursor)
        self.window.start_receiving()
        self.assertIsNone(self.window.worker)
        self.window.close_recording()
        self.start()
        self.window.open_recording(first)
        self.assertIsNone(self.window.replay_loader)
        self.assertIn("остановите", self.window.error_label.text())

    def test_preferences_survive_new_window(self):
        store = Preferences(Path(self.directory.name) / "data")
        self.window.preferences = store
        self.window.host_edit.setText("127.0.0.1")
        self.window.port_spin.setValue(19601)
        self.window.record_check.setChecked(False)
        self.window.speed_combo.setCurrentIndex(self.window.speed_combo.findData(4.0))
        self.window.save_preferences()
        other = MotionViewer(preferences=store)
        try:
            self.assertEqual(19601, other.port_spin.value())
            self.assertFalse(other.record_check.isChecked())
            self.assertEqual(4.0, other.speed_combo.currentData())
        finally:
            other.close()
            other.deleteLater()

    def test_close_during_background_load_cancels_without_leaking_file(self):
        path = Path(self.directory.name) / "many.jsonl"
        write_session(path, 1500)
        self.window.open_recording(path)
        self.window.close()
        self.wait_for(lambda: self.window.replay_loader is None)
        self.assertFalse(self.window.isVisible())

    def test_budgeted_seek_keeps_qt_event_loop_responsive(self):
        from PySide6.QtCore import QTimer
        path = Path(self.directory.name) / "seek.jsonl"
        write_session(path, 3000)
        self.window.open_recording(path)
        self.wait_for(lambda: self.window.replay_engine is not None)
        beats = []
        heartbeat = QTimer()
        heartbeat.setInterval(5)
        heartbeat.timeout.connect(lambda: beats.append(1))
        heartbeat.start()
        try:
            self.window.seek_replay(self.window.playback_clock.duration)
            self.wait_for(lambda: not self.window._seeking, timeout=8)
            self.assertGreater(len(beats), 1)
            self.assertEqual(3000, self.window.replay_engine.cursor)
        finally:
            heartbeat.stop()
