"""Optional packaged-app diagnostic. Exercises bundled Qt, UDP, recording and replay."""

import json
import math
import os
from pathlib import Path
import socket
import sys
import time

from PySide6.QtGui import QFont, QFontDatabase
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

from .gui import MotionViewer
from .preferences import LOGGER, Preferences


def run_smoke_test(report: Path, directory: Path):
    app = QApplication.instance() or QApplication([])
    app.setQuitOnLastWindowClosed(False)
    if os.name == "nt":
        fonts = Path(os.environ.get("WINDIR", "C:/Windows")) / "Fonts"
        for filename in ("segoeui.ttf", "segoeuib.ttf"):
            QFontDatabase.addApplicationFont(str(fonts / filename))
        app.setFont(QFont("Segoe UI", 10))
    window = MotionViewer("127.0.0.1", 5005, Preferences(directory))
    window.record_directory = directory / "recordings"
    window.record_check.setChecked(True)
    window.port_spin.setMinimum(0)
    window.port_spin.setValue(0)
    window.show()

    def wait_for(predicate, timeout=12):
        deadline = time.perf_counter() + timeout
        while not predicate() and time.perf_counter() < deadline:
            time.sleep(0.005)
            QTest.qWait(20)
        if not predicate():
            raise RuntimeError("Packaged smoke test timed out")

    try:
        window.start_receiving()
        wait_for(lambda: window.worker is not None and window.worker._address is not None)
        window.port_spin.setValue(window.worker._address[1])
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            for sequence in range(40):
                angle = sequence / 39 * 1.4
                stamp = 1_000_000_000 + sequence * 20_000_000
                packet = {
                    "protocol_version": 2, "packet_type": "telemetry", "session_id": "sim-exe-test",
                    "sequence": sequence, "timestamp": 1_700_000_000 + sequence / 50,
                    "device_timestamp_ns": stamp,
                    "quaternion": {"w": math.cos(angle/2), "x": math.sin(angle/2), "y": 0, "z": 0},
                    "gyroscope": {"x": .5, "y": math.sin(angle), "z": math.cos(angle)},
                    "linear_acceleration": {"x": math.sin(angle*3), "y": math.cos(angle*3), "z": 0},
                    "height_above_surface_m": 1,
                    "sensor_timestamps_ns": {name: stamp for name in ("quaternion", "gyroscope", "linear_acceleration")},
                    "planar_odometry": {
                        "epoch": 1, "timestamp_ns": stamp, "status": "TRACKING", "camera_active": True,
                        "x_m": math.cos(angle)-1, "y_m": math.sin(angle), "heading_rad": angle,
                        "delta_x_m": 0, "delta_y_m": 0, "surface_distance_m": 1,
                        "tracked_points": 200, "inliers": 180, "inlier_ratio": .9, "image_scale": 1,
                        "intrinsics_approximate": True,
                    },
                }
                sender.sendto(json.dumps(packet).encode(), window.worker._address)
                QTest.qWait(20)
        wait_for(lambda: window._snapshot["datagrams"] == 40)
        if abs(window.phone_view.matrix[2, 1] - math.sin(1.4)) > 1e-6:
            raise RuntimeError("Orientation rendering state is incorrect")
        path = window.record_path
        window.stop_receiving()
        wait_for(lambda: window.worker is None)
        window.open_recording(path)
        wait_for(lambda: window.replay_engine is not None)
        window.seek_replay(window.playback_clock.duration)
        wait_for(lambda: not window._seeking)
        if window.replay_engine.cursor != 40 or len(window.path_curve.xData) != 40:
            raise RuntimeError("Replay did not reproduce the recording")
        window.phone_view.use_current_as_reference()
        if abs(window.phone_view.matrix[2, 2] - 1) > 1e-6:
            raise RuntimeError("Orientation zero reference failed")
        window.phone_view.reset_reference()
        report.parent.mkdir(parents=True, exist_ok=True)
        if not window.grab().save(str(report.with_suffix(".png"))):
            raise RuntimeError("Qt could not render the window")
        window.scene_tabs.setCurrentIndex(1)
        QTest.qWait(50)
        window.grab().save(str(report.with_name(report.stem + "-trajectory.png")))
        window.port_spin.setValue(19500)
        window.close()
        wait_for(lambda: not window.isVisible())
        if Preferences(directory).load().get("port") != 19500:
            raise RuntimeError("Settings did not survive close/reload")
        result = {"ok": True, "frozen": bool(getattr(sys, "frozen", False)),
                  "executable": sys.executable, "records": 40, "replay_records": 40,
                  "settings_roundtrip": True, "recording": str(path), "qt_platform": app.platformName()}
        report.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        LOGGER.info("Packaged smoke test passed")
    finally:
        window.close()
        wait_for(lambda: window.worker is None and window.replay_loader is None)
        app.processEvents()
