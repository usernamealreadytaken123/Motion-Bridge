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


def exercise_h264_codec():
    """Verify that the frozen bundle contains both the AVC encoder libraries and decoder path."""
    import av
    import numpy as np
    from fractions import Fraction
    from .video_decoder import H264Decoder
    encoder = av.CodecContext.create("libx264", "w")
    encoder.width = encoder.height = 64
    encoder.pix_fmt = "yuv420p"
    encoder.time_base = Fraction(1, 1_000_000_000)
    encoder.framerate = 15
    encoder.bit_rate = 250_000
    encoder.options = {"preset": "ultrafast", "tune": "zerolatency", "bf": "0", "g": "15"}
    encoder.open()
    packets = []
    for index in range(2):
        frame = av.VideoFrame.from_ndarray(np.full((64, 64), 40+index*80, np.uint8), format="gray")
        frame.pts = 1_000_000_000+index*66_666_667
        packets.extend(encoder.encode(frame))
    packets.extend(encoder.encode(None))
    first = bytes(packets[0])
    starts = [i for i in range(len(first)-3) if first[i:i+4] == b"\x00\x00\x00\x01"]+[len(first)]
    config = b"".join(first[starts[i]:starts[i+1]] for i in range(len(starts)-1)
                      if first[starts[i]+4] & 0x1f in (7, 8))
    decoder = H264Decoder()
    decoder.configure(dict(width=64, height=64), config)
    output = []
    for index, packet in enumerate(packets):
        stamp = 1_000_000_000+index*66_666_667
        output.extend(decoder.decode(dict(timestamp_ns=stamp, width=64, height=64, seq=index), bytes(packet)))
    if len(output) != 2 or any(len(pixels) != 4096 for _, pixels in output):
        raise RuntimeError("Bundled H.264 round-trip failed")
    return True


def exercise_pc_odometry(window, wait_for, report):
    """Pixel-driven TCP -> Qt -> raw disk -> recomputation, also inside the frozen EXE."""
    import cv2
    import numpy as np
    from .raw_protocol import encode
    window.open_pc_odometry()
    raw = window.odometry_window
    if raw is None:
        raise RuntimeError(window.error_label.text())
    raw.host.setText("127.0.0.1")
    raw.port.setMinimum(0)
    raw.port.setValue(0)
    raw.start()
    worker = raw.worker
    wait_for(lambda: worker.update["address"] is not None)
    with socket.create_connection(worker.update["address"], timeout=3) as sender:
        sender.sendall(encode(dict(type="hello", protocol=1, session="pc-smoke", token=raw.token.text())))
        if sender.recv(1) != b"\x01":
            raise RuntimeError("Raw handshake failed")
        image = cv2.GaussianBlur(np.random.default_rng(123).integers(0, 256, (480, 640), dtype=np.uint8), (3, 3), .7)
        for i in range(8):
            stamp = 1_000_000_000+i*100_000_000
            moved = cv2.warpAffine(image, np.float32([[1, 0, -4*i], [0, 1, 2*i]]), (640, 480))
            header = dict(type="frame", seq=i, timestamp_ns=stamp, width=640, height=480, rotation=0,
                          sensor_orientation=0, realtime=True, back_camera=True, approximate=False,
                          intrinsics=[500., 500., 319.5, 239.5], dropped=0)
            sender.sendall(encode(header, moved.tobytes()))
            sender.sendall(encode(dict(type="sensors", watermark_ns=stamp+140_000_000,
                samples=[["q", t, 1., 0., 0., 0.] for t in range(stamp-60_000_000, stamp+140_000_001, 20_000_000)])))
        wait_for(lambda: raw.last_seq == 7)
        raw.stop()
        wait_for(lambda: raw.worker is None)
    result = worker.update["result"]
    if worker.update["error"] or result["status"] != "TRACKING" or abs(result["x_m"]-.056) > .001 or abs(result["y_m"]+.028) > .001:
        raise RuntimeError(f"Pixel odometry failed: {worker.update['error']}; {result}")
    if len(raw.path) != 8 or raw.preview.pixmap().isNull() or "TRACKING" not in raw.metrics.text():
        raise RuntimeError("PC odometry did not reach Qt widgets")
    report.parent.mkdir(parents=True, exist_ok=True)
    if not raw.grab().save(str(report.with_name(report.stem+"-pc-odometry.png"))):
        raise RuntimeError("Cannot render PC odometry window")
    capture = Path(worker.update["recording"]) / "capture.mvo"
    raw.start(replay_path=capture)
    replay = raw.worker
    wait_for(lambda: raw.worker is None)
    if replay.update["error"] or replay.update["result"] != result:
        raise RuntimeError("Raw-session recalculation differs from live PC results")
    raw.close()
    # Closing the parent must cancel a listening child without blocking Qt.
    window.open_pc_odometry()
    window.odometry_window.port.setMinimum(0)
    window.odometry_window.port.setValue(0)
    window.odometry_window.start()
    return dict(frames=8, x_m=result["x_m"], y_m=result["y_m"], raw_recompute=True)


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
                        "height_source": "IPHONE_LIDAR", "height_valid": True,
                        "lidar_depth_m": 1.4, "lidar_age_ms": 50,
                        "lidar_rtt_ms": 20, "lidar_time_s": sequence / 50,
                    },
                }
                sender.sendto(json.dumps(packet).encode(), window.worker._address)
                QTest.qWait(20)
        wait_for(lambda: window._snapshot["datagrams"] == 40)
        if abs(window.phone_view.matrix[2, 1] - math.sin(1.4)) > 1e-6:
            raise RuntimeError("Orientation rendering state is incorrect")
        if "iPhone LiDAR" not in window.quality_label.text() or "1.400" not in window.quality_label.text():
            raise RuntimeError("LiDAR diagnostics did not reach the packaged UI")
        path = window.record_path
        window.stop_receiving()
        wait_for(lambda: window.worker is None)
        window.open_recording(path)
        wait_for(lambda: window.replay_engine is not None)
        window.seek_replay(window.playback_clock.duration)
        wait_for(lambda: not window._seeking)
        if window.replay_engine.cursor != 40 or len(window.path_curve.xData) != 40:
            raise RuntimeError("Replay did not reproduce the recording")
        if "iPhone LiDAR" not in window.quality_label.text():
            raise RuntimeError("Replay lost LiDAR diagnostics")
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
        pc_result = exercise_pc_odometry(window, wait_for, report)
        h264_roundtrip = exercise_h264_codec()
        from .mp4_smoke import exercise_mp4
        mp4_result = exercise_mp4(window, wait_for, report)
        window.port_spin.setValue(19500)
        window.close()
        wait_for(lambda: not window.isVisible())
        if Preferences(directory).load().get("port") != 19500:
            raise RuntimeError("Settings did not survive close/reload")
        result = {"ok": True, "frozen": bool(getattr(sys, "frozen", False)),
                  "executable": sys.executable, "records": 40, "replay_records": 40,
                  "settings_roundtrip": True, "lidar_roundtrip": True,
                  "h264_roundtrip": h264_roundtrip,
                  "pc_odometry": pc_result,
                  "mp4_odometry": mp4_result,
                  "recording": str(path), "qt_platform": app.platformName()}
        report.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        LOGGER.info("Packaged smoke test passed")
    finally:
        window.close()
        wait_for(lambda: window.worker is None and window.replay_loader is None)
        app.processEvents()
