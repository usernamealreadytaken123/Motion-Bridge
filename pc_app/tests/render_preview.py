"""Render the actual Qt widgets offscreen with explicitly synthetic UDP input."""

import json
import math
import os
from pathlib import Path
import socket
import time

os.environ["QT_QPA_PLATFORM"] = "offscreen"
from PySide6.QtTest import QTest
from PySide6.QtGui import QFont, QFontDatabase
from PySide6.QtWidgets import QApplication

from pc_app.motion_viewer.gui import MotionViewer
from pc_app.tests.test_history import display_event


def main() -> None:
    app = QApplication([])
    # The Windows offscreen plugin does not discover system fonts by itself.
    if os.name == "nt":
        fonts = Path(os.environ.get("WINDIR", "C:/Windows")) / "Fonts"
        for filename in ("segoeui.ttf", "segoeuib.ttf"):
            if QFontDatabase.addApplicationFont(str(fonts / filename)) < 0:
                raise RuntimeError(f"Preview font unavailable: {filename}")
        app.setFont(QFont("Segoe UI", 10))
    window = MotionViewer("127.0.0.1", 5005)
    window.record_check.setChecked(False)
    window.port_spin.setMinimum(0)
    window.port_spin.setValue(0)
    window.show()
    window.start_receiving()
    try:
        deadline = time.monotonic() + 4
        while window.worker._address is None and time.monotonic() < deadline:
            QTest.qWait(20)
        if window.worker._address is None:
            raise RuntimeError("Preview receiver did not bind")
        window.port_spin.setValue(window.worker._address[1])
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            for sequence in range(250):
                angle = sequence / 250 * 5.2
                payload = display_event(sequence=sequence, session="sim-ui-preview",
                                        frame_stamp=sequence + 1, x_m=math.cos(angle) - 1,
                                        y_m=math.sin(angle))["payload"]
                for index, axis in enumerate("xyz"):
                    payload["gyroscope"][axis] = round(math.sin(angle * 2 + index) * 0.4, 4)
                    payload["linear_acceleration"][axis] = round(math.cos(angle * 3 + index) * 0.7, 4)
                payload["sensor_timestamps_ns"] = {name: sequence + 1 for name in (
                    "quaternion", "gyroscope", "linear_acceleration")}
                sender.sendto(json.dumps(payload).encode(), window.worker._address)
                QTest.qWait(20)
        QTest.qWait(150)
        window.refresh()
        path = Path(__file__).resolve().parents[1] / "artifacts/gui-preview.png"
        path.parent.mkdir(parents=True, exist_ok=True)
        if not window.grab().save(str(path)):
            raise RuntimeError("Could not render preview")
        print(path)
        window.resize(1100, 760)
        QTest.qWait(100)
        compact = path.with_name("gui-preview-compact.png")
        if not window.grab().save(str(compact)):
            raise RuntimeError("Could not render compact preview")
        print(compact)
    finally:
        window.close()
        deadline = time.monotonic() + 12
        while window.worker is not None and time.monotonic() < deadline:
            QTest.qWait(20)
        if window.worker is not None:
            raise RuntimeError("Preview worker did not stop")
        app.processEvents()


if __name__ == "__main__":
    main()
