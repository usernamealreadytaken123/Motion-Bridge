"""A separate PC-calculation workspace; legacy telemetry UI stays unchanged."""
from collections import deque
import ipaddress
from pathlib import Path
import secrets
import time
import cv2
import numpy as np
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QLineEdit, QSpinBox, QDoubleSpinBox, QCheckBox, QFileDialog, QSplitter)
import pyqtgraph as pg
from .raw_worker import RawOdometryWorker


class OdometryWindow(QMainWindow):
    def __init__(self, record_directory, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Визуальная одометрия — расчёт на ПК")
        self.resize(1200, 820)
        self.record_directory = Path(record_directory)
        self.worker = None
        self.closing = False
        self.path = deque(maxlen=100000)
        self.last_seq = None
        self.last_result_time = 0
        central = QWidget()
        layout = QVBoxLayout(central)
        controls = QHBoxLayout()
        self.host = QLineEdit("0.0.0.0")
        self.host.setMaximumWidth(140)
        self.port = QSpinBox()
        self.port.setRange(1, 65535)
        self.port.setValue(5006)
        self.token = QLineEdit(secrets.token_hex(4))
        self.token.setMaximumWidth(110)
        self.token.setReadOnly(True)
        self.height = QDoubleSpinBox()
        self.height.setRange(.05, 20)
        self.height.setDecimals(3)
        self.height.setValue(1.)
        self.height.setSuffix(" м")
        self.lidar = QCheckBox("Высота LiDAR")
        self.record = QCheckBox("Запись исходных данных")
        self.record.setChecked(True)
        self.record.setToolTip("RAW: примерно 276 МБ/мин; H.264 1,5 Мбит/с: около 11 МБ/мин.")
        for text, widget in (("IP ПК", self.host), ("TCP", self.port), ("Код", self.token), ("h", self.height)):
            controls.addWidget(QLabel(text)); controls.addWidget(widget)
        controls.addWidget(self.lidar); controls.addWidget(self.record)
        layout.addLayout(controls)
        actions = QHBoxLayout()
        self.start_button = QPushButton("Ожидать Android")
        self.stop_button = QPushButton("Остановить")
        self.stop_button.setEnabled(False)
        self.recompute_button = QPushButton("Пересчитать .mvo…")
        self.folder_button = QPushButton("Выбрать папку записи…")
        self.start_button.clicked.connect(self.start)
        self.stop_button.clicked.connect(self.stop)
        self.recompute_button.clicked.connect(self.recompute)
        self.folder_button.clicked.connect(self.choose_folder)
        for widget in (self.start_button, self.stop_button, self.recompute_button, self.folder_button):
            actions.addWidget(widget)
        layout.addLayout(actions)
        note = QLabel("На Android: IP этого ПК, TCP-порт и код. Сначала «Ожидать Android», затем передача кадров. "
                      "Для сравнения включите локальный расчёт на Android с такой же h. Только доверенная локальная сеть. "
                      "На Android выберите H.264 для экономии трафика или RAW для эталонного сравнения.")
        note.setWordWrap(True)
        layout.addWidget(note)
        self.status = QLabel("Расчёт не запущен")
        self.status.setWordWrap(True)
        self.status.setTextFormat(Qt.TextFormat.PlainText)
        layout.addWidget(self.status)
        self.metrics = QLabel("X: —   Y: —")
        self.metrics.setWordWrap(True)
        layout.addWidget(self.metrics)
        splitter = QSplitter()
        self.preview = QLabel("Изображение с телефона")
        self.preview.setMinimumSize(320, 240)
        self.preview.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.plot = pg.PlotWidget(title="XY, вычисленные на ПК")
        self.plot.setLabel("bottom", "X", "m")
        self.plot.setLabel("left", "Y", "m")
        self.plot.setAspectLocked(True)
        self.plot.invertY(True)
        self.curve = self.plot.plot(pen=pg.mkPen("#56d6bf", width=2), connect="finite")
        splitter.addWidget(self.preview); splitter.addWidget(self.plot)
        layout.addWidget(splitter, 1)
        self.details = QLabel("GRAY8 · OpenCV · компенсация наклона · без интегрирования ускорения")
        self.details.setWordWrap(True)
        layout.addWidget(self.details)
        self.setCentralWidget(central)
        self.timer = QTimer(self)
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.refresh)
        self.timer.start()

    def choose_folder(self):
        chosen = QFileDialog.getExistingDirectory(self, "Папка исходных сессий", str(self.record_directory))
        if chosen:
            self.record_directory = Path(chosen)

    def set_running(self, running):
        for widget in (self.start_button, self.recompute_button, self.folder_button, self.host, self.port,
                       self.height, self.lidar, self.record):
            widget.setEnabled(not running)
        self.stop_button.setEnabled(running)

    def start(self, replay_path=None):
        if self.worker:
            return
        # clicked(bool) is not a filename.
        if isinstance(replay_path, bool):
            replay_path = None
        try:
            if not replay_path:
                ipaddress.IPv4Address(self.host.text().strip())
            self.worker = RawOdometryWorker(self.host.text().strip(), self.port.value(), self.token.text(),
                self.height.value(), self.lidar.isChecked(), self.record_directory if self.record.isChecked() else None,
                replay_path=replay_path)
            self.path.clear()
            self.curve.setData([], [])
            self.last_seq = None
            self.last_result_time = 0
            self.metrics.setText("X: —   Y: —")
            self.status.setText("Подготовка…")
            self.preview.clear()
            self.set_running(True)
            self.worker.start()
        except Exception as exc:
            self.worker = None
            self.set_running(False)
            self.status.setText(str(exc))

    def recompute(self):
        path, _ = QFileDialog.getOpenFileName(self, "Исходная сессия для повторного расчёта",
                                             str(self.record_directory), "Исходные кадры и датчики (*.mvo)")
        if path:
            self.start(replay_path=path)

    def stop(self):
        if self.worker:
            self.worker.stop()

    def refresh(self):
        if not self.worker:
            return
        update = self.worker.snapshot()
        result = update["result"]
        self.status.setText(update["error"] or update["status"])
        if result and result["seq"] != self.last_seq:
            self.last_seq = result["seq"]
            self.last_result_time = time.monotonic()
            for item in update["results"]:
                self.path.append((item["x_m"], item["y_m"]) if item["status"] == "TRACKING" else (np.nan, np.nan))
            if self.path:
                points = np.array(self.path)
                self.curve.setData(points[:, 0], points[:, 1])
            h = f"{result['height_m']:.3f} м" if result["height_valid"] else "нет согласованной высоты"
            tilt = f"{result['tilt_deg']:.1f}°" if result["tilt_deg"] is not None else "—"
            self.metrics.setText(f"X {result['x_m']:+.3f} м   Y {result['y_m']:+.3f} м   h {h}   наклон {tilt}\n"
                                 f"{result['status']} · {result['message']} · точки {result['inliers']}/{result['tracked_points']}\n"
                                 f"Калибровка камеры: {'приближённая — возможна ошибка масштаба' if result['approximate'] else 'из метаданных Android'}")
            gray = update["image"]
            shown = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
            for x0, y0, x1, y1 in result["tracks"]:
                cv2.line(shown, (round(x0), round(y0)), (round(x1), round(y1)), (86, 214, 191), 1)
                cv2.circle(shown, (round(x1), round(y1)), 2, (255, 189, 105), -1)
            image = QImage(shown.data, shown.shape[1], shown.shape[0], shown.strides[0], QImage.Format.Format_RGB888).copy()
            self.preview.setPixmap(QPixmap.fromImage(image).scaled(self.preview.size(), Qt.AspectRatioMode.KeepAspectRatio,
                                                                 Qt.TransformationMode.SmoothTransformation))
        if not update["finished"] and self.last_result_time and time.monotonic()-self.last_result_time > 1.5:
            self.status.setText("Нет новых обработанных кадров >1.5 с · " + (update["error"] or update["status"]))
        self.details.setText(f"Поток: {update['encoding'] or 'ожидание конфигурации'} · "
                             f"обработано кадров: {update['frames']} · сообщений: {update['messages']} · "
                             f"изображения: {update['bytes']/1e6:.1f} МБ\n"
                             f"Пропущено кадров до расчёта: {result['dropped_frames'] if result else 0} · "
                             f"Событий датчиков: {result['sensor_samples'] if result else 0}\n"
                             f"Запись: {update['recording'] or 'выключена / повторный расчёт'}")
        if update["finished"]:
            self.worker = None
            self.set_running(False)
            if self.closing:
                self.close()

    def closeEvent(self, event):
        if self.worker:
            self.closing = True
            self.stop()
            event.ignore()
        else:
            self.timer.stop()
            event.accept()
