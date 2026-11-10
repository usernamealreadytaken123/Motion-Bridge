"""Desktop monitor. All widgets and plot updates belong to the main Qt thread."""

from datetime import datetime
import ipaddress
from pathlib import Path
import time
import sys

import numpy as np
from PySide6.QtCore import QTimer, Qt, QUrl
from PySide6.QtGui import QCloseEvent, QDesktopServices
from PySide6.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QFileDialog, QFormLayout, QGroupBox,
    QHBoxLayout, QLabel, QLineEdit, QMainWindow, QPushButton, QSpinBox,
    QSlider, QSplitter, QTabWidget, QVBoxLayout, QWidget,
    QScrollArea, QSizePolicy,
)
import pyqtgraph as pg

from .history import DisplayHistory, STALE_NS
from .worker import ReceiverWorker
from .phone_view import PhoneView
from .preferences import LOGGER, Preferences
from .replay import PlaybackClock, ReplayEngine, ReplayLoader


COLORS = ("#56d6bf", "#ffbd69", "#8faaff")
STYLES = """
QMainWindow, QWidget { background: #101824; color: #dce6f3; font: 10pt 'Segoe UI'; }
QLabel#title { font-size: 23pt; font-weight: 700; color: #f2f7ff; }
QLabel#muted { color: #9aaec5; }
QLabel#status { background: #192b3b; padding: 10px; border-radius: 6px; }
QLabel#error { color: #ffb8a6; }
QGroupBox { border: 1px solid #2c3e53; border-radius: 8px; margin-top: 12px;
            padding: 16px 10px 10px; font-weight: 600; }
QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 5px; }
QLineEdit, QSpinBox, QComboBox { border: 1px solid #3a4b61; padding: 6px;
                              border-radius: 4px; background: #182536; }
QPushButton { background: #263c53; border: 1px solid #405974; border-radius: 5px; padding: 8px; }
QPushButton:hover { background: #34516d; }
QPushButton:disabled, QLineEdit:disabled, QSpinBox:disabled { color: #6c7c91; }
QPushButton#primary { background: #197b73; border-color: #299b8f; font-weight: 600; }
QPushButton#primary:disabled { background: #244743; color: #819895; }
QSplitter::handle { background: #243448; }
QTabWidget::pane { border: 1px solid #2c3e53; }
QTabBar::tab { background: #192b3b; padding: 8px 12px; }
QTabBar::tab:selected { background: #294459; color: #56d6bf; }
"""


def _label(text: str = "—", name: str = "") -> QLabel:
    label = QLabel(text)
    label.setWordWrap(True)
    if name:
        label.setObjectName(name)
    label.setTextFormat(Qt.TextFormat.PlainText)
    label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
    return label


class MotionViewer(QMainWindow):
    def __init__(self, host: str | None = None, port: int | None = None,
                 preferences: Preferences | None = None):
        super().__init__()
        self.preferences = preferences
        self.settings = preferences.load() if preferences else {}
        host = host if host is not None else self.settings.get("host", "0.0.0.0")
        port = port if port is not None else self.settings.get("port", 5005)
        self.setWindowTitle("Motion Viewer — Android → ПК")
        self.resize(1360, 860)
        self.setMinimumSize(1100, 760)
        self.setStyleSheet(STYLES)
        self.history = DisplayHistory()
        self.worker: ReceiverWorker | None = None
        self.odometry_window = None
        self.mp4_window = None
        self._closing = False
        self._stopping = False
        self._display_dropped = 0
        self._snapshot = {"streams": [], "datagrams": 0, "invalid": 0}
        self._plot_key = None
        self._stream_keys = []
        self._orientation_key = None
        self.replay_engine = None
        self.playback_clock = None
        self.replay_loader = None
        self._seeking = False
        self._resume_after_seek = False
        self.record_path: Path | None = None
        self.record_directory = Path(__file__).resolve().parents[1] / "recordings"
        if preferences:
            self.record_directory = Path(self.settings.get("record_directory", preferences.directory / "recordings"))
        self.replay_directory = Path(self.settings.get("replay_directory", self.record_directory))
        self._build_ui(host, port)
        self.record_check.setChecked(self.settings.get("record", True))
        self.speed_combo.setCurrentIndex(self.speed_combo.findData(self.settings.get("speed", 1.0)))
        self.resize(self.settings.get("width", 1360), self.settings.get("height", 860))
        if preferences and preferences.warning:
            self.error_label.setText(preferences.warning)
        self.timer = QTimer(self)
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.refresh)
        self.timer.start()
        self.replay_pump = QTimer(self)
        self.replay_pump.setSingleShot(True)
        self.replay_pump.setInterval(0)
        self.replay_pump.timeout.connect(self.refresh)

    def _build_ui(self, host: str, port: int) -> None:
        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(22, 18, 22, 16)
        root.setSpacing(12)
        title_row = QHBoxLayout()
        title_row.addWidget(_label("Motion Viewer", "title"), 1)
        self.pc_odometry_button = QPushButton("Расчёт одометрии на ПК · исходные кадры и датчики…")
        self.pc_odometry_button.clicked.connect(self.open_pc_odometry)
        title_row.addWidget(self.pc_odometry_button)
        self.mp4_button = QPushButton("Загрузить MP4…")
        self.mp4_button.clicked.connect(self.open_mp4)
        title_row.addWidget(self.mp4_button)
        root.addLayout(title_row)
        root.addWidget(_label("ANDROID → ПК    /    3D-ОРИЕНТАЦИЯ · ТЕЛЕМЕТРИЯ · 2D-ОДОМЕТРИЯ", "muted"))
        replay_row = QHBoxLayout()
        self.open_button = QPushButton("Открыть JSONL…")
        self.open_button.clicked.connect(lambda: self.open_recording())
        self.replay_label = _label("Живой режим · запись можно открыть после остановки UDP", "muted")
        self.close_replay_button = QPushButton("Закрыть запись")
        self.close_replay_button.setEnabled(False)
        self.close_replay_button.clicked.connect(self.close_recording)
        for widget in (self.open_button, self.replay_label, self.close_replay_button):
            replay_row.addWidget(widget, 1 if widget is self.replay_label else 0)
        root.addLayout(replay_row)
        playback_row = QHBoxLayout()
        self.rewind_button = QPushButton("В начало")
        self.rewind_button.clicked.connect(lambda: self.seek_replay(0.0))
        self.play_button = QPushButton("Воспроизвести")
        self.play_button.clicked.connect(self.toggle_playback)
        self.timeline = QSlider(Qt.Orientation.Horizontal)
        self.timeline.setRange(0, 10000)
        self.timeline.setAccessibleName("Позиция воспроизведения")
        self.timeline.sliderPressed.connect(self._scrub_started)
        self.timeline.sliderReleased.connect(self._scrub_finished)
        self.timeline.valueChanged.connect(self._timeline_changed)
        self.time_label = _label("00:00.000 / 00:00.000")
        self.speed_combo = QComboBox()
        for speed in (0.25, 0.5, 1.0, 2.0, 4.0, 8.0):
            self.speed_combo.addItem(f"{speed:g}×", speed)
        self.speed_combo.setCurrentIndex(2)
        self.speed_combo.currentIndexChanged.connect(self._speed_changed)
        for widget in (self.rewind_button, self.play_button, self.timeline, self.time_label, self.speed_combo):
            playback_row.addWidget(widget, 1 if widget is self.timeline else 0)
        self.playback_panel = QWidget()
        self.playback_panel.setLayout(playback_row)
        self.playback_panel.setEnabled(False)
        root.addWidget(self.playback_panel)
        body = QSplitter(Qt.Orientation.Horizontal)
        root.addWidget(body, 1)
        sidebar = QWidget()
        sidebar.setMinimumWidth(280)
        sidebar.setMaximumWidth(340)
        left = QVBoxLayout(sidebar)
        left.setContentsMargins(0, 0, 12, 0)
        connection = QGroupBox("UDP-приёмник")
        form = QFormLayout(connection)
        self.host_edit = QLineEdit(host)
        self.host_edit.setAccessibleName("Адрес прослушивания")
        self.host_edit.setToolTip("Адрес сетевого интерфейса ПК. 0.0.0.0 — все IPv4-интерфейсы.")
        self.port_spin = QSpinBox()
        self.port_spin.setRange(1, 65535)
        self.port_spin.setValue(port)
        self.port_spin.setAccessibleName("UDP-порт")
        form.addRow("Адрес ПК", self.host_edit)
        form.addRow("Порт", self.port_spin)
        self.record_check = QCheckBox("Записывать сессию в JSONL")
        self.record_check.setChecked(True)
        form.addRow(self.record_check)
        self.folder_button = QPushButton("Папка записей…")
        self.folder_button.clicked.connect(self.choose_record_directory)
        form.addRow(self.folder_button)
        buttons = QHBoxLayout()
        self.start_button = QPushButton("Начать приём")
        self.start_button.setObjectName("primary")
        self.stop_button = QPushButton("Остановить")
        self.stop_button.setEnabled(False)
        self.start_button.clicked.connect(self.start_receiving)
        self.stop_button.clicked.connect(self.stop_receiving)
        buttons.addWidget(self.start_button)
        buttons.addWidget(self.stop_button)
        form.addRow(buttons)
        connection.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        left.addWidget(connection)
        self.status_label = _label("Приём остановлен", "status")
        left.addWidget(self.status_label)
        left.addWidget(_label("Отображаемый поток", "muted"))
        self.stream_combo = QComboBox()
        self.stream_combo.addItem("Авто: последний активный", None)
        self.stream_combo.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        self.stream_combo.setMinimumContentsLength(12)
        self.stream_combo.setAccessibleName("Отображаемая сессия")
        left.addWidget(self.stream_combo)
        self.stream_label = _label("Сессия появится после первого пакета", "muted")
        left.addWidget(self.stream_label)
        self.stats_label = _label("Дейтаграммы: 0\nНекорректные: 0")
        left.addWidget(self.stats_label)
        self.record_label = _label("Запись не начата", "muted")
        left.addWidget(self.record_label)
        self.error_label = _label("", "error")
        left.addWidget(self.error_label)
        left.addStretch()
        clear_button = QPushButton("Очистить графики")
        clear_button.setToolTip("Только отображение на ПК. Не сбрасывает позицию Android и не удаляет запись.")
        clear_button.clicked.connect(self.clear_plots)
        left.addWidget(clear_button)
        log_button = QPushButton("Журнал приложения")
        log_button.setEnabled(self.preferences is not None)
        log_button.clicked.connect(self.open_log)
        left.addWidget(log_button)
        left.addWidget(_label("На телефоне укажите IPv4 компьютера и этот порт.\n"
                              "0.0.0.0 на телефоне указывать нельзя.", "muted"))
        # More diagnostics and the new entry button must not squeeze network controls
        # below readable height on smaller screens or with long recording paths.
        sidebar_scroll = QScrollArea()
        sidebar_scroll.setWidgetResizable(True)
        sidebar_scroll.setFrameShape(QScrollArea.Shape.NoFrame)
        sidebar_scroll.setMinimumWidth(300)
        sidebar_scroll.setMaximumWidth(365)
        sidebar_scroll.setWidget(sidebar)
        body.addWidget(sidebar_scroll)

        content = QWidget()
        right = QVBoxLayout(content)
        right.setContentsMargins(8, 0, 0, 0)
        cards = QHBoxLayout()
        sensors = QGroupBox("Датчики · оси Android")
        sensor_layout = QVBoxLayout(sensors)
        self.quaternion_label = _label("Quaternion (w, x, y, z): —")
        self.gyro_label = _label("Гироскоп, рад/с: —")
        self.accel_label = _label("Линейное ускорение, м/с²: —")
        for widget in (self.quaternion_label, self.gyro_label, self.accel_label):
            sensor_layout.addWidget(widget)
        cards.addWidget(sensors, 1)
        camera = QGroupBox("Камерная одометрия")
        camera_layout = QVBoxLayout(camera)
        self.pose_label = _label("X: —    Y: —")
        self.camera_label = _label("Нет данных камеры")
        self.quality_label = _label("Масштаб и качество: —", "muted")
        for widget in (self.pose_label, self.camera_label, self.quality_label):
            camera_layout.addWidget(widget)
        cards.addWidget(camera, 1)
        right.addLayout(cards)

        plots = QSplitter(Qt.Orientation.Horizontal)
        self.trajectory_plot = self._make_plot("Траектория в плоскости кадра", "Y", "м")
        self.trajectory_plot.setLabel("bottom", "X", units="м")
        self.trajectory_plot.setAspectLocked(True)
        self.trajectory_plot.invertY(True)  # Protocol: +Y points down in the reset image.
        self.trajectory_plot.setRange(xRange=(-1, 1), yRange=(-1, 1), padding=0.05)
        self.path_curve = self.trajectory_plot.plot(pen=pg.mkPen(COLORS[0], width=2), connect="finite")
        self.position_marker = self.trajectory_plot.plot(pen=None, symbol="o", symbolSize=10,
                                                        symbolBrush=COLORS[0], symbolPen=None)
        self.scene_tabs = QTabWidget()
        phone_panel = QWidget()
        phone_layout = QVBoxLayout(phone_panel)
        phone_layout.setContentsMargins(0, 0, 0, 0)
        self.phone_view = PhoneView()
        phone_layout.addWidget(self.phone_view, 1)
        reference_row = QHBoxLayout()
        self.reference_button = QPushButton("Текущий поворот → ноль")
        self.reference_button.clicked.connect(self.phone_view.use_current_as_reference)
        world_button = QPushButton("Мировые оси")
        world_button.clicked.connect(self.phone_view.reset_reference)
        reference_row.addWidget(self.reference_button)
        reference_row.addWidget(world_button)
        phone_layout.addLayout(reference_row)
        self.scene_tabs.addTab(phone_panel, "3D-ориентация")
        self.scene_tabs.addTab(self.trajectory_plot, "2D-траектория")
        plots.addWidget(self.scene_tabs)
        graph_panel = QWidget()
        graph_layout = QVBoxLayout(graph_panel)
        graph_layout.setContentsMargins(8, 0, 0, 0)
        self.sensor_plots = {}
        self.sensor_curves = {}
        for name, title, unit in (("gyroscope", "Гироскоп", "рад/с"),
                                   ("linear_acceleration", "Линейное ускорение", "м/с²")):
            plot = self._make_plot(title, "", unit)
            plot.setLabel("bottom", "Время приёма ПК", units="с")
            plot.addLegend(offset=(8, 8))
            self.sensor_plots[name] = plot
            self.sensor_curves[name] = [
                plot.plot(pen=pg.mkPen(color, width=1.5), name=axis, connect="finite")
                for axis, color in zip("XYZ", COLORS)
            ]
            graph_layout.addWidget(plot, 1)
        plots.addWidget(graph_panel)
        plots.setSizes([500, 430])
        right.addWidget(plots, 1)
        right.addWidget(_label("X вправо, Y вниз · координаты Android в метрах при заданной высоте. "
                               "Разрывы линии = потеря tracking. Z и 3D-перемещение пока не измеряются.", "muted"))
        body.addWidget(content)
        body.setSizes([300, 1030])
        self.setCentralWidget(central)

    @staticmethod
    def _make_plot(title: str, axis: str, unit: str) -> pg.PlotWidget:
        plot = pg.PlotWidget(background="#121e2d")
        plot.setTitle(title, color="#dce6f3", size="11pt")
        plot.setLabel("left", axis, units=unit)
        plot.showGrid(x=True, y=True, alpha=0.18)
        plot.setMinimumSize(270, 180)
        plot.setMenuEnabled(False)
        for position in ("left", "bottom"):
            plot.getAxis(position).setTextPen("#9aaec5")
            plot.getAxis(position).setPen("#3a4b61")
            # Keep units identical to the numeric readouts; don't render mixed-script "mм/с²".
            plot.getAxis(position).enableAutoSIPrefix(False)
        return plot

    def choose_record_directory(self) -> None:
        selected = QFileDialog.getExistingDirectory(self, "Папка записей", str(self.record_directory))
        if selected:
            self.record_directory = Path(selected)
            self.folder_button.setToolTip(selected)

    def _set_running(self, running: bool) -> None:
        for widget in (self.host_edit, self.port_spin, self.record_check, self.folder_button,
                       self.start_button):
            widget.setEnabled(not running)
        self.stop_button.setEnabled(running and not self._stopping)
        replay_mode = self.replay_engine is not None or self.replay_loader is not None
        self.start_button.setEnabled(not running and not replay_mode)
        self.open_button.setEnabled(not running and self.replay_loader is None)
        self.close_replay_button.setEnabled(replay_mode)
        self.playback_panel.setEnabled(self.replay_engine is not None and self.replay_loader is None)

    def start_receiving(self) -> None:
        if self.worker is not None or self.replay_engine is not None or self.replay_loader is not None:
            return
        host = self.host_edit.text().strip()
        try:
            ipaddress.IPv4Address(host)
        except ipaddress.AddressValueError:
            self.error_label.setText("Укажите корректный IPv4-адрес ПК; для всех интерфейсов — 0.0.0.0.")
            return
        self.history = DisplayHistory()
        self._display_dropped = 0
        self._plot_key = None
        self._stopping = False
        self._snapshot = {"streams": [], "datagrams": 0, "invalid": 0}
        self.error_label.clear()
        self.record_path = None
        if self.record_check.isChecked():
            name = datetime.now().strftime("session-%Y%m%d-%H%M%S-%f.jsonl")
            self.record_path = self.record_directory / name
        self.worker = ReceiverWorker(host, self.port_spin.value(), self.record_path)
        self._set_running(True)
        self.status_label.setText("Открытие UDP-порта…")
        self.worker.start()
        self.save_preferences()
        LOGGER.info("UDP reception started on %s:%s; recording=%s", host, self.port_spin.value(), self.record_path is not None)

    def stop_receiving(self) -> None:
        if self.worker is not None:
            self._stopping = True
            self.worker.stop()
            self.stop_button.setEnabled(False)
            self.status_label.setText("Остановка · сохранение записи…")

    def clear_plots(self) -> None:
        self.history.clear_plots()
        self._render()

    def refresh(self) -> None:
        if self.replay_loader is not None:
            if self._poll_replay_loader():
                return
        if self.replay_engine is not None and self.replay_loader is None:
            try:
                target = self.playback_clock.update(time.perf_counter())
                complete = self.replay_engine.advance(target)
                if not complete:
                    self.status_label.setText("Перемотка / подготовка данных…")
                    # Yield to Qt between short batches, then continue without a 100 ms delay.
                    self.replay_pump.start()
                    return
                self.replay_pump.stop()
                self.history = self.replay_engine.history
                self._snapshot = self.replay_engine.state.snapshot(self.replay_engine.now_ns)
                if self._seeking:
                    self._seeking = False
                    self.playback_clock.set_playing(self._resume_after_seek, time.perf_counter())
                    self._resume_after_seek = False
                self.status_label.setText("ВОСПРОИЗВЕДЕНИЕ" if self.playback_clock.playing else "ЗАПИСЬ · ПАУЗА")
                self._update_playback_controls()
            except (OSError, ValueError) as exc:
                self.close_recording()
                self.report_error(f"Не удалось воспроизвести запись: {exc}")
        if self.worker is not None:
            update = self.worker.take_update()
            if update["display_dropped"] > self._display_dropped:
                self.history.mark_gap()
            self._display_dropped = update["display_dropped"]
            for event in update["events"]:
                self.history.observe(event)
            self._snapshot = update["snapshot"]
            if update["error"]:
                self.report_error(update["error"])
            if self.record_path is not None:
                state = "Сохранено" if update["finished"] else "Запись"
                self.record_label.setText(f"{state}: {update['written']} пакетов\n{self.record_path.name}")
                self.record_label.setToolTip(str(self.record_path))
            else:
                self.record_label.setText("Запись выключена")
            if update["finished"] and not self.worker.is_alive():
                self.worker.join(timeout=0)
                self.worker = None
                self._set_running(False)
                self.status_label.setText("Ошибка приёма / записи" if update["error"] else "Приём остановлен")
                if self._closing:
                    self.close()
                    return
            elif not self._stopping and update["address"]:
                self.status_label.setText("Ожидание данных Android…")
        self._render()

    def _render(self) -> None:
        now_ns = self.replay_engine.now_ns if self.replay_engine else time.monotonic_ns()
        keys = list(self.history.streams)
        if set(keys) != set(self._stream_keys):
            selected = self.stream_combo.currentData()
            self.stream_combo.clear()
            self.stream_combo.addItem("Авто: последний активный", None)
            for key in keys:
                ip, port, session = key
                self.stream_combo.addItem(f"{ip}:{port} · {(session or 'v1')[:8]}", key)
            if selected in keys:
                self.stream_combo.setCurrentIndex(keys.index(selected) + 1)
            self._stream_keys = keys
        key = self.stream_combo.currentData() or self.history.last_key
        history = self.history.streams.get(key)
        payload = history.latest if history else None
        if key != self._orientation_key:
            self.phone_view.reset_reference()
            self._orientation_key = key
        has_orientation = history is not None and "quaternion" in history.sensor_updates
        quaternion = tuple(payload["quaternion"][axis] for axis in "wxyz") if has_orientation else None
        stale = has_orientation and (now_ns - history.sensor_updates["quaternion"] > STALE_NS
                                      or (self.worker is None and self.replay_engine is None))
        self.phone_view.set_orientation(quaternion, stale=stale)
        self.reference_button.setEnabled(has_orientation)
        stats = next((stream for stream in self._snapshot["streams"]
                      if key and stream["sender"] == f"{key[0]}:{key[1]}"
                      and stream["session_id"] == key[2]), None)
        counters = (f"Дейтаграммы: {self._snapshot['datagrams']}   ·   "
                    f"Некорректные: {self._snapshot['invalid']}")
        if stats:
            counters += (f"\n{stats['packets_per_second']:.1f} пакетов/с  ·  seq {stats['highest_sequence']}"
                         f"\nПропуски: {stats['missing_finalized']}  ·  ожидаемые: {stats['pending_gaps']}"
                         f"\nДубли: {stats['duplicates']}  ·  перестановки: {stats['reordered']}"
                         f"\nПоздние: {stats['too_late']}  ·  возраст пакета: {stats['age_seconds']:.1f} с")
            if self.worker is not None and not self._stopping:
                self.status_label.setText("Данные поступают" if stats["receiving"] else "Нет новых пакетов > 2 с")
        if self._display_dropped:
            counters += f"\nПропущено только на экране: {self._display_dropped}"
        self.stats_label.setText(counters)
        if self._snapshot.get("last_error") and not self.error_label.text():
            self.error_label.setText("Последний некорректный пакет: " + self._snapshot["last_error"])
        if key:
            prefix = "СИМУЛЯЦИЯ · " if (key[2] or "").startswith("sim-") else ""
            self.stream_label.setText(f"{prefix}{key[0]}:{key[1]}\nСессия: {key[2] or 'v1 (без ID)'}")
        else:
            self.stream_label.setText("Сессия появится после первого пакета")
        for name, label, title, axes in (
            ("quaternion", self.quaternion_label, "Quaternion (w, x, y, z)", "wxyz"),
            ("gyroscope", self.gyro_label, "Гироскоп, рад/с", "xyz"),
            ("linear_acceleration", self.accel_label, "Лин. ускорение, м/с²", "xyz"),
        ):
            text = title + ": —"
            if payload and name in history.sensor_updates:
                text = title + ": " + "  ".join(f"{payload[name][axis]:+.3f}" for axis in axes)
                if now_ns - history.sensor_updates[name] > STALE_NS:
                    text += " · нет обновлений"
            label.setText(text)
        planar = payload.get("planar_odometry") if payload else None
        self.pose_label.setText((f"X {planar['x_m']:+.3f} м   ·   Y {planar['y_m']:+.3f} м\n"
                                 f"Поворот {np.degrees(planar['heading_rad']):+.1f}°   ·   epoch {planar['epoch']}")
                                if planar else "X: —    Y: —")
        self.camera_label.setText(history.camera_state(now_ns) if history else "Нет данных камеры")
        quality = "Масштаб и качество: —"
        if planar:
            lidar = planar.get("height_source") == "IPHONE_LIDAR"
            valid_height = planar.get("height_valid", False) if lidar else True
            height = f"h = {planar['surface_distance_m']:.3f} м" if valid_height else "h: нет свежего LiDAR"
            quality = (f"Точки: {planar['inliers']}/{planar['tracked_points']}   ·   {height}\n"
                       + ("Параметры камеры приблизительные" if planar['intrinsics_approximate']
                          else "Параметры камеры из калибровки"))
            if lidar:
                quality += "\nИсточник: iPhone LiDAR · синхронизация приблизительная"
                if valid_height:
                    quality += (f"\nГлубина {planar['lidar_depth_m']:.3f} м · "
                                f"RTT {planar['lidar_rtt_ms']:.0f} мс")
        self.quality_label.setText(quality)
        points = np.asarray(history.trajectory if history else [], dtype=float)
        if points.size:
            self.path_curve.setData(points[:, 0], points[:, 1])
            self.position_marker.setData(points[-1:, 0], points[-1:, 1])
            live = ((self.worker is not None or self.replay_engine is not None) and stats and stats["receiving"]
                    and history.camera_state(now_ns) == "TRACKING")
            self.position_marker.setSymbolBrush(COLORS[0] if live else "#ffbd69")
        else:
            self.path_curve.setData([], [])
            self.position_marker.setData([], [])
        plot_key = (key, history.epoch if history else None)
        if plot_key != self._plot_key:
            self.trajectory_plot.enableAutoRange()
            self._plot_key = plot_key
        for name, curves in self.sensor_curves.items():
            rows = np.asarray(history.sensors[name] if history else [], dtype=float)
            for axis, curve in enumerate(curves, 1):
                curve.setData(rows[:, 0], rows[:, axis]) if rows.size else curve.setData([], [])
            end = max(1.0, (now_ns - history.start_ns) / 1e9) if history else 30
            if self.worker is None and self.replay_engine is None and rows.size:
                end = max(1.0, rows[-1, 0])
            self.sensor_plots[name].setXRange(max(0.0, end - 30), end, padding=0)

    def closeEvent(self, event: QCloseEvent) -> None:
        if self.mp4_window is not None:
            self.mp4_window.close()
            if self.mp4_window.worker is not None:
                QTimer.singleShot(100, self.close)
                event.ignore()
                return
        if self.odometry_window is not None:
            self.odometry_window.close()
            if self.odometry_window.worker is not None:
                QTimer.singleShot(100, self.close)
                event.ignore()
                return
        if self.worker is not None or self.replay_loader is not None:
            self._closing = True
            self.stop_receiving()
            if self.replay_loader:
                self.replay_loader.cancelled.set()
            event.ignore()  # Timer will close after receiver and writer finish; no blocking GUI wait.
        else:
            if self.replay_engine:
                self.replay_engine.close()
                self.replay_engine = None
            self.save_preferences()
            self.timer.stop()
            self.replay_pump.stop()
            event.accept()


    def report_error(self, message: str):
        if self.error_label.text() != message:
            LOGGER.error(message)
        self.error_label.setText(message)

    def open_pc_odometry(self):
        try:
            from .odometry_window import OdometryWindow
            if self.odometry_window is None or not self.odometry_window.isVisible():
                self.odometry_window = OdometryWindow(self.record_directory, self)
            self.odometry_window.show()
            self.odometry_window.raise_()
        except Exception as exc:
            self.report_error(f"Не удалось открыть расчёт на ПК: {exc}")

    def open_mp4(self):
        try:
            from .mp4_window import Mp4Window
            if self.mp4_window is None or not self.mp4_window.isVisible():
                if self.mp4_window is not None:
                    self.mp4_window.deleteLater()
                self.mp4_window = Mp4Window(self.record_directory, self)
            self.mp4_window.show()
            self.mp4_window.raise_()
        except Exception as exc:
            self.report_error(f"Не удалось открыть MP4-режим: {exc}")

    def save_preferences(self):
        if not self.preferences:
            return
        try:
            self.preferences.save({
                "host": self.host_edit.text().strip(), "port": self.port_spin.value(),
                "record": self.record_check.isChecked(), "record_directory": str(self.record_directory),
                "replay_directory": str(self.replay_directory), "speed": self.speed_combo.currentData(),
                "width": self.width(), "height": self.height(),
            })
        except (OSError, ValueError) as exc:
            self.report_error(f"Не удалось сохранить настройки: {exc}")

    def open_log(self):
        if self.preferences:
            path = self.preferences.directory / "logs/motion-viewer.log"
            if path.exists():
                QDesktopServices.openUrl(QUrl.fromLocalFile(str(path)))

    def open_recording(self, path=None):
        if self.worker is not None or self.replay_loader is not None:
            self.report_error("Сначала остановите приём UDP или дождитесь открытия файла.")
            return
        if path is None:
            filename, _ = QFileDialog.getOpenFileName(self, "Открыть телеметрию", str(self.replay_directory),
                                                     "Записи телеметрии (*.jsonl)")
            if not filename:
                return
            path = Path(filename)
        if self.playback_clock:
            self.playback_clock.set_playing(False, time.perf_counter())
        self.error_label.clear()
        self.replay_directory = Path(path).resolve().parent
        self.replay_loader = ReplayLoader(path)
        self.replay_loader.start()
        self._set_running(False)
        self.replay_label.setText("Проверка записи…")

    def _poll_replay_loader(self):
        loader = self.replay_loader
        if loader.is_alive():
            self.replay_label.setText(f"Проверка записи: {loader.progress:.0%} · {loader.path.name}")
            return False
        loader.join(timeout=0)
        self.replay_loader = None
        if self._closing:
            self.close()
            return True
        if not loader.cancelled.is_set():
            if loader.error:
                self.report_error(loader.error)
                self.replay_label.setText("Файл не открыт; предыдущие данные сохранены")
            else:
                try:
                    engine = ReplayEngine(loader.result)
                    if self.replay_engine:
                        self.replay_engine.close()
                    self.replay_engine = engine
                    self.playback_clock = PlaybackClock(engine.index.duration)
                    self.playback_clock.set_speed(self.speed_combo.currentData(), time.perf_counter())
                    self.history = engine.history
                    self._snapshot = engine.state.snapshot(engine.now_ns)
                    self._plot_key = self._orientation_key = None
                    self.phone_view.reset_reference()
                    self._display_dropped = 0
                    self._seeking = False
                    self._resume_after_seek = False
                    self.record_label.setText("Файл открыт только для чтения")
                    self.replay_label.setText(f"{engine.index.path.name} · {len(engine.index.times)} дейтаграмм")
                    self.replay_label.setToolTip(str(engine.index.path))
                    self.timeline.blockSignals(True)
                    self.timeline.setRange(0, max(1, min(2**31 - 1, round(engine.index.duration * 1000))))
                    self.timeline.setValue(0)
                    self.timeline.blockSignals(False)
                    LOGGER.info("Replay opened: %s records, %.3f seconds", len(engine.index.times), engine.index.duration)
                except (OSError, ValueError) as exc:
                    self.report_error(f"Не удалось открыть запись: {exc}")
        self._set_running(False)
        return False

    def close_recording(self):
        self.replay_pump.stop()
        if self.replay_loader:
            self.replay_loader.cancelled.set()
        if self.replay_engine:
            self.replay_engine.close()
        self.replay_engine = self.playback_clock = None
        self._seeking = self._resume_after_seek = False
        self.history = DisplayHistory()
        self._snapshot = {"streams": [], "datagrams": 0, "invalid": 0}
        self.phone_view.reset_reference()
        self.replay_label.setText("Живой режим · запись закрыта")
        self.status_label.setText("Приём остановлен")
        self.record_label.setText("Запись не начата")
        self.timeline.blockSignals(True)
        self.timeline.setValue(0)
        self.timeline.blockSignals(False)
        self.time_label.setText("00:00.000 / 00:00.000")
        self._set_running(False)
        self._render()

    def toggle_playback(self):
        if self.playback_clock and not self._seeking:
            if self.playback_clock.position >= self.playback_clock.duration:
                self.seek_replay(0.0, resume=True)
            else:
                self.playback_clock.set_playing(not self.playback_clock.playing, time.perf_counter())
            self._update_playback_controls()

    def seek_replay(self, position: float, resume=False):
        if self.playback_clock:
            self.error_label.clear()
            self.playback_clock.set_playing(False, time.perf_counter())
            self.playback_clock.seek(position, time.perf_counter())
            self._seeking = True
            self._resume_after_seek = resume

    def _scrub_started(self):
        if self.playback_clock:
            self._resume_after_seek = self.playback_clock.playing
            self.playback_clock.set_playing(False, time.perf_counter())

    def _scrub_finished(self):
        if self.playback_clock:
            self.seek_replay(self.timeline.value() / self.timeline.maximum() * self.playback_clock.duration,
                             resume=self._resume_after_seek)

    def _timeline_changed(self, value):
        if self.playback_clock and not self.timeline.isSliderDown():
            self.seek_replay(value / self.timeline.maximum() * self.playback_clock.duration,
                             resume=self.playback_clock.playing)

    def _speed_changed(self):
        if self.playback_clock:
            self.playback_clock.set_speed(self.speed_combo.currentData(), time.perf_counter())

    def _update_playback_controls(self):
        clock = self.playback_clock
        self.play_button.setText("Пауза" if clock.playing else "Воспроизвести")
        if not self.timeline.isSliderDown():
            self.timeline.blockSignals(True)
            self.timeline.setValue(round(clock.position / clock.duration * self.timeline.maximum()) if clock.duration else 0)
            self.timeline.blockSignals(False)
        def formatted(seconds):
            minutes, rest = divmod(seconds, 60)
            return f"{int(minutes):02}:{rest:06.3f}"
        self.time_label.setText(f"{formatted(clock.position)} / {formatted(clock.duration)}")


def run_gui(host: str | None = None, port: int | None = None, preferences=None, open_path=None) -> int:
    from .preferences import configure_logging, default_data_directory
    if preferences is None:
        preferences = Preferences(default_data_directory())
        configure_logging(preferences.directory / "logs")
    application = QApplication.instance() or QApplication([])
    application.setApplicationName("Motion Viewer")
    window = MotionViewer(host, port, preferences)
    previous_hook = sys.excepthook
    def exception_hook(kind, value, traceback):
        LOGGER.critical("Unhandled GUI exception", exc_info=(kind, value, traceback))
        window.error_label.setText(f"Внутренняя ошибка: {value}. Подробности в журнале приложения.")
    sys.excepthook = exception_hook
    window.show()
    if open_path:
        window.open_recording(open_path)
    try:
        return application.exec()
    finally:
        sys.excepthook = previous_hook
        LOGGER.info("Motion Viewer closed")
