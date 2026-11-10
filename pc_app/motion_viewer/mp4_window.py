"""MP4-only workspace. The live IMU-driven engine is intentionally not modified."""
from collections import deque
from pathlib import Path
import cv2
import numpy as np
from PySide6.QtCore import QTimer, Qt, QUrl
from PySide6.QtGui import QImage, QPixmap, QDesktopServices
from PySide6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QDoubleSpinBox, QFileDialog, QSplitter, QProgressBar)
import pyqtgraph as pg
from .mp4_worker import Mp4Worker
from .mp4_2d import MIN_HEIGHT_M, MAX_HEIGHT_M


class Mp4Window(QMainWindow):
    def __init__(self, record_directory, parent=None):
        super().__init__(parent)
        self.setWindowTitle("MP4 → 2D-траектория · без компенсации наклона")
        self.resize(1200, 820)
        self.record_directory = Path(record_directory)
        self.file_path = None
        self.worker = None
        self.closing = False
        self.recording = None
        self.last_seq = None
        self.last_segment = None
        self.display_dropped = 0
        self.path = deque(maxlen=100000)
        central = QWidget()
        layout = QVBoxLayout(central)
        controls = QHBoxLayout()
        self.open_button = QPushButton("Открыть MP4…")
        self.open_button.clicked.connect(self.open_video)
        self.height = QDoubleSpinBox()
        self.height.setRange(MIN_HEIGHT_M, MAX_HEIGHT_M)
        self.height.setDecimals(3)
        self.height.setValue(1.)
        self.height.setSuffix(" м")
        self.focal = QDoubleSpinBox()
        self.focal.setRange(.1, 5.)
        self.focal.setDecimals(3)
        self.focal.setSingleStep(.05)
        self.focal.setValue(.9)
        self.focal.setToolTip("Постоянное f в пикселях = коэффициент × длинная сторона обработанного кадра. "
                             "Меняет масштаб метров, не поиск точек и не отбор шагов. Это не калибровка объектива.")
        self.start_button = QPushButton("Рассчитать траекторию")
        self.start_button.setEnabled(False)
        self.start_button.clicked.connect(self.start)
        self.pause_button = QPushButton("Пауза")
        self.pause_button.setEnabled(False)
        self.pause_button.clicked.connect(self.pause)
        self.stop_button = QPushButton("Отменить")
        self.stop_button.setEnabled(False)
        self.stop_button.clicked.connect(self.stop)
        self.folder_button = QPushButton("Папка результата")
        self.folder_button.setEnabled(False)
        self.folder_button.clicked.connect(self.open_folder)
        for widget in (self.open_button, self.start_button,
                       self.pause_button, self.stop_button, self.folder_button):
            controls.addWidget(widget)
        layout.addLayout(controls)
        settings = QHBoxLayout()
        for widget in (QLabel("Постоянная высота h"), self.height, QLabel("f / длинная сторона кадра"), self.focal):
            settings.addWidget(widget)
        settings.addStretch()
        layout.addLayout(settings)
        self.file_label = self.label("Выберите локальную видеозапись MP4. Телефон подключать не нужно.")
        layout.addWidget(self.file_label)
        warning = self.label("2D · Соседние кадры → сдвиг / поворот изображения / масштаб → XY при постоянном h/f. "
            "Пространственный наклон не компенсируется. Держите камеру примерно перпендикулярно неподвижной плоскости. "
            "Колебания высоты, наклоны, зум и стабилизация могут искажать форму пути. После пропуска расчёт продолжается, но движение в разрыве неизвестно.")
        warning.setStyleSheet("color: #ffbd69; padding: 8px; background: #26313e;")
        layout.addWidget(warning)
        self.status = self.label("Файл не выбран")
        layout.addWidget(self.status)
        self.progress = QProgressBar()
        self.progress.setRange(0, 1000)
        self.progress.setValue(0)
        layout.addWidget(self.progress)
        self.metrics = self.label("X: —   Y: —")
        layout.addWidget(self.metrics)
        self.summary = self.label("")
        self.summary.setStyleSheet("color: #d5fff5; padding: 8px; background: #173b39; border: 1px solid #56d6bf;")
        self.summary.hide()
        layout.addWidget(self.summary)
        splitter = QSplitter()
        self.preview = QLabel("Кадры и отслеживаемые точки появятся во время расчёта")
        self.preview.setWordWrap(True)
        self.preview.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.preview.setMinimumSize(320, 240)
        self.plot = pg.PlotWidget(background="#121e2d", title="Оценочная XY-траектория · единое начало координат")
        self.plot.setLabel("bottom", "X", units="m")
        self.plot.setLabel("left", "Y", units="m")
        for axis in ("bottom", "left"):
            self.plot.getAxis(axis).enableAutoSIPrefix(False)
        self.plot.setAspectLocked(True)
        self.plot.invertY(True)
        self.plot.showGrid(x=True, y=True, alpha=.2)
        self.curve = self.plot.plot(pen=pg.mkPen("#56d6bf", width=2), connect="finite")
        self.marker = self.plot.plot(pen=None, symbol="o", symbolSize=8, symbolBrush="#ffbd69")
        splitter.addWidget(self.preview)
        splitter.addWidget(self.plot)
        splitter.setSizes([550, 600])
        layout.addWidget(splitter, 1)
        self.details = self.label("Результаты сохраняются отдельно; исходное видео не изменяется.")
        layout.addWidget(self.details)
        self.setCentralWidget(central)
        self.timer = QTimer(self)
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.refresh)
        self.timer.start()

    @staticmethod
    def label(text):
        label = QLabel(text)
        label.setWordWrap(True)
        label.setTextFormat(Qt.TextFormat.PlainText)
        label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        return label

    def open_video(self, path=None):
        if self.worker:
            return
        if not isinstance(path, (str, Path)):
            path, _ = QFileDialog.getOpenFileName(self, "Загрузить видеозапись", str(self.file_path.parent if self.file_path else Path.home()), "Видео MP4 (*.mp4 *.MP4)")
        if path:
            self.file_path = Path(path)
            self.path.clear()
            self.curve.setData([], [])
            self.marker.setData([], [])
            self.preview.clear()
            self.metrics.setText("X: —   Y: —")
            self.summary.clear()
            self.summary.hide()
            self.details.setText("Исходное видео не изменяется. Результаты появятся после запуска расчёта.")
            self.recording = None
            self.folder_button.setEnabled(False)
            self.progress.setRange(0, 1000)
            self.progress.setValue(0)
            self.file_label.setText(str(self.file_path))
            self.start_button.setEnabled(True)
            self.status.setText("Файл выбран. Задайте постоянную высоту и нажмите «Рассчитать траекторию».")

    def set_running(self, running):
        self.open_button.setEnabled(not running)
        self.height.setEnabled(not running)
        self.focal.setEnabled(not running)
        self.start_button.setEnabled(not running and self.file_path is not None)
        self.pause_button.setEnabled(running)
        self.stop_button.setEnabled(running)
        if not running:
            self.pause_button.setText("Пауза")

    def start(self):
        if self.worker or self.file_path is None:
            return
        self.worker = Mp4Worker(self.file_path, self.height.value(), self.record_directory, self.focal.value())
        self.path.clear()
        self.curve.setData([], [])
        self.marker.setData([], [])
        self.plot.enableAutoRange()
        self.last_seq = self.last_segment = None
        self.display_dropped = 0
        self.recording = None
        self.folder_button.setEnabled(False)
        self.preview.clear()
        self.metrics.setText("X: —   Y: —")
        self.summary.clear()
        self.summary.hide()
        self.details.setText("Открытие видео…")
        self.status.setText("Подготовка…")
        self.progress.setRange(0, 0)
        self.set_running(True)
        self.worker.start()

    def pause(self):
        if self.worker:
            if self.worker.paused.is_set():
                self.worker.paused.clear()
                self.pause_button.setText("Пауза")
            else:
                self.worker.paused.set()
                self.pause_button.setText("Продолжить")

    def stop(self):
        if self.worker:
            self.worker.stop()
            self.pause_button.setEnabled(False)
            self.stop_button.setEnabled(False)

    def open_folder(self):
        if self.recording:
            QDesktopServices.openUrl(QUrl.fromLocalFile(str(Path(self.recording).resolve())))

    def show_final_summary(self, summary):
        title = {"complete": "ИТОГ MP4", "cancelled": "ПРОМЕЖУТОЧНЫЙ ИТОГ · расчёт отменён",
                 "error": "ПРОМЕЖУТОЧНЫЙ ИТОГ · обработка завершилась с ошибкой"}[summary["state"]]
        if not summary["point_available"]:
            text = f"{title}\nТраектория не определена: координаты последней точки и расстояние недоступны."
        else:
            point = "Последняя точка видео (оценка)" if summary["at_video_end"] else "Последняя рассчитанная точка"
            text = (f"{title} · {point}: X = {summary['x_m']:+.3f} м; Y = {summary['y_m']:+.3f} м "
                    f"(t = {summary['point_time_s']:.2f} с)\n"
                    f"Расстояние от (0, 0) по прямой: {summary['distance_from_origin_m']:.3f} м")
            if not summary["at_video_end"]:
                text += "\nФинальная точка видео не определена; показана последняя доступная оценка."
            if summary["has_gaps"]:
                text += "\nВ траектории есть пропуски: итоговые координаты и расстояние условны."
            if summary["origin_time_s"]:
                text += f"\nНачало отсчёта с {summary['origin_time_s']:.2f} с; движение до этой отметки неизвестно."
        self.summary.setText(text)
        self.summary.show()

    def refresh(self):
        if not self.worker:
            return
        update = self.worker.snapshot()
        self.status.setText(update["error"] or ("Пауза расчёта" if self.worker.paused.is_set() and not update["finished"] else update["status"]))
        self.recording = update["recording"]
        self.folder_button.setEnabled(self.recording is not None)
        if update["progress"] is not None:
            self.progress.setRange(0, 1000)
            self.progress.setValue(round(update["progress"]*1000))
        if update["display_dropped"] > self.display_dropped:
            self.path.append((np.nan, np.nan))
            self.display_dropped = update["display_dropped"]
        for item in update["results"]:
            if self.last_segment is not None and item["segment"] != self.last_segment:
                self.path.append((np.nan, np.nan))
            self.last_segment = item["segment"]
            self.path.append((item["x_m"], item["y_m"]) if item["status"] == "TRACKING" else (np.nan, np.nan))
        result = update["result"]
        if result and self.last_seq != result["seq"]:
            self.last_seq = result["seq"]
            if self.path:
                points = np.array(self.path)
                self.curve.setData(points[:, 0], points[:, 1])
            lost = result["status"] == "DEGRADED"
            self.marker.setData([result["x_m"]], [result["y_m"]], symbol="x" if lost else "o",
                                symbolBrush="#ff647c" if lost else "#ffbd69")
            scale = f"{result['image_scale']:.4f}" if result["image_scale"] is not None else "—"
            origin = result["origin_time_s"]
            origin_text = f"Начало координат: {origin:.2f} с" if origin is not None else "Начало координат не определено"
            if origin:
                origin_text += " · движение до этой отметки неизвестно"
            visual = {"TRACKING": "отслеживаются", "LOST": "потеряны", "INITIALIZING": "инициализация",
                      "RESTARTED": "повторная инициализация"}[result["visual_status"]]
            continuity = "" if result["heading_continuous"] else " · после разрыва положение и направление условны"
            self.metrics.setText(f"ОЦЕНКА 2D · участок {result['segment']+1} · t {result['time_s']:.2f} с · "
                f"X {result['x_m']:+.3f} м   Y {result['y_m']:+.3f} м · наклон не компенсируется\n"
                f"{origin_text} · точки: {visual} · масштаб пары {scale} · f = {result['focal_px']:.1f} px{continuity}\n"
                f"Сумма принятых смещений: {result['distance_m']:.3f} м (не полный путь при разрывах) · "
                f"точки {result['inliers']} · {result['tracking_method'] or '—'} · {result['status']}: {result['message']}")
            shown = cv2.cvtColor(update["image"], cv2.COLOR_GRAY2RGB)
            for x0, y0, x1, y1 in result["tracks"]:
                cv2.line(shown, (round(x0), round(y0)), (round(x1), round(y1)), (86, 214, 191), 1)
                cv2.circle(shown, (round(x1), round(y1)), 2, (255, 189, 105), -1)
            image = QImage(shown.data, shown.shape[1], shown.shape[0], shown.strides[0], QImage.Format.Format_RGB888).copy()
            self.preview.setPixmap(QPixmap.fromImage(image).scaled(self.preview.size(), Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation))
        self.details.setText(f"Декодировано {update['decoded']} · обработано {update['frames']} · точки {update['visual_tracking']} · 2D-шаги {update['tracking']} · "
                             f"не рассчитано {update['degraded']} · пропуски отображения {update['display_dropped']}\n"
                             f"Результаты: {self.recording or 'ещё не созданы'}")
        if update["finished"]:
            self.show_final_summary(update["final_summary"])
            self.worker = None
            self.set_running(False)
            if self.progress.maximum() == 0:
                self.progress.setRange(0, 1000)
                self.progress.setValue(0)
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
