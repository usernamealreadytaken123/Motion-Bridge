"""Small solid 3D phone model projected with Qt's software painter (no OpenGL dependency)."""

import numpy as np
from PySide6.QtCore import QPointF, Qt
from PySide6.QtGui import QColor, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import QWidget

from .orientation import relative_matrix


class PhoneView(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(270, 230)
        self.setAccessibleName("3D-модель ориентации телефона")
        self.quaternion = None
        self.reference = None
        self.stale = False
        self.matrix = np.eye(3)
        self._towards_camera = np.array([3.0, -4.0, 3.0])
        self._towards_camera /= np.linalg.norm(self._towards_camera)
        self._right = np.cross(-self._towards_camera, [0, 0, 1])
        self._right /= np.linalg.norm(self._right)
        self._up = np.cross(self._towards_camera, self._right)

    def set_orientation(self, quaternion, stale=False):
        self.quaternion = tuple(quaternion) if quaternion is not None else None
        self.stale = stale
        self.matrix = np.array(relative_matrix(self.quaternion, self.reference)) if quaternion else np.eye(3)
        self.update()

    def use_current_as_reference(self):
        if self.quaternion is not None:
            self.reference = self.quaternion
            self.set_orientation(self.quaternion, self.stale)

    def reset_reference(self):
        self.reference = None
        self.set_orientation(self.quaternion, self.stale)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor("#121e2d"))
        scale = min(self.width() / 3.1, (self.height() - 90) / 2.6)
        center = QPointF(self.width() / 2, self.height() / 2 + 2)

        def project(point, origin=center, size=scale):
            return QPointF(origin.x() + size * float(np.dot(point, self._right)),
                           origin.y() - size * float(np.dot(point, self._up)))

        painter.setPen(QPen(QColor("#243b50"), 1))
        for offset in np.linspace(-1.1, 1.1, 7):
            painter.drawLine(project([offset, -1.1, -0.95]), project([offset, 1.1, -0.95]))
            painter.drawLine(project([-1.1, offset, -0.95]), project([1.1, offset, -0.95]))

        vertices = np.array([[-.38, -.76, -.055], [.38, -.76, -.055], [.38, .76, -.055],
                             [-.38, .76, -.055], [-.38, -.76, .055], [.38, -.76, .055],
                             [.38, .76, .055], [-.38, .76, .055]])
        faces = [([0, 1, 2, 3], "#354e6a"), ([4, 5, 6, 7], "#24394f"),
                 ([0, 1, 5, 4], "#6a839b"), ([1, 2, 6, 5], "#587187"),
                 ([2, 3, 7, 6], "#7991a7"), ([3, 0, 4, 7], "#425d77")]
        polygons = [(vertices[indices], color) for indices, color in faces]
        # Screen, top speaker, and navigation strip identify front and top at any rotation.
        polygons.extend([
            (np.array([[-.32, -.62, .057], [.32, -.62, .057], [.32, .62, .057], [-.32, .62, .057]]),
             "#226d77" if not self.stale else "#5b6675"),
            (np.array([[-.10, .67, .058], [.10, .67, .058], [.10, .70, .058], [-.10, .70, .058]]), "#a0b6ca"),
            (np.array([[-.11, -.70, .058], [.11, -.70, .058], [.11, -.68, .058], [-.11, -.68, .058]]), "#c6d4e3"),
        ])
        transformed = [(points @ self.matrix.T, color) for points, color in polygons]
        transformed.sort(key=lambda item: float(np.mean(item[0], axis=0) @ self._towards_camera))
        painter.setPen(QPen(QColor("#8ca5bc"), 0.8))
        for points, color in transformed:
            painter.setBrush(QColor(color))
            painter.drawPolygon(QPolygonF([project(point) for point in points]))

        for axis, color, vector in zip("XYZ", ("#ff8a86", "#56d6bf", "#8faaff"), np.eye(3)):
            end = project(self.matrix @ (vector * 1.03))
            painter.setPen(QPen(QColor(color), 2))
            painter.drawLine(project([0, 0, 0]), end)
            painter.drawText(end + QPointF(4, -4), axis)

        origin = QPointF(47, self.height() - 49)
        for label, vector, color in zip(("E", "N", "U") if self.reference is None else ("X₀", "Y₀", "Z₀"),
                                        np.eye(3), ("#ff8a86", "#56d6bf", "#8faaff")):
            end = project(vector, origin, 25)
            painter.setPen(QPen(QColor(color), 1.5))
            painter.drawLine(origin, end)
            painter.drawText(end + QPointF(2, 0), label)
        painter.setPen(QColor("#9aaec5"))
        basis = "Мировые оси: East / North / Up" if self.reference is None else "Относительно выбранного положения"
        painter.drawText(12, 22, basis)
        message = "Нет данных ориентации" if self.quaternion is None else (
            "Последнее положение · нет обновлений" if self.stale else "Только вращение · центр неподвижен")
        painter.drawText(self.rect().adjusted(85, 0, -10, -12),
                         Qt.AlignmentFlag.AlignBottom | Qt.AlignmentFlag.AlignRight, message)
        painter.end()
