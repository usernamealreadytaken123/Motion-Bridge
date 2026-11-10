"""Adjacent-frame 2D odometry, matching Android's original LEGACY model.

No spatial pose, quaternion or plane-normal recovery. Image similarity is fitted
in pixels before applying a CONSTANT h/f. Rejected pairs are not integrated and
never pin the next calculation to an old image. Missing motion is not recovered.
"""
from collections import Counter
import math

import cv2
import numpy as np

from .mp4_odometry import (FOCAL_FACTOR, GEOMETRY_ERRORS, ImageReference,
                           tracked_homography, descriptor_homography)

MAX_SCALE_DEVIATION = .06
MAX_FRAME_SECONDS = .35
MAX_ROTATION_RAD = .45
MAX_FLOW_FRACTION = .20
DEADBAND_PX = .65
MIN_HEIGHT_M = .05
MAX_HEIGHT_M = 100.


def similarity_step(a, b, shape):
    """Robust shift/rotation/scale; all acceptance thresholds are image-domain.

    Return flow AT THE IMAGE CENTRE, not affine's top-left translation: a
    centred rotation/zoom must not invent camera translation.
    """
    matrix, mask = cv2.estimateAffinePartial2D(
        a, b, method=cv2.RANSAC, ransacReprojThreshold=2.5,
        maxIters=2000, confidence=.995, refineIters=10)
    if matrix is None or mask is None or not np.isfinite(matrix).all():
        raise ValueError("Не найдено 2D-преобразование")
    selected = mask.ravel() != 0
    old, new = a[selected], b[selected]
    if len(old) < 25 or len(old)/len(a) < .6:
        raise ValueError("Недостаточно точек с общим 2D-движением")
    if np.any(np.ptp(old, axis=0) < np.array(shape[::-1])*.15):
        raise ValueError("Точки сосредоточены на слишком малом участке")
    projected = old @ matrix[:, :2].T + matrix[:, 2]
    residual = float(np.sqrt(np.mean(np.sum((projected-new)**2, axis=1))))
    if not math.isfinite(residual) or residual > 2.:
        raise ValueError("Большая ошибка 2D-модели; возможен наклон / неплоская сцена")
    centre = (np.array(shape[::-1])-1)/2
    flow = matrix[:, :2] @ centre + matrix[:, 2] - centre
    scale = float(np.linalg.norm(matrix[:, 0]))
    turn = -math.atan2(matrix[1, 0], matrix[0, 0])
    return flow, turn, scale, old, new, len(old)/len(a), residual


class Video2DEngine:
    def __init__(self, height, focal_factor=FOCAL_FACTOR):
        if not math.isfinite(height) or not MIN_HEIGHT_M <= height <= MAX_HEIGHT_M:
            raise ValueError(f"Высота должна быть от {MIN_HEIGHT_M:g} до {MAX_HEIGHT_M:g} м")
        if not math.isfinite(focal_factor) or not .1 <= focal_factor <= 5:
            raise ValueError("Коэффициент f должен быть от 0.1 до 5")
        self.height, self.focal_factor = float(height), float(focal_factor)
        self.previous = None
        self.shape = None
        self.focal_px = None
        self.position = np.zeros(2)
        self.heading = 0.
        self.heading_continuous = True
        self.distance = self.segment_distance = 0.
        self.segment = self.rejected_frames = self.recoveries = 0
        self.origin_time_s = None
        self.rejection_reasons = Counter()
        self.orb = cv2.ORB_create(nfeatures=1800)

    def match(self, reference, gray):
        try:
            return (*tracked_homography(reference.gray, gray, reference.points), "LK")
        except GEOMETRY_ERRORS as exc:
            if reference.descriptors is None:
                reference.descriptors = self.orb.detectAndCompute(reference.gray, None)
            try:
                found = descriptor_homography(reference.descriptors, self.orb.detectAndCompute(gray, None), gray.shape)
                return (*found, "ORB")
            except GEOMETRY_ERRORS as other:
                raise ValueError(f"LK: {exc}; ORB: {other}") from other

    def process(self, gray, time_s, seq):
        # Invalid geometry/clock is an input error, not a tracking gap. Do not let
        # it change the fixed focal length or overwrite a valid reference.
        if (gray.ndim != 2 or min(gray.shape) < 32 or not math.isfinite(time_s)
                or (self.previous is not None and time_s <= self.previous.time_s)
                or (self.shape is not None and gray.shape != self.shape)):
            raise ValueError("Неверное время / изменение размера кадра")
        self.shape = gray.shape
        self.focal_px = self.focal_factor*max(self.shape)
        old = self.previous
        points = cv2.goodFeaturesToTrack(gray, 600, .015, 8)
        # Advance even on an unusable frame. No hidden metric anchor or H chain.
        self.previous = ImageReference(gray.copy(), points, float(time_s))
        result = dict(seq=seq, time_s=float(time_s), segment=self.segment, status="DEGRADED", message="",
                      x_m=float(self.position[0]), y_m=float(self.position[1]), delta_x_m=0., delta_y_m=0.,
                      height_m=self.height, focal_px=self.focal_px, focal_factor=self.focal_factor,
                      tilt_deg=None, heading_rad=self.heading, heading_continuous=self.heading_continuous,
                      image_scale=None, rotation_deg=None, centre_flow_px=None,
                      inliers=0, inlier_ratio=0., residual_px=None, similarity_residual_px=None,
                      distance_m=self.distance, segment_distance_m=self.segment_distance,
                      approximate=True, calibration="assumed_constant_focal", algorithm="similarity_2d",
                      tracks=[], coordinate_frame="initial_image_2d", recovered=False, tracking_method=None,
                      pose_method=None, rejected_frames=self.rejected_frames,
                      reference_time_s=old.time_s if old else None,
                      step_time_s=float(time_s-old.time_s) if old else None,
                      visual_reference_time_s=old.time_s if old else None, visual_status="LOST",
                      origin_time_s=self.origin_time_s)
        try:
            if points is None or len(points) < 60:
                raise ValueError("Недостаточно текстуры")
            if self.origin_time_s is None:
                self.origin_time_s = float(time_s)
                self.rejected_frames = 0
                self.heading_continuous = True
                result.update(origin_time_s=self.origin_time_s, rejected_frames=0, heading_continuous=True)
            if old is None or old.points is None or len(old.points) < 60:
                result.update(status="INITIALIZING", visual_status="INITIALIZING",
                              message="Опорный кадр; расчёт начнётся со следующей пары")
                return result
            if time_s-old.time_s > MAX_FRAME_SECONDS:
                raise ValueError("Слишком большой интервал между соседними кадрами")
            _, a, b, ratio, residual, method = self.match(old, gray)
            result.update(visual_status="TRACKING", tracking_method=method, inliers=len(a), inlier_ratio=ratio,
                          residual_px=residual, tracks=np.column_stack((a[:100], b[:100])).tolist())
            flow, turn, scale, a, b, ratio, residual = similarity_step(a, b, gray.shape)
            result.update(image_scale=scale, rotation_deg=math.degrees(turn), centre_flow_px=flow.tolist(),
                          inliers=len(a), inlier_ratio=ratio, similarity_residual_px=residual,
                          tracks=np.column_stack((a[:100], b[:100])).tolist())
            if abs(scale-1) > MAX_SCALE_DEVIATION:
                raise ValueError("Изменение масштаба больше 6%; шаг пропущен")
            if abs(turn) > MAX_ROTATION_RAD or np.linalg.norm(flow) > MAX_FLOW_FRACTION*max(gray.shape):
                raise ValueError("Слишком большой сдвиг / поворот между кадрами")
            local = -self.height/self.focal_px*flow if np.linalg.norm(flow) >= DEADBAND_PX else np.zeros(2)
            middle = self.heading + turn/2
            c, s = math.cos(middle), math.sin(middle)
            delta = np.array([[c, -s], [s, c]]) @ local
            recovered = self.rejected_frames > 0
            if recovered:
                self.segment += 1
                self.recoveries += 1
                self.segment_distance = 0.
            self.position += delta
            self.heading = math.atan2(math.sin(self.heading+turn), math.cos(self.heading+turn))
            step = float(np.linalg.norm(delta))
            self.distance += step
            self.segment_distance += step
            result.update(status="TRACKING", pose_method="SIMILARITY_2D", recovered=recovered, segment=self.segment,
                          message="2D-расчёт возобновлён; движение и поворот в разрыве неизвестны" if recovered else "2D-смещение по соседним кадрам",
                          x_m=float(self.position[0]), y_m=float(self.position[1]), delta_x_m=float(delta[0]), delta_y_m=float(delta[1]),
                          heading_rad=self.heading, distance_m=self.distance, segment_distance_m=self.segment_distance)
            self.rejected_frames = 0
        except GEOMETRY_ERRORS as exc:
            self.rejected_frames += 1
            self.rejection_reasons[str(exc)] += 1
            if self.origin_time_s is not None:
                # Continuing after a gap assumes the last known heading. This is
                # explicitly approximate; a hidden turn cannot be reconstructed.
                self.heading_continuous = False
            result.update(rejected_frames=self.rejected_frames, heading_continuous=self.heading_continuous,
                          message=f"Позиция удерживается; следующая пара будет рассчитана независимо: {exc}")
        return result
