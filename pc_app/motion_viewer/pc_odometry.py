"""PC visual odometry. Inputs are pixels + sensor timestamps, never Android X/Y."""
from collections import deque
import math
import cv2
import numpy as np
from .odometry_math import Orientations, LidarHeights, camera_basis, ground_motion
from .video_decoder import H264Decoder


def upright(header, payload):
    w, h = header["width"], header["height"]
    gray = np.frombuffer(payload, np.uint8).reshape(h, w)
    fx, fy, cx, cy = header["intrinsics"]
    angle = header["rotation"]
    if angle == 90:
        gray, k = cv2.rotate(gray, cv2.ROTATE_90_CLOCKWISE), (fy, fx, h-1-cy, cx)
    elif angle == 180:
        gray, k = cv2.rotate(gray, cv2.ROTATE_180), (fx, fy, w-1-cx, h-1-cy)
    elif angle == 270:
        gray, k = cv2.rotate(gray, cv2.ROTATE_90_COUNTERCLOCKWISE), (fy, fx, cy, w-1-cx)
    else:
        k = (fx, fy, cx, cy)
    return np.ascontiguousarray(gray), k


class PlanarEngine:
    def __init__(self, height=1.0):
        if not math.isfinite(height) or not .05 <= height <= 20:
            raise ValueError("Height must be 0.05–20 m")
        self.height = height
        self.position = np.zeros(2)
        self.axes = None
        self.reference_rotation = 0
        self.previous = None
        self.geometry = None
        self.lidar_generation = None

    def process(self, header, payload, q, height=None, generation=None):
        gray, k = upright(header, payload)
        stamp = header["timestamp_ns"]
        h = self.height if height is None else height
        result = dict(timestamp_ns=stamp, seq=header["seq"], status="DEGRADED", message="",
                      x_m=float(self.position[0]), y_m=float(self.position[1]), delta_x_m=0., delta_y_m=0.,
                      height_m=h if math.isfinite(h) else None, tilt_deg=None, tracked_points=0, inliers=0, residual_m=None,
                      approximate=header["approximate"], heading_rad=0., tracks=[], height_valid=False)
        if not header["realtime"] or not header["back_camera"] or header["sensor_orientation"] is None:
            self.previous = None
            result["message"] = "Нужна задняя камера с общей шкалой времени IMU (REALTIME)"
            return result, gray
        if q is None or not math.isfinite(h):
            self.previous = None
            result["message"] = "Нет синхронной ориентации / свежей высоты LiDAR"
            result["height_m"] = None
            return result, gray
        basis = camera_basis(q, header["sensor_orientation"], header["rotation"])
        tilt = math.degrees(math.acos(np.clip(-basis[2, 2], -1, 1)))
        result["tilt_deg"] = tilt
        if tilt > 55:
            self.previous = None
            result["message"] = "Наклон больше 55°; направьте камеру к поверхности"
            return result, gray
        result["height_valid"] = True
        geometry = (gray.shape, k, header["rotation"], header["sensor_orientation"])
        if geometry != self.geometry or self.lidar_generation != generation:
            self.previous = None
        self.geometry, self.lidar_generation = geometry, generation
        if self.axes is None:
            right = basis[:2, 0]
            self.axes = right / np.linalg.norm(right)
            self.reference_rotation = header["rotation"]
        rx, ry = self.axes
        world_to_local = np.array([[rx, ry], [ry, -rx]])
        heading_right = camera_basis(q, header["sensor_orientation"], self.reference_rotation)[:2, 0]
        local_right = world_to_local @ heading_right
        result["heading_rad"] = math.atan2(local_right[1], local_right[0])
        points = cv2.goodFeaturesToTrack(gray, 500, .015, 8)
        old = self.previous
        self.previous = (gray.copy(), points, stamp, q.copy(), basis, h)
        if old is None or old[1] is None or len(old[1]) < 60:
            result["status"] = "INITIALIZING" if points is not None and len(points) >= 60 else "DEGRADED"
            result["message"] = "Опорный кадр создан" if result["status"] == "INITIALIZING" else "Недостаточно текстуры"
            return result, gray
        old_gray, old_points, old_time, old_q, old_basis, old_height = old
        if not 0 < stamp-old_time <= 350_000_000 or 2*math.acos(np.clip(abs(float(old_q @ q)), 0, 1)) > .45:
            result["message"] = "Пропуск кадров или слишком быстрый поворот"
            return result, gray
        args = dict(winSize=(21, 21), maxLevel=3,
                    criteria=(cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 30, .01), minEigThreshold=.0001)
        following, status, errors = cv2.calcOpticalFlowPyrLK(old_gray, gray, old_points, None, **args)
        if following is None:
            result["message"] = "Не найден оптический поток"
            return result, gray
        backward, back_status, _ = cv2.calcOpticalFlowPyrLK(gray, old_gray, following, None, **args)
        if backward is None:
            result["message"] = "Не пройдена обратная проверка"
            return result, gray
        a, b = old_points.reshape(-1, 2), following.reshape(-1, 2)
        valid = ((status.ravel() != 0) & (back_status.ravel() != 0) & (errors.ravel() <= 30) &
                 (np.linalg.norm(a-backward.reshape(-1, 2), axis=1) <= 1.25) &
                 np.isfinite(b).all(axis=1) & (b[:, 0] >= 0) & (b[:, 0] < gray.shape[1]) &
                 (b[:, 1] >= 0) & (b[:, 1] < gray.shape[0]))
        a, b = a[valid], b[valid]
        result["tracked_points"] = len(a)
        if len(a) < 35:
            result["message"] = "Недостаточно устойчивых соответствий"
            return result, gray
        _, mask = cv2.findHomography(a, b, cv2.RANSAC, 2.5)
        if mask is None or mask.sum() < 25:
            result["message"] = "Плоскость поверхности не найдена"
            return result, gray
        indices = np.flatnonzero(mask.ravel())
        estimate = ground_motion(a[indices], b[indices], k, old_basis, basis, old_height, h)
        if estimate is None:
            result["message"] = "Точки не согласуются с высотой и ориентацией"
            return result, gray
        delta, selected, residual, sensitivity = estimate
        indices = indices[selected]
        spread = np.ptp(a[indices], axis=0)
        result["inliers"] = len(indices)
        result["residual_m"] = residual
        if len(indices) < 25 or len(indices)/len(a) < .45 or np.any(spread < np.array(gray.shape[::-1])*.15) or np.linalg.norm(delta) > .3:
            result["message"] = "Недостаточная согласованность / слишком большое смещение"
            return result, gray
        delta = world_to_local @ delta if np.linalg.norm(delta) >= .65*sensitivity else np.zeros(2)
        self.position += delta
        result.update(status="TRACKING", message="Расчёт на ПК: наклон учтён",
                      x_m=float(self.position[0]), y_m=float(self.position[1]),
                      delta_x_m=float(delta[0]), delta_y_m=float(delta[1]),
                      tracks=np.column_stack((a[indices[:100]], b[indices[:100]])).tolist())
        return result, gray


class CaptureProcessor:
    """Identical processing for network and a saved raw session; source-clock scheduling."""
    def __init__(self, height=1., use_lidar=False):
        self.engine = PlanarEngine(height)
        self.orientations, self.lidar = Orientations(), LidarHeights()
        self.use_lidar = use_lidar
        self.pending = deque()
        self.watermark = 0
        self.last_frame_time = 0
        self.last_frame_seq = -1
        self.dropped = 0
        self.samples = 0
        self.latest_sensors = {}
        self.last_result = None
        self.video = H264Decoder()

    def ingest(self, header, payload):
        kind = header["type"]
        if kind == "video_config":
            self.video.configure(header, payload)
            self.dropped += len(self.pending)
            self.pending.clear()
            self.engine.previous = None
            return []
        if kind == "video_packet":
            output = []
            for frame_header, pixels in self.video.decode(header, payload):
                output.extend(self.ingest(frame_header, pixels))
            return output
        if kind == "sensors":
            if header["watermark_ns"] < self.watermark:
                raise ValueError("Android clock went backwards; reconnect")
            self.watermark = header["watermark_ns"]
            for sample in header["samples"]:
                name, stamp, *values = sample
                self.samples += 1
                self.latest_sensors[name] = (stamp, values)
                if name == "q":
                    self.orientations.add(stamp, values)
        elif kind == "lidar":
            self.lidar.add(header)
        elif kind == "frame":
            if header["timestamp_ns"] <= self.last_frame_time or header["seq"] <= self.last_frame_seq:
                raise ValueError("Non-increasing camera time/sequence; reconnect")
            self.dropped += header["seq"]-self.last_frame_seq-1
            self.last_frame_time, self.last_frame_seq = header["timestamp_ns"], header["seq"]
            self.pending.append((header, payload))
            while len(self.pending) > 8:
                self.pending.popleft()
                self.dropped += 1
                self.engine.previous = None
        return self.drain()

    def drain(self, final=False):
        output = []
        delay = 250_000_000 if self.use_lidar else 120_000_000
        while self.pending and (final or self.watermark-self.pending[0][0]["timestamp_ns"] >= delay):
            header, payload = self.pending.popleft()
            stamp = header["timestamp_ns"]
            q = self.orientations.at(stamp)
            height, generation = None, None
            if self.use_lidar:
                sample = self.lidar.at(stamp, self.watermark, self.orientations)
                height, generation = sample if sample else (float("nan"), None)
            result, gray = self.engine.process(header, payload, q, height, generation)
            result.update(source="PC", height_source="IPHONE_LIDAR" if self.use_lidar else "MANUAL",
                          dropped_frames=self.dropped, sensor_samples=self.samples,
                          quaternion=q.tolist() if q is not None else None)
            self.last_result = result
            output.append((result, gray))
        return output
