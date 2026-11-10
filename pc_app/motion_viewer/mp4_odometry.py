"""Experimental monocular planar odometry. No IMU and no metric calibration.

K is an explicit ASSUMPTION (f = 0.9 * max dimension), not recovered metadata.
Constant perpendicular height and a single static plane constrain decomposition.
Visual references advance independently of accepted metric poses. Short validated
homography chains retain uncommitted motion; broken chains never imply a new pose.
"""
from collections import Counter
from dataclasses import dataclass
import math
import cv2
import numpy as np

FOCAL_FACTOR = .9
MAX_CHAIN_FRAMES = 15
MAX_CHAIN_SECONDS = 1.5
MAX_CHAIN_ERROR_PX = 3.
VISUAL_RESTART_SECONDS = .5
GEOMETRY_ERRORS = (ValueError, cv2.error, np.linalg.LinAlgError)


def assumed_intrinsics(shape):
    h, w = shape
    f = FOCAL_FACTOR * max(w, h)
    return np.array([[f, 0., (w-1)/2], [0., f, (h-1)/2], [0., 0., 1.]])


def rotation_angle(r):
    return math.acos(float(np.clip((np.trace(r)-1)/2, -1, 1)))


def solve_ground_step(homography, k, points, normal=None, next_points=None):
    """Return old->new camera R, t/height, plane normal in old camera; reject ambiguity."""
    if not np.isfinite(homography).all() or np.linalg.cond(homography) > 1e8:
        raise ValueError("Неустойчивая гомография")
    normalized = np.linalg.inv(k) @ homography @ k
    if np.linalg.det(normalized) <= 0:
        raise ValueError("Нарушена геометрия плоскости")
    if normal is not None and next_points is not None:
        try:
            return fit_known_plane(normalized, k, points, next_points, normal)
        except ValueError:
            # The normal itself was estimated, not measured. If the constrained
            # pixel fit fails, still allow the original bounded-normal solution.
            # Neither path bypasses the constant-height / ambiguity checks.
            pass
    normalized /= np.linalg.svd(normalized, compute_uv=False)[1]
    u, _, vt = np.linalg.svd(normalized)
    rotation_only = u @ vt
    # Below this small non-rotational component, translation / plane normal is
    # unobservable. Do not pick an arbitrary decomposition normal from noise.
    if np.linalg.norm(normalized-rotation_only) < .0015:
        if rotation_angle(rotation_only) > .45:
            raise ValueError("Слишком быстрый поворот")
        return rotation_only, np.zeros(3), normal, "ROTATION_ONLY"
    rays = np.column_stack((points, np.ones(len(points)))) @ np.linalg.inv(k).T
    _, rotations, translations, normals = cv2.decomposeHomographyMat(homography, k)
    candidates = []
    for r, t, n in zip(rotations, translations, normals):
        t, n = t.ravel(), n.ravel()
        if not np.isfinite(r).all() or not np.isfinite(t).all() or not np.isfinite(n).all():
            continue
        # Positive distances to the visible ground, bounded tilt at BOTH frames.
        if min(n[2], (r @ n)[2]) < math.cos(math.radians(60)) or rotation_angle(r) > .45:
            continue
        denominators = rays @ n
        if np.any(denominators < .2) or np.any(((rays/denominators[:, None]) @ r.T + t)[:, 2] <= 0):
            continue
        vertical = abs(float((r @ n) @ t))
        if vertical > max(.0015, .15*np.linalg.norm(t)):
            continue
        normal_error = 0. if normal is None else math.acos(float(np.clip(n @ normal, -1, 1)))
        if normal_error > math.radians(12):
            continue
        score = vertical/max(np.linalg.norm(t), .001) + normal_error
        candidates.append((score, r, t, n))
    if not candidates:
        raise ValueError("Нет решения с постоянной высотой / допустимым наклоном")
    candidates.sort(key=lambda item: item[0])
    best = candidates[0]
    for other in candidates[1:]:
        if other[0]-best[0] < .08 and np.linalg.norm(other[2]-best[2]) > .002:
            raise ValueError("Неоднозначное разделение наклона и перемещения")
    return best[1], best[2], best[3], "TRANSLATION"


def fit_known_plane(h, k, points, next_points, normal):
    """Fit 5-DOF constant-height pose against pixels in the existing ground frame.

    Re-decomposing each noisy near-identity H makes its plane normal unstable.
    Instead keep the observed plane and fit R and two tangent translations:
    H = K R (I - d n^T) K^-1, n.d = 0. Reject fits unsupported by image data.
    """
    right = np.array([1., 0., 0.])-normal[0]*normal
    right /= np.linalg.norm(right)
    tangent = np.column_stack((right, np.cross(normal, right)))
    rays = np.column_stack((points, np.ones(len(points)))) @ np.linalg.inv(k).T
    denominators = rays @ normal
    if normal[2] < math.cos(math.radians(60)) or np.any(denominators < .2):
        raise ValueError("Плоскость вне допустимого наклона / поля зрения")
    world_points = rays/denominators[:, None]  # height = 1
    # H acts as a scaled rotation on the two tangent directions.
    columns = h @ tangent
    scale = np.linalg.norm(columns, axis=0).mean()
    u, _, vt = np.linalg.svd(columns, full_matrices=False)
    orthogonal = u @ vt
    r = np.column_stack((orthogonal, np.cross(orthogonal[:, 0], orthogonal[:, 1]))) @ np.column_stack((tangent, normal)).T
    displacement = normal-r.T @ (h @ normal)/scale
    params = np.r_[cv2.Rodrigues(r)[0].ravel(), tangent.T @ displacement]

    def project(value):
        rotation = cv2.Rodrigues(value[:3])[0]
        camera_points = (world_points-tangent @ value[3:]) @ rotation.T
        if np.any(camera_points[:, 2] <= .05):
            raise ValueError("Точки оказались позади камеры")
        pixels = camera_points @ k.T
        return (pixels[:, :2]/pixels[:, 2:3]).ravel()

    target = next_points.ravel()
    damping = .001
    for _ in range(12):
        predicted = project(params)
        residual = predicted-target
        jacobian = np.empty((len(target), 5))
        for j in range(5):
            shifted = params.copy()
            shifted[j] += 1e-5
            jacobian[:, j] = (project(shifted)-predicted)/1e-5
        gradient = jacobian.T @ residual
        curvature = jacobian.T @ jacobian
        step = np.linalg.solve(curvature+damping*np.diag(np.maximum(np.diag(curvature), 1.)), -gradient)
        candidate = params+step
        try:
            improved = np.sum((project(candidate)-target)**2) < residual @ residual
        except ValueError:
            improved = False
        if improved:
            params = candidate
            damping = max(1e-7, damping*.3)
            if np.linalg.norm(step) < 1e-7:
                break
        else:
            damping *= 10
    rms = np.sqrt(np.mean(np.sum((project(params).reshape(-1, 2)-next_points)**2, axis=1)))
    r = cv2.Rodrigues(params[:3])[0]
    if not np.isfinite(rms) or rms > 1.2:
        raise ValueError("Постоянная плоскость не согласуется с точками кадра")
    if (r @ normal)[2] < math.cos(math.radians(60)) or rotation_angle(r) > .45:
        raise ValueError("Слишком большой наклон / поворот относительно опорного кадра")
    t = -r @ tangent @ params[3:]
    return r, t, normal, "TRANSLATION"


def tracked_homography(old, gray, points):
    args = dict(winSize=(21, 21), maxLevel=3,
                criteria=(cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 30, .01))
    next_points, status, error = cv2.calcOpticalFlowPyrLK(old, gray, points, None, **args)
    if next_points is None:
        raise ValueError("Не найден оптический поток")
    back, back_status, _ = cv2.calcOpticalFlowPyrLK(gray, old, next_points, None, **args)
    if back is None:
        raise ValueError("Не пройдена обратная проверка")
    a, b = points.reshape(-1, 2), next_points.reshape(-1, 2)
    valid = ((status.ravel() != 0) & (back_status.ravel() != 0) & (error.ravel() <= 30) &
             (np.linalg.norm(a-back.reshape(-1, 2), axis=1) <= 1.25) & np.isfinite(b).all(axis=1) &
             (b[:, 0] >= 0) & (b[:, 0] < gray.shape[1]) & (b[:, 1] >= 0) & (b[:, 1] < gray.shape[0]))
    a, b = a[valid], b[valid]
    return checked_homography(a, b, gray.shape)


def checked_homography(a, b, shape):
    """Identical geometric checks for optical flow and descriptor recovery."""
    matched = len(a)
    if len(a) < 40:
        raise ValueError("Недостаточно соответствий; возможна смена сцены")
    homography, mask = cv2.findHomography(a, b, cv2.RANSAC, 2., maxIters=2000, confidence=.995)
    if homography is None or mask is None:
        raise ValueError("Плоскость поверхности не найдена")
    selected = mask.ravel() != 0
    count = int(selected.sum())
    if count < 35 or count/len(a) < .65:
        raise ValueError("Недостаточно точек одной плоскости")
    a, b = a[selected], b[selected]
    if np.any(np.ptp(a, axis=0) < np.array(shape[::-1])*.25):
        raise ValueError("Точки сосредоточены на слишком малом участке")
    projected = cv2.perspectiveTransform(a.reshape(-1, 1, 2), homography).reshape(-1, 2)
    residual = float(np.sqrt(np.mean(np.sum((projected-b)**2, axis=1))))
    if residual > 1.2:
        raise ValueError("Большая ошибка перспективной модели")
    return homography, a, b, count/matched, residual


def descriptor_homography(reference, current, shape):
    """Recover overlap after failed LK; no pose is inferred without matched points."""
    old_keys, old_descriptors = reference
    new_keys, new_descriptors = current
    if old_descriptors is None or new_descriptors is None or min(len(old_keys), len(new_keys)) < 40:
        raise ValueError("Недостаточно ORB-точек для восстановления")
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    forward = matcher.knnMatch(old_descriptors, new_descriptors, k=2)
    backward = matcher.knnMatch(new_descriptors, old_descriptors, k=2)
    reverse = {pair[0].queryIdx: pair[0].trainIdx for pair in backward
               if len(pair) == 2 and pair[0].distance < .75*pair[1].distance}
    matches = [pair[0] for pair in forward if len(pair) == 2
               and pair[0].distance < .75*pair[1].distance
               and reverse.get(pair[0].trainIdx) == pair[0].queryIdx]
    a = np.float32([old_keys[m.queryIdx].pt for m in matches]).reshape(-1, 2)
    b = np.float32([new_keys[m.trainIdx].pt for m in matches]).reshape(-1, 2)
    return checked_homography(a, b, shape)


@dataclass
class ImageReference:
    gray: np.ndarray
    points: np.ndarray
    time_s: float
    descriptors: object = None


class VideoPlanarEngine:
    def __init__(self, height):
        if not math.isfinite(height) or not .05 <= height <= 20:
            raise ValueError("Высота должна быть от 0.05 до 20 м")
        self.height = float(height)
        self.previous = None
        self.pose_reference = None
        self.pending_h = None  # pose reference -> visual reference, in pixel coordinates
        self.pending_frames = 0
        self.pending_error = 0.
        self.origin_time_s = None
        self.initialization_restarts = 0
        self.visual_restarts = 0
        self.segment = 0
        self.rejected_frames = 0
        self.recoveries = 0
        self.rejection_reasons = Counter()
        self.orb = cv2.ORB_create(nfeatures=1800)
        self.position = np.zeros(2)
        self.camera_to_origin = np.eye(3)
        self.normal = self.axes = None
        self.distance = 0.
        self.segment_distance = 0.

    def orb_match(self, reference, gray):
        if reference.descriptors is None:
            reference.descriptors = self.orb.detectAndCompute(reference.gray, None)
        return descriptor_homography(reference.descriptors, self.orb.detectAndCompute(gray, None), gray.shape)

    def match(self, reference, gray):
        try:
            return (*tracked_homography(reference.gray, gray, reference.points), "LK")
        except GEOMETRY_ERRORS as exc:
            try:
                return (*self.orb_match(reference, gray), "ORB")
            except GEOMETRY_ERRORS as other:
                raise ValueError(f"LK: {exc}; ORB: {other}") from other

    def set_pose_reference(self, reference):
        self.pose_reference = self.previous = reference
        self.pending_h = np.eye(3)
        self.pending_frames = 0
        self.pending_error = 0.

    def advance_chain(self, match, time_s, k):
        """Compose old->new on the LEFT. Never drop an uncommitted rotation."""
        if self.pending_h is None:
            return None
        h, a, b, _, residual, _ = match
        frames = self.pending_frames+1
        error = self.pending_error+residual
        if frames > MAX_CHAIN_FRAMES or time_s-self.pose_reference.time_s > MAX_CHAIN_SECONDS or error > MAX_CHAIN_ERROR_PX:
            self.pending_h = None
            return None
        # Map current measured correspondences back through the bounded, checked
        # chain. These are propagated coordinates, not independent observations
        # of the original image; error budget and chain limits remain mandatory.
        anchor_points = cv2.perspectiveTransform(a.reshape(-1, 1, 2), np.linalg.inv(self.pending_h)).reshape(-1, 2)
        combined = h @ self.pending_h
        calibrated = np.linalg.inv(k) @ combined @ k
        determinant = np.linalg.det(calibrated)
        if (not np.isfinite(anchor_points).all() or not np.isfinite(calibrated).all()
                or determinant <= 0 or np.linalg.cond(calibrated) > 1e5):
            self.pending_h = None
            raise ValueError("Неустойчивая цепочка преобразований")
        combined /= np.cbrt(determinant)
        self.pending_h, self.pending_frames, self.pending_error = combined, frames, error
        return combined, anchor_points, b

    def pose_candidate(self, h, a, b, k, max_step):
        expected = self.camera_to_origin.T @ self.normal if self.normal is not None else None
        r, t, n, kind = solve_ground_step(h, k, a, expected, b)
        movement = -self.camera_to_origin @ r.T @ t * self.height
        if np.linalg.norm(movement) > max_step:
            raise ValueError("Слишком большое смещение относительно метрической опоры")
        normal, axes = self.normal, self.axes
        if normal is None and n is not None:
            normal = self.camera_to_origin @ n
            right = np.array([1., 0., 0.]) - normal[0]*normal
            right /= np.linalg.norm(right)
            axes = np.array([right, np.cross(normal, right)])
        delta = axes @ movement if axes is not None else np.zeros(2)
        if np.linalg.norm(delta) < .65*self.height/k[0, 0]:
            delta[:] = 0
        u, _, vt = np.linalg.svd(self.camera_to_origin @ r.T)
        return delta, normal, axes, u @ vt, kind

    def accept_pose(self, candidate, reference, result, pose_method):
        delta, normal, axes, rotation, kind = candidate
        recovered = self.rejected_frames > 0 or reference.time_s-self.pose_reference.time_s > .5
        if recovered:
            self.segment += 1
            self.recoveries += 1
            self.segment_distance = 0.
        self.position += delta
        self.distance += float(np.linalg.norm(delta))
        if not recovered:
            self.segment_distance += float(np.linalg.norm(delta))
        self.normal, self.axes, self.camera_to_origin = normal, axes, rotation
        self.set_pose_reference(reference)
        rejected_frames, self.rejected_frames = self.rejected_frames, 0
        tilt = math.degrees(math.acos(float(np.clip((rotation.T @ normal)[2], -1, 1)))) if normal is not None else None
        result.update(status="TRACKING" if axes is not None else "INITIALIZING",
                      message=("Метрическая связь восстановлена; путь внутри разрыва неизвестен" if recovered else
                               "Оценка по видео" if kind == "TRANSLATION" else "Поворот / смещение ниже порога различимости"),
                      segment=self.segment, recovered=recovered, rejected_frames=rejected_frames, pose_method=pose_method,
                      x_m=float(self.position[0]), y_m=float(self.position[1]), delta_x_m=float(delta[0]), delta_y_m=float(delta[1]),
                      distance_m=self.distance, segment_distance_m=self.segment_distance, tilt_deg=tilt)

    def process(self, gray, time_s, seq):
        result = dict(seq=seq, time_s=float(time_s), segment=self.segment, status="DEGRADED", message="",
                      x_m=float(self.position[0]), y_m=float(self.position[1]), delta_x_m=0., delta_y_m=0.,
                      height_m=self.height, tilt_deg=None, inliers=0, inlier_ratio=0., residual_px=None,
                      distance_m=self.distance, segment_distance_m=self.segment_distance,
                      approximate=True, calibration="assumed_focal_0.9_max_dimension", tracks=[],
                      coordinate_frame="initial_camera_ground", recovered=False, tracking_method=None,
                      rejected_frames=self.rejected_frames, reference_time_s=None, step_time_s=None,
                      visual_reference_time_s=None, visual_status="LOST", pose_method=None,
                      pending_frames=0, chain_error_px=0., chain_connected=False,
                      origin_time_s=self.origin_time_s, initialization_restarted=False)
        points = cv2.goodFeaturesToTrack(gray, 600, .015, 8)
        if self.previous is None:
            if points is None or len(points) < 60:
                result["message"] = "Недостаточно текстуры"
                return result
            self.set_pose_reference(ImageReference(gray.copy(), points, time_s))
            self.origin_time_s = time_s
            result.update(status="INITIALIZING", visual_status="INITIALIZING", origin_time_s=time_s,
                          message="Опорный кадр; ожидается движение вдоль поверхности")
            return result
        visual_reference = self.previous
        pose_reference = self.pose_reference
        result.update(reference_time_s=float(pose_reference.time_s), step_time_s=float(time_s-pose_reference.time_s),
                      visual_reference_time_s=float(visual_reference.time_s))
        current = None
        try:
            if gray.shape != visual_reference.gray.shape or not math.isfinite(time_s) or time_s <= visual_reference.time_s:
                raise ValueError("Неверное время / изменение размера кадра")
            if points is None or len(points) < 60:
                raise ValueError("Потеря текстуры")
            current = ImageReference(gray.copy(), points, time_s)
            k = assumed_intrinsics(gray.shape)
            candidates = []
            errors = []
            try:
                match = self.match(visual_reference, gray)
                h, a, b, ratio, residual, method = match
                # Visual tracking is committed BEFORE and independently of pose.
                self.previous = current
                result.update(visual_status="TRACKING", tracking_method=method, inliers=len(a),
                              inlier_ratio=ratio, residual_px=residual,
                              tracks=np.column_stack((a[:100], b[:100])).tolist())
                chained = self.advance_chain(match, time_s, k)
                result.update(pending_frames=self.pending_frames, chain_error_px=self.pending_error,
                              chain_connected=self.pending_h is not None)
                if chained is not None:
                    candidates.append((*chained, "PAIR" if self.pending_frames == 1 else "CHAIN",
                                       .3*self.pending_frames))
                elif visual_reference is pose_reference:
                    # Direct overlap can recover a pose even after a long gap.
                    candidates.append((h, a, b, "DIRECT", .3))
            except GEOMETRY_ERRORS as exc:
                errors.append(str(exc))
                if self.previous is current:
                    # Image advanced but its transform could not be composed:
                    # retaining the old map would attach it to the wrong image.
                    self.pending_h = None
                # After a genuine image-tracking break, bootstrap visual tracking
                # on a fresh frame, but NEVER attach that local track to the pose.
                if self.previous is visual_reference and time_s-visual_reference.time_s > VISUAL_RESTART_SECONDS:
                    self.previous = current
                    self.pending_h = None
                    self.visual_restarts += 1
                    result["visual_status"] = "RESTARTED"
            for h, a, b, pose_method, max_step in candidates:
                try:
                    candidate = self.pose_candidate(h, a, b, k, max_step)
                    self.accept_pose(candidate, current, result, pose_method)
                    return result
                except GEOMETRY_ERRORS as exc:
                    errors.append(str(exc))
            # Independent direct observations can repair a broken/expired chain.
            # Keep the metric reference even while local visual tracking advances.
            try:
                direct = self.match(pose_reference, gray) if visual_reference is not pose_reference else (*self.orb_match(pose_reference, gray), "ORB")
                h, a, b, ratio, residual, method = direct
                candidate = self.pose_candidate(h, a, b, k, .3)
                self.accept_pose(candidate, current, result, "DIRECT")
                result.update(visual_status="TRACKING", tracking_method=method, inliers=len(a),
                              inlier_ratio=ratio, residual_px=residual,
                              tracks=np.column_stack((a[:100], b[:100])).tolist())
                return result
            except GEOMETRY_ERRORS as exc:
                errors.append(str(exc))
            raise ValueError("; ".join(dict.fromkeys(errors)) or "Нет метрической связи с опорой")
        except GEOMETRY_ERRORS as exc:
            self.rejected_frames += 1
            self.rejection_reasons[str(exc)] += 1
            result.update(rejected_frames=self.rejected_frames,
                          message=f"Позиция удерживается; {'точки отслеживаются, метры не подтверждены' if result['visual_status'] == 'TRACKING' else 'поиск соответствий'}: {exc}")
            # Only BEFORE a plane/metric trajectory has been initialized, allow a
            # new starting window. Its timestamp is explicit: the missing prefix
            # is unknown, not zero movement. An established pose is never rebased.
            if current is not None and self.axes is None and time_s-pose_reference.time_s > MAX_CHAIN_SECONDS:
                self.set_pose_reference(current)
                self.camera_to_origin = np.eye(3)
                self.origin_time_s = time_s
                self.initialization_restarts += 1
                result.update(status="INITIALIZING", origin_time_s=time_s, initialization_restarted=True,
                              message="Новая попытка инициализации; перемещение до этой отметки неизвестно")
        return result
