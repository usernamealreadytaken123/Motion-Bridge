"""Camera/ground geometry matching Android TiltCompensation; no GUI or transport."""
from collections import deque
import math
import numpy as np


def rotation(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if q.shape != (4,) or not np.isfinite(norm) or norm < 1e-12:
        raise ValueError("Invalid quaternion")
    w, x, y, z = q / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def camera_basis(q, sensor, image_rotation):
    a = math.radians(sensor - image_rotation)
    c, s = math.cos(a), math.sin(a)
    return rotation(q) @ np.array([[c, -s, 0], [-s, -c, 0], [0, 0, -1]])


def ground_points(points, k, basis, height):
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    fx, fy, cx, cy = k
    rays = np.column_stack(((points[:, 0]-cx)/fx, (points[:, 1]-cy)/fy, np.ones(len(points)))) @ basis.T
    valid = np.isfinite(rays).all(axis=1) & (-rays[:, 2] / np.linalg.norm(rays, axis=1) >= .35)
    with np.errstate(divide="ignore", invalid="ignore"):
        projected = -height * rays[:, :2] / rays[:, 2:3]
    return projected, valid & np.isfinite(projected).all(axis=1)


def ground_motion(previous, current, k, old_basis, basis, old_height, height):
    if not all(math.isfinite(h) and h > 0 for h in (old_height, height)):
        return None
    old, ok0 = ground_points(previous, k, old_basis, old_height)
    new, ok1 = ground_points(current, k, basis, height)
    sensitivity = np.zeros(len(old))
    valid = ok0 & ok1
    for points, origin, pose, h in ((previous, old, old_basis, old_height), (current, new, basis, height)):
        for shift in ([1, 0], [0, 1]):
            moved, ok = ground_points(np.asarray(points) + shift, k, pose, h)
            valid &= ok
            sensitivity = np.maximum(sensitivity, np.linalg.norm(moved-origin, axis=1))
    indices = np.flatnonzero(valid)
    if len(indices) < 3:
        return None
    differences = (old-new)[indices]
    noise = sensitivity[indices]
    tolerance = np.maximum(max(old_height, height)*.002, noise*2.5)
    # One correspondence defines each deterministic translation RANSAC hypothesis.
    errors = differences[:, None, :] - differences[None, :, :]
    votes = np.sum(errors*errors, axis=2) <= tolerance[None, :]**2
    selected = votes[np.argmax(votes.sum(axis=1))]
    if selected.sum() < max(3, .6*len(previous)):
        return None
    delta = np.median(differences[selected], axis=0)
    selected = np.linalg.norm(differences-delta, axis=1) <= tolerance
    if selected.sum() < max(3, .6*len(previous)):
        return None
    return (delta, indices[selected], float(np.sqrt(np.mean(np.sum((differences[selected]-delta)**2, axis=1)))),
            float(np.median(noise[selected])))


class Orientations:
    def __init__(self):
        self.samples = deque(maxlen=4096)

    def add(self, stamp, q):
        if self.samples and stamp <= self.samples[-1][0]:
            return
        q = np.asarray(q, dtype=float)
        if not np.isfinite(q).all() or np.linalg.norm(q) < 1e-12:
            return
        self.samples.append((stamp, q / np.linalg.norm(q)))

    def at(self, stamp):
        before = None
        for t, q in self.samples:
            if t == stamp:
                return q.copy()
            if t > stamp:
                if before is None or t-before[0] > 80_000_000:
                    return None
                f = (stamp-before[0])/(t-before[0])
                a = before[1]
                dot = float(a @ q)
                if dot < 0:
                    q, dot = -q, -dot
                if dot > .9995:
                    result = a + f*(q-a)
                else:
                    theta = math.acos(np.clip(dot, -1, 1))
                    result = (math.sin((1-f)*theta)*a + math.sin(f*theta)*q)/math.sin(theta)
                return result / np.linalg.norm(result)
            before = (t, q)
        return None


class LidarHeights:
    """Raw phyphox readings; approximate clock fit in ANDROID time, never PC time."""
    def __init__(self):
        self.samples = deque(maxlen=64)
        self.session = None
        self.last_time = None
        self.offset = None
        self.generation = 0

    def invalidate(self):
        self.samples.clear()
        self.offset = None
        self.generation += 1

    def add(self, data):
        if not data["ok"]:
            self.invalidate()
            return
        if self.session != data["session"]:
            self.invalidate()
            self.last_time = None
            self.session = data["session"]
        if not data["measuring"]:
            self.invalidate()
            return
        t = data["time_s"]
        if self.last_time is not None and t < self.last_time:
            self.invalidate()
            self.last_time = t
            return
        if self.last_time == t:
            return
        self.last_time = t
        rtt = data["received_ns"]-data["requested_ns"]
        if rtt < 0 or rtt > 150_000_000:
            self.invalidate()
            return
        if self.samples and (data["received_ns"]-self.samples[-1]["received_ns"] > 500_000_000 or
                             t-self.samples[-1]["time_s"] > .5 or data["mount"] != self.samples[-1]["mount"]):
            self.invalidate()
        offset = data["requested_ns"] + rtt//2 - round(t*1e9)
        self.offset = min(self.offset, offset) if self.offset is not None else offset
        self.samples.append(data)

    def at(self, stamp, watermark, orientations):
        if len(self.samples) < 3 or not 0 <= watermark-self.samples[-1]["received_ns"] <= 500_000_000:
            return None
        before = None
        for after in self.samples:
            t1 = round(after["time_s"]*1e9) + self.offset
            if t1 < stamp:
                before = after
                continue
            if before is None:
                return None
            t0 = round(before["time_s"]*1e9) + self.offset
            if not 0 < t1-t0 <= 250_000_000:
                return None
            heights = []
            for sample, t in ((before, t0), (after, t1)):
                q = orientations.at(t)
                if q is None:
                    return None
                r = rotation(q)
                cosine = r[2, 2]
                if cosine < math.cos(math.radians(55)):
                    return None
                heights.append(sample["depth_m"]*cosine + (r @ sample["mount"])[2])
            if not all(.05 <= h <= 20 for h in heights) or abs(heights[1]-heights[0]) > .02+(t1-t0)/1e9:
                return None
            f = (stamp-t0)/(t1-t0)
            return float(heights[0] + f*(heights[1]-heights[0])), self.generation
        return None
