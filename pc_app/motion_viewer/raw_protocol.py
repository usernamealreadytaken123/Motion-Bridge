"""Length-framed MVO stream: RAW GRAY8 v1 and low-latency H.264 v2."""
import json
import math
import struct
import time

PREFIX = struct.Struct("!4sII")
MAX_HEADER = 65_536
MAX_PIXELS = 1280*960
MAX_BINARY = 4*1024*1024


def integer(value, low=0, high=2**63-1):
    if type(value) is not int or not low <= value <= high:
        raise ValueError("Invalid integer in capture stream")
    return value


def numbers(value, count):
    if not isinstance(value, list) or len(value) != count or any(
        type(v) not in (float, int) or not math.isfinite(v) for v in value
    ):
        raise ValueError("Invalid numeric vector in capture stream")
    return value


def boolean(value):
    if type(value) is not bool:
        raise ValueError("Expected boolean in capture stream")
    return value


def validate(header, payload):
    if not isinstance(header, dict):
        raise ValueError("Capture header must be an object")
    kind = header.get("type")
    if kind == "hello":
        protocol = integer(header.get("protocol"), 1, 2)
        encoding = header.get("encoding", "gray8" if protocol == 1 else None)
        if encoding != ("gray8" if protocol == 1 else "h264") or not isinstance(header.get("session"), str) or not 1 <= len(header["session"]) <= 128:
            raise ValueError("Unsupported capture handshake")
    elif kind == "sensors":
        integer(header.get("watermark_ns"), 1)
        samples = header.get("samples")
        if not isinstance(samples, list) or len(samples) > 256:
            raise ValueError("Oversized sensor batch")
        for sample in samples:
            if not isinstance(sample, list) or len(sample) < 2 or sample[0] not in ("q", "gyro", "accel", "linear"):
                raise ValueError("Unknown sensor sample")
            integer(sample[1], 1)
            values = numbers(sample[2:], 4 if sample[0] == "q" else 3)
            if sample[0] == "q" and not .9 <= math.hypot(*values) <= 1.1:
                raise ValueError("Invalid orientation norm")
    elif kind in ("frame", "video_packet"):
        w, h = integer(header.get("width"), 16, 1280), integer(header.get("height"), 16, 1280)
        if w*h > MAX_PIXELS:
            raise ValueError("Invalid frame size")
        if kind == "frame" and len(payload) != w*h:
            raise ValueError("Invalid GRAY8 frame size")
        if kind == "video_packet" and not 1 <= len(payload) <= MAX_BINARY:
            raise ValueError("Invalid H.264 access unit")
        integer(header.get("timestamp_ns"), 1)
        integer(header.get("seq"))
        integer(header.get("dropped", 0))
        if kind == "video_packet":
            pts = integer(header.get("pts_us"), 1)
            if abs(pts*1_000-header["timestamp_ns"]) > 1_000:
                raise ValueError("H.264 PTS does not match CameraX timestamp")
            boolean(header.get("keyframe"))
        if header.get("rotation") not in (0, 90, 180, 270) or type(header["rotation"]) is not int:
            raise ValueError("Invalid image rotation")
        sensor = header.get("sensor_orientation")
        if sensor is not None and (type(sensor) is not int or sensor not in (0, 90, 180, 270)):
            raise ValueError("Invalid sensor orientation")
        for key in ("realtime", "back_camera", "approximate"):
            boolean(header.get(key))
        fx, fy, cx, cy = numbers(header.get("intrinsics"), 4)
        if min(fx, fy) <= 0 or max(fx, fy) > 100000 or abs(cx) > 100000 or abs(cy) > 100000:
            raise ValueError("Invalid camera calibration")
    elif kind == "video_config":
        if header.get("codec") != "h264" or not 1 <= len(payload) <= 65_536:
            raise ValueError("Invalid H.264 configuration")
        w, h = integer(header.get("width"), 16, 1280), integer(header.get("height"), 16, 1280)
        if w*h > MAX_PIXELS or w % 2 or h % 2:
            raise ValueError("Invalid H.264 dimensions")
        integer(header.get("fps"), 1, 60)
        integer(header.get("bitrate"), 250_000, 8_000_000)
    elif kind == "lidar":
        if boolean(header.get("ok")):
            integer(header.get("requested_ns"), 1)
            integer(header.get("received_ns"), 1)
            if not isinstance(header.get("session"), str) or not 1 <= len(header["session"]) <= 128:
                raise ValueError("Invalid LiDAR session")
            if boolean(header.get("measuring")):
                d, t = numbers([header.get("depth_m"), header.get("time_s")], 2)
                if not .05 <= d <= 5 or not 0 <= t <= 1e8:
                    raise ValueError("Invalid LiDAR measurement")
            if any(abs(v) > 1 for v in numbers(header.get("mount"), 3)):
                raise ValueError("Invalid camera mount")
    else:
        raise ValueError("Unknown capture message")
    if kind not in ("frame", "video_config", "video_packet") and payload:
        raise ValueError("Unexpected binary payload")
    return header, payload


def encode(header, payload=b""):
    validate(header, payload)
    data = json.dumps(header, separators=(",", ":"), allow_nan=False).encode("utf-8")
    if len(data) > MAX_HEADER:
        raise ValueError("Oversized capture header")
    return PREFIX.pack(b"MVO1", len(data), len(payload)) + data + payload


def _unique(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("Duplicate capture key")
        result[key] = value
    return result


def read_message(read):
    """read(n) must return exactly n bytes or raise EOFError; no unbounded allocation."""
    magic, nh, nb = PREFIX.unpack(read(PREFIX.size))
    if magic != b"MVO1" or not 1 <= nh <= MAX_HEADER or nb > MAX_BINARY:
        raise ValueError("Invalid MVO1 envelope")
    header = json.loads(read(nh).decode("utf-8"), object_pairs_hook=_unique,
                        parse_constant=lambda v: (_ for _ in ()).throw(ValueError(v)))
    return validate(header, read(nb) if nb else b"")


class SocketReader:
    def __init__(self, sock, stop):
        self.sock, self.stop = sock, stop

    def read(self, n):
        data = bytearray()
        deadline = time.monotonic() + 3
        while len(data) < n:
            if self.stop.is_set():
                raise EOFError("Capture stopped")
            if time.monotonic() > deadline:
                raise TimeoutError("Capture stream stalled")
            try:
                part = self.sock.recv(n-len(data))
            except TimeoutError:
                continue
            if not part:
                raise EOFError("Android disconnected")
            data.extend(part)
        return bytes(data)
