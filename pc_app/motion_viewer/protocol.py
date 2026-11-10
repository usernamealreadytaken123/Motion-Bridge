"""Strict reader for the existing Android JSON and its additive v2 extension."""

from dataclasses import dataclass
import json
import math
from typing import Any

MAX_DATAGRAM_BYTES = 65_507
MAX_INT64 = 2**63 - 1
STATUSES = frozenset({
    "STOPPED", "STARTING_CAMERA", "INITIALIZING", "TRACKING", "DEGRADED", "LOST", "ERROR",
})


class PacketError(ValueError):
    """A bad datagram is counted and recorded, never allowed to kill reception."""


@dataclass(frozen=True)
class Vector3:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class Quaternion:
    w: float
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class PlanarOdometry:
    epoch: int
    timestamp_ns: int
    status: str
    camera_active: bool
    x_m: float
    y_m: float
    heading_rad: float
    delta_x_m: float
    delta_y_m: float
    surface_distance_m: float
    tracked_points: int
    inliers: int
    inlier_ratio: float
    image_scale: float
    intrinsics_approximate: bool


@dataclass(frozen=True)
class Telemetry:
    protocol_version: int
    session_id: str | None
    sequence: int
    timestamp: float
    quaternion: Quaternion
    gyroscope: Vector3
    linear_acceleration: Vector3
    height_above_surface_m: float | None
    device_timestamp_ns: int | None
    sensor_timestamps_ns: dict[str, int] | None
    planar_odometry: PlanarOdometry | None
    payload: dict[str, Any]


def _object(value: Any, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise PacketError(f"{name}: expected object")
    return value


def _integer(value: Any, name: str) -> int:
    if type(value) is not int or not 0 <= value <= MAX_INT64:
        raise PacketError(f"{name}: expected nonnegative int64")
    return value


def _number(value: Any, name: str) -> float:
    if type(value) not in (int, float):
        raise PacketError(f"{name}: expected finite number")
    try:
        result = float(value)
    except (ValueError, OverflowError) as exc:
        raise PacketError(f"{name}: number out of range") from exc
    if not math.isfinite(result):
        raise PacketError(f"{name}: expected finite number")
    return result


def _boolean(value: Any, name: str) -> bool:
    if type(value) is not bool:
        raise PacketError(f"{name}: expected boolean")
    return value


def _vector(value: Any, name: str) -> Vector3:
    obj = _object(value, name)
    return Vector3(*(_number(obj.get(axis), f"{name}.{axis}") for axis in "xyz"))


def _unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    obj: dict[str, Any] = {}
    for key, value in pairs:
        if key in obj:
            raise PacketError(f"Duplicate key: {key}")
        obj[key] = value
    return obj


def _reject_constant(value: str) -> None:
    raise PacketError(f"Invalid JSON number: {value}")


def _parse_planar(value: Any) -> PlanarOdometry | None:
    if value is None:
        return None
    obj = _object(value, "planar_odometry")
    status = obj.get("status")
    if not isinstance(status, str) or status not in STATUSES:
        raise PacketError("planar_odometry.status: unsupported status")
    values = {
        name: _number(obj.get(name), name)
        for name in (
            "x_m", "y_m", "heading_rad", "delta_x_m", "delta_y_m",
            "surface_distance_m", "inlier_ratio", "image_scale",
        )
    }
    tracked = _integer(obj.get("tracked_points"), "tracked_points")
    inliers = _integer(obj.get("inliers"), "inliers")
    if inliers > tracked or not 0 <= values["inlier_ratio"] <= 1:
        raise PacketError("Invalid inlier statistics")
    if values["surface_distance_m"] <= 0 or values["image_scale"] <= 0:
        raise PacketError("Distance and scale must be positive")
    if obj.get("height_source") == "IPHONE_LIDAR":
        valid_height = _boolean(obj.get("height_valid"), "height_valid")
        for name in ("lidar_depth_m", "lidar_age_ms", "lidar_rtt_ms", "lidar_time_s"):
            value = obj.get(name)
            if value is None:
                if valid_height:
                    raise PacketError(f"{name}: required for valid LiDAR height")
            elif _number(value, name) < 0:
                raise PacketError(f"{name}: must be nonnegative")
    return PlanarOdometry(
        epoch=_integer(obj.get("epoch"), "epoch"),
        timestamp_ns=_integer(obj.get("timestamp_ns"), "camera timestamp"),
        status=status,
        camera_active=_boolean(obj.get("camera_active"), "camera_active"),
        tracked_points=tracked,
        inliers=inliers,
        intrinsics_approximate=_boolean(obj.get("intrinsics_approximate"), "intrinsics_approximate"),
        **values,
    )


def parse_packet(data: bytes) -> Telemetry:
    if not data or len(data) > MAX_DATAGRAM_BYTES:
        raise PacketError("Empty or oversized datagram")
    try:
        payload = _object(json.loads(
            data.decode("utf-8"),
            object_pairs_hook=_unique_object,
            parse_constant=_reject_constant,
            parse_float=lambda value: _number(float(value), "JSON float"),
        ), "packet")
    except (UnicodeError, ValueError, RecursionError) as exc:
        raise PacketError(str(exc)) from exc

    version = _integer(payload.get("protocol_version", 1), "protocol_version")
    if version not in (1, 2):
        raise PacketError(f"Unsupported protocol version: {version}")
    quat = _object(payload.get("quaternion"), "quaternion")
    components = [_number(quat.get(axis), f"quaternion.{axis}") for axis in "wxyz"]
    # Do not silently display an invalid rotation; 4-decimal Android rounding is well within this.
    if not 0.9 <= math.hypot(*components) <= 1.1:
        raise PacketError("Quaternion is not approximately unit length")
    height = payload.get("height_above_surface_m")
    if height is not None:
        height = _number(height, "height_above_surface_m")
        if height <= 0:
            raise PacketError("Height must be positive")
    session = None
    device_time = None
    sensor_times = None
    planar = None
    if version == 2:
        if payload.get("packet_type") != "telemetry":
            raise PacketError("Unsupported packet_type")
        session = payload.get("session_id")
        if (not isinstance(session, str) or not 1 <= len(session) <= 64
                or not all(c.isascii() and (c.isalnum() or c in "-_") for c in session)):
            raise PacketError("Invalid session_id")
        device_time = _integer(payload.get("device_timestamp_ns"), "device_timestamp_ns")
        times = _object(payload.get("sensor_timestamps_ns"), "sensor_timestamps_ns")
        sensor_times = {
            name: _integer(times.get(name), f"sensor_timestamps_ns.{name}")
            for name in ("quaternion", "gyroscope", "linear_acceleration")
        }
        if "planar_odometry" not in payload:
            raise PacketError("Missing planar_odometry (use null if unavailable)")
        planar = _parse_planar(payload["planar_odometry"])
    return Telemetry(
        protocol_version=version,
        session_id=session,
        sequence=_integer(payload.get("sequence"), "sequence"),
        timestamp=_number(payload.get("timestamp"), "timestamp"),
        quaternion=Quaternion(*components),
        gyroscope=_vector(payload.get("gyroscope"), "gyroscope"),
        linear_acceleration=_vector(payload.get("linear_acceleration"), "linear_acceleration"),
        height_above_surface_m=height,
        device_timestamp_ns=device_time,
        sensor_timestamps_ns=sensor_times,
        planar_odometry=planar,
        payload=payload,
    )
