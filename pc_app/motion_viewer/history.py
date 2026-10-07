"""Bounded display history, independent of Qt and of the Android odometry algorithm."""

from collections import OrderedDict, deque
from dataclasses import dataclass, field
import math


StreamKey = tuple[str, int, str | None]
SENSOR_NAMES = ("quaternion", "gyroscope", "linear_acceleration")
STALE_NS = 2_000_000_000


@dataclass
class StreamHistory:
    start_ns: int
    latest: dict | None = None
    sensors: dict = field(default_factory=lambda: {
        name: deque(maxlen=1500) for name in SENSOR_NAMES
    })
    sensor_stamps: dict[str, int] = field(default_factory=dict)
    sensor_updates: dict[str, int] = field(default_factory=dict)
    trajectory: deque = field(default_factory=lambda: deque(maxlen=5000))
    epoch: int | None = None
    frame_stamp: int = 0
    frame_updated_ns: int | None = None
    pose_updated_ns: int | None = None
    break_path: bool = True
    last_sequence: int = -1

    def observe(self, payload: dict, received_ns: int) -> None:
        # Defence in depth: display data can only advance, even if used without the receiver.
        if payload["sequence"] <= self.last_sequence:
            return
        self.last_sequence = payload["sequence"]
        self.latest = payload
        time_s = (received_ns - self.start_ns) / 1e9
        timestamps = payload.get("sensor_timestamps_ns")
        for name in SENSOR_NAMES:
            if timestamps is not None:
                stamp = timestamps[name]
                if stamp == 0 or stamp <= self.sensor_stamps.get(name, -1):
                    continue
                self.sensor_stamps[name] = stamp
            axes = "wxyz" if name == "quaternion" else "xyz"
            series = self.sensors[name]
            previous = self.sensor_updates.get(name)
            if previous is not None and received_ns - previous > STALE_NS and series:
                series.append((time_s, *(math.nan for _ in axes)))
            series.append((time_s, *(payload[name][axis] for axis in axes)))
            self.sensor_updates[name] = received_ns

        planar = payload.get("planar_odometry")
        if planar is None:
            self.break_path = True
            return
        if self.epoch is not None and planar["epoch"] < self.epoch:
            return
        if planar["epoch"] != self.epoch:
            self.epoch = planar["epoch"]
            self.trajectory.clear()
            self.frame_stamp = 0
            self.frame_updated_ns = self.pose_updated_ns = None
            self.break_path = True
        tracking = planar["status"] == "TRACKING" and planar["camera_active"]
        if not tracking:
            self.break_path = True
        stamp = planar["timestamp_ns"]
        if stamp == 0 or stamp <= self.frame_stamp:
            return
        self.frame_stamp = stamp
        self.frame_updated_ns = received_ns
        if tracking:
            if self.pose_updated_ns is not None and received_ns - self.pose_updated_ns > STALE_NS:
                self.break_path = True
            if self.break_path and self.trajectory:
                self.trajectory.append((math.nan, math.nan))
            # The Android position is absolute. Repeated frame deltas are never integrated.
            self.trajectory.append((planar["x_m"], planar["y_m"]))
            self.pose_updated_ns = received_ns
            self.break_path = False

    def camera_state(self, now_ns: int) -> str:
        planar = self.latest.get("planar_odometry") if self.latest else None
        if planar is None:
            return "Нет данных камеры"
        if not planar["camera_active"]:
            return f"Камера выключена · {planar['status']}"
        if planar["status"] in ("STOPPED", "ERROR", "STARTING_CAMERA"):
            return planar["status"]
        if self.frame_updated_ns is None:
            return "Ожидание первого кадра"
        if now_ns - self.frame_updated_ns > STALE_NS:
            return "Кадры не обновляются > 2 с"
        return planar["status"]

    def clear_plots(self) -> None:
        for series in self.sensors.values():
            series.clear()
        self.trajectory.clear()
        self.break_path = True


class DisplayHistory:
    def __init__(self) -> None:
        self.streams: OrderedDict[StreamKey, StreamHistory] = OrderedDict()
        self.last_key: StreamKey | None = None

    def observe(self, event: dict) -> None:
        if event.get("payload") is None or event.get("sequence_result") != "new":
            return
        payload = event["payload"]
        key = (*event["sender"], payload.get("session_id"))
        if key not in self.streams:
            if len(self.streams) >= 16:
                self.streams.popitem(last=False)
            self.streams[key] = StreamHistory(event["pc_receive_monotonic_ns"])
        self.streams.move_to_end(key)
        self.streams[key].observe(payload, event["pc_receive_monotonic_ns"])
        self.last_key = key

    def mark_gap(self) -> None:
        # A slow UI may skip display events, but raw recording remains independent.
        for history in self.streams.values():
            history.break_path = True

    def clear_plots(self) -> None:
        for history in self.streams.values():
            history.clear_plots()
