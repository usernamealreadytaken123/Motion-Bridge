"""UI-independent transport: call receive_once from the future Qt worker."""

import base64
from collections import OrderedDict
import socket
import time

from .protocol import PacketError, parse_packet
from .recording import SessionRecorder
from .statistics import StreamStats


class TelemetryState:
    """Packet processing shared by live UDP and offline replay; does not open sockets."""

    def __init__(self, recorder: SessionRecorder | None = None):
        self.recorder = recorder
        self.streams: OrderedDict[tuple[str, int, str | None], StreamStats] = OrderedDict()
        self.datagrams = 0
        self.invalid = 0
        self.last_error: str | None = None
        self.evicted_streams = 0

    def process(self, data: bytes, sender: tuple[str, int], received_ns: int,
                received_unix_ns: int) -> dict:
        self.datagrams += 1
        event = {
            "record_version": 1,
            "pc_receive_monotonic_ns": received_ns,
            "pc_receive_unix_ns": received_unix_ns,
            "sender": list(sender),
            "raw_base64": base64.b64encode(data).decode("ascii"),
            "payload": None,
            "error": None,
        }
        try:
            packet = parse_packet(data)
        except PacketError as exc:
            self.invalid += 1
            self.last_error = str(exc)
            event["error"] = self.last_error
        else:
            key = (*sender, packet.session_id)
            if key not in self.streams:
                if len(self.streams) >= 16:
                    self.streams.popitem(last=False)
                    self.evicted_streams += 1
                self.streams[key] = StreamStats()
            self.streams.move_to_end(key)
            event["sequence_result"] = self.streams[key].observe(packet, received_ns)
            event["payload"] = packet.payload
        if self.recorder:
            self.recorder.submit(event)
        return event

    def snapshot(self, now_ns: int | None = None) -> dict:
        if now_ns is None:
            now_ns = time.monotonic_ns()
        streams = []
        for (ip, port, session_id), stats in self.streams.items():
            latest = stats.latest
            streams.append({
                "sender": f"{ip}:{port}",
                "session_id": session_id,
                "protocol_version": latest.protocol_version if latest else None,
                **stats.snapshot(now_ns),
                "quaternion": latest.payload["quaternion"] if latest else None,
                "gyroscope": latest.payload["gyroscope"] if latest else None,
                "linear_acceleration": latest.payload["linear_acceleration"] if latest else None,
                # Preserve absolute pose and frame timestamps; never sum repeated deltas.
                "planar_odometry": latest.payload.get("planar_odometry") if latest else None,
            })
        return {
            "datagrams": self.datagrams,
            "invalid": self.invalid,
            "last_error": self.last_error,
            "evicted_streams": self.evicted_streams,
            "streams": streams,
        }

class TelemetryReceiver(TelemetryState):
    def __init__(self, host: str = "0.0.0.0", port: int = 5005,
                 recorder: SessionRecorder | None = None):
        super().__init__(recorder)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1_048_576)
            self.socket.bind((host, port))
            self.socket.settimeout(0.1)
        except OSError:
            self.socket.close()
            raise
        self.address = self.socket.getsockname()

    def receive_once(self) -> dict | None:
        if self.recorder:
            self.recorder.check()
        try:
            data, sender = self.socket.recvfrom(65_535)
        except socket.timeout:
            return None
        return self.process(data, sender, time.monotonic_ns(), time.time_ns())

    def close(self) -> None:
        self.socket.close()
