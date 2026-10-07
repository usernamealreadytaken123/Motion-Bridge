"""Read-only JSONL replay with a compact disk index and time-budgeted seeking."""

from array import array
import base64
from bisect import bisect_right
from dataclasses import dataclass
import ipaddress
import json
import math
from pathlib import Path
from threading import Event, Thread
import time

from .history import DisplayHistory
from .protocol import MAX_INT64, PacketError, _unique_object, parse_packet
from .receiver import TelemetryState

MAX_LINE_BYTES = 1_048_576
MAX_RECORDS = 2_000_000


def decode_record(line: bytes, line_number: int) -> tuple[dict, bytes]:
    try:
        if len(line) > MAX_LINE_BYTES:
            raise ValueError("Строка превышает 1 МиБ")
        record = json.loads(line.decode("utf-8"), object_pairs_hook=_unique_object)
        if not isinstance(record, dict) or type(record.get("record_version")) is not int or record["record_version"] != 1:
            raise ValueError("Неизвестный формат записи")
        for field in ("pc_receive_monotonic_ns", "pc_receive_unix_ns"):
            value = record.get(field)
            if type(value) is not int or not 0 <= value <= MAX_INT64:
                raise ValueError(f"Некорректное время: {field}")
        sender = record.get("sender")
        if not isinstance(sender, list) or len(sender) != 2 or not isinstance(sender[0], str):
            raise ValueError("Некорректный sender")
        ipaddress.IPv4Address(sender[0])
        if type(sender[1]) is not int or not 1 <= sender[1] <= 65535:
            raise ValueError("Некорректный порт sender")
        raw = base64.b64decode(record["raw_base64"], validate=True)
        try:
            packet = parse_packet(raw)
        except PacketError:
            if record.get("payload") is not None or not isinstance(record.get("error"), str) or not record["error"]:
                raise ValueError("Повреждённая дейтаграмма не помечена как отклонённая")
        else:
            if record.get("payload") != packet.payload or record.get("error") is not None:
                raise ValueError("Payload не соответствует исходной дейтаграмме")
        return record, raw
    except (ValueError, KeyError, TypeError, RecursionError) as exc:
        raise ValueError(f"Строка {line_number}: {exc}") from exc


def _signature(path: Path) -> tuple[int, int]:
    stat = path.stat()
    return stat.st_size, stat.st_mtime_ns


@dataclass
class RecordingIndex:
    path: Path
    offsets: array
    times: array
    signature: tuple[int, int]

    @property
    def duration(self) -> float:
        return (self.times[-1] - self.times[0]) / 1e9

    @classmethod
    def load(cls, path: Path, cancelled=None, progress=None):
        path = Path(path).resolve()
        signature = _signature(path)
        offsets, times = array("Q"), array("Q")
        with path.open("rb") as source:
            while True:
                if cancelled and cancelled():
                    return None
                offset = source.tell()
                line = source.readline(MAX_LINE_BYTES + 1)
                if not line:
                    break
                record, _ = decode_record(line, len(times) + 1)
                stamp = record["pc_receive_monotonic_ns"]
                if times and stamp < times[-1]:
                    raise ValueError(f"Строка {len(times) + 1}: время ПК идёт назад; файл нельзя склеивать из разных запусков")
                offsets.append(offset)
                times.append(stamp)
                if len(times) > MAX_RECORDS:
                    raise ValueError("Слишком большая запись: максимум 2 000 000 дейтаграмм")
                if progress and len(times) % 256 == 0:
                    progress(source.tell() / max(signature[0], 1))
        if not times:
            raise ValueError("Файл не содержит записей телеметрии")
        if _signature(path) != signature:
            raise ValueError("Файл изменился во время открытия; сначала завершите запись")
        return cls(path, offsets, times, signature)


class ReplayLoader(Thread):
    def __init__(self, path):
        super().__init__(name="replay-index", daemon=True)
        self.path = Path(path)
        self.cancelled = Event()
        self.result = None
        self.error = None
        self.progress = 0.0

    def run(self):
        try:
            self.result = RecordingIndex.load(self.path, self.cancelled.is_set,
                                              lambda value: setattr(self, "progress", value))
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"


class ReplayEngine:
    def __init__(self, index: RecordingIndex):
        self.index = index
        self.source = index.path.open("rb")
        self.state = TelemetryState()
        self.history = DisplayHistory()
        self.cursor = 0
        self.position = 0.0

    @property
    def now_ns(self):
        return self.index.times[0] + round(self.position * 1e9)

    def advance(self, position: float, budget_seconds: float = 0.012) -> bool:
        if not math.isfinite(position):
            raise ValueError("Некорректная позиция воспроизведения")
        if _signature(self.index.path) != self.index.signature:
            raise ValueError("Файл записи изменён после открытия")
        position = min(max(0.0, position), self.index.duration)
        target_ns = self.index.times[0] + round(position * 1e9)
        target_cursor = bisect_right(self.index.times, target_ns)
        if target_cursor < self.cursor or position < self.position:
            self.state = TelemetryState()
            self.history = DisplayHistory()
            self.cursor = 0
        deadline = time.perf_counter() + budget_seconds
        if self.cursor < target_cursor:
            self.source.seek(self.index.offsets[self.cursor])
        while self.cursor < target_cursor:
            record, raw = decode_record(self.source.readline(MAX_LINE_BYTES + 1), self.cursor + 1)
            event = self.state.process(raw, tuple(record["sender"]), record["pc_receive_monotonic_ns"],
                                       record["pc_receive_unix_ns"])
            self.history.observe(event)
            self.cursor += 1
            if time.perf_counter() >= deadline:
                break
        complete = self.cursor == target_cursor
        self.position = position if complete else (self.index.times[self.cursor - 1] - self.index.times[0]) / 1e9
        return complete

    def close(self):
        self.source.close()


class PlaybackClock:
    def __init__(self, duration: float):
        self.duration = duration
        self.position = 0.0
        self.speed = 1.0
        self.playing = False
        self._last = None

    def update(self, now: float):
        if self.playing and self._last is not None:
            self.position = min(self.duration, self.position + max(0.0, now - self._last) * self.speed)
            if self.position >= self.duration:
                self.playing = False
        self._last = now
        return self.position

    def seek(self, position: float, now: float):
        self.position = min(self.duration, max(0.0, position))
        self._last = now

    def set_playing(self, playing: bool, now: float):
        self.update(now)
        self.playing = playing and self.duration > 0

    def set_speed(self, speed: float, now: float):
        if speed not in (0.25, 0.5, 1.0, 2.0, 4.0, 8.0):
            raise ValueError("Unsupported playback speed")
        self.update(now)
        self.speed = speed
