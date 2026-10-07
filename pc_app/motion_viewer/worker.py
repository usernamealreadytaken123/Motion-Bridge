"""UDP/recording worker with a bounded mailbox. No Qt calls outside the GUI thread."""

from collections import deque
from pathlib import Path
from threading import Event, Lock, Thread
import time

from .receiver import TelemetryReceiver
from .recording import SessionRecorder
from .preferences import LOGGER


class ReceiverWorker(Thread):
    def __init__(self, host: str, port: int, record_path: Path | None = None,
                 mailbox_capacity: int = 2048):
        super().__init__(name="udp-receiver", daemon=True)
        self.host, self.port, self.record_path = host, port, record_path
        self._stop_requested = Event()
        self._lock = Lock()
        self._events: deque[dict] = deque(maxlen=mailbox_capacity)
        self._snapshot = {"datagrams": 0, "invalid": 0, "last_error": None,
                          "evicted_streams": 0, "streams": []}
        self._address: tuple[str, int] | None = None
        self._error: str | None = None
        self._finished = False
        self._display_dropped = 0
        self._written = 0

    def stop(self) -> None:
        self._stop_requested.set()

    def take_update(self) -> dict:
        with self._lock:
            events = list(self._events)
            self._events.clear()
            return {"events": events, "snapshot": self._snapshot,
                    "address": self._address, "error": self._error,
                    "finished": self._finished, "display_dropped": self._display_dropped,
                    "written": self._written}

    def run(self) -> None:
        receiver = None
        recorder = None
        try:
            receiver = TelemetryReceiver(self.host, self.port)
            if self.record_path is not None:
                recorder = SessionRecorder(self.record_path)
                receiver.recorder = recorder
            with self._lock:
                self._address = receiver.address
            next_snapshot = 0.0
            while not self._stop_requested.is_set():
                event = receiver.receive_once()
                if event and event.get("sequence_result") == "new":
                    with self._lock:
                        if len(self._events) == self._events.maxlen:
                            self._display_dropped += 1
                        self._events.append(event)
                now = time.monotonic()
                if now >= next_snapshot:
                    snapshot = receiver.snapshot()
                    with self._lock:
                        self._snapshot = snapshot
                        self._written = recorder.written if recorder else 0
                    next_snapshot = now + 0.1
        except Exception as exc:
            LOGGER.exception("UDP receiver failed")
            with self._lock:
                self._error = f"{type(exc).__name__}: {exc}"
        finally:
            if receiver is not None:
                snapshot = receiver.snapshot()
                receiver.close()
                with self._lock:
                    self._snapshot = snapshot
            if recorder is not None:
                try:
                    recorder.close()
                except Exception as exc:
                    LOGGER.exception("Could not finalize recording")
                    with self._lock:
                        self._error = f"{type(exc).__name__}: {exc}"
            with self._lock:
                self._written = recorder.written if recorder else 0
                self._finished = True
