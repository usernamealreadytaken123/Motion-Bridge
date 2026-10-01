"""Append-only session files; disk writes run outside the UDP receiver."""

import json
from pathlib import Path
from queue import Full, Queue
from threading import Thread


class RecordingError(RuntimeError):
    pass


class SessionRecorder:
    def __init__(self, path: Path, capacity: int = 4096):
        path.parent.mkdir(parents=True, exist_ok=True)
        self._file = path.open("x", encoding="utf-8", newline="\n")
        self._queue: Queue[dict | None] = Queue(maxsize=capacity)
        self._error: Exception | None = None
        self._closed = False
        self.written = 0
        self._thread = Thread(target=self._write, name="session-writer", daemon=True)
        self._thread.start()

    def _write(self) -> None:
        try:
            while True:
                item = self._queue.get()
                if item is None:
                    break
                self._file.write(json.dumps(item, ensure_ascii=False, allow_nan=False) + "\n")
                self.written += 1
                # Flush each record at this stage; receive thread never waits for disk.
                self._file.flush()
        except Exception as exc:
            self._error = exc
        finally:
            try:
                self._file.close()
            except OSError as exc:
                self._error = exc

    def check(self) -> None:
        if self._error is not None:
            raise RecordingError(f"Session write failed: {self._error}") from self._error

    def submit(self, event: dict) -> None:
        self.check()
        if self._closed:
            raise RecordingError("Recorder already closed")
        try:
            self._queue.put_nowait(event)
        except Full as exc:
            raise RecordingError("Recording queue full; stopping to avoid silent data loss") from exc

    def close(self) -> None:
        if not self._closed:
            self._closed = True
            if self._thread.is_alive():
                try:
                    self._queue.put(None, timeout=5)
                except Full as exc:
                    raise RecordingError("Recorder could not drain its queue") from exc
                self._thread.join(timeout=5)
                if self._thread.is_alive():
                    raise RecordingError("Recorder is still flushing; file may be incomplete")
        self.check()
