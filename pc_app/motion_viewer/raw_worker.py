"""Bounded TCP receiver / raw-session recomputation, independent of Qt."""
from datetime import datetime
from collections import deque
import json
from pathlib import Path
import secrets
import socket
from threading import Event, Lock, Thread

from .raw_protocol import encode, read_message, SocketReader
from .pc_odometry import CaptureProcessor
from .preferences import LOGGER


class RawRecorder:
    def __init__(self, directory, height, use_lidar, protocol=1):
        self.directory = Path(directory) / (datetime.now().strftime("raw-%Y%m%d-%H%M%S-") + secrets.token_hex(3))
        self.directory.mkdir(parents=True, exist_ok=False)
        self.metadata = dict(format="MVO1", capture_protocol=protocol,
                             encoding="h264" if protocol == 2 else "gray8",
                             height_m=height, use_lidar=use_lidar, complete=False,
                             note="GRAY8 frames and source timestamps; not MP4 or result-only JSONL")
        self._manifest()
        self.capture = None
        self.results = None
        try:
            self.capture = (self.directory / "capture.mvo").open("xb")
            self.results = (self.directory / "results.jsonl").open("x", encoding="utf-8")
        except Exception:
            self.close("Cannot open recording files")
            raise

    def _manifest(self):
        (self.directory / "manifest.json").write_text(json.dumps(self.metadata, indent=2, ensure_ascii=False), encoding="utf-8")

    def message(self, header, payload):
        # Pairing code is transient, not part of the scientific recording.
        header = {k: v for k, v in header.items() if k != "token"}
        self.capture.write(encode(header, payload))

    def result(self, result):
        self.results.write(json.dumps(result, allow_nan=False, separators=(",", ":")) + "\n")

    def close(self, error=None):
        try:
            if self.capture:
                self.capture.close()
        finally:
            if self.results:
                self.results.close()
            self.metadata.update(complete=error is None, error=error)
            self._manifest()


def recorded_messages(path):
    with Path(path).open("rb") as source:
        def exact(n):
            data = source.read(n)
            if len(data) != n:
                raise ValueError("Оборванная запись MVO1")
            return data
        while True:
            position = source.tell()
            if not source.read(1):
                return
            source.seek(position)
            yield read_message(exact)


class RawOdometryWorker(Thread):
    def __init__(self, host="0.0.0.0", port=5006, token="", height=1., use_lidar=False,
                 record_directory=None, replay_path=None):
        super().__init__(name="pc-odometry", daemon=True)
        self.host, self.port, self.token = host, port, token
        self.height, self.use_lidar = height, use_lidar
        self.record_directory, self.replay_path = record_directory, replay_path
        self.cancelled = Event()
        self.lock = Lock()
        self.results = deque(maxlen=2048)
        self.listener = self.client = None
        self.processor = CaptureProcessor(height, use_lidar)
        self.update = dict(finished=False, error=None, status="Подготовка…", address=None,
                           result=None, image=None, frames=0, messages=0, bytes=0, recording=None,
                           encoding=None)

    def publish(self, **values):
        with self.lock:
            self.update.update(values)
            if "result" in values:
                self.results.append(values["result"])

    def snapshot(self):
        with self.lock:
            result = dict(self.update, results=list(self.results))
            self.results.clear()
            return result

    def stop(self):
        self.cancelled.set()
        for sock in (self.client, self.listener):
            if sock:
                try:
                    sock.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                sock.close()

    def run(self):
        recorder = None
        error = None
        try:
            if self.replay_path:
                messages = recorded_messages(self.replay_path)
                self.publish(status="Повторный расчёт исходной сессии…")
            else:
                self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.listener.settimeout(.2)
                self.listener.bind((self.host, self.port))
                self.listener.listen(1)
                self.publish(address=self.listener.getsockname(), status="Ожидание Android · TCP")
                while not self.cancelled.is_set():
                    try:
                        self.client, _ = self.listener.accept()
                        self.client.settimeout(.2)
                        reader = SocketReader(self.client, self.cancelled)
                        hello, payload = read_message(reader.read)
                        if hello["type"] != "hello" or not secrets.compare_digest(str(hello.get("token", "")), self.token):
                            raise ValueError("Неверный код подключения")
                        self.client.sendall(b"\x01")
                        break
                    except (ValueError, EOFError, OSError, UnicodeError, RecursionError) as exc:
                        if self.cancelled.is_set():
                            break
                        if self.client is None:
                            if not isinstance(exc, TimeoutError):
                                raise
                            continue
                        self.publish(status=f"Ожидание Android: {exc}")
                        if self.client:
                            self.client.close()
                            self.client = None
                if self.cancelled.is_set():
                    return
                def stream():
                    yield hello, payload
                    while not self.cancelled.is_set():
                        yield read_message(reader.read)
                messages = stream()
                if self.record_directory:
                    recorder = RawRecorder(self.record_directory, self.height, self.use_lidar, hello["protocol"])
                    self.publish(recording=str(recorder.directory))
                encoding = "H.264" if hello["protocol"] == 2 else "RAW GRAY8"
                self.publish(status=f"Исходные данные поступают · {encoding} · расчёт на ПК", encoding=encoding)
            first = True
            count = frames = total_bytes = 0
            for header, payload in messages:
                if self.cancelled.is_set():
                    break
                if first != (header["type"] == "hello"):
                    raise ValueError("Session must start with exactly one handshake")
                if first:
                    encoding = "H.264" if header["protocol"] == 2 else "RAW GRAY8"
                    self.publish(encoding=encoding)
                first = False
                if recorder:
                    recorder.message(header, payload)
                count += 1
                total_bytes += len(payload)
                for result, image in self.processor.ingest(header, payload):
                    frames += 1
                    if recorder:
                        recorder.result(result)
                    self.publish(result=result, image=image, frames=frames)
                self.publish(messages=count, bytes=total_bytes)
            if first:
                raise ValueError("Пустая исходная сессия")
        except Exception as exc:
            if not self.cancelled.is_set():
                error = f"{type(exc).__name__}: {exc}"
                LOGGER.exception("PC odometry stopped")
                self.publish(error=error)
        finally:
            for sock in (self.client, self.listener):
                if sock:
                    sock.close()
            if recorder:
                try:
                    recorder.close(error)
                except Exception as exc:
                    LOGGER.exception("Raw recording finalization failed")
                    error = f"Не удалось завершить запись: {exc}"
                    self.publish(error=error)
            self.publish(finished=True, status="Расчёт завершён" if self.replay_path and not error else "Приём остановлен")
