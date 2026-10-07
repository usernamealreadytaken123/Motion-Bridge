"""Per-user settings and rotating logs, never written alongside the executable."""

from datetime import datetime
import ipaddress
import json
import logging
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
import shutil
import sys
import tempfile

LOGGER = logging.getLogger("motion_viewer")
LOGGER.addHandler(logging.NullHandler())


def default_data_directory() -> Path:
    if os.name == "nt":
        return Path(os.environ.get("LOCALAPPDATA", str(Path.home() / "AppData/Local"))) / "MotionViewer"
    return Path(os.environ.get("XDG_STATE_HOME", str(Path.home() / ".local/state"))) / "MotionViewer"


class Preferences:
    def __init__(self, directory: Path):
        self.directory = Path(directory).resolve()
        self.path = self.directory / "settings.json"
        self.warning = None

    def load(self) -> dict:
        if not self.path.exists():
            return {}
        try:
            if self.path.stat().st_size > 65536:
                raise ValueError("Settings file is too large")
            value = json.loads(self.path.read_text(encoding="utf-8"))
            if not isinstance(value, dict) or value.get("version") != 1:
                raise ValueError("Unsupported settings version")
            result = {}
            host = value.get("host")
            if isinstance(host, str):
                try:
                    ipaddress.IPv4Address(host)
                    result["host"] = host
                except ValueError:
                    pass
            for key, minimum, maximum in (("port", 1, 65535), ("width", 1100, 3840), ("height", 760, 2160)):
                item = value.get(key)
                if type(item) is int and minimum <= item <= maximum:
                    result[key] = item
            if type(value.get("record")) is bool:
                result["record"] = value["record"]
            if value.get("speed") in (0.25, 0.5, 1.0, 2.0, 4.0, 8.0) and type(value["speed"]) in (int, float):
                result["speed"] = value["speed"]
            for key in ("record_directory", "replay_directory"):
                item = value.get(key)
                if isinstance(item, str) and "\x00" not in item and Path(item).is_absolute():
                    result[key] = item
            return result
        except (OSError, ValueError, RecursionError) as exc:
            self.warning = f"Не удалось прочитать настройки: {exc}. Используются значения по умолчанию."
            LOGGER.warning(self.warning)
            # Keep the damaged file available for diagnosis before the next atomic save.
            try:
                backup = self.path.with_name(datetime.now().strftime("settings-invalid-%Y%m%d-%H%M%S-%f.json"))
                shutil.copy2(self.path, backup)
            except OSError:
                LOGGER.exception("Could not back up settings")
            return {}

    def save(self, settings: dict) -> None:
        self.directory.mkdir(parents=True, exist_ok=True)
        name = None
        try:
            with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=self.directory,
                                             prefix="settings-", suffix=".tmp", delete=False) as file:
                name = file.name
                json.dump({**settings, "version": 1}, file, ensure_ascii=False, allow_nan=False, indent=2)
                file.flush()
                os.fsync(file.fileno())
            os.replace(name, self.path)
            name = None
        finally:
            if name is not None:
                Path(name).unlink(missing_ok=True)


def configure_logging(directory: Path):
    directory.mkdir(parents=True, exist_ok=True)
    handler = RotatingFileHandler(directory / "motion-viewer.log", maxBytes=2_000_000,
                                  backupCount=3, encoding="utf-8")
    handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(threadName)s %(message)s"))
    LOGGER.setLevel(logging.INFO)
    LOGGER.propagate = False
    for previous in LOGGER.handlers[:]:
        LOGGER.removeHandler(previous)
        previous.close()
    LOGGER.addHandler(handler)
    LOGGER.info("Motion Viewer started; frozen=%s; Python=%s", getattr(sys, "frozen", False), sys.version.split()[0])
    return handler
