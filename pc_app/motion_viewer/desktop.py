"""Windowed application entry point with startup error logging and packaging diagnostics."""

import argparse
import json
import os
from pathlib import Path
import sys

from .preferences import LOGGER, Preferences, configure_logging, default_data_directory


def main():
    parser = argparse.ArgumentParser(description="Motion Viewer")
    parser.add_argument("--open", type=Path, help="Open a telemetry JSONL recording")
    parser.add_argument("--data-dir", type=Path, help="Override per-user data directory")
    parser.add_argument("--smoke-test", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.smoke_test:
        os.environ["QT_QPA_PLATFORM"] = "offscreen"
    directory = args.data_dir or (args.smoke_test.parent / "smoke-data" if args.smoke_test else default_data_directory())
    try:
        configure_logging(directory / "logs")
        if args.smoke_test:
            from .smoke_test import run_smoke_test
            run_smoke_test(args.smoke_test.resolve(), directory.resolve())
            return 0
        from .gui import run_gui
        return run_gui(preferences=Preferences(directory), open_path=args.open)
    except Exception as exc:
        LOGGER.exception("Application startup or self-test failed")
        if args.smoke_test:
            args.smoke_test.parent.mkdir(parents=True, exist_ok=True)
            args.smoke_test.write_text(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False), encoding="utf-8")
        else:
            from PySide6.QtWidgets import QApplication, QMessageBox
            app = QApplication.instance() or QApplication([])
            QMessageBox.critical(None, "Motion Viewer", f"Не удалось запустить приложение: {exc}\nЖурнал: {directory / 'logs'}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
