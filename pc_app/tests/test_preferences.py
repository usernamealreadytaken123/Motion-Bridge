import json
import logging
from pathlib import Path
import tempfile
import unittest

from pc_app.motion_viewer.preferences import LOGGER, Preferences, configure_logging


class PreferencesTests(unittest.TestCase):
    def test_save_reload_validated_settings_and_atomic_cleanup(self):
        with tempfile.TemporaryDirectory() as directory:
            store = Preferences(Path(directory))
            self.assertEqual({}, store.load())
            store.save({"host": "127.0.0.1", "port": 19500, "record": False, "speed": 2.0,
                        "width": 1400, "height": 900, "record_directory": directory})
            loaded = store.load()
            self.assertEqual(19500, loaded["port"])
            self.assertEqual(2.0, loaded["speed"])
            self.assertEqual([], list(Path(directory).glob("*.tmp")))
            store.save({"host": "invalid", "port": True, "width": -1, "speed": 100, "record": "yes"})
            self.assertEqual({}, store.load())

    def test_damaged_settings_are_preserved_and_do_not_block_startup(self):
        with tempfile.TemporaryDirectory() as directory:
            store = Preferences(Path(directory))
            store.path.write_text("{bad")
            self.assertEqual({}, store.load())
            self.assertTrue(store.warning)
            backup = list(Path(directory).glob("settings-invalid-*.json"))
            self.assertEqual(1, len(backup))
            self.assertEqual("{bad", backup[0].read_text())

    def test_rotating_log_records_errors_without_telemetry(self):
        with tempfile.TemporaryDirectory() as directory:
            handler = configure_logging(Path(directory))
            try:
                LOGGER.error("test diagnostic")
                handler.flush()
                text = (Path(directory) / "motion-viewer.log").read_text(encoding="utf-8")
                self.assertIn("test diagnostic", text)
                self.assertEqual(3, handler.backupCount)
            finally:
                LOGGER.removeHandler(handler)
                handler.close()
                LOGGER.addHandler(logging.NullHandler())
