import json
from pathlib import Path
import socket
import tempfile
import time
import unittest

from pc_app.motion_viewer.worker import ReceiverWorker
from pc_app.tests.test_transport import packet_bytes


def wait_update(worker, predicate, timeout=4):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        update = worker.take_update()
        if predicate(update):
            return update
        time.sleep(0.01)
    raise AssertionError("Worker condition timed out")


class WorkerTests(unittest.TestCase):
    def test_stop_flushes_recording_and_releases_port(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "worker.jsonl"
            worker = ReceiverWorker("127.0.0.1", 0, path)
            worker.start()
            try:
                address = wait_update(worker, lambda update: update["address"])["address"]
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                    for sequence in range(10):
                        sender.sendto(packet_bytes(sequence), address)
                wait_update(worker, lambda update: update["snapshot"]["datagrams"] == 10)
            finally:
                worker.stop()
                worker.join(5)
            self.assertFalse(worker.is_alive())
            update = worker.take_update()
            self.assertTrue(update["finished"])
            self.assertIsNone(update["error"])
            self.assertEqual(10, update["written"])
            self.assertEqual(10, len(path.read_text().splitlines()))
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
                probe.bind(address)

    def test_slow_ui_drops_only_display_events_not_recorded_datagrams(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "slow-ui.jsonl"
            worker = ReceiverWorker("127.0.0.1", 0, path, mailbox_capacity=3)
            worker.start()
            try:
                address = wait_update(worker, lambda update: update["address"])["address"]
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                    for sequence in range(12):
                        sender.sendto(packet_bytes(sequence), address)
                # Deliberately don't consume the mailbox while the receiver continues.
                deadline = time.monotonic() + 4
                while worker._written < 12 and time.monotonic() < deadline:
                    time.sleep(0.01)
            finally:
                worker.stop()
                worker.join(5)
            update = worker.take_update()
            self.assertIsNone(update["error"])
            self.assertEqual(9, update["display_dropped"])
            self.assertEqual(3, len(update["events"]))
            self.assertEqual(12, update["written"])
            records = [json.loads(line) for line in path.read_text().splitlines()]
            self.assertEqual(list(range(12)), [record["payload"]["sequence"] for record in records])

    def test_port_collision_is_reported_without_hanging(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as occupied:
            occupied.bind(("127.0.0.1", 0))
            worker = ReceiverWorker(*occupied.getsockname())
            worker.start()
            worker.join(3)
            self.assertFalse(worker.is_alive())
            self.assertTrue(worker.take_update()["error"])

    def test_existing_recording_is_not_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "existing.jsonl"
            path.write_text("keep me")
            worker = ReceiverWorker("127.0.0.1", 0, path)
            worker.start()
            worker.join(3)
            self.assertFalse(worker.is_alive())
            self.assertIn("FileExistsError", worker.take_update()["error"])
            self.assertEqual("keep me", path.read_text())
