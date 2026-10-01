from contextlib import closing
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest

from pc_app.motion_viewer.receiver import TelemetryReceiver
from pc_app.motion_viewer.recording import SessionRecorder


PROJECT = Path(__file__).resolve().parents[2]


class CommandLineTests(unittest.TestCase):
    def test_separate_process_simulator_with_loss_and_duplicates_then_inspect_recording(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "loopback.jsonl"
            recorder = SessionRecorder(path)
            try:
                with closing(TelemetryReceiver("127.0.0.1", 0, recorder)) as receiver:
                    simulator = subprocess.Popen(
                        [sys.executable, "-m", "pc_app.motion_viewer", "simulate",
                         "--port", str(receiver.address[1]), "--duration", "1", "--rate", "50",
                         "--drop-every", "10", "--duplicate-every", "15"],
                        cwd=PROJECT, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                    )
                    try:
                        deadline = time.monotonic() + 5
                        while receiver.datagrams < 47 and time.monotonic() < deadline:
                            receiver.receive_once()
                        stdout, stderr = simulator.communicate(timeout=3)
                        self.assertEqual(0, simulator.returncode, stderr)
                        self.assertIn("SIMULATED data: sent 47", stdout)
                        self.assertEqual(47, receiver.datagrams)
                        self.assertEqual(0, receiver.invalid)
                        stats = next(iter(receiver.streams.values()))
                        self.assertEqual(2, stats.sequences.duplicates)
                        # Last dropped packet cannot be detected until a higher sequence arrives.
                        self.assertEqual(4, stats.sequences.pending_gaps)
                        self.assertEqual(48, stats.latest.sequence)
                    finally:
                        if simulator.poll() is None:
                            simulator.kill()
                            simulator.communicate()
            finally:
                recorder.close()
            result = subprocess.run(
                [sys.executable, "-m", "pc_app.motion_viewer", "inspect", str(path)],
                cwd=PROJECT, capture_output=True, text=True, timeout=5,
            )
            self.assertEqual(0, result.returncode, result.stderr)
            self.assertIn("Verified 47 records: valid=47, rejected=0, streams=1", result.stdout)

    def test_inspect_reports_truncated_record_instead_of_success(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "broken.jsonl"
            path.write_text('{"record_version":', encoding="utf-8")
            result = subprocess.run(
                [sys.executable, "-m", "pc_app.motion_viewer", "inspect", str(path)],
                cwd=PROJECT, capture_output=True, text=True, timeout=5,
            )
            self.assertNotEqual(0, result.returncode)
            self.assertIn("Recording line 1", result.stderr)
