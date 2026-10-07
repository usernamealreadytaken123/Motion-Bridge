import json
from pathlib import Path
import tempfile
import unittest

from pc_app.motion_viewer.receiver import TelemetryState
from pc_app.motion_viewer.replay import PlaybackClock, RecordingIndex, ReplayEngine
from pc_app.tests.test_transport import packet_bytes


def write_session(path, count=10):
    state = TelemetryState()
    records = []
    for sequence in range(count):
        payload = json.loads(packet_bytes(sequence))
        payload["quaternion"] = ({"w": 1, "x": 0, "y": 0, "z": 0} if sequence == 0
                                 else {"w": .7071067811865476, "x": 0, "y": 0, "z": .7071067811865476})
        payload["sensor_timestamps_ns"] = {name: sequence + 1 for name in payload["sensor_timestamps_ns"]}
        payload["planar_odometry"].update(timestamp_ns=sequence+1, x_m=sequence/10)
        records.append(state.process(json.dumps(payload).encode(), ("127.0.0.1", 5432),
                                     1_000_000_000 + sequence * 100_000_000, 2_000_000_000))
    path.write_text("".join(json.dumps(record) + "\n" for record in records), encoding="utf-8")
    return records


class ReplayTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.path = Path(self.directory.name) / "session.jsonl"

    def tearDown(self):
        self.directory.cleanup()

    def test_forward_backward_seek_rebuilds_all_state_without_future_samples(self):
        write_session(self.path)
        engine = ReplayEngine(RecordingIndex.load(self.path))
        try:
            self.assertEqual(.9, engine.index.duration)
            self.assertTrue(engine.advance(.9, budget_seconds=1))
            self.assertEqual(10, engine.state.datagrams)
            self.assertTrue(engine.advance(.25, budget_seconds=1))
            self.assertEqual(3, engine.state.datagrams)
            history = engine.history.streams[engine.history.last_key]
            self.assertEqual(2, history.last_sequence)
            self.assertEqual(3, len(history.trajectory))
            self.assertAlmostEqual(.2, history.latest["planar_odometry"]["x_m"])
            self.assertTrue(engine.advance(0, budget_seconds=1))
            self.assertEqual(1, engine.state.datagrams)
            self.assertEqual(1, next(iter(engine.history.streams.values())).latest["quaternion"]["w"])
        finally:
            engine.close()

    def test_seek_and_linear_playback_produce_identical_packet_statistics(self):
        state = TelemetryState()
        records = [state.process(raw, ("127.0.0.1", 1000), 10**9 + i*10**8, 0)
                   for i, raw in enumerate([packet_bytes(0), packet_bytes(2), packet_bytes(1),
                                            packet_bytes(2), b"bad", packet_bytes(0, "new-session")])]
        self.path.write_text("".join(json.dumps(record) + "\n" for record in records))
        engine = ReplayEngine(RecordingIndex.load(self.path))
        try:
            while not engine.advance(.5, budget_seconds=0):
                self.assertLess(engine.cursor, len(records))
            direct = engine.state.snapshot(engine.now_ns)
            self.assertEqual(1, direct["invalid"])
            self.assertEqual(1, direct["streams"][0]["duplicates"])
            self.assertEqual(1, direct["streams"][0]["reordered"])
            self.assertEqual(2, len(direct["streams"]))
            engine.advance(0, budget_seconds=1)
            for step in range(6):
                engine.advance(step / 10, budget_seconds=1)
            self.assertEqual(direct, engine.state.snapshot(engine.now_ns))
        finally:
            engine.close()

    def test_paused_replay_uses_recorded_clock_not_current_pc_uptime(self):
        write_session(self.path)
        engine = ReplayEngine(RecordingIndex.load(self.path))
        try:
            engine.advance(.9, budget_seconds=1)
            self.assertTrue(engine.state.snapshot(engine.now_ns)["streams"][0]["receiving"])
            self.assertEqual("TRACKING", next(iter(engine.history.streams.values())).camera_state(engine.now_ns))
        finally:
            engine.close()

    def test_corrupted_or_invalid_recordings_are_rejected(self):
        records = write_session(self.path, 2)
        bad_cases = [b"", b'{"record_version":', b"[]\n"]
        for key, value in [("pc_receive_monotonic_ns", True), ("sender", ["bad", 1]),
                           ("record_version", 9), ("raw_base64", "!!!"), ("payload", {})]:
            record = dict(records[0])
            record[key] = value
            bad_cases.append(json.dumps(record).encode())
        backwards = [records[1], records[0]]
        bad_cases.append("\n".join(json.dumps(record) for record in backwards).encode())
        for content in bad_cases:
            with self.subTest(content=content[:50]):
                self.path.write_bytes(content)
                with self.assertRaises(ValueError):
                    RecordingIndex.load(self.path)

    def test_file_change_is_detected_and_loading_can_be_cancelled(self):
        write_session(self.path)
        self.assertIsNone(RecordingIndex.load(self.path, cancelled=lambda: True))
        engine = ReplayEngine(RecordingIndex.load(self.path))
        try:
            with self.path.open("ab") as source:
                source.write(b"\n")
            with self.assertRaisesRegex(ValueError, "изменён"):
                engine.advance(0)
        finally:
            engine.close()

    def test_all_packets_at_same_timestamp_are_included_at_start(self):
        records = write_session(self.path, 3)
        for record in records:
            record["pc_receive_monotonic_ns"] = 1000
        self.path.write_text("".join(json.dumps(record) + "\n" for record in records))
        engine = ReplayEngine(RecordingIndex.load(self.path))
        try:
            self.assertTrue(engine.advance(0, budget_seconds=1))
            self.assertEqual(3, engine.state.datagrams)
        finally:
            engine.close()


class PlaybackClockTests(unittest.TestCase):
    def test_pause_resume_speed_seek_and_end(self):
        clock = PlaybackClock(10)
        clock.set_playing(True, 100)
        self.assertEqual(1, clock.update(101))
        clock.set_playing(False, 101)
        self.assertEqual(1, clock.update(110))
        clock.set_speed(2, 110)
        clock.set_playing(True, 110)
        self.assertEqual(5, clock.update(112))
        clock.seek(2, 112)
        self.assertEqual(4, clock.update(113))
        self.assertEqual(10, clock.update(120))
        self.assertFalse(clock.playing)
