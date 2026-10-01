import base64
from contextlib import closing
import json
from pathlib import Path
import socket
import tempfile
import time
import unittest

from pc_app.motion_viewer.protocol import PacketError, parse_packet
from pc_app.motion_viewer.receiver import TelemetryReceiver
from pc_app.motion_viewer.recording import RecordingError, SessionRecorder
from pc_app.motion_viewer.statistics import SequenceWindow, StreamStats


FIXTURE = (Path(__file__).resolve().parents[2] / "app/src/test/resources/telemetry_v2.json").read_bytes().strip()


def packet_bytes(sequence=125, session="session-a", **overrides):
    obj = json.loads(FIXTURE)
    obj.update(sequence=sequence, session_id=session, **overrides)
    return json.dumps(obj).encode()


class ProtocolTests(unittest.TestCase):
    def test_shared_android_serialization_fixture(self):
        packet = parse_packet(FIXTURE)
        self.assertEqual(2, packet.protocol_version)
        self.assertEqual(845126735381, packet.device_timestamp_ns)
        self.assertEqual(845126729000, packet.sensor_timestamps_ns["gyroscope"])
        self.assertEqual(2, packet.planar_odometry.epoch)
        self.assertEqual(-0.18, packet.planar_odometry.y_m)
        self.assertLessEqual(len(FIXTURE), 1400)

    def test_legacy_android_without_camera_remains_supported(self):
        obj = json.loads(FIXTURE)
        for key in ("protocol_version", "packet_type", "session_id", "device_timestamp_ns",
                    "sensor_timestamps_ns", "planar_odometry", "height_above_surface_m"):
            del obj[key]
        packet = parse_packet(json.dumps(obj).encode())
        self.assertEqual(1, packet.protocol_version)
        self.assertIsNone(packet.planar_odometry)
        self.assertIsNone(packet.sensor_timestamps_ns)

    def test_null_camera_and_unknown_extension(self):
        packet = parse_packet(packet_bytes(planar_odometry=None, future_extension={"enabled": True}))
        self.assertIsNone(packet.planar_odometry)
        self.assertIn("future_extension", packet.payload)

    def test_invalid_packets_are_rejected(self):
        bad = [
            b"", b"\xff", b"not json", b"[]", b'{"x":1,"x":2}',
            b"{" + b" " * 65507 + b"}", b"[" * 1500,
            packet_bytes(sequence=True), packet_bytes(sequence=-1), packet_bytes(sequence=2**63),
            packet_bytes(protocol_version=9), packet_bytes(packet_type="command"),
            packet_bytes(session="bad\nname"), packet_bytes(timestamp=float("nan")),
            packet_bytes(gyroscope={"x": "0", "y": 0, "z": 0}),
            packet_bytes(quaternion={"w": 0, "x": 0, "y": 0, "z": 0}),
            packet_bytes(future_extension=float("inf")),
            FIXTURE.replace(b"1723904521.256", b"1e309"),
        ]
        for data in bad:
            with self.subTest(data=data[:80]):
                with self.assertRaises(PacketError):
                    parse_packet(data)

    def test_invalid_camera_quality_is_rejected(self):
        for field, value in [("status", "UNKNOWN"), ("inliers", 500), ("inlier_ratio", 2),
                             ("camera_active", 1), ("surface_distance_m", 0), ("epoch", -1)]:
            obj = json.loads(FIXTURE)
            obj["planar_odometry"][field] = value
            with self.subTest(field=field), self.assertRaises(PacketError):
                parse_packet(json.dumps(obj).encode())


class SequenceTests(unittest.TestCase):
    def test_reordering_fills_pending_gap_and_duplicates_do_not_count_as_loss(self):
        window = SequenceWindow(window_size=4)
        self.assertEqual("new", window.observe(10))
        window.observe(12)
        self.assertEqual(1, window.pending_gaps)
        self.assertEqual("reordered", window.observe(11))
        self.assertEqual(0, window.pending_gaps)
        self.assertEqual("duplicate", window.observe(12))
        self.assertEqual(0, window.missing_finalized)
        self.assertEqual(1, window.duplicates)

    def test_gap_becomes_finalized_only_outside_reordering_window(self):
        window = SequenceWindow(window_size=4)
        for seq in [100, 102, 103, 104]:
            window.observe(seq)
        self.assertEqual(0, window.missing_finalized)
        window.observe(105)
        self.assertEqual(1, window.missing_finalized)
        self.assertEqual("too_late", window.observe(101))
        self.assertEqual(1, window.missing_finalized)

    def test_huge_sequence_jump_keeps_memory_bounded(self):
        window = SequenceWindow()
        window.observe(0)
        window.observe(2**62)
        self.assertLessEqual(len(window.seen), 256)
        self.assertEqual(255, window.pending_gaps)
        self.assertEqual(2**62 - 256, window.missing_finalized)

    def test_receive_timeout_and_rate_use_only_pc_monotonic_time(self):
        stats = StreamStats()
        for i in range(51):
            stats.observe(parse_packet(packet_bytes(sequence=i)), 1_000_000_000 + i * 20_000_000)
        self.assertEqual(50, stats.snapshot(2_000_000_000)["packets_per_second"])
        self.assertFalse(stats.snapshot(5_100_000_000)["receiving"])
        self.assertEqual(0, stats.snapshot(5_100_000_000)["packets_per_second"])


class ReceiverTests(unittest.TestCase):
    def test_bad_packet_does_not_stop_next_valid_packet(self):
        with closing(TelemetryReceiver("127.0.0.1", 0)) as receiver:
            receiver.process(b"bad", ("127.0.0.1", 1234), 1, 1)
            receiver.process(FIXTURE, ("127.0.0.1", 1234), 2, 2)
            self.assertEqual(1, receiver.invalid)
            self.assertEqual(1, len(receiver.streams))

    def test_stream_restart_and_multiple_phones_have_separate_statistics(self):
        with closing(TelemetryReceiver("127.0.0.1", 0)) as receiver:
            for session, ip, seq in [("a", "10.0.0.1", 800), ("b", "10.0.0.1", 0),
                                     ("b", "10.0.0.2", 0), ("a", "10.0.0.1", 799)]:
                receiver.process(packet_bytes(seq, session), (ip, 4321), 1, 1)
            self.assertEqual(3, len(receiver.streams))
            self.assertTrue(all(s.sequences.missing_finalized == 0 for s in receiver.streams.values()))
            self.assertEqual(800, receiver.streams[("10.0.0.1", 4321, "a")].latest.sequence)

    def test_repeated_camera_frame_is_not_integrated_and_epoch_reset_is_preserved(self):
        with closing(TelemetryReceiver("127.0.0.1", 0)) as receiver:
            for seq in range(5):
                receiver.process(packet_bytes(seq), ("127.0.0.1", 4321), seq, seq)
            stats = next(iter(receiver.streams.values()))
            self.assertEqual(0.42, stats.latest.planar_odometry.x_m)
            obj = json.loads(packet_bytes(5))
            obj["planar_odometry"].update(epoch=3, timestamp_ns=0, status="INITIALIZING", x_m=0, y_m=0)
            receiver.process(json.dumps(obj).encode(), ("127.0.0.1", 4321), 6, 6)
            self.assertEqual(3, stats.latest.planar_odometry.epoch)
            self.assertEqual(0, stats.latest.planar_odometry.x_m)

    def test_udp_loopback_recording_preserves_exact_datagrams_even_invalid_ones(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "session.jsonl"
            recorder = SessionRecorder(path)
            try:
                with closing(TelemetryReceiver("127.0.0.1", 0, recorder)) as receiver:
                    datagrams = [packet_bytes(0), packet_bytes(2), packet_bytes(1),
                                 packet_bytes(2), b"\xffbad json", packet_bytes(0, "session-b")]
                    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                        for data in datagrams:
                            sender.sendto(data, receiver.address)
                            deadline = time.monotonic() + 2
                            event = None
                            while event is None and time.monotonic() < deadline:
                                event = receiver.receive_once()
                            self.assertIsNotNone(event, "Local UDP packet not received")
                    stats = next(iter(receiver.streams.values()))
                    self.assertEqual(1, stats.sequences.reordered)
                    self.assertEqual(1, stats.sequences.duplicates)
                    self.assertEqual(0, stats.sequences.pending_gaps)
                    self.assertEqual(1, receiver.invalid)
                    self.assertEqual(2, len(receiver.streams))
            finally:
                recorder.close()
            records = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
            self.assertEqual(datagrams, [base64.b64decode(r["raw_base64"]) for r in records])
            self.assertIsNone(records[4]["payload"])
            self.assertIsNotNone(records[4]["error"])
            with self.assertRaises(FileExistsError):
                SessionRecorder(path)
            with self.assertRaises(RecordingError):
                recorder.submit({})

    def test_stream_cache_is_bounded(self):
        with closing(TelemetryReceiver("127.0.0.1", 0)) as receiver:
            for i in range(20):
                receiver.process(packet_bytes(0, f"session-{i}"), ("127.0.0.1", 1234), i, i)
            self.assertEqual(16, len(receiver.streams))
            self.assertEqual(4, receiver.evicted_streams)


if __name__ == "__main__":
    unittest.main()
