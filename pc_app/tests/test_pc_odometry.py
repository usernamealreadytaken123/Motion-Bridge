import io
import json
import math
from pathlib import Path
import socket
import tempfile
import time
import unittest
import importlib.util
if not all(importlib.util.find_spec(name) for name in ("cv2", "numpy")):
    raise unittest.SkipTest("Install pc_app/requirements-gui.txt for PC visual odometry tests")
import cv2
import numpy as np
import av
from fractions import Fraction

from pc_app.motion_viewer.odometry_math import Orientations, LidarHeights, camera_basis, ground_motion, rotation
from pc_app.motion_viewer.pc_odometry import PlanarEngine, CaptureProcessor, upright
from pc_app.motion_viewer.raw_protocol import encode, read_message, PREFIX
from pc_app.motion_viewer.raw_worker import RawOdometryWorker, recorded_messages
from pc_app.motion_viewer.video_decoder import H264Decoder


def header(seq=0, stamp=1_000_000_000, **extra):
    return dict(type="frame", seq=seq, timestamp_ns=stamp, width=640, height=480, rotation=0,
                sensor_orientation=0, realtime=True, back_camera=True, approximate=False,
                intrinsics=[500., 500., 319.5, 239.5], dropped=0, **extra)


def quaternion(pitch=0., roll=0., yaw=0.):
    def axis(index, degrees):
        q = np.zeros(4); q[0] = math.cos(math.radians(degrees)/2); q[index+1] = math.sin(math.radians(degrees)/2)
        return q
    def mul(a, b):
        return np.r_[a[0]*b[0]-a[1:]@b[1:], a[0]*b[1:]+b[0]*a[1:]+np.cross(a[1:], b[1:])]
    return mul(mul(axis(2, yaw), axis(0, pitch)), axis(1, roll))


def texture():
    noise = np.random.default_rng(123).integers(0, 256, (480, 640), dtype=np.uint8)
    return cv2.GaussianBlur(noise, (3, 3), .7)


def sensor_message(stamp, start=None):
    start = stamp-200_000_000 if start is None else start
    return dict(type="sensors", watermark_ns=stamp,
                samples=[["q", t, 1., 0., 0., 0.] for t in range(start, stamp+1, 20_000_000)])


class MathTests(unittest.TestCase):
    def test_shared_android_ground_motion_cases(self):
        fixture = Path(__file__).resolve().parents[2] / "app/src/test/resources/ground_motion_cases.json"
        for case in json.loads(fixture.read_text()):
            with self.subTest(case=case):
                old_q, q = quaternion(*case["old_angles"]), quaternion(*case["angles"])
                b0, b1 = camera_basis(old_q, 90, 90), camera_basis(q, 90, 90)
                h0, h = case["old_height"], case["height"]
                centre = -h0*b0[:2, 2]/b0[2, 2]
                points = np.array([[centre[0]+x*h0*.08, centre[1]+y*h0*.08, 0]
                                   for x in range(-3, 4) for y in range(-3, 4)])
                def observe(camera, basis):
                    v = (points-np.array(camera)) @ basis
                    return v[:, :2]/v[:, 2:3]*[800, 780]+[319.5, 239.5]
                a, b = observe([0, 0, h0], b0), observe([case["dx"], case["dy"], h], b1)
                estimate = ground_motion(a, b, [800, 780, 319.5, 239.5], b0, b1, h0, h)
                self.assertIsNotNone(estimate)
                np.testing.assert_allclose(estimate[0], [case["dx"], case["dy"]], atol=1e-9)

    def test_orientation_interpolation_never_extrapolates_or_crosses_gap(self):
        history = Orientations()
        history.add(100, [1, 0, 0, 0]); history.add(20_000_100, -quaternion(yaw=20))
        np.testing.assert_allclose(rotation(history.at(10_000_100)), rotation(quaternion(yaw=10)), atol=1e-9)
        self.assertIsNone(history.at(99)); self.assertIsNone(history.at(20_000_101))
        history.add(200_000_000, [1, 0, 0, 0])
        self.assertIsNone(history.at(100_000_000))

    def test_lidar_time_mapping_duplicates_pause_and_height_interpolation(self):
        lidar, orientations = LidarHeights(), Orientations()
        base = 5_000_000_000
        for t in range(base, base+400_000_001, 20_000_000):
            orientations.add(t, [1, 0, 0, 0])
        for i in range(3):
            sample = dict(ok=True, measuring=True, session="a", time_s=i*.1, depth_m=1+i*.02,
                          requested_ns=base+i*100_000_000, received_ns=base+i*100_000_000+20_000_000, mount=[0, 0, 0])
            lidar.add(sample)
        self.assertAlmostEqual(1.03, lidar.at(base+160_000_000, base+230_000_000, orientations)[0])
        lidar.add(dict(sample, requested_ns=base+700_000_000, received_ns=base+720_000_000))
        self.assertIsNone(lidar.at(base+160_000_000, base+730_000_000, orientations))
        lidar.add(dict(sample, measuring=False))
        self.assertIsNone(lidar.at(base+160_000_000, base+230_000_000, orientations))


class EngineTests(unittest.TestCase):
    def test_tilt_translation_rotation_and_changing_height_together(self):
        image = texture()
        old, new = quaternion(25, 2, 30), quaternion(27, 4, 31)
        b0, b1 = camera_basis(old, 0, 0), camera_basis(new, 0, 0)
        k = np.array([[500., 0, 319.5], [0, 500., 239.5], [0, 0, 1.]])
        displacement = np.array([.025, -.015, .02])
        transform = k @ b1.T @ (np.eye(3) + np.outer(displacement, [0, 0, 1])/1.2) @ b0 @ np.linalg.inv(k)
        moved = cv2.warpPerspective(image, transform, (640, 480))
        engine = PlanarEngine(1.2)
        engine.process(header(), image.tobytes(), old, 1.2)
        result, _ = engine.process(header(1, 1_100_000_000), moved.tobytes(), new, 1.22)
        self.assertEqual("TRACKING", result["status"], result)
        rx, ry = b0[:2, 0]/np.linalg.norm(b0[:2, 0])
        expected = np.array([[rx, ry], [ry, -rx]]) @ displacement[:2]
        np.testing.assert_allclose([result["x_m"], result["y_m"]], expected, atol=.001)

    def test_invalid_clock_and_missing_lidar_remain_json_serializable(self):
        meta = header(); meta["realtime"] = False
        result, _ = PlanarEngine().process(meta, texture().tobytes(), None, float("nan"))
        self.assertEqual("DEGRADED", result["status"])
        json.dumps(result, allow_nan=False)

    def test_real_optical_flow_recovers_translation_from_pixels(self):
        image = texture()
        engine = PlanarEngine(1.)
        q = quaternion()
        result, _ = engine.process(header(), image.tobytes(), q)
        self.assertEqual("INITIALIZING", result["status"])
        for i in range(1, 6):
            moved = cv2.warpAffine(image, np.float32([[1, 0, -4*i], [0, 1, 2*i]]), (640, 480))
            result, _ = engine.process(header(i, 1_000_000_000+i*100_000_000), moved.tobytes(), q)
            self.assertEqual("TRACKING", result["status"], result)
        self.assertAlmostEqual(.04, result["x_m"], delta=.001)
        self.assertAlmostEqual(-.02, result["y_m"], delta=.001)

    def test_perspective_rotation_is_not_reported_as_translation(self):
        image = texture()
        engine = PlanarEngine(1.)
        old, new = quaternion(), quaternion(pitch=3., roll=2., yaw=1.)
        k = np.array([[500., 0, 319.5], [0, 500., 239.5], [0, 0, 1.]])
        b0, b1 = camera_basis(old, 0, 0), camera_basis(new, 0, 0)
        transform = k @ b1.T @ b0 @ np.linalg.inv(k)
        moved = cv2.warpPerspective(image, transform, (640, 480))
        engine.process(header(), image.tobytes(), old)
        result, _ = engine.process(header(1, 1_100_000_000), moved.tobytes(), new)
        self.assertEqual("TRACKING", result["status"], result)
        self.assertLess(math.hypot(result["x_m"], result["y_m"]), .002)

    def test_missing_orientation_unknown_clock_blank_frame_and_tilt_fail_closed(self):
        for q, realtime, image in ((None, True, texture()), (quaternion(), False, texture()),
                                    (quaternion(pitch=70), True, texture()), (quaternion(), True, np.zeros((480, 640), np.uint8))):
            engine = PlanarEngine()
            meta = header(); meta["realtime"] = realtime
            result, _ = engine.process(meta, image.tobytes(), q)
            self.assertEqual("DEGRADED", result["status"])
            self.assertEqual(0., result["x_m"])

    def test_frame_rotation_updates_intrinsics_consistently(self):
        meta = header(); meta["rotation"] = 90
        gray, k = upright(meta, texture().tobytes())
        self.assertEqual((640, 480), gray.shape)
        self.assertEqual((500, 500, 239.5, 319.5), k)


class ProtocolTests(unittest.TestCase):
    def test_h264_protocol_and_decoder_preserve_source_frame_metadata(self):
        encoder = av.CodecContext.create("libx264", "w")
        encoder.width = encoder.height = 64
        encoder.pix_fmt = "yuv420p"
        encoder.time_base = Fraction(1, 1_000_000_000)
        encoder.framerate = 15
        encoder.bit_rate = 250_000
        encoder.options = {"preset": "ultrafast", "tune": "zerolatency", "bf": "0", "g": "15"}
        encoder.open()
        packets = []
        for i in range(3):
            frame = av.VideoFrame.from_ndarray(np.full((64, 64), 30+i*40, np.uint8), format="gray")
            frame.pts = 1_000_000_000+i*66_666_667
            packets.extend(encoder.encode(frame))
        packets.extend(encoder.encode(None))
        first = bytes(packets[0])
        starts = [i for i in range(len(first)-3) if first[i:i+4] == b"\x00\x00\x00\x01"]
        starts.append(len(first))
        config = b"".join(first[starts[i]:starts[i+1]] for i in range(len(starts)-1)
                          if first[starts[i]+4] & 0x1f in (7, 8))
        config_header = dict(type="video_config", codec="h264", width=64, height=64,
                             fps=15, bitrate=250_000)
        decoded_config, decoded_payload = read_message(io.BytesIO(encode(config_header, config)).read)
        decoder = H264Decoder()
        decoder.configure(decoded_config, decoded_payload)
        output = []
        for i, packet in enumerate(packets):
            stamp = 1_000_000_000+i*66_666_667
            meta = header(i, stamp)
            meta.update(type="video_packet", width=64, height=64, intrinsics=[55., 55., 31.5, 31.5],
                        pts_us=stamp//1000, keyframe=bool(packet.is_keyframe))
            checked, payload = read_message(io.BytesIO(encode(meta, bytes(packet))).read)
            output.extend(decoder.decode(checked, payload))
        self.assertEqual([0, 1, 2], [item[0]["seq"] for item in output])
        self.assertEqual([(64, 64)]*3, [(item[0]["width"], item[0]["height"]) for item in output])
        self.assertTrue(all(len(item[1]) == 64*64 for item in output))

    def test_lidar_missing_or_stale_never_falls_back_to_manual_height(self):
        processor = CaptureProcessor(2., True)
        processor.ingest(header(), texture().tobytes())
        result = processor.ingest(sensor_message(1_300_000_000, 900_000_000), b"")[0][0]
        self.assertEqual("DEGRADED", result["status"])
        self.assertFalse(result["height_valid"])
        self.assertIsNone(result["height_m"])

    def test_queue_bounded_first_drops_counted_and_backwards_time_rejected(self):
        processor = CaptureProcessor()
        for i in range(12):
            processor.ingest(header(i+3, 1_000_000_000+i*100_000_000), texture().tobytes())
        self.assertEqual(8, len(processor.pending))
        self.assertEqual(7, processor.dropped)
        with self.assertRaises(ValueError):
            processor.ingest(header(), texture().tobytes())
        processor.ingest(sensor_message(2_300_000_000), b"")
        with self.assertRaises(ValueError):
            processor.ingest(sensor_message(2_000_000_000), b"")

    def test_roundtrip_binary_pixels_and_rejects_oversized_or_malformed_messages(self):
        raw = texture().tobytes()
        data = io.BytesIO(encode(header(), raw))
        meta, pixels = read_message(data.read)
        self.assertEqual(header(), meta); self.assertEqual(raw, pixels)
        for blob in (PREFIX.pack(b"BAD!", 1, 0), PREFIX.pack(b"MVO1", 100000, 0),
                     PREFIX.pack(b"MVO1", 1, 999999999)):
            with self.assertRaises(ValueError):
                read_message(io.BytesIO(blob).read)
        invalid = header(); invalid["realtime"] = "true"
        with self.assertRaises(ValueError):
            encode(invalid, raw)
        with self.assertRaises(ValueError):
            encode(header(), b"bad")

    def test_processor_waits_for_source_time_not_pc_arrival_time(self):
        processor = CaptureProcessor()
        self.assertEqual([], processor.ingest(header(), texture().tobytes()))
        result = processor.ingest(sensor_message(1_140_000_000, 900_000_000), b"")
        self.assertEqual("INITIALIZING", result[0][0]["status"])
        self.assertEqual(1_000_000_000, result[0][0]["timestamp_ns"])

    def test_truncated_recording_is_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)/"bad.mvo"
            path.write_bytes(encode(header(), texture().tobytes())[:-1])
            with self.assertRaises(ValueError):
                list(recorded_messages(path))


class WorkerTests(unittest.TestCase):
    def test_stop_before_connection_and_during_partial_handshake(self):
        for partial in (False, True):
            worker = RawOdometryWorker("127.0.0.1", 0, "1234abcd")
            worker.start()
            self.await_condition(lambda: worker.update["address"] is not None)
            sender = socket.create_connection(worker.update["address"]) if partial else None
            try:
                if sender:
                    sender.sendall(b"MVO1")
                worker.stop(); worker.join(2)
                self.assertFalse(worker.is_alive())
                self.assertIsNone(worker.update["error"])
            finally:
                if sender:
                    sender.close()

    def await_condition(self, predicate):
        deadline = time.monotonic()+5
        while not predicate() and time.monotonic() < deadline:
            time.sleep(.01)
        self.assertTrue(predicate())

    def test_tcp_handshake_pixels_sensors_record_and_recompute_same_results(self):
        with tempfile.TemporaryDirectory() as directory:
            worker = RawOdometryWorker("127.0.0.1", 0, "1234abcd", record_directory=directory)
            worker.start()
            self.await_condition(lambda: worker.update["address"] is not None)
            with socket.create_connection(worker.update["address"]) as sender:
                sender.sendall(encode(dict(type="hello", protocol=1, session="test", token="wrong")))
            with socket.create_connection(worker.update["address"]) as sender:
                sender.sendall(encode(dict(type="hello", protocol=1, session="test", token="1234abcd")))
                self.assertEqual(b"\x01", sender.recv(1))
                image = texture()
                for i in range(5):
                    stamp = 1_000_000_000+i*100_000_000
                    moved = cv2.warpAffine(image, np.float32([[1, 0, -4*i], [0, 1, 0]]), (640, 480))
                    sender.sendall(encode(header(i, stamp), moved.tobytes()))
                    sender.sendall(encode(sensor_message(stamp+140_000_000)))
                self.await_condition(lambda: worker.update["frames"] == 5)
                result = worker.update["result"]
                self.assertAlmostEqual(.032, result["x_m"], delta=.001)
                worker.stop()
            worker.join(5)
            self.assertFalse(worker.is_alive())
            self.assertIsNone(worker.update["error"])
            path = Path(worker.update["recording"])/"capture.mvo"
            messages = list(recorded_messages(path))
            self.assertNotIn("token", messages[0][0])
            self.assertEqual(5, sum(h["type"] == "frame" for h, _ in messages))
            replay = RawOdometryWorker(replay_path=path)
            replay.start(); replay.join(5)
            self.assertFalse(replay.is_alive())
            self.assertIsNone(replay.update["error"])
            self.assertEqual(5, replay.update["frames"])
            self.assertEqual(result, replay.update["result"])
