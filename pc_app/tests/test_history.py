import json
import math
import unittest

from pc_app.motion_viewer.history import DisplayHistory, StreamHistory
from pc_app.tests.test_transport import packet_bytes


def display_event(sequence=0, session="session-a", received_ns=1_000_000_000,
                  frame_stamp=100, epoch=2, status="TRACKING", **pose):
    payload = json.loads(packet_bytes(sequence, session))
    payload["planar_odometry"].update(timestamp_ns=frame_stamp, epoch=epoch, status=status, **pose)
    return {"sender": ["127.0.0.1", 9000], "payload": payload,
            "pc_receive_monotonic_ns": received_ns, "sequence_result": "new"}


class HistoryTests(unittest.TestCase):
    def test_repeated_frames_and_sensor_timestamps_are_not_double_counted(self):
        model = DisplayHistory()
        for sequence in range(20):
            model.observe(display_event(sequence=sequence))
        history = model.streams[model.last_key]
        self.assertEqual([(0.42, -0.18)], list(history.trajectory))
        self.assertTrue(all(len(series) == 1 for series in history.sensors.values()))
        self.assertEqual("TRACKING", history.camera_state(1_000_000_001))
        self.assertIn("не обновляются", history.camera_state(4_000_000_000))

    def test_loss_and_network_outage_break_trajectory(self):
        history = StreamHistory(1_000_000_000)
        for sequence, stamp, state in [(0, 100, "TRACKING"), (1, 200, "LOST"), (2, 300, "TRACKING")]:
            event = display_event(sequence=sequence, frame_stamp=stamp, status=state)
            history.observe(event["payload"], event["pc_receive_monotonic_ns"])
        self.assertEqual(3, len(history.trajectory))
        self.assertTrue(math.isnan(history.trajectory[1][0]))
        event = display_event(sequence=3, frame_stamp=400, received_ns=5_000_000_000)
        history.observe(event["payload"], event["pc_receive_monotonic_ns"])
        self.assertTrue(math.isnan(history.trajectory[-2][0]))

    def test_epoch_reset_clears_old_path_even_before_first_frame(self):
        model = DisplayHistory()
        model.observe(display_event())
        model.observe(display_event(sequence=1, epoch=3, frame_stamp=0, status="INITIALIZING", x_m=0, y_m=0))
        history = model.streams[model.last_key]
        self.assertEqual(3, history.epoch)
        self.assertEqual(0, len(history.trajectory))
        self.assertEqual("Ожидание первого кадра", history.camera_state(2_000_000_000))
        model.observe(display_event(sequence=2, epoch=3, frame_stamp=500))
        self.assertEqual(1, len(history.trajectory))

    def test_reordered_datagrams_and_nonincreasing_frames_do_not_move_path(self):
        model = DisplayHistory()
        model.observe(display_event(sequence=10))
        event = display_event(sequence=9, x_m=100)
        event["sequence_result"] = "reordered"
        model.observe(event)
        model.observe(display_event(sequence=11, frame_stamp=99, x_m=200))
        self.assertEqual([(0.42, -0.18)], list(model.streams[model.last_key].trajectory))

    def test_streams_and_history_are_bounded_and_sessions_never_merge(self):
        model = DisplayHistory()
        for index in range(20):
            model.observe(display_event(session=f"session-{index}"))
        self.assertEqual(16, len(model.streams))
        history = model.streams[model.last_key]
        payload = display_event()["payload"]
        for index in range(6000):
            payload["sequence"] = index + 1
            payload["planar_odometry"]["timestamp_ns"] = index + 1000
            payload["sensor_timestamps_ns"] = {name: index + 10**12 for name in history.sensors}
            history.observe(payload, 1_000_000_000 + index * 20_000_000)
        self.assertEqual(5000, len(history.trajectory))
        self.assertTrue(all(len(series) == 1500 for series in history.sensors.values()))

    def test_clear_is_display_only_and_unknown_sensor_timestamps_are_skipped(self):
        model = DisplayHistory()
        event = display_event()
        event["payload"]["sensor_timestamps_ns"] = {name: 0 for name in (
            "quaternion", "gyroscope", "linear_acceleration")}
        model.observe(event)
        history = model.streams[model.last_key]
        self.assertTrue(all(not series for series in history.sensors.values()))
        model.clear_plots()
        self.assertEqual(0, len(history.trajectory))
        self.assertEqual(0.42, history.latest["planar_odometry"]["x_m"])
        self.assertEqual(2, history.epoch)

    def test_legacy_sensors_are_shown_without_camera(self):
        model = DisplayHistory()
        event = display_event()
        del event["payload"]["sensor_timestamps_ns"]
        event["payload"]["planar_odometry"] = None
        model.observe(event)
        history = model.streams[model.last_key]
        self.assertEqual(1, len(history.sensors["gyroscope"]))
        self.assertEqual("Нет данных камеры", history.camera_state(1_000_000_000))
