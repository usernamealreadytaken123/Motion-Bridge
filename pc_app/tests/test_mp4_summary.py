import importlib.util
import math
import unittest

if not all(importlib.util.find_spec(name) for name in ("av", "cv2", "numpy")):
    raise unittest.SkipTest("MP4 dependencies are required")

from pc_app.motion_viewer.mp4_worker import summarize_result


class Mp4SummaryTests(unittest.TestCase):
    def point(self, x, y, seq=10, status="TRACKING", continuous=True, origin=0.):
        return dict(x_m=x, y_m=y, seq=seq, time_s=seq*.1, status=status,
                    heading_continuous=continuous, origin_time_s=origin, distance_m=100.)

    def test_endpoint_distance_is_hypotenuse_not_path_length(self):
        for x, y in ((3., 4.), (-3., 4.), (-3., -4.), (3., -4.)):
            point = self.point(x, y)
            summary = summarize_result(point, point)
            self.assertEqual((x, y), (summary["x_m"], summary["y_m"]))
            self.assertEqual(5., summary["distance_from_origin_m"])
            self.assertTrue(summary["point_available"])
            self.assertTrue(summary["at_video_end"])
            self.assertEqual("complete", summary["state"])

    def test_zero_endpoint_is_valid_and_hypotenuse_uses_unrounded_values(self):
        for x, y in ((0., 0.), (.0004, -.0004)):
            point = self.point(x, y)
            summary = summarize_result(point, point)
            self.assertTrue(summary["point_available"])
            self.assertEqual(math.hypot(x, y), summary["distance_from_origin_m"])

    def test_lost_tail_keeps_timestamp_of_last_accepted_point(self):
        point = self.point(3., -4., seq=7)
        last = self.point(3., -4., seq=11, status="DEGRADED", continuous=False)
        summary = summarize_result(last, point)
        self.assertTrue(summary["point_available"])
        self.assertFalse(summary["at_video_end"])
        self.assertAlmostEqual(.7, summary["point_time_s"])
        self.assertAlmostEqual(1.1, summary["last_processed_time_s"])
        self.assertEqual(5., summary["distance_from_origin_m"])
        self.assertTrue(summary["has_gaps"])

    def test_no_accepted_pose_is_unknown_not_zero(self):
        for last in (None, self.point(0., 0., status="INITIALIZING"), self.point(0., 0., status="DEGRADED")):
            summary = summarize_result(last, None)
            self.assertFalse(summary["point_available"])
            self.assertFalse(summary["at_video_end"])
            for field in ("x_m", "y_m", "distance_from_origin_m", "point_time_s"):
                self.assertIsNone(summary[field])

    def test_cancel_and_error_never_claim_video_endpoint(self):
        point = self.point(3., 4.)
        for options, state in (({"cancelled": True}, "cancelled"), ({"error": "failed"}, "error")):
            summary = summarize_result(point, point, **options)
            self.assertEqual(state, summary["state"])
            self.assertFalse(summary["at_video_end"])
            self.assertEqual(5., summary["distance_from_origin_m"])

    def test_gaps_and_late_origin_remain_explicit_at_video_end(self):
        point = self.point(3., 4., continuous=False, origin=.2)
        summary = summarize_result(point, point)
        self.assertTrue(summary["at_video_end"])
        self.assertTrue(summary["has_gaps"])
        self.assertEqual(.2, summary["origin_time_s"])
