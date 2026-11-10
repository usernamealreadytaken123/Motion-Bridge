"""Small synthetic MP4 fixture and end-to-end check, also available in the EXE."""
from fractions import Fraction
import hashlib
import json
import math
from pathlib import Path
import av
import cv2
import numpy as np


def make_clip(path, count=12, times=None, rotation=0, mirrored=False, blank_indices=(), zoom_indices=()):
    image = cv2.GaussianBlur(np.random.default_rng(123).integers(0, 256, (480, 640), dtype=np.uint8), (3, 3), .7)
    times = [500+i*100 for i in range(count)] if times is None else times
    with av.open(str(path), "w", format="mp4") as container:
        stream = container.add_stream("libx264", rate=30)
        stream.width, stream.height = 640, 480
        stream.pix_fmt = "yuv420p"
        stream.time_base = Fraction(1, 1000)
        stream.codec_context.time_base = Fraction(1, 1000)
        stream.options = {"crf": "12", "preset": "veryfast", "bf": "2"}
        stream.set_display_rotation(rotation, hflip=mirrored)
        for i, stamp in enumerate(times):
            scale = 1.08 if i in zoom_indices else 1.
            moved = cv2.warpAffine(image, np.float32([[scale, 0, (1-scale)*319.5-4*i],
                                                     [0, scale, (1-scale)*239.5+2*i]]), (640, 480))
            if i in blank_indices:
                moved[:] = 0
            frame = av.VideoFrame.from_ndarray(moved, format="gray")
            frame.pts, frame.time_base = stamp, Fraction(1, 1000)
            for packet in stream.encode(frame):
                container.mux(packet)
        for packet in stream.encode(None):
            container.mux(packet)


def exercise_mp4(window, wait_for, report):
    directory = window.record_directory
    directory.mkdir(parents=True, exist_ok=True)
    source = directory / "synthetic-motion.mp4"
    make_clip(source)
    digest = hashlib.sha256(source.read_bytes()).digest()
    window.open_mp4()
    video = window.mp4_window
    if video is None:
        raise RuntimeError(window.error_label.text())
    video.open_video(source)
    video.start()
    worker = video.worker
    wait_for(lambda: video.worker is None)
    update = worker.update
    result = update["result"]
    if update["error"] or not result or result["status"] != "TRACKING":
        raise RuntimeError(f"MP4 calculation failed: {update['error']}; {result}")
    if update["frames"] != 12 or result["segment"] != 0 or abs(result["x_m"]-44/576) > .003 or abs(result["y_m"]+22/576) > .003:
        raise RuntimeError(f"MP4 metric estimate failed: {result}")
    if video.preview.pixmap().isNull() or "ОЦЕНКА" not in video.metrics.text() or len(video.path) != 12:
        raise RuntimeError("MP4 results did not reach Qt widgets")
    manifest = json.loads((Path(video.recording)/"manifest.json").read_text(encoding="utf-8"))
    if (not manifest["complete"] or manifest["calibration"] != "ASSUMED"
            or manifest["algorithm"] != "similarity_2d" or manifest["tilt_compensation"]
            or manifest["frame_sampling"] != "all_decoded_frames"):
        raise RuntimeError("MP4 result manifest is invalid")
    if digest != hashlib.sha256(source.read_bytes()).digest():
        raise RuntimeError("MP4 source was changed")
    summary = update["final_summary"]
    if (summary != manifest["final_summary"] or not summary["at_video_end"]
            or summary["x_m"] != result["x_m"] or summary["y_m"] != result["y_m"]
            or abs(summary["distance_from_origin_m"]-math.hypot(result["x_m"], result["y_m"])) > 1e-12
            or not video.summary.isVisible()
            or video.summary.text().splitlines()[1] != f"Расстояние от (0, 0) по прямой: {summary['distance_from_origin_m']:.3f} м"):
        raise RuntimeError("MP4 final coordinates/distance summary failed")
    if not video.grab().save(str(report.with_name(report.stem+"-mp4.png"))):
        raise RuntimeError("Cannot render MP4 window")
    # A transient occlusion does not reset XY. Unknown steps (4, 5, 6) are
    # SKIPPED, not recovered; next valid pair (6 -> 7) must resume immediately.
    recovery_source = directory / "synthetic-motion-gap.mp4"
    make_clip(recovery_source, blank_indices=(4, 5))
    recovery_digest = hashlib.sha256(recovery_source.read_bytes()).digest()
    video.open_video(recovery_source)
    video.start()
    recovery_worker = video.worker
    wait_for(lambda: video.worker is None)
    recovered = recovery_worker.update["result"]
    if recovery_worker.update["error"] or recovered["status"] != "TRACKING" or recovered["segment"] != 1 or abs(recovered["x_m"]-32/576) > .003:
        raise RuntimeError(f"MP4 recovery failed: {recovered}")
    rows = [json.loads(line) for line in (Path(video.recording)/"results.jsonl").read_text().splitlines()]
    if any(rows[i]["x_m"] != rows[3]["x_m"] or rows[i]["y_m"] != rows[3]["y_m"] for i in (4, 5, 6)) or not rows[7]["recovered"]:
        raise RuntimeError("MP4 rejected frames changed the accepted position")
    if recovered["heading_continuous"] or "условны" not in video.metrics.text():
        raise RuntimeError("MP4 gap warning is missing")
    if abs(video.marker.xData[0]-recovered["x_m"]) > 1e-9 or not np.isnan(video.curve.xData).any():
        raise RuntimeError("MP4 recovery/gap did not reach the plot")
    if recovery_digest != hashlib.sha256(recovery_source.read_bytes()).digest():
        raise RuntimeError("MP4 recovery source was changed")
    if not video.grab().save(str(report.with_name(report.stem+"-mp4-recovery.png"))):
        raise RuntimeError("Cannot render MP4 recovery window")
    # Reject >6% zoom transitions, but the next valid 2D pair must work even
    # BEFORE returning to the original scale. Never pin a metric anchor.
    chain_source = directory / "synthetic-motion-chain.mp4"
    make_clip(chain_source, zoom_indices=(4, 5))
    chain_digest = hashlib.sha256(chain_source.read_bytes()).digest()
    video.open_video(chain_source)
    video.start()
    chain_worker = video.worker
    wait_for(lambda: video.worker is None)
    chain = chain_worker.update["result"]
    rows = [json.loads(line) for line in (Path(video.recording)/"results.jsonl").read_text().splitlines()]
    if chain_worker.update["error"] or chain["status"] != "TRACKING" or abs(chain["x_m"]-36/576) > .003:
        raise RuntimeError(f"MP4 2D resume failed: {chain}")
    for i in (4, 6):
        if (rows[i]["visual_status"] != "TRACKING" or rows[i]["status"] != "DEGRADED"
                or rows[i]["x_m"] != rows[i-1]["x_m"] or abs(rows[i]["reference_time_s"]-(i-1)*.1) > 1e-6):
            raise RuntimeError(f"MP4 rejected pair advanced incorrectly: {rows[i]}")
    for i in (5, 7):
        if rows[i]["pose_method"] != "SIMILARITY_2D" or abs(rows[i]["delta_x_m"]-4/576) > .003:
            raise RuntimeError(f"MP4 next pair did not resume 2D motion: {rows[i]}")
    if chain_digest != hashlib.sha256(chain_source.read_bytes()).digest():
        raise RuntimeError("MP4 chain source was changed")
    if not video.grab().save(str(report.with_name(report.stem+"-mp4-chain.png"))):
        raise RuntimeError("Cannot render MP4 chain window")
    # The packaged UI and engine must both accept the upper height boundary.
    video.open_video(source)
    video.height.setValue(100.)
    if video.height.value() != 100.:
        raise RuntimeError("MP4 height control still clamps below 100 m")
    video.start()
    high_worker = video.worker
    wait_for(lambda: video.worker is None)
    high = high_worker.update["result"]
    high_summary = high_worker.update["final_summary"]
    high_manifest = json.loads((Path(video.recording)/"manifest.json").read_text(encoding="utf-8"))
    if (high_worker.update["error"] or high["status"] != "TRACKING" or high["height_m"] != 100.
            or high_manifest["height_m"] != 100. or high_worker.update["tracking"] != update["tracking"]
            or not np.allclose([high["x_m"], high["y_m"]], 100*np.array([result["x_m"], result["y_m"]]), atol=1e-8)
            or abs(high_summary["distance_from_origin_m"]-100*summary["distance_from_origin_m"]) > 1e-8):
        raise RuntimeError(f"MP4 100 m height scaling failed: {high_summary}")
    if not video.grab().save(str(report.with_name(report.stem+"-mp4-height100.png"))):
        raise RuntimeError("Cannot render MP4 100 m window")
    video.height.setValue(1.)
    video.close()
    return dict(frames=update["frames"], x_m=result["x_m"], y_m=result["y_m"], source_unchanged=True,
                approximate=True, recovery_without_reset=True, adjacent_frame_2d=True,
                missing_motion_skipped=True, final_summary=True, height_100m=True)
