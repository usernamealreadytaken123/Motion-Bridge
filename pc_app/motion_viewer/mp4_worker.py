"""Local MP4 demux/decode + experimental odometry, with bounded UI state."""
from collections import deque
from datetime import datetime
import json
import math
from pathlib import Path
import secrets
from threading import Event, Lock, Thread
import av
import numpy as np

from .mp4_2d import Video2DEngine, MAX_SCALE_DEVIATION, MAX_FRAME_SECONDS
from .mp4_odometry import FOCAL_FACTOR
from .preferences import LOGGER

MAX_SOURCE_PIXELS = 3840*2160
MAX_ANALYSIS_SIDE = 960
MAX_FPS = 15


def summarize_result(last_result, last_tracked, *, cancelled=False, error=None):
    """Endpoint displacement is the chord from (0, 0), not accumulated path.

    A held coordinate on a rejected final frame is NOT an observed endpoint.
    No accepted steps means unknown, not a fabricated zero-distance result.
    """
    state = "error" if error else "cancelled" if cancelled else "complete"
    available = last_tracked is not None
    x = float(last_tracked["x_m"]) if available else None
    y = float(last_tracked["y_m"]) if available else None
    return dict(state=state, point_available=available, x_m=x, y_m=y,
                distance_from_origin_m=math.hypot(x, y) if available else None,
                point_time_s=last_tracked["time_s"] if available else None,
                last_processed_time_s=last_result["time_s"] if last_result else None,
                at_video_end=bool(state == "complete" and available and last_result
                                  and last_result["status"] == "TRACKING"
                                  and last_tracked["seq"] == last_result["seq"]),
                has_gaps=bool(last_result and not last_result["heading_continuous"]),
                origin_time_s=last_result["origin_time_s"] if last_result else None)


def checked_dimensions(width, height):
    if min(width, height) < 32 or max(width, height) > 7680 or width*height > MAX_SOURCE_PIXELS:
        raise ValueError("Поддерживается видео от 32×32 до 3840×2160 пикселей (включая портретное)")


def upright_frame(frame, stream_rotation=0):
    checked_dimensions(frame.width, frame.height)
    matrix = frame.side_data.get("DISPLAYMATRIX")
    angle = frame.rotation if matrix is not None else stream_rotation
    if not math.isfinite(float(angle)) or abs(float(angle)/90-round(float(angle)/90)) > .01:
        raise ValueError("Не поддерживается поворот видео, отличный от 0/90/180/270°")
    if matrix is not None:
        values = np.frombuffer(bytes(matrix), dtype=np.int32)
        if len(values) != 9 or np.linalg.det(values.reshape(3, 3)[:2, :2].astype(float)) <= 0:
            raise ValueError("Не поддерживается зеркальная матрица отображения видео")
    scale = min(1., MAX_ANALYSIS_SIDE/max(frame.width, frame.height))
    gray = frame.reformat(width=max(32, round(frame.width*scale)), height=max(32, round(frame.height*scale)), format="gray").to_ndarray()
    return np.ascontiguousarray(np.rot90(gray, round(float(angle)/90)))


class Mp4Reader:
    def __init__(self, path, max_fps=MAX_FPS):
        self.path = Path(path)
        self.max_fps = max_fps
        self.duration = None
        self.decoded = 0
        self.original_start = None
        self.video_info = {}

    def frames(self, cancelled):
        if not self.path.is_file() or self.path.suffix.lower() != ".mp4":
            raise ValueError("Выберите существующий локальный файл .mp4")
        # Only the MOV/MP4 demuxer, no playlists, URL inputs or external data references.
        with av.open(str(self.path), format="mov", options={"enable_drefs": "0", "use_absolute_path": "0", "protocol_whitelist": "file"}) as container:
            streams = [s for s in container.streams.video if not (s.disposition & av.stream.Disposition.attached_pic)]
            if not streams:
                raise ValueError("В MP4 нет видеодорожки")
            stream = streams[0]
            checked_dimensions(stream.width, stream.height)
            # FFmpeg uses 0/1 for unspecified SAR; ordinary phone video is square.
            if stream.sample_aspect_ratio not in (None, 0, 1):
                raise ValueError("В первой версии поддерживаются только квадратные пиксели")
            rotation = float(stream.metadata.get("rotate", "0"))
            self.duration = float(stream.duration*stream.time_base) if stream.duration and stream.time_base else None
            if self.duration is not None and (not math.isfinite(self.duration) or self.duration <= 0):
                self.duration = None
            self.video_info = dict(width=stream.width, height=stream.height, codec=stream.codec_context.name,
                                   duration_s=self.duration, video_stream=stream.index)
            previous_time = None
            last_selected = -float("inf")
            geometry = None
            for frame in container.decode(stream):
                if cancelled.is_set():
                    return
                if frame.is_corrupt:
                    raise ValueError("Обнаружен повреждённый видеокадр")
                if frame.pts is None or frame.time_base is None:
                    raise ValueError("Нет временной метки кадра; частота кадров не подставляется вместо неё")
                stamp = frame.pts*frame.time_base
                if previous_time is not None and stamp <= previous_time:
                    raise ValueError("Повторяющиеся / убывающие временные метки видео")
                previous_time = stamp
                if self.original_start is None:
                    self.original_start = stamp
                relative = float(stamp-self.original_start)
                self.decoded += 1
                checked_dimensions(frame.width, frame.height)
                if geometry is None:
                    geometry = (frame.width, frame.height, frame.rotation)
                elif geometry != (frame.width, frame.height, frame.rotation):
                    raise ValueError("Размер / поворот видео изменился внутри записи")
                if self.max_fps is not None and relative-last_selected < 1/self.max_fps-1e-7:
                    continue
                gray = upright_frame(frame, rotation)
                last_selected = relative
                yield relative, gray
            if not self.decoded and not cancelled.is_set():
                raise ValueError("В MP4 нет декодируемых кадров")


class Mp4Worker(Thread):
    def __init__(self, path, height, directory, focal_factor=FOCAL_FACTOR):
        super().__init__(name="mp4-odometry", daemon=True)
        self.path = Path(path)
        self.directory = Path(directory)
        self.engine = Video2DEngine(height, focal_factor)
        self.cancelled, self.paused = Event(), Event()
        self.lock = Lock()
        self.queue = deque(maxlen=2048)
        self.update = dict(finished=False, error=None, status="Открытие MP4…", result=None, image=None,
                           frames=0, decoded=0, tracking=0, degraded=0, progress=None, recording=None,
                           display_dropped=0, video_info={}, visual_tracking=0, final_summary=None)

    def snapshot(self):
        with self.lock:
            result = dict(self.update, results=list(self.queue))
            self.queue.clear()
            return result

    def publish(self, **values):
        with self.lock:
            self.update.update(values)
            if "result" in values:
                # The full result (including tracks) goes to disk, not the UI queue.
                item = values["result"]
                if len(self.queue) == self.queue.maxlen:
                    self.update["display_dropped"] += 1
                self.queue.append({k: item[k] for k in ("time_s", "segment", "status", "x_m", "y_m")})

    def stop(self):
        self.cancelled.set()

    def run(self):
        output = None
        metadata = None
        error = None
        last_tracked = None
        # Offline 2D calculation uses EVERY decoded frame. The old 15 Hz cap
        # unnecessarily enlarged LK motion for ordinary 30/60 fps phone clips.
        reader = Mp4Reader(self.path, max_fps=None)
        frames = tracking = degraded = visual_tracking = 0
        try:
            original_stat = self.path.stat()
            # New directory each run. Never overwrite the source or old results.
            output = self.directory / (datetime.now().strftime("mp4-%Y%m%d-%H%M%S-")+secrets.token_hex(3))
            output.mkdir(parents=True, exist_ok=False)
            metadata = dict(format="mp4-planar-estimate-v4", algorithm="similarity_2d", source=str(self.path.resolve()),
                            source_size=original_stat.st_size, source_mtime_ns=original_stat.st_mtime_ns,
                            height_m=self.engine.height, constant_height=True, calibration="ASSUMED",
                            focal_factor=self.engine.focal_factor, max_analysis_side=MAX_ANALYSIS_SIDE,
                            max_analysis_fps=None, frame_sampling="all_decoded_frames",
                            coordinate_frame="initial_image_2d", tracking_loss_policy="adjacent_frames_keep_position_break_path",
                            tilt_compensation=False, max_scale_deviation=MAX_SCALE_DEVIATION, max_frame_seconds=MAX_FRAME_SECONDS,
                            warning="2D similarity only; no spatial tilt compensation. Constant h/f scales accepted image steps. Tilt, actual height changes, stabilisation and gaps can distort path shape. Missing steps are skipped, never recovered; heading after a gap is conditional on the last known heading.",
                            complete=False)
            self.publish(recording=str(output))
            (output/"manifest.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
            with (output/"results.jsonl").open("x", encoding="utf-8") as results:
                for time_s, gray in reader.frames(self.cancelled):
                    while self.paused.is_set() and not self.cancelled.wait(.05):
                        pass
                    if self.cancelled.is_set():
                        break
                    result = self.engine.process(gray, time_s, frames)
                    results.write(json.dumps(result, allow_nan=False, separators=(",", ":"))+"\n")
                    if result["status"] == "TRACKING":
                        last_tracked = result
                    frames += 1
                    tracking += result["status"] == "TRACKING"
                    degraded += result["status"] == "DEGRADED"
                    visual_tracking += result["visual_status"] == "TRACKING"
                    progress = min(.99, time_s/reader.duration) if reader.duration else None
                    self.publish(status="Расчёт MP4 · оценочная траектория", result=result, image=gray, frames=frames,
                                 decoded=reader.decoded, tracking=tracking, degraded=degraded, progress=progress, video_info=reader.video_info,
                                 visual_tracking=visual_tracking)
                current_stat = self.path.stat()
                if (original_stat.st_size, original_stat.st_mtime_ns) != (current_stat.st_size, current_stat.st_mtime_ns):
                    raise ValueError("Исходный MP4 изменился во время обработки")
        except Exception as exc:
            error = f"{type(exc).__name__}: {exc}"
            LOGGER.exception("MP4 odometry failed")
            self.publish(error=error)
        finally:
            if metadata is not None:
                metadata.update(complete=not error and not self.cancelled.is_set(), cancelled=self.cancelled.is_set(), error=error,
                                video=reader.video_info, processed_frames=frames, decoded_frames=reader.decoded,
                                first_pts_s=float(reader.original_start) if reader.original_start is not None else None,
                                tracking_frames=tracking, degraded_frames=degraded, segments=self.engine.segment+1,
                                recoveries=self.engine.recoveries, rejection_reasons=dict(self.engine.rejection_reasons),
                                visual_tracking_frames=visual_tracking, focal_px=self.engine.focal_px,
                                heading_continuous=self.engine.heading_continuous,
                                origin_time_s=self.engine.origin_time_s,
                                tracking_lost_at_end=self.engine.rejected_frames > 0,
                                distance_m=self.engine.distance,
                                final_summary=summarize_result(self.update["result"], last_tracked,
                                                               cancelled=self.cancelled.is_set(), error=error))
                try:
                    (output/"manifest.json").write_text(json.dumps(metadata, allow_nan=False, ensure_ascii=False, indent=2), encoding="utf-8")
                except Exception as exc:
                    LOGGER.exception("MP4 manifest finalization failed")
                    error = f"Не удалось завершить запись результатов: {exc}"
                    self.publish(error=error)
            status = "Ошибка обработки MP4" if error else "Расчёт отменён; сохранён обработанный участок" if self.cancelled.is_set() else "Расчёт завершён · результаты оценочные"
            if not error and not self.cancelled.is_set():
                if tracking == 0:
                    status = "Обработка завершена: траектория не определена; см. статусы кадров"
                elif degraded > frames/4:
                    status += " · много разрывов, полный путь неизвестен"
                if tracking and self.engine.rejected_frames:
                    status += " · в конце отслеживание потеряно, показана последняя рассчитанная позиция"
                if tracking and not self.engine.heading_continuous:
                    status += " · были пропуски: движение и повороты внутри них неизвестны"
                if self.engine.origin_time_s:
                    status += f" · начало координат с {self.engine.origin_time_s:.2f} с; движение до него неизвестно"
            self.publish(finished=True, status=status, decoded=reader.decoded,
                         final_summary=summarize_result(self.update["result"], last_tracked,
                                                        cancelled=self.cancelled.is_set(), error=error),
                         progress=1. if not error and not self.cancelled.is_set() else self.update["progress"])
