"""Stage 1 command-line monitor. Run from the project root: python -m pc_app.motion_viewer."""

import argparse
import base64
import json
import math
from pathlib import Path
import signal
import socket
import sys
import time
import uuid

from .protocol import PacketError, parse_packet
from .receiver import TelemetryReceiver
from .recording import RecordingError, SessionRecorder


def _positive(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("Must be finite and positive")
    return number


def _port(value: str) -> int:
    port = int(value)
    if not 1 <= port <= 65535:
        raise argparse.ArgumentTypeError("Port must be 1..65535")
    return port


def _listen(args: argparse.Namespace) -> int:
    recorder = None
    receiver = None
    stopped = False

    def stop(_signum: int, _frame: object) -> None:
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, stop)
    if hasattr(signal, "SIGBREAK"):
        signal.signal(signal.SIGBREAK, stop)
    try:
        receiver = TelemetryReceiver(args.host, args.port)
        if args.record:
            recorder = SessionRecorder(args.record)
            receiver.recorder = recorder
        print(f"Listening on {args.host}:{args.port}; Ctrl+C stops. UDP v1/v2 accepted.", flush=True)
        if args.record:
            print(f"Recording to {args.record.resolve()}", flush=True)
        started = time.monotonic()
        next_report = started
        while not stopped and (args.duration is None or time.monotonic() - started < args.duration):
            receiver.receive_once()
            now = time.monotonic()
            if now >= next_report:
                _print_status(receiver.snapshot())
                next_report = now + 1.0
        _print_status(receiver.snapshot())
    finally:
        if receiver:
            receiver.close()
        if recorder:
            recorder.close()
            print(f"Saved {recorder.written} datagrams.", flush=True)
    return 0


def _print_status(snapshot: dict) -> None:
    print(f"datagrams={snapshot['datagrams']} invalid={snapshot['invalid']}", flush=True)
    for stream in snapshot["streams"]:
        session = stream["session_id"] or "legacy-v1"
        state = "RECEIVING" if stream["receiving"] else "STALE"
        planar = stream["planar_odometry"]
        pose = "camera=unavailable"
        if planar:
            pose = (f"camera={planar['status']} epoch={planar['epoch']} "
                    f"x={planar['x_m']:+.4f}m y={planar['y_m']:+.4f}m "
                    f"inliers={planar['inliers']}/{planar['tracked_points']}")
        print(
            f"  {state} {stream['sender']} session={session[:8]} "
            f"v{stream['protocol_version']} {stream['packets_per_second']:.1f}pps "
            f"seq={stream['highest_sequence']} missing={stream['missing_finalized']} "
            f"pending={stream['pending_gaps']} dup={stream['duplicates']} "
            f"reordered={stream['reordered']} old={stream['too_late']} "
            f"age={stream['age_seconds']:.2f}s {pose}", flush=True,
        )
        quaternion = stream["quaternion"]
        gyro = stream["gyroscope"]
        acceleration = stream["linear_acceleration"]
        if quaternion and gyro and acceleration:
            print(
                "    q(wxyz)=" + ",".join(f"{quaternion[a]:+.4f}" for a in "wxyz")
                + " gyro(rad/s)=" + ",".join(f"{gyro[a]:+.3f}" for a in "xyz")
                + " lin_acc(m/s2)=" + ",".join(f"{acceleration[a]:+.3f}" for a in "xyz"),
                flush=True,
            )
    if snapshot["last_error"]:
        print(f"  last invalid packet: {snapshot['last_error']}", flush=True)


def _simulate(args: argparse.Namespace) -> int:
    """Deliberately synthetic circular motion; never used as sensor measurements."""
    session = f"sim-{uuid.uuid4()}"
    total = max(1, int(args.duration * args.rate))
    started = time.monotonic()
    sent = 0
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        for sequence in range(total):
            delay = started + sequence / args.rate - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            if args.drop_every and (sequence + 1) % args.drop_every == 0:
                continue
            stamp = time.monotonic_ns()
            angle = sequence / args.rate * 0.2
            previous_angle = max(0, sequence - 1) / args.rate * 0.2
            packet = {
                "sequence": sequence, "timestamp": time.time(),
                "quaternion": {"w": 1, "x": 0, "y": 0, "z": 0},
                "gyroscope": {"x": 0, "y": 0, "z": 0},
                "linear_acceleration": {"x": 0, "y": 0, "z": 0},
                "height_above_surface_m": 1,
                "protocol_version": 2, "packet_type": "telemetry", "session_id": session,
                "device_timestamp_ns": stamp,
                "sensor_timestamps_ns": {
                    "quaternion": stamp, "gyroscope": stamp, "linear_acceleration": stamp,
                },
                "planar_odometry": {
                    "epoch": 1, "timestamp_ns": stamp, "status": "TRACKING",
                    "camera_active": True, "x_m": math.cos(angle) - 1, "y_m": math.sin(angle),
                    "heading_rad": 0,
                    "delta_x_m": math.cos(angle) - math.cos(previous_angle),
                    "delta_y_m": math.sin(angle) - math.sin(previous_angle),
                    "surface_distance_m": 1, "tracked_points": 200, "inliers": 180,
                    "inlier_ratio": 0.9, "image_scale": 1, "intrinsics_approximate": False,
                },
            }
            data = json.dumps(packet, separators=(",", ":")).encode("utf-8")
            parse_packet(data)
            sock.sendto(data, (args.host, args.port))
            sent += 1
            if args.duplicate_every and (sequence + 1) % args.duplicate_every == 0:
                sock.sendto(data, (args.host, args.port))
                sent += 1
    print(f"SIMULATED data: sent {sent} datagrams; {total} sequence slots; session={session}")
    return 0


def _inspect(args: argparse.Namespace) -> int:
    count = valid = rejected = 0
    sessions = set()
    with args.path.open(encoding="utf-8") as source:
        for line_number, line in enumerate(source, 1):
            try:
                record = json.loads(line)
                if record.get("record_version") != 1:
                    raise ValueError("Unsupported recording version")
                raw = base64.b64decode(record["raw_base64"], validate=True)
                try:
                    packet = parse_packet(raw)
                except PacketError:
                    if record["payload"] is not None or not record.get("error"):
                        raise ValueError("Invalid datagram was not marked as rejected")
                    rejected += 1
                else:
                    if record["payload"] != packet.payload or record.get("error") is not None:
                        raise ValueError("Decoded payload differs from original datagram")
                    sessions.add((tuple(record["sender"]), packet.session_id))
                    valid += 1
                count += 1
            except (ValueError, KeyError, TypeError, AttributeError) as exc:
                raise ValueError(f"Recording line {line_number}: {exc}") from exc
    print(f"Verified {count} records: valid={valid}, rejected={rejected}, streams={len(sessions)}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Android Motion Viewer — transport stage")
    commands = parser.add_subparsers(dest="command", required=True)
    listen = commands.add_parser("listen", help="Receive Android UDP telemetry")
    listen.add_argument("--host", default="0.0.0.0")
    listen.add_argument("--port", type=_port, default=5005)
    listen.add_argument("--record", type=Path, help="New JSONL file (existing files are never overwritten)")
    listen.add_argument("--duration", type=_positive, help="Optional run time in seconds")
    listen.set_defaults(run=_listen)
    simulate = commands.add_parser("simulate", help="Send SYNTHETIC v2 data for local testing")
    simulate.add_argument("--host", default="127.0.0.1")
    simulate.add_argument("--port", type=_port, default=5005)
    simulate.add_argument("--duration", type=_positive, default=10.0)
    simulate.add_argument("--rate", type=_positive, default=50.0)
    simulate.add_argument("--drop-every", type=int, default=0)
    simulate.add_argument("--duplicate-every", type=int, default=0)
    simulate.set_defaults(run=_simulate)
    inspect = commands.add_parser("inspect", help="Verify recorded datagrams and decoded payloads")
    inspect.add_argument("path", type=Path)
    inspect.set_defaults(run=_inspect)
    args = parser.parse_args()
    try:
        return args.run(args)
    except KeyboardInterrupt:
        return 130
    except (OSError, ValueError, RecordingError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
