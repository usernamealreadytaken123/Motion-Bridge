#!/usr/bin/env python3
"""Report any UDP traffic received on a port, without parsing its contents."""

import argparse
import signal
import socket
import time


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Count any incoming UDP packets without decoding their payload."
    )
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5005)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    stop_requested = False

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    if hasattr(signal, "SIGBREAK"):
        signal.signal(signal.SIGBREAK, request_stop)

    packet_count = 0
    byte_count = 0
    started_at = time.monotonic()

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind((args.host, args.port))
        sock.settimeout(0.5)
        print(f"UDP probe is listening on {args.host}:{args.port}")
        print("Every received datagram will be reported. Press Ctrl+C to stop.\n")

        while not stop_requested:
            try:
                data, (sender_host, sender_port) = sock.recvfrom(65_535)
            except socket.timeout:
                continue

            packet_count += 1
            byte_count += len(data)
            elapsed = max(time.monotonic() - started_at, 0.001)
            print(
                f"packet={packet_count} source={sender_host}:{sender_port} "
                f"size={len(data)}B total={byte_count}B "
                f"average={packet_count / elapsed:.1f} packets/s",
                flush=True,
            )

    print(f"\nStopped. Received {packet_count} packets ({byte_count} bytes).")


if __name__ == "__main__":
    main()
