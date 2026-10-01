#!/usr/bin/env python3
"""Simple UDP packet receiver for local testing."""

import argparse
import signal
import socket
from datetime import datetime


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Receive UDP packets and print them to the console."
    )
    parser.add_argument(
        "--host",
        default="192.168.0.218",
        help="Address to listen on (default: 0.0.0.0)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=5005,
        help="UDP port to listen on (default: 5005)",
    )
    parser.add_argument(
        "--buffer-size",
        type=int,
        default=65535,
        help="Maximum packet size in bytes (default: 65535)",
    )
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

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind((args.host, args.port))
        # A finite timeout lets Python handle Ctrl+C reliably on Windows
        # while no UDP packets are arriving.
        sock.settimeout(0.5)
        print(f"Listening for UDP packets on {args.host}:{args.port}")
        print("Press Ctrl+C to stop.\n")

        while not stop_requested:
            try:
                data, (sender_host, sender_port) = sock.recvfrom(args.buffer_size)
            except socket.timeout:
                continue

            timestamp = datetime.now().astimezone().isoformat(timespec="seconds")

            try:
                message = data.decode("utf-8")
            except UnicodeDecodeError:
                message = f"<binary data: {data.hex(' ')}>"

            print(
                f"[{timestamp}] {sender_host}:{sender_port} "
                f"({len(data)} bytes): {message}",
                flush=True,
            )

        print("\nReceiver stopped.")


if __name__ == "__main__":
    main()
