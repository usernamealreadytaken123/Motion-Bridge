"""Bounded sequence tracking: pending gaps are not immediately called packet loss."""

from collections import deque
from dataclasses import dataclass, field

from .protocol import Telemetry


@dataclass
class SequenceWindow:
    window_size: int = 256
    highest: int | None = None
    base: int = 0
    seen: set[int] = field(default_factory=set)
    missing_finalized: int = 0
    duplicates: int = 0
    reordered: int = 0
    too_late: int = 0

    @property
    def pending_gaps(self) -> int:
        if self.highest is None:
            return 0
        return self.highest - self.base + 1 - len(self.seen)

    def observe(self, sequence: int) -> str:
        if self.highest is None:
            # Reception may start mid-session. Earlier sequence numbers are not losses.
            self.base = self.highest = sequence
        if sequence < self.base:
            self.too_late += 1
            return "too_late"
        if sequence in self.seen:
            self.duplicates += 1
            return "duplicate"
        outcome = "new"
        if sequence < self.highest:
            self.reordered += 1
            outcome = "reordered"
        if sequence > self.highest:
            new_base = max(self.base, sequence - self.window_size + 1)
            evicted = {number for number in self.seen if number < new_base}
            self.missing_finalized += new_base - self.base - len(evicted)
            self.seen.difference_update(evicted)
            self.base = new_base
            self.highest = sequence
        self.seen.add(sequence)
        return outcome


@dataclass
class StreamStats:
    sequences: SequenceWindow = field(default_factory=SequenceWindow)
    valid_datagrams: int = 0
    last_received_ns: int | None = None
    latest: Telemetry | None = None
    arrival_times: deque[int] = field(default_factory=lambda: deque(maxlen=4096))

    def observe(self, packet: Telemetry, received_ns: int) -> str:
        outcome = self.sequences.observe(packet.sequence)
        self.valid_datagrams += 1
        self.last_received_ns = received_ns
        self.arrival_times.append(received_ns)
        if outcome == "new":
            self.latest = packet
        return outcome

    def snapshot(self, now_ns: int) -> dict:
        while self.arrival_times and now_ns - self.arrival_times[0] > 3_000_000_000:
            self.arrival_times.popleft()
        elapsed = (now_ns - self.arrival_times[0]) / 1e9 if self.arrival_times else 0
        rate = (len(self.arrival_times) - 1) / elapsed if elapsed > 0 else 0.0
        age = (now_ns - self.last_received_ns) / 1e9 if self.last_received_ns is not None else None
        return {
            "receiving": age is not None and age <= 2.0,
            "age_seconds": age,
            "packets_per_second": round(rate, 1),
            "valid_datagrams": self.valid_datagrams,
            "highest_sequence": self.sequences.highest,
            "missing_finalized": self.sequences.missing_finalized,
            "pending_gaps": self.sequences.pending_gaps,
            "duplicates": self.sequences.duplicates,
            "reordered": self.sequences.reordered,
            "too_late": self.sequences.too_late,
        }
