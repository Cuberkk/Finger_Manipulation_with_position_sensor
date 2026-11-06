"""Bounded force/torque history independent of plotting and trial recording."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import threading
import time
from typing import Iterable


HISTORY_WINDOW_SEC = 30.0
HISTORY_MAX_SAMPLES = 32768
StreamKey = tuple[str, str]


@dataclass(frozen=True, slots=True)
class ForceHistorySample:
    """One force or force/torque sample with independent receipt/source clocks."""

    arrival: float
    source_timestamp: float
    values: tuple[float, ...]


class ForceHistory:
    """Keep callback samples within a receipt-time window and a memory cap.

    Streams receive samples in monotonic arrival order, as provided by the
    bridge's serialized ROS callbacks. Snapshots detach immutable sample
    references under a short lock; drawing and signal selection happen later.
    """

    def __init__(self, streams: Iterable[StreamKey], *,
                 window_seconds: float = HISTORY_WINDOW_SEC,
                 max_samples: int = HISTORY_MAX_SAMPLES):
        if (isinstance(window_seconds, bool) or not math.isfinite(window_seconds)
                or window_seconds <= 0):
            raise ValueError("Force history window must be positive and finite.")
        if type(max_samples) is not int or max_samples <= 0:
            raise ValueError("Force history sample cap must be a positive integer.")
        self.window_seconds = float(window_seconds)
        self._streams = {key: deque(maxlen=max_samples) for key in streams}
        self._lock = threading.Lock()

    @staticmethod
    def _prune(stream, cutoff):
        # A sample exactly 30 seconds old remains on the left window boundary.
        while stream and stream[0].arrival < cutoff:
            stream.popleft()

    def append(self, key: StreamKey, sample: ForceHistorySample):
        """Retain an already validated callback sample without reducing its rate."""
        with self._lock:
            stream = self._streams[key]
            self._prune(stream, sample.arrival - self.window_seconds)
            stream.append(sample)

    def snapshot(self, now: float | None = None) -> dict[StreamKey, tuple[ForceHistorySample, ...]]:
        """Return all streams, expiring silent streams using the same clock."""
        now = time.monotonic() if now is None else now
        with self._lock:
            cutoff = now - self.window_seconds
            for stream in self._streams.values():
                self._prune(stream, cutoff)
            return {key: tuple(stream) for key, stream in self._streams.items()}
