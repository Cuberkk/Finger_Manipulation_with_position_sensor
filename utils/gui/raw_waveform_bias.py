"""Apply the central sensor offsets to raw waveform copies for display only."""
from __future__ import annotations

import math
from numbers import Real

from utils.gui.study_io import SENSORS


def raw_bias_offsets(status):
    """Return fresh finger offsets only for a complete, confirmed active bias.

    ROSBridge derives ``available`` and ``ready`` from the current status age
    and transaction state. Missing, stale, or unfinished calibration therefore
    produces no display offsets rather than silently showing uncorrected data.
    """
    if (not isinstance(status, dict) or status.get("ready") is not True
            or status.get("available") is not True
            or status.get("busy") or status.get("pending")):
        return None
    biases = status.get("biases")
    if not isinstance(biases, dict) or set(biases) != {str(sensor) for sensor in SENSORS.values()}:
        return None
    result = {}
    for finger, sensor in SENSORS.items():
        vector = biases[str(sensor)]
        if not isinstance(vector, (list, tuple)) or len(vector) != 6:
            return None
        try:
            if any(isinstance(value, bool) or not isinstance(value, Real)
                   or not math.isfinite(value) for value in vector):
                return None
            result[finger] = tuple(float(value) for value in vector)
        except (TypeError, ValueError, OverflowError):
            return None
    return result


def subtract_raw_bias(segments, offset):
    """Subtract one constant from immutable display segments, preserving time gaps."""
    try:
        if isinstance(offset, bool) or not isinstance(offset, Real) or not math.isfinite(offset):
            raise ValueError("Raw display bias must be a finite number.")
        offset = float(offset)
    except (TypeError, OverflowError) as exc:
        raise ValueError("Raw display bias must be a finite number.") from exc
    return tuple(tuple((arrival, value - offset) for arrival, value in segment)
                 for segment in segments)
