"""Validate recent bias-corrected finger forces for rotation release detection."""
from __future__ import annotations

from collections.abc import Mapping
import math
from numbers import Real


_FINGERS = ("thumb", "index", "middle")


def _finite_real(value) -> float | None:
    if isinstance(value, bool) or not isinstance(value, Real):
        return None
    try:
        number = float(value)
    except (ValueError, OverflowError):
        return None
    return number if math.isfinite(number) else None


def total_force_magnitude(fingers, now, max_age_sec=0.5) -> float | None:
    """Sum three force norms only when every finger has a valid recent sample.

    Each snapshot entry contains ``value=(Fx, Fy, Fz, source_timestamp)`` and a
    monotonic ``arrival`` time. Freshness uses arrival, independent of source
    clocks. Unknown, stale or invalid samples return ``None``, never zero.
    """
    now = _finite_real(now)
    max_age_sec = _finite_real(max_age_sec)
    if (not isinstance(fingers, Mapping) or now is None or max_age_sec is None
            or max_age_sec <= 0):
        return None

    total = 0.0
    for name in _FINGERS:
        sample = fingers.get(name)
        if not isinstance(sample, Mapping):
            return None
        arrival = _finite_real(sample.get("arrival"))
        if arrival is None or not 0 <= now - arrival <= max_age_sec:
            return None
        values = sample.get("value")
        if isinstance(values, (str, bytes, Mapping)):
            return None
        try:
            if len(values) != 4:
                return None
            components = tuple(_finite_real(values[index]) for index in range(4))
        except (TypeError, IndexError, KeyError):
            return None
        if any(component is None for component in components):
            return None
        total += math.hypot(*components[:3])
        if not math.isfinite(total):
            return None
    return total
