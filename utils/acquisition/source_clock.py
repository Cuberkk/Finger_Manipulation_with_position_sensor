"""Read-completion timestamps from the source computer's UTC system clock.

Python before 3.13 can expose a coarse Windows ``time.time()`` clock. At high
acquisition rates that can assign one timestamp to several readings.
Use Windows' precise system-time API directly, while retaining true clock
adjustments so the central processor can detect backward jumps.
"""

import ctypes
import sys
import time


_FILETIME_UNIX_EPOCH_TICKS = 116444736000000000
_FILETIME_TICKS_PER_SECOND = 10_000_000


class _FileTime(ctypes.Structure):
    # DWORD is always 32 bits in the Windows ABI, including 64-bit Windows.
    _fields_ = [("low", ctypes.c_uint32), ("high", ctypes.c_uint32)]


def _windows_source_clock():
    """Bind the precise UTC clock once; each invocation owns its FILETIME."""
    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    try:
        read_time = kernel32.GetSystemTimePreciseAsFileTime
    except AttributeError as exc:
        raise RuntimeError(
            "Force acquisition requires GetSystemTimePreciseAsFileTime "
            "on Windows 8 or later."
        ) from exc
    read_time.argtypes = [ctypes.POINTER(_FileTime)]
    read_time.restype = None

    def precise_time():
        value = _FileTime()
        read_time(ctypes.byref(value))
        ticks = (value.high << 32) | value.low
        # Subtract the large 1601-to-1970 offset before float conversion.
        return (ticks - _FILETIME_UNIX_EPOCH_TICKS) / _FILETIME_TICKS_PER_SECOND

    return precise_time


if sys.platform == "win32":
    source_time = _windows_source_clock()
else:
    def source_time():
        """Return source-host Unix seconds, without changing clock adjustments."""
        return time.time()
