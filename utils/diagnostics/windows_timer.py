"""Temporary Windows timer-resolution requests for acquisition diagnostics."""

from contextlib import contextmanager
import ctypes
import sys
import warnings


def _warn_cleanup_failure(message):
    # A warning filter must not turn cleanup reporting into a replacement for
    # the original exception raised by the diagnostic body.
    try:
        warnings.warn(message, RuntimeWarning, stacklevel=3)
    except RuntimeWarning:
        print(f"RuntimeWarning: {message}", file=sys.stderr)


@contextmanager
def windows_timer_resolution(period_ms=1):
    """Pair timeBeginPeriod/timeEndPeriod, including when the body is interrupted.

    A successful request can improve timer waits; it does not guarantee a
    callback rate. Every successful begin must be matched with the same period:
    https://learn.microsoft.com/en-us/windows/win32/api/timeapi/nf-timeapi-timebeginperiod
    """
    if sys.platform != "win32":
        raise RuntimeError("Windows timer-resolution diagnostics require Windows.")
    if type(period_ms) is not int or not 1 <= period_ms <= 0xFFFFFFFF:
        raise ValueError("period_ms must be a positive 32-bit unsigned integer.")

    winmm = ctypes.WinDLL("winmm")
    begin = winmm.timeBeginPeriod
    end = winmm.timeEndPeriod
    # UINT and MMRESULT are unsigned 32-bit values even on 64-bit Windows.
    for function in (begin, end):
        function.argtypes = [ctypes.c_uint32]
        function.restype = ctypes.c_uint32

    result = begin(period_ms)
    if result != 0:
        raise RuntimeError(
            f"timeBeginPeriod({period_ms}) failed with MMRESULT {result}."
        )
    try:
        yield
    finally:
        try:
            result = end(period_ms)
        except Exception as exc:
            _warn_cleanup_failure(f"timeEndPeriod({period_ms}) failed: {exc}")
        else:
            if result != 0:
                _warn_cleanup_failure(
                    f"timeEndPeriod({period_ms}) failed with MMRESULT {result}."
                )
