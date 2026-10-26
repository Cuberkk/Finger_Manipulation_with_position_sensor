"""Defer terminal signals until the GUI can save and release its resources."""
from __future__ import annotations

import signal


class ShutdownSignals:
    """Keep repeated Ctrl+C from interrupting an in-progress save.

    Signal handlers only record intent. Tk callbacks and the main finally block
    perform all locking, file writes and resource cleanup in normal code.
    """

    def __init__(self):
        self.signum = None
        self.previous = {}

    @property
    def requested(self):
        return self.signum is not None

    @property
    def reason(self):
        if self.signum == signal.SIGINT:
            return "interrupted"
        if self.signum == signal.SIGTERM:
            return "terminated"
        return "closed"

    def _request(self, signum, _frame):
        if self.signum is None:
            self.signum = signum

    def __enter__(self):
        for signum in (signal.SIGINT, signal.SIGTERM):
            self.previous[signum] = signal.signal(signum, self._request)
        return self

    def __exit__(self, *_exc):
        for signum, handler in self.previous.items():
            signal.signal(signum, handler)
