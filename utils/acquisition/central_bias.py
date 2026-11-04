"""Asynchronous, shared bias windows for remotely acquired ATI sensor data."""

import copy
import math
import operator

import numpy as np


SENSORS = (1, 2, 3)


def _positive(value, name):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive.")
    return value


class CentralBiasManager:
    """Collect all three sensor offsets over the same source-time window.

    Callbacks keep running during a tare. A monotonic deadline bounds waiting,
    warmup and collection, while source timestamps exclude packets queued before
    the common measurement window. Output is paused until the whole calibration
    is saved. Failure preserves the previous complete set of offsets.
    """

    def __init__(self, *, bias_time_sec=5.0, warmup_sec=0.5, timeout_sec=15.0,
                 max_age_sec=0.5, max_gap_sec=0.5, min_samples=30, save_callback=None):
        self.duration = _positive(bias_time_sec, "bias_time_sec")
        self.warmup = float(warmup_sec)
        if not math.isfinite(self.warmup) or self.warmup < 0:
            raise ValueError("warmup_sec must be finite and nonnegative.")
        self.timeout = _positive(timeout_sec, "timeout_sec")
        self.max_age = _positive(max_age_sec, "max_age_sec")
        self.max_gap = _positive(max_gap_sec, "max_gap_sec")
        try:
            self.min_samples = operator.index(min_samples)
        except TypeError as exc:
            raise ValueError("min_samples must be an integer >= 2.") from exc
        if isinstance(min_samples, (bool, np.bool_)) or self.min_samples < 2:
            raise ValueError("min_samples must be an integer >= 2.")
        if self.timeout <= self.warmup + self.duration + self.max_age:
            raise ValueError("timeout_sec must exceed warmup + bias window + max_age_sec.")
        self.save_callback = save_callback
        self.state = "idle"
        self.last_error = None
        self.last_rejection = None
        self.generation = 0
        self.request_id = 0
        self._biases = None
        self._bias_metadata = None
        self._latest = {}
        self._last_stamps = {}
        self._samples = {sensor: [] for sensor in SENSORS}
        self._request_wall = self._request_mono = None
        self._start_wall = self._end_wall = None
        self._start_mono = self._end_mono = None

    @property
    def busy(self):
        return self.state in ("waiting", "warming", "collecting")

    @property
    def ready(self):
        return self._biases is not None

    @property
    def biases(self):
        if self._biases is None:
            return None
        return {sensor: values.copy() for sensor, values in self._biases.items()}

    @property
    def sample_counts(self):
        return {sensor: len(values) for sensor, values in self._samples.items()}

    def status(self):
        return {
            "state": self.state, "ready": self.ready, "busy": self.busy,
            "generation": self.generation, "sample_counts": self.sample_counts,
            "last_error": self.last_error,
            "request_id": self.request_id, "bias_time_sec": self.duration,
            "warmup_sec": self.warmup, "timeout_sec": self.timeout,
            "biases": (None if self._biases is None else {
                str(sensor): values.tolist() for sensor, values in self._biases.items()
            }),
            "bias_metadata": copy.deepcopy(self._bias_metadata),
        }

    def request_bias(self, now_wall, now_monotonic):
        if self.busy:
            raise RuntimeError("A shared bias calibration is already in progress.")
        if not math.isfinite(now_wall) or not math.isfinite(now_monotonic):
            raise ValueError("Calibration request times must be finite.")
        self.request_id += 1
        self._request_wall = float(now_wall)
        self._request_mono = float(now_monotonic)
        self._start_wall = self._end_wall = None
        self._start_mono = self._end_mono = None
        self._samples = {sensor: [] for sensor in SENSORS}
        # A deliberate new tare also recovers from corrected/backwards host
        # clocks and requires new packets from all sources before warmup.
        self._latest.clear()
        self._last_stamps.clear()
        self.last_error = None
        self.state = "waiting"

    def ingest(self, sensor, values, stamp, now_wall, now_monotonic):
        """Validate one source sample; return its corrected vector when ready."""
        self.last_rejection = None
        if sensor not in SENSORS:
            raise ValueError("Sensor must be 1, 2, or 3.")
        try:
            values = np.asarray(values, dtype=np.float64)
            stamp = float(stamp)
            valid = (values.shape == (6,) and np.isfinite(values).all()
                     and math.isfinite(stamp) and stamp > 0
                     and math.isfinite(now_wall) and math.isfinite(now_monotonic))
        except (TypeError, ValueError, OverflowError):
            valid = False
        if not valid:
            self.last_rejection = f"sensor{sensor}: invalid values or timestamp"
            return None
        if abs(now_wall - stamp) > self.max_age:
            self.last_rejection = f"sensor{sensor}: stale/future source timestamp; check clock synchronization"
            return None
        previous = self._last_stamps.get(sensor)
        if previous is not None and stamp <= previous:
            if stamp == previous:
                self.last_rejection = (
                    f"sensor{sensor}: duplicate source timestamp {stamp:.9f}; "
                    "check acquisition clock resolution and duplicate publishers"
                )
            else:
                self.last_rejection = (
                    f"sensor{sensor}: out-of-order source timestamp {stamp:.9f} "
                    f"< {previous:.9f} (backwards by {(previous - stamp) * 1000:.3f} ms); "
                    "check source clock changes and duplicate publishers"
                )
            return None
        self._last_stamps[sensor] = stamp
        self._latest[sensor] = (stamp, float(now_monotonic))
        if self.busy:
            if (self._start_wall is not None
                    and self._start_wall <= stamp <= self._end_wall
                    and now_monotonic - self._request_mono < self.timeout):
                self._samples[sensor].append(np.r_[stamp, values])
            return None
        if self.ready:
            return values - self._biases[sensor]
        return None

    def _fail(self, message):
        self.last_error = message
        self.state = "failed"
        return "failed"

    def tick(self, now_wall, now_monotonic):
        """Advance the shared calibration without blocking source subscriptions."""
        if not self.busy:
            return None
        if not math.isfinite(now_wall) or not math.isfinite(now_monotonic):
            return self._fail("Local clock is invalid; repeat calibration after clock synchronization.")
        elapsed = now_monotonic - self._request_mono
        if abs((now_wall - self._request_wall) - elapsed) > self.max_age:
            return self._fail("Local system clock changed during calibration; repeat the bias calibration.")
        if elapsed >= self.timeout:
            return self._fail(f"Bias calibration timed out; valid sample counts: {self.sample_counts}.")
        if self.state == "waiting":
            all_fresh = all(
                sensor in self._latest
                and abs(now_wall - self._latest[sensor][0]) <= self.max_age
                and 0 <= now_monotonic - self._latest[sensor][1] <= self.max_gap
                for sensor in SENSORS
            )
            if all_fresh:
                self._start_wall = now_wall + self.warmup
                self._end_wall = self._start_wall + self.duration
                self._start_mono = now_monotonic + self.warmup
                self._end_mono = self._start_mono + self.duration
                self.state = "warming" if self.warmup else "collecting"
                return "started"
            return None
        if self.state == "warming" and now_monotonic >= self._start_mono:
            self.state = "collecting"
            return "collecting"
        # Allow packets from the end of the shared source-time window to arrive.
        if now_monotonic < self._end_mono + self.max_age:
            return None
        samples = {sensor: np.asarray(rows, dtype=float) for sensor, rows in self._samples.items()}
        for sensor, rows in samples.items():
            if len(rows) < self.min_samples:
                return self._fail(f"sensor{sensor}: only {len(rows)} valid samples; need {self.min_samples}.")
            stamps = rows[:, 0]
            if (stamps[0] - self._start_wall > self.max_gap
                    or self._end_wall - stamps[-1] > self.max_gap
                    or np.any(np.diff(stamps) > self.max_gap)):
                return self._fail(f"sensor{sensor}: tracking the bias window failed due to a source data gap.")
        with np.errstate(over="ignore", invalid="ignore"):
            biases = {sensor: rows[:, 1:].mean(axis=0) for sensor, rows in samples.items()}
        if not all(np.isfinite(values).all() for values in biases.values()):
            return self._fail("Bias calculation produced invalid values.")
        metadata = {
            "data_kind": "calibrated_uncorrected_wrench", "force_unit": "N", "torque_unit": "N-mm",
            "bias_time_sec": self.duration, "warmup_sec": self.warmup,
            "window_start_unix_sec": self._start_wall, "window_end_unix_sec": self._end_wall,
            "elapsed_sec": elapsed,
            "sample_counts": {str(sensor): len(rows) for sensor, rows in samples.items()},
            "timestamp_semantics": "source_host_read_completion",
        }
        try:
            if self.save_callback is not None:
                self.save_callback(samples, biases, metadata)
        except Exception as exc:
            return self._fail(f"Saving the shared bias failed: {exc}")
        self._biases = {sensor: values.copy() for sensor, values in biases.items()}
        self._bias_metadata = copy.deepcopy(metadata)
        self.generation += 1
        self.state = "ready"
        return "completed"
