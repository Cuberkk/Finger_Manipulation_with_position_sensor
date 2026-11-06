"""ROS acquisition and loss-aware trial storage, independent of Tk's redraw rate."""
from __future__ import annotations

import csv
from copy import deepcopy
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
from numbers import Real
from pathlib import Path
import queue
import re
import threading
import time
from collections import deque

import numpy as np

from utils.gui.force_history import ForceHistory, ForceHistorySample
from utils.gui.force_release import total_force_magnitude
from utils.gui.study_model import normalize_quaternion

FINGERS = ("thumb", "index", "middle")
SENSORS = {"thumb": 1, "index": 3, "middle": 2}
RAW_TOPICS = {f: f"/ERIE_Manipulation/acquisition/wrench_s{s}_uncorrected" for f, s in SENSORS.items()}
FORCE_TOPICS = {f: f"/ERIE_Manipulation/force/force_s{s}_finger{s}" for f, s in SENSORS.items()}
FORCE_HEADER = ("source_timestamp_sec", "Fx", "Fy", "Fz")
FT_HEADER = ("source_timestamp_sec", "Fx", "Fy", "Fz", "Tx", "Ty", "Tz")
AXIS_ERROR_HEADER = (
    "source_timestamp_sec", "elapsed_sec", "step_rotation_deg", "step_angle_deg",
    "accumulated_angle_deg", "axis_x", "axis_y", "axis_z", "axis_error_deg",
    "continuous", "status",
)
STALE_SEC = 0.5
FORCE_BIAS_SERVICE = "/ERIE_Manipulation/force/rebias"
FORCE_BIAS_STATUS_TOPIC = "/ERIE_Manipulation/force/processing_status"
FORCE_BIAS_STATUS_MAX_AGE = 2.0
FORCE_BIAS_ACK_TIMEOUT = 3.0


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def _bias_vectors(values):
    """Validate all three sensor offsets and return independent numeric lists."""
    if values is None:
        return None
    if not isinstance(values, dict) or set(values) != {"1", "2", "3"}:
        raise ValueError("Force bias must include all three sensor offsets.")
    result = {}
    for sensor, vector in values.items():
        if (not isinstance(vector, (list, tuple)) or len(vector) != 6
                or any(type(value) not in (int, float) or not math.isfinite(value)
                       for value in vector)):
            raise ValueError("Force-bias offsets must contain six finite numbers per sensor.")
        result[sensor] = [float(value) for value in vector]
    return result


def _bias_metadata(value):
    if value is not None and not isinstance(value, dict):
        raise ValueError("Force-bias calibration metadata must be an object.")
    # Status arrives as JSON. Reject NaN/Infinity anywhere in the provenance,
    # and detach nested containers before exposing them to callers.
    return json.loads(json.dumps(value, allow_nan=False))


def force_bias_record(snapshot):
    """Capture an archival copy of the active, freshly confirmed sensor offsets."""
    if (not isinstance(snapshot, dict) or not snapshot.get("available")
            or not snapshot.get("ready") or snapshot.get("pending") or snapshot.get("busy")):
        raise ValueError("A fresh, completed force bias is required before recording.")
    vectors = _bias_vectors(snapshot.get("biases"))
    if vectors is None:
        raise ValueError("Current force-bias offsets are unavailable; update the central processor and retry.")
    if (not isinstance(snapshot.get("processor_id"), str) or not snapshot["processor_id"]
            or type(snapshot.get("generation")) is not int or snapshot["generation"] < 1):
        raise ValueError("Current force-bias identity is unavailable.")
    for field in ("request_id", "status_seq"):
        if type(snapshot.get(field)) is not int or snapshot[field] < 0:
            raise ValueError("Current force-bias transaction identity is unavailable.")
    return {
        "schema_version": 1, "captured_utc": utc_now(),
        **{field: snapshot[field] for field in ("processor_id", "generation", "request_id", "status_seq")},
        "biases": vectors, "bias_metadata": _bias_metadata(snapshot.get("bias_metadata")),
        "fields": list(FT_HEADER[1:]), "units": {"force": "N", "torque": "N·mm"},
        "coordinate_frame": "ati_sensor", "sensors": dict(SENSORS),
        "correction": "corrected = uncorrected - bias",
    }


def validate_user(value):
    value = str(value).strip()
    if not re.fullmatch(r"[A-Za-z0-9_-]{1,32}", value):
        raise ValueError("User number must contain 1–32 letters, digits, underscores or hyphens.")
    return value


def unique_directory(parent, stem):
    """Reserve a directory atomically; repeat trials never overwrite earlier data."""
    parent = Path(parent)
    parent.mkdir(parents=True, exist_ok=True)
    for attempt in range(10000):
        path = parent / (stem if not attempt else f"{stem}_{attempt + 1:02d}")
        try:
            path.mkdir()
            return path
        except FileExistsError:
            continue
    raise FileExistsError(f"Too many repeated folders for {stem}")


def create_session(root, user, when):
    """Reuse a participant root so repeated runs share their trial archive."""
    identifier = validate_user(user)
    if identifier.isdigit():
        identifier = str(int(identifier)).zfill(3)
    path = Path(root) / f"User{identifier}"
    path.mkdir(parents=True, exist_ok=True)
    return path


_TRIAL_DIRECTORY_LOCK = threading.RLock()
_ATTEMPT_TIMESTAMP = re.compile(r"\d{8}_\d{4}(?:\d{2})?(?:_\d{6})?(?:_\d{2,})?")


def trial_reference(trial):
    """Return the local trial number within one size/task, or Practice."""
    if trial.practice:
        return "Practice"
    if type(trial.repetition) is not int or not 0 <= trial.repetition < 5:
        raise ValueError("Measured trials require a repetition number from 0 to 4.")
    return str(trial.repetition + 1)


def _task_directory(session, trial):
    folder = {"roll": "Roll", "pitch": "Pitch", "yaw": "Yaw", "finger_gating": "FingerGaiting"}[trial.task]
    return Path(session) / f"{trial.diameter_mm}mm" / folder


def _attempt_stem(reference):
    return "Practice" if reference == "Practice" else f"Trial{reference}"


def _unused_archive_path(parent, stem):
    # All normal GUI recordings use the same lock. An existing directory, file,
    # or dangling symlink always occupies a name and must remain untouched.
    for attempt in range(10000):
        path = parent / (stem if not attempt else f"{stem}_{attempt + 1:02d}")
        if not path.exists() and not path.is_symlink():
            return path
    raise FileExistsError(f"Too many archived folders for {stem}")


def _restore_archived(moved):
    errors = []
    for original, archived in reversed(moved):
        try:
            if original.exists() or original.is_symlink():
                raise FileExistsError(f"Original path is occupied: {original}")
            archived.rename(original)
        except OSError as exc:
            errors.append(f"{archived}: {exc}")
    return errors


def archive_trial_directories(parent, reference):
    """Move previous attempts of one local trial into Archive, without deleting.

    Archive names keep their recording timestamp. If a prior run already used
    that name, append a unique suffix. On a move failure, restore earlier moves
    where possible; any unrecovered directory remains intact in Archive.
    """
    parent = Path(parent)
    prefix = _attempt_stem(str(reference)) + "_"
    with _TRIAL_DIRECTORY_LOCK:
        archive = parent / "Archive"
        archive.mkdir(parents=True, exist_ok=True)
        previous = sorted(
            path for path in parent.iterdir()
            if not path.is_symlink() and path.is_dir() and path.name.startswith(prefix)
            and _ATTEMPT_TIMESTAMP.fullmatch(path.name[len(prefix):])
        )
        moved = []
        try:
            for original in previous:
                destination = _unused_archive_path(archive, original.name)
                original.rename(destination)
                moved.append((original, destination))
        except OSError as exc:
            errors = _restore_archived(moved)
            detail = "; rollback incomplete; preserved archive paths: " + "; ".join(errors) if errors else ""
            raise OSError(f"Could not archive existing trial data{detail}") from exc
        return tuple(moved)


def create_trial_directory(session, user, trial, when):
    """Archive the prior local trial and reserve its new recording directory."""
    validate_user(user)
    reference = trial_reference(trial)
    parent = _task_directory(session, trial)
    with _TRIAL_DIRECTORY_LOCK:
        moved = archive_trial_directories(parent, reference)
        try:
            return unique_directory(parent, f"{_attempt_stem(reference)}_{when:%Y%m%d_%H%M%S_%f}")
        except OSError as exc:
            errors = _restore_archived(moved)
            detail = "; rollback incomplete; preserved archive paths: " + "; ".join(errors) if errors else ""
            raise OSError(f"Could not create the new trial directory{detail}") from exc


def create_calibration_directory(session, user, trial, when):
    """Reserve a rotation-range recording separate from measured trial data."""
    validate_user(user)
    parent = _task_directory(session, trial)
    (parent / "Archive").mkdir(parents=True, exist_ok=True)
    return unique_directory(
        parent / "RotationCalibration", f"{when:%Y%m%d_%H%M%S_%f}",
    )


@dataclass(frozen=True)
class PoseSample:
    quaternion: tuple
    source_timestamp: float
    arrival: float


class TrialWriter:
    """Write all incoming samples on one worker, with explicit overflow/error reporting."""
    recording_enabled = True

    def __init__(self, path, metadata, queue_size=150000):
        self.path = Path(path)
        self.metadata = deepcopy(metadata)
        self.metadata["finger_force_schema"] = {
            "schema_version": 1, "calibrated": True, "bias_applied": True,
            "coordinate_frame": "finger", "fields": list(FORCE_HEADER),
            "units": {"force": "N", "timestamp": "Unix seconds"},
            "timestamp_event": "acquisition host read completion",
            "topics": dict(FORCE_TOPICS), "sensors": dict(SENSORS),
        }
        self.metadata["raw_force_schema"] = {
            "schema_version": 2, "calibrated": True, "bias_applied": False,
            "coordinate_frame": "ati_sensor", "fields": list(FT_HEADER),
            "units": {"force": "N", "torque": "N·mm", "timestamp": "Unix seconds"},
            "timestamp_event": "acquisition host read completion",
            "topics": dict(RAW_TOPICS), "sensors": dict(SENSORS),
        }
        self.queue = queue.Queue(maxsize=queue_size)
        self.lock = threading.Lock()
        self.ready = threading.Event()
        self.finished = threading.Event()
        self.stopping = threading.Event()
        self.accepting = True
        self.error = None
        self.counts = {f: 0 for f in FINGERS}
        self.raw_counts = {f: 0 for f in FINGERS}
        self.pose_count = 0
        self.overflows = 0
        self.result = "recording"
        self.thread = threading.Thread(target=self._run, daemon=True, name="study-csv-writer")
        self.thread.start()

    def add(self, kind, finger, values):
        with self.lock:
            if (not self.accepting or kind not in ("force", "raw", "pose", "progress", "axis_error")
                    or (kind not in ("force", "raw") and finger is not None)):
                return False
            if kind in ("force", "raw") and (
                    finger not in self.counts or len(values) != (4 if kind == "force" else 7)):
                return False
            try:
                self.queue.put_nowait((kind, finger, tuple(values)))
                return True
            except queue.Full:
                self.overflows += 1
                return False

    def finish(self, result, extra=None):
        with self.lock:
            if self.stopping.is_set() or self.finished.is_set():
                return
            self.accepting = False
            self.result = result
            self.metadata.update(extra or {})
            self.stopping.set()

    def wait(self):
        """Wait for accepted rows, file closes, and final metadata after finish().

        Shutdown must not use a timeout here: abandoning this daemon worker can
        otherwise leave buffered rows or the trial summary unwritten.
        """
        self.thread.join()

    def _run(self):
        files = {}
        writers = {}
        metadata_stream = None
        try:
            # Claim metadata before creating CSVs. A second writer must never
            # replace a previous trial's metadata, even when CSV creation fails.
            metadata_stream = (self.path / "trial_metadata.json").open("x", encoding="utf-8")
            with self.lock:
                initial_metadata = dict(self.metadata)
            metadata_stream.write(json.dumps(initial_metadata, indent=2))
            metadata_stream.flush()
            if initial_metadata.get("force_bias") is not None:
                # Immutable start-of-recording offsets live beside the raw CSVs.
                # Never append taring updates or rewrite this file at trial end.
                with (self.path / "force_bias.json").open("x", encoding="utf-8") as stream:
                    json.dump(initial_metadata["force_bias"], stream, indent=2, allow_nan=False)
                    stream.write("\n")
            specs = [(('force', f), f"{f}.csv", FORCE_HEADER) for f in FINGERS]
            specs += [(('raw', f), f"{f}_raw_force.csv", FT_HEADER) for f in FINGERS]
            specs += [(("pose", None), "object_orientation.csv", ("source_timestamp_sec", "qx", "qy", "qz", "qw")),
                      (("progress", None), "rotation_progress.csv", ("source_timestamp_sec", "elapsed_sec", "angle_deg", "completed_cycles", "phase")),
                      (("axis_error", None), "rotation_axis_error.csv", AXIS_ERROR_HEADER)]
            for key, name, header in specs:
                stream = (self.path / name).open("x", newline="", buffering=65536)
                files[key] = stream
                writers[key] = csv.writer(stream)
                writers[key].writerow(header)
                stream.flush()
            self.ready.set()
            last_flush = time.monotonic()
            while True:
                try:
                    kind, finger, values = self.queue.get(timeout=0.05)
                except queue.Empty:
                    if self.stopping.is_set():
                        break
                    continue
                writers[(kind, finger)].writerow(values)
                if kind == "force":
                    self.counts[finger] += 1
                elif kind == "raw":
                    self.raw_counts[finger] += 1
                elif kind == "pose":
                    self.pose_count += 1
                if time.monotonic() - last_flush > 1.0:
                    for stream in files.values():
                        stream.flush()
                    last_flush = time.monotonic()
        except Exception as exc:
            self.error = repr(exc)
        finally:
            with self.lock:
                self.accepting = False
                self.stopping.set()
            for stream in files.values():
                try:
                    stream.flush()
                except Exception as exc:
                    self.error = self.error or repr(exc)
                finally:
                    try:
                        stream.close()
                    except Exception as exc:
                        self.error = self.error or repr(exc)
            try:
                summary = {"result": "write_error" if self.error else self.result,
                           "finished_utc": utc_now(), "recorded_counts": dict(self.counts),
                           "raw_recorded_counts": dict(self.raw_counts), "orientation_samples": self.pose_count,
                           "writer_queue_overflows": self.overflows, "write_error": self.error}
                self.metadata.update(summary)
                if metadata_stream is not None:
                    try:
                        payload = json.dumps(self.metadata, indent=2)
                    except (TypeError, ValueError, OverflowError) as exc:
                        self.error = self.error or repr(exc)
                        summary.update(result="write_error", write_error=self.error)
                        self.metadata.update(summary)
                        # Preserve a readable failure report if caller metadata
                        # contains a value that JSON cannot represent.
                        payload = json.dumps(summary, indent=2)
                    metadata_stream.seek(0)
                    metadata_stream.write(payload)
                    metadata_stream.truncate()
                    metadata_stream.flush()
            except Exception as exc:
                self.error = self.error or repr(exc)
            finally:
                if metadata_stream is not None:
                    try:
                        metadata_stream.close()
                    except Exception as exc:
                        self.error = self.error or repr(exc)
                if self.error:
                    self.metadata.update(result="write_error", write_error=self.error)
                # Always release the GUI's startup/completion gates, including
                # metadata serialization, flush, and close failures.
                self.ready.set()
                self.finished.set()


class PreviewWriter:
    """Track a measurement lifecycle without writing or retaining its samples."""
    recording_enabled = False

    def __init__(self, metadata):
        self.path = None
        self.metadata = dict(metadata)
        self.metadata["data_recorded"] = False
        self.lock = threading.Lock()
        self.ready = threading.Event()
        self.finished = threading.Event()
        self.stopping = threading.Event()
        self.accepting = True
        self.error = None
        self.counts = {finger: 0 for finger in FINGERS}
        self.raw_counts = {finger: 0 for finger in FINGERS}
        self.pose_count = 0
        self.overflows = 0
        self.result = "recording"
        self.ready.set()

    def add(self, kind, finger, values):
        with self.lock:
            if not self.accepting:
                return False
            if kind in ("force", "raw") and (
                    finger not in self.counts or len(values) != (4 if kind == "force" else 7)):
                return False
            if kind == "force" and finger in self.counts:
                self.counts[finger] += 1
            elif kind == "raw" and finger in self.raw_counts:
                self.raw_counts[finger] += 1
            elif kind == "pose" and finger is None:
                self.pose_count += 1
            elif kind not in ("progress", "axis_error") or finger is not None:
                return False
            return True

    def finish(self, result, extra=None):
        with self.lock:
            if self.stopping.is_set() or self.finished.is_set():
                return
            self.accepting = False
            self.stopping.set()
            self.result = result
            self.metadata.update(extra or {})
            self.metadata.update(
                result=result, finished_utc=utc_now(), data_recorded=False,
                recorded_counts=dict(self.counts), raw_recorded_counts=dict(self.raw_counts),
                orientation_samples=self.pose_count, writer_queue_overflows=0, write_error=None,
            )
            self.finished.set()

    def wait(self):
        """Preview completion is synchronous and owns no worker to join."""


class ROSBridge:
    """Own ROS callbacks on a worker; Tk reads snapshots and bounded histories."""
    def __init__(self, demo, events, pose_rate=60.0, *, release_force_threshold_n=0.5):
        if isinstance(release_force_threshold_n, bool) or not isinstance(release_force_threshold_n, Real):
            raise ValueError("Release force threshold must be a finite positive number in N.")
        try:
            release_force_threshold_n = float(release_force_threshold_n)
        except (ValueError, OverflowError) as exc:
            raise ValueError("Release force threshold must be a finite positive number in N.") from exc
        if not math.isfinite(release_force_threshold_n) or release_force_threshold_n <= 0:
            raise ValueError("Release force threshold must be a finite positive number in N.")
        self.demo = demo
        self.events = events
        self.pose_rate = pose_rate
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.values = {(kind, f): None for kind in ("raw", "force") for f in FINGERS}
        self.times = {key: 0.0 for key in self.values}
        self.rates = {key: deque(maxlen=120) for key in self.values}
        self.invalid = {key: 0 for key in self.values}
        self.gaps = {key: 0 for key in self.values}
        self.force_history = ForceHistory(
            (kind, finger) for kind in ("raw", "force") for finger in FINGERS
        )
        self.pose = None
        self.poses = deque(maxlen=3000)
        self.pose_drops = 0
        self.writer = None
        self.release_force_threshold_n = release_force_threshold_n
        self._force_release_events = deque(maxlen=4096)
        self._force_release_drops = 0
        self._force_release_state = None
        self._force_release_last_arrival = None
        self.error = None
        self.demo_axis = "x"
        self.demo_target = 30.0
        self.demo_motion_start = None
        self.demo_alignment = np.eye(3)
        self.demo_baseline = np.array([0.0, 0.0, 0.0, 1.0])
        self._force_bias_status = None
        self._force_bias_retired_processors = deque(maxlen=16)
        self._force_bias_arrival = None
        self._force_bias_service_ready = False
        self._force_bias_pending = False
        self._force_bias_queued = False
        self._force_bias_sent_at = None
        self._force_bias_baseline = None
        self._force_bias_expected = None
        self._force_bias_acknowledged = False
        self._force_bias_ack_status_seq = None
        self._force_bias_uncertain = False
        self._force_bias_error = None
        self._force_bias_future = None  # Accessed only by the ROS worker.
        self.thread = threading.Thread(target=self._run, daemon=True, name="study-ros")
        self.thread.start()

    def sample_force(self, kind, finger, data, arrival=None):
        arrival = time.monotonic() if arrival is None else arrival
        key = kind, finger
        if key not in self.values:
            return
        required = 4 if kind == "force" else 7
        try:
            values = tuple(float(v) for v in data)
            if len(values) != required or not all(math.isfinite(v) for v in values):
                raise ValueError("Unexpected force message")
        except (ValueError, TypeError, OverflowError):
            with self.lock:
                self.invalid[key] += 1
            return
        with self.lock:
            old = self.times[key]
            if old and arrival - old > STALE_SEC:
                self.gaps[key] += 1
            self.values[key] = values
            self.times[key] = arrival
            self.rates[key].append(arrival)
            self.force_history.append(
                key, ForceHistorySample(arrival, values[-1], values[:-1])
            )
            if self.writer:
                self.writer.add(kind, finger, (values[-1], *values[:-1]))
                if kind == "force":
                    self._observe_force_release_locked(arrival)

    def _observe_force_release_locked(self, arrival):
        """Latch force transitions at receipt time, independent of GUI redraws."""
        if self.writer is None:
            return
        fingers = {finger: {"value": self.values["force", finger],
                            "arrival": self.times["force", finger]} for finger in FINGERS}
        force_sum = total_force_magnitude(fingers, arrival, max_age_sec=STALE_SEC)
        if force_sum is None:
            return
        released = force_sum <= self.release_force_threshold_n
        if released == self._force_release_state:
            return
        self._force_release_state = released
        # A callback can capture its receipt time before waiting for begin()'s
        # lock. Keep transitions ordered without changing sample/CSV timestamps.
        event_arrival = (arrival if self._force_release_last_arrival is None else
                         max(arrival, self._force_release_last_arrival))
        self._force_release_last_arrival = event_arrival
        if len(self._force_release_events) == self._force_release_events.maxlen:
            self._force_release_drops += 1
        self._force_release_events.append({
            "arrival": event_arrival,
            "released": released,
            "force_sum_magnitude_n": force_sum,
            "source_timestamp_sec": max(item["value"][-1] for item in fingers.values()),
        })

    def drain_force_release_events(self):
        """Detach recorded force transitions and cumulative queue losses."""
        with self.lock:
            events = list(self._force_release_events)
            self._force_release_events.clear()
            return {"events": events, "drops": self._force_release_drops}

    def drain_recording_updates(self):
        """Detach poses and grip transitions at one callback-lock boundary."""
        with self.lock:
            updates = {"poses": list(self.poses), "events": list(self._force_release_events),
                       "drops": self._force_release_drops}
            self.poses.clear()
            self._force_release_events.clear()
            return updates

    def sample_pose(self, quaternion, source_timestamp, arrival=None):
        arrival = time.monotonic() if arrival is None else arrival
        try:
            quaternion = tuple(normalize_quaternion(quaternion))
            if not math.isfinite(source_timestamp):
                return
        except (ValueError, TypeError, OverflowError):
            return
        sample = PoseSample(quaternion, float(source_timestamp), arrival)
        with self.lock:
            self.pose = sample
            if len(self.poses) == self.poses.maxlen:
                self.pose_drops += 1
            self.poses.append(sample)
            if self.writer:
                self.writer.add("pose", None, (source_timestamp, *quaternion))

    def snapshot(self):
        with self.lock:
            fingers = {}
            for finger in FINGERS:
                key = "force", finger
                arrivals = self.rates[key]
                rate = (len(arrivals) - 1) / (arrivals[-1] - arrivals[0]) if len(arrivals) > 1 and arrivals[-1] > arrivals[0] else 0.0
                fingers[finger] = {"value": self.values[key], "arrival": self.times[key], "rate": rate}
            return {"fingers": fingers, "pose": self.pose, "pose_drops": self.pose_drops,
                    "force_release_events_supported": True,
                    "error": self.error, "force_bias": self._force_bias_snapshot_locked(time.monotonic())}

    def force_history_snapshot(self, now=None):
        """Copy the rolling plot history separately from frequent status reads."""
        return self.force_history.snapshot(now)

    def _force_bias_snapshot_locked(self, now):
        status = self._force_bias_status
        fresh = (status is not None and self._force_bias_arrival is not None
                 and 0 <= now - self._force_bias_arrival <= FORCE_BIAS_STATUS_MAX_AGE)
        stopped = self.stop_event.is_set()
        available = bool(not self.demo and not stopped and fresh and self._force_bias_service_ready)
        busy = bool(status and status["busy"] and not self.demo and not stopped)
        pending = self._force_bias_pending and not stopped
        state = status["state"] if status else "unavailable"
        message = "Central force processing is unavailable. Start ATI_processing_node."
        if self.demo:
            state, message = "disabled", "Force bias is unavailable in demo mode."
        elif stopped:
            state, message = "disabled", "Force bias is unavailable after shutdown."
        elif pending:
            if self._force_bias_uncertain:
                state = "uncertain"
                message = ("Force-bias confirmation was lost; calibration may still be running. "
                           "Waiting for current processing status. If confirmation does not recover, "
                           "restart the central processor and GUI; acquisition nodes can stay running.")
            elif self._force_bias_queued:
                state, message = "requesting", "Force-bias request queued."
            elif not self._force_bias_acknowledged:
                state, message = "requesting", "Requesting force bias; keep all sensors unloaded and still."
            else:
                message = "Force bias is running; keep all sensors unloaded and still."
            if not fresh:
                message += " Processing status is missing; recording remains blocked."
        elif status and not available:
            state, message = "stale", "Force processing is disconnected or its status is stale."
        elif available:
            message = {"idle": "Ready to request force bias.",
                       "waiting": "Waiting for fresh readings from all three sensors.",
                       "warming": "Force-bias warmup; keep all sensors unloaded and still.",
                       "collecting": "Collecting force bias; keep all sensors unloaded and still.",
                       "ready": "Force bias completed; all three sensor offsets are active.",
                       "failed": "Force bias failed; check the error and retry."}[state]
        error = self._force_bias_error or (status["last_error"] if status and not pending else None)
        if error and not pending:
            message = error
        return {"available": available, "pending": bool(pending), "busy": busy, "state": state,
                "ready": bool(status and status["ready"] and available and not busy and not pending),
                "generation": status["generation"] if status else None,
                "processor_id": status["processor_id"] if status else None,
                "request_id": status["request_id"] if status else None,
                "status_seq": status["status_seq"] if status else None,
                "sample_counts": dict(status["sample_counts"]) if status else {},
                "biases": deepcopy(status["biases"]) if status else None,
                "bias_metadata": deepcopy(status["bias_metadata"]) if status else None,
                "last_error": error, "message": message}

    def request_force_bias(self):
        """Queue a tare without calling ROS from Tk's thread or blocking it."""
        with self.lock:
            status = self._force_bias_snapshot_locked(time.monotonic())
            if (not status["available"] or status["pending"] or status["busy"]
                    or self.writer is not None):
                return False
            self._force_bias_pending = self._force_bias_queued = True
            self._force_bias_acknowledged = self._force_bias_uncertain = False
            self._force_bias_ack_status_seq = None
            self._force_bias_error = None
            self._force_bias_sent_at = self._force_bias_baseline = self._force_bias_expected = None
            return True

    @staticmethod
    def _parse_force_bias_status(data):
        status = json.loads(data)
        if not isinstance(status, dict):
            raise ValueError("Force-bias status must be an object")
        state = status.get("state")
        if state not in ("idle", "waiting", "warming", "collecting", "ready", "failed"):
            raise ValueError("Unknown force-bias state")
        if type(status.get("busy")) is not bool or type(status.get("ready")) is not bool:
            raise ValueError("Invalid force-bias flags")
        if status["busy"] != (state in ("waiting", "warming", "collecting")):
            raise ValueError("Inconsistent force-bias phase")
        if state == "ready" and not status["ready"]:
            raise ValueError("Completed bias is not ready")
        for field in ("generation", "request_id", "status_seq"):
            if type(status.get(field)) is not int or status[field] < 0:
                raise ValueError("Invalid force-bias transaction identifier")
        if not isinstance(status.get("processor_id"), str) or not status["processor_id"]:
            raise ValueError("Missing force-processing instance identifier")
        counts = status.get("sample_counts")
        if (not isinstance(counts, dict) or set(counts) != {"1", "2", "3"}
                or any(type(value) is not int or value < 0 for value in counts.values())):
            raise ValueError("Invalid force-bias sample counts")
        if status.get("last_error") is not None and not isinstance(status["last_error"], str):
            raise ValueError("Invalid force-bias error")
        return {**status, "sample_counts": dict(counts), "last_error": status.get("last_error"),
                "biases": _bias_vectors(status.get("biases")),
                "bias_metadata": _bias_metadata(status.get("bias_metadata"))}

    def sample_force_bias_status(self, data, arrival=None):
        """Accept ordered, identified status heartbeats from the processing node."""
        arrival = time.monotonic() if arrival is None else arrival
        try:
            status = self._parse_force_bias_status(data)
        except (ValueError, TypeError, OverflowError):
            return False
        with self.lock:
            old = self._force_bias_status
            if status["processor_id"] in self._force_bias_retired_processors:
                return False
            if old and old["processor_id"] == status["processor_id"]:
                if (status["status_seq"] <= old["status_seq"]
                        or status["request_id"] < old["request_id"]
                        or status["generation"] < old["generation"]):
                    return False
            elif old:
                self._force_bias_retired_processors.append(old["processor_id"])
                if self._force_bias_pending:
                    self._force_bias_pending = self._force_bias_queued = False
                    self._force_bias_error = "Force processor restarted; request force bias again."
            self._force_bias_status = status
            self._force_bias_arrival = arrival
            self._resolve_force_bias_locked(arrival)
        return True

    def _resolve_force_bias_locked(self, now=None):
        now = time.monotonic() if now is None else now
        status, baseline = self._force_bias_status, self._force_bias_baseline
        if (not self._force_bias_pending or self._force_bias_sent_at is None
                or status is None or baseline is None
                or status["status_seq"] <= baseline["status_seq"]
                or self._force_bias_arrival is None
                or not 0 <= now - self._force_bias_arrival <= FORCE_BIAS_STATUS_MAX_AGE
                or not (self._force_bias_acknowledged or self._force_bias_uncertain)):
            return
        expected_processor, expected_request = self._force_bias_expected
        if status["processor_id"] != expected_processor:
            return
        if status["request_id"] > expected_request:
            self._force_bias_pending = False
            self._force_bias_error = "Another force-bias request superseded this request; check the current processing status."
            return
        if (status["request_id"] != expected_request
                or (self._force_bias_ack_status_seq is not None
                    and status["status_seq"] <= self._force_bias_ack_status_seq)):
            return
        if status["busy"]:
            return
        if status["state"] == "ready" and status["generation"] > baseline["generation"]:
            self._force_bias_pending = False
            self._force_bias_error = None
        elif status["state"] == "failed":
            self._force_bias_pending = False
            self._force_bias_error = status["last_error"] or "Force bias failed; check sensor input and retry."

    def _poll_force_bias(self, client, request_factory, now=None):
        """Run service readiness, dispatch and future polling on the ROS worker."""
        now = time.monotonic() if now is None else now
        ready = bool(client.service_is_ready())
        with self.lock:
            self._force_bias_service_ready = ready
            queued = self._force_bias_queued
            if queued:
                self._force_bias_queued = False
                current = self._force_bias_snapshot_locked(now)
                if not current["available"] or current["busy"] or self.stop_event.is_set():
                    self._force_bias_pending = False
                    self._force_bias_error = "Force-bias request was not sent; check the processing node and retry."
                    queued = False
                else:
                    self._force_bias_baseline = dict(self._force_bias_status)
                    self._force_bias_expected = (self._force_bias_status["processor_id"],
                                                 self._force_bias_status["request_id"] + 1)
                    self._force_bias_sent_at = now
        if queued:
            self._cancel_force_bias_future(client)
            try:
                self._force_bias_future = client.call_async(request_factory())
            except Exception as exc:
                with self.lock:
                    self._force_bias_pending = False
                    self._force_bias_error = f"Force-bias request failed: {exc}. Retry after checking ROS."
                return
        future = self._force_bias_future
        if future is None:
            return
        if future.done():
            self._force_bias_future = None
            try:
                response = future.result()
            except Exception as exc:
                with self.lock:
                    if self._force_bias_pending:
                        self._force_bias_uncertain = True
                        self._force_bias_error = f"Force-bias response lost: {exc}. Waiting for processing status."
                        self._resolve_force_bias_locked(now)
                return
            with self.lock:
                if not self._force_bias_pending:
                    return
                if not response.success:
                    self._force_bias_pending = False
                    self._force_bias_error = response.message or "Force-bias request rejected; check the processing node."
                else:
                    try:
                        acknowledgement = json.loads(response.message)
                        processor = acknowledgement["processor_id"]
                        request_id = acknowledgement["request_id"]
                        status_seq = acknowledgement["status_seq"]
                        if (not isinstance(processor, str) or not processor
                                or type(request_id) is not int or request_id < 1
                                or type(status_seq) is not int or status_seq < 1
                                or processor != self._force_bias_baseline["processor_id"]
                                or request_id <= self._force_bias_baseline["request_id"]
                                or status_seq <= self._force_bias_baseline["status_seq"]):
                            raise ValueError("Invalid force-bias acknowledgement")
                        self._force_bias_expected = processor, request_id
                        self._force_bias_ack_status_seq = status_seq
                    except (ValueError, TypeError, KeyError):
                        self._force_bias_uncertain = True
                        self._force_bias_error = "Force-bias response has no transaction ID; waiting for processing status."
                    else:
                        self._force_bias_acknowledged = True
                        self._force_bias_uncertain = False
                        self._force_bias_error = None
                    self._resolve_force_bias_locked(now)
        else:
            with self.lock:
                if (self._force_bias_pending and self._force_bias_sent_at is not None
                        and now - self._force_bias_sent_at >= FORCE_BIAS_ACK_TIMEOUT):
                    self._force_bias_uncertain = True
                    self._force_bias_error = "Force-bias response timed out; waiting for current processing status."
                    self._resolve_force_bias_locked(now)

    def _cancel_force_bias_future(self, client):
        """Release a local service future; this never stops the remote processor."""
        future, self._force_bias_future = self._force_bias_future, None
        if future is not None:
            try:
                future.cancel()
            finally:
                client.remove_pending_request(future)

    def drain_poses(self):
        with self.lock:
            result = list(self.poses)
            self.poses.clear()
            return result

    def counters(self):
        with self.lock:
            return {"invalid": {f"{k[0]}_{k[1]}": v for k, v in self.invalid.items()},
                    "gaps": {f"{k[0]}_{k[1]}": v for k, v in self.gaps.items()}, "pose_drops": self.pose_drops}

    def begin(self, writer, axis, target, alignment=None, baseline=None, expected_bias=None):
        """Begin recording, with demo poses expressed using the real trial frames.

        alignment maps object/mesh axes to sensor4 axes; baseline is the sensor4 xyzw
        quaternion captured at trial start. Omitted frames retain the original
        identity-frame demo behavior. expected_bias binds a live writer to the
        offsets captured in its startup files, checked under the callback lock.
        """
        axis = axis or "z"
        if axis not in ("x", "y", "z"):
            raise ValueError("Demo axis must be x, y, or z.")
        alignment = np.eye(3) if alignment is None else np.array(alignment, dtype=float, copy=True)
        if (alignment.shape != (3, 3) or not np.all(np.isfinite(alignment))
                or not np.allclose(alignment.T @ alignment, np.eye(3), atol=1e-6, rtol=0)
                or not math.isclose(float(np.linalg.det(alignment)), 1.0, abs_tol=1e-6)):
            raise ValueError("Demo alignment must be a proper 3 by 3 rotation matrix.")
        baseline = normalize_quaternion([0, 0, 0, 1] if baseline is None else baseline)
        with self.lock:
            bias_status = self._force_bias_snapshot_locked(time.monotonic())
            if bias_status["pending"] or bias_status["busy"]:
                raise ValueError("Wait for force bias to finish before starting a recording.")
            if expected_bias is not None and not self.demo:
                current = force_bias_record(bias_status)
                if (not isinstance(expected_bias, dict)
                        or any(current[field] != expected_bias.get(field)
                               for field in ("processor_id", "generation", "biases"))):
                    raise ValueError("Force bias changed before recording started; retry to capture the current offsets.")
            self.poses.clear()
            self.writer = writer
            self._force_release_events.clear()
            self._force_release_drops = 0
            self._force_release_state = None
            self._force_release_last_arrival = None
            self._observe_force_release_locked(time.monotonic())
            self.demo_axis = axis
            self.demo_target = target
            self.demo_alignment = alignment
            self.demo_baseline = baseline
            self.demo_motion_start = time.monotonic() if self.demo else None

    def end(self):
        with self.lock:
            writer = self.writer
            self.writer = None
            self.demo_motion_start = None
            return writer

    def end_with_poses(self):
        """Detach the writer and return its remaining pose history atomically.

        A separate end()/drain_poses() pair can include a pose arriving after the
        recording boundary. The owner consumes these final recorded poses before
        closing the progress CSV and computing the trial summary. Pending force
        transitions remain frozen for drain_force_release_events(); callbacks
        after writer detachment cannot add events beyond this same boundary.
        """
        with self.lock:
            writer = self.writer
            self.writer = None
            self.demo_motion_start = None
            poses = list(self.poses)
            self.poses.clear()
            return writer, poses

    def _run_demo(self):
        while not self.stop_event.is_set():
            now, stamp = time.monotonic(), time.time()
            for index, finger in enumerate(FINGERS):
                values = (2.0 * math.sin(now * 1.3 + index), 1.5 * math.cos(now + index),
                          5.0 + 2.0 * math.sin(now * .6 + index), 35.0 * math.sin(now + index),
                          18.0 * math.cos(now * .8 + index), 28.0 * math.sin(now * .5 + index), stamp)
                self.sample_force("force", finger, (*values[:3], stamp), now)
                self.sample_force("raw", finger, values, now)
            with self.lock:
                start, axis, target = self.demo_motion_start, self.demo_axis, self.demo_target
                alignment, baseline = self.demo_alignment, self.demo_baseline
            q = np.array([0.0, 0.0, 0.0, 1.0])
            if start is not None:
                phase = (now - start) % 6.0
                angle = float(np.interp(phase, [0, .4, 1.4, 1.8, 2.8, 3.2, 4.2, 4.6, 5.6, 6.0],
                                        [0, 0, -1.2 * target, -1.2 * target, 0, 0, 1.2 * target, 1.2 * target, 0, 0]))
                # Rt = R0 @ A @ Rtask @ A.T. A rotates the task axis into
                # sensor coordinates, then q0 * qrelative applies the baseline.
                half_angle = math.radians(angle) / 2
                vector = alignment[:, "xyz".index(axis)] * math.sin(half_angle)
                scalar = math.cos(half_angle)
                q[:3] = (baseline[3] * vector + scalar * baseline[:3]
                         + np.cross(baseline[:3], vector))
                q[3] = baseline[3] * scalar - np.dot(baseline[:3], vector)
            self.sample_pose(q, stamp, now)
            self.stop_event.wait(.01)

    def _run(self):
        if self.demo:
            self._run_demo()
            return
        context = node = executor = bias_client = None
        external_shutdown = ()
        try:
            import rclpy
            import tf2_ros
            from rclpy.context import Context
            from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.signals import SignalHandlerOptions
            from rclpy.time import Time
            from std_msgs.msg import Float64MultiArray, String
            from std_srvs.srv import Trigger
            external_shutdown = (ExternalShutdownException,)
            context = Context()
            # The main GUI owns SIGINT/SIGTERM so it can save the active trial
            # before tearing down Tk and ROS. Do not install ROS signal handlers.
            rclpy.init(context=context, signal_handler_options=SignalHandlerOptions.NO)
            node = Node("erie_user_study", context=context)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            subscriptions = []
            for kind, topics in (("raw", RAW_TOPICS), ("force", FORCE_TOPICS)):
                for finger, topic in topics.items():
                    def receive(message, kind=kind, finger=finger):
                        self.sample_force(kind, finger, message.data)
                    subscriptions.append(node.create_subscription(Float64MultiArray, topic, receive, 1000))
            subscriptions.append(node.create_subscription(
                String, FORCE_BIAS_STATUS_TOPIC,
                lambda message: self.sample_force_bias_status(message.data), 10))
            bias_client = node.create_client(Trigger, FORCE_BIAS_SERVICE)
            buffer = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(buffer, node, spin_thread=False)
            last_stamp = None

            def receive_pose():
                nonlocal last_stamp
                try:
                    transform = buffer.lookup_transform("polhemus_base", "sensor4", Time())
                    stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
                    now = node.get_clock().now().nanoseconds * 1e-9
                    if stamp == last_stamp or abs(now - stamp) > STALE_SEC:
                        return
                    rotation = transform.transform.rotation
                    self.sample_pose((rotation.x, rotation.y, rotation.z, rotation.w), stamp)
                    last_stamp = stamp
                except tf2_ros.TransformException:
                    pass

            timer = node.create_timer(1.0 / self.pose_rate, receive_pose)
            self.events.put(("info", "ROS connected. Waiting for sensor4 and finger-frame force streams."))
            while not self.stop_event.is_set() and context.ok():
                executor.spin_once(timeout_sec=.05)
                self._poll_force_bias(bias_client, Trigger.Request)
        except external_shutdown:
            # Context shutdown is a normal way for the executor to stop.
            pass
        except Exception as exc:
            self.error = repr(exc)
            self.events.put(("error", "ROS connection failed: " + self.error))
        finally:
            with self.lock:
                self._force_bias_service_ready = False
                self._force_bias_pending = self._force_bias_queued = False
            # Attempt every cleanup even if a preceding resource reports an
            # error. In particular, a node failure must not strand its context.
            for name, resource, close in (
                ("force-bias request", bias_client, self._cancel_force_bias_future),
                ("executor", executor, lambda value: value.shutdown()),
                ("node", node, lambda value: value.destroy_node()),
                ("context", context, lambda value: value.try_shutdown()),
            ):
                if resource is None:
                    continue
                try:
                    close(resource)
                except Exception as exc:
                    self.error = self.error or repr(exc)
                    self.events.put(("error", f"ROS {name} cleanup failed: {exc!r}"))

    def stop(self):
        """Stop acquisition and return after the worker releases its resources."""
        self.stop_event.set()
        self.thread.join()
