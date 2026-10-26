"""ROS acquisition and loss-aware trial storage, independent of Tk's redraw rate."""
from __future__ import annotations

import csv
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import queue
import re
import threading
import time
from collections import deque

import numpy as np

from utils.gui.study_model import normalize_quaternion

FINGERS = ("thumb", "index", "middle")
SENSORS = {"thumb": 1, "index": 3, "middle": 2}
WRENCH_TOPICS = {f: f"/ERIE_Manipulation/force/wrench_s{s}_finger{s}" for f, s in SENSORS.items()}
RAW_TOPICS = {f: f"/ERIE_Manipulation/force/force_s{s}_raw" for f, s in SENSORS.items()}
FORCE_TOPICS = {f: f"/ERIE_Manipulation/force/force_s{s}_finger{s}" for f, s in SENSORS.items()}
FT_HEADER = ("source_timestamp_sec", "Fx", "Fy", "Fz", "Tx", "Ty", "Tz")
STALE_SEC = 0.5


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


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
    return unique_directory(root, f"User{validate_user(user)}_{when:%m%d%Y_%H%M}")


def create_trial_directory(session, user, trial, when):
    task_folder = {"roll": "Roll", "pitch": "Pitch", "yaw": "Yaw", "finger_gating": "FingerGaiting"}[trial.task]
    return unique_directory(Path(session) / task_folder,
                            f"User{validate_user(user)}_Trial{trial.reference}_{when:%m%d%Y_%H%M}")


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
        self.metadata = dict(metadata)
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
            if not self.accepting:
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
            specs = [(('wrench', f), f"{f}.csv", FT_HEADER) for f in FINGERS]
            specs += [(('raw', f), f"{f}_raw_force.csv", FT_HEADER) for f in FINGERS]
            specs += [(("pose", None), "object_orientation.csv", ("source_timestamp_sec", "qx", "qy", "qz", "qw")),
                      (("progress", None), "rotation_progress.csv", ("source_timestamp_sec", "elapsed_sec", "angle_deg", "completed_cycles", "phase"))]
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
                if kind == "wrench":
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
    """Track a demo trial's lifecycle without writing or retaining its samples."""
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
            if kind == "wrench" and finger in self.counts:
                self.counts[finger] += 1
            elif kind == "raw" and finger in self.raw_counts:
                self.raw_counts[finger] += 1
            elif kind == "pose" and finger is None:
                self.pose_count += 1
            elif kind != "progress" or finger is not None:
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
    """Own ROS callbacks on a worker; Tk reads snapshots and a bounded pose history."""
    def __init__(self, demo, events, pose_rate=60.0):
        self.demo = demo
        self.events = events
        self.pose_rate = pose_rate
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.values = {(kind, f): None for kind in ("wrench", "raw", "force") for f in FINGERS}
        self.times = {key: 0.0 for key in self.values}
        self.rates = {key: deque(maxlen=120) for key in self.values}
        self.invalid = {key: 0 for key in self.values}
        self.gaps = {key: 0 for key in self.values}
        self.pose = None
        self.poses = deque(maxlen=3000)
        self.pose_drops = 0
        self.writer = None
        self.error = None
        self.demo_axis = "x"
        self.demo_target = 30.0
        self.demo_motion_start = None
        self.demo_alignment = np.eye(3)
        self.demo_baseline = np.array([0.0, 0.0, 0.0, 1.0])
        self.thread = threading.Thread(target=self._run, daemon=True, name="study-ros")
        self.thread.start()

    def sample_force(self, kind, finger, data, arrival=None):
        arrival = time.monotonic() if arrival is None else arrival
        key = kind, finger
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
            if self.writer and kind in ("raw", "wrench"):
                self.writer.add(kind, finger, (values[-1], *values[:6]))

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
                key = "wrench", finger
                arrivals = self.rates[key]
                rate = (len(arrivals) - 1) / (arrivals[-1] - arrivals[0]) if len(arrivals) > 1 and arrivals[-1] > arrivals[0] else 0.0
                fingers[finger] = {"value": self.values[key], "arrival": self.times[key], "rate": rate,
                                   "force": self.values[("force", finger)], "force_arrival": self.times[("force", finger)]}
            return {"fingers": fingers, "pose": self.pose, "pose_drops": self.pose_drops, "error": self.error}

    def drain_poses(self):
        with self.lock:
            result = list(self.poses)
            self.poses.clear()
            return result

    def counters(self):
        with self.lock:
            return {"invalid": {f"{k[0]}_{k[1]}": v for k, v in self.invalid.items()},
                    "gaps": {f"{k[0]}_{k[1]}": v for k, v in self.gaps.items()}, "pose_drops": self.pose_drops}

    def begin(self, writer, axis, target, alignment=None, baseline=None):
        """Begin recording, with demo poses expressed using the real trial frames.

        alignment maps mesh axes to sensor4 axes; baseline is the sensor4 xyzw
        quaternion captured at trial start. Omitted frames retain the original
        identity-frame demo behavior.
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
            self.poses.clear()
            self.writer = writer
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
        closing the progress CSV and computing the trial summary.
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
                self.sample_force("wrench", finger, values, now)
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
        context = node = executor = None
        external_shutdown = ()
        try:
            import rclpy
            import tf2_ros
            from rclpy.context import Context
            from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.signals import SignalHandlerOptions
            from rclpy.time import Time
            from std_msgs.msg import Float64MultiArray
            external_shutdown = (ExternalShutdownException,)
            context = Context()
            # The main GUI owns SIGINT/SIGTERM so it can save the active trial
            # before tearing down Tk and ROS. Do not install ROS signal handlers.
            rclpy.init(context=context, signal_handler_options=SignalHandlerOptions.NO)
            node = Node("erie_user_study", context=context)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            subscriptions = []
            for kind, topics in (("wrench", WRENCH_TOPICS), ("raw", RAW_TOPICS), ("force", FORCE_TOPICS)):
                for finger, topic in topics.items():
                    def receive(message, kind=kind, finger=finger):
                        self.sample_force(kind, finger, message.data)
                    subscriptions.append(node.create_subscription(Float64MultiArray, topic, receive, 1000))
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
            self.events.put(("info", "ROS connected. Waiting for sensor4 and finger-frame F/T streams."))
            while not self.stop_event.is_set() and context.ok():
                executor.spin_once(timeout_sec=.05)
        except external_shutdown:
            # Context shutdown is a normal way for the executor to stop.
            pass
        except Exception as exc:
            self.error = repr(exc)
            self.events.put(("error", "ROS connection failed: " + self.error))
        finally:
            # Attempt every cleanup even if a preceding resource reports an
            # error. In particular, a node failure must not strand its context.
            for name, resource, close in (
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
