#!/usr/bin/env python3
"""ERIE Finger Manipulation - complete researcher + participant GUI.

- Sensor 1: thumb,  sensor 2: middle, sensor 3: index (repository README).
- Subscribes to the *existing* ROS2 Float64MultiArray force topics.
- Records selected topic set itself; DO NOT also run recoder_launch.py or
  force_trial_csv_recorder.py into the same trial folder.
- Optional sensor4 TF object orientation (display only), and optional external
  three-component rotation-feedback topic (display only; no unverified ratings).
- Demo: python3 erie_complete_study_gui.py --demo
- ROS2: source /opt/ros/humble/setup.bash
        python3 erie_complete_study_gui.py

Tkinter runs on its main thread. ROS callbacks and the CSV writer use threads.
ROS callback receives every sample and sends it directly to the CSV worker
without waiting for Tk redraws, while Tk refreshes only ~10 times/second.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import queue
import re
import threading
import time
import tkinter as tk
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

FINGERS = ("thumb", "middle", "index")
STREAMS = {
    "Finger-frame force (Polhemus transformed)": {
        "thumb": "/ERIE_Manipulation/force/force_s1_finger1",
        "middle": "/ERIE_Manipulation/force/force_s2_finger2",
        "index": "/ERIE_Manipulation/force/force_s3_finger3",
    },
    "Raw sensor force": {
        "thumb": "/ERIE_Manipulation/force/force_s1_raw",
        "middle": "/ERIE_Manipulation/force/force_s2_raw",
        "index": "/ERIE_Manipulation/force/force_s3_raw",
    },
    "Object-origin force (if publishers enabled)": {
        "thumb": "/ERIE_Manipulation/force/object_force_s1_raw",
        "middle": "/ERIE_Manipulation/force/object_force_s2_raw",
        "index": "/ERIE_Manipulation/force/object_force_s3_raw",
    },
}
TASKS = ("roll", "pitch", "yaw", "finger_gating", "single_fg")
TASK_LABELS = {"roll": "ROLL — X axis", "pitch": "PITCH — Y axis",
               "yaw": "YAW — Z axis", "finger_gating": "FINGER GAITING",
               "single_fg": "SINGLE-FINGER GAITING"}
DEFAULT_INSTRUCTIONS = {
    "roll": "1. Place your assigned fingers on the cylinder.\n2. Grip and lift it straight up.\n3. Rotate it about the X axis until the trial ends.",
    "pitch": "1. Place your assigned fingers on the cylinder.\n2. Grip and lift it straight up.\n3. Rotate it about the Y axis until the trial ends.",
    "yaw": "1. Place your assigned fingers on the cylinder.\n2. Grip and lift it straight up.\n3. Rotate it about the vertical Z axis until the trial ends.",
    "finger_gating": "1. Grip and support the cylinder.\n2. Reposition the instructed fingers while keeping control of the cylinder.\n3. Continue until the trial ends.",
    "single_fg": "1. Grip and support the cylinder.\n2. Move only the finger identified by the researcher.\n3. Continue until the trial ends.",
}
DURATION = 35.0
COUNTDOWN = 3.0
STALE_SEC = 1.5
FEEDBACK_TOPIC = "/ERIE_Manipulation/axis_rotational_error"  # OPTIONAL custom topic
BACKGROUND = "#0E1D2F"
FOREGROUND = "#F8FAFF"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def axis_angles_from_quaternion(q):
    """Display-only Euler XYZ angles in degrees (not relative axis error)."""
    x, y, z, w = q
    norm = math.sqrt(sum(v * v for v in q))
    if not norm or not all(math.isfinite(v) for v in q):
        return None
    x, y, z, w = (a / norm for a in q)
    roll = math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x+y*y))
    pitch = math.asin(max(-1., min(1., 2 * (w*y-z*x))))
    yaw = math.atan2(2 * (w*z+x*y), 1 - 2 * (y*y+z*z))
    return tuple(math.degrees(v) for v in (roll, pitch, yaw))


class TrialWriter:
    """Single producer (ROS callback thread), separate disk writer; no data loss from UI redraws.

    A bounded queue prevents unbounded memory use; overflow is recorded and
    a trial with any overflow is marked for review.
    """
    def __init__(self, path: Path, metadata: dict):
        self.path = path
        self.metadata = metadata
        self.messages = queue.Queue(maxsize=150000)
        self.finished = threading.Event()
        self.counts = {f: 0 for f in FINGERS}
        self.overflows = {f: 0 for f in FINGERS}
        self.bad_values = {f: 0 for f in FINGERS}
        self.result = "recording"
        self.error = None
        self.thread = threading.Thread(target=self._run, daemon=True, name="erie-csv-writer")
        self.thread.start()

    def add(self, finger, values):
        try:
            self.messages.put_nowait((finger, values))
        except queue.Full:
            self.overflows[finger] += 1

    def finish(self, result, extra):
        self.result = result
        self.metadata.update(extra)
        # If the disk writer already failed, it has finalized itself; avoid
        # blocking the GUI forever on a full queue with no consumer.
        if not self.finished.is_set():
            try:
                self.messages.put(None, timeout=2)
            except queue.Full:
                self.error = "Writer unable to accept stop marker (queue full)"
                self.result = "write_error"

    def _run(self):
        files = {}
        try:
            writers = {}
            for finger in FINGERS:
                f = (self.path / f"{finger}.csv").open("x", newline="", buffering=65536)
                files[finger] = f
                writer = csv.writer(f)
                writer.writerow(["source_timestamp_sec", "Fx", "Fy", "Fz"])
                writers[finger] = writer
            while True:
                packet = self.messages.get()
                if packet is None:
                    break
                finger, (fx, fy, fz, stamp) = packet
                writers[finger].writerow((f"{stamp:.9f}", f"{fx:.9f}",
                                          f"{fy:.9f}", f"{fz:.9f}"))
                self.counts[finger] += 1
                if self.counts[finger] % 250 == 0:
                    files[finger].flush()
        except Exception as e:
            self.error = repr(e)
            self.result = "write_error"
        finally:
            for f in files.values():
                try:
                    f.flush()
                    f.close()
                except OSError:
                    pass
            self.metadata.update({
                "result": self.result,
                "finished_utc": utc_now(),
                "recorded_counts": dict(self.counts),
                "writer_queue_overflows": dict(self.overflows),
                "write_error": self.error,
            })
            try:
                (self.path / "trial_metadata.json").write_text(
                    json.dumps(self.metadata, indent=2), encoding="utf-8")
            except OSError as e:
                self.error = (self.error or "") + "; metadata: " + repr(e)
            self.finished.set()


class ROSBridge:
    """Owns ROS on one worker thread. Lock protects current samples & recorder gate."""
    def __init__(self, demo: bool, errors: queue.Queue):
        self.demo = demo
        self.errors = errors
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.values = {(mode, f): None for mode in STREAMS for f in FINGERS}
        self.times = {(mode, f): 0.0 for mode in STREAMS for f in FINGERS}
        self.rates = {(mode, f): deque(maxlen=100) for mode in STREAMS for f in FINGERS}
        self.gaps = {(mode, f): 0 for mode in STREAMS for f in FINGERS}
        self.invalid = {(mode, f): 0 for mode in STREAMS for f in FINGERS}
        self.writer = None
        self.active_mode = None
        self.start_monotonic = 0.
        self.stop_monotonic = 0.
        self.object_quaternion = None
        self.object_time = 0.
        self.external_axis = None
        self.external_axis_time = 0.
        self.thread = threading.Thread(target=self._run, daemon=True, name="erie-ros-bridge")
        self.thread.start()

    def _sample(self, mode, finger, vals, arrival):
        try:
            data = tuple(float(v) for v in vals)
            if len(data) < 4 or not all(math.isfinite(v) for v in (data[0], data[1], data[2], data[-1])):
                raise ValueError("Expected finite [Fx, Fy, Fz, timestamp]")
            sample = (data[0], data[1], data[2], data[-1])
        except (ValueError, TypeError, IndexError):
            with self.lock:
                self.invalid[(mode, finger)] += 1
            return
        key = (mode, finger)
        with self.lock:
            old_time = self.times[key]
            if old_time and arrival - old_time > .15:
                self.gaps[key] += 1
            self.values[key] = sample
            self.times[key] = arrival
            self.rates[key].append(arrival)
            # Whole-trial raw sample storage is independent of Tk refresh.
            if (self.writer is not None and mode == self.active_mode
                    and self.start_monotonic <= arrival < self.stop_monotonic):
                self.writer.add(finger, sample)

    def begin(self, mode, writer, start, stop):
        with self.lock:
            self.writer = writer
            self.active_mode = mode
            self.start_monotonic = start
            self.stop_monotonic = stop

    def end(self):
        with self.lock:
            writer = self.writer
            self.writer = None
            self.active_mode = None
            return writer

    def snapshot(self, mode):
        with self.lock:
            samples = {}
            for f in FINGERS:
                key = (mode, f)
                ts = self.rates[key]
                rate = ((len(ts)-1) / (ts[-1]-ts[0])) if len(ts) > 1 and ts[-1] > ts[0] else 0.
                samples[f] = {"value": self.values[key], "at": self.times[key],
                              "rate": rate, "gaps": self.gaps[key], "invalid": self.invalid[key]}
            return (samples, self.object_quaternion, self.object_time,
                    self.external_axis, self.external_axis_time)

    def counters(self, mode):
        with self.lock:
            return {f: {"gaps": self.gaps[(mode,f)], "invalid": self.invalid[(mode,f)]}
                    for f in FINGERS}

    def _run(self):
        if self.demo:
            step = 0
            while not self.stop_event.is_set():
                t = time.monotonic()
                wall = time.time()
                step += 1
                for m in STREAMS:
                    for k, finger in enumerate(FINGERS):
                        self._sample(m, finger,
                                     [1.1*math.sin(step*.035+k), .5*math.cos(step*.017+k),
                                      (2.4+k)*math.sin(step*.019+k*1.7), wall], t)
                with self.lock:
                    self.object_quaternion = (0., 0., math.sin(step*.008), math.cos(step*.008))
                    self.object_time = t
                time.sleep(.01)  # demo ~100 samples/s per finger, NOT hardware 600 Hz
            return
        try:
            import rclpy
            import tf2_ros
            from rclpy.node import Node
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.time import Time
            from std_msgs.msg import Float64MultiArray
            rclpy.init(args=None)
            node = Node("erie_study_gui")
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            subscriptions = []
            for mode, topic_group in STREAMS.items():
                for finger, topic in topic_group.items():
                    def receive(msg, mode=mode, finger=finger):
                        self._sample(mode, finger, msg.data, time.monotonic())
                    subscriptions.append(node.create_subscription(Float64MultiArray, topic, receive, 500))

            def receive_axis(msg):
                data = tuple(float(x) for x in msg.data)
                if len(data) >= 3 and all(math.isfinite(x) for x in data[:3]):
                    with self.lock:
                        self.external_axis = data[:3]
                        self.external_axis_time = time.monotonic()
            subscriptions.append(node.create_subscription(Float64MultiArray, FEEDBACK_TOPIC, receive_axis, 10))
            buffer = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(buffer, node, spin_thread=False)
            def check_object_tf():
                try:
                    trans = buffer.lookup_transform("polhemus_base", "sensor4", Time())
                    stamp = trans.header.stamp
                    ros_timestamp = stamp.sec + stamp.nanosec / 1e9
                    # TF can retain outdated transforms; require a recent header.
                    now_ros = node.get_clock().now().nanoseconds * 1e-9
                    if abs(now_ros - ros_timestamp) > STALE_SEC:
                        return
                    rot = trans.transform.rotation
                    quat = (rot.x, rot.y, rot.z, rot.w)
                    if axis_angles_from_quaternion(quat) is None:
                        return
                    with self.lock:
                        self.object_quaternion = quat
                        self.object_time = time.monotonic()
                except Exception:
                    pass  # Position stream optional; don't spam errors at 5 Hz.
            tf_timer = node.create_timer(.2, check_object_tf)
            self.errors.put(("info", "ROS 2 connected. Waiting for force publishers."))
            try:
                while rclpy.ok() and not self.stop_event.is_set():
                    executor.spin_once(timeout_sec=.1)
            finally:
                executor.remove_node(node)
                node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
        except Exception as e:
            self.errors.put(("error", "ROS 2 bridge stopped: " + repr(e)))

    def stop(self):
        self.stop_event.set()


class StudyApplication:
    def __init__(self, root: tk.Tk, demo=False, output=None):
        self.root = root
        self.root.title("ERIE Lab | Finger Manipulation Researcher Console")
        self.root.geometry("1190x930")
        self.root.minsize(960, 760)
        self.demo = demo
        self.events = queue.Queue()
        self.bridge = ROSBridge(demo, self.events)
        self.mode = tk.StringVar(value=list(STREAMS)[0])
        self.participant = tk.StringVar(value="001")
        self.trial = tk.StringVar(value="001")
        self.diameter = tk.StringVar(value="50")
        self.task = tk.StringVar(value="roll")
        self.data_root = tk.StringVar(value=str(output or (Path(__file__).resolve().parent / "data")))
        self.ready = tk.BooleanVar(value=False)
        self.consent_checked = tk.BooleanVar(value=False)
        self.require_three = tk.BooleanVar(value=True)
        self.require_object = tk.BooleanVar(value=False)
        self.show_axis = tk.BooleanVar(value=False)
        self.show_object = tk.BooleanVar(value=True)
        self.zero = {f: (0., 0., 0.) for f in FINGERS}
        self.last_values = {f: None for f in FINGERS}
        self.phase = "idle"
        self.timer_text = tk.StringVar(value="35.0 s")
        self.session_status = tk.StringVar(value="DEMO MODE: simulated force values" if demo else "Connecting to ROS 2...")
        self.sensor_labels = {}
        self.sensor_numbers = {}
        self.status_labels = {}
        self.rate_labels = {}
        self.sample_counts = {}
        self.bar_canvases = {}
        self.note = None
        self.instructions = None
        self.recorder = None
        self.countdown_start = 0.
        self.start_time = 0.
        self.end_time = 0.
        self.pending_path = None
        self.saved_settings = None
        self.counter_start = None
        self.stale_seen = set()
        self._make_researcher()
        self._make_participant()
        self._sync_task_instruction()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(100, self.update)

    def _make_researcher(self):
        wrapper = ttk.Frame(self.root, padding=12)
        wrapper.pack(fill="both", expand=True)
        ttk.Label(wrapper, text="ERIE | Finger Manipulation User Study", font=("Arial", 20, "bold")).pack(anchor="w")
        ttk.Label(wrapper, text="Researcher controls  •  3 ATI sensors  •  35-second trials  •  ROS 2 Humble" +
                  ("  •  DEMO" if self.demo else "")).pack(anchor="w", pady=(0, 5))
        setup = ttk.LabelFrame(wrapper, text="1   Study setup", padding=10)
        setup.pack(fill="x", pady=5)
        self.setup_inputs = []
        pairs = [("Participant ID", self.participant), ("Trial number", self.trial),
                 ("Cylinder diameter", self.diameter), ("Manipulation task", self.task)]
        for i, (name, var) in enumerate(pairs):
            ttk.Label(setup, text=name).grid(row=0, column=i, sticky="w", padx=5)
            if i == 2:
                widget = ttk.Combobox(setup, textvariable=var, values=("50", "80"), width=16, state="readonly")
            elif i == 3:
                widget = ttk.Combobox(setup, textvariable=var, values=TASKS, width=22, state="readonly")
            else:
                widget = ttk.Entry(setup, textvariable=var, width=20)
            widget.grid(row=1, column=i, sticky="ew", padx=5, pady=3)
            self.setup_inputs.append(widget)
            setup.columnconfigure(i, weight=1)
        ttk.Label(setup, text="Record which force coordinate frame?").grid(row=2, column=0, sticky="w", padx=5, pady=(10, 0))
        self.mode_input = ttk.Combobox(setup, textvariable=self.mode, values=list(STREAMS),
                                       width=54, state="readonly")
        self.mode_input.grid(row=3, column=0, columnspan=3, sticky="ew", padx=5)
        ttk.Label(setup, text="Object-origin streams may be OFF until the publisher is enabled.",
                  foreground="#995500").grid(row=3, column=3, sticky="w", padx=5)
        ttk.Label(setup, text="Data folder").grid(row=4, column=0, sticky="w", padx=5, pady=(10, 0))
        self.path_input = ttk.Entry(setup, textvariable=self.data_root)
        self.path_input.grid(row=5, column=0, columnspan=3, sticky="ew", padx=5)
        self.browse_btn = ttk.Button(setup, text="Browse...", command=self.browse)
        self.browse_btn.grid(row=5, column=3, sticky="ew", padx=5)
        sensors = ttk.LabelFrame(wrapper, text="2   Live monitoring — Fx / Fy / Fz in newtons", padding=10)
        sensors.pack(fill="x", pady=5)
        for i, f in enumerate(FINGERS):
            ttk.Label(sensors, text=f"{f.title()} (S{(1 if f=='thumb' else 2 if f=='middle' else 3)})",
                      font=("Arial", 11, "bold")).grid(row=i, column=0, sticky="w", padx=4, pady=6)
            self.sensor_labels[f] = ttk.Label(sensors, text="WAITING", foreground="#a35a00")
            self.sensor_labels[f].grid(row=i, column=1, sticky="w", padx=5)
            self.sensor_numbers[f] = ttk.Label(sensors, text="Fx: --    Fy: --    Fz: --", width=45)
            self.sensor_numbers[f].grid(row=i, column=2, sticky="w", padx=5)
            self.rate_labels[f] = ttk.Label(sensors, text="-- Hz", width=10)
            self.rate_labels[f].grid(row=i, column=3, sticky="w", padx=5)
            c = tk.Canvas(sensors, width=260, height=26, bg="#EDF1F5", highlightthickness=0)
            c.grid(row=i, column=4, sticky="ew", padx=4)
            self.bar_canvases[f] = c
            self.sample_counts[f] = ttk.Label(sensors, text="0 samples", width=16)
            self.sample_counts[f].grid(row=i, column=5, sticky="w")
        sensors.columnconfigure(4, weight=1)
        self.zero_btn = ttk.Button(sensors, text="Zero display", command=self.zero_display)
        self.zero_btn.grid(row=3, column=0, columnspan=2, sticky="w", pady=5)
        self.restore_btn = ttk.Button(sensors, text="Clear display zero", command=self.reset_zero)
        self.restore_btn.grid(row=3, column=2, sticky="w", pady=5)
        ttk.Label(sensors, text="Zero affects screen only; CSV always preserves the original published values.",
                  foreground="#596579").grid(row=4, column=0, columnspan=6, sticky="w")
        optional = ttk.LabelFrame(wrapper, text="3   Optional position and axis feedback", padding=9)
        optional.pack(fill="x", pady=5)
        self.position_status = tk.StringVar(value="Object TF (polhemus_base ← sensor4): waiting")
        self.axis_status = tk.StringVar(value="External axis feedback: waiting (optional)")
        ttk.Label(optional, textvariable=self.position_status).grid(row=0, column=0, columnspan=4, sticky="w")
        ttk.Label(optional, textvariable=self.axis_status).grid(row=1, column=0, columnspan=4, sticky="w")
        self.chk_object = ttk.Checkbutton(optional, text="Show object orientation on participant display", variable=self.show_object)
        self.chk_object.grid(row=2, column=0, columnspan=2, sticky="w", pady=(7, 0))
        self.chk_axis = ttk.Checkbutton(optional, text="Show external 3-axis values (unvalidated)", variable=self.show_axis)
        self.chk_axis.grid(row=2, column=2, columnspan=2, sticky="w", pady=(7, 0))
        self.chk_require_obj = ttk.Checkbutton(optional, text="Require valid Polhemus object TF before starting", variable=self.require_object)
        self.chk_require_obj.grid(row=3, column=0, columnspan=4, sticky="w")
        guide = ttk.LabelFrame(wrapper, text="4   Participant instructions (editable BEFORE starting)", padding=8)
        guide.pack(fill="x", pady=5)
        self.instructions = tk.Text(guide, height=4, wrap="word", font=("Arial", 10))
        self.instructions.pack(fill="x")
        self.instructions.bind("<<Modified>>", self._instructions_modified)
        study = ttk.LabelFrame(wrapper, text="5   Trial controls", padding=10)
        study.pack(fill="x", pady=5)
        self.chk_consent = ttk.Checkbutton(study, text="Consent/protocol confirmed by researcher", variable=self.consent_checked)
        self.chk_consent.grid(row=0, column=0, columnspan=2, sticky="w")
        self.chk_ready = ttk.Checkbutton(study, text="Participant ready & finger placement checked", variable=self.ready)
        self.chk_ready.grid(row=1, column=0, columnspan=2, sticky="w")
        self.chk_require = ttk.Checkbutton(study, text="Require all three force streams", variable=self.require_three)
        self.chk_require.grid(row=1, column=2, columnspan=2, sticky="w")
        self.start_btn = ttk.Button(study, text="START TRIAL (3s countdown)", command=self.start)
        self.start_btn.grid(row=2, column=0, padx=4, pady=9, sticky="ew")
        self.stop_btn = ttk.Button(study, text="STOP / SAVE EARLY", command=self.stop_trial)
        self.stop_btn.grid(row=2, column=1, padx=4, pady=9, sticky="ew")
        self.next_btn = ttk.Button(study, text="NEXT TRIAL (+1)", command=self.next_trial)
        self.next_btn.grid(row=2, column=2, padx=4, pady=9, sticky="ew")
        ttk.Label(study, textvariable=self.timer_text, font=("Arial", 22, "bold")).grid(row=2, column=3)
        for c in range(4):
            study.columnconfigure(c, weight=1)
        ttk.Label(study, text="Researcher notes (saved in trial_metadata.json)").grid(row=3, column=0, columnspan=4, sticky="w")
        self.note = tk.Text(study, height=2, wrap="word")
        self.note.grid(row=4, column=0, columnspan=4, sticky="ew", pady=3)
        footer = ttk.Frame(wrapper)
        footer.pack(fill="x", pady=5)
        ttk.Button(footer, text="Participant window", command=self.open_participant).pack(side="left")
        ttk.Button(footer, text="Open study log", command=self.show_log).pack(side="left", padx=8)
        ttk.Label(footer, textvariable=self.session_status, foreground="#17619b", wraplength=720).pack(side="left", padx=15)
        self.task.trace_add("write", lambda *_: self._sync_task_instruction())
        self.mode.trace_add("write", lambda *_: self._mode_changed())

    def _make_participant(self):
        win = self.participant_window = tk.Toplevel(self.root)
        win.title("ERIE | Participant")
        win.geometry("930x710+170+70")
        win.configure(bg=BACKGROUND)
        win.protocol("WM_DELETE_WINDOW", win.withdraw)
        self.p_heading = tk.Label(win, text="WAIT FOR INSTRUCTIONS", fg="#A9D6FF", bg=BACKGROUND,
                                  font=("Arial", 30, "bold"))
        self.p_heading.pack(pady=(37, 14))
        self.p_instruction = tk.Label(win, text="", fg=FOREGROUND, bg=BACKGROUND,
                                      wraplength=810, justify="center", font=("Arial", 18))
        self.p_instruction.pack(pady=8)
        self.p_state = tk.Label(win, text="WAIT FOR RESEARCHER", fg="#F6D47E", bg=BACKGROUND,
                                font=("Arial", 26, "bold"))
        self.p_state.pack(pady=(15, 3))
        self.p_clock = tk.Label(win, text="35.0 s", fg=FOREGROUND, bg=BACKGROUND,
                                font=("Arial", 38, "bold"))
        self.p_clock.pack(pady=(4, 8))
        self.p_progress = tk.Canvas(win, height=19, bg="#32465A", highlightthickness=0)
        self.p_progress.pack(fill="x", padx=75, pady=4)
        self.p_object = tk.Canvas(win, height=170, bg=BACKGROUND, highlightthickness=0)
        self.p_object.pack(fill="x", padx=150, pady=5)
        self.p_axis = tk.Label(win, text="", fg="#B7CBDF", bg=BACKGROUND,
                               font=("Arial", 12))
        self.p_axis.pack(pady=3)
        tk.Button(win, text="Fullscreen / Esc to exit", command=self.toggle_fullscreen).pack(pady=6)
        win.bind("<Escape>", lambda e: win.attributes("-fullscreen", False))

    def open_participant(self):
        self.participant_window.deiconify()
        self.participant_window.lift()

    def toggle_fullscreen(self):
        win = self.participant_window
        win.attributes("-fullscreen", not bool(win.attributes("-fullscreen")))

    def _mode_changed(self):
        if self.phase == "idle":
            self.zero = {f: (0., 0., 0.) for f in FINGERS}

    def _sync_task_instruction(self):
        if self.instructions is not None and self.phase == "idle":
            self.instructions.delete("1.0", "end")
            self.instructions.insert("1.0", DEFAULT_INSTRUCTIONS[self.task.get()])
        if hasattr(self, "p_heading"):
            self.p_heading.config(text=TASK_LABELS[self.task.get()])
        self._update_participant_instructions()

    def _instructions_modified(self, event=None):
        if self.instructions.edit_modified():
            self.instructions.edit_modified(False)
            self._update_participant_instructions()

    def _update_participant_instructions(self):
        if hasattr(self, "p_instruction") and self.instructions is not None and self.phase == "idle":
            self.p_instruction.config(text=self.instructions.get("1.0", "end").strip())

    def browse(self):
        value = filedialog.askdirectory(initialdir=self.data_root.get())
        if value:
            self.data_root.set(value)

    def zero_display(self):
        for f, v in self.last_values.items():
            if v is not None:
                self.zero[f] = v[:3]
        self.session_status.set("Display zero applied; CSV values will remain original.")

    def reset_zero(self):
        self.zero = {f: (0., 0., 0.) for f in FINGERS}
        self.session_status.set("Display zero cleared.")

    def _trial_path(self):
        pid = self.participant.get().strip()
        tid = self.trial.get().strip()
        if not re.fullmatch(r"[A-Za-z0-9_-]{1,32}", pid):
            raise ValueError("Participant ID must be 1-32 letters, digits, underscores or hyphens.")
        if not re.fullmatch(r"\d{1,8}", tid):
            raise ValueError("Trial number must contain 1-8 digits.")
        if self.diameter.get() not in ("50", "80") or self.task.get() not in TASKS:
            raise ValueError("Select a valid diameter and task.")
        root = Path(self.data_root.get()).expanduser().resolve()
        return root / f"user_{pid}" / f"{self.diameter.get()}mm" / self.task.get() / f"trial_{tid}"

    def _all_live(self, samples):
        now = time.monotonic()
        return all(samples[f]["value"] is not None and now - samples[f]["at"] < STALE_SEC for f in FINGERS)

    def _set_busy(self, busy):
        for w in self.setup_inputs + [self.mode_input, self.path_input, self.browse_btn,
                                      self.next_btn, self.chk_object, self.chk_axis,
                                      self.chk_require_obj, self.chk_consent, self.chk_ready, self.chk_require]:
            if isinstance(w, ttk.Combobox):
                w.configure(state="disabled" if busy else "readonly")
            else:
                w.configure(state="disabled" if busy else "normal")
        self.instructions.configure(state="disabled" if busy else "normal")
        self.start_btn.configure(state="disabled" if busy else "normal")
        self.stop_btn.configure(state="normal" if busy else "disabled")

    def start(self):
        if self.phase != "idle":
            return
        if not self.consent_checked.get() or not self.ready.get():
            messagebox.showwarning("Study checklist", "Confirm consent/protocol and participant readiness first.")
            return
        samples, quat, qt, _, _ = self.bridge.snapshot(self.mode.get())
        if self.require_three.get() and not self._all_live(samples):
            messagebox.showerror("Streams unavailable", "All three force topics must publish recent values before trial start.")
            return
        if self.require_object.get() and (quat is None or time.monotonic()-qt > STALE_SEC):
            messagebox.showerror("Position sensor unavailable", "A recent polhemus_base ← sensor4 TF is required.")
            return
        try:
            path = self._trial_path()
            if path.exists():
                messagebox.showerror("Trial already exists", f"Will not overwrite data:\n{path}\n\nChoose another trial number.")
                return
            path.parent.mkdir(parents=True, exist_ok=True)
        except (ValueError, OSError) as e:
            messagebox.showerror("Invalid trial setup", str(e))
            return
        self.pending_path = path
        self.saved_settings = {
            "participant_id": self.participant.get().strip(),
            "trial_number": self.trial.get().strip(),
            "diameter_mm": int(self.diameter.get()),
            "task": self.task.get(), "force_stream": self.mode.get(),
            "topics": STREAMS[self.mode.get()].copy(),
            "duration_target_sec": DURATION,
            "instructions": self.instructions.get("1.0", "end").strip(),
            "force_requires_all_three": self.require_three.get(),
            "object_tf_required": self.require_object.get(),
            "consent_confirmed_checkbox": self.consent_checked.get(),
            "participant_ready_checkbox": self.ready.get(),
            "demo_mode": self.demo,
            "display_zero_offsets_N": {k: list(v) for k,v in self.zero.items()},
            "gui_zero_does_not_affect_recorded_data": True,
            "force_csv_columns": ["source_timestamp_sec", "Fx", "Fy", "Fz"],
        }
        self.phase = "countdown"
        self.countdown_start = time.monotonic()
        self._set_busy(True)
        self.p_instruction.config(text=self.saved_settings["instructions"])
        self.p_state.config(text="GET READY", fg="#F6D47E")
        self.session_status.set("Countdown started. Recording begins after three seconds.")

    def _begin_record(self):
        try:
            self.pending_path.mkdir(exist_ok=False)
            m = {**self.saved_settings, "started_utc": utc_now(), "trial_directory": str(self.pending_path)}
            recorder = TrialWriter(self.pending_path, m)
            self.recorder = recorder
            self.start_time = time.monotonic()
            self.end_time = self.start_time + DURATION
            self.counter_start = self.bridge.counters(self.saved_settings["force_stream"])
            self.stale_seen = set()
            self.bridge.begin(self.saved_settings["force_stream"], recorder, self.start_time, self.end_time)
            self.phase = "recording"
            self.p_state.config(text="RECORDING — PERFORM TASK", fg="#8FE7AF")
            self.session_status.set(f"Recording: {self.pending_path}")
        except Exception as e:
            self.phase = "idle"
            self._set_busy(False)
            messagebox.showerror("Recording could not begin", str(e))

    def stop_trial(self):
        if self.phase == "countdown":
            self.phase = "idle"
            self._set_busy(False)
            self.p_state.config(text="CANCELLED", fg="#F6D47E")
            self.session_status.set("Countdown cancelled; no files created.")
        elif self.phase == "recording":
            self._finish("stopped_early")

    def _finish(self, reason):
        if self.phase != "recording":
            return
        self.phase = "saving"
        writer = self.bridge.end()  # ensures callback cannot append after stop marker
        wall_duration = round(time.monotonic() - self.start_time, 3)
        end_counts = self.bridge.counters(self.saved_settings["force_stream"])
        invalid = {f: end_counts[f]["invalid"] - self.counter_start[f]["invalid"] for f in FINGERS}
        gaps = {f: end_counts[f]["gaps"] - self.counter_start[f]["gaps"] for f in FINGERS}
        extra = {
            "actual_duration_wall_sec": wall_duration,
            "recording_stale_sensors_observed": sorted(self.stale_seen),
            "missing_or_large_interarrival_gaps": gaps,
            "invalid_message_counts": invalid,
            "researcher_notes": self.note.get("1.0", "end").strip(),
            "sensor_mapping": {"thumb": "sensor1", "middle": "sensor2", "index": "sensor3"},
        }
        writer.finish(reason, extra)
        self.p_state.config(text="TRIAL ENDED — SAVING", fg="#F6D47E")
        self.session_status.set("Flushing recorded data to disk...")

    def _log_trial(self):
        writer = self.recorder
        metadata = writer.metadata
        quality = "REVIEW" if (writer.error or any(writer.overflows.values())
                               or any(v == 0 for v in writer.counts.values())
                               or metadata.get("recording_stale_sensors_observed")
                               or any(metadata.get("missing_or_large_interarrival_gaps", {}).values())
                               or metadata["result"] != "completed") else "OK"
        fields = ("finished_utc", "participant_id", "diameter_mm", "task", "trial_number",
                  "force_stream", "result", "quality", "actual_duration_wall_sec",
                  "thumb_samples", "middle_samples", "index_samples", "directory")
        row = {
            "finished_utc": metadata.get("finished_utc"), "participant_id": metadata["participant_id"],
            "diameter_mm": metadata["diameter_mm"], "task": metadata["task"],
            "trial_number": metadata["trial_number"], "force_stream": metadata["force_stream"],
            "result": metadata["result"], "quality": quality,
            "actual_duration_wall_sec": metadata.get("actual_duration_wall_sec"),
            "thumb_samples": writer.counts["thumb"], "middle_samples": writer.counts["middle"],
            "index_samples": writer.counts["index"], "directory": str(writer.path),
        }
        metadata["quality_flag"] = quality
        try:
            (writer.path / "trial_metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
            path = Path(self.data_root.get()).expanduser().resolve() / "study_log.csv"
            exists = path.exists() and path.stat().st_size > 0
            with path.open("a", newline="") as output:
                csvwriter = csv.DictWriter(output, fieldnames=fields)
                if not exists:
                    csvwriter.writeheader()
                csvwriter.writerow(row)
        except OSError as e:
            messagebox.showerror("Study log issue", "Trial CSVs may be saved, but updating study_log.csv failed:\n" + str(e))
        return quality

    def next_trial(self):
        if self.phase != "idle":
            return
        tid = self.trial.get()
        if not tid.isdigit():
            messagebox.showerror("Invalid number", "Enter a numeric trial number.")
            return
        self.trial.set(str(int(tid)+1).zfill(len(tid)))
        self.ready.set(False)
        self.note.delete("1.0", "end")
        self.session_status.set("Next trial prepared. Recheck placement, consent/protocol and readiness.")

    def show_log(self):
        path = Path(self.data_root.get()).expanduser().resolve() / "study_log.csv"
        if not path.exists():
            messagebox.showinfo("Study log", "No trial has finished yet. The log will be saved to:\n"+str(path))
            return
        preview = path.read_text(encoding="utf-8").splitlines()[-12:]
        window = tk.Toplevel(self.root)
        window.title("ERIE — recent study log rows")
        window.geometry("1100x380")
        body = tk.Text(window, wrap="none", font=("Courier", 10))
        body.pack(fill="both", expand=True)
        body.insert("1.0", "Study log: " + str(path) + "\n\n" + "\n".join(preview))
        body.configure(state="disabled")

    def _draw_force(self, canvas, fz):
        canvas.delete("all")
        w = max(canvas.winfo_width(), 240)
        h = max(canvas.winfo_height(), 25)
        center = w/2
        canvas.create_line(center, 2, center, h-2, fill="#697A90", width=2)
        if fz is not None:
            bar = min(abs(fz)/25, 1.) * (w/2-6)
            if fz >= 0:
                canvas.create_rectangle(center, 6, center+bar, h-5, fill="#3186C8", outline="")
            else:
                canvas.create_rectangle(center-bar, 6, center, h-5, fill="#2B9C80", outline="")

    def _draw_participant_object(self, quat, active):
        """Simple 3D cylinder rendering from TF quaternion; only orientation, no position."""
        c = self.p_object
        c.delete("all")
        if not active or quat is None:
            return
        w = max(1, c.winfo_width())
        h = max(1, c.winfo_height())
        qx, qy, qz, qw = quat
        norm = math.sqrt(sum(v*v for v in quat))
        if norm < 1e-9:
            return
        qx, qy, qz, qw = (v/norm for v in quat)
        def rotate(point):
            vx, vy, vz = point
            # q * v * q^-1 = v + 2*w*cross(q,v) + 2*cross(q,cross(q,v))
            tx = 2*(qy*vz-qz*vy)
            ty = 2*(qz*vx-qx*vz)
            tz = 2*(qx*vy-qy*vx)
            return (vx+qw*tx+(qy*tz-qz*ty), vy+qw*ty+(qz*tx-qx*tz),
                    vz+qw*tz+(qx*ty-qy*tx))
        def project(point):
            x,y,z = rotate(point)
            return (w/2 + (x*.83-y*.55)*75, h/2 + (x*.25+y*.35-z*.9)*75)
        num = 22
        bottom = [project((.43*math.cos(2*math.pi*i/num), .43*math.sin(2*math.pi*i/num), -.65)) for i in range(num)]
        top = [project((.43*math.cos(2*math.pi*i/num), .43*math.sin(2*math.pi*i/num), .65)) for i in range(num)]
        for i in range(num):
            j = (i+1) % num
            c.create_line(*bottom[i], *top[i], fill="#4C98D3", width=1)
        for points, col in ((bottom,"#6BAED4"),(top,"#8FDBF0")):
            xy = [z for p in points for z in p]
            c.create_polygon(*xy, outline="#A8E4FF", fill=col, width=2)
        c.create_text(8, h-10, anchor="w", text="Object orientation (sensor4 TF)", fill="#ACBFD0", font=("Arial", 10))

    def _draw_progress(self, amount):
        c = self.p_progress
        c.delete("all")
        width = max(100, c.winfo_width())
        c.create_rectangle(0, 0, width, 19, fill="#354D63", outline="")
        c.create_rectangle(0, 0, width*max(0., min(1., amount)), 19, fill="#65DFAE", outline="")

    def update(self):
        try:
            while True:
                kind, msg = self.events.get_nowait()
                self.session_status.set(msg)
        except queue.Empty:
            pass
        now = time.monotonic()
        samples, quat, qt, axis, at = self.bridge.snapshot(self.mode.get())
        if self.phase == "recording":
            for f in FINGERS:
                if now - samples[f]["at"] > STALE_SEC:
                    self.stale_seen.add(f)
        for finger, s in samples.items():
            good = s["value"] is not None and now - s["at"] <= STALE_SEC
            self.sensor_labels[finger].config(text="ONLINE" if good else "OFFLINE / STALE",
                                               foreground="#087A42" if good else "#AC4C22")
            self.last_values[finger] = s["value"] if good else None
            if good:
                shown = [a-b for a,b in zip(s["value"][:3], self.zero[finger])]
                self.sensor_numbers[finger].config(text="Fx:%+7.2f   Fy:%+7.2f   Fz:%+7.2f" % tuple(shown))
                self.rate_labels[finger].config(text=f"{s['rate']:.0f} Hz")
                self._draw_force(self.bar_canvases[finger], shown[2])
            else:
                self.sensor_numbers[finger].config(text="Fx: --    Fy: --    Fz: --")
                self.rate_labels[finger].config(text="-- Hz")
                self._draw_force(self.bar_canvases[finger], None)
            recorded = self.recorder.counts[finger] if self.recorder else 0
            self.sample_counts[finger].config(text=f"{recorded:,} samples")
        object_live = quat is not None and now - qt < STALE_SEC
        angles = axis_angles_from_quaternion(quat) if object_live else None
        if angles:
            self.position_status.set("Object TF ONLINE  •  Euler XYZ: roll=%+.1f°, pitch=%+.1f°, yaw=%+.1f° (absolute, NOT error)" % angles)
        else:
            self.position_status.set("Object TF OFFLINE / STALE (polhemus_base ← sensor4)")
        axis_live = axis is not None and now - at < STALE_SEC
        if axis_live:
            self.axis_status.set("External axis input: X=%+.2f Y=%+.2f Z=%+.2f  (units/meaning depend on YOUR node)" % axis)
        else:
            self.axis_status.set("External 3-component axis topic OFFLINE (optional; repository error node uses another topic)")
        self._draw_participant_object(quat, object_live and self.show_object.get())
        self.p_axis.config(text=("External axis stream: X=%+.2f, Y=%+.2f, Z=%+.2f [UNVALIDATED]" % axis)
                          if self.show_axis.get() and axis_live else "")
        if self.phase == "countdown":
            remaining = COUNTDOWN-(now-self.countdown_start)
            if remaining <= 0:
                self._begin_record()
            else:
                self.p_state.config(text="GET READY", fg="#F6D47E")
                self.p_clock.config(text=str(math.ceil(remaining)))
                self.timer_text.set(f"{remaining:.1f}s countdown")
                self._draw_progress(0)
        elif self.phase == "recording":
            remaining = max(0., self.end_time-now)
            self.timer_text.set(f"{remaining:.1f} s")
            self.p_clock.config(text=f"{remaining:.1f} s")
            self._draw_progress((DURATION-remaining)/DURATION)
            if now >= self.end_time:
                self._finish("completed")
        elif self.phase == "saving":
            if self.recorder.finished.is_set():
                quality = self._log_trial()
                self.phase = "idle"
                self._set_busy(False)
                self.p_state.config(text=f"TRIAL SAVED — {quality}", fg="#8FE7AF" if quality == "OK" else "#F6D47E")
                self.timer_text.set("35.0 s")
                self.p_clock.config(text="COMPLETE")
                self._draw_progress(1)
                self.session_status.set(f"Saved {self.recorder.path} | {quality} | counts: {self.recorder.counts}")
        self.root.after(100, self.update)

    def close(self):
        if self.phase != "idle":
            if not messagebox.askyesno("Close study GUI?", "An active trial will be stopped and saved. Close now?"):
                return
            if self.phase == "recording":
                self._finish("stopped_on_close")
            elif self.phase == "countdown":
                self.phase = "idle"
            if self.phase == "saving" and self.recorder:
                if not self.recorder.finished.wait(timeout=12):
                    messagebox.showerror("Data still saving", "Keep GUI open: data still writing to disk.")
                    return
                self._log_trial()
        self.bridge.stop()
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description="ERIE finger manipulation dual-window study GUI")
    parser.add_argument("--demo", action="store_true", help="Run without ROS or sensors")
    parser.add_argument("--data-root", default=None, help="Save root (defaults to data/ adjacent to script)")
    args = parser.parse_args()
    root = tk.Tk()
    StudyApplication(root, demo=args.demo, output=args.data_root)
    root.mainloop()


if __name__ == "__main__":
    main()
