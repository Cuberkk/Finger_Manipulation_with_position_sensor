#!/usr/bin/env python3
"""Two-window experiment console: python -m utils.gui.study_gui [--demo]."""
from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime
import json
import math
from pathlib import Path
import queue
import sys
import time
import tkinter as tk
from tkinter import ttk
from zoneinfo import ZoneInfo

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from utils.paths import DATA_DIR, PROJECT_ROOT
from utils.gui.mesh_view import MeshView
from utils.gui.responsive import ResponsiveFonts
from utils.gui.rest import RestCountdown, load_rest_times
from utils.gui.rest_view import RestCountdownView
from utils.gui.shutdown import ShutdownSignals
from utils.gui.study_model import RotationCounter, load_trials, relative_rotation, signed_angle_degrees
from utils.gui.study_io import (
    FINGERS, SENSORS, WRENCH_TOPICS, STALE_SEC, ROSBridge, TrialWriter, PreviewWriter,
    create_session, create_trial_directory, validate_user, utc_now,
)

NAVY = "#0E1726"
INK = "#17273D"
MUTED = "#65758A"
TEAL = "#147D79"
LIGHT = "#F0F4F7"
PARTICIPANT_BG = "#FFFFFF"
TASK_NAMES = {"roll": "Roll", "pitch": "Pitch", "yaw": "Yaw", "finger_gating": "Finger gaiting"}
COUNTDOWN_SEC = 5


def report(message, error=False):
    """A closed terminal/pipe must not interrupt data or resource cleanup."""
    try:
        print(f"Study GUI: {message}", file=sys.stderr if error else sys.stdout, flush=True)
    except (OSError, ValueError):
        pass


def alignment_from_rpy(degrees):
    """A = Rz(yaw) Ry(pitch) Rx(roll), mapping mesh axes into sensor 4 axes."""
    r, p, y = np.radians(degrees)
    if not np.isfinite((r, p, y)).all():
        raise ValueError("Mesh alignment must contain finite angles.")
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array(
        [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])


class ForceChart(tk.Canvas):
    """Six signed bars, with separate physical scales for N and N·mm."""
    def __init__(self, master):
        super().__init__(master, background="white", highlightthickness=0, height=236, width=280)
        self.scales = [10.0, 100.0]
        self.font_scale = 1.0
        self.bind("<Configure>", lambda _: self.draw(self.last))
        self.last = None

    def set_font_scale(self, scale):
        self.font_scale = scale
        self.draw(self.last)

    def draw(self, values):
        self.last = values
        self.delete("all")
        w, h = self.winfo_width(), self.winfo_height()
        if w < 80 or h < 80:
            return
        s = self.font_scale
        top, bottom = 48 * s, h - 56 * s
        baseline, half = (top + bottom) / 2, max(4, (bottom - top) / 2)
        def font(size, bold=False):
            return ("DejaVu Sans", max(7, round(size * s)), "bold" if bold else "normal")
        colors = ("#E4826F", "#44A99B", "#648CD2")
        for group, unit in enumerate(("FORCE · N", "TORQUE · N·mm")):
            left, right = group * w / 2 + 12, (group + 1) * w / 2 - 12
            current = None if values is None else values[group * 3:group * 3 + 3]
            finite = [] if current is None else [abs(v) for v in current if v is not None]
            if finite and max(finite) > self.scales[group]:
                self.scales[group] = 10 ** math.ceil(math.log10(max(finite)))
            scale = self.scales[group]
            self.create_text((left + right) / 2, 18 * s, text=unit, fill=MUTED, font=font(9, True))
            self.create_text(left, 39 * s, text=f"±{scale:g}", anchor="w", fill=MUTED, font=font(8))
            for level in (-1, 0, 1):
                line_y = baseline + level * half
                self.create_line(left, line_y, right, line_y, fill="#DCE4EB" if level == 0 else "#EDF1F5")
            for axis in range(3):
                x = left + (axis + .5) * (right - left) / 3
                value = None if current is None else current[axis]
                if value is not None:
                    y = baseline - value / scale * half
                    self.create_rectangle(x - 12 * s, min(y, baseline), x + 12 * s, max(y, baseline) + 1,
                                          fill=colors[axis], outline="")
                self.create_text(x, h - 36 * s, text=("F" if group == 0 else "T") + "xyz"[axis],
                                 fill=INK, font=font(10, True))
                self.create_text(x, h - 16 * s, text="—" if value is None else f"{value:+.2f}",
                                 fill=MUTED, font=font(9))
        self.create_line(w / 2, 10, w / 2, h - 8, fill="#E5EBF0")


class StudyApplication:
    def __init__(self, root, args, shutdown_signals=None):
        self.root, self.args = root, args
        self.shutdown_signals = shutdown_signals
        self.close_reason = "closed"
        self.shutdown_complete = False
        self.shutdown_ok = True
        self.trials = load_trials(args.protocol)
        self.rest_times = load_rest_times(args.rest_config)
        self.rest_countdown = None
        self.rest_next_index = None
        self.rest_user = None
        self.rest_screen_visible = False
        self.rest_skipped = False
        self.rest_skipped_at = None
        self.total_trials = sum(not trial.practice for trial in self.trials)
        self.zone = ZoneInfo(args.timezone)
        self.alignment = alignment_from_rpy(args.mesh_alignment_rpy)
        self.index, self.state = 0, "ready"
        self.baseline = None
        self.counter = RotationCounter()
        self.manual_count = 0
        self.angle = 0.0
        self.rotation = np.eye(3)
        self.started = None
        self.session_path = self.session_user = None
        self.writer = None
        self.trial_path = None
        self.closing = False
        self.participant_start_error = False
        self._space_held = False
        self._space_release = None
        self.previous_tracking = False
        self.tracking_interruptions = 0
        self.last_pose_drops = 0
        self.ft_interruptions = {finger: 0 for finger in FINGERS}
        self.ft_stale = {finger: False for finger in FINGERS}
        self.quality_issues = []
        self.events = queue.Queue()
        self.user = tk.StringVar(value="001")
        self.user.trace_add("write", lambda *_: self.refresh_identity())
        self.selection = tk.StringVar()
        self.target = tk.StringVar(value="30")
        self.neutral = tk.StringVar(value="5")
        self.dwell = tk.StringVar(value="0.12")
        self.require_ft = tk.BooleanVar(value=True)
        self.record_demo_data = tk.BooleanVar(value=True)
        self.record_demo_box = None
        self.notice = tk.StringVar(value="Demo mode: simulated orientation and force data." if args.demo else "Connecting to ROS 2…")
        self.start_label = tk.StringVar(value="—")
        self.folder_label = tk.StringVar(value="A session folder is created when the first trial starts.")
        self.status_label = tk.StringVar(value="READY")
        self.count_label = tk.StringVar(value="0 / 20")
        self.pose_label = tk.StringVar(value="Sensor 4 · waiting")
        self._build_researcher()
        self._build_participant()
        self._select_trial(0)
        self.advisor_fonts = ResponsiveFonts(root, base_size=(1240, 900), on_scale=self._scale_advisor)
        self.participant_fonts = ResponsiveFonts(self.participant, base_size=(1000, 850), on_scale=self._scale_participant)
        for window in (root, self.participant):
            window.bind("<F11>", lambda _, w=window: w.attributes("-fullscreen", not w.attributes("-fullscreen")))
            window.bind("<Escape>", lambda _, w=window: w.attributes("-fullscreen", False))
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(50, self.tick)
        # Start the worker last: widget construction failures cannot orphan it.
        self.bridge = ROSBridge(args.demo, self.events)

    @property
    def trial(self):
        return self.trials[self.index]

    @property
    def count(self):
        return self.counter.count if self.trial.axis else self.manual_count

    def label(self, parent, text=None, variable=None, size=11, color=INK, bg="white", bold=False, **kwargs):
        return tk.Label(parent, text=text, textvariable=variable, font=("DejaVu Sans", size, "bold" if bold else "normal"),
                        fg=color, bg=bg, **kwargs)

    def _build_researcher(self):
        r = self.root
        r.title("ERIE · Study advisor" + (" · DEMO" if self.args.demo else ""))
        r.geometry("1240x900+30+30")
        r.minsize(1060, 790)
        r.configure(bg=LIGHT)
        style = ttk.Style(r)
        style.theme_use("clam")
        style.configure("TButton", font=("DejaVu Sans", 10), padding=(12, 8))
        style.configure("Accent.TButton", background=TEAL, foreground="white")
        style.map("Accent.TButton", background=[("active", "#116B68"), ("disabled", "#A8B7BB")])
        style.configure("TEntry", padding=5)
        style.configure("TCombobox", padding=5)
        style.configure("Study.Horizontal.TProgressbar", troughcolor="#E3EAEF", background=TEAL, borderwidth=0)
        header = tk.Frame(r, bg=NAVY, padx=24, pady=12)
        header.pack(fill="x")
        self.label(header, "ERIE  /  MANIPULATION STUDY", size=15, bold=True, color="white", bg=NAVY).pack(side="left")
        self.label(header, "ADVISOR" + ("   •   DEMO" if self.args.demo else "   •   LIVE"), size=10,
                   color="#88D8CA", bg=NAVY).pack(side="right")
        body = tk.Frame(r, bg=LIGHT, padx=24, pady=12)
        body.pack(fill="both", expand=True)
        body.columnconfigure(0, weight=1)
        body.rowconfigure(3, weight=1)
        setup_panel = tk.Frame(body, bg=LIGHT)
        setup_panel.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        setup = tk.Frame(setup_panel, bg=LIGHT)
        setup.pack(fill="x")
        self.label(setup, "User Number", bg=LIGHT, size=10).pack(side="left", padx=(0, 8))
        self.user_entry = ttk.Entry(setup, textvariable=self.user, width=9)
        self.user_entry.pack(side="left", padx=(0, 22))
        self.label(setup, "Trial", bg=LIGHT, size=10).pack(side="left", padx=(0, 8))
        self.trial_select = ttk.Combobox(setup, textvariable=self.selection, width=28, state="readonly",
                                       values=[f"{t.reference}  ·  {t.code}" for t in self.trials])
        self.trial_select.pack(side="left")
        self.trial_select.bind("<<ComboboxSelected>>", lambda _: self._select_trial(self.trial_select.current()))
        ttk.Button(setup, text="Participant window ↗", command=self.show_participant).pack(side="right")
        if self.args.demo:
            demo_options = tk.Frame(setup_panel, bg="white", padx=12, pady=6,
                                    highlightbackground=TEAL, highlightthickness=1)
            demo_options.pack(fill="x", pady=(8, 0))
            self.label(demo_options, "DEMO · DATA SAVING", size=10, color=TEAL, bold=True).pack(side="left")
            self.record_demo_box = tk.Checkbutton(
                demo_options, text="Save data (CSV + metadata)", variable=self.record_demo_data,
                command=self.demo_recording_changed, bg="white", fg=INK, selectcolor="white",
                activebackground="white", font=("DejaVu Sans", 11, "bold"),
            )
            self.record_demo_box.pack(side="left", padx=(18, 0))
            self.label(demo_options, "Unchecked = preview without files", size=9, color=MUTED).pack(side="right")
        information = tk.Frame(body, bg="white", padx=18, pady=10)
        information.grid(row=1, column=0, sticky="ew", pady=(0, 8))
        self.info_vars = {name: tk.StringVar() for name in ("trial", "user", "code", "start", "task", "action")}
        for col in range(3):
            information.columnconfigure(col, weight=1)
        for pos, (key, title) in enumerate((("trial", "TRIAL NUMBER"), ("user", "USER NUMBER"), ("start", "START TIME"),
                                           ("code", "TRIAL TASK"), ("task", "TASK"), ("action", "MOVEMENT"))):
            frame = tk.Frame(information, bg="white")
            frame.grid(row=pos // 3, column=pos % 3, sticky="ew", pady=(0, 6) if pos < 3 else 0)
            self.label(frame, title, size=8, color=MUTED).pack(anchor="w")
            self.label(frame, variable=self.info_vars[key], size=14, bold=True).pack(anchor="w", pady=(3, 0))
        self.label(body, "REALTIME FINGER FORCE / TORQUE", bg=LIGHT, size=10, bold=True).grid(row=2, column=0, sticky="w", pady=(1, 6))
        charts = tk.Frame(body, bg=LIGHT)
        charts.grid(row=3, column=0, sticky="nsew")
        self.charts, self.chart_status = {}, {}
        for i, finger in enumerate(FINGERS):
            charts.columnconfigure(i, weight=1, uniform="charts")
            card = tk.Frame(charts, bg="white", padx=12, pady=12)
            card.grid(row=0, column=i, sticky="nsew", padx=(0 if i == 0 else 6, 0 if i == 2 else 6))
            self.label(card, finger.title(), size=14, bold=True).pack(anchor="w")
            self.label(card, f"Finger frame  ·  Sensor {SENSORS[finger]}", size=9, color=MUTED).pack(anchor="w", pady=(3, 0))
            chart = ForceChart(card)
            chart.pack(fill="both", expand=True)
            self.charts[finger] = chart
            self.chart_status[finger] = tk.StringVar(value="Waiting for F/T")
            self.label(card, variable=self.chart_status[finger], size=9, color=MUTED).pack(anchor="w")
        charts.rowconfigure(0, weight=1)
        self.label(body, "Torque uses finger-frame axes at the ATI sensing origin. Force and torque have separate scales.",
                   size=9, color=MUTED, bg=LIGHT).grid(row=4, column=0, sticky="w", pady=(4, 8))
        control = tk.Frame(body, bg="white", padx=18, pady=10)
        control.grid(row=5, column=0, sticky="ew")
        heading = tk.Frame(control, bg="white")
        heading.pack(fill="x")
        self.label(heading, variable=self.status_label, size=11, color=TEAL, bold=True).pack(side="left")
        self.label(heading, variable=self.count_label, size=17, bold=True).pack(side="right")
        self.label(heading, variable=self.pose_label, size=9, color=MUTED).pack(side="right", padx=20)
        self.progress = ttk.Progressbar(control, style="Study.Horizontal.TProgressbar", maximum=20)
        self.progress.pack(fill="x", pady=(6, 8))
        settings = tk.Frame(control, bg="white")
        settings.pack(fill="x")
        self.setting_entries = []
        for title, variable, width in (("Target ±°", self.target, 5), ("Neutral ±°", self.neutral, 5), ("Hold (s)", self.dwell, 5)):
            self.label(settings, title, size=9).pack(side="left", padx=(0, 6))
            entry = ttk.Entry(settings, textvariable=variable, width=width)
            entry.pack(side="left", padx=(0, 16))
            self.setting_entries.append(entry)
        self.require_box = tk.Checkbutton(settings, text="Require all 3 F/T streams", variable=self.require_ft, bg="white",
                                          fg=INK, activebackground="white", font=("DejaVu Sans", 9))
        self.require_box.pack(side="left")
        buttons = tk.Frame(control, bg="white")
        buttons.pack(fill="x", pady=(8, 0))
        self.start_button = ttk.Button(buttons, text=f"Start trial  ·  {COUNTDOWN_SEC}s", style="Accent.TButton", command=self.start_trial)
        self.start_button.pack(side="left", padx=(0, 8))
        self.stop_button = ttk.Button(buttons, text="Stop & save", command=lambda: self.finish_trial("stopped"), state="disabled")
        self.stop_button.pack(side="left", padx=(0, 8))
        self.manual_button = ttk.Button(buttons, text="Gaiting +1", command=self.manual_cycle, state="disabled")
        self.manual_button.pack(side="left")
        self.next_button = ttk.Button(buttons, text="Next trial →", command=lambda: self._select_trial(self.index + 1))
        self.next_button.pack(side="right")
        self.previous_button = ttk.Button(buttons, text="← Previous", command=lambda: self._select_trial(self.index - 1))
        self.previous_button.pack(side="right", padx=8)
        note_row = tk.Frame(body, bg=LIGHT)
        note_row.grid(row=6, column=0, sticky="ew", pady=(8, 5))
        self.label(note_row, "Trial note", bg=LIGHT, size=9).pack(side="left", padx=(0, 8))
        self.notes = tk.StringVar()
        ttk.Entry(note_row, textvariable=self.notes).pack(side="left", fill="x", expand=True)
        self.folder_widget = self.label(body, variable=self.folder_label, bg=LIGHT, color=MUTED, size=9, anchor="w", justify="left", wraplength=1160)
        self.folder_widget.grid(row=7, column=0, sticky="ew")
        self.notice_widget = self.label(body, variable=self.notice, bg=LIGHT, color=TEAL, size=10, anchor="w", justify="left", wraplength=1150)
        self.notice_widget.grid(row=8, column=0, sticky="ew", pady=(7, 0))

    def _build_participant(self):
        p = self.participant = tk.Toplevel(self.root)
        p.title("ERIE · Participant")
        p.geometry("1000x850+160+90")
        p.minsize(720, 650)
        p.configure(bg=PARTICIPANT_BG)
        p.protocol("WM_DELETE_WINDOW", p.withdraw)
        p.bind("<KeyPress-space>", self.participant_space_pressed)
        p.bind("<KeyRelease-space>", self.participant_space_released)
        p.bind("<FocusOut>", self._release_participant_space, add="+")
        p.bind("<Button-1>", lambda _: p.focus_set(), add="+")
        top = tk.Frame(p, bg=PARTICIPANT_BG, padx=36, pady=24)
        top.pack(fill="x")
        self.part_trial = self.label(top, size=16, color=INK, bg=PARTICIPANT_BG, bold=True)
        self.part_trial.pack(anchor="w")
        self.part_task = self.label(top, size=11, color=MUTED, bg=PARTICIPANT_BG)
        self.part_task.pack(anchor="w", pady=(7, 0))
        self.participant_body = tk.Frame(p, bg=PARTICIPANT_BG)
        self.participant_body.pack(fill="both", expand=True)
        self.part_count = self.label(self.participant_body, "0 / 20", size=32, color=INK, bg=PARTICIPANT_BG, bold=True)
        self.part_count.pack(pady=(2, 2))
        self.label(self.participant_body, "COMPLETE ROTATIONS", size=9, color=MUTED, bg=PARTICIPANT_BG).pack()
        self.part_progress = tk.Canvas(self.participant_body, height=12, bg=PARTICIPANT_BG, highlightthickness=0)
        self.part_progress.pack(fill="x", padx=100, pady=(14, 12))
        self.part_progress.bind("<Configure>", lambda _: self.draw_progress())
        self.part_instruction = self.label(self.participant_body, "Press SPACE when you are ready to start.", size=18, color=TEAL, bg=PARTICIPANT_BG)
        self.part_instruction.pack(pady=(4, 0))
        self.part_angle = self.label(self.participant_body, size=11, color=MUTED, bg=PARTICIPANT_BG)
        self.part_angle.pack(pady=(8, 0))
        self.mesh = MeshView(self.participant_body, self.args.mesh_dir, height=380)
        self.mesh.pack(fill="both", expand=True, padx=30)
        self.rest_view = RestCountdownView(self.participant_body, bg=PARTICIPANT_BG)
        bottom = tk.Frame(p, bg=PARTICIPANT_BG, padx=36, pady=20)
        bottom.pack(fill="x")
        self.part_axis = self.label(bottom, size=10, color="#9A6000", bg=PARTICIPANT_BG)
        self.part_axis.pack(side="left")
        self.part_user = self.label(bottom, size=12, color=INK, bg=PARTICIPANT_BG)
        self.part_user.pack(side="right")

    def _scale_advisor(self, scale):
        for chart in self.charts.values():
            chart.set_font_scale(scale)
        width = max(300, self.root.winfo_width() - 48)
        self.folder_widget.configure(wraplength=width)
        self.notice_widget.configure(wraplength=width)

    def _scale_participant(self, scale):
        self.mesh.set_font_scale(scale)
        self.rest_view.set_font_scale(scale)
        width = max(300, self.participant.winfo_width() - 72)
        self.part_instruction.configure(wraplength=width)
        self.part_angle.configure(wraplength=width)

    def show_participant(self):
        self.participant.deiconify()
        self.participant.lift()
        self.participant.focus_set()

    def participant_space_pressed(self, event=None):
        # X11 autorepeat can send a release/press pair. Cancel its deferred
        # release so holding Space still represents a single request.
        if self._space_release is not None:
            self.participant.after_cancel(self._space_release)
            self._space_release = None
        if not self._space_held:
            self._space_held = True
            self.participant_proceed()
        return "break"

    def participant_space_released(self, event=None):
        if self._space_release is not None:
            self.participant.after_cancel(self._space_release)
        # Use a short timer: continuous redraw events can starve idle callbacks
        # and otherwise leave Space latched after the participant releases it.
        self._space_release = self.participant.after(30, self._release_participant_space)
        return "break"

    def _release_participant_space(self, event=None):
        if self._space_release is not None:
            self.participant.after_cancel(self._space_release)
        self._space_release = None
        self._space_held = False

    def _completed_trial_saved(self):
        return (self.state == "finished" and self.writer is not None
                and self.writer.finished.is_set() and not self.writer.error
                and self.writer.metadata.get("user_number") == self.user.get().strip()
                and ((self.writer.result == "completed" and self.count >= 20)
                     or (self.writer.result == "demo_skipped" and self.args.demo)))

    def participant_proceed(self, event=None):
        if self.closing:
            return "break"
        if self.args.demo:
            if self.state == "countdown":
                self.begin_recording()
                return "break"
            if self.state == "recording":
                self.finish_trial("demo_skipped")
                self.update_progress()
                return "break"
        if self.state == "resting":
            self.rest_skipped = True
            self.rest_skipped_at = time.monotonic()
            self._update_rest(time.monotonic())
            # Rest is skippable in both modes. Formal trials still enter a
            # full preparation countdown and require all their cycles.
        if self.state not in ("ready", "finished"):
            return "break"
        if self.state == "finished":
            if not self._completed_trial_saved() or self.index == len(self.trials) - 1:
                return "break"
            try:
                # Keep the completed trial selected if settings or sensors
                # currently prevent starting the next one.
                self._prepare_trial_start()
            except (ValueError, OverflowError) as exc:
                self.notice.set(str(exc))
                self.participant_start_error = True
                return "break"
            self._select_trial(self.index + 1)
        self.start_trial()
        return "break"

    def participant_instruction(self, now, tracking):
        if self.state == "resting":
            return "Please rest. Press SPACE to skip the remaining rest."
        if self.state == "countdown":
            suffix = " · SPACE to skip · DEMO" if self.args.demo else ""
            return f"Hold the object at neutral.{suffix}"
        if self.state == "recording":
            if self.args.demo:
                return "DEMO · Press SPACE to skip this trial's cycle counting."
            return ("Tracking lost · pause movement" if not tracking else
                    "Rotate smoothly around the highlighted axis." if self.trial.axis else
                    "Complete the gaiting movement.")
        if self.state == "saving":
            if not getattr(self.writer, "recording_enabled", True):
                return "Finishing demo preview…"
            if self.writer.result == "demo_skipped":
                return "Demo trial skipped · saving"
            return "Trial complete · saving" if self.count == 20 else "Trial stopped · saving"
        if self.participant_start_error:
            return "Unable to start. Please ask the researcher."
        if self.state == "finished":
            if not self._completed_trial_saved():
                return "Please wait for the researcher."
            if self.index == len(self.trials) - 1:
                return "Study complete. Thank you!"
            return "Trial complete. Press SPACE when you are ready for the next trial."
        return "Press SPACE when you are ready to start."

    def refresh_identity(self):
        if hasattr(self, "info_vars"):
            self.info_vars["user"].set(self.user.get())
        if hasattr(self, "part_user"):
            self.part_user.configure(text=f"User Number: {self.user.get()}")

    def _select_trial(self, index):
        if self.state in ("countdown", "recording", "saving", "resting") or not 0 <= index < len(self.trials):
            return
        self._clear_rest()
        self.index, self.state = index, "ready"
        self.participant_start_error = False
        self.selection.set(f"{self.trial.reference}  ·  {self.trial.code}")
        self.counter = RotationCounter()
        self.manual_count, self.baseline, self.angle = 0, None, 0.0
        self.rotation = np.eye(3)
        self.notes.set("")
        self.info_vars["trial"].set(f"{self.trial.reference} / {self.total_trials}" if not self.trial.practice else self.trial.reference + " · Practice")
        self.info_vars["code"].set(self.trial.code)
        task = "Practice" if self.trial.practice else f"{self.trial.task_position} / 5"
        self.info_vars["task"].set(task)
        self.info_vars["start"].set("—")
        self.info_vars["action"].set(f"{TASK_NAMES[self.trial.task]} · {self.trial.diameter_mm} mm")
        self.refresh_identity()
        self.part_trial.configure(text=f"Trial {self.trial.reference} / {self.total_trials}" if not self.trial.practice else f"Practice · {self.trial.reference}")
        self.part_task.configure(text=f"Trial Task: {self.trial.code}     ·     Task {task}")
        axis = f"{TASK_NAMES[self.trial.task]}  /  {self.trial.axis.upper()} axis" if self.trial.axis else "Finger gaiting  /  researcher counts"
        self.part_axis.configure(text=axis)
        self.mesh.set_diameter(self.trial.diameter_mm)
        self.mesh.set_pose(np.eye(3), self.trial.axis, tracking=True)
        self.status_label.set("READY")
        self.notice.set(self.mesh.error or ("Ready. Hold the object at neutral before starting." if self.trial.axis else "Gaiting: the researcher records each completed cycle with +1."))
        self.update_progress()
        self.set_controls()
        self.part_instruction.configure(text=self.participant_instruction(time.monotonic(), tracking=True))

    def set_controls(self):
        busy = self.state in ("countdown", "recording", "saving", "resting")
        for widget in [self.user_entry, *self.setting_entries, self.require_box]:
            widget.configure(state="disabled" if busy else "normal")
        self.trial_select.configure(state="disabled" if busy else "readonly")
        self.start_button.configure(state="disabled" if busy else "normal")
        if getattr(self, "record_demo_box", None) is not None:
            self.record_demo_box.configure(state="disabled" if self.state in
                                           ("countdown", "recording", "saving") else "normal")
        self.stop_button.configure(state="normal" if self.state in ("countdown", "recording") else "disabled",
                                   text="Cancel countdown" if self.state == "countdown" else
                                   "Stop preview" if self.writer is not None and
                                   not getattr(self.writer, "recording_enabled", True) else "Stop & save")
        self.manual_button.configure(state="normal" if self.state == "recording" and not self.trial.axis else "disabled")
        self.next_button.configure(state="normal" if not busy and self.index < len(self.trials) - 1 else "disabled")
        self.previous_button.configure(state="normal" if not busy and self.index > 0 else "disabled")

    def preflight(self, snapshot):
        validate_user(self.user.get())
        now = time.monotonic()
        pose = snapshot["pose"]
        if pose is None or now - pose.arrival > STALE_SEC:
            raise ValueError("Sensor 4 is unavailable. Start the Polhemus TF stream before this trial.")
        if self.mesh.error:
            raise ValueError(self.mesh.error)
        if self.require_ft.get():
            missing = [f.title() for f, v in snapshot["fingers"].items() if v["value"] is None or now - v["arrival"] > STALE_SEC]
            if missing:
                raise ValueError("Waiting for finger-frame F/T: " + ", ".join(missing) + ". Check the updated publishers.")
        return pose

    def _prepare_trial_start(self):
        counter = RotationCounter(float(self.target.get()), float(self.neutral.get()), float(self.dwell.get()))
        self.preflight(self.bridge.snapshot())
        return counter

    def demo_recording_changed(self):
        if not self.args.demo:
            return
        enabled = self.record_demo_data.get()
        self.notice.set("New demo trials will save data." if enabled else
                        "New demo trials will run without creating data files.")
        if self.state == "ready":
            self.folder_label.set("A session folder is created when the first recorded trial starts." if enabled else
                                  "Demo preview · data recording disabled · no files will be written.")

    def start_trial(self):
        if self.closing or self.state in ("countdown", "recording", "saving", "resting"):
            return False
        try:
            counter = self._prepare_trial_start()
        except (ValueError, OverflowError) as exc:
            self.notice.set(str(exc))
            self.participant_start_error = True
            return False
        self._clear_rest()
        self.counter = counter
        self.participant_start_error = False
        self.manual_count = 0
        self.state = "countdown"
        self.deadline = time.monotonic() + COUNTDOWN_SEC
        self.status_label.set("HOLD AT NEUTRAL")
        self.notice.set("Hold the object at neutral. The orientation baseline is captured when the countdown ends.")
        self.set_controls()
        self.update_progress()
        self.rest_view.place(relx=0, rely=0, relwidth=1, relheight=1)
        self.rest_view.lift()
        self._update_preparation(time.monotonic())
        self.part_instruction.configure(text=self.participant_instruction(time.monotonic(), tracking=True))
        return True

    def _update_preparation(self, now):
        if self.state != "countdown":
            return
        self.rest_view.set_preparation(
            remaining=max(1, min(COUNTDOWN_SEC, math.ceil(self.deadline - now))),
            total=COUNTDOWN_SEC, instruction=self.participant_instruction(now, tracking=True),
        )

    def begin_recording(self):
        self.rest_view.place_forget()
        try:
            # Recheck after the countdown; capture R0 from the most recent fresh TF.
            pose = self.preflight(self.bridge.snapshot())
            when = datetime.now(self.zone)
            user = validate_user(self.user.get())
            recording_enabled = not self.args.demo or self.record_demo_data.get()
            if recording_enabled and (self.session_path is None or self.session_user != user):
                session_path = create_session(self.args.data_root, user, when)
                session = {"user_number": user, "started_local": when.isoformat(), "timezone": self.args.timezone,
                           "demo": self.args.demo, "protocol_file": str(self.args.protocol),
                           "rest_config_file": str(self.args.rest_config), "rest_seconds": asdict(self.rest_times),
                           "trials": [asdict(t) for t in self.trials]}
                (session_path / "session_metadata.json").write_text(json.dumps(session, indent=2), encoding="utf-8")
                self.session_path, self.session_user = session_path, user
            self.trial_path = (create_trial_directory(self.session_path, user, self.trial, when)
                               if recording_enabled else None)
            self.baseline = pose.quaternion
            self.started = time.monotonic()
            self.started_utc = utc_now()
            self.tracking_interruptions = 0
            self.ft_interruptions = {finger: 0 for finger in FINGERS}
            self.ft_stale = {finger: False for finger in FINGERS}
            self.quality_issues = []
            self.start_counters = self.bridge.counters()
            self.last_pose_drops = self.start_counters["pose_drops"]
            metadata = {"user_number": user, "trial": asdict(self.trial), "started_local": when.isoformat(),
                        "started_utc": self.started_utc, "timezone": self.args.timezone, "demo": self.args.demo,
                        "data_recorded": recording_enabled, "demo_cycle_count_skipped": False,
                        "target_cycles": 20, "target_deg": self.counter.target_deg, "neutral_deg": self.counter.neutral_deg,
                        "dwell_sec": self.counter.dwell_sec, "count_mode": "automatic_rotation" if self.trial.axis else "manual_gaiting",
                        "mesh": str(self.args.mesh_dir / f"{self.trial.diameter_mm}mm.stl"),
                        "mesh_to_sensor4_matrix": self.alignment.tolist(), "baseline_xyzw": list(self.baseline),
                        "baseline_source_timestamp_sec": pose.source_timestamp,
                        "orientation_tf": {"parent": "polhemus_base", "child": "sensor4"},
                        "relative_orientation_formula": "A.T @ R0.T @ Rt @ A",
                        "force_unit": "N", "torque_unit": "N*mm", "wrench_axes": "finger frame",
                        "torque_origin": "ATI sensing origin; axes rotated only; no moment-arm translation",
                        "topics": WRENCH_TOPICS, "sensor_mapping": SENSORS, "require_all_ft": self.require_ft.get(),
                        "acquisition_counters_start": self.start_counters}
            self.writer = (TrialWriter(self.trial_path, metadata) if recording_enabled else PreviewWriter(metadata))
            if not self.writer.ready.wait(timeout=2):
                raise OSError("Timed out opening trial files.")
            if self.writer.error:
                raise OSError(self.writer.error)
            # Saving a baseline row also makes the quaternion reference explicit in CSV.
            self.writer.add("pose", None, (pose.source_timestamp, *pose.quaternion))
            self.bridge.begin(self.writer, self.trial.axis, self.counter.target_deg,
                              alignment=self.alignment, baseline=self.baseline)
            self.state = "recording"
            self.previous_tracking = True
            self.rotation = np.eye(3)
            self.angle = 0.0
            self.info_vars["start"].set(when.strftime("%H:%M:%S  %Z"))
            self.folder_label.set(str(self.trial_path) if recording_enabled else
                                  "Demo preview · data recording disabled · no files are being written.")
            self.status_label.set("RECORDING" if recording_enabled else "DEMO PREVIEW · NOT RECORDING")
            self.notice.set("Recording all incoming F/T samples. Orientation baseline reset for this trial." if
                            recording_enabled else "Demo preview started. No CSV or metadata files are written.")
            self.set_controls()
        except (OSError, ValueError) as exc:
            if self.writer and not self.writer.finished.is_set():
                self.writer.finish("start_failed", {"start_error": str(exc)})
                self.state = "saving"
            else:
                self.state = "ready"
                self.writer = None
            self.notice.set("Could not start: " + str(exc))
            self.participant_start_error = True
            self.status_label.set("START FAILED")
            self.set_controls()

    def process_poses(self, poses, now, allow_finish=True):
        if self.baseline is None:
            return
        if self.state == "recording" and self.bridge.snapshot()["pose_drops"] != self.last_pose_drops:
            self.counter.reset_partial()
            self.last_pose_drops = self.bridge.snapshot()["pose_drops"]
        for pose in poses:
            if pose.arrival < (self.started or 0):
                continue
            try:
                rotation = relative_rotation(self.baseline, pose.quaternion, self.alignment)
                angle = signed_angle_degrees(rotation, self.trial.axis) if self.trial.axis else 0.0
            except ValueError:
                if self.state == "recording":
                    self.counter.reset_partial()
                continue
            self.rotation, self.angle = rotation, angle
            if self.state == "recording":
                if self.trial.axis:
                    self.counter.update(angle, pose.source_timestamp)
                self.writer.add("progress", None, (pose.source_timestamp, pose.arrival - self.started, angle, self.count,
                                                   self.counter.phase if self.trial.axis else "manual_gaiting"))
                if self.count >= 20 and allow_finish:
                    self.finish_trial("completed")
                    break

    def observe_ft_quality(self, snapshot, now):
        """Also retain interruptions that have not recovered by the end of a trial."""
        for finger, item in snapshot["fingers"].items():
            stale = item["value"] is None or now - item["arrival"] > STALE_SEC
            if stale and not self.ft_stale[finger]:
                self.ft_interruptions[finger] += 1
            self.ft_stale[finger] = stale

    def manual_cycle(self):
        if self.state != "recording" or self.trial.axis:
            return
        pose = self.bridge.snapshot()["pose"]
        if pose is None or time.monotonic() - pose.arrival > STALE_SEC:
            self.notice.set("Tracking unavailable. Restore sensor 4 before counting a cycle.")
            return
        self.manual_count = min(20, self.manual_count + 1)
        self.writer.add("progress", None, (pose.source_timestamp, time.monotonic() - self.started, "", self.count, "manual_cycle"))
        if self.count == 20:
            self.finish_trial("completed")
        self.update_progress()

    def finish_trial(self, result, update_ui=True):
        if self.state == "countdown":
            self.state = "ready"
            if update_ui:
                self.rest_view.place_forget()
                self.status_label.set("READY")
                self.notice.set("Countdown cancelled. No trial was recorded.")
                self.set_controls()
            return
        if self.state != "recording":
            return
        _, poses = self.bridge.end_with_poses()
        # Consume already acquired poses before committing the final count.
        self.process_poses(poses, time.monotonic(), allow_finish=False)
        snapshot = self.bridge.snapshot()
        self.observe_ft_quality(snapshot, time.monotonic())
        end_counters = self.bridge.counters()
        self.quality_issues = []
        for finger in FINGERS:
            key = "wrench_" + finger
            if (self.ft_interruptions[finger] or self.ft_stale[finger]
                    or end_counters["gaps"][key] > self.start_counters["gaps"][key]):
                self.quality_issues.append(f"{finger} F/T interrupted")
            if end_counters["invalid"][key] > self.start_counters["invalid"][key]:
                self.quality_issues.append(f"{finger} invalid F/T samples")
        if self.tracking_interruptions:
            self.quality_issues.append("orientation tracking interrupted")
        if end_counters["pose_drops"] > self.start_counters["pose_drops"]:
            self.quality_issues.append("orientation queue overflow")
        if self.count >= 20 and result in ("stopped", "closed", "interrupted", "terminated"):
            result = "completed"
        try:
            notes = self.notes.get().strip()
        except tk.TclError:
            # A caller may already have destroyed Tk before mainloop returned.
            notes = ""
        demo_skipped = result == "demo_skipped"
        if demo_skipped and snapshot["pose"] is not None:
            self.writer.add("progress", None, (snapshot["pose"].source_timestamp,
                                               time.monotonic() - self.started, self.angle,
                                               self.count, "demo_skipped"))
        self.writer.finish(result, {"completed_cycles": self.count, "duration_sec": time.monotonic() - self.started,
                                   "demo_cycle_count_skipped": demo_skipped,
                                   "notes": notes, "tracking_interruptions": self.tracking_interruptions,
                                   "ft_interruptions": dict(self.ft_interruptions), "ft_stale_at_finish": dict(self.ft_stale),
                                   "quality_issues": self.quality_issues,
                                   "acquisition_counters_end": end_counters})
        self.state = "saving"
        if update_ui:
            self.status_label.set("SAVING" if getattr(self.writer, "recording_enabled", True) else "FINISHING PREVIEW")
            self.notice.set("Finishing CSV writes…" if getattr(self.writer, "recording_enabled", True) else
                            "Finishing demo preview…")
            self.set_controls()

    def poll_writer(self):
        if self.state == "recording" and self.writer.error:
            self.finish_trial("write_error")
        if self.state != "saving" or not self.writer or not self.writer.finished.is_set():
            return
        writer = self.writer
        self.state = "finished"
        incomplete = [f for f in FINGERS if writer.counts[f] == 0]
        if writer.error:
            self.status_label.set("SAVE ERROR")
            self.notice.set("Recording error: " + writer.error)
        elif not getattr(writer, "recording_enabled", True):
            self.status_label.set("DEMO PREVIEW · SKIPPED" if writer.result == "demo_skipped" else
                                  "DEMO PREVIEW · COMPLETE" if writer.result == "completed" else
                                  "DEMO PREVIEW · STOPPED")
            self.notice.set("Demo preview finished. No data files were written.")
        elif writer.overflows or incomplete or self.quality_issues:
            self.status_label.set("SAVED · REVIEW DATA")
            issues = list(self.quality_issues)
            if writer.overflows:
                issues.append(f"{writer.overflows} writer queue drops")
            if incomplete:
                issues.append("empty F/T: " + ", ".join(incomplete))
            self.notice.set(f"Saved {self.count}/20 cycles. Review: " + "; ".join(issues) + ".")
        else:
            self.status_label.set("COMPLETE · SAVED" if writer.result == "completed" else
                                  "DEMO SKIPPED · SAVED" if writer.result == "demo_skipped" else "STOPPED · SAVED")
            self.notice.set(f"Saved {self.count}/20 cycles and {sum(writer.counts.values()):,} finger F/T samples.")
        self.set_controls()
        self.update_progress()
        self._begin_rest()

    def _begin_rest(self):
        """Rest after a saved complete trial, using the next protocol boundary."""
        if not self._completed_trial_saved() or self.index >= len(self.trials) - 1:
            return False
        next_index = self.index + 1
        spec = self.rest_times.between(self.trial, self.trials[next_index])
        if spec.total_sec == 0:
            return False
        now = time.monotonic()
        self.rest_countdown = RestCountdown(spec, now)
        self.rest_next_index = next_index
        self.rest_user = self.user.get().strip()
        self.rest_skipped = False
        self.rest_skipped_at = None
        self.rest_screen_visible = True
        self.state = "resting"
        self.rest_view.place(relx=0, rely=0, relwidth=1, relheight=1)
        self.rest_view.lift()
        prefix = "Data saved." if getattr(self.writer, "recording_enabled", True) else "Demo preview finished."
        self.notice.set(f"{prefix} Rest between {spec.kind}: {spec.total_sec} seconds.")
        self.set_controls()
        self._update_rest(now)
        return True

    def _update_rest(self, now):
        if self.rest_countdown is None:
            return
        status = self.rest_countdown.snapshot(self.rest_skipped_at if self.rest_skipped else now)
        if self.rest_skipped:
            status = status._replace(remaining=0, complete=True)
        if self.state == "resting" and status.complete:
            self.state = "finished"
            self.notice.set("Rest skipped. Press SPACE when ready for the next trial." if self.rest_skipped else
                            "Rest complete. Press SPACE in the participant window when ready for the next trial.")
            self.set_controls()
        upcoming = self.trials[self.rest_next_index]
        next_label = f"Next: {TASK_NAMES[upcoming.task]} · {upcoming.diameter_mm} mm · {upcoming.code}"
        instruction = None
        if self.participant_start_error:
            instruction = "Unable to start. Please ask the researcher."
        elif self.user.get().strip() != self.rest_user:
            instruction = "Please wait for the researcher."
        else:
            prefix = "DEMO · " if self.args.demo else ""
            instruction = (f"{prefix}Rest skipped. Press SPACE for the next trial." if self.rest_skipped else
                           "Press SPACE when you are ready for the next trial." if status.complete else
                           f"{prefix}Press SPACE to skip the remaining rest.")
        self.rest_view.set_rest(
            kind=self.rest_countdown.spec.kind, elapsed=status.elapsed,
            total=self.rest_countdown.spec.total_sec, remaining=status.remaining,
            progress=status.progress, complete=status.complete, next_label=next_label,
            instruction=instruction,
            skipped=self.rest_skipped,
        )
        self.status_label.set("REST SKIPPED" if self.rest_skipped else "REST COMPLETE" if status.complete else
                              f"REST · {status.remaining} s remaining")

    def _clear_rest(self):
        if self.rest_screen_visible:
            self.rest_view.place_forget()
        self.rest_screen_visible = False
        self.rest_countdown = None
        self.rest_next_index = None
        self.rest_user = None
        self.rest_skipped = False
        self.rest_skipped_at = None

    def draw_progress(self):
        if not hasattr(self, "part_progress"):
            return
        c = self.part_progress
        c.delete("all")
        w = max(1, c.winfo_width())
        c.create_rectangle(0, 0, w, 12, fill="#E5EEED", outline="")
        if self.count:
            c.create_rectangle(0, 0, w * self.count / 20, 12, fill=TEAL, outline="")

    def update_progress(self):
        self.count_label.set(f"{self.count} / 20")
        self.part_count.configure(text=f"{self.count} / 20")
        self.progress["value"] = self.count
        self.draw_progress()

    def tick(self):
        if self.shutdown_signals is not None and self.shutdown_signals.requested:
            self.close(self.shutdown_signals.reason)
            return
        if self.closing:
            return
        now = time.monotonic()
        snapshot = self.bridge.snapshot()
        pose = snapshot["pose"]
        tracking = pose is not None and now - pose.arrival <= STALE_SEC
        self.pose_label.set("Sensor 4 · tracking" if tracking else "Sensor 4 · OFFLINE")
        self.process_poses(self.bridge.drain_poses(), now)
        if self.state == "recording":
            self.observe_ft_quality(snapshot, now)
            if not tracking and self.previous_tracking:
                self.counter.reset_partial()
                self.tracking_interruptions += 1
        self.previous_tracking = tracking
        if self.state == "countdown" and now >= self.deadline:
            self.begin_recording()
        self._update_preparation(now)
        self.poll_writer()
        self._update_rest(now)
        for finger, item in snapshot["fingers"].items():
            fresh = item["value"] is not None and now - item["arrival"] <= STALE_SEC
            legacy = item["force"] is not None and now - item["force_arrival"] <= STALE_SEC
            values = item["value"][:6] if fresh else ((*item["force"][:3], None, None, None) if legacy else None)
            self.charts[finger].draw(values)
            self.chart_status[finger].set(f"Live  ·  {item['rate']:.0f} Hz" if fresh else "Force only · torque unavailable" if legacy else "Offline · waiting for F/T")
        while not self.events.empty():
            level, message = self.events.get_nowait()
            if self.state == "ready" or level == "error":
                self.notice.set(message)
        self.part_instruction.configure(text=self.participant_instruction(now, tracking))
        target = self.counter.target_deg
        self.part_angle.configure(text=f"{TASK_NAMES[self.trial.task]}  {self.angle:+.1f}°   ·   Target ±{target:g}°   ·   Neutral ±{self.counter.neutral_deg:g}°"
                                  if self.trial.axis else "20 cycles · counted by the researcher")
        if not self.rest_screen_visible and self.state != "countdown":
            self.mesh.set_pose(self.rotation, self.trial.axis, tracking=tracking if self.baseline is not None else True)
        self.update_progress()
        self.root.after(50, self.tick)

    def close(self, reason="closed"):
        if self.closing:
            return
        self.closing = True
        self.close_reason = reason
        # main() owns the synchronous save/join, including exceptional exits.
        self.root.quit()

    def shutdown(self, reason=None):
        """Finish the active trial and join both workers before destroying Tk."""
        if self.shutdown_complete:
            return self.shutdown_ok
        self.closing = True
        reason = reason or self.close_reason
        errors = []
        report("stopping; finishing CSV and metadata writes…")
        try:
            self.finish_trial(reason, update_ui=False)
        except Exception as exc:
            # Even if summarizing the trial fails, preserve already queued data.
            errors.append(f"trial finalization: {exc}")
            try:
                self.bridge.end()
            except Exception as detach_exc:
                errors.append(f"stopping trial acquisition: {detach_exc}")
        try:
            if self.writer is not None:
                extra = {"shutdown_errors": list(errors)} if errors else None
                # Idempotent: keep an earlier 'completed' or 'start_failed' result.
                self.writer.finish("shutdown_error" if errors else reason, extra)
                self.writer.wait()
                if self.writer.error:
                    errors.append(f"saving {self.writer.path}: {self.writer.error}")
                elif getattr(self.writer, "recording_enabled", True):
                    report(f"saved {self.writer.path}")
                    if self.writer.overflows:
                        report(f"review data; {self.writer.overflows} samples were dropped by the writer queue.", error=True)
                else:
                    report("demo preview ended; no data files were written.")
        except Exception as exc:
            errors.append(f"CSV writer shutdown: {exc}")
        finally:
            try:
                self.bridge.stop()
            except Exception as exc:
                errors.append(f"acquisition shutdown: {exc}")
        # ROS cleanup errors are queued by its worker after all cleanup attempts.
        while not self.events.empty():
            level, message = self.events.get_nowait()
            if level == "error":
                errors.append(message)
        self.shutdown_complete = True
        self.shutdown_ok = not errors
        for message in errors:
            report(message, error=True)
        return self.shutdown_ok


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--demo", action="store_true", help="Simulate sensors; default output is data/demo/.")
    parser.add_argument("--data-root", type=Path, default=None)
    parser.add_argument("--protocol", type=Path, default=PROJECT_ROOT / "docs/user_study_docs/Trials_lookup.csv")
    parser.add_argument("--rest-config", type=Path, default=PROJECT_ROOT / "docs/user_study_docs/Rest_Time.py",
                        help="Rest durations in minutes: trials, tasks, objects.")
    parser.add_argument("--mesh-dir", type=Path, default=PROJECT_ROOT / "utils/object_meshes")
    parser.add_argument("--timezone", default="America/New_York", help="Timezone for session and trial folder names.")
    parser.add_argument("--mesh-alignment-rpy", type=float, nargs=3, default=(0, 0, 0), metavar=("RX", "RY", "RZ"),
                        help="Degrees; Rz Ry Rx maps STL axes into sensor 4 axes (default: aligned).")
    args = parser.parse_args(argv)
    args.data_root = (args.data_root or (DATA_DIR / "demo" if args.demo else DATA_DIR)).expanduser().resolve()
    args.protocol = args.protocol.expanduser().resolve()
    args.rest_config = args.rest_config.expanduser().resolve()
    args.mesh_dir = args.mesh_dir.expanduser().resolve()
    return args


def main(argv=None):
    args = parse_args(argv)
    root = app = None
    status = 0
    with ShutdownSignals() as shutdown_signals:
        try:
            root = tk.Tk()

            def callback_error(_kind, exc, _traceback):
                nonlocal status
                status = 1
                report(str(exc), error=True)
                if app is not None:
                    app.close("error")
                else:
                    root.quit()

            root.report_callback_exception = callback_error
            app = StudyApplication(root, args, shutdown_signals=shutdown_signals)
            root.mainloop()
        except KeyboardInterrupt:
            # Fallback for a dependency explicitly raising KeyboardInterrupt.
            if app is not None:
                app.close_reason = "interrupted"
        except Exception as exc:
            status = 1
            if app is not None:
                app.close_reason = "error"
            report(str(exc), error=True)
        finally:
            try:
                if app is not None:
                    reason = shutdown_signals.reason if shutdown_signals.requested else app.close_reason
                    if not app.shutdown(reason):
                        status = 1
            finally:
                if root is not None:
                    try:
                        root.destroy()
                    except tk.TclError:
                        pass  # Tk may already have been destroyed by its owner.
    return status


if __name__ == "__main__":
    raise SystemExit(main())
