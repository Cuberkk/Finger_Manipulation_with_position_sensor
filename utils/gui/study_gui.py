#!/usr/bin/env python3
"""Two-window experiment console: python -m utils.gui.study_gui [--demo]."""
from __future__ import annotations

import argparse
from copy import deepcopy
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

from utils.paths import GUI_CONFIG_FILE, PROJECT_ROOT, STUDY_DATA_DIR
from utils.gui.mesh_view import MeshView, mesh_asset_path
from utils.gui.advisor_layout import AdvisorBody
from utils.gui.force_visualization import ForceWaveformView
from utils.gui.force_release import total_force_magnitude
from utils.gui.responsive import ResponsiveFonts
from utils.gui.rest import DEFAULT_RELEASE_FORCE_THRESHOLD_N, RestCountdown, load_study_settings
from utils.gui.rest_view import RestCountdownView
from utils.gui.shutdown import ShutdownSignals
from utils.gui.study_model import (
    RotationAxisTracker, RotationCounter, RotationRangeCalibration, load_trials,
    relative_rotation,
)
from utils.gui.study_io import (
    FINGERS, SENSORS, FORCE_TOPICS, RAW_TOPICS, STALE_SEC, ROSBridge, TrialWriter, PreviewWriter,
    create_session, create_trial_directory, validate_user, utc_now,
    force_bias_record, unique_directory,
)

NAVY = "#0E1726"
INK = "#17273D"
MUTED = "#65758A"
TEAL = "#147D79"
LIGHT = "#F0F4F7"
PARTICIPANT_BG = "#FFFFFF"
TASK_NAMES = {"roll": "Roll", "pitch": "Pitch", "yaw": "Yaw", "finger_gating": "Finger gaiting"}


def report(message, error=False):
    """A closed terminal/pipe must not interrupt data or resource cleanup."""
    try:
        print(f"Study GUI: {message}", file=sys.stderr if error else sys.stdout, flush=True)
    except (OSError, ValueError):
        pass


class ForceChart(tk.Canvas):
    """Three horizontal finger-force magnitude bars with sign colors."""

    POSITIVE_COLOR = "#DC2626"
    NEGATIVE_COLOR = "#2563EB"
    ZERO_COLOR = "#94A3B8"

    def __init__(self, master, *, force_limit_n=10.0):
        super().__init__(master, background="white", highlightthickness=0, height=110, width=280)
        self.force_limit_n = float(force_limit_n)
        self.font_scale = 1.0
        self.bind("<Configure>", lambda _: self.draw(self.last))
        self.last = None

    def set_font_scale(self, scale):
        self.font_scale = scale
        self.configure(height=round(110 * scale))
        self.draw(self.last)

    def draw(self, values):
        self.last = values
        self.delete("all")
        w, h = self.winfo_width(), self.winfo_height()
        if w < 80 or h < 65:
            return
        s = self.font_scale
        heading_height = 25 * s
        row_height = max(8, (h - heading_height - 5 * s) / 3)
        text_scale = min(s, max(.7, row_height / 14))
        def font(size, bold=False):
            return ("DejaVu Sans", max(7, round(size * text_scale)), "bold" if bold else "normal")
        bar_left, bar_right = 43 * s, max(43 * s + 8, w - 66 * s)
        self.create_text(12 * s, 10 * s, text="FORCE · N", anchor="w", fill=MUTED, font=font(9, True))
        self.create_text(w - 12 * s, 10 * s, text=f"0-{self.force_limit_n:g}", anchor="e",
                         fill=MUTED, font=font(8))
        half_bar_height = max(2, min(7 * s, row_height * .3))
        for axis in range(3):
            y = heading_height + (axis + .5) * row_height
            self.create_rectangle(bar_left, y - half_bar_height, bar_right, y + half_bar_height,
                                  fill="#EDF1F5", outline="", tags="bar-track")
            value = None if values is None else values[axis]
            if value is not None and not math.isfinite(value):
                value = None
            color = self.ZERO_COLOR
            if value is not None:
                color = (self.POSITIVE_COLOR if value > 0 else
                         self.NEGATIVE_COLOR if value < 0 else self.ZERO_COLOR)
                fraction = min(1.0, abs(value) / self.force_limit_n)
                self.create_rectangle(bar_left, y - half_bar_height,
                                      bar_left + fraction * (bar_right - bar_left), y + half_bar_height,
                                      fill=color, outline="", tags="magnitude-bar")
            self.create_text(12 * s, y, text="F" + "xyz"[axis],
                             anchor="w", fill=INK, font=font(9, True))
            self.create_text(w - 12 * s, y, text="—" if value is None else f"{value:+.2f}",
                             anchor="e", fill=MUTED if value is None else color, font=font(9))


class StudyApplication:
    def __init__(self, root, args, shutdown_signals=None):
        self.root, self.args = root, args
        self.shutdown_signals = shutdown_signals
        self.close_reason = "closed"
        self.shutdown_complete = False
        self.shutdown_ok = True
        self.trials = load_trials(args.protocol)
        settings = load_study_settings(args.rest_config)
        self.rest_times = settings.rest_times
        self.pickup_sec = settings.pickup_sec
        self.hold_neutral_sec = settings.hold_neutral_sec
        self.preparation_sec = self.pickup_sec + self.hold_neutral_sec
        self.force_limit_n = settings.force_limit_n
        self.torque_limit_nmm = settings.torque_limit_nmm
        self.release_force_threshold_n = settings.release_force_threshold_n
        self.gui_config_metadata = {
            "gui_config_file": str(args.rest_config),
            "configured_tf_wait_tolerance_ms": settings.tf_wait_tolerance_ms,
            "release_force_threshold_n": settings.release_force_threshold_n,
            "preparation_seconds": {"pickup": self.pickup_sec, "hold_neutral": self.hold_neutral_sec},
            "advisor_chart_limits": {
                "force_limit_n": self.force_limit_n,
                "torque_limit_nmm": self.torque_limit_nmm,
                "range_mode": "absolute_magnitude", "orientation": "horizontal",
                "positive_color": "red", "negative_color": "blue",
            },
        }
        self.practice_targets_deg = dict(settings.practice_targets_deg)
        missing_sizes = {trial.diameter_mm for trial in self.trials if trial.axis} - self.practice_targets_deg.keys()
        if missing_sizes:
            raise ValueError(f"Missing PRACTICE_TARGETS_DEG in {args.rest_config} for diameters: "
                             + ", ".join(str(size) for size in sorted(missing_sizes)))
        self.rest_countdown = None
        self.rest_next_index = None
        self.rest_user = None
        self.rest_screen_visible = False
        self.rest_skipped = False
        self.rest_skipped_at = None
        self.total_trials = sum(not trial.practice for trial in self.trials)
        self.zone = ZoneInfo(args.timezone)
        self.alignment = settings.sensor4_from_object.copy()
        self.index, self.state = 0, "ready"
        self.baseline = None
        self.counter = RotationCounter()
        self.axis_tracker = None
        self.calibrations = {}
        self.calibrating = False
        self.range_calibration = None
        self.manual_count = 0
        self.angle = 0.0
        self._reset_force_release()
        self.rotation = np.eye(3)
        self.started = None
        self.session_path = self.session_user = None
        self.session_metadata_path = None
        self.writer = None
        self.trial_path = None
        self.closing = False
        self.participant_start_error = False
        self._participant_force_bias_error = False
        self._trial_force_bias_request = None
        self._trial_force_bias_approval = None
        self._space_held = False
        self._space_release = None
        self.previous_tracking = False
        self.tracking_interruptions = 0
        self.last_pose_drops = 0
        self.ft_interruptions = {finger: 0 for finger in FINGERS}
        self.ft_stale = {finger: False for finger in FINGERS}
        self.quality_issues = []
        self.events = queue.Queue()
        self.waveform_view = None
        self._waveform_last_refresh = None
        self.user = tk.StringVar(value="001")
        self.user.trace_add("write", lambda *_: self.refresh_identity())
        self.selection = tk.StringVar()
        self.target = tk.StringVar()
        self.neutral = tk.StringVar(value="5")
        self.require_ft = tk.BooleanVar(value=True)
        self.record_demo_data = tk.BooleanVar(value=True)
        self.record_demo_box = None
        self.force_bias_label = tk.StringVar(value="Demo · simulated force data" if args.demo else
                                            "Waiting for the force processor…")
        self.notice = tk.StringVar(value="Demo mode: simulated orientation and force data." if args.demo else "Connecting to ROS 2…")
        self.start_label = tk.StringVar(value="—")
        self.folder_label = tk.StringVar(value="Participant and trial folders are created when recording starts.")
        self.status_label = tk.StringVar(value="READY")
        self.count_label = tk.StringVar(value="0 / 20")
        self.pose_label = tk.StringVar(value="Sensor 4 · waiting")
        self.axis_motion_label = tk.StringVar(value="Axis error: — · Start trial to measure")
        self._build_researcher()
        self._build_participant()
        self._select_trial(0)
        self.advisor_fonts = ResponsiveFonts(root, base_size=(1240, 1080), on_scale=self._scale_advisor)
        self.participant_fonts = ResponsiveFonts(self.participant, base_size=(1000, 850), on_scale=self._scale_participant)
        for window in (root, self.participant):
            window.bind("<F11>", lambda _, w=window: w.attributes("-fullscreen", not w.attributes("-fullscreen")))
            window.bind("<Escape>", lambda _, w=window: w.attributes("-fullscreen", False))
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(50, self.tick)
        # Start the worker last: widget construction failures cannot orphan it.
        self.bridge = ROSBridge(args.demo, self.events,
                                release_force_threshold_n=self.release_force_threshold_n)

    @property
    def trial(self):
        return self.trials[self.index]

    @property
    def count(self):
        return self.counter.count if self.trial.axis else self.manual_count

    def label(self, parent, text=None, variable=None, size=11, color=INK, bg="white", bold=False, **kwargs):
        return tk.Label(parent, text=text, textvariable=variable, font=("DejaVu Sans", size, "bold" if bold else "normal"),
                        fg=color, bg=bg, **kwargs)

    def _create_force_chart(self, parent):
        """Give every finger chart the same configured physical display ranges."""
        return ForceChart(parent, force_limit_n=self.force_limit_n)

    def _create_force_waveform_view(self, parent):
        """Share the bar charts' physical limits with signed waveforms."""
        return ForceWaveformView(parent, force_limit_n=self.force_limit_n,
                                 torque_limit_nmm=self.torque_limit_nmm)

    def _build_researcher(self):
        r = self.root
        r.title("ERIE · Study advisor" + (" · DEMO" if self.args.demo else ""))
        advisor_height = min(1080, max(790, r.winfo_screenheight() - 90))
        r.geometry(f"1240x{advisor_height}+30+30")
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
        self.advisor_body = AdvisorBody(r, bg=LIGHT)
        self.advisor_body.pack(fill="both", expand=True)
        body = self.advisor_body.body
        body.configure(bg=LIGHT, padx=24, pady=12)
        body.columnconfigure(0, weight=1)
        body.rowconfigure(3, weight=1, minsize=420)
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
        self._build_force_bias_controls(setup_panel)
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
        self.label(body, "REALTIME FINGER FORCE", bg=LIGHT, size=10, bold=True).grid(row=2, column=0, sticky="w", pady=(1, 6))
        self.signal_panes = ttk.Panedwindow(body, orient=tk.VERTICAL)
        self.signal_panes.grid(row=3, column=0, sticky="nsew")
        charts = tk.Frame(self.signal_panes, bg=LIGHT)
        self.signal_panes.add(charts, weight=0)
        self.charts, self.chart_status = {}, {}
        for i, finger in enumerate(FINGERS):
            charts.columnconfigure(i, weight=1, uniform="charts")
            card = tk.Frame(charts, bg="white", padx=8, pady=8)
            card.grid(row=0, column=i, sticky="nsew", padx=(0 if i == 0 else 6, 0 if i == 2 else 6))
            card_header = tk.Frame(card, bg="white")
            card_header.pack(fill="x")
            self.label(card_header, finger.title(), size=12, bold=True).pack(side="left")
            self.label(card_header, f"Finger frame · Sensor {SENSORS[finger]}", size=8,
                       color=MUTED).pack(side="right")
            chart = self._create_force_chart(card)
            chart.pack(fill="both", expand=True)
            self.charts[finger] = chart
            self.chart_status[finger] = tk.StringVar(value="Waiting for force")
            self.label(card, variable=self.chart_status[finger], size=9, color=MUTED).pack(anchor="w")
        charts.rowconfigure(0, weight=1)
        self.waveform_view = self._create_force_waveform_view(self.signal_panes)
        self.signal_panes.add(self.waveform_view, weight=1)
        self.label(body, "Finger-force bars show magnitude; red = positive, blue = negative. Waveforms share one force axis with scaled torque.",
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
        self.label(control, variable=self.axis_motion_label, size=10, color=MUTED).pack(anchor="w", pady=(0, 8))
        settings = tk.Frame(control, bg="white")
        settings.pack(fill="x")
        self.setting_entries = []
        for title, variable, width in (("Target ±°", self.target, 12), ("Neutral ±°", self.neutral, 5)):
            self.label(settings, title, size=9).pack(side="left", padx=(0, 6))
            entry = ttk.Entry(settings, textvariable=variable, width=width)
            entry.pack(side="left", padx=(0, 16))
            self.setting_entries.append(entry)
        self.target_entry = self.setting_entries[0]
        self.require_box = tk.Checkbutton(settings, text="Require all 3 force streams", variable=self.require_ft, bg="white",
                                          fg=INK, activebackground="white", font=("DejaVu Sans", 9))
        self.require_box.pack(side="left")
        buttons = tk.Frame(control, bg="white")
        buttons.pack(fill="x", pady=(8, 0))
        self.start_button = ttk.Button(buttons, text=f"Start trial  ·  {self.preparation_sec}s", style="Accent.TButton", command=self.start_trial)
        self.start_button.pack(side="left", padx=(0, 8))
        self.stop_button = ttk.Button(buttons, text="Stop & save", command=self.stop_current, state="disabled")
        self.stop_button.pack(side="left", padx=(0, 8))
        self.cancel_calibration_button = ttk.Button(buttons, text="Cancel angle test",
                                                   command=lambda: self.finish_trial("calibration_cancelled"),
                                                   state="disabled")
        self.cancel_calibration_button.pack(side="left", padx=(0, 8))
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

    def _build_force_bias_controls(self, parent):
        row = tk.Frame(parent, bg="white", padx=12, pady=6)
        row.pack(fill="x", pady=(8, 0))
        self.force_bias_button = ttk.Button(row, text="Force bias", command=self.request_force_bias,
                                           state="disabled")
        self.force_bias_button.pack(side="left", padx=(0, 12))
        status = tk.Frame(row, bg="white")
        status.pack(side="left", fill="x", expand=True)
        self.force_bias_status_widget = self.label(status, variable=self.force_bias_label, size=10,
                                                  color=TEAL, anchor="w", justify="left", wraplength=900)
        self.force_bias_status_widget.pack(fill="x")
        self.label(status, "Keep all 3 force sensors unloaded and stationary during bias.",
                   size=9, color=MUTED).pack(anchor="w")

    def _build_participant(self):
        p = self.participant = tk.Toplevel(self.root)
        p.title("ERIE · Participant")
        p.geometry("1000x850+160+90")
        p.minsize(720, 650)
        p.configure(bg=PARTICIPANT_BG)
        p.protocol("WM_DELETE_WINDOW", self.close)
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
        self.part_axis_motion = self.label(self.participant_body, size=11, color=TEAL, bg=PARTICIPANT_BG)
        self.part_axis_motion.pack(pady=(5, 0))
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
        waveform_view = getattr(self, "waveform_view", None)
        if waveform_view is not None:
            waveform_view.set_font_scale(scale)
        width = max(300, self.root.winfo_width() - 48)
        self.folder_widget.configure(wraplength=width)
        self.notice_widget.configure(wraplength=width)
        self.force_bias_status_widget.configure(wraplength=max(250, width - 160))

    def _scale_participant(self, scale):
        self.mesh.set_font_scale(scale)
        self.rest_view.set_font_scale(scale)
        width = max(300, self.participant.winfo_width() - 72)
        self.part_instruction.configure(wraplength=width)
        self.part_angle.configure(wraplength=width)
        self.part_axis_motion.configure(wraplength=width)

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
                and self.writer.metadata.get("recording_kind") != "rotation_calibration"
                and self.writer.finished.is_set() and not self.writer.error
                and self.writer.metadata.get("user_number") == self.user.get().strip()
                and ((self.writer.result == "completed" and self.count >= 20)
                     or (self.writer.result == "demo_skipped" and self.args.demo)))

    def participant_proceed(self, event=None):
        if self.closing:
            return "break"
        if self.state in ("ready", "finished", "resting") and self._force_bias_busy():
            self.notice.set("Force bias is in progress. Keep all 3 sensors unloaded and wait before starting.")
            return "break"
        if self.args.demo:
            if self.state == "countdown":
                now = time.monotonic()
                if now < self.deadline - self.hold_neutral_sec:
                    self.deadline = now + self.hold_neutral_sec
                    self._update_preparation(now)
                    self.part_instruction.configure(text=self.participant_instruction(now, tracking=True))
                else:
                    self.begin_recording()
                return "break"
            if self.state == "recording" and not self.calibrating:
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
            self._participant_force_bias_error = False
            try:
                # Keep the completed trial selected if settings or sensors
                # currently prevent starting the next one.
                self._check_trial_force_bias(self.bridge.snapshot(), index=self.index + 1)
                self._prepare_trial_start()
            except (ValueError, OverflowError) as exc:
                self.notice.set(str(exc))
                self.participant_start_error = True
                return "break"
            self._select_trial(self.index + 1)
        self.start_trial()
        return "break"

    def participant_instruction(self, now, tracking):
        if self._force_bias_busy() and self.state not in ("recording", "saving"):
            return "Force bias is in progress. Keep all force sensors unloaded and wait for the researcher."
        if self.state == "resting":
            return "Please rest. Press SPACE to skip the remaining rest."
        if self.state == "countdown":
            suffix = " · SPACE to skip · DEMO" if self.args.demo else ""
            if now < self.deadline - self.hold_neutral_sec:
                return f"Pick up the object.{suffix}"
            purpose = " Angle test starts next." if self.calibrating else ""
            return f"Hold the object at neutral.{purpose}{suffix}"
        if self.state == "recording":
            if self.calibrating:
                if not tracking or not self.range_calibration.continuous:
                    return "Tracking interrupted. Ask the researcher to repeat the angle test."
                return ("Rotate as far as you can in each direction around the highlighted axis, "
                        "returning to neutral between directions. The researcher will finish the test.")
            if self.trial.axis and getattr(self, "_force_released", False):
                return "Object released. Pick it up and continue using the original neutral."
            if self.args.demo:
                return "DEMO · Press SPACE to skip this trial's cycle counting."
            return ("Tracking lost · pause movement" if not tracking else
                    "Rotate smoothly around the highlighted axis." if self.trial.axis else
                    "Complete the gaiting movement.")
        if self.state == "saving":
            if self.calibrating:
                return "Finishing angle test…"
            if not getattr(self.writer, "recording_enabled", True):
                return "Finishing demo preview…"
            if self.writer.result == "demo_skipped":
                return "Demo trial skipped · saving"
            return "Trial complete · saving" if self.count == 20 else "Trial stopped · saving"
        if self.participant_start_error:
            target = self._force_bias_target_index()
            if (self.state in ("ready", "finished") and self._requires_trial_force_bias(target)
                    and not self._trial_force_bias_ready(target)):
                return "Please wait for the researcher to complete Force bias before the next trial."
            return "Unable to start. Please ask the researcher."
        if self.state == "finished":
            if not self._completed_trial_saved():
                return "Please wait for the researcher."
            if self.index == len(self.trials) - 1:
                return "Study complete. Thank you!"
            if (self._requires_trial_force_bias(self.index + 1)
                    and not self._trial_force_bias_ready(self.index + 1)):
                return "Trial complete. Please wait for the researcher to complete Force bias before the next trial."
            return "Trial complete. Press SPACE when you are ready for the next trial."
        if self.needs_calibration():
            return "Angle test required before this task. Press SPACE when you are ready to pick up the object."
        if self._requires_trial_force_bias() and not self._trial_force_bias_ready():
            return "Please wait for the researcher to complete Force bias before this trial."
        return "Press SPACE when you are ready to start."

    def calibration_key(self):
        return (self.user.get().strip(), self.trial.diameter_mm, self.trial.task)

    def current_calibration(self):
        return self.calibrations.get(self.calibration_key())

    def needs_calibration(self):
        return bool(self.trial.axis and not self.trial.practice and self.current_calibration() is None)

    def refresh_target(self):
        if not self.trial.axis:
            self.target.set("N/A")
        elif self.trial.practice:
            self.target.set(f"{self.practice_targets_deg[self.trial.diameter_mm]:g}")
        else:
            calibration = self.current_calibration()
            self.target.set(f"{calibration['target_deg']:.2f}" if calibration else "Test required")

    def refresh_identity(self):
        self._retain_trial_force_bias(self.index)
        self._force_bias_controls = None
        if hasattr(self, "info_vars"):
            self.info_vars["user"].set(self.user.get())
        if hasattr(self, "part_user"):
            self.part_user.configure(text=f"User Number: {self.user.get()}")
        if hasattr(self, "target"):
            self.refresh_target()
        if hasattr(self, "start_button"):
            self.start_button.configure(text=f"Start angle test  ·  {self.preparation_sec}s" if self.needs_calibration() else
                                        f"Start trial  ·  {self.preparation_sec}s")

    def _select_trial(self, index):
        if self.state in ("countdown", "recording", "saving", "resting") or not 0 <= index < len(self.trials):
            return
        self._clear_rest()
        self.index, self.state = index, "ready"
        self._retain_trial_force_bias(index)
        self.calibrating = False
        self.range_calibration = None
        self.participant_start_error = False
        self._participant_force_bias_error = False
        self.selection.set(f"{self.trial.reference}  ·  {self.trial.code}")
        self.counter = RotationCounter()
        self.axis_tracker = RotationAxisTracker(self.trial.axis) if self.trial.axis else None
        self.manual_count, self.baseline, self.angle = 0, None, 0.0
        self._reset_force_release()
        self.axis_motion_label.set(self.axis_motion_text())
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
        axis = f"{TASK_NAMES[self.trial.task]}  /  Object {self.trial.axis.upper()} axis" if self.trial.axis else "Finger gaiting  /  researcher counts"
        self.part_axis.configure(text=axis)
        self.part_axis_motion.configure(text=self.axis_motion_text(participant=True))
        self.mesh.set_diameter(self.trial.diameter_mm)
        self.mesh.set_pose(np.eye(3), self.trial.axis, tracking=True)
        self.status_label.set("READY")
        self.notice.set(self.mesh.error or ("Ready. Start to pick up the object, then hold at neutral when prompted." if self.trial.axis else "Gaiting: the researcher records each completed cycle with +1."))
        self.update_progress()
        self.set_controls()
        self.part_instruction.configure(text=self.participant_instruction(time.monotonic(), tracking=True))

    def set_controls(self):
        busy = self.state in ("countdown", "recording", "saving", "resting")
        force_bias = self._force_bias_snapshot()
        bias_busy = self._force_bias_busy(force_bias=force_bias)
        bias_required = (self._requires_trial_force_bias()
                         and not self._trial_force_bias_ready(force_bias=force_bias))
        for widget in [self.user_entry, *self.setting_entries, self.require_box]:
            widget.configure(state="disabled" if busy else "normal")
        self.trial_select.configure(state="disabled" if busy else "readonly")
        self.start_button.configure(state="disabled" if busy or self.closing or bias_busy or bias_required else "normal",
                                    text=f"Start angle test  ·  {self.preparation_sec}s" if self.needs_calibration() else
                                    f"Start trial  ·  {self.preparation_sec}s")
        self.target_entry.configure(state="disabled" if busy else "normal" if self.trial.practice else "readonly")
        self.cancel_calibration_button.configure(state="normal" if self.calibrating and self.state == "recording" else "disabled")
        if getattr(self, "record_demo_box", None) is not None:
            self.record_demo_box.configure(state="disabled" if self.state in
                                           ("countdown", "recording", "saving") else "normal")
        self.stop_button.configure(state="normal" if self.state in ("countdown", "recording") else "disabled",
                                   text="Cancel countdown" if self.state == "countdown" else
                                   "Finish angle test" if self.calibrating else
                                   "Stop preview" if self.writer is not None and
                                   not getattr(self.writer, "recording_enabled", True) else "Stop & save")
        self.manual_button.configure(state="normal" if self.state == "recording" and not self.trial.axis else "disabled")
        self.next_button.configure(state="normal" if not busy and self.index < len(self.trials) - 1 else "disabled")
        self.previous_button.configure(state="normal" if not busy and self.index > 0 else "disabled")
        if getattr(self, "force_bias_button", None) is not None:
            allowed = (not self.args.demo and not self.closing and
                       self.state not in ("countdown", "recording", "saving") and
                       force_bias is not None and force_bias.get("available", False) and not bias_busy)
            self.force_bias_button.configure(state="normal" if allowed else "disabled")

    def _force_bias_snapshot(self, snapshot=None):
        if snapshot is None:
            bridge = getattr(self, "bridge", None)
            if bridge is None:
                return None
            snapshot = bridge.snapshot()
        status = snapshot.get("force_bias")
        return status if isinstance(status, dict) else None

    def _force_bias_busy(self, snapshot=None, *, force_bias=None):
        if getattr(getattr(self, "args", None), "demo", False):
            return False
        if force_bias is None:
            force_bias = self._force_bias_snapshot(snapshot)
        return bool(force_bias and (force_bias.get("pending") or force_bias.get("busy")))

    def _requires_trial_force_bias(self, index=None):
        """Require a new advisor bias for measured trials after angle testing."""
        trial = self.trials[self.index if index is None else index]
        key = (self.user.get().strip(), trial.diameter_mm, trial.task)
        return (not self.args.demo and not trial.practice
                and (not trial.axis or self.calibrations.get(key) is not None))

    def _trial_force_bias_scope(self, index):
        return self.user.get().strip(), index

    def _force_bias_target_index(self):
        """Bias during the completed trial's rest prepares the upcoming trial."""
        if self.state == "resting":
            return self.rest_next_index
        if self._completed_trial_saved() and self.index + 1 < len(self.trials):
            return self.index + 1
        return self.index

    def _retain_trial_force_bias(self, index):
        """Discard approval when selecting a different participant or trial."""
        scope = self._trial_force_bias_scope(index)
        request = getattr(self, "_trial_force_bias_request", None)
        approval = getattr(self, "_trial_force_bias_approval", None)
        if request is not None and request["scope"] != scope:
            self._trial_force_bias_request = None
        if approval is not None and approval[0] != scope:
            self._trial_force_bias_approval = None

    def _trial_force_bias_ready(self, index=None, *, force_bias=None):
        """Confirm one successful advisor request against the active transaction."""
        if self.args.demo:
            return True
        status = self._force_bias_snapshot() if force_bias is None else force_bias
        if status is None:
            return False
        identity = tuple(status.get(field) for field in ("processor_id", "generation", "request_id"))
        confirmed = (status.get("available") and status.get("ready") and status.get("state") == "ready"
                     and not status.get("pending") and not status.get("busy") and not status.get("last_error"))
        approval = getattr(self, "_trial_force_bias_approval", None)
        if approval is not None and (approval[1] != identity or status.get("last_error")):
            self._trial_force_bias_approval = None
        request = getattr(self, "_trial_force_bias_request", None)
        if request is not None and not status.get("pending") and not status.get("busy"):
            if (status.get("processor_id") != request["processor_id"]
                    or status.get("state") == "failed" or status.get("last_error")):
                self._trial_force_bias_request = None
            elif (confirmed and type(status.get("generation")) is int
                  and type(status.get("request_id")) is int
                  and type(request["generation"]) is int and type(request["request_id"]) is int
                  and status["generation"] > request["generation"]
                  and status["request_id"] > request["request_id"]):
                self._trial_force_bias_approval = (request["scope"], identity)
                self._trial_force_bias_request = None
                if (getattr(self, "_participant_force_bias_error", False)
                        and request["scope"] == self._trial_force_bias_scope(self._force_bias_target_index())):
                    self.participant_start_error = False
                    self._participant_force_bias_error = False
        scope = self._trial_force_bias_scope(self.index if index is None else index)
        return bool(confirmed and getattr(self, "_trial_force_bias_approval", None) == (scope, identity))

    def _check_trial_force_bias(self, snapshot, *, index=None):
        if (self._requires_trial_force_bias(index)
                and not self._trial_force_bias_ready(index, force_bias=self._force_bias_snapshot(snapshot))):
            self._participant_force_bias_error = True
            if self._force_bias_busy(snapshot):
                raise ValueError("Force bias is in progress. Keep all 3 sensors unloaded and wait before starting.")
            raise ValueError("Complete a new Force bias in the advisor before each measured trial.")

    def request_force_bias(self):
        """Queue one shared force bias request; completion arrives in later snapshots."""
        if self.args.demo or self.closing or self.state in ("countdown", "recording", "saving"):
            return False
        snapshot = self.bridge.snapshot()
        status = self._force_bias_snapshot(snapshot)
        if status is None or not status.get("available") or self._force_bias_busy(snapshot):
            self._update_force_bias(snapshot)
            return False
        accepted = self.bridge.request_force_bias()
        if accepted:
            self._trial_force_bias_approval = None
            self._trial_force_bias_request = {
                "scope": self._trial_force_bias_scope(self._force_bias_target_index()),
                **{field: status.get(field) for field in ("processor_id", "generation", "request_id")},
            }
        self._update_force_bias(self.bridge.snapshot())
        if accepted:
            self.notice.set("Force bias requested. Keep all 3 sensors unloaded and stationary until it completes.")
        return accepted

    def _update_force_bias(self, snapshot):
        status = self._force_bias_snapshot(snapshot)
        busy = self._force_bias_busy(snapshot)
        bias_ready = self._trial_force_bias_ready(force_bias=status)
        self._observe_force_bias_quality(snapshot)
        if self.args.demo:
            text = "Demo · simulated force data · force bias disabled"
        elif status is None:
            text = "Force bias unavailable · start the force processor."
        else:
            state = status.get("state")
            if state == "waiting":
                text = "Force bias · waiting for all 3 sensor streams…"
            elif state == "warming":
                text = "Force bias · warming up…"
            elif state == "collecting":
                counts = status.get("sample_counts", {})
                samples = " / ".join(str(counts.get(str(sensor), counts.get(sensor, 0))) for sensor in (1, 2, 3))
                text = f"Force bias · collecting samples · sensors 1 / 2 / 3: {samples}"
            elif state == "uncertain" and busy:
                text = ("Force bias confirmation lost · check the force processor; "
                        "restart the processor and GUI if it does not recover.")
            elif state == "stale" and busy:
                text = "Force bias status unavailable · wait for the force processor to reconnect."
            elif busy:
                text = "Force bias requested · waiting for the force processor…"
            elif state == "failed" or status.get("last_error"):
                detail = status.get("last_error")
                text = ("Force bias failed" + (": " + str(detail).rstrip(".") if detail else "") +
                        ". Check sensor streams and retry.")
            elif not status.get("available"):
                text = "Force bias unavailable · start the force processor."
            elif state == "ready" and status.get("ready"):
                text = "Force bias complete · all 3 sensors ready."
                target = self._force_bias_target_index()
                if (self.state in ("ready", "finished", "resting")
                        and self._requires_trial_force_bias(target)
                        and not self._trial_force_bias_ready(target, force_bias=status)):
                    text = "New Force bias required before this measured trial · unload sensors and click Force bias."
            else:
                text = "Force bias needed · keep sensors unloaded and click Force bias."
        if getattr(self, "force_bias_label", None) is not None:
            self.force_bias_label.set(text)
        controls = (bool(status and status.get("available")), busy, bias_ready, self.state, self.closing)
        if controls != getattr(self, "_force_bias_controls", None):
            self._force_bias_controls = controls
            self.set_controls()

    def _observe_force_bias_quality(self, snapshot):
        if (self.state != "recording" or getattr(self, "_force_bias_recording_changed", False)
                or getattr(getattr(self, "args", None), "demo", False)):
            return
        status = self._force_bias_snapshot(snapshot)
        if status is None:
            return
        initial_generation = getattr(self, "_force_bias_recording_generation", None)
        initial_processor = getattr(self, "_force_bias_recording_processor_id", None)
        changed = (status is not None and initial_generation is not None and
                   status.get("generation") is not None and status["generation"] != initial_generation)
        restarted = (status is not None and initial_processor is not None and
                     status.get("processor_id") is not None and status["processor_id"] != initial_processor)
        if self._force_bias_busy(snapshot) or changed or restarted:
            self._force_bias_recording_changed = True
            self.quality_issues.append("Force bias changed during recording")
            self.notice.set("Force bias changed during recording. This trial will be flagged for review.")

    def preflight(self, snapshot):
        validate_user(self.user.get())
        if self._force_bias_busy(snapshot):
            raise ValueError("Force bias is in progress. Keep all 3 sensors unloaded and wait before starting.")
        if not self.args.demo:
            # Every live recording must carry the actual active offsets, even
            # when the optional finger-frame force requirement is unchecked.
            force_bias_record(self._force_bias_snapshot(snapshot))
        now = time.monotonic()
        pose = snapshot["pose"]
        if pose is None or now - pose.arrival > STALE_SEC:
            raise ValueError("Sensor 4 is unavailable. Start the Polhemus TF stream before this trial.")
        if self.mesh.error:
            raise ValueError(self.mesh.error)
        if self.require_ft.get():
            missing = [f.title() for f, v in snapshot["fingers"].items() if v["value"] is None or now - v["arrival"] > STALE_SEC]
            if missing:
                raise ValueError("Waiting for finger-frame force: " + ", ".join(missing) + ". Check the updated publishers.")
        return pose

    def _prepare_trial_start(self):
        calibration = self.current_calibration() if self.trial.axis and not self.trial.practice else None
        target = (calibration["target_deg"] if calibration else
                  float(self.target.get()) if self.trial.practice and self.trial.axis else 30.)
        counter = RotationCounter(target, float(self.neutral.get()))
        self.preflight(self.bridge.snapshot())
        return counter

    def demo_recording_changed(self):
        if not self.args.demo:
            return
        enabled = self.record_demo_data.get()
        self.notice.set("New demo trials will save data." if enabled else
                        "New demo trials will run without creating data files.")
        if self.state == "ready":
            self.folder_label.set("Participant and trial folders are created when recording starts." if enabled else
                                  "Demo preview · data recording disabled · no files will be written.")

    def start_trial(self):
        if self.closing or self.state in ("countdown", "recording", "saving", "resting"):
            return False
        self._participant_force_bias_error = False
        try:
            self._check_trial_force_bias(self.bridge.snapshot())
            counter = self._prepare_trial_start()
        except (ValueError, OverflowError) as exc:
            self.notice.set(str(exc))
            self.participant_start_error = True
            return False
        self._clear_rest()
        self.counter = counter
        self.calibrating = self.needs_calibration()
        self.range_calibration = (RotationRangeCalibration(
            target_cap_deg=self.practice_targets_deg[self.trial.diameter_mm])
            if self.calibrating else None)
        self.participant_start_error = False
        self.manual_count = 0
        self.state = "countdown"
        self.deadline = time.monotonic() + self.preparation_sec
        self._preparation_stage = None
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
        pickup_deadline = self.deadline - self.hold_neutral_sec
        pickup = now < pickup_deadline
        stage = "pickup" if pickup else "neutral"
        total = self.pickup_sec if pickup else self.hold_neutral_sec
        deadline = pickup_deadline if pickup else self.deadline
        if stage != getattr(self, "_preparation_stage", None):
            self._preparation_stage = stage
            title = "PICK UP THE OBJECT" if pickup else "HOLD AT NEUTRAL"
            self.status_label.set(("ANGLE TEST · " if self.calibrating else "") + title)
            self.notice.set(f"Pick up the object. Hold at neutral starts after this {self.pickup_sec}-second countdown." if pickup else
                            "Hold the object at neutral. The orientation baseline is captured when the countdown ends.")
        self.rest_view.set_preparation(
            remaining=max(1, min(total, math.ceil(deadline - now))),
            total=total, instruction=self.participant_instruction(now, tracking=True),
        )

    def begin_recording(self):
        self.rest_view.place_forget()
        self._participant_force_bias_error = False
        try:
            if self.needs_calibration() and not self.calibrating:
                raise ValueError("Complete the rotation angle test before recording a measured trial.")
            # Recheck after the countdown; capture R0 from the most recent fresh TF.
            snapshot = self.bridge.snapshot()
            self._check_trial_force_bias(snapshot)
            pose = self.preflight(snapshot)
            # Capture the current bias once without requesting another tare.
            # Saved trials archive it; angle tests keep it only in memory.
            force_bias = self._force_bias_snapshot(snapshot)
            bias_record = ({"schema_version": 1, "captured_utc": utc_now(),
                            "demo": True, "biases": None,
                            "description": "Simulated force data; no hardware bias is applied."}
                           if self.args.demo else force_bias_record(force_bias))
            when = datetime.now(self.zone)
            user = validate_user(self.user.get())
            config_metadata = getattr(self, "gui_config_metadata", {"gui_config_file": str(self.args.rest_config)})
            recording_enabled = (not self.calibrating
                                 and (not self.args.demo or self.record_demo_data.get()))
            if recording_enabled and (self.session_path is None or self.session_user != user):
                session_path = create_session(self.args.data_root, user, when)
                metadata_dir = unique_directory(session_path / "SessionMetadata", f"Session_{when:%Y%m%d_%H%M%S_%f}")
                session_metadata_path = metadata_dir / "session_metadata.json"
                session = {"user_number": user, "started_local": when.isoformat(), "timezone": self.args.timezone,
                           "demo": self.args.demo, "protocol_file": str(self.args.protocol),
                           "rest_config_file": str(self.args.rest_config), "rest_seconds": asdict(self.rest_times),
                           **config_metadata,
                           "practice_targets_deg": self.practice_targets_deg,
                           "object_to_sensor4_matrix": self.alignment.tolist(),
                           "storage_layout_version": 2, "participant_directory": session_path.name,
                           "trials": [asdict(t) for t in self.trials]}
                session_metadata_path.write_text(json.dumps(session, indent=2), encoding="utf-8")
                self.session_path, self.session_user = session_path, user
                self.session_metadata_path = session_metadata_path
            self.trial_path = (create_trial_directory(self.session_path, user, self.trial, when)
                               if recording_enabled else None)
            self.baseline = pose.quaternion
            self.axis_tracker = RotationAxisTracker(self.trial.axis) if self.trial.axis else None
            if self.axis_tracker is not None:
                self.axis_tracker.update(np.eye(3), pose.source_timestamp)
                if not self.calibrating:
                    # The captured baseline is neutral. A first moving sample
                    # can already lie beyond the band without losing that pass.
                    self.counter.update(0.0, pose.source_timestamp)
            self.started = time.monotonic()
            self._reset_force_release()
            self.started_utc = utc_now()
            self.tracking_interruptions = 0
            self.ft_interruptions = {finger: 0 for finger in FINGERS}
            self.ft_stale = {finger: False for finger in FINGERS}
            self.quality_issues = []
            self._force_bias_recording_generation = force_bias.get("generation") if force_bias else None
            self._force_bias_recording_processor_id = force_bias.get("processor_id") if force_bias else None
            self._force_bias_recording_changed = False
            self.start_counters = self.bridge.counters()
            self.last_pose_drops = self.start_counters["pose_drops"]
            metadata = {"user_number": user, "trial": asdict(self.trial), "started_local": when.isoformat(),
                        **config_metadata,
                        "started_utc": self.started_utc, "timezone": self.args.timezone, "demo": self.args.demo,
                        "data_recorded": recording_enabled, "demo_cycle_count_skipped": False,
                        "recording_kind": "rotation_calibration" if self.calibrating else "trial",
                        "rotation_calibration": (deepcopy(self.current_calibration()) if not self.calibrating and
                                                 self.trial.axis and not self.trial.practice else None),
                        "target_cycles": None if self.calibrating else 20,
                        "target_deg": None if self.calibrating else self.counter.target_deg,
                        "neutral_deg": self.counter.neutral_deg,
                        "cycle_detection": "pass_through",
                        "cycle_sequence": ["neutral", "negative_target", "neutral", "positive_target", "neutral"],
                        "neutral_crossing": "linear_between_adjacent_valid_samples",
                        "count_mode": ("rotation_range" if self.calibrating else
                                       "automatic_rotation" if self.trial.axis else "manual_gaiting"),
                        "mesh": str(mesh_asset_path(self.args.mesh_dir, self.trial.diameter_mm)),
                        "mesh_origin_xyz": [0.0, 0.0, 0.0], "mesh_coordinate_unit": "mm",
                        "mesh_to_sensor4_matrix": self.alignment.tolist(),
                        "object_to_sensor4_matrix": self.alignment.tolist(),
                        "rotation_frame": "moving_object", "mesh_rotation_frame": "baseline_object",
                        "baseline_xyzw": list(self.baseline),
                        "baseline_source_timestamp_sec": pose.source_timestamp,
                        "orientation_tf": {"parent": "polhemus_base", "child": "sensor4"},
                        "relative_orientation_formula": "A.T @ R0.T @ Rt @ A",
                        "axis_error_frame": "previous_object",
                        "axis_error_formula": "degrees(acos(abs(dot(increment_axis, task_axis))))",
                        "axis_increment_formula": "A.T @ R_previous.T @ R_current @ A",
                        "axis_error_minimum_rotation_deg": (self.axis_tracker.minimum_rotation_deg
                                                             if self.axis_tracker else None),
                        "axis_error_max_gap_sec": self.axis_tracker.max_gap_sec if self.axis_tracker else None,
                        "incremental_angle_definition": "sum of signed task-axis twist from consecutive valid poses",
                        "cycle_angle_definition": "sum of signed task-axis increments since the latest tracking reference",
                        "cycle_angle_reset_policy": "re-zero on first valid pose after a tracking interruption",
                        "rotation_release_detection": {
                            "threshold_n": getattr(self, "release_force_threshold_n", DEFAULT_RELEASE_FORCE_THRESHOLD_N),
                            "force_metric": "sum of three bias-corrected finger-force magnitudes",
                            "max_force_age_sec": STALE_SEC,
                            "neutral_policy": "preserve original trial neutral; cancel unfinished cycle only",
                            "applies_to": "practice and measured rotation trials",
                        },
                        "force_unit": "N", "raw_torque_unit": "N*mm", "force_axes": "finger frame",
                        "transformed_components": ["Fx", "Fy", "Fz"],
                        "topics": FORCE_TOPICS, "sensor_mapping": SENSORS, "require_all_force": self.require_ft.get(),
                        "raw_topics": RAW_TOPICS, "raw_axes": "ATI sensor frame", "raw_bias_applied": False,
                        "force_bias": bias_record,
                        "storage_layout_version": 2, "task_trial_number": self.trial.task_position,
                        "participant_directory": self.session_path.name if recording_enabled else None,
                        "session_metadata_file": (self.session_metadata_path.relative_to(self.session_path).as_posix()
                                                  if recording_enabled else None),
                        "session_metadata_base": "participant_directory",
                        "acquisition_counters_start": self.start_counters}
            self.writer = (TrialWriter(self.trial_path, metadata) if recording_enabled else PreviewWriter(metadata))
            if not self.writer.ready.wait(timeout=2):
                raise OSError("Timed out opening trial files.")
            if self.writer.error:
                raise OSError(self.writer.error)
            # Saved trials archive this baseline; angle tests retain no samples.
            self.writer.add("pose", None, (pose.source_timestamp, *pose.quaternion))
            self.bridge.begin(self.writer, self.trial.axis, self.counter.target_deg,
                              alignment=self.alignment, baseline=self.baseline,
                              expected_bias=None if self.args.demo else bias_record)
            if not self.args.demo and not self.calibrating:
                self._trial_force_bias_request = None
                self._trial_force_bias_approval = None
            self.state = "recording"
            self.previous_tracking = True
            self.rotation = np.eye(3)
            self.angle = 0.0
            self.info_vars["start"].set(when.strftime("%H:%M:%S  %Z"))
            self.folder_label.set("Rotation calibration · measurement only · no files are written." if
                                  self.calibrating else str(self.trial_path) if recording_enabled else
                                  "Demo preview · data recording disabled · no files are being written.")
            self.status_label.set("ANGLE TEST" if self.calibrating else
                                  "RECORDING" if recording_enabled else "DEMO PREVIEW · NOT RECORDING")
            self.notice.set("Measure the maximum rotation in both directions, then click Finish angle test. "
                            "The range will be included in subsequent trial metadata." if
                            self.calibrating else
                            "Recording all incoming raw F/T and finger-force samples. Orientation baseline reset for this trial." if
                            recording_enabled else "Demo preview started. No CSV or metadata files are written.")
            if self.calibrating:
                self.part_trial.configure(text=f"Angle test · {TASK_NAMES[self.trial.task]} · {self.trial.diameter_mm} mm")
            self.set_controls()
        except (OSError, ValueError) as exc:
            if self.writer and not self.writer.finished.is_set():
                self.writer.finish("start_failed", {"start_error": str(exc)})
                self.state = "saving"
            else:
                self.state = "ready"
                self.writer = None
                self.calibrating = False
                self.range_calibration = None
            self.notice.set("Could not start: " + str(exc))
            self.participant_start_error = True
            self.status_label.set("START FAILED")
            self.set_controls()

    def _reset_force_release(self):
        """Clear per-trial grip state without changing any rotation references."""
        self._force_released = False
        self._force_reacquired_at = None
        self._force_release_events = []
        self._force_release_event_drops = 0

    def _apply_force_release_event(self, event):
        """Apply one grip transition in the same receipt order as orientations."""
        released = event["released"]
        if released == getattr(self, "_force_released", False):
            return
        phase = self.counter.phase
        self.counter.reset_partial()
        self._force_released = released
        if released:
            if not hasattr(self, "_force_release_events"):
                self._force_release_events = []
            self._force_release_events.append({
                "elapsed_sec": max(0.0, event["arrival"] - self.started),
                "source_timestamp_sec": event["source_timestamp_sec"],
                "force_sum_magnitude_n": event["force_sum_magnitude_n"],
                "previous_cycle_phase": phase,
                "completed_cycles": self.count,
            })
            self.notice.set("Object released. The unfinished cycle was cancelled; the original neutral is retained.")
        else:
            self._force_reacquired_at = event["arrival"]
            self.notice.set("Object picked up. Continue using the original neutral and complete a full cycle.")

    def _rotation_release_events(self, snapshot, now, force_events=None):
        """Collect callback transitions without copying the waveform histories."""
        active = self.state == "recording" and self.trial.axis and not self.calibrating
        if snapshot.get("force_release_events_supported") is True:
            batch = (force_events if force_events is not None
                     else self.bridge.drain_force_release_events())
            if not active:
                return []
            drops = batch["drops"]
            if drops > getattr(self, "_force_release_event_drops", 0):
                # Unknown grip intervals cannot contribute to a cycle. Tracking
                # continues, and a retained held event can resume the counter.
                self.counter.reset_partial()
                self._force_released = True
            self._force_release_event_drops = drops
            return sorted(batch["events"], key=lambda event: event["arrival"])
        if not active:
            return []
        force_sum = total_force_magnitude(snapshot.get("fingers", {}), now, max_age_sec=STALE_SEC)
        if force_sum is None:
            return []
        threshold = getattr(self, "release_force_threshold_n", DEFAULT_RELEASE_FORCE_THRESHOLD_N)
        return [{
            "arrival": max(snapshot["fingers"][finger]["arrival"] for finger in FINGERS),
            "released": force_sum <= threshold,
            "force_sum_magnitude_n": force_sum,
            "source_timestamp_sec": max(snapshot["fingers"][finger]["value"][-1] for finger in FINGERS),
        }]

    def _break_rotation_tracking(self):
        self.counter.reset_partial()
        if self.axis_tracker is not None:
            self.axis_tracker.break_tracking()
        if self.calibrating:
            self.range_calibration.invalidate()
        self.angle = None

    def process_poses(self, poses, now, allow_finish=True, *, force_events=None):
        if self.baseline is None:
            return
        snapshot = self.bridge.snapshot()
        events = self._rotation_release_events(snapshot, max(now, time.monotonic()), force_events)
        event_index = 0
        pose_drops = snapshot["pose_drops"]
        if self.state == "recording" and pose_drops != self.last_pose_drops:
            self._break_rotation_tracking()
            self.last_pose_drops = pose_drops
        for pose in poses:
            if pose.arrival < (self.started or 0):
                continue
            while event_index < len(events) and events[event_index]["arrival"] <= pose.arrival:
                self._apply_force_release_event(events[event_index])
                event_index += 1
            try:
                # The mesh uses this baseline-relative pose. The tracker takes
                # consecutive poses to measure rotation about moving object axes.
                rotation = relative_rotation(self.baseline, pose.quaternion, self.alignment)
            except ValueError:
                if self.state == "recording":
                    self._break_rotation_tracking()
                continue
            self.rotation = rotation
            if self.state == "recording":
                if self.trial.axis:
                    measurement = self.axis_tracker.update(rotation, pose.source_timestamp)
                    if measurement is None:
                        continue  # A duplicate source timestamp cannot advance the cycle.
                    measured_axis = measurement.axis or (None, None, None)
                    self.writer.add("axis_error", None, (
                        pose.source_timestamp, pose.arrival - self.started,
                        measurement.step_rotation_deg, measurement.step_angle_deg,
                        measurement.accumulated_angle_deg, *measured_axis,
                        measurement.error_deg, measurement.continuous, measurement.status,
                    ))
                    if self.calibrating:
                        self.range_calibration.update(measurement)
                        self.angle = measurement.accumulated_angle_deg if measurement.continuous else None
                        self.writer.add("progress", None, (pose.source_timestamp, pose.arrival - self.started,
                                                           self.angle, 0, "rotation_calibration"))
                        continue
                    if measurement.status in ("waiting", "tracking_gap"):
                        # Missing motion cannot be integrated: establish a new
                        # local zero and require the whole unfinished cycle again.
                        self.angle = 0.0
                        self.counter.reset_partial()
                    elif measurement.step_angle_deg is None:
                        self.angle = None
                        self.counter.reset_partial()
                    else:
                        self.angle = (0.0 if self.angle is None else self.angle) + measurement.step_angle_deg
                    reacquired_at = getattr(self, "_force_reacquired_at", None)
                    released = (getattr(self, "_force_released", False)
                                or (reacquired_at is not None and pose.arrival < reacquired_at))
                    if self.angle is not None and not released:
                        self.counter.update(self.angle, pose.source_timestamp)
                else:
                    self.angle = 0.0
                    released = False
                self.writer.add("progress", None, (pose.source_timestamp, pose.arrival - self.started, self.angle, self.count,
                                                   "object_released" if released else self.counter.phase if
                                                   self.trial.axis else "manual_gaiting"))
                if self.count >= 20 and allow_finish:
                    for event in events[event_index:]:
                        self._apply_force_release_event(event)
                    self.finish_trial("completed")
                    return
        for event in events[event_index:]:
            self._apply_force_release_event(event)

    def axis_motion_text(self, tracking=True, *, participant=False):
        """Keep participant feedback concise while retaining advisor diagnostics."""
        if not self.trial.axis:
            return "" if participant else "Axis error: N/A · Finger gaiting"
        if self.baseline is None or self.axis_tracker is None:
            return "Axis error: — · Start trial to measure"
        measurement = self.axis_tracker.sample
        if not tracking:
            error = "— · Tracking unavailable"
        elif measurement.error_deg is not None:
            error = f"{measurement.error_deg:.1f}°"
        elif measurement.status == "low_motion":
            error = "— · Motion too small"
        else:
            error = "— · Waiting for motion"
        text = f"Axis error: {error}"
        if participant:
            return text
        partial = " (partial)" if not measurement.continuous else ""
        text += f"   ·   Accumulated angle: {measurement.accumulated_angle_deg:+.1f}°{partial}"
        if self.calibrating:
            limits = self.range_calibration
            text += f"   ·   Max +{limits.positive_max_deg:.1f}° / −{limits.negative_max_deg:.1f}°"
        return text

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

    def stop_current(self):
        self.finish_trial("calibration_completed" if self.calibrating else "stopped")

    def finish_trial(self, result, update_ui=True):
        if self.state == "countdown":
            self.state = "ready"
            self.calibrating = False
            self.range_calibration = None
            if update_ui:
                self.rest_view.place_forget()
                self.status_label.set("READY")
                self.notice.set("Countdown cancelled. No trial was recorded.")
                self.set_controls()
            return
        if self.state != "recording":
            return
        _, poses = self.bridge.end_with_poses()
        stopped_at = time.monotonic()
        pose_at_stop = self.bridge.snapshot()["pose"]
        # Consume already acquired poses before committing the final count.
        self.process_poses(poses, time.monotonic(), allow_finish=False)
        snapshot = self.bridge.snapshot()
        self._observe_force_bias_quality(snapshot)
        self.observe_ft_quality(snapshot, time.monotonic())
        if self.axis_tracker is not None and (pose_at_stop is None or stopped_at - pose_at_stop.arrival > STALE_SEC):
            self.axis_tracker.break_tracking()
            if self.calibrating:
                self.range_calibration.invalidate()
        end_counters = self.bridge.counters()
        self.quality_issues = (["Force bias changed during recording"]
                               if getattr(self, "_force_bias_recording_changed", False) else [])
        for finger in FINGERS:
            key = "force_" + finger
            if (self.ft_interruptions[finger] or self.ft_stale[finger]
                    or end_counters["gaps"][key] > self.start_counters["gaps"][key]):
                self.quality_issues.append(f"{finger} force interrupted")
            if end_counters["invalid"][key] > self.start_counters["invalid"][key]:
                self.quality_issues.append(f"{finger} invalid force samples")
            raw_key = "raw_" + finger
            if end_counters["gaps"][raw_key] > self.start_counters["gaps"][raw_key]:
                self.quality_issues.append(f"{finger} raw F/T interrupted")
            if end_counters["invalid"][raw_key] > self.start_counters["invalid"][raw_key]:
                self.quality_issues.append(f"{finger} invalid raw F/T samples")
        if self.tracking_interruptions:
            self.quality_issues.append("orientation tracking interrupted")
        if end_counters["pose_drops"] > self.start_counters["pose_drops"]:
            self.quality_issues.append("orientation queue overflow")
        if getattr(self, "_force_release_event_drops", 0):
            self.quality_issues.append("force-release event queue overflow")
        calibration_extra = {}
        if self.calibrating:
            calibration_extra["rotation_range_observation"] = {
                "positive_max_deg": self.range_calibration.positive_max_deg,
                "negative_max_deg": self.range_calibration.negative_max_deg,
                "continuous": self.range_calibration.continuous,
            }
            if result == "calibration_completed":
                try:
                    if self.writer.overflows:
                        raise ValueError("Samples were lost during the angle test. Repeat it.")
                    calibration = self.range_calibration.result(self.counter.neutral_deg)
                    calibration.update(
                        measured_range_deg=[-calibration["negative_max_deg"], calibration["positive_max_deg"]],
                        target_range_deg=[-calibration["target_deg"], calibration["target_deg"]],
                        user_number=self.user.get().strip(), diameter_mm=self.trial.diameter_mm,
                        task=self.trial.task, axis=self.trial.axis,
                        rotation_frame="moving_object", started_utc=self.started_utc,
                        completed_utc=utc_now(), neutral_deg=self.counter.neutral_deg,
                        recording_path=str(self.trial_path) if self.trial_path else None,
                        data_recorded=getattr(self.writer, "recording_enabled", True), demo=self.args.demo,
                    )
                    calibration_extra["rotation_calibration"] = calibration
                except ValueError as exc:
                    result = "calibration_failed"
                    calibration_extra["calibration_error"] = str(exc)
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
                                   "object_release_count": len(getattr(self, "_force_release_events", [])),
                                   "object_release_events": deepcopy(getattr(self, "_force_release_events", [])),
                                   "force_release_event_drops": getattr(self, "_force_release_event_drops", 0),
                                   "demo_cycle_count_skipped": demo_skipped,
                                   "accumulated_angle_deg": (self.axis_tracker.sample.accumulated_angle_deg
                                                             if self.axis_tracker else None),
                                   "incremental_angle_continuous": (self.axis_tracker.sample.continuous
                                                                    if self.axis_tracker else None),
                                   "notes": notes, "tracking_interruptions": self.tracking_interruptions,
                                   "ft_interruptions": dict(self.ft_interruptions), "ft_stale_at_finish": dict(self.ft_stale),
                                   "quality_issues": self.quality_issues,
                                   "acquisition_counters_end": end_counters, **calibration_extra})
        self.state = "saving"
        if update_ui:
            self.status_label.set("FINISHING ANGLE TEST" if self.calibrating else
                                  "SAVING" if getattr(self.writer, "recording_enabled", True) else "FINISHING PREVIEW")
            self.notice.set("Finishing angle measurement…" if self.calibrating else
                            "Finishing CSV writes…" if getattr(self.writer, "recording_enabled", True) else
                            "Finishing demo preview…")
            self.set_controls()

    def poll_writer(self):
        if self.state == "recording" and self.writer.error:
            self.finish_trial("write_error")
        if self.state != "saving" or not self.writer or not self.writer.finished.is_set():
            return
        writer = self.writer
        self.state = "finished"
        if self.calibrating:
            accepted = (writer.result == "calibration_completed" and not writer.error and not writer.overflows)
            if accepted:
                self.calibrations[self.calibration_key()] = deepcopy(writer.metadata["rotation_calibration"])
            self._select_trial(self.index)
            if accepted:
                calibration = self.current_calibration()
                self.status_label.set("ANGLE TEST COMPLETE")
                next_step = ("Complete Force bias in the advisor before starting the measured trial."
                             if self._requires_trial_force_bias() and not self._trial_force_bias_ready() else
                             "Press SPACE or Start trial when ready.")
                self.notice.set(f"Maximum +{calibration['positive_max_deg']:.2f}° / "
                                f"−{calibration['negative_max_deg']:.2f}°. "
                                f"This task's measured trials use ±{calibration['target_deg']:.2f}°. "
                                + next_step)
            else:
                self.status_label.set("ANGLE TEST · REPEAT REQUIRED")
                error = writer.error or writer.metadata.get("calibration_error") or "Angle test cancelled or incomplete."
                self.notice.set(f"{error} Hold at neutral and start the angle test again.")
            return
        incomplete = [f for f in FINGERS if writer.counts[f] == 0]
        incomplete_raw = [f for f in FINGERS if writer.raw_counts[f] == 0]
        if writer.error:
            self.status_label.set("SAVE ERROR")
            self.notice.set("Recording error: " + writer.error)
        elif not getattr(writer, "recording_enabled", True):
            self.status_label.set("DEMO PREVIEW · SKIPPED" if writer.result == "demo_skipped" else
                                  "DEMO PREVIEW · COMPLETE" if writer.result == "completed" else
                                  "DEMO PREVIEW · STOPPED")
            self.notice.set("Demo preview finished. No data files were written.")
        elif writer.overflows or incomplete or incomplete_raw or self.quality_issues:
            self.status_label.set("SAVED · REVIEW DATA")
            issues = list(self.quality_issues)
            if writer.overflows:
                issues.append(f"{writer.overflows} writer queue drops")
            if incomplete:
                issues.append("empty force: " + ", ".join(incomplete))
            if incomplete_raw:
                issues.append("empty raw F/T: " + ", ".join(incomplete_raw))
            self.notice.set(f"Saved {self.count}/20 cycles. Review: " + "; ".join(issues) + ".")
        else:
            self.status_label.set("COMPLETE · SAVED" if writer.result == "completed" else
                                  "DEMO SKIPPED · SAVED" if writer.result == "demo_skipped" else "STOPPED · SAVED")
            self.notice.set(f"Saved {self.count}/20 cycles and {sum(writer.counts.values()):,} finger-force samples.")
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
        if self._force_bias_busy():
            instruction = "Force bias is in progress. Keep all force sensors unloaded and wait."
        elif (status.complete and self._requires_trial_force_bias(self.rest_next_index)
              and not self._trial_force_bias_ready(self.rest_next_index)):
            instruction = "Please wait for the researcher to complete Force bias before the next trial."
        elif self.participant_start_error:
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
        if self.calibrating:
            self.count_label.set("ANGLE TEST")
            self.part_count.configure(text="ANGLE TEST")
            self.progress["value"] = 0
            self.draw_progress()
            return
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
        self._update_force_bias(snapshot)
        pose = snapshot["pose"]
        tracking = pose is not None and now - pose.arrival <= STALE_SEC
        self.pose_label.set("Sensor 4 · tracking" if tracking else "Sensor 4 · OFFLINE")
        if snapshot.get("force_release_events_supported") is True:
            updates = self.bridge.drain_recording_updates()
            self.process_poses(updates["poses"], now, force_events=updates)
        else:
            self.process_poses(self.bridge.drain_poses(), now)
        if self.state == "recording":
            self.observe_ft_quality(snapshot, now)
            if not tracking and self.previous_tracking:
                self._break_rotation_tracking()
                self.tracking_interruptions += 1
        self.previous_tracking = tracking
        if self.state == "countdown" and now >= self.deadline:
            self.begin_recording()
        self._update_preparation(now)
        self.poll_writer()
        self._update_rest(now)
        for finger, item in snapshot["fingers"].items():
            fresh = item["value"] is not None and now - item["arrival"] <= STALE_SEC
            values = item["value"][:3] if fresh else None
            self.charts[finger].draw(values)
            self.chart_status[finger].set(f"Live  ·  {item['rate']:.0f} Hz" if fresh else "Offline · waiting for force")
        self._refresh_waveforms(now, force_bias=snapshot.get("force_bias"))
        while not self.events.empty():
            level, message = self.events.get_nowait()
            if self.state == "ready" or level == "error":
                self.notice.set(message)
        self.part_instruction.configure(text=self.participant_instruction(now, tracking))
        angle_text = "—" if self.angle is None else f"{self.angle:+.1f}°"
        if self.calibrating:
            limits = self.range_calibration
            angle_label = (f"Angle {angle_text} · Max +{limits.positive_max_deg:.1f}° / "
                           f"−{limits.negative_max_deg:.1f}° · Target ±{limits.target_deg:.1f}°")
        elif self.needs_calibration():
            angle_label = "Target angle will be set by the angle test."
        elif self.trial.axis:
            calibration = self.current_calibration() if not self.trial.practice else None
            target = calibration["target_deg"] if calibration else self.counter.target_deg
            target_text = (self.target.get() if self.trial.practice and self.state in ("ready", "finished")
                           else f"{target:g}")
            angle_label = (f"Cycle angle  {angle_text}   ·   "
                           f"Target ±{target_text}°   ·   Neutral ±{self.counter.neutral_deg:g}°")
        else:
            angle_label = ""
        self.part_angle.configure(text=angle_label)
        self.axis_motion_label.set(self.axis_motion_text(tracking))
        self.part_axis_motion.configure(text=self.axis_motion_text(tracking, participant=True))
        if not self.rest_screen_visible and self.state != "countdown":
            self.mesh.set_pose(self.rotation, self.trial.axis, tracking=tracking if self.baseline is not None else True)
        self.update_progress()
        self.root.after(50, self.tick)

    def open_visualization_options(self):
        """Open the waveform signal chooser while the advisor remains active."""
        view = getattr(self, "waveform_view", None)
        if not self.closing and view is not None:
            view.open_options()

    def _refresh_waveforms(self, now, *, force_bias=None):
        """Refresh visible history at 5 Hz without changing acquisition cadence."""
        view = getattr(self, "waveform_view", None)
        if self.closing or view is None or not view.winfo_ismapped():
            return
        last = getattr(self, "_waveform_last_refresh", None)
        if last is not None and now - last < .2:
            return
        history = self.bridge.force_history_snapshot(now=now)
        view.set_raw_bias_status(force_bias)
        view.refresh(history, now)
        self._waveform_last_refresh = now

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
        report("stopping; finishing the active measurement…")
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
                elif self.writer.metadata.get("recording_kind") == "rotation_calibration":
                    report("rotation calibration ended; no data files were written.")
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
    parser.add_argument("--demo", action="store_true", help="Simulate sensors; default output is User_Study_Data/demo/.")
    parser.add_argument("--data-root", type=Path, default=None)
    parser.add_argument("--protocol", type=Path, default=PROJECT_ROOT / "docs/user_study_docs/Trials_lookup.csv")
    parser.add_argument("--gui-config", "--rest-config", dest="rest_config", metavar="PATH", type=Path, default=GUI_CONFIG_FILE,
                        help="GUI configuration: rest durations, practice targets, sensor4 rotation, TF tolerance, and chart limits.")
    parser.add_argument("--mesh-dir", type=Path, default=PROJECT_ROOT / "utils/object_meshes",
                        help="Prepared 50mm/80mm GLB models; legacy STL is also supported.")
    parser.add_argument("--timezone", default="America/New_York", help="Timezone for session and trial folder names.")
    args = parser.parse_args(argv)
    args.data_root = (args.data_root or (STUDY_DATA_DIR / "demo" if args.demo else STUDY_DATA_DIR)).expanduser().resolve()
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
