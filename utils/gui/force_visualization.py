"""Selectable, bounded Canvas waveforms for the advisor's live force history."""
from __future__ import annotations

from dataclasses import dataclass
import math
import tkinter as tk
from tkinter import ttk

import numpy as np

from utils.gui.raw_waveform_bias import raw_bias_offsets, subtract_raw_bias


WINDOW_SEC = 30.0
GAP_SEC = 0.5
FINGERS = ("thumb", "index", "middle")
COMPONENTS = ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz")
KIND_LABELS = {"raw": "Raw", "force": "Finger transformed"}
KIND_COMPONENTS = {"raw": COMPONENTS, "force": COMPONENTS[:3]}
INK = "#17273D"
MUTED = "#65758A"

# Raw and transformed versions of a component share a color and differ in dash.
TRACE_COLORS = (
    "#B91C1C", "#9A3412", "#A16207", "#15803D", "#0F766E", "#0369A1",
    "#1D4ED8", "#4338CA", "#7E22CE", "#A21CAF", "#BE185D", "#7C2D12",
    "#166534", "#155E75", "#3730A3", "#6B21A8", "#9D174D", "#475569",
)


@dataclass(frozen=True)
class ForceSignal:
    """One component of a raw wrench or a transformed three-axis force."""

    kind: str
    finger: str
    component: int

    def __post_init__(self):
        if self.kind not in KIND_LABELS or self.finger not in FINGERS:
            raise ValueError("Unknown force stream.")
        if (not isinstance(self.component, int) or isinstance(self.component, bool)
                or self.component not in range(len(KIND_COMPONENTS[self.kind]))):
            raise ValueError(f"{KIND_LABELS[self.kind]} component must be an integer from 0 "
                             f"to {len(KIND_COMPONENTS[self.kind]) - 1}.")

    @property
    def label(self):
        return f"{self.finger.title()} · {KIND_LABELS[self.kind]} · {COMPONENTS[self.component]}"

    @property
    def color(self):
        return TRACE_COLORS[FINGERS.index(self.finger) * 6 + self.component]

    @property
    def dash(self):
        return (4, 3) if self.kind == "raw" else ()


ALL_SIGNALS = tuple(ForceSignal(kind, finger, component)
                    for finger in FINGERS for kind in KIND_LABELS
                    for component in range(len(KIND_COMPONENTS[kind])))
DEFAULT_SIGNALS = ()
MAX_SELECTED_SIGNALS = 3


def _validate_trace_options(component, now, window_sec, gap_sec):
    if not isinstance(component, int) or isinstance(component, bool) or component not in range(6):
        raise ValueError("Force component must be an integer from 0 to 5.")
    if not math.isfinite(now) or not math.isfinite(window_sec) or window_sec <= 0:
        raise ValueError("Force history time must be finite and its window positive and finite.")
    if not math.isfinite(gap_sec) or gap_sec <= 0:
        raise ValueError("Force history gap must be positive and finite.")


def _decimate_trace_fallback(samples, component, now, pixel_width, *, window_sec, gap_sec):
    """Handle malformed points defensively, retaining gaps around bad values."""
    columns = max(1, min(600, int(pixel_width)))
    cutoff = now - window_sec
    segments = []
    points = []
    bucket = None
    minimum = maximum = last = None

    def append(point):
        if not points or point[0] != points[-1][0]:
            points.append(point)

    def flush_bucket():
        if minimum is not None:
            for point in sorted((minimum, maximum), key=lambda item: item[0]):
                append(point)

    def flush_segment():
        nonlocal points, bucket, minimum, maximum, last
        if last is not None:
            flush_bucket()
            append(last)
            segments.append(tuple((point[1] - now, point[2]) for point in points))
        points = []
        bucket = minimum = maximum = last = None

    for sequence, sample in enumerate(samples):
        arrival = float(sample.arrival)
        value = float(sample.values[component])
        if not math.isfinite(arrival) or not math.isfinite(value):
            flush_segment()
            continue
        if arrival < cutoff or arrival > now:
            continue
        if last is not None and (arrival - last[1] > gap_sec or arrival < last[1]):
            flush_segment()
        point = (sequence, arrival, value)
        column = min(columns - 1, int((arrival - cutoff) / window_sec * columns))
        if last is None:
            append(point)
        if column != bucket:
            flush_bucket()
            minimum = maximum = point
            bucket = column
        else:
            if value < minimum[2]:
                minimum = point
            if value > maximum[2]:
                maximum = point
        last = point
    flush_segment()
    return tuple(segments)


def decimate_stream(samples, components, now, pixel_width, *, window_sec=WINDOW_SEC, gap_sec=GAP_SEC):
    """Reduce selected components together using their shared receipt timeline.

    Per-column extrema and their original arrival times survive reduction. The
    acquisition worker retains all samples; NumPy only reduces this GUI copy.
    """
    components = tuple(components)
    for component in components:
        _validate_trace_options(component, now, window_sec, gap_sec)
    result = {component: [] for component in components}
    if not samples or not components:
        return {component: () for component in components}
    arrivals = np.fromiter((sample.arrival for sample in samples), dtype=np.float64, count=len(samples))
    vectors = np.asarray([sample.values for sample in samples], dtype=np.float64)
    if vectors.ndim != 2 or vectors.shape[1] not in (3, 6):
        raise ValueError("Force history must contain three force or six force/torque components.")
    if any(component >= vectors.shape[1] for component in components):
        raise ValueError("Requested component is unavailable in this force history.")
    values = vectors[:, components]
    if not np.isfinite(arrivals).all() or not np.isfinite(values).all():
        return {component: _decimate_trace_fallback(samples, component, now, pixel_width,
                                                    window_sec=window_sec, gap_sec=gap_sec)
                for component in components}
    cutoff = now - window_sec
    visible = (arrivals >= cutoff) & (arrivals <= now)
    arrivals, values = arrivals[visible], values[visible]
    if not len(arrivals):
        return {component: () for component in components}
    columns = max(1, min(600, int(pixel_width)))
    gaps = np.diff(arrivals)
    boundaries = np.r_[0, np.flatnonzero((gaps > gap_sec) | (gaps < 0)) + 1, len(arrivals)]
    for begin, end in zip(boundaries[:-1], boundaries[1:]):
        times, current = arrivals[begin:end], values[begin:end]
        bins = np.minimum(columns - 1, ((times - cutoff) / window_sec * columns).astype(np.int64))
        starts = np.r_[0, np.flatnonzero(np.diff(bins)) + 1]
        counts = np.diff(np.r_[starts, len(times)])
        minimum = np.minimum.reduceat(current, starts, axis=0)
        maximum = np.maximum.reduceat(current, starts, axis=0)
        indices = np.arange(len(times))[:, None]
        min_indices = np.minimum.reduceat(
            np.where(current == np.repeat(minimum, counts, axis=0), indices, len(times)), starts, axis=0)
        max_indices = np.minimum.reduceat(
            np.where(current == np.repeat(maximum, counts, axis=0), indices, len(times)), starts, axis=0)
        for column, component in enumerate(components):
            keep = np.unique(np.r_[0, min_indices[:, column], max_indices[:, column], len(times) - 1])
            result[component].append(tuple(zip((times[keep] - now).tolist(), current[keep, column].tolist())))
    return {component: tuple(segments) for component, segments in result.items()}


def decimate_trace(samples, component, now, pixel_width, *, window_sec=WINDOW_SEC, gap_sec=GAP_SEC):
    """Keep chronological per-column extrema, endpoints, and acquisition gaps.

    The returned segments use seconds relative to ``now``. Each display column
    contributes at most two extrema; the first and last point of each segment
    are also retained. Source timestamps are intentionally not the display clock.
    """
    return decimate_stream(samples, (component,), now, pixel_width,
                           window_sec=window_sec, gap_sec=gap_sec)[component]


def trace_limits(traces, *, fallback=1.0):
    """Auto-range selected traces independently per unit, retaining zero."""
    low = high = 0.0
    for segments in traces:
        for segment in segments:
            for _relative_time, value in segment:
                low, high = min(low, value), max(high, value)
    if low == high:
        return -fallback, fallback
    padding = (high - low) * .08
    return low - padding, high + padding


def _tick_label(value):
    return "0" if abs(value) < 1e-12 else f"{value:.3g}"


class ForceWaveformView(tk.Frame):
    """Advisor waveforms refreshed by their owner, without an extra timer."""

    def __init__(self, master, *, force_limit_n=10.0, torque_limit_nmm=100.0):
        limits = []
        for name, value in (("Force", force_limit_n), ("Torque", torque_limit_nmm)):
            try:
                limit = float(value)
            except (TypeError, ValueError, OverflowError) as exc:
                raise ValueError(f"{name} display limit must be positive and finite.") from exc
            if isinstance(value, bool) or not math.isfinite(limit) or limit <= 0:
                raise ValueError(f"{name} display limit must be positive and finite.")
            limits.append(limit)
        super().__init__(master, background="white")
        self.force_limit_n, self.torque_limit_nmm = limits
        self.font_scale = 1.0
        self._selected = set(DEFAULT_SIGNALS)
        self._last_history = {}
        self._last_now = 0.0
        self._options_dialog = None
        self._options_vars = {}
        self._options_checks = {}
        self._selection_status_var = None
        self._raw_bias_enabled = False
        self._raw_bias_offsets = None
        self._raw_bias_var = None
        self._raw_bias_status_var = None
        header = tk.Frame(self, background="white")
        header.pack(fill="x", padx=8, pady=(4, 0))
        self.title_label = tk.Label(header, text="FORCE / TORQUE · LAST 30 SECONDS",
                                    foreground=INK, background="white", anchor="w")
        self.title_label.pack(side="left")
        self.option_button = tk.Button(header, text="Visualization option", command=self.open_options,
                                       foreground=INK, background="#EDF2F7", activebackground="#E2E8F0",
                                       relief="flat", padx=10, pady=3)
        self.option_button.pack(side="right")
        self.canvas = tk.Canvas(self, background="white", highlightthickness=0,
                                width=900, height=220)
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda _: self._draw())
        legend_frame = tk.Frame(self, background="white")
        legend_frame.pack(fill="x", padx=8)
        self.legend = tk.Canvas(legend_frame, background="white", highlightthickness=0, height=23)
        self.legend.pack(fill="x")
        self.legend_scrollbar = ttk.Scrollbar(legend_frame, orient="horizontal", command=self.legend.xview)
        self.legend_scrollbar.pack(fill="x")
        self.legend.configure(xscrollcommand=self.legend_scrollbar.set)
        self.legend.bind("<Configure>", lambda _: self._draw_legend())
        self.set_font_scale(1.0)

    @property
    def selected_signals(self):
        return tuple(signal for signal in ALL_SIGNALS if signal in self._selected)

    @property
    def raw_bias_enabled(self):
        return self._raw_bias_enabled

    def set_raw_bias_status(self, status):
        """Use confirmed central offsets at the next display refresh."""
        offsets = raw_bias_offsets(status)
        changed = offsets != self._raw_bias_offsets
        self._raw_bias_offsets = offsets
        self._update_raw_bias_status()
        if changed and self._raw_bias_enabled:
            self._draw_legend()

    def set_raw_bias_enabled(self, enabled):
        """Toggle raw display correction without changing history or recording."""
        if type(enabled) is not bool:
            raise ValueError("Raw display bias must be enabled or disabled.")
        self._raw_bias_enabled = enabled
        if self._raw_bias_var is not None:
            self._raw_bias_var.set(enabled)
        self._update_raw_bias_status()
        self._draw_legend()
        self._draw()

    def _update_raw_bias_status(self):
        if self._raw_bias_status_var is None:
            return
        if self._raw_bias_offsets is None:
            text = "Unload sensors, click Force bias in the advisor, and wait for completion."
        elif self._raw_bias_enabled:
            text = "Active Force bias is applied to Raw waveforms only."
        else:
            text = "Force bias is ready. Enable to subtract it from Raw waveforms."
        self._raw_bias_status_var.set(text)

    def _raw_bias_option_changed(self):
        self.set_raw_bias_enabled(bool(self._raw_bias_var.get()))

    def set_selected_signals(self, signals):
        """Change live display selection without touching history or recording."""
        selected = set(signals)
        if not selected <= set(ALL_SIGNALS):
            raise ValueError("Unknown force visualization signal.")
        if len(selected) > MAX_SELECTED_SIGNALS:
            raise ValueError(f"Select at most {MAX_SELECTED_SIGNALS} waveform signals.")
        self._selected = selected
        self._sync_signal_options()
        self._draw_legend()
        self._draw()

    def _sync_signal_options(self):
        """Keep selected signals editable when the global display limit is full."""
        full = len(self._selected) >= MAX_SELECTED_SIGNALS
        for signal, variable in self._options_vars.items():
            variable.set(signal in self._selected)
        for signal, checkbox in self._options_checks.items():
            checkbox.configure(state="normal" if signal in self._selected or not full else "disabled")
        if self._selection_status_var is not None:
            self._selection_status_var.set(f"Selected signals: {len(self._selected)}/{MAX_SELECTED_SIGNALS}")

    def refresh(self, history, now):
        """Display an immutable history snapshot at a monotonic receipt time."""
        if not math.isfinite(now):
            raise ValueError("Force visualization time must be finite.")
        self._last_history = history
        self._last_now = float(now)
        self._draw()

    def set_font_scale(self, scale):
        scale = float(scale)
        if not math.isfinite(scale) or scale <= 0:
            raise ValueError("Force visualization font scale must be positive and finite.")
        self.font_scale = scale
        self.title_label.configure(font=self._font(9, True))
        self.option_button.configure(font=self._font(9))
        self.legend.configure(height=max(20, round(23 * scale)))
        self._draw_legend()
        self._draw()

    def _font(self, size, bold=False):
        return ("DejaVu Sans", max(7, round(size * self.font_scale)), "bold" if bold else "normal")

    def _draw_legend(self):
        self.legend.delete("all")
        x, y = 5, max(10, round(11 * self.font_scale))
        if not self._selected:
            self.legend.create_text(x, y, text="No signals selected. Open Visualization option to choose signals.",
                                    anchor="w", fill=MUTED, font=self._font(8))
        for signal in self.selected_signals:
            self.legend.create_line(x, y, x + 22, y, fill=signal.color, width=2, dash=signal.dash)
            label = signal.label
            if signal.kind == "raw" and self._raw_bias_enabled:
                label += " (bias corrected)" if self._raw_bias_offsets is not None else " (waiting for bias)"
            item = self.legend.create_text(x + 28, y, text=label, anchor="w",
                                           fill=INK, font=self._font(8))
            bounds = self.legend.bbox(item)
            x = (bounds[2] if bounds else x + 220) + 22
        bounds = self.legend.bbox("all")
        self.legend.configure(scrollregion=(0, 0, max(self.legend.winfo_width(),
                                                     (bounds[2] + 8) if bounds else 0),
                                            max(20, round(23 * self.font_scale))))

    def _draw(self):
        c = self.canvas
        c.delete("all")
        width, height = c.winfo_width(), c.winfo_height()
        if width < 180 or height < 110:
            return
        if width >= 760:
            panels = ((0, 0, width / 2 - 4, height), (width / 2 + 4, 0, width, height))
        else:
            panels = ((0, 0, width, height / 2 - 3), (0, height / 2 + 3, width, height))
        plot_width = max(1, int(panels[0][2] - panels[0][0] - 74 * self.font_scale))
        selected = self.selected_signals
        traces = {}
        for finger in FINGERS:
            for kind in KIND_LABELS:
                signals = tuple(signal for signal in selected if signal.kind == kind and signal.finger == finger)
                if signals:
                    if kind == "raw" and self._raw_bias_enabled and self._raw_bias_offsets is None:
                        traces.update((signal, ()) for signal in signals)
                        continue
                    components = decimate_stream(self._last_history.get((kind, finger), ()),
                                                 (signal.component for signal in signals), self._last_now, plot_width)
                    for signal in signals:
                        segments = components[signal.component]
                        if kind == "raw" and self._raw_bias_enabled:
                            segments = subtract_raw_bias(segments, self._raw_bias_offsets[finger][signal.component])
                        traces[signal] = segments
        for group, panel in enumerate(panels):
            self._draw_panel(panel, group, {signal: segments for signal, segments in traces.items()
                                           if signal.component // 3 == group})

    def _draw_panel(self, panel, group, traces):
        c, scale = self.canvas, self.font_scale
        x0, y0, x1, y1 = panel
        left, right = x0 + 53 * scale, x1 - 13 * scale
        top, bottom = y0 + 25 * scale, y1 - 34 * scale
        title = "Force (N)" if group == 0 else "Torque (N·mm)"
        c.create_text(x0 + 10 * scale, y0 + 11 * scale, text=title, anchor="w", fill=INK,
                      font=self._font(9, True), tags="waveform-title")
        if right - left < 20 or bottom - top < 15:
            return
        limit = self.force_limit_n if group == 0 else self.torque_limit_nmm
        low, high = -limit, limit
        c.create_rectangle(left, top, right, bottom, fill="#F8FAFC", outline="#DAE2EB",
                           tags="waveform-plot")
        for step in range(5):
            fraction = step / 4
            value = low + fraction * (high - low)
            y = bottom - fraction * (bottom - top)
            c.create_line(left, y, right, y, fill="#E2E8F0", tags="waveform-grid")
            c.create_text(left - 6 * scale, y, text=_tick_label(value), anchor="e", fill=MUTED,
                          font=self._font(8), tags="waveform-y-tick")
        zero_y = bottom - (0 - low) / (high - low) * (bottom - top)
        c.create_line(left, zero_y, right, zero_y, fill="#94A3B8", tags="waveform-zero")
        for elapsed in (0, 10, 20, 30):
            x = left + elapsed / WINDOW_SEC * (right - left)
            c.create_line(x, top, x, bottom, fill="#E2E8F0", tags="waveform-grid")
            c.create_text(x, bottom + 11 * scale, text=str(elapsed), fill=MUTED,
                          font=self._font(8), tags="waveform-time-tick")
        c.create_text((left + right) / 2, y1 - 6 * scale, text="Time in window (s)", fill=MUTED,
                      font=self._font(8))
        any_samples = False

        def visible_segments(segment):
            """Clip straight segments at the fixed limits, retaining reentry gaps."""
            if len(segment) == 1:
                if low <= segment[0][1] <= high:
                    yield segment
                return
            if all(low <= value <= high for _relative, value in segment):
                yield segment
                return
            visible = []
            for first, second in zip(segment, segment[1:]):
                difference = second[1] - first[1]
                if difference == 0:
                    enter, leave = (0.0, 1.0) if low <= first[1] <= high else (1.0, 0.0)
                else:
                    crossings = sorted(((low - first[1]) / difference,
                                        (high - first[1]) / difference))
                    enter, leave = max(0.0, crossings[0]), min(1.0, crossings[1])
                if enter > leave:
                    if visible:
                        yield visible
                    visible = []
                    continue

                def point_at(fraction):
                    if fraction == 0:
                        return first
                    if fraction == 1:
                        return second
                    return (first[0] + fraction * (second[0] - first[0]),
                            max(low, min(high, first[1] + fraction * difference)))

                start, end = point_at(enter), point_at(leave)
                if visible and visible[-1] != start:
                    yield visible
                    visible = []
                if not visible:
                    visible.append(start)
                if visible[-1] != end:
                    visible.append(end)
                if leave < 1:
                    yield visible
                    visible = []
            if visible:
                yield visible

        for signal, segments in traces.items():
            for segment in segments:
                any_samples = any_samples or bool(segment)
                for visible in visible_segments(segment):
                    coordinates = []
                    for relative, value in visible:
                        coordinates.extend((left + (relative + WINDOW_SEC) / WINDOW_SEC * (right - left),
                                            bottom - (value - low) / (high - low) * (bottom - top)))
                    if len(coordinates) == 2:
                        x, y = coordinates
                        c.create_oval(max(left, x - 1.5), max(top, y - 1.5),
                                      min(right, x + 1.5), min(bottom, y + 1.5), fill=signal.color,
                                      outline="", tags=("waveform-trace", signal.kind, signal.finger,
                                                        COMPONENTS[signal.component]))
                    else:
                        c.create_line(*coordinates, fill=signal.color, width=1.5, dash=signal.dash,
                                      tags=("waveform-trace", signal.kind, signal.finger,
                                            COMPONENTS[signal.component]))
        if not any_samples:
            text = "Waiting for selected samples" if traces else "No signals selected"
            if (getattr(self, "_raw_bias_enabled", False)
                    and getattr(self, "_raw_bias_offsets", None) is None
                    and any(signal.kind == "raw" for signal in traces)):
                text = "Waiting for Force bias"
            c.create_text((left + right) / 2, (top + bottom) / 2, text=text, fill=MUTED,
                          font=self._font(9))

    def open_options(self):
        """Open or raise the reusable, nonmodal signal selection dialog."""
        if self._options_dialog is not None and self._options_dialog.winfo_exists():
            self._options_dialog.deiconify()
            self._options_dialog.lift()
            self._options_dialog.focus_set()
            return
        dialog = tk.Toplevel(self)
        self._options_dialog = dialog
        dialog.title("Visualization option")
        dialog.transient(self.winfo_toplevel())
        dialog.protocol("WM_DELETE_WINDOW", self._close_options)
        body = ttk.Frame(dialog, padding=12)
        body.pack(fill="both", expand=True)
        ttk.Label(body, text="Choose up to 3 signals. Raw is calibrated acquisition data before bias correction.").grid(
            row=0, column=0, columnspan=3, sticky="w", pady=(0, 8))
        for column, finger in enumerate(FINGERS):
            section = ttk.LabelFrame(body, text=finger.title(), padding=8)
            section.grid(row=1, column=column, sticky="nsew", padx=(0 if column == 0 else 8, 0))
            body.columnconfigure(column, weight=1)
            for row, kind in enumerate(KIND_LABELS):
                group = ttk.LabelFrame(section, text=KIND_LABELS[kind], padding=6)
                group.grid(row=row, column=0, sticky="ew", pady=(0 if row == 0 else 8, 0))
                for component, label in enumerate(KIND_COMPONENTS[kind]):
                    signal = ForceSignal(kind, finger, component)
                    variable = tk.BooleanVar(master=dialog, value=signal in self._selected)
                    self._options_vars[signal] = variable
                    checkbox = ttk.Checkbutton(group, text=label, variable=variable,
                                               command=self._options_changed)
                    checkbox.grid(row=component // 3, column=component % 3, sticky="w", padx=4)
                    self._options_checks[signal] = checkbox
        raw_bias = ttk.LabelFrame(body, text="Raw display bias", padding=8)
        raw_bias.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(12, 0))
        self._raw_bias_var = tk.BooleanVar(master=dialog, value=self._raw_bias_enabled)
        ttk.Checkbutton(raw_bias, text="Apply force bias to Raw", variable=self._raw_bias_var,
                        command=self._raw_bias_option_changed).pack(anchor="w")
        self._raw_bias_status_var = tk.StringVar(master=dialog)
        ttk.Label(raw_bias, textvariable=self._raw_bias_status_var).pack(anchor="w", pady=(4, 0))
        ttk.Label(raw_bias, text="Display only. Original Raw CSV data stays uncorrected.").pack(anchor="w")
        self._update_raw_bias_status()
        buttons = ttk.Frame(body)
        buttons.grid(row=3, column=0, columnspan=3, sticky="ew", pady=(12, 0))
        self._selection_status_var = tk.StringVar(master=dialog)
        ttk.Label(buttons, textvariable=self._selection_status_var).pack(side="left", padx=(0, 12))
        ttk.Button(buttons, text="Clear all", command=lambda: self.set_selected_signals(())).pack(side="left")
        ttk.Button(buttons, text="Close", command=self._close_options).pack(side="right")
        ttk.Label(body, text="Raw traces are dashed; finger-transformed traces are solid. Scroll the legend to see all labels.").grid(
            row=4, column=0, columnspan=3, sticky="w", pady=(8, 0))
        self._sync_signal_options()

    def _options_changed(self):
        selected = {signal for signal, variable in self._options_vars.items() if variable.get()}
        if len(selected) > MAX_SELECTED_SIGNALS:
            self._sync_signal_options()
            return
        self.set_selected_signals(selected)

    def _close_options(self):
        dialog, self._options_dialog = self._options_dialog, None
        self._options_vars = {}
        self._options_checks = {}
        self._selection_status_var = None
        self._raw_bias_var = None
        self._raw_bias_status_var = None
        if dialog is not None:
            try:
                dialog.destroy()
            except tk.TclError:
                pass

    def destroy(self):
        self._close_options()
        super().destroy()
