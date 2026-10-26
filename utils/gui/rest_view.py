"""Participant preparation and rest countdowns, independent of acquisition."""
from __future__ import annotations

import math
import tkinter as tk


INK = "#17273D"
MUTED = "#65758A"
TEAL = "#147D79"
TRACK = "#E5EEED"


class RestCountdownView(tk.Frame):
    """A centered circular countdown with one integer remaining-seconds display.

    The owner places this frame over the participant's trial content. Updates
    step with whole seconds rather than the acquisition/redraw frequency.
    """

    def __init__(self, master, bg="#FFFFFF"):
        super().__init__(master, background=bg)
        self.canvas = tk.Canvas(self, background=bg, highlightthickness=0,
                                width=300, height=420)
        self.canvas.pack(fill="both", expand=True)
        self.font_scale = 1.0
        self._state = None
        self._rendered = None
        self.canvas.bind("<Configure>", self._on_configure)

    def set_font_scale(self, scale):
        scale = float(scale)
        if not math.isfinite(scale) or scale <= 0:
            raise ValueError("Rest font scale must be positive and finite.")
        if scale != self.font_scale:
            self.font_scale = scale
            self._draw()

    def set_rest(self, kind: str, elapsed: int, total: int, remaining: int,
                 progress: float, complete: bool, next_label: str = "",
                 instruction: str | None = None, skipped: bool = False):
        total = max(0, int(total))
        elapsed = min(total, max(0, int(elapsed)))
        remaining = min(total, max(0, int(remaining)))
        progress = float(progress)
        if not math.isfinite(progress):
            raise ValueError("Rest progress must be finite.")
        # Quantize the arc too: passing sub-second timer updates must not
        # rebuild the whole Canvas dozens of times between displayed seconds.
        steps = max(1, total)
        step = int(min(1.0, max(0.0, progress)) * steps + 1e-9)
        progress = 1.0 if complete and not skipped else min(1.0, step / steps)
        state = (kind, elapsed, total, remaining, progress, bool(complete), next_label, instruction, bool(skipped))
        if state != self._state:
            self._state = state
            self._draw()

    def set_preparation(self, remaining: int, total: int = 5,
                        instruction: str | None = None):
        """Use the same ring for the preparation before recording begins."""
        total = max(0, int(total))
        remaining = min(total, max(0, int(remaining)))
        elapsed = total - remaining
        self.set_rest(kind="preparation", elapsed=elapsed, total=total,
                      remaining=remaining, progress=elapsed / total if total else 1.0,
                      complete=remaining == 0, instruction=instruction)

    def _on_configure(self, event):
        self._draw(event.width, event.height)

    def _draw(self, width=None, height=None):
        if self._state is None:
            return
        width = self.canvas.winfo_width() if width is None else width
        height = self.canvas.winfo_height() if height is None else height
        if width < 40 or height < 40:
            return
        rendered = (width, height, self.font_scale, self._state)
        if rendered == self._rendered:
            return
        self._rendered = rendered
        kind, elapsed, total, remaining, progress, complete, next_label, instruction, skipped = self._state
        c = self.canvas
        c.delete("all")
        scale = max(.45, min(self.font_scale, width / 600, height / 550))
        padding = max(12, min(width * .04, height * .03))
        text_width = max(40, width - 2 * padding)

        def font(size, bold=False):
            return ("DejaVu Sans", max(7, round(size * scale)),
                    "bold" if bold else "normal")

        if kind == "preparation":
            title = "Ready" if complete else "Get ready"
        else:
            title = "Rest skipped" if skipped else "Rest complete" if complete else "Rest between " + {
                "trials": "trials", "tasks": "tasks", "objects": "objects",
            }.get(kind, "trials")
        c.create_text(width / 2, height * .085, text=title, fill=TEAL if complete else INK,
                      font=font(24, True), width=text_width, justify="center")
        if next_label:
            c.create_text(width / 2, height * .16, text=next_label, fill=MUTED,
                          font=font(13), width=text_width, justify="center")

        cx, cy = width / 2, height * .5
        radius = min(width * .28, height * .28, 200 * scale)
        line_width = max(6, 12 * scale)
        bounds = (cx - radius, cy - radius, cx + radius, cy + radius)
        c.create_oval(*bounds, outline=TRACK, width=line_width)
        if (complete and not skipped) or progress >= 1:
            c.create_oval(*bounds, outline=TEAL, width=line_width)
        elif progress > 0:
            c.create_arc(*bounds, start=90, extent=-360 * progress,
                         style="arc", outline=TEAL, width=line_width)
        c.create_text(cx, cy, text=str(remaining), fill=INK,
                      font=font(72, True))
        if kind != "preparation" and not complete and not skipped:
            c.create_text(cx, height * .84, text="Put the cylindrical object on the platform.",
                          fill=INK, font=font(15, True), width=text_width, justify="center")
        if instruction is None:
            instruction = ("Hold the object at neutral." if kind == "preparation" else
                           "Press SPACE when you are ready for the next trial." if complete else
                           "Please rest. You can continue when the countdown ends.")
        c.create_text(cx, height * .9, text=instruction, fill=TEAL if complete else MUTED,
                      font=font(13), width=text_width, justify="center")
