"""Window-local font sizing for the participant and advisor displays.

No global ``tk scaling`` or shared ttk styles are changed: maximizing one
window leaves the other window's font sizes alone. Canvas drawings can follow
the same scale through ``on_scale``.
"""
from __future__ import annotations

import math
import tkinter as tk
from tkinter import font as tkfont, ttk
from typing import Callable


def scale_for_size(width: float, height: float, base_size: tuple[float, float],
                   minimum: float = .85, maximum: float = 1.75) -> float:
    """Use additional window area while reserving room for packed controls.

    The geometric mean also enlarges text when maximizing a window that was
    already almost as tall as the screen. Height and width limits prevent an
    unusually wide or tall window from producing disproportionate text.
    """
    values = (width, height, *base_size, minimum, maximum)
    if len(base_size) != 2 or not all(math.isfinite(v) and v > 0 for v in values):
        raise ValueError("Window dimensions and font scale limits must be positive and finite.")
    if minimum > maximum:
        raise ValueError("Minimum font scale cannot exceed maximum font scale.")
    horizontal, vertical = width / base_size[0], height / base_size[1]
    desired = min(math.sqrt(horizontal * vertical), horizontal * 1.05, vertical * 1.20)
    return max(minimum, min(maximum, desired))


def scaled_font_size(original: int, scale: float) -> int:
    """Preserve Tk's negative pixel-size convention and avoid zero sizes."""
    if not math.isfinite(scale) or scale <= 0 or original == 0:
        raise ValueError("Font size must be nonzero and scale must be positive and finite.")
    size = max(1, round(abs(original) * scale))
    return -size if original < 0 else size


class ResponsiveFonts:
    """Scale widgets below one Tk/Toplevel; construct after building widgets.

    Keep this object alive for as long as the window. Font changes always use
    captured original sizes, so repeated maximize/restore operations do not
    accumulate rounding errors. ``refresh()`` includes widgets added later.
    """

    def __init__(self, window: tk.Misc, base_size: tuple[int, int],
                 on_scale: Callable[[float], None] | None = None):
        # Validate before installing callbacks, including the expected base size.
        scale_for_size(*base_size, base_size)
        self.window = window
        self.base_size = base_size
        self.on_scale = on_scale
        self.scale = 1.0
        self._style = ttk.Style(window)
        self._prefix = f"Responsive{id(self)}"
        self._fonts: list[tuple[tkfont.Font, int]] = []
        self._seen: set[str] = set()
        self._style_fonts: dict[str, tuple[str, tkfont.Font]] = {}
        self._comboboxes: list[tuple[tk.Misc, tkfont.Font]] = []
        self._pending = None
        self._last_size = None
        self._closed = False
        self._configure_binding = window.bind("<Configure>", self._on_configure, add="+")
        self._destroy_binding = window.bind("<Destroy>", self._on_destroy, add="+")
        self.refresh()

    def _copy_font(self, description) -> tkfont.Font:
        font = tkfont.Font(root=self.window, font=description or "TkDefaultFont")
        # Read the configured size rather than asking the remote display to
        # resolve font metrics; this also retains negative pixel-size units.
        original = int(font.cget("size"))
        self._fonts.append((font, original))
        return font

    def _walk(self, widget):
        for child in widget.winfo_children():
            # A second study window is a child of Tk, but needs its own scale.
            if isinstance(child, tk.Toplevel):
                continue
            yield child
            yield from self._walk(child)

    def refresh(self):
        """Capture newly created widgets and apply the current window size."""
        if self._closed:
            return
        for widget in self._walk(self.window):
            name = str(widget)
            if name in self._seen:
                continue
            self._seen.add(name)
            if "font" in widget.keys():
                font = self._copy_font(widget.cget("font"))
                widget.configure(font=font)
                if isinstance(widget, ttk.Combobox):
                    self._comboboxes.append((widget, font))
            elif isinstance(widget, ttk.Widget):
                # ttk buttons/labels use styles rather than a -font option.
                original = widget.cget("style") or widget.winfo_class()
                if original not in self._style_fonts:
                    font = self._copy_font(self._style.lookup(original, "font") or "TkDefaultFont")
                    local_style = f"{self._prefix}.{original}"
                    self._style.configure(local_style, font=font)
                    self._style_fonts[original] = (local_style, font)
                widget.configure(style=self._style_fonts[original][0])
        self._last_size = None
        self._schedule()

    def _on_configure(self, event):
        if event.widget is self.window:
            self._schedule()

    def _schedule(self):
        if self._closed:
            return
        # Keep the first deadline. Some window managers emit Configure events
        # continuously while live labels redraw; cancelling/restarting here
        # could postpone font updates forever. _apply reads the latest size.
        if self._pending is None:
            self._pending = self.window.after(80, self._apply)

    def _apply(self):
        self._pending = None
        if self._closed:
            return
        size = (self.window.winfo_width(), self.window.winfo_height())
        if min(size) <= 1 or size == self._last_size:
            return
        self._last_size = size
        self.scale = scale_for_size(*size, self.base_size)
        for font, original in self._fonts:
            target = scaled_font_size(original, self.scale)
            if int(font.cget("size")) != target:
                font.configure(size=target)
        for widget, font in self._comboboxes:
            try:
                # Tk's popdown is a separate internal toplevel, outside the
                # widget traversal. Give only this combobox its local font.
                popup = widget.tk.call("ttk::combobox::PopdownWindow", str(widget))
                widget.tk.call(f"{popup}.f.l", "configure", "-font", str(font))
            except tk.TclError:
                # Non-listbox native ttk implementations need no special work.
                pass
        if self.on_scale is not None:
            self.on_scale(self.scale)

    def _on_destroy(self, event):
        if event.widget is self.window:
            self.close()

    def close(self):
        """Cancel a deferred resize and release callbacks when the window closes."""
        if self._closed:
            return
        self._closed = True
        try:
            if self._pending is not None:
                self.window.after_cancel(self._pending)
                self._pending = None
            self.window.unbind("<Configure>", self._configure_binding)
            self.window.unbind("<Destroy>", self._destroy_binding)
        except tk.TclError:
            pass
