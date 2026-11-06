"""Keep the advisor's controls and stacked plots reachable on small displays."""
from __future__ import annotations

import tkinter as tk
from tkinter import ttk


class AdvisorBody(tk.Frame):
    """A width-filling body that scrolls only when its content needs more height.

    Existing advisor widgets belong in ``body``. Its requested height is kept
    intact, so plot panes retain their usable minimum sizes on short screens.
    Wheel and layout bindings belong to this advisor toplevel; participant
    windows and unrelated bindings are left independent.
    """

    def __init__(self, master, **kwargs):
        super().__init__(master, **kwargs)
        self._closed = False
        self._pending = None
        self._scrollbar_visible = False
        self._owner = self.winfo_toplevel()
        self._bindings = []
        background = self.cget("background")
        self.rowconfigure(0, weight=1)
        self.columnconfigure(0, weight=1)
        self.canvas = tk.Canvas(self, background=background, highlightthickness=0,
                                borderwidth=0, yscrollincrement=20)
        self.canvas.grid(row=0, column=0, sticky="nsew")
        self.scrollbar = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self.scrollbar.grid(row=0, column=1, sticky="ns")
        self.scrollbar.grid_remove()
        self.canvas.configure(yscrollcommand=self.scrollbar.set)
        self.body = tk.Frame(self.canvas, background=background)
        self._window = self.canvas.create_window(0, 0, window=self.body, anchor="nw")
        # Child geometry changes can alter the requested height while the
        # embedded body keeps the same actual size, for example after scaling
        # fonts. The toplevel sees those Configure events through Tk bindtags.
        for sequence, callback in (("<Configure>", self._on_configure),
                                    ("<MouseWheel>", self._on_mousewheel),
                                    ("<Button-4>", self._on_mousewheel),
                                    ("<Button-5>", self._on_mousewheel)):
            self._bindings.append((sequence, self._owner.bind(sequence, callback, add="+")))
        self.bind("<Destroy>", self._on_destroy, add="+")
        self.refresh()

    def _contains(self, widget):
        """Check ancestry without treating another toplevel as advisor content."""
        while widget is not None:
            if widget is self:
                return True
            if isinstance(widget, tk.Toplevel):
                return False
            widget = getattr(widget, "master", None)
        return False

    def _on_configure(self, event):
        if event.widget is self._owner or self._contains(event.widget):
            self.refresh()

    def refresh(self):
        """Coalesce size changes until Tk has updated the content requests."""
        if not self._closed and self._pending is None:
            self._pending = self.after_idle(self._update_geometry)

    def _update_geometry(self):
        self._pending = None
        if self._closed:
            return
        width = max(1, self.canvas.winfo_width())
        viewport_height = max(1, self.canvas.winfo_height())
        requested_height = max(1, self.body.winfo_reqheight())
        content_height = max(viewport_height, requested_height)
        self.canvas.itemconfigure(self._window, width=width, height=content_height)
        self.canvas.configure(scrollregion=(0, 0, width, content_height))
        overflowing = requested_height > viewport_height
        if overflowing != self._scrollbar_visible:
            self._scrollbar_visible = overflowing
            if overflowing:
                self.scrollbar.grid()
            else:
                self.scrollbar.grid_remove()
        if not overflowing:
            self.canvas.yview_moveto(0)

    def _on_mousewheel(self, event):
        if self._closed or not self._scrollbar_visible or not self._contains(event.widget):
            return None
        number = getattr(event, "num", None)
        if number in (4, 5):
            units = -3 if number == 4 else 3
        else:
            delta = getattr(event, "delta", 0)
            if not delta:
                return None
            magnitude = max(1, int(abs(delta) / 120) * 3)
            units = -magnitude if delta > 0 else magnitude
        self.canvas.yview_scroll(units, "units")
        return "break"

    def _on_destroy(self, event):
        if event.widget is self:
            self._cleanup()

    def _cleanup(self):
        if self._closed:
            return
        self._closed = True
        if self._pending is not None:
            try:
                self.after_cancel(self._pending)
            except tk.TclError:
                pass
            self._pending = None
        for sequence, binding in self._bindings:
            try:
                self._owner.unbind(sequence, binding)
            except tk.TclError:
                pass
        self._bindings.clear()

    def destroy(self):
        self._cleanup()
        super().destroy()
