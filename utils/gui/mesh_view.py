"""A small Tk renderer for the experiment's actual STL object meshes.

STL coordinates are kept in their original XYZ order.  ``set_pose`` takes the
rotation from those local axes into the caller's reference frame.  Centering,
uniform display scaling, and the fixed viewing camera are separate operations.
Each STL is a standalone, normals-corrected display asset.
"""
from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
import tkinter as tk

import numpy as np


from .mesh_repair import MAX_DISPLAY_FACES, load_stl, repair_normals

BACKGROUND = "#FFFFFF"
AXIS_COLORS = {"x": "#BB3445", "y": "#197D58", "z": "#2464BD"}
TARGET_COLOR = "#A96300"


@dataclass(frozen=True)
class MeshGeometry:
    """Display mesh in original millimetre axes, translated by ``center``."""

    vertices: np.ndarray
    faces: np.ndarray
    normals: np.ndarray
    bounds: np.ndarray
    center: np.ndarray
    radius: float
    source_face_count: int


def prepare_mesh(triangles: np.ndarray, max_faces: int = MAX_DISPLAY_FACES) -> MeshGeometry:
    """Build a bounded, shaded display mesh without exchanging any STL axes."""
    triangles = np.asarray(triangles, dtype=np.float64)
    if (triangles.ndim != 3 or triangles.shape[1:] != (3, 3)
            or not len(triangles) or not np.isfinite(triangles).all()):
        raise ValueError("Expected finite STL triangles with shape (N, 3, 3)")
    if max_faces < 4:
        raise ValueError("max_faces must be at least 4")
    if len(triangles) > max_faces:
        raise ValueError("Dense STL requires preparation: run "
                         "python -m utils.gui.mesh_repair <source.stl> <display.stl>")
    triangles, _ = repair_normals(triangles)
    vertices, inverse = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    faces = inverse.reshape(-1, 3)
    bounds = np.stack((vertices.min(axis=0), vertices.max(axis=0)))
    extent = float(np.max(bounds[1] - bounds[0]))
    if extent <= 0:
        raise ValueError("STL has zero extent")
    center = (bounds[0] + bounds[1]) * 0.5
    vertices = vertices - center
    corners = vertices[faces]
    normals = np.cross(corners[:, 1] - corners[:, 0], corners[:, 2] - corners[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    valid = lengths > extent * extent * 1e-12
    faces = faces[valid]
    normals = normals[valid] / lengths[valid, None]
    if not len(faces):
        raise ValueError("STL contains no non-degenerate triangles")
    radius = float(np.linalg.norm(triangles.reshape(-1, 3) - center, axis=1).max())
    for array in (vertices, faces, normals, bounds, center):
        array.setflags(write=False)
    return MeshGeometry(vertices, faces, normals, bounds, center, radius, len(triangles))


@lru_cache(maxsize=8)
def _cached_geometry(triangle_bytes: bytes) -> MeshGeometry:
    # Cache by the decoded vertices, not filesystem timestamps: same-size
    # rapid rewrites can retain identical mtime/ctime on some filesystems.
    # The key and geometry use the same snapshot, even if the file is replaced
    # concurrently. Byte equality also guards against hash collisions.
    triangles = np.frombuffer(triangle_bytes, dtype=np.float64).reshape(-1, 3, 3)
    return prepare_mesh(triangles)


def load_mesh(path: Path) -> MeshGeometry:
    """Load a standalone STL; NumPy is the only mesh dependency."""
    triangles = load_stl(Path(path))
    return _cached_geometry(triangles.tobytes())


def camera_matrix() -> np.ndarray:
    """Fixed oblique camera with world Z up; this does not alter data axes."""
    toward_eye = np.array([0.75, -1.05, 0.9])
    toward_eye /= np.linalg.norm(toward_eye)
    right = np.cross([0.0, 0.0, 1.0], toward_eye)
    right /= np.linalg.norm(right)
    up = np.cross(toward_eye, right)
    return np.stack((right, up, toward_eye))


class MeshView(tk.Canvas):
    """Real STL orientation display.  Call its methods from the Tk thread.

    ``rotation`` maps STL-local vectors to the caller's reference coordinates.
    ``None`` holds the last supplied rotation; ``tracking=False`` labels that
    hold explicitly.  Target axes accept x/y/z, or roll/pitch/yaw aliases.
    ``error`` is None on success and a readable string on mesh/pose failures.
    """

    def __init__(self, master, mesh_dir: Path, **kwargs):
        kwargs.setdefault("background", BACKGROUND)
        kwargs.setdefault("highlightthickness", 0)
        kwargs.setdefault("width", 560)
        kwargs.setdefault("height", 360)
        super().__init__(master, **kwargs)
        self.mesh_dir = Path(mesh_dir)
        self.error: str | None = None
        self._mesh_error: str | None = None
        self._pose_error: str | None = None
        self.diameter: int | None = None
        self.source_path: Path | None = None
        self._font_scale = 1.0
        self.geometry: MeshGeometry | None = None
        self._rotation = np.eye(3)
        self._camera = camera_matrix()
        self._target_axis = None
        self._tracking = False
        self._triangles = []
        self._fills = []
        self._visible_count = 0
        self._pending_render = None
        self._palette = [self._surface_color(level / 63) for level in range(64)]
        self._cue = self.create_line(0, 0, 1, 1, fill=TARGET_COLOR, width=2,
                                     arrow=tk.LAST, arrowshape=(10, 12, 5), state=tk.HIDDEN)
        self._axis_lines = {axis: self.create_line(
            0, 0, 1, 1, fill=color, width=2, arrow=tk.LAST, arrowshape=(9, 11, 4)
        ) for axis, color in AXIS_COLORS.items()}
        self._axis_labels = {axis: self.create_text(
            0, 0, text=axis.upper(), fill=color, font=("TkDefaultFont", 12, "bold")
        ) for axis, color in AXIS_COLORS.items()}
        self._status = self.create_text(18, 18, anchor="nw", fill="#785015",
                                        font=("TkDefaultFont", 10), text="")
        self._message = self.create_text(0, 0, fill="#A32939", justify=tk.CENTER,
                                         font=("TkDefaultFont", 12), text="Select an object size")
        self.bind("<Configure>", self._schedule_render, add="+")
        self.bind("<Destroy>", self._on_destroy, add="+")
        self._schedule_render()

    @staticmethod
    def _surface_color(level):
        rgb = np.array([99.0, 183.0, 197.0]) * (0.34 + level * 0.73)
        return "#%02x%02x%02x" % tuple(np.clip(rgb, 0, 255).astype(int))

    def set_diameter(self, diameter: int):
        diameter = int(diameter)
        if diameter == self.diameter and self.geometry is not None:
            return
        self.diameter = diameter
        try:
            path = self.mesh_dir / f"{diameter}mm.stl"
            self.source_path = path.resolve()
            self.geometry = load_mesh(path)
            self._mesh_error = None
        except (OSError, ValueError) as exc:
            self.geometry = None
            self._mesh_error = f"Unable to load {diameter} mm STL\n{exc}"
        self.error = self._mesh_error or self._pose_error
        self._schedule_render()

    def set_font_scale(self, scale: float):
        """Scale canvas text independently of this window's Tk widget fonts."""
        scale = max(0.7, min(3.0, float(scale)))
        if abs(scale - self._font_scale) < 0.01:
            return
        self._font_scale = scale
        for item in self._axis_labels.values():
            self.itemconfigure(item, font=("TkDefaultFont", round(12 * scale), "bold"))
        self.itemconfigure(self._status, font=("TkDefaultFont", round(10 * scale)))
        self.itemconfigure(self._message, font=("TkDefaultFont", round(12 * scale)))
        self._schedule_render()

    def set_pose(self, rotation: np.ndarray | None, target_axis: str | None,
                 tracking: bool = True):
        aliases = {"roll": "x", "pitch": "y", "yaw": "z"}
        axis = str(target_axis).lower() if target_axis is not None else None
        axis = aliases.get(axis, axis)
        if axis not in (None, "x", "y", "z"):
            raise ValueError("target_axis must be x, y, z, roll, pitch, yaw, or None")
        changed = axis != self._target_axis or bool(tracking) != self._tracking
        self._target_axis = axis
        self._tracking = bool(tracking)
        if rotation is not None:
            candidate = np.asarray(rotation, dtype=float)
            if (candidate.shape != (3, 3) or not np.isfinite(candidate).all()
                    or not np.allclose(candidate.T @ candidate, np.eye(3), atol=1e-4)
                    or not np.isclose(np.linalg.det(candidate), 1.0, atol=1e-4)):
                self._pose_error = "Orientation unavailable: invalid rotation matrix"
                self._tracking = False
                changed = True
            else:
                changed |= not np.allclose(candidate, self._rotation, atol=1e-6)
                changed |= self._pose_error is not None
                self._rotation = candidate.copy()
                self._pose_error = None
        self.error = self._mesh_error or self._pose_error
        if changed:
            self._schedule_render()

    def _schedule_render(self, event=None):
        if self._pending_render is None:
            self._pending_render = self.after_idle(self._render)

    def _on_destroy(self, event):
        if event.widget is self and self._pending_render is not None:
            self.after_cancel(self._pending_render)
            self._pending_render = None

    def _project(self, points, transform, center, scale):
        camera = np.asarray(points) @ transform.T
        return camera[:, :2] * [scale, -scale] + center

    def _render(self):
        self._pending_render = None
        width, height = self.winfo_width(), self.winfo_height()
        if width < 3 or height < 3:
            return
        center = np.array([width * 0.5, height * 0.54])
        self.coords(self._message, *center)
        self.itemconfigure(self._message, width=max(80, width - 56))
        if self.geometry is None:
            for item in self._triangles + list(self._axis_lines.values()) + list(self._axis_labels.values()) + [self._cue]:
                self.itemconfigure(item, state=tk.HIDDEN)
            self._visible_count = 0
            self.itemconfigure(self._message, text=self._mesh_error or "Select an object size", state=tk.NORMAL)
            self.itemconfigure(self._status, text="")
            return
        self.itemconfigure(self._message, state=tk.HIDDEN)
        mesh = self.geometry
        scale = max(1.0, min(width * 0.35, height * 0.34)) / mesh.radius
        transform = self._camera @ self._rotation
        camera_vertices = mesh.vertices @ transform.T
        projected = camera_vertices[:, :2] * [scale, -scale] + center
        corners = projected[mesh.faces]
        normals = mesh.normals @ transform.T
        visible = np.flatnonzero(normals[:, 2] > 0)
        order = visible[np.argsort(
            camera_vertices[mesh.faces[visible], 2].mean(axis=1), kind="stable"
        )]
        light = np.array([-0.35, 0.5, 0.79])
        light /= np.linalg.norm(light)
        levels = np.rint(np.clip(normals @ light, 0, 1) * 63).astype(int)
        count = len(order)
        while len(self._triangles) < count:
            item = self.create_polygon(0, 0, 0, 0, 0, 0, outline="", fill="#438594")
            self._triangles.append(item)
            self._fills.append(None)
        if count > self._visible_count:
            for item in self._triangles[self._visible_count:count]:
                self.itemconfigure(item, state=tk.NORMAL)
        elif count < self._visible_count:
            for item in self._triangles[count:self._visible_count]:
                self.itemconfigure(item, state=tk.HIDDEN)
        self._visible_count = count
        # The fixed canvas item stack represents depth slots.  Reassign sorted
        # faces to those slots, instead of rebuilding or raising every polygon.
        for index, face in enumerate(order):
            self.coords(self._triangles[index], *corners[face].ravel())
            fill = self._palette[levels[face]]
            if self._fills[index] != fill:
                self.itemconfigure(self._triangles[index], fill=fill)
                self._fills[index] = fill
        self._render_axes(transform, center, scale)
        status = self._pose_error or ("" if self._tracking else "Orientation unavailable · holding view")
        self.itemconfigure(self._status, text=status)
        for item in [self._cue, *self._axis_lines.values(), *self._axis_labels.values(),
                     self._status, self._message]:
            self.tag_raise(item)

    def _render_axes(self, transform, center, scale):
        radius = self.geometry.radius
        for index, axis in enumerate(("x", "y", "z")):
            direction = np.eye(3)[index]
            points = np.stack((direction * radius * 0.12, direction * radius * 1.17))
            coordinates = self._project(points, transform, center, scale)
            selected = axis == self._target_axis
            self.coords(self._axis_lines[axis], *coordinates.ravel())
            self.itemconfigure(self._axis_lines[axis], state=tk.NORMAL,
                               fill=TARGET_COLOR if selected else AXIS_COLORS[axis],
                               width=4 if selected else 2)
            label_point = self._project([direction * radius * 1.32], transform, center, scale)[0]
            self.coords(self._axis_labels[axis], *label_point)
            self.itemconfigure(self._axis_labels[axis], state=tk.NORMAL,
                               fill=TARGET_COLOR if selected else AXIS_COLORS[axis])
        if self._target_axis is None:
            self.itemconfigure(self._cue, state=tk.HIDDEN)
            return
        axis = ("x", "y", "z").index(self._target_axis)
        # Cyclic basis ordering makes the arrow follow positive right-hand rotation.
        basis_a = np.eye(3)[(axis + 1) % 3]
        basis_b = np.eye(3)[(axis + 2) % 3]
        angles = np.linspace(np.deg2rad(20), np.deg2rad(325), 55)
        ring = radius * 0.92 * (np.cos(angles[:, None]) * basis_a + np.sin(angles[:, None]) * basis_b)
        self.coords(self._cue, *self._project(ring, transform, center, scale).ravel())
        self.itemconfigure(self._cue, state=tk.NORMAL)
