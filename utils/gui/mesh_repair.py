"""Prepare consistently oriented STL assets without changing their XYZ frame.

Run ``python -m utils.gui.mesh_repair SOURCE.stl OUTPUT.stl`` to prepare one
standalone GUI mesh. VTK is needed only for preparing dense meshes; the study
GUI reads the final STL using NumPy. Original source files are never overwritten.
"""
from __future__ import annotations

import argparse
from collections import deque
import hashlib
import os
from pathlib import Path
import struct
import tempfile

import numpy as np


STL_RECORD = np.dtype([
    ("normal", "<f4", (3,)),
    ("vertices", "<f4", (3, 3)),
    ("attribute", "<u2"),
])
MAX_DISPLAY_FACES = 20000


def load_stl(path: Path) -> np.ndarray:
    """Read binary/ASCII STL vertices, retaining coordinates and face order."""
    path = Path(path)
    content = path.read_bytes()
    count = int.from_bytes(content[80:84], "little") if len(content) >= 84 else 0
    if len(content) >= 84 and len(content) == 84 + 50 * count:
        triangles = np.frombuffer(content, STL_RECORD, count, offset=84)["vertices"].astype(float)
    else:
        try:
            source = content.decode("ascii")
            if not source.lstrip().lower().startswith("solid"):
                raise ValueError("not ASCII STL")
            vertices = []
            for line in source.splitlines():
                words = line.split()
                if words and words[0].lower() == "vertex":
                    if len(words) != 4:
                        raise ValueError("invalid STL vertex")
                    vertices.append([float(value) for value in words[1:]])
            if not vertices or len(vertices) % 3:
                raise ValueError("incomplete STL triangles")
            triangles = np.asarray(vertices).reshape(-1, 3, 3)
        except (UnicodeDecodeError, ValueError) as exc:
            raise ValueError(f"Invalid or truncated STL: {path.name}") from exc
    if not len(triangles) or not np.isfinite(triangles).all():
        raise ValueError(f"STL contains no finite mesh: {path.name}")
    return triangles


def face_normals(triangles: np.ndarray) -> np.ndarray:
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    if not np.isfinite(normals).all() or np.any(lengths <= 1e-12):
        raise ValueError("Mesh contains degenerate or nonfinite faces")
    return normals / lengths[:, None]


def _topology(triangles):
    vertices, inverse = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    faces = inverse.reshape(-1, 3)
    edges = np.stack((faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]), axis=1).reshape(-1, 2)
    unique, membership, count = np.unique(np.sort(edges, axis=1), axis=0,
                                          return_inverse=True, return_counts=True)
    signs = np.sign(edges[:, 1] - edges[:, 0])
    return vertices, faces, unique, membership, count, signs


def topology_report(triangles: np.ndarray) -> dict:
    vertices, faces, edges, membership, count, signs = _topology(triangles)
    sums = np.bincount(membership, weights=signs)
    return {
        "vertices": len(vertices), "faces": len(faces), "edges": len(edges),
        "boundary_edges": int(np.count_nonzero(count == 1)),
        "nonmanifold_edges": int(np.count_nonzero(count > 2)),
        "inconsistent_edges": int(np.count_nonzero((count == 2) & (sums != 0))),
        "euler_characteristic": len(vertices) - len(edges) + len(faces),
    }


def _inside_closed_surface(point: np.ndarray, triangles: np.ndarray) -> bool:
    """Parity ray test independent of the surface's current face winding."""
    direction = np.array([0.712319, 0.437173, 0.549137])
    direction /= np.linalg.norm(direction)
    first = triangles[:, 1] - triangles[:, 0]
    second = triangles[:, 2] - triangles[:, 0]
    h = np.cross(direction, second)
    determinant = np.einsum("ij,ij->i", first, h)
    usable = np.abs(determinant) > 1e-12
    inverse = np.zeros(len(triangles))
    inverse[usable] = 1 / determinant[usable]
    s = point - triangles[:, 0]
    u = inverse * np.einsum("ij,ij->i", s, h)
    q = np.cross(s, first)
    v = inverse * (q @ direction)
    distance = inverse * np.einsum("ij,ij->i", second, q)
    hits = usable & (u >= 0) & (v >= 0) & (u + v <= 1) & (distance > 1e-9)
    return bool(np.count_nonzero(hits) % 2)


def repair_normals(triangles: np.ndarray) -> tuple[np.ndarray, dict]:
    """Orient faces consistently and preserve inside-facing cavity shells.

    Coordinates and triangle order remain unchanged; only the last two corners
    of flipped faces exchange places. Open components have no unambiguous
    outward direction, so their first face retains its supplied orientation.
    Closed shells nested inside another shell alternate orientation with their
    nesting depth: material exterior outward, enclosed cavities inward.
    """
    triangles = np.asarray(triangles, dtype=float)
    if triangles.ndim != 3 or triangles.shape[1:] != (3, 3) or not len(triangles):
        raise ValueError("Expected triangles with shape (N, 3, 3)")
    face_normals(triangles)
    _, faces, _, membership, count, signs = _topology(triangles)
    if np.any(count > 2):
        raise ValueError("Cannot orient a nonmanifold STL without altering its geometry")
    ordered = np.argsort(membership, kind="stable")
    offsets = np.r_[0, np.cumsum(count)]
    neighbors = [[] for _ in range(len(faces))]
    for edge in np.flatnonzero(count == 2):
        a, b = ordered[offsets[edge]:offsets[edge] + 2]
        first, second = int(a // 3), int(b // 3)
        relation = -int(signs[a] * signs[b])
        neighbors[first].append((second, relation))
        neighbors[second].append((first, relation))
    orientation = np.zeros(len(faces), dtype=np.int8)
    components = []
    for seed in range(len(faces)):
        if orientation[seed]:
            continue
        orientation[seed] = 1
        pending = deque([seed])
        component = []
        while pending:
            current = pending.popleft()
            component.append(current)
            for neighbor, relation in neighbors[current]:
                expected = int(orientation[current]) * relation
                if not orientation[neighbor]:
                    orientation[neighbor] = expected
                    pending.append(neighbor)
                elif orientation[neighbor] != expected:
                    raise ValueError("STL surface is not consistently orientable")
        components.append(np.asarray(component, dtype=int))
    face_edges = membership.reshape(-1, 3)
    closed = [bool(np.all(count[face_edges[component]] == 2)) for component in components]
    component_bounds = [np.stack((triangles[c].min((0, 1)), triangles[c].max((0, 1))))
                        for c in components]
    cavity_components = 0
    for index, component in enumerate(components):
        if closed[index]:
            depth = 0
            bounds = component_bounds[index]
            # Surface points identify the containing shell without accidentally
            # sampling an interior cavity of the component being classified.
            samples = triangles[component[np.linspace(0, len(component) - 1, 3, dtype=int)]].mean(axis=1)
            for other, candidate in enumerate(components):
                if other == index or not closed[other]:
                    continue
                outside = component_bounds[other]
                if np.all(bounds[0] >= outside[0] - 1e-9) and np.all(bounds[1] <= outside[1] + 1e-9):
                    if all(_inside_closed_surface(point, triangles[candidate]) for point in samples):
                        depth += 1
            inward = depth % 2 == 1
            cavity_components += int(inward)
            corners = triangles[component]
            # Translate before the signed-volume sum to avoid cancellation for
            # meshes whose physical coordinate origin lies far from the object.
            corners = corners - corners.reshape(-1, 3).mean(axis=0)
            volumes = np.einsum("ij,ij->i", corners[:, 0], np.cross(corners[:, 1], corners[:, 2]))
            volume_sign = float(np.dot(volumes, orientation[component]))
            if (volume_sign < 0) != inward:
                orientation[component] *= -1
    fixed = triangles.copy()
    flipped = orientation < 0
    fixed[flipped] = fixed[flipped][:, [0, 2, 1]]
    report = topology_report(fixed)
    report.update(components=len(components), closed_components=sum(closed),
                  cavity_components=cavity_components,
                  flipped_faces=int(np.count_nonzero(flipped)))
    return fixed, report


def write_stl(path: Path, triangles: np.ndarray):
    """Save binary STL with unit normals recomputed from the saved vertices."""
    records = np.zeros(len(triangles), dtype=STL_RECORD)
    records["vertices"] = triangles
    records["normal"] = face_normals(records["vertices"].astype(float))
    header = b"Finger study: consistent winding, recomputed unit normals".ljust(80, b"\0")
    Path(path).write_bytes(header + struct.pack("<I", len(records)) + records.tobytes())


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def prepare_assets(source: Path, output: Path, feature_angle: float = 24.0,
                   max_error_mm: float = 0.02, *, overwrite: bool = False) -> dict:
    """Save one corrected, topology-preserving STL, retaining the source.

    Validate the serialized result before atomically installing it. The report
    is returned to the caller; no sidecar or intermediate assets are retained.
    """
    source = Path(source).resolve()
    output = Path(output).resolve()
    if output == source or (output.exists() and source.samefile(output)):
        raise ValueError("Source and output must be different files")
    if output.exists() and not overwrite:
        raise FileExistsError(f"Output already exists: {output}; use --overwrite to replace it")
    if not np.isfinite(max_error_mm) or max_error_mm <= 0:
        raise ValueError("max_error_mm must be positive and finite")
    if not np.isfinite(feature_angle) or not 0 <= feature_angle <= 180:
        raise ValueError("feature_angle must be between 0 and 180 degrees")
    fixed, original_report = repair_normals(load_stl(source))
    if len(fixed) <= MAX_DISPLAY_FACES:
        display = fixed
    else:
        try:
            import vtk
            from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray, vtk_to_numpy
        except ImportError as exc:
            raise RuntimeError("Preparing a dense STL requires VTK (python3-vtk9 or pip install vtk); "
                               "VTK is not required to run the GUI with prepared assets") from exc
        vertices, inverse = np.unique(fixed.reshape(-1, 3), axis=0, return_inverse=True)
        points = vtk.vtkPoints()
        points.SetData(numpy_to_vtk(vertices, deep=True))
        connectivity = np.column_stack((np.full(len(fixed), 3), inverse.reshape(-1, 3))).astype(np.int64)
        cells = vtk.vtkCellArray()
        cells.SetCells(len(fixed), numpy_to_vtkIdTypeArray(connectivity.ravel(), deep=True))
        poly = vtk.vtkPolyData()
        poly.SetPoints(points)
        poly.SetPolys(cells)
        decimator = vtk.vtkDecimatePro()
        decimator.SetInputData(poly)
        decimator.PreserveTopologyOn()
        decimator.SplittingOff()
        decimator.BoundaryVertexDeletionOff()
        decimator.SetFeatureAngle(feature_angle)
        # Topology alone does not preserve the curved wall's geometry. Without
        # an error bound, large tilted triangles produce false lighting patches.
        # Accumulation makes this a global error bound across edge collapses.
        decimator.SetErrorIsAbsolute(1)
        decimator.SetAbsoluteError(max_error_mm)
        decimator.AccumulateErrorOn()
        decimator.SetTargetReduction(1.0 - 2600 / len(fixed))
        decimator.Update()
        result = decimator.GetOutput()
        display_vertices = vtk_to_numpy(result.GetPoints().GetData())
        display_faces = vtk_to_numpy(result.GetPolys().GetData()).reshape(-1, 4)[:, 1:]
        display, display_report = repair_normals(display_vertices[display_faces])
        for key in ("boundary_edges", "nonmanifold_edges", "euler_characteristic", "components", "closed_components", "cavity_components"):
            if display_report[key] != original_report[key]:
                raise ValueError(f"Display decimation changed {key}; use a smaller --feature-angle")
    if len(display) > MAX_DISPLAY_FACES:
        raise ValueError(f"Display mesh still has {len(display)} faces (limit {MAX_DISPLAY_FACES}); "
                         "adjust --feature-angle, retaining topology validation")
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".mesh-prepare-", dir=output.parent) as directory:
        temporary = Path(directory) / "prepared.stl"
        write_stl(temporary, display)
        _, display_report = repair_normals(load_stl(temporary))
        if display_report["flipped_faces"]:
            raise ValueError("Saved display mesh changed its orientation")
        for key in ("boundary_edges", "nonmanifold_edges", "euler_characteristic", "components",
                    "closed_components", "cavity_components"):
            if display_report[key] != original_report[key]:
                raise ValueError(f"Saved display mesh changed {key}")
        if overwrite:
            os.replace(temporary, output)
        else:
            # A hard link refuses an output concurrently created by another
            # process and leaves the validated bytes installed atomically.
            os.link(temporary, output)
    result = {
        "version": 1,
        "method": "consistent winding, nesting-aware cavity normals, unit normals; VTK DecimatePro",
        "feature_angle_degrees": feature_angle,
        "max_accumulated_error_mm": max_error_mm,
        "source": {"file": source.name, "sha256": _sha256(source), "topology": original_report},
        "display": {"file": output.name, "sha256": _sha256(output), "topology": display_report},
    }
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="source STL, kept unchanged")
    parser.add_argument("output", type=Path, help="standalone final STL written for the GUI")
    parser.add_argument("--overwrite", action="store_true", help="replace an existing output STL")
    parser.add_argument("--feature-angle", type=float, default=24.0)
    parser.add_argument("--max-error-mm", type=float, default=0.02,
                        help="maximum accumulated geometric error in millimetres (default 0.02)")
    args = parser.parse_args()
    try:
        result = prepare_assets(args.source, args.output, args.feature_angle, args.max_error_mm,
                                overwrite=args.overwrite)
    except (OSError, ValueError, RuntimeError) as exc:
        parser.exit(1, f"Mesh preparation failed: {exc}\n")
    print(f"{args.source.name}: {result['source']['topology']['faces']} original faces -> "
          f"{result['display']['topology']['faces']} display faces; saved {args.output}")


if __name__ == "__main__":
    main()
