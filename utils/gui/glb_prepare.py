"""Prepare compact, coloured GLB display meshes while preserving scene XYZ.

Run ``python -m utils.gui.glb_prepare SOURCE.glb OUTPUT.glb [--overwrite]``.
VTK is used only for dense offline preparation; prepared files need only NumPy.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import struct
import tempfile

import numpy as np

from .glb_mesh import GLBGeometry, load_glb
from .mesh_repair import MAX_DISPLAY_FACES, face_normals, topology_report


def _groups(mesh: GLBGeometry):
    for node in np.unique(mesh.face_nodes):
        for material in np.unique(mesh.face_materials[mesh.face_nodes == node]):
            selected = (mesh.face_nodes == node) & (mesh.face_materials == material)
            yield int(node), int(material), mesh.vertices[mesh.faces[selected]]


def _signature(triangles):
    report = topology_report(triangles)
    vertices, inverse = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    parents = list(range(len(vertices)))

    def root(vertex):
        while parents[vertex] != vertex:
            parents[vertex] = parents[parents[vertex]]
            vertex = parents[vertex]
        return vertex

    for face in inverse.reshape(-1, 3):
        first = root(int(face[0]))
        for vertex in face[1:]:
            other = root(int(vertex))
            if first != other:
                parents[other] = first
    return tuple(report[key] for key in ("boundary_edges", "nonmanifold_edges",
                                         "inconsistent_edges", "euler_characteristic")) + (
        len({root(vertex) for vertex in range(len(vertices))}),)


def _decimate(triangles, feature_angle, max_error_mm, protected_faces=None):
    try:
        import vtk
        from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray, vtk_to_numpy
    except ImportError as exc:
        raise RuntimeError("Preparing dense GLB files requires VTK (python3-vtk9 or pip install vtk).") from exc
    vertices, inverse = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    # VTK's error is a collapse metric, not a bound on each Cartesian extremum.
    # Keep a small original triangle patch around one min/max point per axis.
    # The remaining surface has a pinned boundary, so reattaching these exact
    # patches preserves each coloured part's bounds without moving geometry.
    extrema = np.unique(np.concatenate((vertices.argmin(axis=0), vertices.argmax(axis=0))))
    protected = np.isin(inverse.reshape(-1, 3), extrema).any(axis=1)
    if protected_faces is not None:
        supplied = np.asarray(protected_faces, dtype=bool)
        if supplied.shape != (len(triangles),):
            raise ValueError("Protected faces must match the source triangle count.")
        protected |= supplied
    fixed = triangles[protected]
    free = triangles[~protected]
    if not len(free):
        return triangles.copy()
    vertices, inverse = np.unique(free.reshape(-1, 3), axis=0, return_inverse=True)
    points = vtk.vtkPoints()
    points.SetData(numpy_to_vtk(vertices, deep=True))
    connectivity = np.column_stack((np.full(len(free), 3), inverse.reshape(-1, 3))).astype(np.int64)
    cells = vtk.vtkCellArray()
    cells.SetCells(len(free), numpy_to_vtkIdTypeArray(connectivity.ravel(), deep=True))
    poly = vtk.vtkPolyData()
    poly.SetPoints(points)
    poly.SetPolys(cells)
    decimator = vtk.vtkDecimatePro()
    decimator.SetInputData(poly)
    decimator.PreserveTopologyOn()
    decimator.SplittingOff()
    decimator.BoundaryVertexDeletionOff()
    decimator.SetFeatureAngle(feature_angle)
    decimator.SetErrorIsAbsolute(1)
    decimator.SetAbsoluteError(max_error_mm)
    decimator.AccumulateErrorOn()
    decimator.SetTargetReduction(.98)
    decimator.Update()
    output = decimator.GetOutput()
    if not output.GetNumberOfCells():
        raise ValueError("Decimation removed an entire node/material region.")
    display_vertices = vtk_to_numpy(output.GetPoints().GetData())
    display_faces = vtk_to_numpy(output.GetPolys().GetData()).reshape(-1, 4)[:, 1:]
    return np.concatenate((fixed, display_vertices[display_faces]))


def _surface_error(original, display, refine_limit):
    """Measure both surfaces at vertices/face centres and locate source pins."""
    import vtk
    from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray, vtk_to_numpy

    def poly(triangles):
        vertices, inverse = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
        points = vtk.vtkPoints()
        points.SetData(numpy_to_vtk(vertices, deep=True))
        cells = vtk.vtkCellArray()
        connectivity = np.column_stack((np.full(len(triangles), 3), inverse.reshape(-1, 3))).astype(np.int64)
        cells.SetCells(len(triangles), numpy_to_vtkIdTypeArray(connectivity.ravel(), deep=True))
        result = vtk.vtkPolyData()
        result.SetPoints(points)
        result.SetPolys(cells)
        return result, vertices, inverse.reshape(-1, 3)

    source_poly, source_vertices, source_faces = poly(original)
    display_poly, display_vertices, _ = poly(display)
    distance = vtk.vtkDistancePolyDataFilter()
    distance.SetInputData(0, source_poly)
    distance.SetInputData(1, display_poly)
    distance.SignedDistanceOff()
    distance.ComputeSecondDistanceOn()
    distance.ComputeCellCenterDistanceOn()
    distance.Update()
    source_points = vtk_to_numpy(distance.GetOutput(0).GetPointData().GetArray("Distance"))
    source_centres = vtk_to_numpy(distance.GetOutput(0).GetCellData().GetArray("Distance"))
    display_points = vtk_to_numpy(distance.GetOutput(1).GetPointData().GetArray("Distance"))
    display_centres = vtk_to_numpy(distance.GetOutput(1).GetCellData().GetArray("Distance"))
    maximum = max(float(values.max()) for values in
                  (source_points, source_centres, display_points, display_centres))
    protected = source_centres > refine_limit
    # Keeping one incident original face is enough to pin a bad source vertex;
    # keeping every adjacent face would unnecessarily retain large CAD patches.
    first_face = np.full(len(source_vertices), len(original), dtype=int)
    np.minimum.at(first_face, source_faces.ravel(), np.repeat(np.arange(len(original)), 3))
    protected[first_face[source_points > refine_limit]] = True
    reverse_samples = np.concatenate((display_vertices[display_points > refine_limit],
                                      display[display_centres > refine_limit].mean(axis=1)))
    if len(reverse_samples):
        locator = vtk.vtkStaticCellLocator()
        locator.SetDataSet(source_poly)
        locator.BuildLocator()
        for point in reverse_samples:
            closest = [0., 0., 0.]
            cell, subcell, squared_distance = vtk.reference(0), vtk.reference(0), vtk.reference(0.)
            locator.FindClosestPoint(point, closest, cell, subcell, squared_distance)
            protected[int(cell)] = True
    return maximum, protected


def write_glb(path, mesh: GLBGeometry, groups):
    """Write baked scene positions in metres, retaining node/material identity.

    Groups contain millimetre triangles. Flat per-face normals keep sharp CAD
    boundaries sharp and avoid averaging the normals of adjoining surfaces.
    """
    binary = bytearray()
    document = {
        "asset": {"version": "2.0", "generator": "Finger study: colour-preserving display preparation"},
        "scene": 0, "scenes": [{"nodes": list(range(len(mesh.node_names)))}],
        "nodes": [{"name": name} for name in mesh.node_names],
        "meshes": [], "materials": [], "bufferViews": [], "accessors": [], "buffers": [],
    }
    for name, color in zip(mesh.material_names, mesh.material_rgba):
        material = {"name": name, "doubleSided": True,
                    "pbrMetallicRoughness": {"baseColorFactor": color.tolist(),
                                             "metallicFactor": 0.0, "roughnessFactor": 1.0}}
        if color[3] < 1:
            material["alphaMode"] = "BLEND"
        document["materials"].append(material)

    def accessor(array, component_type, kind, target):
        while len(binary) % 4:
            binary.append(0)
        view = len(document["bufferViews"])
        payload = array.tobytes()
        document["bufferViews"].append({"buffer": 0, "byteOffset": len(binary),
                                         "byteLength": len(payload), "target": target})
        binary.extend(payload)
        entry = {"bufferView": view, "componentType": component_type,
                 "count": len(array), "type": kind,
                 "min": array.min(axis=0).reshape(-1).tolist(),
                 "max": array.max(axis=0).reshape(-1).tolist()}
        document["accessors"].append(entry)
        return len(document["accessors"]) - 1

    node_meshes = {}
    for node, material, triangles in groups:
        positions = (np.asarray(triangles).reshape(-1, 3) / 1000).astype("<f4")
        normals = np.repeat(face_normals(triangles), 3, axis=0).astype("<f4")
        indices = np.arange(len(positions), dtype="<u4")
        primitive = {"attributes": {
            "POSITION": accessor(positions, 5126, "VEC3", 34962),
            "NORMAL": accessor(normals, 5126, "VEC3", 34962)},
            "indices": accessor(indices, 5125, "SCALAR", 34963), "mode": 4}
        if material >= 0:
            primitive["material"] = material
        if node not in node_meshes:
            node_meshes[node] = len(document["meshes"])
            document["meshes"].append({"name": mesh.node_names[node], "primitives": []})
            document["nodes"][node]["mesh"] = node_meshes[node]
        document["meshes"][node_meshes[node]]["primitives"].append(primitive)
    document["buffers"] = [{"byteLength": len(binary)}]
    while len(binary) % 4:
        binary.append(0)
    header = json.dumps(document, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    header += b" " * (-len(header) % 4)
    payload = (struct.pack("<4sII", b"glTF", 2, 12 + 8 + len(header) + 8 + len(binary))
               + struct.pack("<II", len(header), 0x4E4F534A) + header
               + struct.pack("<II", len(binary), 0x004E4942) + binary)
    Path(path).write_bytes(payload)


def prepare_glb(source, output, *, overwrite=False, feature_angle=24.0,
                max_error_mm=.02, max_faces=MAX_DISPLAY_FACES, max_sampled_error_mm=.15):
    """Prepare a standalone GLB; preserve its source and validate before install.

    max_error_mm configures VTK's collapse error for millimetre coordinates; it
    is not a strict surface-distance bound. Original part/material extrema and
    topology are independently preserved and checked after serialization. Dense
    meshes also receive at most two refinement passes to bound measured vertex
    and face-centre distances in both directions; this is not a Hausdorff proof.
    """
    source, output = Path(source).resolve(), Path(output).resolve()
    if source == output or (output.exists() and source.samefile(output)):
        raise ValueError("Source and output must be different files.")
    if output.exists() and not overwrite:
        raise FileExistsError(f"Output already exists: {output}; use --overwrite to replace it.")
    if not np.isfinite(feature_angle) or not 0 <= feature_angle <= 180:
        raise ValueError("feature_angle must be between 0 and 180 degrees.")
    if not np.isfinite(max_error_mm) or max_error_mm <= 0:
        raise ValueError("max_error_mm must be positive and finite.")
    if not np.isfinite(max_sampled_error_mm) or max_sampled_error_mm <= 0:
        raise ValueError("max_sampled_error_mm must be positive and finite.")
    if isinstance(max_faces, bool) or not isinstance(max_faces, int) or max_faces < 1:
        raise ValueError("max_faces must be a positive integer.")
    mesh = load_glb(source)
    original = list(_groups(mesh))
    prepared = []
    dense = len(mesh.faces) > max_faces
    signatures = {}
    group_bounds = {}
    for node, material, triangles in original:
        signature = _signature(triangles)
        display = _decimate(triangles, feature_angle, max_error_mm) if dense else triangles
        if _signature(display) != signature:
            raise ValueError(f"Display decimation changed topology for node {node}, material {material}.")
        face_normals(display)
        signatures[node, material] = signature
        group_bounds[node, material] = np.stack((triangles.min((0, 1)), triangles.max((0, 1))))
        prepared.append((node, material, display))
    measured_error = 0.0
    if dense:
        originals = {(node, material): triangles for node, material, triangles in original}
        protected = {key: np.zeros(len(triangles), dtype=bool) for key, triangles in originals.items()}
        for refinement in range(3):
            measured_error = 0.0
            next_protected = {}
            for node, material, display in prepared:
                error, extra = _surface_error(originals[node, material], display,
                                               max_sampled_error_mm * (2 / 3))
                measured_error = max(measured_error, error)
                next_protected[node, material] = protected[node, material] | extra
            if measured_error <= max_sampled_error_mm:
                break
            if refinement == 2:
                raise ValueError(f"Display mesh sampled surface error {measured_error:.6g} mm exceeds "
                                 f"{max_sampled_error_mm} mm after bounded refinement.")
            updated = []
            for node, material, display in prepared:
                key = node, material
                if not np.array_equal(protected[key], next_protected[key]):
                    display = _decimate(originals[key], feature_angle, max_error_mm,
                                        protected_faces=next_protected[key])
                if _signature(display) != signatures[key]:
                    raise ValueError(f"Surface refinement changed topology for node {node}, material {material}.")
                updated.append((node, material, display))
            if sum(len(group[2]) for group in updated) > max_faces:
                raise ValueError("Surface-distance refinement cannot fit the requested face budget; "
                                 "revise source detail or validated preparation settings.")
            protected, prepared = next_protected, updated
    count = sum(len(group[2]) for group in prepared)
    if count > max_faces:
        raise ValueError(f"Display mesh still has {count} faces (limit {max_faces}); revise the source detail or preparation settings.")
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".glb-prepare-", dir=output.parent) as temporary_directory:
        temporary = Path(temporary_directory) / "prepared.glb"
        write_glb(temporary, mesh, prepared)
        verified = load_glb(temporary)
        if len(verified.faces) != count:
            raise ValueError("Serialized display GLB changed the triangle count.")
        if verified.node_names != mesh.node_names or verified.material_names != mesh.material_names:
            raise ValueError("Serialized display GLB changed node/material identity.")
        if not np.array_equal(verified.material_rgba, mesh.material_rgba):
            raise ValueError("Serialized display GLB changed material colours.")
        if not np.allclose(verified.bounds, mesh.bounds, atol=max_error_mm + 1e-5, rtol=0):
            raise ValueError("Display GLB changed physical bounds beyond the geometric tolerance.")
        for node, material, triangles in _groups(verified):
            if _signature(triangles) != signatures[node, material]:
                raise ValueError(f"Serialized display GLB changed topology for node {node}, material {material}.")
            bounds = np.stack((triangles.min((0, 1)), triangles.max((0, 1))))
            if not np.allclose(bounds, group_bounds[node, material], atol=1e-5, rtol=0):
                raise ValueError(f"Display GLB changed physical bounds for node {node}, material {material}.")
        if overwrite:
            os.replace(temporary, output)
        else:
            os.link(temporary, output)
    return {"source": {"file": source.name, "sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
                        "faces": len(mesh.faces)},
            "display": {"file": output.name, "faces": count,
                        "nodes": len(mesh.node_names), "materials": len(mesh.material_names),
                        "bounds_mm": verified.bounds.tolist(), "origin_mm": verified.origin.tolist()},
            "max_error_mm": max_error_mm, "feature_angle": feature_angle,
            "max_sampled_surface_error_mm": measured_error,
            "sampled_surface_error_limit_mm": max_sampled_error_mm}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--max-error-mm", type=float, default=.02,
                        help="VTK absolute collapse-error setting for millimetre coordinates (default .02)")
    parser.add_argument("--feature-angle", type=float, default=24.0)
    parser.add_argument("--max-sampled-error-mm", type=float, default=.15,
                        help="limit for sampled bidirectional vertex/face-centre distances (default .15)")
    args = parser.parse_args(argv)
    try:
        report = prepare_glb(args.source, args.output, overwrite=args.overwrite,
                             max_error_mm=args.max_error_mm, feature_angle=args.feature_angle,
                             max_sampled_error_mm=args.max_sampled_error_mm)
    except (OSError, ValueError, RuntimeError) as exc:
        parser.exit(1, f"GLB preparation failed: {exc}\n")
    print(f"{args.source.name}: {report['source']['faces']} source faces -> "
          f"{report['display']['faces']} display faces; saved {args.output}")


if __name__ == "__main__":
    main()
