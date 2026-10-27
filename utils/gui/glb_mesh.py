"""Read flat-colour GLB geometry without changing its scene axes or origin.

The Onshape export uses metres; the returned coordinates are millimetres.
``center`` and ``radius`` describe camera framing only. Vertices are never
translated by that center, and the scene origin remains (0, 0, 0).
"""
from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
import json
import math
import operator
from pathlib import Path
import struct

import numpy as np


_COMPONENTS = {5120: np.dtype("i1"), 5121: np.dtype("u1"),
               5122: np.dtype("<i2"), 5123: np.dtype("<u2"),
               5125: np.dtype("<u4"), 5126: np.dtype("<f4")}
_WIDTHS = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}


@dataclass(frozen=True, eq=False)
class GLBGeometry:
    vertices: np.ndarray
    faces: np.ndarray
    normals: np.ndarray
    face_colors: np.ndarray
    face_materials: np.ndarray
    material_names: tuple[str, ...]
    material_rgba: np.ndarray
    face_nodes: np.ndarray
    node_names: tuple[str, ...]
    bounds: np.ndarray
    center: np.ndarray
    origin: np.ndarray
    radius: float


def _integer(value, label):
    if isinstance(value, bool):
        raise ValueError(f"{label} must be a nonnegative integer.")
    try:
        result = operator.index(value)
    except TypeError as exc:
        raise ValueError(f"{label} must be a nonnegative integer.") from exc
    if result < 0:
        raise ValueError(f"{label} must be a nonnegative integer.")
    return result


def _entry(items, index, label):
    index = _integer(index, label)
    if index >= len(items):
        raise ValueError(f"Invalid {label}: {index}.")
    return items[index]


def _container(content):
    if len(content) < 20:
        raise ValueError("Invalid or truncated GLB header.")
    magic, version, length = struct.unpack_from("<4sII", content)
    if magic != b"glTF" or version != 2 or length != len(content):
        raise ValueError("Expected a complete GLB version 2 file.")
    document = binary = None
    offset = 12
    while offset < length:
        if offset + 8 > length:
            raise ValueError("Truncated GLB chunk header.")
        size, kind = struct.unpack_from("<II", content, offset)
        offset += 8
        if size % 4 or offset + size > length:
            raise ValueError("Truncated or unaligned GLB chunk.")
        chunk = content[offset:offset + size]
        if document is None and kind != 0x4E4F534A:
            raise ValueError("The first GLB chunk must contain JSON.")
        if kind == 0x4E4F534A:
            if document is not None:
                raise ValueError("GLB contains multiple JSON chunks.")
            try:
                document = json.loads(chunk.decode("utf-8").rstrip(" \t\r\n\0"))
            except (ValueError, UnicodeDecodeError) as exc:
                raise ValueError("Invalid GLB JSON.") from exc
        elif kind == 0x004E4942:
            if binary is not None:
                raise ValueError("GLB contains multiple binary chunks.")
            binary = chunk
        offset += size
    if not isinstance(document, dict) or binary is None:
        raise ValueError("GLB requires JSON and embedded binary geometry.")
    if document.get("asset", {}).get("version") != "2.0":
        raise ValueError("Only glTF 2.0 assets are supported.")
    if document.get("extensionsRequired"):
        raise ValueError("Unsupported required GLB extensions: " +
                         ", ".join(document["extensionsRequired"]))
    buffers = document.get("buffers", [])
    if len(buffers) != 1 or "uri" in buffers[0]:
        raise ValueError("GLB must use one embedded buffer, without external files.")
    size = _integer(buffers[0].get("byteLength"), "buffer byteLength")
    if size > len(binary):
        raise ValueError("GLB binary buffer is truncated.")
    return document, binary[:size]


def _accessor(document, binary, index):
    accessor = _entry(document.get("accessors", []), index, "accessor")
    if "sparse" in accessor:
        raise ValueError("Sparse GLB accessors are not supported.")
    view = _entry(document.get("bufferViews", []), accessor.get("bufferView"), "bufferView")
    if view.get("extensions"):
        raise ValueError("Compressed GLB bufferViews are not supported; export without compression.")
    if view.get("buffer", 0) != 0:
        raise ValueError("GLB accessor references an external buffer.")
    dtype = _COMPONENTS.get(accessor.get("componentType"))
    width = _WIDTHS.get(accessor.get("type"))
    if dtype is None or width is None:
        raise ValueError("Unsupported GLB accessor component type or vector shape.")
    count = _integer(accessor.get("count"), "accessor count")
    view_start = _integer(view.get("byteOffset", 0), "bufferView byteOffset")
    view_size = _integer(view.get("byteLength"), "bufferView byteLength")
    relative = _integer(accessor.get("byteOffset", 0), "accessor byteOffset")
    packed = width * dtype.itemsize
    stride = _integer(view.get("byteStride", packed), "byteStride")
    if (stride < packed or stride % dtype.itemsize or
            (view_start + relative) % dtype.itemsize):
        raise ValueError("Invalid GLB accessor stride or alignment.")
    needed = relative + ((count - 1) * stride + packed if count else 0)
    if view_start + view_size > len(binary) or needed > view_size:
        raise ValueError("GLB accessor extends beyond its bufferView.")
    result = np.ndarray((count, width), dtype=dtype, buffer=binary,
                        offset=view_start + relative, strides=(stride, dtype.itemsize)).copy()
    if accessor.get("normalized", False):
        if dtype.kind not in "iu":
            raise ValueError("Only integer GLB attributes may be normalized.")
        limit = np.iinfo(dtype).max
        result = result.astype(float) / limit
        if dtype.kind == "i":
            result = np.maximum(result, -1)
    return result, accessor


def _vector(value, width, label):
    try:
        result = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"Invalid {label}.") from exc
    if result.shape != (width,) or not np.isfinite(result).all():
        raise ValueError(f"{label} must contain {width} finite values.")
    return result


def _node_transform(node):
    if "matrix" in node:
        if any(key in node for key in ("translation", "rotation", "scale")):
            raise ValueError("A GLB node cannot specify both matrix and TRS transforms.")
        matrix = _vector(node["matrix"], 16, "node matrix").reshape(4, 4, order="F")
        if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-12, rtol=0):
            raise ValueError("GLB node matrix must be affine.")
    else:
        translation = _vector(node.get("translation", [0, 0, 0]), 3, "node translation")
        scale = _vector(node.get("scale", [1, 1, 1]), 3, "node scale")
        quaternion = _vector(node.get("rotation", [0, 0, 0, 1]), 4, "node quaternion")
        length = np.linalg.norm(quaternion)
        if not math.isclose(float(length), 1.0, abs_tol=1e-5):
            raise ValueError("GLB node quaternion must have unit length.")
        x, y, z, w = quaternion / length
        rotation = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
        ])
        matrix = np.eye(4)
        matrix[:3, :3] = rotation * scale
        matrix[:3, 3] = translation
    determinant = float(np.linalg.det(matrix[:3, :3]))
    if not math.isfinite(determinant) or determinant == 0:
        raise ValueError("GLB node transform is singular.")
    return matrix


def _materials(document):
    names, colors = [], []
    for index, material in enumerate(document.get("materials", [])):
        pbr = material.get("pbrMetallicRoughness", {})
        if (any(key.endswith("Texture") for key in material) or
                any(key.endswith("Texture") for key in pbr)):
            raise ValueError("Textured GLB materials are not supported; use flat face colours.")
        if material.get("alphaMode", "OPAQUE") != "OPAQUE":
            raise ValueError("Transparent GLB materials are not supported.")
        color = _vector(pbr.get("baseColorFactor", [1, 1, 1, 1]), 4, "baseColorFactor")
        if np.any((color < 0) | (color > 1)):
            raise ValueError("GLB baseColorFactor values must be between zero and one.")
        # glTF OPAQUE materials ignore alpha, even when a factor is provided.
        color[3] = 1
        colors.append(color)
        names.append(str(material.get("name", f"material_{index}")))
    return names, colors


@lru_cache(maxsize=4)
def _geometry(content):
    document, binary = _container(content)
    nodes = document.get("nodes", [])
    scenes = document.get("scenes", [])
    if scenes:
        roots = _entry(scenes, document.get("scene", 0), "scene").get("nodes", [])
    else:
        children = {child for node in nodes for child in node.get("children", [])}
        roots = [index for index in range(len(nodes)) if index not in children]
    meshes = document.get("meshes", [])
    material_names, material_colors = _materials(document)
    source_material_count = len(material_colors)
    node_names = tuple(str(node.get("name", f"node_{index}")) for index, node in enumerate(nodes))
    vertices, faces, face_materials, face_nodes = [], [], [], []
    vertex_offset = 0
    visited = set()
    default_material = None

    def visit(node_index, parent):
        nonlocal vertex_offset, default_material
        node_index = _integer(node_index, "node")
        node = _entry(nodes, node_index, "node")
        if node_index in visited:
            raise ValueError("GLB scene contains a cycle or a node with multiple parents.")
        visited.add(node_index)
        transform = parent @ _node_transform(node)
        if "skin" in node:
            raise ValueError("Skinned GLB geometry is not supported.")
        if "mesh" in node:
            mesh = _entry(meshes, node["mesh"], "mesh")
            for primitive in mesh.get("primitives", []):
                if primitive.get("mode", 4) != 4:
                    raise ValueError("GLB display geometry must contain triangles.")
                if primitive.get("extensions") or primitive.get("targets"):
                    raise ValueError("Compressed or morph-target GLB primitives are not supported.")
                attributes = primitive.get("attributes", {})
                if "COLOR_0" in attributes:
                    raise ValueError("GLB vertex colours are not supported; use flat face materials.")
                positions, position_info = _accessor(document, binary, attributes.get("POSITION"))
                if (position_info.get("type") != "VEC3" or
                        position_info.get("componentType") != 5126 or position_info.get("normalized", False)):
                    raise ValueError("GLB POSITION must contain non-normalized float VEC3 values.")
                if not np.isfinite(positions).all():
                    raise ValueError("GLB positions contain nonfinite coordinates.")
                if "indices" in primitive:
                    indices, index_info = _accessor(document, binary, primitive["indices"])
                    if (index_info.get("type") != "SCALAR" or index_info.get("normalized", False)
                            or index_info.get("componentType") not in (5121, 5123, 5125)):
                        raise ValueError("GLB triangle indices must be unsigned integers.")
                    indices = indices.reshape(-1).astype(np.int64)
                else:
                    indices = np.arange(len(positions), dtype=np.int64)
                if not len(indices) or len(indices) % 3 or np.any(indices >= len(positions)):
                    raise ValueError("Invalid or incomplete GLB triangles.")
                triangles = indices.reshape(-1, 3)
                if np.linalg.det(transform[:3, :3]) < 0:
                    triangles = triangles[:, [0, 2, 1]]
                world = (positions.astype(float) @ transform[:3, :3].T + transform[:3, 3]) * 1000.0
                if not np.isfinite(world).all():
                    raise ValueError("GLB transformed positions are nonfinite.")
                if "material" in primitive:
                    material_index = _integer(primitive["material"], "material")
                    if material_index >= source_material_count:
                        raise ValueError(f"Invalid material: {material_index}.")
                else:
                    if default_material is None:
                        default_material = len(material_colors)
                        material_colors.append(np.ones(4))
                        material_names.append("default_material")
                    material_index = default_material
                vertices.append(world)
                faces.append(triangles + vertex_offset)
                face_materials.append(np.full(len(triangles), material_index, dtype=np.int64))
                face_nodes.append(np.full(len(triangles), node_index, dtype=np.int64))
                vertex_offset += len(world)
        for child in node.get("children", []):
            visit(child, transform)

    for root in roots:
        visit(root, np.eye(4))
    if not faces:
        raise ValueError("GLB scene contains no triangle geometry.")
    vertices = np.concatenate(vertices)
    faces = np.concatenate(faces)
    face_materials = np.concatenate(face_materials)
    face_nodes = np.concatenate(face_nodes)
    material_rgba = np.asarray(material_colors, dtype=float)
    corners = vertices[faces]
    normals = np.cross(corners[:, 1] - corners[:, 0], corners[:, 2] - corners[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    valid = np.isfinite(lengths) & (lengths > 1e-12)
    if not np.any(valid):
        raise ValueError("GLB contains no non-degenerate triangle geometry.")
    faces, face_materials, face_nodes = faces[valid], face_materials[valid], face_nodes[valid]
    normals = normals[valid] / lengths[valid, None]
    face_colors = material_rgba[face_materials].copy()
    bounds = np.stack((vertices.min(axis=0), vertices.max(axis=0)))
    center = bounds.mean(axis=0)
    radius = float(np.linalg.norm(vertices - center, axis=1).max())
    if not math.isfinite(radius) or radius <= 0:
        raise ValueError("GLB geometry has zero or nonfinite extent.")
    origin = np.zeros(3)
    for array in (vertices, faces, normals, face_colors, face_materials, material_rgba,
                  face_nodes, bounds, center, origin):
        array.setflags(write=False)
    return GLBGeometry(vertices, faces, normals, face_colors, face_materials,
                       tuple(material_names), material_rgba, face_nodes, node_names,
                       bounds, center, origin, radius)


def load_glb(path: str | Path) -> GLBGeometry:
    """Return a cached, immutable flat-colour scene in its original XYZ frame."""
    return _geometry(Path(path).read_bytes())
