#!/usr/bin/env python3

"""Export the active COLLADA visual scene as metre-scaled binary STL.

The Pro630 source meshes store coordinates in millimetres and declare that
scale in COLLADA metadata.  Some collision loaders ignore the metadata, so the
unit conversion is baked into STL coordinates here.  Geometry definitions are
not a complete scene: the force-control gripper uses nested ``instance_node``
elements and matrices to assemble its parts.  The exporter therefore walks the
active visual scene and applies every node transform before writing triangles.
"""

import argparse
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET


NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}


def _source_positions(mesh, source_id):
    source = mesh.find(f"c:source[@id='{source_id}']", NS)
    if source is None:
        raise ValueError(f"missing position source {source_id}")
    values_tag = source.find("c:float_array", NS)
    accessor = source.find("c:technique_common/c:accessor", NS)
    if values_tag is None:
        raise ValueError(f"position source {source_id} has no float_array")
    stride = int(accessor.attrib.get("stride", "3")) if accessor is not None else 3
    values = [float(value) for value in values_tag.text.split()]
    return [tuple(values[i : i + 3]) for i in range(0, len(values), stride)]


def _identity():
    return (
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def _matmul(left, right):
    return tuple(
        tuple(
            sum(left[row][index] * right[index][column] for index in range(4))
            for column in range(4)
        )
        for row in range(4)
    )


def _matrix(values):
    if len(values) != 16:
        raise ValueError("COLLADA matrix must contain 16 values")
    # These Rhino-exported files store translation in entries 3, 7 and 11,
    # i.e. a row-major matrix multiplying a column vector.
    return tuple(tuple(values[row * 4 + column] for column in range(4)) for row in range(4))


def _translation(values):
    matrix = [list(row) for row in _identity()]
    matrix[0][3], matrix[1][3], matrix[2][3] = values
    return tuple(tuple(row) for row in matrix)


def _scale(values):
    matrix = [list(row) for row in _identity()]
    matrix[0][0], matrix[1][1], matrix[2][2] = values
    return tuple(tuple(row) for row in matrix)


def _rotation(values):
    x, y, z, degrees = values
    length = math.sqrt(x * x + y * y + z * z)
    if length <= 1e-15:
        return _identity()
    x, y, z = x / length, y / length, z / length
    angle = math.radians(degrees)
    cosine, sine = math.cos(angle), math.sin(angle)
    one_minus = 1.0 - cosine
    return (
        (cosine + x * x * one_minus, x * y * one_minus - z * sine, x * z * one_minus + y * sine, 0.0),
        (y * x * one_minus + z * sine, cosine + y * y * one_minus, y * z * one_minus - x * sine, 0.0),
        (z * x * one_minus - y * sine, z * y * one_minus + x * sine, cosine + z * z * one_minus, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def _node_transform(node):
    result = _identity()
    for child in node:
        tag = child.tag.rsplit("}", 1)[-1]
        if tag not in {"matrix", "translate", "rotate", "scale"}:
            continue
        values = [float(value) for value in child.text.split()]
        if tag == "matrix":
            transform = _matrix(values)
        elif tag == "translate":
            transform = _translation(values)
        elif tag == "rotate":
            transform = _rotation(values)
        else:
            transform = _scale(values)
        result = _matmul(result, transform)
    return result


def _transform_point(matrix, point, unit_scale):
    homogeneous = (point[0], point[1], point[2], 1.0)
    transformed = tuple(
        sum(matrix[row][index] * homogeneous[index] for index in range(4))
        for row in range(3)
    )
    return tuple(value * unit_scale for value in transformed)


def _geometry_triangles(geometry):
    result = []
    mesh = geometry.find("c:mesh", NS)
    if mesh is None:
        return result
    vertices_sources = {}
    for vertices in mesh.findall("c:vertices", NS):
        position = vertices.find("c:input[@semantic='POSITION']", NS)
        if position is not None:
            vertices_sources[vertices.attrib["id"]] = position.attrib["source"].lstrip("#")

    position_cache = {}
    for triangles in mesh.findall("c:triangles", NS):
        inputs = triangles.findall("c:input", NS)
        if not inputs:
            continue
        stride = max(int(item.attrib.get("offset", "0")) for item in inputs) + 1
        vertex_input = next(
            (item for item in inputs if item.attrib.get("semantic") == "VERTEX"),
            None,
        )
        if vertex_input is None:
            raise ValueError(f"geometry {geometry.attrib.get('id')}: no VERTEX input")
        vertex_offset = int(vertex_input.attrib.get("offset", "0"))
        vertices_id = vertex_input.attrib["source"].lstrip("#")
        source_id = vertices_sources[vertices_id]
        if source_id not in position_cache:
            position_cache[source_id] = _source_positions(mesh, source_id)
        positions = position_cache[source_id]
        indices = [int(value) for value in triangles.find("c:p", NS).text.split()]
        vertex_indices = indices[vertex_offset::stride]
        if len(vertex_indices) % 3:
            raise ValueError("triangle index count is not divisible by 3")
        for index in range(0, len(vertex_indices), 3):
            result.append(tuple(positions[value] for value in vertex_indices[index : index + 3]))
    return result


def collada_triangles(path):
    root = ET.parse(path).getroot()
    unit = root.find("c:asset/c:unit", NS)
    unit_scale = float(unit.attrib.get("meter", "1")) if unit is not None else 1.0
    geometries = {
        geometry.attrib["id"]: _geometry_triangles(geometry)
        for geometry in root.findall("c:library_geometries/c:geometry", NS)
    }
    nodes = {
        node.attrib["id"]: node
        for node in root.findall(".//c:library_nodes//c:node", NS)
        if "id" in node.attrib
    }
    visual_scene_ref = root.find("c:scene/c:instance_visual_scene", NS)
    if visual_scene_ref is None:
        raise ValueError(f"{path}: no active visual scene")
    visual_scene_id = visual_scene_ref.attrib["url"].lstrip("#")
    visual_scene = root.find(
        f"c:library_visual_scenes/c:visual_scene[@id='{visual_scene_id}']", NS
    )
    if visual_scene is None:
        raise ValueError(f"{path}: missing visual scene {visual_scene_id}")

    result = []
    instances = 0

    def visit(node, parent_transform, stack):
        nonlocal instances
        identifier = node.attrib.get("id", "<anonymous>")
        if identifier in stack:
            raise ValueError(f"{path}: recursive instance_node at {identifier}")
        transform = _matmul(parent_transform, _node_transform(node))
        for instance in node.findall("c:instance_geometry", NS):
            geometry_id = instance.attrib["url"].lstrip("#")
            if geometry_id not in geometries:
                raise ValueError(f"{path}: missing geometry {geometry_id}")
            instances += 1
            for triangle in geometries[geometry_id]:
                result.append(
                    tuple(
                        _transform_point(transform, point, unit_scale)
                        for point in triangle
                    )
                )
        next_stack = stack | {identifier}
        for child in node.findall("c:node", NS):
            visit(child, transform, next_stack)
        for instance in node.findall("c:instance_node", NS):
            node_id = instance.attrib["url"].lstrip("#")
            if node_id not in nodes:
                raise ValueError(f"{path}: missing library node {node_id}")
            visit(nodes[node_id], transform, next_stack)

    for node in visual_scene.findall("c:node", NS):
        visit(node, _identity(), set())
    if not result:
        raise ValueError(f"{path}: no triangles found")
    return result, instances


def _normal(triangle):
    a, b, c = triangle
    ab = tuple(b[i] - a[i] for i in range(3))
    ac = tuple(c[i] - a[i] for i in range(3))
    cross = (
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    )
    length = math.sqrt(sum(value * value for value in cross))
    if length <= 1e-15:
        return (0.0, 0.0, 0.0)
    return tuple(value / length for value in cross)


def write_binary_stl(path, triangles, source_name):
    header = f"Pro630 exact collision from {source_name}".encode("ascii")[:80]
    with path.open("wb") as stream:
        stream.write(header.ljust(80, b"\0"))
        stream.write(struct.pack("<I", len(triangles)))
        for triangle in triangles:
            stream.write(struct.pack("<3f", *_normal(triangle)))
            for vertex in triangle:
                stream.write(struct.pack("<3f", *vertex))
            stream.write(struct.pack("<H", 0))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("meshes", nargs="+", type=Path)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for source in args.meshes:
        triangles, instances = collada_triangles(source)
        destination = args.output_dir / f"{source.stem}_exact.stl"
        write_binary_stl(destination, triangles, source.name)
        bounds = tuple(
            (min(vertex[axis] for triangle in triangles for vertex in triangle),
             max(vertex[axis] for triangle in triangles for vertex in triangle))
            for axis in range(3)
        )
        formatted_bounds = " ".join(
            f"{axis}=[{lower:+.4f},{upper:+.4f}]"
            for axis, (lower, upper) in zip("xyz", bounds)
        )
        print(
            f"{source.name}: instances={instances} triangles={len(triangles)} "
            f"{formatted_bounds} -> {destination}"
        )


if __name__ == "__main__":
    main()
