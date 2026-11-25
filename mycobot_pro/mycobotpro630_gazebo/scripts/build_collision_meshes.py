#!/usr/bin/env python3
"""Rebuild exact MoveIt and stable Gazebo geometry from the same DAE scenes.

Offline maintenance utility; numpy/scipy are needed only for regeneration.
The active visual-scene transforms and COLLADA metre scale are baked into
both outputs. MoveIt uses all source triangles. Gazebo uses enclosing convex
arm hulls and PCA gripper boxes to keep ODE stable. Neither output changes
the semantic allowed-collision matrix or adds a second motion check.
"""
import argparse
from pro630_configuration import geometry_xml
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull, HalfspaceIntersection
from export_exact_collision_meshes import collada_triangles, write_binary_stl


def enclosing_mesh(points, gripper=False, padding=.0005, directions=512, flat_base=False):
    cloud = np.unique(np.asarray(points, dtype=float), axis=0)
    center = cloud.mean(axis=0)
    if gripper:
        _, axes = np.linalg.eigh(np.cov(cloud - center, rowvar=False))
        projected = (cloud - center) @ axes
        lower, upper = projected.min(axis=0) - padding, projected.max(axis=0) + padding
        corners = np.array([(x, y, z) for x in (lower[0], upper[0])
                            for y in (lower[1], upper[1]) for z in (lower[2], upper[2])])
        vertices = center + corners @ axes.T
    else:
        # Support planes enclose every exact visual vertex. Include principal
        # axes to preserve the thin ends of long arm links without shrinkage.
        i = np.arange(directions, dtype=float)
        z = 1. - 2. * (i + .5) / directions
        theta = i * np.pi * (3. - np.sqrt(5.))
        radius = np.sqrt(1. - z*z)
        normals = np.vstack((np.column_stack((radius*np.cos(theta), radius*np.sin(theta), z)),
                             np.eye(3), -np.eye(3)))
        supports = np.array([np.max(cloud @ normal) + padding for normal in normals])
        if flat_base:
            # The mounting base sits exactly at z=0. Expanding its bottom
            # below the floor creates a permanent ODE penetration; preserve
            # the CAD mounting plane while padding the other directions.
            supports[-1] = -cloud[:, 2].min()
        vertices = HalfspaceIntersection(np.column_stack((normals, -supports)), center).intersections
    vertices = np.unique(np.round(vertices, 7), axis=0)
    hull = ConvexHull(vertices)
    triangles = []
    for face, plane in zip(hull.simplices, hull.equations):
        a, b, c = vertices[face]
        if np.dot(np.cross(b-a, c-a), plane[:3]) < 0:
            b, c = c, b
        triangles.append(tuple(tuple(float(v) for v in p) for p in (a, b, c)))
    # Assert coverage against the exported triangles, not only construction
    # planes. The allowance covers float32 STL and the 0.1-micrometre rounding.
    excess = max(float(np.max(cloud @ plane[:3] + plane[3])) for plane in hull.equations)
    if excess > 2e-7:
        raise ValueError(f'Generated hull does not enclose source geometry: {excess}')
    return triangles, excess


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('urdf', type=Path)
    parser.add_argument('--without-gripper', action='store_true')
    parser.add_argument('package_root', type=Path)
    args = parser.parse_args()
    root = geometry_xml(args.urdf, not args.without_gripper)
    output = args.package_root / 'meshes'
    for directory in ('collision_exact', 'collision_gazebo'):
        (output / directory).mkdir(exist_ok=True)
    for link in root.findall('link'):
        mesh = link.find('visual/geometry/mesh')
        if mesh is None:
            continue
        prefix = 'package://mycobotpro630_gazebo/'
        if not mesh.get('filename').startswith(prefix):
            raise ValueError('Unsupported mesh package')
        source = args.package_root / mesh.get('filename')[len(prefix):]
        exact, instances = collada_triangles(source)
        name = source.stem + '_exact.stl'
        write_binary_stl(output / 'collision_exact' / name, exact, source.name)
        points = [v for triangle in exact for v in triangle]
        coarse, excess = enclosing_mesh(points, gripper=link.get('name').startswith('gripper'),
                                       flat_base=link.get('name') == 'base')
        write_binary_stl(output / 'collision_gazebo' / name, coarse, source.name)
        print(f'{link.get("name")}: instances={instances}, exact={len(exact)}, '
              f'gazebo={len(coarse)}, enclosure_error={excess:.3e}', flush=True)


if __name__ == '__main__':
    main()
