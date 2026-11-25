"""Pre-Gazebo floor check using the expanded public URDF collision geometry.

For the z=0 ground half-space, minimum mesh vertex height is exact: every
triangle is a convex combination of its vertices. Primitives use analytic
support bounds. No visual meshes, new dimensions or SDK calls are involved.
"""
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET


IDENTITY = (((1., 0., 0.), (0., 1., 0.), (0., 0., 1.)), (0., 0., 0.))


def vector(text, default='0 0 0'):
    values = tuple(float(v) for v in (text or default).split())
    if len(values) != 3 or not all(math.isfinite(v) for v in values):
        raise ValueError('Invalid URDF vector')
    return values


def apply(rotation, point):
    return tuple(sum(a * b for a, b in zip(row, point)) for row in rotation)


def compose(first, second):
    a, x = first
    b, y = second
    rotation = tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3))
                           for j in range(3)) for i in range(3))
    shifted = apply(a, y)
    return rotation, tuple(x[i] + shifted[i] for i in range(3))


def rotation(axis, angle):
    norm = math.sqrt(sum(v * v for v in axis))
    if norm == 0:
        raise ValueError('Zero URDF joint axis')
    x, y, z = (v / norm for v in axis)
    c, s, t = math.cos(angle), math.sin(angle), 1. - math.cos(angle)
    return ((t*x*x+c, t*x*y-s*z, t*x*z+s*y),
            (t*x*y+s*z, t*y*y+c, t*y*z-s*x),
            (t*x*z-s*y, t*y*z+s*x, t*z*z+c))


def origin(element):
    tag = element.find('origin')
    if tag is None:
        return IDENTITY
    roll, pitch, yaw = vector(tag.get('rpy'))
    result = (rotation((0., 0., 1.), yaw), vector(tag.get('xyz')))
    result = compose(result, (rotation((0., 1., 0.), pitch), (0., 0., 0.)))
    return compose(result, (rotation((1., 0., 0.), roll), (0., 0., 0.)))


def stl_vertices(path):
    data = Path(path).read_bytes()
    if len(data) >= 84 and len(data) == 84 + 50 * struct.unpack_from('<I', data, 80)[0]:
        points = []
        for offset in range(84, len(data), 50):
            coordinates = struct.unpack_from('<9f', data, offset + 12)
            points.extend(tuple(coordinates[i:i+3]) for i in (0, 3, 6))
    else:
        points = [vector(' '.join(parts[1:])) for line in data.decode('ascii').splitlines()
                  if (parts := line.split()) and parts[0] == 'vertex']
    if not points or len(points) % 3 or not all(math.isfinite(v) for point in points for v in point):
        raise ValueError(f'Invalid or empty STL collision mesh: {path}')
    return points


class StartupGroundCheck:
    def __init__(self, urdf_xml, resolve_mesh):
        root = ET.fromstring(urdf_xml)
        self.links = {link.get('name'): link for link in root.findall('link')}
        self.joints = {joint.find('child').get('link'): joint for joint in root.findall('joint')}
        if 'world' not in self.links or 'world' in self.joints:
            raise ValueError('Startup floor check requires the public world-rooted URDF')
        self.geometry = []
        mesh_cache = {}
        for name, link in self.links.items():
            # Same sole base/ground exemption as the existing MoveIt scene.
            if name == 'base':
                continue
            for collision in link.findall('collision'):
                geometry = collision.find('geometry')
                if geometry is None or len(geometry) != 1:
                    raise ValueError(f'Invalid collision geometry on {name}')
                shape = geometry[0]
                if shape.tag == 'mesh':
                    filename = shape.get('filename')
                    if filename not in mesh_cache:
                        mesh_cache[filename] = stl_vertices(resolve_mesh(filename))
                    scale = vector(shape.get('scale'), '1 1 1')
                    if any(v == 0 for v in scale):
                        raise ValueError(f'Zero collision mesh scale on {name}')
                    payload = [tuple(point[i] * scale[i] for i in range(3))
                               for point in mesh_cache[filename]]
                elif shape.tag == 'box':
                    payload = vector(shape.get('size'))
                elif shape.tag == 'cylinder':
                    payload = (float(shape.get('radius')), float(shape.get('length')))
                elif shape.tag == 'sphere':
                    payload = (float(shape.get('radius')),)
                else:
                    raise ValueError(f'Unsupported collision geometry: {shape.tag}')
                if shape.tag != 'mesh' and not all(math.isfinite(v) and v > 0 for v in payload):
                    raise ValueError(f'Invalid collision dimensions on {name}')
                self.geometry.append((name, origin(collision), shape.tag, payload))
        if not self.geometry:
            raise ValueError('No non-base collision geometry available for startup floor check')

    def check(self, positions, allow_collision=False):
        transforms, joint_values = {'world': IDENTITY}, {}

        def value(joint, visiting):
            name = joint.get('name')
            if name in joint_values:
                return joint_values[name]
            if name in visiting:
                raise ValueError('Cyclic URDF mimic joints')
            mimic = joint.find('mimic')
            if mimic is None:
                result = float(positions[name])
            else:
                source = next(j for j in self.joints.values() if j.get('name') == mimic.get('joint'))
                result = value(source, visiting | {name}) * float(mimic.get('multiplier', '1')) + float(mimic.get('offset', '0'))
            if not math.isfinite(result):
                raise ValueError('Non-finite startup joint value')
            joint_values[name] = result
            return result

        def transform(name, visiting):
            if name in transforms:
                return transforms[name]
            if name in visiting or name not in self.joints:
                raise ValueError(f'Invalid URDF chain to {name}')
            joint = self.joints[name]
            parent = transform(joint.find('parent').get('link'), visiting | {name})
            pose = compose(parent, origin(joint))
            kind = joint.get('type')
            if kind != 'fixed':
                axis_tag = joint.find('axis')
                axis = vector(axis_tag.get('xyz') if axis_tag is not None else None, '1 0 0')
                angle = value(joint, set())
                if kind in ('revolute', 'continuous'):
                    pose = compose(pose, (rotation(axis, angle), (0., 0., 0.)))
                elif kind == 'prismatic':
                    pose = compose(pose, (IDENTITY[0], tuple(v * angle for v in axis)))
                else:
                    raise ValueError(f'Unsupported URDF joint type: {kind}')
            transforms[name] = pose
            return pose

        minimum = (float('inf'), '')
        for name, local, kind, payload in self.geometry:
            orientation, translation = compose(transform(name, set()), local)
            normal = orientation[2]
            if kind == 'mesh':
                height = translation[2] + min(sum(a*b for a, b in zip(normal, point)) for point in payload)
            elif kind == 'box':
                height = translation[2] - sum(abs(a) * size / 2. for a, size in zip(normal, payload))
            elif kind == 'cylinder':
                radius, length = payload
                height = translation[2] - abs(normal[2]) * length / 2. - radius * math.hypot(*normal[:2])
            else:
                height = translation[2] - payload[0]
            if not math.isfinite(height):
                raise ValueError(f'Non-finite collision height on {name}')
            minimum = min(minimum, (height, name))
        height, name = minimum
        if height < -1e-9 and not allow_collision:
            raise RuntimeError(
                'Startup rejected: the robot pose is below the base ground plane. '
                'Move it above the base ground plane before restarting. '
                f'(link={name}, minimum Z={height * 1000.:.3f} mm)')
        return minimum
