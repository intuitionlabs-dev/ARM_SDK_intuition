# SPDX-License-Identifier: LGPL-2.1-or-later
"""Small, explicit 3-D operations used by the static KDL/RNE extraction.

No NumPy, Eigen, ROS, KDL binary, or hardware SDK is needed at runtime.
Rotations map vectors from child coordinates to parent coordinates.
"""

from math import cos, isfinite, sin, sqrt

Vector = tuple[float, float, float]
Matrix = tuple[Vector, Vector, Vector]
ZERO: Vector = (0.0, 0.0, 0.0)
IDENTITY: Matrix = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def finite_vector(values, size=3):
    result = tuple(float(x) for x in values)
    if len(result) != size or not all(isfinite(x) for x in result):
        raise ValueError(f"expected {size} finite numbers")
    return result


def add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def scale(a, factor):
    return (factor * a[0], factor * a[1], factor * a[2])


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return (a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2],
            a[0]*b[1] - a[1]*b[0])


def transpose(rotation):
    a, b, c = rotation
    return ((a[0], b[0], c[0]), (a[1], b[1], c[1]), (a[2], b[2], c[2]))


def rotate(rotation, vector):
    return (dot(rotation[0], vector), dot(rotation[1], vector), dot(rotation[2], vector))


def multiply(a, b):
    columns = transpose(b)
    return (rotate(columns, a[0]), rotate(columns, a[1]), rotate(columns, a[2]))


def normalize(axis):
    axis = finite_vector(axis)
    length = sqrt(dot(axis, axis))
    if not isfinite(length) or length == 0:
        raise ValueError("joint axis must have a finite nonzero norm")
    return scale(axis, 1.0 / length)


def axis_rotation(axis, angle):
    """Rodrigues formula; axis must already have unit length."""
    x, y, z = axis
    c, s = cos(angle), sin(angle)
    d = 1.0 - c
    return ((c+x*x*d, x*y*d-z*s, x*z*d+y*s),
            (y*x*d+z*s, c+y*y*d, y*z*d-x*s),
            (z*x*d-y*s, z*y*d+x*s, c+z*z*d))


def rpy_rotation(rpy):
    """URDF fixed-axis RPY: Rz(yaw) * Ry(pitch) * Rx(roll)."""
    roll, pitch, yaw = rpy
    return multiply(multiply(axis_rotation((0, 0, 1), yaw),
                             axis_rotation((0, 1, 0), pitch)),
                    axis_rotation((1, 0, 0), roll))
