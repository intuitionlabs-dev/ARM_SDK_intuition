# SPDX-License-Identifier: LGPL-2.1-or-later
# Copyright (C) 2009 Ruben Smits and Dominick Vanthienen.
"""基于 Orocos KDL RNE 的 X5 六轴逆动力学。"""

from enum import IntEnum
from math import isfinite

from .solver import GravityCompensator, SDK_SCALES
from .spatial import ZERO, add, cross, dot, finite_vector, rotate, scale, transpose


class DynamicsStatus(IntEnum):
    OK = 0
    INVALID_ARGUMENT = -1
    ANGLE_RANGE = -2
    NONFINITE_RESULT = -3


class DynamicsError(ValueError):
    def __init__(self, status):
        self.status = DynamicsStatus(status)
        super().__init__(self.status.name)


class DynamicsCompensator(GravityCompensator):
    def torques(self, q, qd, qdd, *, sdk_scale=True):
        """返回 M(q)qdd+C(q,qd)qd+G(q)，单位为 N·m。"""
        try:
            q, qd, qdd = (finite_vector(v, 6) for v in (q, qd, qdd))
        except (ValueError, TypeError, OverflowError) as error:
            raise DynamicsError(DynamicsStatus.INVALID_ARGUMENT) from error
        if len(self.segments) != 6 or any(s.joint_type != "revolute" for s in self.segments):
            raise DynamicsError(DynamicsStatus.INVALID_ARGUMENT)
        if any(abs(angle) > 128 for angle in q):
            raise DynamicsError(DynamicsStatus.ANGLE_RANGE)
        poses = self._poses(q)
        forces, moments = [], []
        linear_velocity = angular_velocity = angular_acceleration = ZERO
        linear_acceleration = scale(self.gravity, -1.0)
        for segment, (rotation, offset), speed, acc in zip(self.segments, poses, qd, qdd):
            inverse = transpose(rotation)
            linear_velocity = rotate(inverse, add(linear_velocity, cross(angular_velocity, offset)))
            angular_velocity = rotate(inverse, angular_velocity)
            linear_acceleration = rotate(inverse, add(linear_acceleration,
                                                      cross(angular_acceleration, offset)))
            angular_acceleration = rotate(inverse, angular_acceleration)
            joint_speed = scale(segment.axis, speed)
            angular_velocity = add(angular_velocity, joint_speed)
            linear_acceleration = add(linear_acceleration, cross(linear_velocity, joint_speed))
            angular_acceleration = add(angular_acceleration,
                                       add(scale(segment.axis, acc), cross(angular_velocity, joint_speed)))
            force = scale(add(linear_acceleration, cross(angular_acceleration, segment.com)), segment.mass)
            moment = add(rotate(segment.inertia, angular_acceleration), cross(segment.com, force))
            momentum = scale(add(linear_velocity, cross(angular_velocity, segment.com)), segment.mass)
            angular_momentum = add(rotate(segment.inertia, angular_velocity), cross(segment.com, momentum))
            forces.append(add(force, cross(angular_velocity, momentum)))
            moments.append(add(moment, add(cross(angular_velocity, angular_momentum),
                                           cross(linear_velocity, momentum))))
        result = [0.0] * 6
        for i in range(5, -1, -1):
            result[i] = dot(self.segments[i].axis, moments[i])
            if i:
                rotation, offset = poses[i]
                parent_force = rotate(rotation, forces[i])
                forces[i-1] = add(forces[i-1], parent_force)
                moments[i-1] = add(moments[i-1], add(rotate(rotation, moments[i]), cross(offset, parent_force)))
        if sdk_scale:
            result = [t + (s - 1.0) * g for t, s, g in
                      zip(result, SDK_SCALES, self.raw_torques(q))]
        if not all(isfinite(t) for t in result):
            raise DynamicsError(DynamicsStatus.NONFINITE_RESULT)
        return tuple(result)
