"""ARX X5 limits and reset pose, matching the H7 firmware."""

import math

from ..protocol import fnv1a32
from .common import *  # Shared protocol names are also part of each robot profile.

# 固件 `libs/frame/frame_config.h` 的 FRAME_VERSION_STRING；版本码是其 FNV-1a 32。
VERSION_STRING = "Intuition-ARX-X5-2023-v0.3"
VERSION_CODE = fnv1a32(VERSION_STRING)

JOINT_ANGLE_MIN = (-115.0, 0.5, 0.5, -90.0, -90.0, -112.87)
JOINT_ANGLE_MAX = (90.0, 206.26, 171.88, 90.0, 90.0, 119.17)
GRIPPER_ANGLE_MIN = -142.0
GRIPPER_ANGLE_MAX = 142.0
GRIPPER_STROKE_DEG = GRIPPER_ANGLE_MAX - GRIPPER_ANGLE_MIN
AXIS_MIN = JOINT_ANGLE_MIN + (GRIPPER_ANGLE_MIN,)
AXIS_MAX = JOINT_ANGLE_MAX + (GRIPPER_ANGLE_MAX,)
RESET_POSE_DEG = (0.0, 0.5, 0.5, 0.0, 0.0, 0.0)


def axis_lo(axis):
    return AXIS_MIN[axis]


def axis_hi(axis):
    return AXIS_MAX[axis]


def clamp_axis(axis, value):
    """把一根轴的值夹进软限位，返回 `(夹住之后的值, 是否发生了夹)`。

    **SDK 不自动调它** —— 命令方法发出去的就是你给的值。理由：板子是静默夹的，上位机再
    悄悄夹一次，你看到的期望角和电机实际收到的角会不一致，排查时很难发现。要夹就自己夹，
    而且**最好自己知道夹了**（第二个返回值就是干这个的）。
    """
    lo, hi = axis_lo(axis), axis_hi(axis)
    if value < lo:
        return lo, True
    if value > hi:
        return hi, True
    return value, False


def pose_deviation_deg(joint_deg6):
    """Maximum joint deviation from this robot's reset pose, in degrees."""
    return max(abs(a - b) for a, b in zip(joint_deg6, RESET_POSE_DEG))


JOINT_ANGLE_MIN_RAD = tuple(math.radians(v) for v in JOINT_ANGLE_MIN)
JOINT_ANGLE_MAX_RAD = tuple(math.radians(v) for v in JOINT_ANGLE_MAX)
GRIPPER_ANGLE_MIN_RAD = math.radians(GRIPPER_ANGLE_MIN)
GRIPPER_ANGLE_MAX_RAD = math.radians(GRIPPER_ANGLE_MAX)
RESET_POSE_RAD = tuple(math.radians(v) for v in RESET_POSE_DEG)
