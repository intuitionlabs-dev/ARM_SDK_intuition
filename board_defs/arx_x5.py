"""ARX X5 限位与复位参考，与 H7 固件对应。"""

import math

from ..protocol import fnv1a32
from .common import *  # 各型号同时导出公共协议常量。

# 版本串与固件 App/profiles/arx_x5.c 一致；版本码为 FNV-1a 32 位哈希。
VERSION_STRING = "Intuition-ARX-X5-2023-v0.6"
VERSION_CODE = fnv1a32(VERSION_STRING)

JOINT_ANGLE_MIN = (-115.0, 0.5, 0.5, -90.0, -90.0, -112.87)
JOINT_ANGLE_MAX = (90.0, 206.26, 171.88, 90.0, 90.0, 119.17)
# 原始角负向闭合、正向张开；软限位不替代实测开口标定。
GRIPPER_ANGLE_MIN = -157.0
GRIPPER_ANGLE_MAX = 142.0
GRIPPER_TORQUE_MAX_NM = 10.0
DEFAULT_GRIPPER_TORQUE_LIMIT_NM = 0.5
GRIPPER_STROKE_DEG = GRIPPER_ANGLE_MAX - GRIPPER_ANGLE_MIN
AXIS_MIN = JOINT_ANGLE_MIN + (GRIPPER_ANGLE_MIN,)
AXIS_MAX = JOINT_ANGLE_MAX + (GRIPPER_ANGLE_MAX,)
RESET_POSE_DEG = (0.0, 0.5, 0.5, 0.0, 0.0, 0.0)


def axis_lo(axis):
    return AXIS_MIN[axis]


def axis_hi(axis):
    return AXIS_MAX[axis]


def clamp_axis(axis, value):
    """按软限位限制单轴目标，返回 (目标值, 是否限幅)。SDK 命令不会自动调用此函数。"""
    lo, hi = axis_lo(axis), axis_hi(axis)
    if value < lo:
        return lo, True
    if value > hi:
        return hi, True
    return value, False


def pose_deviation_deg(joint_deg6):
    """六轴相对复位参考的最大偏差，单位 deg。"""
    return max(abs(a - b) for a, b in zip(joint_deg6, RESET_POSE_DEG))


JOINT_ANGLE_MIN_RAD = tuple(math.radians(v) for v in JOINT_ANGLE_MIN)
JOINT_ANGLE_MAX_RAD = tuple(math.radians(v) for v in JOINT_ANGLE_MAX)
GRIPPER_ANGLE_MIN_RAD = math.radians(GRIPPER_ANGLE_MIN)
GRIPPER_ANGLE_MAX_RAD = math.radians(GRIPPER_ANGLE_MAX)
RESET_POSE_RAD = tuple(math.radians(v) for v in RESET_POSE_DEG)
