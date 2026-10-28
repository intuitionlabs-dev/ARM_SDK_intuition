"""Metal Arm limits from metal-python-ros bccf3c8; user-confirmed reset pose."""

import math
from bisect import bisect_left

from ..errors import CalibrationError
from ..protocol import fnv1a32
from .common import *  # Shared protocol names are also part of each robot profile.

# 固件 `libs/frame/frame_config.h` 的 FRAME_VERSION_STRING；版本码是其 FNV-1a 32。
VERSION_STRING = "Intuition-Metal-Arm-v0.4"
VERSION_CODE = fnv1a32(VERSION_STRING)

JOINT_ANGLE_MIN = (-160.0, -180.0, 0.0, -123.0, -85.0, -145.0)
JOINT_ANGLE_MAX = (160.0, 4.8, 180.0, 81.0, 85.0, 145.0)
GRIPPER_ANGLE_MIN = 0.0
GRIPPER_ANGLE_MAX = math.degrees(2.4)
GRIPPER_STROKE_DEG = GRIPPER_ANGLE_MAX - GRIPPER_ANGLE_MIN
AXIS_MIN = JOINT_ANGLE_MIN + (GRIPPER_ANGLE_MIN,)
AXIS_MAX = JOINT_ANGLE_MAX + (GRIPPER_ANGLE_MAX,)

# User confirmed the J1-J6 reset reference as all zero on 2026-10-27.
# These are joint target angles; resetting does not rewrite motor zero points.
RESET_POSE_DEG = (0.0,) * 6


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


# ==================== 夹爪换算：厂家查表 ====================
#
# 出厂零点下"电机角 ↔ 开口距离"的查表，取自厂家 C++ 的 distanceToAngle / angleToDistance。
# 开口范围 0..0.1 m，电机角范围 0..2.4 rad（GRIPPER_ANGLE_*）；表外按最近邻取端点。
# 该表对应未改动的出厂零点；找零/设零改了夹爪零点后不适用，此时显式传 calib_store。

GRIPPER_DISTANCES_MM = (
    0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0,
    15.0, 16.0, 17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0,
    29.0, 30.0, 31.0, 32.0, 33.0, 34.0, 35.0, 36.0, 37.0, 38.0, 39.0, 40.0, 41.0, 42.0,
    43.0, 44.0, 45.0, 46.0, 47.0, 48.0, 49.0, 50.0, 51.0, 52.0, 53.0, 54.0, 55.0, 56.0,
    57.0, 58.0, 59.0, 60.0, 61.0, 62.0, 63.0, 64.0, 65.0, 66.0, 67.0, 68.0, 69.0, 70.0,
    71.0, 72.0, 73.0, 74.0, 75.0, 76.0, 77.0, 78.0, 79.0, 80.0, 81.0, 82.0, 83.0, 84.0,
    85.0, 86.0, 87.0, 88.0, 89.0, 90.0, 91.0, 92.0, 93.0, 94.0, 95.0, 96.0, 97.0, 98.0,
    99.0, 100.0, 102.5,
)
GRIPPER_ANGLES_RAD = (
    0.002, 0.01407, 0.0368934, 0.08634, 0.115854, 0.161084, 0.176609, 0.194816, 0.22893,
    0.262277, 0.295, 0.314215, 0.344305, 0.362704, 0.393177, 0.411575, 0.441473, 0.46773,
    0.482871, 0.508743, 0.527718, 0.550333, 0.576397, 0.60227, 0.621053, 0.639452, 0.662833,
    0.683915, 0.707105, 0.722054, 0.748693, 0.763451, 0.782233, 0.805039, 0.815964, 0.841837,
    0.856786, 0.879593, 0.894158, 0.91639, 0.92789, 0.94648, 0.969287, 0.987494, 0.998418,
    1.02084, 1.03617, 1.05112, 1.0701, 1.08505, 1.10383, 1.11897, 1.13411, 1.15557, 1.16382,
    1.18221, 1.20117, 1.21594, 1.23779, 1.24986, 1.26424, 1.28264, 1.29452, 1.31273, 1.3317,
    1.34359, 1.36505, 1.38019, 1.39169, 1.40683, 1.42542, 1.44401, 1.45609, 1.47468, 1.48943,
    1.50419, 1.52259, 1.53735, 1.55575, 1.57453, 1.582, 1.60462, 1.62302, 1.64218, 1.65694,
    1.66825, 1.68722, 1.70984, 1.729, 1.73973, 1.76216, 1.78477, 1.79991, 1.82214, 1.84399,
    1.86297, 1.88577, 1.9036, 1.92717, 1.94844, 1.97527, 2.03143,
)


class FactoryGripperCalibration:
    """厂家查表换算：`opening_from_raw_deg()` / `raw_deg_from_opening()`，单位 m / deg。"""

    @staticmethod
    def _nearest(value, source, target):
        if not math.isfinite(value):
            raise CalibrationError("gripper value must be finite")
        i = bisect_left(source, value)
        if i == 0:
            return target[0]
        if i == len(source):
            return target[-1]
        return target[i - 1] if value - source[i - 1] < source[i] - value else target[i]

    def opening_from_raw_deg(self, value):
        mm = self._nearest(math.radians(value), GRIPPER_ANGLES_RAD, GRIPPER_DISTANCES_MM)
        return min(100.0, mm) / 1000.0

    def raw_deg_from_opening(self, value):
        if not math.isfinite(value) or not 0.0 <= value <= 0.1:
            raise CalibrationError("Metal factory gripper opening must be 0..0.1 m")
        return math.degrees(self._nearest(value * 1000.0, GRIPPER_DISTANCES_MM, GRIPPER_ANGLES_RAD))


class FactoryGripperStore:
    """厂家查表存储：每台臂都返回同一份表，不做按臂标定。"""

    def get(self, arm_id):
        return FactoryGripperCalibration()

    def has(self, arm_id):
        return True


def default_gripper_store():
    """Metal 链路的默认夹爪换算：厂家查表。"""
    return FactoryGripperStore()
