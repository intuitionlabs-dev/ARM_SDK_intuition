"""协议角度与 SDK 单位换算：deg ↔ rad，速度和加速度采用相同比例。

夹爪原始角与开口距离的换算由 gripper.py 或型号配置提供。
"""

import math
from collections.abc import Iterable

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi


def deg_to_rad(deg):
    return deg * DEG2RAD


def rad_to_deg(rad):
    return rad * RAD2DEG


def deg_seq_to_rad(seq):
    """将角度序列从 deg 转为 rad，返回 tuple。"""
    return tuple(v * DEG2RAD for v in seq)


def rad_seq_to_deg(seq):
    return tuple(v * RAD2DEG for v in seq)


def deg_s_to_rad_s(v):
    return v * DEG2RAD


def rad_s_to_deg_s(v):
    return v * RAD2DEG


def deg_s_seq_to_rad_s(seq):
    return tuple(v * DEG2RAD for v in seq)


def rad_s_seq_to_deg_s(seq):
    return tuple(v * RAD2DEG for v in seq)


def check_joint_seq(values, what="关节角"):
    """检查六项数值并转为 tuple，不检查关节限位。"""
    if isinstance(values, (str, bytes)) or not isinstance(values, Iterable):
        raise TypeError("%s 要给 6 个数的序列，收到 %r" % (what, type(values).__name__))
    out = tuple(values)
    if len(out) != 6:
        raise ValueError("%s 要 6 个，收到 %d 个" % (what, len(out)))
    for v in out:
        if not isinstance(v, (int, float)):
            raise TypeError("%s 里混进了非数值：%r" % (what, v))
    return out


def fmt_deg(values):
    """将角度序列格式化为单行日志。"""
    return " ".join("%+7.2f" % v for v in values)
