"""量纲换算策略 —— **SDK 的对外单位**和线上单位之间的那一层。

对外（`arm.py` 的公共属性 / 参数）：
    关节角        rad
    关节角速度    rad/s
    夹爪开合      **m**（要过标定，见 `gripper.py`）
    温度          ℃

线上（`protocol.py`）：关节角 deg、角速度 deg/s、夹爪角 deg、温度 ℃。

为什么分两层而不是"协议层直接出 rad"：协议层要能脱离单位策略单独测（自测里就是拿板上
`FrameEncode()` 的真实字节对拍的），而那些字节天生就是 deg。换算放在这一层，改口径不用动
编解码。

**夹爪不在这里换算** —— 它是"电机原始角 ↔ 开口距离"，方向、偏移、尺度三个都是标定量，
属于 `gripper.py`。这一层只管 deg↔rad 这种纯比例。
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
    """一串 deg → 一串 rad（返回 tuple，保证不可变）。"""
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
    """长度校验，顺便把入参收成 tuple。

    只查个数，**不查限位** —— 限位是策略（见 `board_defs.clamp_axis`），这里不做。
    """
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
    """给日志用的一行 deg 列表。"""
    return " ".join("%+7.2f" % v for v in values)
