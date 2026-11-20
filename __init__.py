"""H7 机械臂 SDK，提供 Link 链路和 Arm 控制接口。

支持 ARX X5 和 Metal Arm。关节使用 rad、rad/s 和 rad/s²，夹爪开口使用 m。
双臂共用一个 Link，通过 arm_id 区分；调用方须为每台臂持续发送命令或保活。

使用说明见 README.md 和 API.md。终端入口：python3 -m sdk.tui。
离线自测：python3 -m sdk.selftest。只有打开实机串口时才需要 pyserial。
"""

from . import board_defs, gripper, protocol, units
from .arm import Arm, Snapshot
from .errors import (ArError, CalibrationError, FeedbackStale, GateRejected,
                     GripperNotCalibrated, LinkError, NotConnected, ProtocolError)
from .gripper import GripperCalibration, GripperCalibStore
from .link import (Link, describe_port, list_links, match_ident, pick_default_port,
                   same_usb_ident, usb_identity)
from .protocol import Feedback, LinkStatus, SetZeroAck

__all__ = [
    # 入口
    "Arm", "Link", "Snapshot",
    # 数据
    "Feedback", "SetZeroAck", "LinkStatus", "GripperCalibration", "GripperCalibStore",
    # 异常
    "ArError", "LinkError", "NotConnected", "ProtocolError", "GateRejected",
    "FeedbackStale", "GripperNotCalibrated", "CalibrationError",
    # 串口工具
    "pick_default_port", "list_links", "describe_port", "usb_identity",
    "same_usb_ident", "match_ident",
    # 子模块
    "protocol", "board_defs", "units", "gripper",
]
