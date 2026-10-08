"""上位机 Python SDK —— 通过串口 / USB 控制 CtrBoard-H7 上的 ARX X5 机械臂。

面向外部调用者的入口就两个类：`Link`（一条链路）和 `Arm`（一台臂）。其余模块按功能分开：

    errors.py        异常层级
    protocol.py      线上协议：CRC、码表、组帧 / 切帧、payload 编解码（**deg 为单位**）
    board_defs.py    板侧常量镜像：软限位、复位位姿、状态 / 模式名字表
    units.py         deg↔rad 换算策略（SDK 对外用 SI）
    link.py          串口 + 后台收包线程 + 按 arm_id 分流 + 口子挑选
    arm.py           `Arm`：全部帧接口 + 闸门预检 + 单位换算（**rad / m**）
    gripper.py       夹爪两点标定：m ↔ 原始 deg 的映射 + JSON 存取
    selftest.py      离线自测：`python3 -m sdk.selftest`（不接硬件）
    calibrate_gripper.py  夹爪标定小工具
    tui/             上机控制台（curses 全屏面板）：`python3 -m sdk.tui`，不接硬件用 `--fake`
                     （`import sdk` 不会把它拉进来 —— curses 只在真要开屏时才 import）
    README.md        **手册：会话流程、每个接口、注意事项 —— 先看它**

最小用法（单台臂，角度是 rad、夹爪是 m）：

    import sdk, time

    arm = sdk.Arm(0)
    arm.connect()                       # 开串口 → 发版本包 → 等第一帧反馈

    def wait_state(want, timeout=10.0):
        # 等到 state == want。**等的期间要一直保活** —— 板子 500 ms 收不到任何有效帧
        # 就切阻尼、退回「离线等待」，它自己发的反馈帧不算数。
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            arm.keepalive()
            if arm.wait_for(lambda a: a.robot_state == want, timeout=0.05):
                return True
        return False

    wait_state(2)                       # 等「待初始化」
    arm.init_arm()                      # 一键复位 + 夹爪找零（先确认臂在复位位姿附近）
    wait_state(4)                       # 等「正常运行」（落点是阻尼档）
    arm.start()                         # 进 run 档，才能发控制帧
    arm.wait_for(lambda a: a.mode == sdk.board_defs.MODE_RUN)   # 等反馈确认切档，
                                        # 否则紧接着发的第一帧控制帧会被闸门挡下
    while True:
        arm.send_ctrl_pos(arm.joint_angles, gripper_m=0.0)   # 照抄现在的位置、夹爪闭合
        time.sleep(0.01)                # 这一帧本身就是保活；停发就要自己 arm.keepalive()

两台臂（**一条链路、两个 Arm** —— 板子只有一条上位机链路）：

    link = sdk.Link(); link.open()
    a, b = sdk.Arm(0, link), sdk.Arm(1, link)
    a.connect(); b.connect()

`import sdk` 不需要 pyserial（只有 `Link.open()` 才 import 它），所以协议层和离线自测
在没有 pyserial 的机器上也能跑。
"""

from . import board_defs, gripper, protocol, units
from .arm import Arm, Snapshot
from .errors import (ArError, CalibrationError, FeedbackStale, GateRejected,
                     GripperNotCalibrated, LinkError, NotConnected, ProtocolError)
from .gripper import GripperCalibration, GripperCalibStore
from .link import Link, describe_port, list_links, pick_default_port, usb_identity
from .protocol import Feedback, SetZeroAck

# 协议版本串（板子 `libs/frame/frame_config.h`）。版本码是它的 FNV-1a 32，
# 见 `protocol.VERSION_CODE` —— 两边必须一致，不一致板子会锁死。
PROTOCOL_VERSION = protocol.VERSION_STRING
VERSION_CODE = protocol.VERSION_CODE

__all__ = [
    # 入口
    "Arm", "Link", "Snapshot",
    # 数据
    "Feedback", "SetZeroAck", "GripperCalibration", "GripperCalibStore",
    # 异常
    "ArError", "LinkError", "NotConnected", "ProtocolError", "GateRejected",
    "FeedbackStale", "GripperNotCalibrated", "CalibrationError",
    # 口子
    "pick_default_port", "list_links", "describe_port", "usb_identity",
    # 常量
    "PROTOCOL_VERSION", "VERSION_CODE", "protocol", "board_defs", "units", "gripper",
]
