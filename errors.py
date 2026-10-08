"""SDK 的异常层级。

单独一个文件，而不是散在各模块里：`protocol` / `link` / `arm` / `gripper` 互相 import
（`gripper` 要用 `units`，`arm` 要用 `link` 和 `gripper`），异常如果定义在 `arm` 里，
`link` 想抛同一个类型就得反过来 import `arm` —— 那就成环了。异常放最底下谁都 import 得到。

全部继承 `ArError`，所以 `except sdk.ArError` 能兜住 SDK 自己抛的一切；串口那条路
（pyserial 的 `SerialException`）原样透出来，不包 —— 那是环境问题，不是 SDK 的语义问题。
"""


class ArError(Exception):
    """SDK 所有异常的基类。"""


# ---- 链路 ----

class LinkError(ArError):
    """串口打不开 / 读写出错 / 连不上（等不到反馈帧）。"""


class NotConnected(LinkError):
    """链路还没 open（或已经 close）就要用它收发。"""


# ---- 协议 ----

class ProtocolError(ArError):
    """收到的帧不符合协议（长度不对、payload 解不出来）。"""


# ---- 闸门 ----

class GateRejected(ArError):
    """客户端预检不通过 —— **这一帧根本没发出去**。

    板子那边对所有命令帧都是**静默丢弃**（协议里没有 NACK），所以 SDK 在发送前先拿
    最近一帧反馈把板子的闸门条件复算一遍，对不上就在这里挡住并说明缺哪一条。这是
    "省钱"的：与其发出去等一个永远不会来的反馈帧，不如当场告诉你 state 是几、该先做什么。

    它**不等于板子会拒绝**，也不代表板上真的没受理 —— 见 README「注意事项」第 1 条：
    预检查不到"遥控器在线"这一条，`force=True` 是唯一的绕过办法。
    """


class FeedbackStale(ArError):
    """要拿状态做判断，但手上的反馈帧太旧（或一帧都还没收到）。

    这类异常只在**需要看状态的命令**上抛，读属性的路不抛（属性读不到就返回 None）。
    """


# ---- 夹爪 ----

class GripperNotCalibrated(ArError):
    """夹爪还没做两点标定，没法把原始电机角换算成开合距离（m）。"""


class CalibrationError(ArError):
    """标定数据本身不合法（FULL_OPEN ≤ 0、两个记录点重合、JSON 损坏……）。"""
