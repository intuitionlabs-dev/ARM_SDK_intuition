"""SDK 异常类型，统一继承 ArError。参数校验和等待超时也可能使用标准库异常。"""


class ArError(Exception):
    """SDK 异常基类。"""


# ---- 链路 ----

class LinkError(ArError):
    """串口连接、读写或等待反馈失败。"""


class NotConnected(LinkError):
    """链路未打开或已断开。"""


# ---- 协议 ----

class ProtocolError(ArError):
    """协议数据异常。"""


# ---- 闸门 ----

class GateRejected(ArError):
    """上位机预检失败，命令未发送；异常消息说明未满足的条件。"""


class FeedbackStale(ArError):
    """反馈缺失或过期，无法执行命令预检。"""


# ---- 夹爪 ----

class GripperNotCalibrated(ArError):
    """缺少夹爪换算，无法将原始角换算为开口距离。"""


class CalibrationError(ArError):
    """夹爪标定数据或文件无效。"""
