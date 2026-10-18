"""控制核心、CAN 适配器与应用程序之间的接口。"""

from typing import Iterable, Protocol

from .types import ArmFeedback, Command, Event, MotorFeedback, MotorRequest


class MotorIO(Protocol):
    """已解码的电机接口，由控制循环的所属线程依次调用。

    两个方法都必须非阻塞。反馈时间戳与 now_ms 使用同一时钟，保留采样时刻，
    不得给旧反馈刷新时间戳。电机请求必须按顺序交付，遵守停机及设零保护间隔；
    输出失败必须抛出异常，不能静默丢弃请求。
    """

    def poll_feedback(self, now_ms: int) -> Iterable[MotorFeedback]: ...
    def write_motor_requests(self, requests: tuple[MotorRequest, ...]) -> None: ...


class ControlIO(MotorIO, Protocol):
    """由一个控制线程依次调用的非阻塞接口。

    poll_* 返回本次取出的全部新输入，不得把旧反馈当作新数据重复上报。
    电机请求保持顺序，包括 SAVE_ZERO/ENABLE/DISABLE；无法接收输出时必须报错。
    电机反馈和设零确认由后续 poll_feedback 调用返回。
    CANControlIO 通过回调提供命令输入和反馈、事件输出。
    """

    def poll_commands(self, now_ms: int) -> Iterable[Command]: ...
    def publish_feedback(self, feedback: tuple[ArmFeedback, ...]) -> None: ...
    def publish_events(self, events: tuple[Event, ...]) -> None: ...
