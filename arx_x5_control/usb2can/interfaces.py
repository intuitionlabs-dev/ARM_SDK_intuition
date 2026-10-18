"""CAN 传输后端的接口约定。"""

from typing import Protocol

from .frame import CANFrame


class Transport(Protocol):
    """由所属收发线程调用的非阻塞帧接口。"""

    def send(self, frame: CANFrame) -> None: ...

    def receive(self) -> list[tuple[CANFrame, int]]: ...

    def close(self) -> None: ...


class ConfigurableTransport(Transport, Protocol):
    """提供启动阶段通道配置的传输后端。"""

    def configure(self) -> None: ...
