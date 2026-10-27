"""可配置的 USB-CAN 传输接口。"""

from .frame import CANFrame
from .interfaces import ConfigurableTransport, Transport
from .registry import TransportConfigError, TransportRegistry, TransportSpec
from .slcan import CANableTransport
from .socketcan import SocketCANTransport

__all__ = ["CANFrame", "Transport", "ConfigurableTransport", "TransportConfigError", "TransportRegistry",
           "TransportSpec", "CANableTransport", "SocketCANTransport"]
