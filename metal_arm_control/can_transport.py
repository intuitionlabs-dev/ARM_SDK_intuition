"""CAN 传输类型的导入入口。"""

from metal_arm_control.usb2can import CANableTransport, SocketCANTransport

__all__ = ["CANableTransport", "SocketCANTransport"]
