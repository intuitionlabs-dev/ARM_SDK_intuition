"""经典 CAN 标准数据帧。"""

from dataclasses import dataclass


@dataclass(frozen=True)
class CANFrame:
    arbitration_id: int
    data: bytes

    def __post_init__(self):
        if type(self.arbitration_id) is not int or not 0 <= self.arbitration_id <= 0x7FF:
            raise ValueError("standard CAN identifier must be 0..2047")
        if not isinstance(self.data, bytes) or len(self.data) > 8:
            raise ValueError("classic CAN payload must contain at most eight bytes")
