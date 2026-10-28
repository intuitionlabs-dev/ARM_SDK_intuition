"""Metal Damiao MIT frames, matching metal-python-ros bccf3c8.

Transmit IDs 0x01..0x07; feedback IDs 0x11..0x17. MIT ranges are
flashed per joint and MUST NOT be replaced with generic Damiao defaults.
"""
from math import isfinite
import struct

from .usb2can import CANFrame
from .config import MOTOR_IDS, MOTOR_RX_IDS, MOTOR_PMAX, MOTOR_VMAX, MOTOR_TMAX
from .types import MotorAction, MotorFeedback, MotorRequest, timestamp


def _f32(value):
    return struct.unpack('=f', struct.pack('=f', value))[0]


def _quantize(value, low, high, bits):
    # Vendor C++ float_to_uint: float arithmetic, truncate toward zero.
    value = _f32(max(low, min(value, high)))
    low, high = _f32(low), _f32(high)
    return int(_f32(_f32(_f32(value-low) / _f32(high-low)) * ((1 << bits)-1)))


def encode_request(request):
    if not isinstance(request, MotorRequest):
        raise TypeError('expected MotorRequest')
    timestamp(request.timestamp_ms)
    i = request.motor_index
    if type(request.arm_id) is not int or request.arm_id not in (0, 1):
        raise ValueError('arm_id must be 0 or 1')
    if type(i) is not int or not 0 <= i < 7 or request.motor_id != MOTOR_IDS[i]:
        raise ValueError('motor index and CAN identifier do not match')
    if request.family != 'dm' or not isinstance(request.action, MotorAction):
        raise ValueError('Metal requires Damiao motor requests')
    values = (request.position, request.velocity, request.torque, request.kp, request.kd)
    if not all(isfinite(v) for v in values) or request.kp < 0 or request.kd < 0:
        raise ValueError('motor references must be finite; gains must be nonnegative')
    special = {MotorAction.ENABLE: 0xFC, MotorAction.DISABLE: 0xFD, MotorAction.SAVE_ZERO: 0xFE}
    if request.action in special:
        return CANFrame(request.motor_id, b'\xff'*7 + bytes((special[request.action],)))
    p, v, t, kp, kd = values if request.action == MotorAction.CONTROL else (0.,)*5
    p = _quantize(p, -MOTOR_PMAX[i], MOTOR_PMAX[i], 16)
    v = _quantize(v, -MOTOR_VMAX[i], MOTOR_VMAX[i], 12)
    t = _quantize(t, -MOTOR_TMAX[i], MOTOR_TMAX[i], 12)
    kp = _quantize(kp, 0., 500., 12)
    kd = _quantize(kd, 0., 5., 12)
    return CANFrame(request.motor_id, bytes((p >> 8, p & 255, v >> 4,
                    ((v & 15) << 4) | (kp >> 8), kp & 255, kd >> 4,
                    ((kd & 15) << 4) | (t >> 8), t & 255)))


def feedback_queries(motor_index):
    """Damiao read-state command (no enable, zeroing or motion)."""
    if type(motor_index) is not int or not 0 <= motor_index < 7:
        raise ValueError('motor_index must be 0..6')
    return (CANFrame(0x7FF, bytes((MOTOR_IDS[motor_index], 0, 0xCC, 0))),)


def decode_feedback(arm_id, frame, timestamp_ms):
    data = frame.data
    if frame.arbitration_id not in MOTOR_RX_IDS or len(data) != 8:
        return None
    i = MOTOR_RX_IDS.index(frame.arbitration_id)
    if data[0] & 15 != MOTOR_IDS[i]:
        return None
    p = int.from_bytes(data[1:3], 'big')
    v = (data[3] << 4) | (data[4] >> 4)
    t = ((data[4] & 15) << 8) | data[5]
    return MotorFeedback(arm_id, i, timestamp_ms,
        position=p / 65535 * (2*MOTOR_PMAX[i]) - MOTOR_PMAX[i],
        velocity=v / 4095 * (2*MOTOR_VMAX[i]) - MOTOR_VMAX[i],
        torque=t / 4095 * (2*MOTOR_TMAX[i]) - MOTOR_TMAX[i],
        error_code=data[0] >> 4, mos_temperature=data[6], temperature=data[7])
