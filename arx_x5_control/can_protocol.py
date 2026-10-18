"""ENCOS V1.20EAP 与达妙 MIT 的标准 CAN 帧编解码。"""

from arx_x5_control.usb2can import CANFrame
from math import isfinite, pi, radians
import struct

from .config import MOTOR_IDS
from .types import MotorAction, MotorFeedback, MotorRequest, timestamp



def _f32(value):
    return struct.unpack("=f", struct.pack("=f", value))[0]


def _quantize(value, low, high, bits, *, encos):
    value = _f32(max(low, min(value, high)))
    if encos:
        return int((value-low) * ((1 << bits)-1) / (high-low))
    return int(_f32(_f32(_f32(value-low) * ((1 << bits)-1)) / (high-low)))


def encode_request(request):
    """验证语义请求，返回一帧标准 CAN 数据帧。"""
    if not isinstance(request, MotorRequest):
        raise TypeError("expected MotorRequest")
    timestamp(request.timestamp_ms)
    i = request.motor_index
    if type(request.arm_id) is not int or request.arm_id not in (0, 1):
        raise ValueError("arm_id must be 0 or 1")
    if type(i) is not int or not 0 <= i < 7 or request.motor_id != MOTOR_IDS[i]:
        raise ValueError("motor index and CAN identifier do not match")
    encos = i < 3
    if request.family != ("encos" if encos else "dm") or not isinstance(request.action, MotorAction):
        raise ValueError("invalid motor family or action")
    values = (request.position, request.velocity, request.torque, request.kp, request.kd)
    if not all(isfinite(v) for v in values) or request.kp < 0 or request.kd < 0:
        raise ValueError("motor references must be finite; gains must be nonnegative")
    action, motor_id = request.action, request.motor_id
    if action == MotorAction.SAVE_ZERO:
        return (CANFrame(0x7FF, bytes((0, motor_id, 0, 3))) if encos
                else CANFrame(motor_id, b"\xff" * 7 + b"\xfe"))
    if action == MotorAction.ENABLE:
        if encos:
            raise ValueError("ENCOS enable is part of mixed control")
        return CANFrame(motor_id, b"\xff" * 7 + b"\xfc")
    if action == MotorAction.DISABLE and not encos:
        return CANFrame(motor_id, b"\xff" * 7 + b"\xfd")
    p, v, t, kp, kd = values if action == MotorAction.CONTROL else (0,) * 5
    position = _quantize(p, -12.5, 12.5, 16, encos=encos)
    speed = _quantize(v, -18 if encos else -45, 18 if encos else 45, 12, encos=encos)
    torque = _quantize(t, -30 if encos else -10, 30 if encos else 10, 12, encos=encos)
    k = _quantize(kp, 0, 500, 12, encos=encos)
    d = _quantize(kd, 0, 50 if encos else 5, 9 if encos else 12, encos=encos)
    if encos:
        payload = bytes((k >> 7, ((k << 1) | (d >> 8)) & 255, d & 255,
                         position >> 8, position & 255, speed >> 4,
                         ((speed << 4) | (torque >> 8)) & 255, torque & 255))
    else:
        payload = bytes((position >> 8, position & 255, speed >> 4,
                         ((speed << 4) | (k >> 8)) & 255, k & 255,
                         d >> 4, ((d << 4) | (torque >> 8)) & 255, torque & 255))
    return CANFrame(motor_id, payload)


def feedback_queries(motor_index):
    """返回位置、速度、电流查询或达妙状态读取帧。"""
    if type(motor_index) is not int or not 0 <= motor_index < 7:
        raise ValueError("motor_index must be 0..6")
    motor_id = MOTOR_IDS[motor_index]
    if motor_index < 3:
        return tuple(CANFrame(motor_id, bytes((0xE0, code))) for code in (1, 2, 3))
    return (CANFrame(0x7FF, bytes((motor_id, 0, 0xCC, 0))),)


def decode_feedback(arm_id, frame, timestamp_ms):
    """按总线身份解码反馈；未知类型和无效载荷返回 None。"""
    data, can_id = frame.data, frame.arbitration_id
    fields = {}
    if can_id == 0x7FF and len(data) == 4 and data[0] == 0 and data[1] in MOTOR_IDS[:3]:
        if data[2] != 1 or data[3] not in (0, 3):
            return None
        return MotorFeedback(arm_id, MOTOR_IDS.index(data[1]), timestamp_ms, zero_ack=data[3] == 3)
    if can_id in MOTOR_IDS[:3] and len(data) >= 2:
        index, error, kind = MOTOR_IDS.index(can_id), data[0] & 31, data[0] >> 5
        if kind == 1 and len(data) == 8:
            fields = dict(position=int.from_bytes(data[1:3], "big") / 65535 * 25 - 12.5,
                          velocity=((data[3] << 4) | (data[4] >> 4)) / 4095 * 36 - 18,
                          current=(((data[4] & 15) << 8) | data[5]) / 4095 * 60 - 30,
                          temperature=(data[6] - 50) / 2, mos_temperature=(data[7] - 50) / 2)
        elif kind in (2, 3) and len(data) == 8:
            value = struct.unpack(">f", data[1:5])[0]
            fields = {"position" if kind == 2 else "velocity": radians(value) if kind == 2 else value*pi/30,
                      "current": int.from_bytes(data[5:7], "big", signed=True) / 100,
                      "temperature": (data[7] - 50) / 2}
        elif kind == 5 and len(data) == 6 and data[1] in (1, 2, 3):
            value = struct.unpack(">f", data[2:])[0]
            name = {1: "position", 2: "velocity", 3: "current"}[data[1]]
            fields[name] = radians(value) if data[1] == 1 else value*pi/30 if data[1] == 2 else value
        else:
            return None
    elif can_id == 0 and len(data) == 8 and (data[0] & 15) in MOTOR_IDS[3:]:
        index, error = MOTOR_IDS.index(data[0] & 15), data[0] >> 4
        # 达妙反馈的归一化分母与固件 DMmotor_FeedbackData_Update 一致。
        fields = dict(position=int.from_bytes(data[1:3], "big") / 65536 * 25 - 12.5,
                      velocity=((data[3] << 4) | (data[4] >> 4)) / 4096 * 90 - 45,
                      torque=((((data[4] & 15) << 8) | data[5]) - 2048) / 2048 * 10,
                      mos_temperature=data[6], temperature=data[7])
    else:
        return None
    if not all(isfinite(v) for v in fields.values()):
        return None
    return MotorFeedback(arm_id, index, timestamp_ms, error_code=error, **fields)
