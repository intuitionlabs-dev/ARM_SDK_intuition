"""H7 串口协议：CRC、版本哈希、帧解析和载荷编解码。

协议使用 deg、deg/s、deg/s²、N·m 和 ℃；SDK 单位换算由上层完成。
帧格式：AA 55 | type_id | datalen | arm_id | crc8 | payload | crc16(LE)。
总长 8 + datalen，最大 263 字节。CRC8 校验前 5 字节，CRC16 校验头部及载荷。
多字节字段均为小端。固件定义见 libs/frame/frame.h 和 type_id.h。
"""

import struct
from dataclasses import dataclass
from math import isfinite

# ==================== CRC ====================
#
# 两张表**在这里生成**，而不是抄 256 个常数下来。理由：手抄 256 个数字出错了我自己看不出来，
# 而这里从"多项式 + 初值"推出来的表和板子 `libs/soft_crc/soft_crc.c` 生成表的方式一模一样。
# 生成出来的值仍然由 `selftest.py` 用**另一套位运算实现**逐字节比对，并且整帧字节要和板上
# `FrameEncode()` 的真实输出对得上 —— 所以"自己算表"不降低可信度，只是消掉了抄错的余地。
#
# crc8  = CRC-8/MAXIM  ：多项式 0x31（反射 0x8C）、初值 0xFF
# crc16 = CRC-16/X25   ：多项式 0x1021（反射 0x8408）、初值 0xFFFF
#         （板子那边的注释写作 "MODBUS 0x8408"，实际用的是 0x8408 这个反射多项式；
#           0x8408 才是 0x1021 的按位反转，别被那句注释带跑。）

_T8_POLY = 0x8C
_T16_POLY = 0x8408


def _build_t8():
    t = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = ((c >> 1) ^ _T8_POLY) if (c & 1) else (c >> 1)
        t.append(c)
    return tuple(t)


def _build_t16():
    t = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = ((c >> 1) ^ _T16_POLY) if (c & 1) else (c >> 1)
        t.append(c)
    return tuple(t)


T8 = _build_t8()
T16 = _build_t16()


def crc8_ref(data):
    """CRC8 位运算参考实现，用于核对查表算法。"""
    c = 0xFF
    for x in data:
        c ^= x
        for _ in range(8):
            c = ((c >> 1) ^ _T8_POLY) if (c & 1) else (c >> 1)
    return c


def crc16_ref(data):
    """CRC16 位运算参考实现，用于核对查表算法。"""
    c = 0xFFFF
    for x in data:
        c ^= x
        for _ in range(8):
            c = ((c >> 1) ^ _T16_POLY) if (c & 1) else (c >> 1)
    return c & 0xFFFF


def crc8(data):
    c = 0xFF
    for x in data:
        c = T8[(c ^ x) & 0xFF]
    return c


def crc16(data):
    c = 0xFFFF
    for x in data:
        c = (c >> 8) ^ T16[(c ^ x) & 0xFF]
    return c & 0xFFFF


# ==================== 版本 ====================

def fnv1a32(s):
    """计算版本串的 FNV-1a 32 位哈希，与固件 FrameVersionHash 一致。"""
    h = 0x811C9DC5
    for b in s.encode("utf-8"):
        h = ((h ^ b) * 0x01000193) & 0xFFFFFFFF
    return h


# arm_id 的语义：**arm_id 就是 CAN 总线索引**，0 = CAN1 = 臂A、1 = CAN2 = 臂B。
# 板子只认 arm_id < ARM_NUM(2)，越界的帧**静默丢弃**（连版本包都不回）。
DEFAULT_ARM_ID = 0

# ==================== 码表（libs/frame/type_id.h） ====================

TID_VERSION = 0x01            # 版本匹配 · 双向（上位机发"我是什么版本"，板子回自报家门）
TID_ARM_INIT = 0x02           # 初始化命令 · 下发（空 payload）
TID_MOTOR_SET_ZERO = 0x10     # 触发一轮设零 · 下发（空 payload）
TID_MOTOR_SET_ZERO_ACK = 0x11  # 设零回复 · 上报
TID_ARM_START = 0x12          # 进 run 档 · 下发（空 payload）
TID_ARM_GRAVITY = 0x13        # 进重力补偿档 · 下发（空 payload）
TID_ARM_STOP = 0x14           # 停止（松劲）· 下发（空 payload）
TID_ARM_DAMPING = 0x15        # 进阻尼档 · 下发（空 payload）
TID_ARM_CTRL_POS = 0x16       # 位置控制 · 下发
TID_ARM_RESET = 0x18          # 一键复位 · 下发（空 payload，不做夹爪找零）
TID_ARM_FEEDBACK = 0x80       # 机械臂反馈帧 · 上报（握手通过后 250 Hz 常流）
TID_LINK_STATUS = 0x81        # 链路状态帧 · 上报（握手通过后 10 Hz，链路级只一帧）

EMPTY_MODE_TIDS = (TID_ARM_START, TID_ARM_GRAVITY, TID_ARM_STOP, TID_ARM_DAMPING)

# 线上固定尺寸（协议定的，不是 App 的 ARM_JOINT_NUM / ARM_MOTOR_NUM）
PKT_JOINT_NUM = 6
PKT_MOTOR_NUM = 7

FB_LEN = 63                    # 4×u8 + 6f + 6f + f + 7×u8
_CTRL_POS = struct.Struct("<20f")
CTRL_POS_LEN = _CTRL_POS.size
SET_ZERO_ACK_LEN = 12          # u8 + u8[7] + u32
LINK_STATUS_LEN = 84           # 10×u32 + 11×u32

# 0x81 里 rx_by_tid[] 的分类顺序：**下标就是它在元组里的位置**，最后一格是「其他」。
# 顺序的唯一出处是板子 `libs/frame/type_id.h` 的 `FRAME_TID_DOWN_LIST`，自测会逐项比对。
# 板子自己发的 0x80 不在表里 —— 上位机不该发它，真收到了就落「其他」，正好是异常信号。
LINK_TID_ORDER = (TID_VERSION, TID_ARM_INIT,
                  TID_MOTOR_SET_ZERO, TID_MOTOR_SET_ZERO_ACK,
                  TID_ARM_START, TID_ARM_GRAVITY, TID_ARM_STOP, TID_ARM_DAMPING,
                  TID_ARM_CTRL_POS, TID_ARM_RESET)
LINK_TID_OTHER = len(LINK_TID_ORDER)      # 「其他」那一格的下标（= 10）
LINK_TID_SLOT_NUM = LINK_TID_OTHER + 1    # 数组长度（= 11）

SYNC = b"\xAA\x55"
HEADER_LEN = 6                 # 0..5：AA 55 / type / datalen / arm_id / crc8
OVERHEAD = 8
MAX_DATA = 255
MAX_FRAME = OVERHEAD + MAX_DATA  # 263

ARM_ID_OFFSET = 4


# ==================== 组帧 ====================

def encode(tid, payload=b"", arm_id=DEFAULT_ARM_ID):
    """组帧并校验载荷长度和 arm_id 字节范围；实际臂编号范围由链路检查。"""
    if len(payload) > MAX_DATA:
        raise ValueError("payload 太长：%d > %d" % (len(payload), MAX_DATA))
    if not 0 <= arm_id <= 0xFF:
        raise ValueError("arm_id 要是一个字节，收到 %r" % (arm_id,))

    head = bytes((0xAA, 0x55, tid & 0xFF, len(payload), arm_id))
    body = head + bytes((crc8(head),)) + bytes(payload)
    c = crc16(body)
    return body + bytes((c & 0xFF, (c >> 8) & 0xFF))


def encode_ctrl_pos(angles_deg7, gripper_torque_limit_nm, *,
                    joint_vel_deg_s=None, joint_acc_deg_s2=None):
    """编码 0x16：关节角、夹爪角、夹爪力矩上限、关节速度和加速度。"""
    angles = tuple(angles_deg7)
    if len(angles) != PKT_JOINT_NUM + 1:
        raise ValueError("0x16 要 %d 个角度，收到 %d" % (PKT_JOINT_NUM + 1, len(angles)))

    def derivatives(values, name):
        result = (0.0,) * PKT_JOINT_NUM if values is None else tuple(values)
        if len(result) != PKT_JOINT_NUM:
            raise ValueError(f"{name}需要 {PKT_JOINT_NUM} 项")
        return result

    velocity = derivatives(joint_vel_deg_s, "关节速度")
    acceleration = derivatives(joint_acc_deg_s2, "关节加速度")
    limit = float(gripper_torque_limit_nm)
    values = (*angles, limit, *velocity, *acceleration)
    if not all(isfinite(value) for value in values) or limit < 0:
        raise ValueError("控制字段必须有限，夹爪力矩上限必须非负")
    try:
        return _CTRL_POS.pack(*values)
    except OverflowError as exc:
        raise ValueError("控制字段超出 float32 范围") from exc


def decode_ctrl_pos(payload):
    """返回 (七个目标角, 夹爪力矩上限, 六个目标速度, 六个目标加速度)。"""
    if len(payload) != CTRL_POS_LEN:
        raise ValueError(f"0x16 载荷必须为 {CTRL_POS_LEN} 字节")
    values = _CTRL_POS.unpack(payload)
    if not all(isfinite(value) for value in values) or values[7] < 0:
        raise ValueError("控制字段必须有限，夹爪力矩上限必须非负")
    return values[:7], values[7], values[8:14], values[14:20]


def encode_version(code):
    """编码 0x01 的四字节小端版本码；主机和固件须使用匹配的协议版本。"""
    return struct.pack("<I", code & 0xFFFFFFFF)


def encode_feedback(mode, robot_state, motor_lost_mask, motor_inited_mask,
                    joint_deg, jvel_deg, gripper_deg, temps):
    """编码 0x80 反馈载荷，共 63 字节，温度须含七项；供模拟器和测试使用。"""
    if len(joint_deg) != PKT_JOINT_NUM or len(jvel_deg) != PKT_JOINT_NUM:
        raise ValueError("0x80 的关节角和角速度都要 %d 个" % PKT_JOINT_NUM)
    if len(temps) != PKT_MOTOR_NUM:
        raise ValueError("0x80 的温度要 %d 个" % PKT_MOTOR_NUM)
    payload = struct.pack("<4B6f6ff7B",
                          mode & 0xFF, robot_state & 0xFF,
                          motor_lost_mask & 0xFF, motor_inited_mask & 0xFF,
                          *joint_deg, *jvel_deg, gripper_deg,
                          *[int(t) & 0xFF for t in temps])
    if len(payload) != FB_LEN:      # 结构体格式写错了就在这里当场炸，别等板子
        raise AssertionError("0x80 payload 组出 %d 字节，应为 %d" % (len(payload), FB_LEN))
    return payload


def encode_link_status(rx_total, rx_by_tid, crc8_err, crc16_err, drop_bytes,
                       bad_arm, over_len, ring_drop, gated, tx_fail, dt_ms):
    """编码 0x81 链路状态，共 84 字节；rx_by_tid 含 11 项，末项为其他类型。"""
    if len(rx_by_tid) != LINK_TID_SLOT_NUM:
        raise ValueError("0x81 的分类计数要 %d 个" % LINK_TID_SLOT_NUM)
    payload = struct.pack("<%dI" % (10 + LINK_TID_SLOT_NUM), rx_total & 0xFFFFFFFF, *rx_by_tid,
                          crc8_err & 0xFFFFFFFF, crc16_err & 0xFFFFFFFF,
                          drop_bytes & 0xFFFFFFFF, bad_arm & 0xFFFFFFFF,
                          over_len & 0xFFFFFFFF, ring_drop & 0xFFFFFFFF,
                          gated & 0xFFFFFFFF, tx_fail & 0xFFFFFFFF,
                          dt_ms & 0xFFFFFFFF)
    if len(payload) != LINK_STATUS_LEN:
        raise AssertionError("0x81 payload 组出 %d 字节，应为 %d"
                             % (len(payload), LINK_STATUS_LEN))
    return payload


# ==================== payload 解包 ====================

@dataclass(frozen=True)
class Feedback:
    """不可变的 0x80 反馈，单位为 deg、deg/s 和 ℃。

    电机掩码 bit0..5 对应六轴，bit6 对应夹爪。SI 单位视图见 Arm.snapshot()。
    """

    mode: int
    robot_state: int
    motor_lost_mask: int
    motor_inited_mask: int
    joint_angle: tuple        # deg，6 个
    joint_vel: tuple          # deg/s，6 个
    gripper_angle: float      # deg —— **电机原始角**，双向都不做 pos_to_angle 换算
    motor_temp: tuple         # ℃，7 个（uint8；板子那边 NAN 会写成 0，两者区分不了）


@dataclass(frozen=True)
class LinkStatus:
    """链路级 0x81 状态，各计数表示相邻报告之间的窗口增量。

    rx_total 包含解析后被拒绝的帧；rx_by_tid 顺序见 LINK_TID_ORDER，末项为其他类型。
    """

    rx_total: int             # 这段窗口里解出来的整帧数
    rx_by_tid: tuple          # 11 个，顺序同 LINK_TID_ORDER
    crc8_err: int             # crc8 错被丢的帧数
    crc16_err: int            # crc16 错被丢的帧数
    drop_bytes: int           # 喂进解析器但没能组成任何有效帧的字节数
    bad_arm: int              # arm_id >= ARM_NUM 被丢的帧数（正常恒 0）
    over_len: int             # payload 超长被丢的帧数（正常恒 0）
    ring_drop: int            # 收包环满被丢的帧数
    gated: int                # 被握手闸门挡下的帧数
    tx_fail: int              # 板子**发**帧失败的次数
    dt_ms: int                # 距上一帧 0x81 的实际毫秒数（标称 100）


@dataclass(frozen=True)
class SetZeroAck:
    """0x11 设零结果，七项 result 依次对应六轴和夹爪。"""

    state: int                # Arm_zero_state（见 board_defs.ZERO_STATE_NAME）
    result: tuple             # 7 个 Arm_zero_result（见 board_defs.ZERO_RES_NAME）
    elapsed_ms: int


def decode_feedback(payload):
    if len(payload) != FB_LEN:
        raise ValueError("0x80 长度 %d != %d" % (len(payload), FB_LEN))
    return Feedback(
        mode=payload[0],
        robot_state=payload[1],
        motor_lost_mask=payload[2],
        motor_inited_mask=payload[3],
        joint_angle=struct.unpack_from("<6f", payload, 4),
        joint_vel=struct.unpack_from("<6f", payload, 28),
        gripper_angle=struct.unpack_from("<f", payload, 52)[0],
        motor_temp=tuple(payload[56:63]),
    )


def decode_link_status(payload):
    if len(payload) != LINK_STATUS_LEN:
        raise ValueError("0x81 长度 %d != %d" % (len(payload), LINK_STATUS_LEN))
    # 和 encode 一样分两段读：前段 = rx_total + rx_by_tid，后段 = 剩下 9 个。
    # 用 LINK_TID_SLOT_NUM 算偏移，别再手写 13/14/… —— 槽位数一改就该自动跟着走。
    head = struct.unpack_from("<%dI" % (1 + LINK_TID_SLOT_NUM), payload, 0)
    tail = struct.unpack_from("<9I", payload, 4 * (1 + LINK_TID_SLOT_NUM))
    return LinkStatus(
        rx_total=head[0],
        rx_by_tid=head[1:],
        crc8_err=tail[0],
        crc16_err=tail[1],
        drop_bytes=tail[2],
        bad_arm=tail[3],
        over_len=tail[4],
        ring_drop=tail[5],
        gated=tail[6],
        tx_fail=tail[7],
        dt_ms=tail[8],
    )


def decode_setzero_ack(payload):
    if len(payload) != SET_ZERO_ACK_LEN:
        raise ValueError("0x11 长度 %d != %d" % (len(payload), SET_ZERO_ACK_LEN))
    return SetZeroAck(
        state=payload[0],
        result=tuple(payload[1:8]),
        elapsed_ms=struct.unpack_from("<I", payload, 8)[0],
    )


def decode_version(payload):
    if len(payload) != 4:
        raise ValueError("0x01 长度 %d != 4" % len(payload))
    return struct.unpack("<I", payload)[0]


# ==================== 切帧 ====================

class FrameParser:
    """将字节流解析为 (type_id, arm_id, payload)。

    校验失败后逐字节寻找 AA 55，支持半帧、粘包和噪声重同步。
    统计有效帧、CRC 错误和丢弃字节；协议无帧序号，无法检测整帧丢失。
    """

    CRC8_LEN = 5   # crc8 覆盖偏移 0..4
    HEADER_LEN = 6

    def __init__(self):
        self.buf = bytearray()
        self.n_ok = 0
        self.n_err_crc8 = 0
        self.n_err_crc16 = 0
        self.n_drop = 0   # 被跳过的字节数（找不到同步头 / 半帧残留）

    def feed(self, chunk):
        out = []
        self.buf += chunk
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                # 一个同步头都没有。最后 1 个字节可能是 AA 的前半，留着等下一片。
                if len(self.buf) > 1:
                    self.n_drop += len(self.buf) - 1
                    del self.buf[:-1]
                break
            if i > 0:
                self.n_drop += i
                del self.buf[:i]
            if len(self.buf) < self.HEADER_LEN:
                break
            n = self.buf[3]
            total = OVERHEAD + n
            if len(self.buf) < total:
                break
            if crc8(self.buf[:self.CRC8_LEN]) != self.buf[self.CRC8_LEN]:
                self.n_err_crc8 += 1
                del self.buf[:1]
                continue
            if crc16(self.buf[:6 + n]) != (self.buf[6 + n] | (self.buf[7 + n] << 8)):
                self.n_err_crc16 += 1
                del self.buf[:1]
                continue
            out.append((self.buf[2], self.buf[4], bytes(self.buf[6:6 + n])))
            self.n_ok += 1
            del self.buf[:total]
        return out

    def reset_stream(self):
        """丢弃旧连接的残留字节并计入 n_drop，保留累计统计。"""
        self.n_drop += len(self.buf)
        self.buf.clear()

    @property
    def n_err(self):
        return self.n_err_crc8 + self.n_err_crc16
