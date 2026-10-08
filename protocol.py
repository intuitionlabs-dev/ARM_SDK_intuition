"""线上协议层：CRC、版本码、码表、组帧 / 切帧、payload 编解码。

**这一层只用线上单位**（deg / deg·s⁻¹ / ℃），一比特都不换算 —— 换算是 `arm.py` 的事。
这样协议层可以脱离硬件、脱离单位策略单独测，也保证"线上到底长什么样"在这里一眼看全。

协议版本 v0.2，权威定义在板子那边：`libs/frame/frame.h`（帧格式）、
`libs/frame/type_id.h`（码表和 payload 布局）、`libs/frame/frame_config.h`（版本串）。

    AA 55 | type_id | datalen | arm_id | crc8 | payload | crc16(LE)
     0  1  |    2    |    3    |    4   |  5   | 6..5+N  | 6+N 7+N

整帧 8 + datalen 字节，最大 263。crc8 盖偏移 0..4（收完头就能把长度字段被干扰的帧挡掉），
crc16 盖偏移 0..(5+N)。**全部小端。**

这一层不 import 任何别的 SDK 模块（连 `errors` 都不需要 —— 它只抛 `ValueError`，
交给上层包装），所以它是最底、最干净的一层。
"""

import struct
from dataclasses import dataclass

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
    """CRC-8/MAXIM 的**位运算参考实现**（不查表）。给自测拿来对拍表。"""
    c = 0xFF
    for x in data:
        c ^= x
        for _ in range(8):
            c = ((c >> 1) ^ _T8_POLY) if (c & 1) else (c >> 1)
    return c


def crc16_ref(data):
    """CRC-16/X25 的**位运算参考实现**（不查表）。给自测拿来对拍表。"""
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
    """板子算版本码用的就是 FNV-1a 32（`libs/frame/frame.c` 的 FrameVersionHash）。

    **任何语言三行就能复刻**，所以上位机不必移植板子那份实现 —— 这个函数就是那三行。
    """
    h = 0x811C9DC5
    for b in s.encode("utf-8"):
        h = ((h ^ b) * 0x01000193) & 0xFFFFFFFF
    return h


def parse_version_string(text):
    """从 `frame_config.h` 的原文里抠出 FRAME_VERSION_STRING（给自测对拍用）。

    找不到返回 None —— 那是"这文件里没这一项"，不是错误。
    """
    import re
    m = re.search(r'FRAME_VERSION_STRING\s+"([^"]*)"', text)
    return m.group(1) if m else None


# 抄自板子 `libs/frame/frame_config.h` 的 FRAME_VERSION_STRING。**版本码不硬写**，
# 从串算（板上改了串这里就跟上，不用记 0xF91DD3C9）—— 自测里再钉死这个值为 0xF91DD3C9，
# 那是为了"改了串必须显式改上位机"这条纪律，是故意的。
#
# v0.1 → v0.2：帧头在 datalen 之后插了一字节 arm_id（payload 起点 5→6、开销 7→8）。
# 这是**不兼容改版**，所以版本串必须跟着改 —— 一段 v0.1 的上位机拿 v0.2 的帧会整体错位，
# 换码能让它在 0x01 上就被板子挡下来锁死，而不是拿旧布局去解析新帧。
VERSION_STRING = "Intuition-ARX-X5-2023-v0.2"
VERSION_CODE = fnv1a32(VERSION_STRING)

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
TID_ARM_CTRL_POS_VEL = 0x17   # 位置 + 速度前馈 · 下发
TID_ARM_RESET = 0x18          # 一键复位 · 下发（空 payload，不做夹爪找零）
TID_ARM_FEEDBACK = 0x80       # 机械臂反馈帧 · 上报（握手通过后 250 Hz 常流）

EMPTY_MODE_TIDS = (TID_ARM_START, TID_ARM_GRAVITY, TID_ARM_STOP, TID_ARM_DAMPING)

# 线上固定尺寸（协议定的，不是 App 的 ARM_JOINT_NUM / ARM_MOTOR_NUM）
PKT_JOINT_NUM = 6
PKT_MOTOR_NUM = 7

FB_LEN = 63                    # 4×u8 + 6f + 6f + f + 7×u8
CTRL_POS_LEN = 28              # 7f
CTRL_POS_VEL_LEN = 52          # 13f
SET_ZERO_ACK_LEN = 12          # u8 + u8[7] + u32

SYNC = b"\xAA\x55"
HEADER_LEN = 6                 # 0..5：AA 55 / type / datalen / arm_id / crc8
OVERHEAD = 8
MAX_DATA = 255
MAX_FRAME = OVERHEAD + MAX_DATA  # 263

ARM_ID_OFFSET = 4


# ==================== 组帧 ====================

def encode(tid, payload=b"", arm_id=DEFAULT_ARM_ID):
    """组一帧：`AA 55 | tid | datalen | arm_id | crc8 | payload | crc16(LE)`。

    arm_id 是"这一包发给哪台臂"（0 = CAN1，1 = CAN2），**所有 type_id 都带**。
    越界的 arm_id 不加拦截（那是上层的事，见 `link.py` 的闸门）—— 帧层保持"不认识业务"。
    """
    if len(payload) > MAX_DATA:
        raise ValueError("payload 太长：%d > %d" % (len(payload), MAX_DATA))
    if not 0 <= arm_id <= 0xFF:
        raise ValueError("arm_id 要是一个字节，收到 %r" % (arm_id,))

    head = bytes((0xAA, 0x55, tid & 0xFF, len(payload), arm_id))
    body = head + bytes((crc8(head),)) + bytes(payload)
    c = crc16(body)
    return body + bytes((c & 0xFF, (c >> 8) & 0xFF))


def encode_ctrl_pos(angles_deg7):
    """0x16 的 payload：6 个关节角 + 夹爪角，float32 LE，deg。"""
    if len(angles_deg7) != PKT_JOINT_NUM + 1:
        raise ValueError("0x16 要 %d 个角度，收到 %d" % (PKT_JOINT_NUM + 1, len(angles_deg7)))
    return struct.pack("<7f", *angles_deg7)


def encode_ctrl_pos_vel(angles_deg6, vels_deg6, gripper_deg):
    """0x17 的 payload：位置 + 速度前馈（deg、deg/s）+ 夹爪角。"""
    if len(angles_deg6) != PKT_JOINT_NUM or len(vels_deg6) != PKT_JOINT_NUM:
        raise ValueError("0x17 的关节角和角速度都要 %d 个" % PKT_JOINT_NUM)
    return struct.pack("<13f", *angles_deg6, *vels_deg6, gripper_deg)


def encode_version(code=VERSION_CODE):
    """0x01 的 payload：4 字节小端版本码。"""
    return struct.pack("<I", code & 0xFFFFFFFF)


def encode_feedback(mode, robot_state, motor_lost_mask, motor_inited_mask,
                    joint_deg, jvel_deg, gripper_deg, temps):
    """0x80 的 payload（63 字节）—— `decode_feedback` 的逆。

    板子只发这一帧，上位机不主动发；这里存在是为了让 **0x80 的布局只有一处定义**：
    模拟板子（`sim.py`）和离线自测都拿它组帧，字段一挪两边同时错，不会各抄一份。
    `temps` 要 7 个（板子那边 NAN 写成 0），其余单位同 `Feedback`（deg / deg·s⁻¹ / ℃）。
    """
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


# ==================== payload 解包 ====================

@dataclass(frozen=True)
class Feedback:
    """一帧 0x80。**字段是线上单位**（deg / deg·s⁻¹ / ℃），不是 SI。

    不可变（frozen）：`link` 每收到一帧就整个换一个新的，读侧拿到的那份永远不会被改写。
    要 rad / m 用 `arm.Arm.snapshot()`。

    `motor_lost_mask` / `motor_inited_mask`：bit0..5 = j1..j6、bit6 = 夹爪。
    `robot_state` / `mode` 的取值名字表在 `board_defs.py`。
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
class SetZeroAck:
    """一帧 0x11（设零回复）。`result` 是 7 台电机各自的结果码，索引 0..5 = j1..j6、6 = 夹爪。"""

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
    """字节流 → 帧。粘包 / 前导噪声 / 半帧 / 坏 CRC 都能重同步。

    重同步策略：找 `AA 55` → 凑够 `8+len` 字节 → 先校 crc8，不对就把窗口**只挪一格**
    再找（而不是丢掉整帧）—— 因为 payload 里完全可能出现 `AA 55`，整帧丢掉会把紧跟其后的
    真帧一起吃掉。

    `feed()` 吐出的是 **(type_id, arm_id, payload)** 三元组：arm_id 是帧头偏移 4 那个字节，
    上层按它分流。

    统计量：`n_ok` / `n_err_crc8` / `n_err_crc16` / `n_drop`（字节）。**没有帧编号、也
    没有时间戳，所以这一层发现不了"整帧丢了"** —— 能看见的只有字节层面的错。
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

    @property
    def n_err(self):
        return self.n_err_crc8 + self.n_err_crc16
