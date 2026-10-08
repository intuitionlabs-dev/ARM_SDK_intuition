"""离线自测：`python3 -m sdk.selftest`（**不接硬件、不需要 pyserial**）。

跑完打印每一条的结果，全过退出码 0，有红的退出码 1。

验的东西分三类：

1. **和板子对拍** —— CRC 参考值、版本码、四条整帧逐字节锚点（这几条是板上
   `FrameEncode()` 的真实输出，用 host 程序编 `libs/frame/frame.c` 打出来抄的），
   以及**直接 parse 固件头文件**核对码表 / 软限位 / 状态个数 / ARM_NUM。
   固件改了而 SDK 没跟上的话，这里会红。
2. **协议层自身** —— 每个帧型的编解码往返、解析器的重同步（粘包 / 前导噪声 / 半帧 /
   坏 CRC 后紧跟好帧 / payload 里含 AA55）、按 arm_id 分流不串台。
3. **SDK 语义** —— 单位换算往返、夹爪标定模型（含未标定抛异常）、
   以及**闸门矩阵**：每个命令在什么状态下该被客户端预检挡下、该发出什么 tid 和 payload、
   `force=True` 会不会真的绕过、以及"**构造完什么都不发**"（证明 SDK 不自动保心跳）。
4. **假板子的模拟状态机**（`sdk.sim.SimLink`）—— 握手闸门（版本码不符就一个字节都不回）、
   `0→1→2` 时序、`0x02/0x10/0x18` 的闸门与时序、两台互不串台、500 ms 看门狗（全链路一份）、
   `--fake-lost`。

第 3 类用 `LoopbackLink`（继承 `Link`、只覆盖收发那三个方法）当替身：SDK 要测的是切帧、
分流、单位、闸门预检，全是包级可测的，不需要状态机。第 4 类才用完整假板子 —— 它现在也是
`--fake` 用的那一份（`sdk/sim.py`），所以不存在"测试的假板子和用户看的假板子不一样"。
"""

import math
import os
import re
import struct
import sys
import threading
import time
import traceback

from . import board_defs, gripper, protocol, units
from .arm import Arm
from .errors import (CalibrationError, FeedbackStale, GateRejected,
                     GripperNotCalibrated, NotConnected)
from .link import Link
from .protocol import (FB_LEN, SET_ZERO_ACK_LEN, TID_ARM_CTRL_POS,
                       TID_ARM_CTRL_POS_VEL, TID_ARM_DAMPING, TID_ARM_FEEDBACK,
                       TID_ARM_GRAVITY, TID_ARM_INIT, TID_ARM_RESET, TID_ARM_START,
                       TID_ARM_STOP, TID_MOTOR_SET_ZERO, TID_VERSION)

# ==================== 结果收集 ====================

OK, BAD, SKIP = "ok", "bad", "skip"


class Checker:
    def __init__(self):
        self.n_ok = 0
        self.n_bad = 0
        self.n_skip = 0
        self.failures = []

    def check(self, name, ok, detail=""):
        if ok:
            self.n_ok += 1
            print("  \033[32mok\033[0m   %s" % name)
        else:
            self.n_bad += 1
            self.failures.append((name, detail))
            print("  \033[31mFAIL\033[0m %s" % name)
            for line in str(detail).splitlines():
                print("         %s" % line)
        return bool(ok)

    def skip(self, name, why=""):
        self.n_skip += 1
        print("  \033[33mskip\033[0m %s%s" % (name, ("  —— %s" % why) if why else ""))

    def section(self, title):
        print("\n\033[1m%s\033[0m" % title)

    def summary(self):
        print("\n" + "-" * 68)
        print("通过 %d，失败 %d，跳过 %d" % (self.n_ok, self.n_bad, self.n_skip))
        if self.n_bad:
            print("\n失败项：")
            for name, _ in self.failures:
                print("  - %s" % name)
        return 1 if self.n_bad else 0


# ==================== 造帧 / 瘦回环替身 ====================

def feedback_payload(mode=board_defs.MODE_RUN, robot_state=board_defs.STATE_NORMAL,
                     lost=0x00, inited=0x7F, joint_deg=None, jvel_deg=None,
                     gripper_deg=-100.0, temps=None):
    """按 0x80 的布局拼 63 字节 payload —— 布局定义在 `protocol.encode_feedback`。"""
    joint_deg = joint_deg if joint_deg is not None else list(board_defs.RESET_POSE_DEG)
    jvel_deg = jvel_deg if jvel_deg is not None else [0.0] * 6
    temps = temps if temps is not None else list(range(36, 43))
    return protocol.encode_feedback(mode, robot_state, lost, inited,
                                    joint_deg, jvel_deg, gripper_deg, temps)


class LoopbackLink(Link):
    """瘦回环：不发串口（把帧记进 `writes`），反馈靠测试手动 `inject_*` 灌。

    只覆盖"会碰硬件"的那几处（`open` / `is_open` / `send`），其余（channel、分流、
    `wait_feedback`、判别码表的逻辑）**用的都是真实现** —— 所以它测的是真代码路径。
    """

    def __init__(self):
        super().__init__(port="<loopback>")
        self.writes = []

    def open(self, port=None):
        self.port = port or "<loopback>"
        self._alive = True
        self.link_up = True
        self.link_err = None
        return self

    @property
    def is_open(self):
        return True

    def close(self):
        self._alive = False
        self.link_up = False

    def send(self, tid, payload=b"", arm_id=protocol.DEFAULT_ARM_ID):
        self.writes.append((tid, arm_id, bytes(payload)))
        self.n_tx += 1
        return protocol.OVERHEAD + len(payload)

    # ---- 灌帧：走**真的**分帧 / 判别路径，只是字节不是从串口进来的 ----
    # `inject_bytes` 直接继承 `Link` 的真实现（切帧 + 分流 + 计数），不在这里另抄一份。

    def inject_feedback(self, arm_id=0, ts=None, **kw):
        """直接投一帧反馈进某个 arm_id 的收包槽（`ts` 可以给一个旧时间戳造"陈旧"）。"""
        fb = protocol.decode_feedback(feedback_payload(**kw))
        self.channel(arm_id).put_feedback(fb, time.monotonic() if ts is None else ts)
        return fb

    def inject_ack(self, arm_id=0, state=5, result=(2,) * 7, elapsed_ms=5527):
        ack = protocol.SetZeroAck(state=state, result=tuple(result), elapsed_ms=elapsed_ms)
        self.channel(arm_id).put_ack(ack, time.monotonic())
        return ack

    # ---- 断言辅助 ----

    def last_write(self):
        return self.writes[-1] if self.writes else None

    def clear(self):
        self.writes = []

    def tids(self):
        return [w[0] for w in self.writes]


def make_arm(link=None, arm_id=0, tmp_calib=None):
    """造一台臂，**用临时标定文件**（别碰用户真实的 sdk/config/gripper_calib.json）。"""
    store = gripper.GripperCalibStore(tmp_calib)
    return Arm(arm_id, link, calib_store=store)


# ==================== 1. CRC / 版本 ====================

def test_crc(c):
    c.section("1. CRC 与版本码")
    c.check("crc8 表 vs 位运算参考实现（256 个单字节 + 参考串）",
            all(protocol.crc8(bytes([i])) == protocol.crc8_ref(bytes([i]))
                for i in range(256))
            and protocol.crc8(b"123456789") == protocol.crc8_ref(b"123456789"))
    c.check("crc16 表 vs 位运算参考实现（256 个单字节 + 参考串）",
            all(protocol.crc16(bytes([i])) == protocol.crc16_ref(bytes([i]))
                for i in range(256))
            and protocol.crc16(b"123456789") == protocol.crc16_ref(b"123456789"))
    c.check("crc8(\"123456789\") == 0x0B", protocol.crc8(b"123456789") == 0x0B,
            "实际 0x%02X" % protocol.crc8(b"123456789"))
    c.check("crc16(\"123456789\") == 0x6F91", protocol.crc16(b"123456789") == 0x6F91,
            "实际 0x%04X" % protocol.crc16(b"123456789"))
    c.check("版本码 == 0xF91DD3C9", protocol.VERSION_CODE == 0xF91DD3C9,
            "实际 0x%08X（版本串 %s）" % (protocol.VERSION_CODE, protocol.VERSION_STRING))


# ==================== 2. 整帧锚点 ====================

def test_anchors(c):
    c.section("2. 整帧逐字节锚点（板上 FrameEncode 的真实输出）")
    cases = [
        ("版本帧 arm_id=0", protocol.encode(TID_VERSION, protocol.encode_version(), 0),
         "aa550104008ec9d31df91e20"),
        ("版本帧 arm_id=1（只差 arm_id 和两个 CRC）",
         protocol.encode(TID_VERSION, protocol.encode_version(), 1),
         "aa55010401d0c9d31df9ef30"),
        ("0x02 空包", protocol.encode(TID_ARM_INIT), "aa5502000051f088"),
        ("0x18 空包", protocol.encode(TID_ARM_RESET), "aa5518000071fdb6"),
    ]
    for name, got, want in cases:
        c.check(name, got.hex() == want, "实际 %s\n期望 %s" % (got.hex(), want))


# ==================== 3. 编解码往返 ====================

def test_roundtrip(c):
    c.section("3. 各帧型 编码 → 切帧 → 解字段")

    for tid, name in [(TID_VERSION, "0x01"), (TID_ARM_INIT, "0x02"),
                      (TID_MOTOR_SET_ZERO, "0x10"), (TID_ARM_START, "0x12"),
                      (TID_ARM_GRAVITY, "0x13"), (TID_ARM_STOP, "0x14"),
                      (TID_ARM_DAMPING, "0x15"), (TID_ARM_RESET, "0x18")]:
        ps = protocol.FrameParser()
        got = ps.feed(protocol.encode(tid, b"", 1))
        c.check("%s 空包往返（arm_id=1）" % name,
                len(got) == 1 and got[0] == (tid, 1, b""), "解析出 %r" % (got,))

    # 0x16：6 关节 + 夹爪，deg
    angles = [-12.5, 0.5, 30.25, 0.0, -45.5, 119.0, -141.75]
    ps = protocol.FrameParser()
    got = ps.feed(protocol.encode(TID_ARM_CTRL_POS, protocol.encode_ctrl_pos(angles)))
    ok = len(got) == 1 and got[0][:2] == (TID_ARM_CTRL_POS, 0) and len(got[0][2]) == 28
    vals = struct.unpack("<7f", got[0][2]) if ok else ()
    c.check("0x16 往返（28 字节，7 个 float）",
            ok and all(abs(a - b) < 1e-5 for a, b in zip(vals, angles)),
            "长度 %d / 值 %r" % (len(got[0][2]) if got else -1, vals))

    # 0x17：位置 + 速度前馈
    vels = [1.0, -2.0, 3.0, 0.0, 0.0, -0.5]
    ps = protocol.FrameParser()
    got = ps.feed(protocol.encode(TID_ARM_CTRL_POS_VEL,
                                  protocol.encode_ctrl_pos_vel(angles[:6], vels, -100.0)))
    ok = len(got) == 1 and len(got[0][2]) == 52
    vals = struct.unpack("<13f", got[0][2]) if ok else ()
    c.check("0x17 往返（52 字节，13 个 float）",
            ok and all(abs(a - b) < 1e-5 for a, b in zip(vals[:6], angles[:6]))
            and all(abs(a - b) < 1e-5 for a, b in zip(vals[6:12], vels))
            and abs(vals[12] - (-100.0)) < 1e-5 if ok else False,
            "长度 %d / 值 %r" % (len(got[0][2]) if got else -1, vals))

    # 0x80：四个 u8 + 6f + 6f + f + 7u8，**按 1 字节对齐读**（uint8 段和 float 段之间没有填充）
    want = dict(mode=1, robot_state=4, lost=0x01, inited=0x7F,
                joint_deg=[-12.5, 0.5, 30.25, 0.0, -45.5, 119.0],
                jvel_deg=[1.0, -2.0, 3.0, 0.0, 0.0, -0.5],
                gripper_deg=-141.75, temps=[36, 37, 38, 39, 40, 41, 42])
    pay = feedback_payload(**want)
    c.check("0x80 payload 恰好 63 字节", len(pay) == FB_LEN, "实际 %d" % len(pay))
    ps = protocol.FrameParser()
    got = ps.feed(protocol.encode(TID_ARM_FEEDBACK, pay, 1))
    fb = protocol.decode_feedback(got[0][2]) if got else None
    c.check("0x80 往返（字段逐个对上）",
            fb is not None and fb.mode == 1 and fb.robot_state == 4
            and fb.motor_lost_mask == 0x01 and fb.motor_inited_mask == 0x7F
            and all(abs(a - b) < 1e-5 for a, b in zip(fb.joint_angle, want["joint_deg"]))
            and all(abs(a - b) < 1e-5 for a, b in zip(fb.joint_vel, want["jvel_deg"]))
            and abs(fb.gripper_angle - want["gripper_deg"]) < 1e-5
            and fb.motor_temp == tuple(want["temps"]),
            "%r" % (fb,))

    # 0x11：u8 + u8[7] + u32
    ack_pay = bytes([5]) + bytes([2, 2, 1, 4, 2, 2, 5]) + struct.pack("<I", 5527)
    c.check("0x11 payload 恰好 12 字节", len(ack_pay) == SET_ZERO_ACK_LEN)
    ack = protocol.decode_setzero_ack(ack_pay)
    c.check("0x11 解出来的 state/result/elapsed_ms 对",
            ack.state == 5 and ack.result == (2, 2, 1, 4, 2, 2, 5) and ack.elapsed_ms == 5527,
            "%r" % (ack,))

    # 长度不对必须抛（而不是悄悄解出垃圾）
    for bad_len in (0, 62, 64):
        try:
            protocol.decode_feedback(b"\x00" * bad_len)
            c.check("0x80 长度 %d 要抛 ValueError" % bad_len, False, "没抛")
        except ValueError:
            c.check("0x80 长度 %d 要抛 ValueError" % bad_len, True)


# ==================== 4. 解析器重同步 ====================

def test_resync(c):
    c.section("4. 切帧 / 重同步")

    good = protocol.encode(TID_ARM_INIT, b"", 1)

    # 前导噪声
    ps = protocol.FrameParser()
    got = ps.feed(b"\x00\x11\x22\x33" + good)
    c.check("前导噪声：跳掉噪声、解出真帧",
            got == [(TID_ARM_INIT, 1, b"")] and ps.n_drop >= 4, "%r n_drop=%d" % (got, ps.n_drop))

    # AA55 拆在两次 feed 之间
    ps = protocol.FrameParser()
    got = ps.feed(b"\x01\x02\xAA") + ps.feed(b"\x55" + good[2:5]) + ps.feed(good[5:])
    c.check("AA 55 拆在两次 feed 之间",
            got == [(TID_ARM_INIT, 1, b"")], "%r" % (got,))

    # 尾部半帧
    ps = protocol.FrameParser()
    got = ps.feed(good + good[:4])
    c.check("尾部半帧先不解（凑齐了再解）",
            got == [(TID_ARM_INIT, 1, b"")], "%r" % (got,))

    # 坏 CRC 后紧跟好帧
    broken = bytearray(good)
    broken[5] ^= 0xFF          # 打坏 crc8
    ps = protocol.FrameParser()
    got = ps.feed(bytes(broken) + good)
    c.check("坏 crc8 后紧跟好帧：重同步成功、坏帧计数",
            got == [(TID_ARM_INIT, 1, b"")] and ps.n_err_crc8 == 1,
            "%r crc8 错 %d" % (got, ps.n_err_crc8))

    broken16 = bytearray(good)
    broken16[-1] ^= 0xFF       # 打坏 crc16
    ps = protocol.FrameParser()
    got = ps.feed(bytes(broken16) + good)
    c.check("坏 crc16 后紧跟好帧：重同步成功、坏帧计数",
            got == [(TID_ARM_INIT, 1, b"")] and ps.n_err_crc16 == 1,
            "%r crc16 错 %d" % (got, ps.n_err_crc16))

    # payload 里含 AA 55（数据区不做帧头搜索，这是"整帧丢掉"那种做法的坑）
    tricky = protocol.encode(TID_ARM_CTRL_POS,
                             protocol.encode_ctrl_pos([0, 0, 0, 0, 0, 0, 0])[:4]
                             + b"\xAA\x55" + b"\x00" * 22)
    ps = protocol.FrameParser()
    got = ps.feed(tricky + good)
    c.check("payload 里含 AA 55：两个帧都解出来",
            len(got) == 2 and got[0][0] == TID_ARM_CTRL_POS and got[1] == (TID_ARM_INIT, 1, b""),
            "%r" % (got,))

    # arm_id 0/1 混流，按序还原
    ps = protocol.FrameParser()
    stream = (protocol.encode(TID_ARM_INIT, b"", 0) + protocol.encode(TID_ARM_INIT, b"", 1)
              + protocol.encode(TID_ARM_RESET, b"", 1))
    got = ps.feed(stream)
    c.check("arm_id 0/1 混流还原",
            [(t, a) for t, a, _ in got] == [(TID_ARM_INIT, 0), (TID_ARM_INIT, 1),
                                            (TID_ARM_RESET, 1)], "%r" % (got,))

    # 一片一片喂（模拟真实串口的任意分片）
    blob = b"".join(protocol.encode(t, protocol.encode_version(), a)
                    for t, a in [(TID_VERSION, 0), (TID_VERSION, 1), (TID_VERSION, 0)])
    ps = protocol.FrameParser()
    got = []
    for i in range(len(blob)):
        got += ps.feed(blob[i:i + 1])
    c.check("按 1 字节一片喂也能全解出来", len(got) == 3, "解出 %d 帧" % len(got))


# ==================== 5. 按 arm_id 分流 ====================

def test_dispatch(c):
    c.section("5. 分流：一条链路、两台 Arm、互不串台")
    lb = LoopbackLink()
    a = make_arm(lb, 0)
    b = make_arm(lb, 1)

    lb.inject_bytes(protocol.encode(TID_ARM_FEEDBACK,
                                    feedback_payload(robot_state=board_defs.STATE_WAIT_INIT,
                                                     gripper_deg=10.0), 0))
    lb.inject_bytes(protocol.encode(TID_ARM_FEEDBACK,
                                    feedback_payload(robot_state=board_defs.STATE_NORMAL,
                                                     mode=board_defs.MODE_DAMPING,
                                                     gripper_deg=-20.0), 1))
    c.check("A 只看到自己那帧的 state",
            a.robot_state == board_defs.STATE_WAIT_INIT and b.robot_state == board_defs.STATE_NORMAL,
            "a=%s b=%s" % (a.robot_state, b.robot_state))
    c.check("A 只看到自己那帧的夹爪角",
            a.raw_gripper_angle_deg == 10.0 and b.raw_gripper_angle_deg == -20.0,
            "a=%s b=%s" % (a.raw_gripper_angle_deg, b.raw_gripper_angle_deg))
    c.check("两台各自数自己的帧数", a.n_feedback == 1 and b.n_feedback == 1)

    # 0x11 也只投给对应那一台
    lb.inject_ack(1, state=5, result=(2,) * 7)
    c.check("0x11 只投给 arm_id=1 那台",
            a.last_setzero_ack is None and b.last_setzero_ack is not None)

    # 越界 arm_id：按板子同口径丢弃 + 计数，**不进任何一台**
    before = (a.n_feedback, b.n_feedback)
    lb.inject_bytes(protocol.encode(TID_ARM_FEEDBACK, feedback_payload(), 7))
    c.check("越界 arm_id(7) 被丢弃并计数",
            lb.n_bad_arm == 1 and (a.n_feedback, b.n_feedback) == before,
            "n_bad_arm=%d" % lb.n_bad_arm)

    # 认不出来的 tid：计数、不报错（板子以后加包不该让老上位机炸）
    lb.inject_bytes(protocol.encode(0x7A, b"\x01\x02", 1))
    c.check("未知 tid 只计数、不抛",
            lb.channel(1).unknown_tids.get(0x7A) == 1 and a.n_feedback == 1,
            "unknown=%r" % (lb.channel(1).unknown_tids,))

    # 版本包落在链路上
    lb.inject_bytes(protocol.encode(TID_VERSION, protocol.encode_version(), 0))
    c.check("0x01 版本包记在链路上",
            lb.board_version == protocol.VERSION_CODE and lb.version_seen)

    # Link.arm() 缓存
    c.check("Link.arm(0) 同一个 arm_id 只建一个对象", lb.arm(0) is lb.arm(0))

    # 口子闸门
    try:
        Link().channel(9)
        c.check("arm_id 越界（>= ARM_NUM）在本地就报错", False, "没抛")
    except ValueError:
        c.check("arm_id 越界（>= ARM_NUM）在本地就报错", True)


# ==================== 6. 单位换算 ====================

def test_units(c):
    c.section("6. 单位换算往返")
    c.check("deg↔rad 往返", all(abs(units.rad_to_deg(units.deg_to_rad(v)) - v) < 1e-9
                               for v in (-180, -33.3, 0, 0.5, 206.26)))
    c.check("deg/s↔rad/s 往返",
            all(abs(units.rad_s_to_deg_s(units.deg_s_to_rad_s(v)) - v) < 1e-9
                for v in (-100, 0, 12.5)))
    c.check("rad↔deg 的常数是 57.295779…",
            abs(units.RAD2DEG - 57.29577951308232) < 1e-12, "实际 %r" % units.RAD2DEG)
    c.check("序列换算保持长度与顺序",
            units.deg_seq_to_rad([1.0, 2.0]) == (units.deg_to_rad(1.0), units.deg_to_rad(2.0)))
    for bad in ([1, 2, 3], [1] * 7, 5, "123456"):
        try:
            units.check_joint_seq(bad)
            c.check("check_joint_seq 拒掉 %r" % (bad,), False, "没抛")
        except (TypeError, ValueError):
            c.check("check_joint_seq 拒掉 %r" % (bad,), True)

    # 板侧软限位镜像也该能和 rad 版本对上
    c.check("限位的 rad 镜像和 deg 版一致",
            all(abs(math.degrees(r) - d) < 1e-9
                for r, d in zip(board_defs.JOINT_ANGLE_MIN_RAD, board_defs.JOINT_ANGLE_MIN)))
    lo, clipped = board_defs.clamp_axis(0, -1000.0)
    c.check("clamp_axis 夹住并报告夹了",
            clipped and lo == board_defs.JOINT_ANGLE_MIN[0])
    v, clipped = board_defs.clamp_axis(0, 0.0)
    c.check("clamp_axis 在范围内原样返回且不报夹", (not clipped) and v == 0.0)


# ==================== 7. 夹爪标定 ====================

def test_gripper(c, tmpdir):
    c.section("7. 夹爪两点标定")

    # 找零之后的口径：闭合端 +142、张开端 −142（用户给的两点，方向是"越负越张开"）
    calib = gripper.GripperCalibration.from_points(
        full_open_m=0.085, deg_at_zero=142.0, deg_at_full_open=-141.5)
    c.check("记录点 A 处 = 0 m", abs(calib.opening_from_raw_deg(142.0)) < 1e-12)
    c.check("记录点 B 处 = FULL_OPEN", abs(calib.opening_from_raw_deg(-141.5) - 0.085) < 1e-12)
    c.check("中点 = 一半行程",
            abs(calib.opening_from_raw_deg(0.25) - 0.085 / 2) < 1e-9)
    c.check("反算 raw deg 往返",
            all(abs(calib.opening_from_raw_deg(calib.raw_deg_from_opening(m)) - m) < 1e-12
                for m in (0.0, 0.01, 0.0425, 0.085)))
    c.check("方向识别：越负越张开 → direction=-1", calib.direction == -1)
    c.check("两点之外是线性外推（不夹住）",
            calib.opening_from_raw_deg(-142.0) > 0.085)

    # 装反了（越正越张开）也自洽 —— 方向由记录点决定，代码不预设
    flipped = gripper.GripperCalibration.from_points(
        full_open_m=0.06, deg_at_zero=-142.0, deg_at_full_open=142.0)
    c.check("装反的夹爪也能标（direction=+1）",
            flipped.direction == 1 and abs(flipped.opening_from_raw_deg(142.0) - 0.06) < 1e-12)

    # 退化标定要拦
    for bad_kw, why in [
        (dict(full_open_m=0.0, deg_at_zero=0.0, deg_at_full_open=-142.0), "FULL_OPEN=0"),
        (dict(full_open_m=-0.01, deg_at_zero=0.0, deg_at_full_open=-142.0), "FULL_OPEN<0"),
        (dict(full_open_m=0.08, deg_at_zero=1.0, deg_at_full_open=1.0), "两点重合"),
    ]:
        try:
            gripper.GripperCalibration.from_points(**bad_kw)
            c.check("拒掉退化标定（%s）" % why, False, "没抛")
        except CalibrationError:
            c.check("拒掉退化标定（%s）" % why, True)

    # 存盘 / 读回 / 原子写
    path = os.path.join(str(tmpdir), "sub", "gripper_calib.json")
    store = gripper.GripperCalibStore(path)
    c.check("文件还不存在时 get() 返回 None（不是错误）", store.get(0) is None)
    store.put(0, calib)
    store.put(1, flipped)
    c.check("落盘后两台都读得回来",
            store.get(0) == calib and store.get(1) == flipped,
            "%r / %r" % (store.get(0), store.get(1)))
    c.check("写一台不会覆盖另一台", store.get(1).full_open_m == 0.06)
    c.check("覆盖写同一台也正常",
            (store.put(0, gripper.GripperCalibration.from_points(0.1, 142.0, -142.0)),
             abs(store.get(0).full_open_m - 0.1) < 1e-12)[1])
    c.check("临时文件没留下", not any(n.endswith(".tmp") for n in os.listdir(os.path.dirname(path))))

    # 坏文件要抛（不静默当"没标定"）
    bad = os.path.join(str(tmpdir), "bad.json")
    with open(bad, "w") as f:
        f.write("{ 这不是 json")
    try:
        gripper.GripperCalibStore(bad).get(0)
        c.check("坏 JSON 抛 CalibrationError", False, "没抛")
    except CalibrationError:
        c.check("坏 JSON 抛 CalibrationError", True)

    # ---- Arm 上：未标定抛、标定后出数、raw 一直可读 ----
    # 用**另一个文件**：上面那份已经被写过了，拿它验不出"未标定"。
    arm_path = os.path.join(str(tmpdir), "arm_calib.json")
    arm_store = gripper.GripperCalibStore(arm_path)
    lb = LoopbackLink()
    arm = make_arm(lb, 0, tmp_calib=arm_path)
    lb.inject_feedback(0, gripper_deg=0.0, inited=0x7F)
    c.check("未标定的臂：snapshot 里 opening=None 且 calibrated=False",
            arm.snapshot().gripper_opening is None
            and arm.snapshot().gripper_calibrated is False)
    c.check("未标定的臂：raw 原始角照常可读", arm.raw_gripper_angle_deg == 0.0)
    try:
        arm.gripper_opening
        c.check("未标定时 gripper_opening 抛 GripperNotCalibrated", False, "没抛")
    except GripperNotCalibrated as e:
        c.check("未标定时 gripper_opening 抛 GripperNotCalibrated",
                "calibrate_gripper" in str(e))

    arm_store.put(0, calib)
    c.check("reload 之前还是未标定状态（有缓存）", arm.calibration is None)
    arm.reload_calibration()
    c.check("reload 之后 arm 0 有标定了", arm.calibration == calib)
    lb.inject_feedback(0, robot_state=board_defs.STATE_NORMAL, mode=board_defs.MODE_RUN,
                       gripper_deg=0.25, inited=0x7F)
    c.check("标定后 gripper_opening 出数（raw 0.25° → 半个行程）",
            abs(arm.gripper_opening - 0.085 / 2) < 1e-9, "实际 %r" % arm.gripper_opening)

    # ---- send_ctrl_pos 的夹爪两条路：m 走标定、raw deg 直给、都不给则保持当前 ----
    play = make_arm(lb, 0, tmp_calib=arm_path)
    lb.inject_feedback(0, robot_state=board_defs.STATE_NORMAL, mode=board_defs.MODE_RUN,
                       gripper_deg=0.0)
    play.send_ctrl_pos([0.0] * 6, gripper_m=0.085)
    tid, aid, pay = lb.last_write()
    c.check("gripper_m= 走标定换算成 raw deg",
            tid == TID_ARM_CTRL_POS and aid == 0 and len(pay) == 28
            and abs(struct.unpack("<7f", pay)[6] - (-141.5)) < 1e-4,
            "写出的夹爪角 %r" % (struct.unpack("<7f", pay)[6] if len(pay) == 28 else None))

    lb.clear()
    play.send_ctrl_pos([0.0] * 6, gripper_raw_deg=-100.0)
    c.check("gripper_raw_deg= 直给、不过标定",
            abs(struct.unpack("<7f", lb.last_write()[2])[6] - (-100.0)) < 1e-5)

    lb.clear()
    play.send_ctrl_pos([0.0] * 6)     # 都不给 → 保持反馈里的当前角（0.0）
    c.check("夹爪两个都不给 → 保持当前（取反馈里的原始角）",
            abs(struct.unpack("<7f", lb.last_write()[2])[6] - 0.0) < 1e-6)

    try:
        play.send_ctrl_pos([0.0] * 6, gripper_m=0.05, gripper_raw_deg=-10.0)
        c.check("夹爪 m 和 raw_deg 同时给 → ValueError", False, "没抛")
    except ValueError:
        c.check("夹爪 m 和 raw_deg 同时给 → ValueError", True)


# ==================== 8. 闸门矩阵 ====================

def gate_case(name, lb, call, tid=None, payload_len=None, exc=None):
    """跑一个命令，断言"该被挡 / 该发出什么"。返回 None 表示这条过了。"""
    lb.clear()
    try:
        call()
    except Exception as e:      # noqa: BLE001  —— 这里就是要看抛了什么
        if exc is None:
            return ("%s：不该抛，抛了 %s: %s" % (name, type(e).__name__, e))
        if not isinstance(e, exc):
            return ("%s：抛的是 %s（期望 %s）：%s" % (name, type(e).__name__, exc.__name__, e))
        if lb.writes:
            return "%s：被挡下了但**竟然写出去了** %r" % (name, lb.writes)
        return None
    if exc is not None:
        return "%s：该抛 %s，结果发出去了（%r）" % (name, exc.__name__, lb.writes)
    if not lb.writes:
        return "%s：该发帧，结果一个字节都没写" % name
    got_tid, _, got_pay = lb.last_write()
    if tid is not None and got_tid != tid:
        return "%s：tid 写成了 0x%02X（期望 0x%02X）" % (name, got_tid, tid)
    if payload_len is not None and len(got_pay) != payload_len:
        return "%s：payload 长 %d（期望 %d）" % (name, len(got_pay), payload_len)
    return None


def test_gates(c, tmpdir):
    """闸门矩阵。

    ⚠ 每一条都要**先灌上那一帧反馈、紧接着就断言** —— 所有用例共用同一个收包槽，
    先攒一批 case 再统一跑的话，它们看到的全是最后一帧反馈（踩过这个坑）。
    """
    c.section("8. 闸门矩阵（客户端预检）")
    lb = LoopbackLink()
    arm = make_arm(lb, 0, tmp_calib=os.path.join(str(tmpdir), "no_calib.json"))

    def fb(state, mode=board_defs.MODE_RUN, lost=0x00, joint=None, gripper_deg=0.0):
        lb.inject_feedback(0, robot_state=state, mode=mode, lost=lost,
                           joint_deg=joint, gripper_deg=gripper_deg)

    def case(name, call, tid=None, plen=None, exc=None):
        bad = gate_case(name, lb, call, tid, plen, exc)
        c.check(name, bad is None, bad or "")

    ST = board_defs
    # ---- 0x02 init_arm ----
    fb(ST.STATE_WAIT_INIT, ST.MODE_STOP)
    case("init_arm 在待初始化：放行", arm.init_arm, TID_ARM_INIT, 0)
    fb(ST.STATE_NORMAL, ST.MODE_STOP)
    case("init_arm 在正常运行+stop：放行（第二入口）", arm.init_arm, TID_ARM_INIT, 0)
    fb(ST.STATE_NORMAL, ST.MODE_DAMPING)
    case("init_arm 在正常运行+damping：放行", arm.init_arm, TID_ARM_INIT, 0)
    fb(ST.STATE_NORMAL, ST.MODE_RUN)
    case("init_arm 在正常运行+run：挡（位置环在跑）", arm.init_arm, exc=GateRejected)
    fb(ST.STATE_NORMAL, ST.MODE_GRAVITY)
    case("init_arm 在正常运行+gravity：挡", arm.init_arm, exc=GateRejected)
    fb(ST.STATE_WAIT_HANDSHAKE)
    case("init_arm 在等待握手：挡", arm.init_arm, exc=GateRejected)
    fb(ST.STATE_WAIT_INIT, ST.MODE_STOP, lost=0x01)
    case("init_arm 有电机掉线：挡", arm.init_arm, exc=GateRejected)
    fb(ST.STATE_WAIT_INIT, ST.MODE_STOP, joint=[30.0, 0.5, 0.5, 0, 0, 0])
    case("init_arm 姿态偏 30°：挡", arm.init_arm, exc=GateRejected)
    fb(ST.STATE_WAIT_INIT, ST.MODE_STOP, joint=[19.0, 0.5, 0.5, 0, 0, 0])
    case("init_arm 姿态偏 19°（容差 20°）：放行", arm.init_arm, TID_ARM_INIT, 0)
    fb(ST.STATE_WAIT_HANDSHAKE, ST.MODE_RUN)
    case("init_arm force=True：绕过预检", lambda: arm.init_arm(force=True), TID_ARM_INIT, 0)

    # ---- 0x10 set_zero ----
    fb(ST.STATE_WAIT_INIT, ST.MODE_STOP)
    case("set_zero 在待初始化：放行", arm.set_zero, TID_MOTOR_SET_ZERO, 0)
    fb(ST.STATE_NORMAL)
    case("set_zero 在正常运行：挡（只能从待初始化）", arm.set_zero, exc=GateRejected)

    # ---- 0x12/13/14/15 模式切换 ----
    for state, blocked in [(ST.STATE_NORMAL, False), (ST.STATE_WAIT_INIT, True)]:
        fb(state)
        for meth, tid, nm in [("start", TID_ARM_START, "start"),
                              ("gravity", TID_ARM_GRAVITY, "gravity"),
                              ("stop", TID_ARM_STOP, "stop"),
                              ("damping", TID_ARM_DAMPING, "damping")]:
            case("%s 在%s：%s" % (nm, ST.state_name(state), "挡" if blocked else "放行"),
                 getattr(arm, meth),
                 exc=GateRejected if blocked else None,
                 tid=None if blocked else tid, plen=None if blocked else 0)

    # ---- 0x16 / 0x17 控制帧 ----
    fb(ST.STATE_NORMAL, ST.MODE_RUN)
    case("send_ctrl_pos 在 正常运行+run：放行（28 字节）",
         lambda: arm.send_ctrl_pos([0.1] * 6), TID_ARM_CTRL_POS, 28)
    case("send_ctrl_pos_vel 在 正常运行+run：放行（52 字节）",
         lambda: arm.send_ctrl_pos_vel([0.1] * 6, [0.0] * 6), TID_ARM_CTRL_POS_VEL, 52)
    fb(ST.STATE_NORMAL, ST.MODE_STOP)
    case("send_ctrl_pos 在 mode=stop：挡（板子只要 run 档的控制帧）",
         lambda: arm.send_ctrl_pos([0.1] * 6), exc=GateRejected)
    fb(ST.STATE_WAIT_INIT, ST.MODE_RUN)
    case("send_ctrl_pos 在待初始化：挡",
         lambda: arm.send_ctrl_pos([0.1] * 6), exc=GateRejected)
    case("send_ctrl_pos force=True：绕过预检",
         lambda: arm.send_ctrl_pos([0.1] * 6, force=True), TID_ARM_CTRL_POS, 28)
    case("send_ctrl_pos 关节数不对：本地就报错（参数错误，不是闸门）",
         lambda: arm.send_ctrl_pos([0.1] * 5, force=True), exc=ValueError)

    # ---- 0x18 reset ----
    fb(ST.STATE_NORMAL, ST.MODE_RUN)
    case("reset 在正常运行且全在线：放行", arm.reset, TID_ARM_RESET, 0)
    fb(ST.STATE_NORMAL, ST.MODE_RUN, lost=0x40)
    case("reset 有电机掉线：挡", arm.reset, exc=GateRejected)
    fb(ST.STATE_WAIT_INIT)
    case("reset 在待初始化：挡", arm.reset, exc=GateRejected)
    fb(ST.STATE_NORMAL, ST.MODE_RUN, joint=[60.0] * 6)
    case("reset 姿态很歪：**放行**（0x18 故意不查姿态）", arm.reset, TID_ARM_RESET, 0)

    # ---- 反馈陈旧 / 完全没有反馈 ----
    lb2 = LoopbackLink()
    arm2 = make_arm(lb2, 0, tmp_calib=os.path.join(str(tmpdir), "no_calib.json"))
    bad = gate_case("一帧反馈都没有时命令要抛 FeedbackStale", lb2, arm2.start,
                    exc=FeedbackStale)
    c.check("一帧反馈都没有时命令要抛 FeedbackStale", bad is None, bad or "")

    lb2.inject_feedback(0, ts=time.monotonic() - 1.0, robot_state=ST.STATE_NORMAL)
    bad = gate_case("反馈帧 1 s 旧（阈值 0.1 s）要抛 FeedbackStale", lb2, arm2.start,
                    exc=FeedbackStale)
    c.check("反馈帧 1 s 旧（阈值 0.1 s）要抛 FeedbackStale", bad is None, bad or "")
    lb2.clear()
    arm2.start(force=True)
    c.check("陈旧反馈下 force=True 照样能发", lb2.tids() == [TID_ARM_START])

    # ---- 不自动保活 ----
    lb3 = LoopbackLink()
    arm3 = make_arm(lb3, 0, tmp_calib=os.path.join(str(tmpdir), "no_calib.json"))
    c.check("构造 Arm 本身不写任何帧", lb3.writes == [])
    lb3.inject_feedback(0, robot_state=ST.STATE_NORMAL)
    time.sleep(0.15)             # 板子的 500 ms 看门狗窗口的一小截
    c.check("**空闲 150 ms 零写入**（证明 SDK 不自动发心跳）", lb3.writes == [],
            "写出了 %r" % (lb3.writes,))
    arm3.keepalive()
    c.check("keepalive() 发的是 0x01 且 payload 是版本码",
            lb3.last_write()[:2] == (TID_VERSION, 0)
            and protocol.decode_version(lb3.last_write()[2]) == protocol.VERSION_CODE)

    # ---- 等待 / 序号 ----
    # wait_feedback 取的是**调用那一刻**的序号，等的是"比手上这帧还新的"，
    # 所以先注入再等是等不到的 —— 要造"等的时候恰好来了一帧"，只能另开一根线程灌。
    c.check("手上这帧之后没有新帧 → wait_feedback 超时返回 None",
            arm3.wait_feedback(timeout=0.05) is None)

    seq_before = lb3.channel(0).snapshot_feedback()[2]
    threading.Thread(target=lambda: (time.sleep(0.05),
                                     lb3.inject_feedback(0, robot_state=ST.STATE_WAIT_INIT)),
                     daemon=True).start()
    fb_new = arm3.wait_feedback(timeout=1.0)
    c.check("等待期间来了新帧 → wait_feedback 拿到它（轮询真的在等）",
            fb_new is not None and fb_new.robot_state == ST.STATE_WAIT_INIT,
            "拿到 %r" % (fb_new,))
    c.check("...而且确实是**序号更大**的那一帧（不是手上那帧重放）",
            lb3.channel(0).snapshot_feedback()[2] > seq_before)
    c.check("wait_for 谓词为真时返回 True",
            arm3.wait_for(lambda a: a.robot_state == ST.STATE_WAIT_INIT, timeout=0.5))
    c.check("wait_for 超时返回 False（不抛）",
            arm3.wait_for(lambda a: a.robot_state == 99, timeout=0.05) is False)

    lb3.clear()
    arm3.set_zero(force=True)
    try:
        arm3.wait_setzero_ack(timeout=0.1)
        c.check("没有 0x11 时 wait_setzero_ack 抛 TimeoutError", False, "没抛")
    except TimeoutError:
        c.check("没有 0x11 时 wait_setzero_ack 抛 TimeoutError", True)
    lb3.inject_ack(0, state=5, result=(2, 2, 1, 4, 2, 2, 5), elapsed_ms=5527)
    c.check("0x11 来了 wait_setzero_ack 拿到结果",
            arm3.wait_setzero_ack(timeout=0.5).elapsed_ms == 5527)

    # 上一次的 0x11 不该被当成这一轮的结果（按序号等）
    arm3.set_zero(force=True)
    try:
        arm3.wait_setzero_ack(timeout=0.15)
        c.check("set_zero 后的 wait 不会拿**上一轮的** 0x11 顶数", False, "拿到了旧的")
    except TimeoutError:
        c.check("set_zero 后的 wait 不会拿**上一轮的** 0x11 顶数", True)

    # ---- send_raw / 未连接 ----
    lb3.clear()
    arm3.send_raw(0x7A, b"\x01\x02\x03")
    c.check("send_raw 原样透传 tid 和 payload",
            lb3.last_write() == (0x7A, 0, b"\x01\x02\x03"))
    try:
        arm3.send_raw(0x7A, "not bytes")
        c.check("send_raw 的 payload 不是 bytes 要报 TypeError", False, "没抛")
    except TypeError:
        c.check("send_raw 的 payload 不是 bytes 要报 TypeError", True)

    naive = Arm(0)     # 自建链路，没 open
    try:
        naive.send_version()
        c.check("链路没 open 时发送抛 NotConnected", False, "没抛")
    except NotConnected:
        c.check("链路没 open 时发送抛 NotConnected", True)


# ==================== 9. 固件头文件对拍 ====================

def repo_root():
    """从本文件往上找到仓库根（以 `libs/frame/type_id.h` 为标志）。找不到返回 None。"""
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(6):
        if os.path.isfile(os.path.join(d, "libs", "frame", "type_id.h")):
            return d
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    return None


def _read(path):
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            return f.read()
    except OSError:
        return None


def _num(token):
    """头文件里的一个数：去掉 f 后缀，允许简单四则（`284.0/2.0` 这种）。"""
    t = token.strip().rstrip("fF")
    t = re.sub(r"[^0-9eE+\-*/.() ]", "", t).strip()
    return float(eval(t, {"__builtins__": {}}, {}))   # noqa: S307 —— 只喂我们自己的头文件


def _int(token):
    """头文件里的一个整型常量：去掉 C 的 `U`/`L` 后缀，支持 `0x` / 十进制。"""
    return int(re.sub(r"[uUlL]+$", "", token.strip()), 0)


def _array(text, name):
    m = re.search(re.escape(name) + r"\s*(?:\[[^\]]*\])?\s*=\s*\{(.*?)\}", text, re.S)
    if not m:
        return None
    body = re.sub(r"//[^\n]*", "", m.group(1))       # 去掉行尾注释
    return [_num(p) for p in body.split(",") if p.strip()]


def _define(text, name):
    """`#define NAME 值` 里的值（不是 `#define` 的话返回 None）。"""
    m = re.search(r"^#define\s+" + re.escape(name) + r"\s+([^\s/]+)", text, re.M)
    return m.group(1) if m else None


def _const(text, name):
    """`CONSTEXPR float NAME = 值;` 里的值（本仓库的标定量不少是这么写的，不是 `#define`）。"""
    m = re.search(r"CONSTEXPR\s+\w+\s+" + re.escape(name) + r"\s*=\s*([^;]+);", text)
    return m.group(1).strip() if m else None


def _scalar(text, name):
    """两种写法都认 —— 标定量是 `#define` 还是 `CONSTEXPR`，对拍的人不该关心。"""
    return _define(text, name) or _const(text, name)


def _enum_body(text, enum_name):
    """取 `typedef enum { ... } <enum_name>;` 的花括号里那一坨（去注释）。

    花括号里用 `[^}]*` 而不是 `.*?` —— 否则会从文件里**前面**那个 enum 的 `{` 一路吞过来。
    """
    m = re.search(r"typedef\s+enum\s*\{([^}]*)\}\s*" + re.escape(enum_name) + r"\s*;", text)
    if not m:
        return None
    body = re.sub(r"/\*.*?\*/", " ", m.group(1), flags=re.S)
    return re.sub(r"//[^\n]*", "", body)


def _enum_names(text, enum_name):
    body = _enum_body(text, enum_name)
    if body is None:
        return None
    return [p.split("=")[0].strip() for p in body.split(",") if p.strip()]


# ==================== 9. 假板子 ====================

def _sim_pump(arms, seconds, extra=None):
    """按真实时间泵保活。假板子的看门狗走的是**它的**虚拟时钟，`speed>1` 时真实等待很短。"""
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        for a in arms:
            a.keepalive()
        if extra is not None:
            extra()
        time.sleep(0.005)


def _sim_wait(arms, pred, timeout=2.0, extra=None):
    """泵保活直到 `pred()` 为真；超时再判一次并返回结果。"""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        for a in arms:
            a.keepalive()
        if extra is not None:
            extra()
        if pred():
            return True
        time.sleep(0.005)
    return pred()


def test_sim(c, tmpdir):
    """`sdk.sim` 的模拟状态机：闸门、时序、两台互不串台、看门狗、--fake-lost。"""
    c.section("9. 假板子（sdk.sim，不接硬件）")
    from .sim import SimLink

    BD = board_defs
    link = SimLink(speed=8.0)      # speed=8：虚拟时钟 8× 真实时间，自测不用真等 3 秒
    link.open()
    try:
        a, b = link.arm(0), link.arm(1)

        # ---- 握手：真板子握手前一个字节都不发，这里照抄 ----
        # **这一段不能泵保活** —— keepalive() 发的就是 0x01 版本包，泵了就等于握手了。
        time.sleep(0.3)
        c.check("没收到版本包之前一帧反馈都不发（板子握手前不发）",
                a.n_feedback == 0 and b.n_feedback == 0,
                "实际 A=%d B=%d" % (a.n_feedback, b.n_feedback))
        link.send(TID_VERSION, struct.pack("<I", 0xDEADBEEF), 0)
        time.sleep(0.3)
        c.check("版本码不符 → 静默锁死、一个字节都不回",
                a.n_feedback == 0 and link.board_version is None,
                "实际 A=%d 板子版本 %r" % (a.n_feedback, link.board_version))

        a.send_version()
        b.send_version()
        ok = _sim_wait([a, b], lambda: a.robot_state == BD.STATE_WAIT_INIT
                       and b.robot_state == BD.STATE_WAIT_INIT, 2.0)
        c.check("版本码对上 → 等待握手→离线等待→待初始化，两台都到",
                ok and a.n_feedback > 0 and b.n_feedback > 0,
                "A=%s B=%s" % (a.robot_state_name, b.robot_state_name))

        # ---- 0x02 初始化 ----
        a.init_arm()
        ok = _sim_wait([a, b], lambda: a.robot_state == BD.STATE_NORMAL, 2.0)
        c.check("0x02 → 初始化中 → 正常运行，收尾停在 damping、inited 回 0x7F",
                ok and a.mode == BD.MODE_DAMPING and a.motor_inited_mask == 0x7F,
                "state=%s mode=%s inited=0x%02X" % (a.robot_state_name, a.mode_name,
                                                    a.motor_inited_mask))
        c.check("另一台不受影响（还在待初始化/stop）",
                b.robot_state == BD.STATE_WAIT_INIT and b.mode == BD.MODE_STOP,
                "B=%s/%s" % (b.robot_state_name, b.mode_name))

        # ---- 0x10 设零：只在待初始化受理，结果走 0x11 ----
        try:
            a.set_zero()
            rejected = False
        except GateRejected:
            rejected = True
        c.check("0x10 在正常运行态被预检挡下（板子也只认待初始化）", rejected)
        b.set_zero()
        ok = _sim_wait([a, b], lambda: b.last_setzero_ack is not None, 2.0)
        ack = b.last_setzero_ack
        c.check("0x10 → 校准零点 → 回待初始化，并收到 0x11 反馈帧",
                ok and b.robot_state == BD.STATE_WAIT_INIT and ack is not None
                and ack.state == BD.ZERO_STATE_DONE,
                "ack=%r state=%s" % (ack, b.robot_state_name))
        c.check("0x11 的 result 逐台给出（夹爪 1=未确认、其它 2/5）",
                ack is not None and ack.result == (5, 5, 5, 2, 2, 2, 1),
                "result=%r" % ((ack.result if ack else None),))

        # ---- 0x16 位控：只在正常运行 + run 档被受理，之后按一阶追目标 ----
        a.start()
        ok = _sim_wait([a, b], lambda: a.mode == BD.MODE_RUN, 1.0)
        want = [0.2, 0.5, 0.5, 0.0, 0.0, 0.0]
        _sim_wait([a, b], lambda: False, 0.1,
                  extra=lambda: a.send_ctrl_pos(want, gripper_raw_deg=0.0))
        c.check("0x16 在 run 档被受理、关节按一阶追到目标角",
                ok and abs(a.joint_angles[0] - want[0]) < 0.01
                and abs(a.joint_angles[1] - want[1]) < 0.01,
                "实际 %s" % ([round(v, 4) for v in a.joint_angles],))

        # ---- 0x18 复位：2 s 轨迹 → 正常运行 + damping，关节落到复位位姿 ----
        a.reset()
        ok = _sim_wait([a, b], lambda: a.robot_state == BD.STATE_RESETTING, 1.0)
        ok = ok and _sim_wait([a, b], lambda: a.robot_state == BD.STATE_NORMAL, 2.0)
        c.check("0x18 → 复位中 → 正常运行(damping)，关节落到复位位姿",
                ok and a.mode == BD.MODE_DAMPING
                and max(abs(x - y) for x, y in zip(a.joint_angles,
                                                   BD.RESET_POSE_RAD)) < 1e-6,
                "state=%s mode=%s joints=%s" % (a.robot_state_name, a.mode_name,
                                                [round(v, 4) for v in a.joint_angles]))

        # ---- 500 ms 看门狗：**全链路一份**，不发帧两台一起掉 ----
        time.sleep(0.3)      # 不泵保活（虚拟时钟 8× ⇒ 这段抵 2.4 s）
        c.check("不发帧超 500 ms → 两台一起进通信离线 + damping",
                a.robot_state == BD.STATE_COMM_LOST and b.robot_state == BD.STATE_COMM_LOST
                and a.mode == BD.MODE_DAMPING and b.mode == BD.MODE_DAMPING,
                "A=%s/%s B=%s/%s" % (a.robot_state_name, a.mode_name,
                                     b.robot_state_name, b.mode_name))
        ok = _sim_wait([a, b], lambda: a.robot_state == BD.STATE_WAIT_INIT, 2.0)
        c.check("链路回来落点是**离线等待→待初始化**（不会自己回正常运行）", ok,
                "A=%s" % a.robot_state_name)
    finally:
        link.close()

    # ---- --fake-lost：只有指定的那台掉线 ----
    link = SimLink(lost_mask=0x41, lost_arm=1, speed=8.0)
    link.open()
    try:
        a, b = link.arm(0), link.arm(1)
        a.send_version()
        b.send_version()
        _sim_wait([a, b], lambda: b.robot_state == BD.STATE_WAIT_INIT, 2.0)
        b.init_arm()
        _sim_wait([a, b], lambda: b.robot_state == BD.STATE_NORMAL, 2.0)
        ok = _sim_wait([a, b], lambda: b.motor_lost_mask != 0, 3.0)
        c.check("--fake-lost → 指定那台报掉线、mode 被打回 stop",
                ok and b.mode == BD.MODE_STOP
                and b.lost_names and a.motor_lost_mask == 0,
                "B lost=0x%02X %s / A lost=0x%02X"
                % (b.motor_lost_mask, b.lost_names, a.motor_lost_mask))
        ok = _sim_wait([a, b], lambda: b.motor_lost_mask == 0, 3.0)
        c.check("掉线自己恢复（掩码回 0）", ok, "B lost=0x%02X" % b.motor_lost_mask)
    finally:
        link.close()


def test_headers(c):
    c.section("10. 固件头文件对拍（仓库在的话）")
    root = repo_root()
    if root is None:
        c.skip("固件头文件对拍", "没找到仓库根（这是打包出去的副本吗？）")
        return

    # ---- 版本串 ----
    cfg = _read(os.path.join(root, "libs/frame/frame_config.h"))
    if cfg:
        s = protocol.parse_version_string(cfg)
        c.check("FRAME_VERSION_STRING 和 SDK 一致",
                s == protocol.VERSION_STRING, "固件里是 %r，SDK 是 %r" % (s, protocol.VERSION_STRING))

    # ---- 码表 ----
    tid_h = _read(os.path.join(root, "libs/frame/type_id.h"))
    if tid_h:
        pairs = [
            ("FRAME_TYPE_ID_VERSION", protocol.TID_VERSION),
            ("FRAME_TYPE_ID_ARM_INIT", protocol.TID_ARM_INIT),
            ("FRAME_TYPE_ID_MOTOR_SET_ZERO", protocol.TID_MOTOR_SET_ZERO),
            ("FRAME_TYPE_ID_MOTOR_SET_ZERO_ACK", protocol.TID_MOTOR_SET_ZERO_ACK),
            ("FRAME_TYPE_ID_ARM_START", protocol.TID_ARM_START),
            ("FRAME_TYPE_ID_ARM_GRAVITY", protocol.TID_ARM_GRAVITY),
            ("FRAME_TYPE_ID_ARM_STOP", protocol.TID_ARM_STOP),
            ("FRAME_TYPE_ID_ARM_DAMPING", protocol.TID_ARM_DAMPING),
            ("FRAME_TYPE_ID_ARM_CTRL_POS", protocol.TID_ARM_CTRL_POS),
            ("FRAME_TYPE_ID_ARM_CTRL_POS_VEL", protocol.TID_ARM_CTRL_POS_VEL),
            ("FRAME_TYPE_ID_ARM_RESET", protocol.TID_ARM_RESET),
            ("FRAME_TYPE_ID_ARM_FEEDBACK", protocol.TID_ARM_FEEDBACK),
        ]
        wrong = []
        for macro, val in pairs:
            got = _define(tid_h, macro)
            if got is None:
                wrong.append("%s 在头文件里没有" % macro)
            elif _int(got) != val:
                wrong.append("%s：头文件 %s，SDK 0x%02X" % (macro, got, val))
        c.check("码表 12 个 tid 全部对上", not wrong, "\n".join(wrong))

        # SDK 有的码板上也得有（防"SDK 自己编了个码"）
        known = {m for m, _ in pairs}
        missing = [m for m in re.findall(r"#define\s+(FRAME_TYPE_ID_\w+)", tid_h)
                   if m not in known and m != "FRAME_TYPE_ID_NONE"]
        c.check("板子上所有 FRAME_TYPE_ID_* 都在 SDK 里有对应", not missing,
                "SDK 漏了：%s" % missing)

        for macro, val in [("FRAME_PKT_JOINT_NUM", protocol.PKT_JOINT_NUM),
                           ("FRAME_PKT_MOTOR_NUM", protocol.PKT_MOTOR_NUM)]:
            got = _define(tid_h, macro)
            c.check("%s == %d" % (macro, val), got is not None and _int(got) == val,
                    "头文件里是 %s" % got)

    # ---- 帧头布局 ----
    frame_h = _read(os.path.join(root, "libs/frame/frame.h"))
    if frame_h:
        for macro, val in [("FRAME_HEADER_LEN", protocol.HEADER_LEN),
                           ("FRAME_TRAILER_LEN", 2),
                           ("FRAME_OVERHEAD", protocol.OVERHEAD),
                           ("FRAME_MAX_DATA", protocol.MAX_DATA),
                           ("FRAME_ARM_ID_OFFSET", protocol.ARM_ID_OFFSET)]:
            got = _define(frame_h, macro)
            c.check("%s == %d" % (macro, val), got is not None and _int(got) == val,
                    "头文件里是 %s" % got)
        c.check("MAX_FRAME == 263", protocol.MAX_FRAME == 263)

    # ---- 软限位 / 复位位姿 ----
    cmd_h = _read(os.path.join(root, "App/inc/gimbal_cmd_def.h"))
    if cmd_h:
        jmin = _array(cmd_h, "JOINT_ANGLE_MIN")
        jmax = _array(cmd_h, "JOINT_ANGLE_MAX")
        c.check("JOINT_ANGLE_MIN 和 SDK 一致",
                jmin == list(board_defs.JOINT_ANGLE_MIN), "头文件 %r" % (jmin,))
        c.check("JOINT_ANGLE_MAX 和 SDK 一致",
                jmax == list(board_defs.JOINT_ANGLE_MAX), "头文件 %r" % (jmax,))
        gmin = _scalar(cmd_h, "GRIPPER_ANGLE_MIN")
        gmax = _scalar(cmd_h, "GRIPPER_ANGLE_MAX")
        c.check("GRIPPER_ANGLE_MAX == +142", gmax and abs(_num(gmax) - board_defs.GRIPPER_ANGLE_MAX) < 1e-9,
                "头文件里是 %s" % gmax)
        c.check("GRIPPER_ANGLE_MIN == −142", gmin and abs(_num(gmin) - board_defs.GRIPPER_ANGLE_MIN) < 1e-9,
                "头文件里是 %s" % gmin)

    # 容差不在 arm_auto.h 里（那边只在注释里引它），真身在 robot_state.h。
    tol = _scalar(_read(os.path.join(root, "App/inc/robot_state.h")) or "",
                  "ROBOT_INIT_POSE_TOL_DEG")
    c.check("ROBOT_INIT_POSE_TOL_DEG 在 robot_state.h 里（SDK 记为 %.1f）"
            % board_defs.ROBOT_INIT_POSE_TOL_DEG,
            tol is not None and abs(_num(tol) - board_defs.ROBOT_INIT_POSE_TOL_DEG) < 1e-9,
            "头文件里是 %s" % tol)

    auto_c = _read(os.path.join(root, "App/arm_auto.c"))
    if auto_c:
        m = re.search(r"ARM_RESET_ANGLE\[[^\]]*\]\s*=\s*\{(.*?)\}", auto_c, re.S)
        if m and cmd_h:
            jmin = _array(cmd_h, "JOINT_ANGLE_MIN") or list(board_defs.JOINT_ANGLE_MIN)
            body = re.sub(r"//[^\n]*", "", m.group(1))
            vals = []
            for tok in body.split(","):
                tok = tok.strip()
                if not tok:
                    continue
                m2 = re.match(r"JOINT_ANGLE_MIN\[JOINT(\d)\]", tok)
                vals.append(jmin[int(m2.group(1)) - 1] if m2 else _num(tok))
            c.check("ARM_RESET_ANGLE == SDK 的 RESET_POSE_DEG",
                    vals == list(board_defs.RESET_POSE_DEG),
                    "固件拼出来是 %r" % (vals,))
    # ---- 拓扑 / 状态个数 ----
    def_h = _read(os.path.join(root, "App/inc/robot_def.h"))
    if def_h:
        got = _define(def_h, "ARM_NUM")
        c.check("ARM_NUM == %d" % board_defs.ARM_NUM,
                got is not None and _int(got) == board_defs.ARM_NUM, "头文件里是 %s" % got)
        modes = _enum_names(def_h, "Arm_mode")
        want = ["arm_stop", "arm_run", "arm_damping", "arm_only_gravity_comp"]
        c.check("Arm_mode 的顺序和 SDK 的 0/1/2/3 一致", modes == want,
                "固件是 %r" % (modes,))

    st_h = _read(os.path.join(root, "App/inc/robot_state.h"))
    if st_h:
        names = _enum_names(st_h, "Robot_state")
        if names:
            n_enum = len(names) - 1 if names and names[-1] == "ROBOT_STATE_NUM" else len(names)
            c.check("Robot_state 的个数和 SDK 的 STATE_NAME 一一对上",
                    n_enum == len(board_defs.STATE_NAME),
                    "固件 %d 个，SDK %d 个：%r" % (n_enum, len(board_defs.STATE_NAME), names))
            c.check("STATE_NAME 的键正好是 0..N-1",
                    sorted(board_defs.STATE_NAME) == list(range(len(board_defs.STATE_NAME))))
            # 名字逐项对上（顺序错位也不会报错的那种错，只能靠比字面量）
            want = ["robot_state_wait_handshake", "robot_state_offline_wait",
                    "robot_state_wait_init", "robot_state_initing", "robot_state_normal",
                    "robot_state_comm_lost", "robot_state_calibrating", "robot_state_resetting"]
            c.check("Robot_state 各项的名字和顺序都对得上",
                    names[:n_enum] == want, "固件是 %r" % (names,))

    # ---- 最终口径：和板子实时同步过的那些数 ----
    c.check("GRIPPER 索引 == ARM_JOINT_NUM（夹爪是第 7 台）",
            board_defs.GRIPPER == board_defs.ARM_JOINT_NUM == 6)
    c.check("ARM_MOTOR_NUM == 7", board_defs.ARM_MOTOR_NUM == 7)


# ==================== main ====================

def main(argv=None):
    import tempfile

    argv = sys.argv[1:] if argv is None else argv
    print("sdk 离线自测 —— 协议 %s（版本码 0x%08X）"
          % (protocol.VERSION_STRING, protocol.VERSION_CODE))
    print("不需要硬件、不需要 pyserial。")

    c = Checker()
    tests = [
        ("CRC 与版本码", lambda: test_crc(c)),
        ("整帧锚点", lambda: test_anchors(c)),
        ("编解码往返", lambda: test_roundtrip(c)),
        ("切帧/重同步", lambda: test_resync(c)),
        ("按 arm_id 分流", lambda: test_dispatch(c)),
        ("单位换算", lambda: test_units(c)),
    ]
    for name, fn in tests:
        fn()

    with tempfile.TemporaryDirectory(prefix="sdk-selftest-") as tmp:
        test_gripper(c, tmp)
        test_gates(c, tmp)
        test_sim(c, tmp)

    test_headers(c)
    return c.summary()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
    except Exception:      # noqa: BLE001 —— 自测自己崩了也要把栈打全，别吞
        traceback.print_exc()
        sys.exit(2)
