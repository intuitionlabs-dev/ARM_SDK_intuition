"""控制台侧离线自测：**不接硬件、不起 curses、不需要 pyserial**。

跑的是后端那一套 —— 两台臂的接线（一条链路、两个 `Arm`、各自的状态机）、单位与软限位、
步长档、闸门有没有把该拦的拦住、命令真的写出去了没有。用 `sdk.sim.SimLink` 当假板子，
走的是**真的**解析、分流、闸门。面板本身（curses 排版）不在这里测。

入口：`python3 -m sdk.tui --selftest`（先跑 sdk 那 165 项，再跑这里的）。
"""

import argparse
import time

from .. import board_defs, protocol
from ..selftest import Checker, main as sdk_selftest_main
from ..sim import SimLink
from . import session as cb
from .session import DEFAULT_STEP_RAD, GRIPPER


def _fake_opts(**kw):
    """自测用的 opts（和 argparse 出来的那份同名同义）。"""
    d = dict(fake=True, port=None, baud=921600, arm_id=0, fake_lost=0, ctrl_hz=100.0,
             fb_hz=5.0, log=None, stop_on_exit=False, step=DEFAULT_STEP_RAD,
             init_tol=board_defs.ROBOT_INIT_POSE_TOL_RAD, handshake_timeout=3.0)
    d.update(kw)
    return argparse.Namespace(**d)


def selftest():
    """离线自测。不接硬件、不起 curses、不需要 pyserial。

    两块：**sdk 那一套**（协议 / 闸门 / 假板子，`sdk/selftest.py`），
    再**控制台后端这一套**（两台臂的接线、单位、步长、命令真的写出去了没有）。
    """
    import math

    print("=" * 68)
    print("1) sdk 离线自测（协议 / 闸门 / 假板子）")
    print("=" * 68)
    rc1 = sdk_selftest_main([])

    print()
    print("=" * 68)
    print("2) 控制台后端自测（sdk.tui.session，用假板子跑）")
    print("=" * 68)
    c = Checker()

    # ---- 单位与限位（rad 是面板的口径）----
    c.section("单位与软限位（rad）")
    c.check("j1 上限 = 90°", abs(cb.axis_hi_rad(0) - math.radians(90.0)) < 1e-12)
    c.check("j1 下限 = -115°", abs(cb.axis_lo_rad(0) - math.radians(-115.0)) < 1e-12)
    c.check("夹爪软限位 = ∓142°（= ∓284/2）",
            abs(cb.axis_hi_rad(GRIPPER) - math.radians(142.0)) < 1e-12
            and abs(cb.axis_lo_rad(GRIPPER) + math.radians(142.0)) < 1e-12)
    v, cl = cb.clamp_axis_rad(0, math.radians(120.0))
    c.check("夹到上限 90°", cl and abs(v - math.radians(90.0)) < 1e-12)
    v, cl = cb.clamp_axis_rad(0, math.radians(-120.0))
    c.check("夹到下限 -115°", cl and abs(v - math.radians(-115.0)) < 1e-12)
    v, cl = cb.clamp_axis_rad(3, 0.0)
    c.check("界内不夹、也不报夹过（就是拍板不要那个数）", (not cl) and v == 0.0)

    # ---- 步长档 / --arm-id 解析 ----
    c.section("步长档与 --arm-id")
    c.check("rad 档是 0.005/0.01/0.02/0.05/0.1/0.2",
            cb.STEP_CHOICES_RAD == [0.005, 0.01, 0.02, 0.05, 0.1, 0.2])
    c.check("默认 rad 档在档位里（≈ 旧版的 1.0°）",
            cb.DEFAULT_STEP_RAD in cb.STEP_CHOICES_RAD)
    c.check("米档递增且默认档在档位里",
            all(b > a > 0 for a, b in zip(cb.STEP_CHOICES_M, cb.STEP_CHOICES_M[1:]))
            and cb.DEFAULT_STEP_M in cb.STEP_CHOICES_M)
    ok = True
    for s in ("0", "1", "0x1", " 1"):
        try:
            if not 0 <= cb.parse_arm_id(s) < cb.N_ARM:
                ok = False
        except Exception:                                   # noqa: BLE001
            ok = False
    for s in ("2", "-1", "x", "", "1.0"):
        try:
            cb.parse_arm_id(s)
            ok = False          # 该拒的没拒
        except argparse.ArgumentTypeError:
            pass
    c.check("parse_arm_id 收 0/1/0x1/空格，拒 2/-1/x/空/1.0", ok)

    # ---- 两台臂的接线（走真实 SimLink：解析、分流、闸门都是真的）----
    class _RecLink(SimLink):
        """记下每一帧发出去的 tid —— 断言"该发的发了、不该发的一帧没发"。"""

        def __init__(self, *a, **kw):
            super().__init__(*a, **kw)
            self.tids = []

        def send(self, tid, payload=b"", arm_id=protocol.DEFAULT_ARM_ID):
            self.tids.append((tid, arm_id))
            return super().send(tid, payload, arm_id)

    c.section("两台臂（一条链路、两个 Arm、状态机各跑各的）")
    opts = _fake_opts()
    # 假板子时钟 ×2：自测要等 0x02 的 3 s，真实时间等不起。
    # **不能再快了** —— 面板的保活是**真实时间**每 150 ms 一帧（tick 的 last_tx 用的是
    # time.monotonic），而假板子的 500 ms 看门狗按它自己的时钟走：×2 时 150 ms 实时 =
    # 300 ms 模拟（还有 40% 余量），×4 就变成 600 ms > 500，板子会一路被踢进 comm_lost
    # 卡在「离线等待」。真机演练就用默认的 ×1（面板本来该长什么样就是什么样）。
    link = _RecLink(speed=2.0)
    sess = cb.ConsoleSession(link, opts)
    link.open()
    try:
        def pump(sec, pred=None):
            """推 tick 到 pred 成立或超时（tick 是面板的唯一节拍来源）。"""
            end = time.monotonic() + sec
            while True:
                sess.tick(time.monotonic())
                if pred is not None and pred():
                    return True
                if time.monotonic() > end:
                    break
                time.sleep(0.002)
            return pred is None or pred()

        def sp(a):
            return sess.views[a].arm.snapshot()

        c.check("两台都收到反馈帧（0x80 按 arm_id 分流）",
                pump(6.0, lambda: sp(0) is not None and sp(1) is not None),
                "0x80 帧数 臂A=%s 臂B=%s"
                % (sess.views[0].arm.n_feedback, sess.views[1].arm.n_feedback))
        c.check("两台的目标角是各自独立的列表",
                sess.views[0].target is not sess.views[1].target)
        c.check("两台各自被播种（都对到自己的反馈上）",
                sess.views[0].seeded and sess.views[1].seeded)

        snap = sess.snapshot()
        fb = snap["fb"]
        c.check("snapshot 的键齐（面板的契约）",
                {"fb", "fb_t", "ack", "ack_t", "board_version", "link_up", "link_err",
                 "n_rx", "n_rx_fb", "rx_rate", "n_tx", "n_drop", "n_crc_err", "n_bad_arm",
                 "unknown_tids", "arm_id", "arms"} <= set(snap))
        c.check("fb 的键齐", {"mode", "state", "lost", "inited", "joint", "jvel", "gripper",
                            "gripper_raw_rad", "gripper_calibrated", "temp"} == set(fb))
        c.check("joint 是 6 个 rad（不是 deg）",
                len(fb["joint"]) == 6 and abs(fb["joint"][0]) <= math.pi)
        c.check("夹爪：标定标志与米数一致（没标定时 m 为 None，原始角一直在）",
                (fb["gripper"] is not None) == fb["gripper_calibrated"]
                and isinstance(fb["gripper_raw_rad"], float))
        c.check("n_rx_fb 是本臂口径（= 当前臂的 n_feedback）",
                snap["n_rx_fb"] == sess.cur.arm.n_feedback)
        c.check("arms 概览两台都在、按 id 排序",
                [a["arm_id"] for a in snap["arms"]] == [0, 1])

        # 切臂 + 点动只作用于当前臂
        before0 = list(sess.views[0].target)
        sess.switch_arm(1)
        c.check("a 键切到臂B（当前臂 = 1）", sess.cur_arm_id == 1 and sess.arm_id == 1)
        sess.jog(0, +1)
        c.check("点动只动当前臂（臂A 的目标角一动没动）",
                sess.views[0].target == before0
                and sess.views[1].target[0] != before0[0])
        c.check("sess.target 读的是当前臂那份", sess.target is sess.views[1].target)
        sess.switch_arm(0)
        c.check("再切回臂A", sess.cur_arm_id == 0 and sess.target is sess.views[0].target)
        c.check("hold 也按当前臂走（只挂臂A）",
                (sess.hold is False) and all(v.hold is False for v in sess.views.values()))

        # ---- 命令：该发的写出去、不该发的一帧都不发 ----
        c.section("命令与闸门（假板子的闸门是真的）")
        c.check("臂A 走到「待初始化」",
                pump(4.0, lambda: sp(0).robot_state == board_defs.STATE_WAIT_INIT))
        link.tids.clear()
        sess.send_arm_init()
        c.check("待初始化态按 i → 真的发出 0x02（盖当前臂的 arm_id）",
                (protocol.TID_ARM_INIT, 0) in link.tids, "发出的：%r" % (link.tids,))

        c.check("初始化跑完回到「正常运行」（板子 ~7 s）",
                pump(8.0, lambda: sp(0).robot_state == board_defs.STATE_NORMAL))
        link.tids.clear()
        sess.send_empty_mode(protocol.TID_ARM_START, "0x12 启动")
        c.check("正常运行态按 s → 发出 0x12（进 run）",
                (protocol.TID_ARM_START, 0) in link.tids)
        c.check("进 run 了（0x16 才有可能被受理）",
                pump(3.0, lambda: sp(0).mode == board_defs.MODE_RUN))

        n_warn = sum(1 for _, lv, _ in sess.events if lv == "warn")
        link.tids.clear()
        sess.send_arm_init()
        c.check("run 档下按 i → **一帧没发**（不谎报「已发」）",
                not link.tids and not sess.cur.force_init,
                "发出的：%r" % (link.tids,))
        c.check("run 档下按 i 给出恰好一条 warn（SDK 说清缺哪条闸门）",
                sum(1 for _, lv, _ in sess.events if lv == "warn") == n_warn + 1)
        c.check("run 档下按 i 不提供「再按一次强发」（强发也是被静默丢弃）",
                not any("可强发" in t for _, _, t in sess.events))

        n_warn = sum(1 for _, lv, _ in sess.events if lv == "warn")
        link.tids.clear()
        sess.send_setzero()
        c.check("正常运行态按 z → 一帧没发（0x10 只认待初始化）",
                not link.tids and sum(1 for _, lv, _ in sess.events if lv == "warn") == n_warn + 1)

        c.check("臂B 还停在「待初始化」（两台状态机各跑各的）",
                sp(1).robot_state == board_defs.STATE_WAIT_INIT, sp(1).robot_state_name)
        c.check("臂A 在发 0x16、臂B 没在发（控制流按台独立）",
                pump(3.0, lambda: sess.snapshot()["arms"][0]["ctrl_live"]
                     and not sess.snapshot()["arms"][1]["ctrl_live"]))
        c.check("streaming_any() 看的是两台（退出确认用）", sess.streaming_any())

        sess.switch_arm(1)
        link.tids.clear()
        sess.send_arm_init()
        c.check("臂B 按 i → 0x02 盖的是 arm_id=1（不是当前面板看的那台那么简单）",
                (protocol.TID_ARM_INIT, 1) in link.tids, "发出的：%r" % (link.tids,))
        c.check("臂B 初始化完到「正常运行」",
                pump(8.0, lambda: sp(1).robot_state == board_defs.STATE_NORMAL))
        sess.send_empty_mode(protocol.TID_ARM_START, "0x12 启动")
        c.check("臂B 进 run", pump(3.0, lambda: sp(1).mode == board_defs.MODE_RUN))
        c.check("两台都在发 0x16，各盖各的 arm_id",
                pump(3.0, lambda: sess.snapshot()["arms"][0]["ctrl_live"]
                     and sess.snapshot()["arms"][1]["ctrl_live"])
                and (protocol.TID_ARM_CTRL_POS, 0) in link.tids
                and (protocol.TID_ARM_CTRL_POS, 1) in link.tids)
        sess.switch_arm(0)

        # ---- reopen：两台都要重新播种 ----
        c.section("reopen（r 键 / 拔线重开）")
        c.check("reopen 成功", sess.reopen())
        c.check("reopen 后两台的播种都清了（等下一帧反馈）",
                all(not v.seeded for v in sess.views.values()))
        c.check("reopen 后两台都重新握手+播种（假板子重上电 → 0→1→2）",
                pump(6.0, lambda: all(v.seeded for v in sess.views.values())))

        # ---- stop_all（--stop-on-exit）----
        link.tids.clear()
        done = sess.stop_all()
        c.check("退出补 0x14：两台都发、都带各自的 arm_id",
                done == ["臂A", "臂B"]
                and (protocol.TID_ARM_STOP, 0) in link.tids
                and (protocol.TID_ARM_STOP, 1) in link.tids, "发出的：%r" % (link.tids,))
    finally:
        link.close()

    print()
    rc2 = c.summary()
    print()
    if rc1 or rc2:
        print("自测**没过**：见上面 FAIL。")
        return 1
    print("自测全部通过（sdk + 控制台后端）。")
    return 0
