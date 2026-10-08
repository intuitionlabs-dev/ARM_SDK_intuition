"""控制台侧离线自测：**不接硬件、不起 curses、不需要 pyserial**。

跑的是后端那一套 —— 两台臂的接线（一条链路、两个 `Arm`、各自的状态机）、单位与软限位、
步长档、闸门有没有把该拦的拦住、命令真的写出去了没有。用 `sdk.sim.SimLink` 当假板子，
走的是**真的**解析、分流、闸门。面板本身（curses 排版）不在这里测。

入口：`python3 -m sdk.tui --selftest`（先跑 sdk 那 165 项，再跑这里的）。
"""

import argparse
import time

from .. import board_defs, i18n, protocol
from ..selftest import Checker, main as sdk_selftest_main
from ..sim import SimLink
from . import session as cb
from .session import DEFAULT_STEP_RAD, GRIPPER


def _fake_opts(**kw):
    """自测用的 opts（和 argparse 出来的那份同名同义）。"""
    d = dict(fake=True, port=None, baud=921600, arm_id=0, fake_lost=0, ctrl_hz=100.0,
             fb_hz=5.0, log=None, stop_on_exit=False, step=DEFAULT_STEP_RAD,
             init_tol=board_defs.ROBOT_INIT_POSE_TOL_RAD, handshake_timeout=3.0,
             lang="zh")
    d.update(kw)
    return argparse.Namespace(**d)


def selftest():
    """离线自测。不接硬件、不起 curses、不需要 pyserial。

    两块：**sdk 那一套**（协议 / 闸门 / 假板子，`sdk/selftest.py`），
    再**控制台后端这一套**（两台臂的接线、单位、步长、命令真的写出去了没有）。
    """
    import math

    print("=" * 68)
    print(i18n.tr("selftest.hdr_sdk"))
    print("=" * 68)
    rc1 = sdk_selftest_main([])

    print()
    print("=" * 68)
    print(i18n.tr("selftest.hdr_tui"))
    print("=" * 68)
    c = Checker()

    # ---- 单位与限位（rad 是面板的口径）----
    c.section(i18n.tr("selftest.sec_units"))
    c.check(i18n.tr("selftest.j1_hi"), abs(cb.axis_hi_rad(0) - math.radians(90.0)) < 1e-12)
    c.check(i18n.tr("selftest.j1_lo"), abs(cb.axis_lo_rad(0) - math.radians(-115.0)) < 1e-12)
    c.check(i18n.tr("selftest.grip_limit"),
            abs(cb.axis_hi_rad(GRIPPER) - math.radians(142.0)) < 1e-12
            and abs(cb.axis_lo_rad(GRIPPER) + math.radians(142.0)) < 1e-12)
    v, cl = cb.clamp_axis_rad(0, math.radians(120.0))
    c.check(i18n.tr("selftest.clamp_hi"), cl and abs(v - math.radians(90.0)) < 1e-12)
    v, cl = cb.clamp_axis_rad(0, math.radians(-120.0))
    c.check(i18n.tr("selftest.clamp_lo"), cl and abs(v - math.radians(-115.0)) < 1e-12)
    v, cl = cb.clamp_axis_rad(3, 0.0)
    c.check(i18n.tr("selftest.clamp_in"), (not cl) and v == 0.0)

    # ---- 步长档 / --arm-id 解析 ----
    c.section(i18n.tr("selftest.sec_step"))
    c.check(i18n.tr("selftest.step_rad"),
            cb.STEP_CHOICES_RAD == [0.005, 0.01, 0.02, 0.05, 0.1, 0.2])
    c.check(i18n.tr("selftest.step_default"),
            cb.DEFAULT_STEP_RAD in cb.STEP_CHOICES_RAD)
    c.check(i18n.tr("selftest.step_m"),
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
    c.check(i18n.tr("selftest.arm_id"), ok)

    # ---- 两台臂的接线（走真实 SimLink：解析、分流、闸门都是真的）----
    class _RecLink(SimLink):
        """记下每一帧发出去的 tid —— 断言"该发的发了、不该发的一帧没发"。"""

        def __init__(self, *a, **kw):
            super().__init__(*a, **kw)
            self.tids = []

        def send(self, tid, payload=b"", arm_id=protocol.DEFAULT_ARM_ID):
            self.tids.append((tid, arm_id))
            return super().send(tid, payload, arm_id)

    c.section(i18n.tr("selftest.sec_two"))
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

        c.check(i18n.tr("selftest.two_fb"),
                pump(6.0, lambda: sp(0) is not None and sp(1) is not None),
                i18n.tr("selftest.fb_counts",
                        sess.views[0].arm.n_feedback, sess.views[1].arm.n_feedback))
        c.check(i18n.tr("selftest.two_target"),
                sess.views[0].target is not sess.views[1].target)
        c.check(i18n.tr("selftest.two_seeded"),
                sess.views[0].seeded and sess.views[1].seeded)

        snap = sess.snapshot()
        fb = snap["fb"]
        c.check(i18n.tr("selftest.snap_keys"),
                {"fb", "fb_t", "ack", "ack_t", "board_version", "link_up", "link_err",
                 "n_rx", "n_rx_fb", "rx_rate", "n_tx", "n_drop", "n_crc_err", "n_bad_arm",
                 "unknown_tids", "arm_id", "arms"} <= set(snap))
        c.check(i18n.tr("selftest.fb_keys"),
                {"mode", "state", "lost", "inited", "joint", "jvel", "gripper",
                 "gripper_raw_rad", "gripper_calibrated", "temp"} == set(fb))
        c.check(i18n.tr("selftest.fb_rad"),
                len(fb["joint"]) == 6 and abs(fb["joint"][0]) <= math.pi)
        c.check(i18n.tr("selftest.grip_cal"),
                (fb["gripper"] is not None) == fb["gripper_calibrated"]
                and isinstance(fb["gripper_raw_rad"], float))
        c.check(i18n.tr("selftest.n_rx_fb"),
                snap["n_rx_fb"] == sess.cur.arm.n_feedback)
        c.check(i18n.tr("selftest.arms_sorted"),
                [a["arm_id"] for a in snap["arms"]] == [0, 1])

        # 切臂 + 点动只作用于当前臂
        before0 = list(sess.views[0].target)
        sess.switch_arm(1)
        c.check(i18n.tr("selftest.sw_to_b"), sess.cur_arm_id == 1 and sess.arm_id == 1)
        sess.jog(0, +1)
        c.check(i18n.tr("selftest.jog_cur"),
                sess.views[0].target == before0
                and sess.views[1].target[0] != before0[0])
        c.check(i18n.tr("selftest.target_cur"), sess.target is sess.views[1].target)
        sess.switch_arm(0)
        c.check(i18n.tr("selftest.sw_back"),
                sess.cur_arm_id == 0 and sess.target is sess.views[0].target)
        c.check(i18n.tr("selftest.hold_cur"),
                (sess.hold is False) and all(v.hold is False for v in sess.views.values()))

        # ---- 命令：该发的写出去、不该发的一帧都不发 ----
        c.section(i18n.tr("selftest.sec_cmd"))
        c.check(i18n.tr("selftest.a_wait_init"),
                pump(4.0, lambda: sp(0).robot_state == board_defs.STATE_WAIT_INIT))
        link.tids.clear()
        sess.send_arm_init()
        c.check(i18n.tr("selftest.init_sent"),
                (protocol.TID_ARM_INIT, 0) in link.tids, i18n.tr("selftest.sent", link.tids))

        c.check(i18n.tr("selftest.init_done"),
                pump(8.0, lambda: sp(0).robot_state == board_defs.STATE_NORMAL))
        link.tids.clear()
        sess.send_empty_mode(protocol.TID_ARM_START, i18n.tr("selftest.tid_start"))
        c.check(i18n.tr("selftest.start_ok"),
                (protocol.TID_ARM_START, 0) in link.tids)
        c.check(i18n.tr("selftest.run_ok"),
                pump(3.0, lambda: sp(0).mode == board_defs.MODE_RUN))

        n_warn = sum(1 for _, lv, _ in sess.events if lv == "warn")
        link.tids.clear()
        sess.send_arm_init()
        c.check(i18n.tr("selftest.i_no_frame"),
                not link.tids and not sess.cur.force_init,
                i18n.tr("selftest.sent", link.tids))
        c.check(i18n.tr("selftest.i_one_warn"),
                sum(1 for _, lv, _ in sess.events if lv == "warn") == n_warn + 1)
        # 再按一次依旧一帧不发：强发通道只给「姿态」那一条闸门。
        # （原来这里匹配中文「可强发」字样 —— 那样在 --lang en 下会永真、静默失效，
        #  所以换成语言无关的判据。）
        link.tids.clear()
        sess.send_arm_init()
        c.check(i18n.tr("selftest.i_again"),
                not link.tids and not sess.cur.force_init
                and sum(1 for _, lv, _ in sess.events if lv == "warn") == n_warn + 2)

        n_warn = sum(1 for _, lv, _ in sess.events if lv == "warn")
        link.tids.clear()
        sess.send_setzero()
        c.check(i18n.tr("selftest.z_no_frame"),
                not link.tids and sum(1 for _, lv, _ in sess.events if lv == "warn") == n_warn + 1)

        c.check(i18n.tr("selftest.b_still"),
                sp(1).robot_state == board_defs.STATE_WAIT_INIT, sp(1).robot_state_name)
        c.check(i18n.tr("selftest.a_stream"),
                pump(3.0, lambda: sess.snapshot()["arms"][0]["ctrl_live"]
                     and not sess.snapshot()["arms"][1]["ctrl_live"]))
        c.check(i18n.tr("selftest.streaming_any"), sess.streaming_any())

        sess.switch_arm(1)
        link.tids.clear()
        sess.send_arm_init()
        c.check(i18n.tr("selftest.b_init"),
                (protocol.TID_ARM_INIT, 1) in link.tids, i18n.tr("selftest.sent", link.tids))
        c.check(i18n.tr("selftest.b_done"),
                pump(8.0, lambda: sp(1).robot_state == board_defs.STATE_NORMAL))
        sess.send_empty_mode(protocol.TID_ARM_START, i18n.tr("selftest.tid_start"))
        c.check(i18n.tr("selftest.b_run"), pump(3.0, lambda: sp(1).mode == board_defs.MODE_RUN))
        c.check(i18n.tr("selftest.both_stream"),
                pump(3.0, lambda: sess.snapshot()["arms"][0]["ctrl_live"]
                     and sess.snapshot()["arms"][1]["ctrl_live"])
                and (protocol.TID_ARM_CTRL_POS, 0) in link.tids
                and (protocol.TID_ARM_CTRL_POS, 1) in link.tids)
        sess.switch_arm(0)

        # ---- reopen：两台都要重新播种 ----
        c.section(i18n.tr("selftest.sec_reopen"))
        c.check(i18n.tr("selftest.reopen_ok"), sess.reopen())
        c.check(i18n.tr("selftest.reopen_cleared"),
                all(not v.seeded for v in sess.views.values()))
        c.check(i18n.tr("selftest.reopen_seeded"),
                pump(6.0, lambda: all(v.seeded for v in sess.views.values())))

        # ---- stop_all（--stop-on-exit）----
        link.tids.clear()
        done = sess.stop_all()
        c.check(i18n.tr("selftest.stop_two"),
                done == [cb.short_label(0), cb.short_label(1)]
                and (protocol.TID_ARM_STOP, 0) in link.tids
                and (protocol.TID_ARM_STOP, 1) in link.tids, i18n.tr("selftest.sent", link.tids))
    finally:
        link.close()

    print()
    rc2 = c.summary()
    print()
    if rc1 or rc2:
        print(i18n.tr("selftest.fail"))
        return 1
    print(i18n.tr("selftest.pass"))
    return 0
