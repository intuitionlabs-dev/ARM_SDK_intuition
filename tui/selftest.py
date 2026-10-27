"""Console-side offline self-test: **no hardware, no curses, no pyserial needed**.

It runs the backend's own set — the two-arm wiring (one link, two `Arm`s, one state machine
each), the units and soft limits, the step presets, whether the gates stop what they should,
and whether commands really went out. `sdk.sim.SimLink` stands in for the board, and the
**real** parsing, routing and gating are exercised. The panel itself (curses layout) is not
tested here.

Entry point: `python3 -m sdk.tui --selftest` (runs sdk's 205 items first, then these).
"""

import argparse
import time

from .. import i18n, protocol
from ..board_defs import arx_x5 as board_defs
from ..selftest import Checker, main as sdk_selftest_main
from ..sim import SimLink
from . import session as cb
from .session import DEFAULT_STEP_RAD, GRIPPER


def _fake_opts(**kw):
    """The opts used by the self-test (same names, same meanings as the argparse one)."""
    d = dict(fake=True, port=None, baud=921600, arm_id=0, fake_lost=0, ctrl_hz=100.0,
             fb_hz=5.0, log=None, stop_on_exit=False, step=DEFAULT_STEP_RAD,
             init_tol=board_defs.ROBOT_INIT_POSE_TOL_RAD, handshake_timeout=3.0,
             lang="zh")
    d.update(kw)
    return argparse.Namespace(**d)


def selftest():
    """Offline self-test. No hardware, no curses, no pyserial needed.

    Two parts: **sdk's own set** (protocol / gates / fake board, `sdk/selftest.py`), then
    **the console backend's set** (the two-arm wiring, the units, the step presets, whether
    commands really went out).
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
    # Language-table self-check: key sets equal / every value a str / the % conversion
    # specifiers of zh and en agree / no Han characters in en.
    # **Deliberately not a c.check** — it is not a runtime criterion of the "run it once
    # and you know" kind, and putting it in would change the "48 items" count that is
    # referenced all over the place.
    probs = i18n.check_tables()
    if probs:
        print(i18n.tr("selftest.table_bad", len(probs)))
        for p in probs:
            print("  " + p)
        return 1
    c = Checker()

    # ---- units and limits (rad is the panel's convention) ----
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

    # ---- step presets / --arm-id parsing ----
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
            ok = False          # something that should have been rejected was not
        except argparse.ArgumentTypeError:
            pass
    c.check(i18n.tr("selftest.arm_id"), ok)

    # ---- two-arm wiring (through the real SimLink: parsing, routing and gates are real) ----
    class _RecLink(SimLink):
        """Records the tid of every frame sent out — to assert "what should have been sent
        was sent, and nothing that should not have been sent went out at all".

        It also plays the two states the real `Link` reaches on its own (they are plain
        attribute moves over there too — the RX thread is the one doing the retrying, and
        the panel only reads the result).
        """

        def __init__(self, *a, **kw):
            super().__init__(*a, **kw)
            self.tids = []

        def send(self, tid, payload=b"", arm_id=protocol.DEFAULT_ARM_ID):
            self.tids.append((tid, arm_id))
            return super().send(tid, payload, arm_id)

        def simulate_reconnect(self):
            """The RX thread reopened the port by itself after a drop (where a successful
            `Link._reconnect` lands)."""
            self.link_up = False
            self.link_err = "读串口失败: 假"
            self.open_gen += 1
            self.n_reconnect += 1
            self.reconnect_tries = 1
            self.reconnect_err = None
            self.reconnect_gave_up = False
            self.link_up = True
            self.link_err = None

        def simulate_give_up(self, tries, err):
            """One round of drops used up the tries and the RX thread stopped (where
            `Link._reconnect` giving up lands)."""
            self.link_up = False
            self.link_err = "读串口失败: 假"
            self.reconnect_tries = tries
            self.reconnect_err = err
            self.reconnect_gave_up = True

    c.section(i18n.tr("selftest.sec_two"))
    opts = _fake_opts()
    # Fake-board clock ×2: the self-test would wait the 3 s for 0x02, which is too long in
    # real time. The panel's keepalive is one frame every **real** 150 ms (tick's last_tx
    # uses time.monotonic), while the fake board's 3 s watchdog runs on its own clock: at
    # ×2, 150 ms of real time = 300 ms simulated, a tenth of the window, so the speed-up
    # does not starve the board into comm_lost. For a real-hardware rehearsal use the
    # default ×1 (the panel then looks exactly the way it should).
    link = _RecLink(speed=2.0)
    sess = cb.ConsoleSession(link, opts)
    link.open()
    try:
        def pump(sec, pred=None):
            """Advance tick until pred holds or it times out (tick is the panel's only
            source of beats)."""
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
                 "link_auto", "link_retry", "link_retry_max", "link_give_up",
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

        # switching arms + jogging only affects the current arm
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

        # ---- commands: what should go out does, what should not goes out not at all ----
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
        # Pressing it once more still sends not one frame: the force path is given to the
        # "pose" gate only. The check is on the warn count, not on the message text — the
        # text depends on the language, the count does not.
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

        # ---- reopen: both arms have to be re-seeded ----
        c.section(i18n.tr("selftest.sec_reopen"))
        c.check(i18n.tr("selftest.reopen_ok"), sess.reopen())
        c.check(i18n.tr("selftest.reopen_cleared"),
                all(not v.seeded for v in sess.views.values()))
        c.check(i18n.tr("selftest.reopen_seeded"),
                pump(6.0, lambda: all(v.seeded for v in sess.views.values())))

        # ---- automatic reconnect: the panel only reads the link's state, so the two ends
        # ---- of it are played by hand here (the retrying itself is tested in sdk's set)
        c.section(i18n.tr("selftest.sec_relink"))
        link.simulate_give_up(tries=10, err="打不开假口子")
        c.check(i18n.tr("selftest.relink_snap"),
                pump(1.0, lambda: sess.snapshot()["link_give_up"]
                     and not sess.snapshot()["link_up"]
                     and sess.snapshot()["link_retry"] == 10
                     and sess.snapshot()["link_retry_max"] == 10
                     and sess.snapshot()["link_auto"]))
        want = i18n.tr("session.relink_giveup", 10, "打不开假口子")
        c.check(i18n.tr("selftest.relink_giveup"),
                pump(1.0, lambda: any(t == want for _, _, t in sess.events)),
                i18n.tr("selftest.sent", [t for _, _, t in list(sess.events)[-3:]]))

        link.simulate_reconnect()
        c.check(i18n.tr("selftest.relink_seeded"),
                pump(6.0, lambda: all(v.seeded for v in sess.views.values())))
        want = i18n.tr("session.relinked", i18n.tr("session.fake_board"))
        c.check(i18n.tr("selftest.relink_msg"),
                any(t == want for _, _, t in sess.events),
                i18n.tr("selftest.sent", [t for _, _, t in list(sess.events)[-3:]]))

        # ---- stop_all (--stop-on-exit) ----
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
