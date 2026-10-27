"""Host console for the robot arm (full-screen curses panel). Runs over both UART and
USB links, and with both arms.

**The backend is just `sdk/`**: the protocol, the serial link, the per-arm_id routing, the
gate pre-checks and the unit conversion are all there; the "one consistent view per frame"
the panel wants comes from `ConsoleSession.snapshot()` in [session.py](session.py), and
what is on screen lives in [app.py](app.py). So this module is only the command-line
shell — changing the protocol or adding a command touches `sdk/`, and at most the layout
here.

How it talks to the board (see protocol_doc.md / sdk/README):

  * **sends a 0x16 periodically** (6 joint angles + gripper angle), **which is also the
    keepalive frame** — on the board side, no valid frame for 3 s drops the arm back to
    damping, and a dropped 0x16 still counts as a valid frame (robot_state.c's
    s_last_rx_ms is refreshed before dispatch), so sending at any time costs nothing.
  * **receives the 0x80 feedback frame** and displays mode / robot_state / joint angles /
    lost mask / gripper angle.
  * a key steps the joint target, and the target is clamped on **the host** against the
    board's soft limits first: the board clamps too (gimbal_cmd.c), but without clamping
    here the target shown on the panel would be a lie.

**Units: on the panel everything is SI** — joints in rad / rad·s⁻¹, gripper in m (needs a
calibration, see sdk/README "gripper calibration"). On the wire it is deg and sdk does the
conversion. **The event log still writes deg** (so it matches the firmware and other
people's logs), so "0.0175 rad on the panel, 1.0° in the log" are the same thing — do not
assume one of them is wrong.

Session flow (the board's state machine, see sdk/README "the life cycle of a session"):
    ① wait for handshake (the board only accepts the 0x01 version frame; a wrong version
       code is **dropped silently**, and the board keeps reporting its own code every
       100 ms until the host sends the right one)
    ② offline wait (the board advances on its own once all 7 motors are up)
    ③ wait-init — press i to send 0x02
    ④ initializing (the board runs one-key reset → one-key gripper zeroing itself, about
       7 s, and the keepalive must be maintained throughout)
    ⑤ normal operation (**damping by default**! press s to send 0x12 to enter run, and
       only under run is 0x16 accepted)
       ↑ in that mode (stop/damping) **you can also press i to re-run the init**; when it
         finishes it returns to ⑤ rather than ③.
         (Opening the board-side gate up is the convention the user asked for; under run,
         pressing i is still silently dropped.)
    ⑥ press z to send 0x10 set-zero → calibration, then back to ③
    ⑦ no valid frame for 3 s → damping. An arm that finished init **stays in ⑤** (press s
       to go back to run); one that has not goes to "comm lost", and one more frame
       returns it to ②, i.e. the init has to be redone

**Two arms**: one board with two boards, two CAN buses and two arms, but the host has only
**one link**. So a process = one link + two `sdk.Arm`s, and **both are being driven** (each
with its own target, its own 0x16 stream, its own keepalive timer); what `a` switches on
the panel is "which arm is being watched / operated". The two state machines are fully
independent — one still sitting in wait-init while the other is already running is normal
(i/o/s/d/v/x have to be pressed once per arm).

**Only one process at a time**: the board's host link is one of a compile-time either/or
pair (ROBOT_LINK in robot_state.h) and is physically a single port, so two processes on the
same tty would steal bytes from each other and split frames.

**Safety**: by default, exiting does **not** send 0x14 (stop), it just stops sending 0x16 —
the target freezes, the arm stays where it is, and after about 3 s the board drops into
damping on its own for lack of frames. To go slack right away, press x by hand or add
--stop-on-exit.

Usage (run it at the repository root — `sdk` is a package in the repo, not installed):
    python3 -m sdk.tui                       # real hardware: pick a port automatically
                                             # (the board's USB first, falling back to a
                                             # 1a86 RS485 adapter), 921600
    python3 -m sdk.tui --port /dev/ttyACM0   # a specific port
    python3 -m sdk.tui --arm-id 1            # current arm watches the CAN2 one at startup
                                             # (a switches it)
    python3 -m sdk.tui --fake                # fake board: walk the panel/keys/state machine
                                             # with no hardware
    python3 -m sdk.tui --selftest            # offline self-test (sdk + console backend),
                                             # no hardware
The old path `python3 scripts/uart_frame_cli.py ...` takes exactly the same arguments; it
is a compatibility shim that forwards here.
Press h in the panel for the key table."""


import argparse
import locale
import os
import sys

from .. import i18n
from ..board_defs import arx_x5 as board_defs
from ..errors import LinkError
from ..link import (assert_port_allowed, describe_port, link_hint,
                    list_links, pick_default_port)
from .session import (DEFAULT_ARM_ID, DEFAULT_STEP_RAD, STEP_CHOICES_RAD,
                      build_session, parse_arm_id)
from .selftest import selftest


def _pre_lang(argv):
    """Pull `--lang` out on its own first.

    The language has to be fixed **before** the real parser is built: `--help` and
    argparse's own errors both happen inside `parse_args`, and calling `set_lang` then
    would be too late (the `help=` text would already be Chinese). The pre-parser carries
    no `choices` and does not open `-h`, so it can never `SystemExit` itself; `set_lang`
    only accepts zh/en and falls back to zh for anything else (including None).
    """
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument("--lang", nargs="?", default=None)
    try:
        return i18n.set_lang(pre.parse_known_args(argv)[0].lang)
    except SystemExit:
        return i18n.set_lang(None)


def main(argv=None, prog=None):
    _pre_lang(argv)
    ap = argparse.ArgumentParser(
        prog=prog or "python3 -m sdk.tui",   # under -m, omitting it shows "__main__.py"
        description=i18n.tr("main.cli_description"),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=i18n.tr("main.epilog"))
    ap.add_argument("--port", default=None, help=i18n.tr("main.help.port"))
    ap.add_argument("--baud", type=int, default=921600, help=i18n.tr("main.help.baud"))
    ap.add_argument("--arm-id", type=parse_arm_id, default=DEFAULT_ARM_ID,
                    help=i18n.tr("main.help.arm_id", DEFAULT_ARM_ID))
    ap.add_argument("--ctrl-hz", type=float, default=100.0, help=i18n.tr("main.help.ctrl_hz"))
    ap.add_argument("--fb-hz", type=float, default=5.0, help=i18n.tr("main.help.fb_hz"))
    ap.add_argument("--step", type=float, default=DEFAULT_STEP_RAD,
                    help=i18n.tr("main.help.step",
                                 " / ".join("%g" % v for v in STEP_CHOICES_RAD),
                                 DEFAULT_STEP_RAD))
    ap.add_argument("--handshake-timeout", type=float, default=3.0,
                    help=i18n.tr("main.help.handshake"))
    ap.add_argument("--init-tol", type=float, default=board_defs.ROBOT_INIT_POSE_TOL_RAD,
                    help=i18n.tr("main.help.init_tol",
                                 board_defs.ROBOT_INIT_POSE_TOL_RAD,
                                 board_defs.ROBOT_INIT_POSE_TOL_DEG,
                                 board_defs.ROBOT_INIT_POSE_TOL_DEG))
    ap.add_argument("--fake", action="store_true", help=i18n.tr("main.help.fake"))
    ap.add_argument("--fake-lost", type=lambda s: int(s, 0), default=0,
                    help=i18n.tr("main.help.fake_lost"))
    ap.add_argument("--selftest", action="store_true", help=i18n.tr("main.help.selftest"))
    ap.add_argument("--log", default=None, help=i18n.tr("main.help.log"))
    ap.add_argument("--stop-on-exit", action="store_true",
                    help=i18n.tr("main.help.stop_on_exit"))
    ap.add_argument("--lang", nargs="?", choices=i18n.LANGS, default=i18n.get_lang(),
                    help=i18n.tr("main.help.lang"))
    opts = ap.parse_args(argv)

    if opts.selftest:
        return selftest()

    if opts.step not in STEP_CHOICES_RAD:
        print(i18n.tr("main.bad_step", STEP_CHOICES_RAD))
        return 2

    if not opts.fake:
        try:
            opts.port = opts.port or pick_default_port()
            assert_port_allowed(opts.port)
        except LinkError as e:
            # A probe port is rejected here (the SDK's message says why); do not lose that
            # hint
            print("%s" % e)
            return 2
        if not os.path.exists(opts.port):
            print(i18n.tr("main.no_device", opts.port, link_hint()))
            cands = list_links()
            if cands:
                for path, _kind, _ident in cands:
                    print("    %s" % describe_port(path))
            else:
                print(i18n.tr("main.no_links"))
            return 2
        try:
            import serial  # noqa: F401
        except ImportError:
            print(i18n.tr("main.no_pyserial"))
            return 2

    # curses has to show Chinese, so the locale must be set **before** initscr (otherwise
    # the wide characters cannot be drawn)
    locale.setlocale(locale.LC_ALL, "")

    # curses and the panel are **deferred to here**: neither --selftest nor the real-
    # hardware pre-checks need a terminal
    import curses
    from .app import Ui

    sess = build_session(opts)
    try:
        sess.link.open()   # link errors have to be reported before entering curses
    except Exception as e:                                  # noqa: BLE001
        print(i18n.tr("main.open_failed", opts.port, e))
        if "Permission" in str(e):
            print(i18n.tr("main.permission_hint"))
        return 2
    # The panel's first line states plainly "which device is being used" — grabbing the
    # wrong port is the easiest mistake to make here. Which link the board side uses is
    # fixed at **compile time** (ROBOT_LINK in App/inc/robot_state.h), and picking the two
    # the wrong way round looks like "everything is right but not one byte comes back", so
    # the link kind is spelled out too.
    sess.info(i18n.tr("main.link_info",
                      describe_port(opts.port) if not opts.fake
                      else i18n.tr("main.fake_link")))
    # Two arms over one link, so "which one is being watched right now" has to be visible
    # at the first glance after power-up
    sess.info(i18n.tr("main.two_arms", i18n.arm_label(0), i18n.arm_label(1),
                      i18n.arm_label(opts.arm_id)))
    if not opts.fake and sess.link.kind == "USB":
        sess.info(i18n.tr("main.usb_note", opts.baud))
    elif not opts.fake and sess.link.kind is None:
        sess.warn(i18n.tr("main.unknown_kind"))

    ui = Ui(None, sess, opts)

    def _main(stdscr):
        ui.scr = stdscr
        ui.run()

    try:
        curses.wrapper(_main)
    finally:
        try:
            if opts.stop_on_exit:
                print(i18n.tr("main.stopped",
                              i18n.tr("main.stop_sep").join(sess.stop_all())))
            else:
                print(i18n.tr("main.exit_note"))
            sess.link.close()
        except Exception:                                   # noqa: BLE001
            pass
        if ui.logf:
            ui.logf.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
