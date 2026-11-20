"""H7 控制台命令行入口。

python3 -m sdk.tui --port /dev/ttyACM0 --robot-type arx_x5
--fake 使用协议模拟器，--selftest 运行离线检查，--lang en 切换英文。
双臂共享串口，按 a 切换操作对象，按 h 查看帮助。

界面使用 rad 和 m，角度日志使用 deg。各臂独立发送目标或保活。
默认退出仅关闭链路；--stop-on-exit 向双臂发送 STOP，设备随后按固件策略处理。
"""


import argparse
import locale
import os
import sys

from .. import i18n
from ..board_defs import ROBOT_TYPES, get_profile
from ..errors import LinkError
from ..link import (assert_port_allowed, describe_port, link_hint,
                    list_links, pick_default_port)
from .session import (DEFAULT_ARM_ID, DEFAULT_STEP_RAD, STEP_CHOICES_RAD,
                      build_session, parse_arm_id)
from .selftest import selftest


def _pre_lang(argv):
    """先解析 --lang，使参数帮助和错误提示使用所选语言。"""
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument("--lang", nargs="?", default=None)
    try:
        return i18n.set_lang(pre.parse_known_args(argv)[0].lang)
    except SystemExit:
        return i18n.set_lang(None)


def parse_options(argv=None, prog=None):
    _pre_lang(argv)
    ap = argparse.ArgumentParser(
        prog=prog or "python3 -m sdk.tui",   # 指定 -m 入口的帮助名称
        description=i18n.tr("main.cli_description"),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=i18n.tr("main.epilog"))
    ap.add_argument("--port", default=None, help=i18n.tr("main.help.port"))
    ap.add_argument("--baud", type=int, default=921600, help=i18n.tr("main.help.baud"))
    ap.add_argument("--robot-type", choices=ROBOT_TYPES, default="arx_x5",
                    help=i18n.tr("main.help.robot_type"))
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
    ap.add_argument("--init-tol", type=float, default=None,
                    help=i18n.tr("main.help.init_tol"))
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
    if opts.init_tol is None:
        opts.init_tol = get_profile(opts.robot_type).ROBOT_INIT_POSE_TOL_RAD
    return opts


def main(argv=None, prog=None):
    opts = parse_options(argv, prog)

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
            # 保留 SDK 的探针端口拒绝原因。
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

    # 初始化 curses 前设置 locale，以显示中文。
    locale.setlocale(locale.LC_ALL, "")

    # 仅实际打开界面时加载 curses。
    import curses
    from .app import Ui

    sess = build_session(opts)
    try:
        sess.link.open()   # 进入界面前报告链路错误。
    except Exception as e:                                  # noqa: BLE001
        print(i18n.tr("main.open_failed", opts.port, e))
        if "Permission" in str(e):
            print(i18n.tr("main.permission_hint"))
        return 2
    # 显示设备路径和链路类型，便于核对固件接线。
    sess.info(i18n.tr("main.link_info",
                      describe_port(opts.port) if not opts.fake
                      else i18n.tr("main.fake_link")))
    sess.info(i18n.tr("main.robot_type", opts.robot_type))
    # 显示共享链路中的双臂及当前操作对象。
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
