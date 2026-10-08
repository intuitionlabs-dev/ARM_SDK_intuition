"""机械臂上位机控制台（curses 全屏面板）。UART / USB 两条链路、两台臂都能跑。

**后端就是 `sdk/`**：协议、串口链路、按 arm_id 分流、闸门预检、单位换算全在那儿；
面板要的"一帧一致视图"由 [session.py](session.py) 的
`ConsoleSession.snapshot()` 给，屏幕上的东西在 [app.py](app.py)。所以本模块只剩
命令行壳 —— 改协议、加命令只动 `sdk/`，这里最多改排版。

和板子说话的方式（口 protocol_doc.md / sdk/README）：

  * **周期发 0x16**（6 个关节角 + 夹爪角），**它同时就是保活帧** —— 板子那边 500 ms
    没收到任意有效帧就进 comm_lost(damping)，而被丢掉的 0x16 也算有效帧（robot_state.c
    的 s_last_rx_ms 在分派之前刷新），所以任何时候发都不亏。
  * **收 0x80 反馈帧**显示 mode / robot_state / 关节角 / 掉线掩码 / 夹爪角。
  * 按键步进关节目标角，目标角按板子的软限位在**上位机**先夹住：板子也会夹
    （gimbal_cmd.c），但上位机不夹的话面板上显示的目标角就是骗人的。

**单位：面板上是 SI** —— 关节 rad / rad·s⁻¹、夹爪 m（要标定，见 sdk/README「夹爪标定」）。
线上是 deg，换算在 sdk 里做。**事件日志仍然写 deg**（和固件、和别人的日志对得上），
所以"面板上 0.0175 rad、日志里 1.0°"说的是同一件事，别以为哪个错了。

会话流程（板子的状态机，详见 sdk/README「一次会话的生命周期」）：
    ① 等握手（板子只认 0x01 版本包，版本码不符会**静默锁死**，只能复位板子）
    ② 离线等待（7 台电机全上线后板子自己走）
    ③ 待初始化 —— 按 i 发 0x02
    ④ 初始化中（板子自己跑一键复位 → 一键夹爪找零，约 7 s，期间必须保持保活）
    ⑤ 正常运行（**默认 damping**！按 s 发 0x12 才进 run，run 下 0x16 才被接受）
       ↑ 这一档（stop/damping）下**也能按 i 重跑一遍初始化**；跑完回 ⑤ 而不是 ③。
         （板侧闸门放开是用户要求的口径；run 档下按 i 仍会被静默丢弃。）
    ⑥ 按 z 发 0x10 设零点 → 校准，跑完回 ③
    ⑦ 500 ms 无有效帧 → comm_lost；再来一帧就回 ②

**两台臂**：一块板子两块板卡、两条 CAN、两台臂，但上位机只有**一条链路**。所以一个
进程 = 一条链路 + 两个 `sdk.Arm`，**两台都在被驱动**（各自的目标角、各自的 0x16 流、
各自的保活计时），面板上按 `a` 切的是"在看/在操作哪一台"。两台的状态机完全独立 ——
一台还停在待初始化、另一台已经 run，是正常的（i/o/s/d/v/x 两台都要各自按一遍）。

**一次只能开一个进程**：板子的上位机链路是编译期二选一的一路（robot_state.h 的
ROBOT_LINK），物理上就一个口子，两个进程同时开同一个 tty 会互相抢字节、把帧拆散。

**安全**：默认退出**不发** 0x14（停止），只停发 0x16 —— 目标冻结、臂保持原样，
约 500 ms 后板子自己因无帧进 damping。要立刻松劲请手动按 x，或加 --stop-on-exit。

用法（在仓库根跑 —— `sdk` 是仓库里的包，没打包安装）：
    python3 -m sdk.tui                       # 真机：自动挑口子（板子 USB 优先，
                                             # 退回 1a86 RS485 适配器），921600
    python3 -m sdk.tui --port /dev/ttyACM0   # 指定口子
    python3 -m sdk.tui --arm-id 1            # 启动时「当前臂」看 CAN2 那台（a 可切）
    python3 -m sdk.tui --fake                # 假板子，不接硬件就能把面板/按键/状态机走一遍
    python3 -m sdk.tui --selftest            # 离线自测（sdk + 控制台后端），不接硬件
旧路径 `python3 scripts/uart_frame_cli.py ...` 参数一模一样，它是转调这里的兼容壳。
面板里按 h 看按键表。"""


import argparse
import locale
import os
import sys

from .. import board_defs, i18n
from ..errors import LinkError
from ..link import (assert_port_allowed, describe_port, link_hint,
                    list_links, pick_default_port)
from .session import (DEFAULT_ARM_ID, DEFAULT_STEP_RAD, STEP_CHOICES_RAD,
                      build_session, parse_arm_id)
from .selftest import selftest


def _pre_lang(argv):
    """先单独把 `--lang` 摘出来。

    必须在建真 parser **之前**定语言：`--help` 和 argparse 自己的报错都发生在
    `parse_args` 里，那时候再 `set_lang` 已经晚了（`help=` 文本早就是中文了）。
    预解析器不带 `choices`、不开 `-h`，所以它自己永远不会 `SystemExit`；
    `set_lang` 只认 zh/en，其余（含 None）一律回落 zh。
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
        prog=prog or "python3 -m sdk.tui",   # -m 下不给就显示成 "__main__.py"
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
            # 探针口在这里被拒（SDK 的文案里写了为什么），别把这条提示弄丢
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

    # curses 里要显示中文，必须在 initscr **之前**设好 locale（否则宽字符画不出来）
    locale.setlocale(locale.LC_ALL, "")

    # curses 与面板**延迟到这里**：--selftest 和真机预检那几条路都不需要终端
    import curses
    from .app import Ui

    sess = build_session(opts)
    try:
        sess.link.open()   # 链路错误要在进 curses 之前报出来
    except Exception as e:                                  # noqa: BLE001
        print(i18n.tr("main.open_failed", opts.port, e))
        if "Permission" in str(e):
            print(i18n.tr("main.permission_hint"))
        return 2
    # 面板第一行就把"抓的是哪个设备"写清楚 —— 抓错口子是这里最容易犯的错。
    # 板子那侧走哪条链路是**编译期**定死的（App/inc/robot_state.h 的 ROBOT_LINK），
    # 两边选反了的现象是"一切都对但一个字节都没有"，所以把链路类型也标出来。
    sess.info(i18n.tr("main.link_info",
                      describe_port(opts.port) if not opts.fake
                      else i18n.tr("main.fake_link")))
    # 两台臂一条链路，所以"现在在看哪一台"必须是开机第一眼就看得见的东西
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
