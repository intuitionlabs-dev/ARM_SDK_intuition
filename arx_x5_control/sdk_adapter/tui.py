"""通过 SDK TUI 操作本地 CAN 机械臂控制器。"""

import argparse
from contextlib import ExitStack, contextmanager
import locale
from math import isfinite
import time

from sdk import board_defs, i18n
from sdk.tui.session import ConsoleSession, DEFAULT_STEP_RAD, STEP_CHOICES_RAD, parse_arm_id

from .. import log
from ..can_io import CANChannel
from ..can_process import ProcessCANMotorIO
from ..config import ArmConfig
from .link import LocalLink
from .telemetry import RuntimeRecorder

_LOG = log.get_logger("arx_x5_control.sdk_adapter.tui")


def parse_options(argv=None):
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument("--lang", choices=("zh", "en"), default="zh")
    language = pre.parse_known_args(argv)[0].lang
    i18n.set_lang(language)
    parser = argparse.ArgumentParser(description=log.text("tui.description", language=language))
    parser.add_argument("--lang", choices=("zh", "en"), default=language)
    parser.add_argument("--simulate", action="store_true", help=log.text("tui.simulate", language=language))
    parser.add_argument("--interface", default="slcan")
    parser.add_argument("--transport-config", help=log.text("can.transport_config", language=language))
    parser.add_argument("--channel", action="append", default=[],
                        help=log.text("can_check.channel", language=language))
    parser.add_argument("--arm-id", type=parse_arm_id, default=0, help=i18n.tr("main.help.arm_id", 0))
    parser.add_argument("--ctrl-hz", type=float, default=100.0, help=i18n.tr("main.help.ctrl_hz"))
    parser.add_argument("--fb-hz", type=float, default=5.0, help=i18n.tr("main.help.fb_hz"))
    parser.add_argument("--step", type=float, default=DEFAULT_STEP_RAD, choices=STEP_CHOICES_RAD)
    parser.add_argument("--log", help=i18n.tr("main.help.log"))
    parser.add_argument("--log-dir", help=log.text("tui.log_dir", language=language))
    options = parser.parse_args(argv)
    if options.simulate and options.channel:
        parser.error(log.text("tui.simulate_channels", language=language))
    if not options.simulate and not 1 <= len(options.channel) <= 2:
        parser.error(log.text("tui.channel_count", language=language))
    if not options.simulate and options.arm_id >= len(options.channel):
        parser.error(log.text("tui.arm_unavailable", language=language))
    if not all(isfinite(x) and 0 < x <= 200 for x in (options.ctrl_hz, options.fb_hz)):
        parser.error(log.text("tui.rates", language=language))
    options.fake = options.simulate
    options.port = "LOCAL-SIM" if options.simulate else "CAN-"+options.interface
    options.baud = 1000000
    options.handshake_timeout = 3.0
    options.init_tol = board_defs.ROBOT_INIT_POSE_TOL_RAD
    return options


class LocalConsoleSession(ConsoleSession):
    """将控制核心的异常事件显示到 SDK TUI 事件栏。"""

    def __init__(self, link, options, recorder=None):
        super().__init__(link, options, arm_ids=range(len(link.configurations)))
        self.recorder = recorder
        self._recording_error_shown = False
        self._command_sequence = 0

    def switch_arm(self, arm_id=None):
        arm_ids = tuple(self.views)
        if arm_id is None:
            arm_id = arm_ids[(arm_ids.index(self.cur_arm_id)+1) % len(arm_ids)]
        if arm_id in self.views:
            super().switch_arm(arm_id)

    def event(self, level, text):
        super().event(level, text)
        if self.recorder is not None:
            self.recorder.emit("ui", dict(level=level, text=text))

    def _send_ctrl(self, view, now):
        started_ns = time.monotonic_ns()
        target = tuple(view.target)
        sent = super()._send_ctrl(view, now)
        if self.recorder is not None:
            self._command_sequence += 1
            self.recorder.emit("command", dict(sequence=self._command_sequence, arm_id=view.arm_id,
                started_ns=started_ns, completed_ns=time.monotonic_ns(), sent=sent, target_rad=target))
        return sent

    def tick(self, now):
        start = time.monotonic_ns()
        recorder = self.recorder
        if recorder is not None:
            recorder.probe.tick("ui.period", start)
        try:
            super().tick(now)
            for event in self.link.drain_events():
                if recorder is not None:
                    recorder.emit("control", event)
                if event.kind in ("rejected", "auto_aborted", "initialization_failed",
                                  "gravity_error", "dynamics_error", "reference_error"):
                    self.err(log.event_text(event, language=self.opts.lang))
            if recorder is not None and recorder.error and not self._recording_error_shown:
                self._recording_error_shown = True
                self.err(log.text("tui.log_failed", error=recorder.error))
        finally:
            if recorder is not None:
                recorder.probe.observe("ui.duration", time.monotonic_ns()-start)


@contextmanager
def open_session(options, recorder=None):
    """按 CAN 进程、控制链路的顺序打开，退出时先停控制再关闭 CAN。"""
    with ExitStack() as stack:
        motor_io = None
        if options.simulate:
            link = LocalLink.simulated(collect_performance=recorder is not None, collect_trace=recorder is not None)
        else:
            channels = [CANChannel(i, channel, interface=options.interface)
                        for i, channel in enumerate(options.channel)]
            motor_io = stack.enter_context(ProcessCANMotorIO(channels, collect_performance=recorder is not None,
                                                            transport_config=options.transport_config))
            link = LocalLink(motor_io, configurations=[ArmConfig() for _ in channels],
                             collect_performance=recorder is not None, collect_trace=recorder is not None)
        if recorder is not None:
            recorder.attach(link, motor_io)
        stack.enter_context(link)
        session = LocalConsoleSession(link, options, recorder)
        if recorder is not None:
            recorder.session(session)
        yield session


def main(argv=None):
    options = parse_options(argv)
    log.configure(language=options.lang, stream=None)
    ui = None
    recorder = None
    result = 0
    try:
        locale.setlocale(locale.LC_ALL, "")
        import curses
        from .tui_app import LocalUi

        with RuntimeRecorder(options) as recorder, open_session(options, recorder) as session:
            session.info(log.text("tui.connected"))
            session.info(log.text("tui.logs_saved", path=str(recorder.directory)))
            ui = LocalUi(None, session, options)

            def run_screen(screen):
                ui.scr = screen
                ui.run()

            curses.wrapper(run_screen)
    except KeyboardInterrupt:
        result = 130
    except Exception as error:
        log.configure_console()
        _LOG.error("tui.failed", error=error)
        result = 1
    finally:
        if ui is not None and ui.logf is not None:
            ui.logf.close()
    log.configure_console()
    if recorder is not None:
        _LOG.info("tui.logs_saved", path=str(recorder.directory))
        if recorder.error:
            _LOG.error("tui.log_failed", error=recorder.error)
    if result in (0, 130):
        _LOG.info("tui.stopped")
    return result


if __name__ == "__main__":
    raise SystemExit(main())
