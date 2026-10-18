"""SDK TUI 的本地 CAN 模式显示适配。"""

from sdk import i18n
from sdk.tui.app import Ui
import time

from .. import log


class LocalUi(Ui):
    def on_key(self, ch):
        recorder = getattr(self.sess, "recorder", None)
        if recorder is None:
            return super().on_key(ch)
        received_ns = time.monotonic_ns()
        before = dict(arm_id=self.sess.cur_arm_id, axis=self.sess.sel,
                      target_rad=tuple(self.sess.target), step=self.sess.step,
                      step_unit="m" if self.sess.step_in_m else "rad")
        try:
            if ch in (ord("m"), ord("M")):
                recorder.emit("marker", dict(received_ns=received_ns, **before))
                self.sess.info(log.text("tui.marker", language=self.opts.lang))
            else:
                super().on_key(ch)
        finally:
            completed_ns = time.monotonic_ns()
            recorder.emit("input", dict(key_code=ch, received_ns=received_ns, completed_ns=completed_ns,
                before=before, after=dict(arm_id=self.sess.cur_arm_id, axis=self.sess.sel,
                                         target_rad=tuple(self.sess.target))))
            recorder.probe.tick("ui.key_input.period", received_ns)
            recorder.probe.observe("ui.key_handler.duration", completed_ns-received_ns)

    def draw(self):
        recorder = getattr(self.sess, "recorder", None)
        if recorder is None:
            return super().draw()
        started_ns = time.monotonic_ns()
        recorder.probe.tick("ui.draw.period", started_ns)
        try:
            return super().draw()
        finally:
            recorder.probe.observe("ui.draw.duration", time.monotonic_ns()-started_ns)

    def put(self, y, x, message, a=0):
        prefix = i18n.tr("app.arms_line", "")
        if message.startswith(prefix):
            message = log.text("tui.arms_prefix", language=self.opts.lang)+message[len(prefix):]
        if self.show_help:
            for key in ("exit", "misc", "arm", "two1", "two2"):
                if message == i18n.tr("app.help."+key):
                    message = log.text("tui.help."+key, language=self.opts.lang)
                    break
        return super().put(y, x, message, a)
