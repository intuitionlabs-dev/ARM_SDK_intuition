"""使用实际解码反馈和模拟终端检查 TUI 渲染。"""
import struct
import unittest
from unittest.mock import Mock, patch

from sdk import i18n, protocol
from sdk.tui.__main__ import parse_options
from sdk.tui.app import Ui
from sdk.tui.session import build_session


class RenderTests(unittest.TestCase):
    def test_zero_ack_draws_in_both_languages_without_sending_commands(self):
        self.addCleanup(i18n.set_lang, i18n.get_lang())
        for lang in ("zh", "en"):
            for robot_type in ("arx_x5", "metal_arm"):
                for arm_id in (0, 1):
                    opts = parse_options(["--fake", "--robot-type", robot_type,
                                          "--lang", lang, "--arm-id", str(arm_id)])
                    session = build_session(opts)
                    profile = session.board_defs
                    feedback = protocol.encode_feedback(
                        profile.MODE_STOP, profile.STATE_WAIT_INIT, 0, 0x7f,
                        (0.,) * 6, (0.,) * 6, 0., (30,) * 7)
                    session.link.inject_bytes(protocol.encode(
                        protocol.TID_ARM_FEEDBACK, feedback, arm_id))
                    session.have_ever_rx = True
                    screen = Mock()
                    screen.getmaxyx.return_value = (80, 300)
                    ui = Ui(screen, session, opts)
                    for state in (*range(7), 255):
                        for results in ((0, 1, 2, 3, 4, 5, 2), (5,) * 6 + (2,)):
                            with self.subTest(lang=lang, robot_type=robot_type,
                                              arm_id=arm_id, state=state, results=results):
                                payload = struct.pack("<8BI", state, *results, 321)
                                session.link.inject_bytes(protocol.encode(
                                    protocol.TID_MOTOR_SET_ZERO_ACK, payload, arm_id))
                                screen.addstr.reset_mock()
                                before = session.link.stats["n_tx"]
                                with patch("sdk.tui.app.curses.color_pair", return_value=0), \
                                        patch("sdk.tui.app.curses.doupdate"):
                                    ui.draw()
                                lines = [call.args[2] for call in screen.addstr.call_args_list]
                                expected = i18n.tr(
                                    "app.setzero_ack", i18n.zero_state_name(state), 321,
                                    " ".join("%s=%s" % (i18n.axis_name(i), i18n.zero_res_name(r))
                                             for i, r in enumerate(results)))
                                self.assertIn(expected, lines)
                                self.assertEqual(i18n.tr("app.setzero_unconfirmed") in lines,
                                                 1 in results)
                                self.assertEqual(session.link.stats["n_tx"], before)


if __name__ == "__main__":
    unittest.main()
