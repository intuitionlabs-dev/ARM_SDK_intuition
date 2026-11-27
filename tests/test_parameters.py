import struct
import unittest
from unittest.mock import patch

from sdk import Link, ParameterFeedback, protocol as p
from sdk.errors import GateRejected
from sdk.sim import SimLink


class ParameterTests(unittest.TestCase):
    def test_wire_layout_and_capacity(self):
        self.assertEqual(p.encode_parameter_set({0: 2}), b"\x00\x00\x02\x00\x00\x00")
        self.assertEqual(p.encode_parameter_set(), b"")
        self.assertEqual(len(p.encode_parameter_set([(0, 1)] * 42)), 252)
        for items in ([(0, 1)] * 43, [(-1, 0)], [(65536, 0)],
                      [(0, -1)], [(0, 2**32)], [(0, 1.0)], [(True, 0)]):
            with self.subTest(items=items[:1]), self.assertRaises(ValueError):
                p.encode_parameter_set(items)

    def test_feedback_length_and_routing(self):
        link = Link()
        arm = link.arm(1)
        frame = p.encode(p.TID_PARAMETER_FEEDBACK, struct.pack("<I", 2), 1)
        link.inject_bytes(frame[:5])
        self.assertIsNone(arm.last_parameter_feedback)
        link.inject_bytes(frame[5:])
        self.assertEqual(arm.last_parameter_feedback, ParameterFeedback((2,)))
        self.assertEqual(arm.last_parameter_feedback.post_process_algorithm, 2)
        self.assertIsNone(link.arm(0).last_parameter_feedback)
        for payload in (b"", b"\x00" * 3, b"\x00" * 6, b"\x00" * 8):
            with self.assertRaises(ValueError):
                p.decode_parameter_feedback(payload)
            link.inject_bytes(p.encode(p.TID_PARAMETER_FEEDBACK, payload, 1))
        self.assertEqual(link.channel(1).snapshot_parameters()[2], 1)
        link._on_serial_open()
        self.assertIsNone(arm.last_parameter_feedback)

    def test_query_requires_new_feedback(self):
        link = Link()
        arm = link.arm(0)
        link.inject_bytes(p.encode(p.TID_PARAMETER_FEEDBACK, struct.pack("<I", 1), 0))
        with patch.object(link, "send", return_value=8) as send:
            with self.assertRaises(TimeoutError):
                arm.query_parameters(timeout=0.01)
        send.assert_called_once_with(p.TID_PARAMETER_SET, b"", 0)

    def test_simulator_batch_gates_and_reopen(self):
        for model, default in (("arx_x5", 2), ("metal_arm", 1)):
            with self.subTest(model=model), SimLink(robot_type=model) as link:
                arm = link.arm(0)
                arm.send_version()
                self.assertEqual(arm.query_parameters().post_process_algorithm, default)
                self.assertIsNone(link.arm(1).last_parameter_feedback)
                b = link.boards[0]
                with link._lock:
                    b.state = arm.board_defs.STATE_NORMAL
                    b.mode = arm.board_defs.MODE_STOP
                    fb = b.feedback_payload()
                link.inject_bytes(p.encode(p.TID_ARM_FEEDBACK, fb, 0))
                for algorithm in (0, 1, 2):
                    arm.set_parameters({0: algorithm})
                    self.assertEqual(arm.query_parameters().param, (algorithm,))
                arm.set_parameters([(0, 0), (65535, 99), (0, 1), (0, 99)])
                self.assertEqual(arm.query_parameters().param, (1,))
                arm.send_raw(p.TID_PARAMETER_SET, p.encode_parameter_set({0: 2}) + b"\x00")
                self.assertEqual(arm.query_parameters().param, (1,))
                for state, mode in ((4, 1), (4, 3), (2, 0)):
                    with link._lock:
                        b.state, b.mode = state, mode
                        fb = b.feedback_payload()
                    link.inject_bytes(p.encode(p.TID_ARM_FEEDBACK, fb, 0))
                    with self.assertRaises(GateRejected):
                        arm.set_parameters({0: 0})
                    arm.set_parameters({0: 0}, force=True)
                    self.assertEqual(arm.query_parameters().param, (1,))
                with link._lock:
                    b.state, b.mode = 4, 2
                    fb = b.feedback_payload()
                link.inject_bytes(p.encode(p.TID_ARM_FEEDBACK, fb, 0))
                arm.set_parameters([(0, 0)] * 42)
                self.assertEqual(arm.query_parameters().param, (0,))
                link.close()
                link.open()
                self.assertIsNone(arm.last_parameter_feedback)

    def test_query_retries_after_missing_reply(self):
        link = Link()
        arm = link.arm(0)
        calls = []

        def send(tid, payload, arm_id):
            calls.append((tid, payload, arm_id))
            if len(calls) == 2:
                link.inject_bytes(p.encode(p.TID_PARAMETER_FEEDBACK, struct.pack("<I", 2), 0))
            return 8

        with patch.object(link, "send", side_effect=send):
            self.assertEqual(arm.query_parameters(timeout=0.3).param, (2,))
        self.assertEqual(calls, [(p.TID_PARAMETER_SET, b"", 0)] * 2)


if __name__ == "__main__":
    unittest.main()
