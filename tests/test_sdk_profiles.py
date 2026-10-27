import unittest
from contextlib import redirect_stderr
from io import StringIO
from types import SimpleNamespace
from unittest.mock import patch

import sdk
from sdk import Arm, Link
from sdk.board_defs import arx_x5 as board_defs
from sdk.board_defs import metal_arm
from arx_x5_control.sdk_adapter import LocalLink as ARXLink
from metal_arm_control.sdk_adapter import LocalLink as MetalLink
from sdk.tui.session import axis_lo_rad, axis_hi_rad, clamp_axis_rad
from sdk.tui.__main__ import parse_options
from sdk.tui.session import build_session
from sdk.sim import SimLink, RESET_S


class ProfileTests(unittest.TestCase):
    def test_default_profile_unchanged(self):
        arm = Arm(0, Link())
        self.assertIs(arm.board_defs, board_defs)
        self.assertEqual(axis_lo_rad(1), board_defs.JOINT_ANGLE_MIN_RAD[1])

    def test_serial_profile_selection_and_feedback_routing(self):
        arx = Link(robot_type="arx_x5")
        metal = Link(robot_type="metal_arm")
        self.assertIs(arx.board_defs, board_defs)
        self.assertIs(metal.board_defs, metal_arm)
        for arm_id in (0, 1):
            arm = metal.arm(arm_id)
            self.assertIs(arm.board_defs, metal_arm)
            self.assertIs(arx.arm(arm_id).board_defs, board_defs)
            payload = sdk.protocol.encode_feedback(
                metal_arm.MODE_STOP, metal_arm.STATE_WAIT_INIT, 0, 0x7F,
                (0., -19.75, 0., 0., 0., 0.), (0.,) * 6, 0., (30,) * 7)
            frame = sdk.protocol.encode(sdk.protocol.TID_ARM_FEEDBACK, payload, arm_id)
            metal.inject_bytes(frame)
            arx.inject_bytes(frame)
            why = []
            arm._check_pose(arm._chan.snapshot_feedback()[0], why)
            self.assertEqual(why, [])
            arx.arm(arm_id)._check_pose(arx.channel(arm_id).snapshot_feedback()[0], why)
            self.assertTrue(why)  # Same feedback exceeds the ARX reset-pose tolerance.
            self.assertAlmostEqual(arm.snapshot().joint_angles_deg[1], -19.75)
        self.assertIs(Link().board_defs, board_defs)

    def test_unknown_serial_profile_is_rejected(self):
        for value in ("metal", "", "unknown", 123, []):
            with self.subTest(robot_type=value), self.assertRaisesRegex(ValueError, "robot_type"):
                Link(robot_type=value)

    def test_tui_profile_controls_jog_limits_for_both_arms(self):
        for fake in (False, True):
            for name, definitions in (("arx_x5", board_defs), ("metal_arm", metal_arm)):
                with self.subTest(fake=fake, robot_type=name):
                    args = ["--robot-type", name] + (["--fake"] if fake else [])
                    session = build_session(parse_options(args))
                    self.assertIs(session.board_defs, definitions)
                    self.assertIs(session.link.board_defs, definitions)
                    self.assertEqual(session.link.is_open, False)
                    for arm_id in (0, 1):
                        payload = sdk.protocol.encode_feedback(
                            definitions.MODE_STOP, definitions.STATE_WAIT_INIT, 0, 0x7F,
                            definitions.RESET_POSE_DEG, (0.,) * 6, 0., (30,) * 7)
                        session.link.inject_bytes(sdk.protocol.encode(
                            sdk.protocol.TID_ARM_FEEDBACK, payload, arm_id))
                        session.switch_arm(arm_id)
                        self.assertIs(session.cur.arm.board_defs, definitions)
                        session.cur.target[1] = 0.0
                        session.jog(1, 1)
                        if name == "metal_arm":
                            self.assertEqual(session.cur.target[1], 0.0)  # J2 cannot go positive.
                        else:
                            self.assertGreater(session.cur.target[1], 0.0)

    def test_tui_options_use_selected_tolerance_and_validate_type(self):
        self.assertEqual(parse_options([]).robot_type, "arx_x5")
        with patch.object(metal_arm, "ROBOT_INIT_POSE_TOL_RAD", 0.125):
            self.assertEqual(parse_options(["--robot-type", "metal_arm"]).init_tol, 0.125)
            self.assertEqual(parse_options([
                "--robot-type", "metal_arm", "--init-tol", "0.3"]).init_tol, 0.3)
        with redirect_stderr(StringIO()), self.assertRaises(SystemExit) as raised:
            parse_options(["--robot-type", "unknown"])
        self.assertEqual(raised.exception.code, 2)

    def test_fake_reset_uses_selected_pose(self):
        for name, definitions in (("arx_x5", board_defs), ("metal_arm", metal_arm)):
            with self.subTest(robot_type=name):
                link = SimLink(robot_type=name)
                # Drive virtual time directly, without a background thread or serial port.
                link._alive = True
                try:
                    board = link.boards[0]
                    board.state = definitions.STATE_NORMAL
                    board.joint = [10.] * 6
                    board.gripper = 15.
                    link.send(sdk.protocol.TID_ARM_RESET, arm_id=0)
                    self.assertEqual(board.target, list(definitions.RESET_POSE_DEG) + [15.])
                    link._step(board, RESET_S + 0.01)
                    self.assertEqual(board.joint, list(definitions.RESET_POSE_DEG))
                    self.assertEqual(board.gripper, 15.)
                    self.assertEqual(board.state, definitions.STATE_NORMAL)
                    self.assertEqual(board.mode, definitions.MODE_DAMPING)
                finally:
                    link.close()

    def test_profile_is_per_link(self):
        link = Link()
        link.board_defs = SimpleNamespace(
            ARM_NUM=2,
            RESET_POSE_DEG=(0., -45., 45., 0., 0., 0.),
            ROBOT_INIT_POSE_TOL_DEG=20.,
            JOINT_ANGLE_MIN_RAD=(-1.,)*6, JOINT_ANGLE_MAX_RAD=(1.,)*6,
            GRIPPER_ANGLE_MIN_RAD=0., GRIPPER_ANGLE_MAX_RAD=2.4,
        )
        why = []
        Arm(0, link)._check_pose(SimpleNamespace(joint_angle=link.board_defs.RESET_POSE_DEG), why)
        self.assertEqual(why, [])
        Arm(0, Link())._check_pose(SimpleNamespace(joint_angle=link.board_defs.RESET_POSE_DEG), why)
        self.assertTrue(why)
        self.assertEqual(clamp_axis_rad(1, -2., link.board_defs), (-1., True))
        self.assertEqual(axis_hi_rad(6, link.board_defs), 2.4)
        self.assertEqual(axis_lo_rad(6, link.board_defs), 0.)
        store = object()
        link.gripper_calib_store = store
        self.assertIs(Arm(0, link)._calib_store, store)

    def test_robot_modules_bind_helpers_to_their_own_limits(self):
        self.assertFalse(hasattr(sdk.board_defs, 'JOINT_ANGLE_MIN'))
        for factory, definitions in ((ARXLink, board_defs), (MetalLink, metal_arm)):
            link = factory.simulated(background=False)
            arm = Arm(0, link)
            self.assertIs(link.board_defs, definitions)
            self.assertIs(arm.board_defs, definitions)
            self.assertEqual(definitions.pose_deviation_deg(definitions.RESET_POSE_DEG), 0)
        self.assertEqual(board_defs.clamp_axis(1, -30), (0.5, True))
        self.assertEqual(metal_arm.clamp_axis(1, -30), (-30, False))
        self.assertEqual(metal_arm.clamp_axis(6, -30), (0.0, True))
        self.assertEqual(board_defs.clamp_axis(6, -30), (-30, False))

    def test_control_core_and_sdk_share_the_profile(self):
        from arx_x5_control import config as arx
        from metal_arm_control import config as metal
        for config, definitions in ((arx, board_defs), (metal, metal_arm)):
            self.assertIs(config.JOINT_MIN, definitions.JOINT_ANGLE_MIN)
            self.assertIs(config.JOINT_MAX, definitions.JOINT_ANGLE_MAX)
            self.assertIs(config.RESET_ANGLE, definitions.RESET_POSE_DEG)
        with self.assertRaises(ValueError):
            Arm(2, MetalLink.simulated(background=False))


if __name__ == '__main__':
    unittest.main()
