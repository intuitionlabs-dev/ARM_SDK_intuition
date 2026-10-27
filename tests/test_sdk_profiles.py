import unittest
from types import SimpleNamespace

import sdk
from sdk import Arm, Link
from sdk.board_defs import arx_x5 as board_defs
from sdk.board_defs import metal_arm
from arx_x5_control.sdk_adapter import LocalLink as ARXLink
from metal_arm_control.sdk_adapter import LocalLink as MetalLink
from sdk.tui.session import axis_lo_rad, axis_hi_rad, clamp_axis_rad


class ProfileTests(unittest.TestCase):
    def test_default_profile_unchanged(self):
        arm = Arm(0, Link())
        self.assertIs(arm.board_defs, board_defs)
        self.assertEqual(axis_lo_rad(1), board_defs.JOINT_ANGLE_MIN_RAD[1])

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
