import unittest
from types import SimpleNamespace

from sdk import Arm, Link, board_defs
from sdk.tui.session import axis_lo_rad, axis_hi_rad, clamp_axis_rad


class ProfileTests(unittest.TestCase):
    def test_default_profile_unchanged(self):
        arm = Arm(0, Link())
        self.assertIs(arm.board_defs, board_defs)
        self.assertEqual(axis_lo_rad(1), board_defs.JOINT_ANGLE_MIN_RAD[1])

    def test_profile_is_per_link(self):
        link = Link()
        link.board_defs = SimpleNamespace(
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


if __name__ == '__main__':
    unittest.main()
