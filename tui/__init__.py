"""机械臂终端控制台，入口为 python3 -m sdk.tui。

session.py 管理双臂会话，app.py 绘制 curses 界面，selftest.py 执行离线检查。
导入本包仅加载会话层，不加载 curses。--fake 使用模拟器，--lang en 切换英文。
"""

from .session import (DEFAULT_ARM_ID, DEFAULT_STEP_M, DEFAULT_STEP_RAD,
                      FEEDBACK_FRESH_S, GRIPPER, MODE_TID_NAME, N_ARM, N_AXIS,
                      STEP_CHOICES_M, STEP_CHOICES_RAD, ArmView,
                      ConsoleSession, axis_hi_rad, axis_lo_rad, build_session,
                      clamp_axis_rad, gripper_m_limits, parse_arm_id,
                      short_label)

__all__ = [
    # 会话
    "ConsoleSession", "ArmView", "build_session",
    # 单位与辅助函数
    "parse_arm_id", "short_label", "axis_lo_rad", "axis_hi_rad", "clamp_axis_rad",
    "gripper_m_limits",
    # 常量
    "DEFAULT_ARM_ID", "N_ARM", "N_AXIS", "GRIPPER", "STEP_CHOICES_RAD",
    "STEP_CHOICES_M", "DEFAULT_STEP_RAD", "DEFAULT_STEP_M", "FEEDBACK_FRESH_S",
    "MODE_TID_NAME",
]
