"""上机控制台（curses 全屏面板）—— `python3 -m sdk.tui` 跑，真机/假板子都行。

    session.py    后端/适配层：`ConsoleSession`（一帧一致视图、每拍 tick、按键动作）、
                  `ArmView`（每台臂一份状态）、步长档与限位口径。**不 import curses**
    app.py        curses 前端：`Ui`（画屏 / 按键 / 对话框）。只认 `session` 那几样接口
    selftest.py   控制台侧离线自测（44 项，用假板子跑）
    __main__.py   argparse + `main()`：`python3 -m sdk.tui [--fake] [--selftest]`

**这个包的 `__init__` 刻意只重导出后端**（下面那些名字），不碰 curses —— 于是
`import sdk.tui` 在没终端的地方也能用（自测、脚本里拿 `ConsoleSession` 驱动假板子），
面板本身只在你真要开屏时才被 import。

面板要的东西一句话说完：`sess.snapshot()` 给一帧一致视图、`sess.tick()` 每拍推进、
`sess.jog/cycle_step/send_*` 是按键动作 —— 细节在 `session.py` 的 docstring 里。
`scripts/uart_frame_cli.py` 是兼容壳，转调的就是 `python3 -m sdk.tui`。
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
    # 口径/工具
    "parse_arm_id", "short_label", "axis_lo_rad", "axis_hi_rad", "clamp_axis_rad",
    "gripper_m_limits",
    # 常量
    "DEFAULT_ARM_ID", "N_ARM", "N_AXIS", "GRIPPER", "STEP_CHOICES_RAD",
    "STEP_CHOICES_M", "DEFAULT_STEP_RAD", "DEFAULT_STEP_M", "FEEDBACK_FRESH_S",
    "MODE_TID_NAME",
]
