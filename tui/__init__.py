"""On-bench console (full-screen curses panel) — run with `python3 -m sdk.tui`; works on a
real board or a fake one.

    session.py    backend/adapter layer: `ConsoleSession` (one consistent view per frame,
                  tick-per-beat, key actions), `ArmView` (one piece of state per arm),
                  step presets and the unit/limit convention. **Does not import curses**
    app.py        curses front end: `Ui` (draw / keys / dialogs). Only knows `session`
    selftest.py   console-side offline self-test (48 items, runs against a fake board)
    __main__.py   argparse + `main()`: `python3 -m sdk.tui [--fake] [--selftest] [--lang en]`

**This package's `__init__` deliberately re-exports the backend only** (the names below)
and never touches curses — so `import sdk.tui` works where there is no terminal (self-tests,
scripts driving a fake board via `ConsoleSession`), and the panel itself is imported only
when you actually open a screen.

What the panel needs, in one sentence: `sess.snapshot()` gives one consistent view per
frame, `sess.tick()` advances one beat, `sess.jog/cycle_step/send_*` are the key actions —
details live in `session.py`'s docstring. `scripts/uart_frame_cli.py` is a compatibility
shim that forwards to `python3 -m sdk.tui`.

Panel language: Chinese by default, `--lang en` for English. That is **one code path plus
the language table in `sdk/i18n.py`**, not two panels, and the Chinese output is
byte-for-byte what it was before the table existed. The name tables in
`sdk/board_defs.py`, the sdk-side self-test's own output and `calibrate_gripper.py`'s own
prompts stay Chinese on purpose — see sdk/README, the "UI language" section.
"""

from .session import (DEFAULT_ARM_ID, DEFAULT_STEP_M, DEFAULT_STEP_RAD,
                      FEEDBACK_FRESH_S, GRIPPER, MODE_TID_NAME, N_ARM, N_AXIS,
                      STEP_CHOICES_M, STEP_CHOICES_RAD, ArmView,
                      ConsoleSession, axis_hi_rad, axis_lo_rad, build_session,
                      clamp_axis_rad, gripper_m_limits, parse_arm_id,
                      short_label)

__all__ = [
    # session
    "ConsoleSession", "ArmView", "build_session",
    # units / helpers
    "parse_arm_id", "short_label", "axis_lo_rad", "axis_hi_rad", "clamp_axis_rad",
    "gripper_m_limits",
    # constants
    "DEFAULT_ARM_ID", "N_ARM", "N_AXIS", "GRIPPER", "STEP_CHOICES_RAD",
    "STEP_CHOICES_M", "DEFAULT_STEP_RAD", "DEFAULT_STEP_M", "FEEDBACK_FRESH_S",
    "MODE_TID_NAME",
]
