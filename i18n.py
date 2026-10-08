"""Language tables for everything a human reads: panel text, event log, exceptions.

`set_lang("en")` switches the *same* code path to English; the default is `zh` and the
Chinese output is byte-for-byte what it was before this module existed (every `zh` entry
below is a literal lifted verbatim out of the call site it replaced).

Two tables, identical key sets, keyed `<module>.<name>`:

    _ZH / _EN      messages, `%`-templates (never tuples or lambdas -- see `check_tables`)

Name tables (state / mode / axis / arm labels) live here too, because `sdk/arm.py` and
`sdk/link.py` put them inside exception messages. The `zh` branches delegate straight to
`board_defs`, so there is exactly one Chinese source of truth; only the English side is a
second table here. `board_defs` itself is deliberately untouched -- `ArmSnapshot.mode_name`
and friends are public API and must keep returning Chinese.

A missing key never raises at runtime: `tr()` falls back to the Chinese entry (and to a
visible `[missing:...]` marker if the key is nowhere). A console driving a robot arm must not
die because one translation is missing. Strictness lives in `check_tables()`, which the
self-test calls.
"""
import re

from . import board_defs

LANGS = ("zh", "en")
DEFAULT_LANG = "zh"

_LANG = DEFAULT_LANG


def set_lang(code):
    """Switch language. **Never raises**: anything but `zh`/`en` (including None) -> `zh`.

    The value comes from a pre-parse of argv, and a console that refuses to start because
    of an ambiguous argument would be worse than one showing the wrong language.
    """
    global _LANG
    _LANG = code if code in LANGS else DEFAULT_LANG
    return _LANG


def get_lang():
    return _LANG


# ============================================================================
# Messages
# ============================================================================

_ZH = {}
_EN = {}

# ---- sdk/tui/session.py：事件流 / 对话框文案 ----
_ZH.update({
    "session.tid.start": "0x12 启动",
    "session.tid.gravity": "0x13 重力补偿",
    "session.tid.stop": "0x14 停止",
    "session.tid.damping": "0x15 阻尼",
    "session.arm_id.type": "--arm-id 要是整数（0..%d）",
    "session.arm_id.range": "--arm-id 只能是 0..%d（板上 ARM_NUM=%d）",
    "session.switch_arm": "面板切到 %s（两台都在被驱动，切走的那台继续跟它自己的目标角）",
    "session.no_feedback": "无反馈",
    "session.seeded": "[%s] 目标角已对齐反馈（%s）",
    "session.no_feedback_yet": "还没收到反馈帧，先别动 —— 没有反馈就无法把目标对齐当前位姿",
    "session.grip_limit_m": "[%s] 夹爪请求 %.4f m 超出软限位 [%.4f, %.4f] m，已停在 %.4f m",
    "session.axis_limit": "[%s] %s 请求 %.2f° 超出软限位 [%.2f, %.2f]，已停在 %.2f°",
    "session.step": "[%s] 步长 = %s",
    "session.send_gated": "[%s] %s 没发出去：%s",
    "session.send_error": "[%s] %s 出错：%s",
    "session.sent_mode": "[%s] 已发 %s（%s）",
    "session.init_no_fb": "[%s] 还没有反馈帧，不发 0x02（不知道板子在哪一步）",
    "session.init_pose_bad":
        "[%s] 姿态超出复位位姿 ±%.0f°（%s）—— 先在失能/阻尼下摆回 %s；"
        "再按一次 i 可强发（板子判据相同，超了照样静默丢弃）",
    "session.init_force": "[%s] （强发 0x02：跳过姿态这一条预检，其余闸门照查）",
    "session.what.init": "0x02 初始化",
    "session.init_sent":
        "[%s] 已发 0x02 初始化：板子将跑「一键复位 → 一键夹爪找零」，约 7 s，别松保活",
    "session.what.reset": "0x18 一键复位",
    "session.reset_sent":
        "[%s] 已发 0x18 一键复位：约 2 s 回到 %s，跑完停 damping、**0x16 自动停发**"
        "（要续位控按 s）",
    "session.what.setzero": "0x10 设零点",
    "session.setzero_sent": "[%s] ！！已发 0x10 设零点：整条臂会失能、把当前位姿写成零点 ！！",
    "session.stop_all_fail": "[%s] 退出前补发 0x14 失败：%s",
    "session.reopen_fail": "重开链路失败：%s",
    "session.reopened": "链路已重开：%s（两台的目标角会在下一帧反馈后重新对齐）",
    "session.fake_board": "假板子",
    "session.ctrl_tx_fail": "[%s] 控制帧发不出去：%s",
    "session.keepalive_tx_fail": "[%s] 保活帧发不出去：%s",
    "session.hs_fb_only":
        "已收到反馈帧（版本回复还没到 —— 板子被版本码锁死时就是一个字节都不回，"
        "本机版本码 0x%08X 可对照）",
    "session.hs_ok": "已握手：板子版本码 0x%08X 相符（%s）",
    "session.hs_bad":
        "版本码不符：板子 0x%08X / 本机 0x%08X —— 板子会静默丢弃一切，"
        "只能复位板子，或把两边版本串改回一致",
    "session.why.first_fb": "首帧反馈",
    "session.grip_uncal":
        "[%s] 夹爪没标定：面板上那一行显原始角，想要米就在终端跑 "
        "python3 -m sdk.calibrate_gripper --arm-id %d",
    "session.state_changed": "[%s] 状态 %s → %s",
    "session.why.wait_init": "重新回到待初始化",
    "session.why.reset_tail": "复位收尾",
    "session.reset_skipped":
        "[%s] 复位**疑似没执行**：%s 仍偏离复位位姿超过 ±%.0f°"
        "（规划池满时板上会静默跳过；或中途被打断）",
    "session.reset_done": "[%s] 复位完成，6 个关节已到复位位姿",
    "session.mode_changed": "[%s] 模式 %s → %s",
    "session.mode_tail_normal": "（一键动作收尾的正常返回，臂已松劲）",
    "session.why.enter_run": "进入 run",
    "session.ctrl_stopped": "（0x16 已自动停发）",
    "session.motor_lost": "[%s] 电机掉线: %s（掩码 0x%02X）",
    "session.motor_recovered": "[%s] 掉线已恢复（掩码 0x%02X）",
    "session.why.reseed": "补播种",
    "session.initing_note": "[%s] 初始化中：应在复位阶段，马上轮到夹爪找零",
    "session.pose_dev_item": "%s 偏 %.1f°",
    "session.pose_dev_sep": "，",
})

_EN.update({
    "session.tid.start": "0x12 start",
    "session.tid.gravity": "0x13 gravity comp",
    "session.tid.stop": "0x14 stop",
    "session.tid.damping": "0x15 damping",
    "session.arm_id.type": "--arm-id must be an integer (0..%d)",
    "session.arm_id.range": "--arm-id must be 0..%d (board ARM_NUM=%d)",
    "session.switch_arm":
        "panel now on %s (both arms keep being driven; the one you left keeps "
        "tracking its own target angles)",
    "session.no_feedback": "no feedback",
    "session.seeded": "[%s] target angles seeded from feedback (%s)",
    "session.no_feedback_yet":
        "no feedback frame yet, hold off -- without feedback the target cannot be "
        "aligned to the current pose",
    "session.grip_limit_m":
        "[%s] gripper asked for %.4f m, outside soft limits [%.4f, %.4f] m; "
        "stopped at %.4f m",
    "session.axis_limit":
        "[%s] %s asked for %.2f°, outside soft limits [%.2f, %.2f]; stopped at %.2f°",
    "session.step": "[%s] step = %s",
    "session.send_gated": "[%s] %s not sent: %s",
    "session.send_error": "[%s] %s failed: %s",
    "session.sent_mode": "[%s] sent %s (%s)",
    "session.init_no_fb":
        "[%s] no feedback frame yet, not sending 0x02 (the board's step is unknown)",
    "session.init_pose_bad":
        "[%s] pose is more than ±%.0f° away from the reset pose (%s) -- move it "
        "back to %s under disable/damping first; pressing i again forces 0x02 "
        "through (the board uses the same test, so a bad pose is dropped silently "
        "either way)",
    "session.init_force":
        "[%s] (forcing 0x02: the pose pre-check is skipped, every other gate still "
        "applies)",
    "session.what.init": "0x02 init",
    "session.init_sent":
        "[%s] sent 0x02 init: the board runs \"one-key reset -> one-key gripper "
        "homing\", about 7 s; keep the keepalive going",
    "session.what.reset": "0x18 reset",
    "session.reset_sent":
        "[%s] sent 0x18 reset: about 2 s back to %s, then it stops in damping and "
        "**the 0x16 stream stops by itself** (press s to resume position control)",
    "session.what.setzero": "0x10 set zero",
    "session.setzero_sent":
        "[%s] !! sent 0x10 set zero: the whole arm disables and the current pose "
        "becomes the zero point !!",
    "session.stop_all_fail": "[%s] sending 0x14 on exit failed: %s",
    "session.reopen_fail": "reopening the link failed: %s",
    "session.reopened":
        "link reopened: %s (both arms re-seed their targets on the next feedback frame)",
    "session.fake_board": "fake board",
    "session.ctrl_tx_fail": "[%s] control frame not sent: %s",
    "session.keepalive_tx_fail": "[%s] keepalive frame not sent: %s",
    "session.hs_fb_only":
        "feedback frame received (version reply not in yet -- with a mismatched "
        "version code the board answers nothing at all; local version code 0x%08X "
        "to compare)",
    "session.hs_ok": "handshake done: board version code 0x%08X matches (%s)",
    "session.hs_bad":
        "version code mismatch: board 0x%08X / local 0x%08X -- the board silently "
        "drops everything; reset the board, or make the version strings match again",
    "session.why.first_fb": "first feedback frame",
    "session.grip_uncal":
        "[%s] gripper not calibrated: that panel row shows the raw angle; for "
        "metres run python3 -m sdk.calibrate_gripper --arm-id %d in a terminal",
    "session.state_changed": "[%s] state %s -> %s",
    "session.why.wait_init": "back to WAIT_INIT",
    "session.why.reset_tail": "reset finished",
    "session.reset_skipped":
        "[%s] reset **probably did not run**: %s still more than ±%.0f° from the "
        "reset pose (the board skips silently when the temp plan pool is full, or "
        "it got interrupted)",
    "session.reset_done": "[%s] reset done, all 6 joints are at the reset pose",
    "session.mode_changed": "[%s] mode %s -> %s",
    "session.mode_tail_normal":
        "(normal return from a one-key action, the arm has gone slack)",
    "session.why.enter_run": "entering run",
    "session.ctrl_stopped": "(0x16 stopped automatically)",
    "session.motor_lost": "[%s] motor(s) lost: %s (mask 0x%02X)",
    "session.motor_recovered": "[%s] motors back online (mask 0x%02X)",
    "session.why.reseed": "late seed",
    "session.initing_note":
        "[%s] initializing: reset phase expected, gripper homing comes next",
    "session.pose_dev_item": "%s off by %.1f°",
    "session.pose_dev_sep": ", ",
})

# ---- sdk/tui/app.py：面板 / 帮助页 / 对话框 ----
_ZH.update({
    # 帮助页（每行一条，空行是 ""，不进表）
    "app.help.select": "1-6 / 0 / Tab      选中 j1..j6 / 夹爪 / 下一个",
    "app.help.jog": "← ↓ / → ↑ / + -    选中轴 ∓ 一个步长（只改目标角，实际下发按 --ctrl-hz）",
    "app.help.step":
        "[ / ]              步长档（rad）0.005 0.01 0.02 0.05 0.1 0.2，默认 0.02；"
        "夹爪标定过时那一行改用米档 0.0005 0.001 0.002 0.005 0.01 0.02",
    "app.help.arm":
        "a                  切「当前臂」（两台都在被同时驱动，切走的那台继续跑它自己的目标角）",
    "app.help.align":
        "c                  目标角对齐当前反馈（重播种；板子切档/掉线后建议先按一下）",
    "app.help.modes":
        "s / d / v / x      0x12 进 run / 0x15 阻尼 / 0x13 重力补偿 / 0x14 停止(松劲)",
    "app.help.init":
        "i                  0x02 初始化（板子跑一键复位 → 一键夹爪找零，约 7 s；"
        "待初始化态、或正常运行态的 stop/damping 档发）",
    "app.help.reset":
        "o                  0x18 一键复位（只收臂回复位位姿 0/0.5/0.5/0/0/0 度，约 2 s；"
        "正常运行态发）",
    "app.help.setzero": "z                  0x10 设零点 —— 要逐字敲 SETZERO 再敲 CONFIRM 才发！",
    "app.help.hold": "Space              暂停/继续发 0x16（冻结目标，臂保持不动，不是停机）",
    "app.help.misc":
        "r / e / h / q      重开串口 / 清事件 / 本帮助 / 退出（任一台正发 0x16 时会二次确认）",
    "app.help.si1":
        "**面板上的单位是 SI**：关节 rad / rad·s⁻¹、夹爪 m（要标定，见 sdk/README「夹爪标定」）；",
    "app.help.si2": "夹爪没标定时那一行显示的是**电机原始角**（rad），软限位 ∓142°。",
    "app.help.si3":
        "事件日志里角度一律写 deg（和固件对得上），和面板上差一个 57.3 —— 不是哪个错了。",
    "app.help.gate1": "状态从反馈帧派生，不自己造：0x16 只在「正常运行 + run + 没暂停」时才发；",
    "app.help.gate2": "其它时候每 150 ms 补一帧 0x01 保活（板子 500 ms 无有效帧就进 comm_lost）。",
    "app.help.gate3":
        "被板子丢弃的 0x16 也算保活（板子的 s_last_rx_ms 在分派之前刷新），所以放心发。",
    "app.help.two1":
        "**两台臂各自跑各的状态机**：i/o/s/d/v/x 只作用于「当前臂」（底下那行 > 标着的那台）。",
    "app.help.two2": "一条链路上开两个进程会互相抢字节，所以一次只开这一个。",
    "app.help.exit": "退出不发 0x14：目标冻结、臂保持，约 500 ms 后板子自己因无帧进 damping。",
    # 面板
    "app.log_head": "# sdk.tui 启动 %s port=%s fake=%s 当前臂=%d\n",
    "app.log_warn": "警告：日志文件打不开(%s)，只在屏幕上显示",
    "app.too_small": "窗口太小（%dx%d），至少要 12 行 60 列",
    "app.title": "机械臂控制台  [当前] %s  %s  %s",
    "app.fake_tag": "（--fake 假板子）",
    "app.link_line": "链路 %s",
    "app.link_down": "**断开**",
    "app.stats": "  收%u(0x80 %u, %.0f Hz)  发%u  丢字节%u  CRC错%u",
    "app.streaming": "在发",
    "app.idle": "停发",
    "app.lost_tag": "  **掉线**",
    "app.arms_line": "两台：%s",
    "app.no_fb": "还没收到任何反馈帧……",
    "app.link_dead": "串口断了。检查接线后按 r 重开。",
    "app.hs_timeout": "握手超时（%ds 没收到一个字节）—— 按顺序查：",
    "app.hs1": "1) 接线方向：适配器 TX → 板子 RX、适配器 RX → 板子 TX、GND 共地"
               "（板子跑 UART 那条时是 RS485 半双工）；",
    "app.hs2": "2) 波特率 921600；3) 板子上跑的是不是这份固件（比对版本码 0x%08X / 版本串 %s）；",
    "app.hs3": "4) 版本码不符时板子**静默不回包**，只能复位板子重来；",
    "app.hs4": "5) 板子的 ROBOT_LINK 是编译期二选一（UART/USB），和插的那个口子要对得上；",
    "app.hs5": "6) 这两条查完还没字节，看下面事件里「版本码不符」那一条。",
    "app.lost_prefix": "  掉线: ",
    "app.uninit_prefix": "  未初始化: ",
    "app.grip_limit_note": "（夹爪软限位要一次成功设零才生效）",
    "app.ctrl_prefix": "控制帧 %s",
    "app.hold_tag": "（已暂停 Space）",
    "app.ctrl_meta": "  %.0f Hz  步长 %s  选中 %s",
    "app.initing": "初始化中 %.1f s（复位 → 夹爪找零）  保活中",
    "app.resetting": "复位中 %.1f s（目标 %s rad）  保活中",
    "app.table_head": " 轴    目标角        反馈角        偏差      软限位(rad)      温度/单位",
    "app.grip_uncal_hint":
        "夹爪未标定：那一行是**电机原始角**(rad)，软限位 ∓%.3f rad。"
        "想要米就跑 python3 -m sdk.calibrate_gripper --arm-id %d",
    "app.setzero_ack": "上次设零 0x11: %s  耗时 %d ms  结果 %s",
    "app.setzero_unconfirmed":
        "  未确认(1)=达妙本来就在读数 0 附近，无法证明写进去了 —— 不是故障",
    "app.hotkeys":
        "a 切臂  1-6/0/Tab 选轴  ←↓→↑/+- 步进  [/] 步长  c 对齐  s run  "
        "d 阻尼  x 停  v 重力  i 初始化  o 复位  z 设零  Space 暂停  "
        "h 帮助  q 退出",
    # 按键 / 对话框
    "app.events_cleared": "事件已清",
    "app.quit_confirm": "有臂正发/曾发控制帧，确定退出？(y/n) ",
    "app.no_fb_align": "还没反馈帧，没法对齐",
    "app.why_manual": "手动 c",
    "app.hold_toggle": "0x16 流已%s（目标冻结，臂保持；真松劲请按 x）",
    "app.hold_paused": "暂停",
    "app.hold_resumed": "继续",
    "app.tid.start": "0x12 启动",
    "app.tid.damping": "0x15 阻尼",
    "app.tid.gravity": "0x13 重力补偿",
    "app.tid.stop": "0x14 停止",
    "app.reset_pose_warn":
        "姿态离复位位姿超过 ±%.4f rad（%s）—— 复位期间是 run 档、臂会**使劲收拢**，行程里别站人",
    "app.pose_dev_sep": "，",
    "app.reset_confirm": "确定运行一键复位？(y/n) ",
    "app.reset_cancelled": "一键复位已取消",
    "app.setzero_no_fb": "还没反馈帧，不发 0x10",
    "app.setzero_state": "0x10 只能在「待初始化」发（现在 %s）",
    "app.setzero_warn1": "0x10 会把**当前位姿**写成零点，并覆盖电机里的原零点！",
    "app.setzero_warn2": "当前姿态(rad)：%s（复位位姿 %s）",
    "app.setzero_step1": "第1步/2：把机械臂摆到基准位姿后逐字输入 SETZERO >",
    "app.setzero_cancel1": "设零已取消（第 1 步没通过）",
    "app.setzero_step2": "第2步/2：再逐字输入 CONFIRM 真正发送 0x10 >",
    "app.setzero_cancel2": "设零已取消（第 2 步没通过）",
})

_EN.update({
    # help page
    "app.help.select": "1-6 / 0 / Tab      select j1..j6 / gripper / next",
    "app.help.jog":
        "← ↓ / → ↑ / + -    nudge the selected axis ∓ one step "
        "(target only; sending is paced by --ctrl-hz)",
    "app.help.step":
        "[ / ]              step size (rad) 0.005 0.01 0.02 0.05 0.1 0.2, default "
        "0.02; a calibrated gripper row uses the metre steps "
        "0.0005 0.001 0.002 0.005 0.01 0.02",
    "app.help.arm":
        "a                  switch which arm the panel watches (both are driven; "
        "the one you leave keeps tracking its own target angles)",
    "app.help.align":
        "c                  align target angles to current feedback (re-seed; press "
        "it after a mode change or a lost motor)",
    "app.help.modes":
        "s / d / v / x      0x12 run / 0x15 damping / 0x13 gravity comp / "
        "0x14 stop (go slack)",
    "app.help.init":
        "i                  0x02 init (the board runs one-key reset -> one-key "
        "gripper homing, about 7 s; allowed in WAIT_INIT, or in NORMAL at "
        "stop/damping)",
    "app.help.reset":
        "o                  0x18 one-key reset (folds back to 0/0.5/0.5/0/0/0 deg, "
        "about 2 s; NORMAL only)",
    "app.help.setzero":
        "z                  0x10 set zero -- you must type SETZERO then CONFIRM "
        "before anything is sent!",
    "app.help.hold":
        "Space              pause/resume the 0x16 stream (freezes the target; the "
        "arm holds, this is not a stop)",
    "app.help.misc":
        "r / e / h / q      reopen port / clear events / this help / quit "
        "(double-confirms while any arm streams 0x16)",
    "app.help.si1":
        "**Panel units are SI**: joints rad / rad·s⁻¹, gripper m (needs "
        "calibration, see sdk/README);",
    "app.help.si2":
        "an uncalibrated gripper row shows the **raw motor angle** (rad), soft "
        "limits ∓142°.",
    "app.help.si3":
        "event-log angles are always deg (they line up with the firmware), 57.3 "
        "away from the panel -- neither one is wrong.",
    "app.help.gate1":
        "State is derived from feedback frames, never invented: 0x16 goes out only "
        "in NORMAL + run + not held;",
    "app.help.gate2":
        "otherwise a 0x01 keepalive goes out every 150 ms (500 ms without a valid "
        "frame drops the board to comm_lost).",
    "app.help.gate3":
        "a 0x16 the board drops still counts as keepalive (its s_last_rx_ms is "
        "refreshed before dispatch), so sending is always safe.",
    "app.help.two1":
        "**Each arm runs its own state machine**: i/o/s/d/v/x only affect the "
        "current arm (the one marked > on the line below).",
    "app.help.two2":
        "two processes on one link steal bytes from each other, so run only this one.",
    "app.help.exit":
        "quitting sends no 0x14: the target freezes, the arm holds, and about "
        "500 ms later the board drops to damping on its own.",
    # panel
    "app.log_head": "# sdk.tui started %s port=%s fake=%s arm=%d\n",
    "app.log_warn": "warning: cannot open the log file (%s), showing on screen only",
    "app.too_small": "window too small (%dx%d), need at least 12 rows x 60 cols",
    "app.title": "Arm Console  [current] %s  %s  %s",
    "app.fake_tag": "(--fake fake board)",
    "app.link_line": "Link %s",
    "app.link_down": "**DOWN**",
    "app.stats": "  rx%u(0x80 %u, %.0f Hz)  tx%u  drop bytes%u  crc err%u",
    "app.streaming": "streaming",
    "app.idle": "idle",
    "app.lost_tag": "  **LOST**",
    "app.arms_line": "Arms: %s",
    "app.no_fb": "no feedback frame received yet...",
    "app.link_dead": "serial link is down. check the wiring, then press r to reopen.",
    "app.hs_timeout": "handshake timeout (%ds without a single byte) -- check in order:",
    "app.hs1": "1) wiring: adapter TX -> board RX, adapter RX -> board TX, common GND "
               "(RS485 half-duplex when the board runs the UART link);",
    "app.hs2": "2) baud 921600; 3) is the board running this firmware (compare version "
               "code 0x%08X / version string %s);",
    "app.hs3": "4) on a version-code mismatch the board **answers nothing at all** -- "
               "reset the board and start over;",
    "app.hs4": "5) the board's ROBOT_LINK is a compile-time either/or (UART/USB) and "
               "must match the port you plugged in;",
    "app.hs5": "6) still no bytes after all that: look for the \"version code mismatch\" "
               "event below.",
    "app.lost_prefix": "  lost: ",
    "app.uninit_prefix": "  not initialized: ",
    "app.grip_limit_note":
        "(gripper soft limits only take effect after one successful set-zero)",
    "app.ctrl_prefix": "control stream %s",
    "app.hold_tag": "(held with Space)",
    "app.ctrl_meta": "  %.0f Hz  step %s  selected %s",
    "app.initing": "initializing %.1f s (reset -> gripper homing)  keepalive on",
    "app.resetting": "resetting %.1f s (target %s rad)  keepalive on",
    "app.table_head": " axis target      feedback      delta          "
                      "soft limits(rad)   temp/unit",
    "app.grip_uncal_hint":
        "gripper not calibrated: that row is the **raw motor angle** (rad), soft "
        "limits ∓%.3f rad. For metres run python3 -m sdk.calibrate_gripper --arm-id %d",
    "app.setzero_ack": "last set-zero 0x11: %s  took %d ms  result %s",
    "app.setzero_unconfirmed":
        "  UNCONFIRMED(1) = the DM motor was already reading ~0, so the write "
        "cannot be proven -- not a fault",
    "app.hotkeys":
        "a switch arm  1-6/0/Tab pick axis  ←↓→↑/+- step  [/] step size  c align  "
        "s run  d damping  x stop  v gravity  i init  o reset  z set-zero  "
        "Space hold  h help  q quit",
    # keys / dialogs
    "app.events_cleared": "events cleared",
    "app.quit_confirm":
        "an arm is streaming (or has streamed) control frames -- quit anyway? (y/n) ",
    "app.no_fb_align": "no feedback frame yet, cannot align",
    "app.why_manual": "manual c",
    "app.hold_toggle":
        "0x16 stream %s (target frozen, arm holds; for real slack press x)",
    "app.hold_paused": "paused",
    "app.hold_resumed": "resumed",
    "app.tid.start": "0x12 start",
    "app.tid.damping": "0x15 damping",
    "app.tid.gravity": "0x13 gravity comp",
    "app.tid.stop": "0x14 stop",
    "app.reset_pose_warn":
        "pose is more than ±%.4f rad from the reset pose (%s) -- during a reset the "
        "arm is in run and **folds in hard**; keep clear of its travel",
    "app.pose_dev_sep": ", ",
    "app.reset_confirm": "run the one-key reset? (y/n) ",
    "app.reset_cancelled": "one-key reset cancelled",
    "app.setzero_no_fb": "no feedback frame yet, not sending 0x10",
    "app.setzero_state": "0x10 can only be sent in WAIT_INIT (now %s)",
    "app.setzero_warn1":
        "0x10 writes the **current pose** as the zero point and overwrites the "
        "motor's original zero!",
    "app.setzero_warn2": "current pose (rad): %s (reset pose %s)",
    "app.setzero_step1":
        "step 1/2: put the arm at the reference pose, then type SETZERO >",
    "app.setzero_cancel1": "set zero cancelled (step 1 not passed)",
    "app.setzero_step2": "step 2/2: type CONFIRM to actually send 0x10 >",
    "app.setzero_cancel2": "set zero cancelled (step 2 not passed)",
})


def message_table(lang=None):
    """The raw `<key> -> template` dict for one language (used by the table checker)."""
    return _ZH if (lang or _LANG) == "zh" else _EN


def tr(key, *args):
    """Look up `key` and `%`-format it with `args` (no args -> returned as is)."""
    s = _ZH.get(key) if _LANG == "zh" else _EN.get(key)
    if s is None:                       # missing translation: fall back, never raise
        s = _ZH.get(key)
        if s is None:
            return "[missing:%s]" % key
    return s % args if args else s


# ============================================================================
# Name tables
# ============================================================================
#
# English names are kept short on purpose: they go into the panel's two-arm overview line
# and into `robot_state %s(%d)   arm_mode %s`, which already run past 80 columns in Chinese.
# Axis names must fit the panel's `%-4s` column, so j1..j6 stay as-is and the gripper is
# "grip" (4 columns), not "gripper".

_MODE_EN = {
    board_defs.MODE_STOP: "stop(relax)",
    board_defs.MODE_RUN: "run(pos)",
    board_defs.MODE_DAMPING: "damping",
    board_defs.MODE_GRAVITY: "gravity(comp)",
}

_STATE_EN = {
    board_defs.STATE_WAIT_HANDSHAKE: "HANDSHAKE",
    board_defs.STATE_OFFLINE_WAIT: "OFFLINE_WAIT",
    board_defs.STATE_WAIT_INIT: "WAIT_INIT",
    board_defs.STATE_INITING: "INITIALIZING",
    board_defs.STATE_NORMAL: "NORMAL",
    board_defs.STATE_COMM_LOST: "COMM_LOST",
    board_defs.STATE_CALIBRATING: "ZERO_CAL",
    board_defs.STATE_RESETTING: "RESETTING",
}

_ZERO_STATE_EN = {
    0: "IDLE", 1: "DISABLING", 2: "WAIT_DISABLE_ACK", 3: "SENDING", 4: "WAIT_RESULT",
    5: "DONE", 6: "TIMEOUT",
}

_ZERO_RES_EN = {
    0: "NOT_REACHED", 1: "UNCONFIRMED", 2: "OK", 3: "FAILED", 4: "TIMEOUT", 5: "SKIPPED",
}

_AXIS_EN = board_defs.JOINT_NAMES + ("grip",)


def state_name(v):
    """`Robot_state` -> readable name. Chinese delegates to `board_defs` (one source)."""
    if _LANG == DEFAULT_LANG:
        return board_defs.state_name(v)
    return _STATE_EN.get(v, "UNKNOWN(%d)" % v)


def mode_name(v):
    if _LANG == DEFAULT_LANG:
        return board_defs.mode_name(v)
    return _MODE_EN.get(v, "UNKNOWN(%d)" % v)


def zero_state_name(v):
    """`Arm_zero_state`. The fallback is the raw value itself, same as `board_defs`."""
    if _LANG == DEFAULT_LANG:
        return board_defs.ZERO_STATE_NAME.get(v, v)
    return _ZERO_STATE_EN.get(v, v)


def zero_res_name(v, default="?"):
    if _LANG == DEFAULT_LANG:
        return board_defs.ZERO_RES_NAME.get(v, default)
    return _ZERO_RES_EN.get(v, default)


def axis_name(i):
    if _LANG == DEFAULT_LANG:
        return board_defs.AXIS_NAMES[i]
    return _AXIS_EN[i]


def arm_label(arm_id):
    if _LANG == DEFAULT_LANG:
        return board_defs.arm_label(arm_id)
    tag = "AB"[arm_id] if 0 <= arm_id < board_defs.ARM_NUM else "?"
    return "Arm%s(arm_id=%d, CAN%d)" % (tag, arm_id, arm_id + 1)


def expand_mask(mask):
    """`motor_lost_mask` / `motor_inited_mask` -> space-separated names."""
    if _LANG == DEFAULT_LANG:
        return board_defs.expand_mask(mask)
    if not mask:
        return "none"
    return " ".join(_AXIS_EN[i] for i in range(board_defs.ARM_MOTOR_NUM) if mask >> i & 1)


# ============================================================================
# Table checker (development only -- not one of the self-test's counted items)
# ============================================================================

# `%`-conversion specifier, minus `%%`. Deliberately permissive: a wrong regex here would
# report a false "all good", so the real safety net is the frozen-Chinese comparison plus a
# human read of the English table, not this.
_SPEC = re.compile(r"%(?:\([^)]*\))?[-+ #0]*[0-9]*(?:\.[0-9]+)?[diouxXeEfFgGcrsa%]")

_FORBIDDEN = (("　", "〿"),      # CJK punctuation
              ("一", "鿿"),      # CJK ideographs
              ("＀", "￯"))      # fullwidth forms


def _bad_chars(s):
    return sorted({c for c in s
                   if any(lo <= c <= hi for lo, hi in _FORBIDDEN)})


def _specs(s):
    """(conversion chars, leftover `%`) of one template, `%%` excluded."""
    chars = [m.group(0)[-1] for m in _SPEC.finditer(s) if m.group(0) != "%%"]
    rest = _SPEC.sub("", s)
    return chars, rest.count("%")


def check_tables():
    """Return a list of problems (empty == fine). Called by the console self-test."""
    bad = []
    for key in sorted(set(_ZH) | set(_EN)):
        zh, en = _ZH.get(key), _EN.get(key)
        if zh is None:
            bad.append("%s: 只有英文，缺中文" % key)
            continue
        if en is None:
            bad.append("%s: 只有中文，缺英文" % key)
            continue
        for tag, s in (("zh", zh), ("en", en)):
            if not isinstance(s, str):
                bad.append("%s[%s]: 不是 str（表里只许放 str）" % (key, tag))
        if not isinstance(zh, str) or not isinstance(en, str):
            continue
        zc, zl = _specs(zh)
        ec, el = _specs(en)
        if sorted(zc) != sorted(ec):
            bad.append("%s: %% 转换符不一致 zh=%r en=%r" % (key, "".join(zc), "".join(ec)))
        if zl:
            bad.append("%s[zh]: 有认不出的 %%（要写 %%%%）" % key)
        if el:
            bad.append("%s[en]: 有认不出的 %%（要写 %%%%）" % key)
        chars = _bad_chars(en)
        if chars:
            bad.append("%s[en]: 混进中文/全角 %r" % (key, "".join(chars)))

    for what, zh_map, en_map in (("STATE_NAME", board_defs.STATE_NAME, _STATE_EN),
                                 ("MODE_NAME", board_defs.MODE_NAME, _MODE_EN),
                                 ("ZERO_STATE_NAME", board_defs.ZERO_STATE_NAME, _ZERO_STATE_EN),
                                 ("ZERO_RES_NAME", board_defs.ZERO_RES_NAME, _ZERO_RES_EN)):
        if set(zh_map) != set(en_map):
            bad.append("%s: 键集和 board_defs 不一致 %r"
                       % (what, sorted(set(zh_map) ^ set(en_map))))
        for k, v in en_map.items():
            chars = _bad_chars(v)
            if chars:
                bad.append("%s[%r][en]: 混进中文/全角 %r" % (what, k, "".join(chars)))
    if len(_AXIS_EN) != len(board_defs.AXIS_NAMES):
        bad.append("AXIS 名表个数和 board_defs 不一致")
    for v in _AXIS_EN:
        chars = _bad_chars(v)
        if chars:
            bad.append("AXIS 名表[en]: 混进中文/全角 %r" % "".join(chars))
    return bad
