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

# ---- sdk/tui/__main__.py：命令行 ----
_ZH.update({
    "main.cli_description": "机械臂上位机控制台（curses 全屏面板）。UART / USB 两条链路、两台臂都能跑。",
    "main.epilog": "按 h 看按键表；--selftest / --fake 都不需要硬件。",
    "main.help.lang": "界面语言（默认 zh 中文；--lang en 换成英文面板和英文报错）",
    "main.help.port": "串口设备（默认自动挑：优先板子 USB 0483:5740，退回 1a86 适配器）",
    "main.help.baud": "波特率（默认 921600）",
    "main.help.arm_id": "启动时「当前臂」是谁：0=CAN1/臂A，1=CAN2/臂B（默认 %d）。"
                        "**两台都在被同时驱动**，面板里按 a 随时切",
    "main.help.ctrl_hz": "0x16 下发频率（默认 100，每台）",
    "main.help.fb_hz": "面板刷新率（默认 5，按键响应不受影响）",
    "main.help.step": "初始步长 rad（%s；默认 %g）",
    "main.help.handshake": "多久没收到字节就提示握手超时",
    "main.help.init_tol": "姿态复核容差 rad（默认 %.4f = 板上的 %.0f°）。只用于 0x18 前的弹确认和"
                          "复位收尾的显示判据；0x02 的闸门在 sdk 里，恒为板上的 %.0f°",
    "main.help.fake": "不接硬件，用内置假板子（sdk/sim.py）",
    "main.help.fake_lost": "假板子每隔一会给「当前臂」制造这个掉线掩码（如 0x41）",
    "main.help.selftest": "离线自测，不接硬件不起 curses",
    "main.help.log": "把事件追加写到文件（用 .log 名字，*.json 被 gitignore）",
    "main.help.stop_on_exit": "退出前给**两台**补发 0x14（默认不发：只停 0x16，约 500 ms 后板子自己进 damping）",
    "main.bad_step": "--step 只能是 %s 之一（单位是 rad）",
    "main.no_device": "链路设备 %s 不存在。\n  %s",
    "main.no_links": "    （一个都没有：两条链路都没插）",
    "main.no_pyserial": "缺 pyserial：pip3 install -r sdk/requirements.txt",
    "main.open_failed": "打开 %s 失败：%s",
    "main.permission_hint": "（多半是权限：把用户加进 dialout 组，或 sudo 跑）",
    "main.link_info": "链路 %s",
    "main.fake_link": "假板子（--fake，不接硬件）",
    "main.two_arms": "两台臂都在被驱动：%s、%s（一条链路、每台一个 Arm）；面板上按 a 切「当前臂」，"
                     "现在看的是 %s",
    "main.usb_note": "板子 USB CDC：波特率 %d 在这条上无意义（CDC 是包传输）",
    "main.unknown_kind": "这个口子既不是板子 USB(0483:5740)也不是 1a86 适配器，"
                         "确认板子的 ROBOT_LINK 和它是对得上的",
    "main.stopped": "已给 %s 补发 0x14 停止。",
    "main.stop_sep": "、",
    "main.exit_note": "已停发两台的 0x16（目标冻结、臂保持）。约 500 ms 后板子会因 500 ms 无"
                      "有效帧进 comm_lost → damping；要立刻松劲按 x，或用 --stop-on-exit。",
})

# ---- sdk/tui/selftest.py：自测条目名 / 小节名 / 汇总 ----
_ZH.update({
    "selftest.hdr_sdk": "1) sdk 离线自测（协议 / 闸门 / 假板子）",
    "selftest.hdr_tui": "2) 控制台后端自测（sdk.tui.session，用假板子跑）",
    "selftest.sec_units": "单位与软限位（rad）",
    "selftest.j1_hi": "j1 上限 = 90°",
    "selftest.j1_lo": "j1 下限 = -115°",
    "selftest.grip_limit": "夹爪软限位 = ∓142°（= ∓284/2）",
    "selftest.clamp_hi": "夹到上限 90°",
    "selftest.clamp_lo": "夹到下限 -115°",
    "selftest.clamp_in": "界内不夹、也不报夹过（就是拍板不要那个数）",
    "selftest.sec_step": "步长档与 --arm-id",
    "selftest.step_rad": "rad 档是 0.005/0.01/0.02/0.05/0.1/0.2",
    "selftest.step_default": "默认 rad 档在档位里（≈ 旧版的 1.0°）",
    "selftest.step_m": "米档递增且默认档在档位里",
    "selftest.arm_id": "parse_arm_id 收 0/1/0x1/空格，拒 2/-1/x/空/1.0",
    "selftest.sec_two": "两台臂（一条链路、两个 Arm、状态机各跑各的）",
    "selftest.pass": "自测全部通过（sdk + 控制台后端）。",
    "selftest.two_fb": "两台都收到反馈帧（0x80 按 arm_id 分流）",
    "selftest.two_target": "两台的目标角是各自独立的列表",
    "selftest.two_seeded": "两台各自被播种（都对到自己的反馈上）",
    "selftest.snap_keys": "snapshot 的键齐（面板的契约）",
    "selftest.fb_keys": "fb 的键齐",
    "selftest.fb_rad": "joint 是 6 个 rad（不是 deg）",
    "selftest.grip_cal": "夹爪：标定标志与米数一致（没标定时 m 为 None，原始角一直在）",
    "selftest.n_rx_fb": "n_rx_fb 是本臂口径（= 当前臂的 n_feedback）",
    "selftest.arms_sorted": "arms 概览两台都在、按 id 排序",
    "selftest.sw_to_b": "a 键切到臂B（当前臂 = 1）",
    "selftest.jog_cur": "点动只动当前臂（臂A 的目标角一动没动）",
    "selftest.target_cur": "sess.target 读的是当前臂那份",
    "selftest.sw_back": "再切回臂A",
    "selftest.hold_cur": "hold 也按当前臂走（只挂臂A）",
    "selftest.sec_cmd": "命令与闸门（假板子的闸门是真的）",
    "selftest.a_wait_init": "臂A 走到「待初始化」",
    "selftest.init_sent": "待初始化态按 i → 真的发出 0x02（盖当前臂的 arm_id）",
    "selftest.init_done": "初始化跑完回到「正常运行」（板子 ~7 s）",
    "selftest.tid_start": "0x12 启动",
    "selftest.start_ok": "正常运行态按 s → 发出 0x12（进 run）",
    "selftest.run_ok": "进 run 了（0x16 才有可能被受理）",
    "selftest.i_no_frame": "run 档下按 i → **一帧没发**（不谎报「已发」）",
    "selftest.i_one_warn": "run 档下按 i 给出恰好一条 warn（SDK 说清缺哪条闸门）",
    "selftest.i_again": "run 档下再按一次 i → 依然一帧没发（强发通道只给姿态闸门）",
    "selftest.z_no_frame": "正常运行态按 z → 一帧没发（0x10 只认待初始化）",
    "selftest.b_still": "臂B 还停在「待初始化」（两台状态机各跑各的）",
    "selftest.a_stream": "臂A 在发 0x16、臂B 没在发（控制流按台独立）",
    "selftest.streaming_any": "streaming_any() 看的是两台（退出确认用）",
    "selftest.b_init": "臂B 按 i → 0x02 盖的是 arm_id=1（不是当前面板看的那台那么简单）",
    "selftest.b_done": "臂B 初始化完到「正常运行」",
    "selftest.b_run": "臂B 进 run",
    "selftest.both_stream": "两台都在发 0x16，各盖各的 arm_id",
    "selftest.sec_reopen": "reopen（r 键 / 拔线重开）",
    "selftest.reopen_ok": "reopen 成功",
    "selftest.reopen_cleared": "reopen 后两台的播种都清了（等下一帧反馈）",
    "selftest.reopen_seeded": "reopen 后两台都重新握手+播种（假板子重上电 → 0→1→2）",
    "selftest.stop_two": "退出补 0x14：两台都发、都带各自的 arm_id",
    "selftest.fail": "自测**没过**：见上面 FAIL。",
    "selftest.fb_counts": "0x80 帧数 臂A=%s 臂B=%s",
    "selftest.sent": "发出的：%r",
})

# ---- sdk/arm.py：闸门拒绝 / send_* 报错 / 握手诊断（会直接印进面板事件区）----
#
# 注意：`ArmSnapshot.robot_state_name` / `mode_name` / `lost_names` / `inited_names`
# 这几个**公开字段**仍旧走 board_defs、仍旧是中文（见类注释），所以 arm.py 的
# `Arm.*_name` 属性也不动；只有**消息里**的渲染改走 i18n，从数值码现算。
_ZH.update({
    "arm.snapshot_line": "臂%s state=%s mode=%s 关节(rad) %s 夹爪 %s 温度 %s 丢 %s",
    "arm.grip_uncal": "未标定(raw %.2f°)",
    "arm.grip_inited_hint": "  （夹爪 motor_inited = 1，零点已经建立了，可以标）",
    "arm.grip_not_homed":
        "  ⚠ 反馈里夹爪的 motor_inited 位是 0 —— **夹爪还没找过零**，它的原始角零点"
        "现在没有含义。先做一次初始化（0x02，里面带夹爪找零）再标。",
    "arm.no_feedback_yet": "  （现在一帧反馈都没收到，先 connect()）",
    "arm.arm_id_range": "arm_id 只能是 0..%d（板上 ARM_NUM=%d），收到 %r",
    "arm.no_version": "(没收到 —— 板子连版本包都没回)",
    "arm.ver_ok": "0x%08X（和 SDK 一致）",
    "arm.ver_mismatch": "0x%08X  ← **和 SDK 的 0x%08X 不一致，板子会锁死不再回任何帧，"
                        "要复位板子**",
    "arm.connect_timeout": "%.1fs 内没等到 %s 的反馈帧。",
    "arm.connect_port": "  口子：%s",
    "arm.no_port": "(没挑到口子)",
    "arm.connect_link": "  链路：open=%s up=%s",
    "arm.link_error": "  错误：%s",
    "arm.connect_rx": "  收到 %d 帧（CRC8 错 %d、CRC16 错 %d、丢弃字节 %d）",
    "arm.connect_board_ver": "  板子回的版本码：%s",
    "arm.connect_zero_bytes": "  一个字节都没收到：口子挑错 / 线没插 / 板子跑的是另一条链路。",
    "arm.bytes_no_version": "  收到了字节但没有版本包：版本码很可能对不上（板子锁死），"
                            "或者收到的是噪声。",
    "arm.joint_angle": "关节角(rad)",
    "arm.joint_vel": "关节角速度(rad/s)",
    "arm.gate_init_state": "总模式不对：现在 %s / %s。要能在「%s」，或「%s」+ mode 是 "
                           "stop/damping（run/gravity 下位置环在跑，板子故意不接）。",
    "arm.gate_setzero_state": "总模式不对：现在 %s，设零只能在「%s」发起。",
    "arm.gate_mode_state": "总模式不对：现在 %s，切 Arm_mode 只能在「%s」发。",
    "arm.gate_ctrl_state": "总模式不对：现在 %s，控制帧只能在「%s」发。",
    "arm.gate_not_run": "Arm_mode 不是 run：现在 %s。先 start()。",
    "arm.gate_reset_state": "总模式不对：现在 %s，一键复位只能在「%s」发。",
    "arm.payload_type": "payload 要给 bytes，收到 %r",
    "arm.setzero_timeout": "%.1fs 内没等到设零回复(0x11)。现在 state=%s。\n"
                           "  0x11 是**跑完一轮才发**的，几秒到十几秒都正常；再等就加大 "
                           "timeout。\n"
                           "  一直不来要查：0x10 有没有真的发出去（闸门？遥控器在线？）、"
                           "板子是不是还在校准零点(6)。",
    "arm.stale_no_fb": "%s 要看板子的状态，但 %s 一帧反馈都没收到。"
                       "先 connect()（它等第一帧反馈）。",
    "arm.stale_old": "%s 要看板子的状态，但手上这帧反馈已经 %.0f ms 旧了（阈值 %.0f ms）"
                     "——不拿旧状态猜。\n"
                     "  反馈断了要查：板子在不在跑、链路 link_up=%s、口子对不对。\n"
                     "  要跳过这一层用 force=True（但闸门判错的责任就归你了）。",
    "arm.gate_rejected": "%s 被客户端预检挡下（**这一帧没发出去**）：\n  - %s\n"
                         "  现在：state=%s(robot_state=%s) mode=%s(%s) 掉线=%s\n"
                         "  （板子对所有命令都是静默丢弃、没有 NACK，所以这里先替你算一遍。\n"
                         "   预检**查不到「遥控器在线」**这一条 —— 那是板子才知道的。\n"
                         "   确认过闸门真的会过，用 force=True 跳过预检。）",
    "arm.motors_lost": "有电机掉线：%s。板子会拒（掉线会强制 arm_stop，动作根本跑不动）。",
    "arm.pose_dev": "姿态偏差 %.2f° > %.0f°（关节 %s 差得最多：现在 %.2f°，要 %.2f°）。"
                    "先把臂摆回复位位姿 —— 注意它是按**反馈角**比的，不是你要它去的地方。",
    "arm.grip_two_targets": "gripper_m 和 gripper_raw_deg 只能给一个"
                            "（两个都给就不知道听谁的）",
    "arm.grip_no_target": "没给夹爪目标、又没收到过反馈帧，不知道该把夹爪放哪儿。\n"
                          "  给一个 gripper_m= 或 gripper_raw_deg=；或者先 connect() "
                          "让我拿当前角当目标（保持不动）。",
    "arm.desc_line": "%s：%s",
    "arm.describe_no_fb": "%s：还没收到反馈（link_up=%s）",
    "arm.no_feedback": "无反馈",
    "arm.no_port_open": "(未开口子)",
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

# ---- sdk/tui/__main__.py ----
_EN.update({
    "main.cli_description":
        "Arm console for the board (curses full-screen panel). "
        "Both links (UART / USB) and both arms.",
    "main.epilog": "press h for the key list; --selftest / --fake need no hardware.",
    "main.help.lang": "UI language (default zh; --lang en switches the panel and the errors)",
    "main.help.port": "serial device (default: auto-pick -- board USB 0483:5740 first, "
                      "then a 1a86 adapter)",
    "main.help.baud": "baud rate (default 921600)",
    "main.help.arm_id": "which arm starts as the current one: 0=CAN1/ArmA, 1=CAN2/ArmB "
                        "(default %d). **both are driven at once**; press a to switch",
    "main.help.ctrl_hz": "0x16 send rate (default 100, per arm)",
    "main.help.fb_hz": "panel refresh rate (default 5; key handling is unaffected)",
    "main.help.step": "initial step in rad (%s; default %g)",
    "main.help.handshake": "how long without a byte before reporting a handshake timeout",
    "main.help.init_tol": "pose-check tolerance in rad (default %.4f = %.0f deg on the board). "
                          "Only used for the confirm prompt before 0x18 and the reset-end "
                          "display check; the 0x02 gate lives in sdk and is always %.0f deg",
    "main.help.fake": "no hardware: use the built-in fake board (sdk/sim.py)",
    "main.help.fake_lost": "the fake board periodically reports this lost mask for the current "
                           "arm (e.g. 0x41)",
    "main.help.selftest": "offline self-test; no hardware, no curses",
    "main.help.log": "append events to a file (use a .log name; *.json is gitignored)",
    "main.help.stop_on_exit": "send 0x14 to **both** arms on exit (default: no -- just stop "
                              "0x16 and the board drops to damping about 500 ms later)",
    "main.bad_step": "--step must be one of %s (the unit is rad)",
    "main.no_device": "link device %s does not exist.\n  %s",
    "main.no_links": "    (none at all: neither link is plugged in)",
    "main.no_pyserial": "pyserial is missing: pip3 install -r sdk/requirements.txt",
    "main.open_failed": "opening %s failed: %s",
    "main.permission_hint": "(probably permissions: add your user to the dialout group, "
                            "or run with sudo)",
    "main.link_info": "Link %s",
    "main.fake_link": "fake board (--fake, no hardware)",
    "main.two_arms": "both arms are driven: %s, %s (one link, one Arm each); press a to "
                     "switch the current arm, now watching %s",
    "main.usb_note": "board USB CDC: baud %d is meaningless here (CDC is packet transport)",
    "main.unknown_kind": "this port is neither board USB (0483:5740) nor a 1a86 adapter -- "
                         "make sure the board's ROBOT_LINK matches it",
    "main.stopped": "sent 0x14 stop to %s.",
    "main.stop_sep": ", ",
    "main.exit_note": "stopped 0x16 for both arms (target frozen, arms hold). About 500 ms "
                      "later the board hits comm_lost -> damping on its own; press x for "
                      "slack right away, or use --stop-on-exit.",
})

# ---- sdk/tui/selftest.py ----
_EN.update({
    "selftest.hdr_sdk": "1) sdk offline self-test (protocol / gates / fake board)",
    "selftest.hdr_tui": "2) console backend self-test (sdk.tui.session, on the fake board)",
    "selftest.sec_units": "units and soft limits (rad)",
    "selftest.j1_hi": "j1 upper = 90 deg",
    "selftest.j1_lo": "j1 lower = -115 deg",
    "selftest.grip_limit": "gripper soft limit = -/+142 deg (= -/+284/2)",
    "selftest.clamp_hi": "clamped at the 90 deg upper limit",
    "selftest.clamp_lo": "clamped at the -115 deg lower limit",
    "selftest.clamp_in": "inside the range: not clamped, not reported as clamped (that is "
                         "what the caller asked for)",
    "selftest.sec_step": "step choices and --arm-id",
    "selftest.step_rad": "rad choices are 0.005/0.01/0.02/0.05/0.1/0.2",
    "selftest.step_default": "the default rad step is one of the choices (about the old 1.0 deg)",
    "selftest.step_m": "metre choices increase and the default is one of them",
    "selftest.arm_id": "parse_arm_id takes 0/1/0x1/space, rejects 2/-1/x/empty/1.0",
    "selftest.sec_two": "two arms (one link, two Arms, independent state machines)",
    "selftest.pass": "self-test passed (sdk + console backend).",
    "selftest.two_fb": "both arms get feedback frames (0x80 split by arm_id)",
    "selftest.two_target": "the two arms keep separate target lists",
    "selftest.two_seeded": "both arms seeded (each aligned to its own feedback)",
    "selftest.snap_keys": "snapshot keys are complete (the panel's contract)",
    "selftest.fb_keys": "fb keys are complete",
    "selftest.fb_rad": "joint is 6 values in rad (not deg)",
    "selftest.grip_cal": "gripper: the calibrated flag matches the metre value (uncalibrated "
                         "-> m is None, the raw angle is always there)",
    "selftest.n_rx_fb": "n_rx_fb is this arm's own count (= the current arm's n_feedback)",
    "selftest.arms_sorted": "the arms overview has both, sorted by id",
    "selftest.sw_to_b": "press a switches to arm B (current arm = 1)",
    "selftest.jog_cur": "jog moves the current arm only (arm A's target did not move)",
    "selftest.target_cur": "sess.target reads the current arm's list",
    "selftest.sw_back": "switching back to arm A",
    "selftest.hold_cur": "hold follows the current arm too (arm A only)",
    "selftest.sec_cmd": "commands and gates (the fake board's gates are real)",
    "selftest.a_wait_init": "arm A reaches WAIT_INIT",
    "selftest.init_sent": "WAIT_INIT + i -> 0x02 really goes out (stamped with the current "
                          "arm's arm_id)",
    "selftest.init_done": "init finishes back in NORMAL (about 7 s on the board)",
    "selftest.tid_start": "0x12 start",
    "selftest.start_ok": "NORMAL + s -> 0x12 goes out (enters run)",
    "selftest.run_ok": "in run now (only then can 0x16 be accepted)",
    "selftest.i_no_frame": "i in run -> **not a single frame** (no bogus \"sent\")",
    "selftest.i_one_warn": "i in run gives exactly one warn (the SDK says which gate is missing)",
    "selftest.i_again": "i again in run -> still not a single frame (the force path is only "
                        "for the pose gate)",
    "selftest.z_no_frame": "z in NORMAL -> not a single frame (0x10 only exists in WAIT_INIT)",
    "selftest.b_still": "arm B is still in WAIT_INIT (the two state machines are independent)",
    "selftest.a_stream": "arm A streams 0x16, arm B does not (the control stream is per arm)",
    "selftest.streaming_any": "streaming_any() looks at both arms (used by the quit prompt)",
    "selftest.b_init": "i for arm B -> 0x02 is stamped arm_id=1 (not just whichever arm "
                       "the panel happens to show)",
    "selftest.b_done": "arm B finishes init in NORMAL",
    "selftest.b_run": "arm B enters run",
    "selftest.both_stream": "both arms stream 0x16, each stamped with its own arm_id",
    "selftest.sec_reopen": "reopen (the r key / replugging)",
    "selftest.reopen_ok": "reopen succeeded",
    "selftest.reopen_cleared": "reopen clears both arms' seeds (waiting for the next frame)",
    "selftest.reopen_seeded": "after reopen both arms handshake and seed again (the fake "
                              "board reboots -> 0->1->2)",
    "selftest.stop_two": "exit sends 0x14 to both arms, each with its own arm_id",
    "selftest.fail": "self-test **FAILED**: see the FAIL lines above.",
    "selftest.fb_counts": "0x80 frames A=%s B=%s",
    "selftest.sent": "sent: %r",
})

# ---- sdk/arm.py ----
_EN.update({
    "arm.snapshot_line": "arm%s state=%s mode=%s joints(rad) %s gripper %s temp %s lost %s",
    "arm.grip_uncal": "uncalibrated (raw %.2f deg)",
    "arm.grip_inited_hint": "  (gripper motor_inited = 1, the zero point is established, "
                            "safe to calibrate)",
    "arm.grip_not_homed":
        "  ! the gripper's motor_inited bit in the feedback is 0 -- **the gripper has never "
        "been homed**, so its raw-angle zero means nothing yet. Run an init (0x02, which "
        "includes gripper homing) before calibrating.",
    "arm.no_feedback_yet": "  (no feedback frame at all yet -- call connect() first)",
    "arm.arm_id_range": "arm_id must be 0..%d (ARM_NUM=%d on the board), got %r",
    "arm.no_version": "(none -- the board did not even answer the version packet)",
    "arm.ver_ok": "0x%08X (matches the SDK)",
    "arm.ver_mismatch": "0x%08X  <- **does not match the SDK's 0x%08X: the board locks up "
                        "and answers nothing, reset the board**",
    "arm.connect_timeout": "%.1fs without a feedback frame from %s.",
    "arm.connect_port": "  port: %s",
    "arm.no_port": "(no port picked)",
    "arm.connect_link": "  link: open=%s up=%s",
    "arm.link_error": "  error: %s",
    "arm.connect_rx": "  %d frames received (CRC8 err %d, CRC16 err %d, dropped bytes %d)",
    "arm.connect_board_ver": "  version code from the board: %s",
    "arm.connect_zero_bytes": "  not a single byte: wrong port / cable not plugged in / "
                              "the board runs the other link.",
    "arm.bytes_no_version": "  bytes arrived but no version packet: the version code very "
                            "likely does not match (board locked up), or what came in is noise.",
    "arm.joint_angle": "joint angles (rad)",
    "arm.joint_vel": "joint velocities (rad/s)",
    "arm.gate_init_state": "wrong overall state: now %s / %s. Allowed in \"%s\", or in "
                           "\"%s\" with mode stop/damping (under run/gravity the position "
                           "loop is live and the board deliberately ignores it).",
    "arm.gate_setzero_state": "wrong overall state: now %s; set-zero can only start in \"%s\".",
    "arm.gate_mode_state": "wrong overall state: now %s; Arm_mode can only be changed in \"%s\".",
    "arm.gate_ctrl_state": "wrong overall state: now %s; control frames can only be sent "
                           "in \"%s\".",
    "arm.gate_not_run": "Arm_mode is not run: now %s. Call start() first.",
    "arm.gate_reset_state": "wrong overall state: now %s; the one-key reset can only be "
                            "sent in \"%s\".",
    "arm.payload_type": "payload must be bytes, got %r",
    "arm.setzero_timeout": "no set-zero reply (0x11) within %.1fs. state=%s now.\n"
                           "  0x11 is sent **only after a whole round finishes**; several "
                           "seconds to a dozen or so is normal, so raise the timeout if you "
                           "need to.\n"
                           "  If it never arrives, check whether 0x10 really went out "
                           "(gate? remote online?) and whether the board is still in zero "
                           "calibration (6).",
    "arm.stale_no_fb": "%s needs the board's state, but %s has not received a single "
                       "feedback frame. Call connect() first (it waits for the first frame).",
    "arm.stale_old": "%s needs the board's state, but the frame in hand is already %.0f ms "
                     "old (threshold %.0f ms) -- no guessing from a stale state.\n"
                     "  If feedback stopped, check: is the board running, link_up=%s, is the "
                     "port right.\n"
                     "  To skip this layer use force=True (and the blame for a wrong gate "
                     "call is yours).",
    "arm.gate_rejected": "%s was blocked by the client-side pre-check (**the frame was not "
                         "sent**):\n  - %s\n"
                         "  now: state=%s(robot_state=%s) mode=%s(%s) lost=%s\n"
                         "  (the board silently drops every command -- there is no NACK -- "
                         "so this is computed for you up front.\n"
                         "   The pre-check **cannot see \"remote online\"** -- only the "
                         "board knows that.\n"
                         "   If you are sure the gate really passes, use force=True to skip "
                         "the pre-check.)",
    "arm.motors_lost": "motors are lost: %s. The board will reject it (a lost motor forces "
                       "arm_stop, the motion cannot run at all).",
    "arm.pose_dev": "pose deviation %.2f deg > %.0f deg (joint %s is worst: now %.2f deg, "
                    "want %.2f deg). Move the arm back to the reset pose first -- note this "
                    "compares **feedback angles**, not where you are asking it to go.",
    "arm.grip_two_targets": "give either gripper_m or gripper_raw_deg, not both (two targets "
                            "and there is no telling which one wins)",
    "arm.grip_no_target": "no gripper target given and no feedback frame ever received, so "
                          "there is no telling where to put the gripper.\n"
                          "  Pass gripper_m= or gripper_raw_deg=; or connect() first so I "
                          "can hold the current angle as the target.",
    "arm.desc_line": "%s: %s",
    "arm.describe_no_fb": "%s: no feedback yet (link_up=%s)",
    "arm.no_feedback": "no feedback",
    "arm.no_port_open": "(port not open)",
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
