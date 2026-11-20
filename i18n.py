"""面板、日志和异常的中英文文案，默认中文。

_ZH 和 _EN 使用相同键及兼容的百分号占位符。
缺少翻译时回退到中文，键不存在时显示 [missing:...]；check_tables() 检查文案表。
公共状态名称仍由 board_defs 提供，消息内的名称按当前语言显示。
"""
import re

from .board_defs import common as board_defs

LANGS = ("zh", "en")
DEFAULT_LANG = "zh"

_LANG = DEFAULT_LANG


def set_lang(code):
    """切换为 zh 或 en；其他值回退到 zh。"""
    global _LANG
    _LANG = code if code in LANGS else DEFAULT_LANG
    return _LANG


def get_lang():
    return _LANG


# 中英文文案

_ZH = {}
_EN = {}

# TUI 会话事件与对话框
_ZH.update({
    "session.tid.start": "0x12 启动",
    "session.tid.gravity": "0x13 重力补偿",
    "session.tid.stop": "0x14 停止",
    "session.tid.damping": "0x15 阻尼",
    "session.arm_id.type": "--arm-id 必须为整数，范围 0..%d",
    "session.arm_id.range": "--arm-id 范围为 0..%d，臂数为 %d",
    "session.switch_arm": "当前臂：%s；双臂控制继续运行",
    "session.no_feedback": "无反馈",
    "session.seeded": "[%s] 目标已对齐反馈（%s）",
    "session.no_feedback_yet": "尚无反馈，无法对齐目标",
    "session.grip_limit_m": "[%s] 夹爪目标 %.4f m 超限 [%.4f, %.4f] m，已限幅至 %.4f m",
    "session.axis_limit": "[%s] %s 目标 %.2f° 超限 [%.2f, %.2f]°，已限幅至 %.2f°",
    "session.step": "[%s] 步长 = %s",
    "session.send_gated": "[%s] %s 未发送：%s",
    "session.send_error": "[%s] %s 发送失败：%s",
    "session.sent_mode": "[%s] 已发 %s（%s）",
    "session.init_no_fb": "[%s] 尚无反馈，初始化未发送",
    "session.init_pose_bad":
        "[%s] 偏离复位角超过 %.0f°（%s）。先在 STOP/阻尼下调整至 %s；再按 i 仅跳过姿态预检，固件仍会检查",
    "session.init_force": "[%s] 跳过姿态预检，其他条件仍检查",
    "session.what.init": "0x02 初始化",
    "session.init_sent":
        "[%s] 已发送 0x02 初始化；持续保活，等待反馈确认完成",
    "session.what.reset": "0x18 复位",
    "session.reset_sent":
        "[%s] 已发送 0x18：约 2 s 回复位 %s，完成后进入阻尼；按 s 恢复位控",
    "session.what.setzero": "0x10 设零点",
    "session.setzero_sent": "[%s] 已发送 0x10：六轴当前位姿将覆盖原零点",
    "session.stop_all_fail": "[%s] 退出 STOP 发送失败：%s",
    "session.reopen_fail": "重开链路失败：%s",
    "session.reopened": "链路已重开：%s；下一帧反馈后对齐双臂目标",
    "session.link_lost": "链路断开：%s；正在重连，达到上限后按 r 重试",
    "session.link_lost_manual": "链路断开：%s；检查接线后按 r 重开",
    "session.relinked": "链路已自动重连：%s；下一帧反馈后对齐双臂目标",
    "session.relink_giveup":
        "重连已停止：%d 次失败（%s）；检查接线后按 r 重试",
    "session.fake_board": "模拟器",
    "session.ctrl_tx_fail": "[%s] 控制帧发送失败：%s",
    "session.keepalive_tx_fail": "[%s] 保活发送失败：%s",
    "session.hs_fb_only":
        "已收到反馈，尚无版本帧；SDK 版本码 0x%08X",
    "session.hs_ok": "握手成功：固件版本 0x%08X 匹配（%s）",
    "session.hs_bad":
        "版本不匹配：固件 0x%08X，SDK 0x%08X；请使用匹配的型号、SDK 和固件",
    "session.why.first_fb": "首帧反馈",
    "session.grip_uncal":
        "[%s] 夹爪未标定，显示原始角；标定：python3 -m sdk.calibrate_gripper --arm-id %d",
    "session.state_changed": "[%s] 状态 %s → %s",
    "session.why.wait_init": "返回待初始化",
    "session.why.reset_tail": "复位结束",
    "session.reset_skipped":
        "[%s] 复位未到位：%s 偏差仍超过 %.0f°；动作可能未执行或被中断",
    "session.reset_done": "[%s] 六轴复位完成",
    "session.mode_changed": "[%s] 模式 %s → %s",
    "session.mode_tail_normal": "（动作结束，进入阻尼）",
    "session.why.enter_run": "进入 RUN",
    "session.ctrl_stopped": "（位置目标已停发）",
    "session.motor_lost": "[%s] 电机离线：%s，掩码 0x%02X",
    "session.motor_recovered": "[%s] 电机恢复在线，掩码 0x%02X",
    "session.why.reseed": "重新对齐目标",
    "session.initing_note": "[%s] 初始化中，等待完成反馈",
    "session.pose_dev_item": "%s 偏 %.1f°",
    "session.pose_dev_sep": "，",
})

_EN.update({
    "session.tid.start": "0x12 start",
    "session.tid.gravity": "0x13 gravity comp",
    "session.tid.stop": "0x14 stop",
    "session.tid.damping": "0x15 damping",
    "session.arm_id.type": "--arm-id must be an integer in 0..%d",
    "session.arm_id.range": "--arm-id must be 0..%d; arm count: %d",
    "session.switch_arm":
        "Current arm: %s; both arms remain active",
    "session.no_feedback": "no feedback",
    "session.seeded": "[%s] Target aligned to feedback (%s)",
    "session.no_feedback_yet":
        "No feedback; cannot align target",
    "session.grip_limit_m":
        "[%s] Gripper target %.4f m outside [%.4f, %.4f] m; clamped to %.4f m",
    "session.axis_limit":
        "[%s] %s target %.2f° outside [%.2f, %.2f]°; clamped to %.2f°",
    "session.step": "[%s] step = %s",
    "session.send_gated": "[%s] %s not sent: %s",
    "session.send_error": "[%s] %s send failed: %s",
    "session.sent_mode": "[%s] sent %s (%s)",
    "session.init_no_fb":
        "[%s] No feedback; initialization not sent",
    "session.init_pose_bad":
        "[%s] Reset-pose error exceeds %.0f° (%s). Move to %s in STOP/damping; "
        "press i again to skip the host pose check. Firmware checks still apply",
    "session.init_force":
        "[%s] Skipping pose pre-check; other checks remain active",
    "session.what.init": "0x02 init",
    "session.init_sent":
        "[%s] Sent 0x02 initialization; keep alive and wait for completion feedback",
    "session.what.reset": "0x18 reset",
    "session.reset_sent":
        "[%s] Sent 0x18: reset to %s in about 2 s, then damping; press s to resume position control",
    "session.what.setzero": "0x10 set zero",
    "session.setzero_sent":
        "[%s] Sent 0x10: current joint pose will overwrite all six joint zero points",
    "session.stop_all_fail": "[%s] Exit STOP send failed: %s",
    "session.reopen_fail": "reopening the link failed: %s",
    "session.reopened":
        "Link reopened: %s; both targets align on the next feedback frame",
    "session.link_lost":
        "Link lost: %s; reconnecting, press r if retries are exhausted",
    "session.link_lost_manual":
        "Link lost: %s; check wiring, then press r to reopen",
    "session.relinked":
        "Link reconnected: %s; both targets align on the next feedback frame",
    "session.relink_giveup":
        "Reconnect stopped after %d failures (%s); check wiring, then press r",
    "session.fake_board": "simulator",
    "session.ctrl_tx_fail": "[%s] Control frame send failed: %s",
    "session.keepalive_tx_fail": "[%s] Keepalive send failed: %s",
    "session.hs_fb_only":
        "Feedback received, no version frame yet; SDK version 0x%08X",
    "session.hs_ok": "Handshake complete: firmware version 0x%08X matches (%s)",
    "session.hs_bad":
        "Version mismatch: firmware 0x%08X, SDK 0x%08X; check profile, SDK and firmware",
    "session.why.first_fb": "first feedback frame",
    "session.grip_uncal":
        "[%s] Gripper uncalibrated; showing raw angle. Run: python3 -m sdk.calibrate_gripper --arm-id %d",
    "session.state_changed": "[%s] state %s -> %s",
    "session.why.wait_init": "returned to WAIT_INIT",
    "session.why.reset_tail": "reset ended",
    "session.reset_skipped":
        "[%s] Reset target not reached: %s error exceeds %.0f°; motion may have been skipped or interrupted",
    "session.reset_done": "[%s] All six joints reached the reset pose",
    "session.mode_changed": "[%s] mode %s -> %s",
    "session.mode_tail_normal":
        "(motion ended; damping)",
    "session.why.enter_run": "entered RUN",
    "session.ctrl_stopped": "(position stream stopped)",
    "session.motor_lost": "[%s] Motors offline: %s, mask 0x%02X",
    "session.motor_recovered": "[%s] Motors online again, mask 0x%02X",
    "session.why.reseed": "realign target",
    "session.initing_note":
        "[%s] Initializing; waiting for completion feedback",
    "session.pose_dev_item": "%s off by %.1f°",
    "session.pose_dev_sep": ", ",
})

# TUI 面板、帮助与对话框
_ZH.update({
    # 每项对应一行帮助，空行不进入文案表。
    "app.help.select": "1-6 / 0 / Tab      选中 j1..j6 / 夹爪 / 下一个",
    "app.help.jog": "← ↓ / → ↑ / + -    调整选中轴目标，按 --ctrl-hz 周期发送",
    "app.help.step":
        "[ / ]              切换步长：关节 rad，已标定夹爪 m",
    "app.help.arm":
        "a                  切换当前臂，双臂控制继续运行",
    "app.help.align":
        "c                  目标对齐当前反馈",
    "app.help.modes":
        "s / d / v / x      RUN / 阻尼 / 重力补偿 / STOP 软停机",
    "app.help.init":
        "i                  初始化：待初始化，或正常运行的 STOP/阻尼模式",
    "app.help.reset":
        "o                  回复位：正常运行时执行，约 2 s",
    "app.help.setzero": "z                  覆盖六轴零点，须输入 SETZERO 和 CONFIRM",
    "app.help.hold": "Space              暂停/继续位置目标流；暂停不是停机",
    "app.help.misc":
        "r / e / h / q      重开链路 / 清事件 / 帮助 / 退出",
    "app.help.si1":
        "界面单位：关节 rad、rad/s，已标定夹爪 m。",
    "app.help.si2": "夹爪未标定时显示原始角 rad，限位取自型号配置。",
    "app.help.si3":
        "日志中的角度使用 deg。",
    "app.help.gate1": "位置目标仅在正常运行、RUN 且未暂停时发送。",
    "app.help.gate2": "其他时间每 150 ms 保活；3 s 无有效命令触发通信超时。",
    "app.help.gate3":
        "发送成功只代表提交，请通过反馈确认执行结果。",
    "app.help.two1":
        "i/o/s/d/v/x 仅作用于 > 标记的当前臂。",
    "app.help.two2": "同一串口只运行一个控制进程。",
    "app.help.exit": "默认退出仅关闭链路，固件约 3 s 后处理超时；--stop-on-exit 可发送 STOP。",
    # 面板
    "app.log_head": "# sdk.tui 启动 %s port=%s fake=%s 当前臂=%d\n",
    "app.log_warn": "日志文件打开失败（%s），仅显示在屏幕",
    "app.too_small": "窗口太小（%dx%d），至少要 12 行 60 列",
    "app.title": "机械臂控制台  [当前] %s  %s  %s",
    "app.fake_tag": "（模拟模式）",
    "app.link_line": "链路 %s",
    "app.link_down": "断开",
    "app.link_retry": "（自动重连中 %d/%d）",
    "app.link_retry_on": "（自动重连中）",
    "app.link_giveup": "（重连失败 %d 次；按 r 重试）",
    "app.link_manual": "（按 r 重开）",
    "app.stats": "  收%u(0x80 %u, %.0f Hz)  发%u  丢字节%u  CRC错%u",
    "app.streaming": "发送中",
    "app.idle": "停发",
    "app.lost_tag": "  离线",
    "app.arms_line": "两台：%s",
    "app.no_fb": "尚无反馈",
    "app.link_dead": "串口断开；检查接线和重连状态，可按 r 重开",
    "app.hs_timeout": "握手超时（%d s 无反馈），请检查：",
    "app.hs1": "1) 检查供电，以及 USB CDC 或 RS485 接线。",
    "app.hs2": "2) 核对串口波特率 921600，版本码 0x%08X，版本串 %s。",
    "app.hs3": "3) SDK 型号参数须与运行的固件匹配。",
    "app.hs4": "4) 固件 ROBOT_LINK（UART/USB）须与接线一致。",
    "app.hs5": "5) 查看事件区的链路或版本错误。",
    "app.lost_prefix": "  掉线: ",
    "app.uninit_prefix": "  未初始化: ",
    "app.grip_limit_note": "（夹爪软限位依赖初始化成功）",
    "app.ctrl_prefix": "控制帧 %s",
    "app.hold_tag": "（已暂停 Space）",
    "app.ctrl_meta": "  %.0f Hz  步长 %s  选中 %s",
    "app.initing": "初始化 %.1f s，保活中",
    "app.resetting": "复位中 %.1f s（目标 %s rad）  保活中",
    "app.table_head": " 轴    目标角        反馈角        偏差      软限位(rad)      温度/单位",
    "app.grip_uncal_hint":
        "夹爪未标定，显示原始角 rad，上限 %.3f rad；标定：python3 -m sdk.calibrate_gripper --arm-id %d",
    "app.setzero_ack": "上次设零 0x11: %s  耗时 %d ms  结果 %s",
    "app.setzero_unconfirmed":
        "  未确认(1)：无法确认零点写入，不等同于失败",
    "app.hotkeys":
        "a 切臂  1-6/0/Tab 选轴  ←↓→↑/+- 步进  [/] 步长  c 对齐  s run  "
        "d 阻尼  x 停  v 重力  i 初始化  o 复位  z 设零  Space 暂停  "
        "h 帮助  q 退出",
    # 按键与对话框
    "app.events_cleared": "事件已清",
    "app.quit_confirm": "有臂处于控制或暂停状态，确定退出？(y/n) ",
    "app.no_fb_align": "尚无反馈，无法对齐",
    "app.why_manual": "手动对齐",
    "app.hold_toggle": "位置目标流已%s；暂停不是停机，STOP 按 x",
    "app.hold_paused": "暂停",
    "app.hold_resumed": "继续",
    "app.tid.start": "0x12 启动",
    "app.tid.damping": "0x15 阻尼",
    "app.tid.gravity": "0x13 重力补偿",
    "app.tid.stop": "0x14 停止",
    "app.reset_pose_warn":
        "偏离复位角超过 %.4f rad（%s）；复位会驱动机械臂，请清空运动路径",
    "app.pose_dev_sep": "，",
    "app.reset_confirm": "确定回复位？(y/n) ",
    "app.reset_cancelled": "复位已取消",
    "app.setzero_no_fb": "尚无反馈，设零未发送",
    "app.setzero_state": "设零要求待初始化，当前为 %s",
    "app.setzero_warn1": "设零会以六轴当前位姿覆盖原零点。",
    "app.setzero_warn2": "当前姿态(rad)：%s（复位位姿 %s）",
    "app.setzero_step1": "1/2：摆到基准位姿后输入 SETZERO >",
    "app.setzero_cancel1": "设零已取消：第 1 步未确认",
    "app.setzero_step2": "2/2：输入 CONFIRM 发送设零命令 >",
    "app.setzero_cancel2": "设零已取消：第 2 步未确认",
})

# 命令行提示
_ZH.update({
    "main.cli_description": "H7 双臂控制台，支持 USB CDC 和 RS485。",
    "main.epilog": "按 h 看按键表；--selftest / --fake 都不需要硬件。",
    "main.help.lang": "界面与消息语言：zh 中文、en 英文（默认 zh）",
    "main.help.port": "串口路径；默认优先板载 USB，其次 1a86 适配器",
    "main.help.baud": "波特率（默认 921600）",
    "main.help.robot_type": "H7 链路使用的上位机机械臂参数（默认 arx_x5）",
    "main.help.arm_id": "当前臂：0=CAN1/A，1=CAN2/B（默认 %d）；按 a 切换",
    "main.help.ctrl_hz": "0x16 下发频率（默认 100，每台）",
    "main.help.fb_hz": "面板刷新率（默认 5，按键响应不受影响）",
    "main.help.step": "初始步长 rad（%s；默认 %g）",
    "main.help.handshake": "无反馈时的握手超时提示阈值，单位 s",
    "main.help.init_tol": "复位确认与到位判断容差，单位 rad；默认取型号参数，不改变初始化预检",
    "main.help.fake": "使用 H7 协议模拟器，不连接硬件",
    "main.help.fake_lost": "周期模拟所选臂的电机掉线掩码，如 0x41",
    "main.help.selftest": "离线自测，不接硬件不起 curses",
    "main.help.log": "追加事件日志到指定文件",
    "main.help.stop_on_exit": "退出前向双臂发送 STOP；默认仅关闭链路",
    "main.bad_step": "--step 必须取自 %s，单位 rad",
    "main.no_device": "链路设备 %s 不存在。\n  %s",
    "main.no_links": "    未找到候选串口",
    "main.no_pyserial": "缺少 pyserial；安装：python3 -m pip install pyserial",
    "main.open_failed": "打开 %s 失败：%s",
    "main.permission_hint": "串口权限不足；请检查设备权限或 dialout 组",
    "main.link_info": "链路 %s",
    "main.robot_type": "上位机机械臂参数：%s",
    "main.fake_link": "H7 协议模拟器，无硬件连接",
    "main.two_arms": "双臂：%s、%s；当前 %s，按 a 切换",
    "main.usb_note": "USB CDC 不使用波特率设置 %d 控制传输速度",
    "main.unknown_kind": "未识别链路类型，请核对端口与固件 ROBOT_LINK 配置",
    "main.stopped": "已向 %s 发送 STOP。",
    "main.stop_sep": "、",
    "main.exit_note": "已停止双臂目标流并关闭链路；固件约 3 s 后处理通信超时。",
})

# 自测项目与结果
_ZH.update({
    "selftest.hdr_sdk": "1) SDK 自测：协议、预检与模拟器",
    "selftest.hdr_tui": "2) TUI 会话自测",
    "selftest.sec_units": "单位与软限位（rad）",
    "selftest.j1_hi": "j1 上限 = 90°",
    "selftest.j1_lo": "j1 下限 = -115°",
    "selftest.grip_limit": "ARX-X5 夹爪软限位 = -157°～142°",
    "selftest.clamp_hi": "夹到上限 90°",
    "selftest.clamp_lo": "夹到下限 -115°",
    "selftest.clamp_in": "限位内目标保持不变",
    "selftest.sec_step": "步长档与 --arm-id",
    "selftest.step_rad": "rad 档是 0.005/0.01/0.02/0.05/0.1/0.2",
    "selftest.step_default": "默认关节步长属于预设列表",
    "selftest.step_m": "米制步长递增，默认值有效",
    "selftest.arm_id": "parse_arm_id 收 0/1/0x1/空格，拒 2/-1/x/空/1.0",
    "selftest.sec_two": "共享链路与双臂独立状态",
    "selftest.pass": "SDK 和 TUI 自测全部通过。",
    "selftest.two_fb": "反馈按 arm_id 分流到双臂",
    "selftest.two_target": "双臂目标独立",
    "selftest.two_seeded": "双臂目标分别对齐反馈",
    "selftest.snap_keys": "快照字段完整",
    "selftest.fb_keys": "反馈字段完整",
    "selftest.fb_rad": "关节反馈为六项 rad",
    "selftest.grip_cal": "夹爪标定标志与开口值一致，原始角可读",
    "selftest.n_rx_fb": "当前臂反馈计数等于 n_feedback",
    "selftest.arms_sorted": "双臂概览按编号排序",
    "selftest.sw_to_b": "切换到臂 B",
    "selftest.jog_cur": "点动仅修改当前臂目标",
    "selftest.target_cur": "sess.target 指向当前臂",
    "selftest.sw_back": "切回臂 A",
    "selftest.hold_cur": "暂停仅作用于当前臂",
    "selftest.sec_cmd": "命令与预检",
    "selftest.a_wait_init": "臂 A 进入待初始化",
    "selftest.init_sent": "待初始化时发送本臂 0x02",
    "selftest.init_done": "初始化完成后进入正常运行",
    "selftest.tid_start": "0x12 启动",
    "selftest.start_ok": "正常运行时发送 0x12",
    "selftest.run_ok": "收到 RUN 模式反馈",
    "selftest.i_no_frame": "RUN 时拒绝初始化，不发送命令",
    "selftest.i_one_warn": "初始化被拒绝时报告一次警告",
    "selftest.i_again": "重复初始化仍须通过状态预检",
    "selftest.z_no_frame": "正常运行时拒绝设零",
    "selftest.b_still": "臂 B 保持待初始化",
    "selftest.a_stream": "仅臂 A 发送位置目标",
    "selftest.streaming_any": "控制状态检查覆盖双臂",
    "selftest.b_init": "臂 B 初始化使用 arm_id=1",
    "selftest.b_done": "臂 B 初始化完成",
    "selftest.b_run": "臂 B 进入 RUN",
    "selftest.both_stream": "双臂按各自编号发送位置目标",
    "selftest.sec_reopen": "重开链路",
    "selftest.reopen_ok": "reopen 成功",
    "selftest.reopen_cleared": "重开后清除双臂目标对齐标志",
    "selftest.reopen_seeded": "重开后双臂重新握手并对齐目标",
    "selftest.stop_two": "退出向双臂分别发送 STOP",
    "selftest.sec_relink": "自动重连",
    "selftest.relink_snap": "快照包含断开、重试次数和停止状态",
    "selftest.relink_seeded": "重连后双臂重新对齐目标",
    "selftest.relink_msg": "记录自动重连成功",
    "selftest.relink_giveup": "记录重连停止及原因",
    "selftest.fail": "自测失败，详见 FAIL 项。",
    "selftest.table_bad": "文案表错误：%d 项",
    "selftest.fb_counts": "0x80 帧数 臂A=%s 臂B=%s",
    "selftest.sent": "发出的：%r",
})

# Arm 预检和连接诊断。
# 公共状态名称保留中文；消息中的名称按当前语言显示。
_ZH.update({
    "arm.snapshot_line": "臂%s 状态=%s 模式=%s 关节(rad)=%s 夹爪=%s 温度(℃)=%s 离线=%s",
    "arm.grip_uncal": "未标定(raw %.2f°)",
    "arm.grip_inited_hint": "  夹爪已初始化，可进行标定",
    "arm.grip_not_homed":
        "  夹爪未初始化；请完成初始化后再标定",
    "arm.no_feedback_yet": "  尚无反馈，请先 connect()",
    "arm.arm_id_range": "arm_id 应为 0..%d（臂数 %d），收到 %r",
    "arm.no_version": "未收到版本帧",
    "arm.ver_ok": "0x%08X（和 SDK 一致）",
    "arm.ver_mismatch": "固件 0x%08X 与 SDK 0x%08X 不匹配，无法握手",
    "arm.connect_timeout": "连接超时：%.1f s 内未收到 %s 的反馈",
    "arm.connect_port": "  端口：%s",
    "arm.no_port": "未选择端口",
    "arm.connect_link": "  链路：open=%s up=%s",
    "arm.link_error": "  错误：%s",
    "arm.connect_rx": "  收到 %d 帧（CRC8 错 %d、CRC16 错 %d、丢弃字节 %d）",
    "arm.connect_board_ver": "  固件版本：%s",
    "arm.connect_zero_bytes": "  未收到有效帧；请检查接线、端口及固件 ROBOT_LINK",
    "arm.bytes_no_version": "  已收到帧但无版本帧；请检查协议版本和链路",
    "arm.joint_angle": "关节角(rad)",
    "arm.gate_init_state": "当前 %s / %s；初始化要求 %s，或 %s 且为 STOP/阻尼",
    "arm.gate_setzero_state": "当前 %s；设零要求 %s",
    "arm.gate_mode_state": "当前 %s；切换模式要求 %s",
    "arm.gate_ctrl_state": "当前 %s；发送目标要求 %s",
    "arm.gate_not_run": "当前模式 %s；请先 start() 并等待 RUN 反馈",
    "arm.gate_reset_state": "当前 %s；复位要求 %s",
    "arm.payload_type": "payload 必须为 bytes，收到 %r",
    "arm.setzero_timeout": "设零超时：%.1f s 内未收到 0x11，当前状态 %s。请检查命令是否受理及校准进度",
    "arm.stale_no_fb": "%s 未发送：%s 尚无反馈，请先 connect()",
    "arm.stale_old": "%s 未发送：反馈年龄 %.0f ms，超过 %.0f ms；link_up=%s，请检查链路",
    "arm.gate_rejected": "%s 未发送，预检未通过：\n  - %s\n  状态=%s(%s)，模式=%s(%s)，离线=%s",
    "arm.motors_lost": "电机离线：%s",
    "arm.pose_dev": "位姿偏差 %.2f° 超过 %.0f°；%s 当前 %.2f°，复位角 %.2f°",
    "arm.grip_two_targets": "gripper_m 与 gripper_raw_deg 不能同时指定",
    "arm.grip_no_target": "缺少夹爪目标和反馈；请指定目标或先 connect()",
    "arm.desc_line": "%s：%s",
    "arm.describe_no_fb": "%s：尚无反馈，link_up=%s",
    "arm.no_feedback": "无反馈",
    "arm.no_port_open": "串口未打开",
})

# 串口选择与连接错误
_ZH.update({
    "link.kind_usb": "H7 USB CDC",
    "link.kind_rs485": "USB 转 RS485 适配器",
    "link.kind_fake": "H7 协议模拟器",
    "link.kind_unknown": "未知链路",
    "link.not_link_dev": "，非控制链路",
    "link.desc_port": "%s（USB %s:%s %s %s%s）",
    "link.desc_no_ident": "%s（查不到 USB 身份）",
    "link.desc_kind_tag": "，%s",
    "link.hint_uart": "ROBOT_LINK_UART：连接 USB 转 RS485 适配器；",
    "link.hint_usb": "ROBOT_LINK_USB：连接板载 USB CDC（%s:%s）。",
    "link.hint_cands": "候选端口：",
    "link.hint_no_tty": "未发现 ttyUSB/ttyACM 设备，请检查接线",
    "link.hint_port_arg": "可用 port= 指定端口。",
    "link.reject_probe_ident": "拒绝打开 %s：J-Link 调试端口，请改用控制链路。\n  %s",
    "link.reject_probe_name": "拒绝打开 %s：无法确认 USB 身份，路径疑似调试端口。\n  %s",
    "link.not_open": "链路未打开；请先 link.open() 或 arm.connect()",
    "link.down_reconnecting": "链路断开，重连 %d/%d；本帧未发送",
    "link.open_failed": "打不开 %s：%s\n%s",
    "link.write_link_err": "写串口失败: %s",
    "link.write_failed": "写 %s 失败：%s",
    "link.read_link_err": "读串口失败: %s",
    "link.parse_link_err": "解析失败: %s",
    "link.arm_id_range": "arm_id 应为 0..%d（臂数 %d），收到 %r",
    "link.describe_line": "%s，%s",
    "link.auto_port": "(自动)",
})

_EN.update({
    # 帮助页
    "app.help.select": "1-6 / 0 / Tab      select j1..j6 / gripper / next",
    "app.help.jog":
        "← ↓ / → ↑ / + -    adjust selected target; send at --ctrl-hz",
    "app.help.step":
        "[ / ]              change step: joints in rad, calibrated gripper in m",
    "app.help.arm":
        "a                  switch current arm; both arms remain active",
    "app.help.align":
        "c                  align target to current feedback",
    "app.help.modes":
        "s / d / v / x      RUN / damping / gravity compensation / STOP",
    "app.help.init":
        "i                  initialize in WAIT_INIT, or NORMAL with STOP/damping",
    "app.help.reset":
        "o                  reset to profile reference in NORMAL, about 2 s",
    "app.help.setzero":
        "z                  overwrite joint zeros; type SETZERO then CONFIRM",
    "app.help.hold":
        "Space              pause/resume position stream; pause is not STOP",
    "app.help.misc":
        "r / e / h / q      reopen link / clear events / help / quit",
    "app.help.si1":
        "Panel units: joints rad and rad/s; calibrated gripper m.",
    "app.help.si2":
        "Uncalibrated grippers show raw rad; limits follow the selected profile.",
    "app.help.si3":
        "Log angles use deg.",
    "app.help.gate1":
        "Position targets require NORMAL, RUN and an unpaused stream.",
    "app.help.gate2":
        "Otherwise keep alive every 150 ms; no valid command for 3 s triggers timeout.",
    "app.help.gate3":
        "A successful send confirms submission; check feedback for execution.",
    "app.help.two1":
        "i/o/s/d/v/x affect only the current arm marked >.",
    "app.help.two2":
        "Use one control process per serial port.",
    "app.help.exit":
        "Exit closes the link; firmware handles timeout after about 3 s. --stop-on-exit sends STOP.",
    # 面板
    "app.log_head": "# sdk.tui started %s port=%s fake=%s arm=%d\n",
    "app.log_warn": "Cannot open log file (%s); using screen output only",
    "app.too_small": "window too small (%dx%d), need at least 12 rows x 60 cols",
    "app.title": "Arm Console  [current] %s  %s  %s",
    "app.fake_tag": "(simulation)",
    "app.link_line": "Link %s",
    "app.link_down": "DOWN",
    "app.link_retry": " (auto-reconnecting %d/%d)",
    "app.link_retry_on": " (auto-reconnecting)",
    "app.link_giveup": " (reconnect failed %d times; press r to retry)",
    "app.link_manual": " (press r to reopen)",
    "app.stats": "  rx%u(0x80 %u, %.0f Hz)  tx%u  drop bytes%u  crc err%u",
    "app.streaming": "streaming",
    "app.idle": "idle",
    "app.lost_tag": "  OFFLINE",
    "app.arms_line": "Arms: %s",
    "app.no_fb": "No feedback yet",
    "app.link_dead": "Serial link lost; check wiring and reconnect status, or press r",
    "app.hs_timeout": "Handshake timeout (%d s without feedback); check:",
    "app.hs1": "1) Check power and USB CDC or RS485 wiring.",
    "app.hs2": "2) Check baud 921600, version code 0x%08X and version string %s.",
    "app.hs3": "3) Match the SDK robot profile to the running firmware.",
    "app.hs4": "4) Match firmware ROBOT_LINK (UART/USB) to the connection.",
    "app.hs5": "5) Check link and version errors in the event log.",
    "app.lost_prefix": "  lost: ",
    "app.uninit_prefix": "  not initialized: ",
    "app.grip_limit_note":
        "(gripper soft limits require successful initialization)",
    "app.ctrl_prefix": "control stream %s",
    "app.hold_tag": "(held with Space)",
    "app.ctrl_meta": "  %.0f Hz  step %s  selected %s",
    "app.initing": "Initializing %.1f s; keepalive active",
    "app.resetting": "resetting %.1f s (target %s rad)  keepalive on",
    "app.table_head": " axis target      feedback      delta          "
                      "soft limits(rad)   temp/unit",
    "app.grip_uncal_hint":
        "Gripper uncalibrated: raw rad, upper limit %.3f rad. Run: python3 -m sdk.calibrate_gripper --arm-id %d",
    "app.setzero_ack": "last set-zero 0x11: %s  took %d ms  result %s",
    "app.setzero_unconfirmed":
        "  UNCONFIRMED(1): zero write not verified; not a confirmed failure",
    "app.hotkeys":
        "a switch arm  1-6/0/Tab pick axis  ←↓→↑/+- step  [/] step size  c align  "
        "s run  d damping  x stop  v gravity  i init  o reset  z set-zero  "
        "Space hold  h help  q quit",
    # 按键与对话框
    "app.events_cleared": "events cleared",
    "app.quit_confirm":
        "An arm is controlling or paused. Quit? (y/n) ",
    "app.no_fb_align": "No feedback; cannot align",
    "app.why_manual": "manual alignment",
    "app.hold_toggle":
        "Position stream %s; pause is not STOP, press x for STOP",
    "app.hold_paused": "paused",
    "app.hold_resumed": "resumed",
    "app.tid.start": "0x12 start",
    "app.tid.damping": "0x15 damping",
    "app.tid.gravity": "0x13 gravity comp",
    "app.tid.stop": "0x14 stop",
    "app.reset_pose_warn":
        "Reset-pose error exceeds %.4f rad (%s); reset moves the arm, keep its path clear",
    "app.pose_dev_sep": ", ",
    "app.reset_confirm": "Move to the reset pose? (y/n) ",
    "app.reset_cancelled": "Reset cancelled",
    "app.setzero_no_fb": "No feedback; set-zero not sent",
    "app.setzero_state": "Set-zero requires WAIT_INIT; current state: %s",
    "app.setzero_warn1":
        "Set-zero overwrites all six joint zero points with the current pose.",
    "app.setzero_warn2": "current pose (rad): %s (reset pose %s)",
    "app.setzero_step1":
        "1/2: move to the reference pose, then type SETZERO >",
    "app.setzero_cancel1": "Set-zero cancelled: step 1 not confirmed",
    "app.setzero_step2": "2/2: type CONFIRM to send set-zero >",
    "app.setzero_cancel2": "Set-zero cancelled: step 2 not confirmed",
})

# 命令行提示
_EN.update({
    "main.cli_description":
        "H7 dual-arm console over USB CDC or RS485.",
    "main.epilog": "press h for the key list; --selftest / --fake need no hardware.",
    "main.help.lang": "UI and message language: zh or en (default zh)",
    "main.help.port": "Serial path; prefer board USB, then a 1a86 adapter",
    "main.help.baud": "baud rate (default 921600)",
    "main.help.robot_type": "host robot parameters for the H7 link (default arx_x5)",
    "main.help.arm_id": "Current arm: 0=CAN1/A, 1=CAN2/B (default %d); press a to switch",
    "main.help.ctrl_hz": "0x16 send rate (default 100, per arm)",
    "main.help.fb_hz": "panel refresh rate (default 5; key handling is unaffected)",
    "main.help.step": "initial step in rad (%s; default %g)",
    "main.help.handshake": "Handshake warning timeout without feedback, in s",
    "main.help.init_tol": "Reset confirmation and completion tolerance in rad; "
                          "defaults to profile, does not change initialization checks",
    "main.help.fake": "Use the H7 protocol simulator without hardware",
    "main.help.fake_lost": "Periodically simulate this lost-motor mask on the selected arm, e.g. 0x41",
    "main.help.selftest": "offline self-test; no hardware, no curses",
    "main.help.log": "Append event logs to this file",
    "main.help.stop_on_exit": "Send STOP to both arms on exit; default only closes the link",
    "main.bad_step": "--step must be one of %s, in rad",
    "main.no_device": "link device %s does not exist.\n  %s",
    "main.no_links": "    No candidate serial ports found",
    "main.no_pyserial": "pyserial missing; run: python3 -m pip install pyserial",
    "main.open_failed": "opening %s failed: %s",
    "main.permission_hint": "Serial permission denied; check device permissions or dialout membership",
    "main.link_info": "Link %s",
    "main.robot_type": "Host robot parameters: %s",
    "main.fake_link": "H7 protocol simulator, no hardware connection",
    "main.two_arms": "Arms: %s, %s; current: %s, press a to switch",
    "main.usb_note": "USB CDC does not use baud setting %d to set transfer speed",
    "main.unknown_kind": "Unknown link type; check port and firmware ROBOT_LINK setting",
    "main.stopped": "Sent STOP to %s.",
    "main.stop_sep": ", ",
    "main.exit_note": "Both position streams stopped and link closed; firmware handles timeout after about 3 s.",
})

# 自测提示
_EN.update({
    "selftest.hdr_sdk": "1) SDK tests: protocol, pre-checks and simulator",
    "selftest.hdr_tui": "2) TUI session tests",
    "selftest.sec_units": "units and soft limits (rad)",
    "selftest.j1_hi": "j1 upper = 90 deg",
    "selftest.j1_lo": "j1 lower = -115 deg",
    "selftest.grip_limit": "ARX-X5 gripper soft limit = -157..142 deg",
    "selftest.clamp_hi": "clamped at the 90 deg upper limit",
    "selftest.clamp_lo": "clamped at the -115 deg lower limit",
    "selftest.clamp_in": "Targets within limits remain unchanged",
    "selftest.sec_step": "step choices and --arm-id",
    "selftest.step_rad": "rad choices are 0.005/0.01/0.02/0.05/0.1/0.2",
    "selftest.step_default": "Default joint step belongs to the presets",
    "selftest.step_m": "Metric steps increase and the default is valid",
    "selftest.arm_id": "parse_arm_id takes 0/1/0x1/space, rejects 2/-1/x/empty/1.0",
    "selftest.sec_two": "Shared link and independent arm states",
    "selftest.pass": "SDK and TUI tests passed.",
    "selftest.two_fb": "Feedback routes to both arms by arm_id",
    "selftest.two_target": "Arm targets are independent",
    "selftest.two_seeded": "Each arm target aligns to its feedback",
    "selftest.snap_keys": "Snapshot fields are complete",
    "selftest.fb_keys": "Feedback fields are complete",
    "selftest.fb_rad": "Joint feedback contains six rad values",
    "selftest.grip_cal": "Gripper calibration flag matches opening value; raw angle is available",
    "selftest.n_rx_fb": "Current-arm feedback count matches n_feedback",
    "selftest.arms_sorted": "Arm overview is sorted by ID",
    "selftest.sw_to_b": "Switch to arm B",
    "selftest.jog_cur": "Jog changes only the current arm target",
    "selftest.target_cur": "sess.target refers to the current arm",
    "selftest.sw_back": "Switch back to arm A",
    "selftest.hold_cur": "Hold affects only the current arm",
    "selftest.sec_cmd": "Commands and pre-checks",
    "selftest.a_wait_init": "Arm A enters WAIT_INIT",
    "selftest.init_sent": "WAIT_INIT sends 0x02 for the current arm",
    "selftest.init_done": "Initialization completes in NORMAL",
    "selftest.tid_start": "0x12 start",
    "selftest.start_ok": "NORMAL sends 0x12",
    "selftest.run_ok": "RUN mode feedback received",
    "selftest.i_no_frame": "Initialization in RUN is rejected without sending",
    "selftest.i_one_warn": "Rejected initialization reports one warning",
    "selftest.i_again": "Repeated initialization still requires a valid state",
    "selftest.z_no_frame": "Set-zero is rejected in NORMAL",
    "selftest.b_still": "Arm B remains in WAIT_INIT",
    "selftest.a_stream": "Only arm A sends position targets",
    "selftest.streaming_any": "Streaming check covers both arms",
    "selftest.b_init": "Arm B initialization uses arm_id=1",
    "selftest.b_done": "Arm B initialization completes",
    "selftest.b_run": "Arm B enters RUN",
    "selftest.both_stream": "Both arms send targets with their own IDs",
    "selftest.sec_reopen": "Reopen link",
    "selftest.reopen_ok": "reopen succeeded",
    "selftest.reopen_cleared": "Reopen clears both target-alignment flags",
    "selftest.reopen_seeded": "Both arms handshake and align targets after reopen",
    "selftest.stop_two": "Exit sends STOP to each arm",
    "selftest.sec_relink": "Automatic reconnect",
    "selftest.relink_snap": "Snapshot includes link loss, retry count and stopped status",
    "selftest.relink_seeded": "Both targets realign after reconnect",
    "selftest.relink_msg": "Automatic reconnect success is logged",
    "selftest.relink_giveup": "Reconnect stop and reason are logged",
    "selftest.fail": "Tests failed; see FAIL entries.",
    "selftest.table_bad": "Message table errors: %d",
    "selftest.fb_counts": "0x80 frames A=%s B=%s",
    "selftest.sent": "sent: %r",
})

# Arm 提示
_EN.update({
    "arm.snapshot_line": "arm%s state=%s mode=%s joints(rad)=%s gripper=%s temp(C)=%s offline=%s",
    "arm.grip_uncal": "uncalibrated (raw %.2f deg)",
    "arm.grip_inited_hint": "  Gripper initialized; calibration available",
    "arm.grip_not_homed":
        "  Gripper not initialized; initialize before calibration",
    "arm.no_feedback_yet": "  No feedback; call connect() first",
    "arm.arm_id_range": "arm_id must be 0..%d (arm count %d), got %r",
    "arm.no_version": "No version frame received",
    "arm.ver_ok": "0x%08X (matches the SDK)",
    "arm.ver_mismatch": "Firmware 0x%08X differs from SDK 0x%08X; handshake blocked",
    "arm.connect_timeout": "Connection timeout: no feedback in %.1f s from %s",
    "arm.connect_port": "  Port: %s",
    "arm.no_port": "No port selected",
    "arm.connect_link": "  link: open=%s up=%s",
    "arm.link_error": "  error: %s",
    "arm.connect_rx": "  %d frames received (CRC8 err %d, CRC16 err %d, dropped bytes %d)",
    "arm.connect_board_ver": "  Firmware version: %s",
    "arm.connect_zero_bytes": "  No valid frames; check wiring, port and firmware ROBOT_LINK",
    "arm.bytes_no_version": "  Frames received without a version frame; check protocol version and link",
    "arm.joint_angle": "joint angles (rad)",
    "arm.gate_init_state": "Current %s / %s; initialization requires %s, or %s with STOP/damping",
    "arm.gate_setzero_state": "Current %s; set-zero requires %s",
    "arm.gate_mode_state": "Current %s; mode change requires %s",
    "arm.gate_ctrl_state": "Current %s; target commands require %s",
    "arm.gate_not_run": "Current mode %s; call start() and wait for RUN feedback",
    "arm.gate_reset_state": "Current %s; reset requires %s",
    "arm.payload_type": "payload must be bytes, got %r",
    "arm.setzero_timeout": "Set-zero timeout: no 0x11 within %.1f s, current state %s. "
                           "Check command acceptance and calibration progress",
    "arm.stale_no_fb": "%s not sent: %s has no feedback; call connect() first",
    "arm.stale_old": "%s not sent: feedback age %.0f ms exceeds %.0f ms; link_up=%s, check link",
    "arm.gate_rejected": "%s not sent; pre-check failed:\n  - %s\n  state=%s(%s), mode=%s(%s), offline=%s",
    "arm.motors_lost": "Motors offline: %s",
    "arm.pose_dev": "Pose error %.2f° exceeds %.0f°; %s current %.2f°, reset %.2f°",
    "arm.grip_two_targets": "Specify either gripper_m or gripper_raw_deg, not both",
    "arm.grip_no_target": "No gripper target or feedback; specify a target or call connect() first",
    "arm.desc_line": "%s: %s",
    "arm.describe_no_fb": "%s: no feedback, link_up=%s",
    "arm.no_feedback": "no feedback",
    "arm.no_port_open": "Port not open",
})

# 链路提示
_EN.update({
    "link.kind_usb": "H7 USB CDC",
    "link.kind_rs485": "USB-to-RS485 adapter",
    "link.kind_fake": "H7 protocol simulator",
    "link.kind_unknown": "unknown link",
    "link.not_link_dev": ", not a control link",
    "link.desc_port": "%s (USB %s:%s %s %s%s)",
    "link.desc_no_ident": "%s (no USB identity found)",
    "link.desc_kind_tag": ", %s",
    "link.hint_uart": "ROBOT_LINK_UART: connect a USB-to-RS485 adapter;",
    "link.hint_usb": "ROBOT_LINK_USB: connect board USB CDC (%s:%s).",
    "link.hint_cands": "Candidate ports:",
    "link.hint_no_tty": "No ttyUSB/ttyACM devices; check wiring",
    "link.hint_port_arg": "Use port= to select a port.",
    "link.reject_probe_ident": "Cannot open %s: J-Link debug port; use a control link.\n  %s",
    "link.reject_probe_name": "Cannot open %s: unknown USB identity and possible debug port.\n  %s",
    "link.not_open": "Link not open; call link.open() or arm.connect() first",
    "link.down_reconnecting": "Link lost, reconnect %d/%d; frame not sent",
    "link.open_failed": "cannot open %s: %s\n%s",
    "link.write_link_err": "serial write failed: %s",
    "link.write_failed": "write to %s failed: %s",
    "link.read_link_err": "serial read failed: %s",
    "link.parse_link_err": "parse failed: %s",
    "link.arm_id_range": "arm_id must be 0..%d (arm count %d), got %r",
    "link.describe_line": "%s, %s",
    "link.auto_port": "(auto)",
})


def message_table(lang=None):
    """返回指定语言的文案表，供校验使用。"""
    return _ZH if (lang or _LANG) == "zh" else _EN


def tr(key, *args):
    """查找文案并按 args 格式化；无参数时返回原文。"""
    s = _ZH.get(key) if _LANG == "zh" else _EN.get(key)
    if s is None:                       # 缺少翻译时回退到中文。
        s = _ZH.get(key)
        if s is None:
            return "[missing:%s]" % key
    return s % args if args else s


# 状态与轴名。英文轴名须适配四字符列宽，夹爪使用 grip。

_MODE_EN = {
    board_defs.MODE_STOP: "stop(soft)",
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
    """返回状态名称；中文使用 board_defs 中的名称。"""
    if _LANG == DEFAULT_LANG:
        return board_defs.state_name(v)
    return _STATE_EN.get(v, "UNKNOWN(%d)" % v)


def mode_name(v):
    if _LANG == DEFAULT_LANG:
        return board_defs.mode_name(v)
    return _MODE_EN.get(v, "UNKNOWN(%d)" % v)


def zero_state_name(v):
    """返回设零状态名称，未知状态显示原始值。"""
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
    """将电机掩码转换为空格分隔的轴名。"""
    if _LANG == DEFAULT_LANG:
        return board_defs.expand_mask(mask)
    if not mask:
        return "none"
    return " ".join(_AXIS_EN[i] for i in range(board_defs.ARM_MOTOR_NUM) if mask >> i & 1)


# 文案表校验

# 匹配百分号格式占位符，转义的 %% 单独处理。
_SPEC = re.compile(r"%(?:\([^)]*\))?[-+ #0]*[0-9]*(?:\.[0-9]+)?[diouxXeEfFgGcrsa%]")

_FORBIDDEN = (("　", "〿"),      # 中日韩标点
              ("一", "鿿"),      # 中日韩表意文字
              ("＀", "￯"))      # 全角字符


def _bad_chars(s):
    return sorted({c for c in s
                   if any(lo <= c <= hi for lo, hi in _FORBIDDEN)})


def _specs(s):
    """提取格式占位符和未匹配的百分号，忽略转义的 %%。"""
    chars = [m.group(0)[-1] for m in _SPEC.finditer(s) if m.group(0) != "%%"]
    rest = _SPEC.sub("", s)
    return chars, rest.count("%")


def check_tables():
    """检查文案键、占位符和语言；无问题时返回空列表。"""
    bad = []
    for key in sorted(set(_ZH) | set(_EN)):
        zh, en = _ZH.get(key), _EN.get(key)
        if zh is None:
            bad.append("%s: 缺少中文消息" % key)
            continue
        if en is None:
            bad.append("%s: 缺少英文消息" % key)
            continue
        for tag, s in (("zh", zh), ("en", en)):
            if not isinstance(s, str):
                bad.append("%s[%s]: 消息须为 str" % (key, tag))
        if not isinstance(zh, str) or not isinstance(en, str):
            continue
        zc, zl = _specs(zh)
        ec, el = _specs(en)
        if sorted(zc) != sorted(ec):
            bad.append("%s: %% 转换符不一致 zh=%r en=%r" % (key, "".join(zc), "".join(ec)))
        if zl:
            bad.append("%s[zh]: 无效的 %% 占位符；字面百分号请写 %%%%" % key)
        if el:
            bad.append("%s[en]: 无效的 %% 占位符；字面百分号请写 %%%%" % key)
        chars = _bad_chars(en)
        if chars:
            bad.append("%s[en]: 包含中文或全角字符 %r" % (key, "".join(chars)))

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
                bad.append("%s[%r][en]: 包含中文或全角字符 %r" % (what, k, "".join(chars)))
    if len(_AXIS_EN) != len(board_defs.AXIS_NAMES):
        bad.append("AXIS 名表个数和 board_defs 不一致")
    for v in _AXIS_EN:
        chars = _bad_chars(v)
        if chars:
            bad.append("AXIS 名表[en]: 包含中文或全角字符 %r" % "".join(chars))
    return bad
