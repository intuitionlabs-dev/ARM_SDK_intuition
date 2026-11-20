"""TUI 会话层：维护双臂目标、发送命令、记录事件并提供反馈视图。

snapshot() 提供同帧 SI 数据，tick() 驱动控制和保活，jog()/send_*() 处理操作。
双臂共享 Link，各自维护目标与保活；切换当前臂仅改变操作和显示对象。
"""

import argparse
import time
from collections import deque

from .. import i18n, protocol, units
from ..board_defs import arx_x5 as board_defs
from ..errors import ArError, FeedbackStale, GateRejected
from ..link import Link
from ..sim import SimLink

# 会话参数与辅助函数

DEFAULT_ARM_ID = protocol.DEFAULT_ARM_ID
N_ARM = board_defs.ARM_NUM                 # 2
N_AXIS = board_defs.ARM_MOTOR_NUM          # 七个电机：六轴和夹爪
GRIPPER = board_defs.GRIPPER               # 6

# 关节步长使用 rad，已标定夹爪使用 m。
STEP_CHOICES_RAD = [0.005, 0.01, 0.02, 0.05, 0.1, 0.2]
STEP_CHOICES_M = [0.0005, 0.001, 0.002, 0.005, 0.01, 0.02]
DEFAULT_STEP_RAD = 0.02                    # 默认关节步长，约 1.1°
DEFAULT_STEP_M = 0.002

# 控制前检查反馈年龄，阈值与 SDK 一致。
FEEDBACK_FRESH_S = 0.1

MODE_TID_NAME = {protocol.TID_ARM_START: ("session.tid.start", "run"),
                 protocol.TID_ARM_GRAVITY: ("session.tid.gravity", "gravity"),
                 protocol.TID_ARM_STOP: ("session.tid.stop", "stop"),
                 protocol.TID_ARM_DAMPING: ("session.tid.damping", "damping")}


def short_label(arm_id):
    """返回日志使用的简短臂名，如 ArmA。"""
    return i18n.arm_label(arm_id).split("(")[0]


def parse_arm_id(s):
    """校验启动时的当前臂编号，范围为 0..ARM_NUM-1。"""
    try:
        v = int(s, 0)
    except ValueError:
        raise argparse.ArgumentTypeError(i18n.tr("session.arm_id.type", N_ARM - 1))
    if not 0 <= v < N_ARM:
        raise argparse.ArgumentTypeError(i18n.tr("session.arm_id.range", N_ARM - 1, N_ARM))
    return v


def axis_lo_rad(axis, defs=board_defs):
    """返回所选型号的单轴下限，单位 rad；夹爪使用原始电机角。"""
    return (defs.GRIPPER_ANGLE_MIN_RAD if axis == GRIPPER
            else defs.JOINT_ANGLE_MIN_RAD[axis])


def axis_hi_rad(axis, defs=board_defs):
    return (defs.GRIPPER_ANGLE_MAX_RAD if axis == GRIPPER
            else defs.JOINT_ANGLE_MAX_RAD[axis])


def clamp_axis_rad(axis, value, defs=board_defs):
    """按型号软限位限制目标，单位 rad，返回 (目标值, 是否限幅)。"""
    lo, hi = axis_lo_rad(axis, defs), axis_hi_rad(axis, defs)
    if value < lo:
        return lo, True
    if value > hi:
        return hi, True
    return value, False


def gripper_m_limits(calib, defs=board_defs):
    """将夹爪原始角软限位换算为 m，按升序返回。"""
    a = calib.opening_from_raw_deg(defs.GRIPPER_ANGLE_MIN)
    b = calib.opening_from_raw_deg(defs.GRIPPER_ANGLE_MAX)
    return (a, b) if a <= b else (b, a)


# 双臂会话，仅由 UI 线程发送命令

class ArmView:
    """单臂界面状态；目标包含六个关节角和夹爪原始角，均使用 rad。"""

    __slots__ = ("arm_id", "arm", "target", "sel", "hold", "seeded", "prev",
                 "last_tx", "last_ctrl", "ctrl_live", "init_t0", "init_told_reset",
                 "reset_t0", "force_init", "have_ever_rx", "handshake_reported",
                 "grip_hint_shown", "ctrl_err_reported", "step_idx_rad", "step_idx_m",
                 "_rate_n", "_rate_t", "_rate_hz")

    def __init__(self, arm, opts):
        self.arm_id = arm.arm_id
        self.arm = arm
        self.target = [0.0] * N_AXIS
        self.sel = 0
        self.hold = False              # 暂停位置目标流，不是停机
        self.seeded = False
        self.prev = None               # 上一帧状态、模式和掉线掩码
        self.last_tx = 0.0             # 最近一次发送时刻，含控制帧
        self.last_ctrl = 0.0
        self.ctrl_live = False         # 上一轮是否发送控制帧
        self.init_t0 = None
        self.init_told_reset = False   # 初始化是否已到复位阶段
                                       # 仅用于显示
        self.reset_t0 = None
        self.force_init = False        # 姿态预检已拒绝一次初始化
                                       # 再次按键可跳过姿态检查
        self.have_ever_rx = False
        self.handshake_reported = False
        self.grip_hint_shown = False   # 未标定提示仅报告一次
        self.ctrl_err_reported = False # 周期发送错误仅报告一次
                                       # 避免重复日志
        self.step_idx_rad = (STEP_CHOICES_RAD.index(opts.step)
                             if opts.step in STEP_CHOICES_RAD
                             else STEP_CHOICES_RAD.index(DEFAULT_STEP_RAD))
        self.step_idx_m = STEP_CHOICES_M.index(DEFAULT_STEP_M)
        self._rate_n = 0
        self._rate_t = time.monotonic()
        self._rate_hz = 0.0

    # 已标定且选中夹爪时使用米制步长

    @property
    def step_in_m(self):
        return self.sel == GRIPPER and self.arm.calibration is not None

    @property
    def step(self):
        return (STEP_CHOICES_M[self.step_idx_m] if self.step_in_m
                else STEP_CHOICES_RAD[self.step_idx_rad])

    @property
    def step_unit(self):
        return "m" if self.step_in_m else "rad"

    @property
    def step_str(self):
        """返回带单位的当前步长文本。"""
        return ("%.4f m" % self.step) if self.step_in_m else ("%.3f rad" % self.step)


# 将界面访问的属性转发到当前臂。
_PER_ARM_ATTRS = frozenset({
    "arm", "arm_id", "target", "sel", "hold", "seeded", "prev", "last_tx",
    "last_ctrl", "ctrl_live", "init_t0", "init_told_reset", "reset_t0",
    "force_init", "have_ever_rx", "handshake_reported", "grip_hint_shown",
    "ctrl_err_reported", "step", "step_unit", "step_str", "step_in_m",
})


class ConsoleSession:
    """从反馈更新双臂状态并发送命令。

    正常运行 / RUN 且未暂停时按 ctrl-hz 发送目标，否则每 150 ms 保活；每臂独立执行。
    """

    KEEPALIVE_S = 0.15

    def __init__(self, link, opts, arm_ids=None):
        # 会话级属性使用 object.__setattr__，避免被臂属性转发规则拦截或递归。
        object.__setattr__(self, "link", link)
        object.__setattr__(self, "board_defs", getattr(link, "board_defs", board_defs))
        object.__setattr__(self, "opts", opts)
        object.__setattr__(self, "events", deque(maxlen=400))
        object.__setattr__(self, "dirty", True)
        object.__setattr__(self, "t0", time.monotonic())
        object.__setattr__(self, "version_reported", False)
        object.__setattr__(self, "cur_arm_id", opts.arm_id)
        # 首次 tick() 记录链路基准，后续报告状态变化。
        object.__setattr__(self, "_link_gen", None)
        object.__setattr__(self, "_link_up", False)
        object.__setattr__(self, "_link_gave_up", False)
        arms = {}
        views = {}
        for arm_id in (range(N_ARM) if arm_ids is None else arm_ids):
            arm = link.arm(arm_id)
            arms[arm_id] = arm
            views[arm_id] = ArmView(arm, opts)
        object.__setattr__(self, "arms", arms)
        object.__setattr__(self, "views", views)

    # 当前臂属性转发

    @property
    def cur(self):
        return self.views[self.cur_arm_id]

    def __getattr__(self, name):
        if name in _PER_ARM_ATTRS:
            return getattr(self.views[self.cur_arm_id], name)
        raise AttributeError(name)

    def __setattr__(self, name, value):
        if name in _PER_ARM_ATTRS:
            setattr(self.cur, name, value)
        else:
            object.__setattr__(self, name, value)

    def switch_arm(self, arm_id=None):
        """切换操作和显示的臂，双臂控制与保活继续运行。"""
        if arm_id is None:
            arm_id = (self.cur_arm_id + 1) % N_ARM
        if arm_id == self.cur_arm_id:
            return
        self.cur_arm_id = arm_id
        self.dirty = True
        self.info(i18n.tr("session.switch_arm", i18n.arm_label(arm_id)))

    # 事件

    def event(self, level, text):
        self.events.append((time.time(), level, text))
        self.dirty = True

    def streaming_any(self):
        """检查任一臂是否正在控制或暂停控制，用于退出确认。"""
        return any(v.ctrl_live or v.hold for v in self.views.values())

    def info(self, text):
        self.event("info", text)

    def warn(self, text):
        self.event("warn", text)

    def err(self, text):
        self.event("err", text)

    # 界面数据读取

    def snapshot(self):
        """返回同帧 SI 视图；夹爪未标定时开口为 None，界面显示原始角。"""
        view = self.cur
        arm = view.arm
        link = self.link
        st = link.stats
        sp = arm.snapshot()
        now = time.monotonic()

        if sp is None:
            fb = None
            fb_t = 0.0
        else:
            fb = {"mode": sp.mode, "state": sp.robot_state,
                  "lost": sp.motor_lost_mask, "inited": sp.motor_inited_mask,
                  "joint": list(sp.joint_angles),                        # rad
                  "jvel": list(sp.joint_velocities),                     # rad/s
                  "gripper": sp.gripper_opening,                         # m / None
                  "gripper_raw_rad": units.deg_to_rad(sp.gripper_raw_deg),
                  "gripper_calibrated": sp.gripper_calibrated,
                  "temp": list(sp.temperatures)}
            fb_t = sp.taken_at

        chan = link.channel(self.cur_arm_id)
        ack, ack_t, _ = chan.snapshot_ack()

        return {
            "fb": fb,
            "fb_t": fb_t,
            "ack": ({"state": ack.state, "result": list(ack.result),
                     "elapsed_ms": ack.elapsed_ms} if ack is not None else None),
            "ack_t": ack_t,
            "board_version": st["board_version"],
            "link_up": st["link_up"],
            "link_err": st["link_err"],
            "link_auto": st["auto_reconnect"],        # 是否启用自动重连
            "link_retry": st["reconnect_tries"],      # try number in this round (0 = none yet)
            "link_retry_max": st["reconnect_max_tries"],
            "link_give_up": st["reconnect_gave_up"],  # 已达到重试上限
            "n_rx": st["n_rx"],                     # 双臂累计链路统计
            "n_rx_fb": arm.n_feedback,              # 本臂反馈帧数
            "rx_rate": self._rate(view, now),       # 本臂反馈频率
            "n_tx": st["n_tx"],
            "n_drop": st["n_drop"],
            "n_crc_err": st["n_err_crc8"] + st["n_err_crc16"],
            "n_bad_arm": st["n_bad_arm"],
            "unknown_tids": dict(chan.unknown_tids),
            "arm_id": self.cur_arm_id,
            "arms": [self._arm_line(arm_id, now) for arm_id in sorted(self.views)],
        }

    def _arm_line(self, arm_id, now):
        """返回双臂概览中的单臂状态。"""
        view = self.views[arm_id]
        sp = view.arm.snapshot()
        return {"arm_id": arm_id,
                "state": None if sp is None else sp.robot_state,
                # 从状态码生成所选语言的名称；公共 Snapshot 名称仍为中文。
                "state_name": i18n.tr("session.no_feedback") if sp is None
                else i18n.state_name(sp.robot_state),
                "mode_name": "-" if sp is None else i18n.mode_name(sp.mode),
                "lost": None if sp is None else sp.motor_lost_mask,
                "ctrl_live": view.ctrl_live,
                "have_ever_rx": view.have_ever_rx,
                "n_feedback": view.arm.n_feedback,
                "rx_rate": self._rate(view, now)}

    def _rate(self, view, now):
        """计算本臂最近 0.5 s 的反馈频率。"""
        dt = now - view._rate_t
        if dt >= 0.5:
            view._rate_hz = (view.arm.n_feedback - view._rate_n) / dt
            view._rate_n = view.arm.n_feedback
            view._rate_t = now
        return view._rate_hz

    def fb(self):
        """返回当前臂的反馈视图；尚无反馈时返回 None。"""
        return self.snapshot()["fb"]

    def gripper_row(self, fb):
        """返回夹爪行数据；有标定时使用 m，否则使用原始角 rad。"""
        view = self.cur
        calib = view.arm.calibration
        if calib is not None:
            lo, hi = gripper_m_limits(calib, self.board_defs)
            return {"calibrated": True, "unit": "m",
                    "tgt": calib.opening_from_raw_deg(units.rad_to_deg(view.target[GRIPPER])),
                    "fb": (fb["gripper"] if fb else None), "lo": lo, "hi": hi}
        return {"calibrated": False, "unit": "rad",
                "tgt": view.target[GRIPPER],
                "fb": (fb["gripper_raw_rad"] if fb else None),
                "lo": axis_lo_rad(GRIPPER, self.board_defs), "hi": axis_hi_rad(GRIPPER, self.board_defs)}

    def state_mode(self):
        snap = self.snapshot()
        fb = snap["fb"]
        if fb is None:
            return None, None, snap
        return fb["state"], fb["mode"], snap

    # 目标与步长

    def seed_from_feedback(self, fb, why):
        view = self.cur
        view.target = list(fb["joint"]) + [fb["gripper_raw_rad"]]
        view.seeded = True
        self.info(i18n.tr("session.seeded", short_label(view.arm_id), why))

    def _seed(self, view, sp, why):
        """将目标对齐当前反馈。"""
        view.target = list(sp.joint_angles) + [units.deg_to_rad(sp.gripper_raw_deg)]
        view.seeded = True
        self.info(i18n.tr("session.seeded", short_label(view.arm_id), why))

    def jog(self, axis, sign):
        """按键只修改目标，周期发送由 tick() 执行。"""
        view = self.cur
        if axis is None:
            return
        sp = view.arm.snapshot()
        if sp is None:
            self.warn(i18n.tr("session.no_feedback_yet"))
            return
        lbl = short_label(view.arm_id)

        if axis == GRIPPER and view.step_in_m:
            # 先按米调整夹爪目标，再换算为原始角。
            calib = view.arm.calibration
            cur_m = calib.opening_from_raw_deg(units.rad_to_deg(view.target[axis]))
            want_m = cur_m + sign * view.step
            lo_m, hi_m = gripper_m_limits(calib, self.board_defs)
            if want_m < lo_m or want_m > hi_m:
                got_m = min(max(want_m, lo_m), hi_m)
                self.warn(i18n.tr("session.grip_limit_m", lbl, want_m, lo_m, hi_m, got_m))
                view.target[axis] = units.deg_to_rad(calib.raw_deg_from_opening(got_m))
                return
            view.target[axis] = units.deg_to_rad(calib.raw_deg_from_opening(want_m))
            self.dirty = True
            return

        want = view.target[axis] + sign * view.step
        got, clamped = clamp_axis_rad(axis, want, self.board_defs)
        view.target[axis] = got
        if clamped:
            self.warn(i18n.tr("session.axis_limit", lbl, i18n.axis_name(axis),
                              units.rad_to_deg(want),
                              units.rad_to_deg(axis_lo_rad(axis, self.board_defs)),
                              units.rad_to_deg(axis_hi_rad(axis, self.board_defs)),
                              units.rad_to_deg(got)))
        else:
            self.dirty = True

    def cycle_step(self, direction=1):
        view = self.cur
        lbl = short_label(view.arm_id)
        if view.step_in_m:
            view.step_idx_m = (view.step_idx_m + direction) % len(STEP_CHOICES_M)
        else:
            view.step_idx_rad = (view.step_idx_rad + direction) % len(STEP_CHOICES_RAD)
        self.info(i18n.tr("session.step", lbl, view.step_str))

    # 命令发送；将 SDK 异常记录为事件

    def _report_send_failure(self, view, what, e):
        """记录发送失败：预检拒绝为 warn，其他发送错误为 err。"""
        lbl = short_label(view.arm_id)
        if isinstance(e, (GateRejected, FeedbackStale)):
            self.warn(i18n.tr("session.send_gated", lbl, what, e))
        else:
            self.err(i18n.tr("session.send_error", lbl, what, e))

    def send_empty_mode(self, tid, name):
        view = self.cur
        arm = view.arm
        fn = {protocol.TID_ARM_START: arm.start, protocol.TID_ARM_GRAVITY: arm.gravity,
              protocol.TID_ARM_STOP: arm.stop, protocol.TID_ARM_DAMPING: arm.damping}[tid]
        try:
            fn()
        except ArError as e:
            self._report_send_failure(view, name, e)
            return
        self.info(i18n.tr("session.sent_mode", short_label(view.arm_id), name,
                          MODE_TID_NAME[tid][1]))
        view.last_tx = time.monotonic()

    def send_arm_init(self):
        """发送初始化；再次按 i 仅跳过姿态预检，仍检查状态和电机在线。"""
        view = self.cur
        arm = view.arm
        lbl = short_label(view.arm_id)
        sp = arm.snapshot()
        if sp is None:
            self.warn(i18n.tr("session.init_no_fb", lbl))
            return

        bad = [(self.board_defs.JOINT_NAMES[i],
                units.rad_to_deg(sp.joint_angles[i] - want))
               for i, want in enumerate(self.board_defs.RESET_POSE_RAD)
               if abs(sp.joint_angles[i] - want) > self.opts.init_tol]
        if bad:
            if not view.force_init:
                view.force_init = True
                self.warn(i18n.tr(
                    "session.init_pose_bad", lbl, units.rad_to_deg(self.opts.init_tol),
                    i18n.tr("session.pose_dev_sep").join(
                        i18n.tr("session.pose_dev_item", n, d) for n, d in bad),
                    " ".join("%+.4f" % v for v in self.board_defs.RESET_POSE_RAD)))
                return
            self.warn(i18n.tr("session.init_force", lbl))
        try:
            arm.init_arm(force=bool(bad))
        except ArError as e:
            view.force_init = False
            self._report_send_failure(view, i18n.tr("session.what.init"), e)
            return
        view.force_init = False
        self.info(i18n.tr("session.init_sent", lbl))
        view.last_tx = time.monotonic()

    def send_arm_reset(self):
        """发送复位，要求正常运行且全部电机在线。"""
        view = self.cur
        lbl = short_label(view.arm_id)
        try:
            view.arm.reset()
        except ArError as e:
            self._report_send_failure(view, i18n.tr("session.what.reset"), e)
            return
        self.info(i18n.tr("session.reset_sent", lbl,
                          " ".join("%+.4f" % v for v in self.board_defs.RESET_POSE_RAD)))
        now = time.monotonic()
        view.last_tx = now
        view.reset_t0 = now

    def send_setzero(self):
        view = self.cur
        try:
            view.arm.set_zero()
        except ArError as e:
            self._report_send_failure(view, i18n.tr("session.what.setzero"), e)
            return
        self.err(i18n.tr("session.setzero_sent", short_label(view.arm_id)))
        view.last_tx = time.monotonic()

    def stop_all(self):
        """退出前向双臂发送 0x14 STOP。"""
        done = []
        for arm_id in sorted(self.views):
            try:
                self.views[arm_id].arm.stop(force=True)
                done.append(short_label(arm_id))
            except ArError as e:
                self.err(i18n.tr("session.stop_all_fail", short_label(arm_id), e))
        return done

    # 链路操作

    def reopen(self):
        """重开链路并记录结果；失败后按配置继续后台重连。"""
        try:
            self.link.reopen()
        except ArError as e:
            self.err(i18n.tr("session.reopen_fail", e))
            self.dirty = True
            return False
        self._link_gen = getattr(self.link, "open_gen", 0)
        self._link_up = True
        self._link_gave_up = False
        self._on_link_back("session.reopened")
        return True

    def _on_link_back(self, key):
        """连接恢复后重新加载标定，并在下一帧反馈到达时对齐目标。"""
        for view in self.views.values():
            view.handshake_reported = False
            view.ctrl_err_reported = False
            view.prev = None
            view.seeded = False
        self.t0 = time.monotonic()
        for view in self.views.values():
            try:
                view.arm.reload_calibration()
            except ArError:
                pass
        self.info(i18n.tr(key, self.link.describe() if not self.opts.fake
                          else i18n.tr("session.fake_board")))
        self.dirty = True

    def _watch_link(self):
        """检测断开、自动重连和重试结束；open_gen 变化表示新连接建立。"""
        link = self.link
        gen = getattr(link, "open_gen", 0)
        up = bool(link.link_up)
        gave_up = bool(getattr(link, "reconnect_gave_up", False))
        if self._link_gen is None:
            self._link_gen, self._link_up, self._link_gave_up = gen, up, gave_up
            return
        if gen != self._link_gen:
            self._link_gen = gen
            if up:
                self._on_link_back("session.relinked")
        if up != self._link_up:
            self._link_up = up
            if not up:
                err = link.link_err or ""
                self.warn(i18n.tr("session.link_lost" if link.auto_reconnect
                                 else "session.link_lost_manual", err))
        if gave_up and not self._link_gave_up:
            self.warn(i18n.tr("session.relink_giveup", link.reconnect_tries,
                              link.reconnect_err or link.link_err or ""))
        self._link_gave_up = gave_up

    # 周期处理

    def tick(self, now):
        self._watch_link()
        for arm_id in sorted(self.views):
            view = self.views[arm_id]
            sp = view.arm.snapshot()
            if sp is not None:
                self._on_feedback(view, sp, now)
            elif now - self.t0 > self.opts.handshake_timeout \
                    and not view.handshake_reported:
                view.handshake_reported = True
                self.dirty = True

            can_ctrl = (sp is not None
                        and sp.robot_state == self.board_defs.STATE_NORMAL
                        and sp.mode == self.board_defs.MODE_RUN
                        and not view.hold
                        and self.link.link_up
                        and sp.age <= FEEDBACK_FRESH_S)
            if can_ctrl:
                if now - view.last_ctrl >= 1.0 / max(1.0, self.opts.ctrl_hz):
                    view.last_ctrl = now
                    view.ctrl_live = self._send_ctrl(view, now)
            else:
                view.ctrl_live = False
                if now - view.last_tx > self.KEEPALIVE_S:
                    self._send_keepalive(view, now)

            if view.init_t0 is not None and sp is not None \
                    and sp.robot_state != self.board_defs.STATE_INITING:
                view.init_t0 = None
            if view.reset_t0 is not None and sp is not None \
                    and sp.robot_state != self.board_defs.STATE_RESETTING:
                view.reset_t0 = None
            self._rate(view, now)

    def _send_ctrl(self, view, now):
        """发送周期目标；会话已检查状态、模式和反馈年龄，跳过 Arm 的重复预检。"""
        try:
            view.arm.send_ctrl_pos(view.target[:6],
                                   gripper_torque_limit_nm=self.board_defs.DEFAULT_GRIPPER_TORQUE_LIMIT_NM,
                                   gripper_raw_deg=units.rad_to_deg(view.target[GRIPPER]),
                                   force=True)
        except ArError as e:
            if not view.ctrl_err_reported:
                view.ctrl_err_reported = True
                self.err(i18n.tr("session.ctrl_tx_fail", short_label(view.arm_id), e))
            return False
        view.ctrl_err_reported = False
        view.last_tx = now
        return True

    def _send_keepalive(self, view, now):
        try:
            view.arm.keepalive()
        except ArError as e:
            if not view.ctrl_err_reported:
                view.ctrl_err_reported = True
                self.err(i18n.tr("session.keepalive_tx_fail", short_label(view.arm_id), e))
            return False
        view.ctrl_err_reported = False
        view.last_tx = now
        return True

    def _report_handshake(self):
        """每条链路仅报告一次握手结果。"""
        if self.version_reported:
            return
        self.version_reported = True
        bv = self.link.board_version
        if bv is None:
            self.info(i18n.tr("session.hs_fb_only", self.board_defs.VERSION_CODE))
        elif bv == self.board_defs.VERSION_CODE:
            self.info(i18n.tr("session.hs_ok", bv, self.board_defs.VERSION_STRING))
        else:
            self.err(i18n.tr("session.hs_bad", bv, self.board_defs.VERSION_CODE))

    def _on_feedback(self, view, sp, now):
        """根据反馈生成事件并对齐目标，角度日志使用 deg。"""
        lbl = short_label(view.arm_id)
        if not view.have_ever_rx:
            view.have_ever_rx = True
            self._report_handshake()
            self._seed(view, sp, i18n.tr("session.why.first_fb"))
            if not sp.gripper_calibrated and view.arm_id == self.cur_arm_id \
                    and not view.grip_hint_shown:
                view.grip_hint_shown = True
                self.info(i18n.tr("session.grip_uncal", lbl, view.arm_id))

        st, mode, lost = sp.robot_state, sp.mode, sp.motor_lost_mask
        cur = (st, mode, lost)
        if view.prev is None:
            view.prev = cur
        elif cur != view.prev:
            pst, pmode, plost = view.prev
            if st != pst:
                self.info(i18n.tr("session.state_changed", lbl,
                                  i18n.state_name(pst), i18n.state_name(st)))
                if st == self.board_defs.STATE_INITING:
                    view.init_t0 = now
                    view.init_told_reset = False
                if st == self.board_defs.STATE_RESETTING:
                    view.reset_t0 = now
                # 通信恢复或初始化完成后对齐目标，避免恢复 RUN 时使用旧目标。
                if st == self.board_defs.STATE_WAIT_INIT and pst in (
                        self.board_defs.STATE_INITING, self.board_defs.STATE_COMM_LOST,
                        self.board_defs.STATE_CALIBRATING):
                    self._seed(view, sp, i18n.tr("session.why.wait_init"))
                if st == self.board_defs.STATE_NORMAL and pst == self.board_defs.STATE_RESETTING:
                    # 复位完成后对齐目标，避免下一帧指令返回复位前的位置。
                    self._seed(view, sp, i18n.tr("session.why.reset_tail"))
                    bad = [(self.board_defs.JOINT_NAMES[i],
                            units.rad_to_deg(sp.joint_angles[i] - want))
                           for i, want in enumerate(self.board_defs.RESET_POSE_RAD)
                           if abs(sp.joint_angles[i] - want) > self.opts.init_tol]
                    if bad:
                        # 状态已结束但关节未到参考角时，报告可能未执行或被中断。
                        self.err(i18n.tr(
                            "session.reset_skipped", lbl,
                            i18n.tr("session.pose_dev_sep").join(
                                i18n.tr("session.pose_dev_item", n, d) for n, d in bad),
                            units.rad_to_deg(self.opts.init_tol)))
                    else:
                        self.info(i18n.tr("session.reset_done", lbl))
                    view.reset_t0 = None
            if mode != pmode:
                good = {1: "info", 2: "warn", 0: "warn", 3: "info"}.get(mode, "warn")
                text = i18n.tr("session.mode_changed", lbl, i18n.mode_name(pmode),
                               i18n.mode_name(mode))
                if pst in (self.board_defs.STATE_INITING, self.board_defs.STATE_RESETTING) \
                        and st == self.board_defs.STATE_NORMAL \
                        and mode == self.board_defs.MODE_DAMPING:
                    # 初始化或复位后的 RUN→阻尼属于正常结束。
                    good = "info"
                    text += i18n.tr("session.mode_tail_normal")
                if mode == self.board_defs.MODE_RUN:
                    self._seed(view, sp, i18n.tr("session.why.enter_run"))
                    self.event("info", text)
                elif pmode == self.board_defs.MODE_RUN:
                    # 电机掉线可能触发 RUN→STOP。
                    self.event("err" if lost else good,
                               text + i18n.tr("session.ctrl_stopped"))
                else:
                    self.event(good, text)
            if lost != plost:
                if lost:
                    self.err(i18n.tr("session.motor_lost", lbl, i18n.expand_mask(lost), lost))
                else:
                    self.info(i18n.tr("session.motor_recovered", lbl, plost))
            view.prev = cur
        elif not view.seeded:
            self._seed(view, sp, i18n.tr("session.why.reseed"))

        if st == self.board_defs.STATE_INITING and view.init_t0 is not None:
            self.dirty = True      # 刷新初始化用时显示
            if now - view.init_t0 > 4.0 and not view.init_told_reset:
                view.init_told_reset = True
                self.info(i18n.tr("session.initing_note", lbl))


# 会话创建

def build_session(opts):
    """根据配置创建共享链路和双臂会话。"""
    robot_type = getattr(opts, "robot_type", None)
    if opts.fake:
        # 掉线模拟仅作用于启动时选择的臂
        link = SimLink(lost_mask=opts.fake_lost, lost_arm=opts.arm_id, robot_type=robot_type)
    else:
        link = Link(opts.port, opts.baud, robot_type=robot_type)
    return ConsoleSession(link, opts)
