"""交互式终端的后端：`sdk/tui/app.py` 里「除 curses 之外的一切」。

为什么要单独一个文件：面板那几百行只管画和按键；链路、协议、闸门预检、单位换算
**全部来自 `sdk/`**。这个模块是唯一的接缝，面板只认下面这几样：

    sess.snapshot()                一帧一致视图（**SI**），键名固定，draw() 全靠它
    sess.tick(now)                 每拍推进：发 0x16，或补一帧 0x01 保活
    sess.jog/cycle_step/send_*     按键动作（闸门交给 sdk.Arm 的预检）
    sess.target/sel/step/hold/...  「当前臂」的状态，面板直接读写
    sess.switch_arm(i)             a 键切「当前臂」

**两台臂**：一条 `Link` + 每台一个 `sdk.Arm`。**两台都在被驱动** —— 各自的目标角、
各自的 0x16 流、各自的保活计时，`a` 键只是切换"面板在看哪一台"。所以切走的那台不会
停在那儿不管（它跟的还是它自己那份目标角）。

面板**不知道有两台**：下面 `ConsoleSession.__getattr__/__setattr__` 把 `target/sel/step/...`
这批名字转发给"当前臂"，面板里那些 `sess.xxx` 一个字都不用改。
"""

import argparse
import time
from collections import deque

from .. import board_defs, i18n, protocol, units
from ..errors import ArError, FeedbackStale, GateRejected
from ..link import Link
from ..sim import SimLink

# ============================================================================
# A 层：常量与工具（协议 / 限位 / 状态名全部来自 sdk，这里只放"面板要的口径"）
# ============================================================================

DEFAULT_ARM_ID = protocol.DEFAULT_ARM_ID
N_ARM = board_defs.ARM_NUM                 # 2
N_AXIS = board_defs.ARM_MOTOR_NUM          # 7（6 关节 + 夹爪）
GRIPPER = board_defs.GRIPPER               # 6

# 点动步长档。**rad 原生** —— 面板和会话内部一律 rad；夹爪标定过之后单独用米档
# （它有真实行程，rad 档在"张开多少"这件事上不直观）。
STEP_CHOICES_RAD = [0.005, 0.01, 0.02, 0.05, 0.1, 0.2]
STEP_CHOICES_M = [0.0005, 0.001, 0.002, 0.005, 0.01, 0.02]
DEFAULT_STEP_RAD = 0.02                    # ≈ 旧版 1.0° 的档位
DEFAULT_STEP_M = 0.002

# 「这一帧反馈够不够新」的门槛：和 `sdk.arm.FEEDBACK_STALE_S` 同一口径（100 ms）。
# 控制流自己判一次是为了不每拍都进 sdk 的预检（100 Hz 下那纯属白跑）。
FEEDBACK_FRESH_S = 0.1

MODE_TID_NAME = {protocol.TID_ARM_START: ("session.tid.start", "run"),
                 protocol.TID_ARM_GRAVITY: ("session.tid.gravity", "gravity"),
                 protocol.TID_ARM_STOP: ("session.tid.stop", "stop"),
                 protocol.TID_ARM_DAMPING: ("session.tid.damping", "damping")}


def short_label(arm_id):
    """日志前缀用的短标签（`臂A`）。全称（含 CAN 号）用 `i18n.arm_label()`。"""
    return i18n.arm_label(arm_id).split("(")[0]


def parse_arm_id(s):
    """`--arm-id` 的取值校验（**启动时的"当前臂"**，运行中按 a 可以切）。

    板子侧只认 `arm_id < ARM_NUM`，越界的帧被**静默丢弃**（连版本包都不回），现象就是
    "什么都不动、也没有字节" —— 所以宁可在启动时直接报错，别让它变成一个要靠抓包才能
    诊断的沉默失败。
    """
    try:
        v = int(s, 0)
    except ValueError:
        raise argparse.ArgumentTypeError(i18n.tr("session.arm_id.type", N_ARM - 1))
    if not 0 <= v < N_ARM:
        raise argparse.ArgumentTypeError(i18n.tr("session.arm_id.range", N_ARM - 1, N_ARM))
    return v


def axis_lo_rad(axis):
    """某一根轴的软限位下限（rad）。夹爪那一路用原始角的 ∓142°，不是米。"""
    return (board_defs.GRIPPER_ANGLE_MIN_RAD if axis == GRIPPER
            else board_defs.JOINT_ANGLE_MIN_RAD[axis])


def axis_hi_rad(axis):
    return (board_defs.GRIPPER_ANGLE_MAX_RAD if axis == GRIPPER
            else board_defs.JOINT_ANGLE_MAX_RAD[axis])


def clamp_axis_rad(axis, value):
    """把一根轴夹进软限位（rad），返回 `(夹住后的值, 是否夹过)`。

    `sdk.board_defs` 只有 deg 版（`clamp_axis`）；会话内部一律 rad，所以这里用它的
    rad 限位镜像组合一份 —— 是组合，不是重抄一份限位表。
    """
    lo, hi = axis_lo_rad(axis), axis_hi_rad(axis)
    if value < lo:
        return lo, True
    if value > hi:
        return hi, True
    return value, False


def gripper_m_limits(calib):
    """标定过之后夹爪的米制软限位 `(lo, hi)`。

    两点的映射方向由标定自己决定（哪端是"张开"不预设），所以这里两端都算一遍再排序。
    """
    a = calib.opening_from_raw_deg(board_defs.GRIPPER_ANGLE_MIN)
    b = calib.opening_from_raw_deg(board_defs.GRIPPER_ANGLE_MAX)
    return (a, b) if a <= b else (b, a)


# ============================================================================
# C 层：会话（全部跑在 UI 线程，发送单写者）
# ============================================================================

class ArmView:
    """**一台臂**在面板里的那一份状态。`a` 键切的就是"面板在看哪一份"。

    目标角 `target` 的规范域：6 个关节 **rad** + 夹爪 **原始角 rad**（夹爪不做
    pos_to_angle 换算，米是标定出来的，不是协议里的量）。显示时按需换算。
    """

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
        self.hold = False              # 暂停这一台的 0x16 流（冻结目标，不是停机）
        self.seeded = False
        self.prev = None               # 上一帧 (state, mode, lost)
        self.last_tx = 0.0             # 上一帧发出去的时刻（0x16 也算）
        self.last_ctrl = 0.0
        self.ctrl_live = False         # 上一拍真的发出去了 0x16
        self.init_t0 = None
        self.init_told_reset = False   # 初始化流程走没走过"复位"这一步（只用来显示）
        self.reset_t0 = None
        self.force_init = False        # i 键第一遍被预检挡下，第二遍才强发
        self.have_ever_rx = False
        self.handshake_reported = False
        self.grip_hint_shown = False   # 夹爪未标定的提示只打一次
        self.ctrl_err_reported = False # 周期流的发送失败只报一次（100 Hz 会刷屏）
        self.step_idx_rad = (STEP_CHOICES_RAD.index(opts.step)
                             if opts.step in STEP_CHOICES_RAD
                             else STEP_CHOICES_RAD.index(DEFAULT_STEP_RAD))
        self.step_idx_m = STEP_CHOICES_M.index(DEFAULT_STEP_M)
        self._rate_n = 0
        self._rate_t = time.monotonic()
        self._rate_hz = 0.0

    # ---- 步长档：夹爪标定过、且选中的就是夹爪那一行 → 米档 ----

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
        """面板上「步长 xx」那一格的文字（单位随行变）。"""
        return ("%.4f m" % self.step) if self.step_in_m else ("%.3f rad" % self.step)


# 「当前臂」的转发表：面板直接读写的这些名字都指"当前臂"那一份。
_PER_ARM_ATTRS = frozenset({
    "arm", "arm_id", "target", "sel", "hold", "seeded", "prev", "last_tx",
    "last_ctrl", "ctrl_live", "init_t0", "init_told_reset", "reset_t0",
    "force_init", "have_ever_rx", "handshake_reported", "grip_hint_shown",
    "ctrl_err_reported", "step", "step_unit", "step_str", "step_in_m",
})


class ConsoleSession:
    """面板的会话：派生板子状态（从反馈帧，不自己造）、发命令、维护事件日志。

    `tick()` 每 2~5 ms 调一次（`getch` 超时）：
      - 能发 0x16 时（正常运行 + run + 没暂停）按 `--ctrl-hz` 发当前目标角；
      - 不能发时 150 ms 补一帧 0x01 保活 —— 否则初始化那 7 s 会掉进 comm_lost。
    **两台臂各跑一遍**，所以切走的那台也在被保活/被位控。
    """

    KEEPALIVE_S = 0.15

    def __init__(self, link, opts, arm_ids=None):
        # events / dirty / t0 / link / opts / cur_arm_id / views 是**会话级**的，
        # 必须用 object.__setattr__ 写：下面的转发规则会拦下同名属性（尤其 views
        # 自己，写错就无限递归）。
        object.__setattr__(self, "link", link)
        object.__setattr__(self, "opts", opts)
        object.__setattr__(self, "events", deque(maxlen=400))
        object.__setattr__(self, "dirty", True)
        object.__setattr__(self, "t0", time.monotonic())
        object.__setattr__(self, "version_reported", False)
        object.__setattr__(self, "cur_arm_id", opts.arm_id)
        arms = {}
        views = {}
        for arm_id in (range(N_ARM) if arm_ids is None else arm_ids):
            arm = link.arm(arm_id)
            arms[arm_id] = arm
            views[arm_id] = ArmView(arm, opts)
        object.__setattr__(self, "arms", arms)
        object.__setattr__(self, "views", views)

    # ---- 「当前臂」的间接层：面板读 sess.target 就是读当前臂的 ----

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
        """切「当前臂」（a 键）。只是换面板看谁 —— 两台一直在被驱动。"""
        if arm_id is None:
            arm_id = (self.cur_arm_id + 1) % N_ARM
        if arm_id == self.cur_arm_id:
            return
        self.cur_arm_id = arm_id
        self.dirty = True
        self.info(i18n.tr("session.switch_arm", i18n.arm_label(arm_id)))

    # ---- 事件 ----

    def event(self, level, text):
        self.events.append((time.time(), level, text))
        self.dirty = True

    def streaming_any(self):
        """有任何一台正在发（或暂停在）控制流 —— 退出确认用。

        **要看两台**：只按当前臂判的话，切到那台没在跑的臂上按 q 就直接退了，
        另一台的 0x16 还挂着。"""
        return any(v.ctrl_live or v.hold for v in self.views.values())

    def info(self, text):
        self.event("info", text)

    def warn(self, text):
        self.event("warn", text)

    def err(self, text):
        self.event("err", text)

    # ---- 读（面板用） ----

    def snapshot(self):
        """面板要的那一帧一致视图 —— 键名是面板的契约，值是 **SI**。

        `fb` 直接取 `arm.snapshot()`：它本来就是"同一帧里换算好"的一致视图，不会再被
        下一帧改掉。夹爪那一格：标定过给**米**，没标定给 None（面板退回显示原始角）。
        """
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
            "n_rx": st["n_rx"],                     # 链路级（两台之和）
            "n_rx_fb": arm.n_feedback,              # **本臂**的 0x80 帧数
            "rx_rate": self._rate(view, now),       # 本臂的 0x80 速率
            "n_tx": st["n_tx"],
            "n_drop": st["n_drop"],
            "n_crc_err": st["n_err_crc8"] + st["n_err_crc16"],
            "n_bad_arm": st["n_bad_arm"],
            "unknown_tids": dict(chan.unknown_tids),
            "arm_id": self.cur_arm_id,
            "arms": [self._arm_line(arm_id, now) for arm_id in sorted(self.views)],
        }

    def _arm_line(self, arm_id, now):
        """状态行下面那一行「两台概览」用的短描述。"""
        view = self.views[arm_id]
        sp = view.arm.snapshot()
        return {"arm_id": arm_id,
                "state": None if sp is None else sp.robot_state,
                # 从**数值码**经 i18n 渲染：`sp.robot_state_name` / `sp.mode_name` 是
                # `board_defs` 预渲染的中文（公开 API，不能动），英文面板要从这里绕过它。
                "state_name": i18n.tr("session.no_feedback") if sp is None
                else i18n.state_name(sp.robot_state),
                "mode_name": "-" if sp is None else i18n.mode_name(sp.mode),
                "lost": None if sp is None else sp.motor_lost_mask,
                "ctrl_live": view.ctrl_live,
                "have_ever_rx": view.have_ever_rx,
                "n_feedback": view.arm.n_feedback,
                "rx_rate": self._rate(view, now)}

    def _rate(self, view, now):
        """本臂 0x80 的到达速率（0.5 s 窗）。SDK 的 stats 是链路级的聚合计数，没有速率，
        而且链路级会把两台加起来 —— 面板上 `0x80 %u` 是本臂的，两个数必须同口径。"""
        dt = now - view._rate_t
        if dt >= 0.5:
            view._rate_hz = (view.arm.n_feedback - view._rate_n) / dt
            view._rate_n = view.arm.n_feedback
            view._rate_t = now
        return view._rate_hz

    def fb(self):
        """当前臂的反馈视图（面板的按键处理用；没有反馈返回 None）。"""
        return self.snapshot()["fb"]

    def gripper_row(self, fb):
        """夹爪那一行要显示的量（面板只管排版，单位换算留在后端）。

        **标定过就给米**（目标/反馈/软限位全按标定那两点映射），**没标定就给原始角**（rad）
        —— 原始角是电机的量，没有"多少毫米"的规格，只能标定出来（见 sdk/README「夹爪标定」）。
        `fb` 可以传 None（还没有反馈帧）。
        """
        view = self.cur
        calib = view.arm.calibration
        if calib is not None:
            lo, hi = gripper_m_limits(calib)
            return {"calibrated": True, "unit": "m",
                    "tgt": calib.opening_from_raw_deg(units.rad_to_deg(view.target[GRIPPER])),
                    "fb": (fb["gripper"] if fb else None), "lo": lo, "hi": hi}
        return {"calibrated": False, "unit": "rad",
                "tgt": view.target[GRIPPER],
                "fb": (fb["gripper_raw_rad"] if fb else None),
                "lo": axis_lo_rad(GRIPPER), "hi": axis_hi_rad(GRIPPER)}

    def state_mode(self):
        snap = self.snapshot()
        fb = snap["fb"]
        if fb is None:
            return None, None, snap
        return fb["state"], fb["mode"], snap

    # ---- 目标角 / 步长 ----

    def seed_from_feedback(self, fb, why):
        view = self.cur
        view.target = list(fb["joint"]) + [fb["gripper_raw_rad"]]
        view.seeded = True
        self.info(i18n.tr("session.seeded", short_label(view.arm_id), why))

    def _seed(self, view, sp, why):
        """从 `sdk` 的快照播种（会话内部的自动重播种走这条，值域和 target 完全一致）。"""
        view.target = list(sp.joint_angles) + [units.deg_to_rad(sp.gripper_raw_deg)]
        view.seeded = True
        self.info(i18n.tr("session.seeded", short_label(view.arm_id), why))

    def jog(self, axis, sign):
        """按键只改 target[]，实际下发由 tick 按周期做。"""
        view = self.cur
        if axis is None:
            return
        sp = view.arm.snapshot()
        if sp is None:
            self.warn(i18n.tr("session.no_feedback_yet"))
            return
        lbl = short_label(view.arm_id)

        if axis == GRIPPER and view.step_in_m:
            # 米档：目标按米加减，再换回原始角（映射由标定那两点决定）
            calib = view.arm.calibration
            cur_m = calib.opening_from_raw_deg(units.rad_to_deg(view.target[axis]))
            want_m = cur_m + sign * view.step
            lo_m, hi_m = gripper_m_limits(calib)
            if want_m < lo_m or want_m > hi_m:
                got_m = min(max(want_m, lo_m), hi_m)
                self.warn(i18n.tr("session.grip_limit_m", lbl, want_m, lo_m, hi_m, got_m))
                view.target[axis] = units.deg_to_rad(calib.raw_deg_from_opening(got_m))
                return
            view.target[axis] = units.deg_to_rad(calib.raw_deg_from_opening(want_m))
            self.dirty = True
            return

        want = view.target[axis] + sign * view.step
        got, clamped = clamp_axis_rad(axis, want)
        view.target[axis] = got
        if clamped:
            self.warn(i18n.tr("session.axis_limit", lbl, i18n.axis_name(axis),
                              units.rad_to_deg(want),
                              units.rad_to_deg(axis_lo_rad(axis)),
                              units.rad_to_deg(axis_hi_rad(axis)),
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

    # ---- 发：命令（预检交给 sdk.Arm，这里只负责把异常翻成人话进事件流） ----

    def _report_send_failure(self, view, what, e):
        """命令发不出去时统一进事件流。发送类是 err，闸门类是 warn（板子也没受理）。"""
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
        """0x02。板子会先比姿态和掉线，不合格**静默丢弃**，所以预检交给 `sdk.Arm`。

        **「再按一次 i 强发」只给姿态这一条闸门**（旧版就是这样的）：其余闸门（态不对、
        有掉线）强发过去也是被板子静默丢弃，可日志会照报"已发 0x02、约 7 s" —— 那就是
        在谎报初始化开始了，正是这个面板要防的坑。所以姿态先自己复核一遍，只有它超了
        才走两次按键的强发通道；别的闸门一律直接由 SDK 报出来，不提供强发。
        """
        view = self.cur
        arm = view.arm
        lbl = short_label(view.arm_id)
        sp = arm.snapshot()
        if sp is None:
            self.warn(i18n.tr("session.init_no_fb", lbl))
            return

        bad = [(board_defs.JOINT_NAMES[i],
                units.rad_to_deg(sp.joint_angles[i] - want))
               for i, want in enumerate(board_defs.RESET_POSE_RAD)
               if abs(sp.joint_angles[i] - want) > self.opts.init_tol]
        if bad:
            if not view.force_init:
                view.force_init = True
                self.warn(i18n.tr(
                    "session.init_pose_bad", lbl, units.rad_to_deg(self.opts.init_tol),
                    i18n.tr("session.pose_dev_sep").join(
                        i18n.tr("session.pose_dev_item", n, d) for n, d in bad),
                    " ".join("%+.4f" % v for v in board_defs.RESET_POSE_RAD)))
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
        """0x18 一键复位。闸门是「正常运行 + 无掉线」（板子**故意不查姿态**）。"""
        view = self.cur
        lbl = short_label(view.arm_id)
        try:
            view.arm.reset()
        except ArError as e:
            self._report_send_failure(view, i18n.tr("session.what.reset"), e)
            return
        self.info(i18n.tr("session.reset_sent", lbl,
                          " ".join("%+.4f" % v for v in board_defs.RESET_POSE_RAD)))
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
        """`--stop-on-exit`：给**两台**补一帧 0x14（失能、松劲）。"""
        done = []
        for arm_id in sorted(self.views):
            try:
                self.views[arm_id].arm.stop(force=True)
                done.append(short_label(arm_id))
            except ArError as e:
                self.err(i18n.tr("session.stop_all_fail", short_label(arm_id), e))
        return done

    # ---- 链路 ----

    def reopen(self):
        """重开链路（r 键）。`--fake` 下就是"假板子重新上电"。

        SDK 的 `Link.open()` 出错时**抛 `LinkError`**（不是 SystemExit），所以这里接住、
        照旧只报事件，不打断面板。顺手重读一遍夹爪标定 —— 标定往往是在另一个进程里做的。
        """
        for view in self.views.values():
            view.handshake_reported = False
            view.ctrl_err_reported = False
            view.prev = None
            view.seeded = False
        self.t0 = time.monotonic()
        try:
            self.link.reopen()
        except ArError as e:
            self.err(i18n.tr("session.reopen_fail", e))
            self.dirty = True
            return False
        for view in self.views.values():
            try:
                view.arm.reload_calibration()
            except ArError:
                pass
        self.info(i18n.tr("session.reopened", self.link.describe() if not self.opts.fake
                          else i18n.tr("session.fake_board")))
        self.dirty = True
        return True

    # ---- 每拍 ----

    def tick(self, now):
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
                        and sp.robot_state == board_defs.STATE_NORMAL
                        and sp.mode == board_defs.MODE_RUN
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
                    and sp.robot_state != board_defs.STATE_INITING:
                view.init_t0 = None
            if view.reset_t0 is not None and sp is not None \
                    and sp.robot_state != board_defs.STATE_RESETTING:
                view.reset_t0 = None
            self._rate(view, now)

    def _send_ctrl(self, view, now):
        """周期 0x16。`force=True` 的理由：上面那道本地门和 `sdk.Arm` 的预检判据**逐条
        一样**（state/mode/age/link_up），100 Hz 下每拍再走一遍预检纯属白跑 —— 但走的是
        `Arm` 的换算和编码，不是自己组帧。"""
        try:
            view.arm.send_ctrl_pos(view.target[:6],
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
        """版本码是**链路级**的（一条链路一台上位机），所以整个会话只报一次。"""
        if self.version_reported:
            return
        self.version_reported = True
        bv = self.link.board_version
        if bv is None:
            self.info(i18n.tr("session.hs_fb_only", protocol.VERSION_CODE))
        elif bv == protocol.VERSION_CODE:
            self.info(i18n.tr("session.hs_ok", bv, protocol.VERSION_STRING))
        else:
            self.err(i18n.tr("session.hs_bad", bv, protocol.VERSION_CODE))

    def _on_feedback(self, view, sp, now):
        """反馈帧 → 事件 / 自动重播种。**策略逐条照搬**原来单臂那一版，只是单位换 SI、
        日志里带臂标签。角度写进日志前换回 deg（日志口径是 deg）。"""
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
                if st == board_defs.STATE_INITING:
                    view.init_t0 = now
                    view.init_told_reset = False
                if st == board_defs.STATE_RESETTING:
                    view.reset_t0 = now
                # 从 comm_lost 回来 / 初始化结束进 normal —— 这两处必须重播种，
                # 否则残留的旧目标会在进 run 的那一帧把臂拽走（板子的 joint_src
                # 自播种只在没有有效控制帧时才跟着反馈走）。
                if st == board_defs.STATE_WAIT_INIT and pst in (
                        board_defs.STATE_INITING, board_defs.STATE_COMM_LOST,
                        board_defs.STATE_CALIBRATING):
                    self._seed(view, sp, i18n.tr("session.why.wait_init"))
                if st == board_defs.STATE_NORMAL and pst == board_defs.STATE_RESETTING:
                    # 复位收尾。**必须重播种**：复位期间臂真的动过，切走之前那个目标
                    # 早过期了（板子的 Ctrl_Invalidate 也把它清了），不重播种的话下一帧
                    # 0x16 会把臂按老目标再拽一次。
                    self._seed(view, sp, i18n.tr("session.why.reset_tail"))
                    bad = [(board_defs.JOINT_NAMES[i],
                            units.rad_to_deg(sp.joint_angles[i] - want))
                           for i, want in enumerate(board_defs.RESET_POSE_RAD)
                           if abs(sp.joint_angles[i] - want) > self.opts.init_tol]
                    if bad:
                        # 板上那个静默失败：临时规划池满时复位起了个空规划、下一拍就
                        # "跑完"，状态机以为做完了但臂没动过。
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
                if pst in (board_defs.STATE_INITING, board_defs.STATE_RESETTING) \
                        and st == board_defs.STATE_NORMAL \
                        and mode == board_defs.MODE_DAMPING:
                    # 初始化/复位收尾都会 run → damping，是**正常收尾**不是异常
                    # （异常那条是掉线把 run 打回 stop，下面那句）。别吓人。
                    good = "info"
                    text += i18n.tr("session.mode_tail_normal")
                if mode == board_defs.MODE_RUN:
                    self._seed(view, sp, i18n.tr("session.why.enter_run"))
                    self.event("info", text)
                elif pmode == board_defs.MODE_RUN:
                    # 最典型的：Motor_Lost_Handle 因为掉线把 arm_mode 打回 stop
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

        if st == board_defs.STATE_INITING and view.init_t0 is not None:
            self.dirty = True      # 面板上要看着秒数走
            if now - view.init_t0 > 4.0 and not view.init_told_reset:
                view.init_told_reset = True
                self.info(i18n.tr("session.initing_note", lbl))


# ============================================================================
# 壳：建会话（面板和 main 用）
# ============================================================================

def build_session(opts):
    """按 opts 建链路 + 会话。**两台臂都建**：一条链路、每台一个 sdk.Arm。"""
    if opts.fake:
        # --fake-lost 的掩码落在 opts.arm_id 那一台上（另一台留着对照）
        link = SimLink(lost_mask=opts.fake_lost, lost_arm=opts.arm_id)
    else:
        link = Link(opts.port, opts.baud)
    return ConsoleSession(link, opts)
