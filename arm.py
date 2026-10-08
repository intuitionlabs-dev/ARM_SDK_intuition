"""`Arm` —— 单台机械臂的门面对象。**这是 SDK 的主入口。**

对外单位统一到 SI：关节角 rad、关节角速度 rad/s、夹爪开口 **m**（要过标定）、温度 ℃。
控制量和反馈量是同一把尺子 —— `send_ctrl_pos(arm.joint_angles)` 这种"照抄现在的位置"是
合法的，不用手算量纲。

## 两台臂怎么用

板子上**只有一条**上位机链路，链路上跑着所有臂的帧（靠帧头里的 arm_id 分）。所以两台臂
必须是**一条 `Link` + 每台一个 `Arm`**：

    link = Link(); link.open()
    a = Arm(0, link)
    b = Arm(1, link)
    a.connect(); b.connect()

不要起两个进程 / 两个 `Link` 抢同一个口子（会互相撕帧，报
`device reports readiness to read but returned no data`，很容易被误读成"臂死了"）。

单台臂可以省掉 Link，`Arm(0)` 会自己建一条（`connect()` 时才打开口子）。

## 闸门预检：为什么会有"没发出去"的异常

板子对所有命令帧都是**静默丢弃** —— 协议里没有 NACK，被拒了和不理你长得一模一样。
所以每个命令方法在发送前先拿最近一帧反馈把板子的闸门条件**复算一遍**，对不上就抛
`GateRejected` 并说明缺哪一条（**帧根本没发出去**）。这是省时间的：被拒的原因通常是
"state 不对，先做 X"，当场告诉你比让你对着线路猜强。

`force=True` 跳过预检（板子那边的闸门仍然生效，只是不再帮你提前挡）。唯一必须用它的场合：
**"遥控器在线"这一条预检查不到** —— 遥控器状态不在 0x80 反馈帧里（见 README「注意事项」），
所以"遥控器不在线"这件事只有板子知道。反过来，如果你确认遥控器是离线的，那就没有预检
挡得住你。

## 保活

**SDK 不自动发心跳。** 板子的判据是"500 ms 内收到过**任意**有效帧"，所以你的周期控制帧
本身就是保活；不在跑控制循环的时候（只看看状态、慢节奏发指令）要显式 `keepalive()`。
注意**板子自己发的 0x80 不算"收到的帧"**，光收不发照样会掉进通信离线。
"""

import time
from dataclasses import dataclass

from . import board_defs, i18n, protocol, units
from .errors import (FeedbackStale, GateRejected, GripperNotCalibrated,
                     LinkError)
from .gripper import GripperCalibStore

# 反馈帧超过这个年龄就不拿它做闸门判断了（板子 250 Hz 常流，100 ms = 丢 25 帧）。
FEEDBACK_STALE_S = 0.1


@dataclass(frozen=True)
class Snapshot:
    """**同一帧**里换算好的一致视图（rad / rad·s⁻¹ / m / ℃）。

    为什么需要它：`arm.joint_angles` 和 `arm.mode` 是**两次独立的读**，中间可能夹进一帧新的
    反馈 —— 那样你拿到的是"上一帧的角度 + 这一帧的模式"。要严格一致就用 `snapshot()`。

    夹爪没标定时 `gripper_opening` 是 None、`gripper_calibrated` 是 False（不抛异常：快照
    是"看一眼"，不该因为你没标定就炸掉）。
    """

    arm_id: int
    taken_at: float          # time.monotonic()
    age: float               # 秒
    joint_angles: tuple      # rad
    joint_velocities: tuple  # rad/s
    joint_angles_deg: tuple  # 原始线上值，留着对照
    joint_velocities_deg_s: tuple
    gripper_opening: float   # m，未标定时 None
    gripper_raw_deg: float
    gripper_calibrated: bool
    temperatures: tuple      # ℃，7 个
    mode: int
    mode_name: str
    robot_state: int
    robot_state_name: str
    motor_lost_mask: int
    motor_inited_mask: int
    lost_names: str
    inited_names: str

    @property
    def motors_online(self):
        """7 台（含夹爪）全在线。0x02 / 0x18 的闸门就是看这个。"""
        return self.motor_lost_mask == 0

    def __str__(self):
        grip = ("%.4f m" % self.gripper_opening) if self.gripper_calibrated \
            else (i18n.tr("arm.grip_uncal", self.gripper_raw_deg))
        return (i18n.tr("arm.snapshot_line",
                        self.arm_id, i18n.state_name(self.robot_state),
                        i18n.mode_name(self.mode),
                        " ".join("%+.4f" % v for v in self.joint_angles), grip,
                        " ".join("%d" % t for t in self.temperatures),
                        i18n.expand_mask(self.motor_lost_mask)))


class Arm:
    """一台臂。所有命令方法都发这一台的帧（帧头 arm_id = `arm_id`）。"""

    def __init__(self, arm_id=protocol.DEFAULT_ARM_ID, link=None, *, calib_store=None):
        if not 0 <= arm_id < board_defs.ARM_NUM:
            raise ValueError(i18n.tr("arm.arm_id_range",
                                     board_defs.ARM_NUM - 1, board_defs.ARM_NUM, arm_id))
        from .link import Link   # 延迟 import：link.py 的 arm() 反过来要用本模块

        self.arm_id = arm_id
        self._own_link = link is None
        self.link = link if link is not None else Link()
        self._chan = self.link.channel(arm_id)
        self._calib_store = calib_store or GripperCalibStore()
        self._calib = None
        self._calib_loaded = False
        self._ack_after = None      # set_zero() 发送前的 0x11 序号

    # ==================== 生命周期 ====================

    def connect(self, timeout=3.0, port=None):
        """打开链路 → 发 0x01 → 等这一台的第一帧反馈。等不到就抛 `LinkError` 带诊断。

        板子在握手通过之前**不发 0x80**，所以"收到反馈帧"就是"握手成了"的判据。
        """
        if not self.link.is_open:
            self.link.open(port)
        before = self._chan.snapshot_feedback()[2]
        self.send_version()
        fb = self.link.wait_feedback(self.arm_id, before, timeout)
        if fb is None:
            raise LinkError(self._connect_failure(timeout))
        return self

    def close(self):
        """关掉自己建的链路（`Arm(0)` 那种）。共享链路时**什么都不做**。

        共享链路要一起关就在 `link.close()` 上关 —— 一台臂把公共口子关掉会把另一台带下水。
        """
        if self._own_link:
            self.link.close()

    def _connect_failure(self, timeout):
        from .link import link_hint

        st = self.link.stats
        if st["board_version"] is None:
            ver = i18n.tr("arm.no_version")
        elif st["board_version"] == protocol.VERSION_CODE:
            ver = i18n.tr("arm.ver_ok", st["board_version"])
        else:
            ver = i18n.tr("arm.ver_mismatch", st["board_version"], protocol.VERSION_CODE)

        lines = [i18n.tr("arm.connect_timeout", timeout,
                         i18n.arm_label(self.arm_id)),
                 i18n.tr("arm.connect_port",
                         self.link.describe() if self.link.port
                         else i18n.tr("arm.no_port")),
                 i18n.tr("arm.connect_link", st["is_open"], st["link_up"])]
        if st["link_err"]:
            lines.append(i18n.tr("arm.link_error", st["link_err"]))
        lines.append(i18n.tr("arm.connect_rx", st["n_rx"], st["n_err_crc8"],
                             st["n_err_crc16"], st["n_drop"]))
        lines.append(i18n.tr("arm.connect_board_ver", ver))
        if st["n_rx"] == 0:
            lines.append(i18n.tr("arm.connect_zero_bytes"))
            lines.append(link_hint())
        elif st["board_version"] is None:
            lines.append(i18n.tr("arm.bytes_no_version"))
        return "\n".join(lines)

    # ==================== 读：反馈 ====================

    @property
    def feedback(self):
        """最近一帧 0x80（`protocol.Feedback`，**线上单位**）。没收到过返回 None。"""
        return self._chan.snapshot_feedback()[0]

    @property
    def feedback_age(self):
        """最近一帧反馈距今秒数。没收到过返回 None。"""
        return self._chan.feedback_age()

    @property
    def feedback_ok(self):
        """手上有足够新的反馈帧（`FEEDBACK_STALE_S` 以内）。"""
        age = self.feedback_age
        return age is not None and age <= FEEDBACK_STALE_S

    @property
    def link_up(self):
        """链路本身还活着（串口没报错）。**它不代表板子在回帧** —— 要看 `feedback_ok`。"""
        return self.link.link_up

    @property
    def n_feedback(self):
        """累计收到多少帧反馈。"""
        return self._chan.snapshot_feedback()[2]

    # ---- 单属性（方便，但**可能跨帧**；要一致用 snapshot()） ----

    @property
    def joint_angles(self):
        """6 个关节角，rad。没收到反馈返回 None。"""
        fb = self.feedback
        return None if fb is None else units.deg_seq_to_rad(fb.joint_angle)

    @property
    def joint_velocities(self):
        """6 个关节角速度，rad/s。没收到反馈返回 None。"""
        fb = self.feedback
        return None if fb is None else units.deg_s_seq_to_rad_s(fb.joint_vel)

    @property
    def temperatures(self):
        """7 台电机温度，℃（索引 0..5 = j1..j6、6 = 夹爪）。

        ⚠ 板子那边温度取不到时会写成 0，所以 **0 ℃ 既可能是"真的 0 度"也可能是"没读到"**，
        协议里区分不了。
        """
        fb = self.feedback
        return None if fb is None else fb.motor_temp

    @property
    def raw_gripper_angle_deg(self):
        """夹爪的**原始电机角**，deg（线上就是这个）。没收到反馈返回 None。"""
        fb = self.feedback
        return None if fb is None else fb.gripper_angle

    @property
    def raw_gripper_angle_rad(self):
        d = self.raw_gripper_angle_deg
        return None if d is None else units.deg_to_rad(d)

    @property
    def gripper_opening(self):
        """夹爪开口距离，**m**。

        没收到过反馈 → None。**没标定 → 抛 `GripperNotCalibrated`**（不返回猜的数，
        因为没有任何真值可猜 —— 见 sdk/README.md 的「夹爪标定」）。
        要"看一眼有没有数"、不想被异常打断，用 `snapshot()`。
        """
        fb = self.feedback
        if fb is None:
            return None
        return self.require_calibration().opening_from_raw_deg(fb.gripper_angle)

    @property
    def mode(self):
        return self.feedback.mode if self.feedback else None

    @property
    def mode_name(self):
        m = self.mode
        return None if m is None else board_defs.mode_name(m)

    @property
    def robot_state(self):
        return self.feedback.robot_state if self.feedback else None

    @property
    def robot_state_name(self):
        s = self.robot_state
        return None if s is None else board_defs.state_name(s)

    @property
    def motor_lost_mask(self):
        return self.feedback.motor_lost_mask if self.feedback else None

    @property
    def motor_inited_mask(self):
        return self.feedback.motor_inited_mask if self.feedback else None

    @property
    def lost_names(self):
        m = self.motor_lost_mask
        return None if m is None else board_defs.expand_mask(m)

    @property
    def inited_names(self):
        m = self.motor_inited_mask
        return None if m is None else board_defs.expand_mask(m)

    def snapshot(self):
        """同一帧里换算好的一致视图。没收到过反馈返回 None。"""
        fb, t, _ = self._chan.snapshot_feedback()
        if fb is None:
            return None
        calib = self.calibration
        opening = calib.opening_from_raw_deg(fb.gripper_angle) if calib else None
        return Snapshot(
            arm_id=self.arm_id,
            taken_at=t,
            age=time.monotonic() - t,
            joint_angles=units.deg_seq_to_rad(fb.joint_angle),
            joint_velocities=units.deg_s_seq_to_rad_s(fb.joint_vel),
            joint_angles_deg=fb.joint_angle,
            joint_velocities_deg_s=fb.joint_vel,
            gripper_opening=opening,
            gripper_raw_deg=fb.gripper_angle,
            gripper_calibrated=calib is not None,
            temperatures=fb.motor_temp,
            mode=fb.mode,
            mode_name=board_defs.mode_name(fb.mode),
            robot_state=fb.robot_state,
            robot_state_name=board_defs.state_name(fb.robot_state),
            motor_lost_mask=fb.motor_lost_mask,
            motor_inited_mask=fb.motor_inited_mask,
            lost_names=board_defs.expand_mask(fb.motor_lost_mask),
            inited_names=board_defs.expand_mask(fb.motor_inited_mask),
        )

    # ==================== 读：夹爪标定 ====================

    @property
    def calibration(self):
        """这一台的夹爪标定，**没标过返回 None**（不抛）。第一次访问才读文件。"""
        if not self._calib_loaded:
            self._calib = self._calib_store.get(self.arm_id)
            self._calib_loaded = True
        return self._calib

    def require_calibration(self):
        """拿标定，没标过就抛 `GripperNotCalibrated`（消息里带怎么标）。"""
        calib = self.calibration
        if calib is None:
            raise GripperNotCalibrated(
                self._calib_store.missing_message(self.arm_id, self._calib_hint()))
        return calib

    def reload_calibration(self):
        """重新读一遍标定文件（刚在另一个进程里标完，不用重启）。"""
        self._calib_loaded = False
        self._calib = None
        return self.calibration

    def _calib_hint(self):
        fb = self.feedback
        if fb is None:
            return i18n.tr("arm.no_feedback_yet")
        if not fb.motor_inited_mask >> board_defs.GRIPPER & 1:
            return i18n.tr("arm.grip_not_homed")
        return i18n.tr("arm.grip_inited_hint")

    # ==================== 发：不用看状态的 ====================

    def send_version(self):
        """发版本匹配包（0x01）。也是**保活**：见 `keepalive()`。"""
        return self._send(protocol.TID_VERSION, protocol.encode_version())

    # 保活就是"500 ms 内发任意有效帧"，版本包最小最无害 —— 板子对已经握过手的手
    # **幂等地再回一个版本包，状态不动**（见 robot_state.c 的 HandleVersion），所以拿它
    # 当心跳不会打扰状态机。
    keepalive = send_version

    # ==================== 发：命令 ====================

    def init_arm(self, *, force=False):
        """0x02 初始化：一键复位 → 一键夹爪找零 → 正常运行。

        闸门（板子侧，`robot_state.c` 的 `HandleArmInit`）：
          ① state == 待初始化(2)，**或** state == 正常运行(4) 且 mode ∈ {stop, damping}；
          ② 7 台电机全部在线；
          ③ 6 个关节（**不含夹爪**）与复位位姿偏差 ≤ 20°；
          ④ 遥控器离线（**预检查不到，见类注释**）。
        """
        if not force:
            fb = self._fresh_feedback("init_arm")
            why = []
            from_soft = (fb.robot_state == board_defs.STATE_NORMAL
                         and fb.mode in (board_defs.MODE_STOP, board_defs.MODE_DAMPING))
            if not (fb.robot_state == board_defs.STATE_WAIT_INIT or from_soft):
                why.append(i18n.tr("arm.gate_init_state",
                                   i18n.state_name(fb.robot_state),
                                   i18n.mode_name(fb.mode),
                                   i18n.state_name(board_defs.STATE_WAIT_INIT),
                                   i18n.state_name(board_defs.STATE_NORMAL)))
            self._check_motors_online(fb, why)
            self._check_pose(fb, why)
            self._raise_if(why, "init_arm")
        return self._send(protocol.TID_ARM_INIT)

    def set_zero(self, *, force=False):
        """0x10 触发一轮设零。**会覆盖电机里的原零点**（把"当前位姿"定义成零点），
        所以触发前先把臂摆到基准位姿。

        闸门：state == 待初始化(2) + 遥控器离线。
        没有即时反馈帧 —— 结果走单独的一帧 0x11（跑完才发，几秒），拿它用
        `wait_setzero_ack()`。**0x10 只管 j1~j6**，夹爪不在默认参与集里（它的零点是行程中点，
        只能由找零给出）。注意这会挪动夹爪所在总线上的东西，标定过的话大概率要重标。
        """
        if not force:
            fb = self._fresh_feedback("set_zero")
            why = []
            if fb.robot_state != board_defs.STATE_WAIT_INIT:
                why.append(i18n.tr("arm.gate_setzero_state",
                                   i18n.state_name(fb.robot_state),
                                   i18n.state_name(board_defs.STATE_WAIT_INIT)))
            self._raise_if(why, "set_zero")
        self._ack_after = self._chan.snapshot_ack()[2]
        return self._send(protocol.TID_MOTOR_SET_ZERO)

    def start(self, *, force=False):
        """0x12 进 run 档（位控）。控制帧只在 run 档被接受。闸门：state == 正常运行(4)。

        切档会**作废上一帧控制指令**（板子侧的 `Ctrl_Invalidate`，防止把切档前的旧目标重新
        灌进位置环把你猛拽一下），所以切过来之后要重新发 0x16/0x17。
        """
        return self._mode_cmd(protocol.TID_ARM_START, "start", force)

    def gravity(self, *, force=False):
        """0x13 进重力补偿档。闸门：state == 正常运行(4)。"""
        return self._mode_cmd(protocol.TID_ARM_GRAVITY, "gravity", force)

    def stop(self, *, force=False):
        """0x14 失能电机（**松劲**，不是"停在这不动"）。闸门：state == 正常运行(4)。"""
        return self._mode_cmd(protocol.TID_ARM_STOP, "stop", force)

    def damping(self, *, force=False):
        """0x15 进阻尼档。闸门：state == 正常运行(4)。"""
        return self._mode_cmd(protocol.TID_ARM_DAMPING, "damping", force)

    def _mode_cmd(self, tid, name, force):
        if not force:
            fb = self._fresh_feedback(name)
            why = []
            if fb.robot_state != board_defs.STATE_NORMAL:
                why.append(i18n.tr("arm.gate_mode_state",
                                   i18n.state_name(fb.robot_state),
                                   i18n.state_name(board_defs.STATE_NORMAL)))
            self._raise_if(why, name)
        return self._send(tid)

    def send_ctrl_pos(self, joint_rad, gripper_m=None, *, gripper_raw_deg=None,
                      force=False):
        """0x16 位置指令。`joint_rad` 是 6 个关节角（**rad**）。

        夹爪目标三选一：
          - `gripper_m=0.03`        按标定换算成原始角（要先标定）
          - `gripper_raw_deg=-141.5` 直接给原始电机角（不用标定）
          - 都不给                  **保持当前**（拿最近一帧反馈里的原始角）

        闸门：state == 正常运行(4) **且** mode == run(1) + 遥控器离线。

        ⚠ **软限位不在这里夹**（`board_defs.clamp_axis()` 给你自己夹）。板子那边确实会
        `fclip`，但**只对 `motor_inited` 为真的电机生效** —— 没初始化过的电机不受保护，
        而且它是静默夹的，你从期望角上看不出来。
        """
        return self._ctrl(protocol.TID_ARM_CTRL_POS, joint_rad, None, gripper_m,
                          gripper_raw_deg, force)

    def send_ctrl_pos_vel(self, joint_rad, joint_vel_rad_s, gripper_m=None, *,
                          gripper_raw_deg=None, force=False):
        """0x17 位置 + 速度前馈。角速度单位 **rad/s**（线上是 deg/s）。

        速度前馈的起点建议对齐当前反馈速度（`arm.joint_velocities`），否则从 0 起规划会在
        起步处顿一下。其余同 `send_ctrl_pos()`。
        """
        return self._ctrl(protocol.TID_ARM_CTRL_POS_VEL, joint_rad, joint_vel_rad_s,
                          gripper_m, gripper_raw_deg, force)

    def _ctrl(self, tid, joint_rad, joint_vel_rad_s, gripper_m, gripper_raw_deg, force):
        name = "send_ctrl_pos" if tid == protocol.TID_ARM_CTRL_POS else "send_ctrl_pos_vel"
        angles = units.check_joint_seq(joint_rad, i18n.tr("arm.joint_angle"))
        vels = (units.check_joint_seq(joint_vel_rad_s, i18n.tr("arm.joint_vel"))
                if joint_vel_rad_s is not None else None)

        if not force:
            fb = self._fresh_feedback(name)
            why = []
            if fb.robot_state != board_defs.STATE_NORMAL:
                why.append(i18n.tr("arm.gate_ctrl_state",
                                   i18n.state_name(fb.robot_state),
                                   i18n.state_name(board_defs.STATE_NORMAL)))
            elif fb.mode != board_defs.MODE_RUN:
                why.append(i18n.tr("arm.gate_not_run", i18n.mode_name(fb.mode)))
            self._raise_if(why, name)

        angles_deg = units.rad_seq_to_deg(angles)
        grip_deg = self._gripper_deg_for_send(gripper_m, gripper_raw_deg)
        if vels is None:
            return self._send(tid, protocol.encode_ctrl_pos(list(angles_deg) + [grip_deg]))
        vels_deg = units.rad_s_seq_to_deg_s(vels)
        return self._send(tid, protocol.encode_ctrl_pos_vel(list(angles_deg), list(vels_deg),
                                                            grip_deg))

    def reset(self, *, force=False):
        """0x18 一键复位：走一段 `ARM_AUTO_RESET_TIME_S` 的轨迹回复位位姿，**不做夹爪找零**，
        跑完停在阻尼。

        闸门：state == 正常运行(4) + 7 台全在线（**故意不查姿态** —— 臂歪着才要收）。

        没有专用反馈帧：受理与否看反馈里的 robot_state（4 → 7 受理了、7 → 4 跑完了）。
        """
        if not force:
            fb = self._fresh_feedback("reset")
            why = []
            if fb.robot_state != board_defs.STATE_NORMAL:
                why.append(i18n.tr("arm.gate_reset_state",
                                   i18n.state_name(fb.robot_state),
                                   i18n.state_name(board_defs.STATE_NORMAL)))
            self._check_motors_online(fb, why)
            self._raise_if(why, "reset")
        return self._send(protocol.TID_ARM_RESET)

    def send_raw(self, tid, payload=b""):
        """**总逃逸口**：任意 type_id + 任意 payload，不做任何预检、不做任何换算。

        给"板子加了新包但 SDK 还没跟上"以及"我就是想试试"用。注意它绕过所有保护：
        单位是你自己的事，闸门是板子的事。
        """
        if not isinstance(payload, (bytes, bytearray)):
            raise TypeError(i18n.tr("arm.payload_type", type(payload).__name__))
        return self._send(tid, bytes(payload))

    # ==================== 等 ====================

    @property
    def last_setzero_ack(self):
        """最近一帧 0x11（`protocol.SetZeroAck`）。没收到过返回 None。"""
        return self._chan.snapshot_ack()[0]

    def wait_setzero_ack(self, timeout=25.0, *, after=None):
        """等一轮设零的结果（0x11）。超时抛 `TimeoutError`。

        `after=None` 时用 `set_zero()` 记下的那个序号（就是"这一轮"）；没调过 `set_zero()`
        就用当前序号（等下一帧来的）。

        一轮设零实测几秒（两臂各 3 台 ENCOS + 3 台达妙，最坏十几秒），所以默认给 25 s。
        注意 `result=1(未确认)` **不是故障**：它表示"帧发了、位置本来就在容差内"，无法证明
        零点写进去了（臂本来就停在零位时必然是它）。
        """
        ch = self._chan
        if after is None:
            after = self._ack_after
        if after is None:
            after = ch.snapshot_ack()[2]
        deadline = time.monotonic() + timeout
        while True:
            ack, _, seq = ch.snapshot_ack()
            if ack is not None and seq > after:
                return ack
            if time.monotonic() >= deadline:
                st = self.robot_state
                # 没有反馈帧时和以前一样打印 "None"（别为一个边角路径改动字面输出）
                raise TimeoutError(i18n.tr("arm.setzero_timeout", timeout,
                                           i18n.state_name(st) if st is not None else None))
            time.sleep(0.02)

    def wait_for(self, pred, timeout=5.0, poll=0.02):
        """轮询等 `pred(arm)` 为真（比如 `lambda a: a.robot_state == 4`）。

        真了返回 True，超时返回 False。**不抛** —— 它是"等一个状态"，不是一个必须成功的
        动作，要不要失败由调用方决定。
        """
        deadline = time.monotonic() + timeout
        while True:
            if pred(self):
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(poll)

    def wait_feedback(self, timeout=1.0):
        """等一帧**新的**反馈（比手上这帧新）。超时返回 None。"""
        before = self._chan.snapshot_feedback()[2]
        return self.link.wait_feedback(self.arm_id, before, timeout)

    # ==================== 内部 ====================

    def _send(self, tid, payload=b""):
        return self.link.send(tid, payload, self.arm_id)

    def _fresh_feedback(self, what):
        """拿闸门判断用的反馈帧，太旧就抛 `FeedbackStale`。"""
        fb = self.feedback
        if fb is None:
            raise FeedbackStale(
                i18n.tr("arm.stale_no_fb", what, i18n.arm_label(self.arm_id)))
        age = self.feedback_age
        if age > FEEDBACK_STALE_S:
            raise FeedbackStale(
                i18n.tr("arm.stale_old", what, age * 1000.0, FEEDBACK_STALE_S * 1000.0,
                        self.link.link_up))
        return fb

    def _raise_if(self, why, what):
        if why:
            raise GateRejected(
                i18n.tr("arm.gate_rejected", what, "\n  - ".join(why),
                        i18n.state_name(self.robot_state), self.robot_state,
                        i18n.mode_name(self.mode), self.mode,
                        i18n.expand_mask(self.motor_lost_mask)))

    def _check_motors_online(self, fb, why):
        if fb.motor_lost_mask:
            why.append(i18n.tr("arm.motors_lost",
                               i18n.expand_mask(fb.motor_lost_mask)))

    def _check_pose(self, fb, why):
        dev = board_defs.pose_deviation_deg(fb.joint_angle)
        if dev > board_defs.ROBOT_INIT_POSE_TOL_DEG:
            worst = max(range(6), key=lambda i: abs(fb.joint_angle[i]
                                                    - board_defs.RESET_POSE_DEG[i]))
            why.append(i18n.tr("arm.pose_dev", dev, board_defs.ROBOT_INIT_POSE_TOL_DEG,
                               i18n.axis_name(worst), fb.joint_angle[worst],
                               board_defs.RESET_POSE_DEG[worst]))

    def _gripper_deg_for_send(self, gripper_m, gripper_raw_deg):
        if gripper_m is not None and gripper_raw_deg is not None:
            raise ValueError(i18n.tr("arm.grip_two_targets"))
        if gripper_raw_deg is not None:
            return float(gripper_raw_deg)
        if gripper_m is not None:
            return self.require_calibration().raw_deg_from_opening(gripper_m)
        fb = self.feedback
        if fb is None:
            raise FeedbackStale(i18n.tr("arm.grip_no_target"))
        return fb.gripper_angle

    # ==================== 杂项 ====================

    def describe(self):
        """日志用的一行。没收到反馈也出得来。"""
        s = self.snapshot()
        head = "%s %s" % (i18n.arm_label(self.arm_id), self.link.describe())
        if s is None:
            return i18n.tr("arm.describe_no_fb", head, self.link.link_up)
        return i18n.tr("arm.desc_line", head, s)

    def __repr__(self):
        st = self.robot_state
        return "<Arm %d %s %s>" % (self.arm_id,
                                   ("state=%s" % i18n.state_name(st)) if st is not None
                                   else i18n.tr("arm.no_feedback"),
                                   (self.link.port or i18n.tr("arm.no_port_open")))
