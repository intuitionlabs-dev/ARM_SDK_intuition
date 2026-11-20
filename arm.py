"""单臂控制接口：命令预检、单位换算和反馈读取。

双臂共用一个 Link，每台臂使用独立的 Arm。关节使用 rad、rad/s 和 rad/s²，
夹爪开口使用 m，温度使用 ℃。需要同一帧的数据时使用 snapshot()。

命令预检失败抛 GateRejected；force=True 仅跳过上位机预检，固件条件仍生效。
遥控器状态不在反馈中，须由调用方确认遥控器离线。
每台臂须在 3 s 内发送有效命令或 keepalive()；SDK 不自动保活。
"""

import time
from dataclasses import dataclass

from . import i18n, protocol, units
from .board_defs import arx_x5 as board_defs
from .errors import (FeedbackStale, GateRejected, GripperNotCalibrated,
                     LinkError)
from .gripper import GripperCalibStore

# 反馈超过 100 ms 时不再用于命令预检。
FEEDBACK_STALE_S = 0.1


@dataclass(frozen=True)
class Snapshot:
    """同一帧的反馈快照，单位为 rad、rad/s、m 和 ℃。

    夹爪未标定时，gripper_opening 为 None，gripper_calibrated 为 False。
    """

    arm_id: int
    taken_at: float          # time.monotonic()
    age: float               # 秒
    joint_angles: tuple      # rad
    joint_velocities: tuple  # rad/s
    joint_angles_deg: tuple  # 协议角度，便于对照
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
        """六个关节和夹爪电机均在线。"""
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
    """通过 arm_id 指定一台机械臂。"""

    def __init__(self, arm_id=protocol.DEFAULT_ARM_ID, link=None, *, calib_store=None):
        from .link import Link

        self.arm_id = arm_id
        self._own_link = link is None
        self.link = link if link is not None else Link()
        self.board_defs = getattr(self.link, "board_defs", board_defs)
        if not 0 <= arm_id < self.board_defs.ARM_NUM:
            raise ValueError(i18n.tr("arm.arm_id_range",
                                     self.board_defs.ARM_NUM - 1, self.board_defs.ARM_NUM, arm_id))
        self._chan = self.link.channel(arm_id)
        self._calib_store = (calib_store if calib_store is not None else
                             getattr(self.link, "gripper_calib_store", None) or GripperCalibStore())
        self._calib = None
        self._calib_loaded = False
        self._ack_after = None      # set_zero() 发送前的 0x11 序号

    # 生命周期

    def connect(self, timeout=3.0, port=None):
        """打开链路、发送版本帧并等待本臂反馈；超时抛 LinkError。

        收到 0x80 反馈表示握手成功；仅收到 0x01 版本帧不足以确认握手。
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
        """关闭自行创建的链路；共享链路由调用方关闭。"""
        if self._own_link:
            self.link.close()

    def _connect_failure(self, timeout):
        from .link import link_hint

        st = self.link.stats
        if st["board_version"] is None:
            ver = i18n.tr("arm.no_version")
        elif st["board_version"] == self.board_defs.VERSION_CODE:
            ver = i18n.tr("arm.ver_ok", st["board_version"])
        else:
            ver = i18n.tr("arm.ver_mismatch", st["board_version"], self.board_defs.VERSION_CODE)

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

    # 读：反馈

    @property
    def feedback(self):
        """最近一帧 Feedback，使用协议单位；尚无反馈时返回 None。"""
        return self._chan.snapshot_feedback()[0]

    @property
    def feedback_age(self):
        """反馈年龄，单位 s；尚无反馈时返回 None。"""
        return self._chan.feedback_age()

    @property
    def feedback_ok(self):
        """是否有 FEEDBACK_STALE_S 以内的反馈。"""
        age = self.feedback_age
        return age is not None and age <= FEEDBACK_STALE_S

    @property
    def link_up(self):
        """串口链路是否正常；反馈是否有效应检查 feedback_ok。"""
        return self.link.link_up

    @property
    def n_feedback(self):
        """累计反馈帧数。"""
        return self._chan.snapshot_feedback()[2]

    # 单属性读取可能跨帧；一致视图使用 snapshot()。

    @property
    def joint_angles(self):
        """六个关节角，单位 rad；尚无反馈时返回 None。"""
        fb = self.feedback
        return None if fb is None else units.deg_seq_to_rad(fb.joint_angle)

    @property
    def joint_velocities(self):
        """六个关节角速度，单位 rad/s；尚无反馈时返回 None。"""
        fb = self.feedback
        return None if fb is None else units.deg_s_seq_to_rad_s(fb.joint_vel)

    @property
    def temperatures(self):
        """七个电机温度，单位 ℃，最后一项为夹爪。0 也可能表示未读到温度。"""
        fb = self.feedback
        return None if fb is None else fb.motor_temp

    @property
    def raw_gripper_angle_deg(self):
        """夹爪原始电机角，单位 deg；尚无反馈时返回 None。"""
        fb = self.feedback
        return None if fb is None else fb.gripper_angle

    @property
    def raw_gripper_angle_rad(self):
        d = self.raw_gripper_angle_deg
        return None if d is None else units.deg_to_rad(d)

    @property
    def gripper_opening(self):
        """夹爪开口，单位 m。无反馈返回 None；未标定抛 GripperNotCalibrated。"""
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
        return None if m is None else self.board_defs.mode_name(m)

    @property
    def robot_state(self):
        return self.feedback.robot_state if self.feedback else None

    @property
    def robot_state_name(self):
        s = self.robot_state
        return None if s is None else self.board_defs.state_name(s)

    @property
    def motor_lost_mask(self):
        return self.feedback.motor_lost_mask if self.feedback else None

    @property
    def motor_inited_mask(self):
        return self.feedback.motor_inited_mask if self.feedback else None

    @property
    def lost_names(self):
        m = self.motor_lost_mask
        return None if m is None else self.board_defs.expand_mask(m)

    @property
    def inited_names(self):
        m = self.motor_inited_mask
        return None if m is None else self.board_defs.expand_mask(m)

    def snapshot(self):
        """返回同一帧的反馈快照；尚无反馈时返回 None。"""
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
            mode_name=self.board_defs.mode_name(fb.mode),
            robot_state=fb.robot_state,
            robot_state_name=self.board_defs.state_name(fb.robot_state),
            motor_lost_mask=fb.motor_lost_mask,
            motor_inited_mask=fb.motor_inited_mask,
            lost_names=self.board_defs.expand_mask(fb.motor_lost_mask),
            inited_names=self.board_defs.expand_mask(fb.motor_inited_mask),
        )

    # 读：夹爪标定

    @property
    def calibration(self):
        """首次访问时加载夹爪换算；未标定返回 None。"""
        if not self._calib_loaded:
            self._calib = self._calib_store.get(self.arm_id)
            self._calib_loaded = True
        return self._calib

    def require_calibration(self):
        """返回夹爪换算；未标定抛 GripperNotCalibrated。"""
        calib = self.calibration
        if calib is None:
            raise GripperNotCalibrated(
                self._calib_store.missing_message(self.arm_id, self._calib_hint()))
        return calib

    def reload_calibration(self):
        """重新加载夹爪换算，使外部更新的标定生效。"""
        self._calib_loaded = False
        self._calib = None
        return self.calibration

    def _calib_hint(self):
        fb = self.feedback
        if fb is None:
            return i18n.tr("arm.no_feedback_yet")
        if not fb.motor_inited_mask >> self.board_defs.GRIPPER & 1:
            return i18n.tr("arm.grip_not_homed")
        return i18n.tr("arm.grip_inited_hint")

    # 无需状态预检的命令

    def send_version(self):
        """发送 0x01 版本帧，也用于保活。"""
        return self._send(protocol.TID_VERSION,
                          protocol.encode_version(self.board_defs.VERSION_CODE))

    # 版本帧可重复发送且不改变状态，用作每臂保活。
    keepalive = send_version

    # 发：命令

    def init_arm(self, *, force=False):
        """发送 0x02 初始化，完成后进入正常运行 / 阻尼。

        要求待初始化，或正常运行且处于 STOP / 阻尼；七台电机在线，
        六轴偏离复位角不超过型号容差。固件还要求遥控器离线。
        """
        if not force:
            fb = self._fresh_feedback("init_arm")
            why = []
            from_soft = (fb.robot_state == self.board_defs.STATE_NORMAL
                         and fb.mode in (self.board_defs.MODE_STOP, self.board_defs.MODE_DAMPING))
            if not (fb.robot_state == self.board_defs.STATE_WAIT_INIT or from_soft):
                why.append(i18n.tr("arm.gate_init_state",
                                   i18n.state_name(fb.robot_state),
                                   i18n.mode_name(fb.mode),
                                   i18n.state_name(self.board_defs.STATE_WAIT_INIT),
                                   i18n.state_name(self.board_defs.STATE_NORMAL)))
            self._check_motors_online(fb, why)
            self._check_pose(fb, why)
            self._raise_if(why, "init_arm")
        return self._send(protocol.TID_ARM_INIT)

    def set_zero(self, *, force=False):
        """发送 0x10，将六轴当前位姿写为电机零点；须先摆到基准位姿。

        要求待初始化且遥控器离线。结果通过 wait_setzero_ack() 读取。
        默认不修改夹爪零点；夹爪零点另有变化时须重新标定。
        """
        if not force:
            fb = self._fresh_feedback("set_zero")
            why = []
            if fb.robot_state != self.board_defs.STATE_WAIT_INIT:
                why.append(i18n.tr("arm.gate_setzero_state",
                                   i18n.state_name(fb.robot_state),
                                   i18n.state_name(self.board_defs.STATE_WAIT_INIT)))
            self._raise_if(why, "set_zero")
        self._ack_after = self._chan.snapshot_ack()[2]
        return self._send(protocol.TID_MOTOR_SET_ZERO)

    def start(self, *, force=False):
        """发送 0x12 进入 RUN，要求正常运行。反馈确认后须重新发送控制目标。"""
        return self._mode_cmd(protocol.TID_ARM_START, "start", force)

    def gravity(self, *, force=False):
        """发送 0x13 进入重力补偿，要求正常运行。"""
        return self._mode_cmd(protocol.TID_ARM_GRAVITY, "gravity", force)

    def stop(self, *, force=False):
        """发送 0x14 软停机，要求正常运行；不保持位置或提供重力补偿。"""
        return self._mode_cmd(protocol.TID_ARM_STOP, "stop", force)

    def damping(self, *, force=False):
        """发送 0x15 进入阻尼，要求正常运行。"""
        return self._mode_cmd(protocol.TID_ARM_DAMPING, "damping", force)

    def _mode_cmd(self, tid, name, force):
        if not force:
            fb = self._fresh_feedback(name)
            why = []
            if fb.robot_state != self.board_defs.STATE_NORMAL:
                why.append(i18n.tr("arm.gate_mode_state",
                                   i18n.state_name(fb.robot_state),
                                   i18n.state_name(self.board_defs.STATE_NORMAL)))
            self._raise_if(why, name)
        return self._send(tid)

    def send_ctrl_pos(self, joint_rad, gripper_m=None, *, gripper_torque_limit_nm, gripper_raw_deg=None,
                      joint_vel_rad_s=None, joint_acc_rad_s2=None, force=False):
        """发送 0x16 轨迹目标，要求正常运行 / RUN 且遥控器离线。

        关节角使用 rad。可选速度和加速度各含六项，单位为 rad/s 和 rad/s²，默认全零。
        夹爪目标可用 gripper_m（经标定）或 gripper_raw_deg，均省略时保持反馈角。
        gripper_torque_limit_nm 为有限非负数，单位 N·m；零表示请求零夹爪力矩。
        固件按型号限制已初始化电机的角度。
        """
        name = "send_ctrl_pos"
        angles = units.check_joint_seq(joint_rad, i18n.tr("arm.joint_angle"))
        if not force:
            fb = self._fresh_feedback(name)
            why = []
            if fb.robot_state != self.board_defs.STATE_NORMAL:
                why.append(i18n.tr("arm.gate_ctrl_state",
                                   i18n.state_name(fb.robot_state),
                                   i18n.state_name(self.board_defs.STATE_NORMAL)))
            elif fb.mode != self.board_defs.MODE_RUN:
                why.append(i18n.tr("arm.gate_not_run", i18n.mode_name(fb.mode)))
            self._raise_if(why, name)

        angles_deg = units.rad_seq_to_deg(angles)
        vel_deg_s = None
        acc_deg_s2 = None
        if joint_vel_rad_s is not None:
            vel_deg_s = units.rad_s_seq_to_deg_s(units.check_joint_seq(joint_vel_rad_s, "关节速度"))
        if joint_acc_rad_s2 is not None:
            acc_deg_s2 = units.rad_seq_to_deg(units.check_joint_seq(joint_acc_rad_s2, "关节加速度"))
        grip_deg = self._gripper_deg_for_send(gripper_m, gripper_raw_deg)
        return self._send(protocol.TID_ARM_CTRL_POS,
                          protocol.encode_ctrl_pos((*angles_deg, grip_deg), gripper_torque_limit_nm,
                                                   joint_vel_deg_s=vel_deg_s, joint_acc_deg_s2=acc_deg_s2))

    def reset(self, *, force=False):
        """发送 0x18 回复位，要求正常运行且七台电机在线。

        约 2 s 后进入阻尼，不修改零点或执行夹爪找零。状态 4→7→4 表示开始和完成。
        """
        if not force:
            fb = self._fresh_feedback("reset")
            why = []
            if fb.robot_state != self.board_defs.STATE_NORMAL:
                why.append(i18n.tr("arm.gate_reset_state",
                                   i18n.state_name(fb.robot_state),
                                   i18n.state_name(self.board_defs.STATE_NORMAL)))
            self._check_motors_online(fb, why)
            self._raise_if(why, "reset")
        return self._send(protocol.TID_ARM_RESET)

    def send_raw(self, tid, payload=b""):
        """透传 type_id 和 payload，不做预检或单位换算；固件仍检查执行条件。"""
        if not isinstance(payload, (bytes, bytearray)):
            raise TypeError(i18n.tr("arm.payload_type", type(payload).__name__))
        return self._send(tid, bytes(payload))

    # 等

    @property
    def last_setzero_ack(self):
        """最近一帧 SetZeroAck；尚无回复时返回 None。"""
        return self._chan.snapshot_ack()[0]

    def wait_setzero_ack(self, timeout=25.0, *, after=None):
        """等待本轮设零回复，超时抛 TimeoutError。

        after 默认取 set_zero() 前的序号；未调用过时等待下一帧。result=1 表示写零未确认。
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
                # 尚无反馈时显示 None。
                raise TimeoutError(i18n.tr("arm.setzero_timeout", timeout,
                                           i18n.state_name(st) if st is not None else None))
            time.sleep(0.02)

    def wait_for(self, pred, timeout=5.0, poll=0.02):
        """轮询 pred(arm)；满足返回 True，超时返回 False。等待期间不自动保活。"""
        deadline = time.monotonic() + timeout
        while True:
            if pred(self):
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(poll)

    def wait_feedback(self, timeout=1.0):
        """等待下一帧反馈；超时返回 None。"""
        before = self._chan.snapshot_feedback()[2]
        return self.link.wait_feedback(self.arm_id, before, timeout)

    # 内部

    def _send(self, tid, payload=b""):
        return self.link.send(tid, payload, self.arm_id)

    def _fresh_feedback(self, what):
        """返回用于预检的反馈；缺失或过期时抛 FeedbackStale。"""
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
        dev = max(abs(a-b) for a, b in zip(fb.joint_angle, self.board_defs.RESET_POSE_DEG))
        if dev > self.board_defs.ROBOT_INIT_POSE_TOL_DEG:
            worst = max(range(6), key=lambda i: abs(fb.joint_angle[i]
                                                    - self.board_defs.RESET_POSE_DEG[i]))
            why.append(i18n.tr("arm.pose_dev", dev, self.board_defs.ROBOT_INIT_POSE_TOL_DEG,
                               i18n.axis_name(worst), fb.joint_angle[worst],
                               self.board_defs.RESET_POSE_DEG[worst]))

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

    # 杂项

    def describe(self):
        """返回单行机械臂状态，支持尚无反馈的情况。"""
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
