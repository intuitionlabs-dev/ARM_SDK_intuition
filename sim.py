"""H7 协议模拟器，供 TUI --fake 和离线测试使用，不访问硬件。

复用 Link 的解析与分流逻辑，模拟握手、状态转换、双臂独立保活和电机掉线。
反馈使用协议编码器生成；实际电机行为和时序仍需硬件验证。
"""

import threading
import time

from . import protocol
from .board_defs import common as board_defs
from .errors import NotConnected
from .link import Link

# 模拟时限对应固件 robot_state.c / arm_auto.c。
POWERUP_S = 0.6           # 上电 → 离线等待
OFFLINE_WAIT_S = 0.8      # 模拟等待电机上线的时长
INIT_S = 3.0              # 0x02 初始化
CALIB_S = 1.2             # 0x10 设零
RESET_S = 2.0             # 0x18：ARM_AUTO_RESET_TIME_S
COMM_TIMEOUT_S = 3.0      # 每臂有效命令超时
LINK_STATUS_S = 0.1       # ROBOT_LINK_STATUS_PERIOD_MS：0x81 的周期
INIT_GRIPPER_CLEAR_S = 1.5  # 初始化末段清除夹爪初始化标志
FAKE_LOST_PERIOD_S = 5.0  # --fake-lost 的翻转周期

_EMPTY_MODE_TID_TO_MODE = {
    protocol.TID_ARM_START: board_defs.MODE_RUN,
    protocol.TID_ARM_GRAVITY: board_defs.MODE_GRAVITY,
    protocol.TID_ARM_STOP: board_defs.MODE_STOP,
    protocol.TID_ARM_DAMPING: board_defs.MODE_DAMPING,
}


class _Board:
    """单臂模拟状态，角度使用协议单位 deg。"""

    def __init__(self, arm_id):
        self.arm_id = arm_id
        self.version_ok = False        # 是否已通过版本握手
        self.state = board_defs.STATE_WAIT_HANDSHAKE
        self.mode = board_defs.MODE_STOP
        self.lost = 0x00
        self.inited = 0x7F
        self.joint = [0.0] * 6         # deg
        self.gripper = 0.0             # 夹爪原始电机角，deg
        self.target = None             # deg，7 个；None = 不追（停在原地）
        self.wait_end = None
        self.init_end = None
        self.calib_end = None
        self.reset_end = None
        self.last_rx_t = 0.0           # 上一帧收到的时刻（每台一份）；0 = 还没收到过

    def feedback_payload(self):
        return protocol.encode_feedback(self.mode, self.state, self.lost, self.inited,
                                        self.joint, [0.0] * 6, self.gripper, [36] * 7)


class SimLink(Link):
    """模拟 H7 链路，后台生成反馈。

    arm_ids 选择模拟臂，lost_mask/lost_arm 配置周期掉线，robot_type 选择参数。
    """

    DT = 0.004                     # 每臂模拟反馈频率 250 Hz

    def __init__(self, port=None, baud=921600, *, arm_ids=(0, 1),
                 lost_mask=0, lost_arm=0, dt=None, speed=1.0, robot_type=None):
        super().__init__(port=port or "<fake>", baud=baud, robot_type=robot_type)
        arm_ids = tuple(arm_ids)
        if not arm_ids:
            raise ValueError("假板子至少要有一台臂")
        for a in arm_ids:
            if not 0 <= a < self.board_defs.ARM_NUM:
                raise ValueError("arm_id 只能是 0..%d，收到 %r"
                                 % (self.board_defs.ARM_NUM - 1, a))
        self.kind = "FAKE"
        self.dt = self.DT if dt is None else dt
        # speed 缩放模拟时钟和运动，仅用于加速离线测试；操作演练使用 1.0。
        self.speed = float(speed)
        self.arm_ids = arm_ids
        self.lost_mask = int(lost_mask) & 0xFF
        self.lost_arm = lost_arm
        self.boards = {a: _Board(a) for a in arm_ids}

        self._lock = threading.Lock()
        self._alive = False
        self._thr = None
        self._t = 0.0                  # 单调递增的模拟时钟
        self._last_ls_t = -1.0         # 上一帧 0x81 的时刻；< 0 = 还没起窗口
        self._rx_by_tid = {}           # 本窗口按帧类型计数

    # 生命周期

    def open(self, port=None):
        if self._alive:
            return self
        self.port = port or self.port
        self._alive = True
        self.link_up = True
        self.link_err = None
        self._t = 0.0
        self.boards = {a: _Board(a) for a in self.arm_ids}   # 重开时重置模拟状态。
        self._thr = threading.Thread(target=self._gen, name="sdk-sim", daemon=True)
        self._thr.start()
        return self

    def close(self):
        self._alive = False
        if self._thr is not None:
            self._thr.join(timeout=1.0)
            self._thr = None
        self.link_up = False

    @property
    def is_open(self):
        return self._alive

    # 处理上位机命令

    def send(self, tid, payload=b"", arm_id=protocol.DEFAULT_ARM_ID):
        """处理模拟命令，不满足执行条件时丢弃。"""
        if not self._alive:
            raise NotConnected("假板子还没 open（先 link.open()）")
        board = self.boards.get(arm_id)
        if board is None:
            # 丢弃并统计越界臂编号。
            self.n_bad_arm += 1
            return 0

        reply = None
        ack = None
        with self._lock:
            self.n_tx += 1
            self._rx_by_tid[tid] = self._rx_by_tid.get(tid, 0) + 1
            # 有效命令仅更新目标臂的保活；离线臂重新进入离线等待。
            board.last_rx_t = self._t
            if board.state == board_defs.STATE_COMM_LOST:
                board.state = board_defs.STATE_OFFLINE_WAIT
                board.wait_end = self._t + OFFLINE_WAIT_S

            if tid == protocol.TID_VERSION and len(payload) == 4:
                if protocol.decode_version(payload) == self.board_defs.VERSION_CODE:
                    board.version_ok = True
                    reply = protocol.encode(
                        protocol.TID_VERSION,
                        protocol.encode_version(self.board_defs.VERSION_CODE), arm_id)
                # 版本不匹配时不回复；收到匹配版本后即可握手。
                # 模拟器不主动广播握手前的版本帧。
            elif tid == protocol.TID_ARM_INIT and (
                    board.state == board_defs.STATE_WAIT_INIT
                    or (board.state == board_defs.STATE_NORMAL
                        and board.mode in (board_defs.MODE_STOP, board_defs.MODE_DAMPING))):
                # 待初始化或正常运行 / STOP / 阻尼时允许初始化，完成后进入正常运行。
                board.state = board_defs.STATE_INITING
                board.init_end = self._t + INIT_S
                board.mode = board_defs.MODE_RUN
            elif tid == protocol.TID_MOTOR_SET_ZERO and \
                    board.state == board_defs.STATE_WAIT_INIT:
                # 仅待初始化时接受设零。
                board.state = board_defs.STATE_CALIBRATING
                board.calib_end = self._t + CALIB_S
            elif tid == protocol.TID_ARM_RESET and board.state == board_defs.STATE_NORMAL \
                    and not board.lost:
                # 正常运行且电机在线时允许复位，不检查姿态。
                board.state = board_defs.STATE_RESETTING
                board.mode = board_defs.MODE_RUN      # 复位时使用 RUN 模式。
                board.reset_end = self._t + RESET_S
                board.target = list(self.board_defs.RESET_POSE_DEG) + [board.gripper]
            elif tid in _EMPTY_MODE_TID_TO_MODE and board.state == board_defs.STATE_NORMAL \
                    and not board.lost:
                board.mode = _EMPTY_MODE_TID_TO_MODE[tid]
            elif tid == protocol.TID_ARM_CTRL_POS and \
                    len(payload) == protocol.CTRL_POS_LEN:
                # 要求正常运行 / RUN；模拟器不建模遥控器。
                if board.state == board_defs.STATE_NORMAL and board.mode == board_defs.MODE_RUN:
                    try:
                        target, _, _, _ = protocol.decode_ctrl_pos(payload)
                    except ValueError:
                        pass
                    else:
                        board.target = list(target)
            else:
                ack = None   # 丢弃未知命令。

        if reply is not None:
            self.inject_bytes(reply)
        return protocol.OVERHEAD + len(payload)

    # 后台推进双臂模拟状态

    def _gen(self):
        while self._alive:
            with self._lock:
                self._t += self.dt * self.speed
                t = self._t
                frames = []
                acks = []
                for board in self.boards.values():
                    ack = self._step(board, t)
                    if ack is not None:
                        acks.append((board.arm_id, ack))
                    if board.version_ok:
                        frames.append(protocol.encode(protocol.TID_ARM_FEEDBACK,
                                                      board.feedback_payload(),
                                                      board.arm_id))

                # 握手后每 100 ms 报告一次链路统计，第一帧等待完整窗口。
                a = next((b for b in self.boards.values() if b.version_ok), None)
                if a is None:
                    self._last_ls_t = -1.0
                elif self._last_ls_t < 0.0:
                    self._last_ls_t = t
                elif t - self._last_ls_t >= LINK_STATUS_S:
                    frames.append(protocol.encode(protocol.TID_LINK_STATUS,
                                                  self._link_status_payload(),
                                                  a.arm_id))
                    self._last_ls_t = t
            for arm_id, ack in acks:
                self.channel(arm_id).put_ack(ack, time.monotonic())
            for fr in frames:
                self.inject_bytes(fr)
            time.sleep(self.dt)

    def _link_status_payload(self):
        """生成 0x81 窗口统计并清零计数；调用时须持有 _lock。"""
        by = [self._rx_by_tid.pop(tid, 0) for tid in protocol.LINK_TID_ORDER]
        by.append(sum(self._rx_by_tid.values()))
        self._rx_by_tid.clear()
        return protocol.encode_link_status(
            sum(by), tuple(by),
            crc8_err=0, crc16_err=0, drop_bytes=0, bad_arm=0, over_len=0,
            ring_drop=0, gated=0, tx_fail=0,
            dt_ms=int(round(LINK_STATUS_S * 1000)))

    def _step(self, board, t):
        """推进单臂模拟状态；需要回复设零结果时返回 0x11 载荷，否则返回 None。"""
        # 每台臂独立检查 3 s 通信超时，等待握手和通信离线状态除外。
        if board.last_rx_t > 0.0 and board.state not in (
                board_defs.STATE_WAIT_HANDSHAKE, board_defs.STATE_COMM_LOST) \
                and t - board.last_rx_t > COMM_TIMEOUT_S:
            # 已初始化的臂保留正常运行状态；其他状态转为通信离线。
            if board.state != board_defs.STATE_NORMAL:
                board.state = board_defs.STATE_COMM_LOST
            board.mode = board_defs.MODE_DAMPING

        if board.state == board_defs.STATE_WAIT_HANDSHAKE:
            if board.version_ok and t > POWERUP_S:
                board.state = board_defs.STATE_OFFLINE_WAIT
                board.wait_end = t + OFFLINE_WAIT_S
        elif board.state == board_defs.STATE_OFFLINE_WAIT:
            if board.wait_end is not None and t > board.wait_end:
                board.state = board_defs.STATE_WAIT_INIT
                board.mode = board_defs.MODE_STOP
                board.wait_end = None

        if board.state == board_defs.STATE_INITING and board.init_end is not None:
            if t > board.init_end - INIT_GRIPPER_CLEAR_S:
                # 夹爪找零期间清除初始化 bit6，成功后恢复。
                board.inited = 0x3F
            if t > board.init_end:
                board.state = board_defs.STATE_NORMAL
                board.mode = board_defs.MODE_DAMPING   # 初始化完成进入阻尼，RUN 需另发 0x12。
                board.init_end = None
                board.gripper = 0.0    # 模拟夹爪找零后的零角反馈。
                # 模拟找零成功，恢复夹爪初始化标志。
                board.inited = 0x7F

        if board.state == board_defs.STATE_RESETTING and board.reset_end is not None \
                and t > board.reset_end:
            board.state = board_defs.STATE_NORMAL
            board.mode = board_defs.MODE_DAMPING
            board.reset_end = None
            # 复位计时结束时直接到达参考角，避免跟踪滞后导致误报。
            board.joint = list(self.board_defs.RESET_POSE_DEG)

        ack = None
        if board.state == board_defs.STATE_CALIBRATING and board.calib_end is not None \
                and t > board.calib_end:
            board.state = board_defs.STATE_WAIT_INIT
            board.mode = board_defs.MODE_STOP
            board.inited = 0x7F
            board.calib_end = None
            # 设零结果：1 未确认、2 成功、5 跳过。
            ack = protocol.SetZeroAck(state=board_defs.ZERO_STATE_DONE,
                                      result=(5, 5, 5, 2, 2, 2, 1), elapsed_ms=420)

        # 按配置周期切换电机掉线状态。
        if self.lost_mask and board.arm_id == self.lost_arm \
                and board.state == board_defs.STATE_NORMAL:
            if int(t / FAKE_LOST_PERIOD_S) % 2 == 1:
                if board.lost == 0:
                    board.lost = self.lost_mask
                    board.mode = board_defs.MODE_STOP   # 电机掉线时切换 STOP。
            elif board.lost:
                board.lost = 0
                board.mode = board_defs.MODE_DAMPING

        if board.target is not None:
            # 按 speed 同时缩放跟踪速度和时钟。
            dt = self.dt * self.speed
            for i in range(6):
                d = board.target[i] - board.joint[i]
                mx = 60.0 * dt               # 关节 60 deg/s
                board.joint[i] += max(-mx, min(mx, d))
            d = board.target[6] - board.gripper
            mx = 200.0 * dt                  # 夹爪 200 deg/s
            board.gripper += max(-mx, min(mx, d))
        return ack
