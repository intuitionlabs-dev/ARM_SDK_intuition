"""假板子（`--fake`）：不接硬件就能把面板、按键、状态机、闸门整条演练一遍。

`SimLink` 是 `sdk.Link` 的子类：

- 收包走的是**真的** `FrameParser` + 按 `arm_id` 分流（继承 `Link.inject_bytes`）；
- 发出去的帧也真的按板子的闸门被模拟状态机消化（静默丢弃的那几条也照样丢弃）；
- 反馈帧用 `protocol.encode_feedback` 组，和板上的 `FramePktArmFeedback` 同一份布局定义。

所以"假板子上好好的、真板子上坏了"这类差别只剩固件本身，模拟这边不会各抄一套。

**两台臂都在**：一块板子两块板卡、两条 CAN 总线 = 两台臂，每台一份 `_Board`。看门狗按
板子的口径建模 —— `s_last_rx_ms` 是**全链路一份**（收到给臂 A 的帧也把臂 B 喂了），所以
只指挥一台的时候，另一台不会凭空掉进通信离线。

**握手也是照抄的**：版本码对不上就一个字节都不回、也不发反馈（真板子就是这样）。所以
"面板上一直没反馈"这个现象在 `--fake` 下同样能复现出来，前提是上位机发的 0x01 版本码
确实对不上。

不 import pyserial（`open()` 也不碰），所以没装它也能 `import sdk.sim`，`--fake` 全程
不碰硬件。
"""

import struct
import threading
import time

from . import board_defs, protocol
from .errors import NotConnected
from .link import Link

# 板子那侧的时间常数（`robot_state.c` / `arm_auto.c`），改这里之前先对一下固件
POWERUP_S = 0.6           # 上电 → 离线等待
OFFLINE_WAIT_S = 0.8      # 离线等待 → 待初始化（真板子等的是"电机全在线"，这里定死）
INIT_S = 3.0              # 0x02：一键复位 + 一键夹爪找零
CALIB_S = 1.2             # 0x10：一轮设零
RESET_S = 2.0             # 0x18：ARM_AUTO_RESET_TIME_S
COMM_TIMEOUT_S = 0.5      # ROBOT_COMM_TIMEOUT_MS：任何有效帧都喂它
INIT_GRIPPER_CLEAR_S = 1.5  # 初始化末段：找零把 motor_inited[夹爪] 先清掉
FAKE_LOST_PERIOD_S = 5.0  # --fake-lost 的翻转周期

_EMPTY_MODE_TID_TO_MODE = {
    protocol.TID_ARM_START: board_defs.MODE_RUN,
    protocol.TID_ARM_GRAVITY: board_defs.MODE_GRAVITY,
    protocol.TID_ARM_STOP: board_defs.MODE_STOP,
    protocol.TID_ARM_DAMPING: board_defs.MODE_DAMPING,
}


class _Board:
    """一台臂在假板子里的全部状态。角度一律是**线上单位**（deg）。"""

    def __init__(self, arm_id):
        self.arm_id = arm_id
        self.version_ok = False        # 收到过版本码相符的 0x01（真板子握手前不发反馈）
        self.state = board_defs.STATE_WAIT_HANDSHAKE
        self.mode = board_defs.MODE_STOP
        self.lost = 0x00
        self.inited = 0x7F
        self.joint = [0.0] * 6         # deg
        self.gripper = 0.0             # deg（电机原始角，不做 pos_to_angle）
        self.target = None             # deg，7 个；None = 不追（停在原地）
        self.wait_end = None
        self.init_end = None
        self.calib_end = None
        self.reset_end = None

    def feedback_payload(self):
        return protocol.encode_feedback(self.mode, self.state, self.lost, self.inited,
                                        self.joint, [0.0] * 6, self.gripper, [36] * 7)


class SimLink(Link):
    """假装板子的链路。接口和 `Link` 一样，`send` 在主线程、造帧在后台线程。

    `arm_ids` 决定这块假板子上有几台臂（默认两台，和真板子一致）。`lost_mask` /
    `lost_arm` 是 `--fake-lost`：周期性地让 `lost_arm` 那台"掉线"，用来演练
    「电机掉线把 arm_mode 打回 stop」那条路径。
    """

    DT = 0.004                     # 反馈节拍：250 Hz/台（真板子实测 250 Hz）

    def __init__(self, port=None, baud=921600, *, arm_ids=(0, 1),
                 lost_mask=0, lost_arm=0, dt=None, speed=1.0):
        arm_ids = tuple(arm_ids)
        if not arm_ids:
            raise ValueError("假板子至少要有一台臂")
        for a in arm_ids:
            if not 0 <= a < board_defs.ARM_NUM:
                raise ValueError("arm_id 只能是 0..%d，收到 %r"
                                 % (board_defs.ARM_NUM - 1, a))
        super().__init__(port=port or "<fake>", baud=baud)
        self.kind = "FAKE"
        self.dt = self.DT if dt is None else dt
        # speed > 1 = 假板子的时钟走得比真实时间快（每一拍虚拟时间前进 dt*speed）。
        # **只给离线自测用**：「0x02 要 3 s」这种等待真等一遍太慢。真机演练保持 1.0，
        # 否则排查"弹对话框期间掉没掉线"这类问题会得到假的时间关系。
        self.speed = float(speed)
        self.arm_ids = arm_ids
        self.lost_mask = int(lost_mask) & 0xFF
        self.lost_arm = lost_arm
        self.boards = {a: _Board(a) for a in arm_ids}

        self._lock = threading.Lock()
        self._alive = False
        self._thr = None
        self._t = 0.0                  # 假板子的时钟（虚拟，只往前走）
        self._last_rx_t = 0.0          # 上一帧收到的时刻；0 = 还没收到过

    # ---- 生命周期 ----

    def open(self, port=None):
        if self._alive:
            return self
        self.port = port or self.port
        self._alive = True
        self.link_up = True
        self.link_err = None
        self._t = 0.0
        self._last_rx_t = 0.0
        self.boards = {a: _Board(a) for a in self.arm_ids}   # 重开 = 板子重新上电
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

    # ---- 发送（上位机说的话）----

    def send(self, tid, payload=b"", arm_id=protocol.DEFAULT_ARM_ID):
        """闸门和真板子逐条对齐：**不合格就静默丢弃**（这里连计数都不额外加，板子也没有）。"""
        if not self._alive:
            raise NotConnected("假板子还没 open（先 link.open()）")
        board = self.boards.get(arm_id)
        if board is None:
            # 越界 arm_id：和真板子同一个口径 —— 丢弃、计数、不报错
            self.n_bad_arm += 1
            return 0

        reply = None
        ack = None
        with self._lock:
            self.n_tx += 1
            # 收到**任意**有效帧都算"上位机还在说话"。注意落点是**离线等待**、不是原状态：
            # 板子会重新走一遍"电机全在线 → 待初始化"。
            self._last_rx_t = self._t
            if board.state == board_defs.STATE_COMM_LOST:
                board.state = board_defs.STATE_OFFLINE_WAIT
                board.wait_end = self._t + OFFLINE_WAIT_S

            if tid == protocol.TID_VERSION and len(payload) == 4:
                if protocol.decode_version(payload) == protocol.VERSION_CODE:
                    board.version_ok = True
                    reply = protocol.encode(protocol.TID_VERSION,
                                            protocol.encode_version(), arm_id)
                # 版本码不符：板子**一个字节都不回**，而且之后什么都不发（要复位板子）
            elif tid == protocol.TID_ARM_INIT and (
                    board.state == board_defs.STATE_WAIT_INIT
                    or (board.state == board_defs.STATE_NORMAL
                        and board.mode in (board_defs.MODE_STOP, board_defs.MODE_DAMPING))):
                # 板侧两个入口(robot_state.c 的 HandleArmInit)：待初始化(2)，或 正常运行(4)
                # 的 stop/damping 档 —— run 档下静默丢弃。跑完都落回 4（不按入口区分）。
                board.state = board_defs.STATE_INITING
                board.init_end = self._t + INIT_S
                board.mode = board_defs.MODE_RUN
            elif tid == protocol.TID_MOTOR_SET_ZERO and \
                    board.state == board_defs.STATE_WAIT_INIT:
                # 0x10 的闸门是"待初始化 + 遥控器离线"，正常运行态**不受理**
                board.state = board_defs.STATE_CALIBRATING
                board.calib_end = self._t + CALIB_S
            elif tid == protocol.TID_ARM_RESET and board.state == board_defs.STATE_NORMAL \
                    and not board.lost:
                # 0x18 的闸门是"正常运行 + 遥控器离线 + 无掉线"，**故意不查姿态**
                board.state = board_defs.STATE_RESETTING
                board.mode = board_defs.MODE_RUN      # 复位期间位置环要使上劲
                board.reset_end = self._t + RESET_S
                board.target = list(board_defs.RESET_POSE_DEG) + [board.gripper]
            elif tid in _EMPTY_MODE_TID_TO_MODE and board.state == board_defs.STATE_NORMAL \
                    and not board.lost:
                board.mode = _EMPTY_MODE_TID_TO_MODE[tid]
            elif tid == protocol.TID_ARM_CTRL_POS and \
                    len(payload) == protocol.CTRL_POS_LEN:
                # 三重闸门：state==normal & mode==run & 遥控器离线（最后一条查不到）
                if board.state == board_defs.STATE_NORMAL and board.mode == board_defs.MODE_RUN:
                    board.target = list(struct.unpack("<7f", payload))
            else:
                ack = None   # 认不出来的 tid：静默丢弃（和板子一样）

        if reply is not None:
            self.inject_bytes(reply)
        return protocol.OVERHEAD + len(payload)

    # ---- 后台线程：推两台板子 ----

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
            for arm_id, ack in acks:
                self.channel(arm_id).put_ack(ack, time.monotonic())
            for fr in frames:
                self.inject_bytes(fr)
            time.sleep(self.dt)

    def _step(self, board, t):
        """推一拍某台板子。返回要补发的一帧 0x11（设零反馈帧），没有就 None。"""
        # 500 ms 通信看门狗：握手通过之后除了"等待握手"和"已经在离线里"两态，**任何状态**
        # 都算。这一条是"上位机卡住不发帧"的兜底 —— 也是"弹个确认框就把臂打出正常运行"的
        # 元凶，所以必须模拟，否则假板子会假装链路永远不断、面板上的状态就骗人。
        # **判据是全链路一份**（`s_last_rx_ms` 在按 arm_id 分派之前刷新）。
        if self._last_rx_t > 0.0 and board.state not in (
                board_defs.STATE_WAIT_HANDSHAKE, board_defs.STATE_COMM_LOST) \
                and t - self._last_rx_t > COMM_TIMEOUT_S:
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
                # 找零把 motor_inited[夹爪] 先清 0、设零成功才置 1（真板子就是这样），
                # 所以初始化末段的 0x80 里 inited 会少 bit6
                board.inited = 0x3F
            if t > board.init_end:
                board.state = board_defs.STATE_NORMAL
                board.mode = board_defs.MODE_DAMPING   # 到 normal 是 damping，要等 0x12
                board.init_end = None
                board.gripper = 0.0    # 找零把零点挪到行程中点
                # 找零成功 → 置回 1。4 配 0x3F = 找零超时/失败（夹爪 ∓142 软限位没生效）
                board.inited = 0x7F

        if board.state == board_defs.STATE_RESETTING and board.reset_end is not None \
                and t > board.reset_end:
            board.state = board_defs.STATE_NORMAL
            board.mode = board_defs.MODE_DAMPING
            board.reset_end = None
            # 真板子那条五次样条是**定时**的（2 s 正好走到位），不像关节那个一阶滞后
            # 追不上就慢慢磨；这里直接落位，免得假板子自己触发"复位疑似没执行"的误报。
            board.joint = list(board_defs.RESET_POSE_DEG)

        ack = None
        if board.state == board_defs.STATE_CALIBRATING and board.calib_end is not None \
                and t > board.calib_end:
            board.state = board_defs.STATE_WAIT_INIT
            board.mode = board_defs.MODE_STOP
            board.inited = 0x7F
            board.calib_end = None
            # state=完成；result 里 1=未确认（达妙本来就在读数 0 附近，不是故障）、
            # 2=成功、5=跳过（没参与这一轮）
            ack = protocol.SetZeroAck(state=board_defs.ZERO_STATE_DONE,
                                      result=(5, 5, 5, 2, 2, 2, 1), elapsed_ms=420)

        # --fake-lost：周期性地制造"电机掉线"，看面板怎么反应
        if self.lost_mask and board.arm_id == self.lost_arm \
                and board.state == board_defs.STATE_NORMAL:
            if int(t / FAKE_LOST_PERIOD_S) % 2 == 1:
                if board.lost == 0:
                    board.lost = self.lost_mask
                    board.mode = board_defs.MODE_STOP   # 板子掉线会把 arm_mode 打回 stop
            elif board.lost:
                board.lost = 0
                board.mode = board_defs.MODE_DAMPING

        if board.target is not None:
            # 一阶滞后追目标。**速率也要乘 speed** —— speed 是"整台假板子的时间缩放"，
            # 只让时钟快跑而不让运动跟着快，跟踪看起来就会慢 speed 倍。
            dt = self.dt * self.speed
            for i in range(6):
                d = board.target[i] - board.joint[i]
                mx = 60.0 * dt               # 关节 60 deg/s
                board.joint[i] += max(-mx, min(mx, d))
            d = board.target[6] - board.gripper
            mx = 200.0 * dt                  # 夹爪 200 deg/s
            board.gripper += max(-mx, min(mx, d))
        return ack
