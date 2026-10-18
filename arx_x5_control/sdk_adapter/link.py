"""通过现有 sdk.Arm 接口运行 Python 机械臂控制核心。

仅转换 SDK 命令参数，不使用串口、完整字节帧或进程间通信。
需要 SDK 接入时显式导入 LocalLink，控制核心自身无需导入 SDK。
"""

from collections import deque
import struct
import threading
import time

from sdk import Link, LinkError, NotConnected, ProtocolError, protocol

from ..config import ArmConfig
from ..diagnostics import ControlTrace
from ..interfaces import MotorIO
from ..performance import PerformanceProbe
from .. import log
from ..system import ControlSystem
from ..types import Command, CommandKind as K, TickResult, timestamp

_LOG = log.get_logger("arx_x5_control.sdk_adapter.link")

_EMPTY_COMMANDS = {
    protocol.TID_ARM_INIT: K.INITIALIZE,
    protocol.TID_MOTOR_SET_ZERO: K.CALIBRATE,
    protocol.TID_ARM_START: K.START,
    protocol.TID_ARM_STOP: K.STOP,
    protocol.TID_ARM_DAMPING: K.DAMPING,
    protocol.TID_ARM_GRAVITY: K.GRAVITY,
    protocol.TID_ARM_RESET: K.RESET,
}


def _command(tid, payload, arm_id):
    if type(tid) is not int or not isinstance(payload, (bytes, bytearray)):
        raise ProtocolError("tid must be int and payload must be bytes")
    try:
        if tid == protocol.TID_VERSION:
            if struct.unpack("<I", payload)[0] != protocol.VERSION_CODE:
                raise ValueError("SDK protocol version mismatch")
            return Command(arm_id, K.CONNECT)
        if tid in _EMPTY_COMMANDS:
            if payload:
                raise ValueError("this command requires an empty payload")
            return Command(arm_id, _EMPTY_COMMANDS[tid])
        if tid == protocol.TID_ARM_CTRL_POS:
            values = struct.unpack("<7f", payload)
            return Command(arm_id, K.POSITION, values[:6], values[6])
        if tid == protocol.TID_ARM_CTRL_POS_VEL:
            values = struct.unpack("<13f", payload)
            return Command(arm_id, K.POSITION_VELOCITY, values[:6], values[12], values[6:12])
        raise ValueError(f"unsupported local command tid: {tid:#x}")
    except (ValueError, struct.error) as exc:
        raise ProtocolError(str(exc)) from exc


class LocalLink(Link):
    """共享 SDK 链路，持有一个 Python 控制器和一个 MotorIO 适配器。

    LocalLink(motor_io) 接入提供的电机接口；无硬件时显式使用 simulated()。
    sdk.Arm(0, link) 和 link.arm(0) 保留 SDK 的单位、预检、标定及等待方法。

    open() 启动目标周期为 1 ms 的后台线程，不保证硬实时。
    background=False 时由调用者执行 step(now_ms)；SDK 的阻塞等待期间，
    必须另有调用者继续推进 step。不自动保活，每台臂仍需在 3 s 内收到有效命令。
    close() 交付停机请求并等待线程退出；重新打开会新建控制器，需重新握手和初始化。

    后台模式的 MotorIO 时间戳为从本链路构造起经过的毫秒数；手动模式使用 step
    的显式时钟。重新打开后仍应保持时间递增。MotorIO 对象会被保留，其设备的
    打开和关闭由调用者负责，本链路只交付控制及停机请求。
    """

    def __init__(self, motor_io: MotorIO, *, configurations=None, background=True, collect_performance=False,
                 collect_trace=False):
        super().__init__(port="local", verify_on_open=False, auto_reconnect=False)
        if not all(callable(getattr(motor_io, name, None)) for name in
                   ("poll_feedback", "write_motor_requests")):
            raise TypeError("motor_io must implement MotorIO")
        self.configurations = tuple(configurations) if configurations is not None else (ArmConfig(), ArmConfig())
        self.controller = ControlSystem(self.configurations)
        self.motor_io = motor_io
        self.background = background
        self.kind = "LOCAL"
        self.last_result = TickResult()
        self._origin_ns = time.monotonic_ns()
        self._last_ms = None
        self._lifecycle_lock = threading.RLock()
        self._step_lock = threading.Lock()
        self._commands = [deque() for _ in self.configurations]
        self._events = deque(maxlen=512)
        self._event_lock = threading.Lock()
        self.event_overruns = 0
        self._failure = self._shutdown_failure = None
        self._performance = PerformanceProbe() if collect_performance else None
        self._trace = ControlTrace() if collect_trace else None

    @classmethod
    def simulated(cls, *, configurations=None, background=True, collect_performance=False, collect_trace=False):
        """显式启用测试电机模型，运行的仍是 ControlSystem 控制逻辑。"""
        from ..simulation import SimulatedMotors
        configs = tuple(configurations) if configurations is not None else (ArmConfig(), ArmConfig())
        return cls(SimulatedMotors(len(configs)), configurations=configs, background=background,
                   collect_performance=collect_performance, collect_trace=collect_trace)

    def _validate_arm(self, arm_id):
        if type(arm_id) is not int or not 0 <= arm_id < len(self.configurations):
            raise ValueError(f"arm_id must be in 0..{len(self.configurations)-1}")

    def channel(self, arm_id):
        self._validate_arm(arm_id)
        return super().channel(arm_id)

    @property
    def is_open(self):
        return self._alive and not self._stop_ev.is_set()

    def _clear_channels(self):
        with self._chan_lock:
            for ch in self._channels.values():
                with ch.lock:
                    ch.feedback = ch.ack = None
                    ch.feedback_t = ch.ack_t = 0.0
                    ch.ack_event.clear()
                    # 保留递增序号，让已有 SDK Arm 对象可以继续等待新反馈。

    def open(self, port=None):
        if port not in (None, "local"):
            raise LinkError("LocalLink does not accept a serial port")
        with self._lifecycle_lock:
            if self.is_open:
                return self
            if self._thr is not None and self._thr.is_alive():
                raise LinkError("previous control worker has not stopped")
            self.controller = ControlSystem(self.configurations)
            if self._performance is not None:
                self._performance.reset_clock()
            self.last_result = TickResult()
            self._failure = self._shutdown_failure = self.link_err = None
            self.board_version = None
            self.version_seen = False
            self._clear_channels()
            with self._write_lock:
                for queue in self._commands:
                    queue.clear()
                self._stop_ev.clear()
                self._alive = self.link_up = True
            self.open_gen += 1
            if self.background:
                self._thr = threading.Thread(target=self._worker, name="arx-x5-control", daemon=True)
                self._thr.start()
            _LOG.info("sdk_link.opened", arm_count=len(self.configurations), background=self.background)
            return self

    def close(self):
        with self._lifecycle_lock:
            with self._write_lock:
                self._stop_ev.set()
                self._alive = self.link_up = False
                for queue in self._commands:
                    queue.clear()
            if self._thr is not None:
                if self._thr is threading.current_thread():
                    raise LinkError("close must be called outside the MotorIO callback")
                self._thr.join(timeout=2.0)
                if self._thr.is_alive():
                    raise LinkError("MotorIO is blocking; control worker has not stopped")
                self._thr = None
            elif not self.background and self.open_gen:
                with self._step_lock:
                    self._shutdown()
            self._clear_channels()
            if self._shutdown_failure is not None:
                raise LinkError(f"motor shutdown delivery failed: {self._shutdown_failure}") from self._shutdown_failure

    def reopen(self, port=None):
        self.close()
        return self.open(port)

    def check_health(self):
        """将控制线程或适配器故障抛为 SDK LinkError，不在此处自动重启。"""
        if self._failure is not None:
            raise LinkError(f"local controller failed: {self.link_err}") from self._failure
        if not self.is_open:
            raise NotConnected("LocalLink is not open")

    def send(self, tid, payload=b"", arm_id=protocol.DEFAULT_ARM_ID):
        self._validate_arm(arm_id)
        command = _command(tid, payload, arm_id)
        with self._write_lock:
            self.check_health()
            queue = self._commands[arm_id]
            if len(queue) >= 7:
                raise LinkError(f"arm {arm_id} command queue full; wait for the next control tick")
            queue.append(command)
            self.n_tx += 1
        # 保留 Link.send 的返回值约定，此处不构造完整字节帧。
        return protocol.OVERHEAD + len(payload)

    def inject_bytes(self, data):
        raise ProtocolError("LocalLink accepts SDK commands, not wire frames")

    def _step_once(self, now_ms):
        probe = self._performance
        if probe is None:
            return self._step_once_impl(now_ms)
        start = time.monotonic_ns()
        before = self.controller.command_last_ms
        probe.tick("control.period", start)
        try:
            return self._step_once_impl(now_ms)
        finally:
            if self.controller.command_last_ms != before:
                probe.tick("command.period", start)
            probe.observe("control.duration", time.monotonic_ns()-start)

    def performance_snapshot(self):
        return self._performance.snapshot() if self._performance is not None else None

    def pop_control_trace(self):
        return self._trace.take() if self._trace is not None else b""

    def _step_once_impl(self, now_ms):
        commands = []
        # 只在 2 ms 命令周期到达时取走输入，避免两批命令在核心处理前累积，
        # 超出核心每台臂最多容纳 7 条命令的队列容量。
        if self.controller.command_last_ms is None or now_ms - self.controller.command_last_ms >= 2:
            with self._write_lock:
                for queue in self._commands:
                    commands.extend(queue)
                    queue.clear()
        result = self.controller.step(now_ms, commands=commands,
                                      motor_feedback=self.motor_io.poll_feedback(now_ms))
        self._last_ms = now_ms
        self.last_result = result
        self.motor_io.write_motor_requests(result.motor_requests)
        if self._trace is not None:
            self._trace.capture(self.controller, result, now_ms)
        self._publish(result)
        return result

    def step(self, now_ms):
        """手动推进一个时刻；后台模式禁止调用。"""
        if self.background:
            raise RuntimeError("step requires background=False")
        timestamp(now_ms)
        with self._step_lock:
            self.check_health()
            if self._last_ms is not None and now_ms <= self._last_ms:
                raise ValueError("LocalLink step timestamps must strictly increase across opens")
            try:
                return self._step_once(now_ms)
            except Exception as exc:
                self._fail(exc)
                self._shutdown()
                self.check_health()

    def _worker(self):
        try:
            while not self._stop_ev.is_set():
                now_ms = (time.monotonic_ns() - self._origin_ns) // 1_000_000
                if self._last_ms is None or now_ms > self._last_ms:
                    with self._step_lock:
                        self._step_once(now_ms)
                delay = (self._origin_ns + (now_ms + 1) * 1_000_000 - time.monotonic_ns()) / 1e9
                self._stop_ev.wait(max(0, delay))
        except Exception as exc:
            self._fail(exc)
        finally:
            self._shutdown()
            self._alive = self.link_up = False

    def _fail(self, exc):
        with self._write_lock:
            if self._failure is None:
                self._failure = exc
            self.link_err = str(self._failure)
            self._alive = self.link_up = False
            self._stop_ev.set()
        _LOG.error("sdk_link.failed", error=exc)

    def _shutdown(self):
        if self.controller.closed:
            return
        now = max(self._last_ms or 0, self.controller.last_ms or 0)
        if self.background:
            now = max(now, (time.monotonic_ns() - self._origin_ns) // 1_000_000)
        try:
            result = self.controller.shutdown(now)
            self.motor_io.write_motor_requests(result.motor_requests)
            self._publish(result)
        except Exception as exc:
            self._shutdown_failure = exc
            self._fail(exc)
            _LOG.error("sdk_link.shutdown_failed", error=exc)

    def _publish(self, result):
        received_at = time.monotonic()
        for fb in result.feedback:
            self.channel(fb.arm_id).put_feedback(protocol.Feedback(
                mode=int(fb.mode), robot_state=int(fb.state),
                motor_lost_mask=sum(int(v) << i for i, v in enumerate(fb.motor_lost)),
                motor_inited_mask=sum(int(v) << i for i, v in enumerate(fb.motor_initialized)),
                joint_angle=fb.joint_angle, joint_vel=fb.joint_velocity,
                gripper_angle=fb.gripper_angle,
                motor_temp=tuple(0 if v is None else min(255, max(0, int(v))) for v in fb.motor_temperature),
            ), received_at)
            self.n_rx += 1
        for event in result.events:
            if event.kind in ("connected", "connection_required"):
                self.board_version = protocol.VERSION_CODE
                self.version_seen = True
                self.n_rx += 1
            elif event.kind == "calibrated":
                self.channel(event.arm_id).put_ack(protocol.SetZeroAck(
                    int(event.zero_state), tuple(map(int, event.zero_results)), event.elapsed_ms,
                ), received_at)
                self.n_rx += 1
        with self._event_lock:
            self.event_overruns += max(0, len(self._events) + len(result.events) - self._events.maxlen)
            self._events.extend(result.events)
        _LOG.events(result.events)

    def drain_events(self):
        """取走诊断事件；缓冲区最多保留最近 512 条，覆盖数量可从 stats 读取。"""
        with self._event_lock:
            events = tuple(self._events)
            self._events.clear()
            return events

    def wait_feedback(self, arm_id, after_seq=0, timeout=3.0, poll=0.005):
        channel = self.channel(arm_id)
        deadline = time.monotonic() + timeout
        while True:
            self.check_health()
            feedback, _, seq = channel.snapshot_feedback()
            if feedback is not None and seq > after_seq:
                return feedback
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            self._stop_ev.wait(min(poll, remaining))

    @property
    def stats(self):
        stats = super().stats
        with self._write_lock:
            stats["pending_commands"] = tuple(len(q) for q in self._commands)
        stats.update(missed_motor_ticks=self.controller.missed_motor_ticks,
                     missed_command_ticks=self.controller.missed_command_ticks,
                     stale_feedback_count=self.controller.stale_feedback_count,
                     rejected_commands=tuple(a.rejected_count for a in self.controller.arms),
                     event_overruns=self.event_overruns)
        stats.update(open_gen=self.open_gen, control_thread_id=self._thr.native_id if self._thr else None,
                     controller_origin_ns=self._origin_ns)
        if self._trace is not None:
            stats["control_trace"] = self._trace.stats()
        return stats

    def describe(self):
        return "local Python ControlSystem / injected MotorIO"
