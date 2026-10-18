"""带有界队列和后台收发线程的 CAN MotorIO。"""

from collections import Counter, deque
from dataclasses import dataclass, replace
from math import isfinite
import os
import threading
import time

from . import log
from .can_protocol import decode_feedback, encode_request, feedback_queries
from arx_x5_control.usb2can import TransportRegistry
from .config import MOTOR_IDS
from .performance import PerformanceProbe
from .types import MotorAction, MotorRequest, timestamp

_LOG = log.get_logger(__name__)


class CANIOError(OSError):
    """CAN 适配器故障，调用者须终止当前控制循环。"""


@dataclass(frozen=True)
class CANChannel:
    arm_id: int
    channel: str
    interface: str = "slcan"
    bitrate: int = 1000000

    def __post_init__(self):
        if type(self.arm_id) is not int or self.arm_id not in (0, 1):
            raise ValueError("arm_id must be 0 or 1")
        if (not isinstance(self.channel, str) or not self.channel
                or not isinstance(self.interface, str) or not self.interface):
            raise ValueError("provide a channel and transport interface name")
        if self.bitrate != 1000000:
            raise ValueError("X5 CAN bitrate must be 1000000")


@dataclass(frozen=True)
class _Pending:
    frame: object
    index: int
    created_ns: int
    action: MotorAction | None = None
    guard_after: bool = False


class CANMotorIO:
    """每臂一条 CAN 总线，支持 MotorIO 的两个非阻塞方法。

    open、flush 和 close 为生命周期操作。控制线程调用 poll_feedback 与
    write_motor_requests。read_only 模式仅接受 query_feedback 的读取请求。
    """

    def __init__(self, channels, *, read_only=False, tx_capacity=256, rx_capacity=4096,
                 max_tx_age_ms=20, max_rx_age_ms=50, command_timeout_ms=100, transport_factory=None,
                 collect_performance=False, transport_config=None):
        self.channels = tuple(channels)
        if not self.channels or not all(isinstance(c, CANChannel) for c in self.channels):
            raise ValueError("provide one or two CANChannel objects")
        arms = [c.arm_id for c in self.channels]
        self._registry = (transport_config if isinstance(transport_config, TransportRegistry)
                          else TransportRegistry(transport_config))
        endpoints = [(self._registry.specification(c.interface).class_path,
                      self._registry.endpoint(c.interface, c.channel)) for c in self.channels]
        if len(set(arms)) != len(arms) or len(set(endpoints)) != len(endpoints):
            raise ValueError("each arm and CAN channel must be unique")
        if any(type(v) is not int or v < 1 for v in (tx_capacity, rx_capacity)):
            raise ValueError("queue capacities must be positive integers")
        if not all(isfinite(v) and v > 0 for v in (max_tx_age_ms, max_rx_age_ms, command_timeout_ms)):
            raise ValueError("queue age limits must be finite and positive")
        self.read_only = bool(read_only)
        self.tx_capacity, self.rx_capacity = tx_capacity, rx_capacity
        self.max_tx_age_ns = int(max_tx_age_ms * 1_000_000)
        self.max_rx_age_ns = int(max_rx_age_ms * 1_000_000)
        self.command_timeout_ns = int(command_timeout_ms * 1_000_000)
        self._factory = transport_factory or self._default_transport
        self._condition = threading.Condition()
        self._stop = threading.Event()
        self._wake = threading.Event()
        self._thread = None
        self._transports = {}
        self._queues = {arm: deque() for arm in arms}
        self._guard_until = {arm: 0 for arm in arms}
        self._next_tx = {arm: 0 for arm in arms}
        self._encos_active = {arm: [False]*3 for arm in arms}
        self._rx = deque()
        self._stats = Counter()
        self._failure = None
        self._inflight = False
        self._opened = self._used = self._closing = False
        self._actuated = set()
        self._control_submitted = False
        self._active = {}
        self._zero_pending = {}
        self._last_errors = {}
        self._epoch_ns = None
        self._last_poll_ms = None
        self._performance = PerformanceProbe() if collect_performance else None

    def _default_transport(self, config):
        return self._registry.create(config.interface, config.channel, bitrate=config.bitrate)

    def open(self):
        if self._used:
            raise RuntimeError("CANMotorIO instances can be opened once")
        self._used = True
        try:
            for config in self.channels:
                self._transports[config.arm_id] = self._factory(config)
            for transport in self._transports.values():
                configure = getattr(transport, "configure", None)
                if configure is not None:
                    configure()
            for transport in self._transports.values():
                start = getattr(transport, "start", None)
                if start is not None:
                    start()
        except BaseException:
            for transport in self._transports.values():
                try:
                    transport.close()
                except Exception:
                    pass
            self._transports.clear()
            raise
        self._opened = True
        self._thread = threading.Thread(target=self._worker, name="arx-can-io", daemon=True)
        self._thread.start()
        _LOG.info("can.opened", channels=len(self.channels), read_only=self.read_only)
        return self

    def __enter__(self):
        return self.open()

    def __exit__(self, exc_type, exc, traceback):
        self.close()

    def _check_locked(self):
        if self._failure is not None:
            raise CANIOError(str(self._failure)) from self._failure
        if not self._opened or self._closing:
            raise CANIOError("CANMotorIO is closed")

    def check_health(self):
        with self._condition:
            self._check_locked()

    def stats(self):
        with self._condition:
            return dict(self._stats, pending_tx=sum(map(len, self._queues.values())),
                        pending_rx=len(self._rx), read_only=self.read_only,
                        failed=self._failure is not None, error=str(self._failure) if self._failure else None,
                        worker_pid=os.getpid(), io_thread_id=self._thread.native_id if self._thread else None)

    def performance_snapshot(self):
        return self._performance.snapshot() if self._performance is not None else None

    def _enqueue(self, jobs):
        with self._condition:
            self._check_locked()
            counts = Counter(arm for arm, _ in jobs)
            if any(len(self._queues[arm])+n > self.tx_capacity for arm, n in counts.items()):
                self._failure = CANIOError("CAN transmit queue full")
                self._stop.set()
                self._wake.set()
                raise self._failure
            for arm, job in jobs:
                self._queues[arm].append(job)
                if job.action is not None:
                    self._control_submitted = True
            if self._performance is not None:
                for arm in counts:
                    self._performance.peak(f"arm{arm}.tx_queue", len(self._queues[arm]))
            self._wake.set()

    def write_motor_requests(self, requests):
        requests = tuple(requests)
        jobs = []
        now = time.monotonic_ns()
        for request in requests:
            frame = encode_request(request)
            if request.arm_id not in self._queues:
                raise ValueError("motor request refers to an unconfigured arm")
            age = max(0, (self._last_poll_ms or request.timestamp_ms)-request.timestamp_ms)
            jobs.append((request.arm_id, _Pending(frame, request.motor_index, now-age*1_000_000,
                         request.action, request.family == "encos" and request.action == MotorAction.SAVE_ZERO)))
        if jobs and self.read_only:
            raise PermissionError("read-only CAN adapter accepts feedback queries only")
        self._enqueue(jobs)

    def query_feedback(self, arm_id=None):
        """排队读取反馈，ENCOS 查询之间保持 501 ms 间隔。"""
        arms = tuple(self._queues) if arm_id is None else (arm_id,)
        if any(type(arm) is not int or arm not in self._queues for arm in arms):
            raise ValueError("query refers to an unconfigured arm")
        with self._condition:
            self._check_locked()
            if self._control_submitted:
                raise CANIOError("feedback queries require an idle control output")
        now = time.monotonic_ns()
        jobs = [(arm, _Pending(frame, i, now, guard_after=i < 3))
                for arm in arms for i in range(7) for frame in feedback_queries(i)]
        self._enqueue(jobs)

    def poll_feedback(self, now_ms):
        timestamp(now_ms)
        with self._condition:
            self._check_locked()
            if self._last_poll_ms is not None and now_ms < self._last_poll_ms:
                raise ValueError("MotorIO clock must be monotonic")
            now = time.monotonic_ns()
            if self._epoch_ns is None:
                self._epoch_ns = now-now_ms*1_000_000
            self._last_poll_ms = now_ms
            output = []
            while self._rx:
                received_ns, feedback = self._rx.popleft()
                acquired_ms = (received_ns-self._epoch_ns)//1_000_000
                if now-received_ns > self.max_rx_age_ns or acquired_ms < 0:
                    self._stats["stale_rx"] += 1
                    continue
                if acquired_ms > now_ms:
                    # 调用者使用离散毫秒时钟；当前毫秒尚未到达的样本留到下一拍。
                    self._rx.appendleft((received_ns, feedback))
                    break
                output.append(replace(feedback, timestamp_ms=acquired_ms))
                if self._performance is not None:
                    self._performance.observe(f"arm{feedback.arm_id}.feedback_age", now-received_ns)
            return tuple(output)

    def flush(self, timeout=2.0):
        """等待当前队列完成；该方法用于生命周期管理。"""
        if not isfinite(timeout) or timeout <= 0:
            raise ValueError("flush timeout must be finite and positive")
        deadline = time.monotonic()+timeout
        with self._condition:
            while True:
                self._check_locked()
                if not self._inflight and not any(self._queues.values()):
                    return
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("CAN transmit flush timed out")
                self._condition.wait(remaining)

    def close(self, timeout=2.0):
        if not isfinite(timeout) or timeout <= 0:
            raise ValueError("close timeout must be finite and positive")
        with self._condition:
            if not self._opened:
                return
            self._closing = True
            self._stop.set()
            self._wake.set()
        self._thread.join(timeout)
        if self._thread.is_alive():
            raise CANIOError("CAN worker did not stop within timeout")
        with self._condition:
            self._opened = False
            failure = self._failure
        if failure is not None:
            raise CANIOError(str(failure)) from failure

    def _receive(self, arm, transport):
        started_ns = time.monotonic_ns()
        rows = transport.receive()
        now = time.monotonic_ns()
        with self._condition:
            self._stats["max_rx_poll_duration_ns"] = max(self._stats["max_rx_poll_duration_ns"], now-started_ns)
        for frame, acquired_ns in rows:
            if self._performance is not None:
                self._performance.count(f"arm{arm}.rx_frames")
                self._performance.count(f"arm{arm}.rx_payload_bytes", len(frame.data))
            if acquired_ns > now:
                raise CANIOError("CAN receive timestamp is in the future")
            if now-acquired_ns > self.max_rx_age_ns:
                with self._condition:
                    self._stats["stale_rx"] += 1
                continue
            feedback = decode_feedback(arm, frame, 0)
            with self._condition:
                self._stats["rx_frames"] += 1
                if feedback is None:
                    self._stats["ignored_rx"] += 1
                    continue
                key = (arm, feedback.motor_index)
                if feedback.zero_ack is not None:
                    sent_ns = self._zero_pending.get(key)
                    if sent_ns is None or not 0 <= acquired_ns-sent_ns <= 1_500_000_000:
                        self._stats["ignored_rx"] += 1
                        continue
                    self._zero_pending.pop(key)
                    feedback = replace(feedback, error_code=self._last_errors.get(key, 0))
                else:
                    self._last_errors[key] = feedback.error_code
                if len(self._rx) >= self.rx_capacity:
                    raise CANIOError("CAN receive queue full")
                self._rx.append((acquired_ns, feedback))
                self._stats["feedback_samples"] += 1
                if self._performance is not None:
                    prefix = f"arm{arm}.motor{MOTOR_IDS[feedback.motor_index]}"
                    self._performance.tick(prefix+".rx_period", acquired_ns)
                    self._performance.count(prefix+".feedback_samples")
                    self._performance.peak("rx_queue", len(self._rx))

    def _take(self, arm, now):
        with self._condition:
            if self._stop.is_set() or now < self._next_tx[arm]:
                return None
            queue = self._queues[arm]
            for index, job in enumerate(queue):
                if job.index < 3 and now < self._guard_until[arm]:
                    continue
                if job.action in (MotorAction.CONTROL, MotorAction.ENABLE) and now-job.created_ns > self.max_tx_age_ns:
                    raise CANIOError("CAN motor request exceeded transmit deadline")
                if job.action == MotorAction.SAVE_ZERO and job.index < 3 and any(self._encos_active[arm]):
                    raise CANIOError("ENCOS zeroing requires stopped joints on this bus")
                del queue[index]
                self._inflight = True
                return job
            return None

    def _transmit(self, arm, transport, job):
        if job.action is not None:
            with self._condition:
                self._actuated.add(arm)
        started_ns = time.monotonic_ns()
        transport.send(job.frame)
        now = time.monotonic_ns()
        if self._performance is not None:
            prefix = f"arm{arm}"
            self._performance.count(prefix+".tx_frames")
            self._performance.count(prefix+".tx_payload_bytes", len(job.frame.data))
            self._performance.count(prefix+".action."+(job.action.value if job.action else "query"))
            self._performance.observe(prefix+".tx_duration", now-started_ns)
            self._performance.observe(prefix+".tx_queue_age", started_ns-job.created_ns)
            if job.action in (MotorAction.CONTROL, MotorAction.SOFT_STOP):
                self._performance.tick(prefix+f".motor{MOTOR_IDS[job.index]}.tx_period", started_ns)
        with self._condition:
            self._next_tx[arm] = started_ns+200_000
            if job.guard_after:
                self._guard_until[arm] = now+501_000_000
            if job.action == MotorAction.SAVE_ZERO and job.index < 3:
                self._zero_pending[arm, job.index] = started_ns
            if job.action is not None:
                if job.action == MotorAction.CONTROL:
                    self._active[arm, job.index] = now
                elif job.action != MotorAction.ENABLE:
                    self._active.pop((arm, job.index), None)
                if job.index < 3:
                    self._encos_active[arm][job.index] = job.action == MotorAction.CONTROL
                self._stats["motor_tx_frames"] += 1
            else:
                self._stats["query_tx_frames"] += 1
            self._stats["tx_frames"] += 1
            self._stats["max_tx_duration_ns"] = max(self._stats["max_tx_duration_ns"], now-started_ns)
            self._inflight = False
            self._condition.notify_all()

    def _wait_delay(self):
        now = time.monotonic_ns()
        due = now+500_000
        with self._condition:
            for arm, queue in self._queues.items():
                for job in queue:
                    eligible = max(self._next_tx[arm], self._guard_until[arm] if job.index < 3 else 0)
                    due = min(due, eligible)
                    if eligible == self._next_tx[arm]:
                        break
        return max(0, due-now)/1e9

    def _stop_motors(self):
        jobs = {arm: deque((i, MotorAction.SOFT_STOP if i < 3 else MotorAction.DISABLE)
                           for i in (3, 4, 5, 6, 0, 1, 2)) for arm in self._actuated}
        errors = []
        while any(jobs.values()):
            for arm, queue in jobs.items():
                now = time.monotonic_ns()
                if not queue or now < self._next_tx[arm]:
                    continue
                i, action = queue[0]
                if i < 3 and now < self._guard_until[arm]:
                    continue
                queue.popleft()
                request = MotorRequest(arm, i, MOTOR_IDS[i], "encos" if i < 3 else "dm", action, 0)
                try:
                    frame = encode_request(request)
                    self._transports[arm].send(frame)
                    if self._performance is not None:
                        self._performance.count(f"arm{arm}.tx_frames")
                        self._performance.count(f"arm{arm}.tx_payload_bytes", len(frame.data))
                        self._performance.count(f"arm{arm}.shutdown_tx_frames")
                    with self._condition:
                        self._stats["shutdown_tx_frames"] += 1
                except Exception as error:
                    errors.append(error)
                self._next_tx[arm] = time.monotonic_ns()+200_000
            time.sleep(.0002)
        if errors:
            raise CANIOError(f"CAN shutdown delivery failed: {errors[0]}") from errors[0]

    def _worker(self):
        previous = time.monotonic_ns()
        try:
            while not self._stop.is_set():
                now = time.monotonic_ns()
                if self._performance is not None:
                    self._performance.tick("can_worker.period", now)
                if now-previous > self.max_rx_age_ns:
                    raise CANIOError("CAN receive worker missed its deadline")
                if any(now-last > self.command_timeout_ns for last in self._active.values()):
                    raise CANIOError("CAN motor command stream timed out")
                previous = now
                for arm, transport in self._transports.items():
                    self._receive(arm, transport)
                    job = self._take(arm, time.monotonic_ns())
                    if job is not None:
                        self._transmit(arm, transport, job)
                self._wake.wait(self._wait_delay())
                self._wake.clear()
        except Exception as error:
            with self._condition:
                self._failure = self._failure or error
            _LOG.error("can.failed", error=error)
        finally:
            try:
                if self._actuated and not self.read_only:
                    self._stop_motors()
            except Exception as error:
                with self._condition:
                    self._failure = self._failure or error
                _LOG.error("can.shutdown_failed", error=error)
            finally:
                for transport in self._transports.values():
                    try:
                        transport.close()
                    except Exception as error:
                        with self._condition:
                            self._failure = self._failure or error
                with self._condition:
                    cancelled = sum(map(len, self._queues.values()))
                    self._stats["cancelled_tx"] += cancelled
                    for queue in self._queues.values():
                        queue.clear()
                    self._inflight = False
                    self._closing = True
                    self._condition.notify_all()


class CANControlIO(CANMotorIO):
    """为 runtime.run 提供命令输入和反馈、事件回调。"""

    def __init__(self, channels, *, command_source=None, feedback_sink=None, event_sink=None, **options):
        for callback in (command_source, feedback_sink, event_sink):
            if callback is not None and not callable(callback):
                raise TypeError("ControlIO callbacks must be callable")
        super().__init__(channels, **options)
        self._command_source, self._feedback_sink, self._event_sink = command_source, feedback_sink, event_sink

    def poll_commands(self, now_ms):
        return tuple(self._command_source(now_ms)) if self._command_source is not None else ()

    def publish_feedback(self, feedback):
        if self._feedback_sink is not None:
            self._feedback_sink(feedback)

    def publish_events(self, events):
        if self._event_sink is not None:
            self._event_sink(events)
