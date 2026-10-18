"""通过独立进程隔离 CAN 收发与控制计算。"""

from collections import deque
from math import isfinite
import multiprocessing
from queue import Empty, Full
import threading
import time

from .can_io import CANIOError, CANMotorIO
from .can_protocol import encode_request
from .types import timestamp


def _serve(channels, options, commands, replies, stop):
    io = None
    failure = None
    stats = {}
    try:
        io = CANMotorIO(channels, **options).open()
        replies.put(("ready", None), timeout=1)
        origin = None
        heartbeat = time.monotonic_ns()
        next_stats = 0
        next_performance = 0
        flushes = deque()
        while not stop.is_set():
            for _ in range(64):
                try:
                    kind, value = commands.get_nowait()
                except Empty:
                    break
                if kind == "anchor":
                    origin = value
                elif kind == "heartbeat":
                    heartbeat = value
                elif kind == "write":
                    io.write_motor_requests(value)
                elif kind == "query":
                    io.query_feedback(value)
                elif kind == "flush":
                    serial, timeout = value
                    flushes.append((serial, time.monotonic()+timeout))
            now = time.monotonic_ns()
            if io._performance is not None:
                io._performance.tick("can_service.period", now)
            if now-heartbeat > 100_000_000:
                raise CANIOError("CAN controller process heartbeat timed out")
            io.check_health()
            if flushes:
                try:
                    io.flush(.001)
                except TimeoutError:
                    if time.monotonic() >= flushes[0][1]:
                        raise TimeoutError("CAN process flush timed out")
                else:
                    replies.put_nowait(("stats", io.stats()))
                    while flushes:
                        replies.put_nowait(("flushed", flushes.popleft()[0]))
            if origin is not None:
                feedback = io.poll_feedback(max(0, (now-origin)//1_000_000))
                if feedback:
                    replies.put_nowait(("feedback", feedback))
            if now >= next_stats:
                replies.put_nowait(("stats", io.stats()))
                next_stats = now+20_000_000
            if now >= next_performance:
                snapshot = io.performance_snapshot()
                if snapshot is not None:
                    replies.put_nowait(("performance", snapshot))
                next_performance = now+1_000_000_000
            stop.wait(.0005)
    except BaseException as error:
        failure = str(error)
    finally:
        if io is not None:
            try:
                io.close()
            except Exception as error:
                failure = failure or str(error)
            stats = io.stats()
            snapshot = io.performance_snapshot()
            if snapshot is not None:
                try:
                    replies.put(("performance", snapshot), timeout=1)
                except (Full, OSError):
                    pass
        try:
            replies.put(("closed", (failure, stats)), timeout=1)
            replies.close()
            replies.join_thread()
        except (Full, OSError):
            pass


class ProcessCANMotorIO:
    """由独立进程持有 CANMotorIO，通过有界队列交换请求和反馈。

    poll_feedback 按实际单调时钟周期调用，并提供进程保活。
    spawn 启动要求程序入口使用 __main__ 保护。
    """

    def __init__(self, channels, *, ipc_capacity=256, **options):
        if type(ipc_capacity) is not int or ipc_capacity <= 0:
            raise ValueError("ipc_capacity must be a positive integer")
        probe = CANMotorIO(channels, **options)
        options = dict(options, transport_config=probe._registry)
        self.channels = probe.channels
        self.read_only = probe.read_only
        self._rx_capacity = probe.rx_capacity
        self._max_rx_age_ms = probe.max_rx_age_ns/1_000_000
        context = multiprocessing.get_context("spawn")
        self._commands = context.Queue(ipc_capacity)
        self._replies = context.Queue(ipc_capacity)
        self._stop = context.Event()
        self._process = context.Process(target=_serve, name="arx-can-process", daemon=False,
            args=(self.channels, options, self._commands, self._replies, self._stop))
        self._lock = threading.RLock()
        self._feedback = deque()
        self._stats = {}
        self._failure = None
        self._ready = self._closed = self._used = self._opened = False
        self._last_poll = self._origin = None
        self._heartbeat = 0
        self._serial = self._flushed = 0
        self._ipc_stale_rx = 0
        self._queues_closed = False
        self._performance = deque(maxlen=64)
        self._performance_dropped = 0

    def _drain(self):
        if self._queues_closed:
            return
        while True:
            try:
                kind, value = self._replies.get_nowait()
            except Empty:
                return
            if kind == "ready":
                self._ready = True
            elif kind == "feedback":
                self._feedback.extend(value)
                if len(self._feedback) > self._rx_capacity:
                    self._failure = "CAN process receive queue full"
                    self._stop.set()
            elif kind == "stats":
                self._stats = value
            elif kind == "performance":
                if len(self._performance) == self._performance.maxlen:
                    self._performance_dropped += 1
                self._performance.append(value)
            elif kind == "flushed":
                self._flushed = value
            elif kind == "closed":
                self._closed = True
                failure, self._stats = value
                self._failure = self._failure or failure

    def _check(self):
        self._drain()
        if self._failure:
            raise CANIOError(self._failure)
        if not self._opened or self._closed or not self._process.is_alive():
            raise CANIOError("CAN worker process is closed")

    def _send(self, kind, value=None):
        self._check()
        try:
            self._commands.put_nowait((kind, value))
        except Full as error:
            self._failure = "CAN process transmit queue full"
            self._stop.set()
            raise CANIOError(self._failure) from error

    def _keepalive(self, now):
        if now-self._heartbeat >= 10_000_000:
            self._send("heartbeat", now)
            self._heartbeat = now

    def open(self, timeout=5.0):
        if not isfinite(timeout) or timeout <= 0:
            raise ValueError("open timeout must be finite and positive")
        if self._used:
            raise RuntimeError("ProcessCANMotorIO instances can be opened once")
        self._used = self._opened = True
        self._process.start()
        deadline = time.monotonic()+timeout
        try:
            while time.monotonic() < deadline:
                with self._lock:
                    self._check()
                    if self._ready:
                        return self
                time.sleep(.001)
            raise TimeoutError("CAN worker process did not open within timeout")
        except BaseException:
            self._stop.set()
            self.close()
            raise

    def __enter__(self):
        return self.open()

    def __exit__(self, exc_type, exc, traceback):
        self.close()

    def check_health(self):
        with self._lock:
            self._check()

    def write_motor_requests(self, requests):
        requests = tuple(requests)
        for request in requests:
            encode_request(request)
            if request.arm_id not in {channel.arm_id for channel in self.channels}:
                raise ValueError("motor request refers to an unconfigured arm")
        if self.read_only and requests:
            raise PermissionError("read-only CAN adapter accepts feedback queries only")
        with self._lock:
            if requests:
                self._send("write", requests)
            else:
                self._check()

    def query_feedback(self, arm_id=None):
        with self._lock:
            self._send("query", arm_id)

    def poll_feedback(self, now_ms):
        timestamp(now_ms)
        with self._lock:
            self._check()
            if self._last_poll is not None and now_ms < self._last_poll:
                raise ValueError("feedback clock must be monotonic")
            self._last_poll = now_ms
            now = time.monotonic_ns()
            if self._origin is None:
                self._origin = now-now_ms*1_000_000
                self._send("anchor", self._origin)
            self._keepalive(now)
            output = []
            retained = deque()
            for item in self._feedback:
                if item.timestamp_ms > now_ms:
                    retained.append(item)
                elif now_ms-item.timestamp_ms <= self._max_rx_age_ms:
                    output.append(item)
                else:
                    self._ipc_stale_rx += 1
            self._feedback = retained
            return tuple(output)

    def stats(self):
        with self._lock:
            self._drain()
            return dict(self._stats, worker_process=True, process_alive=self._process.is_alive(),
                        pending_ipc_rx=len(self._feedback), ipc_stale_rx=self._ipc_stale_rx,
                        failed=bool(self._failure), error=self._failure,
                        worker_pid=self._process.pid, performance_dropped=self._performance_dropped)

    def pop_performance(self):
        with self._lock:
            self._drain()
            rows = tuple(self._performance)
            self._performance.clear()
            return rows

    def flush(self, timeout=2.0):
        if not isfinite(timeout) or timeout <= 0:
            raise ValueError("flush timeout must be finite and positive")
        with self._lock:
            self._serial += 1
            serial = self._serial
            self._send("flush", (serial, timeout))
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            with self._lock:
                self._check()
                self._keepalive(time.monotonic_ns())
                if self._flushed >= serial:
                    return
            time.sleep(.001)
        raise TimeoutError("CAN process flush timed out")

    def close(self, timeout=3.0):
        if not isfinite(timeout) or timeout <= 0:
            raise ValueError("close timeout must be finite and positive")
        if not self._opened:
            return
        self._stop.set()
        deadline = time.monotonic()+timeout
        while self._process.is_alive() and time.monotonic() < deadline:
            with self._lock:
                self._drain()
            self._process.join(.005)
        with self._lock:
            self._drain()
            if self._process.is_alive():
                raise CANIOError("CAN worker process did not stop within timeout")
            self._opened = False
            self._commands.cancel_join_thread()
            self._commands.close()
            self._replies.close()
            self._queues_closed = True
            if self._failure:
                raise CANIOError(self._failure)
