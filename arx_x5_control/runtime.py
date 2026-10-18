"""可选的 PC 单调时钟循环，日志使用本包 log 库，不保证硬实时调度。"""

import time

from .interfaces import ControlIO
from .system import ControlSystem
from . import log

_LOG = log.get_logger(__name__)


def run(system: ControlSystem, io: ControlIO, *, stop=None, duration_s=None):
    """运行至停止信号、指定时长或键盘中断，退出时始终交付停机请求。

    需要尚未运行的控制器。适配器收到相对此次调用的时间戳，设备采样时间
    也必须转换到同一时钟；日志配置只作用于 arx_x5_control 包。
    """
    if system.last_ms is not None or system.closed:
        raise ValueError("run requires a fresh controller")
    if duration_s is not None and (not 0 < duration_s < float("inf")):
        raise ValueError("duration_s must be finite and positive")
    origin = time.monotonic_ns()
    deadline = origin
    _LOG.info("runtime.started")
    try:
        while stop is None or not stop.is_set():
            now_ns = time.monotonic_ns()
            if duration_s is not None and now_ns - origin >= duration_s * 1e9:
                break
            if now_ns < deadline:
                time.sleep((deadline - now_ns) / 1e9)
                continue
            now_ms = (now_ns - origin) // 1_000_000
            if system.last_ms is None or now_ms > system.last_ms:
                result = system.step(now_ms, commands=io.poll_commands(now_ms),
                                     motor_feedback=io.poll_feedback(now_ms))
                io.write_motor_requests(result.motor_requests)
                io.publish_feedback(result.feedback)
                io.publish_events(result.events)
                _LOG.events(result.events)
            deadline = origin + (now_ms + 1) * 1_000_000
    except KeyboardInterrupt:
        _LOG.info("runtime.interrupted")
    except Exception as exc:
        _LOG.error("runtime.failed", error=exc)
        raise
    finally:
        result = system.shutdown((time.monotonic_ns() - origin) // 1_000_000)
        try:
            io.write_motor_requests(result.motor_requests)
            io.publish_events(result.events)
            _LOG.events(result.events)
        except Exception as exc:
            _LOG.error("runtime.shutdown_failed", error=exc)
            raise
