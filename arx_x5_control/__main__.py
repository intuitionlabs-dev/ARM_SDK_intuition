"""通过内存中的测试电机运行完整双臂流程，日志统一使用本包 log 库。"""

from math import isfinite
import time

from . import Command, CommandKind as K, ControlSystem, RobotState as S
from . import log
from .simulation import SimulatedMotors

_LOG = log.get_logger("arx_x5_control.__main__")


def demo(seconds=12.0, realtime=False, emit=None):
    """运行模拟；emit 若提供则接收原始事件和结果 JSON，供现有调用者保存数据。"""
    system, plant = ControlSystem(), SimulatedMotors()
    stage = [0, 0]
    stage_ms = [0, 0]
    origin = time.monotonic()
    try:
        for now in range(int(seconds * 1000)):
            if realtime:
                delay = origin + now / 1000 - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
            commands = []
            for i, arm in enumerate(system.arms):
                if now == 0:
                    commands.append(Command(i, K.CONNECT))
                elif now % 500 == 0:
                    commands.append(Command(i, K.HEARTBEAT))
                if now % 2:
                    continue
                if stage[i] == 0 and arm.state == S.WAIT_INIT:
                    commands.append(Command(i, K.INITIALIZE))
                    stage[i] = 1
                elif stage[i] == 1 and arm.state == S.NORMAL:
                    commands.extend((Command(i, K.START), Command(i, K.POSITION,
                                     (10, 15, 20, 5, -5, 10), -30)))
                    stage[i], stage_ms[i] = 2, now
                elif stage[i] == 2 and now - stage_ms[i] >= 1000:
                    commands.append(Command(i, K.GRAVITY))
                    stage[i], stage_ms[i] = 3, now
                elif stage[i] == 3 and now - stage_ms[i] >= 500:
                    commands.append(Command(i, K.DAMPING))
                    stage[i], stage_ms[i] = 4, now
                elif stage[i] == 4 and now - stage_ms[i] >= 500:
                    commands.append(Command(i, K.RESET))
                    stage[i] = 5
            result = system.step(now, commands=commands, motor_feedback=plant.feedback(now))
            plant.apply(result.motor_requests)
            for event in result.events:
                _LOG.event(event)
                if emit is not None:
                    log.write_data(event, sink=emit)
        summary = {"simulated": True, "duration_s": seconds,
                   "sequence_complete": all(s == 5 and a.state == S.NORMAL for s, a in zip(stage, system.arms)),
                   "arms": [{"arm_id": a.id, "state": a.state.name, "mode": a.actual_mode.name,
                             "gripper_initialized": a.initialized[6]} for a in system.arms]}
        log.write_data(summary, sink=emit)
        return summary
    finally:
        final = system.shutdown()
        plant.apply(final.motor_requests)
        _LOG.events(final.events)


def main(argv=None):
    parser = log.argument_parser("cli.description", argv)
    parser.add_argument("--demo", action="store_true", help=log.text("cli.demo_help"))
    parser.add_argument("--seconds", type=float, default=12.0, help=log.text("cli.seconds_help"))
    parser.add_argument("--realtime", action="store_true", help=log.text("cli.realtime_help"))
    args = parser.parse_args(argv)
    if not isfinite(args.seconds) or args.seconds <= 0:
        parser.error(log.text("cli.invalid_seconds"))
    try:
        demo(args.seconds, args.realtime)
    except KeyboardInterrupt:
        _LOG.info("runtime.interrupted")
        return 130
    except Exception as exc:
        _LOG.error("cli.failed", error=exc)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
