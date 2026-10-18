"""使用 CANMotorIO 读取电机反馈并输出连通性报告。"""

from collections import Counter
from math import isfinite
import time

from . import log
from .can_io import CANChannel, CANMotorIO
from .config import ArmConfig, MOTOR_IDS
from .system import ControlSystem
from .types import ArmMode

_LOG = log.get_logger("arx_x5_control.can_check")


def check(channels, *, duration_s=10.0, transport_config=None):
    if not isfinite(duration_s) or duration_s <= 0:
        raise ValueError("duration must be finite and positive")
    channels = tuple(channels)
    if tuple(sorted(c.arm_id for c in channels)) != tuple(range(len(channels))):
        raise ValueError("diagnostic arm identifiers must start at zero")
    controller = ControlSystem([ArmConfig(gravity_enabled=False)] * len(channels))
    samples = {(c.arm_id, i): [] for c in channels for i in range(7)}
    blocked = Counter()
    with CANMotorIO(channels, read_only=True, transport_config=transport_config) as io:
        origin = time.monotonic_ns()
        next_query = 0
        last_ms = -1
        try:
            while time.monotonic_ns()-origin < duration_s * 1e9:
                now_ms = (time.monotonic_ns()-origin)//1_000_000
                if now_ms > last_ms:
                    feedback = io.poll_feedback(now_ms)
                    for item in feedback:
                        samples[item.arm_id, item.motor_index].append(item)
                    result = controller.step(now_ms, motor_feedback=feedback)
                    blocked.update(item.action.value for item in result.motor_requests)
                    assert all(arm.actual_mode == ArmMode.STOP for arm in controller.arms)
                    if now_ms >= next_query and io.stats()["pending_tx"] == 0:
                        io.query_feedback()
                        next_query = now_ms+5000
                    last_ms = now_ms
                time.sleep(.001)
            stats = io.stats()
        finally:
            blocked.update(item.action.value for item in controller.shutdown().motor_requests)
    report = {"read_only": True, "duration_s": duration_s, "motor_tx_frames": stats.get("motor_tx_frames", 0),
              "io_stats": io.stats(), "core_output_blocked": dict(blocked), "arms": []}
    for channel in channels:
        motors = []
        for i, motor_id in enumerate(MOTOR_IDS):
            values = samples[channel.arm_id, i]
            positions = [v.position for v in values if v.position is not None]
            velocities = [v.velocity for v in values if v.velocity is not None]
            motors.append(dict(motor_index=i, motor_id=motor_id, replies=len(values),
                position_rad=positions[-1] if positions else None,
                position_span_rad=max(positions)-min(positions) if positions else None,
                velocity_rad_s=velocities[-1] if velocities else None,
                error_codes=sorted({v.error_code for v in values})))
        report["arms"].append(dict(arm_id=channel.arm_id, channel=channel.channel, motors=motors))
    report["all_motors_responded"] = all(v["replies"] > 0 for arm in report["arms"] for v in arm["motors"])
    return report


def main(argv=None):
    parser = log.argument_parser("can_check.description", argv)
    parser.add_argument("--channel", action="append", required=True, help=log.text("can_check.channel"))
    parser.add_argument("--interface", default="slcan",
                        help=log.text("can_check.interface"))
    parser.add_argument("--transport-config", help=log.text("can.transport_config"))
    parser.add_argument("--seconds", type=float, default=10, help=log.text("can_check.seconds"))
    args = parser.parse_args(argv)
    if not 1 <= len(args.channel) <= 2:
        parser.error(log.text("can_check.channel_count"))
    if not isfinite(args.seconds) or args.seconds <= 0:
        parser.error(log.text("cli.invalid_seconds"))
    try:
        report = check([CANChannel(i, path, args.interface) for i, path in enumerate(args.channel)],
                       duration_s=args.seconds, transport_config=args.transport_config)
        log.write_data(report)
        return 0 if report["all_motors_responded"] else 2
    except KeyboardInterrupt:
        _LOG.info("runtime.interrupted")
        return 130
    except Exception as error:
        _LOG.error("can.failed", error=error)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
