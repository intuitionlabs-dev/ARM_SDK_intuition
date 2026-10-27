"""控制周期的有界二进制诊断记录。"""

from collections import deque
import struct
import threading
import time

from .types import MotorAction


TRACE_STRUCT = struct.Struct("<QQ7B7H7B38d")
TRACE_SCALARS = ["monotonic_ns", "controller_ms", "arm_id", "robot_state", "requested_mode",
                 "actual_mode", "lost_mask", "soft_stop_mask", "control_mask"]
TRACE_GROUPS = [("feedback_age_ms", 7), ("motor_error_code", 7), ("target_deg", 6),
                ("reference_position_deg", 6), ("reference_velocity_deg_s", 6),
                ("reference_acceleration_deg_s2", 6), ("feedback_position_deg", 7),
                ("feedback_velocity_deg_s", 7)]


def trace_schema():
    return dict(schema_version=1, byte_order="little", struct_format=TRACE_STRUCT.format,
                record_bytes=TRACE_STRUCT.size, target_period_ms=1,
                fields=TRACE_SCALARS+[f"{name}[{i}]" for name, count in TRACE_GROUPS for i in range(count)],
                never_received_age_ms=65535, mask_bit_order="joint1..joint6,gripper",
                angle_unit="degree", velocity_unit="degree/second",
                acceleration_unit="degree/second^2")


class ControlTrace:
    def __init__(self, capacity=8192):
        self.capacity = capacity
        self._rows = deque()
        self._lock = threading.Lock()
        self.captured = self.dropped = 0
        self.error = None

    def capture(self, controller, result, now_ms):
        if self.error is not None:
            return
        try:
            modes = [[0, 0] for _ in controller.arms]
            for request in result.motor_requests:
                if request.action == MotorAction.SOFT_STOP:
                    modes[request.arm_id][0] |= 1 << request.motor_index
                elif request.action == MotorAction.CONTROL:
                    modes[request.arm_id][1] |= 1 << request.motor_index
            captured_ns = time.monotonic_ns()
            rows = []
            for arm in controller.arms:
                motors, command = arm.bank.motors, arm.command
                ages = tuple(65535 if m.last_rx_ms < 0 else min(65534, max(0, now_ms-m.last_rx_ms))
                             for m in motors)
                lost = sum(1 << i for i, motor in enumerate(motors) if not motor.online(now_ms))
                rows.append(TRACE_STRUCT.pack(captured_ns, now_ms, arm.id, int(arm.state), int(arm.mode),
                    int(arm.actual_mode), lost, *modes[arm.id], *ages,
                    *(m.error_code & 255 for m in motors),
                    *command.source, *command.angle, *command.velocity, *command.acceleration,
                    *arm.joint_angle, arm.gripper_angle,
                    *arm.joint_velocity, motors[6].velocity_deg))
            with self._lock:
                for row in rows:
                    if len(self._rows) == self.capacity:
                        self._rows.popleft()
                        self.dropped += 1
                    self._rows.append(row)
                    self.captured += 1
        except Exception as error:
            self.error = str(error)

    def take(self):
        with self._lock:
            rows, self._rows = self._rows, deque()
        return b"".join(rows)

    def stats(self):
        with self._lock:
            return dict(captured=self.captured, dropped=self.dropped,
                        pending=len(self._rows), error=self.error)
