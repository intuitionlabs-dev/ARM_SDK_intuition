"""Deterministic scheduler replacing FreeRTOS tasks, without transport IO."""

from .arm import Arm
from .config import ArmConfig
from .types import (ArmMode, Command, Event, MotorAction, MotorFeedback,
                    TickResult, timestamp)


class ControlSystem:
    """Call step once per monotonic millisecond from one owning thread.

    Input validation is atomic. Commands execute at 2 ms intervals, motor
    algorithms on their original phase masks, feedback at 4 ms intervals.
    Late calls run once at the current time, without a burst of stale outputs.
    """

    def __init__(self, configurations=None):
        configs = tuple(configurations) if configurations is not None else (ArmConfig(), ArmConfig())
        if not 1 <= len(configs) <= 2 or not all(isinstance(c, ArmConfig) for c in configs):
            raise ValueError("provide one or two ArmConfig objects")
        self.arms = tuple(Arm(i, c) for i, c in enumerate(configs))
        self.last_ms = None
        self.command_last_ms = None
        self.pending = [[] for _ in configs]
        self.missed_motor_ticks = self.missed_command_ticks = self.stale_feedback_count = 0
        self.closed = False

    def _arm_index(self, arm_id):
        if type(arm_id) is not int or not 0 <= arm_id < len(self.arms):
            raise ValueError(f"arm_id must be in 0..{len(self.arms)-1}")

    def step(self, now_ms, *, commands=(), motor_feedback=()):
        timestamp(now_ms)
        if self.closed:
            raise RuntimeError("controller is shut down")
        if self.last_ms is not None and now_ms <= self.last_ms:
            raise ValueError("step timestamps must strictly increase")
        commands, motor_feedback = tuple(commands), tuple(motor_feedback)
        for item in commands:
            if not isinstance(item, Command):
                raise TypeError("commands must contain Command objects")
            self._arm_index(item.arm_id)
        for item in motor_feedback:
            if not isinstance(item, MotorFeedback):
                raise TypeError("motor_feedback must contain MotorFeedback objects")
            self._arm_index(item.arm_id)
            if item.timestamp_ms > now_ms:
                raise ValueError("feedback cannot come from the future")
        if self.last_ms is not None:
            self.missed_motor_ticks += max(0, now_ms - self.last_ms - 1)
        self.last_ms = now_ms
        output, feedback, events = [], [], []
        for item in motor_feedback:
            if not self.arms[item.arm_id].bank.motors[item.motor_index].accept(item):
                self.stale_feedback_count += 1
        for command in commands:
            queue = self.pending[command.arm_id]
            if len(queue) < 7:  # H7 ring: 8 slots, one reserved.
                queue.append(command)
            else:
                self.arms[command.arm_id].reject(now_ms, command, "command queue full")
        # 标定流程独占参与电机的输出，其余电机维持控制周期。
        maintenance_active = []
        for arm in self.arms:
            was_active = arm.zero.busy or arm.zero.pending
            output.extend(arm.zero.tick(now_ms))
            maintenance_active.append(arm.zero.mask if was_active or arm.zero.busy else 0)
        if self.command_last_ms is None or now_ms - self.command_last_ms >= 2:
            if self.command_last_ms is not None:
                self.missed_command_ticks += max(0, (now_ms-self.command_last_ms)//2 - 1)
            self.command_last_ms = now_ms
            for arm, queue in zip(self.arms, self.pending):
                arm.refresh_feedback()
                for command in queue:
                    arm.handle(command, now_ms)
                queue.clear()
                arm.state_step(now_ms)
                arm.command.update(now_ms)
                arm.drive(now_ms)
                if not arm.connected and now_ms >= arm.handshake_due_ms:
                    arm.event(now_ms, "connection_required", "API version 1")
                    arm.handshake_due_ms = now_ms + 100
                elif arm.connected and now_ms >= arm.feedback_due_ms:
                    feedback.append(arm.snapshot(now_ms))
                    arm.feedback_due_ms = now_ms + 4
        for arm, maintenance in zip(self.arms, maintenance_active):
            output.extend(arm._maintenance)
            arm._maintenance.clear()
            if arm.zero.pending:
                maintenance |= arm.zero.request_mask or (0x3f if arm.zero.include_encos else 0x38)
            if arm.zero.busy:
                maintenance |= arm.zero.mask
            output.extend(arm.bank.tick(now_ms, excluded_mask=maintenance))
            events.extend(arm._events)
            arm._events.clear()
        return TickResult(tuple(output), tuple(feedback), tuple(events))

    def shutdown(self, now_ms=None):
        """Return final stop requests; the caller must deliver them to its adapter."""
        if self.closed:
            return TickResult()
        now = (self.last_ms or 0) if now_ms is None else timestamp(now_ms)
        if self.last_ms is not None and now < self.last_ms:
            raise ValueError("shutdown timestamp precedes last step")
        output = []
        for arm in self.arms:
            arm.command.cancel()
            output.extend(arm.zero.cancel(now))
            arm.control = None
            arm.mode = arm.actual_mode = ArmMode.STOP
            for motor in arm.bank.motors:
                motor.mode = ArmMode.STOP
                # Explicit stop intents are returned even during a setting guard.
                # The adapter must deliver them after the required guard expires.
                output.append(motor.request(MotorAction.SOFT_STOP, now))
                if not motor.encos:
                    output.append(motor.request(MotorAction.DISABLE, now))
        self.closed = True
        return TickResult(tuple(output), events=(Event(-1, now, "shutdown"),))
