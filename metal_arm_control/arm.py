"""One independent arm: robot state machine, command layer and motor control."""

from math import isfinite, radians

from .gravity_compensation import DynamicsCompensator, DynamicsError, DynamicsStatus

from .calibration import ZeroCalibration
from .command import CommandLayer
from .config import ArmConfig, RESET_ANGLE
from .motor import MotorBank
from .types import ArmFeedback, ArmMode, CommandKind, Event, RobotState


STATE_MODE = {
    RobotState.WAIT_HANDSHAKE: ArmMode.STOP, RobotState.OFFLINE_WAIT: ArmMode.STOP,
    RobotState.WAIT_INIT: ArmMode.STOP, RobotState.INITIALIZING: ArmMode.RUN,
    RobotState.NORMAL: ArmMode.DAMPING, RobotState.COMM_LOST: ArmMode.DAMPING,
    RobotState.CALIBRATING: ArmMode.STOP, RobotState.RESETTING: ArmMode.RUN,
}
MODE_COMMANDS = {CommandKind.START: ArmMode.RUN, CommandKind.STOP: ArmMode.STOP,
                 CommandKind.DAMPING: ArmMode.DAMPING, CommandKind.GRAVITY: ArmMode.GRAVITY}


class Arm:
    def __init__(self, arm_id, config=None):
        self.id, self.config = arm_id, config or ArmConfig()
        self.bank = MotorBank(arm_id)
        self.zero = ZeroCalibration(self.bank, self.config.calibrate_encos)
        self.dynamics = DynamicsCompensator.from_model(self.config.gravity_model,
                                                       gravity=self.config.gravity_base)
        self.gravity = self.dynamics
        self.state = RobotState.WAIT_HANDSHAKE
        self.mode = self.actual_mode = ArmMode.STOP
        self.connected = False
        self.last_rx_ms = 0
        self.feedback_due_ms = 0
        self.handshake_due_ms = 0
        self.initialized = [False] * 7
        self.first_init = False
        self.joint_angle = [0.0] * 6
        self.joint_velocity = [0.0] * 6
        self.gripper_angle = 0.0
        self.gravity_torque = (0.0,) * 6
        self.dynamics_torque = (0.0,) * 6
        self.dynamics_status = DynamicsStatus.OK
        self.control = None
        self.command = CommandLayer(self)
        self.init_entry = RobotState.WAIT_INIT
        self.init_step = self.reset_step = 0
        self.zero_seen_busy = False
        self.zero_started_ms = 0
        self.transition_count = self.control_count = 0
        self.rejected_count = 0
        self._events = []
        self._maintenance = []

    def event(self, now, kind, detail="", **values):
        self._events.append(Event(self.id, now, kind, detail, **values))

    def enter(self, state, now):
        self.control = None
        self.state, self.mode = state, STATE_MODE[state]
        self.feedback_due_ms = now
        self.transition_count += 1
        self.event(now, "state", state.name)

    def all_online(self, now):
        return all(m.online(now) for m in self.bank.motors)

    def refresh_feedback(self):
        c = self.config
        self.joint_angle = [(m.position_deg - c.position_zero_deg[i]) * c.direction[i] / c.reduction[i]
                            for i, m in enumerate(self.bank.motors[:6])]
        self.joint_velocity = [m.velocity_deg * c.direction[i] / c.reduction[i]
                               for i, m in enumerate(self.bank.motors[:6])]
        self.gripper_angle = self.bank.motors[6].position_deg

    def reject(self, now, command, reason):
        self.rejected_count += 1
        self.event(now, "rejected", f"{command.kind.value}: {reason}")

    def handle(self, command, now):
        kind = command.kind
        if not self.connected and kind != CommandKind.CONNECT:
            self.reject(now, command, "connection required")
            return
        if kind == CommandKind.CONNECT and command.api_version != 1:
            self.reject(now, command, "unsupported API version")
            return
        self.last_rx_ms = now
        if self.state == RobotState.COMM_LOST:
            self.enter(RobotState.OFFLINE_WAIT, now)
        if kind == CommandKind.CONNECT:
            if not self.connected:
                self.connected = True
                self.enter(RobotState.OFFLINE_WAIT, now)
            self.event(now, "connected", "API version 1")
        elif kind == CommandKind.HEARTBEAT:
            pass
        elif kind == CommandKind.INITIALIZE:
            allowed = self.state == RobotState.WAIT_INIT or (
                self.state == RobotState.NORMAL and self.mode in (ArmMode.STOP, ArmMode.DAMPING))
            if not allowed or not self.all_online(now):
                self.reject(now, command, "requires WAIT_INIT or passive NORMAL and all motors online")
            elif any(abs(a-b) > 20 for a, b in zip(self.joint_angle, RESET_ANGLE)):
                self.reject(now, command, "initial pose exceeds 20 degree tolerance")
            else:
                self.init_entry, self.init_step = self.state, 0
                self.enter(RobotState.INITIALIZING, now)
        elif kind == CommandKind.CALIBRATE:
            if self.state != RobotState.WAIT_INIT or not self.zero.request():
                self.reject(now, command, "requires WAIT_INIT and idle calibration")
            else:
                self.zero_seen_busy = False
                self.zero_started_ms = now
                self.enter(RobotState.CALIBRATING, now)
        elif kind == CommandKind.RESET:
            if self.state != RobotState.NORMAL or not self.all_online(now):
                self.reject(now, command, "requires NORMAL and all motors online")
            else:
                self.reset_step = 0
                self.enter(RobotState.RESETTING, now)
        elif kind in MODE_COMMANDS:
            if self.state != RobotState.NORMAL:
                self.reject(now, command, "requires NORMAL")
            else:
                self.control = None
                self.mode = MODE_COMMANDS[kind]
        elif kind in (CommandKind.POSITION, CommandKind.POSITION_VELOCITY):
            if self.state != RobotState.NORMAL or self.mode != ArmMode.RUN:
                self.reject(now, command, "requires NORMAL/RUN")
            else:
                self.control = command
                self.control_count += 1

    def _abort_auto(self, now, reason):
        self.command.cancel()
        self._maintenance.extend(self.zero.cancel(now))
        self.init_step = self.reset_step = 0
        self.event(now, "auto_aborted", reason)

    def state_step(self, now):
        if self.connected and self.state != RobotState.COMM_LOST and now - self.last_rx_ms >= 3000:
            if self.state != RobotState.NORMAL:
                self._abort_auto(now, "command watchdog expired")
                self.enter(RobotState.COMM_LOST, now)
            elif self.mode != ArmMode.DAMPING:
                self.enter(RobotState.NORMAL, now)
                self.event(now, "watchdog", "command timeout: damping")
        if self.state == RobotState.OFFLINE_WAIT and self.all_online(now):
            self.enter(RobotState.WAIT_INIT, now)
        elif self.state in (RobotState.INITIALIZING, RobotState.RESETTING):
            if not self.zero.busy and not self.all_online(now):
                previous = self.state
                self._abort_auto(now, "motor feedback lost")
                self.enter(self.init_entry if previous == RobotState.INITIALIZING else RobotState.NORMAL, now)
                return
            if self.state == RobotState.INITIALIZING:
                if self.init_step == 0:
                    self.command.start_auto("reset")
                    self.init_step = 1
                elif self.init_step == 1 and self.command.auto is None:
                    self.command.start_auto("gripper_zero")
                    self.init_step = 2
                elif self.init_step == 2 and self.command.auto is None:
                    ok = self.initialized[6] and not self.command.auto_failed
                    self.event(now, "initialized" if ok else "initialization_failed")
                    self.enter(RobotState.NORMAL if ok else self.init_entry, now)
                    self.init_step = 0
            elif self.reset_step == 0:
                self.command.start_auto("reset")
                self.reset_step = 1
            elif self.command.auto is None:
                self.enter(RobotState.NORMAL, now)
                self.event(now, "reset_complete")
                self.reset_step = 0
        elif self.state == RobotState.CALIBRATING:
            if not self.zero_seen_busy:
                if self.zero.busy:
                    self.zero_seen_busy = True
                elif now - self.zero_started_ms >= 200:
                    self.zero.request_mask = None
                    self._zero_ack(now)
            elif not self.zero.busy:
                self._zero_ack(now)

    def _zero_ack(self, now):
        self.event(now, "calibrated", zero_state=self.zero.state,
                   zero_results=tuple(self.zero.results), elapsed_ms=self.zero.elapsed_ms)
        self.enter(RobotState.WAIT_INIT, now)

    def drive(self, now):
        c, cmd = self.config, self.command
        online = self.all_online(now)
        # 六个关节的初始化位每拍置位，夹爪独立维护。
        self.initialized[:6] = [True] * 6
        if not self.first_init and online:
            self.first_init = True
        healthy = all(m.error_code in (0, 1) and m.motion_ready(now) for m in self.bank.motors)
        self.actual_mode = cmd.mode if online and healthy and self.first_init else ArmMode.STOP
        self.gravity_torque = self.dynamics_torque = (0.0,) * 6
        self.dynamics_status = DynamicsStatus.OK
        if c.gravity_enabled and self.actual_mode in (ArmMode.RUN, ArmMode.GRAVITY):
            try:
                q = tuple(radians(v) for v in self.joint_angle)
                if not all(isfinite(v) for v in q):
                    raise DynamicsError(DynamicsStatus.INVALID_ARGUMENT)
                if any(abs(v) > 128 for v in q):
                    raise DynamicsError(DynamicsStatus.ANGLE_RANGE)
                gravity = (self.gravity.sdk_torques(q) if c.gravity_sdk_scale
                           else self.gravity.raw_torques(q))
                torque = (self.dynamics.torques(q, tuple(radians(v) for v in self.joint_velocity),
                                               tuple(radians(v) for v in cmd.acceleration),
                                               sdk_scale=c.gravity_sdk_scale)
                          if self.actual_mode == ArmMode.RUN else gravity)
                if not all(isfinite(v) for v in torque):
                    raise DynamicsError(DynamicsStatus.NONFINITE_RESULT)
                self.gravity_torque, self.dynamics_torque = gravity, torque
            except DynamicsError as error:
                self.dynamics_status = error.status
                self.actual_mode = ArmMode.STOP
                self.event(now, "dynamics_error", error.status.name)
        for i, motor in enumerate(self.bank.motors):
            motor.mode = self.actual_mode
            if self.actual_mode != ArmMode.RUN:
                motor.reference_deg = motor.position_deg
                motor.speed_comp = motor.output_comp = 0.0
                if self.actual_mode == ArmMode.GRAVITY and i < 6:
                    motor.output_comp = c.direction[i] * self.dynamics_torque[i] * c.torque_scale[i]
            elif i < 6:
                motor.reference_deg = cmd.angle[i] * c.reduction[i] * c.direction[i] + c.position_zero_deg[i]
                motor.speed_comp = cmd.velocity[i] * c.reduction[i] * c.direction[i] * c.velocity_scale[i]
                damping = radians(cmd.velocity[i]) * c.viscous_damping[i]
                motor.output_comp = c.direction[i] * (self.dynamics_torque[i] + damping) * c.torque_scale[i]
            else:
                motor.reference_deg = cmd.gripper_angle
                motor.output_comp = cmd.gripper_torque

    def snapshot(self, now):
        return ArmFeedback(self.id, now, self.state, self.actual_mode, self.mode,
                           tuple(self.joint_angle), tuple(self.joint_velocity), self.gripper_angle,
                           tuple(not m.online(now) for m in self.bank.motors),
                           tuple(self.initialized), tuple(m.temperature for m in self.bank.motors),
                           tuple(self.gravity_torque), tuple(self.dynamics_torque), self.dynamics_status)
