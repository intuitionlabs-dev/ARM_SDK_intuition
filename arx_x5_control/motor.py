"""Motor control policy, feedback freshness and semantic requests (no CAN)."""

from dataclasses import dataclass, field
from math import degrees, radians

from .config import MOTOR_IDS, MOTOR_KP, MOTOR_KD, POSITION_PID, SPEED_PID, TX_GROUPS, ZERO_PID
from .numerics import PID, clip
from .types import ArmMode, MotorAction, MotorFeedback, MotorRequest


@dataclass
class Motor:
    arm_id: int
    index: int
    position: float = 0.0
    velocity: float = 0.0
    current: float | None = None
    torque: float | None = None
    temperature: float | None = None
    mos_temperature: float | None = None
    error_code: int = 0
    last_rx_ms: int = -1
    last_position_ms: int = -1
    last_velocity_ms: int = -1
    monitor_deadline_ms: int = 0
    rounds: int = 0
    zero_pending: bool = False
    zero_sent_ms: int = 0
    zero_ack: bool | None = None
    enable_cleared: bool = False
    reference_deg: float = 0.0
    speed_comp: float = 0.0
    output_comp: float = 0.0
    reference_velocity_deg: float = 0.0
    mode: ArmMode = ArmMode.STOP
    position_pid: PID = field(init=False)
    speed_pid: PID = field(init=False)

    def __post_init__(self):
        self.position_pid = PID(POSITION_PID[self.index])
        self.speed_pid = PID(SPEED_PID[self.index])

    @property
    def encos(self):
        return self.index < 3

    @property
    def position_deg(self):
        # Firmware installs DM_Pos_Fdb_Calc: it uses raw angle, not multi-turn.
        return degrees(self.position)

    @property
    def velocity_deg(self):
        return degrees(self.velocity)

    def keep_alive(self, now):
        # MonitorTask decrements every 10 ms; ENCOS reload=10, DM reload=2.
        self.monitor_deadline_ms = (now // 10 + (10 if self.encos else 2)) * 10

    def online(self, now):
        return now < self.monitor_deadline_ms

    def accept(self, feedback: MotorFeedback):
        if feedback.timestamp_ms < self.last_rx_ms:
            return False
        now = feedback.timestamp_ms
        self.last_rx_ms = now
        self.keep_alive(now)
        self.error_code = feedback.error_code
        if self.encos and feedback.zero_ack is not None and self.zero_pending:
            self.zero_ack = feedback.zero_ack
            self.zero_pending = False
            self.last_position_ms = self.last_velocity_ms = -1
        if feedback.position is not None:
            if not self.encos:
                if feedback.position - self.position > 12.5:
                    self.rounds -= 1
                elif feedback.position - self.position < -12.5:
                    self.rounds += 1
            self.position = feedback.position
            self.last_position_ms = now
        if feedback.velocity is not None:
            self.velocity = feedback.velocity
            self.last_velocity_ms = now
        for name in ("current", "torque", "temperature", "mos_temperature"):
            if getattr(feedback, name) is not None:
                setattr(self, name, getattr(feedback, name))
        return True

    def motion_ready(self, now):
        return (not self.error_code and not self.zero_pending
                and self.last_position_ms >= 0 and now - self.last_position_ms < 100
                and self.last_velocity_ms >= 0 and now - self.last_velocity_ms < 100)

    def request(self, action, now, **values):
        return MotorRequest(self.arm_id, self.index, MOTOR_IDS[self.index],
                            "encos" if self.encos else "dm", action, now, **values)

    def reset_controller(self):
        self.position_pid.reset()
        self.speed_pid.reset()

    def control(self, now):
        enabled = self.mode != ArmMode.STOP
        if self.encos:
            if self.zero_pending and now - self.zero_sent_ms >= 1000:
                self.zero_pending = False
            if enabled and not self.enable_cleared:
                self.reset_controller()
            self.enable_cleared = enabled
        # Zero gains on joints in passive modes. Gravity mode keeps gripper gains.
        passive = self.mode == ArmMode.DAMPING or (self.mode == ArmMode.GRAVITY and self.index < 6)
        self.position_pid.config = ZERO_PID if passive else POSITION_PID[self.index]
        self.speed_pid.config = ZERO_PID if passive else SPEED_PID[self.index]
        self.reference_velocity_deg = self.position_pid.step(self.reference_deg, self.position_deg) + self.speed_comp
        torque = self.speed_pid.step(self.reference_velocity_deg, self.velocity_deg) + self.output_comp
        if not enabled or (self.encos and not self.motion_ready(now)):
            if self.encos:
                self.mode = ArmMode.STOP
            return self.request(MotorAction.SOFT_STOP, now)
        kp, kd = MOTOR_KP[self.index], MOTOR_KD[self.index]
        if self.mode == ArmMode.DAMPING:
            kp = 0.0
        elif self.mode == ArmMode.GRAVITY and self.index < 6:
            kp = kd = 0.0
        position = radians(self.reference_deg) - (0 if self.encos else self.rounds * 25.0)
        speed_limit, torque_limit = (18.0, 30.0) if self.encos else (45.0, 10.0)
        return self.request(MotorAction.CONTROL, now,
                            position=clip(position, -12.5, 12.5),
                            velocity=clip(radians(self.reference_velocity_deg), -speed_limit, speed_limit),
                            torque=clip(torque, -torque_limit, torque_limit), kp=kp, kd=kd)


class MotorBank:
    def __init__(self, arm_id):
        self.motors = [Motor(arm_id, i) for i in range(7)]
        self.encos_guard_until = 0
        self.enable_last_ms = 0
        self.enable_next = 3
        self.next_due_ms = [next(k for k in range(8) if mask & (1 << k)) for mask in TX_GROUPS]

    def operation(self, index, action, now):
        motor = self.motors[index]
        if motor.encos and now < self.encos_guard_until:
            return None
        if action == MotorAction.SAVE_ZERO and motor.encos:
            if any(m.mode != ArmMode.STOP for m in self.motors[:3]):
                return None
            motor.zero_pending = True
            motor.zero_sent_ms = now
            motor.zero_ack = None
            motor.last_position_ms = motor.last_velocity_ms = -1
            self.encos_guard_until = now + 501
        if action == MotorAction.DISABLE and motor.encos:
            motor.mode = ArmMode.STOP
            motor.reset_controller()
            action = MotorAction.SOFT_STOP
        return motor.request(action, now)

    def tick(self, now, excluded_mask=0):
        output = []
        # DM keepalive is sent in STOP as well; STOP is a zero-gain control frame.
        if now - self.enable_last_ms >= 75:
            for offset in range(4):
                index = 3 + (self.enable_next - 3 + offset) % 4
                if not excluded_mask & (1 << index):
                    output.append(self.motors[index].request(MotorAction.ENABLE, now))
                    self.enable_last_ms = now
                    self.enable_next = 3 + (index - 2) % 4
                    break
        # H7's MotorTask visits DM before ENCOS. PID only executes on its group.
        for i in (3, 4, 5, 6, 0, 1, 2):
            if excluded_mask & (1 << i):
                continue
            if now >= self.next_due_ms[i]:
                if i < 3 and now < self.encos_guard_until:
                    continue
                output.append(self.motors[i].control(now))
                period = 2 if i == 0 else 4
                self.next_due_ms[i] += ((now - self.next_due_ms[i]) // period + 1) * period
        return output
