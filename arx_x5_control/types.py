"""Typed communication boundary. No wire format or transport is implemented.

Joint commands/arm feedback use degrees. Motor boundary uses rad, rad/s, N*m.
Timestamps are nonnegative integer milliseconds in the controller's monotonic
clock domain. Motor feedback timestamps mean acquisition time, not dequeue time.
"""

from dataclasses import dataclass
from enum import Enum, IntEnum
from math import isfinite


class ArmMode(IntEnum):
    STOP = 0
    RUN = 1
    DAMPING = 2
    GRAVITY = 3


class RobotState(IntEnum):
    WAIT_HANDSHAKE = 0
    OFFLINE_WAIT = 1
    WAIT_INIT = 2
    INITIALIZING = 3
    NORMAL = 4
    COMM_LOST = 5
    CALIBRATING = 6
    RESETTING = 7


class CommandKind(Enum):
    CONNECT = "connect"
    HEARTBEAT = "heartbeat"
    INITIALIZE = "initialize"
    CALIBRATE = "calibrate"
    START = "start"
    STOP = "stop"
    DAMPING = "damping"
    GRAVITY = "gravity"
    POSITION = "position"
    POSITION_VELOCITY = "position_velocity"
    RESET = "reset"


class MotorAction(Enum):
    CONTROL = "control"
    SOFT_STOP = "soft_stop"
    DISABLE = "disable"
    ENABLE = "enable"
    SAVE_ZERO = "save_zero"


class ZeroState(IntEnum):
    IDLE = 0
    DISABLE = 1
    WAIT_DISABLE = 2
    SEND = 3
    CHECK = 4
    DONE = 5
    TIMEOUT = 6


class ZeroResult(IntEnum):
    NONE = 0
    UNCONFIRMED = 1
    OK = 2
    FAIL = 3
    TIMEOUT = 4
    SKIP = 5


def finite_tuple(values, size, name):
    result = tuple(float(v) for v in values)
    if len(result) != size or not all(isfinite(v) for v in result):
        raise ValueError(f"{name} must contain {size} finite numbers")
    return result


def timestamp(value):
    if type(value) is not int or value < 0:
        raise ValueError("timestamp must be a nonnegative integer in milliseconds")
    return value


@dataclass(frozen=True)
class Command:
    arm_id: int
    kind: CommandKind
    joint_angle: tuple[float, ...] = ()
    gripper_angle: float = 0.0
    joint_velocity: tuple[float, ...] = ()
    api_version: int = 1

    def __post_init__(self):
        object.__setattr__(self, "kind", CommandKind(self.kind))
        position = self.kind in (CommandKind.POSITION, CommandKind.POSITION_VELOCITY)
        object.__setattr__(self, "joint_angle", finite_tuple(
            self.joint_angle, 6 if position else 0, "joint_angle"))
        object.__setattr__(self, "joint_velocity", finite_tuple(
            self.joint_velocity, 6 if self.kind == CommandKind.POSITION_VELOCITY else 0,
            "joint_velocity"))
        if not isfinite(self.gripper_angle):
            raise ValueError("gripper_angle must be finite")


@dataclass(frozen=True)
class MotorFeedback:
    arm_id: int
    motor_index: int                 # j1..j6 = 0..5, gripper = 6
    timestamp_ms: int
    position: float | None = None    # raw, decoded motor angle; rad
    velocity: float | None = None    # rad/s
    torque: float | None = None      # N*m; ENCOS without Kt: leave None
    current: float | None = None     # A
    temperature: float | None = None
    mos_temperature: float | None = None
    error_code: int = 0              # DM: 0 disabled, 1 enabled; ENCOS: 0 healthy
    zero_ack: bool | None = None     # ENCOS setting acknowledgement only

    def __post_init__(self):
        timestamp(self.timestamp_ms)
        if type(self.motor_index) is not int or not 0 <= self.motor_index < 7:
            raise ValueError("motor_index must be 0..6")
        for name in ("position", "velocity", "torque", "current", "temperature", "mos_temperature"):
            value = getattr(self, name)
            if value is not None and not isfinite(value):
                raise ValueError(f"{name} must be finite or None")
        if type(self.error_code) is not int or self.error_code < 0:
            raise ValueError("error_code must be a nonnegative integer")
        if self.zero_ack is not None and type(self.zero_ack) is not bool:
            raise ValueError("zero_ack must be bool or None")


@dataclass(frozen=True)
class MotorRequest:
    arm_id: int
    motor_index: int
    motor_id: int
    family: str
    action: MotorAction
    timestamp_ms: int
    position: float = 0.0
    velocity: float = 0.0
    torque: float = 0.0
    kp: float = 0.0
    kd: float = 0.0


@dataclass(frozen=True)
class ArmFeedback:
    arm_id: int
    timestamp_ms: int
    state: RobotState
    mode: ArmMode                    # actual mode after motor loss gating
    requested_mode: ArmMode
    joint_angle: tuple[float, ...]
    joint_velocity: tuple[float, ...]
    gripper_angle: float
    motor_lost: tuple[bool, ...]
    motor_initialized: tuple[bool, ...]
    motor_temperature: tuple[float | None, ...]
    gravity_torque: tuple[float, ...]
    dynamics_torque: tuple[float, ...] = (0.0,) * 6
    dynamics_status: int = 0


@dataclass(frozen=True)
class Event:
    arm_id: int
    timestamp_ms: int
    kind: str
    detail: str = ""
    zero_state: ZeroState | None = None
    zero_results: tuple[ZeroResult, ...] = ()
    elapsed_ms: int = 0


@dataclass(frozen=True)
class TickResult:
    motor_requests: tuple[MotorRequest, ...] = ()
    feedback: tuple[ArmFeedback, ...] = ()
    events: tuple[Event, ...] = ()
