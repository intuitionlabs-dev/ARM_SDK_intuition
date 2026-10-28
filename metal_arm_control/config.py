"""Metal Arm 参数；来源见 gravity_compensation/PROVENANCE.md。"""

from dataclasses import dataclass

from .numerics import PIDConfig
from .types import finite_tuple

# Vendor MIT ranges are encoding scales, not rated torque/speed.
from sdk.board_defs import metal_arm as board_defs

JOINT_MIN = board_defs.JOINT_ANGLE_MIN
JOINT_MAX = board_defs.JOINT_ANGLE_MAX
GRIPPER_MIN, GRIPPER_MAX = board_defs.GRIPPER_ANGLE_MIN, board_defs.GRIPPER_ANGLE_MAX
RESET_ANGLE = board_defs.RESET_POSE_DEG
TD_R = (1000.0, 4000.0, 4000.0, 2000.0, 2000.0, 2000.0)
JOINT_QUINTIC_DURATION_S = 0.16
PLAN_SPEED_COMP_COEF = (1.0,) * 6
MOTOR_IDS = (1, 2, 3, 4, 5, 6, 7)
MOTOR_RX_IDS = (0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17)
MOTOR_PMAX = (6.28,) * 7
MOTOR_VMAX = (10.0, 10.0, 10.0, 10.0, 30.0, 30.0, 30.0)
MOTOR_TMAX = (30.0, 120.0, 30.0, 30.0, 20.0, 20.0, 20.0)
# Vendor slave_arm follow_mit_kp / follow_mit_kd, J1–J6 then gripper.
MOTOR_KP = (200.0, 500.0, 400.0, 200.0, 20.0, 20.0, 20.0)
MOTOR_KD = (3.0, 5.0, 5.0, 2.0, 0.1, 0.1, 0.1)
# Keep the ARX millisecond scheduler / 250 Hz motor cadence. The vendor
# C++ loop targets 400 Hz; do not claim that timing for this Python port.
MOTOR_PERIOD_MS = 4
TX_GROUPS = (0x11, 0x11, 0x22, 0x22, 0x44, 0x44, 0x88)
FEEDBACK_TIMEOUT_MS = 100
ZERO_PID = PIDConfig()
# Metal has native MIT PD control; do not layer ARX's host PID on top.
POSITION_PID = (ZERO_PID,) * 7
SPEED_PID = (ZERO_PID,) * 7
GRAVITY_SCALES = (1.2, 1.15, 1.1, 1.15, 1.0, 1.0)
FRICTION_COEFFICIENTS = (0.15, 0.3025, 0.52, 0.42, 0.015, 0.015)
FRICTION_SPEED_LIMITS = (None, None, 0.8, 1.2, None, None)
CORIOLIS_SCALES = (1.1, 1.15, 1.0, 1.0, 1.1, 1.1)


@dataclass(frozen=True)
class ArmConfig:
    direction: tuple[float, ...] = (1.0,) * 6
    reduction: tuple[float, ...] = (1.0,) * 6
    position_zero_deg: tuple[float, ...] = (0.0,) * 6
    torque_scale: tuple[float, ...] = (1.0,) * 6
    viscous_damping: tuple[float, ...] = (0.0,) * 6
    velocity_scale: tuple[float, ...] = (1.0,) * 6
    gravity_enabled: bool = True
    gravity_model: str = "metal_with_gripper"
    gravity_sdk_scale: bool = True
    gravity_base: tuple[float, ...] = (0.0, 0.0, -9.81)
    calibrate_encos: bool = True

    def __post_init__(self):
        for name in ("direction", "reduction", "position_zero_deg", "torque_scale",
                     "viscous_damping", "velocity_scale", "gravity_base"):
            object.__setattr__(self, name, finite_tuple(getattr(self, name),
                               3 if name == "gravity_base" else 6, name))
        if any(v not in (-1, 1) for v in self.direction):
            raise ValueError("motor directions must be +1 or -1")
        if any(v <= 0 for v in self.reduction):
            raise ValueError("motor reductions must be positive")
        if self.gravity_model not in ("metal_with_gripper", "metal_no_gripper"):
            raise ValueError("unknown Metal gravity model")
