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

# Vendor factory nearest-neighbour gripper table (mm / motor radians).
GRIPPER_DISTANCES_MM = (0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0, 29.0, 30.0, 31.0, 32.0, 33.0, 34.0, 35.0, 36.0, 37.0, 38.0, 39.0, 40.0, 41.0, 42.0, 43.0, 44.0, 45.0, 46.0, 47.0, 48.0, 49.0, 50.0, 51.0, 52.0, 53.0, 54.0, 55.0, 56.0, 57.0, 58.0, 59.0, 60.0, 61.0, 62.0, 63.0, 64.0, 65.0, 66.0, 67.0, 68.0, 69.0, 70.0, 71.0, 72.0, 73.0, 74.0, 75.0, 76.0, 77.0, 78.0, 79.0, 80.0, 81.0, 82.0, 83.0, 84.0, 85.0, 86.0, 87.0, 88.0, 89.0, 90.0, 91.0, 92.0, 93.0, 94.0, 95.0, 96.0, 97.0, 98.0, 99.0, 100.0, 102.5)
GRIPPER_ANGLES_RAD = (0.002, 0.01407, 0.0368934, 0.08634, 0.115854, 0.161084, 0.176609, 0.194816, 0.22893, 0.262277, 0.295, 0.314215, 0.344305, 0.362704, 0.393177, 0.411575, 0.441473, 0.46773, 0.482871, 0.508743, 0.527718, 0.550333, 0.576397, 0.60227, 0.621053, 0.639452, 0.662833, 0.683915, 0.707105, 0.722054, 0.748693, 0.763451, 0.782233, 0.805039, 0.815964, 0.841837, 0.856786, 0.879593, 0.894158, 0.91639, 0.92789, 0.94648, 0.969287, 0.987494, 0.998418, 1.02084, 1.03617, 1.05112, 1.0701, 1.08505, 1.10383, 1.11897, 1.13411, 1.15557, 1.16382, 1.18221, 1.20117, 1.21594, 1.23779, 1.24986, 1.26424, 1.28264, 1.29452, 1.31273, 1.3317, 1.34359, 1.36505, 1.38019, 1.39169, 1.40683, 1.42542, 1.44401, 1.45609, 1.47468, 1.48943, 1.50419, 1.52259, 1.53735, 1.55575, 1.57453, 1.582, 1.60462, 1.62302, 1.64218, 1.65694, 1.66825, 1.68722, 1.70984, 1.729, 1.73973, 1.76216, 1.78477, 1.79991, 1.82214, 1.84399, 1.86297, 1.88577, 1.9036, 1.92717, 1.94844, 1.97527, 2.03143)
