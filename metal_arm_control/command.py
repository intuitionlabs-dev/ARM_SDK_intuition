"""Command shaping and automatic actions from gimbal_cmd.c / arm_auto.c."""

from math import isfinite

from .config import (GRIPPER_MAX, GRIPPER_MIN, JOINT_MAX, JOINT_MIN,
                     JOINT_QUINTIC_DURATION_S, PLAN_SPEED_COMP_COEF, RESET_ANGLE, TD_R)
from .numerics import TrackingDifferentiator, clip
from .trajectory import JointQuinticReference, PlanGroup, Waypoint
from .types import ArmMode


class CommandLayer:
    def __init__(self, arm):
        self.arm = arm
        self.td = [TrackingDifferentiator(r) for r in TD_R]
        self.quintic = [JointQuinticReference(JOINT_QUINTIC_DURATION_S) for _ in range(6)]
        self.mode = ArmMode.STOP
        self.source = [0.0] * 6
        self.angle = [0.0] * 6
        self.velocity = [0.0] * 6
        self.acceleration = [0.0] * 6
        self.gripper_angle = self.gripper_torque = 0.0
        self.auto = None
        self.auto_step = 0
        self.plan = None
        self.gz = {}
        self.auto_failed = False

    def start_auto(self, kind):
        self.auto, self.auto_step = kind, 0
        self.auto_failed = False

    def cancel(self):
        self.auto, self.auto_step = None, 0
        if self.plan:
            self.plan.cancel()
        self.gripper_torque = 0.0

    def update(self, now):
        arm = self.arm
        self.mode = arm.mode
        self.gripper_torque = 0.0
        self.acceleration = [0.0] * 6
        if arm.mode != ArmMode.RUN:
            self.source = list(arm.joint_angle)
            self.gripper_angle = arm.gripper_angle
            for td, angle in zip(self.td, arm.joint_angle):
                td.follow(angle)
            if arm.mode == ArmMode.STOP:
                self.cancel()
        elif arm.control is not None:
            self.source = list(arm.control.joint_angle)
            self.gripper_angle = arm.control.gripper_angle
        if self.auto == "reset":
            self._reset(now)
        elif self.auto == "gripper_zero":
            self._gripper_zero(now)
        solved = list(self.source if arm.mode == ArmMode.RUN else arm.joint_angle)
        for i in range(6):
            if arm.initialized[i]:
                solved[i] = clip(solved[i], JOINT_MIN[i], JOINT_MAX[i])
        self.source = solved
        if arm.initialized[6]:
            self.gripper_angle = clip(self.gripper_angle, GRIPPER_MIN, GRIPPER_MAX)
        planning = self.auto is not None and self.plan is not None and self.plan.running
        external_velocity = (arm.mode == ArmMode.RUN and arm.control is not None
                             and bool(arm.control.joint_velocity))
        try:
            for i, td in enumerate(self.td):
                td.step(solved[i])
                if planning:
                    td.follow(solved[i])
                    point = Waypoint(solved[i], self.plan.current[i].velocity * PLAN_SPEED_COMP_COEF[i],
                                     self.plan.current[i].acceleration)
                elif external_velocity:
                    point = Waypoint(td.position, arm.control.joint_velocity[i])
                elif arm.mode == ArmMode.RUN:
                    point = self.quintic[i].update(solved[i], arm.joint_angle[i])
                else:
                    point = Waypoint(td.position, td.velocity)
                if not all(isfinite(v) for v in (point.position, point.velocity, point.acceleration)):
                    raise ValueError("nonfinite joint reference")
                self.angle[i], self.velocity[i] = point.position, point.velocity
                self.acceleration[i] = point.acceleration if arm.mode == ArmMode.RUN else 0.0
                if arm.mode != ArmMode.RUN or planning or external_velocity:
                    self.quintic[i].follow(Waypoint(arm.joint_angle[i])
                                           if arm.mode != ArmMode.RUN else point)
        except (ValueError, OverflowError):
            self.mode = ArmMode.DAMPING
            self.angle = list(arm.joint_angle)
            self.velocity, self.acceleration = [0.0] * 6, [0.0] * 6
            self.quintic = [JointQuinticReference(JOINT_QUINTIC_DURATION_S) for _ in range(6)]
            arm.event(now, "reference_error", "nonfinite joint reference")

    def _reset(self, now):
        if self.auto_step == 0:
            self.plan = PlanGroup.temporary(self.arm.joint_angle, RESET_ANGLE, 2.0)
            self.plan.start(now)
            self.source = [p.position for p in self.plan.current]
            self.auto_step = 1
        elif self.auto_step == 1:
            self.plan.update(now)
            self.source = [p.position for p in self.plan.current]
            if not self.plan.running:
                # 终点速度为零，TD 同步到规划终点。
                for td, position in zip(self.td, self.source):
                    td.follow(position)
                self.auto_step = 2
        else:
            self.auto, self.auto_step = None, 0

    def _gripper_zero(self, now):
        """Preserve the factory zero: Metal must never run X5's end-stop homing.

        Keep the same initialization stage/API, but validate fresh feedback and
        keep the measured angle. No SAVE_ZERO or exploratory motion is issued.
        """
        motor = self.arm.bank.motors[6]
        ok = motor.online(now) and motor.motion_ready(now) and GRIPPER_MIN <= motor.position_deg <= GRIPPER_MAX
        self.arm.initialized[6] = ok
        self.gripper_angle = self.arm.gripper_angle
        self.gripper_torque = 0.0
        self.auto_failed = not ok
        self.auto, self.auto_step = None, 0
