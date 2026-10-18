"""Command shaping and automatic actions from gimbal_cmd.c / arm_auto.c."""

from math import isfinite

from .config import (GRIPPER_MAX, GRIPPER_MIN, JOINT_MAX, JOINT_MIN,
                     JOINT_QUINTIC_DURATION_S, PLAN_SPEED_COMP_COEF, RESET_ANGLE, TD_R)
from .numerics import TrackingDifferentiator, clip
from .trajectory import JointQuinticReference, PlanGroup, Waypoint
from .types import ArmMode, ZeroResult, ZeroState


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

    def _gripper_finish(self, failed=False):
        self.gripper_angle = self.arm.gripper_angle
        self.gripper_torque = 0.0
        self.auto_step = 5
        self.auto_failed = failed

    def _gripper_step(self, direction):
        if (self.gripper_angle - self.arm.gripper_angle) * direction < 20:
            self.gripper_angle += direction * .2

    def _gripper_zero(self, now):
        arm, z, fdb = self.arm, self.arm.zero, self.arm.gripper_angle
        g = self.gz
        elapsed = now - g.get("last_ms", now) if g else 0
        g["last_ms"] = now
        if self.auto_step == 0:
            arm.initialized[6] = False
            self.gripper_angle = fdb
            self.gripper_torque = .5
            self.gz = dict(win_pos=fdb, win_ms=0, total_ms=0, back_ms=0, zero_ms=0,
                           settle_ms=0, stall_win=0, zero_sent=False, result=ZeroResult.NONE,
                           last_ms=now)
            self.auto_step = 1
        elif self.auto_step in (1, 2):
            if not arm.bank.motors[6].online(now) or arm.mode != ArmMode.RUN:
                self._gripper_finish(True)
                return
            if self.auto_step == 1:
                self._gripper_step(1)
                self.gripper_torque = .5
                g["total_ms"] += elapsed
                g["win_ms"] += elapsed
                if g["win_ms"] >= 200:
                    g["stall_win"] = g["stall_win"] + 1 if abs(fdb - g["win_pos"]) < .5 else 0
                    g["win_pos"], g["win_ms"] = fdb, 0
                if g["stall_win"] >= 3:
                    g["back_target"] = fdb - 142
                    self.auto_step = 2
                elif g["total_ms"] >= 10000:
                    self._gripper_finish(True)
            else:
                self._gripper_step(-1)
                self.gripper_angle = max(self.gripper_angle, g["back_target"])
                g["back_ms"] += elapsed
                if abs(fdb - g["back_target"]) <= 4:
                    self.auto_step = 3
                elif g["back_ms"] >= 6000:
                    self._gripper_finish(True)
        elif self.auto_step == 3:
            self.gripper_angle = fdb
            if not g["zero_sent"]:
                g["zero_sent"] = z.request(1 << 6)
            arm.bank.motors[6].keep_alive(now)
            g["zero_ms"] += elapsed
            if (g["zero_sent"] and not z.pending and z.state in (ZeroState.DONE, ZeroState.TIMEOUT)
                    or g["zero_ms"] >= 12000):
                g["result"] = z.results[6]
                self.auto_step = 4
        elif self.auto_step == 4:
            self.gripper_angle = fdb
            arm.bank.motors[6].keep_alive(now)
            if z.busy:
                g["settle_ms"] = 0
            else:
                g["settle_ms"] += elapsed
                if g["settle_ms"] >= 400:
                    ok = g["result"] in (ZeroResult.OK, ZeroResult.UNCONFIRMED)
                    arm.initialized[6] = ok
                    self._gripper_finish(not ok)
        else:
            self.auto, self.auto_step = None, 0
