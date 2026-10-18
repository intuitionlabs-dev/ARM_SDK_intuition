"""Deterministic test plant. It exercises interfaces, not physical dynamics."""

from math import radians

from .config import RESET_ANGLE
from .numerics import clip
from .types import MotorAction, MotorFeedback


class SimulatedMotors:
    """Ideal velocity/stiffness response, gripper stops and setting acknowledgements.

    This class never opens devices. It is unsuitable for stability or load tests.
    """

    def __init__(self, arm_count=2):
        self.arm_count = arm_count
        self.positions = [list(map(radians, RESET_ANGLE)) + [0.0] for _ in range(arm_count)]
        self.velocities = [[0.0] * 7 for _ in range(arm_count)]
        self.zero_offsets = [[0.0] * 7 for _ in range(arm_count)]
        self.enabled = [[i < 3 for i in range(7)] for _ in range(arm_count)]
        self.requests = [[None] * 7 for _ in range(arm_count)]
        self.acks = []
        self.last_ms = None

    def apply(self, requests):
        for request in requests:
            a, i = request.arm_id, request.motor_index
            if request.action == MotorAction.ENABLE:
                self.enabled[a][i] = True
            elif request.action == MotorAction.DISABLE:
                self.enabled[a][i] = False
                self.requests[a][i] = None
            elif request.action == MotorAction.SAVE_ZERO:
                self.zero_offsets[a][i] = self.positions[a][i]
                if i < 3:
                    self.acks.append((a, i, request.timestamp_ms + 1))
            else:
                self.requests[a][i] = request

    def feedback(self, now):
        dt = 0 if self.last_ms is None else (now - self.last_ms) / 1000
        self.last_ms = now
        output = []
        due_acks = [x for x in self.acks if x[2] <= now]
        self.acks = [x for x in self.acks if x[2] > now]
        for a, i, at in due_acks:
            output.append(MotorFeedback(a, i, at, zero_ack=True))
        for a in range(self.arm_count):
            for i in range(7):
                request = self.requests[a][i]
                speed = 0.0
                if request and self.enabled[a][i] and request.action == MotorAction.CONTROL:
                    reported = self.positions[a][i] - self.zero_offsets[a][i]
                    if request.kd > 0:
                        speed = request.velocity + request.kp / request.kd * (request.position - reported)
                        # Feedforward gravity is cancelled by assumed load; gripper has none.
                        if i == 6:
                            speed += request.torque / request.kd
                    speed = clip(speed, -6, 6)
                old = self.positions[a][i]
                position = old + speed * dt
                if i == 6:
                    position = clip(position, radians(-142), radians(142))
                self.positions[a][i] = position
                self.velocities[a][i] = (position - old) / dt if dt else 0.0
                output.append(MotorFeedback(a, i, now, position=position-self.zero_offsets[a][i],
                              velocity=self.velocities[a][i], temperature=25.0,
                              error_code=0 if i < 3 or not self.enabled[a][i] else 1))
        return tuple(output)

    def poll_feedback(self, now_ms):
        return self.feedback(now_ms)

    def write_motor_requests(self, requests):
        self.apply(requests)
