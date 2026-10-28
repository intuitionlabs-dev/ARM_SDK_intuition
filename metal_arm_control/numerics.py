"""Active H7 PID/TD algorithms. PID gains are per invocation, without dt."""

from dataclasses import dataclass
from math import isfinite, sqrt


def clip(value, low, high):
    return max(low, min(value, high))


def sign(value):
    return (value > 0) - (value < 0)


def fsg(value, width):
    # C returns int: at a boundary (+/-width), 1 / 2 truncates to zero.
    return int((sign(value + width) - sign(value - width)) / 2)


def fst(error, velocity, r, h0):
    d = r * h0 * h0
    a0 = h0 * velocity
    y = error + a0
    a1 = sqrt(d * (d + 8 * abs(y)))
    a2 = a0 + sign(y) * (a1 - d) / 2
    sy = fsg(y, d)
    a = (a0 + y) * sy + a2 * (1 - sy)
    sa = fsg(a, d)
    return -r * (a / d) * sa - r * sign(a) * (1 - sa)


@dataclass
class TrackingDifferentiator:
    r: float
    h: float = 0.002
    h0: float = 0.004
    position: float = 0.0
    velocity: float = 0.0
    acceleration: float = 0.0

    def __post_init__(self):
        if not all(isfinite(x) and x > 0 for x in (self.r, self.h, self.h0)):
            raise ValueError("TD r, h and h0 must be finite and positive")

    def follow(self, position):
        self.position, self.velocity, self.acceleration = position, 0.0, 0.0

    def step(self, reference):
        p, v = self.position, self.velocity
        self.acceleration = fst(p - reference, v, self.r, self.h0)
        self.position = p + self.h * v
        self.velocity = v + self.h * self.acceleration
        return self.position, self.velocity


@dataclass(frozen=True)
class PIDConfig:
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    integral_limit: float = 0.0
    output_limit: float = 0.0
    integral_range: float = 0.0


@dataclass
class PID:
    config: PIDConfig
    error: float = 0.0
    derivative: float = 0.0
    integral: float = 0.0
    output: float = 0.0
    unlimited: float = 0.0

    def reset(self):
        self.error = self.derivative = self.integral = self.output = self.unlimited = 0.0

    def step(self, reference, feedback):
        error = reference - feedback
        delta = error - self.error
        # Preserve PID_Calc's derivative retention at an unchanged error.
        if abs(delta) > 1e-6 or abs(self.derivative) < 0.001:
            self.derivative = delta
        self.error = error
        c = self.config
        if abs(error) < c.integral_range:
            self.integral = clip(self.integral + error, -c.integral_limit, c.integral_limit)
        else:
            self.integral = 0.0
        self.unlimited = c.kp * error + c.ki * self.integral + c.kd * self.derivative
        self.output = clip(self.unlimited, -c.output_limit, c.output_limit)
        return self.output


def temperature_coefficient(temperature, low, high):
    """Port of the unused firmware helper. Not applied by the arm controller."""
    if not all(isfinite(x) for x in (temperature, low, high)) or high <= low:
        raise ValueError("finite temperature thresholds must satisfy low < high")
    return (high - clip(temperature, low, high)) / (high - low)
