"""Linear/cubic/quintic paths and synchronized groups from libs/move_plan.

The PC scheduler evaluates absolute elapsed time and includes the endpoint.
This fixes the C planner's last-segment out-of-bounds access at exact duration.
"""

from bisect import bisect_right
from dataclasses import dataclass
from enum import IntEnum
from math import isfinite

from .numerics import clip


class SplineKind(IntEnum):
    LINEAR = 0
    CUBIC = 1
    QUINTIC = 2


@dataclass(frozen=True)
class Waypoint:
    position: float
    velocity: float = 0.0
    acceleration: float = 0.0

    def __post_init__(self):
        if not all(isfinite(v) for v in (self.position, self.velocity, self.acceleration)):
            raise ValueError("waypoint values must be finite")


class Spline:
    def __init__(self, start: Waypoint, end: Waypoint, duration: float,
                 kind=SplineKind.QUINTIC):
        if not isfinite(duration) or duration <= 0:
            raise ValueError("spline duration must be finite and positive")
        self.duration = duration
        self.kind = SplineKind(kind)
        p, v, a = start.position, start.velocity, start.acceleration
        dp, t = end.position - p, duration
        if self.kind == SplineKind.LINEAR:
            self.coefficients = (p, dp / t)
        elif self.kind == SplineKind.CUBIC:
            self.coefficients = (p, v, 3*dp/t**2 - (2*v+end.velocity)/t,
                                 -2*dp/t**3 + (v+end.velocity)/t**2)
        else:
            dv, da = end.velocity - v, end.acceleration - a
            residual = dp - v*t - a*t*t/2
            rv = dv - a*t
            self.coefficients = (
                p, v, a/2,
                10*residual/t**3 - 4*rv/t**2 + da/(2*t),
                -15*residual/t**4 + 7*rv/t**3 - da/t**2,
                6*residual/t**5 - 3*rv/t**4 + da/(2*t**3))

    def sample(self, seconds):
        if not isfinite(seconds):
            raise ValueError("sample time must be finite")
        t = clip(seconds, 0, self.duration)
        c = self.coefficients
        return Waypoint(sum(x*t**i for i, x in enumerate(c)),
                        sum(i*c[i]*t**(i-1) for i in range(1, len(c))),
                        sum(i*(i-1)*c[i]*t**(i-2) for i in range(2, len(c))))


class JointQuinticReference:
    """每拍推进的关节五次样条，重规划保留当前速度和加速度。"""

    def __init__(self, duration):
        self.duration = duration
        self.current = Waypoint(0.0)
        self.initialized = False
        self.curve = None
        self.target = 0.0
        self.elapsed = 0.0

    def follow(self, point):
        self.current = point
        self.initialized = True
        self.curve = None

    def update(self, target, feedback, period=0.002):
        if not self.initialized:
            self.follow(Waypoint(feedback))
        if self.curve is None or target != self.target:
            self.curve = Spline(self.current, Waypoint(target), self.duration)
            self.target, self.elapsed = target, 0.0
        self.elapsed = min(self.duration, self.elapsed + period)
        self.current = (Waypoint(self.target) if self.elapsed >= self.duration
                        else self.curve.sample(self.elapsed))
        return self.current


class Trajectory:
    def __init__(self, points, durations, kind=SplineKind.QUINTIC):
        self.points = tuple(points)
        self.durations = tuple(durations)
        if len(self.points) < 2 or len(self.durations) != len(self.points) - 1:
            raise ValueError("a trajectory needs N >= 2 points and N-1 durations")
        self.segments = tuple(Spline(a, b, t, kind) for a, b, t in
                              zip(self.points, self.points[1:], self.durations))
        self.ends = []
        total = 0.0
        for t in self.durations:
            total += t
            self.ends.append(total)
        self.duration = total

    def sample(self, seconds):
        if not isfinite(seconds):
            raise ValueError("sample time must be finite")
        index = min(bisect_right(self.ends, seconds), len(self.segments) - 1)
        start = self.ends[index-1] if index else 0.0
        return self.segments[index].sample(seconds - start)


def _blend(left, right):
    if left * right < 0 or abs(left) < 1e-4 or abs(right) < 1e-4:
        return 0.0
    return (left + right) / 2


class PlanGroup:
    def __init__(self, trajectories):
        self.trajectories = tuple(trajectories)
        if not self.trajectories:
            raise ValueError("plan group must contain at least one axis")
        self.duration = self.trajectories[0].duration
        if any(p.durations != self.trajectories[0].durations for p in self.trajectories):
            raise ValueError("all axes must have synchronized segments")
        self.started_ms = None
        self.running = False
        self.current = tuple(p.sample(0) for p in self.trajectories)

    @classmethod
    def temporary(cls, start, end, duration, kind=SplineKind.QUINTIC):
        start, end = tuple(start), tuple(end)
        if len(start) != len(end):
            raise ValueError("start/end dimensions differ")
        return cls(Trajectory((Waypoint(a), Waypoint(b)), (duration,), kind)
                   for a, b in zip(start, end))

    @classmethod
    def through(cls, positions_by_axis, durations, kind=SplineKind.QUINTIC):
        durations = tuple(durations)
        if not durations or not all(isfinite(t) and t > 0 for t in durations):
            raise ValueError("durations must be positive")
        paths = []
        for positions in positions_by_axis:
            positions = tuple(positions)
            if len(positions) != len(durations) + 1:
                raise ValueError("invalid waypoint count")
            slopes = [(b-a)/t for a, b, t in zip(positions, positions[1:], durations)]
            velocities = [0.0] + [_blend(a, b) for a, b in zip(slopes, slopes[1:])] + [0.0]
            accel = [(b-a)/t for a, b, t in zip(velocities, velocities[1:], durations)]
            accelerations = [0.0] + [_blend(a, b) for a, b in zip(accel, accel[1:])] + [0.0]
            paths.append(Trajectory(tuple(Waypoint(*v) for v in
                         zip(positions, velocities, accelerations)), durations, kind))
        return cls(paths)

    def start(self, now_ms):
        self.started_ms = now_ms
        self.running = True
        self.current = tuple(p.sample(0) for p in self.trajectories)

    def cancel(self):
        self.running = False
        self.started_ms = None

    def update(self, now_ms):
        if self.running:
            elapsed = max(0, now_ms - self.started_ms) / 1000
            self.current = tuple(p.sample(elapsed) for p in self.trajectories)
            if elapsed >= self.duration:
                self.running = False
        return self.current
