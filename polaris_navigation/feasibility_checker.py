"""Trajectory feasibility checking utilities."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence, Tuple

from .trajectory_generator import MinimumJerkSegment


@dataclass
class FeasibilityLimits:
    max_velocity: float
    max_acceleration: float
    max_jerk: float
    min_clearance: float


@dataclass
class FeasibilityViolation(Exception):
    message: str

    def __post_init__(self) -> None:
        super().__init__(self.message)


class TrajectoryFeasibilityChecker:
    """Validate trajectories against dynamic and clearance constraints."""

    def __init__(
        self,
        limits: FeasibilityLimits,
        clearance_function: callable | None = None,
    ) -> None:
        self.limits = limits
        self.clearance_function = clearance_function

    def check(
        self,
        trajectory: Sequence[Tuple[MinimumJerkSegment, MinimumJerkSegment, MinimumJerkSegment]],
        sample_rate: float,
    ) -> None:
        """Raise :class:`FeasibilityViolation` if the trajectory violates limits."""

        if sample_rate <= 0:
            raise ValueError("Sample rate must be positive")

        dt = 1.0 / sample_rate

        for segments in trajectory:
            duration = segments[0].duration
            t = 0.0
            prev_velocity = None
            prev_acceleration = None
            while t <= duration + 1e-6:
                state = [seg.evaluate(min(t, duration)) for seg in segments]
                position = tuple(component[0] for component in state)
                velocity = tuple(component[1] for component in state)
                acceleration = tuple(component[2] for component in state)

                speed = sum(v**2 for v in velocity) ** 0.5
                if speed > self.limits.max_velocity + 1e-6:
                    raise FeasibilityViolation(
                        f"Velocity limit exceeded: {speed:.2f} m/s > {self.limits.max_velocity:.2f}"
                    )

                accel_norm = sum(a**2 for a in acceleration) ** 0.5
                if accel_norm > self.limits.max_acceleration + 1e-6:
                    raise FeasibilityViolation(
                        f"Acceleration limit exceeded: {accel_norm:.2f} m/s^2 > {self.limits.max_acceleration:.2f}"
                    )

                if prev_velocity is not None and prev_acceleration is not None:
                    jerk = sum(
                        (acceleration[i] - prev_acceleration[i]) ** 2 for i in range(3)
                    ) ** 0.5 / dt
                    if jerk > self.limits.max_jerk + 1e-6:
                        raise FeasibilityViolation(
                            f"Jerk limit exceeded: {jerk:.2f} m/s^3 > {self.limits.max_jerk:.2f}"
                        )

                if self.clearance_function is not None:
                    clearance = self.clearance_function(position)
                    if clearance < self.limits.min_clearance:
                        raise FeasibilityViolation(
                            f"Clearance {clearance:.2f} m below minimum {self.limits.min_clearance:.2f}"
                        )

                prev_velocity = velocity
                prev_acceleration = acceleration
                t += dt
