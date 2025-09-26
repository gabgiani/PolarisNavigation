"""Trajectory generation utilities."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Sequence, Tuple


@dataclass
class MinimumJerkSegment:
    """Polynomial segment for minimum-jerk motion in 1D."""

    coefficients: Tuple[float, float, float, float, float, float]
    duration: float

    def evaluate(self, t: float) -> Tuple[float, float, float]:
        """Evaluate position, velocity, and acceleration at time t."""

        if t < 0 or t > self.duration:
            raise ValueError("Time t must be within the segment duration")
        a0, a1, a2, a3, a4, a5 = self.coefficients
        position = a0 + a1 * t + a2 * t**2 + a3 * t**3 + a4 * t**4 + a5 * t**5
        velocity = (
            a1
            + 2 * a2 * t
            + 3 * a3 * t**2
            + 4 * a4 * t**3
            + 5 * a5 * t**4
        )
        acceleration = (
            2 * a2
            + 6 * a3 * t
            + 12 * a4 * t**2
            + 20 * a5 * t**3
        )
        return position, velocity, acceleration


def _minimum_jerk_coefficients(
    start: Tuple[float, float, float],
    end: Tuple[float, float, float],
    duration: float,
) -> Tuple[float, float, float, float, float, float]:
    """Solve for the coefficients of a 5th-order polynomial."""

    if duration <= 0:
        raise ValueError("Duration must be positive")

    p0, v0, a0 = start
    pf, vf, af = end

    T = duration
    T2 = T**2
    T3 = T**3
    T4 = T**4
    T5 = T**5

    a0_coef = p0
    a1_coef = v0
    a2_coef = 0.5 * a0

    c0 = pf - (p0 + v0 * T + 0.5 * a0 * T2)
    c1 = vf - (v0 + a0 * T)
    c2 = af - a0

    a3_coef = (10 * c0 - 4 * c1 + 0.5 * c2) / T3
    a4_coef = (-15 * c0 + 7 * c1 - c2) / T4
    a5_coef = (6 * c0 - 3 * c1 + 0.5 * c2) / T5

    return a0_coef, a1_coef, a2_coef, a3_coef, a4_coef, a5_coef


def generate_minimum_jerk_trajectory(
    waypoints: Sequence[Tuple[float, float, float]],
    times: Sequence[float],
    start_velocity: Tuple[float, float, float] | None = None,
    end_velocity: Tuple[float, float, float] | None = None,
) -> List[Tuple[MinimumJerkSegment, MinimumJerkSegment, MinimumJerkSegment]]:
    """Generate a 3D minimum-jerk trajectory across waypoints.

    Args:
        waypoints: Sequence of 3D positions.
        times: Segment durations for each waypoint transition.
        start_velocity: Optional starting velocity vector.
        end_velocity: Optional ending velocity vector.

    Returns:
        List of tuples containing per-axis polynomial segments.
    """

    if len(waypoints) < 2:
        raise ValueError("At least two waypoints are required")
    if len(times) != len(waypoints) - 1:
        raise ValueError("Times must have length len(waypoints) - 1")

    trajectory: List[
        Tuple[MinimumJerkSegment, MinimumJerkSegment, MinimumJerkSegment]
    ] = []

    prev_velocity = start_velocity or (0.0, 0.0, 0.0)

    for idx in range(len(waypoints) - 1):
        start = waypoints[idx]
        end = waypoints[idx + 1]
        duration = times[idx]

        next_velocity = (
            end_velocity if idx == len(waypoints) - 2 and end_velocity is not None else prev_velocity
        )

        segments: List[MinimumJerkSegment] = []
        for axis in range(3):
            coeffs = _minimum_jerk_coefficients(
                start=(start[axis], prev_velocity[axis], 0.0),
                end=(end[axis], next_velocity[axis], 0.0),
                duration=duration,
            )
            segments.append(MinimumJerkSegment(coefficients=coeffs, duration=duration))
        trajectory.append(tuple(segments))

    return trajectory
