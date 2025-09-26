"""Mission supervisor tying together the planning stack."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple

from .controller_bridge import SetpointPublisher
from .feasibility_checker import TrajectoryFeasibilityChecker
from .global_planner import AStarPlanner
from .trajectory_generator import generate_minimum_jerk_trajectory
from .world_model import OccupancyGrid


@dataclass
class MissionGoal:
    position: Tuple[int, int]


class MissionSupervisor:
    """Orchestrates the global plan, trajectory generation, and setpoint streaming."""

    def __init__(
        self,
        grid: OccupancyGrid,
        planner: AStarPlanner,
        feasibility_checker: TrajectoryFeasibilityChecker,
        controller: SetpointPublisher,
    ) -> None:
        self.grid = grid
        self.planner = planner
        self.feasibility_checker = feasibility_checker
        self.controller = controller

    def execute_mission(
        self,
        start: Tuple[int, int],
        goal: MissionGoal,
        altitude: float = 2.0,
        cruise_speed: float = 1.0,
    ) -> List[Tuple[int, int]]:
        """Plan, verify, and stream a trajectory to the controller."""

        path = self.planner.plan(self.grid, start, goal.position)

        waypoints = [
            (self.grid.index_to_world(cell)[0], self.grid.index_to_world(cell)[1], altitude)
            for cell in path
        ]

        segment_times = [
            max(2.5, 2.0 * self.grid.resolution / max(cruise_speed, 1e-3))
            for _ in range(len(waypoints) - 1)
        ]
        trajectory = generate_minimum_jerk_trajectory(waypoints, segment_times)

        self.feasibility_checker.check(trajectory, sample_rate=50.0)

        samples = []
        for segments in trajectory:
            duration = segments[0].duration
            t = 0.0
            dt = duration / 10
            while t <= duration + 1e-6:
                state = [seg.evaluate(min(t, duration)) for seg in segments]
                position = tuple(component[0] for component in state)
                velocity = tuple(component[1] for component in state)
                samples.append((position, velocity))
                t += dt

        self.controller.publish_trajectory(samples)
        return path


# Extend OccupancyGrid with conversion helper

def _index_to_world(self: OccupancyGrid, cell: Tuple[int, int]) -> Tuple[float, float]:
    x, y = cell
    return (x + 0.5) * self.resolution, (y + 0.5) * self.resolution


OccupancyGrid.index_to_world = _index_to_world  # type: ignore[attr-defined]
