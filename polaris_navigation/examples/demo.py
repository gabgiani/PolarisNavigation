"""Example pipeline showing the Polaris navigation components working together."""

from __future__ import annotations

from polaris_navigation import (
    AStarPlanner,
    FeasibilityLimits,
    MissionGoal,
    MissionSupervisor,
    OccupancyGrid,
    SetpointPublisher,
    TrajectoryFeasibilityChecker,
)


def main() -> None:
    grid = OccupancyGrid(width=10, height=6, resolution=1.0)
    grid.set_obstacles({(3, y) for y in range(1, 5)} - {(3, 3)})

    planner = AStarPlanner()

    limits = FeasibilityLimits(
        max_velocity=2.0,
        max_acceleration=3.0,
        max_jerk=10.0,
        min_clearance=0.2,
    )

    feasibility_checker = TrajectoryFeasibilityChecker(limits)

    controller = SetpointPublisher(lambda sp: print(f"Setpoint: {sp}"))

    supervisor = MissionSupervisor(grid, planner, feasibility_checker, controller)

    start = (0, 0)
    goal = MissionGoal(position=(9, 5))

    path = supervisor.execute_mission(start, goal)
    print("Planned path:", path)
    print(grid.to_ascii(path))


if __name__ == "__main__":
    main()
