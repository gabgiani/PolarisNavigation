"""Grid-based global planner implementations."""

from __future__ import annotations

from dataclasses import dataclass, field
from heapq import heappop, heappush
from typing import Dict, List, Optional, Tuple

from .world_model import OccupancyGrid


@dataclass(order=True)
class PriorityNode:
    """Node wrapper that enables priority queue ordering by total cost."""

    priority: float
    position: Tuple[int, int] = field(compare=False)


class AStarPlanner:
    """A* planner that operates on a 2D occupancy grid."""

    def __init__(self, heuristic: str = "euclidean") -> None:
        self.heuristic = heuristic

    def plan(
        self,
        grid: OccupancyGrid,
        start: Tuple[int, int],
        goal: Tuple[int, int],
        allow_diagonal: bool = True,
    ) -> List[Tuple[int, int]]:
        """Plan a collision-free path on the grid from start to goal.

        Args:
            grid: OccupancyGrid containing obstacle data.
            start: Starting cell index (x, y).
            goal: Goal cell index (x, y).
            allow_diagonal: Whether to allow diagonal moves.

        Returns:
            List of grid coordinates from start to goal inclusive.

        Raises:
            ValueError: If start/goal invalid or path not found.
        """

        if not grid.is_index_valid(start) or not grid.is_index_valid(goal):
            raise ValueError("Start or goal is outside of the grid bounds")
        if grid.is_occupied(start) or grid.is_occupied(goal):
            raise ValueError("Start or goal cell is occupied")

        open_set: List[PriorityNode] = []
        heappush(open_set, PriorityNode(0.0, start))

        came_from: Dict[Tuple[int, int], Optional[Tuple[int, int]]] = {start: None}
        g_score: Dict[Tuple[int, int], float] = {start: 0.0}

        while open_set:
            current = heappop(open_set).position
            if current == goal:
                return self._reconstruct_path(came_from, current)

            for neighbor in grid.neighbors(current, allow_diagonal=allow_diagonal):
                tentative_g = g_score[current] + grid.cost_between(current, neighbor)
                if tentative_g < g_score.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self._heuristic(neighbor, goal)
                    heappush(open_set, PriorityNode(f_score, neighbor))

        raise ValueError("No path found between start and goal")

    def _heuristic(self, node: Tuple[int, int], goal: Tuple[int, int]) -> float:
        dx = abs(node[0] - goal[0])
        dy = abs(node[1] - goal[1])
        if self.heuristic == "manhattan":
            return float(dx + dy)
        if self.heuristic == "octile":
            return float(max(dx, dy) + (2**0.5 - 1) * min(dx, dy))
        return float((dx**2 + dy**2) ** 0.5)

    @staticmethod
    def _reconstruct_path(
        came_from: Dict[Tuple[int, int], Optional[Tuple[int, int]]],
        current: Tuple[int, int],
    ) -> List[Tuple[int, int]]:
        path: List[Tuple[int, int]] = [current]
        while came_from[current] is not None:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
