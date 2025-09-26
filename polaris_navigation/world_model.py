"""World model primitives for occupancy-based planning."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Iterator, List, Sequence, Tuple


@dataclass
class OccupancyGrid:
    """A simple 2D occupancy grid with inflation utilities."""

    width: int
    height: int
    resolution: float = 1.0

    def __post_init__(self) -> None:
        self._data: List[int] = [0] * (self.width * self.height)

    def index(self, cell: Tuple[int, int]) -> int:
        x, y = cell
        return y * self.width + x

    def is_index_valid(self, cell: Tuple[int, int]) -> bool:
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def set_obstacle(self, cell: Tuple[int, int]) -> None:
        if self.is_index_valid(cell):
            self._data[self.index(cell)] = 1

    def clear_obstacle(self, cell: Tuple[int, int]) -> None:
        if self.is_index_valid(cell):
            self._data[self.index(cell)] = 0

    def is_occupied(self, cell: Tuple[int, int]) -> bool:
        if not self.is_index_valid(cell):
            return True
        return self._data[self.index(cell)] > 0

    def inflate(self, radius: int) -> None:
        """Inflate obstacles by a given radius in grid cells."""

        if radius <= 0:
            return
        original = self._data[:]
        for x in range(self.width):
            for y in range(self.height):
                if original[self.index((x, y))] > 0:
                    for nx in range(x - radius, x + radius + 1):
                        for ny in range(y - radius, y + radius + 1):
                            if self.is_index_valid((nx, ny)):
                                self._data[self.index((nx, ny))] = 1

    def neighbors(
        self, cell: Tuple[int, int], allow_diagonal: bool = True
    ) -> Iterator[Tuple[int, int]]:
        steps: Sequence[Tuple[int, int]]
        if allow_diagonal:
            steps = (
                (-1, 0),
                (1, 0),
                (0, -1),
                (0, 1),
                (-1, -1),
                (-1, 1),
                (1, -1),
                (1, 1),
            )
        else:
            steps = ((-1, 0), (1, 0), (0, -1), (0, 1))

        for dx, dy in steps:
            neighbor = (cell[0] + dx, cell[1] + dy)
            if self.is_index_valid(neighbor) and not self.is_occupied(neighbor):
                yield neighbor

    def cost_between(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        if dx == 1 and dy == 1:
            return 2 ** 0.5
        return 1.0

    def set_obstacles(self, cells: Iterable[Tuple[int, int]]) -> None:
        for cell in cells:
            self.set_obstacle(cell)

    def to_ascii(self, path: Sequence[Tuple[int, int]] | None = None) -> str:
        """Render an ASCII representation of the grid with an optional path."""

        path_set = set(path or [])
        rows: List[str] = []
        for y in range(self.height):
            row = []
            for x in range(self.width):
                cell = (x, y)
                if cell in path_set:
                    row.append("*")
                elif self.is_occupied(cell):
                    row.append("#")
                else:
                    row.append(".")
            rows.append("".join(row))
        return "\n".join(rows)
