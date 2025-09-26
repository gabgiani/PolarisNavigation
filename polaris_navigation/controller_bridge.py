"""Controller bridge abstractions."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence, Tuple


@dataclass
class Setpoint:
    position: Tuple[float, float, float]
    velocity: Tuple[float, float, float]
    yaw: float = 0.0
    yaw_rate: float = 0.0


class SetpointPublisher:
    """Mock setpoint publisher emulating a MAVROS offboard interface."""

    def __init__(self, publish_callback: callable | None = None) -> None:
        self.publish_callback = publish_callback
        self.published: list[Setpoint] = []

    def publish(self, setpoint: Setpoint) -> None:
        self.published.append(setpoint)
        if self.publish_callback is not None:
            self.publish_callback(setpoint)

    def publish_trajectory(
        self,
        samples: Sequence[Tuple[Tuple[float, float, float], Tuple[float, float, float]]],
        yaw: float = 0.0,
    ) -> None:
        for position, velocity in samples:
            self.publish(Setpoint(position=position, velocity=velocity, yaw=yaw))
