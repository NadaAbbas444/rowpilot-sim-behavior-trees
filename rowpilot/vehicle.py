"""The interface the mission talks to. The simulator implements it now; a PX4 (ROS 2 / MAVSDK)
backend implements the same methods on the real drone, so the behaviour tree does not change."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np

from .control import Command


@dataclass(frozen=True)
class VehicleState:
    """What the companion computer knows. Pose is the visual-odometry estimate, not the truth."""

    t: float
    x: float
    y: float
    z: float
    yaw: float
    vo_healthy: bool
    battery_pct: float
    armed: bool
    landed: bool


class Vehicle(Protocol):
    def state(self) -> VehicleState: ...

    def arm(self) -> bool: ...

    def disarm(self) -> None: ...

    def send(self, cmd: Command) -> None:
        """Body-frame velocity setpoint, held until the next one."""

    def depth_scan(self) -> np.ndarray:
        """Horizontal depth scan as body-frame points (N, 2): x forward, y left."""
