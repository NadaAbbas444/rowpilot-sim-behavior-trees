"""Greenhouse geometry in the map frame.

Map frame: origin at the start of lane 0, x along the rows, y across them (left when facing +x).
The drone flies in the free *lanes* (paths) between crop rows. Lane i is centred at y = i * pitch.
Crop row k sits between lane k-1 and lane k, so n lanes need n + 1 crop rows.

closed_end (the default, as in the real greenhouse): the lanes are dead ends, closed by the end wall
at x = row_length. The drone enters every lane from the near headland (x < 0), sweeps it to the
closed end, turns round and flies back out the same lane, then crosses the headland to the next one.
With closed_end=False both ends are open and the lanes are swept serpentine: even lanes towards +x,
odd lanes towards -x, turning in the far headland.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class GreenhouseLayout:
    n_lanes: int = 6
    row_length: float = 40.0 / 3  # m, length of the crop rows (x from 0 to row_length)
    path_width: float = 1.6  # m, free width of a lane between the two crop faces
    crop_width: float = 0.4  # m, width of one crop row
    canopy_height: float = 2.6  # m
    headland: float = 1.5  # m, free space before the rows (and beyond them, if the far end is open)
    closed_end: bool = True  # lanes closed at x = row_length: in and back out the same lane
    end_clearance: float = 1.0  # m, how close to the closed end the drone flies before turning round

    @property
    def pitch(self) -> float:
        return self.path_width + self.crop_width

    @property
    def home(self) -> tuple[float, float]:
        """Take-off and landing spot, in the near headland next to lane 0."""
        return (-self.headland - 1.0, 0.0)

    def lane_center_y(self, lane: int) -> float:
        return lane * self.pitch

    def crop_row_centers_y(self) -> list[float]:
        return [(k - 0.5) * self.pitch for k in range(self.n_lanes + 1)]

    def crop_face_ys(self) -> list[float]:
        """y of every long crop face (two per crop row). Faces run from x=0 to x=row_length."""
        faces = []
        for c in self.crop_row_centers_y():
            faces += [c - self.crop_width / 2, c + self.crop_width / 2]
        return faces

    @property
    def far_x(self) -> float:
        """x of the far greenhouse wall: the closed lane ends, or beyond the far headland."""
        return self.row_length if self.closed_end else self.row_length + self.headland + 2.0

    def lane_direction(self, lane: int) -> int:
        """Direction the lane is entered and swept in (+1 towards +x)."""
        return 1 if self.closed_end or lane % 2 == 0 else -1

    def turn_x(self) -> float:
        """Closed lanes: where the drone turns round, end_clearance before the closed end."""
        return self.row_length - self.end_clearance

    def lane_yaw(self, lane: int) -> float:
        return 0.0 if self.lane_direction(lane) > 0 else math.pi

    def lane_entry(self, lane: int) -> tuple[float, float]:
        x = -self.headland if self.lane_direction(lane) > 0 else self.row_length + self.headland
        return (x, self.lane_center_y(lane))

    def lane_exit(self, lane: int) -> tuple[float, float]:
        """Where the drone leaves the lane: back where it came in if the lane is closed."""
        if self.closed_end:
            return self.lane_entry(lane)
        x = self.row_length + self.headland if self.lane_direction(lane) > 0 else -self.headland
        return (x, self.lane_center_y(lane))

    def progress_along_lane(self, lane: int, x: float) -> float:
        """Distance travelled along the crop rows of a lane (0 at the row start, row_length at the end)."""
        return x if self.lane_direction(lane) > 0 else self.row_length - x

    def nearest_lane(self, y: float) -> int:
        return min(max(round(y / self.pitch), 0), self.n_lanes - 1)

    def in_crop(self, x: float, y: float, margin: float = 0.0) -> bool:
        """True if (x, y) lies inside a crop row (expanded by margin, e.g. the drone's radius)."""
        if not (-margin <= x <= self.row_length + margin):
            return False
        half = self.crop_width / 2 + margin
        return any(abs(y - c) <= half for c in self.crop_row_centers_y())
