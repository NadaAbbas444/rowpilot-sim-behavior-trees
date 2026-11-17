"""Lane perception: from a horizontal depth scan to "how far off the lane centre am I, and at what angle".

Input points are in the body frame (x forward, y left), e.g. one row of the RealSense depth image
projected to the ground plane. Each side of the lane is a crop face, which is a straight line in
the body frame: y = a + b * x. We fit a line to each side and average them into the centre line.

lateral_offset > 0  means the lane centre is to the drone's left (steer left).
heading_error  > 0  means the lane runs to the left of the nose (yaw left, counter-clockwise).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class LaneEstimate:
    lateral_offset: float = 0.0
    heading_error: float = 0.0
    n_left: int = 0
    n_right: int = 0
    valid: bool = False


def _fit_line(
    pts: np.ndarray, inlier_tol: float = 0.06, min_inliers: int = 5
) -> tuple[float, float, np.ndarray] | None:
    """Fit y = a + b x robustly (RANSAC over point pairs, then least squares on the inliers).

    Stray returns (row end faces, crop rows further away, leaves) are rejected as outliers."""
    n = len(pts)
    if n < min_inliers:
        return None
    rng = np.random.default_rng(0)  # deterministic: same scan, same answer
    i, j = rng.integers(0, n, (2, min(60, n * (n - 1) // 2)))
    i, j = i[i != j], j[i != j]
    dx = pts[j, 0] - pts[i, 0]
    ok = np.abs(dx) > 0.1
    if not ok.any():
        return None
    b = (pts[j, 1] - pts[i, 1])[ok] / dx[ok]
    a = pts[i, 1][ok] - b * pts[i, 0][ok]
    # Perpendicular residual of every point to every candidate line.
    resid = np.abs(pts[None, :, 1] - (a[:, None] + b[:, None] * pts[None, :, 0])) / np.sqrt(1 + b[:, None] ** 2)
    inl = resid < inlier_tol
    counts = inl.sum(axis=1)
    # Several crop faces can be visible at once (e.g. from the headland). The lane's own face is the
    # nearest well-supported line; steep lines are end faces, not the side of the lane.
    good = (counts >= max(min_inliers, 0.3 * counts.max())) & (np.abs(b) < math.tan(math.radians(35)))
    if not good.any():
        return None
    dist = np.abs(a) / np.sqrt(1 + b**2)
    best = np.flatnonzero(good)[dist[good].argmin()]
    keep = inl[best]
    if keep.sum() < min_inliers or np.ptp(pts[keep, 0]) < 0.3:
        return None
    A = np.column_stack([np.ones(keep.sum()), pts[keep, 0]])
    (a, b), *_ = np.linalg.lstsq(A, pts[keep, 1], rcond=None)
    return float(a), float(b), pts[keep]


def _dominant_slope(pts: np.ndarray, tol: float = 0.06) -> float:
    """Slope shared by most points (both crop faces are parallel). 0 if there is too little to go on."""
    n = len(pts)
    if n < 4:
        return 0.0
    rng = np.random.default_rng(1)
    i, j = rng.integers(0, n, (2, 80))
    dx = pts[j, 0] - pts[i, 0]
    ok = (i != j) & (np.abs(dx) > 0.3)
    if not ok.any():
        return 0.0
    b = (pts[j, 1] - pts[i, 1])[ok] / dx[ok]
    a = pts[i, 1][ok] - b * pts[i, 0][ok]
    flat = np.abs(b) < math.tan(math.radians(35))
    if not flat.any():
        return 0.0
    a, b = a[flat], b[flat]
    resid = np.abs(pts[None, :, 1] - (a[:, None] + b[:, None] * pts[None, :, 0]))
    return float(b[(resid < tol).sum(axis=1).argmax()])


def estimate_lane(points: np.ndarray, path_width: float, min_points: int = 5, max_forward: float = 4.0) -> LaneEstimate:
    """Estimate the lane centre line from body-frame scan points of shape (N, 2)."""
    if points is None or len(points) == 0:
        return LaneEstimate()
    pts = points[(points[:, 0] > 0.05) & (points[:, 0] < max_forward)]
    # Split into left and right faces relative to the lane direction, not the nose: with a heading
    # error the near face crosses in front of the nose, and its far points would land on the wrong side.
    b0 = _dominant_slope(pts)
    side = pts[:, 1] - b0 * pts[:, 0]
    left, right = pts[side > 0], pts[side < 0]

    fl = _fit_line(left, min_inliers=min_points) if len(left) >= min_points else None
    fr = _fit_line(right, min_inliers=min_points) if len(right) >= min_points else None
    n_l, n_r = len(left), len(right)

    if fl and fr:
        (al, bl, pl), (ar, br, pr) = fl, fr
        b = (bl + br) / 2
        width = (al - ar) / math.sqrt(1 + b * b)
        # Sanity: both faces parallel and the measured gap close to the real lane width.
        if abs(bl - br) > 0.12 or abs(width - path_width) > 0.3:
            return LaneEstimate(n_left=n_l, n_right=n_r)
        # Joint fit: two intercepts, one shared slope, so the better-seen face sets the angle.
        A = np.vstack(
            [
                np.column_stack([np.ones(len(pl)), np.zeros(len(pl)), pl[:, 0]]),
                np.column_stack([np.zeros(len(pr)), np.ones(len(pr)), pr[:, 0]]),
            ]
        )
        (al, ar, b), *_ = np.linalg.lstsq(A, np.concatenate([pl[:, 1], pr[:, 1]]), rcond=None)
        c = (al + ar) / 2
    elif fl or fr:
        a, b, _ = fl or fr
        # One face only: the centre line is half a lane width away from it, measured perpendicular.
        c = a - path_width / 2 * math.sqrt(1 + b * b) if fl else a + path_width / 2 * math.sqrt(1 + b * b)
    else:
        return LaneEstimate(n_left=n_l, n_right=n_r)

    heading = math.atan(b)
    offset = c * math.cos(heading)  # perpendicular distance from the drone to the centre line
    if abs(offset) > path_width / 2:  # the centre line cannot be outside the lane we are in
        return LaneEstimate(n_left=n_l, n_right=n_r)
    return LaneEstimate(offset, heading, n_l, n_r, True)
