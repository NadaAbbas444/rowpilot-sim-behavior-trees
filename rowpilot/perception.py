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
    pts: np.ndarray, inlier_tol: float = 0.10, min_inliers: int = 5
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
    good = (counts >= max(min_inliers, 0.6 * counts.max())) & (np.abs(b) < math.tan(math.radians(35)))
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


def _dominant_slope(pts: np.ndarray, tol: float = 0.10) -> float:
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


def _drop_facing_surfaces(pts: np.ndarray, k: int = 2) -> np.ndarray:
    """Remove points on surfaces that face the drone (row end faces seen from the headland, posts).

    Lane walls run along the lane: walking along the scan (in angle order), consecutive points on a
    wall move mainly forward, while points on a surface facing the camera move mainly sideways."""
    if len(pts) < 2 * k + 1:
        return pts
    pts = pts[np.argsort(np.arctan2(pts[:, 1], pts[:, 0]))]
    idx = np.arange(len(pts))
    lo, hi = np.clip(idx - k, 0, len(pts) - 1), np.clip(idx + k, 0, len(pts) - 1)
    d = pts[hi] - pts[lo]
    return pts[np.abs(d[:, 0]) > 0.7 * np.abs(d[:, 1])]


def estimate_lane(points: np.ndarray, path_width: float, min_points: int = 5, max_forward: float = 4.0) -> LaneEstimate:
    """Estimate the lane centre line from body-frame scan points of shape (N, 2)."""
    if points is None or len(points) == 0:
        return LaneEstimate()
    pts = _drop_facing_surfaces(points[(points[:, 0] > 0.05) & (points[:, 0] < max_forward)])
    # Split into left and right faces relative to the lane direction, not the nose: with a heading
    # error the near face crosses in front of the nose, and its far points would land on the wrong side.
    b0 = _dominant_slope(pts)
    sl, sr = _side_faces(pts, b0, min_points)
    if sl is None and sr is None:
        return LaneEstimate(n_left=0, n_right=0)
    # Refine the shared slope on the outer half of each face's points (the stems and the back of the
    # foliage line up; leaf tips sticking into the lane do not), then measure the faces again.
    outer = [f[0][f[1] >= np.median(f[1])] for f in (sl, sr) if f is not None]
    A = np.vstack(
        [np.column_stack([np.eye(len(outer))[i][None, :].repeat(len(o), 0), o[:, 0]]) for i, o in enumerate(outer)]
    )
    sol, *_ = np.linalg.lstsq(A, np.concatenate([o[:, 1] for o in outer]), rcond=None)
    b = float(sol[-1]) if abs(sol[-1]) < math.tan(math.radians(35)) else b0
    sl, sr = _side_faces(pts, b, min_points)
    n_l = 0 if sl is None else len(sl[0])
    n_r = 0 if sr is None else len(sr[0])
    if sl is None and sr is None:
        return LaneEstimate()
    k = math.sqrt(1 + b * b)
    # Each face is the median of its band; leaves pull both faces inwards alike, so the centre is unbiased.
    al = None if sl is None else float(np.quantile(sl[1], ENVELOPE_Q))
    ar = None if sr is None else float(np.quantile(sr[1], ENVELOPE_Q))
    if al is not None and ar is not None:
        if abs((al + ar) / k - path_width) > 0.25:  # sanity: the faces are one lane width apart
            return LaneEstimate(n_left=n_l, n_right=n_r)
        c = (al - ar) / 2
    elif al is not None:
        c = al - path_width / 2 * k
    else:
        c = -ar + path_width / 2 * k

    heading = math.atan(b)
    offset = c * math.cos(heading)  # perpendicular distance from the drone to the centre line
    if abs(offset) > path_width / 2:  # the centre line cannot be outside the lane we are in
        return LaneEstimate(n_left=n_l, n_right=n_r)
    return LaneEstimate(offset, heading, n_l, n_r, True)


ENVELOPE_Q = 0.5  # median of the face band: leaf bias is the same on both sides and cancels
FACE_DEPTH_M = 0.30  # foliage is at most this deep, measured across the lane


def _side_faces(pts: np.ndarray, b: float, min_points: int):
    """Points of the nearest crop face on each side, as (points, outward distance) or None.

    Outward distance is measured across the lane from the drone (perpendicular-ish, along body y
    corrected by the lane slope b). The nearest dense cluster on each side is the adjacent crop row;
    returns from rows further away (seen past row ends from the headland) are left out."""
    v = pts[:, 1] - b * pts[:, 0]
    out = []
    for sign in (1.0, -1.0):
        d = sign * v
        side = d > 0
        face = _nearest_cluster(d[side], min_points)
        if face is None:
            out.append(None)
            continue
        out.append((pts[side][face], d[side][face]))
    return out[0], out[1]


FACE_BAND_M = 0.16  # a crop face, leaves included, piles its points up within this band


def _nearest_cluster(d: np.ndarray, min_points: int) -> np.ndarray | None:
    """The nearest group of points that looks like a crop face, as a mask over d.

    First the nearest window holding enough points (the adjacent row, not rows further away), then
    inside it the densest FACE_BAND_M band: a face piles its points up there, while the short end
    faces of rows seen from the headland spread thinly across the window."""
    for start in np.sort(d):
        m = (d >= start) & (d <= start + FACE_DEPTH_M)
        if m.sum() >= min_points:
            cand = np.sort(d[m])
            counts = [((cand >= c) & (cand <= c + FACE_BAND_M)).sum() for c in cand]
            lo = cand[int(np.argmax(counts))]
            band = m & (d >= lo) & (d <= lo + FACE_BAND_M)
            return band if band.sum() >= min_points else None
    return None
