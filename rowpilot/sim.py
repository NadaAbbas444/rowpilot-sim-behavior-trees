"""A lightweight greenhouse simulator implementing the Vehicle interface.

It is not a physics engine. It models what the mission logic has to cope with:
- velocity setpoints answered with a lag (like a tuned PX4 in offboard mode),
- gusts pushing the drone sideways,
- visual odometry that drifts and can drop out,
- a battery that drains faster when moving,
- a forward depth scan (RealSense-like fan of rays) that hits the crop faces.
The true state is kept separately so tests can judge the mission against ground truth.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

import numpy as np

from .control import Command
from .layout import GreenhouseLayout
from .vehicle import VehicleState


@dataclass
class SimConfig:
    seed: int = 0
    start_battery_pct: float = 100.0
    hover_drain_pct_s: float = 0.12  # about 14 minutes of hover on a full pack
    move_drain_pct_per_m: float = 0.05
    # The flight controller's response to velocity targets, measured on ArduCopter SITL (GUIDED, default
    # jerk/acceleration limits): 0.5 s dead time, then a first-order lag of 0.3 s.
    command_delay_s: float = 0.5
    velocity_lag_s: float = 0.3
    wind_sigma: float = 0.3  # m/s, gust strength (greenhouse fans, open vents)
    wind_tau_s: float = 1.5
    vo_drift_per_sqrt_m: float = 0.01  # random-walk drift of the VO position estimate
    vo_yaw_drift_per_sqrt_m: float = 0.002
    vo_dropouts: list[tuple[float, float]] = field(default_factory=list)  # (start time s, duration s)
    depth_fov_deg: float = 87.0  # RealSense D435/D455 horizontal field of view
    depth_rays: int = 64
    depth_max_range: float = 6.0
    depth_noise_m: float = 0.01
    depth_dropout: float = 0.05  # share of pixels with no depth (dark leaves, edges)
    foliage_m: float = 0.14  # how far leaves stick out of the crop face into the lane
    drone_radius: float = 0.25


class SimVehicle:
    def __init__(self, layout: GreenhouseLayout, cfg: SimConfig | None = None):
        self.layout, self.cfg = layout, cfg or SimConfig()
        self.rng = np.random.default_rng(self.cfg.seed)
        self.t = 0.0
        self.x, self.y = layout.home
        self.z, self.yaw = 0.0, 0.0
        self.vx = self.vy = self.vz = 0.0  # world frame
        self.wind = np.zeros(2)
        self.cmd = Command()
        self._cmd_q: deque[tuple[float, Command]] = deque()
        self.armed = False
        self.battery = self.cfg.start_battery_pct
        self.crashed = False
        self.drift = np.zeros(3)  # VO error: dx, dy, dyaw
        self._frozen_estimate: tuple[float, float, float] | None = None
        self._faces = np.array(layout.crop_face_ys())
        self._centers = np.array(layout.crop_row_centers_y())
        self._ends_x = np.array([0.0, layout.row_length])
        frng = np.random.default_rng(1000 + self.cfg.seed)
        self._fol_k = frng.uniform(2.0, 9.0, (len(self._faces), 4))  # leaf-scale waves, per face
        self._fol_ph = frng.uniform(0, 2 * np.pi, (len(self._faces), 4))

    # ---- Vehicle interface -------------------------------------------------------------
    def vo_healthy(self) -> bool:
        return not any(t0 <= self.t < t0 + d for t0, d in self.cfg.vo_dropouts)

    def state(self) -> VehicleState:
        if self.vo_healthy():
            ex, ey, eyaw = self.x + self.drift[0], self.y + self.drift[1], self.yaw + self.drift[2]
            self._frozen_estimate = (ex, ey, eyaw)
        else:  # lost tracking: the estimate stops updating
            ex, ey, eyaw = self._frozen_estimate or (self.x, self.y, self.yaw)
        return VehicleState(self.t, ex, ey, self.z, eyaw, self.vo_healthy(), self.battery, self.armed, self.landed)

    @property
    def landed(self) -> bool:
        return self.z <= 0.02

    def arm(self) -> bool:
        if self.battery > 5:
            self.armed = True
        return self.armed

    def disarm(self) -> None:
        if self.landed:
            self.armed = False
            self.cmd = Command()

    def send(self, cmd: Command) -> None:
        self.cmd = cmd

    def land(self) -> None:
        """Like a flight controller's land mode: hold position, descend, slow near the ground."""
        self.cmd = Command(vz=-0.5 if self.z > 0.4 else -0.25)

    def depth_scan(self) -> np.ndarray:
        """Cast a horizontal fan of rays and return the nearest crop-face hit of each (body frame)."""
        if self.landed or self.z > self.layout.canopy_height:
            return np.empty((0, 2))
        c = self.cfg
        a = np.radians(np.linspace(-c.depth_fov_deg / 2, c.depth_fov_deg / 2, c.depth_rays))
        th = self.yaw + a
        s, co = np.sin(th), np.cos(th)
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (self._faces[None, :] - self.y) / s[:, None]  # distance along each ray to each face line
            xh = self.x + t * co[:, None]
        ok = (t > 0) & (xh >= 0) & (xh <= self.layout.row_length)
        tf = np.where(ok, t, np.inf)
        face = tf.argmin(axis=1)
        t = tf[np.arange(len(face)), face]
        # Foliage: leaves stick out of the crop face by up to foliage_m, varying along the row
        # (the same leaves every time the drone passes), so the face the camera sees is ragged.
        hit_x = xh[np.arange(len(face)), face]
        p = c.foliage_m * self._foliage(face, hit_x)
        with np.errstate(divide="ignore", invalid="ignore"):
            t = np.where(np.isfinite(t), np.maximum(0.05, t - p / np.abs(s)), t)
        # Short end faces of each crop row, at x = 0 and x = row_length.
        with np.errstate(divide="ignore", invalid="ignore"):
            te = (self._ends_x[None, :] - self.x) / co[:, None]
            yh = self.y + te * s[:, None]
        half = self.layout.crop_width / 2
        on_end = (np.abs(yh[:, :, None] - self._centers[None, None, :]) <= half).any(axis=2)
        t = np.minimum(t, np.where((te > 0) & on_end, te, np.inf).min(axis=1))
        if self.layout.closed_end:  # the end wall closing every lane
            with np.errstate(divide="ignore", invalid="ignore"):
                tw = (self.layout.row_length - self.x) / co
            t = np.minimum(t, np.where(tw > 0, tw, np.inf))
        hit = (t <= c.depth_max_range) & (self.rng.random(len(t)) >= c.depth_dropout)
        r = t[hit] + self.rng.normal(0, c.depth_noise_m, hit.sum()) * (1 + t[hit] ** 2 / 4)
        return np.column_stack([r * np.cos(a[hit]), r * np.sin(a[hit])])

    def _foliage(self, face: np.ndarray, x: np.ndarray) -> np.ndarray:
        """How far (0..1 of foliage_m) leaves stick out at position x along a crop face."""
        with np.errstate(invalid="ignore"):
            w = np.sin(self._fol_k[face] * x[:, None] + self._fol_ph[face]).mean(axis=1)
        return np.nan_to_num(np.clip(0.5 + w, 0.0, 1.0))

    # ---- simulation ---------------------------------------------------------------------
    def step(self, dt: float) -> None:
        c = self.cfg
        self.t += dt
        if not self.armed or self.crashed:
            self.vx = self.vy = self.vz = 0.0
            return
        # The command the flight controller acts on now is the one sent command_delay_s ago.
        self._cmd_q.append((self.t, self.cmd))
        while len(self._cmd_q) > 1 and self._cmd_q[1][0] <= self.t - c.command_delay_s:
            self._cmd_q.popleft()
        cmd = self._cmd_q[0][1]
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        tvx = cy * cmd.vx - sy * cmd.vy
        tvy = sy * cmd.vx + cy * cmd.vy
        airborne = self.z > 0.05
        if airborne:  # Ornstein-Uhlenbeck gusts
            self.wind += -self.wind * dt / c.wind_tau_s + c.wind_sigma * math.sqrt(2 * dt / c.wind_tau_s) * (
                self.rng.normal(size=2)
            )
        else:
            self.wind[:] = 0.0
        # The flight controller's velocity loop sees the real ground velocity (own effort + gust) and
        # closes on it, so steady wind is rejected and only fast changes in the gust leak through.
        k = min(1.0, dt / c.velocity_lag_s)
        self.vx += (tvx - (self.vx + self.wind[0])) * k
        self.vy += (tvy - (self.vy + self.wind[1])) * k
        self.vz += (cmd.vz - self.vz) * k
        if not airborne:
            self.vx = self.vy = 0.0  # no sliding along the floor
        dx, dy = (self.vx + self.wind[0]) * dt, (self.vy + self.wind[1]) * dt
        self.x += dx
        self.y += dy
        self.z = max(0.0, self.z + self.vz * dt)
        if airborne:
            self.yaw = (self.yaw + max(-1.0, min(1.0, cmd.yaw_rate)) * dt + math.pi) % (2 * math.pi) - math.pi

        dist = math.hypot(dx, dy)
        if dist > 0:
            self.drift[:2] += self.rng.normal(0, c.vo_drift_per_sqrt_m * math.sqrt(dist), 2)
            self.drift[2] += self.rng.normal(0, c.vo_yaw_drift_per_sqrt_m * math.sqrt(dist))
        self.battery = max(
            0.0, self.battery - (c.hover_drain_pct_s * dt if airborne else 0.01 * dt) - c.move_drain_pct_per_m * dist
        )
        if airborne and self.layout.in_crop(self.x, self.y, margin=c.drone_radius):
            self.crashed = True
        if airborne and self.layout.closed_end and self.x > self.layout.row_length - c.drone_radius:
            self.crashed = True  # flew into the end wall
        if self.battery <= 0 and airborne:
            self.crashed = True
