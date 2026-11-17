"""Fly the same behaviour tree against PX4 SITL + Gazebo.

Px4Vehicle implements the Vehicle interface over MAVSDK:
  state()       PX4's EKF2 estimate (fused from the external-vision odometry, no GPS)
  send()        offboard body-frame velocity setpoints
  depth_scan()  the forward Gazebo lidar, cropped to the RealSense field of view

Frames: the Gazebo world is the mission's map frame (ENU, x along the rows). PX4 reports a local NED
frame whose origin is where it booted (the spawn point = layout.home), so
  map x = east + home_x,  map y = north + home_y,  map yaw = 90 deg - heading.
On the real drone, swap the lidar for a row of the RealSense depth image; the rest stays the same.

Needs: pip install mavsdk, and Gazebo's Python bindings (python3-gz-transport13, python3-gz-msgs10).
"""

from __future__ import annotations

import json
import math
import threading
import time
from pathlib import Path

import numpy as np
from py_trees.common import Status

from .control import Command
from .layout import GreenhouseLayout
from .mission import MissionContext, MissionParams, SweepLane, build_tree
from .vehicle import VehicleState

REALSENSE_HFOV_DEG = 87.0
LIDAR_FORWARD_OFFSET_M = 0.12


class GazeboFeeds:
    """Latest forward scan and true pose from Gazebo (truth is for scoring only, never for flying)."""

    def __init__(self, world: str, model: str):
        from gz.msgs10.laserscan_pb2 import LaserScan
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node

        self.model = model
        self.scan = np.empty((0, 2))
        self.truth: tuple[float, float, float] | None = None
        self._lock = threading.Lock()
        self.node = Node()
        scan_topic = f"/world/{world}/model/{model}/link/link/sensor/lidar_2d_v2/scan"
        if not self.node.subscribe(LaserScan, scan_topic, self._on_scan):
            raise RuntimeError(f"cannot subscribe to {scan_topic}")
        self.node.subscribe(Pose_V, f"/world/{world}/pose/info", self._on_pose)

    def _on_scan(self, msg) -> None:
        r = np.asarray(msg.ranges, dtype=float)
        a = msg.angle_min + msg.angle_step * np.arange(len(r))
        keep = np.isfinite(r) & (r > msg.range_min) & (r < 6.0) & (np.abs(a) <= math.radians(REALSENSE_HFOV_DEG / 2))
        pts = np.column_stack([r[keep] * np.cos(a[keep]) + LIDAR_FORWARD_OFFSET_M, r[keep] * np.sin(a[keep])])
        with self._lock:
            self.scan = pts

    def _on_pose(self, msg) -> None:
        for p in msg.pose:
            if p.name == self.model:
                q = p.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                with self._lock:
                    self.truth = (p.position.x, p.position.y, yaw)
                return

    def latest_scan(self) -> np.ndarray:
        with self._lock:
            return self.scan.copy()


class Px4Vehicle:
    def __init__(self, layout: GreenhouseLayout, url: str, feeds: GazeboFeeds, connect_timeout_s: float = 30.0):
        from mavsdk import ComponentType, Configuration, Mavsdk
        from mavsdk.plugins.action import Action
        from mavsdk.plugins.offboard import Offboard, VelocityBodyYawspeed
        from mavsdk.plugins.telemetry import LandedState, Telemetry

        self._VBY, self._ON_GROUND = VelocityBodyYawspeed, LandedState.ON_GROUND
        self.layout, self.feeds = layout, feeds
        self.mav = Mavsdk(Configuration.create_with_component_type(ComponentType.GROUND_STATION))
        self.mav.add_any_connection(url)
        self.system = self.mav.first_autopilot(connect_timeout_s)
        if self.system is None:
            raise RuntimeError(f"no PX4 autopilot on {url}")
        self.tel, self.offboard, self.action = Telemetry(self.system), Offboard(self.system), Action(self.system)
        self.t0 = time.monotonic()

    def wait_ready(self, timeout_s: float = 120.0) -> None:
        """Wait until EKF2 has a local position from the vision odometry and PX4 is ready to arm."""
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            h = self.tel.health()
            if h.is_local_position_ok and h.is_armable:
                return
            time.sleep(0.5)
        raise TimeoutError(f"PX4 not ready to arm: {vars(self.tel.health())}")

    def state(self) -> VehicleState:
        pv = self.tel.position_velocity_ned().position
        hx, hy = self.layout.home
        yaw = math.radians(90.0 - self.tel.heading().heading_deg)
        yaw = (yaw + math.pi) % (2 * math.pi) - math.pi
        landed = self.tel.landed_state() == self._ON_GROUND
        return VehicleState(
            t=time.monotonic() - self.t0,
            x=pv.east_m + hx,
            y=pv.north_m + hy,
            z=0.0 if landed else -pv.down_m,
            yaw=yaw,
            vo_healthy=self.tel.health().is_local_position_ok,
            battery_pct=self.tel.battery().remaining_percent,
            armed=self.tel.armed(),
            landed=landed,
        )

    def arm(self) -> bool:
        self.action.arm()
        # PX4 needs a setpoint stream before offboard mode is accepted.
        for _ in range(10):
            self.send(Command())
            time.sleep(0.05)
        self.offboard.start()
        return self.tel.armed()

    def disarm(self) -> None:
        try:
            self.offboard.stop()
        finally:
            if self.tel.armed():
                self.action.disarm()

    def send(self, cmd: Command) -> None:
        # Mission body frame is forward-left-up with CCW yaw; MAVSDK is forward-right-down with CW yaw.
        self.offboard.set_velocity_body(self._VBY(cmd.vx, -cmd.vy, -cmd.vz, -math.degrees(cmd.yaw_rate)))

    def depth_scan(self) -> np.ndarray:
        return self.feeds.latest_scan()


def run_px4(
    layout: GreenhouseLayout,
    params: MissionParams,
    url: str = "udpin://0.0.0.0:14540",
    world: str = "greenhouse",
    model: str = "gh_x500_0",
    max_time_s: float = 900.0,
    out: Path | None = None,
    tick_hz: float = 10.0,
) -> dict:
    """Run the mission tree against PX4 in real time. Returns a summary scored against Gazebo truth."""
    feeds = GazeboFeeds(world, model)
    veh = Px4Vehicle(layout, url, feeds)
    veh.wait_ready()
    events, trace = [], []

    def emit(ev: dict) -> None:
        events.append(ev)
        if ev["type"] in ("log", "end"):
            print(f"{ev['t']:7.1f}s  {ev['node']:<24} {ev['msg']}", flush=True)

    ctx = MissionContext(veh, layout, params, dt=1.0 / tick_hz)
    ctx.log_fn = lambda node, msg: emit({"t": round(ctx.s.t, 2), "type": "log", "node": node, "msg": msg})
    root = build_tree(ctx)
    last: dict[str, Status] = {}
    lane_sq: dict[int, list[float]] = {}
    outcome, hit_crop = "timeout", False
    next_tick = time.monotonic()
    while True:
        ctx.pre_tick()
        root.tick_once()
        for node in root.iterate():
            if last.get(node.name) != node.status:
                last[node.name] = node.status
                emit({"t": round(ctx.s.t, 2), "type": "status", "node": node.name, "status": node.status.value})
        tip, s, tr = root.tip(), ctx.s, feeds.truth
        row = {"t": round(s.t, 2), "est_x": s.x, "est_y": s.y, "z": s.z, "battery": s.battery_pct}
        if tr:
            row.update(x=tr[0], y=tr[1], yaw=tr[2])
            hit_crop |= s.z > 0.3 and layout.in_crop(tr[0], tr[1], margin=0.25)
            if isinstance(tip, SweepLane) and 0 < layout.progress_along_lane(tip.lane_idx, tr[0]) < layout.row_length:
                lane_sq.setdefault(tip.lane_idx, []).append((tr[1] - layout.lane_center_y(tip.lane_idx)) ** 2)
        row["node"] = tip.name if tip else ""
        trace.append(row)
        if root.status != Status.RUNNING and s.landed:
            outcome = "completed" if root.status == Status.SUCCESS else f"aborted: {ctx.abort_reason}"
            break
        if s.t > max_time_s:
            break
        next_tick += 1.0 / tick_hz
        time.sleep(max(0.0, next_tick - time.monotonic()))

    if hit_crop and outcome == "completed":
        outcome = "completed (touched crop)"
    summary = {
        "outcome": outcome,
        "duration_s": round(ctx.s.t, 1),
        "battery_end_pct": round(ctx.s.battery_pct, 1),
        "lanes_completed": ctx.lanes_completed,
        "lane_rms_error_m": {k: round(math.sqrt(sum(v) / len(v)), 3) for k, v in sorted(lane_sq.items())},
        "max_lane_error_m": round(max((math.sqrt(max(v)) for v in lane_sq.values()), default=0.0), 3),
    }
    emit({"t": round(ctx.s.t, 2), "type": "end", "node": "Root", "msg": outcome})
    if out:
        out.mkdir(parents=True, exist_ok=True)
        (out / "summary.json").write_text(json.dumps(summary, indent=1))
        (out / "events.jsonl").write_text("".join(json.dumps(e) + "\n" for e in events))
        (out / "trace.json").write_text(json.dumps(trace))
    return summary
