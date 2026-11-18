"""Fly the behaviour tree with ArduPilot (ArduCopter), as on the real drone, in a GPS-denied greenhouse.

ArduPilotVehicle implements the Vehicle interface over MAVLink (pymavlink):
  state()       EKF3's estimate, which fuses the OpenVINS odometry (ExternalNav), no GPS, no compass
  send()        GUIDED-mode body-frame velocity targets (SET_POSITION_TARGET_LOCAL_NED, BODY_NED)
  land()        LAND mode: ArduCopter descends, detects touchdown and disarms
  depth_scan()  the D435i depth image turned into a lane scan

It also forwards OpenVINS to ArduPilot as MAVLink ODOMETRY (frame LOCAL_FRD, child BODY_FRD, the
frames ArduPilot accepts for visual odometry) and sets the EKF origin, which ArduCopter needs for
GUIDED flight without GPS.

Frames: map M is ENU (x along the rows). ArduPilot's local frame is NED with the same origin:
north = map y, east = map x, down = -map z; yaw_NED = 90 deg - yaw_map.
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
from .sensors import D435i, depth_to_lane_scan
from .vehicle import VehicleState
from .vio_link import BodyPose, VioAligner, rot_to_quat, subscribe_vio

GUIDED, LAND = 4, 9  # ArduCopter custom modes
VIO_STALE_S = 0.5
# Matrix taking ENU vectors to NED, and FLU body vectors to FRD.
ENU_TO_NED = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])
# Ignore position, acceleration and yaw; use velocity and yaw rate.
VEL_YAWRATE_MASK = 0b0000_0101_1100_0111


class GazeboSensors:
    """D435i depth image, Gazebo ground truth (scoring only) and the simulation clock."""

    def __init__(self, world: str, model: str, prefix: str):
        from gz.msgs10.clock_pb2 import Clock
        from gz.msgs10.image_pb2 import Image
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node

        self.model, self.depth, self.truth, self.sim_t = model, None, None, 0.0
        self._lock = threading.Lock()
        self.node = Node()
        self.node.subscribe(Image, f"{prefix}/depth", self._on_depth)
        self.node.subscribe(Pose_V, f"/world/{world}/pose/info", self._on_pose)
        self.node.subscribe(Clock, f"/world/{world}/clock", self._on_clock)

    def _on_depth(self, m) -> None:
        d = np.frombuffer(m.data, np.float32).reshape(m.height, m.width)
        with self._lock:
            self.depth = d

    def _on_pose(self, m) -> None:
        for p in m.pose:
            if p.name == self.model:
                q = p.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                with self._lock:
                    self.truth = (p.position.x, p.position.y, p.position.z, yaw)
                return

    def _on_clock(self, m) -> None:
        self.sim_t = m.sim.sec + 1e-9 * m.sim.nsec

    def latest_depth(self) -> np.ndarray | None:
        with self._lock:
            return self.depth


class ArduPilotVehicle:
    def __init__(
        self,
        layout: GreenhouseLayout,
        sensors: GazeboSensors,
        aligner: VioAligner,
        url: str = "tcp:127.0.0.1:5760",
        d435i: D435i | None = None,
    ):
        from pymavlink import mavutil

        self.mavutil, self.layout, self.sensors, self.aligner = mavutil, layout, sensors, aligner
        self.cam = (d435i or D435i()).depth()
        self.conn = mavutil.mavlink_connection(url, source_system=200, source_component=191)
        self.conn.wait_heartbeat(timeout=60)
        self.msgs: dict[str, object] = {}
        self._took_off = self._landing = False
        self._origin_set = False
        self._z_offset = 0.0  # EKF3 local height is relative to where it started; see wait_ready()
        self._lock = threading.Lock()
        self._stop = False
        self._reader = threading.Thread(target=self._read, daemon=True)
        self._reader.start()
        for msg_id, hz in (
            ("LOCAL_POSITION_NED", 20),
            ("ATTITUDE", 20),
            ("SYS_STATUS", 2),
            ("EXTENDED_SYS_STATE", 5),
            ("EKF_STATUS_REPORT", 2),
            ("GPS_GLOBAL_ORIGIN", 1),
        ):
            self._command(
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{msg_id}"),
                1e6 / hz,
            )

    # ---- MAVLink plumbing -------------------------------------------------------------------
    def _read(self) -> None:
        while not self._stop:
            m = self.conn.recv_match(blocking=True, timeout=0.5)
            if m is None:
                continue
            t = m.get_type()
            with self._lock:
                self.msgs[t] = m
                if t == "STATUSTEXT":
                    print(f"[ardupilot] {m.text}", flush=True)
                if t == "COMMAND_ACK":
                    self.msgs[f"ACK_{m.command}"] = m

    def _get(self, t: str):
        with self._lock:
            return self.msgs.get(t)

    def _command(self, cmd: int, *params: float, wait_ack: bool = False, timeout: float = 5.0) -> bool:
        p = list(params) + [0.0] * (7 - len(params))
        with self._lock:
            self.msgs.pop(f"ACK_{cmd}", None)
        self.conn.mav.command_long_send(self.conn.target_system, self.conn.target_component, cmd, 0, *p)
        if not wait_ack:
            return True
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            ack = self._get(f"ACK_{cmd}")
            if ack is not None:
                return ack.result == self.mavutil.mavlink.MAV_RESULT_ACCEPTED
            time.sleep(0.05)
        return False

    def set_mode(self, mode: int) -> bool:
        return self._command(
            self.mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            self.mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode,
            wait_ack=True,
            timeout=30.0,
        )

    # ---- VIO -> ArduPilot ----------------------------------------------------------------------
    def send_vio(self, pose: BodyPose) -> None:
        """Forward one OpenVINS estimate as MAVLink ODOMETRY (LOCAL_FRD / BODY_FRD)."""
        p = ENU_TO_NED @ pose.p
        R_ned_frd = ENU_TO_NED @ pose.R @ FLU_TO_FRD
        q = rot_to_quat(R_ned_frd)
        v_body = R_ned_frd.T @ (ENU_TO_NED @ pose.v)  # ODOMETRY velocity is in the child (body) frame
        nan_cov = [float("nan")] + [0.0] * 20
        self.conn.mav.odometry_send(
            int(pose.t * 1e6),
            self.mavutil.mavlink.MAV_FRAME_LOCAL_FRD,
            self.mavutil.mavlink.MAV_FRAME_BODY_FRD,
            *p,
            list(q),
            *v_body,
            float("nan"),
            float("nan"),
            float("nan"),
            nan_cov,
            nan_cov,
            0,
            self.mavutil.mavlink.MAV_ESTIMATOR_TYPE_VIO,
            100,
        )
        if not self._origin_set:
            # GUIDED flight without GPS needs an EKF origin; any fixed point works indoors.
            self.conn.mav.set_gps_global_origin_send(
                self.conn.target_system, int(47.0 * 1e7), int(15.0 * 1e7), 350_000, int(time.time() * 1e6)
            )
            self._origin_set = self._get("GPS_GLOBAL_ORIGIN") is not None

    # ---- Vehicle interface ---------------------------------------------------------------------
    def state(self) -> VehicleState:
        lp, att = self._get("LOCAL_POSITION_NED"), self._get("ATTITUDE")
        sys_s, ext, hb = self._get("SYS_STATUS"), self._get("EXTENDED_SYS_STATE"), self._get("HEARTBEAT")
        vio = self.aligner.get()
        vio_fresh = vio is not None and time.monotonic() - vio.wall < VIO_STALE_S
        if lp is not None:
            x, y, z = lp.y, lp.x, -lp.z - self._z_offset
        elif vio is not None:
            x, y, z = vio.p
        else:
            x = y = z = float("nan")
        yaw = math.pi / 2 - att.yaw if att is not None else float("nan")
        armed = bool(hb and hb.base_mode & self.mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        landed = ext is None or ext.landed_state in (0, 1)  # undefined or on ground
        return VehicleState(
            t=self.sensors.sim_t,
            x=x,
            y=y,
            z=max(0.0, z) if landed else z,
            yaw=(yaw + math.pi) % (2 * math.pi) - math.pi,
            vo_healthy=vio_fresh and lp is not None,
            battery_pct=float(sys_s.battery_remaining) if sys_s is not None and sys_s.battery_remaining >= 0 else 100.0,
            armed=armed,
            landed=landed,
        )

    def roll_pitch(self) -> tuple[float, float]:
        att = self._get("ATTITUDE")
        return (att.roll, -att.pitch) if att is not None else (0.0, 0.0)  # FLU pitch > 0 is nose down

    def wait_ready(self, timeout_s: float = 300.0) -> None:
        """Until EKF3 has a horizontal position from the VIO (needs the EKF origin too)."""
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            ekf = self._get("EKF_STATUS_REPORT")
            lp, vio = self._get("LOCAL_POSITION_NED"), self.aligner.get()
            pos_ok = ekf is not None and ekf.flags & self.mavutil.mavlink.EKF_POS_HORIZ_ABS
            if pos_ok and self._origin_set and lp is not None and vio is not None:
                # Standing on the ground: VIO knows the true height of the body above the floor,
                # EKF3 reports height relative to its start. Remember the difference once.
                self._z_offset = -lp.z - vio.p[2]
                return
            time.sleep(0.5)
        raise TimeoutError("ArduPilot EKF3 has no position from the visual odometry")

    def arm(self, timeout_s: float = 60.0, takeoff_alt: float = 1.0) -> bool:
        """See _arm. timeout_s is simulation time: pre-arm checks (e.g. IMU consistency right after
        boot) can take a while to pass, and the simulator may run slower than real time."""
        return self._arm(timeout_s, takeoff_alt)

    def _arm(self, timeout_s: float, takeoff_alt: float) -> bool:
        """GUIDED, arm, and take off at once: ArduCopter flies GUIDED velocity targets only when
        airborne, and disarms by itself if it sits armed on the ground. The mission's Takeoff node
        then climbs to the mission altitude with velocity targets."""
        end = self.sensors.sim_t + timeout_s
        mav = self.mavutil.mavlink
        while self.sensors.sim_t < end:
            if self.set_mode(GUIDED) and self._command(
                mav.MAV_CMD_COMPONENT_ARM_DISARM, 1, wait_ack=True, timeout=30.0
            ):
                self._took_off = self._command(
                    mav.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, takeoff_alt, wait_ack=True, timeout=30.0
                )
                if not self._took_off:
                    print("take-off refused", flush=True)
                return self._took_off
            time.sleep(1.0)
        print("arming refused", flush=True)
        return False

    def disarm(self) -> None:
        if self.state().armed:
            self._command(self.mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)

    def land(self) -> None:
        if not self._landing:
            self._landing = self.set_mode(LAND)

    def send(self, cmd: Command) -> None:
        if self._landing:
            return  # LAND mode is in charge
        self.conn.mav.set_position_target_local_ned_send(
            0,
            self.conn.target_system,
            self.conn.target_component,
            self.mavutil.mavlink.MAV_FRAME_BODY_NED,
            VEL_YAWRATE_MASK,
            0,
            0,
            0,
            cmd.vx,
            -cmd.vy,
            -cmd.vz,
            0,
            0,
            0,
            0,
            -cmd.yaw_rate,
        )

    def depth_scan(self) -> np.ndarray:
        d = self.sensors.latest_depth()
        if d is None:
            return np.empty((0, 2))
        roll, pitch = self.roll_pitch()
        return depth_to_lane_scan(d, self.cam, altitude=max(self.state().z, 0.0), roll=roll, pitch=pitch)

    def close(self) -> None:
        self._stop = True


def run_ardupilot(
    layout: GreenhouseLayout,
    params: MissionParams,
    url: str = "tcp:127.0.0.1:5760",
    world: str = "greenhouse",
    model: str = "gh_iris",
    prefix: str = "/gh_iris",
    max_sim_time_s: float = 900.0,
    out: Path | None = None,
    tick_s: float = 0.1,
    home_z: float = 0.0,
) -> dict:
    """Run the mission tree against ArduPilot SITL. Ticks every tick_s of simulation time."""
    sensors = GazeboSensors(world, model, prefix)
    aligner = VioAligner(layout.home, home_yaw=0.0, home_z=home_z)
    veh_box: list[ArduPilotVehicle] = []
    vio_node = subscribe_vio(aligner, on_pose=lambda p: veh_box and veh_box[0].send_vio(p))  # noqa: F841
    veh = ArduPilotVehicle(layout, sensors, aligner, url)
    veh_box.append(veh)
    print("waiting for EKF3 to use the visual odometry ...", flush=True)
    veh.wait_ready()
    events, trace = [], []

    def emit(ev: dict) -> None:
        events.append(ev)
        if ev["type"] in ("log", "end"):
            print(f"{ev['t']:7.1f}s  {ev['node']:<24} {ev['msg']}", flush=True)

    ctx = MissionContext(veh, layout, params, dt=tick_s)
    ctx.log_fn = lambda node, msg: emit({"t": round(ctx.s.t, 2), "type": "log", "node": node, "msg": msg})
    root = build_tree(ctx)
    last: dict[str, Status] = {}
    lane_sq: dict[int, list[float]] = {}
    outcome, hit_crop = "timeout", False
    t0 = sensors.sim_t
    next_tick = t0
    try:
        while True:
            while sensors.sim_t < next_tick:  # tick on simulation time, whatever the real-time factor
                time.sleep(0.002)
            next_tick += tick_s
            ctx.pre_tick()
            root.tick_once()
            for node in root.iterate():
                if last.get(node.name) != node.status:
                    last[node.name] = node.status
                    emit({"t": round(ctx.s.t, 2), "type": "status", "node": node.name, "status": node.status.value})
            tip, s, tr = root.tip(), ctx.s, sensors.truth
            row = {
                "t": round(s.t, 2),
                "est_x": s.x,
                "est_y": s.y,
                "z": s.z,
                "battery": s.battery_pct,
                "node": tip.name if tip else "",
            }
            if tr:
                row.update(x=tr[0], y=tr[1], yaw=tr[3])
                hit_crop |= s.z > 0.3 and layout.in_crop(tr[0], tr[1], margin=0.25)
                if (
                    isinstance(tip, SweepLane)
                    and 0 < layout.progress_along_lane(tip.lane_idx, tr[0]) < layout.row_length
                ):
                    lane_sq.setdefault(tip.lane_idx, []).append((tr[1] - layout.lane_center_y(tip.lane_idx)) ** 2)
            trace.append(row)
            if root.status != Status.RUNNING and s.landed:
                outcome = "completed" if root.status == Status.SUCCESS else f"aborted: {ctx.abort_reason}"
                break
            if s.t - t0 > max_sim_time_s:
                break
    except KeyboardInterrupt:
        veh.land()
        outcome = "stopped by user (ArduPilot landing in place)"
    if hit_crop and outcome == "completed":
        outcome = "completed (touched crop)"
    summary = {
        "outcome": outcome,
        "sim_duration_s": round(ctx.s.t - t0, 1),
        "battery_end_pct": round(ctx.s.battery_pct, 1),
        "lanes_completed": ctx.lanes_completed,
        "lane_rms_error_m": {k: round(math.sqrt(sum(v) / len(v)), 3) for k, v in sorted(lane_sq.items())},
        "max_lane_error_m": round(max((math.sqrt(max(v)) for v in lane_sq.values()), default=0.0), 3),
    }
    emit({"t": round(ctx.s.t, 2), "type": "end", "node": "Root", "msg": outcome})
    veh.close()
    if out:
        out.mkdir(parents=True, exist_ok=True)
        (out / "summary.json").write_text(json.dumps(summary, indent=1))
        (out / "events.jsonl").write_text("".join(json.dumps(e) + "\n" for e in events))
        (out / "trace.json").write_text(json.dumps(trace))
        try:
            from types import SimpleNamespace

            from .plot import plot_run

            scored = [r for r in trace if "x" in r]
            plot_run(
                SimpleNamespace(
                    trace=scored,
                    duration_s=summary["sim_duration_s"],
                    **{k: summary[k] for k in ("outcome", "battery_end_pct", "lanes_completed", "max_lane_error_m")},
                ),
                layout,
                str(out / "path.png"),
            )
        except ImportError:
            pass
    return summary
