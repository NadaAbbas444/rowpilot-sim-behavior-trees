"""The pipe-rail trolley in Gazebo: its model, the vehicle the tree drives, and the run.

    VEHICLE=trolley ./scripts/run_ardupilot_sim.sh       (no ArduPilot: plain motor control)

Gazebo renders the greenhouse and the D435i (OpenVINS runs on its images and the IMU, as for the drone),
and moves the trolley; the rails are a rule, not physics:
  - the model has no gravity and no collision shapes; Gazebo's VelocityControl moves it at the body
    velocity it is sent (forward speed and turn rate, as wheels would);
  - TrolleyGazebo (here) is its motor controller and rail supervisor, on Gazebo's true pose: the same
    rules as the Python simulator (trolley.mount, floor_collision, derailed, off_rail_start). On the rails
    it holds the trolley on the lane's line, raised onto the pipes, and only lets it move along them.
    Simulating flanged wheels on round pipes is not worth its jitter.
How it knows where it is, as in the Python simulator: on the rails, the rails across and its wheels along;
on the concrete, wheel odometry from where it left the rails (or the dock). OpenVINS runs as a check (lost
for long on the concrete, the trolley stops); its position is in the trace (vio_x, vio_y). It was 10-20 cm
off after every start and stop of the trolley here, which is why the wheels lead on the concrete.
OpenVINS initialises on a jolt (its static initialisation waits for one); a trolley standing at its dock
gives none, so wait_ready() nudges it forwards and back a few centimetres until OpenVINS tracks.
"""

from __future__ import annotations

import json
import math
import threading
import time
from collections.abc import Callable
from pathlib import Path

import numpy as np
import py_trees
from py_trees.common import Status

from .ardupilot import GazeboSensors, Say, print_say
from .control import Command, clamp
from .layout import GreenhouseLayout
from .mission import MissionContext, MissionParams
from .sensors import D435i, PiCamera3, SensorRig, depth_to_lane_scan, rig_model_sdf
from .trolley import (
    TrolleyConfig,
    WheelOdometry,
    build_trolley_tree,
    derailed,
    floor_collision,
    footprint,
    mount,
    off_rail_start,
    wrap,
)
from .vehicle import VehicleState
from .vio_link import VioAligner, subscribe_vio

MODEL = "gh_trolley"
RAIL_HEIGHT = 0.15  # m: on the rails the chassis rides on the heating pipes, this much higher
VIO_STALE_S = 0.5
SUPERVISOR_HZ = 100.0

# The D435i on a short post at the front, above the deck, tilted down a little; the Pi Camera on the left.
TROLLEY_RIG = SensorRig(
    d435i=D435i(mount_x=0.72, mount_z=0.9, tilt_deg=10.0),
    picam=PiCamera3(pose=(0.0, 0.32, 0.9, 0.0, 0.0, math.pi / 2)),
)


# ---- the model ----------------------------------------------------------------------------------


def _box(name: str, size, xyz, rgb, rough: float = 0.6, metal: float = 0.0) -> str:
    s = " ".join(f"{v:g}" for v in size)
    p = " ".join(f"{v:g}" for v in xyz)
    r, g, b = rgb
    return (
        f'      <visual name="{name}"><pose>{p} 0 0 0</pose><geometry><box><size>{s}</size></box></geometry>'
        f"<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>"
        f"<pbr><metal><roughness>{rough}</roughness><metalness>{metal}</metalness></metal></pbr></material></visual>\n"
    )


def _wheel(name: str, xyz, radius: float, width: float, rgb) -> str:
    p = " ".join(f"{v:g}" for v in xyz)
    r, g, b = rgb
    return (
        f'      <visual name="{name}"><pose>{p} {math.pi / 2:.5f} 0 0</pose>'
        f"<geometry><cylinder><radius>{radius}</radius><length>{width}</length></cylinder></geometry>"
        f"<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material></visual>\n"
    )


def trolley_sdf(cfg: TrolleyConfig | None = None, model: str = "trolley") -> str:
    """A pipe-rail trolley (as in Dutch-style greenhouses): blue frame, checker-plate deck, battery box,
    flanged rail wheels on the pipes' 0.55 m gauge and small wheels for the concrete. Origin on the floor
    under the chassis' centre, x forward. No gravity, no collisions: VelocityControl moves it and the rail
    supervisor (TrolleyGazebo) keeps the rules."""
    cfg = cfg or TrolleyConfig()
    L, W = cfg.length, cfg.width
    blue, plate, dark = (0.18, 0.42, 0.58), (0.74, 0.75, 0.77), (0.08, 0.08, 0.09)
    v = _box("frame", (L, W, 0.06), (0, 0, 0.13), blue, 0.5)
    v += _box("deck", (L - 0.04, W - 0.04, 0.02), (0, 0, 0.17), plate, 0.35, 0.8)
    v += _box("battery_box", (0.55, W - 0.08, 0.34), (-0.36, 0, 0.35), blue, 0.5)
    v += _box("battery_lid", (0.57, W - 0.06, 0.02), (-0.36, 0, 0.53), plate, 0.35, 0.8)
    v += _box("camera_post", (0.04, 0.04, 0.72), (L / 2 - 0.05, 0, 0.53), blue, 0.5)
    for i, x in enumerate((-0.55, 0.55)):
        for j, y in enumerate((-0.275, 0.275)):
            v += _wheel(f"rail_wheel_{i}{j}", (x, y, 0.065), 0.06, 0.04, dark)
            v += _wheel(f"rail_flange_{i}{j}", (x, y - 0.025 * (1 if y > 0 else -1), 0.065), 0.075, 0.008, dark)
    for i, x in enumerate((-0.68, 0.68)):
        v += _wheel(f"floor_wheel_{i}", (x, 0, 0.05), 0.05, 0.05, (0.2, 0.2, 0.2))
    m = 80.0  # kg
    ixx, iyy, izz = m * (W**2 + 0.3**2) / 12, m * (L**2 + 0.3**2) / 12, m * (L**2 + W**2) / 12
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<!-- Generated by rowpilot.trolley_gazebo: edit trolley_gazebo.py, not this file. -->
<sdf version="1.9">
  <model name="{model}">
    <link name="base_link">
      <gravity>false</gravity>
      <inertial><pose>0 0 0.25 0 0 0</pose><mass>{m}</mass>
        <inertia><ixx>{ixx:.3f}</ixx><iyy>{iyy:.3f}</iyy><izz>{izz:.3f}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
{v}    </link>
    <plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl"/>
  </model>
</sdf>
"""


def gh_trolley_sdf(model: str = MODEL) -> str:
    """The trolley with the D435i and the Pi Camera; the sensor link, too, without gravity."""
    sdf = rig_model_sdf(model, "model://trolley", "base_link", TROLLEY_RIG, "Pipe-rail trolley, plain motor control.")
    return sdf.replace('<link name="sensor_link">', '<link name="sensor_link">\n      <gravity>false</gravity>', 1)


# ---- the vehicle the tree drives --------------------------------------------------------------


class TrolleyGazebo:
    """The Vehicle interface for the trolley in Gazebo: motor control and the rail supervisor (a thread
    at SUPERVISOR_HZ on Gazebo's true pose and clock), and the trolley's estimate of where it is."""

    robot = "trolley"

    def __init__(
        self,
        layout: GreenhouseLayout,
        sensors: GazeboSensors,
        aligner: VioAligner,
        model: str = MODEL,
        cfg: TrolleyConfig | None = None,
        start_battery_pct: float = 100.0,
        seed: int = 0,
        say: Say = print_say,
    ):
        from gz.msgs10.twist_pb2 import Twist
        from gz.transport13 import Node

        self.layout, self.sensors, self.aligner, self.say = layout, sensors, aligner, say
        self.tc = cfg or TrolleyConfig()
        self.cam = TROLLEY_RIG.d435i.depth()
        self._Twist = Twist
        self.node = Node()
        self.pub = self.node.advertise(f"/model/{model}/cmd_vel", Twist)
        self.rng = np.random.default_rng(seed)
        self.cmd = Command()
        self.last_cmd = Command()
        self.v = self.w = 0.0  # motor speeds after the lag
        self.armed = False
        self.rail: int | None = None
        self.crashed, self.crash_reason = False, ""
        self.battery = start_battery_pct
        self.odo_err = 0.0
        self.wheels = WheelOdometry(self.tc, np.random.default_rng(7000 + seed), *layout.home, 0.0)
        self._lock = threading.Lock()
        self._stop = False
        self._sup = threading.Thread(target=self._supervise, daemon=True)
        self._sup.start()

    # ---- the supervisor: motors, rails, collisions -------------------------------------------
    def _supervise(self) -> None:
        tc, lay = self.tc, self.layout
        last_t, lead_prev = None, None
        while not self._stop:
            time.sleep(1.0 / SUPERVISOR_HZ)
            tr, t = self.sensors.truth, self.sensors.sim_t
            if tr is None:
                continue
            dt = 0.0 if last_t is None else max(0.0, t - last_t)
            last_t = t
            if dt <= 0:
                continue
            x, y, z, yaw = tr
            with self._lock:
                self.battery = max(0.0, self.battery - tc.idle_drain_pct_s * dt)
                if not self.armed or self.crashed or self.battery <= 0:
                    self.v = self.w = 0.0
                else:
                    k = min(1.0, dt / tc.motor_lag_s)
                    self.v += (clamp(self.cmd.vx, -tc.max_speed, tc.max_speed) - self.v) * k
                    w = 0.0 if self.rail is not None else clamp(self.cmd.yaw_rate, -tc.max_turn_rate, tc.max_turn_rate)
                    self.w += (w - self.w) * k
                lead = footprint(x, y, yaw, tc)[:, 0].max()
                # what happens over the next dt, judged now: Gazebo moves the trolley, we keep the rules
                if self.rail is None:
                    self.wheels.update(self.v, self.w, dt)
                if self.rail is not None:
                    self.odo_err += self.rng.normal(0, tc.odometry_noise_per_sqrt_m * math.sqrt(abs(self.v) * dt))
                    if derailed(lay, tc, x):
                        self._crash("ran off the end of the rails")
                    elif off_rail_start(tc, x) and self.v <= 0:
                        self._leave_rails(x, y)
                elif not self.crashed:
                    # Reaches the rail start: about to (lead + v dt), or did since the last pose (Gazebo's poses
                    # arrive in steps, a crossing can fall between two of them).
                    crossed = lead_prev is not None and lead_prev < 0.0 <= lead
                    if (crossed or lead < 0.0 <= lead + self.v * dt) and self.v > 0:
                        lane, why = mount(lay, tc, x, y, yaw)
                        if why:
                            self._crash(why)
                        elif lane is not None:
                            self.rail, self.odo_err = lane, 0.0
                            self.say("trolley", f"on the rails of lane {lane + 1}")
                    if not self.crashed and self.rail is None:
                        why = floor_collision(lay, tc, x, y, yaw)
                        if why:
                            self._crash(why)
                self.battery = max(0.0, self.battery - tc.move_drain_pct_per_m * abs(self.v) * dt)
                lead_prev = lead
                self._publish(x, y, z, yaw)

    def _publish(self, x: float, y: float, z: float, yaw: float) -> None:
        """Body-frame velocity for Gazebo: the motors, plus what the rails do (hold the line, the height)."""
        m = self._Twist()
        if self.rail is not None:
            # along the rails (world x) at the motors' speed, pulled onto the lane's line (world y)
            vx_w, vy_w = self.v, -4.0 * (y - self.layout.lane_center_y(self.rail))
            m.linear.x = vx_w * math.cos(yaw) + vy_w * math.sin(yaw)
            m.linear.y = -vx_w * math.sin(yaw) + vy_w * math.cos(yaw)
            m.linear.z = 4.0 * (RAIL_HEIGHT - z)
            m.angular.z = -4.0 * wrap(yaw)
        else:
            m.linear.x = self.v
            m.linear.z = 4.0 * (0.0 - z)
            m.angular.z = self.w
        self.pub.publish(m)

    def _crash(self, reason: str) -> None:
        self.crashed, self.crash_reason = True, reason
        self.v = self.w = 0.0
        self.say("trolley", f"crash: {reason}")

    def _leave_rails(self, x: float, y: float) -> None:
        """Off the rail start, back on the concrete: the wheels take over from where the rails left it."""
        lane = self.rail
        self.rail = None
        self.wheels.reset(x + self.odo_err, self.layout.lane_center_y(lane), 0.0)
        self.say("trolley", f"off the rails of lane {lane + 1}")

    # ---- estimate ------------------------------------------------------------------------------
    def _vio(self):
        p = self.aligner.get()
        if p is None or self.sensors.sim_t - p.t > VIO_STALE_S:
            return None
        return p

    def vo_healthy(self) -> bool:
        return self._vio() is not None

    def state(self) -> VehicleState:
        tr = self.sensors.truth or (*self.layout.home, 0.0, 0.0)
        with self._lock:
            rail, armed, battery = self.rail, self.armed, self.battery
            landed = abs(self.v) < 0.01 and abs(self.w) < 0.01
            wheels = (self.wheels.x, self.wheels.y, self.wheels.yaw)
        if rail is not None:  # the rails across, the wheels along
            ex, ey, eyaw = tr[0] + self.odo_err, self.layout.lane_center_y(rail), 0.0
        else:  # the concrete: wheel odometry
            ex, ey, eyaw = wheels
        healthy = self._vio() is not None
        return VehicleState(self.sensors.sim_t, ex, ey, 0.0, eyaw, healthy, battery, armed, landed, rail)

    # ---- Vehicle interface --------------------------------------------------------------------
    @property
    def landed(self) -> bool:
        return abs(self.v) < 0.01 and abs(self.w) < 0.01

    @property
    def at_rest(self) -> bool:
        return not self.armed and self.landed

    def arm(self) -> bool:
        with self._lock:
            if self.battery > 5 and not self.crashed:
                self.armed = True
            return self.armed

    def disarm(self) -> None:
        with self._lock:
            if self.landed:
                self.armed = False
                self.cmd = Command()

    def send(self, cmd: Command) -> None:
        with self._lock:
            self.cmd = cmd
            self.last_cmd = cmd

    def land(self) -> None:
        self.send(Command())

    def depth_scan(self) -> np.ndarray:
        d = self.sensors.latest_depth()
        if d is None:
            return np.empty((0, 2))
        z = self.sensors.truth[2] if self.sensors.truth else 0.0
        return depth_to_lane_scan(d, self.cam, altitude=max(z, 0.0))

    def wait_ready(self, timeout_s: float = 240.0) -> None:
        """Wait for Gazebo, then nudge the trolley forwards and back until OpenVINS tracks."""
        t0 = time.monotonic()
        while self.sensors.truth is None or self.sensors.sim_t <= 0:
            if time.monotonic() - t0 > timeout_s:
                raise TimeoutError("no pose from Gazebo")
            time.sleep(0.1)
        self.say("trolley", "waiting for OpenVINS to initialise (it needs a jolt: nudging the trolley)")
        nudges = 0
        while not self.vo_healthy():
            if time.monotonic() - t0 > timeout_s:
                raise TimeoutError("OpenVINS did not initialise")
            self._nudge()
            nudges += 1
            t_wait = self.sensors.sim_t
            while self.sensors.sim_t - t_wait < 4.0 and not self.vo_healthy():
                time.sleep(0.05)
        self.say("vio", f"OpenVINS tracking (after {nudges} nudge{'s' if nudges != 1 else ''})")

    def _nudge(self) -> None:
        self.armed = True
        for vx in (0.25, -0.25):
            self.send(Command(vx=vx))
            t1 = self.sensors.sim_t
            while self.sensors.sim_t - t1 < 0.4:
                time.sleep(0.01)
        self.send(Command())
        t1 = self.sensors.sim_t
        while self.sensors.sim_t - t1 < 0.5:
            time.sleep(0.01)
        self.armed = False

    def close(self) -> None:
        self._stop = True
        self.send(Command())


# ---- the run ------------------------------------------------------------------------------------


def run_trolley_gazebo(
    layout: GreenhouseLayout,
    params: MissionParams,
    world: str = "greenhouse",
    model: str = MODEL,
    max_sim_time_s: float = 1200.0,
    out: Path | None = None,
    tick_s: float = 0.1,
    *,
    tree_factory: Callable[[MissionContext], py_trees.behaviour.Behaviour] = build_trolley_tree,
    on_event: Callable[[dict], None] | None = None,
    on_tick: Callable[[dict], bool | None] | None = None,
    say: Say = print_say,
) -> dict:
    """Run the trolley's tree in Gazebo, ticking every tick_s of simulation time (as run_ardupilot)."""
    sensors = GazeboSensors(world, model, f"/{model}")
    aligner = VioAligner(layout.home, home_yaw=0.0, home_z=0.0, d435i=TROLLEY_RIG.d435i)
    vio_node = subscribe_vio(aligner)  # noqa: F841 - keep the subscription alive
    user_say = say

    def say(source: str, msg: str) -> None:  # noqa: F811 - stamp with sim time
        user_say(source, msg, t=round(sensors.sim_t, 2))

    veh = TrolleyGazebo(layout, sensors, aligner, model, say=say)
    veh.wait_ready()
    events, trace = [], []
    ids: dict[str, str] = {}

    def emit(ev: dict) -> None:
        if ev["node"] in ids:
            ev["id"] = ids[ev["node"]]
        events.append(ev)
        if on_event:
            on_event(ev)
        elif ev["type"] in ("log", "end"):
            print(f"{ev['t']:7.1f}s  {ev['node']:<26} {ev['msg']}", flush=True)

    ctx = MissionContext(veh, layout, params, dt=tick_s)
    ctx.log_fn = lambda node, msg: emit({"t": round(ctx.s.t, 2), "type": "log", "node": node, "msg": msg})
    root = tree_factory(ctx)
    ids.update({n.name: n.graph_id for n in root.iterate() if getattr(n, "graph_id", None) is not None})
    last: dict[str, Status] = {}
    vio_was_ok = True
    outcome = "timeout"
    t0 = sensors.sim_t
    next_tick = t0
    try:
        while True:
            while sensors.sim_t < next_tick:
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
                "z": tr[2] if tr else 0.0,
                "battery": s.battery_pct,
                "node": tip.name if tip else "",
                "on_rail": s.on_rail,
                **({"vio_x": vio.p[0], "vio_y": vio.p[1]} if (vio := veh._vio()) is not None else {}),
                "cmd_vx": veh.last_cmd.vx,
                "cmd_yaw_rate": veh.last_cmd.yaw_rate,
            }
            if tr:
                row.update(x=tr[0], y=tr[1], yaw=tr[3])
            trace.append(row)
            if s.vo_healthy != vio_was_ok:
                vio_was_ok = s.vo_healthy
                if s.on_rail is None:
                    say("vio", "OpenVINS tracking again" if s.vo_healthy else "OpenVINS estimate stale or lost")
            if on_tick:
                tick = {
                    "t": row["t"],
                    "x": row.get("x", s.x),
                    "y": row.get("y", s.y),
                    "z": row["z"],
                    "yaw": row.get("yaw", s.yaw),
                    "est_x": s.x,
                    "est_y": s.y,
                    "battery": s.battery_pct,
                    "vo_ok": s.vo_healthy,
                    "armed": s.armed,
                    "node": row["node"],
                }
                if on_tick(tick) is False:
                    raise KeyboardInterrupt
            if veh.crashed:
                outcome = f"crashed: {veh.crash_reason}"
                break
            if root.status != Status.RUNNING and veh.at_rest:
                outcome = "completed" if root.status == Status.SUCCESS else f"aborted: {ctx.abort_reason}"
                break
            if s.t - t0 > max_sim_time_s:
                break
    except KeyboardInterrupt:
        outcome = "stopped by user (trolley stopped in place)"
    veh.close()
    summary = {
        "outcome": outcome,
        "sim_duration_s": round(ctx.s.t - t0, 1),
        "battery_end_pct": round(ctx.s.battery_pct, 1),
        "lanes_completed": ctx.lanes_completed,
        "lane_rms_error_m": {},
        "max_lane_error_m": 0.0,
    }
    emit({"t": round(ctx.s.t, 2), "type": "end", "node": root.name, "msg": outcome})
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
                robot="trolley",
            )
        except ImportError:
            pass
    return summary
