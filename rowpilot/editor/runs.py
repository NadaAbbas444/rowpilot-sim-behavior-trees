"""Mission runs started from the editor, in one of three simulators:

- "python": rowpilot's own simulator (sim.py), in a thread. Fast (any speed), no Gazebo, ArduPilot
  or OpenVINS: the tree, perception and control against a modelled drone.
- "gazebo": the full chain, as scripts/run_ardupilot_sim.sh runs it: Gazebo (with its window, the
  drone and live camera screens), ArduCopter SITL, OpenVINS on the simulated D435i, and the tree
  flying ArduPilot over MAVLink. Real time. The script's `rowpilot ardupilot --stream` prints the run
  as JSON lines, which are read here; OpenVINS's own log is followed for its initialisation.
- "isaac": the same full chain with NVIDIA Isaac Sim in place of Gazebo (the run script with SIM=isaac;
  FireFly only). It runs as a Gazebo run does: same script, messages and results.

A run records everything it produces as a list of messages:

    {"kind": "event", "t": ..., "type": "status" | "log" | "end", "node": name, "id": graph id, ...}
    {"kind": "event", "t": ..., "type": "system", "source": "ardupilot" | "mavlink" | "ekf" | "vio" | "sim"
                      | "setup" | "editor", "node": source, "msg": ...}      what the flight stack says
    {"kind": "tick", "t", "x", "y", "z", "yaw", "est_x", "est_y", "battery", "vo_ok", "node"}
    {"kind": "end", "status": "finished" | "stopped" | "error", "summary": {...}, "error": str | None}

A viewer (the WebSocket in server.py) reads the list from the start and then waits for more, so
it can connect late, reconnect, or several viewers can watch one run.
"""

from __future__ import annotations

import asyncio
import json
import os
import re
import shlex
import signal
import subprocess
import threading
import time
import traceback
import uuid
from dataclasses import dataclass, field
from pathlib import Path

from ..layout import GreenhouseLayout
from ..mission import MissionParams
from ..runner import RunConfig, run_mission
from ..sim import SimConfig
from .nodes import build_from_graph

REPO = Path(__file__).resolve().parents[2]


@dataclass
class Settings:
    """What the editor's Run panel sets: the simulator, the greenhouse and the mission."""

    simulator: str = "python"  # "python", "gazebo" or "isaac"
    robot: str = "drone"  # "drone" (the FireFly 2) or "trolley" (the pipe-rail trolley: no ArduPilot; not in Isaac Sim)
    lanes: int = 6
    open_ends: bool = False
    row_length: float = GreenhouseLayout.row_length
    path_width: float = 1.6
    # python simulator only
    seed: int = 0
    battery: float = 100.0
    wind: float = 0.3
    vo_dropouts: list[tuple[float, float]] = field(default_factory=list)
    speed: float = 20.0
    max_time_s: float = 1800.0
    # both
    altitude: float = 1.5
    sweep_speed: float = 0.8
    # gazebo and isaac only
    vehicle: str = "firefly"  # isaac: firefly only
    gazebo_window: bool = True  # the simulator's window (Gazebo's or Isaac Sim's)

    def layout(self) -> GreenhouseLayout:
        return GreenhouseLayout(
            n_lanes=self.lanes, row_length=self.row_length, path_width=self.path_width, closed_end=not self.open_ends
        )

    def run_config(self) -> RunConfig:
        return RunConfig(
            layout=self.layout(),
            sim=SimConfig(
                seed=self.seed,
                start_battery_pct=self.battery,
                wind_sigma=self.wind,
                vo_dropouts=[tuple(d) for d in self.vo_dropouts],
            ),
            mission=MissionParams(altitude=self.altitude, sweep_speed=self.sweep_speed),
            max_time_s=self.max_time_s,
            robot=self.robot,
        )

    def greenhouse_args(self) -> list[str]:
        """The greenhouse as `rowpilot world` / `rowpilot ardupilot` options."""
        args = ["--lanes", str(self.lanes), "--length", f"{self.row_length:g}", "--path-width", f"{self.path_width:g}"]
        return args + (["--open-ends"] if self.open_ends else [])


class Run:
    """A run in the Python simulator (and the message log every run kind shares)."""

    def __init__(self, graph: dict, settings: Settings):
        self.id = uuid.uuid4().hex[:12]
        self.graph, self.settings = graph, settings
        self.status = "running"
        self.summary: dict | None = None
        self.error: str | None = None
        self.created = time.time()
        self.messages: list[dict] = []
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._waiters: set[tuple[asyncio.AbstractEventLoop, asyncio.Event]] = set()
        self.thread = threading.Thread(target=self._run, name=f"run-{self.id}", daemon=True)

    # ---- producer (the run's thread) ---------------------------------------------------------
    def _push(self, msg: dict) -> None:
        with self._lock:
            self.messages.append(msg)
            waiters = list(self._waiters)
        for loop, ev in waiters:
            try:
                loop.call_soon_threadsafe(ev.set)
            except RuntimeError:  # that viewer's event loop has closed
                pass

    def _system(self, source: str, msg: str, t: float = 0.0) -> None:
        self._push({"kind": "event", "t": t, "type": "system", "source": source, "node": source, "msg": msg})

    def _run(self) -> None:
        s = self.settings
        wall0 = time.monotonic()
        if s.robot == "trolley":
            self._system(
                "sim",
                "Python simulator: the tree drives a modelled pipe-rail trolley (plain motor control: on the "
                "rails in the lanes, on the concrete in the headland)",
            )
        else:
            self._system(
                "sim",
                "Python simulator: the tree, lane perception and control against a modelled drone "
                "(no Gazebo, ArduPilot or OpenVINS; choose Gazebo in Run settings for those)",
            )

        def on_tick(row: dict) -> bool:
            self._push({"kind": "tick", **row})
            if s.speed > 0:
                ahead = row["t"] / s.speed - (time.monotonic() - wall0)
                if ahead > 0:
                    self._stop.wait(ahead)
            return not self._stop.is_set()

        try:
            res = run_mission(
                s.run_config(),
                on_event=lambda ev: self._push({"kind": "event", **ev}),
                tree_factory=lambda ctx: build_from_graph(self.graph, ctx),
                on_tick=on_tick,
            )
            self.summary = res.summary()
            self.status = "stopped" if res.outcome == "stopped" else "finished"
        except Exception as e:  # a node raised: report it to the editor instead of losing the thread
            self.status, self.error = "error", f"{type(e).__name__}: {e}"
            traceback.print_exc()
        self._finish()

    def _finish(self) -> None:
        self._push({"kind": "end", "status": self.status, "summary": self.summary, "error": self.error})

    # ---- control ---------------------------------------------------------------------------------
    def start(self) -> Run:
        self.thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()

    def kill(self) -> None:
        self.stop()

    @property
    def done(self) -> bool:
        return self.status != "running"

    def info(self) -> dict:
        return {
            "id": self.id,
            "simulator": self.settings.simulator,
            "status": self.status,
            "created": self.created,
            "summary": self.summary,
            "error": self.error,
            "messages": len(self.messages),
        }

    # ---- consumers ---------------------------------------------------------------------------------
    async def follow(self, start: int = 0):
        """Yield batches of messages from index `start` on, as they come, until the run has ended."""
        loop, ev = asyncio.get_running_loop(), asyncio.Event()
        key = (loop, ev)
        with self._lock:
            self._waiters.add(key)
        try:
            n = start
            while True:
                ev.clear()
                with self._lock:
                    batch = self.messages[n:]
                if batch:
                    n += len(batch)
                    yield batch
                    if batch[-1]["kind"] == "end":
                        return
                    continue
                await ev.wait()
        finally:
            with self._lock:
                self._waiters.discard(key)


# ---- the full chain: Gazebo + ArduPilot SITL + OpenVINS -----------------------------------------------
@dataclass
class GazeboSetup:
    """Where the full simulation lives (scripts/setup_ardupilot_sim.sh puts it there)."""

    ardupilot: Path = Path.home() / "ardupilot"
    ardupilot_gazebo: Path = Path.home() / "ardupilot_gazebo"
    script: Path = REPO / "scripts" / "run_ardupilot_sim.sh"
    runs_dir: Path = REPO / "runs"
    check_tools: bool = True  # tests run a stand-in script and skip the checks
    isaac_python: Path = Path(os.environ.get("ISAAC_PYTHON", REPO / ".venv-isaac/bin/python"))

    def missing(self, simulator: str = "gazebo", robot: str = "drone") -> list[str]:
        """What is not there yet, in words; empty when the full simulation can run in that simulator for
        that robot. Isaac Sim needs ArduPilot, OpenVINS and .venv-sim, not Gazebo or its plugin; the trolley
        (Gazebo only) needs Gazebo and OpenVINS, not ArduPilot (plain motor control)."""
        if not self.check_tools:
            return [] if self.script.is_file() else [f"{self.script} is missing"]
        need = {"the run script": self.script}
        if robot != "trolley":
            need["ArduCopter SITL (scripts/setup_ardupilot_sim.sh)"] = self.ardupilot / "build/sitl/bin/arducopter"
        need |= {
            "the simulation's Python environment .venv-sim (scripts/setup_ardupilot_sim.sh)": REPO
            / ".venv-sim/bin/python",
            "the OpenVINS bridge build/vio/vio_bridge (scripts/setup_ardupilot_sim.sh)": REPO / "build/vio/vio_bridge",
        }
        if simulator == "isaac":
            need["Isaac Sim (scripts/setup_isaac_sim.sh)"] = self.isaac_python
        elif robot != "trolley":
            need["the ardupilot_gazebo plugin (scripts/setup_ardupilot_sim.sh)"] = self.ardupilot_gazebo / "build"
        out = [what for what, path in need.items() if not path.exists()]
        has_gz = any((Path(d) / "gz").is_file() for d in os.environ.get("PATH", "").split(os.pathsep))
        if simulator != "isaac" and not has_gz:
            out.append("Gazebo Harmonic (the gz command)")
        return out


ANSI = re.compile(r"\x1b\[[0-9;]*m")


def openvins_line(line: str) -> str | None:
    """The OpenVINS log lines worth showing (it prints its state at 20 Hz); None for the rest."""
    line = ANSI.sub("", line).strip()
    if line.startswith("[vio_bridge] listening"):
        return "VIO bridge running: D435i infrared pair + IMU -> OpenVINS"
    if "failed static init" in line:
        return "OpenVINS waiting to initialise: " + line.split("failed static init:", 1)[1].strip()
    if line.startswith("[init]: successful initialization"):
        return "OpenVINS initialised"
    if line.startswith(("[init]: orientation", "[init]: position", "[init]: velocity", "[init]: bias")):
        return "OpenVINS " + line.removeprefix("[init]: ")
    return None


class GazeboRun(Run):
    """A run of the full simulation: starts the run script and turns its output into messages."""

    STOP_GRACE_S = 60.0  # landing and closing everything after "stop"

    def __init__(self, graph: dict, settings: Settings, setup: GazeboSetup):
        super().__init__(graph, settings)
        self.setup = setup
        self.out = setup.runs_dir / f"editor-{time.strftime('%Y%m%d-%H%M%S')}-{self.id[:6]}"
        self.proc: subprocess.Popen | None = None
        self.t = 0.0  # latest simulation time, for the flight stack's messages
        self.mission_started = False  # the mission process is up (it reads 'stop' and lands)

    def _system(self, source: str, msg: str, t: float | None = None) -> None:
        super()._system(source, msg, self.t if t is None else t)

    def _run(self) -> None:
        s, setup = self.settings, self.setup
        try:
            self.out.mkdir(parents=True, exist_ok=True)
            tree = self.out / "tree.json"
            tree.write_text(json.dumps(self.graph, indent=1))
            env = {
                **os.environ,
                "PYTHONNOUSERSITE": "1",
                "TREE": str(tree),
                "STREAM": "1",
                "OUT_DIR": str(self.out),
                "VEHICLE": "trolley" if s.robot == "trolley" else s.vehicle,
                "SIM": "isaac" if s.simulator == "isaac" else "gazebo",
                "HEADLESS": "0" if s.gazebo_window else "1",
                "GREENHOUSE_ARGS": shlex.join(s.greenhouse_args()),
            }
            cmd = [str(setup.script), str(setup.ardupilot), str(setup.ardupilot_gazebo)]
            sim = "Isaac Sim" if s.simulator == "isaac" else "Gazebo"
            what = "the pipe-rail trolley, no ArduPilot" if s.robot == "trolley" else s.vehicle
            self._system("setup", f"starting the full simulation in {sim} ({what}); results in {self.out}")
            self.proc = subprocess.Popen(
                cmd,
                env=env,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,  # its own process group, so it can all be ended together
            )
            threading.Thread(target=self._follow_openvins, daemon=True).start()
            for line in self.proc.stdout:
                self._line(line.rstrip("\n"))
            code = self.proc.wait()
            if self.summary is not None:
                self.status = "stopped" if self._stop.is_set() else "finished"
            elif self._stop.is_set():
                self.status = "stopped"
            else:
                self.status, self.error = "error", f"the simulation ended (exit code {code}) before the mission did"
        except Exception as e:
            self.status, self.error = "error", f"{type(e).__name__}: {e}"
            traceback.print_exc()
        self._stop.set()  # ends the OpenVINS log follower
        self._finish()

    def _line(self, line: str) -> None:
        if line.startswith("@rowpilot "):
            self.mission_started = True
            try:
                msg = json.loads(line[len("@rowpilot ") :])
            except json.JSONDecodeError:
                return
            kind = msg.pop("kind", None)
            if kind == "summary":
                msg["duration_s"] = msg.pop("sim_duration_s", 0.0)
                self.summary = msg
            elif kind == "tick":
                self.t = msg["t"]
                self._push({"kind": "tick", **msg})
            elif kind == "event":
                if msg.get("t") is None:
                    msg["t"] = self.t
                self._push({"kind": "event", **msg})
            return
        if line.strip():  # the run script's own progress, and any error output
            self._system("setup", line.rstrip())

    def _follow_openvins(self) -> None:
        """OpenVINS writes to its own log file: show its initialisation as it happens."""
        path, pos, said = self.out / "openvins.log", 0, set()
        while not self._stop.is_set():
            if path.is_file():
                with open(path, errors="replace") as f:
                    f.seek(pos)
                    chunk = f.read()
                    pos = f.tell()
                for raw in chunk.splitlines():
                    msg = openvins_line(raw)
                    if msg and msg not in said:
                        said.add(msg)
                        self._system("vio", msg)
            self._stop.wait(0.5)

    def stop(self) -> None:
        """Land and end: 'stop' to the mission process, then (if it hangs) end the whole simulation.
        Still starting up (nothing flying yet): end the simulation at once."""
        if self._stop.is_set():
            return
        self._stop.set()
        p = self.proc
        if p is None or p.poll() is not None:
            return
        if not self.mission_started:
            self._system("editor", "stopped while starting: closing the simulation")
            threading.Thread(target=self.kill, daemon=True).start()
            return
        self._system("editor", "stop: landing, then closing the simulation")
        try:
            p.stdin.write("stop\n")
            p.stdin.flush()
        except (BrokenPipeError, OSError):
            pass
        threading.Thread(target=self._kill_later, daemon=True).start()

    def _kill_later(self) -> None:
        p = self.proc
        try:
            p.wait(self.STOP_GRACE_S)
        except subprocess.TimeoutExpired:
            self.kill()

    def kill(self) -> None:
        """End the run script's process group (its exit trap closes Gazebo, ArduPilot and OpenVINS)."""
        self._stop.set()
        p = self.proc
        if p is None or p.poll() is not None:
            return
        for sig, wait in ((signal.SIGTERM, 15.0), (signal.SIGKILL, 5.0)):
            try:
                os.killpg(p.pid, sig)
                p.wait(wait)
                return
            except subprocess.TimeoutExpired:
                continue
            except ProcessLookupError:
                return


class RunManager:
    """Keeps the latest runs; at most `max_active` Python runs and one full-chain run (Gazebo or Isaac
    Sim: they share ArduPilot's ports) at once."""

    def __init__(self, keep: int = 20, max_active: int = 4, gazebo: GazeboSetup | None = None):
        self.keep, self.max_active = keep, max_active
        self.gazebo = gazebo or GazeboSetup()
        self.runs: dict[str, Run] = {}

    def simulators(self) -> dict:
        out = {"python": {"available": True, "missing": []}}
        for sim in ("gazebo", "isaac"):
            missing = self.gazebo.missing(sim)
            out[sim] = {"available": not missing, "missing": missing}
        trolley = self.gazebo.missing("gazebo", "trolley")
        out["gazebo_trolley"] = {"available": not trolley, "missing": trolley}
        return out

    def start(self, graph: dict, settings: Settings) -> Run:
        active = [r for r in self.runs.values() if not r.done]
        if settings.robot not in ("drone", "trolley"):
            raise ValueError(f"unknown robot {settings.robot!r}")
        if settings.simulator in ("gazebo", "isaac"):
            name = "Isaac Sim" if settings.simulator == "isaac" else "Gazebo"
            if settings.simulator == "isaac" and (settings.robot != "drone" or settings.vehicle != "firefly"):
                raise ValueError("Isaac Sim flies the FireFly only")
            missing = self.gazebo.missing(settings.simulator, settings.robot)
            if missing:
                raise ValueError(f"the {name} simulation is not set up: missing " + "; ".join(missing))
            if any(isinstance(r, GazeboRun) for r in active):
                raise RuntimeError("a Gazebo or Isaac Sim run is already going (one at a time); stop it first")
            run: Run = GazeboRun(graph, settings, self.gazebo)
        else:
            if len(active) >= self.max_active:
                raise RuntimeError(f"{len(active)} runs are already going; stop one first")
            run = Run(graph, settings)
        self.runs[run.id] = run
        for old in sorted(self.runs.values(), key=lambda r: r.created)[: max(0, len(self.runs) - self.keep)]:
            if old.done:
                del self.runs[old.id]
        return run.start()

    def get(self, run_id: str) -> Run | None:
        return self.runs.get(run_id)

    def stop_all(self) -> None:
        for r in self.runs.values():
            r.stop()
        for r in self.runs.values():
            r.thread.join(timeout=5)
            if r.thread.is_alive():
                r.kill()
