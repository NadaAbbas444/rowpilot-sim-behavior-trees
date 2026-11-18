"""Mission runs started from the editor: each one a simulator run in its own thread.

A run records everything it produces as a list of messages:

    {"kind": "event", "t": ..., "type": "status" | "log" | "end", "node": name, "id": graph id, ...}
    {"kind": "tick", "t", "x", "y", "z", "yaw", "est_x", "est_y", "battery", "vo_ok", "node"}
    {"kind": "end", "status": "finished" | "stopped" | "error", "summary": {...}, "error": str | None}

A viewer (the WebSocket in server.py) reads the list from the start and then waits for more, so
it can connect late, reconnect, or several viewers can watch one run. `speed` paces the run:
simulated seconds per real second (0 runs as fast as possible).
"""

from __future__ import annotations

import asyncio
import threading
import time
import traceback
import uuid
from dataclasses import dataclass, field

from ..layout import GreenhouseLayout
from ..mission import MissionParams
from ..runner import RunConfig, run_mission
from ..sim import SimConfig
from .nodes import build_from_graph


@dataclass
class Settings:
    """What the editor's Run panel sets: the greenhouse, the simulation and the mission."""

    lanes: int = 6
    open_ends: bool = False
    row_length: float = GreenhouseLayout.row_length
    path_width: float = 1.6
    seed: int = 0
    battery: float = 100.0
    wind: float = 0.3
    vo_dropouts: list[tuple[float, float]] = field(default_factory=list)
    altitude: float = 1.5
    sweep_speed: float = 0.8
    speed: float = 20.0
    max_time_s: float = 1800.0

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
        )


class Run:
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

    def _run(self) -> None:
        s = self.settings
        wall0 = time.monotonic()

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
        self._push({"kind": "end", "status": self.status, "summary": self.summary, "error": self.error})

    # ---- control ---------------------------------------------------------------------------------
    def start(self) -> Run:
        self.thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()

    @property
    def done(self) -> bool:
        return self.status != "running"

    def info(self) -> dict:
        return {
            "id": self.id,
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


class RunManager:
    """Keeps the latest runs; at most `max_active` run at once."""

    def __init__(self, keep: int = 20, max_active: int = 4):
        self.keep, self.max_active = keep, max_active
        self.runs: dict[str, Run] = {}

    def start(self, graph: dict, settings: Settings) -> Run:
        active = [r for r in self.runs.values() if not r.done]
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
