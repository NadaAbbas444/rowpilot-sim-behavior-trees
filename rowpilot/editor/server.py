"""The editor's web server (FastAPI).

    GET  /api/node-types               the node palette: types, categories, parameters
    GET  /api/default-tree             the mission tree (mission.build_tree) as a graph
    GET  /api/lane?lane=7              one lane's subtree, as in the mission tree (for Add lane)
    GET  /api/layout                   greenhouse geometry and the planned path, for the map
    GET  /api/simulators               which simulators can run here (Gazebo needs the full setup)
    POST /api/validate                 {graph, settings} -> {errors: [{node, message}]}
    POST /api/runs                     {graph, settings} -> run info (422 with errors if the graph is wrong)
    GET  /api/runs, /api/runs/{id}     run info and summary
    POST /api/runs/{id}/stop
    WS   /api/runs/{id}/stream         every message of the run, from the start, in batches (see runs.py)
    GET/PUT/DELETE /api/trees/{name}   saved trees (JSON files in the trees folder); GET /api/trees lists them

The built web page (web/dist) is served at /, if it has been built.
"""

from __future__ import annotations

import json
import re
from contextlib import asynccontextmanager
from dataclasses import asdict
from pathlib import Path
from typing import Annotated, Any, Literal

from fastapi import FastAPI, HTTPException, Query, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from ..layout import GreenhouseLayout
from ..trolley import planned_route as trolley_route
from .nodes import GRAPH_VERSION, default_graph, lane_graph, node_types, validate
from .runs import GazeboSetup, RunManager, Settings

WEB_DIST = Path(__file__).resolve().parents[2] / "web" / "dist"
TREE_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")


class GraphNode(BaseModel):
    id: str
    type: str
    name: str
    params: dict[str, Any] = {}
    position: dict[str, float] | None = None


class GraphEdge(BaseModel):
    source: str
    target: str


class Graph(BaseModel):
    version: int = GRAPH_VERSION
    nodes: list[GraphNode]
    edges: list[GraphEdge] = []


class SettingsModel(BaseModel):
    simulator: Literal["python", "gazebo"] = "python"
    robot: Literal["drone", "trolley"] = "drone"
    vehicle: Literal["firefly", "iris"] = "firefly"
    gazebo_window: bool = True
    lanes: int = Field(6, ge=1, le=20)
    open_ends: bool = False
    row_length: float = Field(GreenhouseLayout.row_length, ge=3, le=100)
    path_width: float = Field(1.6, ge=0.8, le=4)
    seed: int = Field(0, ge=0)
    battery: float = Field(100.0, ge=0, le=100)
    wind: float = Field(0.3, ge=0, le=3)
    vo_dropouts: list[tuple[float, float]] = []
    altitude: float = Field(1.5, ge=0.5, le=2.4)
    sweep_speed: float = Field(0.8, ge=0.1, le=2)
    speed: float = Field(20.0, ge=0, le=1000, description="simulated seconds per real second, 0 = flat out")
    max_time_s: float = Field(1800.0, ge=1, le=7200)

    def settings(self) -> Settings:
        return Settings(**self.model_dump())


class RunRequest(BaseModel):
    graph: Graph
    settings: SettingsModel = SettingsModel()


def _graph(g: Graph) -> dict:
    return g.model_dump(exclude_none=True)


def layout_info(lay: GreenhouseLayout, robot: str = "drone") -> dict:
    return {
        "n_lanes": lay.n_lanes,
        "row_length": lay.row_length,
        "crop_width": lay.crop_width,
        "pitch": lay.pitch,
        "headland": lay.headland,
        "closed_end": lay.closed_end,
        "far_x": lay.far_x,
        "home": lay.home,
        "crop_rows_y": lay.crop_row_centers_y(),
        "lane_centers_y": [lay.lane_center_y(i) for i in range(lay.n_lanes)],
        "planned_route": trolley_route(lay) if robot == "trolley" else lay.planned_route(),
    }


def create_app(
    trees_dir: Path | str = "trees", runs: RunManager | None = None, gazebo: GazeboSetup | None = None
) -> FastAPI:
    manager = runs or RunManager(gazebo=gazebo)

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        yield
        manager.stop_all()

    app = FastAPI(title="rowpilot behaviour tree editor", version="0.1.0", lifespan=lifespan)
    trees = Path(trees_dir)
    app.state.runs = manager

    @app.get("/api/health")
    def health() -> dict:
        return {"ok": True}

    @app.get("/api/node-types")
    def get_node_types() -> list[dict]:
        return node_types()

    @app.get("/api/default-tree")
    def get_default_tree(
        lanes: Annotated[int, Query(ge=1, le=20)] = 6,
        open_ends: bool = False,
        robot: Literal["drone", "trolley"] = "drone",
    ) -> dict:
        return default_graph(GreenhouseLayout(n_lanes=lanes, closed_end=not open_ends), robot)

    @app.get("/api/lane")
    def get_lane(
        lane: Annotated[int, Query(ge=1, le=20)],
        open_ends: bool = False,
        robot: Literal["drone", "trolley"] = "drone",
    ) -> dict:
        return lane_graph(lane, closed_end=not open_ends, robot=robot)

    @app.get("/api/layout")
    def get_layout(
        lanes: Annotated[int, Query(ge=1, le=20)] = 6,
        open_ends: bool = False,
        row_length: Annotated[float, Query(ge=3, le=100)] = GreenhouseLayout.row_length,
        path_width: Annotated[float, Query(ge=0.8, le=4)] = 1.6,
        robot: Literal["drone", "trolley"] = "drone",
    ) -> dict:
        return layout_info(
            GreenhouseLayout(n_lanes=lanes, row_length=row_length, path_width=path_width, closed_end=not open_ends),
            robot,
        )

    @app.get("/api/simulators")
    def get_simulators() -> dict:
        return manager.simulators()

    @app.get("/api/settings/defaults")
    def get_settings_defaults() -> dict:
        return asdict(Settings())

    @app.post("/api/validate")
    def post_validate(req: RunRequest) -> dict:
        s = req.settings.settings()
        return {"errors": validate(_graph(req.graph), s.layout(), s.robot)}

    @app.post("/api/runs", status_code=201)
    def post_run(req: RunRequest) -> dict:
        graph, settings = _graph(req.graph), req.settings.settings()
        errors = validate(graph, settings.layout(), settings.robot)
        if errors:
            raise HTTPException(422, detail={"errors": errors})
        try:
            run = manager.start(graph, settings)
        except ValueError as e:  # the Gazebo simulation is not set up
            raise HTTPException(409, detail=str(e)) from e
        except RuntimeError as e:
            raise HTTPException(429, detail=str(e)) from e
        return run.info()

    @app.get("/api/runs")
    def list_runs() -> list[dict]:
        return [r.info() for r in sorted(manager.runs.values(), key=lambda r: -r.created)]

    def _run(run_id: str):
        run = manager.get(run_id)
        if run is None:
            raise HTTPException(404, detail=f"no run {run_id}")
        return run

    @app.get("/api/runs/{run_id}")
    def get_run(run_id: str) -> dict:
        return _run(run_id).info()

    @app.post("/api/runs/{run_id}/stop")
    def stop_run(run_id: str) -> dict:
        run = _run(run_id)
        run.stop()
        return run.info()

    @app.websocket("/api/runs/{run_id}/stream")
    async def stream_run(ws: WebSocket, run_id: str, start: int = 0) -> None:
        run = manager.get(run_id)
        await ws.accept()
        if run is None:
            await ws.send_json([{"kind": "end", "status": "error", "summary": None, "error": f"no run {run_id}"}])
            await ws.close()
            return
        try:
            async for batch in run.follow(start):
                await ws.send_json(batch)
            await ws.close()
        except WebSocketDisconnect:
            pass

    # ---- saved trees ------------------------------------------------------------------------------
    def _tree_path(name: str) -> Path:
        if not TREE_NAME.match(name):
            raise HTTPException(400, detail="tree names: letters, digits, '_', '-', '.', up to 64")
        return trees / f"{name}.json"

    @app.get("/api/trees")
    def list_trees() -> list[dict]:
        if not trees.is_dir():
            return []
        return [
            {"name": p.stem, "modified": p.stat().st_mtime}
            for p in sorted(trees.glob("*.json"), key=lambda p: -p.stat().st_mtime)
        ]

    @app.get("/api/trees/{name}")
    def get_tree(name: str) -> dict:
        p = _tree_path(name)
        if not p.is_file():
            raise HTTPException(404, detail=f"no tree {name}")
        return json.loads(p.read_text())

    @app.put("/api/trees/{name}")
    def put_tree(name: str, graph: Graph) -> dict:
        p = _tree_path(name)
        trees.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(_graph(graph), indent=1))
        return {"name": name, "modified": p.stat().st_mtime}

    @app.delete("/api/trees/{name}", status_code=204)
    def delete_tree(name: str) -> None:
        p = _tree_path(name)
        if not p.is_file():
            raise HTTPException(404, detail=f"no tree {name}")
        p.unlink()

    # ---- the web page -------------------------------------------------------------------------------
    if (WEB_DIST / "index.html").is_file():
        app.mount("/assets", StaticFiles(directory=WEB_DIST / "assets"), name="assets")

        @app.get("/{path:path}", include_in_schema=False)
        def page(path: str) -> FileResponse:
            if path.startswith("api/"):
                raise HTTPException(404, detail=f"no API endpoint /{path}")
            f = (WEB_DIST / path).resolve()
            inside = f.is_relative_to(WEB_DIST.resolve())
            return FileResponse(f if path and inside and f.is_file() else WEB_DIST / "index.html")

    return app
