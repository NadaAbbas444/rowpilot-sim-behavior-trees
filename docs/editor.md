# The behaviour tree editor

A web page for the mission's behaviour tree: draw or change the tree, run it in the simulator, and
watch every node live. Running nodes light up, each node shows its own log, and a map shows where
the drone flies. The page is React + React Flow; the server is FastAPI and runs the same Python
tree code as `python -m rowpilot run` and the ArduPilot simulation.

![The mission running: the active branch in amber, finished steps in green, the log and the map on the right](editor/editor-run.png)

## Start it

You need Python 3.10+ and Node.js 18+ (tested with Node 22).

```bash
pip install -e ".[editor]"            # FastAPI and uvicorn (".[dev]" includes them too)
cd web && npm ci && npm run build && cd ..
python -m rowpilot editor             # then open http://127.0.0.1:8000
```

`npm run build` puts the page into `web/dist/`, which the Python server serves. Saved trees go to
`trees/` (change it with `--trees <folder>`; `--port` and `--host` as usual).

**Working on the page itself:** run the server (`python -m rowpilot editor`) and, in a second terminal,
`cd web && npm run dev`, then open http://localhost:5173. Vite reloads the page on every change and
passes `/api` (and the run's WebSocket) to the Python server.

## Using it

| | |
|---|---|
| **Mission tree** | loads the greenhouse mission (`mission.build_tree`) for the lanes set in *Run settings*. Lanes 2 onwards start folded; **Unfold all** shows them |
| **Palette** (left) | every node type, grouped. Drag one onto the canvas, or double-click it: with a node selected, the new node becomes its last child (or, if the selected node cannot take children, comes right after it) |
| **Connecting** | drag from a parent's right handle to a child's left handle. The editor refuses what cannot work: children on an action or condition, a second child on a decorator, two parents, a cycle |
| **Order** | a parent ticks its children **top to bottom, as drawn**. Move a node up or down to change the order; **Tidy** lays the tree out again keeping that order |
| **Node** tab | the selected node: what it does, its name and parameters, and its own log from the last run |
| **Problems** | the server checks the tree as you edit (unknown lane, missing child, duplicate name, loose node...). Nodes with a problem get a dashed red border, the status bar counts them, and **Run** waits until there are none |
| **Run** | runs the tree in the Python simulator at the chosen **speed** (simulated seconds per real second; *max* as fast as it goes). **follow** keeps the running node in view |
| **Log** tab | every node's log lines as they happen; click a node's name to jump to it. *status changes* adds every SUCCESS / FAILURE / RUNNING |
| **Run settings** | the greenhouse (lanes, dead-end or open lanes, sizes) and the simulation: start battery, gusts, visual odometry dropouts, altitude, sweep speed, seed |
| **Map** (top right) | the greenhouse from above: the planned path (green), where the drone really flew (blue) and where visual odometry thought it was (dashed orange) |
| **Save / Open** | trees saved on the server (`trees/<name>.json`); **Export / Import** downloads or loads a JSON file. The page also keeps your current tree in the browser |

Node names must be unique: mission steps remember by name that they are done, so the mission can
pick up where it was after a failsafe pauses it.

![A run with the battery at 45%: the guard fails, the failsafe flies home and lands](editor/editor-failsafe.png)

*Start battery 45%: in lane 3 `BatteryAboveReserve` fails (red), so `SafetyGuard` switches to the
failsafe branch, which flies back out of the lane and home (amber); the mission branch is no longer
ticked.*

![One node selected: its parameters and its own log](editor/editor-node.png)

## The nodes

The editor offers the nodes the mission is built from, plus a few general ones. Hover over one in
the palette for what it does.

| Kind | Nodes |
|---|---|
| Composites | `Sequence`, `Selector` (both with `memory`: off = start from the first child every tick, so a failing check interrupts what runs after it, as the mission does), `Parallel` |
| Decorators | `Inverter`, `SuccessIsFailure`, `FailureIsSuccess`, `Timeout` (simulated seconds) |
| Conditions | `NotAborted`, `BatteryAboveReserve`, `BatteryAbove`, `VOHealthy`, `VOLostBriefly`, `MustLandNow`, `Airborne` |
| Flight | `Preflight`, `Takeoff`, `Hover`, `Wait`, `GoToPoint`, `ReturnHome`, `FailsafeReturnHome`, `Land`, `MissionLand` |
| Lanes | `GoToLaneEntry`, `AlignToLane`, `SweepLane`, `TurnAround`, `ExitLane` (lanes numbered from 1, next to home) |

How the mission tree works, node by node: [how-it-works.md](how-it-works.md).

## How it is built

```mermaid
flowchart LR
    subgraph Browser
        UI["React + React Flow<br/>web/src"]
    end
    subgraph Server["python -m rowpilot editor (FastAPI)"]
        API["server.py<br/>REST + WebSocket"]
        NODES["nodes.py<br/>node types, graph checks,<br/>graph → py_trees"]
        RUNS["runs.py<br/>one thread per run"]
        SIM["runner.py + sim.py<br/>the tree ticking the simulator"]
    end
    UI -- "GET node types, mission tree, map<br/>POST validate, run, stop" --> API
    API --> NODES
    API --> RUNS
    RUNS --> SIM
    SIM -- "status changes, log lines,<br/>pose every tick" --> RUNS
    RUNS -- "WebSocket: batches of messages" --> UI
```

- **One tree format.** The page and the server exchange a graph: nodes (`id`, `type`, `name`, `params`,
  `position`) and parent → child edges. `rowpilot/editor/nodes.py` checks it and builds the py_trees
  tree; every behaviour carries its graph node's id, so the run's events can say which node on the
  canvas they come from. The mission tree loaded by **Mission tree** is built from the same node types
  and gives exactly the same flight as `mission.build_tree` (a test runs both and compares every event).
- **Runs.** A run is the simulator ticking the tree in its own thread, paced to the chosen speed.
  Everything it produces (status changes, log lines, the pose every tick, the end) is kept in order;
  the WebSocket sends it in batches from any point (`?start=n`), so a page can connect late, reconnect
  after a dropped connection, or several pages can watch one run. The page applies a batch once per
  animation frame.
- **The server decides.** The page refuses obviously wrong connections as you draw, but the rules
  that matter (types, parameters, lanes, names, one root, no cycles) are checked on the server, which
  the page asks as you edit and which refuses to run a tree with problems.

### API

| | |
|---|---|
| `GET /api/node-types` | the palette: types, categories, parameters with defaults and limits |
| `GET /api/default-tree?lanes=6&open_ends=false` | the mission tree as a graph |
| `GET /api/layout?lanes=...` | greenhouse geometry and the planned path, for the map |
| `GET /api/settings/defaults` | default run settings |
| `POST /api/validate` `{graph, settings}` | `{errors: [{node, message}]}` |
| `POST /api/runs` `{graph, settings}` | starts a run (422 with the errors if the tree has problems) |
| `GET /api/runs`, `GET /api/runs/{id}` | run status and summary |
| `POST /api/runs/{id}/stop` | stops a run |
| `WS /api/runs/{id}/stream?start=0` | the run's messages: `event` (status / log / end), `tick` (pose, battery, active node), `end` (summary) |
| `GET/PUT/DELETE /api/trees/{name}`, `GET /api/trees` | saved trees |

The interactive API docs are at http://127.0.0.1:8000/docs while the server runs.

### Code and tests

| | |
|---|---|
| `rowpilot/editor/nodes.py` | node types, graph checks, graph → py_trees, the mission tree as a graph, tidy layout |
| `rowpilot/editor/runs.py` | runs in threads, message log, pacing, stop |
| `rowpilot/editor/server.py` | the FastAPI app |
| `web/src/graph.ts` | graph ↔ React Flow, tick order, tidy layout, connection rules |
| `web/src/runState.ts` | a run's messages → node status, per-node logs, paths |
| `web/src/App.tsx`, `web/src/components/` | the page |
| `tests/test_editor.py` | the mission graph flies exactly like the mission tree; checks; a hand-made tree flies; the API: runs streamed over the WebSocket from any point, stop, errors, saved trees, the page served |
| `web/src/*.test.ts(x)` | `npm test` (Vitest): graph conversion, order, layout, connection rules, run state, the node panel |

CI runs both: the Python tests (with the editor's API), and the page's tests, type check and build,
followed by the editor's Python tests serving the built page.
