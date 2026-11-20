import {
  Background,
  Controls,
  MiniMap,
  ReactFlow,
  ReactFlowProvider,
  addEdge,
  useEdgesState,
  useNodesInitialized,
  useNodesState,
  useReactFlow,
  type Connection,
  type Edge,
  type IsValidConnection,
} from '@xyflow/react';
import '@xyflow/react/dist/style.css';
import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { ApiError, api, followRun } from './api';
import BTNodeView, { CanvasContext, type CanvasInfo } from './components/BTNodeView';
import Inspector from './components/Inspector';
import LogPanel, { SOURCE_LABEL } from './components/LogPanel';
import MapView from './components/MapView';
import Palette, { DRAG_TYPE } from './components/Palette';
import RunPanel from './components/RunPanel';
import {
  DX,
  childrenOf,
  connectionProblem,
  defaultParams,
  descendants,
  edgeFor,
  fromFlow,
  insertSubtree,
  laneNumbers,
  laneSlot,
  nextLane,
  removeSubtree,
  nextId,
  tidy,
  toFlow,
  uniqueName,
  type BTNode,
} from './graph';
import { applyMessages, formatTime, idleRun, startRun, type RunView } from './runState';
import type { Graph, GraphIssue, LayoutInfo, NodeTypeSpec, Robot, RunMessage, Settings, Simulators } from './types';

const ROBOT_LABEL: Record<Robot, string> = { drone: 'drone', trolley: 'trolley' };
const nodeTypes = { bt: BTNodeView };
const SPEEDS: [number, string][] = [
  [1, '1×'],
  [5, '5×'],
  [20, '20×'],
  [100, '100×'],
  [0, 'max'],
];

const store = {
  get<T>(key: string): T | null {
    try {
      const s = localStorage.getItem(key);
      return s ? (JSON.parse(s) as T) : null;
    } catch {
      return null;
    }
  },
  set(key: string, value: unknown): void {
    try {
      localStorage.setItem(key, JSON.stringify(value));
    } catch {
      /* private window or storage full: the editor works without it */
    }
  },
};

function Editor() {
  const flow = useReactFlow();
  const [specList, setSpecList] = useState<NodeTypeSpec[]>([]);
  const specs = useMemo(() => new Map(specList.map((s) => [s.type, s])), [specList]);
  const [nodes, setNodes, onNodesChange] = useNodesState<BTNode>([]);
  const [edges, setEdges, onEdgesChange] = useEdgesState<Edge>([]);
  const [settings, setSettings] = useState<Settings | null>(null);
  const [layout, setLayout] = useState<LayoutInfo | null>(null);
  const [issues, setIssues] = useState<GraphIssue[]>([]);
  const [selected, setSelected] = useState<string | null>(null);
  const [collapsed, setCollapsed] = useState<Set<string>>(new Set());
  const [tab, setTab] = useState<'node' | 'log' | 'run'>('run');
  const [run, setRun] = useState<RunView>(idleRun);
  const [followNode, setFollowNode] = useState(false);
  const [treeName, setTreeName] = useState('mission');
  const [saved, setSaved] = useState<string[]>([]);
  const [toast, setToast] = useState<{ text: string; bad?: boolean } | null>(null);
  const [ready, setReady] = useState(false);
  const [simulators, setSimulators] = useState<Simulators | null>(null);
  const pending = useRef<RunMessage[]>([]);
  const unfollow = useRef<(() => void) | null>(null);
  const fileInput = useRef<HTMLInputElement>(null);
  const fitPending = useRef(false);
  const measured = useNodesInitialized();
  useEffect(() => {
    if (measured && fitPending.current) {
      fitPending.current = false;
      flow.fitView({ padding: 0.08, maxZoom: 1 });
    }
  }, [measured, nodes, flow]);

  const say = useCallback((text: string, bad = false) => {
    setToast({ text, bad });
    window.setTimeout(() => setToast((t) => (t?.text === text ? null : t)), bad ? 6000 : 2500);
  }, []);

  /** Show a tree. fold: 'lanes' folds every lane but the first (the mission tree is long),
   *  or the ids to keep folded (a tree restored in this browser). */
  const loadGraph = useCallback(
    (g: Graph, sp: Map<string, NodeTypeSpec> = specs, fold: 'lanes' | string[] = []) => {
      const f = toFlow(g, sp);
      const fold_ = new Set(
        fold === 'lanes'
          ? f.nodes.filter((n) => /^Lane \d+$/.test(n.data.name) && n.data.name !== 'Lane 1').map((n) => n.id)
          : fold.filter((id) => f.nodes.some((n) => n.id === id)),
      );
      const needsLayout = fold === 'lanes' || g.nodes.some((n) => !n.position);
      setNodes(needsLayout ? tidy(f.nodes, f.edges, fold_) : f.nodes);
      setEdges(f.edges);
      setCollapsed(fold_);
      setSelected(null);
      fitPending.current = true;
    },
    [specs, setNodes, setEdges, flow],
  );

  // ---- start up: node types, settings, the last tree (or the mission tree) ------------------------
  useEffect(() => {
    (async () => {
      try {
        const [types, defaults, sims] = await Promise.all([api.nodeTypes(), api.settingsDefaults(), api.simulators()]);
        setSimulators(sims);
        const sp = new Map(types.map((s) => [s.type, s]));
        setSpecList(types);
        const s = { ...defaults, ...(store.get<Partial<Settings>>('rowpilot.settings') ?? {}) };
        const gzOk = s.robot === 'trolley' ? sims.gazebo_trolley?.available : sims[s.simulator]?.available;
        const trolleyInIsaac = s.robot === 'trolley' && s.simulator === 'isaac';
        if (s.simulator !== 'python' && (!gzOk || trolleyInIsaac)) s.simulator = 'python';
        setSettings(s);
        const last = store.get<Graph>('rowpilot.graph');
        if (last?.nodes?.length) loadGraph(last, sp, store.get<string[]>('rowpilot.collapsed') ?? []);
        else loadGraph(await api.defaultTree(s.lanes, s.open_ends, s.robot), sp, 'lanes');
        setTreeName(store.get<string>('rowpilot.treeName') ?? 'mission');
        setReady(true);
      } catch (e) {
        say(`cannot reach the server: ${(e as Error).message}. Start it with: python -m rowpilot editor`, true);
      }
    })();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  const graph = useMemo(() => fromFlow(nodes, edges), [nodes, edges]);

  // remember the tree and settings in this browser; check the tree on the server
  useEffect(() => {
    if (!ready || !settings) return;
    const h = window.setTimeout(() => {
      store.set('rowpilot.graph', graph);
      store.set('rowpilot.settings', settings);
      store.set('rowpilot.treeName', treeName);
      store.set('rowpilot.collapsed', [...collapsed]);
      api
        .validate(graph, settings)
        .then((r) => setIssues(r.errors))
        .catch(() => undefined);
    }, 350);
    return () => window.clearTimeout(h);
  }, [graph, settings, treeName, collapsed, ready]);

  const geomKey =
    settings && `${settings.lanes}|${settings.open_ends}|${settings.row_length}|${settings.path_width}|${settings.robot}`;
  useEffect(() => {
    if (!settings) return;
    api.layout(settings).then(setLayout).catch(() => undefined);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [geomKey]);

  const refreshSaved = useCallback(() => {
    api
      .trees()
      .then((t) => setSaved(t.map((x) => x.name)))
      .catch(() => undefined);
  }, []);
  useEffect(refreshSaved, [refreshSaved]);

  // ---- run messages arrive in batches; apply them once per animation frame -------------------------
  useEffect(() => {
    let frame = 0;
    const loop = () => {
      if (pending.current.length) {
        const msgs = pending.current.splice(0);
        setRun((v) => applyMessages(v, msgs));
      }
      frame = requestAnimationFrame(loop);
    };
    frame = requestAnimationFrame(loop);
    return () => cancelAnimationFrame(frame);
  }, []);
  useEffect(() => () => unfollow.current?.(), []);

  const running = run.phase === 'running';
  const gazeboSim = settings?.simulator === 'gazebo' || settings?.simulator === 'isaac'; // the full chain, real time
  const simName = settings?.simulator === 'isaac' ? 'Isaac Sim' : 'Gazebo';

  const startTheRun = async () => {
    if (!settings) return;
    try {
      const info = await api.startRun(graph, settings);
      unfollow.current?.();
      pending.current = [];
      setRun(startRun(info.id));
      unfollow.current = followRun(info.id, (batch) => pending.current.push(...batch));
      setTab((t) => (t === 'run' ? 'log' : t));
    } catch (e) {
      if (e instanceof ApiError && e.issues.length) {
        setIssues(e.issues);
        say(`the tree has ${e.issues.length} problem${e.issues.length > 1 ? 's' : ''}: ${e.issues[0].message}`, true);
      } else say((e as Error).message, true);
    }
  };
  const stopTheRun = () => run.runId && api.stopRun(run.runId).catch((e) => say(e.message, true));

  // follow the running action on the canvas
  const nameToId = useMemo(() => new Map(nodes.map((n) => [n.data.name, n.id])), [nodes]);
  const tipId = run.last ? nameToId.get(run.last.node) : undefined;
  useEffect(() => {
    if (!followNode || !tipId) return;
    const n = flow.getNode(tipId);
    if (n && !n.hidden) flow.setCenter(n.position.x + 105, n.position.y + 30, { zoom: flow.getZoom(), duration: 300 });
  }, [tipId, followNode, flow]);

  // ---- editing --------------------------------------------------------------------------------
  const kids = useMemo(() => childrenOf(nodes, edges), [nodes, edges]);
  const hidden = useMemo(() => {
    const h = new Set<string>();
    collapsed.forEach((id) => descendants(id, kids).forEach((d) => h.add(d)));
    return h;
  }, [collapsed, kids]);

  const toggleCollapse = useCallback(
    (id: string) => {
      setCollapsed((c) => {
        const next = new Set(c);
        if (next.has(id)) next.delete(id);
        else next.add(id);
        setNodes((ns) => tidy(ns, edges, next));
        return next;
      });
    },
    [edges, setNodes],
  );

  const issuesByNode = useMemo(() => {
    const m: Record<string, string[]> = {};
    for (const i of issues) if (i.node) (m[i.node] ??= []).push(i.message);
    return m;
  }, [issues]);
  const globalIssues = issues.filter((i) => !i.node || !nodes.some((n) => n.id === i.node));

  const canvas: CanvasInfo = useMemo(
    () => ({
      specs,
      status: run.status,
      lastLog: run.lastLog,
      issues: issuesByNode,
      collapsed,
      hiddenCount: Object.fromEntries([...collapsed].map((id) => [id, descendants(id, kids).length])),
      toggleCollapse,
    }),
    [specs, run.status, run.lastLog, issuesByNode, collapsed, kids, toggleCollapse],
  );

  const shownNodes = useMemo(
    () => nodes.map((n) => (hidden.has(n.id) !== Boolean(n.hidden) ? { ...n, hidden: hidden.has(n.id) } : n)),
    [nodes, hidden],
  );
  const shownEdges = useMemo(
    () =>
      edges.map((e) => {
        const st = run.status[e.target];
        const cls = st && st !== 'INVALID' ? `e-${st.toLowerCase()}` : '';
        const hide = hidden.has(e.target);
        return e.className === cls && Boolean(e.hidden) === hide && e.animated === (st === 'RUNNING')
          ? e
          : { ...e, className: cls, hidden: hide, animated: st === 'RUNNING' };
      }),
    [edges, run.status, hidden],
  );

  const isValidConnection: IsValidConnection = useCallback(
    (c) => !connectionProblem(nodes, edges, specs, c.source, c.target),
    [nodes, edges, specs],
  );
  const onConnect = useCallback(
    (c: Connection) => {
      const problem = connectionProblem(nodes, edges, specs, c.source, c.target);
      if (problem) return say(problem, true);
      setEdges((es) => addEdge(edgeFor(c.source, c.target), es));
    },
    [nodes, edges, specs, setEdges, say],
  );

  const addNode = useCallback(
    (type: string, position?: { x: number; y: number }) => {
      const spec = specs.get(type);
      if (!spec) return;
      const id = nextId(nodes);
      const name = uniqueName(type, nodes.map((n) => n.data.name));
      // Added from the palette with a node selected: as the selected node's last child if it can take
      // one, otherwise right after it in its parent. Then tidy, so nothing overlaps.
      const sel = !position && selected ? nodes.find((n) => n.id === selected) : undefined;
      const canTake = (pid: string) => !connectionProblem(nodes, edges, specs, pid, '__new__');
      let pos = position;
      let link: string | null = null;
      if (sel && canTake(sel.id)) {
        const siblings = (kids.get(sel.id) ?? []).map((k) => nodes.find((n) => n.id === k)!);
        pos = { x: sel.position.x + DX, y: siblings.reduce((m, s) => Math.max(m, s.position.y), sel.position.y) + 1 };
        link = sel.id;
      } else if (sel) {
        const up = edges.find((e) => e.target === sel.id)?.source;
        if (up && canTake(up)) {
          pos = { x: sel.position.x, y: sel.position.y + 1 };
          link = up;
        }
      }
      if (!pos) {
        const b = document.querySelector('.canvas')!.getBoundingClientRect();
        pos = flow.screenToFlowPosition({ x: b.x + b.width / 2, y: b.y + b.height / 2 });
      }
      const node: BTNode = {
        id,
        type: 'bt',
        position: pos,
        selected: true,
        data: { type, name, params: defaultParams(spec), category: spec.category },
      };
      const ns = [...nodes.map((n) => (n.selected ? { ...n, selected: false } : n)), node];
      if (link) {
        const es = [...edges, edgeFor(link, id)];
        setEdges(es);
        setNodes(tidy(ns, es, collapsed));
      } else setNodes(ns);
      setSelected(id);
      setTab('node');
    },
    [specs, nodes, edges, kids, selected, collapsed, flow, setNodes, setEdges],
  );

  const onDrop = (e: React.DragEvent) => {
    const type = e.dataTransfer.getData(DRAG_TYPE);
    if (!type) return;
    e.preventDefault();
    addNode(type, flow.screenToFlowPosition({ x: e.clientX - 100, y: e.clientY - 20 }));
  };

  const patchNode = (id: string, patch: Partial<BTNode['data']>) =>
    setNodes((ns) => ns.map((n) => (n.id === id ? { ...n, data: { ...n.data, ...patch } } : n)));

  const deleteNode = (id: string) => {
    // with everything under it: a lane's steps without the lane would be left loose
    const r = removeSubtree(nodes, edges, id);
    setNodes(r.nodes);
    setEdges(r.edges);
    setSelected(null);
  };

  /** The greenhouse is fixed (Run settings); the tree chooses which of its lanes to sweep. + adds the
   *  first lane the tree does not sweep yet, as the mission tree builds it; − removes the last one. */
  const lanesInTree = useMemo(() => laneNumbers(nodes), [nodes]);
  const addLane = async () => {
    if (!settings) return;
    const k = nextLane(nodes, settings.lanes);
    if (k === null) return say(`the tree already sweeps all ${settings.lanes} lanes of the greenhouse`, true);
    const slot = laneSlot(nodes, edges, specs, selected);
    if (!slot) return say('Select the sequence the lane should go into (e.g. Mission), then press + again', true);
    try {
      const sub = await api.lane(k, settings.open_ends, settings.robot);
      const r = insertSubtree(nodes, edges, sub, specs, slot.parent, slot.y);
      const ns = tidy(r.nodes, r.edges, collapsed).map((n) => ({ ...n, selected: n.id === r.root }));
      setEdges(r.edges);
      setNodes(ns);
      setSelected(r.root);
      say(`added Lane ${k}: the tree sweeps ${lanesInTree.length + 1} of ${settings.lanes} lanes`);
    } catch (e) {
      say((e as Error).message, true);
    }
  };
  const removeLane = () => {
    const last = nodes
      .filter((n) => /^Lane \d+$/.test(n.data.name))
      .sort((a, b) => Number(b.data.name.slice(5)) - Number(a.data.name.slice(5)))[0];
    if (!last) return say('the tree has no lanes to remove', true);
    const r = removeSubtree(nodes, edges, last.id);
    setEdges(r.edges);
    setNodes(tidy(r.nodes, r.edges, collapsed));
    setSelected(null);
    say(`removed ${last.data.name}: the tree sweeps ${lanesInTree.length - 1} of ${settings?.lanes} lanes`);
  };

  // ---- files ---------------------------------------------------------------------------------------
  const loadMission = async () => {
    if (!settings) return;
    loadGraph(await api.defaultTree(settings.lanes, settings.open_ends, settings.robot), specs, 'lanes');
    setTreeName('mission');
    const kind = settings.open_ends ? 'open' : 'dead-end';
    say(`${ROBOT_LABEL[settings.robot]} mission tree for ${settings.lanes} ${kind} lanes`);
  };
  // A different robot needs its own tree (the drone's nodes cannot drive the trolley, and back).
  const changeSettings = async (next: Settings) => {
    const robotChanged = settings !== null && next.robot !== settings.robot;
    setSettings(next);
    if (!robotChanged) return;
    loadGraph(await api.defaultTree(next.lanes, next.open_ends, next.robot), specs, 'lanes');
    setTreeName('mission');
    say(`switched to the ${ROBOT_LABEL[next.robot]}: loaded its mission tree for ${next.lanes} lanes`);
  };
  const newTree = () => {
    loadGraph({
      version: 1,
      nodes: [{ id: 'n1', type: 'Sequence', name: 'Root', params: { memory: false }, position: { x: 0, y: 0 } }],
      edges: [],
    });
    setTreeName('untitled');
  };
  const save = async () => {
    try {
      await api.saveTree(treeName, graph);
      refreshSaved();
      say(`saved “${treeName}”`);
    } catch (e) {
      say((e as Error).message, true);
    }
  };
  const open = async (name: string) => {
    try {
      loadGraph(await api.loadTree(name));
      setTreeName(name);
    } catch (e) {
      say((e as Error).message, true);
    }
  };
  const exportFile = () => {
    const a = document.createElement('a');
    a.href = URL.createObjectURL(new Blob([JSON.stringify(graph, null, 1)], { type: 'application/json' }));
    a.download = `${treeName || 'tree'}.json`;
    a.click();
    URL.revokeObjectURL(a.href);
  };
  const importFile = async (f: File) => {
    try {
      const g = JSON.parse(await f.text()) as Graph;
      if (!Array.isArray(g.nodes) || !Array.isArray(g.edges)) throw new Error('not a tree file');
      loadGraph(g);
      setTreeName(f.name.replace(/\.json$/, ''));
    } catch (e) {
      say(`cannot open ${f.name}: ${(e as Error).message}`, true);
    }
  };

  const sel = nodes.find((n) => n.id === selected);
  const last = run.last;

  return (
    <div className="app">
      <header className="toolbar">
        <h1>
          rowpilot <span>behaviour tree editor</span>
        </h1>
        <div className="group">
          <button onClick={loadMission} title="the mission tree for the greenhouse in Run settings">
            Mission tree
          </button>
          <button onClick={newTree}>New</button>
          <select
            value=""
            onFocus={refreshSaved}
            onChange={(e) => e.target.value && open(e.target.value)}
            aria-label="Open a saved tree"
          >
            <option value="">Open…</option>
            {saved.map((n) => (
              <option key={n}>{n}</option>
            ))}
          </select>
          <input
            className="tree-name"
            value={treeName}
            onChange={(e) => setTreeName(e.target.value)}
            aria-label="Tree name"
          />
          <button onClick={save}>Save</button>
          <button onClick={() => fileInput.current?.click()}>Import</button>
          <button onClick={exportFile}>Export</button>
          <input
            ref={fileInput}
            type="file"
            accept=".json,application/json"
            hidden
            onChange={(e) => {
              const f = e.target.files?.[0];
              if (f) importFile(f);
              e.target.value = '';
            }}
          />
        </div>
        <div className="group">
          <button onClick={() => setNodes((ns) => tidy(ns, edges, collapsed))} title="arrange the tree, keeping its order">
            Tidy
          </button>
          <button
            onClick={() => {
              const composites = nodes.filter((n) => (kids.get(n.id)?.length ?? 0) > 0 && /^Lane \d+$/.test(n.data.name));
              const next = new Set(composites.map((n) => n.id));
              setCollapsed(next);
              setNodes((ns) => tidy(ns, edges, next));
            }}
            title="fold every lane into one node"
          >
            Fold lanes
          </button>
          <button
            onClick={() => {
              setCollapsed(new Set());
              setNodes((ns) => tidy(ns, edges, new Set()));
            }}
          >
            Unfold all
          </button>
        </div>
        <div className="group lanes" title="the lanes the tree sweeps, of the greenhouse's lanes (Run settings)">
          <span className="muted">Lanes</span>
          <button onClick={removeLane} disabled={!lanesInTree.length} aria-label="remove the last lane from the tree">
            −
          </button>
          <span className="lane-count">
            {lanesInTree.length} of {settings?.lanes ?? '–'}
          </span>
          <button
            onClick={addLane}
            disabled={!settings || lanesInTree.length >= settings.lanes}
            aria-label="add the next lane to the tree"
          >
            +
          </button>
        </div>
        <div className="group run-group">
          <button
            className={`sim-badge ${gazeboSim ? 'gz' : ''}`}
            onClick={() => setTab('run')}
            title="the simulator the next run uses (change it in Run settings)"
          >
            {gazeboSim
              ? `${simName}${settings?.robot === 'trolley' ? ': trolley' : ' + ArduPilot'}`
              : `Python sim: ${ROBOT_LABEL[settings?.robot ?? 'drone']}`}
          </button>
          <label className="speed">
            speed
            <select
              value={gazeboSim ? 1 : (settings?.speed ?? 20)}
              onChange={(e) => settings && setSettings({ ...settings, speed: Number(e.target.value) })}
              disabled={running || gazeboSim}
              title={gazeboSim ? `${simName} runs in real time` : 'simulated seconds per real second'}
            >
              {SPEEDS.map(([v, l]) => (
                <option key={v} value={v}>
                  {l}
                </option>
              ))}
            </select>
          </label>
          <label className="follow">
            <input type="checkbox" checked={followNode} onChange={(e) => setFollowNode(e.target.checked)} /> follow
          </label>
          {running ? (
            <button className="stop" onClick={stopTheRun}>
              ■ Stop
            </button>
          ) : (
            <button className="run" onClick={startTheRun} disabled={!ready || issues.length > 0}>
              ▶ Run
            </button>
          )}
        </div>
      </header>

      <Palette specs={specList.filter((s) => s.robots.includes(settings?.robot ?? 'drone'))} onAdd={(t) => addNode(t)} />

      <main className="canvas" onDragOver={(e) => e.preventDefault()} onDrop={onDrop}>
        <CanvasContext.Provider value={canvas}>
          <ReactFlow
            nodes={shownNodes}
            edges={shownEdges}
            nodeTypes={nodeTypes}
            onNodesChange={onNodesChange}
            onEdgesChange={onEdgesChange}
            onConnect={onConnect}
            isValidConnection={isValidConnection}
            onSelectionChange={({ nodes: s }) => {
              const id = s.length === 1 ? s[0].id : null;
              setSelected(id);
              if (id) setTab('node');
            }}
            deleteKeyCode={['Backspace', 'Delete']}
            onBeforeDelete={async ({ nodes: del, edges: delEdges }) => {
              // a composite goes with everything under it, as with the Node tab's button
              const ids = new Set(del.flatMap((n) => [n.id, ...descendants(n.id, kids)]));
              return {
                nodes: nodes.filter((n) => ids.has(n.id)),
                edges: [...delEdges, ...edges.filter((e) => ids.has(e.source) || ids.has(e.target))],
              };
            }}
            minZoom={0.1}
            colorMode="system"
            proOptions={{ hideAttribution: true }}
          >
            <Background gap={16} />
            <Controls />
            <MiniMap pannable zoomable nodeClassName={(n) => `mm-${(n.data as BTNode['data']).category} mm-${(run.status[n.id] ?? '').toLowerCase()}`} />
          </ReactFlow>
        </CanvasContext.Provider>
        {globalIssues.length > 0 && (
          <div className="canvas-issues">
            {globalIssues.map((i) => (
              <div key={i.message}>⚠ {i.message}</div>
            ))}
          </div>
        )}
        {toast && <div className={`toast ${toast.bad ? 'bad' : ''}`}>{toast.text}</div>}
      </main>

      <aside className="side">
        <MapView
          layout={layout}
          truePath={run.truePath}
          estPath={run.estPath}
          last={last}
          robot={settings?.robot ?? 'drone'}
        />
        <div className="map-legend">
          <span className="lg-planned">planned</span>
          <span className="lg-true">flown</span>
          <span className="lg-est">odometry</span>
        </div>
        <nav className="tabs">
          {(['node', 'log', 'run'] as const).map((t) => (
            <button key={t} className={tab === t ? 'active' : ''} onClick={() => setTab(t)}>
              {{ node: 'Node', log: `Log${run.logs.length ? ` (${run.logs.filter((l) => l.kind !== 'status').length})` : ''}`, run: 'Run settings' }[t]}
            </button>
          ))}
        </nav>
        <div className="tab-body" key={`${tab}-${tab === "node" ? selected : ""}`}>
          {tab === 'node' &&
            (sel ? (
              <Inspector
                id={sel.id}
                data={sel.data}
                spec={specs.get(sel.data.type)}
                status={run.status[sel.id]}
                issues={issuesByNode[sel.id] ?? []}
                logs={run.logs.filter((l) => l.id === sel.id)}
                children={kids.get(sel.id) ?? []}
                onChange={(p) => patchNode(sel.id, p)}
                onDelete={() => deleteNode(sel.id)}
              />
            ) : (
              <p className="muted pad">
                Select a node to edit it and see its log. Connect a parent's right handle to a child's left handle;
                children run top to bottom.
              </p>
            ))}
          {tab === 'log' && (
            <LogPanel
              logs={run.logs}
              onPick={(id) => {
                const n = flow.getNode(id);
                setNodes((ns) => ns.map((x) => ({ ...x, selected: x.id === id })));
                setSelected(id);
                setTab('node');
                if (n && !n.hidden) flow.setCenter(n.position.x + 105, n.position.y + 30, { zoom: Math.max(0.8, flow.getZoom()), duration: 300 });
              }}
            />
          )}
          {tab === 'run' && settings && (
            <RunPanel settings={settings} onChange={changeSettings} run={run} simulators={simulators} />
          )}
        </div>
      </aside>

      <footer className="statusbar">
        <span className={`phase ph-${run.phase}`}>
          {run.phase === 'idle' ? 'ready' : run.phase === 'running' && !last ? 'starting' : run.phase}
        </span>
        {running && !last && run.lastSystem && (
          <span className="sys">
            {SOURCE_LABEL[run.lastSystem.source ?? ''] ?? run.lastSystem.source}: {run.lastSystem.msg}
          </span>
        )}
        {last && (
          <>
            <span>t {formatTime(last.t)}</span>
            <span>battery {last.battery.toFixed(0)}%</span>
            <span className={last.vo_ok ? '' : 'warn'}>odometry {last.vo_ok ? 'ok' : 'LOST'}</span>
            <span>
              at ({last.x.toFixed(1)}, {last.y.toFixed(1)}) m
            </span>
            <span className="now">now: {last.node || '–'}</span>
          </>
        )}
        {run.summary && <span className="outcome">{run.summary.outcome}</span>}
        <span className="spacer" />
        <span className={issues.length ? 'warn' : 'ok'}>
          {issues.length ? `${issues.length} problem${issues.length > 1 ? 's' : ''} in the tree` : 'tree ok'}
        </span>
        <span>{nodes.length} nodes</span>
      </footer>
    </div>
  );
}

export default function App() {
  return (
    <ReactFlowProvider>
      <Editor />
    </ReactFlowProvider>
  );
}
