// Talking to the Python server (rowpilot/editor/server.py).
import type { Graph, GraphIssue, LayoutInfo, NodeTypeSpec, RunInfo, RunMessage, Settings, Simulators } from './types';

export class ApiError extends Error {
  constructor(
    message: string,
    public status: number,
    public issues: GraphIssue[] = [],
  ) {
    super(message);
  }
}

async function call<T>(path: string, init?: RequestInit): Promise<T> {
  const r = await fetch(path, {
    ...init,
    headers: init?.body ? { 'Content-Type': 'application/json' } : undefined,
  });
  if (r.status === 204) return undefined as T;
  const body = await r.json().catch(() => null);
  if (!r.ok) {
    const detail = body?.detail;
    const issues: GraphIssue[] = detail?.errors ?? [];
    const msg = typeof detail === 'string' ? detail : issues.length ? issues[0].message : `HTTP ${r.status}`;
    throw new ApiError(msg, r.status, issues);
  }
  return body as T;
}

const json = (body: unknown): RequestInit => ({ method: 'POST', body: JSON.stringify(body) });

export const api = {
  nodeTypes: () => call<NodeTypeSpec[]>('/api/node-types'),
  defaultTree: (lanes: number, openEnds: boolean) =>
    call<Graph>(`/api/default-tree?lanes=${lanes}&open_ends=${openEnds}`),
  settingsDefaults: () => call<Settings>('/api/settings/defaults'),
  simulators: () => call<Simulators>('/api/simulators'),
  layout: (s: Settings) =>
    call<LayoutInfo>(
      `/api/layout?lanes=${s.lanes}&open_ends=${s.open_ends}&row_length=${s.row_length}&path_width=${s.path_width}`,
    ),
  validate: (graph: Graph, settings: Settings) =>
    call<{ errors: GraphIssue[] }>('/api/validate', json({ graph, settings })),
  startRun: (graph: Graph, settings: Settings) => call<RunInfo>('/api/runs', json({ graph, settings })),
  stopRun: (id: string) => call<RunInfo>(`/api/runs/${id}/stop`, { method: 'POST' }),
  trees: () => call<{ name: string; modified: number }[]>('/api/trees'),
  loadTree: (name: string) => call<Graph>(`/api/trees/${encodeURIComponent(name)}`),
  saveTree: (name: string, graph: Graph) =>
    call<{ name: string }>(`/api/trees/${encodeURIComponent(name)}`, { method: 'PUT', body: JSON.stringify(graph) }),
  deleteTree: (name: string) => call<void>(`/api/trees/${encodeURIComponent(name)}`, { method: 'DELETE' }),
};

/**
 * Follow a run's messages. The server sends everything from `start` on, then new messages as they
 * come; if the connection drops before the run ends, reconnect and carry on from where it stopped.
 * Returns a function that stops following.
 */
export function followRun(
  id: string,
  onBatch: (msgs: RunMessage[]) => void,
  start = 0,
  onDrop?: (received: number) => void,
): () => void {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  let received = start;
  let ended = false;
  let closed = false;
  let ws: WebSocket;
  const open = () => {
    ws = new WebSocket(`${proto}://${location.host}/api/runs/${id}/stream?start=${received}`);
    ws.onmessage = (e) => {
      const batch = JSON.parse(e.data) as RunMessage[];
      received += batch.length;
      if (batch.some((m) => m.kind === 'end')) ended = true;
      onBatch(batch);
    };
    ws.onclose = () => {
      if (closed || ended) return;
      onDrop?.(received);
      setTimeout(() => !closed && open(), 1000);
    };
  };
  open();
  return () => {
    closed = true;
    ws.close();
  };
}
