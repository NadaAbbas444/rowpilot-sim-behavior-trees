// What a run has shown so far, built from its stream of messages. Pure; tested in runState.test.ts.
import type { EndMsg, RunMessage, Status, Summary, TickMsg } from './types';

export interface LogLine {
  t: number;
  id?: string;
  node: string;
  msg: string;
  kind: 'log' | 'status' | 'end' | 'system';
  status?: Status;
  source?: string; // system messages: who said it
}

export interface PathPoint {
  x: number;
  y: number;
}

export interface RunView {
  runId: string | null;
  phase: 'idle' | 'running' | EndMsg['status'];
  status: Record<string, Status>; // by graph node id
  lastLog: Record<string, string>;
  logs: LogLine[];
  truePath: PathPoint[];
  estPath: PathPoint[];
  last: TickMsg | null;
  lastSystem: LogLine | null; // the flight stack's latest message
  summary: Summary | null;
  error: string | null;
  received: number; // messages so far, to resume the stream after a reconnect
}

export const idleRun: RunView = {
  runId: null,
  phase: 'idle',
  status: {},
  lastLog: {},
  logs: [],
  truePath: [],
  estPath: [],
  last: null,
  lastSystem: null,
  summary: null,
  error: null,
  received: 0,
};

export function startRun(runId: string): RunView {
  return { ...idleRun, runId, phase: 'running' };
}

const MIN_STEP = 0.05; // m between stored path points

function addPoint(path: PathPoint[], x: number, y: number): void {
  const p = path[path.length - 1];
  if (!p || Math.hypot(p.x - x, p.y - y) >= MIN_STEP) path.push({ x, y });
}

/** The run view after a batch of messages (returns a new object; the input is not changed). */
export function applyMessages(view: RunView, msgs: RunMessage[]): RunView {
  if (!msgs.length) return view;
  const v: RunView = {
    ...view,
    status: { ...view.status },
    lastLog: { ...view.lastLog },
    logs: view.logs.slice(),
    truePath: view.truePath.slice(),
    estPath: view.estPath.slice(),
    received: view.received + msgs.length,
  };
  for (const m of msgs) {
    if (m.kind === 'tick') {
      addPoint(v.truePath, m.x, m.y);
      addPoint(v.estPath, m.est_x, m.est_y);
      v.last = m;
    } else if (m.kind === 'event') {
      if (m.type === 'status' && m.id && m.status) {
        const first = !(m.id in v.status);
        v.status[m.id] = m.status;
        if (first && m.status === 'INVALID') continue; // not ticked yet: nothing to report
        v.logs.push({ t: m.t, id: m.id, node: m.node, msg: m.status, kind: 'status', status: m.status });
      } else if (m.type === 'log') {
        if (m.id) v.lastLog[m.id] = m.msg ?? '';
        v.logs.push({ t: m.t, id: m.id, node: m.node, msg: m.msg ?? '', kind: 'log' });
      } else if (m.type === 'system') {
        const line: LogLine = { t: m.t, node: m.source ?? m.node, msg: m.msg ?? '', kind: 'system', source: m.source };
        v.logs.push(line);
        v.lastSystem = line;
      } else if (m.type === 'end') {
        v.logs.push({ t: m.t, id: m.id, node: m.node, msg: `run ended: ${m.msg}`, kind: 'end' });
      }
    } else if (m.kind === 'end') {
      v.phase = m.status;
      v.summary = m.summary;
      v.error = m.error;
      if (m.error) v.logs.push({ t: v.last?.t ?? 0, node: 'server', msg: m.error, kind: 'end' });
    }
  }
  return v;
}

export function formatTime(t: number): string {
  const m = Math.floor(t / 60);
  const s = t - m * 60;
  return `${m}:${s.toFixed(1).padStart(4, '0')}`;
}
