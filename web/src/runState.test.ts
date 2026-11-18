import { describe, expect, it } from 'vitest';
import { applyMessages, formatTime, startRun } from './runState';
import type { RunMessage } from './types';

const tick = (t: number, x: number, node = 'Takeoff'): RunMessage => ({
  kind: 'tick',
  t,
  x,
  y: 0,
  z: 1,
  yaw: 0,
  est_x: x + 0.01,
  est_y: 0,
  battery: 99,
  vo_ok: true,
  node,
});

describe('applyMessages', () => {
  it('tracks node status, logs per node, the path and the end', () => {
    const msgs: RunMessage[] = [
      { kind: 'event', t: 0, type: 'status', node: 'Takeoff', id: 'n2', status: 'RUNNING' },
      tick(0.1, 0),
      tick(0.2, 0.01), // closer than 5 cm to the last point: not stored
      tick(0.3, 0.2),
      { kind: 'event', t: 0.3, type: 'log', node: 'Takeoff', id: 'n2', msg: 'done in 0.3s' },
      { kind: 'event', t: 0.3, type: 'status', node: 'Takeoff', id: 'n2', status: 'SUCCESS' },
      { kind: 'event', t: 0.3, type: 'end', node: 'Root', id: 'n1', msg: 'completed' },
      {
        kind: 'end',
        status: 'finished',
        error: null,
        summary: { outcome: 'completed', duration_s: 0.3, battery_end_pct: 99, lanes_completed: [], lane_rms_error_m: {}, max_lane_error_m: 0 },
      },
    ];
    const before = startRun('abc');
    const v = applyMessages(before, msgs);
    expect(before.logs).toEqual([]); // not changed
    expect(v.status).toEqual({ n2: 'SUCCESS' });
    expect(v.lastLog).toEqual({ n2: 'done in 0.3s' });
    expect(v.truePath).toHaveLength(2);
    expect(v.last!.t).toBe(0.3);
    expect(v.logs.map((l) => l.kind)).toEqual(['status', 'log', 'status', 'end']);
    expect(v.phase).toBe('finished');
    expect(v.summary!.outcome).toBe('completed');
    expect(v.received).toBe(msgs.length);
  });

  it('reports a server error', () => {
    const v = applyMessages(startRun('x'), [{ kind: 'end', status: 'error', summary: null, error: 'ZeroDivisionError: x' }]);
    expect(v.phase).toBe('error');
    expect(v.logs.at(-1)!.msg).toMatch(/ZeroDivision/);
  });
});

it('formats simulated time', () => {
  expect(formatTime(0)).toBe('0:00.0');
  expect(formatTime(371.25)).toBe('6:11.3');
});
