import { describe, expect, it } from 'vitest';
import {
  childrenOf,
  connectionProblem,
  fromFlow,
  insertSubtree,
  laneNumbers,
  laneSlot,
  nextId,
  nextLane,
  removeSubtree,
  paramSummary,
  tidy,
  toFlow,
  uniqueName,
  type BTNode,
} from './graph';
import type { Graph, NodeTypeSpec } from './types';

const spec = (type: string, category: NodeTypeSpec['category'], max: number | null, params: NodeTypeSpec['params'] = []): NodeTypeSpec => ({
  type,
  category,
  group: category,
  help: '',
  params,
  min_children: max === 0 ? 0 : 1,
  max_children: max,
  robots: ['drone', 'trolley'],
});

const SPECS = new Map(
  [
    spec('Sequence', 'composite', null),
    spec('Inverter', 'decorator', 1),
    spec('Land', 'action', 0),
    spec('SweepLane', 'action', 0, [
      { name: 'lane', kind: 'lane', default: 1, help: '' },
      { name: 'direction', kind: 'choice', default: 'auto', help: '', choices: ['auto', '+x', '-x'] },
      { name: 'stop_at_end', kind: 'bool', default: false, help: '' },
    ]),
  ].map((s) => [s.type, s]),
);

const graph: Graph = {
  version: 1,
  nodes: [
    { id: 'r', type: 'Sequence', name: 'Root', params: {}, position: { x: 0, y: 50 } },
    { id: 'a', type: 'Land', name: 'A', params: {}, position: { x: 260, y: 200 } },
    { id: 'b', type: 'Land', name: 'B', params: {}, position: { x: 260, y: 0 } },
    { id: 'i', type: 'Inverter', name: 'Not', params: {}, position: { x: 260, y: 100 } },
    { id: 'c', type: 'Land', name: 'C', params: {}, position: { x: 520, y: 100 } },
  ],
  edges: [
    { source: 'r', target: 'a' },
    { source: 'r', target: 'b' },
    { source: 'r', target: 'i' },
    { source: 'i', target: 'c' },
  ],
};

const flow = () => toFlow(graph, SPECS);

describe('graph <-> React Flow', () => {
  it('round-trips', () => {
    const { nodes, edges } = flow();
    expect(fromFlow(nodes, edges)).toEqual(graph);
    expect(nodes.find((n) => n.id === 'i')!.data.category).toBe('decorator');
  });

  it('orders children top to bottom, as the server ticks them', () => {
    const { nodes, edges } = flow();
    expect(childrenOf(nodes, edges).get('r')).toEqual(['b', 'i', 'a']);
  });
});

describe('tidy', () => {
  it('keeps the order and puts parents level with their children', () => {
    const { nodes, edges } = flow();
    const pos = new Map(tidy(nodes, edges).map((n) => [n.id, n.position]));
    expect(pos.get('b')!.y).toBeLessThan(pos.get('i')!.y);
    expect(pos.get('i')!.y).toBeLessThan(pos.get('a')!.y);
    expect(pos.get('i')!.y).toBe(pos.get('c')!.y);
    expect(pos.get('r')!.y).toBe((pos.get('b')!.y + pos.get('a')!.y) / 2);
    expect(pos.get('c')!.x).toBeGreaterThan(pos.get('i')!.x);
  });

  it('gives a collapsed subtree one row', () => {
    const { nodes, edges } = flow();
    const open = tidy(nodes, edges);
    const shut = tidy(nodes, edges, new Set(['r']));
    const rows = (ns: BTNode[]) => new Set(ns.map((n) => n.position.y)).size;
    expect(rows(open)).toBe(3);
    expect(rows(shut)).toBe(1);
  });
});

describe('connections', () => {
  const { nodes, edges } = flow();
  const extra: BTNode = { ...nodes[1], id: 'x', data: { ...nodes[1].data, name: 'X' } };
  const all = [...nodes, extra];
  it('allow a composite to take another child', () => {
    expect(connectionProblem(all, edges, SPECS, 'r', 'x')).toBeNull();
  });
  it('refuse children for leaves, a second child for decorators, two parents and cycles', () => {
    expect(connectionProblem(all, edges, SPECS, 'a', 'x')).toMatch(/cannot have children/);
    expect(connectionProblem(all, edges, SPECS, 'i', 'x')).toMatch(/only 1 child/);
    expect(connectionProblem(all, edges, SPECS, 'r', 'c')).toMatch(/already has a parent/);
    const seq2 = { ...nodes[0], id: 's2', data: { ...nodes[0].data, name: 'S2' } };
    expect(connectionProblem([...all, seq2], [...edges, { id: 'e', source: 's2', target: 'r' }], SPECS, 'r', 's2')).toMatch(
      /cycle/,
    );
    expect(connectionProblem(all, edges, SPECS, 'r', 'r')).toMatch(/own child/);
  });
});

describe('helpers', () => {
  it('make unique names and ids', () => {
    expect(uniqueName('Land', ['A'])).toBe('Land');
    expect(uniqueName('Land', ['Land', 'Land 2'])).toBe('Land 3');
    expect(nextId([{ id: 'n3' }, { id: 'x' }, { id: 'n12' }])).toBe('n13');
  });
  it('summarise parameters', () => {
    const s = SPECS.get('SweepLane');
    expect(paramSummary({ lane: 2, direction: '+x', stop_at_end: true }, s)).toBe('lane 2 · +x · stop at end');
    expect(paramSummary({ lane: 1, direction: 'auto', stop_at_end: false }, s)).toBe('lane 1');
  });
});

describe('lanes', () => {
  // Mission -> Preflight, Lane 1 (-> 2 steps), Lane 2 (-> 2 steps), ReturnHome
  const lanesGraph: Graph = {
    version: 1,
    nodes: [
      { id: 'm', type: 'Sequence', name: 'Mission', params: {}, position: { x: 0, y: 200 } },
      { id: 'p', type: 'Land', name: 'Preflight', params: {}, position: { x: 260, y: 0 } },
      { id: 'l1', type: 'Sequence', name: 'Lane 1', params: {}, position: { x: 260, y: 100 } },
      { id: 'a1', type: 'Land', name: 'Lane 1: A', params: {}, position: { x: 520, y: 80 } },
      { id: 'b1', type: 'Land', name: 'Lane 1: B', params: {}, position: { x: 520, y: 120 } },
      { id: 'l2', type: 'Sequence', name: 'Lane 2', params: {}, position: { x: 260, y: 200 } },
      { id: 'a2', type: 'Land', name: 'Lane 2: A', params: {}, position: { x: 520, y: 180 } },
      { id: 'b2', type: 'Land', name: 'Lane 2: B', params: {}, position: { x: 520, y: 220 } },
      { id: 'h', type: 'Land', name: 'ReturnHome', params: {}, position: { x: 260, y: 300 } },
    ],
    edges: [
      ['m', 'p'], ['m', 'l1'], ['l1', 'a1'], ['l1', 'b1'], ['m', 'l2'], ['l2', 'a2'], ['l2', 'b2'], ['m', 'h'],
    ].map(([source, target]) => ({ source, target })),
  };
  const lane3 = {
    version: 1,
    root: 'n3',
    nodes: [
      { id: 'n1', type: 'Land', name: 'Lane 3: A', params: {}, position: { x: 260, y: 0 } },
      { id: 'n2', type: 'Land', name: 'Lane 3: B', params: {}, position: { x: 260, y: 64 } },
      { id: 'n3', type: 'Sequence', name: 'Lane 3', params: {}, position: { x: 0, y: 32 } },
    ],
    edges: [{ source: 'n3', target: 'n1' }, { source: 'n3', target: 'n2' }],
  };

  it('finds the lanes and the next one the tree does not sweep', () => {
    const { nodes } = toFlow(lanesGraph, SPECS);
    expect(laneNumbers(nodes).sort()).toEqual([1, 2]);
    expect(nextLane(nodes, 6)).toBe(3);
    expect(nextLane(nodes, 2)).toBeNull();
    expect(nextLane(nodes.filter((n) => n.id !== 'l1'), 6)).toBe(1); // a gap is filled first
  });

  it('adds a lane right after the last one, before ReturnHome, with fresh ids', () => {
    const { nodes, edges } = toFlow(lanesGraph, SPECS);
    const slot = laneSlot(nodes, edges, SPECS, null)!;
    expect(slot.parent).toBe('m');
    const r = insertSubtree(nodes, edges, lane3, SPECS, slot.parent, slot.y);
    const order = childrenOf(r.nodes, r.edges).get('m')!.map((id) => r.nodes.find((n) => n.id === id)!.data.name);
    expect(order).toEqual(['Preflight', 'Lane 1', 'Lane 2', 'Lane 3', 'ReturnHome']);
    const lane = childrenOf(r.nodes, r.edges).get(r.root)!.map((id) => r.nodes.find((n) => n.id === id)!.data.name);
    expect(lane).toEqual(['Lane 3: A', 'Lane 3: B']);
    expect(new Set(r.nodes.map((n) => n.id)).size).toBe(r.nodes.length);
  });

  it('goes into the Mission sequence when there are no lanes yet', () => {
    const g = { ...lanesGraph, nodes: lanesGraph.nodes.filter((n) => !n.name.startsWith('Lane')) };
    g.edges = lanesGraph.edges.filter((e) => g.nodes.some((n) => n.id === e.target));
    const { nodes, edges } = toFlow(g, SPECS);
    const r = insertSubtree(nodes, edges, lane3, SPECS, ...Object.values(laneSlot(nodes, edges, SPECS, null)!) as [string, number]);
    const order = childrenOf(r.nodes, r.edges).get('m')!.map((id) => r.nodes.find((n) => n.id === id)!.data.name);
    expect(order).toEqual(['Preflight', 'Lane 3', 'ReturnHome']);
  });

  it('removes a lane with all its steps', () => {
    const { nodes, edges } = toFlow(lanesGraph, SPECS);
    const r = removeSubtree(nodes, edges, 'l2');
    expect(r.nodes.map((n) => n.id)).toEqual(['m', 'p', 'l1', 'a1', 'b1', 'h']);
    expect(r.edges.some((e) => ['l2', 'a2', 'b2'].includes(e.target))).toBe(false);
  });
});
