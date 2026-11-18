// The tree as React Flow nodes and edges, and back. Pure functions, tested in graph.test.ts.
import type { Edge, Node } from '@xyflow/react';
import type { Category, Graph, NodeTypeSpec, ParamValue } from './types';

export interface BTData extends Record<string, unknown> {
  type: string;
  name: string;
  params: Record<string, ParamValue>;
  category: Category;
}

export type BTNode = Node<BTData, 'bt'>;

export const NODE_W = 210;
export const DX = 260; // depth step
export const DY = 84; // one leaf per row (a node with parameters and a log line is ~75 px)

export function categoryOf(specs: Map<string, NodeTypeSpec>, type: string): Category {
  return specs.get(type)?.category ?? 'action';
}

export function toFlow(graph: Graph, specs: Map<string, NodeTypeSpec>): { nodes: BTNode[]; edges: Edge[] } {
  const nodes: BTNode[] = graph.nodes.map((n) => ({
    id: n.id,
    type: 'bt',
    position: n.position ?? { x: 0, y: 0 },
    data: { type: n.type, name: n.name, params: { ...n.params }, category: categoryOf(specs, n.type) },
  }));
  const edges: Edge[] = graph.edges.map((e) => edgeFor(e.source, e.target));
  return { nodes, edges };
}

export function edgeFor(source: string, target: string): Edge {
  return { id: `${source}->${target}`, source, target };
}

export function fromFlow(nodes: BTNode[], edges: Edge[]): Graph {
  return {
    version: 1,
    nodes: nodes.map((n) => ({
      id: n.id,
      type: n.data.type,
      name: n.data.name,
      params: n.data.params,
      position: { x: Math.round(n.position.x), y: Math.round(n.position.y) },
    })),
    edges: edges.map((e) => ({ source: e.source, target: e.target })),
  };
}

/** Children of every node, in tick order: top to bottom as drawn (edge order breaks ties). */
export function childrenOf(nodes: BTNode[], edges: Edge[]): Map<string, string[]> {
  const y = new Map(nodes.map((n) => [n.id, n.position.y]));
  const kids = new Map<string, { id: string; y: number; i: number }[]>(nodes.map((n) => [n.id, []]));
  edges.forEach((e, i) => kids.get(e.source)?.push({ id: e.target, y: y.get(e.target) ?? Infinity, i }));
  return new Map(
    [...kids].map(([id, ks]) => [id, ks.sort((a, b) => a.y - b.y || a.i - b.i).map((k) => k.id)]),
  );
}

export function descendants(id: string, kids: Map<string, string[]>): string[] {
  const out: string[] = [];
  const todo = [...(kids.get(id) ?? [])];
  while (todo.length) {
    const k = todo.pop()!;
    if (out.includes(k)) continue;
    out.push(k);
    todo.push(...(kids.get(k) ?? []));
  }
  return out;
}

/**
 * Left-to-right tree layout, the same as the server's tidy_layout: depth along x, one leaf per
 * row, parents level with the middle of their children. Keeps the current tick order. Collapsed
 * subtrees take one row; their hidden nodes sit on the collapsed node's row.
 */
export function tidy(nodes: BTNode[], edges: Edge[], collapsed: Set<string> = new Set()): BTNode[] {
  const kids = childrenOf(nodes, edges);
  const hasParent = new Set(edges.map((e) => e.target));
  const pos = new Map<string, { x: number; y: number }>();
  let row = 0;
  const hide = (id: string, x: number, y: number) => {
    for (const k of kids.get(id) ?? []) {
      if (pos.has(k)) continue;
      pos.set(k, { x: x + DX, y });
      hide(k, x + DX, y);
    }
  };
  const place = (id: string, depth: number): number => {
    if (pos.has(id)) return pos.get(id)!.y; // a cycle: place each node once
    pos.set(id, { x: depth * DX, y: 0 });
    const ks = kids.get(id) ?? [];
    let y: number;
    if (ks.length && !collapsed.has(id)) {
      const ys = ks.map((k) => place(k, depth + 1));
      y = (ys[0] + ys[ys.length - 1]) / 2;
    } else {
      y = row++ * DY;
      hide(id, depth * DX, y);
    }
    pos.set(id, { x: depth * DX, y });
    return y;
  };
  const roots = nodes.filter((n) => !hasParent.has(n.id)).sort((a, b) => a.position.y - b.position.y);
  roots.forEach((r, i) => {
    if (i > 0) row += 1; // a gap before each loose subtree
    place(r.id, 0);
  });
  for (const n of nodes) if (!pos.has(n.id)) place(n.id, 0); // only in cycles
  return nodes.map((n) => ({ ...n, position: pos.get(n.id)! }));
}

/** Why `source` -> `target` cannot be connected, or null if it can. */
export function connectionProblem(
  nodes: BTNode[],
  edges: Edge[],
  specs: Map<string, NodeTypeSpec>,
  source: string,
  target: string,
): string | null {
  if (source === target) return 'a node cannot be its own child';
  const s = nodes.find((n) => n.id === source);
  if (!s) return 'unknown node';
  const spec = specs.get(s.data.type);
  const max = spec ? spec.max_children : 0; // null: any number
  const nKids = edges.filter((e) => e.source === source).length;
  if (max === 0) return `${s.data.type} cannot have children`;
  if (max !== null && nKids >= max) return `${s.data.type} takes only ${max} child`;
  if (edges.some((e) => e.target === target)) return 'that node already has a parent';
  if (descendants(target, childrenOf(nodes, edges)).includes(source)) return 'that would make a cycle';
  return null;
}

export function uniqueName(base: string, taken: Iterable<string>): string {
  const names = new Set(taken);
  if (!names.has(base)) return base;
  for (let i = 2; ; i++) if (!names.has(`${base} ${i}`)) return `${base} ${i}`;
}

export function nextId(nodes: { id: string }[]): string {
  const max = nodes.reduce((m, n) => {
    const k = /^n(\d+)$/.exec(n.id);
    return k ? Math.max(m, Number(k[1])) : m;
  }, 0);
  return `n${max + 1}`;
}

export function defaultParams(spec: NodeTypeSpec): Record<string, ParamValue> {
  return Object.fromEntries(spec.params.map((p) => [p.name, p.default]));
}

/** A short line under the node's name: its parameters. */
export function paramSummary(params: Record<string, ParamValue>, spec?: NodeTypeSpec): string {
  if (!spec) return '';
  return spec.params
    .map((p) => {
      const v = params[p.name] ?? p.default;
      if (p.kind === 'bool') return v ? p.name.replace(/_/g, ' ') : '';
      if (p.kind === 'lane') return `lane ${v}`;
      if (p.name === 'from_lane') return v ? `from lane ${v}` : '';
      if (p.kind === 'choice' && v === 'auto') return '';
      return p.kind === 'choice' ? String(v) : `${p.name.replace(/_/g, ' ')} ${v}`;
    })
    .filter(Boolean)
    .join(' · ');
}

const LANE_NAME = /^Lane (\d+)$/;

/** The lane numbers of the lane subtrees ("Lane 3") in the tree. */
export function laneNumbers(nodes: BTNode[]): number[] {
  return nodes.map((n) => LANE_NAME.exec(n.data.name)).filter(Boolean).map((m) => Number(m![1]));
}

/**
 * Where a new lane goes, as (parent, y): right after the highest-numbered lane in its parent; with no
 * lanes yet, into the Mission sequence before ReturnHome (or last); else into the selected node if it
 * can take children. null if there is nowhere sensible.
 */
export function laneSlot(
  nodes: BTNode[],
  edges: Edge[],
  specs: Map<string, NodeTypeSpec>,
  selected: string | null,
): { parent: string; y: number } | null {
  const byId = new Map(nodes.map((n) => [n.id, n]));
  const parentOf = new Map(edges.map((e) => [e.target, e.source]));
  const kids = childrenOf(nodes, edges);
  const below = (id: string) => byId.get(id)!.position.y + 0.5; // sorts right after id, before its next sibling
  const lanes = nodes
    .filter((n) => LANE_NAME.test(n.data.name) && parentOf.has(n.id))
    .sort((a, b) => Number(LANE_NAME.exec(b.data.name)![1]) - Number(LANE_NAME.exec(a.data.name)![1]));
  if (lanes.length) return { parent: parentOf.get(lanes[0].id)!, y: below(lanes[0].id) };
  const lastChild = (pid: string) => {
    const ks = kids.get(pid) ?? [];
    const p = byId.get(pid)!;
    return ks.length ? below(ks[ks.length - 1]) : p.position.y;
  };
  const takesKids = (n: BTNode) => {
    const spec = specs.get(n.data.type);
    return spec !== undefined && spec.max_children !== 0; // null: any number
  };
  const mission = nodes.find((n) => n.data.name === 'Mission' && takesKids(n));
  if (mission) {
    const home = (kids.get(mission.id) ?? []).find((k) => /ReturnHome/.test(byId.get(k)!.data.type + byId.get(k)!.data.name));
    return { parent: mission.id, y: home ? byId.get(home)!.position.y - 0.5 : lastChild(mission.id) };
  }
  if (selected && byId.has(selected) && specs.get(byId.get(selected)!.data.type)?.max_children === null) {
    return { parent: selected, y: lastChild(selected) };
  }
  return null;
}

/** Add a subtree (a graph with a root) under `parent` at height y, with fresh ids. */
export function insertSubtree(
  nodes: BTNode[],
  edges: Edge[],
  sub: Graph & { root: string },
  specs: Map<string, NodeTypeSpec>,
  parent: string,
  y: number,
): { nodes: BTNode[]; edges: Edge[]; root: string } {
  let next = Number(nextId(nodes).slice(1));
  const ids = new Map(sub.nodes.map((n) => [n.id, `n${next++}`]));
  const p = nodes.find((n) => n.id === parent)!;
  const r = sub.nodes.find((n) => n.id === sub.root)!.position ?? { x: 0, y: 0 };
  const flow = toFlow(sub, specs);
  const added = flow.nodes.map((n) => ({
    ...n,
    id: ids.get(n.id)!,
    position: { x: p.position.x + DX + (n.position.x - r.x), y: y + (n.position.y - r.y) },
  }));
  const addedEdges = sub.edges.map((e) => edgeFor(ids.get(e.source)!, ids.get(e.target)!));
  const root = ids.get(sub.root)!;
  return { nodes: [...nodes, ...added], edges: [...edges, ...addedEdges, edgeFor(parent, root)], root };
}

/** The lowest lane number (1..total) the tree does not sweep yet, or null if it has them all. */
export function nextLane(nodes: BTNode[], total: number): number | null {
  const have = new Set(laneNumbers(nodes));
  for (let k = 1; k <= total; k++) if (!have.has(k)) return k;
  return null;
}

/** The tree without `id` and everything under it. */
export function removeSubtree(nodes: BTNode[], edges: Edge[], id: string): { nodes: BTNode[]; edges: Edge[] } {
  const gone = new Set([id, ...descendants(id, childrenOf(nodes, edges))]);
  return {
    nodes: nodes.filter((n) => !gone.has(n.id)),
    edges: edges.filter((e) => !gone.has(e.source) && !gone.has(e.target)),
  };
}
