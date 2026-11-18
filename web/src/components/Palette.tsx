import { useState } from 'react';
import type { NodeTypeSpec } from '../types';

export const DRAG_TYPE = 'application/x-rowpilot-node';

const GROUPS: [string, string][] = [
  ['composite', 'Composites'],
  ['decorator', 'Decorators'],
  ['condition', 'Conditions'],
  ['flight', 'Actions: flight'],
  ['lane', 'Actions: lanes'],
];

export default function Palette({ specs, onAdd }: { specs: NodeTypeSpec[]; onAdd: (type: string) => void }) {
  const [filter, setFilter] = useState('');
  const f = filter.trim().toLowerCase();
  const shown = specs.filter((s) => !f || s.type.toLowerCase().includes(f) || s.help.toLowerCase().includes(f));
  return (
    <aside className="palette">
      <input
        className="palette-filter"
        placeholder="Find a node…"
        value={filter}
        onChange={(e) => setFilter(e.target.value)}
        aria-label="Find a node type"
      />
      <p className="hint">Drag onto the canvas, or double-click to add next to the selected node.</p>
      {GROUPS.map(([g, title]) => {
        const items = shown.filter((s) => s.group === g);
        if (!items.length) return null;
        return (
          <section key={g}>
            <h3>{title}</h3>
            {items.map((s) => (
              <div
                key={s.type}
                className={`palette-item cat-${s.category}`}
                draggable
                onDragStart={(e) => {
                  e.dataTransfer.setData(DRAG_TYPE, s.type);
                  e.dataTransfer.effectAllowed = 'move';
                }}
                onDoubleClick={() => onAdd(s.type)}
                title={s.help}
              >
                <span className="swatch" />
                <span>{s.type}</span>
              </div>
            ))}
          </section>
        );
      })}
    </aside>
  );
}
