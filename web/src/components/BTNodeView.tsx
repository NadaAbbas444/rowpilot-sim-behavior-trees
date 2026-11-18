import { Handle, Position, type NodeProps } from '@xyflow/react';
import { createContext, memo, useContext } from 'react';
import { paramSummary, type BTNode } from '../graph';
import type { NodeTypeSpec, Status } from '../types';

export interface CanvasInfo {
  specs: Map<string, NodeTypeSpec>;
  status: Record<string, Status>;
  lastLog: Record<string, string>;
  issues: Record<string, string[]>;
  collapsed: Set<string>;
  hiddenCount: Record<string, number>;
  toggleCollapse: (id: string) => void;
}

export const CanvasContext = createContext<CanvasInfo | null>(null);

const ICON: Record<string, string> = {
  Sequence: '→',
  Selector: '?',
  Parallel: '⇉',
  Inverter: '¬',
  SuccessIsFailure: '✓→✗',
  FailureIsSuccess: '✗→✓',
  Timeout: '⏱',
};

export const STATUS_LABEL: Record<Status, string> = {
  RUNNING: 'running',
  SUCCESS: 'success',
  FAILURE: 'failure',
  INVALID: 'idle',
};

function BTNodeView({ id, data, selected }: NodeProps<BTNode>) {
  const c = useContext(CanvasContext)!;
  const spec = c.specs.get(data.type);
  const status = c.status[id];
  const issues = c.issues[id];
  const leaf = spec ? spec.max_children === 0 : true;
  const collapsed = c.collapsed.has(id);
  const summary = paramSummary(data.params, spec);
  const log = c.lastLog[id];
  const cls = [
    'bt-node',
    `cat-${data.category}`,
    status ? `st-${status.toLowerCase()}` : '',
    issues ? 'has-issue' : '',
    selected ? 'selected' : '',
  ].join(' ');
  return (
    <div className={cls} title={issues?.join('\n') ?? spec?.help}>
      <Handle type="target" position={Position.Left} />
      <div className="bt-head">
        <span className="bt-icon">{ICON[data.type] ?? (data.category === 'condition' ? '◇' : '▶')}</span>
        <span className="bt-type">{data.type}</span>
        {status && status !== 'INVALID' && <span className={`bt-status st-${status.toLowerCase()}`}>{STATUS_LABEL[status]}</span>}
      </div>
      <div className="bt-name">{data.name}</div>
      {summary && <div className="bt-params">{summary}</div>}
      {log && <div className="bt-log">{log}</div>}
      {issues && <div className="bt-issue">⚠ {issues[0]}</div>}
      {!leaf && (
        <>
          <button
            className="bt-collapse nodrag"
            onClick={(e) => {
              e.stopPropagation();
              c.toggleCollapse(id);
            }}
            title={collapsed ? 'show children' : 'hide children'}
          >
            {collapsed ? `+${c.hiddenCount[id] ?? ''}` : '−'}
          </button>
          <Handle type="source" position={Position.Right} />
        </>
      )}
    </div>
  );
}

export default memo(BTNodeView);
