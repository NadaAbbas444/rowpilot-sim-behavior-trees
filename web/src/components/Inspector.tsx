import type { BTData } from '../graph';
import type { LogLine } from '../runState';
import { formatTime } from '../runState';
import type { NodeTypeSpec, ParamSpec, ParamValue, Status } from '../types';
import { STATUS_LABEL } from './BTNodeView';

interface Props {
  id: string;
  data: BTData;
  spec?: NodeTypeSpec;
  status?: Status;
  issues: string[];
  logs: LogLine[];
  children: string[];
  onChange: (patch: Partial<Pick<BTData, 'name' | 'params'>>) => void;
  onDelete: () => void;
}

function ParamField({
  p,
  value,
  onChange,
}: {
  p: ParamSpec;
  value: ParamValue;
  onChange: (v: ParamValue) => void;
}) {
  const id = `param-${p.name}`;
  let input;
  if (p.kind === 'bool') {
    input = <input id={id} type="checkbox" checked={Boolean(value)} onChange={(e) => onChange(e.target.checked)} />;
  } else if (p.kind === 'choice') {
    input = (
      <select id={id} value={String(value)} onChange={(e) => onChange(e.target.value)}>
        {p.choices!.map((c) => (
          <option key={c}>{c}</option>
        ))}
      </select>
    );
  } else {
    input = (
      <input
        id={id}
        type="number"
        value={Number.isFinite(value as number) ? String(value) : ''}
        step={p.kind === 'float' ? 'any' : 1}
        min={p.min}
        max={p.max}
        onChange={(e) => onChange(e.target.value === '' ? NaN : Number(e.target.value))}
      />
    );
  }
  return (
    <div className={`field ${p.kind === 'bool' ? 'field-inline' : ''}`}>
      <label htmlFor={id}>{p.name.replace(/_/g, ' ')}</label>
      {input}
      {p.help && <small>{p.help}</small>}
    </div>
  );
}

export default function Inspector({ id, data, spec, status, issues, logs, children, onChange, onDelete }: Props) {
  return (
    <div className="inspector">
      <div className={`insp-head cat-${data.category}`}>
        <span className="insp-type">{data.type}</span>
        <span className="insp-cat">{data.category}</span>
        {status && <span className={`bt-status st-${status.toLowerCase()}`}>{STATUS_LABEL[status]}</span>}
      </div>
      {spec && <p className="help">{spec.help}</p>}
      <div className="field">
        <label htmlFor="node-name">name</label>
        <input id="node-name" value={data.name} onChange={(e) => onChange({ name: e.target.value })} />
        <small>unique; mission steps remember by name that they are done</small>
      </div>
      {spec?.params.map((p) => (
        <ParamField
          key={p.name}
          p={p}
          value={data.params[p.name] ?? p.default}
          onChange={(v) => onChange({ params: { ...data.params, [p.name]: v } })}
        />
      ))}
      {spec && spec.max_children !== 0 && (
        <p className="muted">
          {children.length} {children.length === 1 ? 'child' : 'children'}, ticked top to bottom
          {spec.max_children === 1 ? ' (takes exactly one)' : ''}
        </p>
      )}
      {issues.length > 0 && (
        <ul className="issues">
          {issues.map((m) => (
            <li key={m}>{m}</li>
          ))}
        </ul>
      )}
      <button className="danger" onClick={onDelete}>
        {children.length ? `Delete with everything under it` : 'Delete node'}
      </button>
      <h4>This node's log</h4>
      {logs.length === 0 ? (
        <p className="muted">Nothing yet. Run the tree to see what this node does.</p>
      ) : (
        <ol className="log node-log">
          {logs.map((l, i) => (
            <li key={i} className={`log-${l.kind} ${l.status ? `st-${l.status.toLowerCase()}` : ''}`}>
              <span className="log-t">{formatTime(l.t)}</span>
              <span className="log-msg">{l.kind === 'status' ? `→ ${STATUS_LABEL[l.status!]}` : l.msg}</span>
            </li>
          ))}
        </ol>
      )}
      <p className="muted small">id {id}</p>
    </div>
  );
}
