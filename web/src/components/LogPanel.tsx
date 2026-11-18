import { useEffect, useMemo, useRef, useState } from 'react';
import type { LogLine } from '../runState';
import { formatTime } from '../runState';
import { STATUS_LABEL } from './BTNodeView';

export default function LogPanel({ logs, onPick }: { logs: LogLine[]; onPick: (id: string) => void }) {
  const [showStatus, setShowStatus] = useState(false);
  const [filter, setFilter] = useState('');
  const [follow, setFollow] = useState(true);
  const end = useRef<HTMLLIElement>(null);
  const f = filter.trim().toLowerCase();
  const shown = useMemo(
    () =>
      logs.filter(
        (l) =>
          (showStatus || l.kind !== 'status') &&
          (!f || l.node.toLowerCase().includes(f) || l.msg.toLowerCase().includes(f)),
      ),
    [logs, showStatus, f],
  );
  useEffect(() => {
    if (follow) end.current?.scrollIntoView({ block: 'end' });
  }, [shown.length, follow]);
  return (
    <div className="logpanel">
      <div className="log-tools">
        <input placeholder="Filter…" value={filter} onChange={(e) => setFilter(e.target.value)} aria-label="Filter the log" />
        <label>
          <input type="checkbox" checked={showStatus} onChange={(e) => setShowStatus(e.target.checked)} /> status changes
        </label>
        <label>
          <input type="checkbox" checked={follow} onChange={(e) => setFollow(e.target.checked)} /> follow
        </label>
      </div>
      {shown.length === 0 ? (
        <p className="muted">No log lines yet.</p>
      ) : (
        <ol className="log">
          {shown.map((l, i) => (
            <li key={i} className={`log-${l.kind} ${l.status ? `st-${l.status.toLowerCase()}` : ''}`}>
              <span className="log-t">{formatTime(l.t)}</span>
              <button className="log-node" disabled={!l.id} onClick={() => l.id && onPick(l.id)} title="show this node">
                {l.node}
              </button>
              <span className="log-msg">{l.kind === 'status' ? `→ ${STATUS_LABEL[l.status!]}` : l.msg}</span>
            </li>
          ))}
          <li ref={end} className="log-end" aria-hidden />
        </ol>
      )}
    </div>
  );
}
