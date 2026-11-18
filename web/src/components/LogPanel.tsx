import { useEffect, useMemo, useRef, useState } from 'react';
import type { LogLine } from '../runState';
import { formatTime } from '../runState';
import { STATUS_LABEL } from './BTNodeView';

export const SOURCE_LABEL: Record<string, string> = {
  ardupilot: 'ArduPilot',
  mavlink: 'MAVLink',
  ekf: 'EKF3',
  vio: 'VIO',
  sim: 'simulator',
  setup: 'setup',
  editor: 'editor',
};

const SOURCE_HELP: Record<string, string> = {
  ardupilot: "ArduCopter's own messages (STATUSTEXT): pre-arm checks, arming, EKF3, modes",
  mavlink: 'what rowpilot asked ArduCopter over MAVLink: mode, arm, take-off, land',
  ekf: "ArduPilot's EKF3 position estimate, fed by the visual odometry",
  vio: 'OpenVINS visual-inertial odometry on the D435i infrared pair + IMU',
  sim: 'the simulator',
  setup: 'starting Gazebo, ArduPilot SITL and OpenVINS',
  editor: 'the editor',
};

export default function LogPanel({ logs, onPick }: { logs: LogLine[]; onPick: (id: string) => void }) {
  const [showStatus, setShowStatus] = useState(false);
  const [showTree, setShowTree] = useState(true);
  const [showSystem, setShowSystem] = useState(true);
  const [filter, setFilter] = useState('');
  const [follow, setFollow] = useState(true);
  const end = useRef<HTMLLIElement>(null);
  const f = filter.trim().toLowerCase();
  const shown = useMemo(
    () =>
      logs.filter(
        (l) =>
          (showStatus || l.kind !== 'status') &&
          (showSystem || l.kind !== 'system') &&
          (showTree || l.kind === 'system' || l.kind === 'end') &&
          (!f || l.node.toLowerCase().includes(f) || l.msg.toLowerCase().includes(f)),
      ),
    [logs, showStatus, showTree, showSystem, f],
  );
  useEffect(() => {
    if (follow) end.current?.scrollIntoView({ block: 'end' });
  }, [shown.length, follow]);
  return (
    <div className="logpanel">
      <div className="log-tools">
        <input placeholder="Filter…" value={filter} onChange={(e) => setFilter(e.target.value)} aria-label="Filter the log" />
        <label title="the behaviour tree's nodes">
          <input type="checkbox" checked={showTree} onChange={(e) => setShowTree(e.target.checked)} /> tree
        </label>
        <label title="ArduPilot, MAVLink, EKF3, OpenVINS and the simulation's start-up">
          <input type="checkbox" checked={showSystem} onChange={(e) => setShowSystem(e.target.checked)} /> flight stack
        </label>
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
              {l.kind === 'system' ? (
                <span className={`log-src src-${l.source}`} title={SOURCE_HELP[l.source ?? ''] ?? ''}>
                  {SOURCE_LABEL[l.source ?? ''] ?? l.source}
                </span>
              ) : (
                <button className="log-node" disabled={!l.id} onClick={() => l.id && onPick(l.id)} title="show this node">
                  {l.node}
                </button>
              )}
              <span className="log-msg">{l.kind === 'status' ? `→ ${STATUS_LABEL[l.status!]}` : l.msg}</span>
            </li>
          ))}
          <li ref={end} className="log-end" aria-hidden />
        </ol>
      )}
    </div>
  );
}
