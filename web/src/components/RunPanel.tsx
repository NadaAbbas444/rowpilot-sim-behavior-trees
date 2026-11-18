import { useState } from 'react';
import type { RunView } from '../runState';
import type { Settings } from '../types';

interface Props {
  settings: Settings;
  onChange: (s: Settings) => void;
  run: RunView;
}

type NumKey = { [K in keyof Settings]: Settings[K] extends number ? K : never }[keyof Settings];

const FIELDS: [NumKey, string, string, number][] = [
  ['lanes', 'lanes', 'number of lanes in the greenhouse', 1],
  ['row_length', 'row length (m)', '', 0.1],
  ['path_width', 'lane width (m)', 'free width between the crop faces', 0.1],
  ['battery', 'start battery (%)', '45 shows the low-battery failsafe', 1],
  ['wind', 'gusts (m/s)', '', 0.1],
  ['altitude', 'altitude (m)', '', 0.1],
  ['sweep_speed', 'sweep speed (m/s)', '', 0.1],
  ['seed', 'random seed', 'wind, odometry drift and depth noise', 1],
  ['max_time_s', 'time limit (s)', 'simulated time', 10],
];

function parseDropouts(s: string): [number, number][] | null {
  if (!s.trim()) return [];
  const out: [number, number][] = [];
  for (const part of s.split(',')) {
    const m = /^\s*(\d+(?:\.\d+)?)\s*:\s*(\d+(?:\.\d+)?)\s*$/.exec(part);
    if (!m) return null;
    out.push([Number(m[1]), Number(m[2])]);
  }
  return out;
}

export default function RunPanel({ settings, onChange, run }: Props) {
  const [dropText, setDropText] = useState(settings.vo_dropouts.map(([a, b]) => `${a}:${b}`).join(', '));
  const dropOk = parseDropouts(dropText) !== null;
  const s = run.summary;
  return (
    <div className="runpanel">
      {s && (
        <div className={`summary ${s.outcome === 'completed' ? 'ok' : 'bad'}`}>
          <h4>{s.outcome}</h4>
          <dl>
            <dt>lanes swept</dt>
            <dd>
              {s.lanes_completed.length} / {settings.lanes}
            </dd>
            <dt>flight time</dt>
            <dd>{s.duration_s.toFixed(0)} s</dd>
            <dt>battery left</dt>
            <dd>{s.battery_end_pct.toFixed(0)} %</dd>
            <dt>worst distance from lane centre</dt>
            <dd>{s.max_lane_error_m.toFixed(2)} m</dd>
          </dl>
        </div>
      )}
      {run.error && <p className="issues">{run.error}</p>}
      <h4>Greenhouse and simulation</h4>
      <div className="field field-inline">
        <label htmlFor="open-ends">lanes open at both ends</label>
        <input
          id="open-ends"
          type="checkbox"
          checked={settings.open_ends}
          onChange={(e) => onChange({ ...settings, open_ends: e.target.checked })}
        />
        <small>off: dead-end lanes, as in the real greenhouse (in, turn round, back out)</small>
      </div>
      {FIELDS.map(([key, label, help, step]) => (
        <div className="field" key={key}>
          <label htmlFor={`set-${key}`}>{label}</label>
          <input
            id={`set-${key}`}
            type="number"
            step={step}
            value={Math.round(settings[key] * 1000) / 1000}
            onChange={(e) => onChange({ ...settings, [key]: Number(e.target.value) })}
          />
          {help && <small>{help}</small>}
        </div>
      ))}
      <div className="field">
        <label htmlFor="set-dropouts">visual odometry dropouts</label>
        <input
          id="set-dropouts"
          className={dropOk ? '' : 'invalid'}
          placeholder="start:seconds, e.g. 60:1.5, 200:10"
          value={dropText}
          onChange={(e) => {
            setDropText(e.target.value);
            const d = parseDropouts(e.target.value);
            if (d) onChange({ ...settings, vo_dropouts: d });
          }}
        />
        <small>a short one is waited out (hover); one longer than 3 s lands the drone</small>
      </div>
    </div>
  );
}
