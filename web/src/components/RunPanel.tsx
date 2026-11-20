import { useState } from 'react';
import type { RunView } from '../runState';
import type { Settings, Simulators } from '../types';

interface Props {
  settings: Settings;
  onChange: (s: Settings) => void;
  run: RunView;
  simulators: Simulators | null;
}

type NumKey = { [K in keyof Settings]: Settings[K] extends number ? K : never }[keyof Settings];

type Field = [NumKey, string, string, number];
const GREENHOUSE: Field[] = [
  ['lanes', 'lanes', 'how many lanes the greenhouse has (6 in the real one); the tree chooses which to sweep: Lanes − + in the toolbar', 1],
  ['row_length', 'row length (m)', '', 0.1],
  ['path_width', 'lane width (m)', 'free width between the crop faces', 0.1],
];
const MISSION: Field[] = [
  ['altitude', 'altitude (m)', '', 0.1],
  ['sweep_speed', 'sweep speed (m/s)', '', 0.1],
];
const PYTHON_ONLY: Field[] = [
  ['battery', 'start battery (%)', '45 shows the low-battery failsafe', 1],
  ['wind', 'gusts (m/s)', '', 0.1],
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

export default function RunPanel({ settings, onChange, run, simulators }: Props) {
  const [dropText, setDropText] = useState(settings.vo_dropouts.map(([a, b]) => `${a}:${b}`).join(', '));
  const dropOk = parseDropouts(dropText) !== null;
  const s = run.summary;
  const gz = simulators?.gazebo;
  const isaac = simulators?.isaac;
  const gazebo = settings.simulator === 'gazebo' || settings.simulator === 'isaac'; // the full chain
  const onIsaac = settings.simulator === 'isaac';
  const running = run.phase === 'running';
  const numField = ([key, label, help, step]: Field) => (
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
  );
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
      <h4>Simulator</h4>
      <div className="sim-choice" role="radiogroup" aria-label="Simulator">
        <label className={settings.simulator === 'python' ? 'on' : ''}>
          <input
            type="radio"
            name="simulator"
            checked={settings.simulator === 'python'}
            disabled={running}
            onChange={() => onChange({ ...settings, simulator: 'python' })}
          />
          <b>Python simulator</b>
          <small>
            Fast, any speed. The tree, lane perception and control against a modelled drone; no Gazebo,
            ArduPilot or OpenVINS.
          </small>
        </label>
        <label className={`${settings.simulator === 'gazebo' ? 'on' : ''} ${gz?.available === false ? 'off' : ''}`}>
          <input
            type="radio"
            name="simulator"
            checked={settings.simulator === 'gazebo'}
            disabled={running || !gz?.available}
            onChange={() => onChange({ ...settings, simulator: 'gazebo' })}
          />
          <b>Gazebo + ArduPilot + OpenVINS</b>
          <small>
            The full chain, as on the drone: the Gazebo window with the drone and its cameras, ArduCopter SITL arming
            and flying, OpenVINS on the D435i images. Real time; about a minute to start.
          </small>
          {gz && !gz.available && (
            <small className="warn-text">
              Not set up on this computer: missing {gz.missing.join('; ')}. See docs/setup.md, level 2.
            </small>
          )}
        </label>
        <label className={`${onIsaac ? 'on' : ''} ${isaac?.available === false ? 'off' : ''}`}>
          <input
            type="radio"
            name="simulator"
            checked={onIsaac}
            disabled={running || !isaac?.available}
            onChange={() => onChange({ ...settings, simulator: 'isaac', vehicle: 'firefly' })}
          />
          <b>Isaac Sim + ArduPilot + OpenVINS</b>
          <small>
            The same full chain with NVIDIA Isaac Sim in place of Gazebo: the same greenhouse, ArduCopter SITL and
            OpenVINS. FireFly only; needs an RTX GPU. The first start compiles shaders (several minutes).
          </small>
          {isaac && !isaac.available && (
            <small className="warn-text">
              Not set up on this computer: missing {isaac.missing.join('; ')}. See docs/setup.md, Isaac Sim.
            </small>
          )}
        </label>
      </div>
      {gazebo && (
        <>
          <div className="field">
            <label htmlFor="set-vehicle">drone</label>
            <select
              id="set-vehicle"
              value={settings.vehicle}
              disabled={onIsaac}
              title={onIsaac ? 'Isaac Sim flies the FireFly only' : undefined}
              onChange={(e) => onChange({ ...settings, vehicle: e.target.value as Settings['vehicle'] })}
            >
              <option value="firefly">FireFly 2 (the real drone)</option>
              <option value="iris">ArduPilot Iris</option>
            </select>
          </div>
          <div className="field field-inline">
            <label htmlFor="set-window">{onIsaac ? 'Isaac Sim window' : 'Gazebo window'}</label>
            <input
              id="set-window"
              type="checkbox"
              checked={settings.gazebo_window}
              onChange={(e) => onChange({ ...settings, gazebo_window: e.target.checked })}
            />
            <small>the 3D view following the drone, with live screens of its cameras; off runs headless</small>
          </div>
        </>
      )}
      <h4>Greenhouse</h4>
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
      {GREENHOUSE.map(numField)}
      <h4>Mission</h4>
      {MISSION.map(numField)}
      <h4>Python simulator only</h4>
      {gazebo && <p className="muted small">In Gazebo, the battery, wind and camera come from the simulation itself.</p>}
      <fieldset disabled={gazebo}>
        {PYTHON_ONLY.map(numField)}
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
      </fieldset>
    </div>
  );
}
