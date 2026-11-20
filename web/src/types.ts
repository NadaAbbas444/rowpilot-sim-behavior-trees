// Shapes shared with the Python server (rowpilot/editor). See server.py for the API.

export type Category = 'composite' | 'decorator' | 'condition' | 'action';
export type ParamKind = 'int' | 'float' | 'bool' | 'choice' | 'lane';
export type ParamValue = number | boolean | string;

export interface ParamSpec {
  name: string;
  kind: ParamKind;
  default: ParamValue;
  help: string;
  choices?: string[];
  min?: number;
  max?: number;
}

export interface NodeTypeSpec {
  type: string;
  category: Category;
  group: string;
  robots: Robot[];
  help: string;
  params: ParamSpec[];
  min_children: number;
  max_children: number | null;
}

export interface GraphNode {
  id: string;
  type: string;
  name: string;
  params: Record<string, ParamValue>;
  position?: { x: number; y: number };
}

export interface GraphEdge {
  source: string;
  target: string;
}

export interface Graph {
  version: number;
  nodes: GraphNode[];
  edges: GraphEdge[];
}

export interface GraphIssue {
  node: string | null;
  message: string;
}

export type Status = 'RUNNING' | 'SUCCESS' | 'FAILURE' | 'INVALID';

export type Simulator = 'python' | 'gazebo';

/** The FireFly 2 drone, or the pipe-rail trolley (Python simulator only, so far). */
export type Robot = 'drone' | 'trolley';

export interface Settings {
  simulator: Simulator;
  robot: Robot;
  vehicle: 'firefly' | 'iris';
  gazebo_window: boolean;
  lanes: number;
  open_ends: boolean;
  row_length: number;
  path_width: number;
  seed: number;
  battery: number;
  wind: number;
  vo_dropouts: [number, number][];
  altitude: number;
  sweep_speed: number;
  speed: number;
  max_time_s: number;
}

export interface EventMsg {
  kind: 'event';
  t: number;
  type: 'status' | 'log' | 'end' | 'system';
  node: string;
  source?: string; // system messages: ardupilot, mavlink, ekf, vio, sim, setup, editor
  id?: string;
  status?: Status;
  msg?: string;
}

export interface TickMsg {
  kind: 'tick';
  t: number;
  x: number;
  y: number;
  z: number;
  yaw: number;
  est_x: number;
  est_y: number;
  battery: number;
  vo_ok: boolean;
  node: string;
}

export interface Summary {
  outcome: string;
  duration_s: number;
  battery_end_pct: number;
  lanes_completed: number[];
  lane_rms_error_m: Record<string, number>;
  max_lane_error_m: number;
}

export interface EndMsg {
  kind: 'end';
  status: 'finished' | 'stopped' | 'error';
  summary: Summary | null;
  error: string | null;
}

export type RunMessage = EventMsg | TickMsg | EndMsg;

export interface LayoutInfo {
  n_lanes: number;
  row_length: number;
  crop_width: number;
  pitch: number;
  headland: number;
  closed_end: boolean;
  far_x: number;
  home: [number, number];
  crop_rows_y: number[];
  lane_centers_y: number[];
  planned_route: [number, number][];
}

export interface SimAvailability {
  available: boolean;
  missing: string[];
}

/** Which simulators can run here; gazebo_trolley: Gazebo for the trolley (it needs no ArduPilot). */
export type Simulators = Record<Simulator, SimAvailability> & { gazebo_trolley?: SimAvailability };

export interface RunInfo {
  id: string;
  status: string;
  summary: Summary | null;
  error: string | null;
}
