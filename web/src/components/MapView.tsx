import { memo } from 'react';
import type { PathPoint } from '../runState';
import type { LayoutInfo, TickMsg } from '../types';

interface Props {
  layout: LayoutInfo | null;
  truePath: PathPoint[];
  estPath: PathPoint[];
  last: TickMsg | null;
}

/** The greenhouse from above: crop rows, the planned path, where the drone flew and where it thought it was. */
function MapView({ layout: L, truePath, estPath, last }: Props) {
  if (!L) return <div className="map muted">loading the greenhouse…</div>;
  const x0 = L.home[0] - 1;
  const x1 = L.far_x + (L.closed_end ? 0.6 : -1);
  const y0 = L.crop_rows_y[0] - 0.6;
  const y1 = L.crop_rows_y[L.crop_rows_y.length - 1] + 0.6;
  const Y = (y: number) => y1 - y; // map y points left/up; SVG y points down
  const pts = (p: { x: number; y: number }[]) => p.map((q) => `${q.x.toFixed(2)},${Y(q.y).toFixed(2)}`).join(' ');
  const route = L.planned_route.map(([x, y]) => ({ x, y }));
  return (
    <svg
      className="map"
      viewBox={`${x0} 0 ${x1 - x0} ${y1 - y0}`}
      preserveAspectRatio="xMidYMid meet"
      role="img"
      aria-label="greenhouse map"
    >
      {L.crop_rows_y.map((c) => (
        <rect
          key={c}
          className="map-crop"
          x={0}
          y={Y(c + L.crop_width / 2)}
          width={L.row_length}
          height={L.crop_width}
        />
      ))}
      {L.closed_end && (
        <line className="map-wall" x1={L.far_x} x2={L.far_x} y1={Y(L.crop_rows_y[0] - 0.4)} y2={Y(y1 - 0.2)} />
      )}
      {L.lane_centers_y.map((y, i) => (
        <text key={i} className="map-label" x={-0.45} y={Y(y) + 0.18} textAnchor="middle">
          {i + 1}
        </text>
      ))}
      <polyline className="map-planned" points={pts(route)} />
      <polyline className="map-est" points={pts(estPath)} />
      <polyline className="map-true" points={pts(truePath)} />
      <rect className="map-home" x={L.home[0] - 0.2} y={Y(L.home[1]) - 0.2} width={0.4} height={0.4} />
      {last && (
        <g transform={`translate(${last.x} ${Y(last.y)}) rotate(${(-last.yaw * 180) / Math.PI})`}>
          <polygon className="map-drone" points="0.45,0 -0.3,0.3 -0.15,0 -0.3,-0.3" />
        </g>
      )}
    </svg>
  );
}

export default memo(MapView);
