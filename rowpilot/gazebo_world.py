"""Generate a Gazebo (Harmonic) greenhouse world from the same GreenhouseLayout the Python simulator uses.

The world frame is the map frame (ENU: x along the rows, y across them), so a pose in Gazebo is a
pose in the mission's map frame. Crop rows are static boxes with collision, so a lidar or depth
camera sees them and a drone that drifts into one actually hits it.

PX4 needs its own world plugins (physics, sensors, IMU, magnetometer, ...). Rather than copy them,
pass PX4's `Tools/simulation/gz/worlds/default.sdf` as the template: the rows are inserted into it.
"""

from __future__ import annotations

import re
from pathlib import Path

from .layout import GreenhouseLayout

ROW_TEMPLATE = """
    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{lx:.3f} {ly:.3f} {lz:.3f}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{lx:.3f} {ly:.3f} {lz:.3f}</size></box></geometry>
          <material>
            <ambient>0.25 0.45 0.15 1</ambient>
            <diffuse>0.30 0.55 0.20 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


def crop_row_models(layout: GreenhouseLayout) -> str:
    h = layout.canopy_height
    return "".join(
        ROW_TEMPLATE.format(
            name=f"crop_row_{k}",
            x=layout.row_length / 2,
            y=c,
            z=h / 2,
            lx=layout.row_length,
            ly=layout.crop_width,
            lz=h,
        )
        for k, c in enumerate(layout.crop_row_centers_y())
    )


def make_world(layout: GreenhouseLayout, template_sdf: str, world_name: str = "greenhouse") -> str:
    """Insert the crop rows into a PX4 world SDF and rename the world."""
    sdf, n = re.subn(r'<world name="[^"]*">', f'<world name="{world_name}">', template_sdf, count=1)
    if n != 1 or "</world>" not in sdf:
        raise ValueError("template is not a single-world SDF file")
    head, tail = sdf.rsplit("</world>", 1)
    return f"{head}{crop_row_models(layout)}\n  </world>{tail}"


def spawn_pose(layout: GreenhouseLayout, z: float = 0.25) -> str:
    """Value for PX4_GZ_MODEL_POSE: the drone starts at home, facing along the rows."""
    hx, hy = layout.home
    return f"{hx},{hy},{z},0,0,0"


def write_world(layout: GreenhouseLayout, template_path: Path, out_path: Path, world_name: str = "greenhouse") -> Path:
    out_path.write_text(make_world(layout, template_path.read_text(encoding="utf-8-sig"), world_name))
    return out_path
