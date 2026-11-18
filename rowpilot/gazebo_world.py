"""Generate a Gazebo (Harmonic) greenhouse world from the same GreenhouseLayout the Python simulator uses.

The world frame is the map frame (ENU: x along the rows, y across them), so a pose in Gazebo is a
pose in the mission's map frame.

Each crop row is an invisible collision box (so a drone that drifts into the crop hits it) filled
with procedural tomato plants (see greenhouse_assets.py). The lanes have the heating-pipe rails of a
tomato greenhouse, the floor is ground cover with dirt and leaf litter (visual odometry needs features
to track), steel posts stand in the crop rows, yellow sticky tape hangs above each row at 2.8 m, and
glass facades with columns and glazing bars close the greenhouse. With closed lanes (the default)
the far facade stands at the end of the rows, so every lane is a dead end.
Without plant meshes (plants=None) the rows are plain green boxes, which is quicker to render.

PX4 needs its own world plugins (physics, sensors, IMU, magnetometer, ...). Rather than copy them,
pass PX4's `Tools/simulation/gz/worlds/default.sdf` as the template: everything is inserted into it.
"""

from __future__ import annotations

import re
from pathlib import Path

import numpy as np

from .layout import GreenhouseLayout

ASSETS = "model://greenhouse_assets/meshes"
PLANT_SPACING_M = 0.25  # about 2 stems per m2 with a 2 m row pitch
PIPE_OFFSET_M = 0.275  # each pipe rail is this far from the lane centre (55 cm rail gauge)
PIPE_RADIUS_M = 0.0255  # 51 mm heating pipe
TAPE_HEIGHT_M = 2.8
ACROSS_ROW_SQUEEZE = 0.42  # leaves trained along the row: they reach ~14 cm into the lane, as in sim.py


def _static(name: str, body: str, pose=(0, 0, 0, 0, 0, 0)) -> str:
    p = " ".join(f"{v:.4f}" for v in pose)
    return f'\n    <model name="{name}">\n      <static>true</static>\n      <pose>{p}</pose>\n{body}    </model>'


def _box_link(size, colour=None, collision=True, visual=True) -> str:
    s = " ".join(f"{v:.4f}" for v in size)
    geo = f"<geometry><box><size>{s}</size></box></geometry>"
    out = '      <link name="link">\n'
    if collision:
        out += f'        <collision name="collision">{geo}</collision>\n'
    if visual:
        r, g, b = colour
        out += (
            f'        <visual name="visual">{geo}<material><ambient>{r} {g} {b} 1</ambient>'
            f"<diffuse>{r} {g} {b} 1</diffuse></material></visual>\n"
        )
    return out + "      </link>\n"


def crop_rows(layout: GreenhouseLayout, plants: list[str] | None, seed: int = 0) -> str:
    """One model per crop row: a collision box, and either plant meshes or a green box as visuals."""
    h, L, w = layout.canopy_height, layout.row_length, layout.crop_width
    rng = np.random.default_rng(seed)
    out = []
    for k, c in enumerate(layout.crop_row_centers_y()):
        geo = f"<geometry><box><size>{L:.4f} {w:.4f} {h:.4f}</size></box></geometry>"
        body = '      <link name="link">\n'
        body += f'        <collision name="collision"><pose>{L / 2:.4f} 0 {h / 2:.4f} 0 0 0</pose>{geo}</collision>\n'
        if not plants:
            body += (
                f'        <visual name="visual"><pose>{L / 2:.4f} 0 {h / 2:.4f} 0 0 0</pose>{geo}'
                "<material><ambient>0.25 0.45 0.15 1</ambient><diffuse>0.30 0.55 0.20 1</diffuse>"
                "</material></visual>\n"
            )
        else:
            for i, x in enumerate(np.arange(PLANT_SPACING_M / 2, L, PLANT_SPACING_M)):
                x += rng.normal(0, 0.03)
                y = (0.06 if i % 2 else -0.06) + rng.normal(0, 0.015)  # zig-zag, as plants are trained
                yaw = rng.choice([0.0, np.pi]) + rng.normal(0, 0.2)
                s = rng.uniform(0.9, 1.08)
                mesh = plants[int(rng.integers(0, len(plants)))]
                # Leaves are trained along the row: squeeze the plant across it.
                body += (
                    f'        <visual name="plant_{i}"><pose>{x:.3f} {y:.3f} 0 0 0 {yaw:.3f}</pose>'
                    f"<geometry><mesh><uri>{ASSETS}/{mesh}</uri>"
                    f"<scale>{s:.3f} {ACROSS_ROW_SQUEEZE * s:.3f} {s:.3f}</scale></mesh></geometry></visual>\n"
                )
        body += "      </link>\n"
        out.append(_static(f"crop_row_{k}", body, (0, c, 0, 0, 0, 0)))
    return "".join(out)


def infrastructure(layout: GreenhouseLayout, textured_floor: bool) -> str:
    L, out = layout.row_length, []
    # Heating-pipe rails in every lane.
    for i in range(layout.n_lanes):
        for s in (-1, 1):
            y = layout.lane_center_y(i) + s * PIPE_OFFSET_M
            geo = f"<geometry><cylinder><radius>{PIPE_RADIUS_M}</radius><length>{L:.4f}</length></cylinder></geometry>"
            body = (
                '      <link name="link">\n'
                f'        <collision name="collision">{geo}</collision>\n'
                f'        <visual name="visual">{geo}<material><ambient>0.55 0.55 0.57 1</ambient>'
                "<diffuse>0.7 0.7 0.72 1</diffuse><specular>0.6 0.6 0.6 1</specular></material></visual>\n"
                "      </link>\n"
            )
            out.append(_static(f"pipe_{i}_{'l' if s > 0 else 'r'}", body, (L / 2, y, 0.12, 0, np.pi / 2, 0)))
    # Steel posts inside the crop rows every ~4 m, and the yellow sticky tape above each row.
    for k, c in enumerate(layout.crop_row_centers_y()):
        for j, x in enumerate(np.arange(0.0, L + 0.01, L / max(1, round(L / 4.0)))):
            out.append(_static(f"post_{k}_{j}", _box_link((0.08, 0.08, 4.0), (0.6, 0.6, 0.62)), (x, c, 2.0, 0, 0, 0)))
        out.append(
            _static(
                f"tape_{k}",
                _box_link((L, 0.003, 0.25), (0.95, 0.85, 0.05), collision=False),
                (L / 2, c, TAPE_HEIGHT_M, 0, 0, 0),
            )
        )
    x0, x1 = layout.home[0] - 2.0, layout.far_x  # closed lanes end at the far wall
    y0, y1 = -layout.pitch - 1.0, layout.n_lanes * layout.pitch + 1.0
    out.append(floor(x0, x1, y0, y1, textured_floor))
    out.append(walls(x0, x1, y0, y1))
    return "".join(out)


def floor(x0: float, x1: float, y0: float, y1: float, textured: bool) -> str:
    """Floor covering the greenhouse, slightly above PX4's ground plane to avoid z-fighting.

    Textured, it is tiled with the ground-cover texture (one texture per FLOOR_TILE_M tile, turned by
    multiples of 90 deg so the pattern does not repeat in lines). Not metallic: SDF's default metalness
    (0.5) renders it dark and low in contrast, with few features for the visual odometry."""
    from .greenhouse_assets import FLOOR_TILE_M

    if not textured:
        geo = f"<geometry><box><size>{x1 - x0:.3f} {y1 - y0:.3f} 0.002</size></box></geometry>"
        vis = f'        <visual name="visual">{geo}<material><diffuse>0.6 0.6 0.6 1</diffuse></material></visual>\n'
        pose = ((x0 + x1) / 2, (y0 + y1) / 2, 0.001, 0, 0, 0)
        return _static("floor", f'      <link name="link">\n{vis}      </link>\n', pose)
    mat = (
        f"<material><diffuse>1 1 1 1</diffuse><pbr><metal><albedo_map>{ASSETS}/floor.png</albedo_map>"
        "<roughness>0.9</roughness><metalness>0.0</metalness></metal></pbr></material>"
    )
    body = '      <link name="link">\n'
    nx, ny = int(np.ceil((x1 - x0) / FLOOR_TILE_M)), int(np.ceil((y1 - y0) / FLOOR_TILE_M))
    for i in range(nx):
        for j in range(ny):
            cx, cy = x0 + (i + 0.5) * FLOOR_TILE_M, y0 + (j + 0.5) * FLOOR_TILE_M
            yaw = ((i * 3 + j * 5) % 4) * np.pi / 2
            geo = f"<geometry><box><size>{FLOOR_TILE_M} {FLOOR_TILE_M} 0.002</size></box></geometry>"
            pose = f"<pose>{cx:.3f} {cy:.3f} 0 0 0 {yaw:.4f}</pose>"
            body += f'        <visual name="tile_{i}_{j}">{pose}{geo}{mat}</visual>\n'
    return _static("floor", body + "      </link>\n", (0, 0, 0.001, 0, 0, 0))


WALL_HEIGHT_M = 4.5
WALL_POST_SPACING_M = 3.2  # facade columns
GLAZING_BAR_SPACING_M = 0.8


def walls(x0: float, x1: float, y0: float, y1: float) -> str:
    """Glass facades around the floor: a concrete kerb, steel columns, girders and glazing bars over
    light panes. In the headlands, where the drone turns, they are what the cameras see."""
    body = '      <link name="link">\n'
    n = 0

    def box(size, pose, colour, collide=False):
        nonlocal body, n
        n += 1
        s = " ".join(f"{v:.3f}" for v in size)
        p = " ".join(f"{v:.3f}" for v in pose)
        geo = f"<geometry><box><size>{s}</size></box></geometry>"
        r, g, b = colour
        mat = f"<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>"
        if collide:
            body += f'        <collision name="c{n}"><pose>{p}</pose>{geo}</collision>\n'
        body += f'        <visual name="v{n}"><pose>{p}</pose>{geo}{mat}</visual>\n'

    H = WALL_HEIGHT_M
    sides = (  # (start, end) along the wall, and the fixed coordinate; along x or along y
        ((x0, x1), y0, "x"),
        ((x0, x1), y1, "x"),
        ((y0, y1), x0, "y"),
        ((y0, y1), x1, "y"),
    )
    for (a0, a1), c, axis in sides:
        length, mid = a1 - a0, (a0 + a1) / 2

        def at(u, z, _axis=axis, _c=c):
            return (u, _c, z) if _axis == "x" else (_c, u, z)

        def sized(along, across, height, _axis=axis):
            return (along, across, height) if _axis == "x" else (across, along, height)

        box(sized(length, 0.02, H), (*at(mid, H / 2), 0, 0, 0), (0.86, 0.88, 0.9), collide=True)  # panes
        box(sized(length, 0.12, 0.6), (*at(mid, 0.3), 0, 0, 0), (0.55, 0.55, 0.53))  # concrete kerb
        for z in (0.6, 2.4, H - 0.1):  # girders
            box(sized(length, 0.1, 0.12), (*at(mid, z), 0, 0, 0), (0.3, 0.31, 0.33))
        for u in np.arange(a0, a1 + 1e-6, GLAZING_BAR_SPACING_M):
            box(sized(0.04, 0.05, H - 0.6), (*at(u, 0.6 + (H - 0.6) / 2), 0, 0, 0), (0.45, 0.46, 0.48))
        for u in np.arange(a0, a1 + 1e-6, WALL_POST_SPACING_M):
            box(sized(0.12, 0.14, H), (*at(u, H / 2), 0, 0, 0), (0.22, 0.23, 0.25))
    return _static("walls", body + "      </link>\n")


def make_world(
    layout: GreenhouseLayout,
    template_sdf: str,
    world_name: str = "greenhouse",
    plants: list[str] | None = None,
    seed: int = 0,
    vehicle: str | None = None,
) -> str:
    """Insert the greenhouse into a world SDF and rename the world. With vehicle (a model URI, e.g.
    model://gh_iris for ArduPilot), the drone is placed at home; PX4 spawns its own instead."""
    sdf, n = re.subn(r'<world name="[^"]*">', f'<world name="{world_name}">', template_sdf, count=1)
    if n != 1 or "</world>" not in sdf:
        raise ValueError("template is not a single-world SDF file")
    head, tail = sdf.rsplit("</world>", 1)
    content = crop_rows(layout, plants, seed) + infrastructure(layout, textured_floor=bool(plants))
    if vehicle:
        hx, hy = layout.home
        name = vehicle.rsplit("/", 1)[-1]
        content += (
            f"\n    <include>\n      <uri>{vehicle}</uri>\n      <name>{name}</name>\n"
            f"      <pose>{hx:.3f} {hy:.3f} 0.2 0 0 0</pose>\n    </include>"
        )
    return f"{head}{content}\n  </world>{tail}"


def spawn_pose(layout: GreenhouseLayout, z: float = 0.25) -> str:
    """Value for PX4_GZ_MODEL_POSE: the drone starts at home, facing along the rows."""
    hx, hy = layout.home
    return f"{hx},{hy},{z},0,0,0"


def write_world(
    layout: GreenhouseLayout,
    template_path: Path,
    out_path: Path,
    world_name: str = "greenhouse",
    assets_dir: Path | None = None,
    seed: int = 0,
    vehicle: str | None = None,
) -> Path:
    """Write the world. With assets_dir (a Gazebo model folder, e.g. <PX4>/Tools/simulation/gz/models/
    greenhouse_assets), plant meshes and the floor texture are generated there and used."""
    plants = None
    if assets_dir is not None:
        from .greenhouse_assets import write_plant_model

        plants = write_plant_model(assets_dir, height=layout.canopy_height, seed=seed)
    out_path.write_text(
        make_world(layout, template_path.read_text(encoding="utf-8-sig"), world_name, plants, seed, vehicle)
    )
    return out_path
