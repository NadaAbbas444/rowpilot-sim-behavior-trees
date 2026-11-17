"""Procedural assets for the Gazebo greenhouse: tomato plant meshes and a floor texture.

Everything is generated from a seed, so worlds are reproducible and there are no third-party model
licences to worry about. Plants follow high-wire greenhouse tomato growing: one stem per plant,
leaves every ~22 cm in a spiral, the bottom ~60 cm de-leafed, fruit trusses that ripen from the
bottom up. Leaves are double-sided quads (cameras see them from both sides) with ~7 leaflets each.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

MATERIALS = {
    "stem": (0.33, 0.45, 0.20),
    "leaf_dark": (0.13, 0.36, 0.10),
    "leaf_mid": (0.20, 0.46, 0.14),
    "leaf_light": (0.32, 0.56, 0.20),
    "fruit_red": (0.78, 0.12, 0.08),
    "fruit_orange": (0.88, 0.45, 0.10),
    "fruit_green": (0.45, 0.62, 0.22),
}


class _Mesh:
    def __init__(self) -> None:
        self.v: list[tuple[float, float, float]] = []
        self.groups: dict[str, list[tuple[int, int, int]]] = {}

    def add(self, verts: np.ndarray, tris: list[tuple[int, int, int]], material: str, double_sided=False) -> None:
        base = len(self.v)
        self.v += [tuple(map(float, p)) for p in verts]
        faces = self.groups.setdefault(material, [])
        for a, b, c in tris:
            faces.append((base + a, base + b, base + c))
            if double_sided:
                faces.append((base + a, base + c, base + b))

    def dae(self) -> str:
        """COLLADA text with one coloured material per group (Gazebo honours COLLADA materials; it
        ignores .mtl colours for OBJ). Faces are flat-shaded: each triangle gets its own normal."""
        v = np.asarray(self.v)
        effects, materials, geometries, nodes = [], [], [], []
        for name, faces in self.groups.items():
            if not faces:
                continue
            r, g, b = MATERIALS[name]
            effects.append(
                f'<effect id="{name}-fx"><profile_COMMON><technique sid="t"><lambert>'
                f"<diffuse><color>{r} {g} {b} 1</color></diffuse></lambert></technique></profile_COMMON></effect>"
            )
            materials.append(f'<material id="{name}-mat"><instance_effect url="#{name}-fx"/></material>')
            f = np.asarray(faces)
            tri = v[f]  # (n, 3, 3)
            nrm = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
            nrm /= np.linalg.norm(nrm, axis=1, keepdims=True) + 1e-12
            pos = " ".join(f"{x:.4f}" for x in tri.reshape(-1))
            nor = " ".join(f"{x:.3f}" for x in nrm.reshape(-1))
            n = len(f)
            idx = " ".join(f"{3 * i + k} {i}" for i in range(n) for k in range(3))
            geometries.append(
                f'<geometry id="{name}-geo"><mesh>'
                f'<source id="{name}-pos"><float_array id="{name}-pos-a" count="{9 * n}">{pos}</float_array>'
                f'<technique_common><accessor source="#{name}-pos-a" count="{3 * n}" stride="3">'
                '<param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>'
                "</accessor></technique_common></source>"
                f'<source id="{name}-nrm"><float_array id="{name}-nrm-a" count="{3 * n}">{nor}</float_array>'
                f'<technique_common><accessor source="#{name}-nrm-a" count="{n}" stride="3">'
                '<param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>'
                "</accessor></technique_common></source>"
                f'<vertices id="{name}-vtx"><input semantic="POSITION" source="#{name}-pos"/></vertices>'
                f'<triangles material="{name}" count="{n}">'
                f'<input semantic="VERTEX" source="#{name}-vtx" offset="0"/>'
                f'<input semantic="NORMAL" source="#{name}-nrm" offset="1"/><p>{idx}</p></triangles>'
                "</mesh></geometry>"
            )
            nodes.append(
                f'<node id="{name}-node"><instance_geometry url="#{name}-geo"><bind_material><technique_common>'
                f'<instance_material symbol="{name}" target="#{name}-mat"/>'
                "</technique_common></bind_material></instance_geometry></node>"
            )
        return (
            '<?xml version="1.0" encoding="utf-8"?>\n'
            '<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">'
            '<asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>'
            f"<library_effects>{''.join(effects)}</library_effects>"
            f"<library_materials>{''.join(materials)}</library_materials>"
            f"<library_geometries>{''.join(geometries)}</library_geometries>"
            f'<library_visual_scenes><visual_scene id="scene">{"".join(nodes)}</visual_scene></library_visual_scenes>'
            '<scene><instance_visual_scene url="#scene"/></scene></COLLADA>\n'
        )


def _cylinder(p0, p1, r: float, sides: int = 6) -> tuple[np.ndarray, list]:
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    axis = p1 - p0
    axis /= np.linalg.norm(axis)
    tmp = np.array([1.0, 0, 0]) if abs(axis[0]) < 0.9 else np.array([0, 1.0, 0])
    u = np.cross(axis, tmp)
    u /= np.linalg.norm(u)
    w = np.cross(axis, u)
    ring = [r * (math.cos(t) * u + math.sin(t) * w) for t in np.linspace(0, 2 * math.pi, sides, endpoint=False)]
    verts = np.array([p0 + q for q in ring] + [p1 + q for q in ring])
    tris = []
    for i in range(sides):
        j = (i + 1) % sides
        tris += [(i, j, sides + i), (j, sides + j, sides + i)]
    return verts, tris


def _sphere(c, r: float) -> tuple[np.ndarray, list]:
    """Low-poly sphere (octahedron subdivided once: 18 vertices, 32 faces)."""
    v = [np.array(p, float) for p in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]]
    f = [(0, 2, 4), (2, 1, 4), (1, 3, 4), (3, 0, 4), (2, 0, 5), (1, 2, 5), (3, 1, 5), (0, 3, 5)]
    mid: dict[tuple[int, int], int] = {}

    def m(a, b):
        k = (min(a, b), max(a, b))
        if k not in mid:
            p = v[a] + v[b]
            v.append(p / np.linalg.norm(p))
            mid[k] = len(v) - 1
        return mid[k]

    f2 = []
    for a, b, cc in f:
        ab, bc, ca = m(a, b), m(b, cc), m(cc, a)
        f2 += [(a, ab, ca), (b, bc, ab), (cc, ca, bc), (ab, bc, ca)]
    return np.array(c) + r * np.array(v), f2


def _leaflet(origin, direction, side_dir, length: float, width: float) -> np.ndarray:
    """A diamond-shaped leaflet (4 vertices) lying along `direction`."""
    o = np.asarray(origin)
    return np.array(
        [
            o,
            o + 0.35 * length * direction + 0.5 * width * side_dir,
            o + length * direction,
            o + 0.35 * length * direction - 0.5 * width * side_dir,
        ]
    )


def tomato_plant_mesh(seed: int, height: float = 2.6) -> _Mesh:
    rng = np.random.default_rng(seed)
    mesh = _Mesh()
    lean = rng.normal(0, 0.03, 2)
    top = np.array([lean[0], lean[1], height])

    def stem_at(z: float) -> np.ndarray:
        return np.array([lean[0] * z / height, lean[1] * z / height, z])

    mesh.add(*_cylinder((0, 0, 0), top, 0.012), "stem")
    z, phi = 0.55 + rng.uniform(0, 0.1), rng.uniform(0, 2 * math.pi)  # bottom is de-leafed
    node = 0
    while z < height - 0.08:
        node += 1
        phi += math.radians(137.5) + rng.normal(0, 0.25)
        young = max(0.45, min(1.0, (height - z) / 0.6))  # leaves near the top are smaller
        length = rng.uniform(0.42, 0.55) * young
        out = np.array([math.cos(phi), math.sin(phi), 0.0])
        rise = rng.uniform(0.15, 0.45)
        base = stem_at(z)
        # rachis: rises, then droops (two segments)
        p1 = base + 0.5 * length * (out * math.cos(rise) + np.array([0, 0, math.sin(rise)]))
        droop = rise - rng.uniform(0.5, 0.9)
        d2 = out * math.cos(droop) + np.array([0, 0, math.sin(droop)])
        p2 = p1 + 0.5 * length * d2
        mesh.add(*_cylinder(base, p1, 0.004, 4), "stem")
        mesh.add(*_cylinder(p1, p2, 0.003, 4), "stem")
        side = np.cross(out, [0, 0, 1.0])
        mat = ["leaf_dark", "leaf_mid", "leaf_light"][min(2, int(rng.integers(0, 3)) if z < height - 0.5 else 2)]
        for k, t in enumerate((0.15, 0.32, 0.49, 0.66, 0.83)):  # leaflet pairs along the rachis
            p = base + t * (p2 - base)
            for s in (-1, 1):
                d = side * s * 0.8 + out * 0.5 + np.array([0, 0, rng.normal(-0.3, 0.15)])
                d /= np.linalg.norm(d)
                w = np.cross(d, [0, 0, 1.0])
                w = w / (np.linalg.norm(w) + 1e-9)
                ll = rng.uniform(0.13, 0.19) * young * (1.0 - 0.06 * k)
                mesh.add(_leaflet(p, d, w, ll, ll * 0.62), [(0, 1, 2), (0, 2, 3)], mat, double_sided=True)
        tip_w = np.cross(d2, [0, 0, 1.0])
        tip_w /= np.linalg.norm(tip_w) + 1e-9
        tl = rng.uniform(0.12, 0.17) * young
        mesh.add(_leaflet(p2, d2, tip_w, tl, tl * 0.55), [(0, 1, 2), (0, 2, 3)], mat, double_sided=True)
        # fruit truss on every third node, opposite-ish to the leaf, in the lower part of the plant
        if node % 3 == 0 and z < height - 0.7:
            colour = "fruit_red" if z < 1.25 else ("fruit_orange" if z < 1.6 else "fruit_green")
            t_dir = -out + rng.normal(0, 0.2, 3) * np.array([1, 1, 0])
            t_dir[2] = 0
            t_dir /= np.linalg.norm(t_dir)
            for i in range(int(rng.integers(4, 7))):
                c = base + t_dir * (0.06 + 0.035 * i) + np.array([0, 0, -0.05 - 0.02 * i + rng.normal(0, 0.01)])
                mesh.add(*_sphere(c, rng.uniform(0.022, 0.032)), colour)
        z += rng.uniform(0.17, 0.22)
    return mesh


def floor_texture(path: Path, size_px: tuple[int, int] = (2048, 1536), seed: int = 0) -> Path:
    """Concrete-and-grit floor: multi-scale noise plus speckles, so a camera finds features on it."""
    from PIL import Image

    rng = np.random.default_rng(seed)
    w, h = size_px
    img = np.full((h, w), 150.0)
    for scale, amp in ((256, 18), (64, 14), (16, 10), (4, 8)):
        small = rng.normal(0, 1, (h // scale + 2, w // scale + 2))
        big = np.kron(small, np.ones((scale, scale)))[:h, :w]
        img += amp * big
    img += rng.normal(0, 6, (h, w))
    n = w * h // 400  # dark and light grit
    ys, xs = rng.integers(0, h, n), rng.integers(0, w, n)
    img[ys, xs] = rng.choice([60, 220], n)
    img = np.clip(img, 0, 255).astype(np.uint8)
    rgb = np.stack([img, (img * 0.98).astype(np.uint8), (img * 0.94).astype(np.uint8)], axis=-1)
    Image.fromarray(rgb).save(path)
    return path


def write_plant_model(model_dir: Path, variants: int = 8, height: float = 2.6, seed: int = 0) -> list[str]:
    """Write a Gazebo model folder with plant meshes (COLLADA) and the floor texture. Returns mesh file names."""
    meshes = model_dir / "meshes"
    meshes.mkdir(parents=True, exist_ok=True)
    names = []
    for i in range(variants):
        names.append(f"tomato_{i}.dae")
        (meshes / names[-1]).write_text(tomato_plant_mesh(seed * 100 + i, height).dae())
    floor_texture(meshes / "floor.png", seed=seed)
    (model_dir / "model.config").write_text(
        '<?xml version="1.0"?>\n<model>\n  <name>greenhouse_assets</name>\n  <version>1.0</version>\n'
        '  <sdf version="1.9">model.sdf</sdf>\n  <description>Procedural tomato plants and floor texture for the'
        " rowpilot greenhouse (meshes are referenced by the world).</description>\n</model>\n"
    )
    (model_dir / "model.sdf").write_text(
        '<?xml version="1.0"?>\n<sdf version="1.9">\n  <model name="greenhouse_assets"><static>true</static>'
        '<link name="link"/></model>\n</sdf>\n'
    )
    return names
