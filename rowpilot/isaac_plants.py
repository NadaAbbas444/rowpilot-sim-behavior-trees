"""Detailed tomato plants and textures for Isaac Sim (no Isaac Sim needed here).

The plants stand where Gazebo's do and have the same parts (greenhouse_assets.tomato_plant_parts: every
stem, leaflet and fruit at the same place), but in the detail an RTX renderer shows:
  - stems: smooth cylinders (10 sides, smooth normals);
  - leaflets: curved cards (curling down towards the tip, folded along the midrib) carrying a generated
    tomato-leaflet texture: serrated outline cut out by alpha, midrib and lateral veins, colour variation;
  - fruit: smooth spheres, slightly flattened, as tomatoes are.
Textures are generated from a seed, like everything else in the greenhouse (no third-party assets):
the leaflet's colour + alpha and normal map, and a normal map for the floor's ground cover.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .greenhouse_assets import MATERIALS, tomato_plant_parts

LEAF_CARD_WIDTH = 1.35  # the card is wider than the Gazebo leaflet: the texture's outline cuts it to shape
LEAF_CURL = 0.18  # tip droop, as a fraction of the leaflet length
LEAF_FOLD = 0.12  # V-fold along the midrib, as a fraction of the card width


@dataclass
class MeshData:
    points: list = field(default_factory=list)
    normals: list = field(default_factory=list)
    uvs: list = field(default_factory=list)
    tris: list = field(default_factory=list)

    def add(self, p: np.ndarray, n: np.ndarray, uv: np.ndarray, tris: np.ndarray) -> None:
        base = sum(len(a) for a in self.points)
        self.points.append(np.asarray(p, np.float32))
        self.normals.append(np.asarray(n, np.float32))
        self.uvs.append(np.asarray(uv, np.float32))
        self.tris.append(np.asarray(tris, np.int32) + base)

    def arrays(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """(points (n, 3), normals (n, 3), uvs (n, 2), triangles (m, 3)): one normal and uv per point."""
        if not self.points:
            z = np.zeros((0, 3), np.float32)
            return z, z, np.zeros((0, 2), np.float32), np.zeros((0, 3), np.int32)
        return (
            np.concatenate(self.points),
            np.concatenate(self.normals),
            np.concatenate(self.uvs),
            np.concatenate(self.tris),
        )


def _unit(v: np.ndarray) -> np.ndarray:
    return v / (np.linalg.norm(v, axis=-1, keepdims=True) + 1e-12)


def stem(p0, p1, r: float, sides: int = 10):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    axis = _unit(p1 - p0)
    tmp = np.array([1.0, 0, 0]) if abs(axis[0]) < 0.9 else np.array([0, 1.0, 0])
    u = _unit(np.cross(axis, tmp))
    w = np.cross(axis, u)
    ang = np.linspace(0, 2 * math.pi, sides + 1)  # seam vertex repeated for the uvs
    ring = np.cos(ang)[:, None] * u + np.sin(ang)[:, None] * w
    pts = np.concatenate([p0 + r * ring, p1 + r * ring])
    nrm = np.concatenate([ring, ring])
    uv = np.concatenate(
        [np.stack([ang / (2 * math.pi), np.zeros_like(ang)], 1), np.stack([ang / (2 * math.pi), np.ones_like(ang)], 1)]
    )
    n = sides + 1
    tris = [(i, i + 1, n + i) for i in range(sides)] + [(i + 1, n + i + 1, n + i) for i in range(sides)]
    return pts, nrm, uv, np.array(tris)


def leaflet(origin, d, w, length: float, width: float, nu: int = 5, nv: int = 3):
    """A curved leaflet card from the leaflet's base along d, across w; uv (0, 0.5) at the base."""
    o, d, w = np.asarray(origin, float), _unit(np.asarray(d, float)), _unit(np.asarray(w, float))
    n = _unit(np.cross(d, w))
    if n[2] < 0:  # the leaf's upper side faces up
        n = -n
    cw = width * LEAF_CARD_WIDTH
    u, v = np.meshgrid(np.linspace(0, 1, nu), np.linspace(0, 1, nv), indexing="ij")
    across = (v - 0.5) * cw
    lift = -LEAF_CURL * length * u**2 + LEAF_FOLD * cw * np.abs(v - 0.5) * 2 * (1 - 0.5 * u)
    pts = o + (u * length)[..., None] * d + across[..., None] * w + lift[..., None] * n
    du = np.gradient(pts, axis=0)
    dv = np.gradient(pts, axis=1)
    nrm = _unit(np.cross(du, dv))
    nrm *= np.sign((nrm @ n).mean() + 1e-12)
    uv = np.stack([u, v], -1)
    idx = np.arange(nu * nv).reshape(nu, nv)
    tris = []
    for i in range(nu - 1):
        for j in range(nv - 1):
            a, b, c, e = idx[i, j], idx[i + 1, j], idx[i + 1, j + 1], idx[i, j + 1]
            tris += [(a, b, c), (a, c, e)]
    return pts.reshape(-1, 3), nrm.reshape(-1, 3), uv.reshape(-1, 2), np.array(tris)


def fruit(c, r: float, nu: int = 16, nv: int = 10):
    th, ph = np.meshgrid(np.linspace(0, 2 * math.pi, nu + 1), np.linspace(0, math.pi, nv + 1), indexing="ij")
    unit = np.stack([np.sin(ph) * np.cos(th), np.sin(ph) * np.sin(th), np.cos(ph)], -1)
    squash = np.array([1.0, 1.0, 0.82])  # tomatoes are wider than tall
    pts = np.asarray(c, float) + r * unit * squash
    nrm = _unit(unit / squash)
    uv = np.stack([th / (2 * math.pi), ph / math.pi], -1)
    idx = np.arange((nu + 1) * (nv + 1)).reshape(nu + 1, nv + 1)
    tris = []
    for i in range(nu):
        for j in range(nv):
            a, b, cc, e = idx[i, j], idx[i + 1, j], idx[i + 1, j + 1], idx[i, j + 1]
            tris += [(a, b, cc), (a, cc, e)]
    return pts.reshape(-1, 3), nrm.reshape(-1, 3), uv.reshape(-1, 2), np.array(tris)


def detailed_plant(seed: int, height: float = 2.6) -> dict[str, MeshData]:
    """The plant greenhouse_assets.tomato_plant_mesh(seed, height) describes, in detail; by material."""
    out: dict[str, MeshData] = {}
    for part in tomato_plant_parts(seed, height):
        kind = part[0]
        if kind == "stem":
            _, p0, p1, r, _sides = part
            out.setdefault("stem", MeshData()).add(*stem(p0, p1, r))
        elif kind == "leaflet":
            _, p, d, w, length, width, mat = part
            out.setdefault(mat, MeshData()).add(*leaflet(p, d, w, length, width))
        else:
            _, c, r, mat = part
            out.setdefault(mat, MeshData()).add(*fruit(c, r))
    return out


def colour(material: str) -> tuple[float, float, float]:
    return MATERIALS[material]


# ---- textures ---------------------------------------------------------------------------------


def leaf_texture(w: int = 512, h: int = 256, seed: int = 0) -> tuple[np.ndarray, np.ndarray]:
    """A tomato leaflet, base at the left, tip at the right: (RGBA uint8, height map 0..1).
    Ovate with a pointed tip, serrated edge, a pale midrib and lateral veins, slight colour mottling.
    The colour is near white-green so the material's tint sets the shade (dark, mid, light leaves)."""
    rng = np.random.default_rng(seed)
    x = (np.arange(w) + 0.5) / w
    y = (np.arange(h) + 0.5) / h - 0.5
    X, Y = np.meshgrid(x, y)
    half = 0.48 * np.sin(np.pi * np.clip(X, 0, 1) ** 0.75) ** 0.85
    saw = (X * 14.0) % 1.0  # serrated: each tooth rises slowly and drops sharply towards the tip
    teeth = 1.0 + 0.09 * saw - 0.045
    inside = np.abs(Y) < half * teeth * (X > 0.02)
    # veins: the midrib, and laterals leaving it at ~50 deg towards the tip
    midrib = np.exp(-((Y / 0.012) ** 2)) * (X < 0.97)
    phase = (X - np.abs(Y) * 0.9) * 9.0
    lateral = np.exp(-(((phase - np.round(phase)) / 0.06) ** 2)) * (np.abs(Y) < half * 0.9) * (X > 0.06)
    veins = np.clip(midrib + 0.55 * lateral, 0, 1)
    # mottling: smooth noise
    noise = rng.normal(size=(h // 16 + 2, w // 16 + 2))
    yi = np.linspace(0, noise.shape[0] - 1.001, h)
    xi = np.linspace(0, noise.shape[1] - 1.001, w)
    y0, x0 = yi.astype(int), xi.astype(int)
    fy, fx = (yi - y0)[:, None], (xi - x0)[None, :]
    n00, n01 = noise[y0][:, x0], noise[y0][:, x0 + 1]
    n10, n11 = noise[y0 + 1][:, x0], noise[y0 + 1][:, x0 + 1]
    mott = (n00 * (1 - fx) + n01 * fx) * (1 - fy) + (n10 * (1 - fx) + n11 * fx) * fy
    edge = np.clip(1 - (half * teeth - np.abs(Y)) / 0.05, 0, 1) * inside
    g = 0.78 + 0.06 * mott + 0.18 * veins - 0.08 * edge
    rgb = np.stack([g * (0.80 + 0.12 * veins + 0.10 * edge), g, g * (0.62 + 0.15 * veins)], -1)
    rgba = np.concatenate([np.clip(rgb, 0, 1), inside[..., None].astype(float)], -1)
    height = np.clip(0.6 - 0.5 * veins + 0.05 * mott, 0, 1)
    return (rgba * 255 + 0.5).astype(np.uint8), height


def normal_map(height: np.ndarray, strength: float = 2.0) -> np.ndarray:
    """Tangent-space normal map (RGB uint8) from a height map (0..1), as renderers read them."""
    gy, gx = np.gradient(height.astype(float))
    n = _unit(np.stack([-gx * strength, gy * strength, np.ones_like(gx)], -1))
    return ((n * 0.5 + 0.5) * 255 + 0.5).astype(np.uint8)


def write_textures(out: Path, floor_png: Path | None = None, seed: int = 0) -> dict[str, Path]:
    """Writes the textures Isaac Sim's greenhouse uses into out; returns their paths by name."""
    from PIL import Image

    out.mkdir(parents=True, exist_ok=True)
    rgba, h = leaf_texture(seed=seed)
    files = {"leaf": out / "leaf.png", "leaf_normal": out / "leaf_normal.png"}
    Image.fromarray(rgba, "RGBA").save(files["leaf"])
    Image.fromarray(normal_map(h, 3.0), "RGB").save(files["leaf_normal"])
    if floor_png is not None and floor_png.exists():
        lum = np.asarray(Image.open(floor_png).convert("L"), float) / 255.0
        files["floor_normal"] = out / "floor_normal.png"
        Image.fromarray(normal_map(lum, 4.0), "RGB").save(files["floor_normal"])
    return files


# ---- which plant a Gazebo mesh is ----------------------------------------------------------------


def read_plant_index(meshes_dir: Path) -> dict[str, dict]:
    """{mesh file name: {"seed", "height"}}, as greenhouse_assets.write_plant_model records them."""
    f = meshes_dir / "plants.json"
    return json.loads(f.read_text()) if f.exists() else {}
