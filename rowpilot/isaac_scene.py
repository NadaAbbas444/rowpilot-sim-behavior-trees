"""The Gazebo greenhouse world, read back as plain shapes for Isaac Sim (no Isaac Sim needed here).

Isaac Sim does not load SDF worlds. Rather than describe the greenhouse a second time, this reads
the world rowpilot already generates (gazebo_world.py, written by ardupilot/install.sh), so both
simulators fly the same greenhouse: every static model's visuals and collisions with their world
poses, colours, the floor texture and the plant meshes. isaac/greenhouse_isaac.py turns them into USD.

Only what gazebo_world.py writes is understood: static models with one link, box / cylinder / plane /
mesh geometry, <ambient>/<diffuse> colours and a PBR albedo map. <include>d models (the drone) are
skipped: Isaac Sim builds the drone itself from airframe.py.
"""

from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .sensors import _rot


@dataclass
class Shape:
    model: str
    name: str
    kind: str  # box | cylinder | plane | mesh
    T: np.ndarray  # 4x4 pose in the world (map frame)
    size: tuple[float, float, float] = (1.0, 1.0, 1.0)  # box size; plane: (x, y, 0)
    radius: float = 0.0  # cylinder (along its local z)
    length: float = 0.0
    uri: str = ""  # mesh
    scale: tuple[float, float, float] = (1.0, 1.0, 1.0)
    rgb: tuple[float, float, float] = (0.7, 0.7, 0.7)
    texture: str = ""  # albedo map URI
    visual: bool = True  # False: a collision shape


@dataclass
class World:
    name: str
    shapes: list[Shape] = field(default_factory=list)
    sun_direction: tuple[float, float, float] = (-0.5, 0.1, -0.9)
    includes: list[tuple[str, np.ndarray]] = field(default_factory=list)  # (uri, pose)

    def visuals(self) -> list[Shape]:
        return [s for s in self.shapes if s.visual]

    def colliders(self) -> list[Shape]:
        return [s for s in self.shapes if not s.visual]


def pose_matrix(text: str | None) -> np.ndarray:
    """SDF <pose> (x y z roll pitch yaw, radians) -> 4x4."""
    T = np.eye(4)
    if text:
        v = [float(x) for x in text.split()]
        T[:3, :3] = _rot(*v[3:6])
        T[:3, 3] = v[:3]
    return T


def _floats(text: str | None, n: int, default: float = 0.0) -> tuple[float, ...]:
    v = [float(x) for x in text.split()] if text else []
    return tuple((v + [default] * n)[:n])


def _colour(el: ET.Element | None) -> tuple[tuple[float, float, float], str]:
    if el is None:
        return (0.7, 0.7, 0.7), ""
    tex = el.findtext("pbr/metal/albedo_map", "") or ""
    for tag in ("diffuse", "ambient"):
        t = el.findtext(tag)
        if t:
            return _floats(t, 3), tex  # type: ignore[return-value]
    return (1.0, 1.0, 1.0), tex


def _shape(model: str, el: ET.Element, T_model: np.ndarray, visual: bool) -> Shape | None:
    geo = el.find("geometry")
    if geo is None or len(geo) == 0:
        return None
    g = geo[0]
    T = T_model @ pose_matrix(el.findtext("pose"))
    rgb, tex = _colour(el.find("material")) if visual else ((0.0, 0.0, 0.0), "")
    s = Shape(model, el.get("name", ""), g.tag, T, rgb=rgb, texture=tex, visual=visual)
    if g.tag == "box":
        s.size = _floats(g.findtext("size"), 3, 1.0)  # type: ignore[assignment]
    elif g.tag == "cylinder":
        s.radius, s.length = float(g.findtext("radius", "0.5")), float(g.findtext("length", "1"))
    elif g.tag == "plane":
        sx, sy = _floats(g.findtext("size"), 2, 100.0)
        s.size = (sx, sy, 0.0)
    elif g.tag == "mesh":
        s.uri = g.findtext("uri", "")
        s.scale = _floats(g.findtext("scale"), 3, 1.0)  # type: ignore[assignment]
    else:
        return None
    return s


def read_world(sdf_text: str) -> World:
    root = ET.fromstring(sdf_text)
    w = root.find("world")
    if w is None:
        raise ValueError("no <world> in the SDF")
    world = World(w.get("name", "world"))
    for light in w.findall("light"):
        if light.get("type") == "directional" and light.findtext("direction"):
            world.sun_direction = _floats(light.findtext("direction"), 3)  # type: ignore[assignment]
    for inc in w.findall("include"):
        world.includes.append((inc.findtext("uri", ""), pose_matrix(inc.findtext("pose"))))
    for m in w.findall("model"):
        name = m.get("name", "")
        T_model = pose_matrix(m.findtext("pose"))
        for link in m.findall("link"):
            T_link = T_model @ pose_matrix(link.findtext("pose"))
            for tag, visual in (("visual", True), ("collision", False)):
                for el in link.findall(tag):
                    s = _shape(name, el, T_link, visual)
                    if s is not None:
                        world.shapes.append(s)
    return world


def resolve_uri(uri: str, model_paths: list[Path]) -> Path | None:
    """model://<model>/<path> -> a file in one of the model folders (GZ_SIM_RESOURCE_PATH)."""
    if uri.startswith("file://"):
        return Path(uri[7:])
    if not uri.startswith("model://"):
        p = Path(uri)
        return p if p.exists() else None
    rel = uri[len("model://") :]
    for d in model_paths:
        if (d / rel).exists():
            return d / rel
    return None


@dataclass
class MeshPart:
    material: str
    rgb: tuple[float, float, float]
    points: np.ndarray  # (n, 3)
    triangles: np.ndarray  # (m, 3) indices into points


def read_dae(text: str) -> list[MeshPart]:
    """The COLLADA files greenhouse_assets.py writes: per material a triangle soup and a colour."""
    ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
    root = ET.fromstring(text)
    colours = {}
    for eff in root.iterfind(".//c:effect", ns):
        col = eff.findtext(".//c:diffuse/c:color", namespaces=ns)
        if col:
            colours[re.sub(r"-fx$", "", eff.get("id", ""))] = _floats(col, 3)
    parts = []
    for geo in root.iterfind(".//c:geometry", ns):
        tris = geo.find(".//c:triangles", ns)
        if tris is None:
            continue
        vtx = tris.find("c:input[@semantic='VERTEX']", ns)
        src_id = None
        if vtx is not None:
            vert = geo.find(f".//c:vertices[@id='{vtx.get('source', '').lstrip('#')}']", ns)
            inp = vert.find("c:input[@semantic='POSITION']", ns) if vert is not None else None
            src_id = inp.get("source", "").lstrip("#") if inp is not None else None
        src = geo.find(f".//c:source[@id='{src_id}']", ns) if src_id else None
        arr = src.findtext("c:float_array", namespaces=ns) if src is not None else None
        if not arr:
            continue
        pts = np.array(arr.split(), float).reshape(-1, 3)
        stride = len(tris.findall("c:input", ns))
        p = np.array((tris.findtext("c:p", "", ns) or "").split(), int).reshape(-1, stride)[:, 0]
        mat = tris.get("material", "")
        parts.append(MeshPart(mat, colours.get(mat, (0.7, 0.7, 0.7)), pts, p.reshape(-1, 3)))
    return parts


def mat_to_usd(T: np.ndarray) -> list[list[float]]:
    """USD stores transforms for row vectors (p' = p M): the transpose of ours, row by row."""
    return [[float(x) for x in row] for row in np.asarray(T, float).T]


# A USD camera looks along its -z with +y up; rowpilot cameras look along +x of an FLU link.
CAM_LINK_TO_USD = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])  # columns: usd x, y, z in link


def camera_matrix(pose: tuple[float, float, float, float, float, float]) -> np.ndarray:
    """4x4 of a USD camera in the body frame, for a rowpilot Camera.pose (body FLU)."""
    T = np.eye(4)
    T[:3, :3] = _rot(*pose[3:]) @ CAM_LINK_TO_USD
    T[:3, 3] = pose[:3]
    return T


def camera_apertures(width: int, height: int, hfov_deg: float, focal_mm: float = 1.93) -> tuple[float, float]:
    """USD horizontal and vertical aperture (mm) for a pinhole with this field of view and square pixels."""
    h = 2.0 * focal_mm * np.tan(np.radians(hfov_deg) / 2)
    return float(h), float(h * height / width)


# ---- Isaac Sim's look: NVIDIA MDL materials for the greenhouse ---------------------------------
# Gazebo's world has plain colours. In Isaac Sim each part gets a physically based material from the
# MDL library that ships with it (OmniPBR, OmniGlass): what it is made of, not only its colour.


@dataclass(frozen=True)
class Look:
    module: str  # MDL module and material: "OmniPBR" or "OmniGlass"
    name: str  # one USD material per name
    inputs: tuple  # ((input name, value), ...): float, bool, (r, g, b), or a texture path (str)

    def get(self, key: str, default=None):
        return dict(self.inputs).get(key, default)


def _close(rgb, ref, tol: float = 0.02) -> bool:
    return all(abs(a - b) <= tol for a, b in zip(rgb, ref, strict=True))


def look_for(s: Shape, floor_normal: str = "") -> Look:
    """The material a greenhouse shape gets in Isaac Sim, from its model and colour in the generated world
    (gazebo_world.py: crop rows, pipe rails, posts, sticky tape, floor, glass facades)."""
    m, rgb = s.model, tuple(round(float(c), 3) for c in s.rgb)
    if s.texture:  # the floor's ground cover
        inputs = [("diffuse_texture", s.texture), ("reflection_roughness_constant", 0.88), ("diffuse_tint", (0.8,) * 3)]
        if floor_normal:
            inputs += [("normalmap_texture", floor_normal), ("bump_factor", 0.8)]
        return Look("OmniPBR", "ground_cover", tuple(inputs))
    if m.startswith("pipe_"):  # heating pipes: polished steel
        return Look(
            "OmniPBR",
            "pipe_steel",
            (
                ("diffuse_color_constant", (0.78, 0.78, 0.8)),
                ("metallic_constant", 1.0),
                ("reflection_roughness_constant", 0.22),
            ),
        )
    if m.startswith("post_"):  # galvanised steel posts
        return Look(
            "OmniPBR",
            "galvanised_steel",
            (
                ("diffuse_color_constant", (0.66, 0.67, 0.69)),
                ("metallic_constant", 0.85),
                ("reflection_roughness_constant", 0.42),
            ),
        )
    if m.startswith("tape_"):  # yellow sticky tape: glossy plastic
        return Look("OmniPBR", "sticky_tape", (("diffuse_color_constant", rgb), ("reflection_roughness_constant", 0.3)))
    if m == "walls":
        if _close(rgb, (0.86, 0.88, 0.9)):  # the panes
            return Look(
                "OmniGlass",
                "greenhouse_glass",
                (
                    ("glass_color", (0.93, 0.97, 0.96)),
                    ("glass_ior", 1.52),
                    ("thin_walled", True),
                    ("frosting_roughness", 0.04),
                ),
            )
        if _close(rgb, (0.55, 0.55, 0.53)):  # the kerb
            return Look(
                "OmniPBR",
                "concrete",
                (("diffuse_color_constant", (0.58, 0.57, 0.55)), ("reflection_roughness_constant", 0.95)),
            )
        return Look(
            "OmniPBR",
            f"aluminium_{rgb[0]:.2f}",
            (("diffuse_color_constant", rgb), ("metallic_constant", 0.7), ("reflection_roughness_constant", 0.38)),
        )
    if m == "ground_plane":
        return Look(
            "OmniPBR",
            "concrete",
            (("diffuse_color_constant", (0.58, 0.57, 0.55)), ("reflection_roughness_constant", 0.95)),
        )
    return Look(
        "OmniPBR",
        f"plain_{'_'.join(f'{c:.2f}' for c in rgb)}",
        (("diffuse_color_constant", rgb), ("reflection_roughness_constant", 0.7)),
    )
