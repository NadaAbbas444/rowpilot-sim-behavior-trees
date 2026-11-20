"""Isaac Sim in place of Gazebo: the greenhouse, the FireFly 2 and its sensors, lock-stepped with
ArduCopter SITL over ArduPilot's JSON interface (the protocol ardupilot_gazebo's plugin speaks).

    <isaac python> isaac/greenhouse_isaac.py --gz-dir build/gz [--headless]

Run by scripts/run_ardupilot_sim.sh with SIM=isaac; it needs the resources ardupilot/install.sh
writes into build/gz (the greenhouse world, plant meshes, floor texture) and rowpilot.isaac_relay
listening on 127.0.0.1:9100 (it republishes the sensors on gz-transport for OpenVINS and the tree).

What it does, every physics step (1 ms, one per ArduPilot servo packet):
  1. rotor forces from ArduPilot's PWM (rowpilot.isaac_link.RotorThrust), applied to the body;
  2. one PhysX step;
  3. the state back to ArduPilot (position, attitude, velocity, gyro, accelerometer);
  4. the D435i IMU at 200 Hz, the clock and the ground-truth pose to the relay;
  5. at 30 Hz: a render, and the D435i infrared pair (30 Hz), depth (15 Hz) and Pi Camera (2 Hz).

Written for Isaac Sim 4.5 (pip package isaacsim==4.5.0, Python 3.10, set up by
scripts/setup_isaac_sim.sh), with fallbacks to the 4.0-4.2 module names. Not yet flown: the protocol,
rotor, IMU and scene code it uses is tested (tests/test_isaac_*.py); the Isaac Sim calls are not.
"""

from __future__ import annotations

import argparse
import dataclasses
import math
import socket
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

# NVIDIA's sky library (the content Isaac Sim's own samples use): a clear noon over grass, seen through
# the greenhouse glass. Isaac Sim reads it straight from the server and caches it.
SKY_DEFAULT = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Skies/Clear/noon_grass_4k.hdr"

ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
ap.add_argument("--gz-dir", type=Path, default=REPO / "build" / "gz", help="what ardupilot/install.sh wrote")
ap.add_argument("--world", default="greenhouse")
ap.add_argument("--model", default="gh_firefly", help="topic prefix, as the Gazebo model's name")
ap.add_argument("--fdm-port", type=int, default=9002, help="UDP port ArduPilot SITL (--model JSON) sends to")
ap.add_argument("--relay-port", type=int, default=9100)
ap.add_argument("--headless", action="store_true")
ap.add_argument(
    "--chase", type=int, default=640, help="width of the chase view streamed for rowpilot.isaac_view; 0: none"
)
ap.add_argument(
    "--look",
    choices=("isaac", "gazebo"),
    default="isaac",
    help="isaac: NVIDIA MDL materials, sky, detailed plants; gazebo: the Gazebo world's plain look",
)
ap.add_argument(
    "--quality",
    choices=("fast", "high"),
    default="fast",
    help="isaac look: fast for flights; high adds global illumination (for stills and videos)",
)
ap.add_argument("--sky", default=SKY_DEFAULT, help="HDRI sky for the dome light (URL or file); '' for none")
ap.add_argument("--sun", type=float, default=None, help="sun intensity (default: 3200 isaac look, 2500 gazebo)")
ap.add_argument("--sky-intensity", type=float, default=None, help="dome light intensity (default: 900 / 700)")
ap.add_argument("--preview", type=Path, help="render stills of the scene into this folder and exit (no ArduPilot)")
ap.add_argument("--preview-pose", type=float, nargs=4, default=(2.0, 0.0, 1.5, 0.0), metavar=("X", "Y", "Z", "YAW"))
ap.add_argument(
    "--picam",
    action="store_true",
    help="render the Pi Camera too (1280x720, a quarter of the render time; nothing in the flight uses it)",
)
ap.add_argument("--no-plants", action="store_true", help="crop rows as green boxes (quicker to render)")
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402  (must start before any omni / pxr import)

app = SimulationApp({"headless": args.headless, "renderer": "RayTracedLighting", "width": 1280, "height": 720})

import carb  # noqa: E402
import numpy as np  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
from pxr import Gf, PhysxSchema, Sdf, Tf, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt  # noqa: E402

try:
    from isaacsim.core.api import World
    from isaacsim.core.prims import RigidPrim as _RigidView
except ImportError:  # Isaac Sim 4.0-4.2 names
    from omni.isaac.core import World
    from omni.isaac.core.prims import RigidPrimView as _RigidView

from rowpilot.airframe import FIREFLY_RIG, FireFly  # noqa: E402
from rowpilot.cad_import import read_stl  # noqa: E402
from rowpilot.greenhouse_assets import MATERIALS  # noqa: E402
from rowpilot.imu_synth import GRAVITY  # noqa: E402
from rowpilot.isaac_link import (  # noqa: E402
    ImuSampler,
    RotorThrust,
    body_specific_force,
    fdm_reply,
    image_frame,
    pack_frame,
    parse_servo_packet,
    to_gray,
)
from rowpilot.isaac_plants import detailed_plant, read_plant_index, write_textures  # noqa: E402
from rowpilot.isaac_scene import (  # noqa: E402
    Look,
    Shape,
    camera_apertures,
    camera_matrix,
    look_for,
    read_dae,
    read_world,
    resolve_uri,
)
from rowpilot.layout import GreenhouseLayout  # noqa: E402
from rowpilot.sensors import _rot  # noqa: E402
from rowpilot.vio_link import quat_to_rot, rot_to_quat  # noqa: E402

DT = 0.001  # physics step = ArduPilot's IMU period (it needs >= 720 Hz, see world_template.sdf)
FRAME_HZ = 30.0  # renders per simulated second: the infrared rate
CLOCK_EVERY, POSE_EVERY = 4, 20  # physics steps
CHASE_OFFSET = (-1.6, -0.5, 0.9)  # view camera behind the drone (its heading frame), as the Gazebo window's
CHASE_HZ = 10.0

settings = carb.settings.get_settings()
# Render synchronously, so an annotator's image is the frame just rendered (not one or two earlier):
# OpenVINS pairs the images with the IMU by their stamps. (Untested: check the image lag on first run.)
settings.set("/omni/replicator/asyncRendering", False)
settings.set("/app/asyncRendering", False)
settings.set("/app/asyncRenderingLowLatency", False)
ISAAC_LOOK = args.look == "isaac"
if ISAAC_LOOK:  # RTX real-time effects the MDL materials need to look right
    for key in ("ambientOcclusion", "reflections", "translucency"):
        settings.set(f"/rtx/{key}/enabled", True)
    # Global illumination (light bounced off the floor and leaves) is the costliest effect: --quality high.
    settings.set("/rtx/indirectDiffuse/enabled", args.quality == "high")

world = World(stage_units_in_meters=1.0, physics_dt=DT, rendering_dt=DT)
stage = world.stage
physics = world.get_physics_context()
physics.set_gravity(float(GRAVITY[2]))  # -9.8, Gazebo's default
physics.enable_gpu_dynamics(False)  # one small body: the CPU solver is the right one
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

# ---- helpers ---------------------------------------------------------------------------------


def quatf(R) -> Gf.Quatf:
    w, x, y, z = rot_to_quat(np.asarray(R, float))
    return Gf.Quatf(float(w), float(x), float(y), float(z))


def place(prim, T, scale=(1.0, 1.0, 1.0)) -> None:
    x = UsdGeom.Xformable(prim)
    x.ClearXformOpOrder()
    x.AddTranslateOp().Set(Gf.Vec3d(*(float(v) for v in T[:3, 3])))
    x.AddOrientOp().Set(quatf(T[:3, :3]))
    if tuple(scale) != (1.0, 1.0, 1.0):
        x.AddScaleOp().Set(Gf.Vec3f(*(float(v) for v in scale)))


_materials: dict[tuple, UsdShade.Material] = {}


def material(rgb, texture: Path | None = None, roughness: float = 0.7) -> UsdShade.Material:
    key = (tuple(round(c, 3) for c in rgb), str(texture or ""))
    if key in _materials:
        return _materials[key]
    path = f"/World/Looks/m{len(_materials)}"
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/surface")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)  # not metallic, as in the Gazebo world
    if texture:
        st = UsdShade.Shader.Define(stage, path + "/st")
        st.CreateIdAttr("UsdPrimvarReader_float2")
        st.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        tex = UsdShade.Shader.Define(stage, path + "/tex")
        tex.CreateIdAttr("UsdUVTexture")
        tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(str(texture))
        tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st.ConnectableAPI(), "result")
        tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
    else:
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*(float(c) for c in rgb)))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    _materials[key] = mat
    return mat


def bind(prim, mat) -> None:
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)


_mdl: dict[str, UsdShade.Material] = {}
_MDL_TYPES = {bool: Sdf.ValueTypeNames.Bool, float: Sdf.ValueTypeNames.Float, int: Sdf.ValueTypeNames.Int}


def mdl(look: Look) -> UsdShade.Material:
    """A material from NVIDIA's MDL library (OmniPBR.mdl, OmniGlass.mdl ship with Isaac Sim)."""
    if look.name in _mdl:
        return _mdl[look.name]
    path = f"/World/Looks/{Tf.MakeValidIdentifier(look.name)}"
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/Shader")
    sh.SetSourceAsset(Sdf.AssetPath(f"{look.module}.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier(look.module, "mdl")
    for key, value in look.inputs:
        if isinstance(value, str):
            sh.CreateInput(key, Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(value))
        elif isinstance(value, tuple) and len(value) == 3:
            sh.CreateInput(key, Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*(float(v) for v in value)))
        elif isinstance(value, tuple) and len(value) == 2:
            sh.CreateInput(key, Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(*(float(v) for v in value)))
        else:
            sh.CreateInput(key, _MDL_TYPES[type(value)]).Set(value)
    out = sh.CreateOutput("out", Sdf.ValueTypeNames.Token)
    for output in (mat.CreateSurfaceOutput("mdl"), mat.CreateDisplacementOutput("mdl"), mat.CreateVolumeOutput("mdl")):
        output.ConnectToSource(out)
    _mdl[look.name] = mat
    return mat


def look_material(s: Shape, texture: Path | None = None) -> UsdShade.Material:
    """The material for a greenhouse shape: Isaac Sim's look (MDL), or the Gazebo world's plain one."""
    if ISAAC_LOOK:
        shape = s if texture is None else dataclasses.replace(s, texture=str(texture))
        return mdl(look_for(shape, str(textures.get("floor_normal", ""))))
    return material((1, 1, 1) if texture else s.rgb, texture, roughness=0.9 if texture else 0.7)


def mesh(path: str, points: np.ndarray, tris: np.ndarray, rgb) -> UsdGeom.Mesh:
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(points, np.float32)))
    m.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(tris)))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray(np.asarray(tris, np.int32).reshape(-1).tolist()))
    m.CreateSubdivisionSchemeAttr("none")
    m.CreateDoubleSidedAttr(True)
    bind(m.GetPrim(), material(rgb))
    return m


def textured_quad(path: str, sx: float, sy: float, z: float, texture: Path) -> UsdGeom.Mesh:
    """The top face of a textured box (UsdGeom.Cube has no texture coordinates)."""
    m = UsdGeom.Mesh.Define(stage, path)
    hx, hy = sx / 2, sy / 2
    m.CreatePointsAttr([Gf.Vec3f(-hx, -hy, z), Gf.Vec3f(hx, -hy, z), Gf.Vec3f(hx, hy, z), Gf.Vec3f(-hx, hy, z)])
    m.CreateFaceVertexCountsAttr([4])
    m.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    m.CreateSubdivisionSchemeAttr("none")
    st = UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    st.Set([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])
    return m


def unit_cube(stage, path: str) -> UsdGeom.Cube:
    """A 1 m cube, sized by its scale (USD's default cube is 2 m on a side)."""
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    return c


def collider(prim, static_friction: float = 0.8) -> None:
    UsdPhysics.CollisionAPI.Apply(prim)
    PhysxSchema.PhysxCollisionAPI.Apply(prim).CreateContactOffsetAttr(0.005)


# ---- the greenhouse --------------------------------------------------------------------------

gz_models = [args.gz_dir / "models"]
sdf = (args.gz_dir / "worlds" / "greenhouse.sdf").read_text()
gh = read_world(sdf)
assets_meshes = args.gz_dir / "models" / "greenhouse_assets" / "meshes"
textures = (
    {k: v.resolve() for k, v in write_textures(args.gz_dir / "isaac_textures", assets_meshes / "floor.png").items()}
    if ISAAC_LOOK
    else {}
)
UsdGeom.Xform.Define(stage, "/World/greenhouse")
mesh_cache: dict[str, str] = {}  # uri -> prototype path
instances: dict[str, list[Shape]] = {}
n_prims = 0

for i, s in enumerate(gh.shapes):
    path = f"/World/greenhouse/{s.model}_{'v' if s.visual else 'c'}{i}"
    if s.kind == "mesh":
        if s.visual and not args.no_plants:
            instances.setdefault(s.uri, []).append(s)
        continue
    if s.kind == "plane":
        if s.visual:
            m = UsdGeom.Mesh.Define(stage, path)
            hx, hy = s.size[0] / 2, s.size[1] / 2
            m.CreatePointsAttr([Gf.Vec3f(-hx, -hy, 0), Gf.Vec3f(hx, -hy, 0), Gf.Vec3f(hx, hy, 0), Gf.Vec3f(-hx, hy, 0)])
            m.CreateFaceVertexCountsAttr([4])
            m.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
            bind(m.GetPrim(), look_material(s))
            place(m.GetPrim(), s.T)
        else:  # the ground: a thick static box, its top at the plane
            c = unit_cube(stage, path)
            T = s.T.copy()
            T[:3, 3] -= T[:3, :3] @ [0, 0, 0.5]
            place(c.GetPrim(), T, (s.size[0], s.size[1], 1.0))
            c.MakeInvisible()
            collider(c.GetPrim())
        n_prims += 1
        continue
    if s.visual and s.texture and s.kind == "box":
        tex = resolve_uri(s.texture, gz_models)
        if tex is not None:
            q = textured_quad(path, s.size[0], s.size[1], s.size[2] / 2, tex)
            bind(q.GetPrim(), look_material(s, tex.resolve()))
            place(q.GetPrim(), s.T)
            n_prims += 1
            continue
    if s.kind == "box":
        g = unit_cube(stage, path)
        place(g.GetPrim(), s.T, s.size)
    elif s.kind == "cylinder":
        g = UsdGeom.Cylinder.Define(stage, path)
        g.CreateRadiusAttr(s.radius)
        g.CreateHeightAttr(s.length)
        g.CreateAxisAttr("Z")
        place(g.GetPrim(), s.T)
    else:
        continue
    if s.visual:
        bind(g.GetPrim(), look_material(s))
    else:
        g.MakeInvisible()
        collider(g.GetPrim())
    n_prims += 1

# Plants: one PointInstancer, the meshes greenhouse_assets.py wrote as prototypes.
plant_index = read_plant_index(assets_meshes)


def plant_material(name: str) -> UsdShade.Material:
    """Leaves: the leaflet texture (cut out by its alpha) tinted dark, mid or light; glossy fruit."""
    rgb = MATERIALS[name]
    if name.startswith("leaf_"):
        tint = tuple(min(1.0, c / 0.78) for c in rgb)
        return mdl(
            Look(
                "OmniPBR",
                f"tomato_{name}",
                (
                    ("diffuse_texture", str(textures["leaf"])),
                    ("diffuse_tint", tint),
                    ("normalmap_texture", str(textures["leaf_normal"])),
                    ("bump_factor", 0.5),
                    ("enable_opacity", True),
                    ("enable_opacity_texture", True),
                    ("opacity_texture", str(textures["leaf"])),
                    ("opacity_mode", 0),
                    ("opacity_threshold", 0.5),
                    ("reflection_roughness_constant", 0.55),
                ),
            )
        )
    if name.startswith("fruit_"):
        return mdl(
            Look(
                "OmniPBR", f"tomato_{name}", (("diffuse_color_constant", rgb), ("reflection_roughness_constant", 0.18))
            )
        )
    return mdl(
        Look("OmniPBR", "tomato_stem", (("diffuse_color_constant", rgb), ("reflection_roughness_constant", 0.6)))
    )


def detailed_mesh(path, points, normals, uvs, tris, mat) -> None:
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(points))
    m.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(normals))
    m.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    m.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(tris)))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray(tris.reshape(-1).tolist()))
    m.CreateSubdivisionSchemeAttr("none")
    m.CreateDoubleSidedAttr(True)
    st = UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    st.Set(Vt.Vec2fArray.FromNumpy(uvs))
    bind(m.GetPrim(), mat)


if instances:
    inst = UsdGeom.PointInstancer.Define(stage, "/World/greenhouse/plants")
    protos, pos, ori, scl, idx = [], [], [], [], []
    for k, (uri, shapes) in enumerate(sorted(instances.items())):
        f = resolve_uri(uri, gz_models)
        if f is None:
            print(f"[isaac] mesh not found: {uri}", flush=True)
            continue
        proto = f"/World/greenhouse/plants/protos/p{k}"
        UsdGeom.Xform.Define(stage, proto)
        plant = plant_index.get(f.name)
        if ISAAC_LOOK and plant:  # the same plant, in detail (isaac_plants.py)
            for mat_name, data in detailed_plant(plant["seed"], plant["height"]).items():
                detailed_mesh(f"{proto}/{mat_name}", *data.arrays(), plant_material(mat_name))
        else:
            for j, part in enumerate(read_dae(f.read_text())):
                mesh(f"{proto}/{part.material or j}", part.points, part.triangles, part.rgb)
        protos.append(Sdf.Path(proto))
        for s in shapes:
            w, x, y, z = rot_to_quat(s.T[:3, :3])
            pos.append(Gf.Vec3f(*(float(v) for v in s.T[:3, 3])))
            ori.append(Gf.Quath(float(w), float(x), float(y), float(z)))
            scl.append(Gf.Vec3f(*(float(v) for v in s.scale)))
            idx.append(len(protos) - 1)
    inst.CreatePrototypesRel().SetTargets(protos)
    inst.CreatePositionsAttr(Vt.Vec3fArray(pos))
    inst.CreateOrientationsAttr(Vt.QuathArray(ori))
    inst.CreateScalesAttr(Vt.Vec3fArray(scl))
    inst.CreateProtoIndicesAttr(Vt.IntArray(idx))
    print(f"[isaac] {len(idx)} plants from {len(protos)} meshes", flush=True)
elif not args.no_plants:
    print("[isaac] the world has no plant meshes: crop rows are their collision boxes only", flush=True)

if args.no_plants or not instances:  # show the crop rows as green boxes
    for s in gh.colliders():
        if s.model.startswith("crop_row_") and s.kind == "box":
            g = unit_cube(stage, f"/World/greenhouse/{s.model}_box")
            place(g.GetPrim(), s.T, s.size)
            bind(g.GetPrim(), material((0.30, 0.55, 0.20)) if not ISAAC_LOOK else plant_material("leaf_mid"))

# Light: the world's sun, and a sky. Isaac look: NVIDIA's HDRI sky lights the scene and is seen through the
# glass; a warm sun with the sun's real angular size (0.53 deg) gives crisp shadows. Gazebo look: a plain
# dome for the ambient light (Gazebo's ambient 0.6).
sun = UsdLux.DistantLight.Define(stage, "/World/sun")
sun.CreateIntensityAttr(args.sun if args.sun is not None else (3200.0 if ISAAC_LOOK else 2500.0))
sun.CreateAngleAttr(0.53 if ISAAC_LOOK else 1.0)
if ISAAC_LOOK:
    sun.CreateColorAttr(Gf.Vec3f(1.0, 0.96, 0.9))
d = -np.asarray(gh.sun_direction, float)
d /= np.linalg.norm(d)  # a distant light shines along its -z: point its +z at the sun
zx = np.cross([0, 0, 1], d)
R_sun = np.eye(3) if np.linalg.norm(zx) < 1e-9 else None
if R_sun is None:
    ax = zx / np.linalg.norm(zx)
    ang = math.acos(float(np.clip(d[2], -1, 1)))
    K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    R_sun = np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K
T_sun = np.eye(4)
T_sun[:3, :3] = R_sun
place(sun.GetPrim(), T_sun)
sky = UsdLux.DomeLight.Define(stage, "/World/sky")
sky.CreateIntensityAttr(args.sky_intensity if args.sky_intensity is not None else (900.0 if ISAAC_LOOK else 700.0))
if ISAAC_LOOK and args.sky:
    sky.CreateTextureFileAttr(Sdf.AssetPath(args.sky))
    sky.CreateTextureFormatAttr(UsdLux.Tokens.latlong)
print(f"[isaac] greenhouse: {n_prims} shapes", flush=True)

# ---- the FireFly 2 ---------------------------------------------------------------------------

ff, rig = FireFly(), FIREFLY_RIG
# Start where the world places the drone (home); the default layout's home if it places none.
T0 = next((T.copy() for uri, T in gh.includes if uri.rstrip("/").endswith(args.model)), None)
if T0 is None:
    T0 = np.eye(4)
    T0[:2, 3] = GreenhouseLayout().home
T0[2, 3] = 0.02
body_path = f"/World/{args.model}"
body = UsdGeom.Xform.Define(stage, body_path)
place(body.GetPrim(), T0)
UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
mass = UsdPhysics.MassAPI.Apply(body.GetPrim())
mass.CreateMassAttr(ff.total_mass)
mass.CreateCenterOfMassAttr(Gf.Vec3f(*ff.com))
mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*ff.inertia))
PhysxSchema.PhysxRigidBodyAPI.Apply(body.GetPrim()).CreateSleepThresholdAttr(0.0)  # never sleep on the floor
box = unit_cube(stage, body_path + "/collision")
Tc = np.eye(4)
Tc[2, 3] = ff.size[2] / 2
place(box.GetPrim(), Tc, ff.size)
box.MakeInvisible()
collider(box.GetPrim())
pm = UsdShade.Material.Define(stage, "/World/Looks/drone_contact")
pmat = UsdPhysics.MaterialAPI.Apply(pm.GetPrim())
pmat.CreateStaticFrictionAttr(1.0)
pmat.CreateDynamicFrictionAttr(1.0)
pmat.CreateRestitutionAttr(0.0)
UsdShade.MaterialBindingAPI.Apply(box.GetPrim()).Bind(pm, UsdShade.Tokens.weakerThanDescendants, "physics")

meshes = REPO / "ardupilot" / "firefly" / "meshes"
tris = read_stl(meshes / "body.stl")
mesh(body_path + "/body", tris.reshape(-1, 3), np.arange(len(tris) * 3).reshape(-1, 3), (0.82, 0.82, 0.8))
for r in ff.rotors:
    tris = read_stl(meshes / r.mesh)
    pr = UsdGeom.Xform.Define(stage, f"{body_path}/{r.name}")
    Tr = np.eye(4)
    Tr[:3, :3], Tr[:3, 3] = _rot(*r.mesh_rpy), r.xyz
    place(pr.GetPrim(), Tr)
    mesh(f"{body_path}/{r.name}/prop", tris.reshape(-1, 3), np.arange(len(tris) * 3).reshape(-1, 3), (0.15, 0.15, 0.15))

# Cameras, at the rig's poses on the body (the D435i's are on the tilted sensor link, as in Gazebo).
d435 = rig.d435i
# Every camera renders every frame (switching a camera's rendering off and on again lost its images), so a
# camera costs render time whether its frame is due or not: the Pi Camera only renders with --picam.
sensors = [d435.ir("left"), d435.ir("right"), d435.depth()] + ([rig.picam.camera()] if args.picam else [])
cams = {c.name: c for c in sensors}
products, annot = {}, {}
for name, c in cams.items():
    cp = f"{body_path}/{name}"
    cam = UsdGeom.Camera.Define(stage, cp)
    focal = 1.93
    h, v = camera_apertures(c.width, c.height, c.hfov_deg, focal)
    cam.CreateFocalLengthAttr(focal)
    cam.CreateHorizontalApertureAttr(h)
    cam.CreateVerticalApertureAttr(v)
    cam.CreateClippingRangeAttr(Gf.Vec2f(c.near, c.far))
    place(cam.GetPrim(), camera_matrix(c.pose))
    products[name] = rep.create.render_product(cp, (c.width, c.height))
    a = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane" if c.kind == "depth_camera" else "rgb")
    a.attach([products[name]])
    annot[name] = a
# The view camera follows the drone's position and heading, not its roll and pitch (a level view, as
# Gazebo's follow camera). Streamed as /<model>/chase for rowpilot.isaac_view, and the viewport's camera
# when Isaac Sim has a window.
CHASE_PATH = "/World/chase"
chase = UsdGeom.Camera.Define(stage, CHASE_PATH)
chase.CreateFocalLengthAttr(1.93)
ch_h, ch_v = camera_apertures(16, 9, 80.0, 1.93)
chase.CreateHorizontalApertureAttr(ch_h)
chase.CreateVerticalApertureAttr(ch_v)
chase.CreateClippingRangeAttr(Gf.Vec2f(0.05, 100.0))
T_chase_body = camera_matrix((*CHASE_OFFSET, 0.0, 0.35, math.atan2(-CHASE_OFFSET[1], -CHASE_OFFSET[0]) * 0.5))


def follow(p: np.ndarray, R: np.ndarray) -> None:
    yaw = math.atan2(R[1, 0], R[0, 0])
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = _rot(0.0, 0.0, yaw), p
    place(chase.GetPrim(), T @ T_chase_body)


follow(T0[:3, 3], np.eye(3))
chase_annot = None
if args.chase > 0:
    chase_size = (args.chase, args.chase * 9 // 16)
    chase_annot = rep.AnnotatorRegistry.get_annotator("rgb")
    chase_annot.attach([rep.create.render_product(CHASE_PATH, chase_size)])
if not args.headless:
    try:
        from omni.kit.viewport.utility import get_active_viewport

        get_active_viewport().camera_path = CHASE_PATH
    except Exception as e:  # noqa: BLE001  (the window just keeps its default view)
        print(f"[isaac] could not follow the drone in the viewport: {e}", flush=True)


# ---- preview: stills of the scene, no ArduPilot ---------------------------------------------

if args.preview is not None:
    from PIL import Image

    from rowpilot.depth_view import colourise

    px, py, pz, pyaw = args.preview_pose
    Tp = np.eye(4)
    Tp[:3, :3], Tp[:3, 3] = _rot(0.0, 0.0, math.radians(pyaw)), (px, py, pz)
    place(body.GetPrim(), Tp)
    follow(Tp[:3, 3], Tp[:3, :3])
    world.reset()  # the timeline plays (annotators only deliver while it does); render() does not step physics
    for _ in range(60):  # shaders, textures and the sky load; RTX settles
        world.render()
    t_render = time.monotonic()
    for _ in range(10):
        world.render()
    ms = (time.monotonic() - t_render) * 100
    args.preview.mkdir(parents=True, exist_ok=True)
    stills = {**annot, **({"chase": chase_annot} if chase_annot is not None else {})}
    for name, a in stills.items():
        img = np.asarray(a.get_data())
        if name == "depth":
            img = colourise(img.astype(np.float32))
        elif name in cams and cams[name].pixel_format == "L8":
            img = to_gray(img)
        else:
            img = img[..., :3]
        Image.fromarray(np.ascontiguousarray(img).astype(np.uint8)).save(args.preview / f"{name}.png")
    print(f"[isaac] preview: {ms:.0f} ms per frame (all cameras), stills in {args.preview}", flush=True)
    app.close()
    sys.exit(0)

# ---- connections -----------------------------------------------------------------------------

relay = None
for _ in range(100):
    try:
        relay = socket.create_connection(("127.0.0.1", args.relay_port))
        relay.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        break
    except OSError:
        time.sleep(0.2)
if relay is None:
    print(f"[isaac] no relay on 127.0.0.1:{args.relay_port} (python -m rowpilot.isaac_relay)", flush=True)
    app.close()
    sys.exit(1)

fdm = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
fdm.bind(("127.0.0.1", args.fdm_port))
fdm.settimeout(0.05)

world.reset()
view = _RigidView(body_path)  # created after reset (physics playing), so it binds to the body
try:
    view.initialize()
except Exception:  # noqa: BLE001  (Isaac Sim 4.5's RigidPrim initializes itself)
    pass
rotors = RotorThrust(ff)
imu = ImuSampler(d435)
R_link = _rot(0.0, d435.tilt, 0.0)
p_link = np.array([d435.mount_x, 0.0, d435.mount_z])
com = np.array(ff.com)
rng = np.random.default_rng(0)
topic = {n: f"/{args.model}/{n}" for n in cams}
imu_topic = f"/{args.model}/d435i_imu_kin"
pose_topic, clock_topic = f"/world/{args.world}/pose/info", f"/world/{args.world}/clock"


def state() -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Body origin position, attitude (ENU <- FLU), origin velocity and angular velocity, world frame."""
    p, q = view.get_world_poses()
    v, w = view.get_linear_velocities(), view.get_angular_velocities()
    p, q, v, w = (np.asarray(a, float).reshape(-1)[:n] for a, n in ((p, 3), (q, 4), (v, 3), (w, 3)))
    R = quat_to_rot(*q)
    return p, R, v - np.cross(w, R @ com), w  # PhysX reports the centre of mass's velocity


def send(frame: bytes) -> None:
    relay.sendall(frame)


print(f"[isaac] waiting for ArduPilot SITL on UDP {args.fdm_port} (lock-step, {1 / DT:.0f} Hz)", flush=True)
t, step, last_count, reply, v_prev = 0.0, 0, None, b"", None
frame_k, next_frame = 0, 1.0 / FRAME_HZ
wall0, t_report = time.monotonic(), 10.0
while app.is_running():
    try:
        data, addr = fdm.recvfrom(1024)
    except TimeoutError:
        if not args.headless:
            world.render()  # keep the window responsive while ArduPilot is not running
        continue
    pkt = parse_servo_packet(data)
    if pkt is None:
        continue
    if pkt.frame_count == last_count:  # ArduPilot resent: answer again, do not step
        if reply:
            fdm.sendto(reply, addr)
        continue
    if last_count is not None and pkt.frame_count < last_count:
        print("[isaac] ArduPilot restarted", flush=True)
    last_count = pkt.frame_count

    p, R, v, w = state()
    force, torque = rotors.step(pkt.pwm, DT, R.T @ v)
    view.apply_forces_and_torques_at_pos(forces=force[None], torques=torque[None], is_global=False)
    world.step(render=False)
    t += DT
    step += 1

    p, R, v, w = state()
    acc = body_specific_force(v if v_prev is None else v_prev, v, DT, R)
    v_prev = v
    reply = fdm_reply(t, p, R, v, R.T @ w, acc)
    fdm.sendto(reply, addr)

    out = []
    for ts, g, a in imu.add(t, p + R @ p_link, R @ R_link, w):
        out.append(pack_frame({"k": "imu", "topic": imu_topic, "t": ts, "gyro": g.tolist(), "acc": a.tolist()}))
    if step % CLOCK_EVERY == 0:
        out.append(pack_frame({"k": "clock", "topic": clock_topic, "t": t}))
    if step % POSE_EVERY == 0:
        qw, qx, qy, qz = rot_to_quat(R)
        out.append(
            pack_frame(
                {"k": "pose", "topic": pose_topic, "t": t, "poses": [[args.model, *map(float, p), qw, qx, qy, qz]]}
            )
        )
    if out:
        send(b"".join(out))

    if t >= next_frame - 1e-9:  # a camera frame is due
        frame_k += 1
        next_frame = (frame_k + 1) / FRAME_HZ
        due = {n: (frame_k % max(1, round(FRAME_HZ / c.rate_hz)) == 0) for n, c in cams.items()}
        follow(p, R)
        world.render()
        out = []
        if chase_annot is not None and frame_k % round(FRAME_HZ / CHASE_HZ) == 0:
            img = chase_annot.get_data()
            if img is not None and getattr(img, "size", 0):
                out.append(image_frame(f"/{args.model}/chase", t, np.asarray(img)[..., :3], "RGB8"))
        for n, on in due.items():
            if not on:
                continue
            img = annot[n].get_data()
            if img is None or getattr(img, "size", 0) == 0:
                continue
            c = cams[n]
            if c.kind == "depth_camera":
                dimg = np.asarray(img, np.float32).copy()
                dimg += rng.normal(0.0, c.noise_std, dimg.shape).astype(np.float32)
                dimg[~np.isfinite(dimg) | (dimg < c.near) | (dimg > c.far)] = np.inf
                out.append(image_frame(topic[n], t, dimg, "F32"))
            elif c.pixel_format == "L8":
                out.append(image_frame(topic[n], t, to_gray(np.asarray(img)), "L8"))
            else:
                out.append(image_frame(topic[n], t, np.asarray(img)[..., :3], "RGB8"))
        if out:
            send(b"".join(out))

    if t >= t_report:
        t_report += 10.0
        rtf = t / max(time.monotonic() - wall0, 1e-6)
        print(f"[isaac] t={t:6.1f}s  {rtf:.2f}x real time  z={p[2]:.2f} m", flush=True)

fdm.close()
relay.close()
app.close()
