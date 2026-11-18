"""Bring a SolidWorks drone design (sw_urdf_exporter output) into the simulation.

The SolidWorks URDF exporter writes one STL per link in the link's frame. The body mesh of a full
assembly is far too detailed for a simulator (FireFly 2: 620k triangles, 31 MB), so it is simplified
by vertex clustering: vertices in the same grid cell are merged and triangles that collapse are
dropped. That keeps the outline and the parts the cameras could see, at a fraction of the size.
Only the look comes from the mesh: collisions use a box, mass and inertia are set in airframe.py.

    python -m rowpilot.cad_import "<...>/Drone V3.2 D435.SLDASM" ardupilot/firefly/meshes
"""

from __future__ import annotations

import argparse
import struct
from pathlib import Path

import numpy as np

# sw_urdf_exporter link mesh -> file name in the model, for the FireFly 2 export
FIREFLY_MESHES = {
    "base_link.STL": "body.stl",
    "FrontRightArm.STL": "prop_front_right.stl",
    "FrontLeftArm.STL": "prop_front_left.stl",
    "BackRightArm.STL": "prop_back_right.stl",
    "BackLeftArm.STL": "prop_back_left.stl",
}


def read_stl(path: Path) -> np.ndarray:
    """Triangles of a binary or ASCII STL, shape (n, 3, 3)."""
    b = path.read_bytes()
    if len(b) >= 84:
        n = struct.unpack("<I", b[80:84])[0]
        if 84 + 50 * n == len(b):
            rec = np.dtype([("normal", "<3f4"), ("v", "<9f4"), ("attr", "<u2")])
            return np.frombuffer(b, dtype=rec, count=n, offset=84)["v"].reshape(-1, 3, 3).astype(float)
    words = [line.split() for line in b.decode(errors="ignore").splitlines()]
    v = np.array([[float(x) for x in w[1:4]] for w in words if w and w[0] == "vertex"])
    return v.reshape(-1, 3, 3)


def write_stl(path: Path, tris: np.ndarray, name: str = "rowpilot") -> None:
    """Binary STL with face normals."""
    tris = np.asarray(tris, dtype=np.float32)
    nrm = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-12)
    rec = np.zeros(len(tris), dtype=np.dtype([("normal", "<3f4"), ("v", "<9f4"), ("attr", "<u2")]))
    rec["normal"], rec["v"] = nrm, tris.reshape(-1, 9)
    path.write_bytes(name.encode()[:80].ljust(80, b" ") + struct.pack("<I", len(tris)) + rec.tobytes())


def simplify(tris: np.ndarray, cell: float) -> np.ndarray:
    """Vertex clustering: snap every vertex to the mean of the vertices in its grid cell (size `cell`,
    metres) and drop triangles that collapse or repeat."""
    v = tris.reshape(-1, 3)
    key = np.floor(v / cell).astype(np.int64)
    _, cluster, counts = np.unique(key, axis=0, return_inverse=True, return_counts=True)
    cluster = cluster.reshape(-1)
    centre = np.zeros((len(counts), 3))
    np.add.at(centre, cluster, v)
    centre /= counts[:, None]
    f = cluster.reshape(-1, 3)
    ok = (f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])
    f = f[ok]
    _, first = np.unique(np.sort(f, axis=1), axis=0, return_index=True)
    return centre[f[np.sort(first)]]


def import_meshes(export_dir: Path, out_dir: Path, body_cell: float = 0.002) -> dict[str, int]:
    """Simplified body and the four props from a FireFly sw_urdf_exporter folder. Returns triangle counts."""
    out_dir.mkdir(parents=True, exist_ok=True)
    counts = {}
    for src, dst in FIREFLY_MESHES.items():
        tris = read_stl(export_dir / "meshes" / src)
        tris = simplify(tris, body_cell if src == "base_link.STL" else body_cell / 2)
        write_stl(out_dir / dst, tris, f"FireFly 2 {src}")
        counts[dst] = len(tris)
    return counts


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("export_dir", type=Path, help="sw_urdf_exporter folder (has meshes/ and urdf/)")
    ap.add_argument("out_dir", type=Path)
    ap.add_argument("--cell", type=float, default=0.002, help="body simplification grid (m)")
    a = ap.parse_args(argv)
    for name, n in import_meshes(a.export_dir, a.out_dir, a.cell).items():
        print(f"{name}: {n} triangles")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
