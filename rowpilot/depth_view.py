"""Simulation display only: the D435i depth image, colour-coded, for a live screen in the Gazebo window.

Gazebo's ImageDisplay panel cannot draw float depth images, so this republishes /<model>/depth as an
RGB image on /<model>/depth_view: blue = near (0.3 m), red = far (6 m), black = no depth.

    python -m rowpilot.depth_view [--model gh_firefly]
"""

from __future__ import annotations

import argparse
import time

import numpy as np

NEAR_M, FAR_M = 0.3, 6.0


def colourise(depth: np.ndarray, near: float = NEAR_M, far: float = FAR_M) -> np.ndarray:
    """Depth (m) -> RGB uint8 (h, w, 3), a blue-green-yellow-red ramp; no depth is black."""
    ok = np.isfinite(depth) & (depth >= near) & (depth <= far)
    u = np.clip((np.where(ok, depth, far) - near) / (far - near), 0.0, 1.0)
    # piecewise-linear "jet"-like ramp: 0 blue, 0.33 cyan-green, 0.66 yellow, 1 red
    r = np.clip(1.5 * u * 2 - 1.0, 0, 1)
    g = np.clip(1.5 - np.abs(3 * u - 1.5), 0, 1)
    b = np.clip(1.0 - 1.5 * u * 2 + 0.5, 0, 1)
    rgb = (np.stack([r, g, b], axis=-1) * 255).astype(np.uint8)
    rgb[~ok] = 0
    return rgb


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--model", default="gh_firefly")
    a = ap.parse_args(argv)

    from gz.msgs10.image_pb2 import Image
    from gz.transport13 import Node

    node = Node()
    pub = node.advertise(f"/{a.model}/depth_view", Image)

    def on_depth(m) -> None:
        d = np.frombuffer(m.data, dtype=np.float32).reshape(m.height, m.width)
        o = Image()
        o.header.CopyFrom(m.header)
        o.width, o.height, o.step = m.width, m.height, 3 * m.width
        o.pixel_format_type = 3  # RGB_INT8
        o.data = colourise(d).tobytes()
        pub.publish(o)

    node.subscribe(Image, f"/{a.model}/depth", on_depth)
    print(f"[depth_view] /{a.model}/depth -> /{a.model}/depth_view", flush=True)
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
