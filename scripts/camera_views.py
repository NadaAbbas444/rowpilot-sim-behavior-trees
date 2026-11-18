"""Save what the drone's cameras see, from a running Gazebo simulation.

    PYTHONNOUSERSITE=1 GZ_IP=127.0.0.1 .venv-sim/bin/python scripts/camera_views.py [--model gh_firefly]

Writes <out>/<prefix>infra1.png (D435i left infrared, what OpenVINS tracks), <prefix>depth.png (D435i
depth, colour-coded 0.3-6 m, what the lane perception uses) and <prefix>picam.png (Pi Camera Module 3,
the side view of the crop row), plus <prefix>views.png with all three side by side.
"""

from __future__ import annotations

import argparse
import threading
import time
from pathlib import Path

import numpy as np


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--model", default="gh_firefly")
    ap.add_argument("--out", type=Path, default=Path("docs/views"))
    ap.add_argument("--prefix", default="", help="file name prefix, e.g. lane-")
    ap.add_argument("--timeout", type=float, default=10.0)
    a = ap.parse_args()

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from gz.msgs10.image_pb2 import Image
    from gz.transport13 import Node

    got: dict[str, np.ndarray] = {}
    lock = threading.Lock()

    def grab(name: str):
        def cb(m) -> None:
            if m.pixel_format_type == 13:  # R_FLOAT32 (depth)
                img = np.frombuffer(m.data, dtype=np.float32).reshape(m.height, m.width)
            else:
                ch = len(m.data) // (m.height * m.width)
                img = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, ch).squeeze()
            with lock:
                got[name] = img.copy()

        return cb

    node = Node()
    names = ("infra1", "depth", "picam")
    for n in names:
        node.subscribe(Image, f"/{a.model}/{n}", grab(n))
    t0 = time.time()
    while time.time() - t0 < a.timeout and len(got) < len(names):
        time.sleep(0.1)
    if len(got) < len(names):
        print(f"no images on {sorted(set(names) - set(got))}: is the simulation running?", flush=True)
        return 1

    a.out.mkdir(parents=True, exist_ok=True)
    titles = {
        "infra1": "D435i infrared (left): OpenVINS",
        "depth": "D435i depth (0.3-6 m): lane perception",
        "picam": "Pi Camera Module 3 (left side): plants",
    }
    fig, axes = plt.subplots(1, 3, figsize=(18, 5.2))
    for ax, n in zip(axes, names, strict=True):
        img = got[n]
        if n == "depth":
            d = np.where(np.isfinite(img), img, np.nan)
            plt.imsave(a.out / f"{a.prefix}{n}.png", np.nan_to_num(d, nan=0.0), cmap="turbo", vmin=0.3, vmax=6.0)
            ax.imshow(d, cmap="turbo", vmin=0.3, vmax=6.0)
        else:
            plt.imsave(a.out / f"{a.prefix}{n}.png", img, cmap="gray" if img.ndim == 2 else None)
            ax.imshow(img, cmap="gray" if img.ndim == 2 else None)
        ax.set_title(titles[n])
        ax.axis("off")
    fig.tight_layout()
    fig.savefig(a.out / f"{a.prefix}views.png", dpi=90)
    print("saved", ", ".join(str(a.out / f"{a.prefix}{n}.png") for n in (*names, "views")), flush=True)
    return 0


if __name__ == "__main__":
    import os

    code = main()
    os._exit(code)  # skip interpreter teardown: gz-transport's threads crash it otherwise
