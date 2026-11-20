"""A window on an Isaac Sim run: the view following the drone and its cameras, live.

Isaac Sim renders headless in the simulation (its own window needs a display it can draw on with
Vulkan, which a desktop on software rendering does not have). It streams a level camera following the
drone (/<model>/chase) besides the drone's own cameras; this shows them, as the Gazebo window does:

    chase view (large) | D435i infrared (left) | D435i depth | Pi Camera Module 3

    python -m rowpilot.isaac_view [--model gh_firefly]      (run in .venv-sim: needs the gz bindings)
"""

from __future__ import annotations

import argparse
import math
import threading

import numpy as np

from .depth_view import colourise

PANELS = ("chase", "infra1", "depth", "picam")
TITLES = {
    "chase": "Isaac Sim: following the drone",
    "infra1": "D435i infrared (left)",
    "depth": "D435i depth",
    "picam": "Pi Camera Module 3",
}
WAITING = {"picam": "Pi Camera Module 3 (off: ISAAC_PICAM=1 renders it)"}


def to_rgb(kind: str, width: int, height: int, data: bytes) -> np.ndarray:
    """An image message's pixels as an RGB uint8 array (depth colour-coded as depth_view does)."""
    if kind == "depth":
        return colourise(np.frombuffer(data, np.float32).reshape(height, width))
    a = np.frombuffer(data, np.uint8)
    if a.size == width * height:
        return np.repeat(a.reshape(height, width, 1), 3, axis=2)
    return a.reshape(height, width, 3)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--model", default="gh_firefly")
    ap.add_argument("--world", default="greenhouse")
    ap.add_argument("--hz", type=float, default=5.0, help="window refresh rate")
    a = ap.parse_args(argv)

    import matplotlib.pyplot as plt  # matplotlib picks an interactive backend that works (GTK, Qt or Tk)
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node

    latest: dict[str, tuple] = {}
    pose = {"t": 0.0, "xyz": (0.0, 0.0, 0.0), "yaw": 0.0}
    lock = threading.Lock()
    node = Node()

    def on_image(kind):
        def cb(m) -> None:
            with lock:
                latest[kind] = (m.width, m.height, m.data)

        return cb

    def on_pose(m) -> None:
        for p in m.pose:
            if p.name == a.model:
                q = p.orientation
                pose["t"] = m.header.stamp.sec + 1e-9 * m.header.stamp.nsec
                pose["xyz"] = (p.position.x, p.position.y, p.position.z)
                pose["yaw"] = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    for kind in PANELS:
        node.subscribe(Image, f"/{a.model}/{kind}", on_image(kind))
    node.subscribe(Pose_V, f"/world/{a.world}/pose/info", on_pose)

    fig = plt.figure(figsize=(13, 6.5), facecolor="#1e1e1e")
    fig.canvas.manager.set_window_title("rowpilot: Isaac Sim")
    grid = fig.add_gridspec(3, 3, width_ratios=(2.2, 2.2, 1.6), wspace=0.04, hspace=0.18)
    axes = {
        "chase": fig.add_subplot(grid[:, :2]),
        "infra1": fig.add_subplot(grid[0, 2]),
        "depth": fig.add_subplot(grid[1, 2]),
        "picam": fig.add_subplot(grid[2, 2]),
    }
    shown = {}
    for kind, ax in axes.items():
        ax.set_axis_off()
        ax.set_title(WAITING.get(kind, TITLES[kind]), color="#dddddd", fontsize=9)
        shown[kind] = ax.imshow(np.zeros((9, 16, 3), np.uint8))
    status = fig.text(0.01, 0.01, "waiting for Isaac Sim ...", color="#dddddd", fontsize=10, family="monospace")
    plt.subplots_adjust(left=0.01, right=0.99, top=0.95, bottom=0.05)

    def refresh() -> None:
        with lock:
            frames = dict(latest)
            latest.clear()
        for kind, (w, h, data) in frames.items():
            img = to_rgb(kind, w, h, data)
            im = shown[kind]
            if im.get_array().shape != img.shape:
                im.set_extent((-0.5, w - 0.5, h - 0.5, -0.5))
                axes[kind].set_xlim(-0.5, w - 0.5)
                axes[kind].set_ylim(h - 0.5, -0.5)
            im.set_data(img)
            axes[kind].set_title(TITLES[kind], color="#dddddd", fontsize=9)
        x, y, z = pose["xyz"]
        status.set_text(f"sim t {pose['t']:7.1f} s   x {x:6.2f}  y {y:6.2f}  z {z:5.2f} m   yaw {pose['yaw']:6.1f} deg")
        fig.canvas.draw_idle()

    timer = fig.canvas.new_timer(interval=int(1000 / a.hz))
    timer.add_callback(refresh)
    timer.start()
    print(f"[isaac_view] showing /{a.model}/{{{','.join(PANELS)}}}", flush=True)
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
