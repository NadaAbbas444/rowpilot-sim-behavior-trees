"""Draw the running simulation's processes, topics and message types (like ROS's rqt_graph).

The simulation does not use ROS: the processes talk over gz-transport (Gazebo's publish/subscribe,
like ROS topics), MAVLink (to ArduPilot) and ArduPilot's JSON UDP link (to the Gazebo plugin). This
script asks gz-transport who publishes and subscribes to each topic, maps each address to its process
(`ss -tlnp`), adds the MAVLink and JSON links, and writes:

    <out>.md    a Mermaid diagram (GitHub renders it)
    <out>.png   the same graph drawn with matplotlib

    GZ_IP=127.0.0.1 PYTHONNOUSERSITE=1 .venv-sim/bin/python scripts/topic_graph.py [--out docs/topic-graph]
"""

from __future__ import annotations

import argparse
import re
import subprocess
from pathlib import Path

SKIP = re.compile(r"^/(clock|stats|gui|gazebo|sensors/marker|marker)|/scene/|/light_config|/material_color|/state$")
PROCESS_NAMES = {  # command line fragment -> label
    "gz sim": "Gazebo (physics, sensors)",
    "vio_bridge": "vio_bridge (OpenVINS)",
    "rowpilot.imu_synth": "imu_synth",
    "rowpilot ardupilot": "rowpilot (behaviour tree)",
    "arducopter": "ArduCopter SITL",
}


def sh(*cmd: str) -> str:
    return subprocess.run(cmd, capture_output=True, text=True, timeout=20).stdout


def port_owners() -> dict[str, str]:
    """TCP port -> process label, from ss."""
    owners = {}
    for line in sh("ss", "-tlnpH").splitlines():
        m = re.search(r":(\d+)\s.*pid=(\d+)", line)
        if not m:
            continue
        port, pid = m.groups()
        try:
            cmdline = Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\0", b" ").decode()
        except OSError:
            continue
        for frag, label in PROCESS_NAMES.items():
            if frag in cmdline:
                owners[port] = label
                break
    return owners


def collect() -> list[tuple[str, str, str, str]]:
    """(publisher, topic, message type, subscriber) for every gz topic with a known publisher."""
    owners = port_owners()
    edges = []
    for topic in sh("gz", "topic", "-l").split():
        if SKIP.search(topic) or "joint_state" in topic:
            continue
        info = sh("gz", "topic", "-i", "-t", topic)
        pubs, subs, section = [], [], None
        for line in info.splitlines():
            if line.startswith("Publishers"):
                section = pubs
            elif line.startswith("Subscribers"):
                section = subs
            elif (m := re.search(r":(\d+), (\S+)", line)) and section is not None:
                section.append((owners.get(m.group(1), f"port {m.group(1)}"), m.group(2)))
        if not subs and not topic.endswith("/picam"):
            continue  # published but unused (stats, point cloud, ...)
        for pub, msg in pubs:
            for sub, _ in subs or [("(plant images: saved on the real drone)", msg)]:
                if sub != pub:
                    edges.append((pub, topic, msg, sub))
    return sorted(set(edges))


STATIC = [  # links that are not gz-transport
    (
        "rowpilot (behaviour tree)",
        "MAVLink tcp:5760",
        "ODOMETRY, SET_POSITION_TARGET_LOCAL_NED, COMMAND_LONG",
        "ArduCopter SITL",
    ),
    (
        "ArduCopter SITL",
        "MAVLink tcp:5760",
        "HEARTBEAT, LOCAL_POSITION_NED, ATTITUDE, SYS_STATUS",
        "rowpilot (behaviour tree)",
    ),
    ("ArduCopter SITL", "JSON UDP 9002", "motor PWM", "Gazebo (physics, sensors)"),
    ("Gazebo (physics, sensors)", "JSON UDP 9002", "IMU, pose, velocity (lock-step)", "ArduCopter SITL"),
]


def mermaid(edges) -> str:
    ids: dict[str, str] = {}

    def nid(name: str, shape: str) -> str:
        if name not in ids:
            ids[name] = f"n{len(ids)}"
            label = name.replace('"', "'")
            lines.append(f'    {ids[name]}{shape[0]}"{label}"{shape[1]}')
        return ids[name]

    lines = ["```mermaid", "flowchart LR"]
    for pub, topic, msg, sub in edges:
        p, s = nid(pub, "[]"), nid(sub, "[]")
        t = nid(f"{topic}\n{msg}", "()")
        lines += [f"    {p} --> {t}", f"    {t} --> {s}"]
    lines.append("```")
    return "\n".join(dict.fromkeys(lines))  # drop repeated edges, keep order


LAYOUT = {  # process -> (x, y) on the drawing: the data flows left to right
    "Gazebo (physics, sensors)": (0.0, 0.0),
    "imu_synth": (3.0, 1.6),
    "vio_bridge (OpenVINS)": (6.0, 1.6),
    "rowpilot (behaviour tree)": (9.0, 0.0),
    "ArduCopter SITL": (4.5, -2.4),
    "(plant images: saved on the real drone)": (0.0, 2.6),
}


def short(msg: str) -> str:
    return msg.replace("gz.msgs.", "")


def draw_png(edges, path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    pos = dict(LAYOUT)
    for name in sorted({e[0] for e in edges} | {e[3] for e in edges}):
        pos.setdefault(name, (4.5, 3.2 + 0.6 * len(pos)))
    groups: dict[tuple[str, str], list[str]] = {}
    for pub, topic, msg, sub in edges:
        groups.setdefault((pub, sub), []).append(f"{topic}  [{short(msg)}]")
    fig, ax = plt.subplots(figsize=(16, 8.5))
    for (pub, sub), labels in groups.items():
        (x0, y0), (x1, y1) = pos[pub], pos[sub]
        both_ways = (sub, pub) in groups
        rad = 0.18 if both_ways else 0.0
        ax.annotate(
            "",
            (x1, y1),
            (x0, y0),
            arrowprops=dict(
                arrowstyle="-|>", color="#1f5fa8", lw=1.3, shrinkA=60, shrinkB=60, connectionstyle=f"arc3,rad={rad}"
            ),
        )
        mx, my = (x0 + x1) / 2, (y0 + y1) / 2
        if both_ways:  # put each direction's label on its own side of the pair
            nx, ny = (y1 - y0), -(x1 - x0)
            n = max((nx * nx + ny * ny) ** 0.5, 1e-9)
            mx, my = mx - 0.35 * nx / n * 1.6, my - 0.35 * ny / n * 1.6
        ax.text(
            mx,
            my,
            "\n".join(labels),
            ha="center",
            va="center",
            fontsize=7.5,
            bbox=dict(boxstyle="round", fc="#f6eedc", ec="#c9a96a"),
        )
    for name, (x, y) in pos.items():
        if any(name in (e[0], e[3]) for e in edges):
            ax.text(
                x,
                y,
                name,
                ha="center",
                va="center",
                fontsize=10,
                weight="bold",
                bbox=dict(boxstyle="round,pad=0.6", fc="#dfe8f5", ec="#1f5fa8"),
            )
    ax.set_title("rowpilot simulation: processes (blue) and the topics / links between them (beige, [message type])")
    xs, ys = [p[0] for p in pos.values()], [p[1] for p in pos.values()]
    ax.set_xlim(min(xs) - 1.5, max(xs) + 1.5)
    ax.set_ylim(min(ys) - 1.2, max(ys) + 1.2)
    ax.axis("off")
    fig.tight_layout()
    fig.savefig(path, dpi=110)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", type=Path, default=Path("docs/topic-graph"))
    a = ap.parse_args()
    edges = collect()
    if not edges:
        print("no topics found: is the simulation running (and GZ_IP=127.0.0.1 set)?")
        return 1
    edges += STATIC
    a.out.parent.mkdir(parents=True, exist_ok=True)
    body = (
        "# Processes, topics and messages\n\n"
        "Generated from the running simulation by `scripts/topic_graph.py` (the simulation's rqt_graph).\n"
        "Boxes are processes; rounded boxes are topics or links, with their message type.\n\n"
        + mermaid(edges)
        + "\n\n| Publisher | Topic / link | Message | Subscriber |\n|---|---|---|---|\n"
        + "".join(f"| {p} | `{t}` | {m} | {s} |\n" for p, t, m, s in edges)
    )
    a.out.with_suffix(".md").write_text(body)
    draw_png(edges, a.out.with_suffix(".png"))
    print(f"wrote {a.out.with_suffix('.md')} and {a.out.with_suffix('.png')} ({len(edges)} links)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
