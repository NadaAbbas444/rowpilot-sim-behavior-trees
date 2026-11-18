"""Draw the behaviour tree diagrams in docs/bt/ from the tree the code builds.

    python scripts/bt_diagrams.py [--out docs/bt]

  tree.png        the whole tree (lanes 2..n are the same as lane 1 and drawn as one box)
  lane.png        one dead-end lane, step by step, seen from above
  tick.png        what happens on every tick: the guard, the failsafes, the mission

Run it again after changing the tree (rowpilot/mission.py), so the pictures stay true.
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import py_trees  # noqa: E402
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Rectangle  # noqa: E402

from rowpilot.layout import GreenhouseLayout  # noqa: E402
from rowpilot.mission import Condition, MissionContext, build_tree  # noqa: E402
from rowpilot.sim import SimVehicle  # noqa: E402

COLOURS = {
    "sequence": ("#dfe8f5", "#1f5fa8"),
    "selector": ("#f6eedc", "#b8862b"),
    "decorator": ("#ece3f5", "#7a4fa8"),
    "condition": ("#e3f2e1", "#3c8a3a"),
    "action": ("#ffffff", "#555555"),
    "group": ("#f2f2f2", "#999999"),
}


def kind(node) -> str:
    if isinstance(node, py_trees.composites.Sequence):
        return "sequence"
    if isinstance(node, py_trees.composites.Selector):
        return "selector"
    if isinstance(node, py_trees.decorators.Decorator):
        return "decorator"
    if isinstance(node, Condition):
        return "condition"
    return "action"


def label(node, k: str) -> str:
    prefix = {"sequence": "→ ", "selector": "? ", "decorator": "◇ "}.get(k, "")
    name = node.name.split(": ", 1)[-1] if ": " in node.name and k != "sequence" else node.name
    return prefix + name


LANE = re.compile(r"^Lane \d+$")  # a lane subtree (its steps are named "Lane n: ...")


class Box:
    def __init__(self, text: str, k: str, children: list[Box] | None = None, note: str = ""):
        self.text, self.k, self.children, self.note = text, k, children or [], note
        self.x = self.y = 0.0


def to_boxes(node, depth: int = 0, lanes_shown: int = 1) -> Box:
    k = kind(node)
    kids = []
    lane_count = 0
    n_lanes = sum(1 for c in node.children if LANE.match(c.name))
    for c in node.children:
        if LANE.match(c.name):
            lane_count += 1
            if lane_count == lanes_shown + 1:
                kids.append(Box(f"Lane 2 … Lane {n_lanes}", "group", note="same steps as Lane 1"))
            if lane_count > lanes_shown:
                continue
        kids.append(to_boxes(c, depth + 1, lanes_shown))
    return Box(label(node, k), k, kids)


def layout(b: Box, depth: int, next_x: list[float], dx: float, dy: float) -> None:
    b.y = -depth * dy
    if not b.children:
        b.x = next_x[0]
        next_x[0] += dx
        return
    for c in b.children:
        layout(c, depth + 1, next_x, dx, dy)
    b.x = (b.children[0].x + b.children[-1].x) / 2


def draw_box(ax, x, y, text, k, w=1.55, h=0.5, fs=8.5, note=""):
    fc, ec = COLOURS[k]
    style = "round,pad=0.02,rounding_size=0.22" if k == "condition" else "round,pad=0.02,rounding_size=0.06"
    ax.add_patch(FancyBboxPatch((x - w / 2, y - h / 2), w, h, boxstyle=style, fc=fc, ec=ec, lw=1.4))
    ax.text(
        x,
        y + (0.07 if note else 0),
        text,
        ha="center",
        va="center",
        fontsize=fs,
        weight="bold" if k in ("sequence", "selector", "decorator") else "normal",
    )
    if note:
        ax.text(x, y - 0.14, note, ha="center", va="center", fontsize=6.5, style="italic", color="#555")


def draw_tree(root: Box, path: Path) -> None:
    layout(root, 0, [0.0], 1.75, 1.15)
    boxes = []

    def walk(b):
        boxes.append(b)
        for c in b.children:
            walk(c)

    walk(root)
    xs, ys = [b.x for b in boxes], [b.y for b in boxes]
    fig, ax = plt.subplots(figsize=(max(xs) - min(xs) + 3, (max(ys) - min(ys)) + 3.2))
    for b in boxes:
        for c in b.children:
            ax.plot([b.x, c.x], [b.y - 0.25, c.y + 0.25], color="#888", lw=1, zorder=0)
    for b in boxes:
        draw_box(ax, b.x, b.y, b.text, b.k, note=b.note)
    legend = [
        ("sequence", "→ Sequence: children in order, all must succeed"),
        ("selector", "? Selector: first child that does not fail"),
        ("decorator", "◇ Decorator: changes its child's answer"),
        ("condition", "Condition: a check, answers at once"),
        ("action", "Action: does something, RUNNING until done"),
    ]
    lx, ly = min(xs) - 0.6, min(ys) - 1.1
    for i, (k, t) in enumerate(legend):
        fc, ec = COLOURS[k]
        ax.add_patch(Rectangle((lx + i * 3.4, ly - 0.15), 0.35, 0.3, fc=fc, ec=ec))
        ax.text(lx + i * 3.4 + 0.45, ly, t, va="center", fontsize=7.5)
    ax.set_title("rowpilot behaviour tree (ticked 10 times a second, left to right, top to bottom)", fontsize=12)
    ax.set_xlim(min(xs) - 1, max(xs) + 1)
    ax.set_ylim(min(ys) - 1.5, max(ys) + 0.7)
    ax.axis("off")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def arrow(ax, a, b, text="", colour="#333", rad=0.0, fs=8):
    ax.add_patch(
        FancyArrowPatch(
            a,
            b,
            arrowstyle="-|>",
            mutation_scale=12,
            color=colour,
            lw=1.3,
            connectionstyle=f"arc3,rad={rad}",
            shrinkA=4,
            shrinkB=4,
        )
    )
    if text:
        ax.text((a[0] + b[0]) / 2, (a[1] + b[1]) / 2 + 0.12, text, ha="center", fontsize=fs, color=colour)


def draw_lane(lay: GreenhouseLayout, path: Path) -> None:
    """One dead-end lane from above, with the steps the drone flies."""
    fig, ax = plt.subplots(figsize=(14, 4.4))
    L, w, cw = lay.row_length, lay.path_width, lay.crop_width
    for yc in (-(w + cw) / 2, (w + cw) / 2):
        ax.add_patch(Rectangle((0, yc - cw / 2), L, cw, color="#5b8c3a", alpha=0.85))
    ax.text(L / 2, (w + cw) / 2 + 0.38, "crop row", ha="center", fontsize=9, color="#2f5d1e")
    ax.text(L / 2, -(w + cw) / 2 - 0.45, "crop row", ha="center", fontsize=9, color="#2f5d1e")
    ax.plot([L, L], [-1.6, 1.6], color="#444", lw=5)
    ax.text(L + 0.15, 0, "end wall\n(lane closed)", va="center", fontsize=9)
    ax.axvspan(-lay.headland - 1.4, 0, color="#eeeeee", zorder=0)
    ax.text(-lay.headland / 2 - 0.6, 1.45, "headland", ha="center", fontsize=9, color="#666")
    t = lay.turn_x()
    steps = [
        ((-2.5, -0.95), (-lay.headland, -0.12), "", "#1f5fa8", 0.0),
        ((-lay.headland, -0.12), (0.6, -0.12), "2 Align", "#3c8a3a", 0.0),
        ((0.6, -0.12), (t, -0.12), "3 Sweep in  (lane following, brakes before the wall)", "#1f5fa8", 0.0),
        ((t, 0.12), (0.0, 0.12), "5 Sweep out  (Pi Camera now sees the other row)", "#b8862b", 0.0),
        ((0.0, 0.12), (-lay.headland, 0.12), "6 Exit", "#7a4fa8", 0.0),
    ]
    for a, b, text, c, rad in steps:
        arrow(ax, a, b, colour=c, rad=rad)
        ax.text((a[0] + b[0]) / 2, a[1] + (-0.3 if a[1] < 0 else 0.22), text, ha="center", fontsize=9, color=c)
    ax.text(-2.0, -0.45, "1 GoToEntry", fontsize=9, color="#1f5fa8", ha="right")
    ax.add_patch(plt.Circle((t, 0), 0.33, fill=False, ec="#c0392b", lw=1.6, ls="--"))
    ax.text(t, -0.62, "4 TurnAround\n180° on the spot", ha="center", va="top", fontsize=9, color="#c0392b")
    ax.annotate("", (L, 0.75), (t, 0.75), arrowprops=dict(arrowstyle="<->", color="#555"))
    ax.text((t + L) / 2, 0.85, f"{lay.end_clearance:.1f} m", ha="center", fontsize=8.5, color="#555")
    ax.plot(-2.5, -0.95, marker="s", color="k")
    ax.text(-2.6, -1.12, "start: home, or the\nprevious lane's exit", ha="right", fontsize=8, va="top")
    ax.text(
        -lay.headland - 1.3, -1.9, "then GoToEntry of the next lane, 2 m across the headland", fontsize=9, color="#555"
    )
    ax.set_title(
        "One lane, as the tree flies it: Lane n = GoToEntry → Align → Sweep in → TurnAround → Sweep out → Exit",
        fontsize=11.5,
    )
    ax.set_aspect("equal")
    ax.set_xlim(-lay.headland - 3.2, L + 1.6)
    ax.set_ylim(-2.1, 1.75)
    ax.set_xlabel("along the lane (m)")
    ax.set_yticks([])
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def draw_tick(path: Path) -> None:
    """Flowchart of one tick of the tree."""
    fig, ax = plt.subplots(figsize=(13, 8.2))

    def box(x, y, text, k, w=2.9, h=0.75, fs=9):
        draw_box(ax, x, y, text, k, w=w, h=h, fs=fs)

    box(0, 6.6, "Every 0.1 s: read sensors once\n(pose, battery, VO health, lane estimate)", "action", w=4.2)
    box(0, 5.2, "All healthy?\nnot aborted · battery > 25 % · VO tracking", "condition", w=4.2)
    box(4.6, 5.2, "Mission: tick the current step\n(finished steps answer SUCCESS at once)", "sequence", w=4.0)
    box(4.6, 3.9, "the step sends one velocity command\nRUNNING → next tick continues it", "action", w=4.0)
    box(-4.4, 3.9, "Failsafe: pick the first that applies", "selector", w=3.6)
    box(-7.4, 2.2, "VO lost < 3 s\n→ Hover", "condition", w=2.4)
    box(-4.4, 2.2, "battery < 12 % or\nVO lost ≥ 3 s\n→ Land here", "condition", w=2.4, h=0.95)
    box(
        -1.3,
        2.2,
        "anything else, airborne\n(battery < 25 %, a step failed)\n→ fly home, land",
        "condition",
        w=2.6,
        h=0.95,
    )
    box(-7.4, 0.6, "VO back → mission resumes\nat the same step, same waypoints", "sequence", w=3.0)
    box(-3.1, 0.6, "landed → the run ends", "decorator", w=2.6)
    arrow(ax, (0, 6.22), (0, 5.6))
    arrow(ax, (2.1, 5.2), (2.6, 5.2), "yes", "#3c8a3a")
    arrow(ax, (4.6, 4.82), (4.6, 4.28))
    arrow(ax, (-1.4, 4.9), (-3.4, 4.25), "no", "#c0392b")
    for x in (-7.4, -4.4, -1.3):
        arrow(ax, (-4.4, 3.52), (x, 2.62))
    arrow(ax, (-7.4, 1.82), (-7.4, 0.98))
    arrow(ax, (-4.4, 1.72), (-3.6, 0.98))
    arrow(ax, (-1.3, 1.72), (-2.6, 0.98))
    arrow(ax, (6.6, 3.9), (6.6, 6.6), colour="#888")
    ax.text(6.75, 5.9, "next\ntick", fontsize=8, color="#888")
    arrow(ax, (6.6, 6.6), (2.1, 6.6), colour="#888")
    ax.text(
        4.6,
        2.6,
        "While a failsafe runs, the mission is not ticked:\nthe failsafe has control.\n"
        "Mission steps are re-entrant: each remembers\nthat it finished, and GoTo keeps its waypoints.",
        ha="center",
        fontsize=9,
        color="#444",
        bbox=dict(boxstyle="round", fc="#fafafa", ec="#ccc"),
    )
    ax.set_title("One tick of the rowpilot behaviour tree", fontsize=12)
    ax.set_xlim(-9.2, 7.6)
    ax.set_ylim(-0.2, 7.3)
    ax.axis("off")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", type=Path, default=Path("docs/bt"))
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    lay = GreenhouseLayout()
    root = build_tree(MissionContext(SimVehicle(lay), lay))
    draw_tree(to_boxes(root), a.out / "tree.png")
    draw_lane(lay, a.out / "lane.png")
    draw_tick(a.out / "tick.png")
    print(f"wrote {a.out}/tree.png, lane.png, tick.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
