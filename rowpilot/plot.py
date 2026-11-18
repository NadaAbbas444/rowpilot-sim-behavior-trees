"""Top-down plot of a run: crop rows, the planned path, the true flight path, and what visual odometry thought."""

from __future__ import annotations

from .layout import GreenhouseLayout
from .runner import RunResult


def plot_run(result: RunResult, layout: GreenhouseLayout, path: str) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    x_span = layout.far_x + layout.headland + 3.0  # sized to the greenhouse, so the equal-aspect plot fills it
    y_span = (layout.n_lanes + 1) * layout.pitch
    fig, ax = plt.subplots(figsize=(min(16.0, 4.0 + 0.42 * x_span), 1.6 + 0.42 * y_span))
    for c in layout.crop_row_centers_y():
        ax.add_patch(
            Rectangle(
                (0, c - layout.crop_width / 2), layout.row_length, layout.crop_width, color="#5b8c3a", alpha=0.85, lw=0
            )
        )
    px, py = zip(*layout.planned_route(), strict=True)
    ax.plot(px, py, color="#2ca02c", lw=3.2, alpha=0.35, solid_joinstyle="round", label="planned path", zorder=1)
    xs, ys = [t["x"] for t in result.trace], [t["y"] for t in result.trace]
    ax.plot(
        [t["est_x"] for t in result.trace],
        [t["est_y"] for t in result.trace],
        color="#e08a1e",
        lw=1,
        ls="--",
        label="visual odometry estimate",
    )
    ax.plot(xs, ys, color="#1f5fa8", lw=1.4, label="true path")
    hx, hy = layout.home
    ax.plot(hx, hy, marker="s", color="k", ms=6, label="home")
    ax.plot(xs[-1], ys[-1], marker="v", color="#c0392b", ms=8, label="end")
    ax.set_aspect("equal")
    if layout.closed_end:
        y0, y1 = layout.crop_row_centers_y()[0], layout.crop_row_centers_y()[-1]
        ax.plot([layout.far_x] * 2, [y0 - 0.5, y1 + 0.5], color="#555", lw=3, label="end wall")
    ax.set_xlim(hx - 1, layout.far_x + (0.6 if layout.closed_end else -1.0))
    ax.set_xlabel("along rows (m)")
    ax.set_ylabel("across (m)")
    ax.set_title(
        f"{result.outcome} · {len(result.lanes_completed)}/{layout.n_lanes} lanes · "
        f"{result.duration_s:.0f} s · battery left {result.battery_end_pct:.0f}% · "
        f"worst distance from lane centre {result.max_lane_error_m:.2f} m",
        fontsize=10,
    )
    ax.legend(loc="center left", bbox_to_anchor=(1.01, 0.5), frameon=False, fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
