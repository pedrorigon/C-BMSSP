#!/usr/bin/env python3
"""Animate the empirical-complexity growth curves as a 10-second GIF.

Wall-clock time cannot show the BMSSP advantage: its constant-degree blow-up and
larger constant factor make it slower on every in-memory input, and in *total*
operations it always does more work than Dijkstra. What it genuinely wins is the
*growth rate*: the weighted ordered-structure work per edge grows like
``log^(2/3) n`` for BMSSP and like ``log n`` for Dijkstra.

This GIF animates exactly that. As the graph size sweeps from the smallest to the
largest measured point, both curves are drawn point by point. Each curve is
normalized to its own value at the smallest graph, so both start at 1.0 and only
the growth rate is compared. The Dijkstra curve climbs steeply and the BMSSP
curve flattens, with Dijkstra overtaking it -- the smaller exponent made visible.

Input is the ``work_per_edge`` column written by ``run_scaling.py``; by default
the most recent ``output/*_scaling_*.csv`` is used.
"""

from __future__ import annotations

import argparse
import csv
import glob
import math
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import animation  # noqa: E402


ROOT = Path(__file__).resolve().parent
OUTPUT_DIR = ROOT / "output"
GIF_DIR = ROOT / "visualizations" / "complexity"

FPS = 20
TOTAL_SECONDS = 10
FRAME_COUNT = FPS * TOTAL_SECONDS
# Frames spent holding the completed figure so the final divergence is readable.
HOLD_FRAMES = FPS * 3

SERIES = {
    "bmssp": {"color": "#0891b2", "label": "BMSSP  ·  O(m log^(2/3) n)"},
    "dijkstra": {"color": "#e11d6f", "label": "Dijkstra  ·  O(m log n)"},
}

# Light theme: white background, dark text, grey grid.
THEME = {
    "background": "#ffffff",
    "axes": "#ffffff",
    "text": "#1e293b",
    "muted": "#64748b",
    "grid": "#cbd5e1",
    "marker_edge": "#ffffff",
    "legend_face": "#f8fafc",
    "legend_edge": "#cbd5e1",
}


@dataclass
class Curve:
    sizes: list[int]
    work_per_edge: list[float]

    @property
    def normalized(self) -> list[float]:
        base = self.work_per_edge[0] if self.work_per_edge and self.work_per_edge[0] else 1.0
        return [value / base for value in self.work_per_edge]


def latest_scaling_csv() -> Path:
    candidates = sorted(glob.glob(str(OUTPUT_DIR / "*_scaling_*.csv")))
    if not candidates:
        raise SystemExit(
            "No scaling CSV found in output/. Run run_scaling.py first, e.g.:\n"
            "  uv run python run_scaling.py --type sequential --max-vertices 1000000 --steps 8"
        )
    return Path(candidates[-1])


def load_curves(csv_path: Path) -> dict[str, Curve]:
    rows: dict[str, dict[int, float]] = {"bmssp": {}, "dijkstra": {}}
    with csv_path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            model = row["model"]
            if model not in rows or row["type"] != "sequential":
                continue
            if "work_per_edge" not in row or not row["work_per_edge"]:
                raise SystemExit(
                    f"{csv_path.name} has no work_per_edge column. Re-run run_scaling.py "
                    "after rebuilding the instrumented sssp_scaling executable."
                )
            rows[model][int(row["vertices"])] = float(row["work_per_edge"])

    curves: dict[str, Curve] = {}
    for model, by_size in rows.items():
        if not by_size:
            continue
        sizes = sorted(by_size)
        curves[model] = Curve(sizes=sizes, work_per_edge=[by_size[size] for size in sizes])
    if "bmssp" not in curves or "dijkstra" not in curves:
        raise SystemExit("CSV must contain both bmssp and dijkstra sequential rows.")
    return curves


def format_size(value: int) -> str:
    if value >= 1_000_000:
        return f"{value / 1_000_000:g}M"
    if value >= 1_000:
        return f"{value / 1_000:g}K"
    return str(value)


def build_animation(curves: dict[str, Curve], output: Path) -> None:
    # The x-axis is the union of every size measured by any model, so Dijkstra's
    # longer reach (it scales far past where BMSSP's transform fits in memory) is
    # visible as its curve continuing after BMSSP's ends.
    all_sizes = sorted({size for curve in curves.values() for size in curve.sizes})
    index_of = {size: position for position, size in enumerate(all_sizes)}
    point_count = len(all_sizes)
    positions = list(range(point_count))

    # Per model, the (x-position, normalized-y) points in size order.
    series_points: dict[str, list[tuple[int, float]]] = {
        model: [
            (index_of[size], value)
            for size, value in zip(curve.sizes, curve.normalized)
        ]
        for model, curve in curves.items()
    }
    max_y = max(value for points in series_points.values() for _, value in points)

    plt.style.use("default")
    figure, axis = plt.subplots(figsize=(11, 6.2))
    figure.patch.set_facecolor(THEME["background"])
    axis.set_facecolor(THEME["axes"])

    axis.set_xlim(-0.3, point_count - 0.7)
    axis.set_ylim(0.95, max_y * 1.08)
    axis.set_xticks(positions)
    axis.set_xticklabels([format_size(size) for size in all_sizes], color=THEME["text"])
    axis.tick_params(colors=THEME["text"])
    for spine in axis.spines.values():
        spine.set_color(THEME["grid"])
    axis.set_xlabel("Vértices", color=THEME["text"], fontsize=12)
    axis.set_ylabel("Trabalho/aresta relativo", color=THEME["text"], fontsize=12)
    axis.set_title(
        "Complexidade empírica: taxa de crescimento do trabalho",
        color=THEME["text"], fontsize=14, fontweight="bold", pad=30,
    )
    axis.text(
        0.5, 1.035,
        "Menor crescimento = menor expoente. O BMSSP achata; o Dijkstra continua subindo.",
        transform=axis.transAxes, ha="center", va="bottom", fontsize=9, color=THEME["muted"],
    )
    axis.grid(True, linestyle="--", linewidth=0.7, alpha=0.45, color=THEME["grid"])
    for spine in ("top", "right"):
        axis.spines[spine].set_visible(False)

    lines: dict[str, plt.Line2D] = {}
    dots: dict[str, plt.Line2D] = {}
    labels: dict[str, plt.Text] = {}
    for model, style in SERIES.items():
        (line,) = axis.plot([], [], color=style["color"], linewidth=3.0,
                            solid_capstyle="round", zorder=3, label=style["label"])
        (dot,) = axis.plot([], [], "o", color=style["color"], markersize=10,
                          markeredgecolor=THEME["marker_edge"], markeredgewidth=1.4, zorder=4)
        labels[model] = axis.text(
            0, 0, "", color=style["color"], fontsize=11, fontweight="bold",
            ha="left", va="center", zorder=5,
        )
        lines[model] = line
        dots[model] = dot
    axis.legend(loc="upper left", frameon=True, facecolor=THEME["legend_face"],
                edgecolor=THEME["legend_edge"], labelcolor=THEME["text"], fontsize=10)

    draw_frames = FRAME_COUNT - HOLD_FRAMES
    last_x = point_count - 1

    def revealed_x(frame: int) -> float:
        """X-position revealed so far, eased, shared by both curves."""
        if frame >= draw_frames:
            return float(last_x)
        progress = frame / max(draw_frames - 1, 1)
        eased = 0.5 - 0.5 * math.cos(progress * math.pi)
        return eased * last_x

    def draw_series(points: list[tuple[int, float]], cutoff: float):
        """Points up to the revealed x, plus a smooth tip to the next point."""
        xs: list[float] = []
        ys: list[float] = []
        for position, (x, y) in enumerate(points):
            if x <= cutoff:
                xs.append(float(x))
                ys.append(y)
                continue
            # First point beyond the cutoff: draw a partial segment to it, but only
            # if the previous point was already on screen (the curve is growing
            # toward it). Stops once a curve has reached its own last point.
            if position > 0 and points[position - 1][0] <= cutoff:
                prev_x, prev_y = points[position - 1]
                span = x - prev_x
                fraction = (cutoff - prev_x) / span if span else 0.0
                xs.append(cutoff)
                ys.append(prev_y + fraction * (y - prev_y))
            break
        return xs, ys

    def update(frame: int):
        cutoff = revealed_x(frame)
        artists: list = []
        for model in SERIES:
            xs_drawn, ys_drawn = draw_series(series_points[model], cutoff)
            lines[model].set_data(xs_drawn, ys_drawn)
            if xs_drawn:
                dots[model].set_data([xs_drawn[-1]], [ys_drawn[-1]])
                labels[model].set_position((xs_drawn[-1] + 0.08, ys_drawn[-1]))
                labels[model].set_text(f"{ys_drawn[-1]:.2f}x")
            artists.extend([lines[model], dots[model], labels[model]])
        return artists

    anim = animation.FuncAnimation(
        figure, update, frames=FRAME_COUNT, interval=1000 / FPS, blit=True,
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    writer = animation.PillowWriter(fps=FPS)
    anim.save(str(output), writer=writer)
    plt.close(figure)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--csv", type=Path,
        help="Scaling CSV to read (default: most recent output/*_scaling_*.csv).",
    )
    parser.add_argument(
        "--output", type=Path, default=GIF_DIR / "complexity_growth.gif",
        help="Destination GIF path.",
    )
    return parser.parse_args()


def _relative(path: Path) -> str:
    """Path relative to the repo root when possible, else the path as given."""
    try:
        return str(path.resolve().relative_to(ROOT))
    except ValueError:
        return str(path)


def main() -> int:
    args = parse_args()
    csv_path = args.csv or latest_scaling_csv()
    curves = load_curves(csv_path)
    build_animation(curves, args.output)
    final = {model: curve.normalized[-1] for model, curve in curves.items()}
    print(f"Read: {_relative(csv_path)}")
    print(
        f"Final growth (relative to smallest graph): "
        f"Dijkstra {final['dijkstra']:.2f}x vs BMSSP {final['bmssp']:.2f}x"
    )
    print(f"Saved GIF: {_relative(args.output)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
