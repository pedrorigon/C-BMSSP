#!/usr/bin/env python3
"""Shared helpers for the dataset and scalability benchmarks.

Holds the console formatter, the statistics aggregation (IQR outlier removal plus
a 95% t-Student confidence interval) and the scaling-plot renderer used by both
``run_benchmarks.py`` and ``run_scaling.py``.
"""

from __future__ import annotations

import math
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any

try:
    import matplotlib
except ImportError as error:  # pragma: no cover - dependency guard
    raise SystemExit(
        "matplotlib is required to generate the scaling plots. "
        "Install it with: pip install matplotlib"
    ) from error

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

try:
    from scipy import stats as scipy_stats
except ImportError as error:  # pragma: no cover - dependency guard
    raise SystemExit(
        "scipy is required for the t-Student confidence interval. "
        "Install it with: pip install scipy"
    ) from error


CONFIDENCE_LEVEL = 0.95
# Smallest positive value used as the lower bound of a log-scale CI band, since
# log axes cannot show zero or negative values.
LOG_FLOOR = 1e-7

SERIES_STYLES = {
    ("bmssp", "sequential"): {"color": "#2563eb"},
    ("bmssp", "parallel"): {"color": "#06b6d4"},
    ("dijkstra", "sequential"): {"color": "#64748b"},
    ("dijkstra", "parallel"): {"color": "#ec4899"},
}

# Portuguese series labels for the complexity figure legend.
SERIES_LABELS_PT = {
    ("bmssp", "sequential"): "BMSSP sequencial",
    ("bmssp", "parallel"): "BMSSP paralelo",
    ("dijkstra", "sequential"): "Dijkstra sequencial",
    ("dijkstra", "parallel"): "Dijkstra paralelo",
}


class Console:
    """Small ANSI-aware formatter for readable benchmark progress."""

    def __init__(self) -> None:
        self.color = sys.stdout.isatty()

    def _style(self, text: str, code: str) -> str:
        return f"\033[{code}m{text}\033[0m" if self.color else text

    def title(self, text: str) -> None:
        line = "=" * 72
        print(
            f"\n{self._style(line, '1;36')}\n{self._style(text, '1;36')}\n{self._style(line, '1;36')}"
        )

    def step(self, text: str) -> None:
        print(f"\n{self._style('>>', '1;34')} {self._style(text, '1')}")

    def success(self, text: str) -> None:
        print(f"{self._style('OK', '1;32')} {text}")

    def error(self, text: str) -> None:
        print(f"{self._style('ERROR', '1;31')} {text}")

    def detail(self, text: str) -> None:
        print(f"   {text}")


def _quantile(ordered: list[float], fraction: float) -> float:
    """Linear-interpolation quantile over an already sorted list."""
    if len(ordered) == 1:
        return ordered[0]
    position = fraction * (len(ordered) - 1)
    low = math.floor(position)
    high = math.ceil(position)
    if low == high:
        return ordered[low]
    weight = position - low
    return ordered[low] * (1.0 - weight) + ordered[high] * weight


def remove_outliers_iqr(samples: list[float]) -> tuple[list[float], int]:
    """Drop samples outside the 1.5x IQR fences; keep all if too few to judge."""
    if len(samples) < 4:
        return list(samples), 0
    ordered = sorted(samples)
    q1 = _quantile(ordered, 0.25)
    q3 = _quantile(ordered, 0.75)
    iqr = q3 - q1
    lower = q1 - 1.5 * iqr
    upper = q3 + 1.5 * iqr
    kept = [value for value in samples if lower <= value <= upper]
    if not kept:
        return list(samples), 0
    return kept, len(samples) - len(kept)


def summarize_samples(samples: list[float]) -> dict[str, Any]:
    """Mean and 95% t-Student CI after IQR outlier removal."""
    kept, removed = remove_outliers_iqr(samples)
    count = len(kept)
    mean = sum(kept) / count
    if count > 1:
        variance = sum((value - mean) ** 2 for value in kept) / (count - 1)
        std = math.sqrt(variance)
        critical = float(scipy_stats.t.ppf(1.0 - (1.0 - CONFIDENCE_LEVEL) / 2.0, count - 1))
        margin = critical * std / math.sqrt(count)
    else:
        std = 0.0
        margin = 0.0
    return {
        "samples_used": count,
        "outliers_removed": removed,
        "mean_time_seconds": mean,
        "std_time_seconds": std,
        "ci_lower_seconds": mean - margin,
        "ci_upper_seconds": mean + margin,
        "min_time_seconds": min(kept),
        "max_time_seconds": max(kept),
    }


def format_count(value: int) -> str:
    """Render a graph size as a compact label: 5000 -> '5K', 1000000 -> '1M'."""
    if value >= 1_000_000 and value % 1_000_000 == 0:
        return f"{value // 1_000_000}M"
    if value >= 1_000_000:
        return f"{value / 1_000_000:g}M"
    if value >= 1_000 and value % 1_000 == 0:
        return f"{value // 1_000}K"
    if value >= 1_000:
        return f"{value / 1_000:g}K"
    return str(value)


def render_scaling_figure(
    results: list[dict[str, Any]],
    x_key: str,
    x_axis_label: str,
    subtitle: str,
    log_scale: bool,
    pdf_path: Path,
    jpg_path: Path,
    root: Path,
    console: Console,
) -> None:
    """Draw and save one scaling figure (linear or log-Y) as PDF and JPG.

    ``results`` is a flat list of aggregated points; each must carry ``model``,
    ``type``, the chosen ``x_key`` and the timing statistics. The x-axis renders
    the distinct ``x_key`` values as evenly spaced categories so the curve is
    read at a regular cadence regardless of the spacing of the underlying sizes.
    """
    series: dict[tuple[str, str], dict[int, dict[str, Any]]] = defaultdict(dict)
    sizes: set[int] = set()
    for result in results:
        size = int(result[x_key])
        series[(result["model"], result["type"])][size] = result
        sizes.add(size)
    if not series or not sizes:
        console.detail("No data points to plot.")
        return

    ordered_sizes = sorted(sizes)
    position = {size: index for index, size in enumerate(ordered_sizes)}

    plt.style.use("default")
    figure, axis = plt.subplots(figsize=(11, 6))
    figure.patch.set_facecolor("white")
    axis.set_facecolor("white")

    for key, points_by_size in series.items():
        points = [points_by_size[size] for size in ordered_sizes if size in points_by_size]
        xs = [position[int(item[x_key])] for item in points]
        means = [item["mean_time_seconds"] for item in points]
        upper = [item["ci_upper_seconds"] for item in points]
        if log_scale:
            means = [max(value, LOG_FLOOR) for value in means]
            lower = [max(item["ci_lower_seconds"], LOG_FLOOR) for item in points]
        else:
            lower = [item["ci_lower_seconds"] for item in points]
        color = SERIES_STYLES.get(key, {}).get("color")
        axis.plot(
            xs,
            means,
            label=f"{key[0]} / {key[1]}",
            color=color,
            linewidth=2.2,
            zorder=3,
            solid_capstyle="round",
        )
        axis.fill_between(xs, lower, upper, color=color, alpha=0.20, linewidth=0, zorder=1)
        axis.scatter(xs, means, color=color, s=42, edgecolors="white", linewidths=1.2, zorder=4)

    if log_scale:
        axis.set_yscale("log")
    axis.set_title("Performance Scaling", fontsize=15, fontweight="bold", pad=14)
    if subtitle:
        axis.text(
            0.5, 1.01, subtitle, transform=axis.transAxes, ha="center", va="bottom", fontsize=9,
            color="#475569",
        )
    axis.set_xlabel(x_axis_label)
    ylabel = "Execution time (seconds, log scale)" if log_scale else "Execution time (seconds)"
    axis.set_ylabel(ylabel)
    axis.set_xticks(range(len(ordered_sizes)))
    axis.set_xticklabels([format_count(size) for size in ordered_sizes])
    axis.margins(x=0.02)
    axis.grid(True, which="both", linestyle="--", linewidth=0.7, alpha=0.45)
    for spine in ("top", "right"):
        axis.spines[spine].set_visible(False)
    axis.legend(title="Model / Type", frameon=True, framealpha=0.9)
    figure.tight_layout()

    pdf_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(pdf_path)
    figure.savefig(jpg_path, dpi=200)
    plt.close(figure)
    console.success(
        f"Saved scaling plots: {pdf_path.relative_to(root)} and {jpg_path.relative_to(root)}"
    )


def render_complexity_figure(
    results: list[dict[str, Any]],
    x_key: str,
    x_axis_label: str,
    subtitle: str,
    pdf_path: Path,
    jpg_path: Path,
    root: Path,
    console: Console,
) -> None:
    """Draw the empirical-complexity figure: weighted work per edge vs graph size.

    Wall-clock time cannot show the BMSSP advantage; the constant-degree blow-up
    and the larger constant factor dominate every in-memory input. The complexity
    claim is about the *number* of ordered-structure operations the analysis
    charges (``work_counter.h``). Dividing that work by the edge count gives the
    work-per-edge, which grows like ``log n`` for Dijkstra and ``log^(2/3) n`` for
    BMSSP. Two panels are drawn:

    * left: raw work/edge, showing the real shape (BMSSP flattens, Dijkstra does
      not) while honestly keeping BMSSP's larger constant above Dijkstra;
    * right: each curve divided by its own first point, so both start at 1.0 and
      the *growth rate* is compared directly -- Dijkstra climbs above BMSSP,
      exposing the smaller exponent.
    """
    series: dict[tuple[str, str], dict[int, dict[str, Any]]] = defaultdict(dict)
    sizes: set[int] = set()
    for result in results:
        if "work_per_edge" not in result:
            continue
        size = int(result[x_key])
        series[(result["model"], result["type"])][size] = result
        sizes.add(size)
    if not series or not sizes:
        console.detail("No work data points to plot.")
        return

    ordered_sizes = sorted(sizes)
    position = {size: index for index, size in enumerate(ordered_sizes)}

    plt.style.use("default")
    figure, (axis_raw, axis_norm) = plt.subplots(1, 2, figsize=(15, 6))
    figure.patch.set_facecolor("white")

    for axis in (axis_raw, axis_norm):
        axis.set_facecolor("white")

    for key, points_by_size in sorted(series.items()):
        points = [points_by_size[size] for size in ordered_sizes if size in points_by_size]
        if not points:
            continue
        xs = [position[int(item[x_key])] for item in points]
        work_per_edge = [item["work_per_edge"] for item in points]
        baseline = work_per_edge[0] if work_per_edge[0] else 1.0
        normalized = [value / baseline for value in work_per_edge]
        color = SERIES_STYLES.get(key, {}).get("color")
        label = SERIES_LABELS_PT.get(key, f"{key[0]} / {key[1]}")
        for axis, ys in ((axis_raw, work_per_edge), (axis_norm, normalized)):
            axis.plot(
                xs, ys, label=label, color=color, linewidth=2.4, zorder=3,
                solid_capstyle="round",
            )
            axis.scatter(xs, ys, color=color, s=46, edgecolors="white", linewidths=1.2, zorder=4)

    axis_raw.set_title("Trabalho por aresta", fontsize=13, fontweight="bold", pad=10)
    axis_raw.set_ylabel("Operações / aresta")
    axis_norm.set_title(
        "Taxa de crescimento (relativa ao menor grafo)", fontsize=13, fontweight="bold", pad=10
    )
    axis_norm.set_ylabel("Trabalho/aresta relativo")

    for axis in (axis_raw, axis_norm):
        axis.set_xlabel("Número de vértices")
        axis.set_xticks(range(len(ordered_sizes)))
        axis.set_xticklabels([format_count(size) for size in ordered_sizes])
        axis.margins(x=0.02)
        axis.grid(True, which="both", linestyle="--", linewidth=0.7, alpha=0.45)
        for spine in ("top", "right"):
            axis.spines[spine].set_visible(False)
        axis.legend(title="Algoritmo", frameon=True, framealpha=0.9)

    figure.suptitle(
        "Complexidade empírica: BMSSP O(m log^(2/3) n) vs Dijkstra O(m log n)",
        fontsize=15, fontweight="bold",
    )
    if subtitle:
        figure.text(0.5, 0.925, subtitle, ha="center", va="bottom", fontsize=9, color="#475569")
    figure.tight_layout(rect=(0, 0, 1, 0.93))

    pdf_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(pdf_path)
    figure.savefig(jpg_path, dpi=200)
    plt.close(figure)
    console.success(
        f"Saved complexity plots: {pdf_path.relative_to(root)} and {jpg_path.relative_to(root)}"
    )


def _rel(path: Path, root: Path) -> str:
    """Path relative to root when possible, else the path unchanged."""
    try:
        return str(path.relative_to(root))
    except ValueError:
        return str(path)


def _format_big_n(value: float) -> str:
    """Render a large vertex count as a power of ten, e.g. 1.2e11 -> '~10^11'."""
    if value <= 0:
        return "n/d"
    exponent = math.log10(value)
    return f"~10^{exponent:.0f}"


def render_crossover_figure(
    results: list[dict[str, Any]],
    pdf_path: Path,
    jpg_path: Path,
    root: Path,
    console: Console,
) -> None:
    """Plot the BMSSP/Dijkstra work-per-edge ratio and extrapolate its crossover.

    The ratio of weighted work per edge between BMSSP and Dijkstra falls roughly
    linearly with ``ln(n)`` -- the empirical signature of ``log^(2/3) n`` growing
    slower than ``log n``. Fitting that line and extending it to the point where it
    reaches 1.0 estimates the graph size at which BMSSP would do *less* total work
    than Dijkstra. That size is far beyond any in-memory input, so the dashed
    extrapolation is labelled explicitly as a projection from measured points, not
    a measurement -- honest for a slide.
    """
    by_size: dict[int, dict[str, float]] = defaultdict(dict)
    for result in results:
        if result.get("type") != "sequential" or "work_per_edge" not in result:
            continue
        by_size[int(result["vertices"])][result["model"]] = float(result["work_per_edge"])

    points = sorted(
        (size, models["bmssp"] / models["dijkstra"])
        for size, models in by_size.items()
        if "bmssp" in models and "dijkstra" in models and models["dijkstra"] > 0
    )
    if len(points) < 2:
        console.detail("Not enough points to draw the crossover extrapolation.")
        return

    sizes = [size for size, _ in points]
    ratios = [ratio for _, ratio in points]
    ln_sizes = [math.log(size) for size in sizes]

    # Least-squares fit ratio ~ slope * ln(n) + intercept.
    count = len(points)
    mean_x = sum(ln_sizes) / count
    mean_y = sum(ratios) / count
    denominator = sum((x - mean_x) ** 2 for x in ln_sizes)
    slope = (
        sum((x - mean_x) * (y - mean_y) for x, y in zip(ln_sizes, ratios)) / denominator
        if denominator
        else 0.0
    )
    intercept = mean_y - slope * mean_x
    crossover_n = math.exp((1.0 - intercept) / slope) if slope < 0 else 0.0

    plt.style.use("default")
    figure, axis = plt.subplots(figsize=(11, 6))
    figure.patch.set_facecolor("white")
    axis.set_facecolor("white")

    measured_color = "#7c3aed"
    axis.plot(ln_sizes, ratios, color=measured_color, linewidth=2.6, zorder=3,
              solid_capstyle="round", label="Razão medida (BMSSP / Dijkstra)")
    axis.scatter(ln_sizes, ratios, color=measured_color, s=52, edgecolors="white",
                 linewidths=1.3, zorder=4)

    # Extrapolated fit out to the crossover (or one decade past the data).
    right_ln = math.log(crossover_n) if crossover_n > sizes[-1] else ln_sizes[-1] + math.log(10)
    fit_x = [ln_sizes[0], right_ln]
    fit_y = [slope * x + intercept for x in fit_x]
    axis.plot(fit_x, fit_y, color="#475569", linewidth=1.8, linestyle="--", zorder=2,
              label="Ajuste linear em ln(n) (extrapolado)")
    axis.axhline(1.0, color="#dc2626", linewidth=1.4, linestyle=":", zorder=1,
                 label="Empate de trabalho (razão = 1)")

    if crossover_n > sizes[-1]:
        cross_x = math.log(crossover_n)
        axis.scatter([cross_x], [1.0], color="#dc2626", s=80, zorder=5, marker="X")
        axis.annotate(
            f"Cruzamento projetado: n {_format_big_n(crossover_n)}",
            xy=(cross_x, 1.0), xytext=(cross_x, 1.0 + (max(ratios) - 1.0) * 0.28),
            ha="center", color="#b91c1c", fontsize=10, fontweight="bold",
            arrowprops={"arrowstyle": "->", "color": "#b91c1c"},
        )

    axis.set_title(
        "Razão de trabalho BMSSP/Dijkstra cai com o tamanho do grafo",
        fontsize=14, fontweight="bold", pad=26,
    )
    axis.text(
        0.5, 1.03,
        "Tendência medida extrapolada até o empate de trabalho (projeção além da memória)",
        transform=axis.transAxes, ha="center", va="bottom", fontsize=9, color="#475569",
    )
    axis.set_xlabel("ln(número de vértices)")
    axis.set_ylabel("Trabalho/aresta: BMSSP ÷ Dijkstra")

    # Secondary x ticks showing the actual sizes at the measured points.
    tick_positions = ln_sizes + ([math.log(crossover_n)] if crossover_n > sizes[-1] else [])
    tick_labels = [format_count(size) for size in sizes] + (
        [_format_big_n(crossover_n)] if crossover_n > sizes[-1] else []
    )
    axis.set_xticks(tick_positions)
    axis.set_xticklabels(tick_labels, rotation=30, ha="right")
    axis.margins(x=0.04)
    axis.grid(True, linestyle="--", linewidth=0.7, alpha=0.45)
    for spine in ("top", "right"):
        axis.spines[spine].set_visible(False)
    axis.legend(frameon=True, framealpha=0.9, loc="upper right")
    figure.tight_layout()

    pdf_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(pdf_path)
    figure.savefig(jpg_path, dpi=200)
    plt.close(figure)
    console.success(
        f"Saved crossover plot: {_rel(pdf_path, root)} and {_rel(jpg_path, root)} "
        f"(projected crossover at n {_format_big_n(crossover_n)})"
    )
