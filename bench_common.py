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
