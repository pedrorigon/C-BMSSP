#!/usr/bin/env python3
"""Scalability benchmark on generated sparse weighted graphs.

Generates random sparse directed graphs with real edge weights at increasing
sizes and measures full single-source shortest paths (source to every vertex)
for the C++ BMSSP and Dijkstra solvers. This synthetic workload scales far
beyond the SNAP datasets and is meant to expose the BMSSP asymptotic advantage
(``O(m log^(2/3) n)``) over Dijkstra (``O(m + n log n)``) as the graph grows.

Each size is run ``--iterations`` times (default 10); times are aggregated with
1.5x IQR outlier removal and a 95% t-Student confidence interval. Results are
written to a CSV under ``output/`` and plotted (linear and log-Y) under
``plots/scaling/``.
"""

from __future__ import annotations

import argparse
import csv
import subprocess
import sys
from collections import defaultdict
from datetime import datetime
from pathlib import Path
from typing import Any

from bench_common import Console, render_scaling_figure, summarize_samples


ROOT = Path(__file__).resolve().parent
OUTPUT_DIR = ROOT / "output"
PLOTS_DIR = ROOT / "plots"
SCALING_EXECUTABLE = ROOT / "build" / "release" / "test" / "sssp_scaling"

DEFAULT_ITERATIONS = 10
DEFAULT_DEGREE = 4
DEFAULT_SEED = 42
DEFAULT_MAX_VERTICES = 10_000_000
DEFAULT_STEPS = 12

CSV_FIELDS = [
    "run_timestamp",
    "model",
    "type",
    "avg_degree",
    "seed",
    "iterations",
    "samples_used",
    "outliers_removed",
    "mean_time_seconds",
    "std_time_seconds",
    "ci_lower_seconds",
    "ci_upper_seconds",
    "min_time_seconds",
    "max_time_seconds",
    "vertices",
    "edges",
    "reachable",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Scalability benchmark of BMSSP vs Dijkstra on generated sparse graphs."
    )
    parser.add_argument(
        "--model",
        choices=("bmssp", "dijkstra"),
        help="Run only the selected model. By default, both models are run.",
    )
    parser.add_argument(
        "--type",
        choices=("sequential", "parallel"),
        help="Run only the selected implementation type. By default, both types are run.",
    )
    parser.add_argument(
        "--iterations",
        type=int,
        default=DEFAULT_ITERATIONS,
        help=(
            "Number of timed runs per size, averaged with IQR outlier removal and a "
            f"95%% t-Student confidence interval (default: {DEFAULT_ITERATIONS})."
        ),
    )
    parser.add_argument(
        "--avg-degree",
        type=int,
        default=DEFAULT_DEGREE,
        help=f"Average out-degree of the generated graphs (default: {DEFAULT_DEGREE}).",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=DEFAULT_SEED,
        help=f"Random seed for graph generation (default: {DEFAULT_SEED}).",
    )
    parser.add_argument(
        "--max-vertices",
        type=int,
        default=DEFAULT_MAX_VERTICES,
        help=f"Largest graph size in vertices (default: {DEFAULT_MAX_VERTICES:,}).",
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=DEFAULT_STEPS,
        help=f"Number of geometrically spaced sizes to benchmark (default: {DEFAULT_STEPS}).",
    )
    parser.add_argument(
        "--skip-build",
        action="store_true",
        help="Use the existing release scaling executable without rebuilding it.",
    )
    args = parser.parse_args()
    if args.iterations < 1:
        parser.error("--iterations must be a positive integer")
    if args.avg_degree < 1:
        parser.error("--avg-degree must be a positive integer")
    if args.steps < 2:
        parser.error("--steps must be at least 2")
    if args.max_vertices < 1000:
        parser.error("--max-vertices must be at least 1000")
    return args


def build_sizes(max_vertices: int, steps: int) -> list[int]:
    """Geometrically spaced, rounded vertex counts from 1,000 to max_vertices."""
    start = 1_000
    if max_vertices <= start:
        return [max_vertices]
    ratio = (max_vertices / start) ** (1.0 / (steps - 1))
    sizes: list[int] = []
    for index in range(steps):
        value = start * (ratio**index)
        rounded = int(round(value, -2)) if value < 10_000 else int(round(value, -3))
        sizes.append(rounded)
    sizes[0] = start
    sizes[-1] = max_vertices
    # Deduplicate while preserving order (small step counts can collide).
    seen: set[int] = set()
    unique = [size for size in sizes if not (size in seen or seen.add(size))]
    return unique


def run_build(console: Console) -> None:
    console.step("Building the release scaling executable")
    commands = (
        ["cmake", "--preset", "release"],
        ["cmake", "--build", "--preset", "release", "--target", "sssp_scaling"],
    )
    for command in commands:
        completed = subprocess.run(command, cwd=ROOT, text=True, capture_output=True)
        if completed.returncode != 0:
            output = (completed.stdout + completed.stderr).strip()
            raise RuntimeError(f"Command failed: {' '.join(command)}\n{output}")
    console.success(f"Scaling executable ready: {SCALING_EXECUTABLE.relative_to(ROOT)}")


def run_scaling_once(
    model: str, implementation_type: str, degree: int, seed: int, sizes: list[int]
) -> tuple[dict[int, dict[str, int]], dict[tuple[int, str, str], dict[str, Any]]]:
    """Run the executable once over all sizes; return graph metadata and results."""
    command = [
        str(SCALING_EXECUTABLE),
        model,
        implementation_type,
        str(degree),
        str(seed),
        *(str(size) for size in sizes),
    ]
    process = subprocess.Popen(
        command, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1
    )
    if process.stdout is None:
        raise RuntimeError("failed to capture scaling output")

    graphs: dict[int, dict[str, int]] = {}
    results: dict[tuple[int, str, str], dict[str, Any]] = {}
    messages: list[str] = []
    for raw_line in process.stdout:
        line = raw_line.rstrip()
        parts = line.split("|")
        if parts[0] == "GRAPH" and len(parts) == 4:
            graphs[int(parts[1])] = {"vertices": int(parts[2]), "edges": int(parts[3])}
        elif parts[0] == "RESULT" and len(parts) == 6:
            step = int(parts[1])
            results[(step, parts[2], parts[3])] = {
                "model": parts[2],
                "type": parts[3],
                "reachable": int(parts[4]),
                "time_seconds": float(parts[5]) / 1000.0,
            }
        elif line:
            messages.append(line)

    if process.wait() != 0:
        raise RuntimeError("\n".join(messages) or "scaling executable failed")
    return graphs, results


def run_scaling(
    model: str,
    implementation_type: str,
    args: argparse.Namespace,
    sizes: list[int],
    run_timestamp: str,
    console: Console,
) -> list[dict[str, Any]]:
    graphs: dict[int, dict[str, int]] = {}
    identities: dict[tuple[int, str, str], dict[str, Any]] = {}
    samples: dict[tuple[int, str, str], list[float]] = defaultdict(list)

    for iteration in range(1, args.iterations + 1):
        console.detail(f"Iteration {iteration}/{args.iterations}")
        graphs_run, results_run = run_scaling_once(
            model, implementation_type, args.avg_degree, args.seed, sizes
        )
        if not graphs:
            graphs = graphs_run
        for key, result in results_run.items():
            identities.setdefault(key, result)
            samples[key].append(result["time_seconds"])

    results: list[dict[str, Any]] = []
    last_step: int | None = None
    for (step, _model, _type), identity in identities.items():
        graph = graphs[step]
        if step != last_step:
            console.step(
                f"Size {graph['vertices']:,} vertices, {graph['edges']:,} edges"
            )
            last_step = step
        result = {
            "run_timestamp": run_timestamp,
            "model": identity["model"],
            "type": identity["type"],
            "avg_degree": args.avg_degree,
            "seed": args.seed,
            "iterations": args.iterations,
            "reachable": identity["reachable"],
            **summarize_samples(samples[(step, _model, _type)]),
            **graph,
        }
        results.append(result)
        spread = (
            f"[{result['ci_lower_seconds']:.6f}, {result['ci_upper_seconds']:.6f}]"
            f" n={result['samples_used']}/{result['iterations']}"
        )
        console.success(
            f"{identity['model']} / {identity['type']:<10} "
            f"mean={result['mean_time_seconds']:.6f}s  95%CI={spread}"
        )
    return results


def write_results(path: Path, results: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(results)


def main() -> int:
    args = parse_args()
    console = Console()
    started_at = datetime.now()
    timestamp = started_at.strftime("%Y%m%d_%H%M%S")
    model = args.model or "all"
    implementation_type = args.type or "all"
    sizes = build_sizes(args.max_vertices, args.steps)

    console.title("C-BMSSP Scalability Benchmark (generated graphs)")
    console.detail(f"Model: {model} | Type: {implementation_type}")
    console.detail(f"Avg degree: {args.avg_degree} | Seed: {args.seed} | Iterations: {args.iterations}")
    console.detail(f"Sizes (vertices): {', '.join(f'{size:,}' for size in sizes)}")

    try:
        if args.skip_build:
            if not SCALING_EXECUTABLE.exists():
                raise RuntimeError(f"scaling executable not found: {SCALING_EXECUTABLE}")
            console.success("Using the existing release scaling executable")
        else:
            run_build(console)
        results = run_scaling(model, implementation_type, args, sizes, timestamp, console)
    except Exception as error:
        console.error(str(error))
        return 1

    csv_path = OUTPUT_DIR / f"{timestamp}_scaling_{model}_{implementation_type}.csv"
    write_results(csv_path, results)

    console.step("Generating scaling plots")
    subtitle = (
        f"Generated sparse graphs (avg degree {args.avg_degree}), full SSSP, "
        f"mean of {args.iterations} runs (IQR outlier removal, 95% t-Student CI)"
    )
    target_dir = PLOTS_DIR / "scaling"
    base = f"{timestamp}_scaling"
    render_scaling_figure(
        results, "vertices", "Graph size (number of vertices)", subtitle, False,
        target_dir / f"{base}.pdf", target_dir / f"{base}.jpg", ROOT, console,
    )
    render_scaling_figure(
        results, "vertices", "Graph size (number of vertices)", subtitle, True,
        target_dir / f"{base}_log.pdf", target_dir / f"{base}_log.jpg", ROOT, console,
    )

    elapsed = (datetime.now() - started_at).total_seconds()
    console.title("Scalability Summary")
    console.success(f"Recorded {len(results)} result(s) in {csv_path.relative_to(ROOT)}")
    console.detail(f"Total elapsed time: {elapsed:.3f}s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
