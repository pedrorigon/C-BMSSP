#!/usr/bin/env python3
"""Benchmark the C++ BMSSP and Dijkstra solvers on full SNAP datasets.

Downloads the Stanford, Google, Pokec, and LiveJournal SNAP datasets as needed
(already-downloaded files are reused, never re-downloaded) and measures full
single-source shortest paths (source to every vertex). This keeps the comparison
aligned with the BMSSP vs Dijkstra theoretical complexity result. Each
measurement is repeated ``--iterations`` times (default 10) and aggregated with
1.5x IQR outlier removal and a 95% t-Student confidence interval. Results are
printed as a table and written to a CSV under ``output/``.

For a scalability study that varies the graph size, see ``run_scaling.py``.
"""

from __future__ import annotations

import argparse
import csv
import gzip
import shutil
import subprocess
import urllib.request
from collections import defaultdict
from datetime import datetime
from pathlib import Path
from typing import Any

from bench_common import Console, summarize_samples


ROOT = Path(__file__).resolve().parent
DATA_DIR = ROOT / "data"
OUTPUT_DIR = ROOT / "output"
BENCHMARK_EXECUTABLE = ROOT / "build" / "release" / "test" / "sssp_benchmark"

DEFAULT_ITERATIONS = 10

DATASETS = {
    "stanford": {
        "url": "https://snap.stanford.edu/data/web-Stanford.txt.gz",
        "filename": "web-Stanford.txt",
        "source": 235899,
    },
    "google": {
        "url": "https://snap.stanford.edu/data/web-Google.txt.gz",
        "filename": "web-Google.txt",
        "source": 895428,
    },
    "pokec": {
        "url": "https://snap.stanford.edu/data/soc-pokec-relationships.txt.gz",
        "filename": "soc-pokec-relationships.txt",
        "source": 1452585,
    },
    "livejournal": {
        "url": "https://snap.stanford.edu/data/soc-LiveJournal1.txt.gz",
        "filename": "soc-LiveJournal1.txt",
        "source": 1469803,
    },
}

CSV_FIELDS = [
    "run_timestamp",
    "dataset",
    "model",
    "type",
    "reachable",
    "iterations",
    "samples_used",
    "outliers_removed",
    "mean_time_seconds",
    "std_time_seconds",
    "ci_lower_seconds",
    "ci_upper_seconds",
    "min_time_seconds",
    "max_time_seconds",
    "dataset_vertices",
    "dataset_edges",
    "load_time_seconds",
    "source",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Benchmark the C++ BMSSP and Dijkstra implementations on full SNAP datasets."
    )
    parser.add_argument(
        "--dataset",
        choices=DATASETS,
        help="Run only the selected dataset. By default, all datasets are run.",
    )
    parser.add_argument(
        "--model",
        choices=("all", "bmssp", "dijkstra"),
        help="Run only the selected model. By default, both models are run.",
    )
    parser.add_argument(
        "--type",
        choices=("all", "sequential", "parallel"),
        help=(
            "Run only the selected implementation type. By default, all types are run. "
            "Use --type sequential for the direct SequentialSolver::solve_all vs "
            "DijkstraSolver::solve_all comparison."
        ),
    )
    parser.add_argument(
        "--iterations",
        type=int,
        default=DEFAULT_ITERATIONS,
        help=(
            "Number of timed runs per benchmark, averaged with IQR outlier removal "
            f"and a 95%% t-Student confidence interval (default: {DEFAULT_ITERATIONS})."
        ),
    )
    parser.add_argument(
        "--skip-build",
        action="store_true",
        help="Use the existing release benchmark executable without rebuilding it.",
    )
    args = parser.parse_args()
    if args.iterations < 1:
        parser.error("--iterations must be a positive integer")
    return args


def run_build(console: Console) -> None:
    console.step("Building the release benchmark executable")
    commands = (
        ["cmake", "--preset", "release"],
        ["cmake", "--build", "--preset", "release", "--target", "sssp_benchmark"],
    )
    for command in commands:
        completed = subprocess.run(command, cwd=ROOT, text=True, capture_output=True)
        if completed.returncode != 0:
            output = (completed.stdout + completed.stderr).strip()
            raise RuntimeError(f"Command failed: {' '.join(command)}\n{output}")
    console.success(f"Benchmark executable ready: {BENCHMARK_EXECUTABLE.relative_to(ROOT)}")


def download_progress(console: Console):
    last_percent = -10

    def report(block_count: int, block_size: int, total_size: int) -> None:
        nonlocal last_percent
        if total_size <= 0:
            return
        percent = min(100, block_count * block_size * 100 // total_size)
        if percent >= last_percent + 10:
            last_percent = percent
            console.detail(f"Download progress: {percent}%")

    return report


def prepare_dataset(name: str, console: Console) -> Path:
    info = DATASETS[name]
    final_path = DATA_DIR / str(info["filename"])
    if final_path.exists():
        console.success(f"Dataset found locally: {final_path.relative_to(ROOT)}")
        return final_path

    DATA_DIR.mkdir(parents=True, exist_ok=True)
    compressed_path = DATA_DIR / Path(str(info["url"])).name
    console.detail(f"Downloading {info['url']}")
    try:
        urllib.request.urlretrieve(
            str(info["url"]), compressed_path, reporthook=download_progress(console)
        )
        console.detail(f"Extracting {compressed_path.name}")
        with gzip.open(compressed_path, "rb") as source, final_path.open("wb") as destination:
            shutil.copyfileobj(source, destination)
    except Exception:
        final_path.unlink(missing_ok=True)
        raise
    finally:
        compressed_path.unlink(missing_ok=True)

    console.success(f"Dataset ready: {final_path.relative_to(ROOT)}")
    return final_path


def run_benchmark_once(
    command: list[str],
) -> tuple[dict[str, Any], dict[tuple[str, str], dict[str, Any]]]:
    """Run the executable once on the full graph; return dataset info and results."""
    process = subprocess.Popen(
        command, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1
    )
    if process.stdout is None:
        raise RuntimeError("failed to capture benchmark output")

    dataset_info: dict[str, Any] = {}
    results: dict[tuple[str, str], dict[str, Any]] = {}
    messages: list[str] = []
    for raw_line in process.stdout:
        line = raw_line.rstrip()
        parts = line.split("|")
        if parts[0] == "DATASET" and len(parts) == 4:
            dataset_info = {
                "dataset_vertices": int(parts[1]),
                "dataset_edges": int(parts[2]),
                "load_time_seconds": float(parts[3]) / 1000.0,
            }
        elif parts[0] == "RESULT" and len(parts) == 6:
            results[(parts[2], parts[3])] = {
                "model": parts[2],
                "type": parts[3],
                "reachable": int(parts[4]),
                "time_seconds": float(parts[5]) / 1000.0,
            }
        elif line:
            messages.append(line)

    if process.wait() != 0:
        raise RuntimeError("\n".join(messages) or "benchmark executable failed")
    return dataset_info, results


def run_dataset(
    dataset: str,
    path: Path,
    model: str,
    implementation_type: str,
    iterations: int,
    run_timestamp: str,
    console: Console,
) -> list[dict[str, Any]]:
    info = DATASETS[dataset]
    # No size arguments: the benchmark CLI then times solve_all on the complete graph.
    command = [
        str(BENCHMARK_EXECUTABLE),
        str(path),
        str(info["source"]),
        model,
        implementation_type,
    ]
    console.detail(f"Source={info['source']} Model={model} Type={implementation_type}")
    console.detail(f"Iterations per benchmark: {iterations}")

    dataset_info: dict[str, Any] = {}
    identities: dict[tuple[str, str], dict[str, Any]] = {}
    samples: dict[tuple[str, str], list[float]] = defaultdict(list)

    for iteration in range(1, iterations + 1):
        console.detail(f"Iteration {iteration}/{iterations}")
        info_run, results_run = run_benchmark_once(command)
        if not dataset_info:
            dataset_info = info_run
        for key, result in results_run.items():
            identities.setdefault(key, result)
            samples[key].append(result["time_seconds"])

    if dataset_info:
        console.success(
            f"Dataset loaded: {dataset_info['dataset_vertices']:,} vertices, "
            f"{dataset_info['dataset_edges']:,} edges in "
            f"{dataset_info['load_time_seconds']:.3f}s"
        )

    results: list[dict[str, Any]] = []
    for key, identity in identities.items():
        result = {
            "run_timestamp": run_timestamp,
            "dataset": dataset,
            "model": identity["model"],
            "type": identity["type"],
            "reachable": identity["reachable"],
            "iterations": iterations,
            **summarize_samples(samples[key]),
            **dataset_info,
            "source": info["source"],
        }
        results.append(result)
        spread = (
            f"[{result['ci_lower_seconds']:.6f}, {result['ci_upper_seconds']:.6f}]"
            f" n={result['samples_used']}/{result['iterations']}"
        )
        label = f"{identity['model']} / {identity['type']}"
        console.success(
            f"{label:<24} reachable={identity['reachable']:,}  "
            f"mean={result['mean_time_seconds']:.6f}s  95%CI={spread}"
        )
    return results


def output_path(args: argparse.Namespace, timestamp: str) -> Path:
    model = args.model if args.model else "all_models"
    implementation_type = args.type if args.type else "all_types"
    return OUTPUT_DIR / f"{timestamp}_{model}_{implementation_type}.csv"


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
    selected_datasets = [args.dataset] if args.dataset else list(DATASETS)
    model = args.model or "all"
    implementation_type = args.type or "all"
    results: list[dict[str, Any]] = []
    failures: list[str] = []

    console.title("C-BMSSP Dataset Benchmark")
    console.detail(f"Datasets: {', '.join(selected_datasets)}")
    console.detail(f"Model: {model} | Type: {implementation_type}")
    console.detail("Workload: full single-source shortest paths via solve_all(source)")
    console.detail(f"Iterations: {args.iterations}")

    try:
        if args.skip_build:
            if not BENCHMARK_EXECUTABLE.exists():
                raise RuntimeError(f"benchmark executable not found: {BENCHMARK_EXECUTABLE}")
            console.success("Using the existing release benchmark executable")
        else:
            run_build(console)
    except Exception as error:
        console.error(str(error))
        return 1

    for index, dataset in enumerate(selected_datasets, start=1):
        console.title(f"Dataset {index}/{len(selected_datasets)}: {dataset}")
        try:
            path = prepare_dataset(dataset, console)
            console.step(f"Running benchmarks for {dataset}")
            results.extend(
                run_dataset(
                    dataset, path, model, implementation_type, args.iterations, timestamp, console
                )
            )
        except Exception as error:
            failures.append(dataset)
            console.error(f"{dataset}: {error}")

    csv_path = output_path(args, timestamp)
    write_results(csv_path, results)
    elapsed = (datetime.now() - started_at).total_seconds()

    console.title("Benchmark Summary")
    console.success(f"Recorded {len(results)} result(s) in {csv_path.relative_to(ROOT)}")
    console.detail(f"Total elapsed time: {elapsed:.3f}s")
    if failures:
        console.error(f"Failed datasets: {', '.join(failures)}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
