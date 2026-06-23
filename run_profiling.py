#!/usr/bin/env python3
"""Run reproducible timing and hardware-counter comparisons."""

from __future__ import annotations

import csv
import os
import re
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parent
EXECUTABLE = ROOT / "build" / "release" / "test" / "sssp_profile"
OUTPUT = ROOT / "output" / "profiling"
VERTICES = 100_000
DEGREE = 8
BANDWIDTH = 64
REPETITIONS = 5

ALGORITHMS = (
    "dijkstra-sequential",
    "dijkstra-transformed-sequential",
    "bmssp-sequential",
    "dijkstra-parallel",
    "bmssp-parallel",
)
MODES = ("path", "full")
TOPOLOGIES = ("random", "banded")

ELAPSED_PATTERN = re.compile(
    r"elapsed_seconds=(?P<elapsed>[0-9.eE+-]+) "
    r"seconds_per_run=(?P<per_run>[0-9.eE+-]+) "
    r"checksum=(?P<checksum>[0-9.eE+-]+)"
)
TRANSFORMED_PATTERN = re.compile(
    r"transformed_vertices=(?P<vertices>\d+) "
    r"transformed_edges=(?P<edges>\d+)"
)


def environment() -> dict[str, str]:
    return {
        **os.environ,
        "OMP_NUM_THREADS": "6",
        "OMP_PROC_BIND": "close",
        "OMP_PLACES": "cores",
    }


def command(
    algorithm: str, mode: str, topology: str, repetitions: int
) -> list[str]:
    return [
        str(EXECUTABLE),
        algorithm,
        mode,
        topology,
        str(VERTICES),
        str(DEGREE),
        str(BANDWIDTH),
        str(repetitions),
    ]


def run_timings() -> None:
    rows: list[dict[str, str | int | float]] = []
    for topology in TOPOLOGIES:
        for mode in MODES:
            for algorithm in ALGORITHMS:
                completed = subprocess.run(
                    command(algorithm, mode, topology, REPETITIONS),
                    cwd=ROOT,
                    env=environment(),
                    check=True,
                    text=True,
                    capture_output=True,
                )
                timing = ELAPSED_PATTERN.search(completed.stdout)
                if timing is None:
                    raise RuntimeError(f"missing timing output:\n{completed.stdout}")
                transformed = TRANSFORMED_PATTERN.search(completed.stdout)
                row = {
                    "algorithm": algorithm,
                    "mode": mode,
                    "topology": topology,
                    "vertices": VERTICES,
                    "edges": VERTICES * DEGREE - 1,
                    "degree": DEGREE,
                    "bandwidth": BANDWIDTH,
                    "repetitions": REPETITIONS,
                    "seconds_per_run": float(timing["per_run"]),
                    "checksum": float(timing["checksum"]),
                    "transformed_vertices": (
                        int(transformed["vertices"]) if transformed else ""
                    ),
                    "transformed_edges": (
                        int(transformed["edges"]) if transformed else ""
                    ),
                }
                rows.append(row)
                print(
                    f"{topology:6} {mode:4} {algorithm:34} "
                    f"{row['seconds_per_run']:.6f}s"
                )

    path = OUTPUT / "profile_summary.csv"
    OUTPUT.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    print(f"saved {path.relative_to(ROOT)}")


def run_perf_stat() -> None:
    events = (
        "task-clock,cycles,instructions,branches,branch-misses,"
        "cache-references,cache-misses"
    )
    rows: list[dict[str, str]] = []
    for algorithm in ALGORITHMS:
        completed = subprocess.run(
            [
                "perf",
                "stat",
                "-x,",
                "-e",
                events,
                *command(algorithm, "full", "random", 3),
            ],
            cwd=ROOT,
            env=environment(),
            check=True,
            text=True,
            capture_output=True,
        )
        metrics: dict[str, str] = {}
        for line in completed.stderr.splitlines():
            fields = line.split(",")
            if len(fields) >= 3 and fields[0] and fields[2]:
                metrics[fields[2]] = fields[0]
        rows.append({"algorithm": algorithm, **metrics})
        print(f"perf {algorithm}")

    metric_names = sorted({name for row in rows for name in row if name != "algorithm"})
    path = OUTPUT / "perf_summary.csv"
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=["algorithm", *metric_names])
        writer.writeheader()
        writer.writerows(rows)
    print(f"saved {path.relative_to(ROOT)}")


def main() -> int:
    subprocess.run(
        ["cmake", "--build", "--preset", "release", "--target", "sssp_profile"],
        cwd=ROOT,
        check=True,
    )
    run_timings()
    run_perf_stat()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
