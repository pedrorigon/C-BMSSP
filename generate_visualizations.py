#!/usr/bin/env python3
"""Generate smooth 10-second neon city animations for Dijkstra and BMSSP."""

from __future__ import annotations

import argparse
import bisect
import functools
import math
import os
import subprocess
from dataclasses import dataclass, field
from pathlib import Path

from PIL import Image, ImageDraw, ImageFilter, ImageFont


ROOT = Path(__file__).resolve().parent
EXECUTABLE = ROOT / "build" / "release" / "test" / "sssp_visualization_trace"
OUTPUT_DIR = ROOT / "visualizations"

FPS = 20
TOTAL_SECONDS = 10
FRAME_COUNT = FPS * TOTAL_SECONDS

COLORS = {
    "background": "#070510",
    "panel": "#0d0918",
    "text": "#f8fafc",
    "muted": "#a78bfa",
    "border": "#34234a",
    "road": "#351234",
    "road_alt": "#49202f",
    "dijkstra": "#ff2f86",
    "dijkstra_frontier": "#ff8a3d",
    "bmssp": "#00e5ff",
    "bmssp_frontier": "#b6ff36",
    "source": "#60ff8b",
    "goal": "#ff315c",
    "path": "#ffffff",
}

DISPLAY_NAMES = {
    "dijkstra-sequential": "Dijkstra · Sequential",
    "bmssp-sequential": "BMSSP · Sequential",
    "dijkstra-parallel": "Dijkstra · Parallel",
    "bmssp-parallel": "BMSSP · Parallel",
}


@dataclass
class Event:
    relative_time: float
    kind: str
    vertex: int


@dataclass
class Run:
    name: str
    measured_seconds: float
    raw_events: list[tuple[float, str, int]] = field(default_factory=list)
    events: list[Event] = field(default_factory=list)
    path: list[int] = field(default_factory=list)


@dataclass
class TraceData:
    width: int
    height: int
    source: int
    goal: int
    coordinates: dict[int, tuple[float, float]]
    edges: list[tuple[int, int]]
    adjacency: dict[int, list[int]]
    runs: dict[str, Run]


@functools.lru_cache(maxsize=None)
def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    return ImageFont.truetype(f"/usr/share/fonts/truetype/dejavu/{name}", size)


def build_trace_executable() -> None:
    for command in (
        ["cmake", "--preset", "release"],
        ["cmake", "--build", "--preset", "release", "--target", "sssp_visualization_trace"],
    ):
        subprocess.run(command, cwd=ROOT, check=True)


def capture_trace(mode: str) -> Path:
    output_dir = ROOT / "output" / "profiling"
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / f"visual_trace_{mode}.txt"
    completed = subprocess.run(
        [str(EXECUTABLE), mode],
        cwd=ROOT,
        check=True,
        text=True,
        capture_output=True,
        env={**os.environ, "OMP_NUM_THREADS": "6"},
    )
    path.write_text(completed.stdout, encoding="utf-8")
    return path


def parse_trace(path: Path) -> TraceData:
    width = height = source = goal = 0
    coordinates: dict[int, tuple[float, float]] = {}
    edges: list[tuple[int, int]] = []
    adjacency: dict[int, list[int]] = {}
    runs: dict[str, Run] = {}

    for line in path.read_text(encoding="utf-8").splitlines():
        parts = line.split("|")
        if parts[0] == "META":
            width, height, source, goal = map(int, parts[1:5])
        elif parts[0] == "NODE":
            coordinates[int(parts[1])] = (float(parts[2]), float(parts[3]))
        elif parts[0] == "EDGE":
            left, right = int(parts[1]), int(parts[2])
            edges.append((left, right))
            adjacency.setdefault(left, []).append(right)
            adjacency.setdefault(right, []).append(left)
        elif parts[0] == "RUN":
            name = parts[2]
            runs[name] = Run(name=name, measured_seconds=float(parts[3]))
        elif parts[0] == "PATH":
            runs[parts[2]].path = [int(value) for value in parts[3:]]
        elif parts[0] == "TRACE":
            runs[parts[2]].raw_events.append(
                (float(parts[4]), parts[5], int(parts[6]))
            )

    # Callback timestamps contain unavoidable scheduling bursts. The measured
    # total duration remains real, while events are resampled uniformly by rank
    # to produce continuous visual propagation at 20 FPS.
    for run in runs.values():
        count = len(run.raw_events)
        run.events = [
            Event(run.measured_seconds * (index + 1) / max(count, 1), kind, vertex)
            for index, (_, kind, vertex) in enumerate(run.raw_events)
        ]
    return TraceData(
        width, height, source, goal, coordinates, edges, adjacency, runs
    )


def panel_layout(count: int) -> tuple[int, int, int, int]:
    if count == 2:
        return 2, 1, 1200, 520
    return 2, 2, 1200, 900


def map_coordinates(
    data: TraceData, box: tuple[int, int, int, int]
) -> dict[int, tuple[float, float]]:
    left, top, right, bottom = box
    padding = 15
    scale = min(
        (right - left - 2 * padding) / max(data.width - 1, 1),
        (bottom - top - 2 * padding) / max(data.height - 1, 1),
    )
    used_width = scale * (data.width - 1)
    used_height = scale * (data.height - 1)
    offset_x = left + (right - left - used_width) / 2
    offset_y = top + (bottom - top - used_height) / 2
    return {
        vertex: (offset_x + x * scale, offset_y + y * scale)
        for vertex, (x, y) in data.coordinates.items()
    }


def glow_lines(
    size: tuple[int, int],
    lines: list[tuple[tuple[float, float], tuple[float, float]]],
    color: str,
    core_width: int,
    blur_radius: int,
    origin: tuple[int, int] = (0, 0),
) -> Image.Image:
    origin_x, origin_y = origin
    local_lines = [
        (
            (start[0] - origin_x, start[1] - origin_y),
            (end[0] - origin_x, end[1] - origin_y),
        )
        for start, end in lines
    ]
    glow = Image.new("RGBA", size, (0, 0, 0, 0))
    glow_draw = ImageDraw.Draw(glow)
    for start, end in local_lines:
        glow_draw.line((start, end), fill=color, width=core_width * 4)
    glow = glow.filter(ImageFilter.GaussianBlur(blur_radius))

    core = Image.new("RGBA", size, (0, 0, 0, 0))
    core_draw = ImageDraw.Draw(core)
    for start, end in local_lines:
        core_draw.line((start, end), fill=color, width=core_width)
    glow.alpha_composite(core)
    return glow


def draw_glowing_point(
    image: Image.Image,
    position: tuple[float, float],
    color: str,
    radius: int,
) -> None:
    diameter = radius * 8 + 1
    layer = Image.new("RGBA", (diameter, diameter), (0, 0, 0, 0))
    draw = ImageDraw.Draw(layer)
    center = diameter // 2
    draw.ellipse(
        (
            center - radius * 3,
            center - radius * 3,
            center + radius * 3,
            center + radius * 3,
        ),
        fill=color,
    )
    layer = layer.filter(ImageFilter.GaussianBlur(radius * 2))
    x, y = position
    destination = (round(x) - center, round(y) - center)
    image.alpha_composite(layer, dest=destination)
    draw = ImageDraw.Draw(image)
    draw.ellipse(
        (x - radius, y - radius, x + radius, y + radius), fill="#ffffff"
    )


def render_panel(
    image: Image.Image,
    data: TraceData,
    run: Run,
    box: tuple[int, int, int, int],
    animation_seconds: float,
    comparison_max_seconds: float,
) -> None:
    draw = ImageDraw.Draw(image)
    left, top, right, bottom = box
    draw.rounded_rectangle(box, radius=18, fill=COLORS["panel"], outline=COLORS["border"], width=2)
    draw.text((left + 18, top + 13), DISPLAY_NAMES[run.name], fill=COLORS["text"], font=font(18, True))

    algorithm_seconds = comparison_max_seconds * animation_seconds / TOTAL_SECONDS
    shown_seconds = min(algorithm_seconds, run.measured_seconds)
    finished = algorithm_seconds >= run.measured_seconds
    if run.measured_seconds < 0.001:
        timing = f"{run.measured_seconds * 1e6:.1f} µs"
    else:
        timing = f"{run.measured_seconds * 1e3:.2f} ms"
    draw.text(
        (right - 18, top + 17),
        f"{timing} · {'finished' if finished else 'running'}",
        fill=COLORS["muted"],
        font=font(12),
        anchor="ra",
    )

    road_box = (left + 12, top + 43, right - 12, bottom - 26)
    points = map_coordinates(data, road_box)

    for edge_index, (first, second) in enumerate(data.edges):
        road_color = COLORS["road_alt"] if edge_index % 7 == 0 else COLORS["road"]
        draw.line((points[first], points[second]), fill=road_color, width=2)

    times = [event.relative_time for event in run.events]
    event_limit = bisect.bisect_right(times, shown_seconds)
    done: set[int] = set()
    frontier: set[int] = set()
    recent: list[int] = []
    for event in run.events[:event_limit]:
        if event.kind in {"settled", "completion"}:
            done.add(event.vertex)
            frontier.discard(event.vertex)
        else:
            if event.vertex not in done:
                frontier.add(event.vertex)
        recent.append(event.vertex)
    active = set(recent[-42:]) if not finished else set()

    base_color = COLORS["bmssp"] if run.name.startswith("bmssp") else COLORS["dijkstra"]
    frontier_color = (
        COLORS["bmssp_frontier"]
        if run.name.startswith("bmssp")
        else COLORS["dijkstra_frontier"]
    )
    done_lines = [
        (points[first], points[second])
        for first, second in data.edges
        if first in done and second in done
    ]
    if done_lines:
        image.alpha_composite(
            glow_lines(
                (right - left, bottom - top),
                done_lines,
                base_color,
                2,
                7,
                (left, top),
            ),
            dest=(left, top),
        )

    frontier_vertices = frontier | active
    frontier_lines = [
        (points[first], points[second])
        for first, second in data.edges
        if (first in frontier_vertices and second in done)
        or (second in frontier_vertices and first in done)
    ]
    if frontier_lines:
        image.alpha_composite(
            glow_lines(
                (right - left, bottom - top),
                frontier_lines,
                frontier_color,
                2,
                9,
                (left, top),
            ),
            dest=(left, top),
        )

    if finished and run.path:
        path_lines = [
            (points[first], points[second])
            for first, second in zip(run.path, run.path[1:])
            if first in points and second in points
        ]
        image.alpha_composite(
            glow_lines(
                (right - left, bottom - top),
                path_lines,
                COLORS["path"],
                3,
                11,
                (left, top),
            ),
            dest=(left, top),
        )

    pulse = 1.0 + 0.25 * math.sin(animation_seconds * math.tau * 1.2)
    draw_glowing_point(image, points[data.source], COLORS["source"], round(3 * pulse))
    draw_glowing_point(image, points[data.goal], COLORS["goal"], round(3 * pulse))

    progress = min(shown_seconds / run.measured_seconds, 1.0) if run.measured_seconds else 1.0
    bar_left, bar_right = left + 18, right - 18
    bar_y = bottom - 14
    draw.rounded_rectangle((bar_left, bar_y, bar_right, bar_y + 5), radius=3, fill="#281b38")
    draw.rounded_rectangle(
        (bar_left, bar_y, bar_left + (bar_right - bar_left) * progress, bar_y + 5),
        radius=3,
        fill=base_color,
    )


def render_gif(data: TraceData, names: list[str], title: str, output_path: Path) -> None:
    columns, rows, width, height = panel_layout(len(names))
    margin = 16
    header = 58
    panel_width = (width - margin * (columns + 1)) // columns
    panel_height = (height - header - margin * (rows + 1)) // rows
    comparison_max = max(data.runs[name].measured_seconds for name in names)

    frames: list[Image.Image] = []
    for frame_index in range(FRAME_COUNT):
        animation_seconds = TOTAL_SECONDS * frame_index / (FRAME_COUNT - 1)
        image = Image.new("RGBA", (width, height), COLORS["background"])
        draw = ImageDraw.Draw(image)
        draw.text((margin, 13), title, fill=COLORS["text"], font=font(25, True))
        draw.text(
            (width - margin, 20),
            f"real runtime ratio · slowest mapped to {TOTAL_SECONDS}s · {FPS} FPS",
            fill=COLORS["muted"],
            font=font(12),
            anchor="ra",
        )

        for index, name in enumerate(names):
            column = index % columns
            row = index // columns
            left = margin + column * (panel_width + margin)
            top = header + margin + row * (panel_height + margin)
            render_panel(
                image,
                data,
                data.runs[name],
                (left, top, left + panel_width, top + panel_height),
                animation_seconds,
                comparison_max,
            )
        palette_frame = image.convert("RGB").quantize(
            colors=128,
            method=Image.Quantize.FASTOCTREE,
            dither=Image.Dither.NONE,
        )
        frames.append(palette_frame)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(
        output_path,
        save_all=True,
        append_images=frames[1:],
        duration=round(1000 / FPS),
        loop=0,
        optimize=False,
        disposal=2,
    )
    print(f"saved {output_path.relative_to(ROOT)}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument("--reuse-traces", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.skip_build:
        build_trace_executable()

    traces: dict[str, TraceData] = {}
    for mode in ("path", "full"):
        path = ROOT / "output" / "profiling" / f"visual_trace_{mode}.txt"
        if not args.reuse_traces or not path.exists():
            path = capture_trace(mode)
        traces[mode] = parse_trace(path)

    groups = {
        "sequential": ["dijkstra-sequential", "bmssp-sequential"],
        "parallel": ["dijkstra-parallel", "bmssp-parallel"],
        "all": [
            "dijkstra-sequential",
            "bmssp-sequential",
            "dijkstra-parallel",
            "bmssp-parallel",
        ],
    }
    for mode, label in (("path", "Point-to-point"), ("full", "Full SSSP")):
        for group, names in groups.items():
            render_gif(
                traces[mode],
                names,
                f"Dijkstra vs BMSSP · Neon city · {label}",
                OUTPUT_DIR / mode / f"{mode}_{group}.gif",
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
