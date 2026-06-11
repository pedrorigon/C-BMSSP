# C-BMSSP: Bounded Multi-Source Shortest Path

A C++ implementation inspired by *["Breaking the Sorting Barrier for Directed Single-Source Shortest Paths"](https://arxiv.org/abs/2504.17033)* by Ran Duan, Jiayi Mao, Xiao Mao, Xinkai Shu, and
Longhui Yin (STOC 2025). 

The algorithm breaks the `O(m + n log n)` sorting barrier for directed single-source shortest paths (SSSP). It reduces the logarithmic complexity factor to `O(log^(2/3) n)`, yielding an overall time complexity of `O(m log^(2/3) n)` for directed graphs with real,
non-negative edge weights in the comparison-addition model. This improvement becomes increasingly relevant for large sparse graphs.

## Algorithm Properties

- **Time Complexity**: `O(m log^(2/3) n)`
- **Space Complexity**: `O(n + m)`
- **Recommended use**: Best For Large sparse directed graphs where breaking the sorting barrier matters
- **Advantages**: First deterministic algorithm to break Dijkstra's
  `O(m + n log n)` complexity barrier, even for undirected graphs

## Requirements

- A C++20 compiler and [CMake](https://cmake.org/) (≥ 3.20) with a build tool
  such as Ninja or Make. OpenMP is optional (enables the parallel solvers).
- [uv](https://docs.astral.sh/uv/) for the Python benchmark scripts. It manages
  Python 3.12 and the dependencies automatically — no manual `pip` needed.

## Quick Start

After cloning, build the C++ targets and set up the Python environment:

```bash
git clone <repository-url>
cd C-BMSSP

# Build the optimized C++ executables (solvers, tests, benchmark CLIs)
cmake --preset release
cmake --build --preset release

# Create the Python environment from pyproject.toml / uv.lock
uv sync
```

You are then ready to run the [benchmarks](#dataset-benchmarks) and
[scalability study](#scalability-benchmark) with `uv run python ...`.

## Project Structure

```text
src/
  include/                    Public API and internal headers
  core/                       Core C++ implementation
validation/dijkstra/
  src/include/                Validation API
  src/core/                   Sequential and OpenMP Dijkstra implementations
test/
  unit/                       Module-level unit tests
  comparison/                 Cross-implementation comparisons and CLI
  support/                    Test graph generators and helpers
  data/                       Test datasets
```

Validation includes sequential and OpenMP Dijkstra solvers. The parallel solver
processes equal-distance minimum batches and relaxes edges in parallel, then
applies updates deterministically. Batches containing zero-weight edges preserve
the sequential heap order so both Dijkstra solvers return the same distance and
path. When OpenMP is unavailable, the parallel solver uses the same deterministic
algorithm without parallel regions.

## Build

Build and run the instrumented test suite:

```bash
cmake --preset tests
cmake --build --preset tests
ctest --preset tests
```

Other available presets:

- `release`: Optimized build with the full test suite
- `debug`: Debug build without coverage instrumentation
- `sanitizers`: Debug build with AddressSanitizer and UndefinedBehaviorSanitizer
- `library-release`: Optimized library-only build without tests or validation code

Build only the main library:

```bash
cmake --preset library-release
cmake --build --preset library-release
```

## Manual Comparison

The comparison executable accepts 1-based DIMACS vertex IDs:

```bash
cmake --preset release
cmake --build --preset release
./build/release/test/sssp_compare test/data/Rome99 1 3353
```

It runs sequential and parallel Dijkstra alongside sequential and parallel SSSP,
then fails if their distances differ.

## Python Environment

The benchmark scripts use [uv](https://docs.astral.sh/uv/) to manage Python
(3.12) and the `matplotlib`/`scipy` dependencies, pinned in `pyproject.toml` and
`uv.lock`. Install them once with:

```bash
uv sync
```

Then run any script with `uv run`, which always uses this locked environment:

```bash
uv run python run_benchmarks.py
```

(`uv run` also creates the environment automatically on first use, so `uv sync`
is optional but makes the install step explicit.)

## Dataset Benchmarks

`run_benchmarks.py` benchmarks the solvers on the **complete** Stanford, Google,
Pokec, and LiveJournal SNAP graphs (downloaded on demand, reused if present) and
writes a CSV under `output/`. Each run is a single **point-to-point query**
(source to goal) on the **unweighted** graph, matching the
[bmssp-python](https://github.com/bzantium/bmssp-python) reference; the C++
timings are expected to be much lower than that pure-Python baseline. The size is
fixed per dataset — see [Scalability Benchmark](#scalability-benchmark) for a
scaling study.

Each benchmark runs `--iterations` times (default 10), aggregated with 1.5×IQR
outlier removal and a 95% t-Student confidence interval.

```bash
uv run python run_benchmarks.py                       # all datasets, models, types
uv run python run_benchmarks.py --dataset google      # filter by dataset, model, or type
uv run python run_benchmarks.py --iterations 20
```

Pass `--skip-build` to reuse an existing executable.

## Scalability Benchmark

`run_scaling.py` measures how the solvers scale by running **full single-source
shortest paths** on generated **sparse weighted** graphs of increasing size. This
targets the regime where the BMSSP advantage (`O(m log^(2/3) n)`) over Dijkstra
(`O(m + n log n)`) can appear: large, very sparse (`m ≈ n`) graphs with real
weights and a large diameter. The default `banded` topology builds such graphs
(each vertex links forward within `--bandwidth`); `--topology random` produces a
small-world graph instead. Real weights are required — unit weights reduce SSSP
to a BFS, where the algorithms tie. Graphs are generated deterministically (fixed
`--seed`), so every iteration measures the same problem. Results go to a CSV under
`output/` plus linear and log-Y plots (PDF + JPG) under `plots/scaling/`.

```bash
uv run python run_scaling.py                                   # banded, up to 10M vertices
uv run python run_scaling.py --max-vertices 1000000 --steps 8  # quicker run
uv run python run_scaling.py --topology random --avg-degree 4  # small-world variant
uv run python run_scaling.py --bandwidth 16 --iterations 20    # deeper graph, more samples
```

## References

  - **Primary Paper**: ["Breaking the Sorting Barrier for Directed Single-Source Shortest Paths"](https://arxiv.org/abs/2504.17033) by Ran Duan, Jiayi Mao, Xiao Mao, Xinkai Shu, Longhui Yin (2025).
  - **Rust Implementation**: [alphastrata/DunMaoSSSP](https://github.com/alphastrata/DunMaoSSSP.git)
  - **Python Implementation**: [bzantium/bmssp-python](https://github.com/bzantium/bmssp-python)
