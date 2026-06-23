# C-BMSSP: Bounded Multi-Source Shortest Path

A C++ implementation of the deterministic BMSSP algorithm from
*["Breaking the Sorting Barrier for Directed Single-Source Shortest Paths"](https://arxiv.org/abs/2504.17033)*
by Ran Duan, Jiayi Mao, Xiao Mao, Xinkai Shu, and Longhui Yin (STOC 2025).

The algorithm breaks the `O(m + n log n)` sorting barrier for directed single-source shortest paths (SSSP). It reduces the logarithmic complexity factor to `O(log^(2/3) n)`, yielding an overall time complexity of `O(m log^(2/3) n)` for directed graphs with real,
non-negative edge weights in the comparison-addition model. This improvement becomes increasingly relevant for large sparse graphs.

## Algorithm Properties

- **Time Complexity**: `O(m log^(2/3) n)`
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
  core/                       BMSSP, partial-order queue, and graph transform
validation/dijkstra/
  src/include/                Validation API
  src/core/                   Sequential and OpenMP Dijkstra implementations
test/
  unit/                       Module-level unit tests
  comparison/                 Cross-implementation comparisons and CLI
  support/                    Test graph generators and helpers
  data/                       Test datasets
```

The BMSSP implementation includes the paper's constant-degree transformation,
total path order, `FindPivots`, bounded base case, recursive Algorithm 3, and the
block structure from Lemma 3.3. The OpenMP variant shares the same recursion and
parallelizes scans of edges leaving completed batches.

Validation includes sequential heap-based Dijkstra and OpenMP Delta-stepping.
Updates are applied deterministically so all solvers return the same distance
and path. When OpenMP is unavailable, parallel targets remain correct without
parallel regions.

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
shortest paths** on generated **weighted** graphs of increasing size. Graphs are
generated deterministically (fixed `--seed`), so every iteration measures the
same problem, and real weights are required — unit weights reduce SSSP to a BFS
where everything ties. The default `random` topology (moderate density) gives the
parallel solvers enough work per distance level to scale; `--topology banded`
builds a large-diameter sparse graph instead. Results go to a CSV under `output/`
plus several plots (PDF + JPG) under `plots/scaling/`: the wall-clock scaling
curves (linear and log-Y) and, more importantly, the **empirical-complexity
plots** described below.

```bash
uv run python run_scaling.py                                   # random, degree 4, up to 100M vertices
uv run python run_scaling.py --max-vertices 1000000 --steps 8  # quicker run
uv run python run_scaling.py --avg-degree 16                   # denser graph
uv run python run_scaling.py --topology banded --avg-degree 2  # large-diameter sparse
uv run python run_scaling.py --memory-limit-gb 32              # raise the per-run RAM cap
```

The default is degree 4 (sparse graphs are the fairest regime for BMSSP) and a
100M-vertex ceiling. Dijkstra reaches that ceiling on the untransformed graph;
BMSSP is stopped by `--memory-limit-gb` (default 16 GiB) once the constant-degree
transform no longer fits — around 5M vertices on a 32 GiB machine — so the two
curves intentionally end at different sizes.

### Proving the complexity (work, not wall-clock)

Wall-clock cannot show the BMSSP advantage: the constant-degree blow-up and its
larger constant factor make it slower on every in-memory input. The complexity
claim is about the **number of ordered-structure operations**, so both solvers are
instrumented (`src/include/work_counter.h`): Dijkstra charges `log n` per heap op,
BMSSP charges the Lemma 3.3 costs. `run_scaling.py` then emits two extra figures:

- `..._complexity.{jpg,pdf}` — weighted work per edge (raw and normalized to the
  smallest graph). Dijkstra's normalized work grows like `log n` (~2.0x over
  1K→1M); BMSSP grows like `log^(2/3) n` (~1.4x). The smaller exponent is visible
  as the flatter curve.
- `..._crossover.{jpg,pdf}` — the BMSSP/Dijkstra work ratio, which falls with
  `ln(n)`, with a dashed extrapolation to the projected work break-even (~10^11
  vertices, far beyond memory — a projection, not a measurement).

`generate_complexity_gif.py` animates the normalized growth race as a 10-second
GIF under `visualizations/complexity/`.

### Interpreting the results

- The BMSSP target always runs the paper algorithm; there is no Dijkstra
  completion pass or graph-size fallback.
- BMSSP first transforms the input to constant degree. This preserves shortest
  paths and the asymptotic bound, but it can substantially increase the practical
  vertex and edge counts.
- Dijkstra usually has lower constants on current hardware because its hot loop
  is compact and cache-friendly. BMSSP trades that simplicity for a better
  asymptotic comparison bound, with recursion, pivot discovery, and partial-order
  bookkeeping.
- OpenMP helps only when completed batches are large enough to amortize worker
  synchronization and the deterministic serial commit.

For a reproducible local micro-profile and the six 10-second visual comparisons:

```bash
uv run python run_profiling.py
uv run python generate_visualizations.py
```

## References

  - **Primary Paper**: ["Breaking the Sorting Barrier for Directed Single-Source Shortest Paths"](https://arxiv.org/abs/2504.17033) by Ran Duan, Jiayi Mao, Xiao Mao, Xinkai Shu, Longhui Yin (2025).
  - **Rust Implementation**: [alphastrata/DunMaoSSSP](https://github.com/alphastrata/DunMaoSSSP.git)
  - **Python Implementation**: [bzantium/bmssp-python](https://github.com/bzantium/bmssp-python)
