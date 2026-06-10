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

## References

  - **Primary Paper**: ["Breaking the Sorting Barrier for Directed Single-Source Shortest Paths"](https://arxiv.org/abs/2504.17033) by Ran Duan, Jiayi Mao, Xiao Mao, Xinkai Shu, Longhui Yin (2025).
  - **Rust Implementation**: [alphastrata/DunMaoSSSP](https://github.com/alphastrata/DunMaoSSSP.git)
  - **Python Implementation**: [bzantium/bmssp-python](https://github.com/bzantium/bmssp-python)
