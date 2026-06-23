#pragma once

#include <cmath>
#include <cstdint>

namespace sssp {

// Weighted-work accounting for the empirical complexity study.
//
// Wall-clock time cannot demonstrate the BMSSP asymptotic advantage: the
// constant-degree transformation and the much larger constant factor of the
// recursion dominate every input that fits in memory. The complexity claim is
// about the *number of ordered-structure operations*, each weighted by the cost
// the analysis assigns to it:
//
//   * Dijkstra pays O(log n) per priority-queue push/pop.
//   * BMSSP pays the partial-order cost from Lemma 3.3 (block-tree depth for an
//     insert/erase, the batch-prepend partition cost, the pull scan).
//
// Summed over a full single-source run and divided by the edge count, this
// "work per edge" grows like log n for Dijkstra and like log^(2/3) n for BMSSP.
// Plotting it against n exposes the smaller exponent directly, independent of
// the implementation's constant factors.
//
// The counter is a single process-global accumulator. The solvers are run one
// at a time in the scaling harness, so a global avoids threading a context
// object through every method. It is reset before each timed run.
class WorkCounter {
public:
  static void reset() noexcept { total_ = 0.0; }
  static double total() noexcept { return total_; }

  // Adds the cost of one operation acting on a structure currently holding
  // `size` elements: log2(size) charges, with a floor of 1 so that operations
  // on tiny structures still count as real work.
  static void add_log(const std::size_t size) noexcept {
    total_ += size > 1 ? std::log2(static_cast<double>(size)) : 1.0;
  }

  // Adds a raw, already-computed amount of work (e.g. a linear scan).
  static void add(const double amount) noexcept { total_ += amount; }

private:
  inline static double total_ = 0.0;
};

} // namespace sssp
