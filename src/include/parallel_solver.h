#pragma once

#include "sequential_solver.h"

namespace sssp {

// Work-efficient OpenMP implementation. It shares the paper recursion and
// partial-order structure with SequentialSolver and parallelizes edge scans in
// completed BMSSP batches.
class ParallelSolver final : public SequentialSolver {
public:
  explicit ParallelSolver(const Graph& graph, SolverOptions options = {});

private:
  void relax_completed(const std::vector<Vertex>& completed, detail::QueueKey recursive_bound,
                       detail::QueueKey pull_bound, detail::QueueKey call_bound,
                       detail::BlockQueue& data_structure,
                       std::vector<detail::QueueItem>& prepend) override;

  static constexpr std::size_t kParallelRelaxThreshold = 512;
};

} // namespace sssp
