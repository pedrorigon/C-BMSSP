#pragma once

#include "sequential_solver.h"

namespace sssp {

class ParallelSolver final : public SequentialSolver {
public:
  explicit ParallelSolver(const Graph& graph, SolverOptions options = {});

private:
  [[nodiscard]] std::pair<std::vector<Vertex>, std::vector<Vertex>>
  find_pivots(double bound, const std::vector<Vertex>& frontier) override;
};

} // namespace sssp
