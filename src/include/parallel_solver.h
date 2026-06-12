#pragma once

#include "sequential_solver.h"

#include <cstddef>
#include <utility>
#include <vector>

namespace sssp {

namespace detail {
class BlockQueue;
}

class ParallelSolver final : public SequentialSolver {
public:
  explicit ParallelSolver(const Graph& graph, SolverOptions options = {});

private:
  [[nodiscard]] std::pair<std::vector<Vertex>, std::vector<Vertex>>
  find_pivots(double bound, const std::vector<Vertex>& frontier) override;

  [[nodiscard]] std::pair<double, std::vector<Vertex>>
  bounded_search(std::size_t level, double bound, std::vector<Vertex> pivots,
                 std::optional<Vertex> goal) override;

  void complete_shortest_paths(std::optional<Vertex> goal) override;

  void relax_edges_parallel(const std::vector<Vertex>& completed, double sub_bound,
                            double subset_bound, double bound,
                            detail::BlockQueue& data_structure,
                            std::vector<std::pair<Vertex, double>>& prepend);

  static constexpr std::size_t kParallelRelaxThreshold = 256;
};

} // namespace sssp
