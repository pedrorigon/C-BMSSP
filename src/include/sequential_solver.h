#pragma once

#include "graph.h"
#include "path_result.h"

#include <cstddef>
#include <optional>
#include <vector>

namespace sssp {

namespace detail {
class BlockQueue;
}

struct SolverOptions {
  std::size_t minimum_optimized_vertices{50'000};
  std::size_t minimum_optimized_edges{200'000};
};

class SequentialSolver {
public:
  explicit SequentialSolver(const Graph& graph, SolverOptions options = {});
  virtual ~SequentialSolver() = default;

  [[nodiscard]] PathResult solve(Vertex source, Vertex goal);

  // Full single-source shortest paths (source to every vertex, no early exit).
  // Returns the distance to each vertex; used by the scalability benchmark.
  [[nodiscard]] std::vector<double> solve_all(Vertex source);

protected:
  const Graph& graph_;
  std::vector<double> distances_;
  std::vector<Vertex> predecessors_;
  std::vector<bool> complete_;
  std::size_t k_;
  std::size_t t_;
  SolverOptions options_;

  void reset();
  void validate_query(Vertex source, Vertex goal) const;
  [[nodiscard]] PathResult solve_small_graph(Vertex source, Vertex goal);
  void complete_shortest_paths(std::optional<Vertex> goal);
  [[nodiscard]] PathResult solve_optimized(Vertex source, Vertex goal);
  [[nodiscard]] std::pair<double, std::vector<Vertex>> bounded_search(std::size_t level,
                                                                      double bound,
                                                                      std::vector<Vertex> pivots,
                                                                      std::optional<Vertex> goal);
  [[nodiscard]] std::pair<double, std::vector<Vertex>>
  base_case(double bound, const std::vector<Vertex>& frontier, std::optional<Vertex> goal);
  [[nodiscard]] virtual std::pair<std::vector<Vertex>, std::vector<Vertex>>
  find_pivots(double bound, const std::vector<Vertex>& frontier);
  [[nodiscard]] std::vector<Vertex> reconstruct_path(Vertex source, Vertex goal) const;
};

} // namespace sssp
