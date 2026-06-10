#pragma once

#include "graph.h"
#include "path_result.h"

#include <vector>

namespace validation {

class DijkstraSolver {
public:
  explicit DijkstraSolver(const sssp::Graph& graph);

  [[nodiscard]] sssp::PathResult solve(sssp::Vertex source, sssp::Vertex goal);
  [[nodiscard]] std::vector<double> solve_all(sssp::Vertex source);

private:
  const sssp::Graph& graph_;
  std::vector<double> distances_;
  std::vector<sssp::Vertex> predecessors_;

  void reset();
  void validate_vertex(sssp::Vertex vertex) const;
  [[nodiscard]] std::vector<sssp::Vertex> reconstruct_path(sssp::Vertex source,
                                                           sssp::Vertex goal) const;
};

} // namespace validation
