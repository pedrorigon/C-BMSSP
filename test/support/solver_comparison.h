#pragma once

#include "dijkstra_solver.h"
#include "graph.h"
#include "parallel_dijkstra_solver.h"
#include "path_result.h"
#include "test_assert.h"

#include <algorithm>
#include <cmath>
#include <string_view>

namespace test {

inline void require_valid_path(const sssp::Graph& graph, const sssp::Vertex source,
                               const sssp::Vertex goal, const sssp::PathResult& result) {
  require(result.reachable(), "expected a reachable path");
  require(!result.path.empty(), "reachable result has an empty path");
  require(result.path.front() == source, "path does not begin at source");
  require(result.path.back() == goal, "path does not end at goal");

  double path_cost = 0.0;
  for (std::size_t index = 1; index < result.path.size(); ++index) {
    const auto& edges = graph.edges_from(result.path[index - 1]);
    double edge_weight = sssp::infinity;
    for (const auto& edge : edges) {
      if (edge.to == result.path[index]) {
        edge_weight = std::min(edge_weight, edge.weight);
      }
    }
    require(edge_weight != sssp::infinity, "path contains a missing edge");
    path_cost += edge_weight;
  }
  require_near(result.distance, path_cost, 1e-9, "path cost differs from reported distance");
}

template <typename Solver>
void compare_with_dijkstra(const sssp::Graph& graph, Solver& solver, const sssp::Vertex source,
                           const sssp::Vertex goal, const std::string_view context) {
  validation::DijkstraSolver reference(graph);
  const sssp::PathResult expected = reference.solve(source, goal);
  const sssp::PathResult actual = solver.solve(source, goal);

  require(expected.reachable() == actual.reachable(), std::string(context) + ": reachability");
  if (actual.reachable()) {
    require_near(expected.distance, actual.distance, 1e-9, std::string(context) + ": distance");
    require_valid_path(graph, source, goal, actual);
  }
}

inline void compare_dijkstra_implementations(const sssp::Graph& graph, const sssp::Vertex source,
                                             const sssp::Vertex goal,
                                             const std::string_view context) {
  validation::DijkstraSolver sequential(graph);
  validation::ParallelDijkstraSolver parallel(graph);
  const sssp::PathResult expected = sequential.solve(source, goal);
  const sssp::PathResult actual = parallel.solve(source, goal);

  require(expected.reachable() == actual.reachable(), std::string(context) + ": reachability");
  if (actual.reachable()) {
    require_near(expected.distance, actual.distance, 1e-9, std::string(context) + ": distance");
    // The parallel Dijkstra (Delta-stepping) guarantees identical shortest-path
    // distances but may pick a different predecessor when several edges tie for
    // the minimum. We therefore require a valid shortest path of equal cost, not
    // a byte-identical predecessor chain.
    require_valid_path(graph, source, goal, actual);
  }
}

} // namespace test
