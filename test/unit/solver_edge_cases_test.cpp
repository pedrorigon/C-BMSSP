#include "parallel_solver.h"
#include "sequential_solver.h"
#include "solver_comparison.h"
#include "test_assert.h"

#include <limits>
#include <stdexcept>

namespace {

template <typename Solver> void test_solver_edge_cases() {
  const sssp::SolverOptions force_optimized{0, 0};

  const sssp::Graph empty_graph;
  Solver empty_solver(empty_graph, force_optimized);
  test::require_throws<std::out_of_range>(
      [&empty_solver] { static_cast<void>(empty_solver.solve(0, 0)); },
      "empty graph accepted a query");

  const sssp::Graph single_vertex_graph(1);
  Solver single_vertex_solver(single_vertex_graph, force_optimized);
  const auto same_vertex = single_vertex_solver.solve(0, 0);
  test::require_near(0.0, same_vertex.distance, 1e-9, "single vertex distance mismatch");
  test::require_valid_path(single_vertex_graph, 0, 0, same_vertex);

  sssp::Graph graph(5);
  graph.add_edge(0, 1, 5.0);
  graph.add_edge(0, 1, 1.0);
  graph.add_edge(1, 2, 0.0);
  graph.add_edge(2, 1, 0.0);
  graph.add_edge(2, 3, 2.0);
  graph.add_edge(0, 3, 20.0);
  graph.add_edge(3, 4, std::numeric_limits<double>::max() / 4.0);

  Solver solver(graph, force_optimized);
  test::compare_with_dijkstra(graph, solver, 0, 3, "zero cycle and duplicate edges");
  test::compare_with_dijkstra(graph, solver, 0, 4, "large finite weight");
  test::compare_with_dijkstra(graph, solver, 4, 0, "unreachable reverse query");
}

void run_tests() {
  test_solver_edge_cases<sssp::SequentialSolver>();
  test_solver_edge_cases<sssp::ParallelSolver>();
}

} // namespace

int main() { return test::run(run_tests); }
