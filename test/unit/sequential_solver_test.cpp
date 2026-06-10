#include "sequential_solver.h"
#include "test_assert.h"

#include <stdexcept>
#include <vector>

namespace {

sssp::Graph make_graph() {
  sssp::Graph graph(6);
  graph.add_edge(0, 1, 1.0);
  graph.add_edge(0, 2, 4.0);
  graph.add_edge(1, 2, 2.0);
  graph.add_edge(1, 3, 5.0);
  graph.add_edge(2, 3, 1.0);
  graph.add_edge(3, 4, 3.0);
  return graph;
}

void run_tests() {
  const sssp::Graph graph = make_graph();
  sssp::SequentialSolver solver(graph);

  const auto result = solver.solve(0, 4);
  test::require_near(7.0, result.distance, 1e-9, "incorrect shortest distance");
  test::require(result.path == std::vector<sssp::Vertex>({0, 1, 2, 3, 4}), "incorrect path");
  test::require(!solver.solve(0, 5).reachable(), "unreachable vertex reported as reachable");
  test::require(solver.solve(2, 2).path == std::vector<sssp::Vertex>({2}), "source=goal failed");
  test::require_near(3.0, solver.solve(1, 3).distance, 1e-9, "solver state was not reset");
  test::require_throws<std::out_of_range>([&solver] { static_cast<void>(solver.solve(0, 6)); },
                                          "out-of-range query accepted");

  const sssp::SolverOptions force_optimized{0, 0};
  sssp::SequentialSolver forced_solver(graph, force_optimized);
  test::require_near(7.0, forced_solver.solve(0, 4).distance, 1e-9, "forced optimized path failed");
}

} // namespace

int main() { return test::run(run_tests); }
