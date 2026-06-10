#include "parallel_solver.h"
#include "test_assert.h"

namespace {

void run_tests() {
  sssp::Graph graph(8);
  for (sssp::Vertex vertex = 0; vertex < 7; ++vertex) {
    graph.add_edge(vertex, vertex + 1, 1.0);
  }
  graph.add_edge(0, 7, 20.0);
  graph.add_edge(2, 7, 2.0);

  const sssp::SolverOptions force_optimized{0, 0};
  sssp::ParallelSolver solver(graph, force_optimized);
  test::require_near(4.0, solver.solve(0, 7).distance, 1e-9, "parallel shortest path failed");
  test::require(!solver.solve(7, 0).reachable(), "parallel unreachable query failed");
  test::require_near(2.0, solver.solve(1, 3).distance, 1e-9, "parallel state reset failed");
}

} // namespace

int main() { return test::run(run_tests); }
