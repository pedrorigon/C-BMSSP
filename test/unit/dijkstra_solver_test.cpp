#include "dijkstra_solver.h"
#include "graph.h"
#include "test_assert.h"

#include <stdexcept>

namespace {

void run_tests() {
  sssp::Graph graph(4);
  graph.add_edge(0, 1, 1.0);
  graph.add_edge(1, 2, 2.0);
  graph.add_edge(0, 2, 10.0);

  validation::DijkstraSolver solver(graph);
  test::require_near(3.0, solver.solve(0, 2).distance, 1e-9, "Dijkstra distance mismatch");
  test::require(!solver.solve(0, 3).reachable(), "Dijkstra unreachable query failed");

  const auto distances = solver.solve_all(0);
  test::require_near(0.0, distances[0], 1e-9, "source distance mismatch");
  test::require_near(3.0, distances[2], 1e-9, "all-distances mismatch");
  test::require_throws<std::out_of_range>([&solver] { static_cast<void>(solver.solve_all(4)); },
                                          "Dijkstra accepted invalid source");
}

} // namespace

int main() { return test::run(run_tests); }
