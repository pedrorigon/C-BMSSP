#include "graph_factory.h"
#include "parallel_solver.h"
#include "sequential_solver.h"
#include "solver_comparison.h"
#include "test_assert.h"

#include <array>
#include <utility>

namespace {

void run_tests() {
  const sssp::Graph graph = test::make_ring_lattice(50'000, 4);
  test::require(graph.edge_count() == 200'000, "large graph does not cross optimized threshold");

  sssp::SequentialSolver sequential(graph);
  sssp::ParallelSolver parallel(graph);
  constexpr std::array<std::pair<sssp::Vertex, sssp::Vertex>, 5> queries{{
      {0, 2'000},
      {12'345, 18'000},
      {49'990, 100},
      {25'000, 25'000},
      {40'000, 10'000},
  }};

  for (const auto& [source, goal] : queries) {
    test::compare_dijkstra_implementations(graph, source, goal, "large Dijkstra");
    test::compare_with_dijkstra(graph, sequential, source, goal, "large sequential");
    test::compare_with_dijkstra(graph, parallel, source, goal, "large parallel");
  }
}

} // namespace

int main() { return test::run(run_tests); }
