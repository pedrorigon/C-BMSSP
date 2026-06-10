#include "dimacs_reader.h"
#include "graph.h"
#include "parallel_solver.h"
#include "sequential_solver.h"
#include "solver_comparison.h"
#include "test_assert.h"

#include <array>
#include <utility>

namespace {

void run_tests() {
  const sssp::Graph graph = sssp::read_dimacs_graph(SSSP_ROME99_PATH);
  test::require(graph.vertex_count() == 3'353, "Rome99 vertex count mismatch");
  test::require(graph.edge_count() == 8'870, "Rome99 edge count mismatch");

  sssp::SequentialSolver sequential(graph);
  sssp::ParallelSolver parallel(graph);
  constexpr std::array<std::pair<sssp::Vertex, sssp::Vertex>, 8> queries{{
      {0, 3'352},
      {10, 1'000},
      {100, 2'000},
      {2'000, 100},
      {42, 42},
      {3'000, 1},
      {1'500, 3'200},
      {3'352, 0},
  }};

  for (const auto& [source, goal] : queries) {
    test::compare_dijkstra_implementations(graph, source, goal, "Rome99 Dijkstra");
    test::compare_with_dijkstra(graph, sequential, source, goal, "Rome99 sequential");
    test::compare_with_dijkstra(graph, parallel, source, goal, "Rome99 parallel");
  }
}

} // namespace

int main() { return test::run(run_tests); }
