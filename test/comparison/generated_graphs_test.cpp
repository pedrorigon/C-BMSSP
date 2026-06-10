#include "graph_factory.h"
#include "parallel_solver.h"
#include "sequential_solver.h"
#include "solver_comparison.h"
#include "test_assert.h"

#include <array>
#include <cstddef>
#include <string>

namespace {

void compare_generated_graph(const std::size_t vertex_count, const std::size_t extra_edges,
                             const std::uint64_t seed) {
  const sssp::Graph graph = test::make_random_graph(vertex_count, extra_edges, seed);
  const sssp::SolverOptions force_optimized{0, 0};
  sssp::SequentialSolver sequential(graph, force_optimized);
  sssp::ParallelSolver parallel(graph, force_optimized);

  const std::array<sssp::Vertex, 3> sources{0, vertex_count / 3, vertex_count - 1};
  const std::array<sssp::Vertex, 3> goals{vertex_count - 1, vertex_count / 2, 0};
  for (const sssp::Vertex source : sources) {
    for (const sssp::Vertex goal : goals) {
      const std::string context =
          "vertices=" + std::to_string(vertex_count) + " seed=" + std::to_string(seed);
      test::compare_dijkstra_implementations(graph, source, goal, context + " Dijkstra");
      test::compare_with_dijkstra(graph, sequential, source, goal, context + " sequential");
      test::compare_with_dijkstra(graph, parallel, source, goal, context + " parallel");
    }
  }
}

void run_tests() {
  compare_generated_graph(1, 0, 1);
  compare_generated_graph(2, 2, 2);
  compare_generated_graph(7, 15, 3);
  compare_generated_graph(64, 256, 4);
  compare_generated_graph(512, 2'048, 5);
  compare_generated_graph(2'000, 8'000, 6);
}

} // namespace

int main() { return test::run(run_tests); }
