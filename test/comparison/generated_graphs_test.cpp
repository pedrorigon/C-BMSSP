#include "dijkstra_solver.h"
#include "graph_factory.h"
#include "parallel_solver.h"
#include "sequential_solver.h"
#include "solver_comparison.h"
#include "test_assert.h"

#include <array>
#include <cmath>
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
    const std::string context =
        "vertices=" + std::to_string(vertex_count) + " seed=" + std::to_string(seed);
    for (const sssp::Vertex goal : goals) {
      test::compare_dijkstra_implementations(graph, source, goal, context + " Dijkstra");
      test::compare_with_dijkstra(graph, sequential, source, goal, context + " sequential");
      test::compare_with_dijkstra(graph, parallel, source, goal, context + " parallel");
    }

    validation::DijkstraSolver reference(graph);
    const auto expected = reference.solve_all(source);
    const auto sequential_distances = sequential.solve_all(source);
    const auto parallel_distances = parallel.solve_all(source);
    for (sssp::Vertex vertex = 0; vertex < vertex_count; ++vertex) {
      const bool expected_reachable = expected[vertex] != sssp::infinity;
      test::require((sequential_distances[vertex] != sssp::infinity) == expected_reachable,
                    context + " sequential full reachability");
      test::require((parallel_distances[vertex] != sssp::infinity) == expected_reachable,
                    context + " parallel full reachability");
      if (expected_reachable) {
        test::require_near(expected[vertex], sequential_distances[vertex], 1e-9,
                           context + " sequential full distance");
        test::require_near(expected[vertex], parallel_distances[vertex], 1e-9,
                           context + " parallel full distance");
      }
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
  for (std::uint64_t seed = 10; seed < 22; ++seed) {
    compare_generated_graph(96, 480, seed);
    compare_generated_graph(384, 2'304, seed + 100);
  }
}

} // namespace

int main() { return test::run(run_tests); }
