#include "dijkstra_solver.h"
#include "graph_factory.h"
#include "parallel_dijkstra_solver.h"
#include "solver_comparison.h"
#include "test_assert.h"

#include <stdexcept>
#include <vector>

namespace {

void require_same_distances(const std::vector<double>& expected,
                            const std::vector<double>& actual) {
  test::require(expected.size() == actual.size(), "distance vector size mismatch");
  for (std::size_t vertex = 0; vertex < expected.size(); ++vertex) {
    if (expected[vertex] == sssp::infinity || actual[vertex] == sssp::infinity) {
      test::require(expected[vertex] == actual[vertex], "distance reachability mismatch");
    } else {
      test::require_near(expected[vertex], actual[vertex], 1e-9, "distance mismatch");
    }
  }
}

void test_basic_queries() {
  sssp::Graph graph(6);
  graph.add_edge(0, 1, 1.0);
  graph.add_edge(0, 2, 1.0);
  graph.add_edge(1, 3, 2.0);
  graph.add_edge(2, 3, 2.0);
  graph.add_edge(3, 4, 0.0);

  validation::DijkstraSolver sequential(graph);
  validation::ParallelDijkstraSolver parallel(graph);
  test::compare_with_dijkstra(graph, parallel, 0, 4, "parallel Dijkstra basic query");
  test::compare_with_dijkstra(graph, parallel, 0, 5, "parallel Dijkstra unreachable query");
  test::compare_with_dijkstra(graph, parallel, 3, 3, "parallel Dijkstra source equals goal");
  require_same_distances(sequential.solve_all(0), parallel.solve_all(0));
  test::require_throws<std::out_of_range>([&parallel] { static_cast<void>(parallel.solve(0, 6)); },
                                          "parallel Dijkstra accepted an invalid goal");
  test::require_throws<std::out_of_range>([&parallel] { static_cast<void>(parallel.solve_all(6)); },
                                          "parallel Dijkstra accepted an invalid source");
}

void test_high_degree_relaxation() {
  constexpr std::size_t vertex_count = 1'024;
  sssp::Graph graph(vertex_count);
  for (sssp::Vertex vertex = 1; vertex < vertex_count; ++vertex) {
    graph.add_edge(0, vertex, static_cast<double>(vertex % 17));
  }

  validation::DijkstraSolver sequential(graph);
  validation::ParallelDijkstraSolver parallel(graph);
  require_same_distances(sequential.solve_all(0), parallel.solve_all(0));
}

void test_wide_equal_distance_batch() {
  constexpr std::size_t middle_count = 256;
  const sssp::Vertex goal = middle_count + 1;
  sssp::Graph graph(goal + 1);
  for (sssp::Vertex vertex = 1; vertex <= middle_count; ++vertex) {
    graph.add_edge(0, vertex, 1.0);
    graph.add_edge(vertex, goal, static_cast<double>((vertex % 13) + 1));
  }

  validation::DijkstraSolver sequential(graph);
  validation::ParallelDijkstraSolver parallel(graph);
  require_same_distances(sequential.solve_all(0), parallel.solve_all(0));
  test::compare_with_dijkstra(graph, parallel, 0, goal, "parallel Dijkstra wide batch");
}

void test_zero_weight_tie_breaking() {
  sssp::Graph graph(6);
  graph.add_edge(0, 2, 1.0);
  graph.add_edge(0, 4, 1.0);
  graph.add_edge(2, 1, 0.0);
  graph.add_edge(1, 5, 1.0);
  graph.add_edge(4, 5, 1.0);

  test::compare_dijkstra_implementations(graph, 0, 5, "parallel Dijkstra zero-weight tie");
}

void test_generated_graphs() {
  for (std::uint64_t seed = 1; seed <= 8; ++seed) {
    const sssp::Graph graph = test::make_random_graph(500, 4'000, seed);
    validation::DijkstraSolver sequential(graph);
    validation::ParallelDijkstraSolver parallel(graph);

    for (const sssp::Vertex source : {sssp::Vertex{0}, sssp::Vertex{127}, sssp::Vertex{499}}) {
      require_same_distances(sequential.solve_all(source), parallel.solve_all(source));
      for (const sssp::Vertex goal :
           {sssp::Vertex{0}, sssp::Vertex{89}, sssp::Vertex{311}, sssp::Vertex{499}}) {
        test::compare_with_dijkstra(graph, parallel, source, goal,
                                    "parallel Dijkstra generated graph");
      }
    }
  }

  // The large-diameter banded generator must yield the same distances too.
  for (std::uint64_t seed = 1; seed <= 4; ++seed) {
    const sssp::Graph graph = test::make_banded_graph(800, 3, 16, seed);
    validation::DijkstraSolver sequential(graph);
    validation::ParallelDijkstraSolver parallel(graph);
    for (const sssp::Vertex source : {sssp::Vertex{0}, sssp::Vertex{400}}) {
      require_same_distances(sequential.solve_all(source), parallel.solve_all(source));
      test::compare_with_dijkstra(graph, parallel, source, sssp::Vertex{799},
                                  "parallel Dijkstra banded graph");
    }
  }
}

void run_tests() {
  test_basic_queries();
  test_high_degree_relaxation();
  test_wide_equal_distance_batch();
  test_zero_weight_tie_breaking();
  test_generated_graphs();
}

} // namespace

int main() { return test::run(run_tests); }
