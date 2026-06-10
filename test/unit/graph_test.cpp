#include "graph.h"
#include "test_assert.h"

#include <limits>
#include <stdexcept>

namespace {

void run_tests() {
  sssp::Graph graph(3);
  graph.add_edge(0, 1, 1.5);
  graph.add_edge(1, 2, 2.5);
  test::require(graph.vertex_count() == 3, "vertex count mismatch");
  test::require(graph.edge_count() == 2, "edge count mismatch");
  test::require(graph.edges_from(0).front().to == 1, "edge target mismatch");

  test::require_throws<std::out_of_range>([&graph] { graph.add_edge(3, 0, 1.0); },
                                          "out-of-range source accepted");
  test::require_throws<std::invalid_argument>([&graph] { graph.add_edge(0, 1, -1.0); },
                                              "negative weight accepted");
  test::require_throws<std::invalid_argument>(
      [&graph] { graph.add_edge(0, 1, std::numeric_limits<double>::infinity()); },
      "infinite weight accepted");
  test::require_throws<std::invalid_argument>(
      [&graph] { graph.add_edge(0, 1, std::numeric_limits<double>::quiet_NaN()); },
      "NaN weight accepted");

  graph.add_edge(0, 0, 0.0);
  graph.add_edge(0, 1, 0.0);
  test::require(graph.edge_count() == 4, "zero-weight or duplicate edge was rejected");
  test::require_throws<std::out_of_range>([&graph] { static_cast<void>(graph.edges_from(3)); },
                                          "out-of-range adjacency access accepted");
}

} // namespace

int main() { return test::run(run_tests); }
