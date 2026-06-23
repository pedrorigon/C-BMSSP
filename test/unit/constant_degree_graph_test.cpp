#include "constant_degree_graph.h"
#include "test_assert.h"

#include <cstddef>
#include <vector>

namespace {

void run_tests() {
  sssp::Graph graph(4);
  graph.add_edge(0, 1, 2.0);
  graph.add_edge(0, 2, 5.0);
  graph.add_edge(1, 2, 1.0);
  graph.add_edge(2, 3, 3.0);
  graph.add_edge(3, 0, 4.0);

  const auto transformed = sssp::detail::make_constant_degree_graph(graph);
  test::require(transformed.graph.vertex_count() == graph.vertex_count() + 2 * graph.edge_count(),
                "degree reduction produced the wrong vertex count");
  test::require(transformed.original_vertex.size() == transformed.graph.vertex_count(),
                "degree reduction mapping has the wrong size");

  std::vector<std::size_t> indegree(transformed.graph.vertex_count(), 0);
  for (sssp::Vertex from = 0; from < transformed.graph.vertex_count(); ++from) {
    test::require(transformed.graph.edges_from(from).size() <= 2,
                  "transformed out-degree exceeds two");
    for (const auto& edge : transformed.graph.edges_from(from)) {
      ++indegree[edge.to];
    }
  }
  for (const std::size_t degree : indegree) {
    test::require(degree <= 2, "transformed in-degree exceeds two");
  }
}

} // namespace

int main() { return test::run(run_tests); }
