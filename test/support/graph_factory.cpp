#include "graph_factory.h"

#include <random>

namespace test {

sssp::Graph make_path_graph(const std::size_t vertex_count, const double weight) {
  sssp::Graph graph(vertex_count);
  for (sssp::Vertex vertex = 1; vertex < vertex_count; ++vertex) {
    graph.add_edge(vertex - 1, vertex, weight);
  }
  return graph;
}

sssp::Graph make_ring_lattice(const std::size_t vertex_count, const std::size_t degree) {
  sssp::Graph graph(vertex_count);
  for (sssp::Vertex vertex = 0; vertex < vertex_count; ++vertex) {
    for (std::size_t offset = 1; offset <= degree; ++offset) {
      graph.add_edge(vertex, (vertex + offset) % vertex_count, static_cast<double>(offset));
    }
  }
  return graph;
}

sssp::Graph make_random_graph(const std::size_t vertex_count, const std::size_t extra_edges,
                              const std::uint64_t seed) {
  sssp::Graph graph = make_path_graph(vertex_count);
  if (vertex_count == 0) {
    return graph;
  }

  std::mt19937_64 generator(seed);
  std::uniform_int_distribution<std::size_t> vertex_distribution(0, vertex_count - 1);
  std::uniform_real_distribution<double> weight_distribution(0.01, 100.0);
  for (std::size_t edge = 0; edge < extra_edges; ++edge) {
    graph.add_edge(vertex_distribution(generator), vertex_distribution(generator),
                   weight_distribution(generator));
  }
  return graph;
}

sssp::Graph make_banded_graph(const std::size_t vertex_count, const std::size_t degree,
                              const std::size_t bandwidth, const std::uint64_t seed) {
  sssp::Graph graph(vertex_count);
  if (vertex_count < 2) {
    return graph;
  }
  const std::size_t span = std::max<std::size_t>(bandwidth, 1);

  std::mt19937_64 generator(seed);
  std::uniform_int_distribution<std::size_t> offset_distribution(1, span);
  std::uniform_real_distribution<double> weight_distribution(0.01, 100.0);
  for (sssp::Vertex vertex = 0; vertex < vertex_count; ++vertex) {
    // Path backbone keeps the graph connected and the diameter large.
    if (vertex + 1 < vertex_count) {
      graph.add_edge(vertex, vertex + 1, weight_distribution(generator));
    }
    for (std::size_t edge = 1; edge < degree; ++edge) {
      const std::size_t target = vertex + offset_distribution(generator);
      if (target < vertex_count) {
        graph.add_edge(vertex, target, weight_distribution(generator));
      }
    }
  }
  return graph;
}

} // namespace test
