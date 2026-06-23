#include "constant_degree_graph.h"

#include <cstddef>
#include <utility>
#include <vector>

namespace sssp::detail {

namespace {

struct OriginalEdge {
  Vertex from;
  Vertex to;
  double weight;
  Vertex from_slot;
  Vertex to_slot;
};

} // namespace

ConstantDegreeGraph make_constant_degree_graph(const Graph& input) {
  const std::size_t original_count = input.vertex_count();
  std::vector<OriginalEdge> edges;
  edges.reserve(input.edge_count());
  for (Vertex from = 0; from < original_count; ++from) {
    for (const Edge& edge : input.edges_from(from)) {
      edges.push_back({from, edge.to, edge.weight, 0, 0});
    }
  }

  const std::size_t transformed_count = original_count + 2 * edges.size();
  ConstantDegreeGraph result{Graph(transformed_count), std::vector<Vertex>(transformed_count),
                             original_count};
  std::vector<std::vector<Vertex>> cycles(original_count);
  for (Vertex vertex = 0; vertex < original_count; ++vertex) {
    result.original_vertex[vertex] = vertex;
    cycles[vertex].push_back(vertex);
  }

  Vertex next = original_count;
  for (OriginalEdge& edge : edges) {
    edge.from_slot = next++;
    edge.to_slot = next++;
    result.original_vertex[edge.from_slot] = edge.from;
    result.original_vertex[edge.to_slot] = edge.to;
    cycles[edge.from].push_back(edge.from_slot);
    cycles[edge.to].push_back(edge.to_slot);
  }

  for (const auto& cycle : cycles) {
    if (cycle.size() == 1) {
      continue;
    }
    for (std::size_t index = 0; index < cycle.size(); ++index) {
      result.graph.add_edge(cycle[index], cycle[(index + 1) % cycle.size()], 0.0);
    }
  }
  for (const OriginalEdge& edge : edges) {
    result.graph.add_edge(edge.from_slot, edge.to_slot, edge.weight);
  }
  return result;
}

} // namespace sssp::detail
