#pragma once

#include "graph.h"

#include <vector>

namespace sssp::detail {

struct ConstantDegreeGraph {
  Graph graph;
  std::vector<Vertex> original_vertex;
  std::size_t original_vertex_count;
};

// Frederickson's classical degree reduction used by the paper. Original
// vertices retain IDs [0,n); every incidence gets a private zero-cycle vertex.
// Each transformed vertex has in-degree and out-degree at most two.
[[nodiscard]] ConstantDegreeGraph make_constant_degree_graph(const Graph& graph);

} // namespace sssp::detail
