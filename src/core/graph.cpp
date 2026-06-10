#include "graph.h"

#include <cmath>
#include <stdexcept>

namespace sssp {

Graph::Graph(const std::size_t vertices) : adjacency_(vertices) {}

void Graph::add_edge(const Vertex from, const Vertex to, const double weight) {
  if (from >= adjacency_.size() || to >= adjacency_.size()) {
    throw std::out_of_range("edge vertex is outside graph");
  }
  if (!std::isfinite(weight) || weight < 0.0) {
    throw std::invalid_argument("SSSP requires finite, non-negative edge weights");
  }
  adjacency_[from].push_back({to, weight});
  ++edge_count_;
}

std::size_t Graph::vertex_count() const noexcept { return adjacency_.size(); }

std::size_t Graph::edge_count() const noexcept { return edge_count_; }

const std::vector<Edge>& Graph::edges_from(const Vertex vertex) const {
  return adjacency_.at(vertex);
}

} // namespace sssp
