#pragma once

#include <cstddef>
#include <vector>

namespace sssp {

using Vertex = std::size_t;

struct Edge {
  Vertex to;
  double weight;
};

class Graph {
public:
  explicit Graph(std::size_t vertices = 0);

  void add_edge(Vertex from, Vertex to, double weight);

  [[nodiscard]] std::size_t vertex_count() const noexcept;
  [[nodiscard]] std::size_t edge_count() const noexcept;
  [[nodiscard]] const std::vector<Edge>& edges_from(Vertex vertex) const;

private:
  std::vector<std::vector<Edge>> adjacency_;
  std::size_t edge_count_{0};
};

} // namespace sssp
