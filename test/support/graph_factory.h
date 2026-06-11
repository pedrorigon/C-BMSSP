#pragma once

#include "graph.h"

#include <cstddef>
#include <cstdint>

namespace test {

[[nodiscard]] sssp::Graph make_path_graph(std::size_t vertex_count, double weight = 1.0);
[[nodiscard]] sssp::Graph make_ring_lattice(std::size_t vertex_count, std::size_t degree);
[[nodiscard]] sssp::Graph make_random_graph(std::size_t vertex_count, std::size_t extra_edges,
                                            std::uint64_t seed);

// Large-diameter sparse weighted graph. Each vertex links to `degree` forward
// neighbours within `bandwidth` positions (plus a path backbone for
// connectivity), with random weights. A small bandwidth yields a deep
// shortest-path tree (diameter grows with n), which stresses Dijkstra's heap and
// is the regime where the BMSSP sorting-barrier advantage shows up.
[[nodiscard]] sssp::Graph make_banded_graph(std::size_t vertex_count, std::size_t degree,
                                            std::size_t bandwidth, std::uint64_t seed);

} // namespace test
