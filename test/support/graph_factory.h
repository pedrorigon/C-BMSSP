#pragma once

#include "graph.h"

#include <cstddef>
#include <cstdint>

namespace test {

[[nodiscard]] sssp::Graph make_path_graph(std::size_t vertex_count, double weight = 1.0);
[[nodiscard]] sssp::Graph make_ring_lattice(std::size_t vertex_count, std::size_t degree);
[[nodiscard]] sssp::Graph make_random_graph(std::size_t vertex_count, std::size_t extra_edges,
                                            std::uint64_t seed);

} // namespace test
