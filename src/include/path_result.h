#pragma once

#include "graph.h"

#include <limits>
#include <vector>

namespace sssp {

inline constexpr double infinity = std::numeric_limits<double>::infinity();

struct PathResult {
  double distance{infinity};
  std::vector<Vertex> path;

  [[nodiscard]] bool reachable() const noexcept { return distance != infinity; }
};

} // namespace sssp
