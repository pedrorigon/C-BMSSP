#pragma once

#include "graph.h"

#include <filesystem>

namespace sssp {

[[nodiscard]] Graph read_snap_graph(const std::filesystem::path& path, bool directed = true);

} // namespace sssp
