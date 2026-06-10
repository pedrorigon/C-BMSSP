#pragma once

#include "graph.h"

#include <filesystem>

namespace sssp {

[[nodiscard]] Graph read_dimacs_graph(const std::filesystem::path& path);

} // namespace sssp
