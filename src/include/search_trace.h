#pragma once

#include "graph.h"

#include <functional>

namespace sssp {

enum class SearchEventKind {
  frontier,
  settled,
  relaxation,
  completion,
};

struct SearchEvent {
  SearchEventKind kind;
  Vertex vertex;
  double distance;
};

using SearchTraceCallback = std::function<void(const SearchEvent&)>;

} // namespace sssp
