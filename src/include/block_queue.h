#pragma once

#include "graph.h"

#include <cstddef>
#include <deque>
#include <optional>
#include <utility>
#include <vector>

namespace sssp::detail {

class BlockQueue {
public:
  BlockQueue(std::size_t block_size, double bound);

  void insert(Vertex vertex, double distance);
  void batch_prepend(std::vector<std::pair<Vertex, double>> items);
  [[nodiscard]] std::pair<double, std::vector<Vertex>> pull();
  [[nodiscard]] bool empty() const noexcept;

private:
  [[nodiscard]] std::optional<double> peek_min() const;

  std::deque<std::vector<std::pair<Vertex, double>>> batch_blocks_;
  std::vector<std::vector<std::pair<Vertex, double>>> sorted_blocks_;
  std::size_t block_size_;
  double bound_;
};

} // namespace sssp::detail
