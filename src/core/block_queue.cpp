#include "block_queue.h"

#include "path_result.h"

#include <algorithm>

namespace sssp::detail {

BlockQueue::BlockQueue(const std::size_t block_size, const double bound)
    : block_size_(block_size), bound_(bound) {}

void BlockQueue::insert(const Vertex vertex, const double distance) {
  if (distance >= bound_) {
    return;
  }
  if (sorted_blocks_.empty() || sorted_blocks_.back().size() >= block_size_) {
    sorted_blocks_.emplace_back();
    sorted_blocks_.back().reserve(block_size_);
  }
  sorted_blocks_.back().emplace_back(vertex, distance);
}

void BlockQueue::batch_prepend(std::vector<std::pair<Vertex, double>> items) {
  if (!items.empty()) {
    batch_blocks_.push_back(std::move(items));
  }
}

std::pair<double, std::vector<Vertex>> BlockQueue::pull() {
  std::vector<std::pair<Vertex, double>> block;
  if (!batch_blocks_.empty()) {
    block = std::move(batch_blocks_.front());
    batch_blocks_.pop_front();
  } else if (!sorted_blocks_.empty()) {
    block = std::move(sorted_blocks_.back());
    sorted_blocks_.pop_back();
  } else {
    return {bound_, {}};
  }

  std::ranges::sort(block, {}, &std::pair<Vertex, double>::second);
  std::vector<Vertex> vertices;
  vertices.reserve(block.size());
  for (const auto& item : block) {
    vertices.push_back(item.first);
  }
  return {peek_min().value_or(bound_), std::move(vertices)};
}

bool BlockQueue::empty() const noexcept { return batch_blocks_.empty() && sorted_blocks_.empty(); }

std::optional<double> BlockQueue::peek_min() const {
  double minimum = infinity;
  const auto inspect = [&minimum](const auto& blocks) {
    for (const auto& block : blocks) {
      for (const auto& item : block) {
        minimum = std::min(minimum, item.second);
      }
    }
  };
  inspect(batch_blocks_);
  inspect(sorted_blocks_);
  return minimum == infinity ? std::nullopt : std::optional<double>{minimum};
}

} // namespace sssp::detail
