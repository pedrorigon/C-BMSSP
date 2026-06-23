#include "block_queue.h"

#include "work_counter.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>

namespace sssp::detail {

QueueKey QueueKey::infinity() noexcept {
  return {std::numeric_limits<double>::infinity(), std::numeric_limits<std::size_t>::max(),
          std::numeric_limits<Vertex>::max()};
}

BlockQueue::BlockQueue(const std::size_t vertex_capacity)
    : locations_(vertex_capacity), batch_marks_(vertex_capacity, 0),
      batch_minima_(vertex_capacity, QueueKey::infinity()), block_size_(1),
      bound_(QueueKey::infinity()) {
  initialize(1, QueueKey::infinity());
}

void BlockQueue::initialize(const std::size_t block_size, const QueueKey bound) {
  const auto clear_locations = [this](const BlockSequence& blocks) {
    for (const BlockOwner& block : blocks) {
      for (const Entry& entry : block->entries) {
        locations_[entry.vertex].block = nullptr;
      }
    }
  };
  clear_locations(batch_blocks_);
  clear_locations(insert_blocks_);
  batch_blocks_.clear();
  insert_blocks_.clear();
  insert_index_.clear();
  size_ = 0;
  block_size_ = std::max<std::size_t>(block_size, 1);
  bound_ = bound;

  auto sentinel = std::make_unique<Block>();
  sentinel->entries.reserve(block_size_);
  sentinel->upper_bound = bound_;
  sentinel->batch = false;
  Block* const pointer = sentinel.get();
  insert_blocks_.push_back(std::move(sentinel));
  pointer->owner = std::prev(insert_blocks_.end());
  insert_index_.emplace(bound_, pointer);
}

void BlockQueue::rebuild_locations(Block* const block) {
  for (std::size_t index = 0; index < block->entries.size(); ++index) {
    locations_.at(block->entries[index].vertex) = {block, index};
  }
}

bool BlockQueue::is_last_insert_block(const Block* const block) const {
  return !insert_blocks_.empty() && insert_blocks_.back().get() == block;
}

QueueKey BlockQueue::maximum_key(const Block& block) {
  assert(!block.entries.empty());
  return std::ranges::max(block.entries, {}, &Entry::key).key;
}

QueueKey BlockQueue::minimum_key(const Block& block) {
  assert(!block.entries.empty());
  return std::ranges::min(block.entries, {}, &Entry::key).key;
}

void BlockQueue::erase_location(const Location location) {
  Block* const block = location.block;
  if (!block->batch) {
    insert_index_.erase(block->upper_bound);
  }
  const std::size_t last = block->entries.size() - 1;
  if (location.index != last) {
    block->entries[location.index] = std::move(block->entries[last]);
    locations_[block->entries[location.index].vertex] = {block, location.index};
  }
  block->entries.pop_back();
  --size_;

  if (!block->batch) {
    if (block->entries.empty()) {
      if (is_last_insert_block(block)) {
        block->upper_bound = bound_;
        insert_index_.emplace(bound_, block);
      } else {
        insert_blocks_.erase(block->owner);
      }
    } else {
      // A stale separator remains valid, but reinserting it keeps the tree in
      // sync after deletion of the entry that previously attained the bound.
      insert_index_.emplace(block->upper_bound, block);
    }
  } else if (block->entries.empty()) {
    batch_blocks_.erase(block->owner);
  }
}

void BlockQueue::erase(const Vertex vertex) {
  Location& stored = locations_.at(vertex);
  if (stored.block == nullptr) {
    return;
  }
  const Location location = stored;
  stored.block = nullptr;
  erase_location(location);
}

void BlockQueue::split_insert_block(Block* const block) {
  if (block->entries.size() <= block_size_) {
    return;
  }

  std::vector<Entry> entries;
  entries.reserve(block->entries.size());
  for (const Entry& entry : block->entries) {
    entries.push_back(entry);
  }
  const std::size_t middle = entries.size() / 2;
  std::ranges::nth_element(entries, entries.begin() + static_cast<std::ptrdiff_t>(middle), {},
                           &Entry::key);

  const QueueKey previous_bound = block->upper_bound;
  insert_index_.erase(previous_bound);
  block->entries.clear();
  for (std::size_t index = 0; index < middle; ++index) {
    block->entries.push_back(entries[index]);
  }
  block->upper_bound = maximum_key(*block);
  insert_index_.emplace(block->upper_bound, block);
  rebuild_locations(block);

  auto right = std::make_unique<Block>();
  right->entries.reserve(entries.size() - middle);
  right->upper_bound = previous_bound;
  right->batch = false;
  for (std::size_t index = middle; index < entries.size(); ++index) {
    right->entries.push_back(entries[index]);
  }
  Block* const right_pointer = right.get();

  const auto inserted_owner = insert_blocks_.insert(std::next(block->owner), std::move(right));
  right_pointer->owner = inserted_owner;
  insert_index_.emplace(right_pointer->upper_bound, right_pointer);
  rebuild_locations(right_pointer);
}

void BlockQueue::insert(const Vertex vertex, const QueueKey key) {
  if (!(key < bound_)) {
    return;
  }
  Location& existing = locations_.at(vertex);
  if (existing.block != nullptr) {
    if (!(key < existing.block->entries[existing.index].key)) {
      return;
    }
    erase(vertex);
  }

  // Lemma 3.3: an insert locates its block through the O(N/M) balanced tree,
  // costing O(max{1, log(N/M)}). The tree holds one separator per insert block.
  WorkCounter::add_log(insert_index_.size());
  auto block_entry = insert_index_.lower_bound(key);
  if (block_entry == insert_index_.end()) {
    block_entry = std::prev(insert_index_.end());
  }
  Block* const block = block_entry->second;
  block->entries.push_back({vertex, key});
  locations_[vertex] = {block, block->entries.size() - 1};
  ++size_;
  split_insert_block(block);
}

void BlockQueue::append_partitioned_batch(std::vector<Entry>& entries, const std::size_t first,
                                          const std::size_t last, std::vector<BlockOwner>& output) {
  const std::size_t count = last - first;
  if (count <= block_size_) {
    auto block = std::make_unique<Block>();
    block->entries.reserve(count);
    block->batch = true;
    for (std::size_t index = first; index < last; ++index) {
      block->entries.push_back(entries[index]);
    }
    block->upper_bound = maximum_key(*block);
    output.push_back(std::move(block));
    return;
  }

  const std::size_t middle = first + count / 2;
  std::nth_element(entries.begin() + static_cast<std::ptrdiff_t>(first),
                   entries.begin() + static_cast<std::ptrdiff_t>(middle),
                   entries.begin() + static_cast<std::ptrdiff_t>(last),
                   [](const Entry& left, const Entry& right) { return left.key < right.key; });
  append_partitioned_batch(entries, first, middle, output);
  append_partitioned_batch(entries, middle, last, output);
}

void BlockQueue::batch_prepend(std::vector<QueueItem> items) {
  if (items.empty()) {
    return;
  }

  if (batch_mark_ == std::numeric_limits<std::uint32_t>::max()) {
    std::fill(batch_marks_.begin(), batch_marks_.end(), 0);
    batch_mark_ = 0;
  }
  ++batch_mark_;
  std::vector<Vertex> touched;
  touched.reserve(items.size());
  for (const QueueItem& item : items) {
    if (!(item.key < bound_)) {
      continue;
    }
    if (batch_marks_.at(item.vertex) != batch_mark_) {
      batch_marks_[item.vertex] = batch_mark_;
      batch_minima_[item.vertex] = item.key;
      touched.push_back(item.vertex);
    } else if (item.key < batch_minima_[item.vertex]) {
      batch_minima_[item.vertex] = item.key;
    }
  }

  std::vector<Entry> entries;
  entries.reserve(touched.size());
  for (const Vertex vertex : touched) {
    const QueueKey key = batch_minima_[vertex];
    Location& existing = locations_[vertex];
    if (existing.block != nullptr && !(key < existing.block->entries[existing.index].key)) {
      continue;
    }
    if (existing.block != nullptr) {
      erase(vertex);
    }
    entries.push_back({vertex, key});
  }
  if (entries.empty()) {
    return;
  }

#ifndef NDEBUG
  if (const auto current = minimum_key()) {
    for (const Entry& entry : entries) {
      assert(entry.key < *current);
    }
  }
#endif

  // Lemma 3.3: a batch prepend of L items partitions by medians into blocks of
  // size <= M, costing O(L max{1, log(L/M)}).
  {
    const double length = static_cast<double>(entries.size());
    const double ratio = length / static_cast<double>(std::max<std::size_t>(block_size_, 1));
    WorkCounter::add(length * std::max(1.0, ratio > 1.0 ? std::log2(ratio) : 1.0));
  }

  std::vector<BlockOwner> new_blocks;
  new_blocks.reserve((entries.size() + block_size_ - 1) / block_size_);
  append_partitioned_batch(entries, 0, entries.size(), new_blocks);
  for (auto iterator = new_blocks.rbegin(); iterator != new_blocks.rend(); ++iterator) {
    Block* const block = iterator->get();
    const auto owner = batch_blocks_.insert(batch_blocks_.begin(), std::move(*iterator));
    block->owner = owner;
    rebuild_locations(block);
  }
  size_ += entries.size();
}

void BlockQueue::remove_empty_insert_blocks() {
  for (auto iterator = insert_blocks_.begin(); iterator != insert_blocks_.end();) {
    Block* const block = iterator->get();
    if (!block->entries.empty() || is_last_insert_block(block)) {
      ++iterator;
      continue;
    }
    insert_index_.erase(block->upper_bound);
    iterator = insert_blocks_.erase(iterator);
  }
}

void BlockQueue::remove_empty_batch_blocks() {
  for (auto iterator = batch_blocks_.begin(); iterator != batch_blocks_.end();) {
    if ((*iterator)->entries.empty()) {
      iterator = batch_blocks_.erase(iterator);
    } else {
      ++iterator;
    }
  }
}

std::optional<QueueKey> BlockQueue::minimum_key() const {
  std::optional<QueueKey> result;
  const auto inspect = [&result](const BlockSequence& blocks) {
    for (const BlockOwner& block : blocks) {
      if (!block->entries.empty()) {
        const QueueKey candidate = BlockQueue::minimum_key(*block);
        if (!result || candidate < *result) {
          result = candidate;
        }
        return;
      }
    }
  };
  inspect(batch_blocks_);
  inspect(insert_blocks_);
  return result;
}

std::pair<QueueKey, std::vector<Vertex>> BlockQueue::pull() {
  if (empty()) {
    return {bound_, {}};
  }

  std::vector<Entry*> candidates;
  candidates.reserve(block_size_ * 4);
  const auto collect_prefix = [this, &candidates](BlockSequence& blocks) {
    std::size_t collected = 0;
    for (BlockOwner& block : blocks) {
      for (Entry& entry : block->entries) {
        candidates.push_back(&entry);
        ++collected;
      }
      if (collected >= block_size_) {
        break;
      }
    }
  };
  collect_prefix(batch_blocks_);
  collect_prefix(insert_blocks_);

  // Lemma 3.3: a pull scans and partial-sorts the collected prefix, O(|S'|).
  WorkCounter::add(static_cast<double>(candidates.size()));

  const std::size_t take = std::min(block_size_, candidates.size());
  if (take < candidates.size()) {
    std::ranges::nth_element(candidates, candidates.begin() + static_cast<std::ptrdiff_t>(take), {},
                             [](const Entry* entry) { return entry->key; });
  }

  std::vector<Vertex> vertices;
  vertices.reserve(take);
  for (std::size_t index = 0; index < take; ++index) {
    vertices.push_back(candidates[index]->vertex);
  }
  for (const Vertex vertex : vertices) {
    erase(vertex);
  }

  return {minimum_key().value_or(bound_), std::move(vertices)};
}

bool BlockQueue::empty() const noexcept { return size_ == 0; }

std::size_t BlockQueue::size() const noexcept { return size_; }

} // namespace sssp::detail
