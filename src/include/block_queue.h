#pragma once

#include "graph.h"

#include <compare>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <list>
#include <map>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace sssp::detail {

// A path label ordered exactly as required when comparing labels belonging to
// different vertices: numeric length, number of traversed vertices, then the
// endpoint. The predecessor tie-break used while relaxing one fixed endpoint is
// maintained by SequentialSolver.
struct QueueKey {
  double distance;
  std::size_t vertices;
  Vertex endpoint;

  [[nodiscard]] static QueueKey infinity() noexcept;
  auto operator<=>(const QueueKey&) const = default;
};

struct QueueItem {
  Vertex vertex;
  QueueKey key;
};

// Lemma 3.3's partial-order data structure. Insertions are kept in an ordered
// sequence of O(N/M) blocks indexed by a balanced tree. Batch prepends are kept
// in a separate ordered block sequence. Blocks contain at most M live entries.
class BlockQueue {
public:
  explicit BlockQueue(std::size_t vertex_capacity);
  void initialize(std::size_t block_size, QueueKey bound);

  void insert(Vertex vertex, QueueKey key);
  void erase(Vertex vertex);
  void batch_prepend(std::vector<QueueItem> items);
  [[nodiscard]] std::pair<QueueKey, std::vector<Vertex>> pull();
  [[nodiscard]] bool empty() const noexcept;
  [[nodiscard]] std::size_t size() const noexcept;

private:
  struct Entry {
    Vertex vertex;
    QueueKey key;
  };

  struct Block;
  using BlockOwner = std::unique_ptr<Block>;
  using BlockSequence = std::list<BlockOwner>;

  struct Block {
    std::vector<Entry> entries;
    QueueKey upper_bound;
    bool batch;
    BlockSequence::iterator owner;
  };

  struct Location {
    Block* block{nullptr};
    std::size_t index{0};
  };

  void erase_location(Location location);
  void split_insert_block(Block* block);
  void rebuild_locations(Block* block);
  void append_partitioned_batch(std::vector<Entry>& entries, std::size_t first, std::size_t last,
                                std::vector<BlockOwner>& output);
  [[nodiscard]] std::optional<QueueKey> minimum_key() const;
  [[nodiscard]] static QueueKey maximum_key(const Block& block);
  [[nodiscard]] static QueueKey minimum_key(const Block& block);
  [[nodiscard]] bool is_last_insert_block(const Block* block) const;
  void remove_empty_insert_blocks();
  void remove_empty_batch_blocks();

  BlockSequence batch_blocks_;
  BlockSequence insert_blocks_;
  std::map<QueueKey, Block*> insert_index_;
  std::vector<Location> locations_;
  std::vector<std::uint32_t> batch_marks_;
  std::vector<QueueKey> batch_minima_;
  std::uint32_t batch_mark_{0};
  std::size_t block_size_;
  QueueKey bound_;
  std::size_t size_{0};
};

} // namespace sssp::detail
