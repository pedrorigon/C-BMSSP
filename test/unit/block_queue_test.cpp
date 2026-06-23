#include "block_queue.h"
#include "test_assert.h"

#include <algorithm>
#include <map>
#include <random>
#include <vector>

namespace {

sssp::detail::QueueKey key(const double distance, const sssp::Vertex vertex) {
  return {distance, 1, vertex};
}

void require_same_vertices(std::vector<sssp::Vertex> actual, std::vector<sssp::Vertex> expected,
                           const std::string& message) {
  std::ranges::sort(actual);
  std::ranges::sort(expected);
  test::require(actual == expected, message);
}

void test_fixed_operations() {
  using sssp::detail::BlockQueue;
  using sssp::detail::QueueItem;
  using sssp::detail::QueueKey;

  BlockQueue queue(20);
  queue.initialize(2, QueueKey::infinity());
  test::require(queue.empty(), "new queue is not empty");

  queue.insert(1, key(5.0, 1));
  queue.insert(2, key(2.0, 2));
  queue.insert(3, key(12.0, 3));
  queue.insert(4, key(4.0, 4));
  queue.insert(1, key(3.0, 1)); // decrease-key; old value must disappear.
  queue.insert(2, key(9.0, 2)); // larger duplicate must be ignored.
  test::require(queue.size() == 4, "queue did not keep unique keys");

  const auto [first_bound, first_vertices] = queue.pull();
  require_same_vertices(first_vertices, {1, 2}, "pull did not return the global minima");
  test::require_near(4.0, first_bound.distance, 1e-9,
                     "pull separator is not the remaining minimum");

  queue.batch_prepend(
      {QueueItem{5, key(1.5, 5)}, QueueItem{6, key(1.0, 6)}, QueueItem{5, key(1.25, 5)}});
  const auto [batch_bound, batch_vertices] = queue.pull();
  require_same_vertices(batch_vertices, {5, 6}, "batch prepend was not pulled first");
  test::require_near(4.0, batch_bound.distance, 1e-9, "minimum after batch pull is incorrect");

  // More than M batch items exercise recursive median partitioning.
  queue.batch_prepend({QueueItem{7, key(0.7, 7)}, QueueItem{8, key(0.8, 8)},
                       QueueItem{9, key(0.9, 9)}, QueueItem{10, key(0.6, 10)},
                       QueueItem{11, key(0.5, 11)}});
  const auto [partition_bound, partition_vertices] = queue.pull();
  require_same_vertices(partition_vertices, {10, 11},
                        "partitioned batch did not return its two minima");
  test::require_near(0.7, partition_bound.distance, 1e-9,
                     "partitioned batch separator is incorrect");

  std::vector<sssp::Vertex> remaining;
  while (!queue.empty()) {
    auto [unused, vertices] = queue.pull();
    static_cast<void>(unused);
    remaining.insert(remaining.end(), vertices.begin(), vertices.end());
  }
  require_same_vertices(remaining, {3, 4, 7, 8, 9}, "queue lost or duplicated entries");

  const auto [empty_bound, empty_vertices] = queue.pull();
  test::require(empty_vertices.empty(), "empty pull returned vertices");
  test::require(empty_bound == QueueKey::infinity(), "empty pull changed the bound");
}

void test_randomized_operations() {
  using sssp::detail::BlockQueue;
  using sssp::detail::QueueItem;
  using sssp::detail::QueueKey;

  constexpr std::size_t vertex_count = 128;
  std::mt19937_64 random(0xB55BULL);
  std::uniform_int_distribution<sssp::Vertex> vertex_distribution(0, vertex_count - 1);
  std::uniform_real_distribution<double> distance_distribution(0.0, 10'000.0);

  for (const std::size_t block_size : {1U, 2U, 7U, 19U}) {
    BlockQueue queue(vertex_count);
    queue.initialize(block_size, QueueKey::infinity());
    std::map<QueueKey, sssp::Vertex> reference;
    std::vector<QueueKey> stored(vertex_count, QueueKey::infinity());

    const auto insert_reference = [&](const sssp::Vertex vertex, const QueueKey candidate) {
      if (!(candidate < stored[vertex])) {
        return;
      }
      if (stored[vertex] != QueueKey::infinity()) {
        reference.erase(stored[vertex]);
      }
      stored[vertex] = candidate;
      reference.emplace(candidate, vertex);
    };

    for (std::size_t operation = 0; operation < 4'000; ++operation) {
      if (reference.empty() || operation % 5 < 3) {
        const sssp::Vertex vertex = vertex_distribution(random);
        const QueueKey candidate = key(distance_distribution(random), vertex);
        queue.insert(vertex, candidate);
        insert_reference(vertex, candidate);
      } else if (operation % 5 == 3) {
        std::vector<QueueItem> batch;
        const double current_minimum = reference.begin()->first.distance;
        for (std::size_t item = 0; item < 2 * block_size + 3; ++item) {
          const sssp::Vertex vertex = vertex_distribution(random);
          const QueueKey candidate{current_minimum - 1.0 -
                                       static_cast<double>(item) /
                                           static_cast<double>(2 * block_size + 3),
                                   1, vertex};
          batch.push_back({vertex, candidate});
          insert_reference(vertex, candidate);
        }
        queue.batch_prepend(std::move(batch));
      } else {
        const std::size_t take = std::min(block_size, reference.size());
        std::vector<sssp::Vertex> expected;
        expected.reserve(take);
        for (std::size_t index = 0; index < take; ++index) {
          const auto iterator = reference.begin();
          expected.push_back(iterator->second);
          stored[iterator->second] = QueueKey::infinity();
          reference.erase(iterator);
        }
        const auto [next_bound, actual] = queue.pull();
        require_same_vertices(actual, expected, "randomized pull differs from ordered map");
        const QueueKey expected_bound =
            reference.empty() ? QueueKey::infinity() : reference.begin()->first;
        test::require(next_bound == expected_bound, "randomized pull returned the wrong separator");
      }
      test::require(queue.size() == reference.size(),
                    "randomized queue size differs from reference");
    }
  }
}

void run_tests() {
  test_fixed_operations();
  test_randomized_operations();
}

} // namespace

int main() { return test::run(run_tests); }
