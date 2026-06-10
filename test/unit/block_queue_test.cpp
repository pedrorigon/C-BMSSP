#include "block_queue.h"
#include "test_assert.h"

#include <vector>

namespace {

void run_tests() {
  sssp::detail::BlockQueue queue(2, 10.0);
  test::require(queue.empty(), "new queue is not empty");

  queue.insert(1, 5.0);
  queue.insert(2, 2.0);
  queue.insert(3, 12.0);
  queue.insert(4, 4.0);
  test::require(!queue.empty(), "inserted queue is empty");

  const auto [first_bound, first_vertices] = queue.pull();
  test::require(first_vertices == std::vector<sssp::Vertex>({4}), "latest block was not pulled");
  test::require_near(2.0, first_bound, 1e-9, "remaining minimum is incorrect");

  queue.batch_prepend({{5, 3.0}, {6, 1.0}});
  const auto [batch_bound, batch_vertices] = queue.pull();
  test::require(batch_vertices == std::vector<sssp::Vertex>({6, 5}), "batch order is incorrect");
  test::require_near(2.0, batch_bound, 1e-9, "minimum after batch pull is incorrect");

  const auto [last_bound, last_vertices] = queue.pull();
  test::require(last_vertices == std::vector<sssp::Vertex>({2, 1}), "sorted block is incorrect");
  test::require_near(10.0, last_bound, 1e-9, "empty queue bound is incorrect");
  test::require(queue.empty(), "queue is not empty after all pulls");

  const auto [empty_bound, empty_vertices] = queue.pull();
  test::require(empty_vertices.empty(), "empty pull returned vertices");
  test::require_near(10.0, empty_bound, 1e-9, "empty pull bound is incorrect");
}

} // namespace

int main() { return test::run(run_tests); }
