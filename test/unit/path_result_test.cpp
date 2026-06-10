#include "path_result.h"
#include "test_assert.h"

namespace {

void run_tests() {
  const sssp::PathResult unreachable;
  test::require(!unreachable.reachable(), "default result must be unreachable");

  const sssp::PathResult reachable{0.0, {0}};
  test::require(reachable.reachable(), "finite result must be reachable");
}

} // namespace

int main() { return test::run(run_tests); }
