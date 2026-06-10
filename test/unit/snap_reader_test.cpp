#include "snap_reader.h"
#include "test_assert.h"

#include <filesystem>

namespace {

void run_tests() {
  const std::filesystem::path path = std::filesystem::path(SSSP_TEST_DATA_DIR) / "tiny.snap";

  const sssp::Graph directed = sssp::read_snap_graph(path);
  test::require(directed.vertex_count() == 4, "SNAP vertex count mismatch");
  test::require(directed.edge_count() == 5, "SNAP directed edge count mismatch");
  test::require_near(1.0, directed.edges_from(0).front().weight, 1e-9, "SNAP weight mismatch");

  const sssp::Graph undirected = sssp::read_snap_graph(path, false);
  test::require(undirected.edge_count() == 10, "SNAP undirected edge count mismatch");
}

} // namespace

int main() { return test::run(run_tests); }
