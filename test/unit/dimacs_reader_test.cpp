#include "dimacs_reader.h"
#include "test_assert.h"

#include <filesystem>
#include <stdexcept>

namespace {

void require_invalid(const std::filesystem::path& path) {
  test::require_throws<std::exception>(
      [&path] { static_cast<void>(sssp::read_dimacs_graph(path)); },
      "invalid DIMACS input was accepted");
}

void run_tests() {
  const std::filesystem::path data_directory = SSSP_TEST_DATA_DIR;
  const sssp::Graph graph = sssp::read_dimacs_graph(data_directory / "tiny.dimacs");
  test::require(graph.vertex_count() == 4, "DIMACS vertex count mismatch");
  test::require(graph.edge_count() == 5, "DIMACS edge count mismatch");
  test::require_near(1.5, graph.edges_from(0).front().weight, 1e-9, "DIMACS weight mismatch");

  require_invalid(data_directory / "missing.dimacs");
  require_invalid(data_directory / "missing_problem.dimacs");
  require_invalid(data_directory / "arc_before_problem.dimacs");
  require_invalid(data_directory / "edge_count_mismatch.dimacs");
  require_invalid(data_directory / "duplicate_problem.dimacs");
  require_invalid(data_directory / "invalid_problem_type.dimacs");
  require_invalid(data_directory / "out_of_range_arc.dimacs");
  require_invalid(data_directory / "negative_weight.dimacs");
  require_invalid(data_directory / "invalid_arc.dimacs");
}

} // namespace

int main() { return test::run(run_tests); }
