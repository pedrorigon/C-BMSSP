#include "block_queue.h"
#include "sequential_solver.h"
#include "test_assert.h"

#include <optional>
#include <utility>
#include <vector>

namespace {

class SolverHarness final : public sssp::SequentialSolver {
public:
  explicit SolverHarness(const sssp::Graph& graph)
      : SequentialSolver(graph, sssp::SolverOptions{0, 0}) {}

  void initialize(const sssp::Vertex source) {
    reset();
    distances_[source] = 0.0;
  }

  void set_distance(const sssp::Vertex vertex, const double distance) {
    distances_[vertex] = distance;
  }

  void mark_complete(const sssp::Vertex vertex) { complete_[vertex] = true; }

  [[nodiscard]] std::pair<double, std::vector<sssp::Vertex>>
  search(const std::size_t level, const double bound, std::vector<sssp::Vertex> frontier,
         const std::optional<sssp::Vertex> goal = std::nullopt) {
    return bounded_search(level, bound, std::move(frontier), goal);
  }

  [[nodiscard]] std::pair<double, std::vector<sssp::Vertex>>
  run_base_case(const double bound, const std::vector<sssp::Vertex>& frontier,
                const std::optional<sssp::Vertex> goal = std::nullopt) {
    return base_case(bound, frontier, goal);
  }

  [[nodiscard]] std::pair<std::vector<sssp::Vertex>, std::vector<sssp::Vertex>>
  select_pivots(const double bound, const std::vector<sssp::Vertex>& frontier) {
    return find_pivots(bound, frontier);
  }

  void relax(const std::vector<sssp::Vertex>& vertices, const double lower_bound,
             const double upper_bound, sssp::detail::BlockQueue& queue) {
    relax_completed(vertices, lower_bound, upper_bound, queue);
  }

  void complete(const sssp::Vertex goal) { complete_shortest_paths(goal); }

  [[nodiscard]] std::vector<sssp::Vertex> path(const sssp::Vertex source,
                                               const sssp::Vertex goal) const {
    return reconstruct_path(source, goal);
  }
};

void test_base_case_and_completion() {
  sssp::Graph graph(1'102);
  graph.add_edge(0, 1, 10.0);
  graph.add_edge(0, 2, 1.0);
  graph.add_edge(2, 1, 1.0);
  for (sssp::Vertex vertex = 2; vertex + 1 < graph.vertex_count(); ++vertex) {
    graph.add_edge(vertex, vertex + 1, 1.0);
  }

  SolverHarness solver(graph);
  solver.initialize(0);
  const auto [bound, visited] = solver.run_base_case(sssp::infinity, {0});
  test::require(bound == sssp::infinity, "base case changed the bound");
  test::require(visited.size() == 1'001, "base case processing limit was not applied");

  solver.initialize(0);
  solver.complete(1);
  test::require(solver.path(0, 1) == std::vector<sssp::Vertex>({0, 2, 1}),
                "completion did not build the shortest path");
  test::require(solver.path(0, 1'101).empty(), "broken predecessor chain was accepted");
}

void test_search_and_relaxation() {
  sssp::Graph graph(5);
  graph.add_edge(0, 1, 2.0);
  graph.add_edge(0, 2, 12.0);
  graph.add_edge(0, 3, 25.0);
  graph.add_edge(1, 4, 1.0);

  SolverHarness solver(graph);
  solver.initialize(0);
  const auto [empty_bound, empty_result] = solver.search(0, 9.0, {});
  test::require_near(9.0, empty_bound, 1e-9, "empty search changed the bound");
  test::require(empty_result.empty(), "empty search returned vertices");

  solver.mark_complete(4);
  const auto [complete_bound, complete_result] = solver.search(1, 8.0, {0}, 4);
  test::require_near(8.0, complete_bound, 1e-9, "completed-goal search changed the bound");
  test::require(complete_result.empty(), "completed-goal search returned vertices");

  solver.initialize(0);
  const auto [pivots, working_set] = solver.select_pivots(sssp::infinity, {0});
  test::require(!pivots.empty(), "pivot selection returned no pivots");
  test::require(working_set.size() >= 4, "pivot selection did not expand the frontier");

  solver.initialize(0);
  sssp::detail::BlockQueue queue(4, 30.0);
  solver.relax({0}, 10.0, 20.0, queue);
  const auto [remaining_bound, prepended] = queue.pull();
  test::require(prepended == std::vector<sssp::Vertex>({1}), "lower-bound batch is incorrect");
  test::require_near(12.0, remaining_bound, 1e-9, "queued minimum is incorrect");
  const auto [final_bound, queued] = queue.pull();
  test::require(queued == std::vector<sssp::Vertex>({2}), "bounded insertion is incorrect");
  test::require_near(30.0, final_bound, 1e-9, "final queue bound is incorrect");
}

void run_tests() {
  test_base_case_and_completion();
  test_search_and_relaxation();
}

} // namespace

int main() { return test::run(run_tests); }
