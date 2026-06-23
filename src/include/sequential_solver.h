#pragma once

#include "block_queue.h"
#include "constant_degree_graph.h"
#include "graph.h"
#include "path_result.h"
#include "search_trace.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace sssp {

struct SolverOptions {
  // Retained for source compatibility. The paper algorithm is now used for all
  // graph sizes; these thresholds are no longer semantic switches.
  std::size_t minimum_optimized_vertices{0};
  std::size_t minimum_optimized_edges{0};
};

class SequentialSolver {
public:
  explicit SequentialSolver(const Graph& graph, SolverOptions options = {});
  virtual ~SequentialSolver() = default;

  [[nodiscard]] PathResult solve(Vertex source, Vertex goal);
  [[nodiscard]] std::vector<double> solve_all(Vertex source);
  void set_trace_callback(SearchTraceCallback callback);

protected:
  const Graph& original_graph_;
  detail::ConstantDegreeGraph transformed_;
  const Graph& graph_;
  std::vector<double> distances_;
  std::vector<std::size_t> path_vertices_;
  std::vector<Vertex> predecessors_;
  std::vector<bool> complete_;

  std::vector<std::uint32_t> working_marks_;
  std::vector<std::uint32_t> layer_marks_;
  std::vector<std::uint32_t> result_marks_;
  std::uint32_t working_mark_{0};
  std::uint32_t layer_mark_{0};
  std::uint32_t result_mark_{0};

  std::vector<Vertex> pivot_roots_;
  std::vector<std::size_t> pivot_tree_sizes_;
  std::vector<std::size_t> completed_levels_;
  std::vector<std::unique_ptr<detail::BlockQueue>> level_queues_;

  std::size_t k_;
  std::size_t t_;
  SolverOptions options_;
  SearchTraceCallback trace_callback_;

  void reset();
  void validate_query(Vertex source, Vertex goal) const;
  [[nodiscard]] PathResult solve_paper(Vertex source, std::optional<Vertex> goal);
  [[nodiscard]] virtual std::pair<detail::QueueKey, std::vector<Vertex>>
  bounded_search(std::size_t level, detail::QueueKey bound, std::vector<Vertex> sources,
                 std::optional<Vertex> goal);
  [[nodiscard]] std::pair<detail::QueueKey, std::vector<Vertex>>
  base_case(detail::QueueKey bound, const std::vector<Vertex>& sources, std::optional<Vertex> goal);
  [[nodiscard]] virtual std::pair<std::vector<Vertex>, std::vector<Vertex>>
  find_pivots(detail::QueueKey bound, const std::vector<Vertex>& sources);
  virtual void relax_completed(const std::vector<Vertex>& completed,
                               detail::QueueKey recursive_bound, detail::QueueKey pull_bound,
                               detail::QueueKey call_bound, detail::BlockQueue& data_structure,
                               std::vector<detail::QueueItem>& prepend);

  [[nodiscard]] detail::QueueKey key(Vertex vertex) const;
  [[nodiscard]] bool relax(Vertex from, const Edge& edge, detail::QueueKey& candidate);
  [[nodiscard]] bool candidate_valid(Vertex from, const Edge& edge,
                                     detail::QueueKey& candidate) const;
  [[nodiscard]] bool label_less(Vertex target, double distance, std::size_t vertices,
                                Vertex predecessor) const;
  [[nodiscard]] std::vector<Vertex> reconstruct_path(Vertex source, Vertex goal) const;
  [[nodiscard]] std::vector<double> original_distances() const;
  [[nodiscard]] std::size_t power_of_two_capped(std::size_t exponent) const;
  void append_unique(std::vector<Vertex>& destination, const std::vector<Vertex>& source,
                     std::uint32_t mark);
  void trace(SearchEventKind kind, Vertex vertex, double distance) const;
  void advance_mark(std::uint32_t& mark, std::vector<std::uint32_t>& marks);
};

} // namespace sssp
