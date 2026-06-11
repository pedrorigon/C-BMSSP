#include "sequential_solver.h"

#include "block_queue.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace sssp {

namespace {
using QueueEntry = std::pair<double, Vertex>;

std::size_t capped_power_of_two(const std::size_t exponent) {
  return std::size_t{1} << std::min<std::size_t>(exponent, 20);
}
} // namespace

SequentialSolver::SequentialSolver(const Graph& graph, const SolverOptions options)
    : graph_(graph), distances_(graph.vertex_count(), infinity),
      predecessors_(graph.vertex_count(), graph.vertex_count()),
      complete_(graph.vertex_count(), false), options_(options) {
  const double log_n =
      graph.vertex_count() > 1 ? std::log(static_cast<double>(graph.vertex_count())) : 0.0;
  k_ = std::max<std::size_t>(3, static_cast<std::size_t>(std::floor(std::cbrt(log_n) * 2.0)));
  t_ = std::max<std::size_t>(2, static_cast<std::size_t>(std::floor(std::pow(log_n, 2.0 / 3.0))));
}

PathResult SequentialSolver::solve(const Vertex source, const Vertex goal) {
  validate_query(source, goal);
  if (graph_.vertex_count() < options_.minimum_optimized_vertices ||
      graph_.edge_count() < options_.minimum_optimized_edges) {
    return solve_small_graph(source, goal);
  }
  return solve_optimized(source, goal);
}

std::vector<double> SequentialSolver::solve_all(const Vertex source) {
  if (source >= graph_.vertex_count()) {
    throw std::out_of_range("query vertex is outside graph");
  }

  // Small graphs: a plain Dijkstra over the whole graph. Larger graphs use the
  // BMSSP machinery with no goal so the full shortest-path tree is built.
  if (graph_.vertex_count() < options_.minimum_optimized_vertices ||
      graph_.edge_count() < options_.minimum_optimized_edges) {
    reset();
    distances_[source] = 0.0;
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<>> queue;
    queue.emplace(0.0, source);
    while (!queue.empty()) {
      const auto [distance, vertex] = queue.top();
      queue.pop();
      if (distance > distances_[vertex]) {
        continue;
      }
      for (const auto& edge : graph_.edges_from(vertex)) {
        const double candidate = distance + edge.weight;
        if (candidate < distances_[edge.to]) {
          distances_[edge.to] = candidate;
          predecessors_[edge.to] = vertex;
          queue.emplace(candidate, edge.to);
        }
      }
    }
    return distances_;
  }

  reset();
  distances_[source] = 0.0;
  const auto max_level = static_cast<std::size_t>(
      std::ceil(std::log(static_cast<double>(graph_.vertex_count())) / static_cast<double>(t_)));
  static_cast<void>(bounded_search(max_level, infinity, {source}, std::nullopt));
  complete_shortest_paths(std::nullopt);
  return distances_;
}

void SequentialSolver::reset() {
  std::fill(distances_.begin(), distances_.end(), infinity);
  std::fill(predecessors_.begin(), predecessors_.end(), graph_.vertex_count());
  std::fill(complete_.begin(), complete_.end(), false);
}

void SequentialSolver::validate_query(const Vertex source, const Vertex goal) const {
  if (source >= graph_.vertex_count() || goal >= graph_.vertex_count()) {
    throw std::out_of_range("query vertex is outside graph");
  }
}

PathResult SequentialSolver::solve_small_graph(const Vertex source, const Vertex goal) {
  reset();
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<>> queue;
  distances_[source] = 0.0;
  queue.emplace(0.0, source);

  while (!queue.empty()) {
    const auto [distance, vertex] = queue.top();
    queue.pop();
    if (distance > distances_[vertex]) {
      continue;
    }
    if (vertex == goal) {
      return {distance, reconstruct_path(source, goal)};
    }
    for (const auto& edge : graph_.edges_from(vertex)) {
      const double candidate = distance + edge.weight;
      if (candidate < distances_[edge.to]) {
        distances_[edge.to] = candidate;
        predecessors_[edge.to] = vertex;
        queue.emplace(candidate, edge.to);
      }
    }
  }
  return {};
}

void SequentialSolver::complete_shortest_paths(const std::optional<Vertex> goal) {
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<>> queue;
  for (Vertex vertex = 0; vertex < graph_.vertex_count(); ++vertex) {
    if (distances_[vertex] != infinity) {
      queue.emplace(distances_[vertex], vertex);
    }
  }

  std::fill(complete_.begin(), complete_.end(), false);
  while (!queue.empty()) {
    const auto [distance, vertex] = queue.top();
    queue.pop();
    if (complete_[vertex] || distance > distances_[vertex]) {
      continue;
    }
    complete_[vertex] = true;
    if (goal && vertex == *goal) {
      return;
    }
    for (const auto& edge : graph_.edges_from(vertex)) {
      const double candidate = distance + edge.weight;
      if (candidate < distances_[edge.to]) {
        distances_[edge.to] = candidate;
        predecessors_[edge.to] = vertex;
        queue.emplace(candidate, edge.to);
      }
    }
  }
}

PathResult SequentialSolver::solve_optimized(const Vertex source, const Vertex goal) {
  reset();
  distances_[source] = 0.0;
  const auto max_level = static_cast<std::size_t>(
      std::ceil(std::log(static_cast<double>(graph_.vertex_count())) / static_cast<double>(t_)));
  static_cast<void>(bounded_search(max_level, infinity, {source}, goal));
  complete_shortest_paths(goal);
  return distances_[goal] == infinity
             ? PathResult{}
             : PathResult{distances_[goal], reconstruct_path(source, goal)};
}

std::pair<double, std::vector<Vertex>>
SequentialSolver::bounded_search(const std::size_t level, const double bound,
                                 std::vector<Vertex> pivots, const std::optional<Vertex> goal) {
  if (level == 0) {
    return base_case(bound, pivots, goal);
  }
  if (goal && complete_[*goal]) {
    return {bound, {}};
  }

  auto [selected_pivots, working_set] = find_pivots(bound, pivots);
  pivots = std::move(selected_pivots);

  detail::BlockQueue data_structure(capped_power_of_two((level - 1) * t_), bound);
  double current_bound = infinity;
  for (const Vertex pivot : pivots) {
    if (distances_[pivot] != infinity) {
      data_structure.insert(pivot, distances_[pivot]);
      current_bound = std::min(current_bound, distances_[pivot]);
    }
  }

  // Algorithm 3 (BMSSP), recursive case. We pull a block (B_i, S_i) from D,
  // recurse to settle U_i, then relax U_i's edges into the two distance bands:
  //   [B_i, B)      -> Insert into D (handled in the current sub-problem later),
  //   [B'_i, B_i)   -> BatchPrepend (pulled before the current block's range).
  // Crucially, vertices of S_i that the sub-call did NOT complete (their distance
  // fell back into [B'_i, B_i)) are batch-prepended too, so the search keeps
  // expanding instead of stalling.
  std::vector<Vertex> result;
  const std::size_t max_result_size = k_ * capped_power_of_two(level * t_);
  while (result.size() < max_result_size && !data_structure.empty()) {
    if (goal && complete_[*goal]) {
      break;
    }
    auto [subset_bound, subset] = data_structure.pull();
    if (subset.empty()) {
      break;
    }

    auto [sub_bound, sub_result] =
        bounded_search(level - 1, subset_bound, std::vector<Vertex>(subset), goal);
    result.insert(result.end(), sub_result.begin(), sub_result.end());
    current_bound = std::min(current_bound, sub_bound);

    std::vector<std::pair<Vertex, double>> prepend;
    for (const Vertex vertex : sub_result) {
      complete_[vertex] = true;
      for (const auto& edge : graph_.edges_from(vertex)) {
        const double candidate = distances_[vertex] + edge.weight;
        if (candidate < distances_[edge.to]) {
          distances_[edge.to] = candidate;
          predecessors_[edge.to] = vertex;
          if (candidate >= subset_bound && candidate < bound) {
            data_structure.insert(edge.to, candidate);
          } else if (candidate >= sub_bound && candidate < subset_bound) {
            prepend.emplace_back(edge.to, candidate);
          }
        }
      }
    }
    // Re-queue the S_i vertices the sub-call left incomplete in [B'_i, B_i).
    for (const Vertex vertex : subset) {
      if (!complete_[vertex] && distances_[vertex] >= sub_bound &&
          distances_[vertex] < subset_bound) {
        prepend.emplace_back(vertex, distances_[vertex]);
      }
    }
    data_structure.batch_prepend(std::move(prepend));
  }

  // Vertices reached within the FindPivots expansion that are already below the
  // achieved boundary are complete too (Algorithm 3, line 6).
  for (const Vertex vertex : working_set) {
    if (!complete_[vertex] && distances_[vertex] < current_bound) {
      complete_[vertex] = true;
      result.push_back(vertex);
    }
  }
  return {std::min(current_bound, bound), std::move(result)};
}

std::pair<double, std::vector<Vertex>>
SequentialSolver::base_case(const double bound, const std::vector<Vertex>& frontier,
                            const std::optional<Vertex> goal) {
  if (frontier.empty()) {
    return {bound, {}};
  }

  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<>> queue;
  for (const Vertex start : frontier) {
    if (distances_[start] < bound) {
      queue.emplace(distances_[start], start);
    }
  }

  // Algorithm 2 (BaseCase): a bounded Dijkstra that settles up to k+1 closest
  // vertices. If it settles at most k, the whole reachable-within-B set fit, so
  // the boundary stays B; otherwise the boundary tightens to the largest settled
  // distance and only vertices strictly below it count as complete.
  std::vector<Vertex> settled;
  const std::size_t limit = k_ + 1;
  while (!queue.empty() && settled.size() < limit) {
    const auto [distance, vertex] = queue.top();
    queue.pop();
    if (distance > distances_[vertex]) {
      continue;
    }
    complete_[vertex] = true;
    settled.push_back(vertex);
    if (goal && vertex == *goal) {
      return {bound, std::move(settled)};
    }
    for (const auto& edge : graph_.edges_from(vertex)) {
      const double candidate = distance + edge.weight;
      if (candidate < distances_[edge.to] && candidate < bound) {
        distances_[edge.to] = candidate;
        predecessors_[edge.to] = vertex;
        queue.emplace(candidate, edge.to);
      }
    }
  }

  if (settled.size() <= k_) {
    return {bound, std::move(settled)};
  }
  // Boundary tightens: keep only vertices strictly below the largest distance,
  // marking the rest incomplete so the caller re-queues them.
  double boundary = 0.0;
  for (const Vertex vertex : settled) {
    boundary = std::max(boundary, distances_[vertex]);
  }
  std::vector<Vertex> result;
  result.reserve(settled.size());
  for (const Vertex vertex : settled) {
    if (distances_[vertex] < boundary) {
      result.push_back(vertex);
    } else {
      complete_[vertex] = false;
    }
  }
  return {boundary, std::move(result)};
}

std::pair<std::vector<Vertex>, std::vector<Vertex>>
SequentialSolver::find_pivots(const double bound, const std::vector<Vertex>& frontier) {
  std::unordered_set<Vertex> working_set(frontier.begin(), frontier.end());
  working_set.reserve(k_ * frontier.size());
  std::vector<Vertex> current_layer = frontier;

  for (std::size_t iteration = 0; iteration < k_; ++iteration) {
    std::unordered_set<Vertex> next_layer;
    next_layer.reserve(current_layer.size() * 2);
    for (const Vertex vertex : current_layer) {
      for (const auto& edge : graph_.edges_from(vertex)) {
        const double candidate = distances_[vertex] + edge.weight;
        if (candidate < distances_[edge.to] && candidate < bound) {
          distances_[edge.to] = candidate;
          predecessors_[edge.to] = vertex;
          if (working_set.insert(edge.to).second) {
            next_layer.insert(edge.to);
          }
        }
      }
    }
    if (next_layer.empty()) {
      break;
    }
    current_layer.assign(next_layer.begin(), next_layer.end());
    if (working_set.size() > k_ * frontier.size()) {
      return {frontier, {working_set.begin(), working_set.end()}};
    }
  }

  std::unordered_map<Vertex, std::size_t> subtree_sizes;
  for (const Vertex vertex : working_set) {
    const Vertex predecessor = predecessors_[vertex];
    if (predecessor != graph_.vertex_count()) {
      ++subtree_sizes[predecessor];
    }
  }

  std::vector<Vertex> pivots;
  for (const Vertex vertex : frontier) {
    if (subtree_sizes[vertex] >= k_) {
      pivots.push_back(vertex);
    }
  }
  if (pivots.empty()) {
    pivots = frontier;
  }
  return {std::move(pivots), {working_set.begin(), working_set.end()}};
}

std::vector<Vertex> SequentialSolver::reconstruct_path(const Vertex source,
                                                       const Vertex goal) const {
  std::vector<Vertex> path;
  for (Vertex current = goal;; current = predecessors_[current]) {
    path.push_back(current);
    if (current == source) {
      break;
    }
    if (predecessors_[current] == graph_.vertex_count()) {
      return {};
    }
  }
  std::reverse(path.begin(), path.end());
  return path;
}

} // namespace sssp
