#include "sequential_solver.h"

#include "work_counter.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>

namespace sssp {

namespace {

using detail::QueueItem;
using detail::QueueKey;

struct MinQueueCompare {
  bool operator()(const QueueItem& left, const QueueItem& right) const {
    return right.key < left.key;
  }
};

} // namespace

SequentialSolver::SequentialSolver(const Graph& graph, const SolverOptions options)
    : original_graph_(graph), transformed_(detail::make_constant_degree_graph(graph)),
      graph_(transformed_.graph), distances_(graph_.vertex_count(), infinity),
      path_vertices_(graph_.vertex_count(), std::numeric_limits<std::size_t>::max()),
      predecessors_(graph_.vertex_count(), graph_.vertex_count()),
      complete_(graph_.vertex_count(), false), working_marks_(graph_.vertex_count(), 0),
      layer_marks_(graph_.vertex_count(), 0), result_marks_(graph_.vertex_count(), 0),
      pivot_roots_(graph_.vertex_count(), graph_.vertex_count()),
      pivot_tree_sizes_(graph_.vertex_count(), 0),
      completed_levels_(graph_.vertex_count(), std::numeric_limits<std::size_t>::max()),
      options_(options) {
  const double log_n =
      graph_.vertex_count() > 1 ? std::log2(static_cast<double>(graph_.vertex_count())) : 1.0;
  k_ = std::max<std::size_t>(1, static_cast<std::size_t>(std::floor(std::cbrt(log_n))));
  t_ = std::max<std::size_t>(1, static_cast<std::size_t>(std::floor(std::pow(log_n, 2.0 / 3.0))));
}

void SequentialSolver::set_trace_callback(SearchTraceCallback callback) {
  trace_callback_ = std::move(callback);
}

void SequentialSolver::trace(const SearchEventKind kind, const Vertex vertex,
                             const double distance) const {
  if (trace_callback_) {
    trace_callback_({kind, transformed_.original_vertex.at(vertex), distance});
  }
}

void SequentialSolver::advance_mark(std::uint32_t& mark, std::vector<std::uint32_t>& marks) {
  if (mark == std::numeric_limits<std::uint32_t>::max()) {
    std::fill(marks.begin(), marks.end(), 0);
    mark = 0;
  }
  ++mark;
}

void SequentialSolver::reset() {
  std::fill(distances_.begin(), distances_.end(), infinity);
  std::fill(path_vertices_.begin(), path_vertices_.end(), std::numeric_limits<std::size_t>::max());
  std::fill(predecessors_.begin(), predecessors_.end(), graph_.vertex_count());
  std::fill(complete_.begin(), complete_.end(), false);
  std::fill(completed_levels_.begin(), completed_levels_.end(),
            std::numeric_limits<std::size_t>::max());
}

void SequentialSolver::validate_query(const Vertex source, const Vertex goal) const {
  if (source >= original_graph_.vertex_count() || goal >= original_graph_.vertex_count()) {
    throw std::out_of_range("query vertex is outside graph");
  }
}

QueueKey SequentialSolver::key(const Vertex vertex) const {
  if (distances_[vertex] == infinity) {
    return QueueKey::infinity();
  }
  return {distances_[vertex], path_vertices_[vertex], vertex};
}

bool SequentialSolver::label_less(const Vertex target, const double distance,
                                  const std::size_t vertices, const Vertex predecessor) const {
  if (distance != distances_[target]) {
    return distance < distances_[target];
  }
  if (vertices != path_vertices_[target]) {
    return vertices < path_vertices_[target];
  }
  return predecessor < predecessors_[target];
}

bool SequentialSolver::candidate_valid(const Vertex from, const Edge& edge,
                                       QueueKey& candidate) const {
  if (distances_[from] == infinity) {
    return false;
  }
  const double distance = distances_[from] + edge.weight;
  const std::size_t vertices = path_vertices_[from] + 1;
  candidate = {distance, vertices, edge.to};
  if (label_less(edge.to, distance, vertices, from)) {
    return true;
  }
  return distance == distances_[edge.to] && vertices == path_vertices_[edge.to] &&
         from == predecessors_[edge.to];
}

bool SequentialSolver::relax(const Vertex from, const Edge& edge, QueueKey& candidate) {
  if (!candidate_valid(from, edge, candidate)) {
    return false;
  }
  if (label_less(edge.to, candidate.distance, candidate.vertices, from)) {
    distances_[edge.to] = candidate.distance;
    path_vertices_[edge.to] = candidate.vertices;
    predecessors_[edge.to] = from;
    trace(SearchEventKind::relaxation, edge.to, candidate.distance);
  }
  return true;
}

std::size_t SequentialSolver::power_of_two_capped(const std::size_t exponent) const {
  const std::size_t cap = std::max<std::size_t>(graph_.vertex_count(), 1);
  if (exponent >= std::numeric_limits<std::size_t>::digits || (std::size_t{1} << exponent) >= cap) {
    return cap;
  }
  return std::size_t{1} << exponent;
}

void SequentialSolver::append_unique(std::vector<Vertex>& destination,
                                     const std::vector<Vertex>& source, const std::uint32_t mark) {
  for (const Vertex vertex : source) {
    if (result_marks_[vertex] != mark) {
      result_marks_[vertex] = mark;
      destination.push_back(vertex);
    }
  }
}

PathResult SequentialSolver::solve(const Vertex source, const Vertex goal) {
  validate_query(source, goal);
  return solve_paper(source, goal);
}

std::vector<double> SequentialSolver::solve_all(const Vertex source) {
  if (source >= original_graph_.vertex_count()) {
    throw std::out_of_range("query vertex is outside graph");
  }
  static_cast<void>(solve_paper(source, std::nullopt));
  return original_distances();
}

PathResult SequentialSolver::solve_paper(const Vertex source, const std::optional<Vertex> goal) {
  reset();
  distances_[source] = 0.0;
  path_vertices_[source] = 1;
  complete_[source] = true;
  trace(SearchEventKind::settled, source, 0.0);

  const std::size_t max_level = static_cast<std::size_t>(
      std::ceil(std::log2(static_cast<double>(std::max<std::size_t>(graph_.vertex_count(), 2))) /
                static_cast<double>(t_)));
  level_queues_.reserve(max_level + 1);
  for (std::size_t level = level_queues_.size(); level <= max_level; ++level) {
    level_queues_.push_back(std::make_unique<detail::BlockQueue>(graph_.vertex_count()));
  }
  static_cast<void>(bounded_search(max_level, QueueKey::infinity(), {source}, goal));

  if (!goal) {
    return {};
  }
  if (distances_[*goal] == infinity) {
    return {};
  }
  return {distances_[*goal], reconstruct_path(source, *goal)};
}

std::vector<double> SequentialSolver::original_distances() const {
  return {distances_.begin(),
          distances_.begin() + static_cast<std::ptrdiff_t>(original_graph_.vertex_count())};
}

std::pair<QueueKey, std::vector<Vertex>>
SequentialSolver::base_case(const QueueKey bound, const std::vector<Vertex>& sources,
                            const std::optional<Vertex> goal) {
  if (sources.empty()) {
    return {bound, {}};
  }
  if (sources.size() != 1) {
    throw std::logic_error("BMSSP level zero requires a singleton source set");
  }

  std::priority_queue<QueueItem, std::vector<QueueItem>, MinQueueCompare> queue;
  queue.push({sources.front(), key(sources.front())});
  std::vector<Vertex> settled;
  settled.reserve(k_ + 1);

  advance_mark(layer_mark_, layer_marks_);
  while (!queue.empty() && settled.size() < k_ + 1) {
    // The bounded base case is a heap-based mini-Dijkstra: O(log) per extract.
    WorkCounter::add_log(queue.size());
    const QueueItem item = queue.top();
    queue.pop();
    if (item.key != key(item.vertex) || layer_marks_[item.vertex] == layer_mark_) {
      continue;
    }

    layer_marks_[item.vertex] = layer_mark_;
    complete_[item.vertex] = true;
    settled.push_back(item.vertex);
    trace(SearchEventKind::settled, item.vertex, item.key.distance);
    if (goal && item.vertex == *goal) {
      return {bound, std::move(settled)};
    }

    for (const Edge& edge : graph_.edges_from(item.vertex)) {
      QueueKey candidate{};
      if (relax(item.vertex, edge, candidate) && candidate < bound) {
        WorkCounter::add_log(queue.size());
        queue.push({edge.to, key(edge.to)});
      }
    }
  }

  if (settled.size() <= k_) {
    return {bound, std::move(settled)};
  }

  QueueKey boundary = key(settled.front());
  for (const Vertex vertex : settled) {
    boundary = std::max(boundary, key(vertex));
  }
  std::vector<Vertex> result;
  result.reserve(k_);
  for (const Vertex vertex : settled) {
    if (key(vertex) < boundary) {
      result.push_back(vertex);
    } else {
      complete_[vertex] = false;
    }
  }
  return {boundary, std::move(result)};
}

std::pair<std::vector<Vertex>, std::vector<Vertex>>
SequentialSolver::find_pivots(const QueueKey bound, const std::vector<Vertex>& sources) {
  advance_mark(working_mark_, working_marks_);
  std::vector<Vertex> working_set;
  working_set.reserve(std::min(graph_.vertex_count(), k_ * sources.size() + 2));
  for (const Vertex vertex : sources) {
    if (working_marks_[vertex] != working_mark_) {
      working_marks_[vertex] = working_mark_;
      working_set.push_back(vertex);
    }
    pivot_roots_[vertex] = vertex;
    pivot_tree_sizes_[vertex] = 0;
  }

  std::vector<Vertex> current_layer = sources;
  for (std::size_t iteration = 0; iteration < k_ && !current_layer.empty(); ++iteration) {
    advance_mark(layer_mark_, layer_marks_);
    std::vector<Vertex> next_layer;
    next_layer.reserve(current_layer.size() * 2);
    for (const Vertex from : current_layer) {
      trace(SearchEventKind::frontier, from, distances_[from]);
      // FindPivots performs k rounds of edge relaxation; over a recursion depth
      // its disjoint completed sets sum to the O(nk) term of the analysis.
      WorkCounter::add(static_cast<double>(graph_.edges_from(from).size()));
      for (const Edge& edge : graph_.edges_from(from)) {
        QueueKey candidate{};
        if (!relax(from, edge, candidate) || !(candidate < bound)) {
          continue;
        }
        pivot_roots_[edge.to] = pivot_roots_[from];
        if (layer_marks_[edge.to] != layer_mark_) {
          layer_marks_[edge.to] = layer_mark_;
          next_layer.push_back(edge.to);
        }
        if (working_marks_[edge.to] != working_mark_) {
          working_marks_[edge.to] = working_mark_;
          working_set.push_back(edge.to);
        }
      }
    }

    if (working_set.size() > k_ * sources.size()) {
      return {sources, std::move(working_set)};
    }
    current_layer = std::move(next_layer);
  }

  for (const Vertex vertex : working_set) {
    const Vertex root = pivot_roots_[vertex];
    if (root < graph_.vertex_count()) {
      ++pivot_tree_sizes_[root];
    }
  }

  std::vector<Vertex> pivots;
  for (const Vertex vertex : sources) {
    if (pivot_tree_sizes_[vertex] >= k_) {
      pivots.push_back(vertex);
    }
    pivot_tree_sizes_[vertex] = 0;
  }
  return {std::move(pivots), std::move(working_set)};
}

void SequentialSolver::relax_completed(const std::vector<Vertex>& completed,
                                       const QueueKey recursive_bound, const QueueKey pull_bound,
                                       const QueueKey call_bound,
                                       detail::BlockQueue& data_structure,
                                       std::vector<QueueItem>& prepend) {
  for (const Vertex from : completed) {
    complete_[from] = true;
    trace(SearchEventKind::completion, from, distances_[from]);
    for (const Edge& edge : graph_.edges_from(from)) {
      QueueKey candidate{};
      if (!relax(from, edge, candidate)) {
        continue;
      }
      const QueueKey current = key(edge.to);
      if (!(current < call_bound)) {
        continue;
      }
      if (!(current < pull_bound)) {
        data_structure.insert(edge.to, current);
      } else if (!(current < recursive_bound)) {
        prepend.push_back({edge.to, current});
      }
    }
  }
}

std::pair<QueueKey, std::vector<Vertex>>
SequentialSolver::bounded_search(const std::size_t level, const QueueKey bound,
                                 std::vector<Vertex> sources, const std::optional<Vertex> goal) {
  if (level == 0) {
    return base_case(bound, sources, goal);
  }
  if (goal && complete_[*goal]) {
    return {bound, {}};
  }

  auto [pivots, working_set] = find_pivots(bound, sources);
  const std::size_t block_size = power_of_two_capped((level - 1) * t_);
  detail::BlockQueue& data_structure = *level_queues_.at(level);
  data_structure.initialize(block_size, bound);
  for (const Vertex pivot : pivots) {
    data_structure.insert(pivot, key(pivot));
  }

  QueueKey achieved_bound = pivots.empty() ? bound : key(pivots.front());
  for (const Vertex pivot : pivots) {
    achieved_bound = std::min(achieved_bound, key(pivot));
  }

  advance_mark(result_mark_, result_marks_);
  const std::uint32_t call_result_mark = result_mark_;
  std::vector<Vertex> result;
  const std::size_t level_scale = power_of_two_capped(level * t_);
  const std::size_t max_result = level_scale > graph_.vertex_count() / std::max<std::size_t>(k_, 1)
                                     ? graph_.vertex_count()
                                     : k_ * level_scale;
  result.reserve(std::min(max_result, working_set.size() + block_size));

  while (result.size() < max_result && !data_structure.empty()) {
    if (goal && complete_[*goal]) {
      break;
    }
    auto [pull_bound, subset] = data_structure.pull();
    if (subset.empty()) {
      break;
    }

    auto [recursive_bound, sub_result] = bounded_search(level - 1, pull_bound, subset, goal);
    achieved_bound = recursive_bound;
    append_unique(result, sub_result, call_result_mark);

    std::vector<QueueItem> prepend;
    prepend.reserve(sub_result.size() * 2 + subset.size());
    for (const Vertex vertex : sub_result) {
      data_structure.erase(vertex);
      completed_levels_[vertex] = level;
    }
    relax_completed(sub_result, recursive_bound, pull_bound, bound, data_structure, prepend);
    for (const Vertex vertex : subset) {
      const QueueKey current = key(vertex);
      if (!(current < recursive_bound) && current < pull_bound) {
        prepend.push_back({vertex, current});
      }
    }
    data_structure.batch_prepend(std::move(prepend));
  }

  achieved_bound = data_structure.empty() ? bound : std::min(achieved_bound, bound);
  for (const Vertex vertex : working_set) {
    if (completed_levels_[vertex] != level && key(vertex) < achieved_bound) {
      complete_[vertex] = true;
      if (result_marks_[vertex] != call_result_mark) {
        result_marks_[vertex] = call_result_mark;
        result.push_back(vertex);
      }
    }
  }
  return {achieved_bound, std::move(result)};
}

std::vector<Vertex> SequentialSolver::reconstruct_path(const Vertex source,
                                                       const Vertex goal) const {
  std::vector<Vertex> transformed_path;
  for (Vertex current = goal;; current = predecessors_[current]) {
    transformed_path.push_back(current);
    if (current == source) {
      break;
    }
    if (predecessors_[current] >= graph_.vertex_count()) {
      return {};
    }
  }
  std::ranges::reverse(transformed_path);

  std::vector<Vertex> result;
  result.reserve(transformed_path.size());
  for (const Vertex transformed_vertex : transformed_path) {
    const Vertex original = transformed_.original_vertex[transformed_vertex];
    if (result.empty() || result.back() != original) {
      result.push_back(original);
    }
  }
  return result;
}

} // namespace sssp
