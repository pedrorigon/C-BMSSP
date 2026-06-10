#include "parallel_solver.h"

#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#ifdef SSSP_HAS_OPENMP
#include <omp.h>
#endif

namespace sssp {

namespace {
struct Update {
  Vertex vertex;
  double distance;
  Vertex predecessor;
};
} // namespace

ParallelSolver::ParallelSolver(const Graph& graph, const SolverOptions options)
    : SequentialSolver(graph, options) {}

std::pair<std::vector<Vertex>, std::vector<Vertex>>
ParallelSolver::find_pivots(const double bound, const std::vector<Vertex>& frontier) {
  std::unordered_set<Vertex> working_set(frontier.begin(), frontier.end());
  working_set.reserve(k_ * frontier.size());
  std::vector<Vertex> current_layer = frontier;

  for (std::size_t iteration = 0; iteration < k_ && !current_layer.empty(); ++iteration) {
#ifdef SSSP_HAS_OPENMP
    const int thread_count = omp_get_max_threads();
#else
    const int thread_count = 1;
#endif
    std::vector<std::vector<Update>> thread_updates(static_cast<std::size_t>(thread_count));
    const std::size_t updates_per_thread =
        std::max<std::size_t>(16, (current_layer.size() * 4) / thread_updates.size());
    for (auto& updates : thread_updates) {
      updates.reserve(updates_per_thread);
    }

#ifdef SSSP_HAS_OPENMP
#pragma omp parallel for schedule(dynamic, 64) if (current_layer.size() >= 256)
#endif
    for (std::size_t index = 0; index < current_layer.size(); ++index) {
#ifdef SSSP_HAS_OPENMP
      auto& updates = thread_updates[static_cast<std::size_t>(omp_get_thread_num())];
#else
      auto& updates = thread_updates.front();
#endif
      const Vertex from = current_layer[index];
      const double from_distance = distances_[from];
      for (const auto& edge : graph_.edges_from(from)) {
        const double candidate = from_distance + edge.weight;
        if (candidate < distances_[edge.to] && candidate < bound) {
          updates.push_back({edge.to, candidate, from});
        }
      }
    }

    std::vector<Update> updates;
    std::size_t update_count = 0;
    for (const auto& local : thread_updates) {
      update_count += local.size();
    }
    updates.reserve(update_count);
    for (auto& local : thread_updates) {
      updates.insert(updates.end(), std::make_move_iterator(local.begin()),
                     std::make_move_iterator(local.end()));
    }
    std::ranges::sort(updates, [](const Update& left, const Update& right) {
      if (left.vertex != right.vertex) {
        return left.vertex < right.vertex;
      }
      return left.distance < right.distance;
    });

    std::unordered_set<Vertex> next_layer;
    next_layer.reserve(current_layer.size() * 2);
    for (std::size_t index = 0; index < updates.size();) {
      const Update& best = updates[index];
      if (best.distance < distances_[best.vertex]) {
        distances_[best.vertex] = best.distance;
        predecessors_[best.vertex] = best.predecessor;
        if (working_set.insert(best.vertex).second) {
          next_layer.insert(best.vertex);
        }
      }
      const Vertex vertex = best.vertex;
      while (index < updates.size() && updates[index].vertex == vertex) {
        ++index;
      }
    }

    if (working_set.size() > k_ * frontier.size()) {
      return {frontier, {working_set.begin(), working_set.end()}};
    }
    current_layer.assign(next_layer.begin(), next_layer.end());
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

} // namespace sssp
