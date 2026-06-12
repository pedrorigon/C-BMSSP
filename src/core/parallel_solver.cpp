#include "parallel_solver.h"

#include "block_queue.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <queue>
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

std::size_t capped_power_of_two(const std::size_t exponent) {
  return std::size_t{1} << std::min<std::size_t>(exponent, 20);
}
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

void ParallelSolver::relax_edges_parallel(
    const std::vector<Vertex>& completed, const double sub_bound, const double subset_bound,
    const double bound, detail::BlockQueue& data_structure,
    std::vector<std::pair<Vertex, double>>& prepend) {
  if (completed.size() < kParallelRelaxThreshold) {
    for (const Vertex vertex : completed) {
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
    return;
  }

#ifdef SSSP_HAS_OPENMP
  const auto thread_count = static_cast<std::size_t>(omp_get_max_threads());
#else
  const std::size_t thread_count = 1;
#endif

  std::vector<std::vector<Update>> thread_updates(thread_count);
  for (auto& updates : thread_updates) {
    updates.reserve(completed.size() * 4 / thread_count);
  }

#ifdef SSSP_HAS_OPENMP
#pragma omp parallel for schedule(dynamic, 64)
#endif
  for (std::size_t i = 0; i < completed.size(); ++i) {
#ifdef SSSP_HAS_OPENMP
    auto& updates = thread_updates[static_cast<std::size_t>(omp_get_thread_num())];
#else
    auto& updates = thread_updates.front();
#endif
    const Vertex vertex = completed[i];
    const double from_distance = distances_[vertex];
    for (const auto& edge : graph_.edges_from(vertex)) {
      const double candidate = from_distance + edge.weight;
      if (candidate < distances_[edge.to] && candidate < bound) {
        updates.push_back({edge.to, candidate, vertex});
      }
    }
  }

  std::vector<Update> all_updates;
  std::size_t total = 0;
  for (const auto& local : thread_updates) {
    total += local.size();
  }
  all_updates.reserve(total);
  for (auto& local : thread_updates) {
    all_updates.insert(all_updates.end(), std::make_move_iterator(local.begin()),
                       std::make_move_iterator(local.end()));
  }

  std::ranges::sort(all_updates, [](const Update& a, const Update& b) {
    if (a.vertex != b.vertex)
      return a.vertex < b.vertex;
    if (a.distance != b.distance)
      return a.distance < b.distance;
    return a.predecessor < b.predecessor;
  });

  for (std::size_t i = 0; i < all_updates.size();) {
    const Update& best = all_updates[i];
    if (best.distance < distances_[best.vertex]) {
      distances_[best.vertex] = best.distance;
      predecessors_[best.vertex] = best.predecessor;
      if (best.distance >= subset_bound && best.distance < bound) {
        data_structure.insert(best.vertex, best.distance);
      } else if (best.distance >= sub_bound && best.distance < subset_bound) {
        prepend.emplace_back(best.vertex, best.distance);
      }
    }
    const Vertex target = best.vertex;
    while (i < all_updates.size() && all_updates[i].vertex == target) {
      ++i;
    }
  }

  for (const Vertex vertex : completed) {
    complete_[vertex] = true;
  }
}

std::pair<double, std::vector<Vertex>>
ParallelSolver::bounded_search(const std::size_t level, const double bound,
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
    relax_edges_parallel(sub_result, sub_bound, subset_bound, bound, data_structure, prepend);

    for (const Vertex vertex : subset) {
      if (!complete_[vertex] && distances_[vertex] >= sub_bound &&
          distances_[vertex] < subset_bound) {
        prepend.emplace_back(vertex, distances_[vertex]);
      }
    }
    data_structure.batch_prepend(std::move(prepend));
  }

  for (const Vertex vertex : working_set) {
    if (!complete_[vertex] && distances_[vertex] < current_bound) {
      complete_[vertex] = true;
      result.push_back(vertex);
    }
  }
  return {std::min(current_bound, bound), std::move(result)};
}

void ParallelSolver::complete_shortest_paths(const std::optional<Vertex> goal) {
  const std::size_t vertex_count = graph_.vertex_count();
  if (vertex_count == 0) {
    return;
  }

  double weight_sum = 0.0;
  std::size_t edge_total = 0;
  for (Vertex v = 0; v < vertex_count; ++v) {
    for (const auto& edge : graph_.edges_from(v)) {
      weight_sum += edge.weight;
      ++edge_total;
    }
  }
  const double delta = edge_total > 0 ? weight_sum / static_cast<double>(edge_total) : 1.0;

  std::vector<std::vector<Vertex>> buckets;
  std::vector<std::size_t> bucket_of(vertex_count, static_cast<std::size_t>(-1));

  auto bucket_index = [delta](const double distance) {
    return static_cast<std::size_t>(std::floor(distance / delta));
  };
  auto ensure_bucket = [&buckets](const std::size_t index) {
    if (index >= buckets.size()) {
      buckets.resize(index + 1);
    }
  };
  auto place = [&](const Vertex vertex) {
    const std::size_t index = bucket_index(distances_[vertex]);
    ensure_bucket(index);
    buckets[index].push_back(vertex);
    bucket_of[vertex] = index;
  };

  for (Vertex vertex = 0; vertex < vertex_count; ++vertex) {
    if (distances_[vertex] != infinity) {
      place(vertex);
    }
  }

  std::fill(complete_.begin(), complete_.end(), false);

#ifdef SSSP_HAS_OPENMP
  const auto thread_count = static_cast<std::size_t>(omp_get_max_threads());
#else
  const std::size_t thread_count = 1;
#endif

  std::vector<std::vector<Update>> thread_updates(thread_count);
  std::vector<Update> merged;
  std::vector<Vertex> frontier;
  std::vector<Vertex> active;

  for (std::size_t current = 0; current < buckets.size(); ++current) {
    if (buckets[current].empty()) {
      continue;
    }

    std::vector<Vertex> removed;

    while (!buckets[current].empty()) {
      frontier.clear();
      frontier.swap(buckets[current]);

      active.clear();
      active.reserve(frontier.size());
      for (const Vertex vertex : frontier) {
        if (bucket_of[vertex] == current && !complete_[vertex]) {
          bucket_of[vertex] = static_cast<std::size_t>(-1);
          active.push_back(vertex);
          removed.push_back(vertex);
        }
      }
      if (active.empty()) {
        continue;
      }

      if (active.size() < kParallelRelaxThreshold) {
        for (const Vertex vertex : active) {
          const double base = distances_[vertex];
          for (const auto& edge : graph_.edges_from(vertex)) {
            const double candidate = base + edge.weight;
            if (candidate < distances_[edge.to]) {
              distances_[edge.to] = candidate;
              predecessors_[edge.to] = vertex;
              place(edge.to);
            }
          }
        }
      } else {
        for (auto& chunk : thread_updates) {
          chunk.clear();
        }

#ifdef SSSP_HAS_OPENMP
#pragma omp parallel for schedule(dynamic, 64)
#endif
        for (std::size_t i = 0; i < active.size(); ++i) {
#ifdef SSSP_HAS_OPENMP
          auto& sink = thread_updates[static_cast<std::size_t>(omp_get_thread_num())];
#else
          auto& sink = thread_updates.front();
#endif
          const Vertex vertex = active[i];
          const double base = distances_[vertex];
          for (const auto& edge : graph_.edges_from(vertex)) {
            const double candidate = base + edge.weight;
            if (candidate < distances_[edge.to]) {
              sink.push_back({edge.to, candidate, vertex});
            }
          }
        }

        merged.clear();
        for (auto& chunk : thread_updates) {
          merged.insert(merged.end(), chunk.begin(), chunk.end());
        }
        if (!merged.empty()) {
          std::ranges::sort(merged, [](const Update& a, const Update& b) {
            if (a.vertex != b.vertex)
              return a.vertex < b.vertex;
            if (a.distance != b.distance)
              return a.distance < b.distance;
            return a.predecessor < b.predecessor;
          });
          for (std::size_t i = 0; i < merged.size();) {
            const Update& best = merged[i];
            if (best.distance < distances_[best.vertex]) {
              distances_[best.vertex] = best.distance;
              predecessors_[best.vertex] = best.predecessor;
              place(best.vertex);
            }
            const Vertex target = best.vertex;
            while (i < merged.size() && merged[i].vertex == target) {
              ++i;
            }
          }
        }
      }
    }
    for (const Vertex vertex : removed) {
      complete_[vertex] = true;
      if (goal && vertex == *goal) {
        return;
      }
    }
  }
}

} // namespace sssp
