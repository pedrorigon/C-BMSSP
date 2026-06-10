#include "parallel_dijkstra_solver.h"

#include <algorithm>
#include <functional>
#include <queue>
#include <stdexcept>
#include <utility>

#ifdef VALIDATION_HAS_OPENMP
#include <omp.h>
#endif

namespace validation {

namespace {

using QueueEntry = std::pair<double, sssp::Vertex>;

struct Candidate {
  sssp::Vertex target;
  double distance;
  sssp::Vertex predecessor;
};

bool candidate_less(const Candidate& left, const Candidate& right) {
  if (left.target != right.target) {
    return left.target < right.target;
  }
  if (left.distance != right.distance) {
    return left.distance < right.distance;
  }
  return left.predecessor < right.predecessor;
}

} // namespace

ParallelDijkstraSolver::ParallelDijkstraSolver(const sssp::Graph& graph)
    : graph_(graph), distances_(graph.vertex_count(), sssp::infinity),
      predecessors_(graph.vertex_count(), graph.vertex_count()),
      complete_(graph.vertex_count(), false) {}

void ParallelDijkstraSolver::reset() {
  std::fill(distances_.begin(), distances_.end(), sssp::infinity);
  std::fill(predecessors_.begin(), predecessors_.end(), graph_.vertex_count());
  std::fill(complete_.begin(), complete_.end(), false);
}

void ParallelDijkstraSolver::validate_vertex(const sssp::Vertex vertex) const {
  if (vertex >= graph_.vertex_count()) {
    throw std::out_of_range("query vertex is outside graph");
  }
}

sssp::PathResult ParallelDijkstraSolver::solve(const sssp::Vertex source, const sssp::Vertex goal) {
  validate_vertex(source);
  validate_vertex(goal);
  run(source, &goal);
  return distances_[goal] == sssp::infinity
             ? sssp::PathResult{}
             : sssp::PathResult{distances_[goal], reconstruct_path(source, goal)};
}

std::vector<double> ParallelDijkstraSolver::solve_all(const sssp::Vertex source) {
  validate_vertex(source);
  run(source, nullptr);
  return distances_;
}

void ParallelDijkstraSolver::run(const sssp::Vertex source, const sssp::Vertex* const goal) {
  reset();
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<>> queue;
  distances_[source] = 0.0;
  queue.emplace(0.0, source);

  while (!queue.empty()) {
    const double batch_distance = queue.top().first;
    std::vector<sssp::Vertex> batch;
    while (!queue.empty() && queue.top().first == batch_distance) {
      const auto [distance, vertex] = queue.top();
      queue.pop();
      if (!complete_[vertex] && distance == distances_[vertex]) {
        batch.push_back(vertex);
      }
    }
    if (batch.empty()) {
      continue;
    }

    std::ranges::sort(batch);
    batch.erase(std::unique(batch.begin(), batch.end()), batch.end());

    const bool has_zero_weight_edge = std::ranges::any_of(batch, [this](const sssp::Vertex vertex) {
      const auto& edges = graph_.edges_from(vertex);
      return std::ranges::any_of(edges, [](const auto& edge) { return edge.weight == 0.0; });
    });
    if (batch.size() > 1 && has_zero_weight_edge) {
      for (std::size_t index = 1; index < batch.size(); ++index) {
        queue.emplace(batch_distance, batch[index]);
      }
      batch.resize(1);
    }

    for (const sssp::Vertex vertex : batch) {
      complete_[vertex] = true;
    }
    if (goal != nullptr && std::binary_search(batch.begin(), batch.end(), *goal)) {
      return;
    }

    std::vector<Candidate> candidates;
    if (batch.size() == 1) {
      const sssp::Vertex predecessor = batch.front();
      const auto& edges = graph_.edges_from(predecessor);
      std::vector<Candidate> slots(edges.size(), {graph_.vertex_count(), 0.0, predecessor});

#ifdef VALIDATION_HAS_OPENMP
#pragma omp parallel for schedule(static) if (edges.size() >= 256)
#endif
      for (std::size_t index = 0; index < edges.size(); ++index) {
        const auto& edge = edges[index];
        const double candidate = batch_distance + edge.weight;
        if (candidate < distances_[edge.to]) {
          slots[index] = {edge.to, candidate, predecessor};
        }
      }

      candidates.reserve(slots.size());
      for (const Candidate& candidate : slots) {
        if (candidate.target != graph_.vertex_count()) {
          candidates.push_back(candidate);
        }
      }
    } else {
#ifdef VALIDATION_HAS_OPENMP
      const int thread_count = omp_get_max_threads();
#else
      const int thread_count = 1;
#endif
      std::vector<std::vector<Candidate>> thread_candidates(static_cast<std::size_t>(thread_count));

#ifdef VALIDATION_HAS_OPENMP
#pragma omp parallel for schedule(dynamic, 16) if (batch.size() >= 64)
#endif
      for (std::size_t index = 0; index < batch.size(); ++index) {
#ifdef VALIDATION_HAS_OPENMP
        auto& local = thread_candidates[static_cast<std::size_t>(omp_get_thread_num())];
#else
        auto& local = thread_candidates.front();
#endif
        const sssp::Vertex predecessor = batch[index];
        for (const auto& edge : graph_.edges_from(predecessor)) {
          const double candidate = batch_distance + edge.weight;
          if (candidate < distances_[edge.to]) {
            local.push_back({edge.to, candidate, predecessor});
          }
        }
      }

      std::size_t candidate_count = 0;
      for (const auto& local : thread_candidates) {
        candidate_count += local.size();
      }
      candidates.reserve(candidate_count);
      for (auto& local : thread_candidates) {
        candidates.insert(candidates.end(), std::make_move_iterator(local.begin()),
                          std::make_move_iterator(local.end()));
      }
    }

    std::ranges::sort(candidates, candidate_less);
    for (std::size_t index = 0; index < candidates.size();) {
      const Candidate& best = candidates[index];
      if (best.distance < distances_[best.target]) {
        distances_[best.target] = best.distance;
        predecessors_[best.target] = best.predecessor;
        queue.emplace(best.distance, best.target);
      }
      const sssp::Vertex target = best.target;
      while (index < candidates.size() && candidates[index].target == target) {
        ++index;
      }
    }
  }
}

std::vector<sssp::Vertex> ParallelDijkstraSolver::reconstruct_path(const sssp::Vertex source,
                                                                   const sssp::Vertex goal) const {
  std::vector<sssp::Vertex> path;
  for (sssp::Vertex current = goal;; current = predecessors_[current]) {
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

} // namespace validation
