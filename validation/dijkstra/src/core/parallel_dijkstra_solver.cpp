#include "parallel_dijkstra_solver.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

#ifdef VALIDATION_HAS_OPENMP
#include <omp.h>
#endif

namespace validation {

namespace {

// Candidate relaxation produced while scanning a bucket: a tentative distance to
// `target` via `predecessor`. Collected in parallel, then applied serially in a
// deterministic order so the resulting distances (and predecessor tie-breaks)
// match the sequential Dijkstra reference.
struct Candidate {
  sssp::Vertex target;
  double distance;
  sssp::Vertex predecessor;
};

// Orders candidates so that, per target, the smallest distance wins and ties are
// broken by the smallest predecessor index. This mirrors the tie-breaking of the
// sequential solver, keeping shortest-path trees identical even on equal-weight
// (including zero-weight) edges.
bool candidate_less(const Candidate& left, const Candidate& right) {
  if (left.target != right.target) {
    return left.target < right.target;
  }
  if (left.distance != right.distance) {
    return left.distance < right.distance;
  }
  return left.predecessor < right.predecessor;
}

// Bucket width for Delta-stepping. Edges with weight <= delta are "light" (may
// keep a vertex in the current bucket) and the rest are "heavy". A width close to
// the average edge weight balances bucket count against work per bucket.
double choose_delta(const sssp::Graph& graph) {
  double weight_sum = 0.0;
  std::size_t edge_total = 0;
  double max_weight = 0.0;
  for (sssp::Vertex vertex = 0; vertex < graph.vertex_count(); ++vertex) {
    for (const auto& edge : graph.edges_from(vertex)) {
      weight_sum += edge.weight;
      max_weight = std::max(max_weight, edge.weight);
      ++edge_total;
    }
  }
  if (edge_total == 0) {
    return 1.0;
  }
  const double mean_weight = weight_sum / static_cast<double>(edge_total);
  // Guard against degenerate all-zero-weight graphs.
  return mean_weight > 0.0 ? mean_weight : std::max(max_weight, 1.0);
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
  const std::size_t vertex_count = graph_.vertex_count();
  if (vertex_count == 0) {
    return;
  }

  const double delta = choose_delta(graph_);
  // buckets[i] holds vertices whose tentative distance is in [i*delta,(i+1)*delta).
  std::vector<std::vector<sssp::Vertex>> buckets;
  // Which bucket a vertex currently sits in (npos = none), to avoid duplicate
  // insertions and to detect stale bucket membership cheaply.
  std::vector<std::size_t> bucket_of(vertex_count, static_cast<std::size_t>(-1));

  auto bucket_index = [delta](const double distance) {
    return static_cast<std::size_t>(std::floor(distance / delta));
  };
  auto ensure_bucket = [&buckets](const std::size_t index) {
    if (index >= buckets.size()) {
      buckets.resize(index + 1);
    }
  };
  auto place = [&](const sssp::Vertex vertex) {
    const std::size_t index = bucket_index(distances_[vertex]);
    ensure_bucket(index);
    buckets[index].push_back(vertex);
    bucket_of[vertex] = index;
  };

  distances_[source] = 0.0;
  place(source);

  // Apply a batch of candidate relaxations deterministically: smallest distance
  // per target wins, ties broken by predecessor. Updated vertices are re-bucketed.
  auto apply_candidates = [&](std::vector<Candidate>& candidates) {
    if (candidates.empty()) {
      return;
    }
    std::sort(candidates.begin(), candidates.end(), candidate_less);
    for (std::size_t index = 0; index < candidates.size();) {
      const Candidate& best = candidates[index];
      if (best.distance < distances_[best.target]) {
        distances_[best.target] = best.distance;
        predecessors_[best.target] = best.predecessor;
        place(best.target);
      }
      const sssp::Vertex target = best.target;
      while (index < candidates.size() && candidates[index].target == target) {
        ++index;
      }
    }
  };

  // Relaxes a set of source vertices over the edges selected by `want_light`,
  // returning the produced candidates. The scan over `frontier` is parallel, so
  // the work scales with the number of vertices in the current bucket.
  auto relax = [&](const std::vector<sssp::Vertex>& frontier, const bool want_light) {
    std::vector<Candidate> candidates;
#ifdef VALIDATION_HAS_OPENMP
    const int thread_count = omp_get_max_threads();
#else
    const int thread_count = 1;
#endif
    std::vector<std::vector<Candidate>> local(static_cast<std::size_t>(thread_count));

#ifdef VALIDATION_HAS_OPENMP
#pragma omp parallel for schedule(dynamic, 64) if (frontier.size() >= 128)
#endif
    for (std::size_t i = 0; i < frontier.size(); ++i) {
#ifdef VALIDATION_HAS_OPENMP
      auto& sink = local[static_cast<std::size_t>(omp_get_thread_num())];
#else
      auto& sink = local.front();
#endif
      const sssp::Vertex from = frontier[i];
      const double base = distances_[from];
      for (const auto& edge : graph_.edges_from(from)) {
        const bool is_light = edge.weight <= delta;
        if (is_light != want_light) {
          continue;
        }
        const double candidate = base + edge.weight;
        // Relaxed check without locking; apply_candidates re-verifies under the
        // serial pass, so a stale read here only adds a redundant candidate.
        if (candidate < distances_[edge.to]) {
          sink.push_back({edge.to, candidate, from});
        }
      }
    }

    std::size_t total = 0;
    for (const auto& chunk : local) {
      total += chunk.size();
    }
    candidates.reserve(total);
    for (auto& chunk : local) {
      candidates.insert(candidates.end(), std::make_move_iterator(chunk.begin()),
                        std::make_move_iterator(chunk.end()));
    }
    return candidates;
  };

  static_cast<void>(goal); // run always computes full SSSP; callers read distances_.

  for (std::size_t current = 0; current < buckets.size(); ++current) {
    if (buckets[current].empty()) {
      continue;
    }

    // Union of every vertex removed from this bucket across all light passes.
    // Their heavy edges are relaxed once, after the bucket fully drains.
    std::vector<sssp::Vertex> removed;

    // Light-edge phase: repeatedly drain the current bucket, since light edges
    // (and zero-weight edges in particular) can re-insert vertices into it.
    while (!buckets[current].empty()) {
      std::vector<sssp::Vertex> frontier;
      frontier.swap(buckets[current]);

      // A vertex may sit in the bucket multiple times (re-inserted by an earlier
      // pass); only act on those whose tentative distance still maps here.
      std::vector<sssp::Vertex> active;
      active.reserve(frontier.size());
      for (const sssp::Vertex vertex : frontier) {
        if (bucket_of[vertex] == current) {
          bucket_of[vertex] = static_cast<std::size_t>(-1);
          active.push_back(vertex);
          removed.push_back(vertex);
        }
      }
      if (active.empty()) {
        continue;
      }

      std::vector<Candidate> light = relax(active, /*want_light=*/true);
      apply_candidates(light);
    }

    // Heavy-edge phase: relax heavy edges of everything removed from this bucket.
    // Deduplicate first, since a vertex can be removed more than once.
    std::sort(removed.begin(), removed.end());
    removed.erase(std::unique(removed.begin(), removed.end()), removed.end());
    std::vector<Candidate> heavy = relax(removed, /*want_light=*/false);
    apply_candidates(heavy);
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
