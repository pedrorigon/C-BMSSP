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

// Below this frontier size a bucket is processed on a cheap serial fast path
// (no per-thread buffers, no sort): the parallel machinery would cost more than
// it saves. Above it, edges are relaxed in parallel across the bucket.
constexpr std::size_t kParallelThreshold = 2048;

// Bucket width for Delta-stepping. Edges with weight <= delta are "light" (may
// keep a vertex in the current bucket) and the rest are "heavy". The classic
// choice delta ~= mean_weight / max_degree keeps the light-edge re-scan of a
// bucket cheap; making delta too large turns almost every edge light and
// degenerates the light phase into repeated Bellman-Ford-style passes. We use
// mean_weight scaled down by the average degree so each vertex contributes only
// a small, bounded number of light edges.
double choose_delta(const sssp::Graph& graph) {
  const std::size_t vertex_count = graph.vertex_count();
  double weight_sum = 0.0;
  std::size_t edge_total = 0;
  double max_weight = 0.0;
  for (sssp::Vertex vertex = 0; vertex < vertex_count; ++vertex) {
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
  if (mean_weight <= 0.0) {
    // Degenerate all-zero-weight graph: any positive width keeps it correct.
    return std::max(max_weight, 1.0);
  }
  const double avg_degree =
      static_cast<double>(edge_total) / static_cast<double>(std::max<std::size_t>(vertex_count, 1));
  return mean_weight / std::max(1.0, avg_degree);
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

  // Relax `edge.to` from `from`. Smaller distance wins; on an exact tie the
  // smaller predecessor wins, matching the sequential solver's tie-breaking so
  // shortest-path trees stay identical even on equal- and zero-weight edges.
  auto relax_edge = [&](const sssp::Vertex from, const sssp::Edge& edge, const double base) {
    const double candidate = base + edge.weight;
    double& best = distances_[edge.to];
    if (candidate < best || (candidate == best && from < predecessors_[edge.to])) {
      best = candidate;
      predecessors_[edge.to] = from;
      place(edge.to);
    }
  };

#ifdef VALIDATION_HAS_OPENMP
  const auto thread_count = static_cast<std::size_t>(omp_get_max_threads());
#else
  const std::size_t thread_count = 1;
#endif
  // Per-thread candidate buffers, allocated once and reused across buckets to
  // avoid repeated allocation on large-diameter graphs with many buckets.
  std::vector<std::vector<Candidate>> thread_local_candidates(thread_count);
  std::vector<Candidate> merged;

  // Relax the `want_light` edges of every vertex in `frontier` and apply the
  // results. Small frontiers take a cheap serial path; large ones relax in
  // parallel, then merge and apply deterministically.
  auto process = [&](const std::vector<sssp::Vertex>& frontier, const bool want_light) {
    if (frontier.size() < kParallelThreshold) {
      for (const sssp::Vertex from : frontier) {
        const double base = distances_[from];
        for (const auto& edge : graph_.edges_from(from)) {
          if ((edge.weight <= delta) == want_light) {
            relax_edge(from, edge, base);
          }
        }
      }
      return;
    }

    for (auto& chunk : thread_local_candidates) {
      chunk.clear();
    }
#ifdef VALIDATION_HAS_OPENMP
#pragma omp parallel for schedule(dynamic, 256)
#endif
    for (std::size_t i = 0; i < frontier.size(); ++i) {
#ifdef VALIDATION_HAS_OPENMP
      auto& sink = thread_local_candidates[static_cast<std::size_t>(omp_get_thread_num())];
#else
      auto& sink = thread_local_candidates.front();
#endif
      const sssp::Vertex from = frontier[i];
      const double base = distances_[from];
      for (const auto& edge : graph_.edges_from(from)) {
        if ((edge.weight <= delta) != want_light) {
          continue;
        }
        const double candidate = base + edge.weight;
        // Lock-free relaxed read; the serial apply below re-checks every value.
        if (candidate < distances_[edge.to]) {
          sink.push_back({edge.to, candidate, from});
        }
      }
    }

    merged.clear();
    for (auto& chunk : thread_local_candidates) {
      merged.insert(merged.end(), chunk.begin(), chunk.end());
    }
    if (merged.empty()) {
      return;
    }
    std::sort(merged.begin(), merged.end(), candidate_less);
    for (std::size_t index = 0; index < merged.size();) {
      const Candidate& best = merged[index];
      if (best.distance < distances_[best.target]) {
        distances_[best.target] = best.distance;
        predecessors_[best.target] = best.predecessor;
        place(best.target);
      }
      const sssp::Vertex target = best.target;
      while (index < merged.size() && merged[index].target == target) {
        ++index;
      }
    }
  };

  static_cast<void>(goal); // run always computes full SSSP; callers read distances_.

  std::vector<sssp::Vertex> frontier;
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
      frontier.clear();
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

      process(active, /*want_light=*/true);
    }

    // Heavy-edge phase: relax heavy edges of everything removed from this bucket.
    // Deduplicate first, since a vertex can be removed more than once.
    std::sort(removed.begin(), removed.end());
    removed.erase(std::unique(removed.begin(), removed.end()), removed.end());
    process(removed, /*want_light=*/false);
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
