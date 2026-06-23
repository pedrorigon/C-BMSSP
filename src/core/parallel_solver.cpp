#include "parallel_solver.h"

#include <vector>

#ifdef SSSP_HAS_OPENMP
#include <omp.h>
#endif

namespace sssp {

namespace {

struct Candidate {
  Vertex from;
  Edge edge;
};

} // namespace

ParallelSolver::ParallelSolver(const Graph& graph, const SolverOptions options)
    : SequentialSolver(graph, options) {}

void ParallelSolver::relax_completed(const std::vector<Vertex>& completed,
                                     const detail::QueueKey recursive_bound,
                                     const detail::QueueKey pull_bound,
                                     const detail::QueueKey call_bound,
                                     detail::BlockQueue& data_structure,
                                     std::vector<detail::QueueItem>& prepend) {
  if (completed.size() < kParallelRelaxThreshold) {
    SequentialSolver::relax_completed(completed, recursive_bound, pull_bound, call_bound,
                                      data_structure, prepend);
    return;
  }

#ifdef SSSP_HAS_OPENMP
  const std::size_t thread_count = static_cast<std::size_t>(omp_get_max_threads());
#else
  const std::size_t thread_count = 1;
#endif
  std::vector<std::vector<Candidate>> local_candidates(thread_count);
  for (auto& candidates : local_candidates) {
    candidates.reserve(completed.size() * 2 / thread_count + 1);
  }

#ifdef SSSP_HAS_OPENMP
#pragma omp parallel for schedule(static)
#endif
  for (std::size_t index = 0; index < completed.size(); ++index) {
#ifdef SSSP_HAS_OPENMP
    auto& output = local_candidates[static_cast<std::size_t>(omp_get_thread_num())];
#else
    auto& output = local_candidates.front();
#endif
    const Vertex from = completed[index];
    for (const Edge& edge : graph_.edges_from(from)) {
      detail::QueueKey candidate{};
      if (candidate_valid(from, edge, candidate)) {
        output.push_back({from, edge});
      }
    }
  }

  std::vector<Candidate> candidates;
  std::size_t total = 0;
  for (const auto& local : local_candidates) {
    total += local.size();
  }
  candidates.reserve(total);
  for (auto& local : local_candidates) {
    candidates.insert(candidates.end(), std::make_move_iterator(local.begin()),
                      std::make_move_iterator(local.end()));
  }

  for (const Vertex vertex : completed) {
    complete_[vertex] = true;
    trace(SearchEventKind::completion, vertex, distances_[vertex]);
  }
  for (const Candidate& candidate : candidates) {
    detail::QueueKey ignored{};
    if (!relax(candidate.from, candidate.edge, ignored)) {
      continue;
    }
    const detail::QueueKey current = key(candidate.edge.to);
    if (!(current < call_bound)) {
      continue;
    }
    if (!(current < pull_bound)) {
      data_structure.insert(candidate.edge.to, current);
    } else if (!(current < recursive_bound)) {
      prepend.push_back({candidate.edge.to, current});
    }
  }
}

} // namespace sssp
