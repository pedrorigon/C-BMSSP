#include "dijkstra_solver.h"

#include "work_counter.h"

#include <algorithm>
#include <functional>
#include <queue>
#include <stdexcept>
#include <utility>

namespace validation {

namespace {
using QueueEntry = std::pair<double, sssp::Vertex>;
}

DijkstraSolver::DijkstraSolver(const sssp::Graph& graph)
    : graph_(graph), distances_(graph.vertex_count(), sssp::infinity),
      predecessors_(graph.vertex_count(), graph.vertex_count()) {}

void DijkstraSolver::set_trace_callback(sssp::SearchTraceCallback callback) {
  trace_callback_ = std::move(callback);
}

void DijkstraSolver::trace(const sssp::SearchEventKind kind, const sssp::Vertex vertex,
                           const double distance) const {
  if (trace_callback_) {
    trace_callback_({kind, vertex, distance});
  }
}

void DijkstraSolver::reset() {
  std::fill(distances_.begin(), distances_.end(), sssp::infinity);
  std::fill(predecessors_.begin(), predecessors_.end(), graph_.vertex_count());
}

void DijkstraSolver::validate_vertex(const sssp::Vertex vertex) const {
  if (vertex >= graph_.vertex_count()) {
    throw std::out_of_range("query vertex is outside graph");
  }
}

sssp::PathResult DijkstraSolver::solve(const sssp::Vertex source, const sssp::Vertex goal) {
  validate_vertex(source);
  validate_vertex(goal);
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
    trace(sssp::SearchEventKind::settled, vertex, distance);
    if (vertex == goal) {
      return {distance, reconstruct_path(source, goal)};
    }
    for (const auto& edge : graph_.edges_from(vertex)) {
      const double candidate = distance + edge.weight;
      if (candidate < distances_[edge.to]) {
        distances_[edge.to] = candidate;
        predecessors_[edge.to] = vertex;
        trace(sssp::SearchEventKind::relaxation, edge.to, candidate);
        queue.emplace(candidate, edge.to);
      }
    }
  }
  return {};
}

std::vector<double> DijkstraSolver::solve_all(const sssp::Vertex source) {
  validate_vertex(source);
  reset();

  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<>> queue;
  distances_[source] = 0.0;
  queue.emplace(0.0, source);

  while (!queue.empty()) {
    // An extract-min sifts down through a heap of the current size: O(log n).
    sssp::WorkCounter::add_log(queue.size());
    const auto [distance, vertex] = queue.top();
    queue.pop();
    if (distance > distances_[vertex]) {
      continue;
    }
    trace(sssp::SearchEventKind::settled, vertex, distance);
    for (const auto& edge : graph_.edges_from(vertex)) {
      const double candidate = distance + edge.weight;
      if (candidate < distances_[edge.to]) {
        distances_[edge.to] = candidate;
        predecessors_[edge.to] = vertex;
        trace(sssp::SearchEventKind::relaxation, edge.to, candidate);
        // A decrease-key, modelled here as a lazy insertion, sifts up: O(log n).
        sssp::WorkCounter::add_log(queue.size());
        queue.emplace(candidate, edge.to);
      }
    }
  }
  return distances_;
}

std::vector<sssp::Vertex> DijkstraSolver::reconstruct_path(const sssp::Vertex source,
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
