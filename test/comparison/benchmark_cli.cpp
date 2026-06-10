#include "dijkstra_solver.h"
#include "parallel_dijkstra_solver.h"
#include "parallel_solver.h"
#include "sequential_solver.h"
#include "snap_reader.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <queue>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

bool selected(const std::string_view requested, const std::string_view value) {
  return requested == "all" || requested == value;
}

template <typename Solver>
void run_solver(const std::string_view model, const std::string_view type, Solver& solver,
                const sssp::Vertex source, const sssp::Vertex goal, const std::size_t step) {
  // Measure a point-to-point query (source to goal, stopping once the goal is
  // settled), matching the reference bmssp-python benchmark methodology.
  const auto start = Clock::now();
  const sssp::PathResult result = solver.solve(source, goal);
  const double elapsed_ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();

  std::cout << "RESULT|" << step << '|' << model << '|' << type << '|'
            << (result.reachable() ? "reachable" : "unreachable") << '|';
  if (result.reachable()) {
    std::cout << result.distance;
  }
  std::cout << '|' << result.path.size() << '|' << elapsed_ms << '\n' << std::flush;
}

std::pair<std::size_t, std::size_t> parse_size(const std::string_view value) {
  const std::size_t separator = value.find(':');
  if (separator == std::string_view::npos) {
    throw std::invalid_argument("graph size must use the <vertices>:<edges> format");
  }
  const std::size_t vertices = std::stoull(std::string(value.substr(0, separator)));
  const std::size_t edges = std::stoull(std::string(value.substr(separator + 1)));
  if (vertices == 0 || edges == 0) {
    throw std::invalid_argument("graph size values must be greater than zero");
  }
  return {vertices, edges};
}

struct BenchmarkGraph {
  sssp::Graph graph;
  sssp::Vertex source;
  sssp::Vertex goal;
};

BenchmarkGraph make_bfs_graph(const sssp::Graph& source_graph, const sssp::Vertex source,
                              const std::size_t requested_vertices,
                              const std::size_t requested_edges) {
  const std::size_t vertex_limit = std::min(requested_vertices, source_graph.vertex_count());
  const std::size_t edge_limit = std::min(requested_edges, source_graph.edge_count());
  const sssp::Vertex unmapped = source_graph.vertex_count();
  std::vector<sssp::Vertex> mapping(source_graph.vertex_count(), unmapped);
  std::vector<std::tuple<sssp::Vertex, sssp::Vertex, double>> edges;
  edges.reserve(edge_limit);
  std::queue<sssp::Vertex> queue;

  mapping[source] = 0;
  queue.push(source);
  std::size_t mapped_vertices = 1;
  sssp::Vertex goal = 0;
  while (!queue.empty() && (mapped_vertices < vertex_limit || edges.size() < edge_limit)) {
    const sssp::Vertex original_from = queue.front();
    queue.pop();
    const sssp::Vertex from = mapping[original_from];
    for (const auto& edge : source_graph.edges_from(original_from)) {
      const bool can_discover = mapping[edge.to] == unmapped && mapped_vertices < vertex_limit;
      if (can_discover) {
        mapping[edge.to] = mapped_vertices++;
        goal = mapping[edge.to];
        queue.push(edge.to);
      }

      if (mapping[edge.to] == unmapped) {
        continue;
      }
      const std::size_t undiscovered_vertices = vertex_limit - mapped_vertices;
      const bool must_reserve_tree_edges = edges.size() + undiscovered_vertices >= edge_limit;
      if (edges.size() < edge_limit && (can_discover || !must_reserve_tree_edges)) {
        edges.emplace_back(from, mapping[edge.to], edge.weight);
      }
      if (mapped_vertices == vertex_limit && edges.size() == edge_limit) {
        break;
      }
    }
  }

  if (mapped_vertices < 2 || goal == 0) {
    throw std::invalid_argument("dataset source has no reachable vertex pair");
  }

  sssp::Graph graph(mapped_vertices);
  for (const auto& [from, to, weight] : edges) {
    graph.add_edge(from, to, weight);
  }
  return {std::move(graph), 0, goal};
}

void run_graph(const sssp::Graph& graph, const std::size_t step,
               const std::size_t requested_vertices, const std::size_t requested_edges,
               const sssp::Vertex source, const sssp::Vertex goal, const std::string_view model,
               const std::string_view type) {
  std::cout << "GRAPH|" << step << '|' << requested_vertices << '|' << requested_edges << '|'
            << graph.vertex_count() << '|' << graph.edge_count() << '|' << source << '|' << goal
            << '\n'
            << std::flush;

  if (selected(model, "bmssp") && selected(type, "sequential")) {
    sssp::SequentialSolver solver(graph);
    run_solver("bmssp", "sequential", solver, source, goal, step);
  }
  if (selected(model, "bmssp") && selected(type, "parallel")) {
    sssp::ParallelSolver solver(graph);
    run_solver("bmssp", "parallel", solver, source, goal, step);
  }
  if (selected(model, "dijkstra") && selected(type, "sequential")) {
    validation::DijkstraSolver solver(graph);
    run_solver("dijkstra", "sequential", solver, source, goal, step);
  }
  if (selected(model, "dijkstra") && selected(type, "parallel")) {
    validation::ParallelDijkstraSolver solver(graph);
    run_solver("dijkstra", "parallel", solver, source, goal, step);
  }
}

void validate_selection(const std::string_view model, const std::string_view type) {
  if (model != "all" && model != "bmssp" && model != "dijkstra") {
    throw std::invalid_argument("model must be all, bmssp, or dijkstra");
  }
  if (type != "all" && type != "sequential" && type != "parallel") {
    throw std::invalid_argument("type must be all, sequential, or parallel");
  }
}

} // namespace

int main(const int argc, const char* argv[]) {
  if (argc < 6) {
    std::cerr << "usage: sssp_benchmark <snap-file> <source-id> <goal-id> "
                 "<all|bmssp|dijkstra> <all|sequential|parallel> [<vertices>:<edges> ...]\n";
    return EXIT_FAILURE;
  }

  try {
    const auto source = static_cast<sssp::Vertex>(std::stoull(argv[2]));
    const auto goal = static_cast<sssp::Vertex>(std::stoull(argv[3]));
    const std::string_view model = argv[4];
    const std::string_view type = argv[5];
    validate_selection(model, type);

    const auto load_start = Clock::now();
    const sssp::Graph graph = sssp::read_snap_graph(argv[1]);
    const double load_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - load_start).count();
    std::cout << std::setprecision(17) << "DATASET|" << graph.vertex_count() << '|'
              << graph.edge_count() << '|' << load_ms << '\n'
              << std::flush;

    std::size_t step = 0;
    for (int index = 6; index < argc; ++index) {
      const auto [vertices, edges] = parse_size(argv[index]);
      if (vertices >= graph.vertex_count() && edges >= graph.edge_count()) {
        continue;
      }
      BenchmarkGraph sample = make_bfs_graph(graph, source, vertices, edges);
      run_graph(sample.graph, step++, vertices, edges, sample.source, sample.goal, model, type);
    }
    run_graph(graph, step, graph.vertex_count(), graph.edge_count(), source, goal, model, type);
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
