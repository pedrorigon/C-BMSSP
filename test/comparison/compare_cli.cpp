#include "dijkstra_solver.h"
#include "dimacs_reader.h"
#include "graph.h"
#include "parallel_dijkstra_solver.h"
#include "parallel_solver.h"
#include "sequential_solver.h"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <string_view>

namespace {

template <typename Solver>
sssp::PathResult run_and_report(const std::string_view name, Solver& solver,
                                const sssp::Vertex source, const sssp::Vertex goal) {
  const auto start = std::chrono::steady_clock::now();
  const sssp::PathResult result = solver.solve(source, goal);
  const auto elapsed =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start);

  std::cout << std::left << std::setw(20) << name << "  ";
  if (result.reachable()) {
    std::cout << "distance=" << result.distance << " path_vertices=" << result.path.size();
  } else {
    std::cout << "unreachable";
  }
  std::cout << " time_ms=" << elapsed.count() << '\n';
  return result;
}

bool same_distance(const sssp::PathResult& left, const sssp::PathResult& right) {
  if (left.reachable() != right.reachable()) {
    return false;
  }
  return !left.reachable() || std::abs(left.distance - right.distance) < 1e-9;
}

} // namespace

int main(const int argc, const char* argv[]) {
  if (argc != 4) {
    std::cerr << "usage: sssp_compare <dimacs-file> <source-id> <goal-id>\n"
                 "source-id and goal-id use DIMACS 1-based numbering\n";
    return EXIT_FAILURE;
  }

  try {
    const sssp::Graph graph = sssp::read_dimacs_graph(argv[1]);
    const auto source_id = std::stoull(argv[2]);
    const auto goal_id = std::stoull(argv[3]);
    if (source_id == 0 || goal_id == 0) {
      throw std::invalid_argument("DIMACS vertex IDs start at 1");
    }
    const sssp::Vertex source = source_id - 1;
    const sssp::Vertex goal = goal_id - 1;

    std::cout << "vertices=" << graph.vertex_count() << " edges=" << graph.edge_count() << '\n';
    validation::DijkstraSolver dijkstra(graph);
    validation::ParallelDijkstraSolver parallel_dijkstra(graph);
    sssp::SequentialSolver sequential(graph);
    sssp::ParallelSolver parallel(graph);

    const auto dijkstra_result = run_and_report("dijkstra", dijkstra, source, goal);
    const auto parallel_dijkstra_result =
        run_and_report("dijkstra parallel", parallel_dijkstra, source, goal);
    const auto sequential_result = run_and_report("sequential", sequential, source, goal);
    const auto parallel_result = run_and_report("parallel", parallel, source, goal);

    if (!same_distance(dijkstra_result, parallel_dijkstra_result) ||
        !same_distance(dijkstra_result, sequential_result) ||
        !same_distance(dijkstra_result, parallel_result)) {
      std::cerr << "error: solvers produced different results\n";
      return EXIT_FAILURE;
    }
    std::cout << "results_match=true\n";
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
