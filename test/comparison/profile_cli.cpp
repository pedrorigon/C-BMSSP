#include "constant_degree_graph.h"
#include "dijkstra_solver.h"
#include "graph_factory.h"
#include "parallel_dijkstra_solver.h"
#include "parallel_solver.h"
#include "sequential_solver.h"

#include <chrono>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string_view>

namespace {

using Clock = std::chrono::steady_clock;

template <typename Solver>
double run_solver(Solver& solver, const std::string_view mode, const sssp::Vertex source,
                  const sssp::Vertex goal, const std::size_t repetitions) {
  double checksum = 0.0;
  for (std::size_t repetition = 0; repetition < repetitions; ++repetition) {
    if (mode == "path") {
      const auto result = solver.solve(source, goal);
      checksum += result.distance;
      checksum += static_cast<double>(result.path.size());
    } else {
      const auto distances = solver.solve_all(source);
      for (std::size_t index = 0; index < distances.size(); index += 4096) {
        if (distances[index] != sssp::infinity) {
          checksum += distances[index];
        }
      }
    }
  }
  return checksum;
}

template <typename Solver>
void measure(Solver& solver, const std::string_view mode, const sssp::Vertex source,
             const sssp::Vertex goal, const std::size_t repetitions) {
  static_cast<void>(run_solver(solver, mode, source, goal, 1));
  const auto start = Clock::now();
  const double checksum = run_solver(solver, mode, source, goal, repetitions);
  const double elapsed = std::chrono::duration<double>(Clock::now() - start).count();
  std::cout << std::setprecision(17) << "elapsed_seconds=" << elapsed
            << " seconds_per_run=" << elapsed / static_cast<double>(repetitions)
            << " checksum=" << checksum << '\n';
}

} // namespace

int main(const int argc, const char* argv[]) {
  if (argc != 8) {
    std::cerr << "usage: sssp_profile <dijkstra-sequential|dijkstra-parallel|"
                 "dijkstra-transformed-sequential|"
                 "dijkstra-transformed-parallel|bmssp-sequential|"
                 "bmssp-parallel> <path|full> <random|banded> "
                 "<vertices> <degree> <bandwidth> <repetitions>\n";
    return EXIT_FAILURE;
  }

  try {
    const std::string_view algorithm = argv[1];
    const std::string_view mode = argv[2];
    const std::string_view topology = argv[3];
    const auto vertices = std::stoull(argv[4]);
    const auto degree = std::stoull(argv[5]);
    const auto bandwidth = std::stoull(argv[6]);
    const auto repetitions = std::stoull(argv[7]);
    if (vertices < 2 || degree < 1 || repetitions < 1) {
      throw std::invalid_argument("vertices, degree, and repetitions must be positive");
    }
    if (mode != "path" && mode != "full") {
      throw std::invalid_argument("mode must be path or full");
    }
    if (topology != "random" && topology != "banded") {
      throw std::invalid_argument("topology must be random or banded");
    }

    sssp::Graph graph = topology == "random"
                            ? test::make_random_graph(vertices, (degree - 1) * vertices, 42)
                            : test::make_banded_graph(vertices, degree, bandwidth, 42);
    const sssp::Vertex source = 0;
    const sssp::Vertex goal = vertices - 1;
    std::cout << "algorithm=" << algorithm << " mode=" << mode << " topology=" << topology
              << " vertices=" << graph.vertex_count() << " edges=" << graph.edge_count()
              << " repetitions=" << repetitions << '\n';

    if (algorithm == "dijkstra-sequential") {
      validation::DijkstraSolver solver(graph);
      measure(solver, mode, source, goal, repetitions);
    } else if (algorithm == "dijkstra-parallel") {
      validation::ParallelDijkstraSolver solver(graph);
      measure(solver, mode, source, goal, repetitions);
    } else if (algorithm == "dijkstra-transformed-sequential") {
      const auto transformed = sssp::detail::make_constant_degree_graph(graph);
      std::cout << "transformed_vertices=" << transformed.graph.vertex_count()
                << " transformed_edges=" << transformed.graph.edge_count() << '\n';
      validation::DijkstraSolver solver(transformed.graph);
      measure(solver, mode, source, goal, repetitions);
    } else if (algorithm == "dijkstra-transformed-parallel") {
      const auto transformed = sssp::detail::make_constant_degree_graph(graph);
      std::cout << "transformed_vertices=" << transformed.graph.vertex_count()
                << " transformed_edges=" << transformed.graph.edge_count() << '\n';
      validation::ParallelDijkstraSolver solver(transformed.graph);
      measure(solver, mode, source, goal, repetitions);
    } else if (algorithm == "bmssp-sequential") {
      sssp::SequentialSolver solver(graph);
      measure(solver, mode, source, goal, repetitions);
    } else if (algorithm == "bmssp-parallel") {
      sssp::ParallelSolver solver(graph);
      measure(solver, mode, source, goal, repetitions);
    } else {
      throw std::invalid_argument("unknown algorithm");
    }
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
