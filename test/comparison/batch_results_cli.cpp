#include "dijkstra_solver.h"
#include "dimacs_reader.h"
#include "parallel_dijkstra_solver.h"
#include "parallel_solver.h"
#include "sequential_solver.h"

#include <cstdlib>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace {

void print_distance(const sssp::PathResult& result) {
  if (result.reachable()) {
    std::cout << result.distance;
  } else {
    std::cout << "inf";
  }
}

} // namespace

int main(const int argc, const char* argv[]) {
  if (argc != 3) {
    std::cerr << "usage: sssp_batch_results <dimacs-file> <query-file>\n";
    return EXIT_FAILURE;
  }

  try {
    const sssp::Graph graph = sssp::read_dimacs_graph(argv[1]);
    std::ifstream queries(argv[2]);
    if (!queries) {
      throw std::runtime_error("failed to open query file");
    }

    validation::DijkstraSolver dijkstra(graph);
    validation::ParallelDijkstraSolver parallel_dijkstra(graph);
    sssp::SequentialSolver sequential(graph);
    sssp::ParallelSolver parallel(graph);
    std::cout << std::setprecision(17);

    sssp::Vertex source = 0;
    sssp::Vertex goal = 0;
    while (queries >> source >> goal) {
      const auto dijkstra_result = dijkstra.solve(source, goal);
      const auto parallel_dijkstra_result = parallel_dijkstra.solve(source, goal);
      const auto sequential_result = sequential.solve(source, goal);
      const auto parallel_result = parallel.solve(source, goal);

      std::cout << source << ',' << goal << ',';
      print_distance(dijkstra_result);
      std::cout << ',';
      print_distance(parallel_dijkstra_result);
      std::cout << ',';
      print_distance(sequential_result);
      std::cout << ',';
      print_distance(parallel_result);
      std::cout << '\n';
    }
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
