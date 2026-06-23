#include "dijkstra_solver.h"
#include "graph_factory.h"
#include "parallel_dijkstra_solver.h"
#include "parallel_solver.h"
#include "sequential_solver.h"
#include "work_counter.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

bool selected(const std::string_view requested, const std::string_view value) {
  return requested == "all" || requested == value;
}

// Counts vertices reached by the source (finite distance), reported for sanity.
std::size_t reachable_count(const std::vector<double>& distances) {
  return static_cast<std::size_t>(
      std::count_if(distances.begin(), distances.end(),
                    [](const double distance) { return distance != sssp::infinity; }));
}

template <typename Solver>
void run_solver(const std::string_view model, const std::string_view type, Solver& solver,
                const sssp::Vertex source, const std::size_t step) {
  // Measure full single-source shortest paths (source to all vertices). This is
  // where the BMSSP asymptotic advantage over Dijkstra shows up as the graph
  // grows, because the whole shortest-path tree is built.
  //
  // Alongside wall-clock we report weighted ordered-structure work: the count of
  // priority-queue / block-queue operations charged by the complexity analysis
  // (log n per heap op for Dijkstra, the Lemma 3.3 costs for BMSSP). Wall-clock
  // is dominated by constant factors and the constant-degree transformation, so
  // the work metric is what actually exhibits the O(m log^(2/3) n) exponent.
  sssp::WorkCounter::reset();
  const auto start = Clock::now();
  const std::vector<double> distances = solver.solve_all(source);
  const double elapsed_ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
  const double work = sssp::WorkCounter::total();

  std::cout << "RESULT|" << step << '|' << model << '|' << type << '|' << reachable_count(distances)
            << '|' << elapsed_ms << '|' << work << '\n'
            << std::flush;
}

void run_graph(const sssp::Graph& graph, const std::size_t step, const std::string_view model,
               const std::string_view type) {
  std::cout << "GRAPH|" << step << '|' << graph.vertex_count() << '|' << graph.edge_count() << '\n'
            << std::flush;

  const sssp::Vertex source = 0;
  if (selected(model, "bmssp") && selected(type, "sequential")) {
    sssp::SequentialSolver solver(graph);
    run_solver("bmssp", "sequential", solver, source, step);
  }
  if (selected(model, "bmssp") && selected(type, "parallel")) {
    sssp::ParallelSolver solver(graph);
    run_solver("bmssp", "parallel", solver, source, step);
  }
  if (selected(model, "dijkstra") && selected(type, "sequential")) {
    validation::DijkstraSolver solver(graph);
    run_solver("dijkstra", "sequential", solver, source, step);
  }
  if (selected(model, "dijkstra") && selected(type, "parallel")) {
    validation::ParallelDijkstraSolver solver(graph);
    run_solver("dijkstra", "parallel", solver, source, step);
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
  if (argc < 8) {
    std::cerr << "usage: sssp_scaling <all|bmssp|dijkstra> <all|sequential|parallel> "
                 "<random|banded> <avg-degree> <bandwidth> <seed> <vertices> [<vertices> ...]\n";
    return EXIT_FAILURE;
  }

  try {
    const std::string_view model = argv[1];
    const std::string_view type = argv[2];
    validate_selection(model, type);

    const std::string_view topology = argv[3];
    if (topology != "random" && topology != "banded") {
      throw std::invalid_argument("topology must be random or banded");
    }
    const auto degree = std::stoull(argv[4]);
    const auto bandwidth = std::stoull(argv[5]);
    const auto seed = static_cast<std::uint64_t>(std::stoull(argv[6]));
    if (degree < 1) {
      throw std::invalid_argument("avg-degree must be at least 1");
    }

    std::cout << std::setprecision(17);
    std::size_t step = 0;
    for (int index = 7; index < argc; ++index) {
      const auto vertices = std::stoull(argv[index]);
      if (vertices < 2) {
        throw std::invalid_argument("vertices must be at least 2");
      }
      sssp::Graph graph =
          topology == "banded"
              ? test::make_banded_graph(vertices, degree, bandwidth, seed)
              // random: path backbone (n-1 edges) + (degree-1)*n random edges.
              : test::make_random_graph(vertices, (degree - 1) * vertices, seed);
      run_graph(graph, step++, model, type);
    }
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
