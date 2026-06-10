#include "dimacs_reader.h"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace sssp {

Graph read_dimacs_graph(const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("failed to open DIMACS graph: " + path.string());
  }

  Graph graph;
  bool has_problem_line = false;
  std::size_t declared_edges = 0;
  std::size_t line_number = 0;
  std::string line;
  while (std::getline(input, line)) {
    ++line_number;
    std::istringstream stream(line);
    char kind{};
    if (!(stream >> kind) || kind == 'c') {
      continue;
    }

    if (kind == 'p') {
      if (has_problem_line) {
        throw std::runtime_error("duplicate DIMACS problem line at line " +
                                 std::to_string(line_number));
      }
      std::string problem_type;
      std::size_t vertices{};
      if (!(stream >> problem_type >> vertices >> declared_edges) || problem_type != "sp" ||
          vertices == 0) {
        throw std::runtime_error("invalid DIMACS problem line at line " +
                                 std::to_string(line_number));
      }
      graph = Graph(vertices);
      has_problem_line = true;
    } else if (kind == 'a') {
      if (!has_problem_line) {
        throw std::runtime_error("DIMACS arc appears before problem line at line " +
                                 std::to_string(line_number));
      }
      Vertex from{};
      Vertex to{};
      double weight{};
      if (!(stream >> from >> to >> weight) || from == 0 || to == 0) {
        throw std::runtime_error("invalid DIMACS arc line at line " + std::to_string(line_number));
      }
      graph.add_edge(from - 1, to - 1, weight);
    }
  }

  if (!has_problem_line) {
    throw std::runtime_error("DIMACS graph has no problem line");
  }
  if (graph.edge_count() != declared_edges) {
    throw std::runtime_error("DIMACS edge count mismatch: declared " +
                             std::to_string(declared_edges) + ", read " +
                             std::to_string(graph.edge_count()));
  }
  return graph;
}

} // namespace sssp
