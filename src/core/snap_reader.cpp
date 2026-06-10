#include "snap_reader.h"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace sssp {

namespace {

std::ifstream open_graph(const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("failed to open SNAP graph: " + path.string());
  }
  return input;
}

std::size_t scan_vertex_count(const std::filesystem::path& path) {
  std::ifstream input = open_graph(path);
  bool has_edge = false;
  Vertex maximum_vertex = 0;
  std::string line;
  while (std::getline(input, line)) {
    if (line.empty() || line.front() == '#') {
      continue;
    }
    std::istringstream stream(line);
    Vertex from = 0;
    Vertex to = 0;
    if (!(stream >> from >> to)) {
      throw std::runtime_error("invalid SNAP edge line");
    }
    maximum_vertex = std::max({maximum_vertex, from, to});
    has_edge = true;
  }
  return has_edge ? maximum_vertex + 1 : 0;
}

} // namespace

Graph read_snap_graph(const std::filesystem::path& path, const bool directed) {
  // SNAP node IDs can be sparse, so the declared node count is not a safe allocation size.
  const std::size_t vertices = scan_vertex_count(path);
  Graph graph(vertices);
  std::ifstream input = open_graph(path);

  std::size_t line_number = 0;
  std::string line;
  while (std::getline(input, line)) {
    ++line_number;
    if (line.empty() || line.front() == '#') {
      continue;
    }

    std::istringstream stream(line);
    Vertex from = 0;
    Vertex to = 0;
    if (!(stream >> from >> to)) {
      throw std::runtime_error("invalid SNAP edge at line " + std::to_string(line_number));
    }
    // SNAP graphs are unweighted; assign a unit weight to every edge, matching
    // the reference bmssp-python loader.
    graph.add_edge(from, to, 1.0);
    if (!directed && from != to) {
      graph.add_edge(to, from, 1.0);
    }
  }
  return graph;
}

} // namespace sssp
