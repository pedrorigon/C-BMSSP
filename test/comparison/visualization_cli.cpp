#include "dijkstra_solver.h"
#include "parallel_dijkstra_solver.h"
#include "parallel_solver.h"
#include "search_trace.h"
#include "sequential_solver.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <numeric>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

constexpr std::size_t kWidth = 108;
constexpr std::size_t kHeight = 68;
constexpr std::size_t kIterations = 60;
constexpr std::size_t kMissing = std::numeric_limits<std::size_t>::max();

struct RoadSegment {
  sssp::Vertex from;
  sssp::Vertex to;
};

struct VisualGraph {
  sssp::Graph graph;
  std::vector<std::pair<double, double>> coordinates;
  std::vector<RoadSegment> segments;
  sssp::Vertex source;
  sssp::Vertex goal;
};

struct TimedEvent {
  double elapsed_seconds;
  sssp::SearchEvent event;
};

bool near_curve(const double value, const double target, const double half_width) {
  return std::abs(value - target) <= half_width;
}

bool is_road(const std::size_t x, const std::size_t y) {
  const double dx = static_cast<double>(x) - 54.0;
  const double dy = static_cast<double>(y) - 34.0;
  const double ellipse = (dx * dx) / (43.0 * 43.0) + (dy * dy) / (25.0 * 25.0);

  const bool base_grid = x % 12 == 4 || y % 10 == 4 ||
                         (x < 48 && y < 32 && (x % 6 == 2 || y % 7 == 2)) ||
                         (x > 58 && y < 36 && (x % 7 == 3 || y % 6 == 1)) ||
                         (x < 52 && y > 38 && (x % 8 == 5 || y % 6 == 3)) ||
                         (x > 62 && y > 39 && (x % 6 == 1 || y % 8 == 5));

  const bool north_south =
      near_curve(static_cast<double>(x), 53.0 + 9.0 * std::sin(static_cast<double>(y) / 9.0), 1.2);
  const bool east_west =
      near_curve(static_cast<double>(y), 34.0 + 7.0 * std::sin(static_cast<double>(x) / 15.0), 1.2);
  const bool diagonal_one =
      near_curve(static_cast<double>(y), 0.48 * static_cast<double>(x) + 5.0, 1.0);
  const bool diagonal_two =
      near_curve(static_cast<double>(y), -0.42 * static_cast<double>(x) + 60.0, 1.0);
  const bool ring_road = std::abs(ellipse - 1.0) < 0.075;
  return base_grid || north_south || east_west || diagonal_one || diagonal_two || ring_road;
}

double road_weight(const std::size_t from_x, const std::size_t from_y, const std::size_t to_x,
                   const std::size_t to_y) {
  const double geometric = std::hypot(static_cast<double>(to_x) - static_cast<double>(from_x),
                                      static_cast<double>(to_y) - static_cast<double>(from_y));
  const std::size_t hash = (from_x * 41 + from_y * 67 + to_x * 23 + to_y * 31) % 29;
  const double traffic = 1.0 + static_cast<double>(hash) / 55.0;

  const bool arterial = std::abs(static_cast<double>(to_x) -
                                 (53.0 + 9.0 * std::sin(static_cast<double>(to_y) / 9.0))) < 1.6 ||
                        std::abs(static_cast<double>(to_y) -
                                 (34.0 + 7.0 * std::sin(static_cast<double>(to_x) / 15.0))) < 1.6;
  return geometric * traffic * (arterial ? 0.62 : 1.0);
}

sssp::Vertex nearest_vertex(const std::vector<std::pair<double, double>>& coordinates,
                            const double target_x, const double target_y) {
  sssp::Vertex best = 0;
  double best_distance = std::numeric_limits<double>::infinity();
  for (sssp::Vertex vertex = 0; vertex < coordinates.size(); ++vertex) {
    const auto [x, y] = coordinates[vertex];
    const double distance = std::hypot(x - target_x, y - target_y);
    if (distance < best_distance) {
      best_distance = distance;
      best = vertex;
    }
  }
  return best;
}

VisualGraph make_visual_graph() {
  std::vector<std::size_t> cell_vertex(kWidth * kHeight, kMissing);
  std::vector<std::pair<double, double>> coordinates;
  for (std::size_t y = 0; y < kHeight; ++y) {
    for (std::size_t x = 0; x < kWidth; ++x) {
      if (is_road(x, y)) {
        cell_vertex[y * kWidth + x] = coordinates.size();
        coordinates.emplace_back(static_cast<double>(x), static_cast<double>(y));
      }
    }
  }

  sssp::Graph graph(coordinates.size());
  std::vector<RoadSegment> segments;
  constexpr std::pair<int, int> directions[]{{1, 0}, {0, 1}, {1, 1}, {-1, 1}};
  for (std::size_t y = 0; y < kHeight; ++y) {
    for (std::size_t x = 0; x < kWidth; ++x) {
      const sssp::Vertex from = cell_vertex[y * kWidth + x];
      if (from == kMissing) {
        continue;
      }
      for (const auto& [delta_x, delta_y] : directions) {
        const auto next_x = static_cast<long>(x) + delta_x;
        const auto next_y = static_cast<long>(y) + delta_y;
        if (next_x < 0 || next_y < 0 || next_x >= static_cast<long>(kWidth) ||
            next_y >= static_cast<long>(kHeight)) {
          continue;
        }
        const sssp::Vertex to = cell_vertex[static_cast<std::size_t>(next_y) * kWidth +
                                            static_cast<std::size_t>(next_x)];
        if (to == kMissing) {
          continue;
        }
        const double forward =
            road_weight(x, y, static_cast<std::size_t>(next_x), static_cast<std::size_t>(next_y));
        const double reverse =
            road_weight(static_cast<std::size_t>(next_x), static_cast<std::size_t>(next_y), x, y);
        graph.add_edge(from, to, forward);
        graph.add_edge(to, from, reverse);
        segments.push_back({from, to});
      }
    }
  }

  const sssp::Vertex source = nearest_vertex(coordinates, 5.0, 5.0);
  const sssp::Vertex goal = nearest_vertex(coordinates, static_cast<double>(kWidth - 6),
                                           static_cast<double>(kHeight - 6));
  return {std::move(graph), std::move(coordinates), std::move(segments), source, goal};
}

std::string_view event_name(const sssp::SearchEventKind kind) {
  switch (kind) {
  case sssp::SearchEventKind::frontier:
    return "frontier";
  case sssp::SearchEventKind::settled:
    return "settled";
  case sssp::SearchEventKind::relaxation:
    return "relaxation";
  case sssp::SearchEventKind::completion:
    return "completion";
  }
  return "unknown";
}

template <typename Solver>
double benchmark(Solver& solver, const std::string_view mode, const sssp::Vertex source,
                 const sssp::Vertex goal) {
  if (mode == "path") {
    static_cast<void>(solver.solve(source, goal));
  } else {
    static_cast<void>(solver.solve_all(source));
  }

  std::vector<double> samples;
  samples.reserve(kIterations);
  for (std::size_t iteration = 0; iteration < kIterations; ++iteration) {
    const auto start = Clock::now();
    if (mode == "path") {
      static_cast<void>(solver.solve(source, goal));
    } else {
      static_cast<void>(solver.solve_all(source));
    }
    samples.push_back(std::chrono::duration<double>(Clock::now() - start).count());
  }
  std::ranges::sort(samples);
  const std::size_t trim = samples.size() / 10;
  return std::accumulate(samples.begin() + static_cast<std::ptrdiff_t>(trim),
                         samples.end() - static_cast<std::ptrdiff_t>(trim), 0.0) /
         static_cast<double>(samples.size() - 2 * trim);
}

template <typename Solver>
void trace_solver(const std::string_view name, const std::string_view mode, Solver& solver,
                  const sssp::Vertex source, const sssp::Vertex goal,
                  const std::size_t vertex_count, const double measured_seconds) {
  std::vector<TimedEvent> events;
  std::vector<std::uint8_t> seen(vertex_count, 0);
  std::mutex event_mutex;
  const auto trace_start = Clock::now();
  solver.set_trace_callback([&](const sssp::SearchEvent& event) {
    const std::scoped_lock lock(event_mutex);
    const auto bit = static_cast<std::uint8_t>(1U << static_cast<unsigned>(event.kind));
    if ((seen.at(event.vertex) & bit) != 0) {
      return;
    }
    seen[event.vertex] = static_cast<std::uint8_t>(seen[event.vertex] | bit);
    const double elapsed = std::chrono::duration<double>(Clock::now() - trace_start).count();
    events.push_back({elapsed, event});
  });

  std::vector<sssp::Vertex> path;
  std::size_t reachable = 0;
  if (mode == "path") {
    path = solver.solve(source, goal).path;
    reachable = path.empty() ? 0 : 1;
  } else {
    const auto distances = solver.solve_all(source);
    reachable = static_cast<std::size_t>(
        std::count_if(distances.begin(), distances.end(),
                      [](const double value) { return value != sssp::infinity; }));
  }
  solver.set_trace_callback({});

  std::ranges::sort(events, {}, &TimedEvent::elapsed_seconds);
  std::cout << "RUN|" << mode << '|' << name << '|' << measured_seconds << '|' << events.size()
            << '|' << reachable << '\n';
  if (!path.empty()) {
    std::cout << "PATH|" << mode << '|' << name;
    for (const sssp::Vertex vertex : path) {
      std::cout << '|' << vertex;
    }
    std::cout << '\n';
  }
  for (std::size_t index = 0; index < events.size(); ++index) {
    const TimedEvent& timed = events[index];
    std::cout << "TRACE|" << mode << '|' << name << '|' << index << '|' << timed.elapsed_seconds
              << '|' << event_name(timed.event.kind) << '|' << timed.event.vertex << '|'
              << timed.event.distance << '\n';
  }
}

template <typename Solver>
void run_one(const std::string_view name, const std::string_view mode, Solver& solver,
             const sssp::Vertex source, const sssp::Vertex goal, const std::size_t vertex_count) {
  const double seconds = benchmark(solver, mode, source, goal);
  trace_solver(name, mode, solver, source, goal, vertex_count, seconds);
}

void print_graph(const VisualGraph& visual) {
  std::cout << "META|" << kWidth << '|' << kHeight << '|' << visual.source << '|' << visual.goal
            << '|' << visual.graph.vertex_count() << '|' << visual.graph.edge_count() << '|'
            << kIterations << '\n';
  for (sssp::Vertex vertex = 0; vertex < visual.coordinates.size(); ++vertex) {
    const auto [x, y] = visual.coordinates[vertex];
    std::cout << "NODE|" << vertex << '|' << x << '|' << y << '\n';
  }
  for (const RoadSegment& segment : visual.segments) {
    std::cout << "EDGE|" << segment.from << '|' << segment.to << '\n';
  }
}

} // namespace

int main(const int argc, const char* argv[]) {
  if (argc != 2 || (std::string_view(argv[1]) != "path" && std::string_view(argv[1]) != "full")) {
    std::cerr << "usage: sssp_visualization_trace <path|full>\n";
    return EXIT_FAILURE;
  }

  try {
    const std::string_view mode = argv[1];
    const VisualGraph visual = make_visual_graph();
    print_graph(visual);
    std::cout << std::setprecision(17);

    validation::DijkstraSolver dijkstra_sequential(visual.graph);
    validation::ParallelDijkstraSolver dijkstra_parallel(visual.graph);
    sssp::SequentialSolver bmssp_sequential(visual.graph);
    sssp::ParallelSolver bmssp_parallel(visual.graph);

    run_one("dijkstra-sequential", mode, dijkstra_sequential, visual.source, visual.goal,
            visual.graph.vertex_count());
    run_one("bmssp-sequential", mode, bmssp_sequential, visual.source, visual.goal,
            visual.graph.vertex_count());
    run_one("dijkstra-parallel", mode, dijkstra_parallel, visual.source, visual.goal,
            visual.graph.vertex_count());
    run_one("bmssp-parallel", mode, bmssp_parallel, visual.source, visual.goal,
            visual.graph.vertex_count());
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
