#include "simulator/GeneralSimulator.hpp"
#include "solver/astar-based/GenPOMovingBlockAStarSolver.hpp"

#include <cstddef>
#include <functional>
#include <numeric>
#include <ranges>

std::size_t std::hash<cda_rail::simulator::SimulatorState>::operator()(
    const cda_rail::simulator::SimulatorState& state) const noexcept {
  // Based on boost::hash_combine implementation

  size_t seed         = 0;
  auto   hash_combine = [&seed](size_t h) {
    seed ^= h + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  };

  hash_combine(std::hash<size_t>{}(state.train_edges.size()));
  for (const auto& vec : state.train_edges) {
    hash_combine(std::hash<size_t>{}(vec.size()));
    for (const size_t v : vec) {
      hash_combine(std::hash<size_t>{}(v));
    }
  }
  hash_combine(std::hash<size_t>{}(state.ttd_orders.size()));
  for (const auto& vec : state.ttd_orders) {
    hash_combine(std::hash<size_t>{}(vec.size()));
    for (const size_t v : vec) {
      hash_combine(std::hash<size_t>{}(v));
    }
  }
  hash_combine(std::hash<size_t>{}(state.vertex_orders.size()));
  for (const auto& vec : state.vertex_orders) {
    hash_combine(std::hash<size_t>{}(vec.size()));
    for (const size_t v : vec) {
      hash_combine(std::hash<size_t>{}(v));
    }
  }
  hash_combine(std::hash<size_t>{}(state.stop_positions.size()));
  for (const auto& vec : state.stop_positions) {
    hash_combine(std::hash<size_t>{}(vec.size()));
    for (const double v : vec) {
      hash_combine(std::hash<double>{}(v));
    }
  }

  return seed;
}
