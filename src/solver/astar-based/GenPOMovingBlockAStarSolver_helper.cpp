#include "solver/astar-based/GenPOMovingBlockAStarSolver.hpp"

#include <algorithm>
#include <bits/ranges_algo.h>
#include <cstddef>
#include <functional>

bool cda_rail::solver::astar_based::GreedySimulatorState::operator==(
    const GreedySimulatorState& other) const {
  return train_edges == other.train_edges && ttd_orders == other.ttd_orders &&
         vertex_orders == other.vertex_orders &&
         stop_positions == other.stop_positions;
}

bool cda_rail::solver::astar_based::GreedySimulatorState::operator>(
    const GreedySimulatorState& other) const {
  auto get_obj = [](const auto& edges) {
    return std::ranges::fold_left(
        edges | std::views::transform([](const auto& e) { return e.size(); }),
        0.0, std::plus<>{});
  };

  return get_obj(train_edges) > get_obj(other.train_edges);
}

std::size_t
std::hash<cda_rail::solver::astar_based::GreedySimulatorState>::operator()(
    const cda_rail::solver::astar_based::GreedySimulatorState& state)
    const noexcept {
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
