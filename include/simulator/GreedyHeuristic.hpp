#pragma once

#include "GreedySimulator.hpp"

#include <cstddef>
#include <utility>
#include <vector>

namespace cda_rail::simulator {
enum class BrakingTimeHeuristicType : std::uint8_t { Simple };
enum class RemainingTimeHeuristicType : std::uint8_t { Zero, Simple };

// Braking time heuristics for A*
[[nodiscard]] double simple_braking_time_heuristic(
    size_t tr, const GreedySimulator& simulator,
    const std::vector<int>&                    tr_exit_times,
    const std::vector<std::pair<int, double>>& braking_times);

[[nodiscard]] double braking_time_heuristic(
    BrakingTimeHeuristicType type, size_t tr, const GreedySimulator& simulator,
    const std::vector<int>&                    tr_exit_times,
    const std::vector<std::pair<int, double>>& braking_times) {
  switch (type) {
  case BrakingTimeHeuristicType::Simple:
    return simple_braking_time_heuristic(tr, simulator, tr_exit_times,
                                         braking_times);
  }
  // This should never be reached
  throw cda_rail::exceptions::ConsistencyException(
      "This code should not have been reachable...");
};

// Remaining time heuristics for A*

} // namespace cda_rail::simulator
