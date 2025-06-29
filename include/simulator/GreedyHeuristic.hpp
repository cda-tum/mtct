#pragma once

#include "CustomExceptions.hpp"
#include "GreedySimulator.hpp"

#include <cstddef>
#include <cstdint>
#include <utility>

namespace cda_rail::simulator {
enum class BrakingTimeHeuristicType : std::uint8_t { Simple };
enum class RemainingTimeHeuristicType : std::uint8_t { Zero, Simple };

// Braking time heuristics for A*
[[nodiscard]] double
simple_braking_time_heuristic(size_t tr, const GreedySimulator& simulator,
                              int                           tr_exit_time,
                              const std::pair<int, double>& braking_time);

[[nodiscard]] inline double
braking_time_heuristic(BrakingTimeHeuristicType type, size_t tr,
                       const GreedySimulator& simulator, int tr_exit_time,
                       const std::pair<int, double>& braking_time) {
  switch (type) {
  case BrakingTimeHeuristicType::Simple:
    return simple_braking_time_heuristic(tr, simulator, tr_exit_time,
                                         braking_time);
  }
  // This should never be reached
  throw cda_rail::exceptions::ConsistencyException(
      "This code should not have been reachable...");
};

// Remaining time heuristics for A*

} // namespace cda_rail::simulator
