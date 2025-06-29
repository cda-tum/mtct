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
[[nodiscard]] std::pair<bool, double> simple_remaining_time_heuristic(
    size_t tr, const GreedySimulator& simulator, int tr_exit_time,
    double braking_time_heuristic, bool late_stop_possible,
    bool late_exit_possible, bool consider_earliest_exit);

[[nodiscard]] inline std::pair<bool, double>
remaining_time_heuristic(RemainingTimeHeuristicType type, size_t tr,
                         const GreedySimulator& simulator, int tr_exit_time,
                         double braking_time_heuristic, bool late_stop_possible,
                         bool late_exit_possible, bool consider_earliest_exit) {
  switch (type) {
  case RemainingTimeHeuristicType::Zero:
    return {true, 0.0};
  case RemainingTimeHeuristicType::Simple:
    return simple_remaining_time_heuristic(
        tr, simulator, tr_exit_time, braking_time_heuristic, late_stop_possible,
        late_exit_possible, consider_earliest_exit);
  }
  // This should never be reached
  throw cda_rail::exceptions::ConsistencyException(
      "This code should not have been reachable...");
};

} // namespace cda_rail::simulator
