#pragma once

#include "CustomExceptions.hpp"
#include "GeneralHelper.hpp"
#include "GeneralSimulator.hpp"
#include "GreedySimulator.hpp"

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace cda_rail::simulator {
enum class BrakingTimeHeuristicType : std::uint8_t { Simple = 0 };
enum class RemainingTimeHeuristicType : std::uint8_t { Zero = 0, Simple = 1 };

// --------------------------
// Objective Value
// --------------------------

[[nodiscard]] inline double
objective_val(const GreedySimulator&                  simulator,
              const std::vector<double>&              tr_exit_times,
              const std::vector<std::vector<double>>& stop_times) {
  return simulator.get_instance()->get_objective_val(tr_exit_times, stop_times,
                                                     false);
}

// --------------------------
// Braking Time Heuristic
// --------------------------

[[nodiscard]] double
simple_braking_time_heuristic(size_t tr, const GreedySimulator& simulator,
                              double tr_exit_time, double braking_time,
                              double braking_distance);

[[nodiscard]] inline double
braking_time_heuristic(BrakingTimeHeuristicType type, size_t tr,
                       const GreedySimulator& simulator, double tr_exit_time,
                       double braking_time, double braking_distance) {
  switch (type) {
  case BrakingTimeHeuristicType::Simple:
    return simple_braking_time_heuristic(tr, simulator, tr_exit_time,
                                         braking_time, braking_distance);
  }
  // This should never be reached
  throw cda_rail::exceptions::ConsistencyException(
      "This code should not have been reachable...");
};

// ----------------------------
// Remaining Time Heuristic
// ----------------------------

struct RemainingTimeHeuristicResult {
  bool   feasible;
  double remaining_exit_time;
  double average_remaining_stop_delay;
};
// Remaining time heuristics for A*
[[nodiscard]] RemainingTimeHeuristicResult simple_remaining_time_heuristic(
    size_t tr, const GreedySimulator& simulator, double tr_exit_time,
    double braking_time_heuristic, bool consider_earliest_exit);

[[nodiscard]] inline RemainingTimeHeuristicResult
remaining_time_heuristic(RemainingTimeHeuristicType type, size_t tr,
                         const GreedySimulator& simulator, double tr_exit_time,
                         double braking_time_heuristic,
                         bool   consider_earliest_exit) {
  switch (type) {
  case RemainingTimeHeuristicType::Zero:
    return {.feasible                     = true,
            .remaining_exit_time          = 0.0,
            .average_remaining_stop_delay = 0.0};
  case RemainingTimeHeuristicType::Simple:
    return simple_remaining_time_heuristic(tr, simulator, tr_exit_time,
                                           braking_time_heuristic,
                                           consider_earliest_exit);
  }
  // This should never be reached
  throw cda_rail::exceptions::ConsistencyException(
      "This code should not have been reachable...");
}

// --------------------------
// Full Heuristic
// --------------------------

struct HeuristicResult {
  bool   feasible;
  double objective_value_difference;
};
[[nodiscard]] HeuristicResult
greedy_heuristic(BrakingTimeHeuristicType   braking_time_heuristic_type,
                 RemainingTimeHeuristicType remaining_time_heuristic_type,
                 size_t tr, const GreedySimulator& simulator,
                 double tr_exit_time, double braking_time,
                 double braking_distance, bool consider_earliest_exit);

[[nodiscard]] HeuristicResult
full_greedy_heuristic(BrakingTimeHeuristicType   braking_time_heuristic_type,
                      RemainingTimeHeuristicType remaining_time_heuristic_type,
                      const GreedySimulator&     simulator,
                      const SimulatorResults&    sim_results,
                      bool                       consider_earliest_exit);

} // namespace cda_rail::simulator
