#pragma once

#include "CustomExceptions.hpp"
#include "GeneralSimulator.hpp"
#include "GreedySimulator.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace cda_rail::simulator {
enum class BrakingTimeHeuristicType : std::uint8_t { Simple = 0 };
enum class RemainingTimeHeuristicType : std::uint8_t { Zero = 0, Simple = 1 };

// --------------------------
// Objective Value
/**
 * @brief Computes the objective value based on train exit times and stop times.
 *
 * @return double The computed objective value.
 */

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

/**
 * @brief Executes the selected braking-time heuristic.
 *
 * @param type The braking-time heuristic implementation to use.
 * @return double Computed braking-time estimate.
 */
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

/**
 * @brief Computes a remaining-time heuristic estimate using the selected
 * strategy.
 *
 * Dispatches to the appropriate remaining-time heuristic implementation based
 * on the specified type. The `Zero` strategy returns a trivial feasible result
 * with zero remaining time and delay. The `Simple` strategy delegates to the
 * simple remaining-time heuristic.
 *
 * @param type The remaining-time heuristic strategy to apply.
 * @param tr Train index.
 * @param simulator Reference to the greedy simulator.
 * @param tr_exit_time The train's exit time.
 * @param braking_time_heuristic The estimated braking time.
 * @param consider_earliest_exit Whether to consider the train's earliest exit
 * time.
 *
 * @return A `RemainingTimeHeuristicResult` containing feasibility status,
 * remaining exit time, and average remaining stop delay.
 */
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
