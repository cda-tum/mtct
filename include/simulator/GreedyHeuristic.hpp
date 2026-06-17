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

/**
 * @brief Computes the braking-time heuristic for a train.
 *
 * @param tr The train identifier.
 * @param simulator The simulator providing train and network data.
 * @param tr_exit_time The time at which the train exits the network.
 * @param braking_time The time at which braking begins.
 * @param braking_distance The distance over which the train brakes.
 *
 * @return The braking-time heuristic value, typically representing time lost to
 * braking constraints.
 */
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
/**
 * @brief Combines braking-time and remaining-time heuristics into a single
 * objective-value difference for a train.
 *
 * Computes the objective-value difference as remaining-exit-time plus the
 * station-delay weight multiplied by average-remaining-stop-delay.
 *
 * @param braking_time_heuristic_type Type of braking-time heuristic variant to
 * use.
 * @param remaining_time_heuristic_type Type of remaining-time heuristic variant
 * to use.
 * @param consider_earliest_exit Whether to enforce earliest departure and exit
 * constraints.
 *
 * @return `HeuristicResult` containing feasibility status and the computed
 * objective-value difference.
 */
[[nodiscard]] HeuristicResult
greedy_heuristic(BrakingTimeHeuristicType   braking_time_heuristic_type,
                 RemainingTimeHeuristicType remaining_time_heuristic_type,
                 size_t tr, const GreedySimulator& simulator,
                 double tr_exit_time, double braking_time,
                 double braking_distance, bool consider_earliest_exit);

/**
 * @brief Computes a weighted-sum objective-value difference for all trains.
 *
 * @param braking_time_heuristic_type Braking-time heuristic type.
 * @param remaining_time_heuristic_type Remaining-time heuristic type.
 * @param simulator The greedy simulator.
 * @param sim_results Simulation results with exit times and braking parameters.
 * @param consider_earliest_exit Whether to enforce earliest departure and exit
 * times.
 *
 * @return HeuristicResult with feasibility flag (true if all trains feasible)
 * and objective difference (weighted sum across all trains).
 *
 * @throws cda_rail::exceptions::ConsistencyException If result sizes do not
 * match train count.
 * @throws cda_rail::exceptions::InvalidInputException If the simulation failed.
 */
[[nodiscard]] HeuristicResult
full_greedy_heuristic(BrakingTimeHeuristicType   braking_time_heuristic_type,
                      RemainingTimeHeuristicType remaining_time_heuristic_type,
                      const GreedySimulator&     simulator,
                      const SimulatorResults&    sim_results,
                      bool                       consider_earliest_exit);

} // namespace cda_rail::simulator
