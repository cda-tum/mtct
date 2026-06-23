#pragma once

#include "CustomExceptions.hpp"
#include "GeneralSimulator.hpp"
#include "GreedySimulator.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace cda_rail::simulator {
enum class RemainingTimeHeuristicType : std::uint8_t { Zero = 0, Simple = 1 };
constexpr std::string
remaining_time_heuristic_type_to_string(RemainingTimeHeuristicType const type) {
  switch (type) {
  case RemainingTimeHeuristicType::Zero:
    return "Zero";
  case RemainingTimeHeuristicType::Simple:
    return "Simple";
  default:
    throw cda_rail::exceptions::ConsistencyException(
        "Unknown remaining-time heuristic type");
  }
}

// --------------------------
// Objective Value
/**
 * @brief Computes the objective value based on train exit times and stop times.
 *
 * @param simulator Greedy simulator providing the problem instance.
 * @param tr_exit_times Exit times per train.
 * @param stop_times Stop times per train and stop.
 * @return double The computed objective value.
 */

[[nodiscard]] inline double
objective_val(const GreedySimulator&                  simulator,
              const std::vector<double>&              tr_exit_times,
              const std::vector<std::vector<double>>& stop_times) {
  return simulator.get_instance()->get_objective_val(tr_exit_times, stop_times,
                                                     false);
}

// ----------------------------
// Remaining Time Heuristic
// ----------------------------

struct RemainingTimeHeuristicResult {
  bool   feasible;
  double remaining_exit_time;
  double average_remaining_stop_delay;
};
// Remaining time heuristics for A*
/**
 * @brief Computes the simple remaining-time heuristic for one train.
 *
 * @param tr Train index.
 * @param simulator Greedy simulator providing route and instance data.
 * @param tr_exit_time Current simulated exit time.
 * @param consider_earliest_exit Whether earliest-exit constraints are enforced.
 * @return Remaining-time heuristic result for the train.
 */
[[nodiscard]] RemainingTimeHeuristicResult
simple_remaining_time_heuristic(size_t tr, const GreedySimulator& simulator,
                                double tr_exit_time,
                                bool   consider_earliest_exit);

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
 * @param consider_earliest_exit Whether to consider the train's earliest exit
 * time.
 *
 * @return A `RemainingTimeHeuristicResult` containing feasibility status,
 * remaining exit time, and average remaining stop delay.
 */
[[nodiscard]] inline RemainingTimeHeuristicResult
remaining_time_heuristic(RemainingTimeHeuristicType type, size_t tr,
                         const GreedySimulator& simulator, double tr_exit_time,
                         bool consider_earliest_exit) {
  switch (type) {
  case RemainingTimeHeuristicType::Zero:
    return {.feasible                     = true,
            .remaining_exit_time          = 0.0,
            .average_remaining_stop_delay = 0.0};
  case RemainingTimeHeuristicType::Simple:
    return simple_remaining_time_heuristic(tr, simulator, tr_exit_time,
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
 * @param remaining_time_heuristic_type Type of remaining-time heuristic variant
 * to use.
 * @param tr Train index.
 * @param simulator Greedy simulator.
 * @param tr_exit_time Current simulated exit time of the train.
 * @param consider_earliest_exit Whether to enforce earliest departure and exit
 * constraints.
 *
 * @return `HeuristicResult` containing feasibility status and the computed
 * objective-value difference.
 */
[[nodiscard]] HeuristicResult
greedy_heuristic(RemainingTimeHeuristicType remaining_time_heuristic_type,
                 size_t tr, const GreedySimulator& simulator,
                 double tr_exit_time, bool consider_earliest_exit);

/**
 * @brief Computes a weighted-sum objective-value difference for all trains.
 *
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
full_greedy_heuristic(RemainingTimeHeuristicType remaining_time_heuristic_type,
                      const GreedySimulator&     simulator,
                      const SimulatorResults&    sim_results,
                      bool                       consider_earliest_exit);

} // namespace cda_rail::simulator
