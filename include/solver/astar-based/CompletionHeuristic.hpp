#pragma once

#include "probleminstances/GeneralProblemInstance.hpp"
#include "simulator/GeneralSimulator.hpp"

#include <optional>

namespace cda_rail::solver::astar_based {
/**
 * @brief Completes the routes and stops of a state heuristically.
 *
 * Every train continues from the end of its route, or from its entry vertex
 * if it has no route yet, along quickest paths, ignoring its dynamics, through
 * its remaining stations to its exit vertex. At every station, it stops at
 * the end of a station track on which it fits, and from which it can continue
 * to its next station, respectively its exit. Where there are several such
 * platforms, the trains take turns in using them in the order of their entry
 * times, i.e., a train takes the one used least recently, counting the stops
 * of the state. A train whose route ends at a valid stop position without a
 * stop there continues beyond it. The routes, stops and orders of the state
 * are kept.
 *
 * @param instance The problem instance.
 * @param state The state to complete.
 * @return The state with complete routes and stops, or `std::nullopt` if some
 * train cannot be routed that way.
 */
[[nodiscard]] std::optional<simulator::SimulatorState> complete_routes(
    const instances::GeneralProblemInstanceWithScheduleAndRoutes& instance,
    simulator::SimulatorState                                     state);

/**
 * @brief Completes a state to a final state heuristically.
 *
 * The routes and stops are completed by complete_routes, and the orders that
 * the state leaves open are decided first come, first served, see
 * simulator::GeneralSimulator::first_come_first_served_state.
 *
 * @param simulator The simulator deciding the orders, whose state is
 * replaced.
 * @param state The state to complete.
 * @param dt See simulator::GeneralSimulator::simulate.
 * @param late_entry_possible See simulator::GeneralSimulator::simulate.
 * @param limit_speed_by_leaving_edges See
 * simulator::GeneralSimulator::simulate.
 * @return The final state, or `std::nullopt` if the routes cannot be
 * completed or the trains do not all reach their exits.
 */
[[nodiscard]] std::optional<simulator::SimulatorState>
complete_state(simulator::GeneralSimulator& simulator,
               simulator::SimulatorState state, double dt,
               bool late_entry_possible, bool limit_speed_by_leaving_edges);
} // namespace cda_rail::solver::astar_based
