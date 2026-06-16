#include "simulator/GreedyHeuristic.hpp"

#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "GeneralHelper.hpp"
#include "simulator/GeneralSimulator.hpp"
#include "simulator/GreedySimulator.hpp"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <iterator>
#include <ranges>
#include <utility>
#include <vector>

/**
 * @brief Computes the braking-time heuristic for a train.
 *
 * @param tr The train identifier.
 * @param simulator The simulator providing train and network data.
 * @param tr_exit_time The time at which the train exits the network.
 * @param braking_time The time at which braking begins.
 * @param braking_distance The distance over which the train brakes.
 *
 * @return The braking-time heuristic value, typically representing time lost to braking constraints.
 */
double cda_rail::simulator::simple_braking_time_heuristic(
    size_t tr, const cda_rail::simulator::GreedySimulator& simulator,
    double tr_exit_time, double braking_time, double braking_distance) {
  /**
   * This heuristic how much time was lost due to braking, i.e., the minimal
   * traveling time from the braking point minus the actual time spent. The
   * result will usually be negative.
   *
   * @param tr The train for which the heuristic is calculated.
   * @param simulator The simulator instance containing the train and its edges.
   * @param tr_exit_time The time at which the train exits the network.
   * @param braking_time time at which the train starts braking
   * @param braking_distance distance over which the train brakes
   *
   * @return The heuristic value, which is the difference between the time spent
   * braking and the minimal traveling time from the braking point.
   */

  if (braking_time < 0 && braking_distance < 0) {
    return 0.0; // No braking time, no heuristic value
  }
  assert(braking_time >= 0);
  assert(braking_distance >= 0);
  assert(tr_exit_time >= 0);
  const auto& tr_edges = simulator.get_train_edges_of_tr(tr);
  assert(!tr_edges.empty());

  const auto& train =
      simulator.get_instance()->get_const_train_list().get_train(tr);
  double ret_val = braking_time - tr_exit_time;
  double len     = 0.0;
  for (auto it = tr_edges.rbegin();
       (len <= braking_distance) && (it != tr_edges.rend()); ++it) {
    const auto& edge =
        simulator.get_instance()->get_const_network().get_edge(*it);
    const auto& speed_limit = std::min(edge.max_speed, train.get_max_speed());
    const auto  rel_len     = std::min(edge.length, braking_distance - len);
    ret_val += rel_len / speed_limit;
    len += edge.length;
  }
  return ret_val;
}

/**
 * @brief Estimates remaining time for a train to exit the network at maximum speed.
 *
 * Computes the estimated time for a train to traverse all scheduled stops and reach the exit
 * vertex, assuming travel at maximum speed. Optionally enforces earliest departure constraints
 * at stops and the network exit.
 *
 * @param tr The train identifier.
 * @param simulator The simulator instance providing train state and network data.
 * @param tr_exit_time The baseline exit time from the network.
 * @param braking_time_heuristic A braking-time heuristic value to adjust the exit time estimate.
 * @param consider_earliest_exit If true, enforces earliest departure times at stops and at the network exit.
 *
 * @return A `RemainingTimeHeuristicResult` struct with:
 *   - `feasible`: true if a valid path to the exit vertex exists; false if reachable stops or exit vertex cannot be reached.
 *   - `remaining_exit_time`: the estimated time remaining until the train fully exits, adjusted for the braking heuristic.
 *   - `average_remaining_stop_delay`: the average delay (excess arrival time over service time) across all scheduled stops.
 */
cda_rail::simulator::RemainingTimeHeuristicResult
cda_rail::simulator::simple_remaining_time_heuristic(
    size_t tr, const cda_rail::simulator::GreedySimulator& simulator,
    double tr_exit_time, double braking_time_heuristic,
    bool consider_earliest_exit) {
  /**
   * This heuristic calculates the remaining time for a train to exit the
   * network. It is assumed that the train will travel at its maximum speed.
   * Acceleration and deceleration are not considered in this heuristic.
   *
   * @param tr The train for which the heuristic is calculated.
   * @param simulator The simulator instance containing the train and its edges.
   * @param tr_exit_time The time at which the train exits the network.
   * @param braking_time_heuristic The heuristic value for braking time, usually
   * negative
   * @param late_stop_possible Indicates if the train can stop later than
   * planned.
   * @param late_exit_possible Indicates if the train can exit later than
   * planned.
   * @param consider_earliest_exit If true, the heuristic will consider the
   * earliest exit time of each station and exit point.
   *
   * @return A pair containing:
   * - bool: indicates if a valid timetable can still be achieved
   * - double: the estimated remaining time for the train to exit the network
   */

  const double rel_exit_time =
      std::max(tr_exit_time + braking_time_heuristic, 0.0);
  double heuristic_exit_time = rel_exit_time;
  double average_stop_delay  = 0.0;

  const auto& tr_edges    = simulator.get_train_edges_of_tr(tr);
  const auto& tr_schedule = simulator.get_instance()->get_const_schedule(tr);
  const auto& tr_stops    = tr_schedule.get_stops();
  const auto& tr_obj =
      simulator.get_instance()->get_const_train_list().get_train(tr);

  if (tr_edges.empty()) {
    heuristic_exit_time =
        std::max(heuristic_exit_time, tr_schedule.get_entry_time());
  } else {
    // Check if the train is already at the exit vertex
    const auto& last_edge =
        simulator.get_instance()->get_const_network().get_edge(tr_edges.back());
    if (tr_schedule.get_exit_vertex() == last_edge.target &&
        tr_stops.size() == simulator.get_stop_positions_of_tr(tr).size()) {
      // Train has reached the exit vertex, no further time needed
      return {.feasible                     = true,
              .remaining_exit_time          = 0.0,
              .average_remaining_stop_delay = 0.0};
    }
  }

  const auto first_next_stop = simulator.get_stop_positions_of_tr(tr).size();

  // Initial position of the train
  auto start_edges =
      tr_edges.empty()
          ? simulator.get_instance()->get_const_network().out_edges(
                tr_schedule.get_entry_vertex())
          : cda_rail::index_set{tr_edges.back()};
  bool include_first_edge = tr_edges.empty();

  for (size_t next_stop = first_next_stop; next_stop < tr_stops.size();
       ++next_stop) {
    // Quickest path to next station
    const auto&         next_station = tr_stops.at(next_stop).get_station();
    cda_rail::index_set next_station_tracks;
    const auto&         stop_tracks =
        simulator.get_instance()->get_stop_tracks(tr, next_station.name);
    next_station_tracks.reserve(stop_tracks.size());
    for (const auto& track : stop_tracks | std::ranges::views::keys) {
      next_station_tracks.insert(track);
    }

    if (next_station_tracks.empty()) {
      return {.feasible                     = false,
              .remaining_exit_time          = cda_rail::INF,
              .average_remaining_stop_delay = cda_rail::INF};
    }
    auto const time_diff =
        simulator.get_instance()
            ->get_const_network()
            .shortest_path_length_between_edge_sets(
                start_edges, next_station_tracks, include_first_edge, true,
                tr_obj.get_max_speed());
    if (!time_diff.has_value()) {
      return {.feasible                     = false,
              .remaining_exit_time          = cda_rail::INF,
              .average_remaining_stop_delay = cda_rail::INF};
    }
    heuristic_exit_time += time_diff.value();

    average_stop_delay +=
        relu(heuristic_exit_time - tr_stops.at(next_stop).get_service_time());

    // Stop train
    heuristic_exit_time += tr_stops.at(next_stop).get_service_duration();
    if (consider_earliest_exit) {
      heuristic_exit_time = std::max(
          heuristic_exit_time, tr_stops.at(next_stop).get_earliest_departure());
    }

    // Initialize next iteration
    start_edges        = std::move(next_station_tracks);
    include_first_edge = false;
  }

  if (!tr_stops.empty()) {
    average_stop_delay /= static_cast<double>(
        tr_stops.size()); // Use known station number, since this is used as
                          // objective difference
  }

  // Move to exit vertex
  auto const time_diff =
      simulator.get_instance()
          ->get_const_network()
          .shortest_path_length_between_edge_and_vertex_set(
              start_edges, {tr_schedule.get_exit_vertex()}, include_first_edge,
              true, tr_obj.get_max_speed());
  if (!time_diff.has_value()) {
    return {.feasible                     = false,
            .remaining_exit_time          = cda_rail::INF,
            .average_remaining_stop_delay = cda_rail::INF};
  }

  heuristic_exit_time += time_diff.value();
  heuristic_exit_time +=
      tr_obj.get_length() /
      tr_obj.get_max_speed(); // Only left after fully leaving the network
  if (consider_earliest_exit) {
    heuristic_exit_time =
        std::max(heuristic_exit_time, tr_schedule.get_exit_time());
  }

  return {.feasible = true,
          .remaining_exit_time =
              heuristic_exit_time - rel_exit_time + braking_time_heuristic,
          .average_remaining_stop_delay = average_stop_delay};
}

/**
 * @brief Combines braking-time and remaining-time heuristics into a single objective-value difference for a train.
 *
 * Computes the objective-value difference as remaining-exit-time plus the station-delay weight multiplied by average-remaining-stop-delay.
 *
 * @param braking_time_heuristic_type Type of braking-time heuristic variant to use.
 * @param remaining_time_heuristic_type Type of remaining-time heuristic variant to use.
 * @param consider_earliest_exit Whether to enforce earliest departure and exit constraints.
 *
 * @return `HeuristicResult` containing feasibility status and the computed objective-value difference.
 */
cda_rail::simulator::HeuristicResult cda_rail::simulator::greedy_heuristic(
    BrakingTimeHeuristicType   braking_time_heuristic_type,
    RemainingTimeHeuristicType remaining_time_heuristic_type, size_t tr,
    const GreedySimulator& simulator, double tr_exit_time, double braking_time,
    double braking_distance, bool consider_earliest_exit) {
  const double bt_val =
      braking_time_heuristic(braking_time_heuristic_type, tr, simulator,
                             tr_exit_time, braking_time, braking_distance);
  const auto [feasible, remaining_exit_time, average_remaining_stop_delay] =
      remaining_time_heuristic(remaining_time_heuristic_type, tr, simulator,
                               tr_exit_time, bt_val, consider_earliest_exit);
  return {.feasible = feasible,
          .objective_value_difference =
              remaining_exit_time +
              (simulator.get_instance()->get_station_delay_weight() *
               average_remaining_stop_delay)};
}

/**
 * @brief Computes a weighted-sum objective-value difference for all trains.
 *
 * @param braking_time_heuristic_type Braking-time heuristic type.
 * @param remaining_time_heuristic_type Remaining-time heuristic type.
 * @param simulator The greedy simulator.
 * @param sim_results Simulation results with exit times and braking parameters.
 * @param consider_earliest_exit Whether to enforce earliest departure and exit times.
 *
 * @return HeuristicResult with feasibility flag (true if all trains feasible) and
 * objective difference (weighted sum across all trains).
 *
 * @throws cda_rail::exceptions::ConsistencyException If result sizes do not match train count.
 * @throws cda_rail::exceptions::InvalidInputException If the simulation failed.
 */
cda_rail::simulator::HeuristicResult cda_rail::simulator::full_greedy_heuristic(
    BrakingTimeHeuristicType   braking_time_heuristic_type,
    RemainingTimeHeuristicType remaining_time_heuristic_type,
    const GreedySimulator& simulator, const SimulatorResults& sim_results,
    bool consider_earliest_exit) {
  const auto train_count =
      simulator.get_instance()->get_const_train_list().size();
  if (sim_results.exit_times.size() != train_count ||
      sim_results.braking_times.size() != train_count ||
      sim_results.braking_distances.size() != train_count) {
    throw cda_rail::exceptions::ConsistencyException(
        "SimulatorResults size does not match simulator train count.");
  }
  if (!sim_results.success) {
    // this should never be reached
    throw cda_rail::exceptions::InvalidInputException(
        "SimulatorResults indicate unsuccessful simulation, heuristic cannot "
        "be calculated.");
  }
  bool   feas = true;
  double obj  = 0.0;
  for (size_t tr = 0; tr < train_count; ++tr) {
    const auto [feas_tr, obj_tr] = greedy_heuristic(
        braking_time_heuristic_type, remaining_time_heuristic_type, tr,
        simulator, sim_results.exit_times.at(tr),
        sim_results.braking_times.at(tr), sim_results.braking_distances.at(tr),
        consider_earliest_exit);
    feas = feas && feas_tr;
    obj += simulator.get_instance()->get_train_weights().at(tr) * obj_tr;
  }
  return {.feasible = feas, .objective_value_difference = obj};
}
