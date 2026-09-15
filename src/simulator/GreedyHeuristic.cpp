#include "simulator/GreedyHeuristic.hpp"

#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "GeneralHelper.hpp"
#include "simulator/GeneralSimulator.hpp"
#include "simulator/GreedySimulator.hpp"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <ranges>
#include <utility>
#include <vector>

cda_rail::simulator::RemainingTimeHeuristicResult
cda_rail::simulator::simple_remaining_time_heuristic(
    size_t tr, const cda_rail::simulator::GreedySimulator& simulator,
    double tr_exit_time, bool consider_earliest_exit) {
  double const relevant_tr_time    = std::max(tr_exit_time, 0.0);
  double       heuristic_exit_time = relevant_tr_time;
  double       average_stop_delay  = 0.0;

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
          : simulator.get_instance()->get_const_network().get_successors(
                tr_edges.back());
  bool include_first_edge = true;

  if (start_edges.empty()) {
    // dead-end which is not exit vertex
    return {.feasible                     = false,
            .remaining_exit_time          = cda_rail::INF,
            .average_remaining_stop_delay = cda_rail::INF};
  }

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
  if (consider_earliest_exit) {
    heuristic_exit_time =
        std::max(heuristic_exit_time, tr_schedule.get_exit_time());
  }

  return {.feasible            = true,
          .remaining_exit_time = heuristic_exit_time - relevant_tr_time,
          .average_remaining_stop_delay = average_stop_delay};
}

cda_rail::simulator::HeuristicResult cda_rail::simulator::greedy_heuristic(
    RemainingTimeHeuristicType remaining_time_heuristic_type, size_t tr,
    const GreedySimulator& simulator, double tr_exit_time,
    bool consider_earliest_exit) {
  const auto [feasible, remaining_exit_time, average_remaining_stop_delay] =
      remaining_time_heuristic(remaining_time_heuristic_type, tr, simulator,
                               tr_exit_time, consider_earliest_exit);
  return {.feasible = feasible,
          .objective_value_difference =
              remaining_exit_time +
              (simulator.get_instance()->get_station_delay_weight() *
               average_remaining_stop_delay)};
}

cda_rail::simulator::HeuristicResult cda_rail::simulator::full_greedy_heuristic(
    RemainingTimeHeuristicType remaining_time_heuristic_type,
    const GreedySimulator& simulator, const SimulatorResults& sim_results,
    bool consider_earliest_exit) {
  const auto train_count =
      simulator.get_instance()->get_const_train_list().size();
  if (sim_results.exit_times.size() != train_count) {
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
    const auto [feas_tr, obj_tr] =
        greedy_heuristic(remaining_time_heuristic_type, tr, simulator,
                         sim_results.exit_times.at(tr), consider_earliest_exit);
    if (!feas_tr) {
      // shortcut on infeasibility
      return {.feasible = false, .objective_value_difference = INF};
    }
    feas = feas && feas_tr;
    obj += simulator.get_instance()->get_train_weights().at(tr) * obj_tr;
  }
  return {.feasible = feas, .objective_value_difference = obj};
}
