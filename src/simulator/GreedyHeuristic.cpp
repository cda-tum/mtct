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

  const auto running_times = simulator.get_instance()->minimum_running_times(
      tr, start_edges, include_first_edge, heuristic_exit_time, first_next_stop,
      consider_earliest_exit);
  if (!running_times.feasible) {
    return {.feasible                     = false,
            .remaining_exit_time          = cda_rail::INF,
            .average_remaining_stop_delay = cda_rail::INF};
  }

  for (size_t i = 0; i < running_times.stop_arrivals.size(); ++i) {
    average_stop_delay +=
        relu(running_times.stop_arrivals.at(i) -
             tr_stops.at(first_next_stop + i).get_service_time());
  }
  if (!tr_stops.empty()) {
    average_stop_delay /= static_cast<double>(
        tr_stops.size()); // Use known station number, since this is used as
                          // objective difference
  }

  heuristic_exit_time = running_times.exit_arrival;
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
