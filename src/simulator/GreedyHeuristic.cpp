#include "simulator/GreedyHeuristic.hpp"

#include "Definitions.hpp"
#include "simulator/GreedySimulator.hpp"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <iterator>
#include <utility>
#include <vector>

double cda_rail::simulator::simple_braking_time_heuristic(
    size_t tr, const cda_rail::simulator::GreedySimulator& simulator,
    int tr_exit_time, const std::pair<int, double>& braking_time) {
  /**
   * This heuristic how much time was lost due to braking, i.e., the minimal
   * traveling time from the braking point minus the actual time spent. The
   * result will usually be negative.
   *
   * @param tr The train for which the heuristic is calculated.
   * @param simulator The simulator instance containing the train and its edges.
   * @param tr_exit_time The time at which the train exits the network.
   * @param braking_time A pair containing the time at which the train started
   * braking and the distance it has traveled while braking.
   *
   * @return The heuristic value, which is the difference between the time spent
   * braking and the minimal traveling time from the braking point.
   */

  if (braking_time.first < 0 && braking_time.second < 0) {
    return 0.0; // No braking time, no heuristic value
  }
  assert(braking_time.first >= 0);
  assert(braking_time.second >= 0);
  assert(tr_exit_time >= 0);
  const auto& tr_edges = simulator.get_train_edges_of_tr(tr);
  assert(!tr_edges.empty());

  const auto& train = simulator.get_instance()->get_train_list().get_train(tr);
  double      ret_val = static_cast<double>(braking_time.first) -
                   static_cast<double>(tr_exit_time);
  double len = 0.0;
  for (auto it = tr_edges.rbegin();
       (len <= braking_time.second) && (it != tr_edges.rend()); ++it) {
    const auto& edge        = simulator.get_instance()->const_n().get_edge(*it);
    const auto& speed_limit = std::min(edge.max_speed, train.max_speed);
    const auto  rel_len     = std::min(edge.length, braking_time.second - len);
    ret_val += rel_len / speed_limit;
    len += edge.length;
  }
  return ret_val;
}

std::pair<bool, double> cda_rail::simulator::simple_remaining_time_heuristic(
    size_t tr, const cda_rail::simulator::GreedySimulator& simulator,
    int tr_exit_time, double braking_time_heuristic, bool late_stop_possible,
    bool late_exit_possible, bool consider_earliest_exit) {
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
      std::max(static_cast<double>(tr_exit_time) + braking_time_heuristic, 0.0);
  double heuristic_exit_time = rel_exit_time;

  const auto& tr_edges    = simulator.get_train_edges_of_tr(tr);
  const auto& tr_schedule = simulator.get_instance()->get_schedule(tr);
  const auto& tr_stops    = tr_schedule.get_stops();
  const auto& tr_obj = simulator.get_instance()->get_train_list().get_train(tr);

  if (tr_edges.empty()) {
    heuristic_exit_time =
        std::max(heuristic_exit_time,
                 static_cast<double>(tr_schedule.get_t_0_range().first));
  } else {
    // Check if the train is already at the exit vertex
    const auto& last_edge =
        simulator.get_instance()->const_n().get_edge(tr_edges.back());
    if (tr_schedule.get_exit() == last_edge.target &&
        tr_stops.size() == simulator.get_stop_positions_of_tr(tr).size()) {
      // Train has reached the exit vertex, no further time needed
      return {true, 0.0};
    }
  }

  const auto first_next_stop = simulator.get_stop_positions_of_tr(tr).size();

  // Initial position of the train
  auto start_edges =
      tr_edges.empty()
          ? std::vector<size_t>{simulator.get_instance()->const_n().out_edges(
                tr_schedule.get_entry())}
          : std::vector<size_t>{tr_edges.back()};
  bool include_first_edge = tr_edges.empty();

  bool feasible = true;
  for (size_t next_stop = first_next_stop; next_stop < tr_stops.size();
       ++next_stop) {
    // Quickest path to next station
    const auto& next_station_name = tr_stops.at(next_stop).get_station_name();
    std::vector<size_t> next_station_tracks;
    const auto&         stop_tracks =
        simulator.get_instance()->get_stop_tracks(tr, next_station_name);
    next_station_tracks.reserve(stop_tracks.size());
    std::transform(stop_tracks.begin(), stop_tracks.end(),
                   std::back_inserter(next_station_tracks),
                   [](const auto& track_pair) { return track_pair.first; });

    if (next_station_tracks.empty()) {
      return {false, cda_rail::INF};
    }
    heuristic_exit_time += simulator.get_instance()
                               ->const_n()
                               .shortest_path_between_sets(
                                   start_edges, next_station_tracks, true,
                                   include_first_edge, true, tr_obj.max_speed)
                               .value_or(cda_rail::INF);

    if (!late_stop_possible &&
        heuristic_exit_time > tr_stops.at(next_stop).get_begin_range().second) {
      feasible = false;
    }

    // Stop train
    if (consider_earliest_exit) {
      heuristic_exit_time = std::max(
          heuristic_exit_time,
          static_cast<double>(tr_stops.at(next_stop).get_begin_range().first));
    }
    heuristic_exit_time += tr_stops.at(next_stop).get_min_stopping_time();
    if (consider_earliest_exit) {
      heuristic_exit_time = std::max(
          heuristic_exit_time,
          static_cast<double>(tr_stops.at(next_stop).get_end_range().first));
    }

    // Initialize next iteration
    start_edges        = std::move(next_station_tracks);
    include_first_edge = false;
  }

  // Move to exit vertex
  heuristic_exit_time += simulator.get_instance()
                             ->const_n()
                             .shortest_path_between_sets(
                                 start_edges, {tr_schedule.get_exit()}, false,
                                 include_first_edge, true, tr_obj.max_speed)
                             .value_or(cda_rail::INF);
  heuristic_exit_time +=
      tr_obj.length /
      tr_obj.max_speed; // Only left after fully leaving the network
  if (!late_exit_possible &&
      heuristic_exit_time > tr_schedule.get_t_n_range().second) {
    feasible = false;
  }
  if (consider_earliest_exit) {
    heuristic_exit_time =
        std::max(heuristic_exit_time,
                 static_cast<double>(tr_schedule.get_t_n_range().first));
  }

  return {feasible, heuristic_exit_time - rel_exit_time};
}
