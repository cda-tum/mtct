#include "simulator/GreedySimulator.hpp"

#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "EOMHelper.hpp"
#include "plog/Log.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <limits>
#include <optional>
#include <string>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

std::pair<bool, std::vector<int>>
cda_rail::simulator::GreedySimulator::simulate(
    int dt, bool late_entry_possible, bool late_exit_possible,
    bool late_stop_possible, bool limit_speed_by_leaving_edges,
    bool debug_input) const {
  /**
   * This function simulates train movements as specified by the member
   * variables. It returns a vector of doubles denoting the travel times of each
   * of the trains The (possibly weighted) sum of these values is usually the
   * current objective value.
   *
   * @param dt: The time step for the simulation. This is the time in seconds.
   * Default: 6s
   * @param late_entry_possible: If true, trains can enter the network later
   * than scheduled, otherwise the settings are infeasible. Default: false
   * @param late_exit_possible: If true, trains can exit the network later than
   * scheduled, otherwise the settings are infeasible. Default: false
   * @param late_stop_possible: If true, trains can stop later than scheduled,
   * otherwise the settings are infeasible. Default: false
   *
   * @return: A pair containing a boolean indicating whether the simulation was
   * successful and a vector of doubles with the exit times of each train.
   */

  cda_rail::initalize_plog(debug_input);

  // Initialize return values
  std::vector<int> exit_times(
      instance->get_timetable().get_train_list().size(),
      0); // 0 indicates that a train has not entered the network yet

  // Find first time step
  int min_T              = std::numeric_limits<int>::max();
  int last_entering_time = std::numeric_limits<int>::min();
  for (size_t tr = 0; tr < instance->get_timetable().get_train_list().size();
       ++tr) {
    min_T = std::min(
        min_T,
        instance->get_timetable().get_schedule(tr).get_t_0_range().first);
    last_entering_time = std::max(
        last_entering_time,
        instance->get_timetable().get_schedule(tr).get_t_0_range().second);
  }
  int max_T = instance->get_timetable().max_t();

  // Initialize variables to keep track of positions and velocities
  std::vector<std::pair<double, double>> train_positions(
      instance->get_timetable().get_train_list().size(),
      {-1.0, -1.0}); // {rear, front} positions
  std::vector<double> train_velocities(
      instance->get_timetable().get_train_list().size(), -1.0); // velocities
  std::unordered_set<size_t> trains_in_network;
  std::unordered_set<size_t> trains_left;
  std::unordered_set<size_t> trains_finished_simulating;
  std::vector<int>           tr_stop_until(
      instance->get_timetable().get_train_list().size(),
      -1); // time until train stops in station
  std::vector<std::optional<size_t>> tr_next_stop_id(
      instance->get_timetable()
          .get_train_list()
          .size()); // next scheduled stop position
  std::vector<int> vertex_headways(instance->const_n().number_of_vertices(),
                                   0); // headways for vertices

  const auto trains_on_edges = tr_on_edges();

  int  t                   = min_T;
  bool continue_simulation = true;

  PLOGD << "Starting simulation from time " << min_T
        << " to approximately time " << max_T;

  while (continue_simulation) {
    PLOGD << "----------------------------";
    PLOGD << "Current time: " << t;

    bool movement_detected = false;

    for (const auto& tr : trains_in_network) {
      const auto& train_object = instance->get_train_list().get_train(tr);

      if (trains_finished_simulating.contains(tr)) {
        PLOGD << train_object.name << " skipped.";
        continue;
      }

      // Calculate MAs for every train
      const auto& tr_schedule = instance->get_timetable().get_schedule(tr);
      const auto  h =
          std::max({tr_schedule.get_t_n_range().first - t,
                    vertex_headways.at(tr_schedule.get_exit()) - t, 0});

      const auto [tr_ma, tr_v1] =
          get_ma_and_maxv(tr, train_velocities, tr_next_stop_id.at(tr), h, dt,
                          train_positions, trains_in_network, trains_left,
                          trains_on_edges, limit_speed_by_leaving_edges);
      PLOGD << train_object.name << " positioned at "
            << train_positions.at(tr).second
            << " has MA: " << train_positions.at(tr).second + tr_ma
            << " and max velocity: " << tr_v1;
      PLOGD << "h = " << h;
      auto tr_new_speed =
          std::min(tr_v1, get_v1_from_ma(train_velocities.at(tr), tr_ma,
                                         train_object.deceleration, dt));
      if (tr_new_speed < V_MIN) {
        tr_new_speed = 0.0; // Train is stopped
      }

      // Move trains
      if (move_train(tr, train_velocities.at(tr), tr_new_speed, tr_ma, dt,
                     train_positions)) {
        movement_detected = true;
      }
      train_velocities.at(tr) = tr_new_speed;
      PLOGD << "At time " << t << ", " << train_object.name << " moved to "
            << train_positions.at(tr).second << " with speed " << tr_new_speed
            << " and MA "
            << train_positions.at(tr).second +
                   cda_rail::braking_distance(tr_new_speed,
                                              train_object.deceleration);
    }

    // Update rear positions of trains
    update_rear_positions(train_positions);

    std::vector<size_t> trains_to_remove;
    for (const auto& tr : trains_in_network) {
      if (trains_finished_simulating.contains(tr)) {
        continue;
      }

      // Remove trains that have left the network
      const auto tr_status = tr_reached_end(tr, train_positions);
      if (tr_status == DestinationType::Network) {
        trains_to_remove.emplace_back(tr);
        trains_left.insert(tr);
        trains_finished_simulating.insert(tr);
        exit_times.at(tr) = t;
        PLOGD << "At time " << t << ", "
              << instance->get_train_list().get_train(tr).name
              << " left the network.";
        continue;
      } else if (tr_status == DestinationType::Edge) {
        trains_finished_simulating.insert(tr);
        exit_times.at(tr) = t;
        PLOGD << "At time " << t << ", "
              << instance->get_train_list().get_train(tr).name
              << " reached the end of its route on an edge within the network.";
        continue;
      } else if (tr_status == DestinationType::Station) {
        assert(tr_next_stop_id.at(tr).has_value());
        const auto& last_stop =
            instance->get_timetable().get_schedule(tr).get_stops().at(
                tr_next_stop_id.at(tr).value());
        exit_times.at(tr)      = std::max(last_stop.get_end_range().first,
                                          t + last_stop.get_min_stopping_time());
        tr_next_stop_id.at(tr) = {};
        trains_finished_simulating.insert(tr);
        PLOGD << "At time " << t << ", "
              << instance->get_train_list().get_train(tr).name
              << " reached the end of its route at station "
              << last_stop.get_station_name() << ", stopping until "
              << exit_times.at(tr);
        continue;
      }

      // Update stop information if a train has reached its next stop
      if (tr_next_stop_id.at(tr).has_value()) {
        // TODO
      }
    }

    // Remove trains that have left the network
    for (const auto& tr : trains_to_remove) {
      trains_in_network.erase(tr);
    }

    // Check for new trains entering the network
    const auto [tr_to_enter_success, tr_to_enter] = get_entering_trains(
        t, trains_in_network, trains_left, late_entry_possible);
    if (!tr_to_enter_success) {
      PLOGD
          << "Simulation failed: Not all trains can enter the network at time "
          << t;
      return {false, exit_times};
    }
    for (const auto& tr : tr_to_enter) {
      const auto& train_schedule = instance->get_timetable().get_schedule(tr);
      const auto& entry_vertex =
          instance->const_n().get_vertex(train_schedule.get_entry());
      if (vertex_headways.at(train_schedule.get_entry()) > t) {
        PLOGD << "At time " << t << ", "
              << instance->get_train_list().get_train(tr).name
              << " cannot enter the network at " << entry_vertex.name
              << " due to vertex headway constraints until time "
              << vertex_headways.at(train_schedule.get_entry());
      } else if (!is_ok_to_enter(tr, train_positions, train_velocities,
                                 trains_in_network, trains_on_edges)) {
        PLOGD << "At time " << t << ", "
              << instance->get_train_list().get_train(tr).name
              << " cannot enter the network at " << entry_vertex.name
              << "due to moving authority constraints constraints.";
      } else {
        trains_in_network.insert(tr);
        vertex_headways.at(train_schedule.get_entry()) =
            t + static_cast<int>(std::ceil(entry_vertex.headway));
        train_positions.at(tr) = {
            -instance->get_train_list().get_train(tr).length,
            0.0}; // Initialize positions
        train_velocities.at(tr) = train_schedule.get_v_0();
        if (!train_schedule.get_stops().empty()) {
          tr_next_stop_id.at(tr) = 0;
        }
        movement_detected = true;
        PLOGD << "At time " << t << ", "
              << instance->get_train_list().get_train(tr).name
              << " entered the network at " << entry_vertex.name;
        PLOGD << "New entry blocked until time "
              << vertex_headways.at(train_schedule.get_entry());
      }
    }

    // Check if the end state can still be reached
    // TODO

    // Check if all trains have reached their destination
    // TODO

    // Check if there might be a deadlock situation
    // TODO

    // Update time
    t += dt;

    continue_simulation = t <= max_T;
  }

  return {true, exit_times};
}

bool cda_rail::simulator::GreedySimulator::check_consistency() const {
  if (!instance->check_consistency(false)) {
    return false;
  }

  // All vectors are of correct size
  if (train_edges.size() != instance->get_timetable().get_train_list().size()) {
    return false;
  }
  if (ttd_orders.size() != ttd_sections.size()) {
    return false;
  }
  if (vertex_orders.size() != instance->const_n().number_of_vertices()) {
    return false;
  }
  if (stop_positions.size() !=
      instance->get_timetable().get_train_list().size()) {
    return false;
  }

  // All edges are valid
  for (const auto& edges : train_edges) {
    for (const auto& edge : edges) {
      if (!instance->const_n().has_edge(edge)) {
        return false;
      }
    }
  }

  // All edges are valid successors of each other
  for (const auto& edges : train_edges) {
    const auto& s = edges.size();
    for (size_t i = 0; s > 1 && i < s - 1; ++i) {
      if (!instance->const_n().is_valid_successor(edges[i], edges[i + 1])) {
        return false;
      }
    }
  }

  // If not empty, the first edge of each train must be an entry edge
  for (size_t train_id = 0; train_id < train_edges.size(); ++train_id) {
    if (!train_edges[train_id].empty()) {
      const auto& first_edge_id = train_edges[train_id].front();
      const auto& first_edge    = instance->const_n().get_edge(first_edge_id);
      if (first_edge.source !=
          instance->get_timetable().get_schedule(train_id).get_entry()) {
        return false;
      }
    }
  }

  // ttd_orders only contains valid train indices
  for (const auto& orders : ttd_orders) {
    for (const auto& train_id : orders) {
      if (!instance->get_timetable().get_train_list().has_train(train_id)) {
        return false;
      }
    }
  }

  // vertex_orders only contains valid train indices
  for (const auto& orders : vertex_orders) {
    for (const auto& train_id : orders) {
      if (!instance->get_timetable().get_train_list().has_train(train_id)) {
        return false;
      }
    }
  }

  // entry orders only contain trains that are entering the network through the
  // respective vertex
  for (size_t vertex_id = 0; vertex_id < vertex_orders.size(); ++vertex_id) {
    for (const auto& train_id : vertex_orders[vertex_id]) {
      if (vertex_id !=
          instance->get_timetable().get_schedule(train_id).get_entry()) {
        return false;
      }
    }
  }

  // Every train can have at most as many stop positions as scheduled stops
  for (size_t train_id = 0;
       train_id < instance->get_timetable().get_train_list().size();
       ++train_id) {
    const auto train_schedule_size =
        instance->get_timetable().get_schedule(train_id).get_stops().size();
    if (stop_positions.at(train_id).size() > train_schedule_size) {
      return false; // Too many stop positions for the train
    }
    // stop_positions.at(train_id) must be non-negative and sorted
    if (std::any_of(stop_positions.at(train_id).begin(),
                    stop_positions.at(train_id).end(),
                    [](double pos) { return pos < 0; })) {
      return false; // Negative stop position found
    }
    if (!std::is_sorted(stop_positions.at(train_id).begin(),
                        stop_positions.at(train_id).end())) {
      return false; // Stop positions are not sorted
    }
  }

  return true;
}

double cda_rail::simulator::GreedySimulator::braking_distance(size_t tr,
                                                              double v) const {
  /**
   * Calculates the braking distance for a train with id `tr` at velocity `v`.
   *
   * @param tr: The id of the train for which the braking distance is
   * calculated.
   * @param v: The velocity at which the braking distance is calculated.
   *
   * @return: The braking distance for the train at the given velocity.
   */
  if (!instance->get_timetable().get_train_list().has_train(tr)) {
    throw cda_rail::exceptions::TrainNotExistentException(tr);
  }
  if (v < -EPS) {
    throw cda_rail::exceptions::InvalidInputException(
        "Velocity must be non-negative.");
  }
  if (v < 0) {
    return 0.0; // No braking distance if the train is not moving
  }
  return cda_rail::braking_distance(
      v, instance->get_train_list().get_train(tr).deceleration);
}

std::pair<bool, std::unordered_set<size_t>>
cda_rail::simulator::GreedySimulator::get_entering_trains(
    int t, const std::unordered_set<size_t>& tr_present,
    const std::unordered_set<size_t>& tr_left, bool late_entry_possible) const {
  /**
   * This function checks which trains are scheduled to enter the network at
   * time `t` or later, and returns a vector of their indices.
   *
   * @param t: The time at which to check for entering trains.
   * @param tr_present: A set of train indices that are currently present in the
   * network.
   * @param tr_left: A set of train indices that have left the network.
   * @param late_entry_possible: If true, trains can enter the network later
   * than scheduled.
   *
   * @return: A pair containing a boolean indicating whether the operation was
   * successful, and a set of train indices that are scheduled to enter the
   * network at time `t` or later.
   */

  std::unordered_set<size_t> entering_trains;
  for (size_t tr = 0; tr < instance->get_timetable().get_train_list().size();
       ++tr) {
    // Check if the train is already present in the network
    if (tr_present.contains(tr) || tr_left.contains(tr)) {
      continue; // Train is already present, skip it
    }
    // Check if the train is scheduled to enter the network at time t or later
    const auto& schedule = instance->get_timetable().get_schedule(tr);
    if (t < schedule.get_t_0_range().first) {
      continue; // Train is not scheduled to enter at this time
    }

    // Check if the train scheduled to enter before tr is already in the network
    const auto& entry_node  = schedule.get_entry();
    const auto& entry_order = vertex_orders.at(entry_node);
    // Find index of tr in the entry order (if it exists)
    const auto it = std::find(entry_order.begin(), entry_order.end(), tr);
    // If tr is not in the entry order, it means it is scheduled to enter
    if (it == entry_order.end()) {
      continue; // Train is not scheduled to enter at all
    }

    if (!late_entry_possible && t > schedule.get_t_0_range().second) {
      // Train can no longer enter the network
      return {false, {tr}};
    }

    // If tr is not the first train in the entry order, check if previous trains
    // are already in the network
    if (it != entry_order.begin()) {
      // Check if any previous train in the entry order is already in the
      // network
      const auto& prev_tr    = *(it - 1);
      const auto& prev_entry = instance->get_schedule(prev_tr).get_entry();
      if (((entry_node != prev_entry) || !tr_present.contains(prev_tr)) &&
          !tr_left.contains(prev_tr)) {
        // Previous train is not in the network (or left if needed), so tr
        // cannot enter yet
        continue;
      }
    }

    // If we reach here, the train is scheduled to enter the network at time t
    entering_trains.insert(tr);
  }
  return {true, entering_trains};
}

std::vector<double>
cda_rail::simulator::GreedySimulator::edge_milestones(size_t tr) const {
  /**
   * This function returns the individual milestones, i.e., the distance on the
   * individual route to each edges starting point. The last value is the
   * distance to the exit node.
   *
   * @param tr: The id of the train for which the milestones are calculated.
   * @return: A vector of doubles with the milestones for each edge of the
   * train.
   */
  if (!instance->get_timetable().get_train_list().has_train(tr)) {
    throw cda_rail::exceptions::TrainNotExistentException(tr);
  }
  const auto& edges = train_edges.at(tr);
  if (edges.empty()) {
    return {}; // No edges, no milestones
  }
  std::vector<double> milestones;
  milestones.reserve(edges.size());
  milestones.emplace_back(0.0); // First milestone is always 0
  for (const auto& edge_id : edges) {
    const auto& edge = instance->const_n().get_edge(edge_id);
    milestones.emplace_back(milestones.back() + edge.length);
  }
  return milestones;
}

std::tuple<bool, std::pair<bool, bool>, std::pair<double, double>>
cda_rail::simulator::GreedySimulator::get_position_on_route_edge(
    size_t tr, const std::pair<double, double>& pos, size_t edge_number,
    std::vector<double> milestones) const {
  /**
   * This function returns the position of a train on a specific edge of its
   * route.
   *
   * @param tr: The id of the train for which the position is calculated.
   * @param pos: The position of the train on its route.
   * @param edge_number: The index of the edge in the train's route.
   * @param milestones: A vector of doubles representing the possibly
   * precomputed milestones.
   *
   * @return: A tuple containing:
   * - a boolean indicating whether the train is on the edge,
   * - a pair of booleans indicating whether the trains rear and/or front is on
   * the edge,
   * - a pair of doubles with the rear and front positions of the train on the
   * edge.
   */
  if (!instance->get_timetable().get_train_list().has_train(tr)) {
    throw cda_rail::exceptions::TrainNotExistentException(tr);
  }

  if (milestones.empty()) {
    milestones = edge_milestones(tr);
  }
  if (train_edges.at(tr).size() + 1 != milestones.size()) {
    throw cda_rail::exceptions::ConsistencyException(
        "Milestones size does not match number of edges for train " +
        std::to_string(tr) + ". Expected " +
        std::to_string(train_edges.at(tr).size() + 1) + ", got " +
        std::to_string(milestones.size()) + ".");
  }
  if (edge_number >= train_edges.at(tr).size()) {
    throw cda_rail::exceptions::InvalidInputException(
        "Edge number out of bounds for train " + std::to_string(tr) +
        ". Train has only " + std::to_string(train_edges.at(tr).size()) +
        " edges.");
  }

  const std::pair<double, double> milestone_pair = {
      milestones[edge_number], milestones[edge_number + 1]};
  const bool is_on_edge   = (pos.second > milestone_pair.first + EPS &&
                           pos.first < milestone_pair.second - EPS);
  const bool rear_on_edge = is_on_edge && (pos.first >= milestone_pair.first);
  const bool front_on_edge =
      is_on_edge && (pos.second <= milestone_pair.second);

  return {is_on_edge,
          {rear_on_edge, front_on_edge},
          {std::max(0.0, pos.first - milestone_pair.first),
           std::min(milestone_pair.second - milestone_pair.first,
                    pos.second - milestone_pair.first)}};
}

bool cda_rail::simulator::GreedySimulator::is_on_ttd(
    size_t tr, size_t ttd, const std::pair<double, double>& pos,
    TTDOccupationType occupation_type) const {
  /**
   * This function checks if a train is on a TTD section at a given position.
   *
   * @param tr: The id of the train to check.
   * @param ttd: The index of the TTD section to check.
   * @param pos: The position of the train on its route.
   * @param occupation_type: The type of occupation to check for. It can be one
   * of the following:
   * - OnlyOccupied: The train must be on the TTD section.
   * - OnlyBehind: The train must be behind the TTD section.
   * - OccupiedOrBehind: The train can be either on or behind the TTD section.
   *
   * @return: A boolean indicating whether the train is on the TTD section.
   */
  if (ttd >= ttd_sections.size()) {
    throw cda_rail::exceptions::InvalidInputException(
        "TTD index out of bounds: " + std::to_string(ttd) +
        ". Maximum index is " + std::to_string(ttd_sections.size() - 1) + ".");
  }
  const auto  milestones         = edge_milestones(tr);
  const auto& ttd_section        = ttd_sections[ttd];
  bool        potentially_behind = false;
  for (const auto& edge_id : ttd_section) {
    if (!is_on_route(tr, edge_id)) {
      continue; // Train is not on this edge
    }
    [[maybe_unused]] const auto [occ_status, detailed_occ_status, pos_on_edge] =
        get_position_on_edge(tr, pos, edge_id, milestones);
    if (occ_status &&
        (occupation_type == TTDOccupationType::OnlyOccupied ||
         occupation_type == TTDOccupationType::OccupiedOrBehind)) {
      return true; // Train is on the edge
    }
    if (occ_status && occupation_type == TTDOccupationType::OnlyBehind) {
      return false; // Train is on the edge, but we only want to check if it is
                    // truly behind
    }
    if (!occ_status &&
        (occupation_type == TTDOccupationType::OnlyBehind ||
         occupation_type == TTDOccupationType::OccupiedOrBehind) &&
        pos_on_edge.first >= instance->const_n().get_edge(edge_id).length) {
      potentially_behind =
          true; // Train is either behind ttd section or on a later ttd edge
      if (occupation_type == TTDOccupationType::OccupiedOrBehind) {
        return true;
      }
    }
  }
  return potentially_behind; // Train is not on the TTD section
}

std::vector<std::unordered_set<size_t>>
cda_rail::simulator::GreedySimulator::tr_on_edges() const {
  /**
   * This function returns a vector of unordered sets, where each set
   * contains the indices of trains that are routed on a specific edge.
   */

  std::vector<std::unordered_set<size_t>> trains_on_edges(
      instance->const_n().number_of_edges());
  for (size_t tr = 0; tr < instance->get_timetable().get_train_list().size();
       ++tr) {
    const auto& edges = train_edges.at(tr);
    for (const auto& edge_id : edges) {
      trains_on_edges[edge_id].insert(tr);
    }
  }
  return trains_on_edges;
}

std::optional<size_t>
cda_rail::simulator::GreedySimulator::get_ttd(size_t edge_id) const {
  if (!instance->const_n().has_edge(edge_id)) {
    throw cda_rail::exceptions::EdgeNotExistentException(edge_id);
  }
  for (size_t ttd_index = 0; ttd_index < ttd_sections.size(); ++ttd_index) {
    const auto& ttd_section = ttd_sections[ttd_index];
    if (std::find(ttd_section.begin(), ttd_section.end(), edge_id) !=
        ttd_section.end()) {
      return ttd_index; // Found the TTD section containing the edge
    }
  }
  return {}; // Edge is not part of any TTD section
}

bool cda_rail::simulator::GreedySimulator::is_ok_to_enter(
    size_t tr, const std::vector<std::pair<double, double>>& train_positions,
    const std::vector<double>&                     train_velocities,
    const std::unordered_set<size_t>&              trains_in_network,
    const std::vector<std::unordered_set<size_t>>& tr_on_edges) const {
  /**
   * This function checks if it is ok for a train to enter the network, i.e., if
   * all of its initial braking distance is cleared.
   *
   * @param tr: The id of the train to check.
   * @param train_positions: A vector of pairs containing the rear and front
   * positions of each train in the network.
   * @param trains_in_network: A set of train ids that are currently in the
   * network.
   */

  const auto v0         = instance->get_timetable().get_schedule(tr).get_v_0();
  const auto bd         = braking_distance(tr, v0);
  const auto milestones = edge_milestones(tr);
  for (size_t i = 0; i < train_edges.at(tr).size() && milestones[i] + EPS < bd;
       ++i) {
    const auto& edge_id          = train_edges.at(tr).at(i);
    const auto& potential_trains = tr_on_edges.at(edge_id);
    for (const auto& other_tr : potential_trains) {
      if (other_tr == tr || !trains_in_network.contains(other_tr)) {
        continue; // Skip the train itself or trains that are not in the network
      }
      const auto& other_pos = train_positions.at(other_tr);
      [[maybe_unused]] const auto [occ, det_occ, det_pos] =
          get_position_on_edge(other_tr, other_pos, edge_id);
      if (occ && det_pos.first <= bd - milestones[i] + EPS) {
        return false; // Other train is occupying the edge within the braking
                      // distance
      }
    }

    const auto ttd_sec = get_ttd(edge_id);
    if (!ttd_sec.has_value()) {
      continue;
    }
    const auto& ttd_order = ttd_orders.at(ttd_sec.value());
    const auto  ttd_pos   = std::find(ttd_order.begin(), ttd_order.end(), tr);
    if (ttd_pos == ttd_order.begin()) {
      continue; // Train is the first in the TTD order, no other train can block
                // it
    }
    const auto& other_tr  = *(ttd_pos - 1); // Previous train in the TTD order
    const auto& other_pos = train_positions.at(other_tr);
    if (!trains_in_network.contains(other_tr) ||
        !is_behind_ttd(other_tr, ttd_sec.value(), other_pos)) {
      return false; // Other train is occupying the TTD section
    }
  }
  return true;
}

double cda_rail::simulator::GreedySimulator::max_displacement(
    const cda_rail::Train& train, double v_0, int dt) const {
  /**
   * Calculate the maximum displacement of a train in a given time step.
   *
   * @param train: The train for which the maximum displacement is calculated.
   * @param v_0: The initial velocity of the train in m/s.
   * @param dt: The time step in seconds.
   *
   * @return: The maximum displacement of the train in the given time step.
   */
  return cda_rail::max_braking_pos_after_dt_linear_movement(
      v_0, train.max_speed, train.acceleration, train.deceleration, dt);
}

double cda_rail::simulator::GreedySimulator::get_absolute_distance_ma(
    size_t tr, double max_displacement,
    const std::vector<std::pair<double, double>>&  train_positions,
    const std::vector<double>&                     train_velocities,
    const std::unordered_set<size_t>&              trains_in_network,
    const std::unordered_set<size_t>&              trains_left,
    const std::vector<std::unordered_set<size_t>>& tr_on_edges) const {
  /**
   * Calculate the shortest distance of tr to the following train.
   *
   * @param tr: The id of the train for which the distance is calculated.
   * @param max_displacement: The maximum displacement of the train (no search
   * after this distance).
   * @param train_positions: A vector of pairs containing the rear and front
   * positions of each train in the network.
   * @param train_velocities: A vector containing the velocities of each train.
   * This is needed to check for collisions with trains traveling in the
   * opposite direction.
   * @param trains_in_network: A set of train ids that are currently in the
   * network.
   * @param tr_on_edges: A vector of unordered sets, where each set contains the
   * indices of trains that are routed on a specific edge.
   *
   * @return: The absolute distance of the train to the next train in the
   * network.
   */
  if (!trains_in_network.contains(tr)) {
    throw cda_rail::exceptions::ConsistencyException(
        "Train " + std::to_string(tr) + " is not in the network.");
  }
  if (max_displacement < 0) {
    throw cda_rail::exceptions::InvalidInputException(
        "Maximum displacement must be non-negative.");
  }

  const auto milestones = edge_milestones(tr);
  bool       first_edge = true;
  for (size_t i = 0; i < train_edges.at(tr).size() &&
                     milestones.at(i) + EPS <
                         train_positions.at(tr).second + max_displacement;
       ++i) {
    if (milestones.at(i + 1) <= train_positions.at(tr).second) {
      continue; // The edge is behind the train's front position
    }
    const auto& edge_id = train_edges.at(tr).at(i);

    // First edge is entirely blocked due to TTD section or train travelling in
    // opposite direction
    if (milestones.at(i) >= train_positions.at(tr).second) {
      const auto ttd_sec = get_ttd(edge_id);
      if (ttd_sec.has_value()) {
        bool check_ttd = true;
        if (i >= 1) {
          const auto& prev_edge_id = train_edges.at(tr).at(i - 1);
          const auto  prev_ttd_sec = get_ttd(prev_edge_id);
          if (prev_ttd_sec.has_value() &&
              prev_ttd_sec.value() == ttd_sec.value()) {
            check_ttd = false; // Previous edge is part of the same TTD section,
                               // hence, TTD condition has already been checked
          }
        }
        if (check_ttd) {
          const auto& ttd_order = ttd_orders.at(ttd_sec.value());
          const auto  ttd_pos =
              std::find(ttd_order.begin(), ttd_order.end(), tr);
          if (ttd_pos != ttd_order.begin()) {
            const auto& other_tr =
                *(ttd_pos - 1); // Previous train in the TTD order
            const auto& other_pos = train_positions.at(other_tr);
            if (!trains_left.contains(other_tr) &&
                (!trains_in_network.contains(other_tr) ||
                 !is_behind_ttd(other_tr, ttd_sec.value(), other_pos))) {
              return milestones.at(i) -
                     train_positions.at(tr).second; // Other train is occupying
                                                    // the future TTD section
            }
          }
        }
      }

      const auto reverse_edge_id =
          instance->const_n().get_reverse_edge_index(edge_id);
      if (reverse_edge_id.has_value()) {
        const auto& potential_trains_reverse =
            tr_on_edges.at(reverse_edge_id.value());
        for (const auto& other_tr : potential_trains_reverse) {
          if (other_tr == tr || !trains_in_network.contains(other_tr)) {
            continue; // Skip the train itself or trains that are not in the
                      // network
          }
          [[maybe_unused]] const auto [occ_rev, det_occ_rev, det_pos_rev] =
              get_position_on_edge(
                  other_tr,
                  {train_positions.at(other_tr).first,
                   train_positions.at(other_tr).second +
                       braking_distance(other_tr,
                                        train_velocities.at(other_tr))},
                  reverse_edge_id.value());
          if (occ_rev) {
            // Other train has already been cleared to enter the reverse edge
            return milestones.at(i) - train_positions.at(tr).second;
          }
        }
      }
    }

    // Absolute distance to trains on edge
    double      potential_limit   = milestones.at(i + 1) - milestones.at(i);
    bool        found_other_train = false;
    const auto& potential_trains  = tr_on_edges.at(edge_id);
    for (const auto& other_tr : potential_trains) {
      if (other_tr == tr || !trains_in_network.contains(other_tr)) {
        continue; // Skip the train itself or trains that are not in the network
      }
      const auto& other_pos = train_positions.at(other_tr);
      [[maybe_unused]] const auto [occ, det_occ, det_pos] =
          get_position_on_edge(other_tr, other_pos, edge_id);
      bool check_other_tr = occ;
      if (check_other_tr && first_edge) {
        // Other train could be behind train on the same edge
        [[maybe_unused]] const auto [occ_tr, det_occ_tr, det_pos_tr] =
            get_position_on_route_edge(tr, train_positions.at(tr), i,
                                       milestones);
        if (occ_tr && det_pos_tr.first >= det_pos.second) {
          check_other_tr = false; // Train is not behind the other train
        }
      }
      if (check_other_tr) {
        found_other_train = true;
        potential_limit   = std::min(potential_limit, det_pos.first);
      }
    }
    if (found_other_train) {
      return std::min(
          max_displacement,
          milestones.at(i) + potential_limit -
              train_positions.at(tr).second); // Found a train on the edge
    }
    first_edge = false; // After the first edge, we do not need to check if
                        // other train might be behind
  }
  return max_displacement; // No train found within the maximum displacement
                           // range
}

std::pair<double, double>
cda_rail::simulator::GreedySimulator::get_future_max_speed_constraints(
    size_t tr, const cda_rail::Train& train, double pos, double v_0,
    double max_displacement, int dt, bool also_limit_by_leaving_edges) const {
  /**
   * This function calculates the future maximum speed constraints for a train.
   * If an edge is reachable, the trains speed is restricted directly.
   * Otherwise, future speed restrictions are modelled through restrictions on
   * the trains moving authority to ensure braking well in advance.
   * OBS: The speed restriction can also be induced by the exit velocity of the
   * train.
   *
   * @param tr: The id of the train for which the constraints are calculated.
   * @param train: The train object containing the train's properties.
   * @param pos: The current position of the train on its route.
   * @param v_0: The initial velocity of the train in m/s.
   * @param max_displacement: The maximum displacement of the train in the next
   * time step.
   * @param dt: The time step in seconds.
   * @param also_limit_by_leaving_edges: If true, the speed is limited by the
   * edges the train is leaving, otherwise only by the front of the train.
   *
   * @return: A pair of doubles representing the:
   * - maximum moving authority from the trains current position and
   * - the maximum speed allowed for the next speed
   */

  if (std::abs(pos) < EPS) {
    pos = 0;
  }
  if (std::abs(v_0) < EPS) {
    v_0 = 0;
  }
  if (std::abs(max_displacement) < EPS) {
    max_displacement = 0;
  }

  if (pos < 0) {
    throw cda_rail::exceptions::InvalidInputException(
        "Position must be non-negative.");
  }
  if (v_0 < 0) {
    throw cda_rail::exceptions::InvalidInputException(
        "Initial velocity must be non-negative.");
  }
  if (max_displacement < 0) {
    throw cda_rail::exceptions::InvalidInputException(
        "Maximum displacement must be non-negative.");
  }
  if (dt < 0) {
    throw cda_rail::exceptions::InvalidInputException(
        "Time step must be non-negative.");
  }

  double max_v = std::min(train.max_speed, v_0 + (train.acceleration * dt));
  double ma    = max_displacement;

  const auto milestones = edge_milestones(tr);

  // Check exit
  const auto& last_edge_id = train_edges.at(tr).back();
  const auto& last_edge    = instance->const_n().get_edge(last_edge_id);
  if (pos + max_displacement >= milestones.back()) {
    const auto tr_schedule = instance->get_schedule(tr);
    const bool last_edge_leaves_network =
        (last_edge.target == tr_schedule.get_exit());
    const double last_edge_exit_restriction =
        last_edge_leaves_network ? tr_schedule.get_v_n() : 0;
    std::tie(ma, max_v) = speed_restriction_helper(
        ma, max_v, pos, milestones.back(), v_0, last_edge_exit_restriction,
        train.deceleration, dt);
  }

  for (size_t i = 0; i < train_edges.at(tr).size() &&
                     milestones.at(i) + EPS < pos + max_displacement;
       ++i) {
    if (milestones.at(i + 1) <= pos - train.length) {
      continue; // Train has fully left the edge already
    }
    if (!also_limit_by_leaving_edges && milestones.at(i + 1) <= pos) {
      continue; // Train's front has already left the edge
    }

    const auto& edge_id = train_edges.at(tr).at(i);
    const auto& edge    = instance->const_n().get_edge(edge_id);
    [[maybe_unused]] const auto [occ, det_occ, det_pos] =
        get_position_on_route_edge(tr, {pos - train.length, pos}, i,
                                   milestones);
    if (det_occ.second || (!also_limit_by_leaving_edges && occ)) {
      max_v = std::min(max_v, edge.max_speed); // Train is on the edge
    } else {
      std::tie(ma, max_v) =
          speed_restriction_helper(ma, max_v, pos, milestones.at(i), v_0,
                                   edge.max_speed, train.deceleration, dt);
    }
  }
  return {ma, max_v};
}

std::pair<double, double>
cda_rail::simulator::GreedySimulator::speed_restriction_helper(
    double ma, double max_v, double pos, double vertex_pos, double v_0,
    double v_m, double d, int dt) {
  // Can the train reach the next edge within one time step?
  const auto max_dist = (v_0 + v_m) * dt / 2.0;
  if (pos + max_dist >= vertex_pos) {
    // Train can reach the edge -> limit speed directly
    max_v = std::min(max_v, v_m);
    if (v_m == 0) {
      // Possibly full stop before dt ends
      ma = std::min(ma, vertex_pos - pos);
    }
  } else {
    // Train cannot reach the next edge, so we limit the moving authority
    ma = std::min(ma, vertex_pos + ((v_m * v_m) / (2.0 * d)) - pos);
  }
  return {ma, max_v};
}

double cda_rail::simulator::GreedySimulator::get_next_stop_ma(
    double max_displacement, double pos, std::optional<double> next_stop_pos) {
  /*
   * This function calculates the maximum moving authority to the next stop.
   */

  if (!next_stop_pos.has_value()) {
    // No next stop, return the maximum displacement
    return max_displacement;
  }
  return std::min(max_displacement, next_stop_pos.value() - pos);
}

double cda_rail::simulator::GreedySimulator::get_max_speed_exit_headway(
    size_t tr, const cda_rail::Train& train, double pos, double v_0, int h,
    int dt) const {
  /**
   * This function limits the maximum speed so that the exit headway can still
   * be maintained using the exit velocity.
   *
   * @param tr: The id of the train for which the maximum speed is calculated.
   * @param train: The train object containing the train's properties.
   * @param pos: The current position of the train on its route.
   * @param v_0: The initial velocity of the train in m/s.
   * @param h: The (remaining) exit headway in seconds.
   * @param dt: The time step in seconds.
   *
   * @return: The maximum speed allowed for the next speed step.
   */

  if (std::abs(pos) < EPS) {
    pos = 0;
  }
  if (std::abs(v_0) < EPS) {
    v_0 = 0;
  }
  if (h < 0) {
    throw cda_rail::exceptions::InvalidInputException(
        "Exit headway must be non-negative.");
  }
  if (dt < 0) {
    throw cda_rail::exceptions::InvalidInputException(
        "Time step must be non-negative.");
  }
  if (pos < 0) {
    throw cda_rail::exceptions::InvalidInputException(
        "Position must be non-negative.");
  }
  if (v_0 < 0) {
    throw cda_rail::exceptions::InvalidInputException(
        "Initial velocity must be non-negative.");
  }

  double v_ub = std::min(train.max_speed, v_0 + (train.acceleration * dt));
  double v_lb = std::max(0.0, v_0 - (train.deceleration * dt));

  const auto& last_edge_id = train_edges.at(tr).back();
  const auto& last_edge    = instance->const_n().get_edge(last_edge_id);
  const auto& tr_schedule  = instance->get_schedule(tr);
  if (last_edge.target != tr_schedule.get_exit()) {
    // Train does not leave the network at the end of its route
    return v_ub; // No exit headway to consider
  }

  const auto milestones    = edge_milestones(tr);
  const auto exit_distance = milestones.back() - pos;
  if (h == 0) {
    return v_ub; // No constraint by exit headway, maximal speed ensured by
                 // future speed restriction headway
  }
  v_ub = std::min(v_ub, ((2.0 * exit_distance) / dt) - v_0);
  if (v_ub < v_lb - EPS) {
    throw cda_rail::exceptions::ConsistencyException(
        "v_ub < v_lb, this should not have happened.");
  }

  auto [bool_vub, obj_vub] =
      time_to_exit_objective(v_0, v_ub, tr_schedule.get_v_n(), exit_distance,
                             train.acceleration, train.deceleration, dt);
  auto [bool_vlb, obj_vlb] =
      time_to_exit_objective(v_0, v_lb, tr_schedule.get_v_n(), exit_distance,
                             train.acceleration, train.deceleration, dt);

  if (bool_vub && (obj_vub >= h)) {
    return v_ub;
  }

  // Binary search for the maximum speed that satisfies the exit headway
  // constraint We want to solve exit_time(v_1) >= h (as close to h as possible)
  while (v_ub - v_lb > LINE_SPEED_ACCURACY) {
    const double v_mid = (v_ub + v_lb) / 2.0;
    const auto [bool_vmid, obj_vmid] =
        time_to_exit_objective(v_0, v_mid, tr_schedule.get_v_n(), exit_distance,
                               train.acceleration, train.deceleration, dt);
    if ((obj_vmid < h) || (bool_vlb && !bool_vmid)) {
      v_ub    = v_mid; // We need to decrease the speed
      obj_vub = obj_vmid;
      // bool_vub = bool_vmid; not needed hence not updated
    } else {
      v_lb     = v_mid; // We can still increase the speed
      obj_vlb  = obj_vmid;
      bool_vlb = bool_vmid;
    }
  }

  return (!bool_vlb && obj_vub >= h)
             ? v_ub
             : v_lb; // Return the lower bound as the maximum speed
}

std::pair<bool, double>
cda_rail::simulator::GreedySimulator::time_to_exit_objective(
    double v_0, double v_1, double v_e, double s, double a, double d, int dt) {
  /**
   * This function calculates the time to exit objective value used in the
   * headway constraints calculation.
   *
   * @param v_0: The initial velocity of the train in m/s.
   * @param v_1: The velocity of the train at the end of the time step in m/s.
   * @param v_e: The exit velocity of the train in m/s.
   * @param s: The distance to the exit in m.
   * @param a: The acceleration of the train in m/s^2.
   * @param d: The deceleration of the train in m/s^2.
   * @param dt: The time step in seconds.
   *
   * @return: The time to exit objective value.
   */
  if (std::abs(v_0) < EPS) {
    v_0 = 0;
  }
  if (std::abs(v_1) < EPS) {
    v_1 = 0;
  }
  if (std::abs(s) < EPS) {
    s = 0;
  }
  if (dt <= 0) {
    throw cda_rail::exceptions::InvalidInputException(
        "Time step must be positive.");
  }
  if (v_0 < 0) {
    throw cda_rail::exceptions::InvalidInputException(
        "Initial velocity must be non-negative.");
  }
  if (v_1 < 0) {
    throw cda_rail::exceptions::InvalidInputException(
        "Velocity at the end of the time step must be non-negative.");
  }
  if (v_e < V_MIN) {
    throw cda_rail::exceptions::InvalidInputException(
        "Exit velocity must be positive for simulation.");
  }
  if (s < 0) {
    throw cda_rail::exceptions::InvalidInputException(
        "Distance to the exit must be non-negative.");
  }
  if (a < EPS) {
    throw cda_rail::exceptions::InvalidInputException(
        "Acceleration must be positive.");
  }
  if (d < EPS) {
    throw cda_rail::exceptions::InvalidInputException(
        "Deceleration must be positive.");
  }

  const auto x_1 =
      (v_0 + v_1) * dt / 2.0; // Distance traveled in the first time step
  if (x_1 >= s) {
    return {std::abs(v_1 - v_e) < EPS,
            dt}; // Train reaches the exit in the first time step
  }
  if (v_1 == 0) {
    return {
        true,
        std::numeric_limits<double>::infinity()}; // Train is stopped, can reach
                                                  // exit as late as needed
  }
  const auto distance_remaining = s - x_1;
  if (!possible_by_eom(v_1, v_e, a, d, distance_remaining)) {
    // In this case, we assume that the train accelerates or decelerates as much
    // as possible i.e., at rate -d or a (say p) respectively
    const auto p = (v_1 < v_e) ? a : -d;
    // x(t) = integral_0^t (v_1 + p * t) dt = v_1 * t + 0.5 * p * t^2
    // We need to solve the equation x(t) = distance_remaining (say x) for t
    // Wolframalpha: t = (sqrt(2*p*x + v_1^2) - v_1) / p
    // This is not numerically stable, so we multiply by (sqrt(2*p*x + v_1^2) +
    // v_1) / (sqrt(2*p*x + v_1^2) + v_1) t = (2*p*x + v_1^2 - v_1^2) / (p *
    // (sqrt(2*p*x + v_1^2) + v_1)) Simplifying gives us: t = 2 * x /
    // (sqrt(2*p*x + v_1^2) + v_1)
    return {false,
            ((2.0 * distance_remaining) /
             (std::sqrt((2.0 * p * distance_remaining) + (v_1 * v_1)) + v_1)) +
                static_cast<double>(dt)};
  }
  return {true,
          cda_rail::max_travel_time_stopping_allowed(v_1, v_e, a, d, s - x_1) +
              static_cast<double>(dt)};
}

std::pair<double, double> cda_rail::simulator::GreedySimulator::get_ma_and_maxv(
    size_t tr, const std::vector<double>& train_velocities,
    std::optional<double> next_stop, double h, int dt,
    const std::vector<std::pair<double, double>>&  train_positions,
    const std::unordered_set<size_t>&              trains_in_network,
    const std::unordered_set<size_t>&              trains_left,
    const std::vector<std::unordered_set<size_t>>& tr_on_edges,
    bool also_limit_speed_by_leaving_edges) const {
  const auto& train = instance->get_timetable().get_train_list().get_train(tr);
  double      ma    = max_displacement(train, train_velocities.at(tr), dt);
  ma = get_next_stop_ma(ma, train_positions.at(tr).second, next_stop);
  double max_v;
  ma = get_absolute_distance_ma(tr, ma, train_positions, train_velocities,
                                trains_in_network, trains_left, tr_on_edges);
  std::tie(ma, max_v) = get_future_max_speed_constraints(
      tr, train, train_positions.at(tr).second, train_velocities.at(tr), ma, dt,
      also_limit_speed_by_leaving_edges);
  max_v = std::min(max_v, get_max_speed_exit_headway(
                              tr, train, train_positions.at(tr).second,
                              train_velocities.at(tr), h, dt));

  return {ma, max_v};
}

double cda_rail::simulator::GreedySimulator::get_v1_from_ma(double v_0,
                                                            double ma, double d,
                                                            int dt) {
  /**
   * This function calculates the maximum velocity at the end of a time step
   * without exceeding the moving authority.
   *
   * @param v_0: The initial velocity of the train in m/s.
   * @param ma: The moving authority in m.
   * @param d: The deceleration of the train in m/s^2.
   * @param dt: The time step in seconds.
   *
   * @return: The maximum velocity at the end of the time step in m/s.
   */

  // Assume that a train accelerates linearly during the time step from v_0
  // (given) to v_1 (to be calculated). x_1 = (v_0 + v_1) * dt/2 bd = v_1 * v_1
  // / (2 * d) x_1 + bd = ma Wolframalpha: v_1 = 0.5 *
  // (sqrt((d*dt)^2+4*(2*d*ma-d*dt*v_0)) - d*dt) Set A := d*dt, B :=
  // 2*d*ma-d*dt*v_0) v_1 = 0.5 * (sqrt(A^2 + 4*B) - A)
  if (std::abs(v_0) < EPS) {
    v_0 = 0;
  }
  if (std::abs(ma) < EPS) {
    ma = 0;
  }
  if (std::abs(d) < EPS) {
    throw cda_rail::exceptions::InvalidInputException(
        "Deceleration must be positive.");
  }
  if (v_0 < 0) {
    throw cda_rail::exceptions::InvalidInputException(
        "Initial velocity must be non-negative.");
  }
  if (dt < 0) {
    throw cda_rail::exceptions::InvalidInputException(
        "Time step must be positive.");
  }
  if (ma < 0) {
    throw cda_rail::exceptions::InvalidInputException(
        "Moving authority must be non-negative.");
  }

  if (v_0 * static_cast<double>(dt) / 2.0 >= ma) {
    // If linearly stopping, train will run over the moving authority
    // Hence, the train must brake quicker and stop within the time step already
    return 0; // Train stops at ma
  }

  const double a = d * dt;
  const double b = (2 * d * ma) - (d * dt * v_0);

  // Recall: v_1 = 0.5 * (sqrt(A^2 + 4*B) - A)
  // This is not numerically stable, so we multiply by
  // (sqrt(A^2 + 4*B) + A) / (sqrt(A^2 + 4*B) + A)
  // v_1 = 0.5 * (A^2 + 4*B - A^2) / (A + sqrt(A^2 + 4*B))
  // Simplifying gives us:
  // v_1 = 0.5 * (4*B) / (A + sqrt(A^2 + 4*B))
  // v_1 = 2 * B / (A + sqrt(A^2 + 4*B))
  return 2.0 * b / (a + std::sqrt((a * a) + (4.0 * b)));
}

bool cda_rail::simulator::GreedySimulator::move_train(
    size_t tr, double v_0, double v_1, double ma, int dt,
    std::vector<std::pair<double, double>>& train_positions) {
  /**
   * Move the trains front position based on the given parameters assuming
   * linear movement. This function changes the train_position passed by
   * reference.
   *
   * @param tr: The id of the train to move.
   * @param v_0: The initial velocity of the train in m/s.
   * @param v_1: The velocity of the train at the end of the time step in m/s.
   * @param ma: The moving authority in m.
   * @param dt: The time step in seconds.
   * @param train_positions: A vector of pairs containing the rear and front
   *
   * @return : A boolean indicating whether the train moved forward.
   */

  if (tr >= train_positions.size()) {
    throw cda_rail::exceptions::TrainNotExistentException(tr);
  }

  const double move_distance =
      std::min(ma, (v_0 + v_1) * dt / 2.0); // Distance moved in the time step
  if (std::abs(move_distance) < EPS) {
    return false;
  }
  train_positions.at(tr).second += move_distance; // Update the front position
  return move_distance > 0;
}

void cda_rail::simulator::GreedySimulator::update_rear_positions(
    std::vector<std::pair<double, double>>& train_positions) const {
  /**
   * Update the rear positions of all trains based on their front positions and
   * train lengths. Tis function changes the train_positions passed by
   * reference.
   *
   * @param train_positions: A vector of pairs containing the rear and front
   */

  if (train_positions.size() !=
      instance->get_timetable().get_train_list().size()) {
    throw cda_rail::exceptions::InvalidInputException(
        "Train positions size does not match the number of trains.");
  }

  for (size_t tr = 0; tr < train_positions.size(); ++tr) {
    const auto& train =
        instance->get_timetable().get_train_list().get_train(tr);
    train_positions.at(tr).first =
        train_positions.at(tr).second - train.length; // Update rear position
  }
}

bool cda_rail::simulator::GreedySimulator::is_feasible_to_schedule(
    int t, const std::vector<std::optional<size_t>>& next_stop_id,
    const std::vector<std::pair<double, double>>& train_positions,
    const std::unordered_set<size_t>&             trains_in_network,
    const std::unordered_set<size_t>& trains_left, bool late_entry_possible,
    bool late_exit_possible, bool late_stop_possible) const {
  /**
   * This function checks if the current state of the simulation is feasible
   * (after all trains have been moved to time t) or some violation becomes
   * unavoidable. This function assumes that the next_stop will be updated as
   * soon as a train reaches a stop. The minimal stopping time in a station has
   * to be ensured without the usage of the next_stop_id vector.
   *
   * @param t: The current time step.
   * @param next_stop_id: A vector of optional size_t containing the next stop
   * id for each train.
   * @param tr_stopped_until: A vector of integers indicating until which time
   * step each train is stopped.
   * @param train_positions: A vector of pairs containing the rear and front
   * positions of each train in the network.
   * @param trains_in_network: A set of train ids that are currently in the
   * network.
   * @param trains_left: A set of train ids that have not yet left the network.
   * @param late_entry_possible: If true, late entry is allowed.
   * @param late_exit_possible: If true, late exit is allowed.
   * @param late_stop_possible: If true, late stop is allowed.
   *
   * @return: A boolean indicating whether the current state is feasible.
   */

  if (next_stop_id.size() !=
      instance->get_timetable().get_train_list().size()) {
    throw cda_rail::exceptions::InvalidInputException(
        "Next stop id size does not match the number of trains.");
  }
  if (train_positions.size() !=
      instance->get_timetable().get_train_list().size()) {
    throw cda_rail::exceptions::InvalidInputException(
        "Train positions size does not match the number of trains.");
  }
  if (t < 0) {
    throw cda_rail::exceptions::InvalidInputException(
        "Time step must be non-negative.");
  }

  for (size_t tr = 0; tr < train_positions.size(); ++tr) {
    const auto& tr_schedule = instance->get_timetable().get_schedule(tr);
    if (!late_entry_possible && (tr_schedule.get_t_0_range().second <= t) &&
        !trains_in_network.contains(tr) && !trains_left.contains(tr)) {
      return false; // Train is not allowed to enter the network late
    }
    if (!late_exit_possible && (tr_schedule.get_t_n_range().second <= t) &&
        !trains_left.contains(tr)) {
      // Train is not allowed to leave the network late, however, maybe this is
      // because its route is not yet completely specified and it has already
      // reached the end of its specified route.
      if (!train_edges.at(tr).empty() &&
          (instance->const_n().get_edge(train_edges.at(tr).back()).target ==
               tr_schedule.get_exit() ||
           train_positions.at(tr).second < train_edge_length(tr))) {
        // The above reason does not hold
        return false;
      }
    }
    if (!late_stop_possible && next_stop_id.at(tr).has_value() &&
        tr_schedule.get_stops()
                .at(next_stop_id.at(tr).value())
                .get_begin_range()
                .second <= t) {
      // Train is not allowed to stop late
      return false;
    }
  }

  return true;
}

cda_rail::simulator::GreedySimulator::DestinationType
cda_rail::simulator::GreedySimulator::tr_reached_end(
    size_t tr, const std::vector<std::pair<double, double>>& train_pos) const {
  /**
   * This function checks if a train has reached the end of its route.
   * If yes, it determines if this is due to the train leaving the network or
   * stopping at the end of its route. In the latter case, it is checked if the
   * route end is a station stop.
   */

  const auto  route_len = train_edge_length(tr);
  const auto& pos       = train_pos.at(tr).second;
  if (pos < route_len) {
    // Train has not reached the end of its route
    return DestinationType::None;
  }
  // Determine type of end
  if (instance->get_timetable().get_schedule(tr).get_exit() ==
      instance->const_n().get_edge(train_edges.at(tr).back()).target) {
    // Train leaves the network at the end of its route
    return DestinationType::Network;
  }
  if (!stop_positions.at(tr).empty() &&
      stop_positions.at(tr).back() >= route_len - EPS) {
    // Train stops at the end of its route
    return DestinationType::Station;
  }
  // Train stops at the end of its route, but not at a station stop
  return DestinationType::Edge;
}
