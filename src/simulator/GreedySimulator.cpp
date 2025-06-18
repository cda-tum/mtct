#include "simulator/GreedySimulator.hpp"

#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "EOMHelper.hpp"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

std::pair<bool, std::vector<int>>
cda_rail::simulator::GreedySimulator::simulate(int dt, bool late_entry_possible,
                                               bool late_exit_possible,
                                               bool late_stop_possible) const {
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
   * successful and a vector of doubles with the travel times of each train.
   */

  // Initialize return values
  std::vector<int> travel_times(
      instance->get_timetable().get_train_list().size(),
      -1); // -1 indicates that a train has not entered the network yet
  bool feasible = true;

  // Find first time step
  int min_T = std::numeric_limits<int>::max();
  for (const auto& train : instance->get_timetable().get_train_list()) {
    min_T = std::min(min_T, instance->get_timetable()
                                .get_schedule(train.name)
                                .get_t_0_range()
                                .first);
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
  std::vector<int>           tr_stop_until(
      instance->get_timetable().get_train_list().size(),
      -1); // time until train stops in station
  std::vector<double> tr_next_stop(
      instance->get_timetable().get_train_list().size(),
      -1.0); // next scheduled stop position

  const auto tr_to_enter = get_entering_trains(
      min_T, trains_in_network, trains_left, late_entry_possible);

  return {feasible, travel_times};
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
      const auto& prev_tr = *(it - 1);
      if (!tr_present.contains(prev_tr) && !tr_left.contains(prev_tr)) {
        // Previous train is not in the network, so tr cannot enter yet
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

std::tuple<bool, std::pair<double, double>, std::pair<double, double>>
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
  const auto& tr_obj = instance->get_timetable().get_train_list().get_train(tr);
  const bool  is_on_edge   = (pos.second > milestone_pair.first + EPS &&
                           pos.first < milestone_pair.second - EPS);
  const bool  rear_on_edge = is_on_edge && (pos.first >= milestone_pair.first);
  const bool  front_on_edge =
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
    const auto [occ_status, detailed_occ_status, pos_on_edge] =
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
      const auto [occ, det_occ, det_pos] =
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
    for (const auto& other_tr : ttd_orders.at(ttd_sec.value())) {
      if (other_tr == tr || !trains_in_network.contains(other_tr)) {
        continue; // Skip the train itself or trains that are not in the network
      }
      const auto& other_pos = train_positions.at(other_tr);
      if (is_on_ttd(tr, ttd_sec.value(), other_pos)) {
        return false; // Other train is occupying the TTD section
      }
    }
  }
  return true;
}
