#include "simulator/GreedySimulator.hpp"

#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "EOMHelper.hpp"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <set>
#include <utility>
#include <vector>

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
  if (entry_orders.size() != instance->const_n().number_of_vertices()) {
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

  // entry_orders only contains valid train indices
  for (const auto& orders : entry_orders) {
    for (const auto& train_id : orders) {
      if (!instance->get_timetable().get_train_list().has_train(train_id)) {
        return false;
      }
    }
  }

  // entry orders only contain trains that are entering the network through the
  // respective vertex
  for (size_t vertex_id = 0; vertex_id < entry_orders.size(); ++vertex_id) {
    for (const auto& train_id : entry_orders[vertex_id]) {
      if (vertex_id !=
          instance->get_timetable().get_schedule(train_id).get_entry()) {
        return false;
      }
    }
  }

  return true;
}

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
  std::set<size_t> trains_in_network;
  std::vector<int> tr_stop_until(
      instance->get_timetable().get_train_list().size(),
      -1); // time until train stops in station
  std::vector<double> tr_next_stop(
      instance->get_timetable().get_train_list().size(),
      -1.0); // next scheduled stop position

  return {feasible, travel_times};
}

double cda_rail::simulator::GreedySimulator::braking_distance(size_t tr,
                                                              double v) {
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
