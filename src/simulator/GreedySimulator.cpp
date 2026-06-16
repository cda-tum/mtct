#include "simulator/GreedySimulator.hpp"

#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "EOMHelper.hpp"
#include "GeneralHelper.hpp"
#include "StringHelper.hpp"
#include "plog/Log.h"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "simulator/GeneralSimulator.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast)

// ----------------
// CONSTRUCTOR
// ----------------

cda_rail::simulator::GreedySimulator::GreedySimulator(
    cda_rail::instances::GeneralPerformanceOptimizationInstance const& instance,
    std::vector<cda_rail::index_set> ttd_sections)
    : GeneralSimulator(
          std::make_shared<
              const instances::GeneralPerformanceOptimizationInstance>(
              instance),
          std::move(ttd_sections)) {}

cda_rail::simulator::GreedySimulator::GreedySimulator(
    cda_rail::instances::GeneralPerformanceOptimizationInstance const& instance,
    std::vector<cda_rail::index_set>    ttd_sections,
    std::vector<cda_rail::index_vector> train_edges,
    std::vector<cda_rail::index_vector> ttd_orders,
    std::vector<cda_rail::index_vector> vertex_orders,
    std::vector<std::vector<double>>    stop_positions)
    : GeneralSimulator(
          std::make_shared<
              const instances::GeneralPerformanceOptimizationInstance>(
              instance),
          std::move(ttd_sections), std::move(train_edges),
          std::move(ttd_orders), std::move(vertex_orders),
          std::move(stop_positions)) {}

// ---------------
// SIMULATE
// ---------------

cda_rail::simulator::SimulatorResults
cda_rail::simulator::GreedySimulator::simulate(
    double dt, bool late_entry_possible, bool limit_speed_by_leaving_edges,
    bool save_trajectories) const {
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
   * @return: A tuple containing
   *  - a boolean indicating whether the simulation was successful,
   *  - a vector of doubles with the exit times of each train,
   *  - a vector with the braking times and distances due to route ending for
   * every train,
   *  - a vector of doubles with the final vertex headways
   */

  // NOLINTBEGIN(*-inconsistent-ifelse-braces)

  exceptions::throw_if_non_positive(dt, "Time step length dt");

  cda_rail::initialize_plog(false);

  // Initialize return values
  auto const&         train_list       = get_instance()->get_const_train_list();
  auto const          number_of_trains = train_list.size();
  std::vector<double> exit_times(
      number_of_trains,
      0); // Initially each train is simulated until
          // time t=0, for t>0 the heuristic is needed.
  std::vector<double>              braking_times(number_of_trains, -1);
  std::vector<double>              braking_distances(number_of_trains, -1);
  std::vector<std::vector<double>> stop_times(number_of_trains);
  std::vector<std::map<double, PosVel>>
      train_trajectories; // time -> {pos, vel}

  // Find first time step
  double min_t              = std::numeric_limits<int>::max();
  double last_entering_time = std::numeric_limits<int>::min();
  for (size_t tr = 0; tr < number_of_trains; ++tr) {
    auto const entry_time =
        get_instance()->get_const_schedule(tr).get_entry_time();
    min_t              = std::min(min_t, entry_time);
    last_entering_time = std::max(last_entering_time, entry_time);
  }

  // Initialize variables to keep track of positions and velocities
  if (save_trajectories) {
    train_trajectories.clear();
    train_trajectories.resize(number_of_trains);
  }

  std::vector<TrainPosition> train_positions(
      number_of_trains,
      {.rear = -1.0, .front = -1.0}); // {rear, front} positions
  std::vector<double> train_velocities(number_of_trains, -1.0); // velocities
  std::unordered_set<size_t> trains_in_network;
  std::unordered_set<size_t> trains_left;
  std::unordered_set<size_t> trains_finished_simulating;
  std::vector<double> tr_stop_until(number_of_trains,
                                    -1.0); // time until train stops in station
  std::vector<std::optional<size_t>> tr_next_stop_id(
      number_of_trains); // next scheduled stop position
  std::vector<double> vertex_headways(
      get_instance()->get_const_network().number_of_vertices(),
      0.0); // headways for vertices

  auto build_results = [&](bool success) {
    return SimulatorResults{.success           = success,
                            .exit_times        = std::move(exit_times),
                            .stop_times        = std::move(stop_times),
                            .braking_times     = std::move(braking_times),
                            .braking_distances = std::move(braking_distances),
                            .vertex_headways   = vertex_headways,
                            .train_trajectories =
                                std::move(train_trajectories)};
  };

  const auto trains_on_edges = tr_on_edges();

  double t = min_t;

  PLOGV << "Starting simulation from time " << min_t;

  // Detect trains that are not scheduled to enter the network
  for (size_t tr = 0; tr < number_of_trains; ++tr) {
    if (get_train_edges_of_tr(tr).empty()) {
      trains_finished_simulating.insert(tr);
    }
  }

  int cycles_without_movement = 0;
  while (cycles_without_movement < CYCLE_LIMIT) {
    PLOGV << "----------------------------";
    PLOGV << "Current time: " << t;

    bool movement_detected = false;

    // Blocked Vertices, i.e., headway > t
    cda_rail::index_set blocked_vertices;
    for (size_t vertex = 0; vertex < vertex_headways.size(); ++vertex) {
      if (vertex_headways.at(vertex) > t) {
        blocked_vertices.insert(vertex);
      }
    }

    // Move existing trains
    for (const auto& tr : trains_in_network) {
      const auto& train_object = train_list.get_train(tr);

      if (trains_finished_simulating.contains(tr) ||
          t < tr_stop_until.at(tr) + dt) {
        PLOGV << train_object.get_name() << " skipped.";
        continue;
      }

      // Calculate MA

      auto blocked_vertices_tr = blocked_vertices;

      if (t < get_instance()->get_const_schedule(tr).get_exit_time()) {
        PLOGV << train_object.get_name()
              << "'s exit vertex is blocked by earliest exit.";
        const auto tr_exit_vertex =
            get_instance()->get_const_schedule(tr).get_exit_vertex();
        blocked_vertices_tr.insert(tr_exit_vertex);
      }

      const auto tr_ma_data = get_ma_and_maxv(
          tr, train_velocities, tr_next_stop_id.at(tr), dt, blocked_vertices_tr,
          train_positions, trains_in_network, trains_left, trains_on_edges,
          limit_speed_by_leaving_edges);
      PLOGV << train_object.get_name() << " positioned at "
            << train_positions.at(tr).front
            << " has MA: " << train_positions.at(tr).front + tr_ma_data.ma
            << " (without route end this would be: "
            << train_positions.at(tr).front + tr_ma_data.ma_without_route_end
            << ")"
            << " and max velocity: " << tr_ma_data.max_v
            << " (without route end this would be: "
            << tr_ma_data.max_v_without_route_end << ")";
      const auto tr_edge_len = train_edge_length(tr);

      auto tr_new_speed =
          std::min(tr_ma_data.max_v,
                   get_v1_from_ma(train_velocities.at(tr), tr_ma_data.ma,
                                  train_object.get_deceleration(), dt));
      if (tr_new_speed < V_MIN) {
        tr_new_speed = 0.0; // Train is stopped
      }
      auto tr_new_speed_without_route_end =
          std::min(tr_ma_data.max_v_without_route_end,
                   get_v1_from_ma(train_velocities.at(tr),
                                  tr_ma_data.ma_without_route_end,
                                  train_object.get_deceleration(), dt));
      if (tr_new_speed_without_route_end < V_MIN) {
        tr_new_speed_without_route_end = 0.0;
      }

      PLOGV << "tr_new_speed = " << tr_new_speed
            << " (without route end this would be: "
            << tr_new_speed_without_route_end << ")";

      if ((braking_distances.at(tr) < 0) &&
          (tr_new_speed < tr_new_speed_without_route_end)) {
        PLOGV << train_object.get_name()
              << " starts braking due to end of route constraint.";
        braking_times.at(tr)     = t - dt;
        braking_distances.at(tr) = tr_edge_len - train_positions.at(tr).front;
      }

      // Move trains
      if (move_train(tr, train_velocities.at(tr), tr_new_speed, tr_ma_data.ma,
                     dt, train_positions)) {
        movement_detected = true;
      }
      train_velocities.at(tr) = tr_new_speed;
      PLOGV << "At time " << t << ", " << train_object.get_name()
            << " moved to " << train_positions.at(tr).front << " with speed "
            << tr_new_speed << " and MA "
            << train_positions.at(tr).front +
                   cda_rail::braking_distance(tr_new_speed,
                                              train_object.get_deceleration());
      if (save_trajectories) {
        train_trajectories.at(tr)[t] = {.pos = train_positions.at(tr).front,
                                        .vel = train_velocities.at(tr)};
      }
    }

    // Update rear positions of trains
    update_rear_positions(train_positions);

    cda_rail::index_vector trains_to_remove;
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
        const auto& exit_vertex_idx =
            get_instance()->get_const_schedule(tr).get_exit_vertex();
        const auto& exit_vertex =
            get_instance()->get_const_network().get_vertex(exit_vertex_idx);
        vertex_headways.at(exit_vertex_idx) = t + exit_vertex.headway;
        PLOGV << "At time " << t << ", " << train_list.get_train(tr).get_name()
              << " left the network.";
      } else if (tr_status == DestinationType::Edge) {
        trains_finished_simulating.insert(tr);
        exit_times.at(tr) = t;
        PLOGV << "At time " << t << ", " << train_list.get_train(tr).get_name()
              << " reached the end of its route on an edge within the network.";
      } else if (tr_status == DestinationType::Station) {
        assert(tr_next_stop_id.at(tr).has_value());
        const auto& last_stop =
            get_instance()->get_const_schedule(tr).get_stops().at(
                tr_next_stop_id.at(tr).value());
        exit_times.at(tr) = std::max(t + last_stop.get_service_duration(),
                                     last_stop.get_earliest_departure());
        stop_times.at(tr).emplace_back(t);
        tr_next_stop_id.at(tr) = {};
        trains_finished_simulating.insert(tr);
        // set braking_times to the route end. Only after stopping at the
        // station, the underdefined route is the only cause for the train not
        // to continue
        braking_times.at(tr)     = exit_times.at(tr);
        braking_distances.at(tr) = 0;
        PLOGV << "At time " << t << ", " << train_list.get_train(tr).get_name()
              << " reached the end of its route at station "
              << last_stop.get_station().name << ", stopping until "
              << exit_times.at(tr);
      } else {
        // Train is still in the network
        // Update stop information if a train has reached its next stop
        if (tr_next_stop_id.at(tr).has_value()) {
          if (train_velocities.at(tr) < V_MIN &&
              (train_positions.at(tr).front >=
               get_stop_positions_of_tr(tr).at(tr_next_stop_id.at(tr).value()) -
                   STOP_TOLERANCE)) {
            // Train has reached its next stop
            stop_times.at(tr).push_back(t);
            const auto& tr_stops =
                get_instance()->get_const_schedule(tr).get_stops();
            const auto& stop_info = tr_stops.at(tr_next_stop_id.at(tr).value());
            tr_stop_until.at(tr) =
                std::max(t + stop_info.get_service_duration(),
                         stop_info.get_earliest_departure());
            PLOGV << "At time " << t << ", "
                  << train_list.get_train(tr).get_name()
                  << " reached its next stop at "
                  << stop_info.get_station().name << ", stopping until "
                  << tr_stop_until.at(tr);

            if (tr_next_stop_id.at(tr).value() + 1 <
                get_stop_positions_of_tr(tr).size()) {
              // Update next stop ID
              tr_next_stop_id.at(tr) = tr_next_stop_id.at(tr).value() + 1;
              PLOGV << "Next stop: "
                    << tr_stops.at(tr_next_stop_id.at(tr).value())
                           .get_station()
                           .name;
            } else {
              // No more stops scheduled
              tr_next_stop_id.at(tr) = std::nullopt;
              PLOGV << "No more stops scheduled";
            }
          }
        }
      }
    }

    // Remove trains that have left the network
    for (const auto& tr : trains_to_remove) {
      trains_in_network.erase(tr);
    }

    // Check for new trains entering the network
    const auto [tr_to_enter_success, tr_to_enter] = get_entering_trains(
        t, trains_in_network, trains_left, trains_finished_simulating,
        late_entry_possible, dt);
    if (!tr_to_enter_success) {
      PLOGV
          << "Simulation failed: Not all trains can enter the network at time "
          << t;
      return build_results(false);
    }
    for (const auto& tr : tr_to_enter) {
      const auto& train_schedule = get_instance()->get_const_schedule(tr);
      const auto& entry_vertex = get_instance()->get_const_network().get_vertex(
          train_schedule.get_entry_vertex());
      if (vertex_headways.at(train_schedule.get_entry_vertex()) > t) {
        PLOGV << "At time " << t << ", " << train_list.get_train(tr).get_name()
              << " cannot enter the network at " << entry_vertex.name
              << " due to vertex headway constraints until time "
              << vertex_headways.at(train_schedule.get_entry_vertex());
      } else if (!is_ok_to_enter(tr, train_positions, train_velocities,
                                 trains_in_network, trains_on_edges)) {
        PLOGV << "At time " << t << ", " << train_list.get_train(tr).get_name()
              << " cannot enter the network at " << entry_vertex.name
              << " due to moving authority constraints constraints.";
      } else {
        trains_in_network.insert(tr);
        vertex_headways.at(train_schedule.get_entry_vertex()) =
            t + entry_vertex.headway;
        train_positions.at(tr)  = {.rear =
                                       -train_list.get_train(tr).get_length(),
                                   .front = 0.0}; // Initialize positions
        train_velocities.at(tr) = train_schedule.get_initial_velocity();
        if (!get_stop_positions_of_tr(tr).empty()) {
          tr_next_stop_id.at(tr) = 0;
        }
        movement_detected = true;
        PLOGV << "At time " << t << ", " << train_list.get_train(tr).get_name()
              << " entered the network at " << entry_vertex.name;
        PLOGV << "New entry blocked until time "
              << vertex_headways.at(train_schedule.get_entry_vertex());
        if (save_trajectories) {
          train_trajectories.at(tr)[t] = {.pos = train_positions.at(tr).front,
                                          .vel = train_velocities.at(tr)};
        }
      }
    }

    // Check if all trains have reached their destination
    if (trains_finished_simulating.size() == number_of_trains) {
      PLOGV << "All trains have reached their destination at time " << t;
      return build_results(true);
    }

    cycles_without_movement =
        movement_detected ? 0 : cycles_without_movement + 1;

    // Check if there might be a deadlock situation
    if (!movement_detected) {
      // There might be a deadlock situation if none of the constraints depend
      // on time
      PLOGV << "No movement detected at time " << t;
      bool reason_found = false;
      if (std::ranges::any_of(vertex_headways, [t](int vertex_headway) {
            return vertex_headway > t;
          })) {
        PLOGV << "Vertex headway constraint prevents movement.";
        reason_found = true;
      } else if (std::ranges::any_of(tr_stop_until, [t, dt](int stop_time) {
                   return stop_time + dt > t;
                 })) {
        PLOGV << "Train stop constraint prevents movement.";
        reason_found = true;
      } else {
        for (size_t tr = 0; tr < train_velocities.size(); ++tr) {
          if (!trains_finished_simulating.contains(tr)) {
            if (trains_in_network.contains(tr)) {
              if ((get_instance()
                       ->get_const_network()
                       .get_edge(get_train_edges_of_tr(tr).back())
                       .target ==
                   get_instance()->get_const_schedule(tr).get_exit_vertex()) &&
                  get_instance()->get_const_schedule(tr).get_exit_time() > t) {
                PLOGV << train_list.get_train(tr).get_name()
                      << " is blocked by earliest exit.";
                reason_found = true;
                break;
              }
            } else {
              if (get_instance()->get_const_schedule(tr).get_entry_time() > t) {
                PLOGV << train_list.get_train(tr).get_name()
                      << " is blocked by earliest entry.";
                reason_found = true;
                break;
              }
            }
          }
        }
      }
      if (!reason_found) {
        PLOGV << "Trains are in a deadlock situation.";
        return build_results(false);
      }
    }

    // Update time
    t += dt;
  }

  throw std::runtime_error("Simulation failed: Cycle limit reached.");
  // NOLINTEND(*-inconsistent-ifelse-braces)
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)

// ----------------------------
// PRIVATE HELPER FUNCTIONS
// ----------------------------

// Positioning Helper

cda_rail::simulator::GreedySimulator::PosOnEdgeReturn
cda_rail::simulator::GreedySimulator::get_position_on_route_edge(
    size_t tr, const TrainPosition& pos, size_t edge_number,
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
  get_instance()->get_const_train_list().throw_if_train_not_exist(tr);

  if (milestones.empty()) {
    milestones = edge_milestones(tr);
  }
  if (get_train_edges_of_tr(tr).size() + 1 != milestones.size()) {
    throw cda_rail::exceptions::ConsistencyException(concatenate_string_views(
        {"Milestones size does not match number of edges for train ",
         get_instance()->get_const_train_list().get_train(tr).get_name(),
         ". Expected: ", std::to_string(get_train_edges_of_tr(tr).size() + 1),
         ", Actual: ", std::to_string(milestones.size()), "."}));
  }

  if (edge_number >= get_train_edges_of_tr(tr).size()) {
    throw cda_rail::exceptions::InvalidInputException(concatenate_string_views(
        {"Edge number (", std::to_string(edge_number),
         ") out of bounds for train ",
         get_instance()->get_const_train_list().get_train(tr).get_name(),
         ". Train has only ", std::to_string(get_train_edges_of_tr(tr).size()),
         " edges."}));
  }

  const std::pair<double, double> milestone_pair = {
      milestones.at(edge_number), milestones.at(edge_number + 1)};
  const bool is_on_edge    = (pos.front > milestone_pair.first + EPS &&
                              pos.rear < milestone_pair.second - EPS);
  const bool rear_on_edge  = is_on_edge && (pos.rear >= milestone_pair.first);
  const bool front_on_edge = is_on_edge && (pos.front <= milestone_pair.second);

  return {.tr_on_edge_indicator = {.tr_on_edge    = is_on_edge,
                                   .rear_on_edge  = rear_on_edge,
                                   .front_on_edge = front_on_edge},
          .tr_position_on_edge  = {
              .rear  = std::max(0.0, pos.rear - milestone_pair.first),
              .front = std::min(milestone_pair.second - milestone_pair.first,
                                pos.front - milestone_pair.first)}};
}

cda_rail::simulator::GreedySimulator::PosOnEdgeReturn
cda_rail::simulator::GreedySimulator::get_position_on_edge(
    size_t tr, const TrainPosition& pos, size_t edge_id,
    std::vector<double> milestones) const {
  get_instance()->get_const_train_list().throw_if_train_not_exist(tr);
  if (!get_instance()->get_const_network().has_edge(edge_id)) {
    throw cda_rail::exceptions::EdgeNotExistentException(edge_id);
  }

  const auto& tr_edges    = get_train_edges_of_tr(tr);
  const auto  edge_number = std::ranges::find(tr_edges, edge_id);
  if (edge_number == tr_edges.end()) {
    throw cda_rail::exceptions::ConsistencyException(concatenate_string_views(
        {"Train ",
         get_instance()->get_const_train_list().get_train(tr).get_name(),
         " does not have edge ",
         get_instance()->get_const_network().get_edge_name(edge_id),
         " on its route."}));
  }
  const auto edge_index = std::distance(tr_edges.begin(), edge_number);
  return get_position_on_route_edge(tr, pos, edge_index, std::move(milestones));
}

bool cda_rail::simulator::GreedySimulator::is_on_ttd(
    size_t tr, size_t ttd, const TrainPosition& pos,
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
  if (ttd >= get_ttd_sections().size()) {
    throw cda_rail::exceptions::InvalidInputException(concatenate_string_views(
        {"TTD index out of bounds: ", std::to_string(ttd),
         ". Maximum index is ", std::to_string(get_ttd_sections().size() - 1),
         "."}));
  }

  const auto  milestones         = edge_milestones(tr);
  const auto& ttd_section        = get_ttd_sections().at(ttd);
  bool        potentially_behind = false;
  for (const auto& edge_id : ttd_section) {
    if (!is_on_route(tr, edge_id)) {
      continue; // Train is not on this edge
    }

    [[maybe_unused]] const auto [occ_status, pos_on_edge] =
        get_position_on_edge(tr, pos, edge_id, milestones);
    if (occ_status.tr_on_edge &&
        (occupation_type == TTDOccupationType::OnlyOccupied ||
         occupation_type == TTDOccupationType::OccupiedOrBehind)) {
      return true; // Train is on the edge
    }
    if (occ_status.tr_on_edge &&
        occupation_type == TTDOccupationType::OnlyBehind) {
      return false; // Train is on the edge, but we only want to check if it is
                    // truly behind
    }
    if (!occ_status.tr_on_edge &&
        (occupation_type == TTDOccupationType::OnlyBehind ||
         occupation_type == TTDOccupationType::OccupiedOrBehind) &&
        pos_on_edge.rear >=
            get_instance()->get_const_network().get_edge(edge_id).length) {
      potentially_behind =
          true; // Train is either behind ttd section or on a later ttd edge
      if (occupation_type == TTDOccupationType::OccupiedOrBehind) {
        return true;
      }
    }
  }
  return potentially_behind; // Train is not on the TTD section
}

// Entering Helper

bool cda_rail::simulator::GreedySimulator::is_ok_to_enter(
    size_t tr, const std::vector<TrainPosition>& train_positions,
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

  const auto v0 = get_instance()->get_const_schedule(tr).get_initial_velocity();
  const auto bd = tr_braking_distance(tr, v0);
  const auto milestones = edge_milestones(tr);
  for (size_t i = 0;
       i < get_train_edges_of_tr(tr).size() && milestones.at(i) + EPS < bd;
       ++i) {
    const auto& edge_id          = get_train_edges_of_tr(tr).at(i);
    const auto& potential_trains = tr_on_edges.at(edge_id);
    for (const auto& other_tr : potential_trains) {
      if (other_tr == tr || !trains_in_network.contains(other_tr)) {
        continue; // Skip the train itself or trains that are not in the network
      }
      const auto& other_pos = train_positions.at(other_tr);
      [[maybe_unused]] const auto [occ, pos] =
          get_position_on_edge(other_tr, other_pos, edge_id);
      if (occ.tr_on_edge && pos.rear <= bd - milestones.at(i) + EPS) {
        return false; // Other train is occupying the edge within the braking
                      // distance
      }
    }

    // Potentially train on reverse edge
    const auto reverse_edge_id =
        get_instance()->get_const_network().get_reverse_edge_index(edge_id);
    if (reverse_edge_id.has_value()) {
      const auto& potential_trains_reverse =
          tr_on_edges.at(reverse_edge_id.value());
      for (const auto& other_tr : potential_trains_reverse) {
        if (other_tr == tr || !trains_in_network.contains(other_tr)) {
          continue; // Skip the train itself or trains that are not in the
                    // network
        }
        // NOLINTBEGIN(bugprone-unchecked-optional-access)
        // false positive
        [[maybe_unused]] const auto [occ_rev, pos_rev] = get_position_on_edge(
            other_tr,
            {.rear = train_positions.at(other_tr).rear,
             .front =
                 train_positions.at(other_tr).front +
                 tr_braking_distance(other_tr, train_velocities.at(other_tr))},
            reverse_edge_id.value());
        // NOLINTEND(bugprone-unchecked-optional-access)
        if (occ_rev.tr_on_edge) {
          return false; // Other train is occupying the reverse edge within the
                        // braking distance
        }
      }
    }

    const auto ttd_sec = get_ttd(edge_id);
    if (ttd_sec.has_value()) {
      const auto& ttd_order = get_ttd_orders_of_ttd(ttd_sec.value());
      const auto  ttd_pos   = std::ranges::find(ttd_order, tr);
      if (ttd_pos == ttd_order.begin()) {
        continue; // Train is the first in the TTD order, no other train can
                  // block it
      }
      const auto& other_tr  = *(ttd_pos - 1); // Previous train in the TTD order
      const auto& other_pos = train_positions.at(other_tr);
      if (!trains_in_network.contains(other_tr) ||
          !is_behind_ttd(other_tr, ttd_sec.value(), other_pos)) {
        return false; // Other train is occupying the TTD section
      }
    }
  }
  return true;
}

std::pair<bool, std::unordered_set<size_t>>
cda_rail::simulator::GreedySimulator::get_entering_trains(
    double t, const std::unordered_set<size_t>& tr_present,
    const std::unordered_set<size_t>& tr_left,
    const std::unordered_set<size_t>& tr_finished_simulating,
    bool late_entry_possible, double buffer_time) const {
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

  exceptions::throw_if_negative(buffer_time, "Buffer time for late entry");

  std::unordered_set<size_t> entering_trains;
  for (size_t tr = 0; tr < get_instance()->get_const_train_list().size();
       ++tr) {
    // Check if the train is already present in the network
    if (tr_present.contains(tr) || tr_finished_simulating.contains(tr) ||
        tr_left.contains(tr)) {
      continue; // Train is already present, skip it
    }
    // Check if the train is scheduled to enter the network at time t or later
    const auto& schedule = get_instance()->get_const_schedule(tr);
    if (t < schedule.get_entry_time()) {
      continue; // Train is not scheduled to enter at this time
    }

    // Check if the train scheduled to enter before tr is already in the network
    const auto& entry_node  = schedule.get_entry_vertex();
    const auto& entry_order = get_vertex_orders_of_vertex(entry_node);
    // Find index of tr in the entry order (if it exists)
    const auto it = std::ranges::find(entry_order, tr);
    // If tr is not in the entry order, it means it is scheduled to enter
    if (it == entry_order.end()) {
      continue; // Train is not scheduled to enter at all
    }

    if (!late_entry_possible && t > schedule.get_entry_time() + buffer_time) {
      // Train can no longer enter the network
      return {false, {tr}};
    }

    // If tr is not the first train in the entry order, check if previous trains
    // are already in the network
    if (it != entry_order.begin()) {
      // Check if any previous train in the entry order is already in the
      // network
      const auto& prev_tr = *(it - 1);
      const auto& prev_entry =
          get_instance()->get_const_schedule(prev_tr).get_entry_vertex();
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

double cda_rail::simulator::GreedySimulator::max_displacement(
    const cda_rail::Train& train, double v_0, double dt) {
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
      v_0, train.get_max_speed(), train.get_acceleration(),
      train.get_deceleration(), dt);
}

double cda_rail::simulator::GreedySimulator::get_absolute_distance_ma(
    size_t tr, double max_displacement,
    const std::vector<TrainPosition>&              train_positions,
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
    throw cda_rail::exceptions::ConsistencyException(concatenate_string_views(
        {"Train ",
         get_instance()->get_const_train_list().get_train(tr).get_name(),
         " is not in the network. Cannot calculate distance to next train."}));
  }
  cda_rail::exceptions::throw_if_negative(max_displacement,
                                          "Maximum displacement");

  const auto milestones = edge_milestones(tr);
  bool       first_edge = true;
  for (size_t i = 0;
       i < get_train_edges_of_tr(tr).size() &&
       milestones.at(i) + EPS < train_positions.at(tr).front + max_displacement;
       ++i) {
    if (milestones.at(i + 1) <= train_positions.at(tr).front) {
      continue; // The edge is behind the train's front position
    }
    const auto& edge_id = get_train_edges_of_tr(tr).at(i);

    // First edge is entirely blocked due to TTD section or train traveling in
    // opposite direction
    if (milestones.at(i) >= train_positions.at(tr).front) {
      const auto ttd_sec = get_ttd(edge_id);
      if (ttd_sec.has_value()) {
        bool check_ttd = true;
        if (i >= 1) {
          const auto& prev_edge_id = get_train_edges_of_tr(tr).at(i - 1);
          const auto  prev_ttd_sec = get_ttd(prev_edge_id);
          if (prev_ttd_sec.has_value() &&
              prev_ttd_sec.value() == ttd_sec.value()) {
            check_ttd = false; // Previous edge is part of the same TTD section,
                               // hence, TTD condition has already been checked
          }
        }
        if (check_ttd) {
          const auto& ttd_order = get_ttd_orders_of_ttd(ttd_sec.value());
          const auto  ttd_pos   = std::ranges::find(ttd_order, tr);
          if (ttd_pos != ttd_order.begin()) {
            const auto& other_tr =
                *(ttd_pos - 1); // Previous train in the TTD order
            const auto& other_pos = train_positions.at(other_tr);
            if (!trains_left.contains(other_tr) &&
                (!trains_in_network.contains(other_tr) ||
                 !is_behind_ttd(other_tr, ttd_sec.value(), other_pos))) {
              return milestones.at(i) -
                     train_positions.at(tr).front; // Other train is occupying
                                                   // the future TTD section
            }
          }
        }
      }

      const auto reverse_edge_id =
          get_instance()->get_const_network().get_reverse_edge_index(edge_id);
      if (reverse_edge_id.has_value()) {
        const auto& potential_trains_reverse =
            tr_on_edges.at(reverse_edge_id.value());
        for (const auto& other_tr : potential_trains_reverse) {
          if (other_tr == tr || !trains_in_network.contains(other_tr)) {
            continue; // Skip the train itself or trains that are not in the
                      // network
          }
          [[maybe_unused]] const auto [occ_rev, pos_rev] = get_position_on_edge(
              other_tr,
              {.rear  = train_positions.at(other_tr).rear,
               .front = train_positions.at(other_tr).front +
                        tr_braking_distance(other_tr,
                                            train_velocities.at(other_tr))},
              reverse_edge_id.value());
          if (occ_rev.tr_on_edge) {
            // Other train has already been cleared to enter the reverse edge
            return milestones.at(i) - train_positions.at(tr).front;
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
      [[maybe_unused]] const auto [occ, pos] =
          get_position_on_edge(other_tr, other_pos, edge_id);
      bool check_other_tr = occ.tr_on_edge;
      if (check_other_tr && first_edge) {
        // Other train could be behind train on the same edge
        [[maybe_unused]] const auto [occ_tr, pos_tr] =
            get_position_on_route_edge(tr, train_positions.at(tr), i,
                                       milestones);
        if (occ_tr.tr_on_edge && pos_tr.rear >= pos.front) {
          check_other_tr = false; // Train is not behind the other train
        }
      }
      if (check_other_tr) {
        found_other_train = true;
        potential_limit   = std::min(potential_limit, pos.rear);
      }
    }
    if (found_other_train) {
      return std::min(
          max_displacement,
          milestones.at(i) + potential_limit -
              train_positions.at(tr).front); // Found a train on the edge
    }
    first_edge = false; // After the first edge, we do not need to check if
                        // other train might be behind
  }
  return max_displacement; // No train found within the maximum displacement
                           // range
}

cda_rail::simulator::PosVel
cda_rail::simulator::GreedySimulator::speed_restriction_helper(
    double ma, double max_v, double pos, double vertex_pos, double v_0,
    double v_m, double d, double dt) {
  /**
   * This function serves as a helper function for the simulator at a certain
   * position. If the train can reach the next edge within one time step, it
   * limits the speed directly. Otherwise, it calculates the moving authority to
   * ensure that the train can brake well in advance to arrive only at the
   * limited speed when entering the edge.
   */

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
  return {.pos = ma, .vel = max_v};
}

cda_rail::simulator::GreedySimulator::MaAndMaxVResult
cda_rail::simulator::GreedySimulator::get_future_max_speed_constraints(
    size_t tr, const cda_rail::Train& train, double pos, double v_0,
    double max_displacement, double dt,
    cda_rail::index_set const& blocked_vertices,
    bool                       also_limit_by_leaving_edges) const {
  /**
   * This function calculates the future maximum speed constraints for a train.
   * If an edge is reachable, the trains speed is restricted directly.
   * Otherwise, future speed restrictions are modeled through restrictions on
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
   * @param blocked_vertices: Vertices that are currently blocked due to headway
   * constraints
   * @param also_limit_by_leaving_edges: If true, the speed is limited by the
   * edges the train is leaving, otherwise only by the front of the train.
   *
   * @return: A pair of doubles representing the:
   * - maximum moving authority from the trains current position and
   * - the maximum speed allowed for the next speed
   */

  round_small_numbers_to_zero_inplace(pos);
  round_small_numbers_to_zero_inplace(v_0);
  round_small_numbers_to_zero_inplace(max_displacement);

  cda_rail::exceptions::throw_if_negative(pos, "Position");
  cda_rail::exceptions::throw_if_negative(v_0, "Initial velocity");
  cda_rail::exceptions::throw_if_negative(max_displacement,
                                          "Maximum displacement");
  cda_rail::exceptions::throw_if_negative(dt, "Time step");

  PosVel retval = {.pos = max_displacement,
                   .vel = std::min(train.get_max_speed(),
                                   v_0 + (train.get_acceleration() * dt))};

  const auto milestones = edge_milestones(tr);
  for (size_t i = 0; i < get_train_edges_of_tr(tr).size() &&
                     milestones.at(i) + EPS < pos + max_displacement;
       ++i) {
    if (milestones.at(i + 1) <= pos - train.get_length()) {
      continue; // Train has fully left the edge already
    }
    if (!also_limit_by_leaving_edges && milestones.at(i + 1) <= pos) {
      continue; // Train's front has already left the edge
    }

    const auto& edge_id = get_train_edges_of_tr(tr).at(i);
    const auto& edge    = get_instance()->get_const_network().get_edge(edge_id);
    [[maybe_unused]] const auto [occ, det_pos] = get_position_on_route_edge(
        tr, {.rear = pos - train.get_length(), .front = pos}, i, milestones);

    if ((!occ.tr_on_edge || occ.front_on_edge) &&
        blocked_vertices.contains(edge.target)) {
      // Cannot leave edge
      retval.pos = std::min(retval.pos, milestones.at(i + 1) - pos);
    }

    if (occ.tr_on_edge) {
      // Train is on the edge.
      // If also_limit_by_leaving_edges=false, this must be the front, because
      // otherwise we would already have continued the loop.
      retval.vel = std::min(retval.vel, edge.max_speed);
    } else {
      retval = speed_restriction_helper(retval.pos, retval.vel, pos,
                                        milestones.at(i), v_0, edge.max_speed,
                                        train.get_deceleration(), dt);
    }
  }

  // Check exit
  const auto& last_edge_id = get_train_edges_of_tr(tr).back();
  const auto& last_edge =
      get_instance()->get_const_network().get_edge(last_edge_id);
  const auto tr_schedule = get_instance()->get_const_schedule(tr);
  const bool last_edge_leaves_network =
      (last_edge.target == tr_schedule.get_exit_vertex());
  const auto relevant_last_pos =
      milestones.back() + (last_edge_leaves_network
                               ? train.get_length()
                               : 0); // + train.length because train needs to
                                     // fully leave the network
  PosVel retval_without_route_end = {.pos = retval.pos, .vel = retval.vel};
  if (pos + max_displacement >= relevant_last_pos) {
    const double last_edge_exit_restriction =
        last_edge_leaves_network ? tr_schedule.get_exit_velocity() : 0;
    retval = speed_restriction_helper(
        retval.pos, retval.vel, pos, relevant_last_pos, v_0,
        last_edge_exit_restriction, train.get_deceleration(), dt);
  }
  if (last_edge_leaves_network) {
    retval_without_route_end = retval;
  }
  return {.ma                      = retval.pos,
          .ma_without_route_end    = retval_without_route_end.pos,
          .max_v                   = retval.vel,
          .max_v_without_route_end = retval_without_route_end.vel};
}

double cda_rail::simulator::GreedySimulator::get_next_stop_ma(
    double max_displacement, double pos, double next_stop_pos) {
  /*
   * This function calculates the maximum moving authority to the next stop.
   */

  return std::min(max_displacement, next_stop_pos - pos);
}

double cda_rail::simulator::GreedySimulator::get_exit_vertex_order_ma(
    size_t tr, double pos, double max_displacement,
    const std::unordered_set<size_t>& trains_in_network,
    const std::unordered_set<size_t>& trains_left) const {
  /**
   * This function limits the moving authority if the train cannot leave the
   * network due to the exit vertex order.
   *
   * @param tr: The id of the train for which the moving authority is
   * calculated.
   * @param pos: The current position of the train on its route.
   * @param max_displacement: The maximum displacement of the train in the next
   * time step.
   * @param trains_in_network: A unordered_set containing the ids of trains that
   * are currently in the network.
   * @param trains_left: A unordered_set containing the ids of trains that have
   * not yet left the network.
   *
   * @return: The maximum moving authority to the exit vertex order.
   */

  if (get_train_edges_of_tr(tr).empty()) {
    return max_displacement; // No edges, no moving authority
  }
  const auto& last_edge = get_instance()->get_const_network().get_edge(
      get_train_edges_of_tr(tr).back());
  const auto& tr_schedule = get_instance()->get_const_schedule(tr);
  if (last_edge.target != tr_schedule.get_exit_vertex()) {
    return max_displacement; // Train does not leave the network at the end of
                             // its route
  }
  const auto& exit_vertex_order = get_vertex_orders_of_vertex(last_edge.target);
  const auto  idx               = std::ranges::find(exit_vertex_order, tr);
  if (idx == exit_vertex_order.begin() || idx == exit_vertex_order.end()) {
    // Train is the first in the exit vertex order (or does not leave), hence,
    // no restriction
    return max_displacement;
  }
  const auto& prev_tr = *(idx - 1); // Previous train in the exit vertex order
  const auto  prev_tr_entering =
      (get_instance()->get_const_schedule(prev_tr).get_entry_vertex() ==
       last_edge.target);
  if (trains_left.contains(prev_tr) ||
      (prev_tr_entering && trains_in_network.contains(prev_tr))) {
    // Previous train has already cleared vertex
    return max_displacement; // No restriction
  }

  // Train cannot leave the network due to the exit vertex order
  return std::min(max_displacement, train_edge_length(tr) - pos);
}

cda_rail::simulator::GreedySimulator::MaAndMaxVResult
cda_rail::simulator::GreedySimulator::get_ma_and_maxv(
    size_t tr, const std::vector<double>& train_velocities,
    std::optional<size_t> next_stop, double dt,
    cda_rail::index_set const&                     blocked_vertices,
    const std::vector<TrainPosition>&              train_positions,
    const std::unordered_set<size_t>&              trains_in_network,
    const std::unordered_set<size_t>&              trains_left,
    const std::vector<std::unordered_set<size_t>>& tr_on_edges,
    bool also_limit_speed_by_leaving_edges) const {
  const auto& train = get_instance()->get_const_train_list().get_train(tr);
  double      ma    = max_displacement(train, train_velocities.at(tr), dt);
  if (next_stop.has_value()) {
    ma = get_next_stop_ma(ma, train_positions.at(tr).front,
                          get_stop_positions_of_tr(tr).at(next_stop.value()));
  }
  ma = get_exit_vertex_order_ma(tr, train_positions.at(tr).front, ma,
                                trains_in_network, trains_left);
  ma = get_absolute_distance_ma(tr, ma, train_positions, train_velocities,
                                trains_in_network, trains_left, tr_on_edges);
  return get_future_max_speed_constraints(
      tr, train, train_positions.at(tr).front, train_velocities.at(tr), ma, dt,
      blocked_vertices, also_limit_speed_by_leaving_edges);
}

// MA to Speed Helper

double cda_rail::simulator::GreedySimulator::get_v1_from_ma(double v_0,
                                                            double ma, double d,
                                                            double dt) {
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
  round_small_numbers_to_zero_inplace(v_0);
  round_small_numbers_to_zero_inplace(ma);
  cda_rail::exceptions::throw_if_non_positive(d, EPS, "Deceleration");
  cda_rail::exceptions::throw_if_negative(v_0, "Initial velocity");
  cda_rail::exceptions::throw_if_non_positive(dt, "Time step");
  cda_rail::exceptions::throw_if_negative(ma, "Moving authority");

  if (v_0 * dt / 2.0 >= ma) {
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

// Train Movement Helper

bool cda_rail::simulator::GreedySimulator::move_train(
    size_t tr, double v_0, double v_1, double ma, double dt,
    std::vector<TrainPosition>& train_positions) {
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

  double move_distance =
      std::min(ma, (v_0 + v_1) * dt / 2.0); // Distance moved in the time step
  if (std::abs(move_distance) < EPS) {
    return false;
  }
  if (v_1 < V_MIN && std::abs(ma - move_distance) < STOP_TOLERANCE) {
    move_distance = ma;
  }
  train_positions.at(tr).front += move_distance; // Update the front position
  return move_distance > 0;
}

void cda_rail::simulator::GreedySimulator::update_rear_positions(
    std::vector<TrainPosition>& train_positions) const {
  /**
   * Update the rear positions of all trains based on their front positions and
   * train lengths. Tis function changes the train_positions passed by
   * reference.
   *
   * @param train_positions: A vector of pairs containing the rear and front
   */

  for (size_t tr = 0; tr < train_positions.size(); ++tr) {
    const auto& train = get_instance()->get_const_train_list().get_train(tr);
    train_positions.at(tr).rear = train_positions.at(tr).front -
                                  train.get_length(); // Update rear position
  }
}

// Final State Helper

cda_rail::simulator::GreedySimulator::DestinationType
cda_rail::simulator::GreedySimulator::tr_reached_end(
    size_t tr, const std::vector<TrainPosition>& train_pos) const {
  /**
   * This function checks if a train has reached the end of its route.
   * If yes, it determines if this is due to the train leaving the network or
   * stopping at the end of its route. In the latter case, it is checked if the
   * route end is a station stop.
   */

  const auto  route_len = train_edge_length(tr);
  const auto& pos       = train_pos.at(tr).front;
  if (pos < route_len) {
    // Train has not reached the end of its route
    return DestinationType::None;
  }
  // Determine type of end
  if (get_instance()->get_const_schedule(tr).get_exit_vertex() ==
      get_instance()
          ->get_const_network()
          .get_edge(get_train_edges_of_tr(tr).back())
          .target) {
    // Train leaves the network at the end of its route, hence it has to fully
    // leave the network
    return pos >= route_len + get_instance()
                                  ->get_const_train_list()
                                  .get_train(tr)
                                  .get_length()
               ? DestinationType::Network
               : DestinationType::None;
  }
  if (!get_stop_positions_of_tr(tr).empty() &&
      get_stop_positions_of_tr(tr).back() >= route_len - EPS) {
    // Train stops at the end of its route
    return DestinationType::Station;
  }
  // Train stops at the end of its route, but not at a station stop
  return DestinationType::Edge;
}
