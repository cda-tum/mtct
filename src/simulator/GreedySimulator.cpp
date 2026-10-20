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
#include <ranges>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {
// Highest speed between 0 and v_high that is not too early, given that 0 is
// not and that every speed above a too early one is too early as well.
template <typename TooEarly>
double highest_allowed_speed(double v_high, const TooEarly& too_early) {
  double v_low = 0;
  while (v_high - v_low > cda_rail::EPS) {
    const auto v_mid = (v_low + v_high) / 2.0;
    if (too_early(v_mid)) {
      v_high = v_mid;
    } else {
      v_low = v_mid;
    }
  }
  return v_low;
}
} // namespace

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast)

// ----------------
// CONSTRUCTOR

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

cda_rail::simulator::SimulatorResults
cda_rail::simulator::GreedySimulator::simulate(
    double dt, bool late_entry_possible, bool limit_speed_by_leaving_edges,
    bool save_trajectories, bool block_vertices_after_disappearing) const {
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
  for (size_t tr = 0; tr < number_of_trains; ++tr) {
    exit_times.at(tr) = get_instance()->get_const_schedule(tr).get_entry_time();
  }

  std::vector<double>              blocked_positions(number_of_trains, INF);
  std::vector<std::vector<double>> stop_times(number_of_trains);
  std::vector<std::map<double, PosVel>>
      train_trajectories; // time -> {pos, vel}

  // Find first time step
  double min_t = std::numeric_limits<double>::max();
  for (size_t tr = 0; tr < number_of_trains; ++tr) {
    auto const entry_time =
        get_instance()->get_const_schedule(tr).get_entry_time();
    min_t = std::min(min_t, entry_time);
  }

  // Initialize variables to keep track of positions and velocities
  if (save_trajectories) {
    train_trajectories.clear();
    train_trajectories.resize(number_of_trains);
  }
  std::vector<double> overshooting_times(number_of_trains, 0);
  // Time at which the rear of a train passes its exit vertex
  std::vector<double> exit_passing_times(number_of_trains, 0);

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

  auto build_results = [&exit_times, &stop_times, &vertex_headways,
                        &train_trajectories](bool success) {
    return SimulatorResults{.success         = success,
                            .exit_times      = std::move(exit_times),
                            .stop_times      = std::move(stop_times),
                            .vertex_headways = vertex_headways,
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
  while (cycles_without_movement < CycleLimit) {
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

      if (save_trajectories && !train_trajectories.at(tr).empty() &&
          train_trajectories.at(tr).rbegin()->first < t - dt - GRB_EPS) {
        // The train was skipped since its last sample, i.e., it stood in a
        // station. Hence, the end of that standstill is saved as well.
        train_trajectories.at(tr)[t - dt] = {.pos =
                                                 train_positions.at(tr).front,
                                             .vel = train_velocities.at(tr)};
      }

      // Calculate MA

      // A* partial-route search: trains disappear at route end, so reverse-edge
      // conflicts cannot occur by definition
      const auto tr_ma_data = get_ma_and_maxv(
          tr, train_velocities, tr_next_stop_id.at(tr), t, dt, blocked_vertices,
          train_positions, trains_in_network, trains_left, trains_on_edges,
          limit_speed_by_leaving_edges, block_vertices_after_disappearing);
      PLOGV << train_object.get_name() << " positioned at "
            << train_positions.at(tr).front
            << " has MA: " << train_positions.at(tr).front + tr_ma_data.ma
            << " and max velocity: " << tr_ma_data.max_v;

      auto tr_new_speed =
          std::min(tr_ma_data.max_v,
                   get_v1_from_ma(train_velocities.at(tr), tr_ma_data.ma,
                                  train_object.get_deceleration(), dt));
      auto const tr_route_len = train_edge_length(tr);
      auto const last_edge_leaves_network =
          get_instance()
              ->get_const_network()
              .get_edge(get_train_edges_of_tr(tr).back())
              .target ==
          get_instance()->get_const_schedule(tr).get_exit_vertex();
      // Where the train's front is once its rear has passed the exit vertex
      auto const tr_exit_pos = tr_route_len + train_object.get_length();
      if (tr_new_speed < V_MIN &&
          !(last_edge_leaves_network &&
            train_positions.at(tr).front +
                    std::min(tr_ma_data.ma,
                             (train_velocities.at(tr) + tr_new_speed) * dt /
                                 2.0) >=
                tr_exit_pos - GRB_EPS)) {
        // Train is stopped, unless it leaves within this time step, after
        // which its speed does not matter
        tr_new_speed = 0.0;
      }

      PLOGV << "tr_new_speed = " << tr_new_speed;

      // Move trains
      auto const prev_front = train_positions.at(tr).front;
      // Time and speed at which the train's rear passes the exit vertex, if
      // it does so within this time step
      struct ExitPassing {
        double t;
        double vel;
      };
      std::optional<ExitPassing> exit_passing;
      if (move_train(tr, train_velocities.at(tr), tr_new_speed, tr_ma_data.ma,
                     dt, train_positions)) {
        movement_detected = true;

        if (last_edge_leaves_network &&
            train_positions.at(tr).front > tr_exit_pos - GRB_EPS) {
          if (prev_front < tr_exit_pos - GRB_EPS) {
            // The train's rear passes the exit vertex within this time step
            exit_passing = ExitPassing{.t = t, .vel = tr_new_speed};
            auto const overshooting_distance =
                train_positions.at(tr).front - tr_exit_pos;
            if (overshooting_distance > GRB_EPS) {
              auto const reverse_acceleration =
                  (train_velocities.at(tr) - tr_new_speed) / dt;
              auto const overshooting_time =
                  time_to_travel_distance_with_constant_acceleration(
                      tr_new_speed, reverse_acceleration,
                      overshooting_distance);
              exit_passing->t -= overshooting_time;
              exit_passing->vel += reverse_acceleration * overshooting_time;
            }
            exit_passing_times.at(tr) = exit_passing->t;
            PLOGV << "Train " << train_object.get_name()
                  << " passes the exit vertex with its rear at time "
                  << exit_passing->t;
          }
          // The train cannot move any further than until it has left the
          // network, where it has to wait if it is not allowed to disappear
          train_positions.at(tr).front = tr_exit_pos;
        } else if (!last_edge_leaves_network &&
                   tr_route_len < train_positions.at(tr).front + GRB_EPS) {
          PLOGV << "Train " << train_object.get_name()
                << " overshot the end of its route at "
                << train_positions.at(tr).front << " and is now stopped.";
          auto const overshooting_distance =
              train_positions.at(tr).front - tr_route_len;
          PLOGV << "Overshooting distance: " << overshooting_distance;
          if (overshooting_distance > GRB_EPS) {
            auto const reverse_acceleration =
                (train_velocities.at(tr) - tr_new_speed) / dt;
            overshooting_times.at(tr) =
                time_to_travel_distance_with_constant_acceleration(
                    tr_new_speed, reverse_acceleration, overshooting_distance);
            PLOGV << "Overshooting time: " << overshooting_times.at(tr);
          }
          train_positions.at(tr).front = tr_route_len;
          tr_new_speed                 = 0.0;
        }
      }
      train_velocities.at(tr) = tr_new_speed;
      PLOGV << "At time " << t << ", " << train_object.get_name()
            << " moved to " << train_positions.at(tr).front << " with speed "
            << tr_new_speed << " and MA "
            << train_positions.at(tr).front +
                   cda_rail::braking_distance(tr_new_speed,
                                              train_object.get_deceleration());

      if (save_trajectories) {
        // A train that leaves is sampled when its rear passes the exit vertex
        train_trajectories.at(
            tr)[exit_passing.has_value() ? exit_passing->t : t] = {
            .pos = train_positions.at(tr).front,
            .vel = exit_passing.has_value() ? exit_passing->vel
                                            : train_velocities.at(tr)};
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
      const auto tr_status = tr_reached_end(tr, train_positions,
                                            tr_next_stop_id.at(tr).has_value());
      auto const tr_exit_time =
          get_instance()->get_const_schedule(tr).get_exit_time();
      auto const exit_blocked =
          is_exit_vertex_blocked(tr, trains_in_network, trains_left);
      auto const has_remaining_stop    = tr_next_stop_id.at(tr).has_value();
      auto const after_stop_until_time = t >= tr_stop_until.at(tr);
      if (tr_status == DestinationType::Network && t + GRB_EPS < tr_exit_time) {
        PLOGV << "At time " << t << ", " << train_list.get_train(tr).get_name()
              << " has reached the end of its route but cannot leave the "
                 "network until "
              << tr_exit_time;
      }
      if (tr_status == DestinationType::Network && exit_blocked) {
        PLOGV << "At time " << t << ", " << train_list.get_train(tr).get_name()
              << " has reached the end of its route but cannot leave the "
                 "network since the exit is blocked by train order.";
      }
      if (tr_status == DestinationType::Network && has_remaining_stop) {
        PLOGV << "At time " << t << ", " << train_list.get_train(tr).get_name()
              << " has reached the end of its route but cannot leave the "
                 "network since there is at least one scheduled stop left.";
      }
      if (!after_stop_until_time) {
        PLOGV << "At time " << t << ", " << train_list.get_train(tr).get_name()
              << " cannot be removed until " << tr_stop_until.at(tr)
              << " due to station stop requirements.";
      }
      if (tr_status == DestinationType::Network &&
          t + GRB_EPS >= tr_exit_time && !exit_blocked && !has_remaining_stop &&
          after_stop_until_time) {
        trains_to_remove.emplace_back(tr);
        trains_left.insert(tr);
        trains_finished_simulating.insert(tr);
        // The train leaves when its rear passes the exit vertex, which is not
        // before its exit time
        exit_times.at(tr) = std::max(exit_passing_times.at(tr), tr_exit_time);
        const auto& exit_vertex_idx =
            get_instance()->get_const_schedule(tr).get_exit_vertex();
        const auto& exit_vertex =
            get_instance()->get_const_network().get_vertex(exit_vertex_idx);
        vertex_headways.at(exit_vertex_idx) =
            exit_times.at(tr) + exit_vertex.headway;
        PLOGV << "At time " << t << ", " << train_list.get_train(tr).get_name()
              << " left the network.";
      } else if (tr_status == DestinationType::Edge && after_stop_until_time) {
        trains_finished_simulating.insert(tr);
        exit_times.at(tr) = t - overshooting_times.at(tr);
        PLOGV << "At time " << t << ", " << train_list.get_train(tr).get_name()
              << " reached the end of its route on an edge within the network "
                 "and overshooting time "
              << overshooting_times.at(tr) << " is deduced from the exit time.";
        trains_to_remove.emplace_back(tr);
        trains_left.insert(tr);
        PLOGV << "At time " << exit_times.at(tr) << ", "
              << train_list.get_train(tr).get_name()
              << " disappeared at the end of its route.";
        if (block_vertices_after_disappearing) {
          auto const blocking_vertex =
              get_instance()
                  ->get_const_network()
                  .get_edge(get_train_edges_of_tr(tr).back())
                  .target;
          for (size_t tr_other = 0; tr_other < number_of_trains; ++tr_other) {
            if (tr_other == tr) {
              continue;
            }
            if (trains_finished_simulating.contains(tr_other)) {
              continue;
            }
            if (trains_left.contains(tr_other)) {
              continue;
            }

            auto const blocking_vertex_pos =
                get_vertex_pos(tr_other, blocking_vertex);
            if (blocking_vertex_pos.is_on_route &&
                train_positions.at(tr_other).front <= blocking_vertex_pos.pos) {
              blocked_positions.at(tr_other) = std::min(
                  blocked_positions.at(tr_other), blocking_vertex_pos.pos);
              PLOGV << "At time " << t << ", "
                    << train_list.get_train(tr).get_name() << " blocks vertex "
                    << get_instance()
                           ->get_const_network()
                           .get_vertex(blocking_vertex)
                           .name
                    << " for train "
                    << train_list.get_train(tr_other).get_name()
                    << " at position " << blocking_vertex_pos.pos
                    << " and is now blocked until "
                    << blocked_positions.at(tr_other);
            }
          }
        }
      } else if (tr_status == DestinationType::Station) {
        assert(tr_next_stop_id.at(tr).has_value());
        const auto& last_stop =
            get_instance()->get_const_schedule(tr).get_stops().at(
                tr_next_stop_id.at(tr).value());
        tr_stop_until.at(tr) = std::max(t + last_stop.get_service_duration(),
                                        last_stop.get_earliest_departure());
        stop_times.at(tr).emplace_back(t);
        tr_next_stop_id.at(tr) = {};
        PLOGV << "At time " << t << ", " << train_list.get_train(tr).get_name()
              << " reached the end of its route at station "
              << last_stop.get_station().name << ", stopping until "
              << tr_stop_until.at(tr);
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
    if (!trains_to_remove.empty()) {
      // A train leaving the network is progress, even if no train moved
      movement_detected = true;
    }

    if (block_vertices_after_disappearing) {
      for (auto const& tr : trains_in_network) {
        if (train_positions.at(tr).front >=
            blocked_positions.at(tr) + GRB_EPS) {
          PLOGV << "At time " << t << ", "
                << train_list.get_train(tr).get_name() << " is at position "
                << train_positions.at(tr).front
                << " which is past its blocked position "
                << blocked_positions.at(tr);
          // Simulation failed
          return build_results(false);
        }
      }
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
      const auto entry_bound =
          m_position_bound ? m_position_bound(tr, t) : std::nullopt;
      if (vertex_headways.at(train_schedule.get_entry_vertex()) > t) {
        PLOGV << "At time " << t << ", " << train_list.get_train(tr).get_name()
              << " cannot enter the network at " << entry_vertex.name
              << " due to vertex headway constraints until time "
              << vertex_headways.at(train_schedule.get_entry_vertex());
      } else if (!is_ok_to_enter(tr, train_positions, train_velocities,
                                 trains_in_network, trains_on_edges,
                                 trains_left)) {
        PLOGV << "At time " << t << ", " << train_list.get_train(tr).get_name()
              << " cannot enter the network at " << entry_vertex.name
              << " due to moving authority constraints.";
      } else if (entry_bound.has_value() &&
                 tr_braking_distance(tr,
                                     train_schedule.get_initial_velocity()) >
                     entry_bound->braking_point + GRB_EPS) {
        PLOGV << "At time " << t << ", " << train_list.get_train(tr).get_name()
              << " cannot enter the network at " << entry_vertex.name
              << " due to its position bound.";
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
      if (std::ranges::any_of(vertex_headways, [t](double vertex_headway) {
            return vertex_headway > t;
          })) {
        PLOGV << "Vertex headway constraint prevents movement.";
        reason_found = true;
      } else if (std::ranges::any_of(tr_stop_until, [t, dt](double stop_time) {
                   return stop_time + dt > t;
                 })) {
        PLOGV << "Train stop constraint prevents movement.";
        reason_found = true;
      } else if (m_position_bound &&
                 std::ranges::any_of(
                     std::views::iota(size_t{0}, number_of_trains),
                     [this, t, &trains_finished_simulating](size_t tr) {
                       return !trains_finished_simulating.contains(tr) &&
                              m_position_bound(tr, t).has_value();
                     })) {
        PLOGV << "Position bound prevents movement.";
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
      // Waiting for a reason that resolves over time is no lack of progress
      cycles_without_movement = 0;
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

cda_rail::simulator::GreedySimulator::PosOnEdgeReturn
cda_rail::simulator::GreedySimulator::get_position_on_route_edge(
    size_t tr, const TrainPosition& pos, size_t edge_number,
    std::vector<double> milestones) const {
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

bool cda_rail::simulator::GreedySimulator::is_ok_to_enter(
    size_t tr, const std::vector<TrainPosition>& train_positions,
    const std::vector<double>&                     train_velocities,
    const std::unordered_set<size_t>&              trains_in_network,
    const std::vector<std::unordered_set<size_t>>& tr_on_edges,
    const std::unordered_set<size_t>&              trains_left) const {
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
      if (occ.tr_on_edge && pos.rear + EPS <= bd - milestones.at(i)) {
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
      if (!trains_left.contains(other_tr) &&
          (!trains_in_network.contains(other_tr) ||
           !is_behind_ttd(other_tr, ttd_sec.value(), other_pos))) {
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
    const std::vector<std::unordered_set<size_t>>& tr_on_edges,
    bool also_check_reverse_edges) const {
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

      if (also_check_reverse_edges) {
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
            [[maybe_unused]] const auto [occ_rev, pos_rev] =
                get_position_on_edge(
                    other_tr,
                    {.rear  = train_positions.at(other_tr).rear,
                     .front = train_positions.at(other_tr).front +
                              tr_braking_distance(
                                  other_tr, train_velocities.at(other_tr))},
                    reverse_edge_id.value());
            if (occ_rev.tr_on_edge) {
              // Other train has already been cleared to enter the reverse edge
              return milestones.at(i) - train_positions.at(tr).front;
            }
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
    double max_displacement, double current_time, double dt,
    cda_rail::index_set const& blocked_vertices,
    bool                       also_limit_by_leaving_edges) const {
  // NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast)
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

  // Check exit, i.e., the train's rear passing the exit vertex
  const auto& last_edge_id = get_train_edges_of_tr(tr).back();
  const auto& last_edge =
      get_instance()->get_const_network().get_edge(last_edge_id);
  const auto tr_schedule = get_instance()->get_const_schedule(tr);
  const bool last_edge_leaves_network =
      (last_edge.target == tr_schedule.get_exit_vertex());

  if (last_edge_leaves_network) {
    // Where the train's front is once its rear has passed the exit vertex
    const auto exit_pos = milestones.back() + train.get_length();

    if (pos >= milestones.back()) {
      // Beyond the exit vertex, the route continues at the speed limit of its
      // last edge, irrespective of also_limit_by_leaving_edges
      retval.vel = std::min(retval.vel, last_edge.max_speed);
    }

    if (pos >= exit_pos - GRB_EPS) {
      // The train has left the network but is not allowed to disappear yet.
      // Hence, it waits there.
      retval.pos = std::min(retval.pos, std::max(0.0, exit_pos - pos));
    } else if (pos + max_displacement >= exit_pos) {
      // Train's moving authority can potentially leave the network
      retval = speed_restriction_helper(retval.pos, retval.vel, pos, exit_pos,
                                        v_0, tr_schedule.get_exit_velocity(),
                                        train.get_deceleration(), dt);
    }

    if (pos < exit_pos - GRB_EPS) {
      retval = apply_exit_time(tr, train, pos, v_0, retval, current_time, dt);
    }
  }
  return {.ma = retval.pos, .max_v = retval.vel};
  // NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)
}

cda_rail::simulator::PosVel
cda_rail::simulator::GreedySimulator::apply_exit_time(
    size_t tr, const cda_rail::Train& train, double pos, double v_0,
    PosVel ma_and_max_v, double current_time, double dt) const {
  const auto exit_time = get_instance()->get_const_schedule(tr).get_exit_time();
  if (current_time - dt + GRB_EPS >= exit_time) {
    // This time step starts at or after the exit time
    return ma_and_max_v;
  }
  const auto exit_vertex_pos = train_edge_length(tr);
  const auto exit_pos        = exit_vertex_pos + train.get_length();
  // Nothing has to be restricted if, after this time step, the train can
  // still stop with its front at the exit vertex (or has already passed it),
  // and if it cannot leave within this time step and needs at least until its
  // exit time to do so afterwards
  const auto max_pos_after          = pos + max_displacement(train, v_0, dt);
  const auto min_remaining_distance = exit_pos - max_pos_after;
  if ((pos > exit_vertex_pos + GRB_EPS ||
       max_pos_after <= exit_vertex_pos + GRB_EPS) &&
      min_remaining_distance > 0 &&
      current_time + (min_remaining_distance / train.get_max_speed()) +
              GRB_EPS >=
          exit_time) {
    return ma_and_max_v;
  }

  const auto& last_edge = get_instance()->get_const_network().get_edge(
      get_train_edges_of_tr(tr).back());
  const auto a = train.get_acceleration();
  const auto d = train.get_deceleration();
  // The front's position at the end of this time step, given the speed v_1
  // there
  const auto pos_after = [&](double v_1) {
    return pos + ((v_0 + v_1) / 2.0 * dt);
  };

  // The front must not pass the exit vertex if the train could then not
  // leave without stopping outside the network and not before its exit time.
  // The latest it can leave without stopping is by accelerating just as much
  // as needed and braking to the minimal speed until its rear passes the exit
  // vertex.
  const auto v_end = std::min(
      V_MIN, get_instance()->get_const_schedule(tr).get_exit_velocity());
  const auto latest_exit_without_stopping = [&](double pos_1, double v_1) {
    const auto s = exit_pos - pos_1;
    if (s <= 0) {
      return current_time;
    }
    if (v_1 * v_1 >= (2 * d * s) + (v_end * v_end)) {
      return current_time + ((v_1 - std::sqrt((v_1 * v_1) - (2 * d * s))) / d);
    }
    const auto v_peak = std::sqrt(
        ((2 * a * d * s) + (d * v_1 * v_1) + (a * v_end * v_end)) / (a + d));
    const auto v_max =
        std::max(std::min(train.get_max_speed(), last_edge.max_speed), v_1);
    if (v_peak <= v_max) {
      return current_time + ((v_peak - v_1) / a) + ((v_peak - v_end) / d);
    }
    // In between, the train drives at its maximal speed
    const auto cruising_distance =
        s - (((v_max * v_max) - (v_1 * v_1)) / (2 * a)) -
        (((v_max * v_max) - (v_end * v_end)) / (2 * d));
    return current_time + ((v_max - v_1) / a) + (cruising_distance / v_max) +
           ((v_max - v_end) / d);
  };
  const auto front_passes_early = [&](double v_1) {
    const auto pos_1 = pos_after(v_1);
    return pos_1 + braking_distance(v_1, d) > exit_vertex_pos + GRB_EPS &&
           latest_exit_without_stopping(pos_1, v_1) + GRB_EPS < exit_time;
  };

  // The rear must not pass the exit vertex before the exit time. As long as
  // this time step ends before, the train has to be able to stop before it
  // leaves or, braking fully, leave no earlier than its exit time. Within the
  // time step that contains the exit time, the front must not be beyond its
  // exit position at the exit time, where the speed changes linearly within a
  // time step. Following this limit, the train brakes fully and leaves exactly
  // at its exit time.
  const auto rear_passes_early = [&](double v_1) {
    if (current_time + GRB_EPS < exit_time) {
      const auto remaining_distance = exit_pos - pos_after(v_1);
      if (remaining_distance <= 0) {
        return true;
      }
      if (braking_distance(v_1, d) <= remaining_distance) {
        return false;
      }
      return current_time +
                 ((v_1 -
                   std::sqrt(std::max(0.0, (v_1 * v_1) -
                                               (2 * d * remaining_distance)))) /
                  d) +
                 GRB_EPS <
             exit_time;
    }
    const auto tau = exit_time - (current_time - dt);
    return pos + (v_0 * tau) + ((v_1 - v_0) * tau * tau / (2 * dt)) >
           exit_pos + GRB_EPS;
  };

  const auto v_1 = [&]() {
    return std::min(ma_and_max_v.vel,
                    get_v1_from_ma(v_0, ma_and_max_v.pos, d, dt));
  };

  // Since a train stops at its moving authority, it can always wait at the
  // exit vertex before its front has passed it
  if (pos <= exit_vertex_pos + GRB_EPS && current_time + GRB_EPS < exit_time &&
      front_passes_early(v_1())) {
    const auto v_allowed = highest_allowed_speed(v_1(), front_passes_early);
    if (v_allowed < V_MIN ||
        pos_after(v_allowed) + braking_distance(v_allowed, d) <=
            exit_vertex_pos + GRB_EPS) {
      // Passing the exit vertex is too early, hence, the train has to be able
      // to wait there
      PLOGV << "At time " << current_time << ", train " << train.get_name()
            << " has to be able to stop at the exit vertex in order not to "
               "leave before "
            << exit_time;
      ma_and_max_v.pos =
          std::min(ma_and_max_v.pos, std::max(0.0, exit_vertex_pos - pos));
    } else {
      ma_and_max_v.vel = std::min(ma_and_max_v.vel, v_allowed);
    }
  }

  if (std::abs(pos - exit_vertex_pos) <= GRB_EPS && v_0 < V_MIN &&
      current_time + GRB_EPS < exit_time &&
      earliest_exit_from_standstill(tr, train, pos, current_time, dt) <=
          exit_time + GRB_EPS) {
    // The train waits at the exit vertex as long as it can still leave in time
    // afterwards
    ma_and_max_v.pos = 0;
  }

  if (rear_passes_early(v_1())) {
    // If even stopping does not help anymore, the train brakes as much as it
    // can
    const auto v_allowed =
        rear_passes_early(0) ? std::max(0.0, v_0 - (d * dt))
                             : highest_allowed_speed(v_1(), rear_passes_early);
    PLOGV << "At time " << current_time << ", train " << train.get_name()
          << " has speed limit " << v_allowed
          << " in order not to leave before " << exit_time;
    ma_and_max_v.vel = std::min(ma_and_max_v.vel, v_allowed);
  }
  return ma_and_max_v;
}

double cda_rail::simulator::GreedySimulator::earliest_exit_from_standstill(
    size_t tr, const cda_rail::Train& train, double pos, double start_time,
    double dt) const {
  // The train moves as in simulate, where a time after its exit time lifts
  // the restrictions of apply_exit_time
  const auto exit_pos = train_edge_length(tr) + train.get_length();
  double     v        = 0;
  double     t        = start_time;
  while (pos < exit_pos - GRB_EPS) {
    t += dt;
    const auto [ma, max_v] = get_future_max_speed_constraints(
        tr, train, pos, v, max_displacement(train, v, dt), INF, dt, {}, true);
    auto v_1 =
        std::min(max_v, get_v1_from_ma(v, ma, train.get_deceleration(), dt));
    if (v_1 < V_MIN &&
        pos + std::min(ma, (v + v_1) * dt / 2.0) < exit_pos - GRB_EPS) {
      v_1 = 0;
    }
    auto move_distance = std::min(ma, (v + v_1) * dt / 2.0);
    if (v_1 < V_MIN && std::abs(ma - move_distance) < STOP_TOLERANCE) {
      move_distance = ma;
    }
    if (move_distance < EPS) {
      return INF;
    }
    if (pos + move_distance > exit_pos + GRB_EPS) {
      // The rear passes the exit vertex within this time step
      return t - time_to_travel_distance_with_constant_acceleration(
                     v_1, (v - v_1) / dt, pos + move_distance - exit_pos);
    }
    pos += move_distance;
    v = v_1;
  }
  return t;
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

  return is_exit_vertex_blocked(tr, trains_in_network, trains_left)
             ? std::min(max_displacement, train_edge_length(tr) - pos)
             : max_displacement;
}

bool cda_rail::simulator::GreedySimulator::is_exit_vertex_blocked(
    size_t tr, const std::unordered_set<size_t>& trains_in_network,
    const std::unordered_set<size_t>& trains_left) const {
  const auto& tr_schedule = get_instance()->get_const_schedule(tr);
  const auto& exit_vertex_order =
      get_vertex_orders_of_vertex(tr_schedule.get_exit_vertex());
  const auto idx = std::ranges::find(exit_vertex_order, tr);
  if (idx == exit_vertex_order.begin() || idx == exit_vertex_order.end()) {
    // Train is the first in the exit vertex order (or does not leave), hence,
    // no restriction
    return false;
  }

  const auto& prev_tr = *(idx - 1); // Previous train in the exit vertex order
  const auto  prev_tr_entering =
      (get_instance()->get_const_schedule(prev_tr).get_entry_vertex() ==
       tr_schedule.get_exit_vertex());
  const bool previous_train_cleared_vertex =
      trains_left.contains(prev_tr) ||
      (prev_tr_entering && trains_in_network.contains(prev_tr));

  return !previous_train_cleared_vertex;
}

cda_rail::simulator::GreedySimulator::MaAndMaxVResult
cda_rail::simulator::GreedySimulator::apply_position_bound(
    size_t tr, double pos, double v_0, MaAndMaxVResult ma_and_max_v, double t,
    double dt) const {
  if (!m_position_bound) {
    return ma_and_max_v;
  }
  const auto bound = m_position_bound(tr, t);
  if (!bound.has_value()) {
    return ma_and_max_v;
  }

  ma_and_max_v.ma =
      std::min(ma_and_max_v.ma, std::max(bound->braking_point - pos,
                                         tr_braking_distance(tr, v_0)));

  // With linearly changing speed, the front moves (v_0 + v_1) * dt / 2 within
  // the time step, which bounds v_1 if the front is to stay behind the bound
  const auto& train = get_instance()->get_const_train_list().get_train(tr);
  const auto  v_full_braking =
      std::max(0.0, v_0 - (train.get_deceleration() * dt));
  const auto front_max_v =
      std::max((2.0 * (bound->front - pos) / dt) - v_0, v_full_braking);
  ma_and_max_v.max_v = std::min(ma_and_max_v.max_v, front_max_v);
  if (front_max_v < V_MIN) {
    // The train stops, and a stopping train is moved up to its moving
    // authority, which hence must not lie beyond the bound either
    ma_and_max_v.ma =
        std::min(ma_and_max_v.ma,
                 std::max(bound->front - pos, tr_braking_distance(tr, v_0)));
  }
  return ma_and_max_v;
}

cda_rail::simulator::GreedySimulator::MaAndMaxVResult
cda_rail::simulator::GreedySimulator::get_ma_and_maxv(
    size_t tr, const std::vector<double>& train_velocities,
    std::optional<size_t> next_stop, double current_time, double dt,
    cda_rail::index_set const&                     blocked_vertices,
    const std::vector<TrainPosition>&              train_positions,
    const std::unordered_set<size_t>&              trains_in_network,
    const std::unordered_set<size_t>&              trains_left,
    const std::vector<std::unordered_set<size_t>>& tr_on_edges,
    bool also_limit_speed_by_leaving_edges,
    bool also_check_reverse_edges) const {
  const auto& train = get_instance()->get_const_train_list().get_train(tr);
  double      ma    = max_displacement(train, train_velocities.at(tr), dt);
  if (next_stop.has_value()) {
    ma = get_next_stop_ma(ma, train_positions.at(tr).front,
                          get_stop_positions_of_tr(tr).at(next_stop.value()));
  }
  ma = get_exit_vertex_order_ma(tr, train_positions.at(tr).front, ma,
                                trains_in_network, trains_left);
  ma = get_absolute_distance_ma(tr, ma, train_positions, train_velocities,
                                trains_in_network, trains_left, tr_on_edges,
                                also_check_reverse_edges);
  // The movement of this time step yields the train's position at current_time
  return apply_position_bound(
      tr, train_positions.at(tr).front, train_velocities.at(tr),
      get_future_max_speed_constraints(tr, train, train_positions.at(tr).front,
                                       train_velocities.at(tr), ma,
                                       current_time, dt, blocked_vertices,
                                       also_limit_speed_by_leaving_edges),
      current_time, dt);
}

double cda_rail::simulator::GreedySimulator::get_v1_from_ma(double v_0,
                                                            double ma, double d,
                                                            double dt) {
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

bool cda_rail::simulator::GreedySimulator::move_train(
    size_t tr, double v_0, double v_1, double ma, double dt,
    std::vector<TrainPosition>& train_positions) {
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
  for (size_t tr = 0; tr < train_positions.size(); ++tr) {
    const auto& train = get_instance()->get_const_train_list().get_train(tr);
    train_positions.at(tr).rear = train_positions.at(tr).front -
                                  train.get_length(); // Update rear position
  }
}

cda_rail::simulator::GreedySimulator::DestinationType
cda_rail::simulator::GreedySimulator::tr_reached_end(
    size_t tr, const std::vector<TrainPosition>& train_pos,
    bool has_stop_left) const {
  const auto  route_len = train_edge_length(tr);
  const auto& pos       = train_pos.at(tr).front;
  if (pos < route_len - GRB_EPS) {
    // Train has not reached the end of its route
    return DestinationType::None;
  }
  // Determine type of end
  if (get_instance()->get_const_schedule(tr).get_exit_vertex() ==
      get_instance()
          ->get_const_network()
          .get_edge(get_train_edges_of_tr(tr).back())
          .target) {
    const auto tr_len =
        get_instance()->get_const_train_list().get_train(tr).get_length();
    if (pos < route_len + tr_len - GRB_EPS) {
      // The train's rear has not passed the exit vertex yet
      return DestinationType::None;
    }
    return DestinationType::Network;
  }
  if (has_stop_left && !get_stop_positions_of_tr(tr).empty() &&
      get_stop_positions_of_tr(tr).back() >= route_len - GRB_EPS) {
    // Train stops at the end of its route
    return DestinationType::Station;
  }
  // Train stops at the end of its route, but not at a station stop
  return DestinationType::Edge;
}

cda_rail::simulator::GreedySimulator::VertexRoutePos
cda_rail::simulator::GreedySimulator::get_vertex_pos(size_t tr,
                                                     size_t vertex) const {
  double      pos{0.0};
  auto const& tr_edges = get_train_edges_of_tr(tr);
  for (size_t i = 0; i < tr_edges.size(); ++i) {
    const auto& edge =
        get_instance()->get_const_network().get_edge(tr_edges.at(i));
    if (i == 0 && edge.source == vertex) {
      return {.is_on_route = true, .pos = pos};
    }
    pos += edge.length;
    if (edge.target == vertex) {
      return {.is_on_route = true, .pos = pos};
    }
  }
  return {.is_on_route = false, .pos = INF};
}
