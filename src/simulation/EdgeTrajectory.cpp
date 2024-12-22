#include "simulation/EdgeTrajectory.hpp"

#include <algorithm>

cda_rail::EdgeTrajectory::EdgeTrajectory(SimulationInstance& instance,
                                         Train& train, SpeedTargets& v_targets,
                                         TrainState initial_state)
    : instance(instance), train(train),
      initial_timestep(initial_state.timestep), edge(initial_state.edge),
      orientation(initial_state.orientation) {
  double edge_length         = instance.network.get_edge(edge).length;
  double edge_length_divisor = 1 / edge_length;

  speeds.push_back(initial_state.speed);
  positions.push_back(initial_state.position);

  double speed_disparity;
  double acceleration;

  for (int timestep = initial_timestep + 1; timestep < instance.n_timesteps;
       timestep++) {
    speed_disparity = v_targets.find_target_speed(timestep - 1) - speeds.back();
    if (speed_disparity >= 0) {
      acceleration = std::min(speed_disparity, train.acceleration);
    } else {
      acceleration = std::max(speed_disparity, train.deceleration);
    }
    speeds.push_back(speeds.back() + acceleration);
    positions.push_back(positions.back() +
                        (orientation * speeds.back() * edge_length_divisor));

    if (positions.back() > 1 || positions.back() < 0)
      transition = determine_transition(positions.back(), speeds.back());
    positions.pop_back();
    speeds.pop_back();
    return;
  }

  transition = {};
}

cda_rail::EdgeTransition
cda_rail::EdgeTrajectory::determine_transition(double exit_position,
                                               double exit_speed) {
  double edge_length = instance.network.get_edge(edge).length;
  bool   exit_point  = (exit_position > 1);
  size_t traversed_node;
  bool   traversal_direction;
  double leftover_movement;

  if (exit_point) {
    return EdgeTransition{
        // Forward exit in edge direction
        .exit_point          = exit_point,
        .traversed_node      = instance.network.get_edge(edge).target,
        .traversal_direction = orientation,
        .leftover_movement   = (exit_position - 1) * edge_length,
        .traversal_speed     = exit_speed,
    };
  } else {
    return EdgeTransition{
        // Backward exit in edge direction
        .exit_point          = exit_point,
        .traversed_node      = instance.network.get_edge(edge).source,
        .traversal_direction = !orientation,
        .leftover_movement   = -exit_position * edge_length,
        .traversal_speed     = exit_speed,
    };
  }
}

cda_rail::EdgeTransitionResult
cda_rail::EdgeTrajectory::enter_next_edge(double switch_direction) {
  if (!transition.has_value())
    return EdgeTransitionResult{
        .outcome   = TIME_END,
        .new_state = {},
    };

  std::vector<size_t> viable_next_edges;
  if (transition.value().exit_point) {
    viable_next_edges = instance.network.get_successors(edge);
  } else {
    viable_next_edges = instance.network.get_predecessors(edge);
  }

  if (viable_next_edges.size() < 1) {
    return EdgeTransitionResult{
        .outcome   = DEADEND,
        .new_state = {},
    };
  }

  size_t next_edge = viable_next_edges.at(
      std::round(switch_direction * (viable_next_edges.size() - 1)));

  if (is_planned_stop()) {
    return EdgeTransitionResult{
        .outcome   = PLANNED_STOP,
        .new_state = {},
    };
  }

  double edge_entry_position;
  bool   edge_entry_point = (instance.network.get_edge(next_edge).target ==
                           transition.value().traversed_node);

  if (edge_entry_point) {
    edge_entry_position =
        1 - std::abs(transition.value().leftover_movement /
                     instance.network.get_edge(next_edge).length);
  } else {
    edge_entry_position = std::abs(transition.value().leftover_movement /
                                   instance.network.get_edge(next_edge).length);
  }

  EdgeTransitionOutcome outcome;
  if (std::abs(transition.value().traversal_speed) >
      instance.network.get_edge(next_edge).max_speed) {
    outcome = OVERSPEED;
  } else {
    outcome = NORMAL;
  }

  return EdgeTransitionResult{
      .outcome = outcome,
      .new_state =
          TrainState{
              .timestep    = initial_timestep + positions.size(),
              .edge        = next_edge,
              .position    = edge_entry_position,
              .orientation = !(edge_entry_point !=
                               !transition.value().traversal_direction), // XNOR
              .speed       = transition.value().traversal_speed,
          },
  };
}

bool cda_rail::EdgeTrajectory::is_planned_stop() {
  for (auto stop : instance.timetable.get_schedule(train.name).get_stops()) {
    auto stop_station = instance.timetable.get_station_list().get_station(
        stop.get_station_name());

    if (std::find(stop_station.tracks.begin(), stop_station.tracks.end(),
                  edge) != stop_station.tracks.end())
      return true;
  }

  return false;
}

ulong cda_rail::EdgeTrajectory::get_initial_timestep() {
  return initial_timestep;
}

ulong cda_rail::EdgeTrajectory::get_last_timestep() {
  // Size is larger by one than timesteps on edge
  return initial_timestep + positions.size() - 1;
}

ulong cda_rail::EdgeTrajectory::get_edge() { return edge; }

bool cda_rail::EdgeTrajectory::get_orientation() { return orientation; }

std::vector<double> cda_rail::EdgeTrajectory::get_positions() {
  return positions;
}

std::vector<double> cda_rail::EdgeTrajectory::get_speeds() { return speeds; }
