#include "simulation/EdgeTrajectory.hpp"

cda_rail::EdgeTrajectory::EdgeTrajectory(const SimulationInstance& instance,
                                         const Train&              train,
                                         SpeedTargets&             v_targets,
                                         InitialEdgeState initial_state)
    : initial_timestep(initial_state.timestep), edge(initial_state.edge),
      orientation(initial_state.orientation) {
  double edge_length         = instance.network.get_edge(edge).length;
  double edge_length_divisor = 1 / edge_length;

  speeds.push_back(initial_state.speed);
  positions.push_back(initial_state.position);

  double speed_disparity;
  double acceleration;

  for (int timestep = initial_timestep + 1; timestep <= instance.n_timesteps;
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
      return;
  }
}

cda_rail::EdgeTransition
cda_rail::EdgeTrajectory::get_transition(const SimulationInstance& instance,
                                         const Train&              train,
                                         double switch_direction) {
  if (initial_timestep + positions.size() - 1 >= instance.n_timesteps) {
    return EdgeTransition{
        .outcome   = cda_rail::TIME_END,
        .new_state = {},
    };
  }

  size_t              traversed_node;
  bool                traversal_direction;
  double              leftover_movement;
  std::vector<size_t> viable_next_edges;

  double edge_length     = instance.network.get_edge(edge).length;
  bool   edge_exit_point = (positions.back() > 1);

  if (edge_exit_point) {
    // Forward exit in edge direction
    traversed_node      = instance.network.get_edge(edge).target;
    traversal_direction = orientation;
    leftover_movement   = (positions.back() - 1) * edge_length;
    viable_next_edges   = instance.network.get_successors(edge);
  } else {
    // Backward exit in edge direction
    traversed_node      = instance.network.get_edge(edge).source;
    traversal_direction = !orientation;
    leftover_movement   = -positions.back() * edge_length;
    viable_next_edges   = instance.network.get_predecessors(edge);
  }

  if (viable_next_edges.size() < 1) {
    return EdgeTransition{
        .outcome   = cda_rail::DEADEND,
        .new_state = {},
    };
  }

  size_t next_edge = viable_next_edges.at(
      std::round(switch_direction * (viable_next_edges.size() - 1)));

  if (is_planned_stop(instance, train, next_edge)) {
    return EdgeTransition{
        .outcome   = cda_rail::PLANNED_STOP,
        .new_state = {},
    };
  }

  double edge_entry_position;
  bool   edge_entry_point =
      (instance.network.get_edge(next_edge).target == traversed_node);

  if (edge_entry_point) {
    edge_entry_position =
        1 - std::abs(leftover_movement /
                     instance.network.get_edge(next_edge).length);
  } else {
    edge_entry_position = std::abs(leftover_movement /
                                   instance.network.get_edge(next_edge).length);
  }

  return EdgeTransition{
      .outcome = cda_rail::NORMAL,
      .new_state =
          InitialEdgeState{
              .timestep = initial_timestep + positions.size() - 1,
              .edge     = next_edge,
              .position = edge_entry_position,
              .orientation =
                  !(edge_entry_point != !traversal_direction), // XNOR
              .speed = speeds.back(),
          },
  };
}

bool cda_rail::EdgeTrajectory::is_planned_stop(
    const SimulationInstance& instance, const Train& train, size_t next_edge) {
  size_t train_idx =
      instance.timetable.get_train_list().get_train_index(train.name);

  // TODO: check for stop
  // instance.timetable.get_schedule(train_idx).get_stops();
}
