#include "simulation/EdgeTrajectory.hpp"

cda_rail::EdgeTrajectory::EdgeTrajectory(const SimulationInstance& instance,
                                         const Train&              train,
                                         InitialEdgeState initial_state,
                                         SpeedTargets&    v_targets)
    : initial_timestep(initial_state.timestep), edge(initial_state.edge),
      orientation(initial_state.orientation) {
  double edge_length         = instance.network.get_edge(edge).length;
  double edge_length_divisor = 1 / edge_length;

  speed.push_back(initial_state.speed);
  position.push_back(initial_state.position);

  double speed_disparity;
  double acceleration;

  for (int timestep = 2; timestep <= instance.n_timesteps; timestep++) {
    speed_disparity = v_targets.find_target_speed(timestep - 1) - speed.back();
    if (speed_disparity >= 0) {
      acceleration = std::min(speed_disparity, train.acceleration);
    } else {
      acceleration = std::max(speed_disparity, train.deceleration);
    }
    speed.push_back(speed.back() + acceleration);
    position.push_back(position.back() +
                       (orientation * speed.back() * edge_length_divisor));

    if (position.back() > 1 || position.back() < 0)
      return;
  }
}

std::optional<cda_rail::InitialEdgeState>
cda_rail::EdgeTrajectory::get_next_edge(
    const cda_rail::SimulationInstance& instance, double switch_direction) {
  double              edge_length    = instance.network.get_edge(edge).length;
  bool                exit_point     = (position.back() > 1);
  size_t              final_timestep = position.size() - 1;
  size_t              traversed_node;
  bool                traversal_direction;
  double              leftover_movement;
  std::vector<size_t> viable_next_edges;

  if (exit_point) {
    // Forward exit in edge direction
    traversed_node      = instance.network.get_edge(edge).target;
    traversal_direction = orientation;
    leftover_movement   = (position.back() - 1) * edge_length;
    viable_next_edges   = instance.network.get_successors(edge);
  } else {
    // Backward exit in edge direction
    traversed_node      = instance.network.get_edge(edge).source;
    traversal_direction = !orientation;
    leftover_movement   = -position.back() * edge_length;
    viable_next_edges   = instance.network.get_predecessors(edge);
  }

  size_t next_edge_selection =
      std::round(switch_direction * viable_next_edges.size() - 1);
}
