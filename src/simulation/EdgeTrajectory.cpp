#include "simulation/EdgeTrajectory.hpp"

cda_rail::EdgeTrajectory::EdgeTrajectory(const SimulationInstance& instance,
                                         const Train&              train,
                                         InitialEdgeState initial_state,
                                         SpeedTargets&    v_targets)
    : initial_timestep(initial_state.timestep), edge(initial_state.edge),
      orientation(initial_state.orientation) {
  double edge_length = instance.network.get_edge(initial_state.edge).length;
  double edge_length_divisor = 1 / edge_length;

  speed.push_back(initial_state.speed);
  position.push_back(initial_state.position);

  double speed_disparity;
  double acceleration;

  for (int timestep = 2; timestep <= instance.n_timesteps; timestep++) {
    speed_disparity = v_targets.find_target_speed(timestep - 1) - speed.back();
    acceleration    = std::signbit(speed_disparity) *
                   std::min(std::abs(speed_disparity), train.acceleration);
    speed.push_back(speed.back() + acceleration);
    position.push_back(position.back() +
                       (orientation * speed.back() * edge_length_divisor));

    // Leaving edge
    if (position.back() > 1 || position.back() < 0) {
      exit_point = (position.back() > 1);

      if (exit_point) {
        leftover_movement = (position.back() - 1) * edge_length;
      } else {
        leftover_movement = -position.back() * edge_length;
      }

      final_timestep = timestep - 1;

      // Remove the last timestep since it is not on edge
      position.pop_back();
      speed.pop_back();
      return;
    }
  }
}

std::optional<cda_rail::InitialEdgeState>
get_next_edge(double switch_direction) {}
