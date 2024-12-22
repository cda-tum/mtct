#include "simulation/EdgeTrajectory.hpp"

#include <iterator>
#include <optional>
#include <vector>

namespace cda_rail {

class TrainTrajectory {
  /**
   * Path of a single train over entire timespan
   * Repairs solution speeds
   */
  SimulationInstance& instance;
  Train&              train;

  std::vector<EdgeTrajectory> edge_trajectories;
  std::vector<TrainState>     initial_edge_states;
  RoutingSolution             solution;

public:
  TrainTrajectory(SimulationInstance& instance, Train& train,
                  RoutingSolution solution);

  // Modify speed targets to reach velocity before edge transition
  SpeedTargets match_velocity(double               target_speed,
                              std::optional<ulong> hold_until_timestep);

  std::tuple<ulong, ulong>
  find_braking_point(double               target_speed,
                     std::optional<ulong> hold_until_timestep);

  TrainState read_initial_train_state();

  bool is_feasible_braking_point(ulong timestep, double target_speed);

  TrainState get_state(ulong timestep);

  size_t find_relevant_trajectory(ulong timestep);
};

}; // namespace cda_rail
