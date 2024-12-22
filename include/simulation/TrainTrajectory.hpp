#include "simulation/EdgeTrajectory.hpp"

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

  std::vector<EdgeTrajectory>   edge_trajectories;
  std::vector<InitialEdgeState> initial_edge_states;
  RoutingSolution               solution;

public:
  TrainTrajectory(SimulationInstance& instance, Train& train,
                  RoutingSolution solution);

  // Modify speed targets to reach velocity before edge transition
  SpeedTargets match_velocity(EdgeTransition transition, SpeedTargets v_targets,
                              double               target_speed,
                              std::optional<ulong> hold_until_timestep);

  std::tuple<ulong, ulong>
  find_braking_point(EdgeTransition transition, SpeedTargets v_targets,
                     double               target_speed,
                     std::optional<ulong> hold_until_timestep);

  InitialEdgeState read_initial_train_state();
};

}; // namespace cda_rail
