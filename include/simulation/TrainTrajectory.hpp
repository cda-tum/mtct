#include "simulation/EdgeTrajectory.hpp"

#include <algorithm>
#include <iterator>
#include <optional>
#include <vector>

namespace cda_rail {

class TrainTrajectory {
  /**
   * Path of a single train over entire timespan
   * Repairs solution speeds to be feasible
   */
  SimulationInstance& instance;
  Train&              train;

  std::vector<EdgeTrajectory> edge_trajs;
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
                     std::optional<ulong> hold_until_timestep) const;

  bool is_feasible_braking_point(ulong timestep, double target_speed) const;

  TrainState get_state(ulong timestep) const;

  double distance_to_last_transition(ulong timestep) const;

  size_t get_relevant_trajectory(ulong timestep) const;

  TrainState read_initial_train_state() const;
};

}; // namespace cda_rail
