#include "simulation/EdgeTrajectory.hpp"

#include <algorithm>
#include <iterator>
#include <optional>
#include <vector>

namespace cda_rail {

using BrakingPeriod = std::tuple<ulong, ulong>;

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

  // Modify speed targets to reach velocity before last edge transition
  BrakingPeriod brake_before_transit(double               target_speed,
                                     std::optional<ulong> hold_until_timestep);

  std::optional<BrakingPeriod>
  find_latest_braking_period(double target_speed) const;

  // Returns end of braking if braking is feasible
  std::optional<ulong> is_feasible_braking_point(ulong  start_braking,
                                                 double target_speed) const;

  TrainState get_state(ulong timestep) const;

  double distance_to_last_transition(ulong timestep) const;

  size_t get_relevant_trajectory(ulong timestep) const;

  TrainState read_initial_train_state() const;
};

}; // namespace cda_rail
