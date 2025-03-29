#pragma once

#include "datastructure/Route.hpp"
#include "simulation/EdgeTrajectory.hpp"

#include <algorithm>
#include <iterator>
#include <optional>
#include <vector>

namespace cda_rail::sim {

using BrakingPeriod = std::tuple<u_int64_t, u_int64_t>;

class TrainTrajectory {
  /**
   * Path of a single train over entire timespan
   * Repairs solution speeds to be feasible
   */
  std::reference_wrapper<const SimulationInstance> instance_r;
  std::reference_wrapper<const Train>              train_r;

  std::vector<EdgeTrajectory> edge_trajs;
  std::vector<ScheduledStop>  remaining_planned_stops;
  RoutingSolution             solution;

private:
  void backtrack_trajectory(u_int64_t timestep);

  // Modify speed targets to reach velocity before last edge traversal
  BrakingPeriod add_braking(double                   target_speed,
                            std::optional<u_int64_t> hold_until,
                            std::optional<u_int64_t> hold_at_least);

  std::optional<BrakingPeriod>
  find_latest_braking_period(double target_speed) const;

  // Returns end of braking if braking is feasible
  std::optional<u_int64_t> is_feasible_braking_point(u_int64_t start_braking,
                                                     double target_speed) const;

  double distance_to_last_traversal(u_int64_t timestep) const;

public:
  TrainTrajectory() = delete;
  TrainTrajectory(const SimulationInstance& instance, const Train& train,
                  RoutingSolution solution);

  std::optional<TrainState> get_state(u_int64_t timestep) const;
  size_t                    find_traj_idx(u_int64_t timestep) const;
  size_t                    get_remaining_stop_amount() const;
  size_t get_earliest_affected_trajectory(u_int64_t timestep) const;
  size_t get_first_timestep() const;
  size_t get_last_timestep() const;

  const Train&              get_train() const;
  const SimulationInstance& get_instance() const;

  TrainState            read_initial_train_state() const;
  void                  check_speed_limits() const;
  std::optional<double> train_vertex_distance(size_t vertex,
                                              size_t timestep) const;

  // Return route in bidirectional format and position on this route
  std::tuple<cda_rail::Route, std::vector<std::pair<double, double>>>
  convert_to_vss_format(const Network& network_bidirec) const;
};

}; // namespace cda_rail::sim
