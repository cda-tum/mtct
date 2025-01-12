#pragma once

#include "simulation/TrainTrajectorySet.hpp"

namespace cda_rail::sim {

double combined_objective(const TrainTrajectorySet& traj_set);

double collision_penalty(const TrainTrajectorySet& traj_set);

double destination_penalty(const TrainTrajectorySet& traj_set);

double stop_penalty(const TrainTrajectorySet& traj_set);

double reciprocal_dist_penalty(double dist);

} // namespace cda_rail::sim
