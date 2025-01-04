#pragma once

#include "simulation/TrainTrajectorySet.hpp"

namespace cda_rail::sim {

double collision_penalty(const TrainTrajectorySet&     traj_set,
                         std::function<double(double)> dist_penalty_fct,
                         double                        dist_safe_thres);

double reciprocal_dist_penalty(double dist, double thres);

} // namespace cda_rail::sim
