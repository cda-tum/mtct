#pragma once

#include "simulation/TrainTrajectorySet.hpp"

namespace cda_rail::sim {

double collision_penalty(const TrainTrajectorySet& traj_set);

}
