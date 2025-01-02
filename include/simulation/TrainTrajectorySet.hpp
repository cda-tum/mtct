#pragma once

#include "simulation/RoutingSolutionSet.hpp"
#include "simulation/TrainTrajectory.hpp"

namespace cda_rail {

class TrainTrajectorySet {
  /**
   * Trajectories for all trains in a timetable
   */

  std::unordered_map<std::string, TrainTrajectory> trajectories;

public:
  TrainTrajectorySet() = delete;
  TrainTrajectorySet(const SimulationInstance& instance,
                     RoutingSolutionSet        solution_set);
};

}; // namespace cda_rail
