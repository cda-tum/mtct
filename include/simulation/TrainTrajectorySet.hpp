#pragma once

#include "simulation/RoutingSolutionSet.hpp"
#include "simulation/TrainTrajectory.hpp"

namespace cda_rail {

class TrainTrajectorySet {
  /**
   * Trajectories for all trains in a timetable
   *
   * @param trajectories Map containing train name and corresponding
   * TrainTrajectory
   */

  std::unordered_map<std::string, TrainTrajectory> trajectories;

public:
  TrainTrajectorySet() = delete;
  TrainTrajectorySet(const SimulationInstance& instance,
                     const RoutingSolutionSet& solution_set);

  size_t size() const;
};

}; // namespace cda_rail
