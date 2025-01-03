#pragma once

#include "simulation/RoutingSolutionSet.hpp"
#include "simulation/TrainTrajectory.hpp"

#include <fstream>
#include <iostream>

namespace cda_rail::sim {

class TrainTrajectorySet {
  /**
   * Trajectories for all trains in a timetable
   *
   * @param trajectories Map containing train name and corresponding
   * TrainTrajectory
   */
  const SimulationInstance& instance;

  std::unordered_map<std::string, TrainTrajectory> trajectories;

public:
  TrainTrajectorySet() = delete;
  TrainTrajectorySet(const SimulationInstance& instance,
                     const RoutingSolutionSet& solution_set);

  void export_csv(const std::filesystem::path& p) const;

  void check_speed_limits() const;

  size_t size() const;
};

}; // namespace cda_rail::sim
