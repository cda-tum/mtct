#pragma once

#include "simulation/RoutingSolutionSet.hpp"
#include "simulation/TrainTrajectorySet.hpp"

namespace cda_rail::sim {

class SolverResult {
  RoutingSolutionSet solutions;
  TrainTrajectorySet trajectories;

public:
  SolverResult(const SimulationInstance& instance);
  SolverResult(const RoutingSolutionSet& solutions,
               const TrainTrajectorySet& trajectories);

  void                     insert_or_assign(const RoutingSolution& solution,
                                            const TrainTrajectory& trajectory);
  const RoutingSolutionSet get_solutions() const;
  const TrainTrajectorySet get_trajectories() const;
};

}; // namespace cda_rail::sim
