#pragma once

#include "simulation/Objectives.hpp"
#include "simulation/RoutingSolutionSet.hpp"
#include "simulation/TrainTrajectorySet.hpp"

namespace cda_rail::sim {

class SolverResult {
  RoutingSolutionSet solutions;
  TrainTrajectorySet trajectories;

  std::unordered_map<std::string, double> stop_scores;
  std::unordered_map<std::string, double> destination_scores;
  double                                  collision_score;

public:
  SolverResult(const SimulationInstance& instance);
  SolverResult(const SimulationInstance& instance,
               const RoutingSolutionSet& solutions);

  void insert_or_assign(const RoutingSolution& solution,
                        const TrainTrajectory& trajectory);

  const RoutingSolutionSet& get_solutions() const;
  const TrainTrajectorySet& get_trajectories() const;
  double                    get_score() const;
};

}; // namespace cda_rail::sim
