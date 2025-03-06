#pragma once

#include "simulation/Objectives.hpp"
#include "simulation/RoutingSolutionSet.hpp"
#include "simulation/TrainTrajectorySet.hpp"

namespace cda_rail::sim {

struct ScoreSet {
  std::unordered_map<std::string, double> stop_scores;
  std::unordered_map<std::string, double> destination_scores;
  double                                  collision_score = 0;

  double get_score() const;
  double get_collision_score() const;
  double get_stop_score() const;
  double get_destination_score() const;
};

class SolverResult {
  RoutingSolutionSet solutions;
  TrainTrajectorySet trajectories;
  ScoreSet           scores;

public:
  SolverResult(const SimulationInstance& instance);
  SolverResult(const SimulationInstance& instance,
               const RoutingSolutionSet& solutions);

  void insert_or_assign(const RoutingSolution& solution,
                        const TrainTrajectory& trajectory);

  const RoutingSolutionSet& get_solutions() const;
  const TrainTrajectorySet& get_trajectories() const;
  const ScoreSet&           get_score_set() const;
};

}; // namespace cda_rail::sim
