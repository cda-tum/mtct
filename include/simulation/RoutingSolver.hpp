#pragma once

#include "simulation/TrainTrajectorySet.hpp"

#include <chrono>
#include <functional>
#include <plog/Log.h>
#include <random>

namespace cda_rail::sim {

struct SolverResult {
  RoutingSolutionSet solution;
  TrainTrajectorySet trajectories;
};

class RoutingSolver {
  /**
   * Performs heuristic routing for a SimulationInstance
   *
   * @param instance contains constant parameters
   * @param rng_engine used for solution generation
   */

  const SimulationInstance instance;
  std::ranlux24_base       rng_engine;

public:
  RoutingSolver() = delete;
  RoutingSolver(SimulationInstance instance);

  std::optional<SolverResult>
  random_search(std::function<double(TrainTrajectorySet)> objective_fct,
                size_t                                    timeout);

  std::optional<SolverResult>
  greedy_search(std::function<double(TrainTrajectorySet)> objective_fct,
                size_t                                    timeout);
};

}; // namespace cda_rail::sim
