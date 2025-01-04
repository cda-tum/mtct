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

  /**
   * Random search exits when score decreases slower than abort_improv_rate
   *
   * @param abort_improv_rate Minimum improvement in score / milliseconds
   * @param objective Objective function that gets minimized
   */
  std::optional<SolverResult>
  random_search(double                                    abort_improv_rate,
                std::function<double(TrainTrajectorySet)> objective_fct);
};

}; // namespace cda_rail::sim
