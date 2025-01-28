#pragma once

#include "simulation/RoutingSolutionSet.hpp"
#include "simulation/SolverResult.hpp"
#include "simulation/TrainTrajectorySet.hpp"

#include <chrono>
#include <functional>
#include <plog/Log.h>
#include <random>

namespace cda_rail::sim {

using ScoreHistory = std::vector<std::tuple<std::chrono::milliseconds, double>>;

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

  std::tuple<std::optional<SolverResult>, ScoreHistory>
  random_search(size_t timeout);

  std::tuple<std::optional<SolverResult>, ScoreHistory>
  greedy_search(size_t global_timeout, size_t per_train_timeout);

  std::optional<SolverResult> greedy_solution(size_t per_train_timeout);
};

}; // namespace cda_rail::sim
