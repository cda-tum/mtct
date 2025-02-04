#pragma once

#include "simulation/RoutingSolutionSet.hpp"
#include "simulation/SolverResult.hpp"
#include "simulation/TrainTrajectorySet.hpp"

#include <chrono>
#include <functional>
#include <plog/Log.h>
#include <random>

namespace cda_rail::sim {

struct ScoreHistory
    : public std::vector<std::tuple<std::chrono::milliseconds, double>> {
  void export_csv(const std::filesystem::path& p) const;
};

class ScoreHistoryCollection : std::vector<ScoreHistory> {
public:
  void export_csv(const std::filesystem::path& p) const;
  void add(ScoreHistory hist);
};

class RoutingSolver {
  /**
   * Performs heuristic routing for a SimulationInstance
   *
   * @param instance contains constant parameters
   * @param rng_engine used for solution generation
   */

  const SimulationInstance& instance;
  std::ranlux24_base        rng_engine;

public:
  RoutingSolver() = delete;
  RoutingSolver(const SimulationInstance& instance);

  std::tuple<std::optional<SolverResult>, ScoreHistory>
  random_search(std::optional<std::chrono::seconds> max_search_time,
                std::optional<std::chrono::seconds> max_stall_time);

  std::tuple<std::optional<SolverResult>, ScoreHistory>
  greedy_search(std::optional<std::chrono::seconds> max_search_time,
                std::optional<std::chrono::seconds> max_stall_time,
                std::chrono::milliseconds           per_train_stall_time);

  std::tuple<SolverResult, ScoreHistory>
  local_search(RoutingSolutionSet starting_solution,
               double             start_sampling_range_fraction,
               double             abort_sampling_range_fraction);

  std::optional<SolverResult>
  greedy_solution(std::chrono::milliseconds per_train_stall_time);
};

}; // namespace cda_rail::sim
