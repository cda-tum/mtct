#pragma once

#include "../extern/openGA/src/openGA.hpp"
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
  random_local_search(std::chrono::seconds max_search_time,
                      double start_sampl_frac, double stop_sampl_frac,
                      double contr_coeff);

  std::tuple<std::optional<SolverResult>, ScoreHistory>
  grasp_search(std::chrono::seconds      max_search_time,
               std::chrono::milliseconds per_train_stall_time,
               double start_sampl_frac, double stop_sampl_frac,
               double contr_coeff);

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
               double abort_sampling_range_fraction, double contr_coeff);

  std::optional<SolverResult>
  greedy_solution(std::chrono::milliseconds per_train_stall_time);

  std::tuple<std::optional<SolverResult>, ScoreHistory> genetic_search();

  // GA Helpers
  struct MiddleCost {
    double score;
  };

  typedef EA::Genetic<RoutingSolutionSet, MiddleCost>        GA_Type;
  typedef EA::GenerationType<RoutingSolutionSet, MiddleCost> Generation_Type;

  void init_genes(RoutingSolutionSet&                p,
                  const std::function<double(void)>& rnd01);

  bool eval_solution(const RoutingSolutionSet& p, MiddleCost& c);

  RoutingSolutionSet mutate(const RoutingSolutionSet&          X_base,
                            const std::function<double(void)>& rnd01,
                            double                             shrink_scale);

  RoutingSolutionSet crossover(const RoutingSolutionSet&          X1,
                               const RoutingSolutionSet&          X2,
                               const std::function<double(void)>& rnd01);

  double calculate_SO_total_fitness(const GA_Type::thisChromosomeType& X);

  void SO_report_generation(
      const std::chrono::steady_clock::time_point starting_time,
      ScoreHistory& hist, int generation_number,
      const EA::GenerationType<RoutingSolutionSet, MiddleCost>& last_generation,
      const RoutingSolutionSet&                                 best_genes);
};

}; // namespace cda_rail::sim
