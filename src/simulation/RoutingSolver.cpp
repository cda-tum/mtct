#include "simulation/RoutingSolver.hpp"

cda_rail::sim::RoutingSolver::RoutingSolver(SimulationInstance instance)
    : instance(instance), rng_engine(std::ranlux24_base(time(NULL))) {};

std::optional<cda_rail::sim::SolverResult>
cda_rail::sim::RoutingSolver::random_search(
    double                                    min_improv_rate,
    std::function<double(TrainTrajectorySet)> objective_fct) {
  double                      best_score = DBL_MAX;
  std::optional<SolverResult> best_result;

  std::chrono::steady_clock::time_point last_time =
      std::chrono::steady_clock::now();
  for (;;) {
    RoutingSolutionSet round_sol{instance, rng_engine};
    TrainTrajectorySet round_traj{instance, round_sol};

    double round_score = objective_fct(round_traj);

    if (round_score < best_score) {
      std::chrono::steady_clock::time_point round_time =
          std::chrono::steady_clock::now();
      auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                          round_time - last_time)
                          .count();
      double improv_rate = (best_score - round_score) / interval;

      last_time  = round_time;
      best_score = round_score;
      best_result.reset();
      best_result.emplace(SolverResult{
          .solution     = round_sol,
          .trajectories = round_traj,
      });

      if (improv_rate < min_improv_rate)
        break;
    }
  }

  return best_result;
}
