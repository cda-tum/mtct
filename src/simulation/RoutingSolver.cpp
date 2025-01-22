#include "simulation/RoutingSolver.hpp"

cda_rail::sim::RoutingSolver::RoutingSolver(SimulationInstance instance)
    : instance(instance), rng_engine(std::ranlux24_base(time(NULL))) {};

std::optional<cda_rail::sim::SolverResult>
cda_rail::sim::RoutingSolver::random_search(
    std::function<double(TrainTrajectorySet)> objective_fct, size_t timeout) {
  /**
   * Random search exits when score decreases slower than abort_improv_rate
   *
   * @param objective Objective function that gets minimized
   * @param timeout Stop search after no improvement during this time
   */

  double                      best_score = DBL_MAX;
  std::optional<SolverResult> best_result;

  std::chrono::steady_clock::time_point last_time =
      std::chrono::steady_clock::now();
  for (;;) {
    RoutingSolutionSet                    round_sol{instance, rng_engine};
    TrainTrajectorySet                    round_traj{instance, round_sol};
    std::chrono::steady_clock::time_point round_time =
        std::chrono::steady_clock::now();
    auto interval =
        std::chrono::duration_cast<std::chrono::seconds>(round_time - last_time)
            .count();

    double round_score = objective_fct(round_traj);

    if (round_score < best_score) {
      last_time  = round_time;
      best_score = round_score;
      std::cerr << "Best Score: " << best_score << std::endl;

      best_result.reset();
      best_result.emplace(SolverResult{
          .solution     = round_sol,
          .trajectories = round_traj,
      });
    }

    if (interval > timeout)
      break;
  }

  return best_result;
}

std::optional<cda_rail::sim::SolverResult>
cda_rail::sim::RoutingSolver::greedy_search(
    std::function<double(TrainTrajectorySet)> objective_fct,
    size_t                                    round_timeout) {
  std::chrono::steady_clock::time_point last_time =
      std::chrono::steady_clock::now();
  /**
   * Greedily place one train after another considering objective
   *
   * @param objective Objective function that gets minimized
   * @param round_timeout Stop work on single train trajectory after no
   * improvement during this time Maximum runtime is n_trains * round_timeout
   */

  TrainTrajectorySet working_traj_set{instance};
  RoutingSolutionSet working_sol_set{};

  const TrainList& train_list = instance.timetable.get_train_list();
  double           score      = 0;

  for (auto train = train_list.begin(); train != train_list.end(); train++) {
    std::cerr << "Train " << (*train).name << std::endl;
    double                                best_score = DBL_MAX;
    std::chrono::steady_clock::time_point last_time =
        std::chrono::steady_clock::now();

    for (;;) {
      std::chrono::steady_clock::time_point round_time =
          std::chrono::steady_clock::now();
      auto interval = std::chrono::duration_cast<std::chrono::seconds>(
                          round_time - last_time)
                          .count();

      RoutingSolution new_sol{instance, (*train), rng_engine};
      TrainTrajectory new_traj{instance, (*train), new_sol};

      TrainTrajectorySet test_traj_set{working_traj_set};
      test_traj_set.insert_or_assign((*train).name, new_traj);

      double round_score = objective_fct(test_traj_set);

      if (round_score < best_score) {
        std::cerr << "Best Score: " << best_score << std::endl;
        last_time  = round_time;
        best_score = round_score;
        working_traj_set.insert_or_assign((*train).name, new_traj);
        working_sol_set.solutions.insert_or_assign((*train).name, new_sol);
      }

      if (interval > round_timeout) {
        if (!working_traj_set.get_traj((*train).name))
          return {};

        break;
      }
    }
  }

  return SolverResult{
      .solution     = working_sol_set,
      .trajectories = working_traj_set,
  };
}
