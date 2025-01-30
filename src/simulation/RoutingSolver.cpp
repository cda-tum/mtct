#include "simulation/RoutingSolver.hpp"

void cda_rail::sim::ScoreHistory::export_csv(
    const std::filesystem::path& p) const {
  std::ofstream csvfile(p);
  csvfile << "time,score" << std::endl;

  for (auto it = begin(); it != end(); it++) {
    csvfile << std::get<0>(*it).count() << "," << std::get<1>(*it) << std::endl;
  }

  csvfile.close();
}

void cda_rail::sim::ScoreHistoryCollection::export_csv(
    const std::filesystem::path& p) const {
  std::ofstream csvfile(p);
  csvfile << "time,score" << std::endl;

  for (auto hist_it = begin(); hist_it != end(); hist_it++) {
    for (auto it = (*hist_it).begin(); it != (*hist_it).end(); it++) {
      csvfile << std::get<0>(*it).count() << "," << std::get<1>(*it)
              << std::endl;
    }
  }

  csvfile.close();
}

void cda_rail::sim::ScoreHistoryCollection::add(ScoreHistory hist) {
  push_back(hist);
}

cda_rail::sim::RoutingSolver::RoutingSolver(SimulationInstance instance)
    : instance(instance), rng_engine(std::ranlux24_base(time(NULL))) {};

std::tuple<std::optional<cda_rail::sim::SolverResult>,
           cda_rail::sim::ScoreHistory>
cda_rail::sim::RoutingSolver::random_search(size_t timeout) {
  /**
   * Random search exits when score decreases slower than abort_improv_rate
   *
   * @param objective Objective function that gets minimized
   * @param timeout Stop search after no improvement during this time
   */

  double                      best_score = DBL_MAX;
  std::optional<SolverResult> best_result;
  ScoreHistory                hist;

  std::chrono::steady_clock::time_point initial_time =
      std::chrono::steady_clock::now();

  std::chrono::steady_clock::time_point last_time = initial_time;
  for (;;) {
    std::chrono::steady_clock::time_point round_time =
        std::chrono::steady_clock::now();
    auto interval =
        std::chrono::duration_cast<std::chrono::seconds>(round_time - last_time)
            .count();

    SolverResult round_result{instance,
                              RoutingSolutionSet{instance, rng_engine}};

    double round_score = round_result.get_score();

    if (round_score < best_score) {
      last_time  = round_time;
      best_score = round_score;
      std::cerr << "Best Score: " << best_score << std::endl;
      best_result.reset();
      best_result.emplace(round_result);
      hist.push_back({std::chrono::duration_cast<std::chrono::milliseconds>(
                          round_time - initial_time),
                      best_score});
    }

    if (interval > timeout)
      break;
  }

  return {best_result, hist};
}

std::tuple<std::optional<cda_rail::sim::SolverResult>,
           cda_rail::sim::ScoreHistory>
cda_rail::sim::RoutingSolver::greedy_search(size_t global_timeout,
                                            size_t per_train_timeout) {
  /**
   * Random search exits when score decreases slower than abort_improv_rate
   *
   * @param objective Objective function that gets minimized
   * @param global_timeout Stop search after no improvement during this time in
   * seconds
   * @param per_train_timeout Stop search after no improvement during this time
   * in milliseconds
   */

  double                      best_score = DBL_MAX;
  std::optional<SolverResult> best_result;
  ScoreHistory                hist;

  std::chrono::steady_clock::time_point initial_time =
      std::chrono::steady_clock::now();

  std::chrono::steady_clock::time_point last_time = initial_time;
  for (;;) {
    std::chrono::steady_clock::time_point round_time =
        std::chrono::steady_clock::now();
    auto interval =
        std::chrono::duration_cast<std::chrono::seconds>(round_time - last_time)
            .count();

    if (interval > global_timeout)
      break;

    std::optional<cda_rail::sim::SolverResult> round_result_opt =
        greedy_solution(per_train_timeout);
    if (!round_result_opt)
      continue;

    double round_score = round_result_opt.value().get_score();

    if (round_score < best_score) {
      last_time  = round_time;
      best_score = round_score;
      std::cerr << "Best Score: " << best_score << std::endl;
      best_result.reset();
      best_result.emplace(round_result_opt.value());
      hist.push_back({std::chrono::duration_cast<std::chrono::milliseconds>(
                          round_time - initial_time),
                      best_score});
    }
  }

  return {best_result, hist};
}

std::optional<cda_rail::sim::SolverResult>
cda_rail::sim::RoutingSolver::greedy_solution(size_t per_train_timeout) {
  std::chrono::steady_clock::time_point last_time =
      std::chrono::steady_clock::now();
  /**
   * Greedily place one train after another considering objective
   *
   * @param objective Objective function that gets minimized
   * @param per_train_timeout Stop search after no improvement during this time
   * in milliseconds improvement during this time Maximum runtime is n_trains *
   * round_timeout
   */

  SolverResult result{instance};

  TrainList        train_list = instance.timetable.get_train_list();
  double           score      = 0;
  std::vector<int> train_idxs(train_list.size());
  std::iota(std::begin(train_idxs), std::end(train_idxs), 0);
  std::shuffle(train_idxs.begin(), train_idxs.end(), rng_engine);

  for (auto train_idx = train_idxs.begin(); train_idx != train_idxs.end();
       train_idx++) {
    const auto train = train_list.get_train(*train_idx);
    std::chrono::steady_clock::time_point last_time =
        std::chrono::steady_clock::now();

    double          best_score = DBL_MAX;
    RoutingSolution best_sol{instance};
    TrainTrajectory best_traj{instance, train, best_sol};

    for (;;) {
      std::chrono::steady_clock::time_point round_time =
          std::chrono::steady_clock::now();
      auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                          round_time - last_time)
                          .count();

      RoutingSolution round_sol{instance, train, rng_engine};
      TrainTrajectory round_traj{instance, train, round_sol};
      result.insert_or_assign(round_sol, round_traj);

      double round_score = result.get_score();

      if (round_score < best_score) {
        last_time  = round_time;
        best_score = round_score;
        best_sol   = round_sol;
        best_traj  = round_traj;
      }

      if (interval > per_train_timeout) {
        if (!result.get_trajectories().get_traj(train.name))
          return {};

        result.insert_or_assign(best_sol, best_traj);
        break;
      }
    }
  }

  return result;
};
