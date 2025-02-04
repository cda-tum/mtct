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

cda_rail::sim::RoutingSolver::RoutingSolver(const SimulationInstance& instance)
    : instance(instance), rng_engine(std::ranlux24_base(time(NULL))) {};

std::tuple<std::optional<cda_rail::sim::SolverResult>,
           cda_rail::sim::ScoreHistory>
cda_rail::sim::RoutingSolver::random_local_search(
    std::chrono::seconds max_search_time, double start_sampling_frac,
    double stop_sampling_frac) {
  double                      best_score = DBL_MAX;
  std::optional<SolverResult> best_result;
  ScoreHistory                hist;

  std::chrono::steady_clock::time_point initial_time =
      std::chrono::steady_clock::now();

  for (;;) {
    std::chrono::steady_clock::time_point round_time =
        std::chrono::steady_clock::now();

    auto res = local_search(RoutingSolutionSet{instance, rng_engine},
                            start_sampling_frac, stop_sampling_frac);

    double round_score = std::get<0>(res).get_score();

    if (round_score < best_score) {
      best_score = round_score;
      best_result.reset();
      best_result.emplace(std::get<0>(res));
      hist.push_back({std::chrono::duration_cast<std::chrono::milliseconds>(
                          round_time - initial_time),
                      best_score});
    }

    if (std::chrono::duration_cast<std::chrono::seconds>(
            round_time - initial_time) > max_search_time)
      break;
  }

  return {best_result, hist};
}

std::tuple<std::optional<cda_rail::sim::SolverResult>,
           cda_rail::sim::ScoreHistory>
cda_rail::sim::RoutingSolver::grasp_search(
    std::chrono::seconds      max_search_time,
    std::chrono::milliseconds per_train_stall_time, double start_sampling_frac,
    double stop_sampling_frac) {
  double                      best_score = DBL_MAX;
  std::optional<SolverResult> best_result;
  ScoreHistory                hist;

  std::chrono::steady_clock::time_point initial_time =
      std::chrono::steady_clock::now();

  for (;;) {
    std::chrono::steady_clock::time_point round_time =
        std::chrono::steady_clock::now();

    std::optional<cda_rail::sim::SolverResult> greedy_sol_opt =
        greedy_solution(per_train_stall_time);
    if (!greedy_sol_opt.has_value())
      continue;

    auto   res         = local_search(greedy_sol_opt.value().get_solutions(),
                                      start_sampling_frac, stop_sampling_frac);
    double round_score = std::get<0>(res).get_score();

    if (round_score < best_score) {
      best_score = round_score;
      best_result.reset();
      best_result.emplace(std::get<0>(res));
      hist.push_back({std::chrono::duration_cast<std::chrono::milliseconds>(
                          round_time - initial_time),
                      best_score});
    }

    if (std::chrono::duration_cast<std::chrono::seconds>(
            round_time - initial_time) > max_search_time)
      break;
  }

  return {best_result, hist};
}

std::tuple<std::optional<cda_rail::sim::SolverResult>,
           cda_rail::sim::ScoreHistory>
cda_rail::sim::RoutingSolver::random_search(
    std::optional<std::chrono::seconds> max_search_time,
    std::optional<std::chrono::seconds> max_stall_time) {
  if (!max_search_time.has_value() && !max_stall_time.has_value())
    throw std::invalid_argument(
        "Need at least one abort criterium for search.");

  double                      best_score = DBL_MAX;
  std::optional<SolverResult> best_result;
  ScoreHistory                hist;

  std::chrono::steady_clock::time_point initial_time =
      std::chrono::steady_clock::now();

  std::chrono::steady_clock::time_point last_time = initial_time;
  for (;;) {
    std::chrono::steady_clock::time_point round_time =
        std::chrono::steady_clock::now();
    auto interval = std::chrono::duration_cast<std::chrono::seconds>(
        round_time - last_time);
    auto total_time = std::chrono::duration_cast<std::chrono::seconds>(
        round_time - initial_time);

    SolverResult round_result{instance,
                              RoutingSolutionSet{instance, rng_engine}};

    double round_score = round_result.get_score();

    if (round_score < best_score) {
      last_time  = round_time;
      best_score = round_score;
      best_result.reset();
      best_result.emplace(round_result);
      hist.push_back({std::chrono::duration_cast<std::chrono::milliseconds>(
                          round_time - initial_time),
                      best_score});
    }

    if (max_stall_time.has_value()) {
      if (interval > max_stall_time.value())
        break;
    }

    if (max_search_time.has_value()) {
      if (total_time > max_search_time.value())
        break;
    }
  }

  return {best_result, hist};
}

std::tuple<std::optional<cda_rail::sim::SolverResult>,
           cda_rail::sim::ScoreHistory>
cda_rail::sim::RoutingSolver::greedy_search(
    std::optional<std::chrono::seconds> max_search_time,
    std::optional<std::chrono::seconds> max_stall_time,
    std::chrono::milliseconds           per_train_stall_time) {
  if (!max_search_time.has_value() && !max_stall_time.has_value())
    throw std::invalid_argument(
        "Need at least one abort criterium for search.");

  double                      best_score = DBL_MAX;
  std::optional<SolverResult> best_result;
  ScoreHistory                hist;

  std::chrono::steady_clock::time_point initial_time =
      std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point last_time = initial_time;

  for (;;) {
    std::optional<cda_rail::sim::SolverResult> round_result_opt =
        greedy_solution(per_train_stall_time);

    std::chrono::steady_clock::time_point round_time =
        std::chrono::steady_clock::now();

    auto interval = std::chrono::duration_cast<std::chrono::seconds>(
        round_time - last_time);
    auto total_time = std::chrono::duration_cast<std::chrono::seconds>(
        round_time - initial_time);

    double round_score = round_result_opt.value().get_score();

    if (round_score < best_score) {
      last_time  = round_time;
      best_score = round_score;
      best_result.reset();
      best_result.emplace(round_result_opt.value());
      auto total_time_spent =
          std::chrono::duration_cast<std::chrono::milliseconds>(round_time -
                                                                initial_time);
      hist.push_back({total_time_spent, best_score});
    }

    if (max_stall_time.has_value()) {
      if (interval > max_stall_time.value())
        break;
    }

    if (max_search_time.has_value()) {
      if (total_time > max_search_time.value())
        break;
    }

    if (!round_result_opt)
      continue;
  }

  return {best_result, hist};
}

std::tuple<cda_rail::sim::SolverResult, cda_rail::sim::ScoreHistory>
cda_rail::sim::RoutingSolver::local_search(
    cda_rail::sim::RoutingSolutionSet starting_solution,
    double                            start_sampling_range_fraction,
    double                            abort_sampling_range_fraction) {
  /**
   * Luus–Jaakola Local Search
   */
  std::chrono::steady_clock::time_point initial_time =
      std::chrono::steady_clock::now();

  ScoreHistory hist;
  double       sampling_range_fraction = start_sampling_range_fraction;
  SolverResult last_result{instance, starting_solution};
  double       last_score = last_result.get_score();
  hist.push_back({std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::steady_clock::now() - initial_time),
                  last_score});

  while (sampling_range_fraction > abort_sampling_range_fraction) {
    RoutingSolutionSet new_sol = last_result.get_solutions();
    new_sol.perturb(instance, sampling_range_fraction, rng_engine);
    SolverResult new_result{instance, new_sol};

    if (double new_score = new_result.get_score(); new_score < last_score) {
      last_result = new_result;
      last_score  = new_score;
      hist.push_back({std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::steady_clock::now() - initial_time),
                      last_score});
    } else {
      sampling_range_fraction = sampling_range_fraction * 0.95;
    }
  }

  return {last_result, hist};
}

std::optional<cda_rail::sim::SolverResult>
cda_rail::sim::RoutingSolver::greedy_solution(
    std::chrono::milliseconds per_train_stall_time) {
  std::chrono::steady_clock::time_point last_time =
      std::chrono::steady_clock::now();

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
      RoutingSolution round_sol{instance, train, rng_engine};
      TrainTrajectory round_traj{instance, train, round_sol};
      result.insert_or_assign(round_sol, round_traj);
      double round_score = result.get_score();

      std::chrono::steady_clock::time_point round_time =
          std::chrono::steady_clock::now();
      auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
          round_time - last_time);

      if (round_score < best_score) {
        last_time  = round_time;
        best_score = round_score;
        best_sol   = round_sol;
        best_traj  = round_traj;
      }

      if (interval > per_train_stall_time) {
        if (!result.get_trajectories().get_traj(train.name))
          return {};

        result.insert_or_assign(best_sol, best_traj);
        break;
      }
    }
  }

  return result;
};
