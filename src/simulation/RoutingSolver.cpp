#include "simulation/RoutingSolver.hpp"

void cda_rail::sim::ScoreHistory::export_csv(
    const std::filesystem::path& p) const {
  std::ofstream csvfile(p);
  csvfile << "time,overall_score,collision_score,destination_score,stop_score"
          << std::endl;

  for (auto it = begin(); it != end(); it++) {
    const ScoreSet& set = std::get<1>(*it);
    csvfile << std::get<0>(*it).count() << "," << set.get_score() << ","
            << set.get_collision_score() << "," << set.get_destination_score()
            << "," << set.get_stop_score() << "," << std::endl;
  }

  csvfile.close();
}

void cda_rail::sim::ScoreHistoryCollection::export_csv(
    const std::filesystem::path& p) const {
  std::ofstream csvfile(p);
  csvfile << "time,overall_score,collision_score,destination_score,stop_score"
          << std::endl;

  for (auto hist_it = begin(); hist_it != end(); hist_it++) {
    for (auto it = (*hist_it).begin(); it != (*hist_it).end(); it++) {
      const ScoreSet& set = std::get<1>(*it);
      csvfile << std::get<0>(*it).count() << "," << set.get_score() << ","
              << set.get_collision_score() << "," << set.get_destination_score()
              << "," << set.get_stop_score() << "," << std::endl;
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
    std::chrono::seconds max_search_time, LocalParams params) {
  double                      best_score = DBL_MAX;
  std::optional<SolverResult> best_result;
  ScoreHistory                hist;

  std::chrono::steady_clock::time_point initial_time =
      std::chrono::steady_clock::now();

  for (;;) {
    auto res = local_search(RoutingSolutionSet{instance, rng_engine}, params);
    ScoreSet round_score_set = std::get<0>(res).get_score_set();
    double   round_score     = round_score_set.get_score();

    std::chrono::steady_clock::time_point round_time =
        std::chrono::steady_clock::now();

    if (round_score < best_score) {
      best_score = round_score;
      best_result.reset();
      best_result.emplace(std::get<0>(res));
      hist.push_back({std::chrono::duration_cast<std::chrono::milliseconds>(
                          round_time - initial_time),
                      round_score_set});
    }

    if (std::chrono::duration_cast<std::chrono::seconds>(
            round_time - initial_time) > max_search_time)
      break;
  }

  return {best_result, hist};
}

std::tuple<std::optional<cda_rail::sim::SolverResult>,
           cda_rail::sim::ScoreHistory>
cda_rail::sim::RoutingSolver::grasp_search(std::chrono::seconds max_search_time,
                                           GreedyParams         gre_params,
                                           LocalParams          loc_params) {
  double                      best_score = DBL_MAX;
  std::optional<SolverResult> best_result;
  ScoreHistory                hist;

  std::chrono::steady_clock::time_point initial_time =
      std::chrono::steady_clock::now();

  for (;;) {
    std::optional<cda_rail::sim::SolverResult> greedy_sol_opt =
        greedy_solution(gre_params);
    if (!greedy_sol_opt.has_value())
      continue;

    auto res =
        local_search(RoutingSolutionSet{instance, rng_engine}, loc_params);
    ScoreSet round_score_set = std::get<0>(res).get_score_set();
    double   round_score     = round_score_set.get_score();

    std::chrono::steady_clock::time_point round_time =
        std::chrono::steady_clock::now();

    if (round_score < best_score) {
      best_score = round_score;
      best_result.reset();
      best_result.emplace(std::get<0>(res));
      hist.push_back({std::chrono::duration_cast<std::chrono::milliseconds>(
                          round_time - initial_time),
                      round_score_set});
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
    SolverResult round_result{instance,
                              RoutingSolutionSet{instance, rng_engine}};
    ScoreSet     round_score_set = round_result.get_score_set();
    double       round_score     = round_score_set.get_score();

    std::chrono::steady_clock::time_point round_time =
        std::chrono::steady_clock::now();
    auto interval = std::chrono::duration_cast<std::chrono::seconds>(
        round_time - last_time);
    auto total_time = std::chrono::duration_cast<std::chrono::seconds>(
        round_time - initial_time);

    if (round_score < best_score) {
      last_time  = round_time;
      best_score = round_score;
      best_result.reset();
      best_result.emplace(round_result);
      hist.push_back({std::chrono::duration_cast<std::chrono::milliseconds>(
                          round_time - initial_time),
                      round_score_set});
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
    std::optional<std::chrono::seconds> max_stall_time, GreedyParams params) {
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
        greedy_solution(params);

    if (!round_result_opt.has_value())
      continue;

    ScoreSet round_score_set = round_result_opt.value().get_score_set();
    double   round_score     = round_score_set.get_score();

    std::chrono::steady_clock::time_point round_time =
        std::chrono::steady_clock::now();
    auto interval = std::chrono::duration_cast<std::chrono::seconds>(
        round_time - last_time);
    auto total_time = std::chrono::duration_cast<std::chrono::seconds>(
        round_time - initial_time);

    if (round_score < best_score) {
      last_time  = round_time;
      best_score = round_score;
      best_result.reset();
      best_result.emplace(round_result_opt.value());
      auto total_time_spent =
          std::chrono::duration_cast<std::chrono::milliseconds>(round_time -
                                                                initial_time);
      hist.push_back({total_time_spent, round_score_set});
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
    cda_rail::sim::RoutingSolutionSet starting_solution, LocalParams params) {
  /**
   * Luus–Jaakola Local Search
   */
  std::chrono::steady_clock::time_point initial_time =
      std::chrono::steady_clock::now();

  ScoreHistory hist;
  double       sampling_range_fraction = params.start_sampling_range_fraction;
  SolverResult last_result{instance, starting_solution};
  ScoreSet     last_score_set = last_result.get_score_set();
  double       last_score     = last_score_set.get_score();
  hist.push_back({std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::steady_clock::now() - initial_time),
                  last_score_set});

  while (sampling_range_fraction > params.abort_sampling_range_fraction) {
    RoutingSolutionSet new_sol = last_result.get_solutions();
    new_sol.perturb(instance, sampling_range_fraction, rng_engine);
    SolverResult new_result{instance, new_sol};
    ScoreSet     new_score_set = new_result.get_score_set();

    if (double new_score = new_score_set.get_score(); new_score < last_score) {
      last_result = new_result;
      last_score  = new_score;
      hist.push_back({std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::steady_clock::now() - initial_time),
                      new_score_set});
    } else {
      sampling_range_fraction =
          sampling_range_fraction * params.contraction_coeff;
    }
  }

  return {last_result, hist};
}

std::tuple<cda_rail::sim::SolverResult, cda_rail::sim::ScoreHistory>
cda_rail::sim::RoutingSolver::local_search(
    RoutingSolutionSet starting_solution, LocalParams params,
    const std::function<double(void)>& rng01) {
  std::chrono::steady_clock::time_point initial_time =
      std::chrono::steady_clock::now();

  ScoreHistory hist;
  double       sampling_range_fraction = params.start_sampling_range_fraction;
  SolverResult last_result{instance, starting_solution};
  ScoreSet     last_score_set = last_result.get_score_set();
  double       last_score     = last_score_set.get_score();
  hist.push_back({std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::steady_clock::now() - initial_time),
                  last_score_set});

  while (sampling_range_fraction > params.abort_sampling_range_fraction) {
    RoutingSolutionSet new_sol = last_result.get_solutions();
    new_sol.perturb(instance, sampling_range_fraction, rng01);
    SolverResult new_result{instance, new_sol};
    ScoreSet     new_score_set = new_result.get_score_set();

    if (double new_score = new_score_set.get_score(); new_score < last_score) {
      last_result = new_result;
      last_score  = new_score;
      hist.push_back({std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::steady_clock::now() - initial_time),
                      new_score_set});
    } else {
      sampling_range_fraction =
          sampling_range_fraction * params.contraction_coeff;
    }
  }

  return {last_result, hist};
}

std::optional<cda_rail::sim::SolverResult>
cda_rail::sim::RoutingSolver::greedy_solution(GreedyParams params) {
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
      ScoreSet round_score_set = result.get_score_set();
      double   round_score     = round_score_set.get_score();

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

      if (interval > params.per_train_stall_time) {
        if (!result.get_trajectories().get_traj(train.name))
          return {};

        result.insert_or_assign(best_sol, best_traj);
        break;
      }
    }
  }

  return result;
};

std::tuple<std::optional<cda_rail::sim::SolverResult>,
           cda_rail::sim::ScoreHistory>
cda_rail::sim::RoutingSolver::genetic_search(GeneticParams params,
                                             bool          local_improv) {
  /**
   * Genetic algorithm for entire solution sets
   */
  ScoreHistory hist;

  GA_Type ga_obj;
  ga_obj.problem_mode               = EA::GA_MODE::SOGA;
  ga_obj.multi_threading            = params.is_multithread;
  ga_obj.idle_delay_us              = 0; // switch between threads quickly
  ga_obj.verbose                    = false;
  ga_obj.population                 = params.population;
  ga_obj.generation_max             = params.gen_max;
  ga_obj.calculate_SO_total_fitness = std::bind(
      &RoutingSolver::calculate_SO_total_fitness, this, std::placeholders::_1);
  ga_obj.init_genes = std::bind(&RoutingSolver::init_genes, this,
                                std::placeholders::_1, std::placeholders::_2);
  ga_obj.eval_solution =
      std::bind(&RoutingSolver::eval_solution, this, std::placeholders::_1,
                std::placeholders::_2);
  ga_obj.mutate = std::bind(&RoutingSolver::mutate, this, std::placeholders::_1,
                            std::placeholders::_2, std::placeholders::_3);

  if (local_improv) {
    ga_obj.crossover = std::bind(&RoutingSolver::crossover_local_improv, this,
                                 std::placeholders::_1, std::placeholders::_2,
                                 std::placeholders::_3);
  } else {
    ga_obj.crossover =
        std::bind(&RoutingSolver::crossover, this, std::placeholders::_1,
                  std::placeholders::_2, std::placeholders::_3);
  }

  std::chrono::steady_clock::time_point starting_time =
      std::chrono::steady_clock::now();
  ga_obj.SO_report_generation = std::bind(
      &RoutingSolver::SO_report_generation, this, starting_time, std::ref(hist),
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

  ga_obj.best_stall_max     = params.stall_max;
  ga_obj.elite_count        = params.n_elite;
  ga_obj.crossover_fraction = params.xover_frac;
  ga_obj.mutation_rate      = params.mut_rate;
  ga_obj.solve();

  const RoutingSolutionSet& best_solution =
      ga_obj.last_generation.chromosomes
          .at(ga_obj.last_generation.best_chromosome_index)
          .genes;

  return {SolverResult{instance, best_solution}, hist};
}

void cda_rail::sim::RoutingSolver::init_genes(
    RoutingSolutionSet& p, const std::function<double(void)>& rnd01) {
  p = RoutingSolutionSet{instance, rnd01};
}

bool cda_rail::sim::RoutingSolver::eval_solution(const RoutingSolutionSet& p,
                                                 MiddleCost&               c) {
  SolverResult round_result{instance, p};
  c.score = round_result.get_score_set().get_score();
  return true;
}

cda_rail::sim::RoutingSolutionSet
cda_rail::sim::RoutingSolver::mutate(const RoutingSolutionSet&          X_base,
                                     const std::function<double(void)>& rnd01,
                                     double shrink_scale) {
  RoutingSolutionSet X_new(X_base);
  X_new.perturb(instance, shrink_scale, rnd01);
  return X_new;
}

cda_rail::sim::RoutingSolutionSet cda_rail::sim::RoutingSolver::crossover(
    const RoutingSolutionSet& X1, const RoutingSolutionSet& X2,
    const std::function<double(void)>& rnd01) {
  RoutingSolutionSet X_new;

  // Randomly select individual train solutions
  for (auto sol_it = X1.solutions.begin(); sol_it != X1.solutions.end();
       sol_it++) {
    if (rnd01() > 0.5) {
      X_new.solutions.insert_or_assign((*sol_it).first, (*sol_it).second);
    } else {
      X_new.solutions.insert_or_assign((*sol_it).first,
                                       X2.solutions.at((*sol_it).first));
    }
  }

  return X_new;
}

cda_rail::sim::RoutingSolutionSet
cda_rail::sim::RoutingSolver::crossover_local_improv(
    const RoutingSolutionSet& X1, const RoutingSolutionSet& X2,
    const std::function<double(void)>& rnd01) {
  auto X_new = crossover(X1, X2, rnd01);
  // TODO: pass localsearch parameters without modifying openGA.hpp
  auto res = local_search(X_new, {0.05, 0.01, 0.9}, rnd01);

  return std::get<0>(res).get_solutions();
}

double cda_rail::sim::RoutingSolver::calculate_SO_total_fitness(
    const GA_Type::thisChromosomeType& X) {
  return X.middle_costs.score;
}

void cda_rail::sim::RoutingSolver::SO_report_generation(
    const std::chrono::steady_clock::time_point starting_time,
    ScoreHistory& hist, int generation_number,
    const EA::GenerationType<RoutingSolutionSet, MiddleCost>& last_generation,
    const RoutingSolutionSet&                                 best_genes) {
  std::cout << "Generation [" << generation_number << "], "
            << "Best=" << last_generation.best_total_cost << ", "
            << "Average=" << last_generation.average_cost << ", "
            << "Exe_time=" << last_generation.exe_time << std::endl;

  std::chrono::steady_clock::time_point round_time =
      std::chrono::steady_clock::now();
  std::chrono::milliseconds past_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(round_time -
                                                            starting_time);
  auto best_individual =
      last_generation.chromosomes[last_generation.best_chromosome_index];
  SolverResult best_result{instance, best_individual.genes};
  hist.push_back({past_time, best_result.get_score_set()});
}
