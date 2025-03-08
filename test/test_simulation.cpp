#include "simulation/Objectives.hpp"
#include "simulation/RoutingSolver.hpp"
#include "simulation/TrainTrajectorySet.hpp"

using namespace cda_rail;

#include "gtest/gtest.h"
#include <plog/Log.h>
#include <plog/Logger.h>
#include <vector>

TEST(Simulation, RandomSolution) {
  const Network network = Network::import_network(
      "./example-networks-unidirec/SimpleNetwork/network/");
  const Timetable timetable = Timetable::import_timetable(
      "./example-networks-unidirec/SimpleNetwork/timetable/", network);
  const ulong seed =
      std::chrono::high_resolution_clock::now().time_since_epoch().count();
  std::ranlux24_base                   rng_engine(seed);
  std::uniform_int_distribution<ulong> random_train_index(
      0, timetable.get_train_list().size() - 1);

  for (int i = 0; i <= 100; i++) {
    const Train& train =
        timetable.get_train_list().get_train(random_train_index(rng_engine));

    sim::SimulationInstance    instance(network, timetable, true);
    const sim::RoutingSolution sol{instance, train, rng_engine};
    for (auto target : sol.v_targets.targets) {
      ASSERT_GE(target.first, 0);
      ASSERT_LT(target.first, instance.n_timesteps);
      ASSERT_GE(target.second, -(instance.allow_reversing * train.max_speed));
      ASSERT_LE(target.second, train.max_speed);
    }
    ASSERT_GE(sol.v_targets.size(), 1);
    ASSERT_LE(sol.v_targets.size(), instance.n_timesteps);
    ASSERT_EQ(sol.switch_directions.size(), instance.n_switch_vars);
  }
}

TEST(Simulation, SpeedTargets) {
  std::vector<ulong>  timesteps = {3, 20, 50, 75, 87};
  std::vector<double> speeds    = {0.4, 0.6, 0.5, -0.2, -0.5};

  sim::SpeedTargets v_targets(timesteps, speeds);
  ASSERT_EQ(v_targets.find_target_speed(2), 0.4);
  ASSERT_EQ(v_targets.find_target_speed(11), 0.4);
  ASSERT_EQ(v_targets.find_target_speed(21), 0.6);
  ASSERT_EQ(v_targets.find_target_speed(50), 0.5);
  ASSERT_EQ(v_targets.find_target_speed(74), 0.5);
  ASSERT_EQ(v_targets.find_target_speed(84), -0.2);
  ASSERT_EQ(v_targets.find_target_speed(89), -0.5);
  ASSERT_EQ(v_targets.find_next_reversal(0), 75);
  ASSERT_EQ(v_targets.find_next_reversal(10), 75);
  ASSERT_EQ(v_targets.find_next_reversal(21), 75);

  v_targets.limit_speed_from(0.3, 45);
  ASSERT_EQ(v_targets.targets.at(3), 0.4);
  ASSERT_EQ(v_targets.targets.at(20), 0.6);
  ASSERT_EQ(v_targets.targets.at(50), 0.3);
  ASSERT_EQ(v_targets.targets.at(75), -0.2);
  ASSERT_EQ(v_targets.targets.at(87), -0.3);

  std::map<ulong, double> cop = v_targets.copy_range(20, 50);
  ASSERT_EQ(cop.size(), 3);
  ASSERT_EQ(cop.at(20), 0.6);
  ASSERT_EQ(cop.at(45), 0.3);
  ASSERT_EQ(cop.at(50), 0.3);

  sim::SpeedTargets v_targets_original = v_targets;
  v_targets.delete_range(20, 50);
  ASSERT_EQ(v_targets.find_target_speed(35), 0.4);
  ASSERT_EQ(v_targets.size(), 3);

  v_targets.insert(cop);
  ASSERT_EQ(v_targets_original.targets, v_targets.targets);
}

TEST(Simulation, SimulationInstance) {
  Network network = Network::import_network(
      "./example-networks-unidirec/SimpleNetwork/network/");
  Timetable timetable = Timetable::import_timetable(
      "./example-networks-unidirec/SimpleNetwork/timetable/", network);

  sim::SimulationInstance instance(network, timetable, true);

  ASSERT_EQ(instance.get_max_train_speed(), 50);
  ASSERT_EQ(instance.get_shortest_track(), 25);
}

TEST(Simulation, EdgeTrajectory) {
  const ulong seed =
      std::chrono::high_resolution_clock::now().time_since_epoch().count();
  std::ranlux24_base rng_engine(seed);
  Network            network = Network::import_network(
      "./example-networks-unidirec/SimpleNetwork/network/");
  Timetable timetable = Timetable::import_timetable(
      "./example-networks-unidirec/SimpleNetwork/timetable/", network);

  sim::SimulationInstance              instance(network, timetable, true);
  std::uniform_int_distribution<ulong> random_train_index(
      0, timetable.get_train_list().size() - 1);

  for (int i = 0; i < 500; i++) {
    Train train =
        timetable.get_train_list().get_train(random_train_index(rng_engine));

    sim::SimulationInstance instance(network, timetable, true);
    sim::RoutingSolution    solution(instance, train, rng_engine);

    Schedule train_schedule = instance.timetable.get_schedule(train.name);

    size_t entry    = train_schedule.get_entry();
    auto   outedges = instance.network.get_successors(entry);
    if (outedges.empty())
      outedges = instance.network.get_predecessors(entry);
    if (outedges.empty())
      throw std::logic_error("Train entry vertex is has no connected edges.");

    sim::TrainState init_state{.timestep    = (ulong)train_schedule.get_t_0(),
                               .edge        = outedges.front(),
                               .position    = 0,
                               .orientation = true,
                               .speed       = train_schedule.get_v_0()};

    sim::EdgeTrajectory edge_traj(instance, train, solution.v_targets,
                                  init_state);
    sim::EdgeEntry      transition = edge_traj.enter_next_edge(0.3);

    edge_traj.check_speed_limits();
  }
}

TEST(Simulation, TrainTrajectory) {
  const ulong seed =
      std::chrono::high_resolution_clock::now().time_since_epoch().count();
  std::ranlux24_base rng_engine(seed);
  Network            network = Network::import_network(
      "./example-networks-unidirec/SimpleNetwork/network/");
  Timetable timetable = Timetable::import_timetable(
      "./example-networks-unidirec/SimpleNetwork/timetable/", network);

  sim::SimulationInstance               instance(network, timetable, true);
  std::uniform_int_distribution<size_t> random_train_index(
      0, timetable.get_train_list().size() - 1);

  for (int i = 0; i < 100; i++) {
    size_t train_idx = random_train_index(rng_engine);

    Train train = timetable.get_train_list().get_train(train_idx);
    sim::SimulationInstance instance(network, timetable, true);
    sim::RoutingSolution    solution(instance, train, rng_engine);

    sim::TrainTrajectory traj(instance, train, solution);

    traj.check_speed_limits();
  }
}

TEST(Simulation, TrainTrajectorySet) {
  const ulong seed =
      std::chrono::high_resolution_clock::now().time_since_epoch().count();
  std::ranlux24_base rng_engine(seed);
  Network            network = Network::import_network(
      "./example-networks-unidirec/SimpleNetwork/network/");
  Timetable timetable = Timetable::import_timetable(
      "./example-networks-unidirec/SimpleNetwork/timetable/", network);

  sim::SimulationInstance               instance(network, timetable, true);
  std::uniform_int_distribution<size_t> random_train_index(
      0, timetable.get_train_list().size() - 1);
  std::uniform_int_distribution<size_t> random_target_amount(1, 100);

  for (int i = 0; i < 100; i++) {
    sim::RoutingSolutionSet solution_set{instance, rng_engine};
    sim::TrainTrajectorySet traj{instance, solution_set};

    ASSERT_EQ(solution_set.solutions.size(), 4);
    ASSERT_EQ(traj.size(), 4);
    traj.check_speed_limits();

    const auto old_solution_set = solution_set;
    solution_set.perturb(instance, 0.01, rng_engine);
    for (auto old_solution : old_solution_set.solutions) {
      ASSERT_FALSE(
          old_solution.second.v_targets.targets ==
          solution_set.solutions.at(old_solution.first).v_targets.targets);
      ASSERT_FALSE(
          old_solution.second.switch_directions ==
          solution_set.solutions.at(old_solution.first).switch_directions);
    }
  }
}

TEST(Simulation, TrainDistance) {
  // Generate a stationary solution set
  Network network = Network::import_network(
      "./example-networks-unidirec/SimpleNetwork/network/");
  Timetable timetable = Timetable::import_timetable(
      "./example-networks-unidirec/SimpleNetwork/timetable/", network);

  sim::SimulationInstance instance(network, timetable, true);
  sim::RoutingSolutionSet solution_set{instance};
  sim::TrainTrajectorySet traj{instance, solution_set};
  const TrainList&        list = instance.timetable.get_train_list();

  // Starting positions
  ASSERT_EQ(traj.train_distance("tr2lr", "tr2lr", 0), 0);
  ASSERT_EQ(traj.train_distance("tr2rl", "tr2rl", 0), 0);

  ASSERT_EQ(traj.train_distance("tr2lr", "tr2rl", 0), 23000);
  ASSERT_EQ(traj.train_distance("tr2rl", "tr2lr", 0), 23000);
  ASSERT_EQ(traj.train_distance("tr1lr", "tr1rl", 180), 23000);
  ASSERT_EQ(traj.train_distance("tr1rl", "tr1lr", 180), 23000);
  ASSERT_FALSE(traj.train_distance("tr1rl", "tr2lr", 0).has_value());
  ASSERT_FALSE(traj.train_distance("tr2rl", "tr1lr", 0).has_value());
}

TEST(Simulation, Penalties) {
  const ulong seed =
      std::chrono::high_resolution_clock::now().time_since_epoch().count();
  std::ranlux24_base rng_engine(seed);
  Network            network = Network::import_network(
      "./example-networks-unidirec/SimpleNetwork/network/");
  Timetable timetable = Timetable::import_timetable(
      "./example-networks-unidirec/SimpleNetwork/timetable/", network);

  sim::SimulationInstance instance(network, timetable, true);

  for (int i = 0; i < 100; i++) {
    sim::RoutingSolutionSet solution_set{instance, rng_engine};
    sim::TrainTrajectorySet traj{instance, solution_set};
    sim::collision_penalty(traj);
    sim::destination_penalty(traj);
    sim::stop_penalty(traj);
  }
}

TEST(Simulation, SolverResult) {
  Network network = Network::import_network(
      "./example-networks-unidirec/SimpleNetwork/network/");
  Timetable timetable = Timetable::import_timetable(
      "./example-networks-unidirec/SimpleNetwork/timetable/", network);
  const ulong seed =
      std::chrono::high_resolution_clock::now().time_since_epoch().count();
  std::ranlux24_base rng_engine(seed);

  sim::SimulationInstance instance{network, timetable, false};

  TrainList train_list = instance.timetable.get_train_list();

  for (size_t i = 0; i < 100; i++) {
    sim::SolverResult result{instance};

    std::vector<int> train_idxs(train_list.size());
    std::iota(std::begin(train_idxs), std::end(train_idxs), 0);
    std::shuffle(train_idxs.begin(), train_idxs.end(), rng_engine);

    for (auto train_idx = train_idxs.begin(); train_idx != train_idxs.end();
         train_idx++) {
      const auto           train          = train_list.get_train(*train_idx);
      double               previous_score = result.get_score_set().get_score();
      sim::RoutingSolution round_sol{instance, train, rng_engine};
      sim::TrainTrajectory round_traj{instance, train, round_sol};
      result.insert_or_assign(round_sol, round_traj);
      ASSERT_GE(result.get_score_set().get_score(), previous_score);
    }
  }
}

TEST(Simulation, GreedySolution) {
  Network network = Network::import_network(
      "./example-networks-unidirec/SimpleNetwork/network/");
  Timetable timetable = Timetable::import_timetable(
      "./example-networks-unidirec/SimpleNetwork/timetable/", network);

  sim::SimulationInstance          instance{network, timetable, false};
  sim::RoutingSolver               solver{instance};
  std::optional<sim::SolverResult> sol =
      solver.greedy_solution({std::chrono::milliseconds{10}});
}

TEST(Simulation, RandomSearch) {
  Network network = Network::import_network(
      "./example-networks-unidirec/SimpleNetwork/network/");
  Timetable timetable = Timetable::import_timetable(
      "./example-networks-unidirec/SimpleNetwork/timetable/", network);

  sim::SimulationInstance instance{network, timetable, false};
  sim::RoutingSolver      solver{instance};

  auto res =
      solver.random_search(std::chrono::seconds{1}, std::chrono::seconds{1});

  if (std::get<0>(res)) {
    cda_rail::is_directory_and_create("tmp");
    std::get<0>(res).value().get_trajectories().export_csv(
        "tmp/test_traj_random.csv");
    std::get<1>(res).export_csv("tmp/test_hist_random.csv");
  }
}

TEST(Simulation, GreedySearch) {
  Network network = Network::import_network(
      "./example-networks-unidirec/SimpleNetwork/network/");
  Timetable timetable = Timetable::import_timetable(
      "./example-networks-unidirec/SimpleNetwork/timetable/", network);

  sim::SimulationInstance instance{network, timetable, false};
  sim::RoutingSolver      solver{instance};

  auto res = solver.greedy_search({}, std::chrono::seconds{1},
                                  {std::chrono::milliseconds{50}});

  if (std::get<0>(res)) {
    cda_rail::is_directory_and_create("tmp");
    std::get<0>(res).value().get_trajectories().export_csv(
        "tmp/test_traj_greedy.csv");
    std::get<1>(res).export_csv("tmp/test_hist_greedy.csv");
  }
}

TEST(Simulation, LocalSearch) {
  const ulong seed =
      std::chrono::high_resolution_clock::now().time_since_epoch().count();
  std::ranlux24_base rng_engine(seed);
  Network            network = Network::import_network(
      "./example-networks-unidirec/SimpleNetwork/network/");
  Timetable timetable = Timetable::import_timetable(
      "./example-networks-unidirec/SimpleNetwork/timetable/", network);

  sim::SimulationInstance instance{network, timetable, false};
  sim::RoutingSolver      solver{instance};

  sim::RoutingSolutionSet solution_set{instance, rng_engine};
  auto res = solver.local_search(solution_set, {0.1, 0.01, 0.95});
}

TEST(Simulation, RandomLocalSearch) {
  const ulong seed =
      std::chrono::high_resolution_clock::now().time_since_epoch().count();
  std::ranlux24_base rng_engine(seed);
  Network            network = Network::import_network(
      "./example-networks-unidirec/SimpleNetwork/network/");
  Timetable timetable = Timetable::import_timetable(
      "./example-networks-unidirec/SimpleNetwork/timetable/", network);

  sim::SimulationInstance instance{network, timetable, false};
  sim::RoutingSolver      solver{instance};

  sim::RoutingSolutionSet solution_set{instance, rng_engine};
  auto                    res =
      solver.random_local_search(std::chrono::seconds{1}, {0.1, 1e-3, 0.95});
}

TEST(Simulation, GraspSearch) {
  const ulong seed =
      std::chrono::high_resolution_clock::now().time_since_epoch().count();
  std::ranlux24_base rng_engine(seed);
  Network            network = Network::import_network(
      "./example-networks-unidirec/SimpleNetwork/network/");
  Timetable timetable = Timetable::import_timetable(
      "./example-networks-unidirec/SimpleNetwork/timetable/", network);

  sim::SimulationInstance instance{network, timetable, false};
  sim::RoutingSolver      solver{instance};

  sim::RoutingSolutionSet solution_set{instance, rng_engine};
  auto                    res =
      solver.grasp_search(std::chrono::seconds{1},
                          {std::chrono::milliseconds{50}}, {0.1, 1e-3, 0.95});
}

TEST(Simulation, GeneticSearch) {
  const ulong seed =
      std::chrono::high_resolution_clock::now().time_since_epoch().count();
  std::ranlux24_base rng_engine(seed);
  Network            network = Network::import_network(
      "./example-networks-unidirec/SimpleNetwork/network/");
  Timetable timetable = Timetable::import_timetable(
      "./example-networks-unidirec/SimpleNetwork/timetable/", network);

  cda_rail::sim::GeneticParams ga_params{
      .is_multithread = true,
      .population     = 10,
      .gen_max        = 3,
      .stall_max      = 2,
      .n_elite        = 2,
      .xover_frac     = 0.7,
      .mut_rate       = 0.1,
  };

  sim::SimulationInstance instance{network, timetable, false};
  sim::RoutingSolver      solver{instance};

  sim::RoutingSolutionSet solution_set{instance, rng_engine};
  auto                    res  = solver.genetic_search(ga_params);
  auto                    res2 = solver.genetic_search(ga_params, true);
}

TEST(Simulation, ExportVSSSolution) {
  const ulong seed =
      std::chrono::high_resolution_clock::now().time_since_epoch().count();
  std::ranlux24_base rng_engine(seed);
  Network            network = Network::import_network(
      "./example-networks-unidirec/SimpleNetwork/network/");
  Network network_bidirec =
      Network::import_network("./example-networks/SimpleNetwork/network/");
  Timetable timetable = Timetable::import_timetable(
      "./example-networks-unidirec/SimpleNetwork/timetable/", network);
  sim::SimulationInstance instance{network, timetable, false};

  for (size_t i = 0; i < 100; i++) {
    sim::RoutingSolutionSet sol{instance, rng_engine};
    sim::TrainTrajectorySet traj{instance, sol};
    auto converted_sol = traj.to_vss_solution(network_bidirec);

    // TODO: needs one directional tracks to contain only valid routes
  }
}

// TODO: test for invariance of solution after being repaired and used again
