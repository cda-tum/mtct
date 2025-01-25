#include "simulation/Objectives.hpp"
#include "simulation/RoutingSolver.hpp"
#include "simulation/TrainTrajectorySet.hpp"

using namespace cda_rail;

#include "gtest/gtest.h"
#include <plog/Log.h>
#include <plog/Logger.h>
#include <vector>

TEST(Simulation, RandomSolution) {
  const Network network =
      Network::import_network("./example-networks/SimpleStation/network/");
  const Timetable timetable = Timetable::import_timetable(
      "./example-networks/SimpleStation/timetable/", network);
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
      ASSERT_GE(target.second,
                -(instance.bidirectional_travel * train.max_speed));
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
  Network network =
      Network::import_network("./example-networks/SimpleStation/network/");
  Timetable timetable = Timetable::import_timetable(
      "./example-networks/SimpleStation/timetable/", network);

  sim::SimulationInstance instance(network, timetable, true);

  ASSERT_EQ(instance.get_max_train_speed(), 83.33);
  ASSERT_EQ(instance.get_shortest_track(), 5);
}

TEST(Simulation, EdgeTrajectory) {
  const ulong seed =
      std::chrono::high_resolution_clock::now().time_since_epoch().count();
  std::ranlux24_base rng_engine(seed);
  Network            network =
      Network::import_network("./example-networks/SimpleNetwork/network/");
  Timetable timetable = Timetable::import_timetable(
      "./example-networks/SimpleNetwork/timetable/", network);

  sim::SimulationInstance              instance(network, timetable, true);
  std::uniform_int_distribution<ulong> random_train_index(
      0, timetable.get_train_list().size() - 1);

  for (int i = 0; i < 500; i++) {
    Train train =
        timetable.get_train_list().get_train(random_train_index(rng_engine));

    sim::SimulationInstance instance(network, timetable, true);
    sim::RoutingSolution    solution(instance, train, rng_engine);

    Schedule train_schedule = instance.timetable.get_schedule(train.name);
    sim::TrainState init_state{
        .timestep = (ulong)train_schedule.get_t_0(),
        .edge =
            instance.network.get_successors(train_schedule.get_entry()).front(),
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
  Network            network =
      Network::import_network("./example-networks/SimpleNetwork/network/");
  Timetable timetable = Timetable::import_timetable(
      "./example-networks/SimpleNetwork/timetable/", network);

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
  Network            network =
      Network::import_network("./example-networks/SimpleNetwork/network/");
  Timetable timetable = Timetable::import_timetable(
      "./example-networks/SimpleNetwork/timetable/", network);

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
  }
}

TEST(Simulation, TrainDistance) {
  // Generate a stationary solution set
  Network network =
      Network::import_network("./example-networks/SimpleNetwork/network/");
  Timetable timetable = Timetable::import_timetable(
      "./example-networks/SimpleNetwork/timetable/", network);

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

  // We give a solution with speeds 0. Due to initial speed trains still run
  // tr1lr comes to a halt at time 204 position 0.00753769 on edge v4-v5
  // tr1rl comes to a halt at time 204 position 0.00753769 on edge v11-v10
  // tr2lr comes to a halt at time 16 position 0.412632 on edge v1c-v2c
  // tr2rl comes to a halt at time 16 position 0.412632 on edge v14a-v13a

  ASSERT_NEAR(traj.train_distance("tr1lr", "tr1rl", 204).value(),
              1950.0 + 2.0 * (50.0 + (1.0 - 0.00753769) * 9950.0), 1e-3);
  ASSERT_NEAR(traj.train_distance("tr1rl", "tr1lr", 204).value(),
              1950.0 + 2.0 * (50.0 + (1.0 - 0.00753769) * 9950.0), 1e-3);

  ASSERT_NEAR(traj.train_distance("tr1lr", "tr2lr", 204).value(),
              (1.0 - 0.412632) * 475.0 + 50.0 + 0.00753769 * 9950.0, 1e-3);
  ASSERT_NEAR(traj.train_distance("tr2lr", "tr1lr", 204).value(),
              (1.0 - 0.412632) * 475.0 + 50.0 + 0.00753769 * 9950.0, 1e-3);

  ASSERT_NEAR(traj.train_distance("tr1rl", "tr2rl", 204).value(),
              (1.0 - 0.412632) * 475.0 + 50.0 + 0.00753769 * 9950.0, 1e-3);
  ASSERT_NEAR(traj.train_distance("tr2rl", "tr1rl", 204).value(),
              (1.0 - 0.412632) * 475.0 + 50.0 + 0.00753769 * 9950.0, 1e-3);

  ASSERT_NEAR(traj.train_distance("tr2lr", "tr2rl", 204).value(),
              1950.0 + 2.0 * (50.0 + 9950.0 + 50.0 + (1.0 - 0.412632) * 475.0),
              1e-3);
  ASSERT_NEAR(traj.train_distance("tr2rl", "tr2lr", 204).value(),
              1950.0 + 2.0 * (50.0 + 9950.0 + 50.0 + (1.0 - 0.412632) * 475.0),
              1e-3);
}

TEST(Simulation, Penalties) {
  const ulong seed =
      std::chrono::high_resolution_clock::now().time_since_epoch().count();
  std::ranlux24_base rng_engine(seed);
  Network            network =
      Network::import_network("./example-networks/SimpleNetwork/network/");
  Timetable timetable = Timetable::import_timetable(
      "./example-networks/SimpleNetwork/timetable/", network);

  sim::SimulationInstance instance(network, timetable, true);

  for (int i = 0; i < 100; i++) {
    sim::RoutingSolutionSet solution_set{instance, rng_engine};
    sim::TrainTrajectorySet traj{instance, solution_set};
    sim::collision_penalty(traj);
    sim::destination_penalty(traj);
    sim::stop_penalty(traj);
  }
}

TEST(Simulation, RandomSearch) {
  Network network =
      Network::import_network("./example-networks/SimpleNetwork/network/");
  Timetable timetable = Timetable::import_timetable(
      "./example-networks/SimpleNetwork/timetable/", network);

  sim::SimulationInstance instance{network, timetable, false};
  sim::RoutingSolver      solver{instance};
  if (std::optional<sim::SolverResult> sol = solver.random_search(5)) {
    cda_rail::is_directory_and_create("tmp");
    std::filesystem::path p = "tmp/test_traj_random.csv";
    sol.value().get_trajectories().export_csv(p);
    // std::filesystem::remove_all("tmp");
  }
}

TEST(Simulation, GreedySolution) {
  Network network =
      Network::import_network("./example-networks/SimpleNetwork/network/");
  Timetable timetable = Timetable::import_timetable(
      "./example-networks/SimpleNetwork/timetable/", network);

  sim::SimulationInstance          instance{network, timetable, false};
  sim::RoutingSolver               solver{instance};
  std::optional<sim::SolverResult> sol = solver.greedy_solution(0.02);
}

TEST(Simulation, GreedySearch) {
  Network network =
      Network::import_network("./example-networks/SimpleNetwork/network/");
  Timetable timetable = Timetable::import_timetable(
      "./example-networks/SimpleNetwork/timetable/", network);

  sim::SimulationInstance instance{network, timetable, false};
  sim::RoutingSolver      solver{instance};
  if (std::optional<sim::SolverResult> sol = solver.greedy_search(5, 0.002)) {
    cda_rail::is_directory_and_create("tmp");
    std::filesystem::path p = "tmp/test_traj_greedy.csv";
    sol.value().get_trajectories().export_csv(p);
    // std::filesystem::remove_all("tmp");
  }
}

// TODO: test for invariance of solution after being repaired and used again
