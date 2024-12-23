#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Timetable.hpp"
#include "simulation/RoutingSolution.hpp"

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

  for (int i = 0; i <= 1000; i++) {
    const Train& train =
        timetable.get_train_list().get_train(random_train_index(rng_engine));
    const RoutingSolution sol = RoutingSolution(10, 10, 100, train, rng_engine);
    for (auto target : sol.v_targets.targets) {
      ASSERT_GE(target.first, 0);
      ASSERT_LE(target.first, 100);
      ASSERT_GE(target.second, -train.max_speed);
      ASSERT_LE(target.second, train.max_speed);
    }
    ASSERT_EQ(sol.v_targets.targets.size(), 10);
    ASSERT_EQ(sol.switch_directions.size(), 10);
  }
}

TEST(Simulation, SpeedTargets) {
  std::vector<ulong>  timesteps = {3, 20, 50, 75, 87};
  std::vector<double> speeds    = {0.4, 0.6, 0.5, -0.2, -0.5};

  SpeedTargets v_targets(timesteps, speeds);
  ASSERT_EQ(v_targets.find_target_speed(2), 0.4);
  ASSERT_EQ(v_targets.find_target_speed(11), 0.4);
  ASSERT_EQ(v_targets.find_target_speed(21), 0.6);
  ASSERT_EQ(v_targets.find_target_speed(50), 0.5);
  ASSERT_EQ(v_targets.find_target_speed(74), 0.5);
  ASSERT_EQ(v_targets.find_target_speed(84), -0.2);
  ASSERT_EQ(v_targets.find_target_speed(89), -0.5);

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

  SpeedTargets v_targets_original = v_targets;
  v_targets.delete_range(20, 50);
  ASSERT_EQ(v_targets.find_target_speed(35), 0.4);
  ASSERT_EQ(v_targets.targets.size(), 3);

  v_targets.insert(cop);
  ASSERT_EQ(v_targets_original.targets, v_targets.targets);
}
