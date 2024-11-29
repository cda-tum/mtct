#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Timetable.hpp"
#include "simulation/RoutingSolution.hpp"
#include "simulation/SimulationInstance.hpp"

using namespace cda_rail;

#include "gtest/gtest.h"
#include <plog/Log.h>
#include <plog/Logger.h>

TEST(Simulation, SimulationInstance) {
  Network network =
      Network::import_network("./example-networks/SimpleStation/network/");
  Timetable timetable = Timetable::import_timetable(
      "./example-networks/SimpleStation/timetable/", network);

  SimulationInstance instance = SimulationInstance(network, timetable, 200, 20);

  ASSERT_EQ(instance.get_max_train_speed(), 83.33);
  ASSERT_EQ(instance.get_shortest_track(), 5);
}

TEST(Simulation, RandomSolution) {
  uint64_t seed =
      std::chrono::high_resolution_clock::now().time_since_epoch().count();
  std::ranlux24_base rng_engine(seed);
  for (int i = 0; i <= 100; i++) {
    const RoutingSolution sol = RoutingSolution(10, 10, rng_engine);

    // Check size
    ASSERT_EQ(sol.switch_directions.size(), 10);
    ASSERT_EQ(sol.v_targets.size(), 10);

    // Check range
    for (double direction : sol.switch_directions) {
      ASSERT_GE(direction, 0);
      ASSERT_LE(direction, 1);
    }
    for (std::tuple<double, double> v_target : sol.v_targets) {
      ASSERT_GE(std::get<0>(v_target), 0);
      ASSERT_LE(std::get<0>(v_target), 1);
      ASSERT_GE(std::get<1>(v_target), 0);
      ASSERT_LE(std::get<1>(v_target), 1);
    }
  }
}
