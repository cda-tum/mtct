#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Timetable.hpp"
#include "simulation/RoutingSolution.hpp"
#include "simulation/SimulationInstance.hpp"
#include "simulation/SpeedTargets.hpp"

using namespace cda_rail;

#include "gtest/gtest.h"
#include <plog/Log.h>
#include <plog/Logger.h>
#include <vector>

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
  uint seed =
      std::chrono::high_resolution_clock::now().time_since_epoch().count();
  std::ranlux24_base rng_engine(seed);
  for (int i = 0; i <= 100; i++) {
    const RoutingSolution sol = RoutingSolution(10, 10, rng_engine);
    ASSERT_TRUE(sol.check_consistency());
    ASSERT_EQ(sol.v_targets.size(), 10);
    ASSERT_EQ(sol.switch_directions.size(), 10);
  }
}

TEST(Simulation, SpeedTargets) {
  std::vector<uint>   timesteps = {1, 3, 5, 10, 87};
  std::vector<double> speeds    = {0.4, 0.6, 0.5, -0.2, -0.5};
  SpeedTargets        v_targets(timesteps, speeds);
}
