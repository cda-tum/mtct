#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Timetable.hpp"
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
