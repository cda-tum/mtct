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
  SimulationParameters parameters = {.n_timesteps = 200, .n_v_target_vars = 20};

  const SimulationInstance instance =
      SimulationInstance(network, timetable, parameters);
}
