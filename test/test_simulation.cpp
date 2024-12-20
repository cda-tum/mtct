#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Timetable.hpp"
#include "simulation/EdgeTrajectory.hpp"

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

  SimulationInstance instance(network, timetable, 200, 20);

  ASSERT_EQ(instance.get_max_train_speed(), 83.33);
  ASSERT_EQ(instance.get_shortest_track(), 5);
}

TEST(Simulation, EdgeTrajectory) {
  Network network =
      Network::import_network("./example-networks/SimpleStation/network/");
  Timetable timetable = Timetable::import_timetable(
      "./example-networks/SimpleStation/timetable/", network);
  SimulationInstance instance(network, timetable, 200, 20);

  std::vector<ulong>  timesteps = {3, 20, 50, 75, 87};
  std::vector<double> speeds    = {0.4, 0.6, 0.5, -0.2, -0.5};
  Train               train     = timetable.get_train_list().get_train(1);

  InitialEdgeState init_state{.timestep    = 15,
                              .edge        = 3,
                              .position    = 0.05,
                              .orientation = false,
                              .speed       = 5};
  SpeedTargets     v_targets(timesteps, speeds);
  EdgeTrajectory   edge_traj(instance, train, v_targets, init_state);
  EdgeTransition   transition = edge_traj.get_transition(instance, train, 0.3);
}
