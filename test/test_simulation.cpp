#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Timetable.hpp"
#include "simulation/TrainTrajectory.hpp"

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

  SimulationInstance instance(network, timetable, 20);

  ASSERT_EQ(instance.get_max_train_speed(), 83.33);
  ASSERT_EQ(instance.get_shortest_track(), 5);
}

TEST(Simulation, EdgeTrajectory) {
  const ulong seed =
      std::chrono::high_resolution_clock::now().time_since_epoch().count();
  std::ranlux24_base rng_engine(seed);
  Network            network =
      Network::import_network("./example-networks/SimpleStation/network/");
  Timetable timetable = Timetable::import_timetable(
      "./example-networks/SimpleStation/timetable/", network);

  SimulationInstance                   instance(network, timetable, 20);
  std::uniform_int_distribution<ulong> random_train_index(
      0, timetable.get_train_list().size() - 1);

  for (int i = 0; i < 1000; i++) {
    Train train =
        timetable.get_train_list().get_train(random_train_index(rng_engine));
    RoutingSolution solution(10, 10, instance.n_timesteps, train, rng_engine);

    cda_rail::Schedule train_schedule =
        instance.timetable.get_schedule(train.name);
    InitialEdgeState init_state{
        .timestep = (ulong)train_schedule.get_t_0(),
        .edge =
            instance.network.get_successors(train_schedule.get_entry()).front(),
        .position    = 0,
        .orientation = true,
        .speed       = train_schedule.get_v_0()};

    EdgeTrajectory edge_traj(instance, train, solution.v_targets, init_state);
    EdgeTransition transition = edge_traj.get_transition(0.3);
  }
}
