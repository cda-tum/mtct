#include "Definitions.hpp"
#include "datastructure/GeneralTimetable.hpp"
#include "datastructure/Route.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "simulator/GreedyHeuristic.hpp"
#include "simulator/GreedySimulator.hpp"

#include "gtest/gtest.h"

using namespace cda_rail;

// NOLINTBEGIN
// (clang-analyzer-deadcode.DeadStores,misc-const-correctness,clang-diagnostic-unused-result)

TEST(GreedyHeuristic, SimpleBrakingTimeHeuristic) {
  Network    network;
  const auto v0 = network.add_vertex("v0", VertexType::TTD);
  const auto v1 = network.add_vertex("v1", VertexType::TTD);
  const auto v2 = network.add_vertex("v2", VertexType::TTD);
  const auto v3 = network.add_vertex("v3", VertexType::TTD);
  const auto v4 = network.add_vertex("v4", VertexType::TTD);

  const auto v2_v3 = network.add_edge(v2, v3, 100, 10);
  const auto v0_v1 = network.add_edge(v0, v1, 70, 20);
  const auto v3_v4 = network.add_edge(v3, v4, 250, 40);
  const auto v1_v2 = network.add_edge(v1, v2, 50, 25);

  network.add_successor(v0_v1, v1_v2);
  network.add_successor(v1_v2, v2_v3);
  network.add_successor(v2_v3, v3_v4);

  GeneralTimetable<GeneralSchedule<GeneralScheduledStop>> timetable;
  const auto tr1 = timetable.add_train("Train1", 100, 50, 4, 2, true, {0, 60},
                                       15, v0, {300, 600}, 40, v4, network);
  RouteMap   routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);
  cda_rail::simulator::GreedySimulator simulator(instance, {});

  EXPECT_DEATH(cda_rail::simulator::simple_braking_time_heuristic(
                   tr1, simulator, 100, {50, 60}),
               "^Assertion failed");

  simulator.set_train_edges_of_tr(tr1, {v0_v1, v1_v2, v2_v3});

  // 50 meters before exit
  // 50 / 10 = 5 seconds time
  // Instead it took 12 seconds
  // Result should be -7 seconds
  EXPECT_DOUBLE_EQ(cda_rail::simulator::simple_braking_time_heuristic(
                       tr1, simulator, 68, {68 - 12, 50}),
                   -7.0);

  // 100 meters before exit
  // 100 / 10 = 10 seconds time
  // Instead it took 25 seconds
  // Result should be -15 seconds
  EXPECT_DOUBLE_EQ(cda_rail::simulator::simple_braking_time_heuristic(
                       tr1, simulator, 85, {85 - 25, 100}),
                   -15.0);

  // 125 meters before exit
  // 25 / 25 = 1 second time
  // 100 / 10 = 10 seconds time
  // Total 11 seconds time
  // Instead it took 30 seconds
  // Result should be -19 seconds
  EXPECT_DOUBLE_EQ(cda_rail::simulator::simple_braking_time_heuristic(
                       tr1, simulator, 90, {90 - 30, 125}),
                   -19.0);

  // 150 meters before exit
  // 50 / 25 = 2 seconds time
  // 100 / 10 = 10 seconds time
  // Total 12 seconds time
  // Instead it took 40 seconds
  // Result should be -28 seconds
  EXPECT_DOUBLE_EQ(cda_rail::simulator::simple_braking_time_heuristic(
                       tr1, simulator, 100, {100 - 40, 150}),
                   -28.0);

  // 160 meters before exit
  // 10 / 20 = 0.5 seconds time
  // 50 / 25 = 2 seconds time
  // 100 / 10 = 10 seconds time
  // Total 12.5 seconds time
  // Instead it took 50 seconds
  // Result should be -37.5 seconds
  EXPECT_DOUBLE_EQ(cda_rail::simulator::simple_braking_time_heuristic(
                       tr1, simulator, 110, {110 - 50, 160}),
                   -37.5);

  // 220 meters before exit
  // 70 / 20 = 3.5 seconds time
  // 50 / 25 = 2 seconds time
  // 100 / 10 = 10 seconds time
  // Total 15.5 seconds time
  // Instead it took 70 seconds
  // Result should be -54.5 seconds
  EXPECT_DOUBLE_EQ(cda_rail::simulator::simple_braking_time_heuristic(
                       tr1, simulator, 130, {130 - 70, 220}),
                   -54.5);
}

// NOLINTEND
// (clang-analyzer-deadcode.DeadStores,misc-const-correctness,clang-diagnostic-unused-result)
