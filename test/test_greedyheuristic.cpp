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
               "Assertion.*failed");

  EXPECT_DOUBLE_EQ(cda_rail::simulator::simple_braking_time_heuristic(
                       tr1, simulator, 0, {-1, -1}),
                   0);

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

TEST(GreedyHeuristic, SimpleRemainingTimeHeuristic) {
  Network    network;
  const auto v0t = network.add_vertex("v0t", VertexType::TTD);
  const auto v0b = network.add_vertex("v0b", VertexType::TTD);
  const auto v1t = network.add_vertex("v1t", VertexType::TTD);
  const auto v1b = network.add_vertex("v1b", VertexType::TTD);
  const auto v2  = network.add_vertex("v2", VertexType::TTD);
  const auto v3  = network.add_vertex("v3", VertexType::TTD);
  const auto v4t = network.add_vertex("v4t", VertexType::TTD);
  const auto v4b = network.add_vertex("v4b", VertexType::TTD);
  const auto v5  = network.add_vertex("v5", VertexType::TTD);
  const auto v6  = network.add_vertex("v6", VertexType::TTD);
  const auto v7  = network.add_vertex("v7", VertexType::TTD);
  const auto v8  = network.add_vertex("v8", VertexType::TTD);

  const auto v0t_v1t = network.add_edge(v0t, v1t, 100, 10);
  const auto v0b_v1b = network.add_edge(v0b, v1b, 50, 25);
  const auto v1t_v2  = network.add_edge(v1t, v2, 10, 10);
  const auto v1b_v2  = network.add_edge(v1b, v2, 10, 10);
  const auto v2_v3   = network.add_edge(v2, v3, 150, 10);
  const auto v3_v4t  = network.add_edge(v3, v4t, 50, 20);
  const auto v3_v4b  = network.add_edge(v3, v4b, 100, 20);
  const auto v4t_v5  = network.add_edge(v4t, v5, 1000, 50);
  const auto v4b_v5  = network.add_edge(v4b, v5, 500, 20);
  const auto v5_v6   = network.add_edge(v5, v6, 50, 20);
  const auto v6_v7   = network.add_edge(v6, v7, 150, 20);
  const auto v7_v8   = network.add_edge(v7, v8, 50, 25);

  network.add_successor(v0t_v1t, v1t_v2);
  network.add_successor(v0b_v1b, v1b_v2);
  network.add_successor(v1t_v2, v2_v3);
  network.add_successor(v1b_v2, v2_v3);
  network.add_successor(v2_v3, v3_v4t);
  network.add_successor(v2_v3, v3_v4b);
  network.add_successor(v3_v4t, v4t_v5);
  network.add_successor(v3_v4b, v4b_v5);
  network.add_successor(v4t_v5, v5_v6);
  network.add_successor(v4b_v5, v5_v6);
  network.add_successor(v5_v6, v6_v7);
  network.add_successor(v6_v7, v7_v8);

  GeneralTimetable<GeneralSchedule<GeneralScheduledStop>> timetable;
  timetable.add_station("Station1");
  timetable.add_track_to_station("Station1", v3_v4b, network);
  timetable.add_track_to_station("Station1", v3_v4t, network);
  timetable.add_station("Station2");
  timetable.add_track_to_station("Station2", v5_v6, network);
  timetable.add_track_to_station("Station2", v6_v7, network);
  timetable.add_track_to_station("Station2", v7_v8, network);

  // Train 1 (Length 100, Max Speed 50)
  // Quickest path from v0t to v8
  // Entering: 30 seconds
  // v0t -> v1t: 100 / 10 = 10 seconds
  // v1t -> v2: 10 / 10 = 1 second
  // v2 -> v3: 150 / 10 = 15 seconds
  // v3 -> v4t: 50 / 20 = 2.5 seconds
  // v4t -> v5: 1000 / 50 = 20 seconds
  // v5 -> v6: 50 / 20 = 2.5 seconds
  // v6 -> v7: 150 / 20 = 7.5 seconds
  // v7 -> v8: 50 / 25 = 2 seconds
  // Exit: 100 / 50 = 2 seconds
  // Total: 92.5
  const auto tr1 = timetable.add_train("Train1", 100, 50, 4, 2, true, {30, 60},
                                       15, v0t, {300, 600}, 20, v8, network);

  // Train 2 (Length 300, Max Speed 20)
  // Quickest path from v0b to v8
  // Entering: 60 seconds
  // v0b -> v1b: 50 / 20 = 2.5 seconds
  // v1b -> v2: 10 / 10 = 1 second
  // v2 -> v3: 150 / 10 = 15 seconds
  // v3 -> v4b: 100 / 20 = 5 seconds
  // v4b -> v5: 500 / 20 = 25 seconds
  // v5 -> v6: 50 / 20 = 2.5 seconds
  // v6 -> v7: 150 / 20 = 7.5 seconds
  // v7 -> v8: 50 / 20 = 2.5 seconds
  // Exit: 300 / 20 = 15 seconds
  // Total: 136
  const auto tr2 = timetable.add_train("Train2", 300, 20, 4, 2, true, {60, 90},
                                       15, v0b, {340, 600}, 20, v8, network);

  // Train 3 (Length 50, Max Speed 20)
  // Quickest path from v0t to Station1
  // Entering: 90 seconds
  // v0t -> v1t: 100 / 10 = 10 seconds
  // v1t -> v2: 10 / 10 = 1 second
  // v2 -> v3: 150 / 10 = 15 seconds
  // v3 -> v4t: 50 / 20 = 2.5 seconds
  // Total: 118.5
  // Stopping for 60 seconds until 178.5
  // If earliest exit is considered, until 120 + 60 = 180 seconds
  const auto tr3 = timetable.add_train("Train3", 50, 20, 4, 2, true, {90, 120},
                                       15, v0t, {200, 600}, 20, v8, network);
  timetable.add_stop(tr3, "Station1", {120, 200}, {140, 260}, 60);
  // Quickest path from Station1 to Station2
  // v4b -> v5: 500 / 20 = 25 seconds
  // v5 -> v6: 50 / 20 = 2.5 seconds
  // Total: 27.5 seconds
  // Hence, at time 178.5 + 27.5 = 206 seconds
  // Stopping for 30 seconds until 236
  timetable.add_stop(tr3, "Station2", {200, 300}, {234, 360}, 30);
  // Quickest path from Station2 to v8
  // Exit: 50 / 20 = 2.5 seconds
  // Total: 2.5 seconds
  // Hence, at time 236 + 2.5 = 238.5 seconds

  // Train 4 (Length 100, Max Speed 50)
  const auto tr4 = timetable.add_train("Train4", 100, 50, 4, 2, true, {0, 60},
                                       15, v0b, {100, 600}, 20, v8, network);
  // Entering: 0 seconds
  // v0b -> v1b: 50 / 25 = 2 seconds
  // v1b -> v2: 10 / 10 = 1 second
  // v2 -> v3: 150 / 10 = 15 seconds
  // v3 -> v4b: 100 / 20 = 5 seconds
  // Arriving at Station 1 at 23 seconds
  // Stopping for 30 seconds until 53 seconds
  timetable.add_stop(tr4, "Station1", {20, 100}, {40, 120}, 30);
  // v4b -> v5: 500 / 20 = 25 seconds
  // v5 -> v6: 50 / 20 = 2.5 seconds
  // v6 -> v7: 150 / 20 = 7.5 seconds
  // Arriving at Station 2 at 88 seconds
  // Stopping for 45 seconds until 133 seconds
  timetable.add_stop(tr4, "Station2", {80, 200}, {120, 240}, 45);
  // Exit: 100 / 50 = 2 seconds
  // Total: 135 seconds

  // Train 5 (Length 120, Max Speed 20)
  const auto tr5 = timetable.add_train("Train5", 120, 20, 4, 2, true, {0, 60},
                                       15, v0t, {100, 600}, 20, v8, network);
  timetable.add_stop(tr5, "Station1", {20, 100}, {40, 120}, 30);

  RouteMap                                                    routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);
  cda_rail::simulator::GreedySimulator simulator(instance, {});

  const auto [feas_tr1_a, obj_tr1_a] =
      simulator::simple_remaining_time_heuristic(tr1, simulator, -1, -1, false,
                                                 false, false);
  EXPECT_TRUE(feas_tr1_a);
  EXPECT_EQ(obj_tr1_a, 92.5);
  const auto [feas_tr1_b, obj_tr1_b] =
      simulator::simple_remaining_time_heuristic(tr1, simulator, -1, -1, false,
                                                 false, true);
  EXPECT_TRUE(feas_tr1_b);
  EXPECT_EQ(obj_tr1_b, 300);
  simulator.set_train_edges_of_tr(tr1, {v0t_v1t, v1t_v2, v2_v3});
  // Now the train is at v3
  // v3 -> v4t: 50 / 20 = 2.5 seconds
  // v4t -> v5: 1000 / 50 = 20 seconds
  // v5 -> v6: 50 / 20 = 2.5 seconds
  // v6 -> v7: 150 / 20 = 7.5 seconds
  // v7 -> v8: 50 / 25 = 2 seconds
  // Exit: 100 / 50 = 2 seconds
  // Total: 36.5 seconds
  const auto [feas_tr1_c, obj_tr1_c] =
      simulator::simple_remaining_time_heuristic(tr1, simulator, 90, -20, false,
                                                 false, false);
  EXPECT_TRUE(feas_tr1_c);
  EXPECT_EQ(obj_tr1_c, 36.5);
  const auto [feas_tr1_d, obj_tr1_d] =
      simulator::simple_remaining_time_heuristic(tr1, simulator, 90, -20, false,
                                                 false, true);
  EXPECT_TRUE(feas_tr1_d);
  EXPECT_EQ(obj_tr1_d, 300 - 90 + 20);
  simulator.set_train_edges_of_tr(tr1, {v0t_v1t, v1t_v2, v2_v3, v3_v4b});
  // Now the train is at v4b
  // v4b -> v5: 500 / 20 = 25 seconds
  // v5 -> v6: 50 / 20 = 2.5 seconds
  // v6 -> v7: 150 / 20 = 7.5 seconds
  // v7 -> v8: 50 / 25 = 2 seconds
  // Exit: 100 / 50 = 2 seconds
  // Total: 39 seconds
  const auto [feas_tr1_e, obj_tr1_e] =
      simulator::simple_remaining_time_heuristic(tr1, simulator, 70, -2.4,
                                                 false, false, false);
  EXPECT_TRUE(feas_tr1_e);
  EXPECT_EQ(obj_tr1_e, 39);
  const auto [feas_tr1_f, obj_tr1_f] =
      simulator::simple_remaining_time_heuristic(tr1, simulator, 70, -2.4,
                                                 false, false, true);
  EXPECT_TRUE(feas_tr1_f);
  EXPECT_EQ(obj_tr1_f, 300 - 70 + 2.4);
  simulator.set_train_edges_of_tr(
      tr1, {v0t_v1t, v1t_v2, v2_v3, v3_v4b, v4b_v5, v5_v6, v6_v7, v7_v8});
  // Now the train is at v8
  // Exit: 100 / 50 = 2 seconds
  const auto [feas_tr1_g, obj_tr1_g] =
      simulator::simple_remaining_time_heuristic(tr1, simulator, 100, -5, false,
                                                 false, false);
  EXPECT_TRUE(feas_tr1_g);
  EXPECT_EQ(obj_tr1_g, 2);
  const auto [feas_tr1_h, obj_tr1_h] =
      simulator::simple_remaining_time_heuristic(tr1, simulator, 100, -5, false,
                                                 false, true);
  EXPECT_TRUE(feas_tr1_h);
  EXPECT_EQ(obj_tr1_h, 300 - 100 + 5);

  // Train 2
  const auto [feas_tr2_a, obj_tr2_a] =
      simulator::simple_remaining_time_heuristic(tr2, simulator, -1, -1, false,
                                                 false, false);
  EXPECT_TRUE(feas_tr2_a);
  EXPECT_EQ(obj_tr2_a, 136);
  const auto [feas_tr2_b, obj_tr2_b] =
      simulator::simple_remaining_time_heuristic(tr2, simulator, -1, -1, false,
                                                 false, true);
  EXPECT_TRUE(feas_tr2_b);
  EXPECT_EQ(obj_tr2_b, 340);
  simulator.set_train_edges_of_tr(tr2, {v0b_v1b, v1b_v2, v2_v3});
  // Now the train is at v3
  // v3 -> v4b: 100 / 20 = 5 seconds
  // v4b -> v5: 500 / 20 = 25 seconds
  // v5 -> v6: 50 / 20 = 2.5 seconds
  // v6 -> v7: 150 / 20 = 7.5 seconds
  // v7 -> v8: 50 / 20 = 2.5 seconds
  // Exit: 300 / 20 = 15 seconds
  // Total: 57.5 seconds
  const auto [feas_tr2_c, obj_tr2_c] =
      simulator::simple_remaining_time_heuristic(tr2, simulator, 90, -5, false,
                                                 false, false);
  EXPECT_TRUE(feas_tr2_c);
  EXPECT_EQ(obj_tr2_c, 57.5);
  const auto [feas_tr2_d, obj_tr2_d] =
      simulator::simple_remaining_time_heuristic(tr2, simulator, 90, -5, false,
                                                 false, true);
  EXPECT_TRUE(feas_tr2_d);
  EXPECT_EQ(obj_tr2_d, 340 - 90 + 5);
  // If tr_exit - 5 + 57.5 > 600 the train cannot exit the network in time
  // tr_exit > 547.5
  const auto [feas_tr2_e, obj_tr2_e] =
      simulator::simple_remaining_time_heuristic(tr2, simulator, 548, -5, false,
                                                 false, false);
  EXPECT_FALSE(feas_tr2_e);
  EXPECT_EQ(obj_tr2_e, 57.5);
  const auto [feas_tr2_f, obj_tr2_f] =
      simulator::simple_remaining_time_heuristic(tr2, simulator, 548, -5, false,
                                                 true, false);
  EXPECT_TRUE(feas_tr2_f);
  EXPECT_EQ(obj_tr2_f, 57.5);

  // Train 3
  const auto [feas_tr3_a, obj_tr3_a] =
      simulator::simple_remaining_time_heuristic(tr3, simulator, -1, -1, false,
                                                 false, false);
  EXPECT_TRUE(feas_tr3_a);
  EXPECT_EQ(obj_tr3_a, 238.5);
  const auto [feas_tr3_b, obj_tr3_b] =
      simulator::simple_remaining_time_heuristic(tr3, simulator, -1, -1, false,
                                                 false, true);
  EXPECT_TRUE(feas_tr3_b);
  EXPECT_EQ(obj_tr3_b, 240);
  simulator.set_train_edges_of_tr(tr3,
                                  {v0t_v1t, v1t_v2, v2_v3, v3_v4t, v4t_v5});
  simulator.append_stop_edge_to_tr(tr3, v3_v4t);
  // Now the train is at v5
  // v5 -> v6: 50 / 20 = 2.5 seconds
  // Stopping at Station2 for 30 seconds
  // Exit: 50 / 20 = 2.5 seconds
  const auto [feas_tr3_c, obj_tr3_c] =
      simulator::simple_remaining_time_heuristic(tr3, simulator, 200, -2.5,
                                                 false, false, false);
  EXPECT_TRUE(feas_tr3_c);
  EXPECT_EQ(obj_tr3_c, 35.0);
  const auto [feas_tr3_d, obj_tr3_d] =
      simulator::simple_remaining_time_heuristic(tr3, simulator, 200, -2.5,
                                                 false, false, true);
  EXPECT_TRUE(feas_tr3_d);
  EXPECT_EQ(obj_tr3_d, 39.0);
  const auto [feas_tr3_e, obj_tr3_e] =
      simulator::simple_remaining_time_heuristic(tr3, simulator, 300, -2.4,
                                                 false, false, false);
  EXPECT_FALSE(feas_tr3_e);
  EXPECT_EQ(obj_tr3_e, 35.0);
  const auto [feas_tr3_f, obj_tr3_f] =
      simulator::simple_remaining_time_heuristic(tr3, simulator, 300, -2.4,
                                                 true, false, false);
  EXPECT_TRUE(feas_tr3_f);
  EXPECT_EQ(obj_tr3_f, 35.0);

  // Train 4
  const auto [feas_tr4_a, obj_tr4_a] =
      simulator::simple_remaining_time_heuristic(tr4, simulator, -1, -1, false,
                                                 false, false);
  EXPECT_TRUE(feas_tr4_a);
  EXPECT_EQ(obj_tr4_a, 135);

  // Train 5, too long for station
  const auto [feas_tr5_a, obj_tr5_a] =
      simulator::simple_remaining_time_heuristic(tr5, simulator, -1, -1, false,
                                                 false, false);
  EXPECT_FALSE(feas_tr5_a);
  EXPECT_EQ(obj_tr5_a, cda_rail::INF);
}

// NOLINTEND
// (clang-analyzer-deadcode.DeadStores,misc-const-correctness,clang-diagnostic-unused-result)
