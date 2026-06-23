#include "Definitions.hpp"
#include "datastructure/Route.hpp"
#include "datastructure/Timetable.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "simulator/GreedyHeuristic.hpp"
#include "simulator/GreedySimulator.hpp"

#include "gtest/gtest.h"
#include <cstdlib>

using namespace cda_rail;

#define EXPECT_APPROX_EQ_6(a, b) EXPECT_APPROX_EQ(a, b, 1e-6)

#define EXPECT_APPROX_EQ(a, b, c)                                              \
  EXPECT_TRUE(std::abs((a) - (b)) < (c)) << (a) << " !=(approx.) " << (b)

// NOLINTBEGIN
// (clang-analyzer-deadcode.DeadStores,misc-const-correctness,clang-diagnostic-unused-result)

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

  Timetable timetable;
  timetable.add_empty_station("Station1");
  timetable.add_track_to_station("Station1", v3_v4b, network);
  timetable.add_track_to_station("Station1", v3_v4t, network);
  timetable.add_empty_station("Station2");
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
  const auto tr1 = timetable.add_train("Train1", 100, 50, 4, 2, true, 30, 15,
                                       v0t, 300, 20, v8, network);

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
  const auto tr2 = timetable.add_train("Train2", 300, 20, 4, 2, true, 60, 15,
                                       v0b, 340, 20, v8, network);

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
  const auto tr3 = timetable.add_train("Train3", 50, 20, 4, 2, true, 90, 15,
                                       v0t, 200, 20, v8, network);
  timetable.insert_stop(tr3, "Station1", 120, 60);
  // Quickest path from Station1 to Station2
  // v4b -> v5: 500 / 20 = 25 seconds
  // v5 -> v6: 50 / 20 = 2.5 seconds
  // Total: 27.5 seconds
  // Hence, at time 178.5 + 27.5 = 206 seconds (if earliest exit: 207.5)
  // Stopping for 30 seconds until 236
  timetable.insert_stop(tr3, "Station2", 204, 30);
  // Quickest path from Station2 to v8
  // Exit: 50 / 20 = 2.5 seconds
  // Total: 2.5 seconds
  // Hence, at time 236 + 2.5 = 238.5 seconds

  // Train 4 (Length 100, Max Speed 50)
  const auto tr4 = timetable.add_train("Train4", 100, 50, 4, 2, true, 0, 15,
                                       v0b, 100, 20, v8, network);
  // Entering: 0 seconds
  // v0b -> v1b: 50 / 25 = 2 seconds
  // v1b -> v2: 10 / 10 = 1 second
  // v2 -> v3: 150 / 10 = 15 seconds
  // v3 -> v4b: 100 / 20 = 5 seconds
  // Arriving at Station 1 at 23 seconds
  // Stopping for 30 seconds until 53 seconds
  timetable.insert_stop(tr4, "Station1", 20, 30);
  // v4b -> v5: 500 / 20 = 25 seconds
  // v5 -> v6: 50 / 20 = 2.5 seconds
  // v6 -> v7: 150 / 20 = 7.5 seconds
  // Arriving at Station 2 at 88 seconds
  // Stopping for 45 seconds until 133 seconds
  timetable.insert_stop(tr4, "Station2", 180, 45);
  // Exit: 100 / 50 = 2 seconds
  // Total: 135 seconds

  // Train 5 (Length 120, Max Speed 20)
  const auto tr5 = timetable.add_train("Train5", 120, 20, 4, 2, true, 0, 15,
                                       v0t, 100, 20, v8, network);
  timetable.insert_stop(tr5, "Station1", 20, 30);

  RouteMap                                                    routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);
  cda_rail::simulator::GreedySimulator simulator(instance, {});

  const auto [feas_tr1_a, obj_tr1_a, stops_tr1_a] =
      simulator::simple_remaining_time_heuristic(tr1, simulator, -1, false);
  EXPECT_TRUE(feas_tr1_a);
  EXPECT_EQ(obj_tr1_a, 92.5);
  EXPECT_APPROX_EQ_6(stops_tr1_a, 0);
  const auto [feas_tr1_b, obj_tr1_b, stops_tr1_b] =
      simulator::simple_remaining_time_heuristic(tr1, simulator, -1, true);
  EXPECT_TRUE(feas_tr1_b);
  EXPECT_EQ(obj_tr1_b, 300);
  EXPECT_APPROX_EQ_6(stops_tr1_b, 0);
  simulator.set_train_edges_of_tr(tr1, {v0t_v1t, v1t_v2, v2_v3});
  // Now the train is at v3
  // v3 -> v4t: 50 / 20 = 2.5 seconds
  // v4t -> v5: 1000 / 50 = 20 seconds
  // v5 -> v6: 50 / 20 = 2.5 seconds
  // v6 -> v7: 150 / 20 = 7.5 seconds
  // v7 -> v8: 50 / 25 = 2 seconds
  // Exit: 100 / 50 = 2 seconds
  // Total: 36.5 seconds
  const auto [feas_tr1_c, obj_tr1_c, stops_tr1_c] =
      simulator::simple_remaining_time_heuristic(tr1, simulator, 90, false);
  EXPECT_TRUE(feas_tr1_c);
  EXPECT_EQ(obj_tr1_c, 36.5);
  EXPECT_APPROX_EQ_6(stops_tr1_c, 0);
  const auto [feas_tr1_d, obj_tr1_d, stops_tr1_d] =
      simulator::simple_remaining_time_heuristic(tr1, simulator, 90, true);
  EXPECT_TRUE(feas_tr1_d);
  EXPECT_EQ(obj_tr1_d, 300 - 90);
  EXPECT_APPROX_EQ_6(stops_tr1_d, 0);
  simulator.set_train_edges_of_tr(tr1, {v0t_v1t, v1t_v2, v2_v3, v3_v4b});
  // Now the train is at v4b
  // v4b -> v5: 500 / 20 = 25 seconds
  // v5 -> v6: 50 / 20 = 2.5 seconds
  // v6 -> v7: 150 / 20 = 7.5 seconds
  // v7 -> v8: 50 / 25 = 2 seconds
  // Exit: 100 / 50 = 2 seconds
  // Total: 39 seconds
  const auto [feas_tr1_e, obj_tr1_e, stops_tr1_e] =
      simulator::simple_remaining_time_heuristic(tr1, simulator, 70, false);
  EXPECT_TRUE(feas_tr1_e);
  EXPECT_EQ(obj_tr1_e, 39);
  EXPECT_APPROX_EQ_6(stops_tr1_e, 0);
  const auto [feas_tr1_f, obj_tr1_f, stops_tr1_f] =
      simulator::simple_remaining_time_heuristic(tr1, simulator, 70, true);
  EXPECT_TRUE(feas_tr1_f);
  EXPECT_EQ(obj_tr1_f, 300 - 70);
  EXPECT_APPROX_EQ_6(stops_tr1_f, 0);
  simulator.set_train_edges_of_tr(
      tr1, {v0t_v1t, v1t_v2, v2_v3, v3_v4b, v4b_v5, v5_v6, v6_v7, v7_v8});
  // Now the train is at v8, i.e., at final destination already
  const auto [feas_tr1_g, obj_tr1_g, stops_tr1_g] =
      simulator::simple_remaining_time_heuristic(tr1, simulator, 100, false);
  EXPECT_TRUE(feas_tr1_g);
  EXPECT_EQ(obj_tr1_g, 0);
  EXPECT_APPROX_EQ_6(stops_tr1_g, 0);

  // Train 2
  const auto [feas_tr2_a, obj_tr2_a, stops_tr2_a] =
      simulator::simple_remaining_time_heuristic(tr2, simulator, -1, false);
  EXPECT_TRUE(feas_tr2_a);
  EXPECT_EQ(obj_tr2_a, 136);
  EXPECT_APPROX_EQ_6(stops_tr2_a, 0);
  const auto [feas_tr2_b, obj_tr2_b, stops_tr2_b] =
      simulator::simple_remaining_time_heuristic(tr2, simulator, -1, true);
  EXPECT_TRUE(feas_tr2_b);
  EXPECT_EQ(obj_tr2_b, 340);
  EXPECT_APPROX_EQ_6(stops_tr2_b, 0);
  simulator.set_train_edges_of_tr(tr2, {v0b_v1b, v1b_v2, v2_v3});
  // Now the train is at v3
  // v3 -> v4b: 100 / 20 = 5 seconds
  // v4b -> v5: 500 / 20 = 25 seconds
  // v5 -> v6: 50 / 20 = 2.5 seconds
  // v6 -> v7: 150 / 20 = 7.5 seconds
  // v7 -> v8: 50 / 20 = 2.5 seconds
  // Exit: 300 / 20 = 15 seconds
  // Total: 57.5 seconds
  const auto [feas_tr2_c, obj_tr2_c, stops_tr2_c] =
      simulator::simple_remaining_time_heuristic(tr2, simulator, 90, false);
  EXPECT_TRUE(feas_tr2_c);
  EXPECT_EQ(obj_tr2_c, 57.5);
  EXPECT_APPROX_EQ_6(stops_tr2_c, 0);
  const auto [feas_tr2_d, obj_tr2_d, stops_tr2_d] =
      simulator::simple_remaining_time_heuristic(tr2, simulator, 90, true);
  EXPECT_TRUE(feas_tr2_d);
  EXPECT_EQ(obj_tr2_d, 340 - 90);
  EXPECT_APPROX_EQ_6(stops_tr2_d, 0);
  // If tr_exit - 5 + 57.5 > 600 the train cannot exit the network in time
  // tr_exit > 547.5
  const auto [feas_tr2_f, obj_tr2_f, stops_tr2_f] =
      simulator::simple_remaining_time_heuristic(tr2, simulator, 548, false);
  EXPECT_TRUE(feas_tr2_f);
  EXPECT_EQ(obj_tr2_f, 57.5);
  EXPECT_APPROX_EQ_6(stops_tr2_f, 0);

  // Train 3
  const auto [feas_tr3_a, obj_tr3_a, stops_tr3_a] =
      simulator::simple_remaining_time_heuristic(tr3, simulator, -1, false);
  EXPECT_TRUE(feas_tr3_a);
  EXPECT_EQ(obj_tr3_a, 238.5);
  EXPECT_APPROX_EQ_6(stops_tr3_a, 1);
  const auto [feas_tr3_b, obj_tr3_b, stops_tr3_b] =
      simulator::simple_remaining_time_heuristic(tr3, simulator, -1, true);
  EXPECT_TRUE(feas_tr3_b);
  EXPECT_EQ(obj_tr3_b, 240);
  EXPECT_APPROX_EQ_6(stops_tr3_b, 1.75);
  simulator.set_train_edges_of_tr(tr3,
                                  {v0t_v1t, v1t_v2, v2_v3, v3_v4t, v4t_v5});
  simulator.append_stop_edge_to_tr(tr3, v3_v4t);
  // Now the train is at v5
  // v5 -> v6: 50 / 20 = 2.5 seconds
  // Stopping at Station2 for 30 seconds
  // Exit: 50 / 20 = 2.5 seconds
  const auto [feas_tr3_c, obj_tr3_c, stops_tr3_c] =
      simulator::simple_remaining_time_heuristic(tr3, simulator, 200, false);
  EXPECT_TRUE(feas_tr3_c);
  EXPECT_EQ(obj_tr3_c, 35.0);
  EXPECT_APPROX_EQ_6(stops_tr3_c, 0);
  const auto [feas_tr3_d, obj_tr3_d, stops_tr3_d] =
      simulator::simple_remaining_time_heuristic(tr3, simulator, 200, true);
  EXPECT_TRUE(feas_tr3_d);
  EXPECT_EQ(obj_tr3_d, 204 + 30 + 2.5 - 200);
  EXPECT_APPROX_EQ_6(stops_tr3_d, 0);

  const auto [feas_tr3_f, obj_tr3_f, stops_tr3_f] =
      simulator::simple_remaining_time_heuristic(tr3, simulator, 300, false);
  EXPECT_TRUE(feas_tr3_f);
  EXPECT_EQ(obj_tr3_f, 35.0);
  EXPECT_APPROX_EQ_6(stops_tr3_f,
                     98.5 / 2); // 302.5 instead of 204 -> 98.5 delayed

  // Train 4
  const auto [feas_tr4_a, obj_tr4_a, stops_tr4_a] =
      simulator::simple_remaining_time_heuristic(tr4, simulator, -1, false);
  EXPECT_TRUE(feas_tr4_a);
  EXPECT_EQ(obj_tr4_a, 135);
  EXPECT_APPROX_EQ_6(stops_tr4_a, 1.5);

  const auto [feas_tr4_b, obj_tr4_b, stops_tr4_b] =
      simulator::remaining_time_heuristic(
          simulator::RemainingTimeHeuristicType::Zero, tr4, simulator, -1,
          false);
  EXPECT_TRUE(feas_tr4_b);
  EXPECT_EQ(obj_tr4_b, 0);
  EXPECT_APPROX_EQ_6(stops_tr4_b, 0);

  // Train 5, too long for station
  const auto [feas_tr5_a, obj_tr5_a, stops_tr5_a] =
      simulator::simple_remaining_time_heuristic(tr5, simulator, -1, false);
  EXPECT_FALSE(feas_tr5_a);
  EXPECT_APPROX_EQ_6(obj_tr5_a, cda_rail::INF);
}

TEST(GreedyHeuristic, InfeasibleStopRequests) {
  Network    network;
  auto const v0 = network.add_vertex("v0", VertexType::TTD);
  auto const v1 = network.add_vertex("v1", VertexType::TTD);
  auto const v2 = network.add_vertex("v2", VertexType::TTD);
  auto const v3 = network.add_vertex("v3", VertexType::TTD);
  auto const v4 = network.add_vertex("v4", VertexType::TTD);
  auto const v5 = network.add_vertex("v5", VertexType::TTD);
  auto const v6 = network.add_vertex("v6", VertexType::TTD);

  auto const v0_v1 = network.add_edge(v0, v1, 100, 10);
  auto const v1_v2 = network.add_edge(v1, v2, 100, 5);  // Station
  auto const v2_v3 = network.add_edge(v2, v3, 100, 25); // Station
  auto const v3_v4 = network.add_edge(v3, v4, 100, 50);
  auto const v4_v5 = network.add_edge(v4, v5, 100, 100); // Station (too short)
  auto const v5_v6 = network.add_edge(v5, v6, 100, 20);

  network.add_successor(v0_v1, v1_v2);
  network.add_successor(v1_v2, v2_v3);
  network.add_successor(v2_v3, v3_v4);
  network.add_successor(v3_v4, v4_v5);
  network.add_successor(v4_v5, v5_v6);

  Timetable timetable;
  timetable.add_empty_station("Station1");
  timetable.add_track_to_station("Station1", v1_v2, network);
  timetable.add_track_to_station("Station1", v2_v3, network);
  timetable.add_track_to_station("Station1", v4_v5, network);

  auto const tr1 = timetable.add_train("Train1", 150, 100, 4, 2, true, 0, 10,
                                       v0, 0, 20, v6, network);
  timetable.insert_stop(tr1, "Station1", 30, 200);

  RouteMap                                                    routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);
  cda_rail::simulator::GreedySimulator simulator(instance, {});

  auto [feas_tr1, obj_tr1, stops_tr1] =
      simulator::simple_remaining_time_heuristic(tr1, simulator, -1, false);
  EXPECT_TRUE(feas_tr1);
  EXPECT_EQ(obj_tr1, 100.0 / 10.0 + 100.0 / 5.0 + 100.0 / 25.0 + 100.0 / 50.0 +
                         100.0 / 100.0 + 100.0 / 20.0 + 150.0 / 100.0 + 200.0);
  EXPECT_EQ(stops_tr1, 100.0 / 10.0 + 100.0 / 5.0 + 100.0 / 25.0 - 30.0);

  simulator.append_train_edge_to_tr(tr1, v0_v1);
  simulator.append_train_edge_to_tr(tr1, v1_v2);
  auto [feas_tr2, obj_tr2, stops_tr2] =
      simulator::simple_remaining_time_heuristic(tr1, simulator, 500, false);
  EXPECT_TRUE(feas_tr2);
  EXPECT_EQ(obj_tr2, 100.0 / 25.0 + 100.0 / 50.0 + 100.0 / 100.0 +
                         100.0 / 20.0 + 150.0 / 100.0 + 200.0);
  EXPECT_EQ(stops_tr2, 500 + 100.0 / 25.0 - 30.0);

  simulator.append_train_edge_to_tr(tr1, v2_v3);
  auto [feas_tr3, obj_tr3, stops_tr3] =
      simulator::simple_remaining_time_heuristic(tr1, simulator, 500, false);
  EXPECT_FALSE(feas_tr3);
  EXPECT_EQ(obj_tr3, cda_rail::INF);
  EXPECT_EQ(stops_tr3, cda_rail::INF);

  simulator.append_current_stop_position_of_tr(tr1);
  auto [feas_tr4, obj_tr4, stops_tr4] =
      simulator::simple_remaining_time_heuristic(tr1, simulator, 500, false);
  EXPECT_TRUE(feas_tr4);
  EXPECT_EQ(obj_tr4,
            100.0 / 50.0 + 100.0 / 100.0 + 100.0 / 20.0 + 150.0 / 100.0);
  EXPECT_EQ(stops_tr4, 0);
}

TEST(GreedyHeuristic, FullGreedyHeuristicRejectsMismatchedResultSizes) {
  Network    network;
  const auto v0    = network.add_vertex("v0", VertexType::TTD);
  const auto v1    = network.add_vertex("v1", VertexType::TTD);
  const auto v0_v1 = network.add_edge(v0, v1, 100, 10);

  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 100, 10, 1, 1, true, 0, 0, v0,
                                       120, 0, v1, network);

  RouteMap                                                    routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);
  cda_rail::simulator::GreedySimulator simulator(instance, {});
  simulator.set_train_edges_of_tr(tr1, {v0_v1});

  cda_rail::simulator::SimulatorResults sim_results{};
  sim_results.success            = true;
  sim_results.exit_times         = {120.0};
  sim_results.stop_times         = {{}};
  sim_results.vertex_headways    = {};
  sim_results.train_trajectories = {};

  auto bad_exit_times = sim_results;
  bad_exit_times.exit_times.clear();
  EXPECT_THROW((void)cda_rail::simulator::full_greedy_heuristic(
                   cda_rail::simulator::RemainingTimeHeuristicType::Simple,
                   simulator, bad_exit_times, false),
               cda_rail::exceptions::ConsistencyException);
}

TEST(GreedyHeuristic, FinalStateHeuristic) {
  Network    network;
  const auto v0 = network.add_vertex("v0", VertexType::TTD);
  const auto v1 = network.add_vertex("v1", VertexType::TTD);
  const auto v2 = network.add_vertex("v2", VertexType::TTD);

  const auto v0_v1 = network.add_edge(v0, v1, 100, 10);
  const auto v1_v2 = network.add_edge(v1, v2, 100, 10);
  network.add_successor(v0_v1, v1_v2);

  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 100, 10, 4, 2, true, 0, 10, v0,
                                       10, 10, v2, network);
  RouteMap   routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);
  cda_rail::simulator::GreedySimulator simulator(instance, {});

  simulator.set_train_edges_of_tr(tr1, {v0_v1, v1_v2});

  const auto [heur_feas, heur_val, stops_val] =
      cda_rail::simulator::simple_remaining_time_heuristic(tr1, simulator, 30,
                                                           true);
  EXPECT_TRUE(heur_feas);
  EXPECT_EQ(heur_val, 0);  // already at exit
  EXPECT_EQ(stops_val, 0); // already at exit
}

TEST(GreedyHeuristic, RemainingTimeHeuristicTypeToString) {
  EXPECT_EQ(remaining_time_heuristic_type_to_string(
                simulator::RemainingTimeHeuristicType::Zero),
            "Zero");
  EXPECT_EQ(remaining_time_heuristic_type_to_string(
                simulator::RemainingTimeHeuristicType::Simple),
            "Simple");
  EXPECT_THROW(remaining_time_heuristic_type_to_string(
                   static_cast<simulator::RemainingTimeHeuristicType>(255)),
               cda_rail::exceptions::ConsistencyException);
}

// NOLINTEND
// (clang-analyzer-deadcode.DeadStores,misc-const-correctness,clang-diagnostic-unused-result)
