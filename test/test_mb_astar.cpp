#include <cstdlib>
#define TEST_FRIENDS true

#include "Definitions.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Route.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "simulator/GreedySimulator.hpp"
#include "solver/astar-based/GenPOMovingBlockAStarSolver.hpp"

#include "gtest/gtest.h"

using namespace cda_rail;

// NOLINTBEGIN
// (clang-analyzer-deadcode.DeadStores,misc-const-correctness,clang-diagnostic-unused-result)

TEST(GenPOMovingBlockAStarSolver, NextStates) {
  Network    network;
  const auto v0  = network.add_vertex("v0", VertexType::TTD);
  const auto v1  = network.add_vertex("v1", VertexType::TTD);
  const auto v2  = network.add_vertex("v2", VertexType::TTD);
  const auto v3a = network.add_vertex("v3a", VertexType::TTD);
  const auto v3b = network.add_vertex("v3b", VertexType::TTD);
  const auto v4a = network.add_vertex("v4a", VertexType::TTD);
  const auto v4b = network.add_vertex("v4b", VertexType::TTD);
  const auto v5  = network.add_vertex("v5", VertexType::TTD);
  const auto v6  = network.add_vertex("v6", VertexType::TTD);
  const auto v7  = network.add_vertex("v7", VertexType::TTD);

  const auto v0_v1   = network.add_edge(v0, v1, 100, 50);
  const auto v1_v2   = network.add_edge(v1, v2, 10, 50, false);
  const auto v2_v3a  = network.add_edge(v2, v3a, 10, 50, false);
  const auto v2_v3b  = network.add_edge(v2, v3b, 10, 50, false);
  const auto v3a_v4a = network.add_edge(v3a, v4a, 200, 50);
  const auto v3b_v4b = network.add_edge(v3b, v4b, 210, 50);
  const auto v4a_v5  = network.add_edge(v4a, v5, 10, 50, false);
  const auto v4b_v5  = network.add_edge(v4b, v5, 10, 50, false);
  const auto v5_v6   = network.add_edge(v5, v6, 10, 50, false);
  const auto v6_v7   = network.add_edge(v6, v7, 200, 50);

  network.add_successor(v0_v1, v1_v2);
  network.add_successor(v1_v2, v2_v3a);
  network.add_successor(v1_v2, v2_v3b);
  network.add_successor(v2_v3a, v3a_v4a);
  network.add_successor(v2_v3b, v3b_v4b);
  network.add_successor(v3a_v4a, v4a_v5);
  network.add_successor(v3b_v4b, v4b_v5);
  network.add_successor(v4a_v5, v5_v6);
  network.add_successor(v4b_v5, v5_v6);
  network.add_successor(v5_v6, v6_v7);

  GeneralTimetable<GeneralSchedule<GeneralScheduledStop>> timetable;
  timetable.add_station("Station1");
  timetable.add_track_to_station("Station1", v3a_v4a, network);
  timetable.add_track_to_station("Station1", v3b_v4b, network);
  timetable.add_station("Station2");
  timetable.add_track_to_station("Station2", v6_v7, network);

  const auto tr1 = timetable.add_train("Train1", 100, 50, 4, 2, true, {0, 60},
                                       15, v0, {300, 600}, 40, v7, network);
  const auto tr2 = timetable.add_train("Train2", 100, 50, 4, 2, true, {0, 60},
                                       25, v0, {300, 600}, 40, v7, network);
  timetable.add_stop(tr1, "Station1", {20, 100}, {40, 120}, 30);
  timetable.add_stop(tr1, "Station2", {200, 300}, {220, 320}, 30);

  RouteMap                                                    routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);
  cda_rail::simulator::GreedySimulator simulator(
      instance, {{v1_v2, v2_v3a, v2_v3b}, {v4a_v5, v4b_v5, v5_v6}});

  cda_rail::solver::astar_based::GreedySimulatorState expected_state1_1{
      .train_edges    = {{v0_v1}, {}},
      .ttd_orders     = {{}, {}},
      .vertex_orders  = std::vector<std::vector<size_t>>(10),
      .stop_positions = {{}, {}}};
  expected_state1_1.vertex_orders.at(v0).emplace_back(tr1);
  cda_rail::solver::astar_based::GreedySimulatorState expected_state1_2{
      .train_edges    = {{}, {v0_v1, v1_v2, v2_v3a, v3a_v4a}},
      .ttd_orders     = {{tr2}, {}},
      .vertex_orders  = std::vector<std::vector<size_t>>(10),
      .stop_positions = {{}, {}}};
  expected_state1_2.vertex_orders.at(v0).emplace_back(tr2);
  cda_rail::solver::astar_based::GreedySimulatorState expected_state1_3{
      .train_edges    = {{}, {v0_v1, v1_v2, v2_v3b, v3b_v4b}},
      .ttd_orders     = {{tr2}, {}},
      .vertex_orders  = std::vector<std::vector<size_t>>(10),
      .stop_positions = {{}, {}}};
  expected_state1_3.vertex_orders.at(v0).emplace_back(tr2);
  const auto next_states1 =
      cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::next_states(
          simulator,
          cda_rail::solver::astar_based::NextStateStrategy::SingleEdge);
  EXPECT_EQ(next_states1.size(), 3);
  EXPECT_TRUE(next_states1.contains(expected_state1_1));
  EXPECT_TRUE(next_states1.contains(expected_state1_2));
  EXPECT_TRUE(next_states1.contains(expected_state1_3));
  EXPECT_EQ(simulator.get_train_edges().size(), 2);
  EXPECT_TRUE(simulator.get_train_edges_of_tr(tr1).empty());
  EXPECT_TRUE(simulator.get_train_edges_of_tr(tr2).empty());

  simulator.set_train_edges_of_tr(tr2, {v0_v1, v1_v2});
  simulator.set_vertex_orders_of_vertex(v0, {tr2});
  simulator.set_ttd_orders_of_ttd(0, {tr2});

  cda_rail::solver::astar_based::GreedySimulatorState expected_state2_1{
      .train_edges    = {{v0_v1}, {v0_v1, v1_v2}},
      .ttd_orders     = {{tr2}, {}},
      .vertex_orders  = std::vector<std::vector<size_t>>(10),
      .stop_positions = {{}, {}}};
  expected_state2_1.vertex_orders.at(v0).emplace_back(tr2);
  expected_state2_1.vertex_orders.at(v0).emplace_back(tr1);
  cda_rail::solver::astar_based::GreedySimulatorState expected_state2_2 = {
      .train_edges    = {{}, {v0_v1, v1_v2, v2_v3a}},
      .ttd_orders     = {{tr2}, {}},
      .vertex_orders  = std::vector<std::vector<size_t>>(10),
      .stop_positions = {{}, {}}};
  expected_state2_2.vertex_orders.at(v0).emplace_back(tr2);
  cda_rail::solver::astar_based::GreedySimulatorState expected_state2_3 = {
      .train_edges    = {{}, {v0_v1, v1_v2, v2_v3b}},
      .ttd_orders     = {{tr2}, {}},
      .vertex_orders  = std::vector<std::vector<size_t>>(10),
      .stop_positions = {{}, {}}};
  expected_state2_3.vertex_orders.at(v0).emplace_back(tr2);
  const auto next_states2 = cda_rail::solver::astar_based::
      GenPOMovingBlockAStarSolver::next_states_single_edge(simulator);
  EXPECT_EQ(next_states2.size(), 3);
  EXPECT_TRUE(next_states2.contains(expected_state2_1));
  EXPECT_TRUE(next_states2.contains(expected_state2_2));
  EXPECT_TRUE(next_states2.contains(expected_state2_3));

  simulator.set_train_edges_of_tr(
      tr2, {v0_v1, v1_v2, v2_v3a, v3a_v4a, v4a_v5, v5_v6});
  simulator.set_train_edges_of_tr(tr1, {v0_v1});
  simulator.set_vertex_orders_of_vertex(v0, {tr2, tr1});
  simulator.set_ttd_orders_of_ttd(0, {tr2});
  simulator.set_ttd_orders_of_ttd(1, {tr2});

  cda_rail::solver::astar_based::GreedySimulatorState expected_state3_1{
      .train_edges    = {{v0_v1},
                         {v0_v1, v1_v2, v2_v3a, v3a_v4a, v4a_v5, v5_v6, v6_v7}},
      .ttd_orders     = {{tr2}, {tr2}},
      .vertex_orders  = std::vector<std::vector<size_t>>(10),
      .stop_positions = {{}, {}}};
  expected_state3_1.vertex_orders.at(v0) = {tr2, tr1};
  expected_state3_1.vertex_orders.at(v7).emplace_back(tr2);
  cda_rail::solver::astar_based::GreedySimulatorState expected_state3_2 = {
      .train_edges    = {{v0_v1, v1_v2},
                         {v0_v1, v1_v2, v2_v3a, v3a_v4a, v4a_v5, v5_v6}},
      .ttd_orders     = {{tr2, tr1}, {tr2}},
      .vertex_orders  = std::vector<std::vector<size_t>>(10),
      .stop_positions = {{}, {}}};
  expected_state3_2.vertex_orders.at(v0) = {tr2, tr1};
  const auto next_states3                = cda_rail::solver::astar_based::
      GenPOMovingBlockAStarSolver::next_states_single_edge(simulator);
  EXPECT_EQ(next_states3.size(), 2);
  EXPECT_TRUE(next_states3.contains(expected_state3_1));
  EXPECT_TRUE(next_states3.contains(expected_state3_2));

  simulator.set_train_edges_of_tr(
      tr2, {v0_v1, v1_v2, v2_v3a, v3a_v4a, v4a_v5, v5_v6, v6_v7});
  simulator.set_vertex_orders_of_vertex(v7, {tr2});
  simulator.set_train_edges_of_tr(tr1, {v0_v1, v1_v2, v2_v3b, v3b_v4b});
  simulator.set_ttd_orders_of_ttd(0, {tr2, tr1});

  cda_rail::solver::astar_based::GreedySimulatorState expected_state4_1{
      .train_edges    = {{v0_v1, v1_v2, v2_v3b, v3b_v4b, v4b_v5},
                         {v0_v1, v1_v2, v2_v3a, v3a_v4a, v4a_v5, v5_v6, v6_v7}},
      .ttd_orders     = {{tr2, tr1}, {tr2, tr1}},
      .vertex_orders  = std::vector<std::vector<size_t>>(10),
      .stop_positions = {{}, {}}};
  expected_state4_1.vertex_orders.at(v0) = {tr2, tr1};
  expected_state4_1.vertex_orders.at(v7) = {tr2};
  cda_rail::solver::astar_based::GreedySimulatorState expected_state4_2 = {
      .train_edges    = {{v0_v1, v1_v2, v2_v3b, v3b_v4b},
                         {v0_v1, v1_v2, v2_v3a, v3a_v4a, v4a_v5, v5_v6, v6_v7}},
      .ttd_orders     = {{tr2, tr1}, {tr2}},
      .vertex_orders  = std::vector<std::vector<size_t>>(10),
      .stop_positions = {{330}, {}}};
  expected_state4_2.vertex_orders.at(v0) = {tr2, tr1};
  expected_state4_2.vertex_orders.at(v7) = {tr2};
  const auto next_states4                = cda_rail::solver::astar_based::
      GenPOMovingBlockAStarSolver::next_states_single_edge(simulator);
  EXPECT_EQ(next_states4.size(), 2);
  EXPECT_TRUE(next_states4.contains(expected_state4_1));
  EXPECT_TRUE(next_states4.contains(expected_state4_2));

  simulator.append_current_stop_position_of_tr(tr1);
  cda_rail::solver::astar_based::GreedySimulatorState expected_state5_1 =
      expected_state4_1;
  expected_state5_1.stop_positions.at(0).emplace_back(330);
  const auto next_states5 = cda_rail::solver::astar_based::
      GenPOMovingBlockAStarSolver::next_states_single_edge(simulator);
  EXPECT_EQ(next_states5.size(), 1);
  EXPECT_TRUE(next_states5.contains(expected_state5_1));

  simulator.set_train_edges_of_tr(
      tr1, {v0_v1, v1_v2, v2_v3b, v3b_v4b, v4b_v5, v5_v6, v6_v7});
  simulator.set_ttd_orders_of_ttd(1, {tr2, tr1});

  cda_rail::solver::astar_based::GreedySimulatorState expected_state6_1 = {
      .train_edges    = {{v0_v1, v1_v2, v2_v3b, v3b_v4b, v4b_v5, v5_v6, v6_v7},
                         {v0_v1, v1_v2, v2_v3a, v3a_v4a, v4a_v5, v5_v6, v6_v7}},
      .ttd_orders     = {{tr2, tr1}, {tr2, tr1}},
      .vertex_orders  = std::vector<std::vector<size_t>>(10),
      .stop_positions = {{330, 550}, {}}};
  expected_state6_1.vertex_orders.at(v0) = {tr2, tr1};
  expected_state6_1.vertex_orders.at(v7) = {tr2, tr1};
  const auto next_states6                = cda_rail::solver::astar_based::
      GenPOMovingBlockAStarSolver::next_states_single_edge(simulator);
  EXPECT_EQ(next_states6.size(), 1);
  EXPECT_TRUE(next_states6.contains(expected_state6_1));

  simulator.append_current_stop_position_of_tr(tr1);
  const auto next_states7 = cda_rail::solver::astar_based::
      GenPOMovingBlockAStarSolver::next_states_single_edge(simulator);
  EXPECT_TRUE(next_states7.empty());
}

TEST(GenPOMovingBlockAStarSolver, NextStatesTTD) {
  Network    network;
  const auto v0  = network.add_vertex("v0", VertexType::TTD);
  const auto v1  = network.add_vertex("v1", VertexType::TTD);
  const auto v2  = network.add_vertex("v2", VertexType::NoBorder);
  const auto v3a = network.add_vertex("v3a", VertexType::TTD);
  const auto v3b = network.add_vertex("v3b", VertexType::TTD);
  const auto v4a = network.add_vertex("v4a", VertexType::TTD);
  const auto v5a = network.add_vertex("v5a", VertexType::TTD);
  const auto v5b = network.add_vertex("v5b", VertexType::TTD);
  const auto v6  = network.add_vertex("v6", VertexType::NoBorder);
  const auto v7a = network.add_vertex("v7a", VertexType::TTD);
  const auto v7b = network.add_vertex("v7b", VertexType::TTD);
  const auto v8a = network.add_vertex("v8a", VertexType::TTD);
  const auto v8b = network.add_vertex("v8b", VertexType::TTD);

  const auto v0_v1   = network.add_edge(v0, v1, 100, 50, true);
  const auto v1_v2   = network.add_edge(v1, v2, 10, 50, false);
  const auto v2_v3a  = network.add_edge(v2, v3a, 10, 50, false);
  const auto v2_v3b  = network.add_edge(v2, v3b, 10, 50, false);
  const auto v3a_v4a = network.add_edge(v3a, v4a, 100, 50, true);
  const auto v4a_v5a = network.add_edge(v4a, v5a, 100, 50, true);
  const auto v3b_v5b = network.add_edge(v3b, v5b, 200, 50, true);
  const auto v5a_v6  = network.add_edge(v5a, v6, 10, 50, false);
  const auto v5b_v6  = network.add_edge(v5b, v6, 10, 50, false);
  const auto v6_v7a  = network.add_edge(v6, v7a, 10, 50, false);
  const auto v6_v7b  = network.add_edge(v6, v7b, 10, 50, false);
  const auto v7a_v8a = network.add_edge(v7a, v8a, 200, 50, true);
  const auto v7b_v8b = network.add_edge(v7b, v8b, 200, 50, true);

  network.add_successor(v0_v1, v1_v2);
  network.add_successor(v1_v2, v2_v3a);
  network.add_successor(v1_v2, v2_v3b);
  network.add_successor(v2_v3a, v3a_v4a);
  network.add_successor(v2_v3b, v3b_v5b);
  network.add_successor(v3a_v4a, v4a_v5a);
  network.add_successor(v3b_v5b, v5b_v6);
  network.add_successor(v4a_v5a, v5a_v6);
  network.add_successor(v5a_v6, v6_v7a);
  network.add_successor(v5a_v6, v6_v7b);
  network.add_successor(v5b_v6, v6_v7a);
  network.add_successor(v5b_v6, v6_v7b);
  network.add_successor(v6_v7a, v7a_v8a);
  network.add_successor(v6_v7b, v7b_v8b);

  GeneralTimetable<GeneralSchedule<GeneralScheduledStop>> timetable;
  timetable.add_station("Station1");
  timetable.add_track_to_station("Station1", v3a_v4a, network);
  timetable.add_track_to_station("Station1", v4a_v5a, network);
  timetable.add_track_to_station("Station1", v3b_v5b, network);
  timetable.add_station("Station2");
  timetable.add_track_to_station("Station2", v7a_v8a, network);
  timetable.add_track_to_station("Station2", v7b_v8b, network);
  timetable.add_station("Station0");
  timetable.add_track_to_station("Station0", v0_v1, network);

  const auto tr1 = timetable.add_train("Train1", 100, 50, 4, 2, true, {0, 60},
                                       15, v0, {300, 600}, 40, v8a, network);
  timetable.add_stop(tr1, "Station0", {20, 100}, {40, 120}, 30);
  timetable.add_stop(tr1, "Station1", {100, 150}, {130, 180}, 30);
  timetable.add_stop(tr1, "Station2", {200, 300}, {220, 320}, 30);

  RouteMap                                                    routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);

  const std::vector<std::vector<size_t>> ttd_sections = {
      {v1_v2, v2_v3a, v3a_v4a}, {v5a_v6, v5b_v6, v6_v7a, v7a_v8a}};
  cda_rail::simulator::GreedySimulator simulator(instance, ttd_sections);

  const auto num_vertices = network.number_of_vertices();

  cda_rail::solver::astar_based::GreedySimulatorState expected_state1_1{
      .train_edges    = {{v0_v1}},
      .ttd_orders     = {{}, {}},
      .vertex_orders  = std::vector<std::vector<size_t>>(num_vertices),
      .stop_positions = {{}}};
  expected_state1_1.vertex_orders.at(v0).emplace_back(tr1);
  cda_rail::solver::astar_based::GreedySimulatorState expected_state1_2{
      .train_edges    = {{v0_v1}},
      .ttd_orders     = {{}, {}},
      .vertex_orders  = std::vector<std::vector<size_t>>(num_vertices),
      .stop_positions = {{100}}};
  expected_state1_2.vertex_orders.at(v0).emplace_back(tr1);
  const auto next_states1 =
      cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::next_states(
          simulator, cda_rail::solver::astar_based::NextStateStrategy::NextTTD);
  EXPECT_EQ(next_states1.size(), 2);
  EXPECT_TRUE(next_states1.contains(expected_state1_1));
  EXPECT_TRUE(next_states1.contains(expected_state1_2));

  simulator.set_train_edges_of_tr(tr1, {v0_v1});
  simulator.set_vertex_orders_of_vertex(v0, {tr1});

  cda_rail::solver::astar_based::GreedySimulatorState expected_state2_1{
      .train_edges    = {{v0_v1, v1_v2, v2_v3a, v3a_v4a, v4a_v5a}},
      .ttd_orders     = {{tr1}, {}},
      .vertex_orders  = std::vector<std::vector<size_t>>(num_vertices),
      .stop_positions = {{}}};
  expected_state2_1.vertex_orders.at(v0).emplace_back(tr1);
  cda_rail::solver::astar_based::GreedySimulatorState expected_state2_2{
      .train_edges    = {{v0_v1, v1_v2, v2_v3b, v3b_v5b}},
      .ttd_orders     = {{tr1}, {}},
      .vertex_orders  = std::vector<std::vector<size_t>>(num_vertices),
      .stop_positions = {{}}};
  expected_state2_2.vertex_orders.at(v0).emplace_back(tr1);
  const auto next_states2 = cda_rail::solver::astar_based::
      GenPOMovingBlockAStarSolver::next_states_next_ttd(simulator);
  EXPECT_EQ(next_states2.size(), 2);
  EXPECT_TRUE(next_states2.contains(expected_state2_1));
  EXPECT_TRUE(next_states2.contains(expected_state2_2));

  simulator.append_current_stop_position_of_tr(tr1);
  cda_rail::solver::astar_based::GreedySimulatorState expected_state3_1 =
      expected_state2_1;
  expected_state3_1.stop_positions.at(0).emplace_back(100);
  cda_rail::solver::astar_based::GreedySimulatorState expected_state3_2 =
      expected_state2_2;
  expected_state3_2.stop_positions.at(0).emplace_back(100);
  cda_rail::solver::astar_based::GreedySimulatorState expected_state3_3{
      .train_edges    = {{v0_v1, v1_v2, v2_v3a, v3a_v4a, v4a_v5a}},
      .ttd_orders     = {{tr1}, {}},
      .vertex_orders  = std::vector<std::vector<size_t>>(num_vertices),
      .stop_positions = {{100, 320}}};
  expected_state3_3.vertex_orders.at(v0).emplace_back(tr1);
  cda_rail::solver::astar_based::GreedySimulatorState expected_state3_4{
      .train_edges    = {{v0_v1, v1_v2, v2_v3a, v3a_v4a}},
      .ttd_orders     = {{tr1}, {}},
      .vertex_orders  = std::vector<std::vector<size_t>>(num_vertices),
      .stop_positions = {{100, 220}}};
  expected_state3_4.vertex_orders.at(v0).emplace_back(tr1);
  cda_rail::solver::astar_based::GreedySimulatorState expected_state3_5{
      .train_edges    = {{v0_v1, v1_v2, v2_v3b, v3b_v5b}},
      .ttd_orders     = {{tr1}, {}},
      .vertex_orders  = std::vector<std::vector<size_t>>(num_vertices),
      .stop_positions = {{100, 320}}};
  expected_state3_5.vertex_orders.at(v0).emplace_back(tr1);
  const auto next_states3 = cda_rail::solver::astar_based::
      GenPOMovingBlockAStarSolver::next_states_next_ttd(simulator);
  EXPECT_EQ(next_states3.size(), 5);
  EXPECT_TRUE(next_states3.contains(expected_state3_1));
  EXPECT_TRUE(next_states3.contains(expected_state3_2));
  EXPECT_TRUE(next_states3.contains(expected_state3_3));
  EXPECT_TRUE(next_states3.contains(expected_state3_4));
  EXPECT_TRUE(next_states3.contains(expected_state3_5));

  simulator.set_train_edges_of_tr(tr1,
                                  {v0_v1, v1_v2, v2_v3a, v3a_v4a, v4a_v5a});
  simulator.set_ttd_orders_of_ttd(0, {tr1});
  simulator.append_current_stop_position_of_tr(tr1);
  cda_rail::solver::astar_based::GreedySimulatorState expected_state4_1{
      .train_edges   = {{v0_v1, v1_v2, v2_v3a, v3a_v4a, v4a_v5a, v5a_v6, v6_v7a,
                         v7a_v8a}},
      .ttd_orders    = {{tr1}, {tr1}},
      .vertex_orders = std::vector<std::vector<size_t>>(num_vertices),
      .stop_positions = {{100, 320}}};
  expected_state4_1.vertex_orders.at(v0).emplace_back(tr1);
  cda_rail::solver::astar_based::GreedySimulatorState expected_state4_2 =
      expected_state4_1;
  expected_state4_2.stop_positions.at(0).emplace_back(540);
  expected_state4_2.vertex_orders.at(v8a).emplace_back(tr1);
  const auto next_states4 = cda_rail::solver::astar_based::
      GenPOMovingBlockAStarSolver::next_states_next_ttd(simulator);
  EXPECT_EQ(next_states4.size(), 2);
  EXPECT_TRUE(next_states4.contains(expected_state4_1));
  EXPECT_TRUE(next_states4.contains(expected_state4_2));
}

TEST(GenPOMovingBlockAStarSolver, SimpleInstance) {
  Network    network;
  const auto v0 = network.add_vertex("v0", VertexType::TTD, 60);
  const auto v1 = network.add_vertex("v1", VertexType::TTD, 30);

  const auto v0_v1 = network.add_edge(v0, v1, 5000, 50, true);
  GeneralTimetable<GeneralSchedule<GeneralScheduledStop>> timetable;
  const auto tr1 = timetable.add_train("Train1", 100, 50, 4, 2, true, {0, 60},
                                       15, v0, {30, 400}, 40, v1, network);
  RouteMap   routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);

  cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver solver(instance);
  const auto sol_obj = solver.solve(-1, true);
}

TEST(GenPOMovingBlockAStarSolver, SimpleNetwork) {
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      "example-networks-gen-po/GeneralSimpleNetworkB3Trains");

  cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver solver(instance);
  const auto sol_obj = solver.solve(
      {},
      {.next_state_strategy =
           cda_rail::solver::astar_based::NextStateStrategy::NextTTD,
       .consider_earliest_exit = true},
      {}, -1, false);

  EXPECT_TRUE(sol_obj.has_solution());
  EXPECT_EQ(sol_obj.get_status(), cda_rail::SolutionStatus::Optimal);
}
TEST(GenPOMovingBlockAStarSolver, Timeout) {
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      "example-networks-gen-po/GeneralSimpleNetworkB6Trains");

  cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver solver(instance);
  const auto sol_obj = solver.solve(
      {},
      {.next_state_strategy =
           cda_rail::solver::astar_based::NextStateStrategy::NextTTD,
       .consider_earliest_exit = true},
      {}, 1, false);

  EXPECT_FALSE(sol_obj.has_solution());
  EXPECT_EQ(sol_obj.get_status(), cda_rail::SolutionStatus::Timeout);
}

// NOLINTEND
// (clang-analyzer-deadcode.DeadStores,misc-const-correctness,clang-diagnostic-unused-result)
