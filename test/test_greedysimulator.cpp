#include <cstdlib>
#define TEST_FRIENDS true

#include "CustomExceptions.hpp"
#include "EOMHelper.hpp"
#include "GeneralHelper.hpp"
#include "datastructure/Timetable.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "simulator/GreedySimulator.hpp"

#include "gtest/gtest.h"
#include <cmath>
#include <plog/Appenders/ColorConsoleAppender.h>
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Init.h>
#include <plog/Log.h>
#include <plog/Severity.h>

using namespace cda_rail;

#define EXPECT_APPROX_EQ_6(a, b) EXPECT_APPROX_EQ(a, b, 1e-6)

#define EXPECT_APPROX_EQ(a, b, c)                                              \
  EXPECT_TRUE(std::abs((a) - (b)) < (c)) << (a) << " !=(approx.) " << (b)

// NOLINTBEGIN
// (clang-analyzer-deadcode.DeadStores,misc-const-correctness,clang-diagnostic-unused-result)

TEST(GreedySimulator, BasicFunctions) {
  // Create instance
  Network     network("SimpleStation", "./data/");
  const auto& ttd_sections = network.unbreakable_sections();
  const auto& l0_l1        = network.get_edge_index({"l0", "l1"});
  const auto& l1_l2        = network.get_edge_index({"l1", "l2"});
  const auto& l2_l3        = network.get_edge_index({"l2", "l3"});
  const auto& r0_r1        = network.get_edge_index({"r0", "r1"});
  const auto& r1_r2        = network.get_edge_index({"r1", "r2"});
  const auto& l3_g00       = network.get_edge_index({"l3", "g00"});
  const auto& g00_g01      = network.get_edge_index({"g00", "g01"});
  const auto& g01_r2       = network.get_edge_index({"g01", "r2"});
  const auto& r2_r1        = network.get_edge_index({"r2", "r1"});
  const auto& r1_r0        = network.get_edge_index({"r1", "r0"});

  Timetable  timetable;
  const auto l0  = network.get_vertex_index("l0");
  const auto r0  = network.get_vertex_index("r0");
  const auto tr1 = timetable.add_train("Train1", 100, 10, 1, 1, true, 0, 0,
                                       {"l0"}, 360, 0, {"r0"}, network);
  const auto tr2 = timetable.add_train("Train2", 100, 10, 1, 1, false, 30, 10,
                                       r0, 400, 5, l0, network);
  timetable.add_empty_station("Station1");
  timetable.add_track_to_station("Station1", {"g00", "g01"}, network);
  timetable.add_track_to_station("Station1", {"g01", "g00"}, network);
  timetable.add_track_to_station("Station1", {"g10", "g11"}, network);
  timetable.add_track_to_station("Station1", {"g11", "g10"}, network);
  timetable.insert_stop("Train1", "Station1", 60, 60);

  timetable.add_empty_station("Station2");
  timetable.add_track_to_station("Station2", {"r2", "r1"}, network);
  timetable.insert_stop("Train1", "Station2", 140, 60);
  EXPECT_TRUE(timetable.check_consistency(network));

  RouteMap routes;

  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);
  EXPECT_TRUE(instance.check_consistency(false));

  cda_rail::instances::GeneralPerformanceOptimizationInstance instance2(
      network, timetable, routes);
  instance2.add_empty_route("Train1");
  instance2.push_back_edge_to_route("Train1", {"l0", "l1"});
  EXPECT_FALSE(instance2.check_consistency(false));

  // Test basic functions of GreedySimulator

  // Train Edges
  cda_rail::simulator::GreedySimulator simulator(instance, ttd_sections);
  EXPECT_THROW((void)simulator.set_train_edges({{l0_l1}}),
               cda_rail::exceptions::InvalidInputException);
  simulator.set_train_edges({{l0_l1, l1_l2, l2_l3}, {r0_r1, r1_r2}});
  const auto& tr_edges1 = simulator.get_train_edges();
  EXPECT_EQ(tr_edges1.size(), 2);
  EXPECT_EQ(tr_edges1[0].size(), 3);
  EXPECT_EQ(tr_edges1[1].size(), 2);
  EXPECT_EQ(tr_edges1[0][0], l0_l1);
  EXPECT_EQ(tr_edges1[0][1], l1_l2);
  EXPECT_EQ(tr_edges1[0][2], l2_l3);
  EXPECT_EQ(tr_edges1[1][0], r0_r1);
  EXPECT_EQ(tr_edges1[1][1], r1_r2);
  simulator.set_train_edges_of_tr(0, {l0_l1, l1_l2});
  const auto& tr_edges2 = simulator.get_train_edges_of_tr(0);
  EXPECT_EQ(tr_edges2.size(), 2);
  EXPECT_EQ(tr_edges2[0], l0_l1);
  EXPECT_EQ(tr_edges2[1], l1_l2);
  simulator.append_train_edge_to_tr(0, l2_l3);
  const auto& tr_edges3 = simulator.get_train_edges_of_tr(0);
  EXPECT_EQ(tr_edges3.size(), 3);
  EXPECT_EQ(tr_edges3[0], l0_l1);
  EXPECT_EQ(tr_edges3[1], l1_l2);
  EXPECT_EQ(tr_edges3[2], l2_l3);

  EXPECT_THROW((void)simulator.get_train_edges_of_tr(1000),
               cda_rail::exceptions::TrainNotExistentException);
  EXPECT_THROW((void)simulator.set_train_edges_of_tr(1000, {l0_l1}),
               cda_rail::exceptions::TrainNotExistentException);

  // TTD Orders
  EXPECT_THROW((void)simulator.set_ttd_orders({}),
               cda_rail::exceptions::InvalidInputException);
  auto duplicate_ttd_orders = std::vector<std::vector<size_t>>(
      ttd_sections.size(), std::vector<size_t>());
  duplicate_ttd_orders.at(0) = {tr1, tr1};
  EXPECT_THROW((void)simulator.set_ttd_orders(duplicate_ttd_orders),
               cda_rail::exceptions::InvalidInputException);
  simulator.set_ttd_orders(std::vector<std::vector<size_t>>(
      ttd_sections.size(), std::vector<size_t>()));
  const auto& ttd_orders1 = simulator.get_ttd_orders();
  EXPECT_EQ(ttd_orders1.size(), ttd_sections.size());
  for (const auto& orders : ttd_orders1) {
    EXPECT_TRUE(orders.empty());
  }
  simulator.set_ttd_orders_of_ttd(0, {tr1, tr2});
  const auto& ttd_orders2 = simulator.get_ttd_orders_of_ttd(0);
  EXPECT_EQ(ttd_orders2.size(), 2);
  EXPECT_EQ(ttd_orders2[0], tr1);
  EXPECT_EQ(ttd_orders2[1], tr2);
  EXPECT_THROW((void)simulator.get_ttd_orders_of_ttd(1000),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW((void)simulator.set_ttd_orders_of_ttd(1000, {tr1}),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW((void)simulator.set_ttd_orders_of_ttd(0, {tr1, tr1}),
               cda_rail::exceptions::InvalidInputException);

  // Entry Orders
  EXPECT_THROW((void)simulator.set_vertex_orders({}),
               cda_rail::exceptions::InvalidInputException);
  auto duplicate_vertex_orders = std::vector<std::vector<size_t>>(
      network.number_of_vertices(), std::vector<size_t>());
  duplicate_vertex_orders.at(l0) = {tr1, tr1};
  EXPECT_THROW((void)simulator.set_vertex_orders(duplicate_vertex_orders),
               cda_rail::exceptions::InvalidInputException);
  simulator.set_vertex_orders(std::vector<std::vector<size_t>>(
      network.number_of_vertices(), std::vector<size_t>()));
  const auto& vertex_orders1 = simulator.get_vertex_orders();
  EXPECT_EQ(vertex_orders1.size(), network.number_of_vertices());
  for (const auto& orders : vertex_orders1) {
    EXPECT_TRUE(orders.empty());
  }
  simulator.set_vertex_orders_of_vertex(l0, {tr1});
  const auto& vertex_orders2 = simulator.get_vertex_orders_of_vertex(l0);
  EXPECT_EQ(vertex_orders2.size(), 1);
  EXPECT_EQ(vertex_orders2[0], tr1);
  EXPECT_THROW((void)simulator.get_vertex_orders_of_vertex(1000),
               cda_rail::exceptions::VertexNotExistentException);
  EXPECT_THROW((void)simulator.set_vertex_orders_of_vertex(1000, {tr1}),
               cda_rail::exceptions::VertexNotExistentException);
  EXPECT_THROW((void)simulator.set_vertex_orders_of_vertex(l0, {tr1, tr1}),
               cda_rail::exceptions::InvalidInputException);

  // Stop positions
  simulator.set_train_edges_of_tr(
      tr1, {l0_l1, l1_l2, l2_l3, l3_g00, g00_g01, g01_r2, r2_r1});

  EXPECT_THROW((void)simulator.set_stop_positions({{}}),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW((void)simulator.set_stop_positions({{100}, {}}),
               cda_rail::exceptions::InvalidInputException);
  simulator.set_stop_positions({{1200}, {}});
  const auto& stop_positions1 = simulator.get_stop_positions();
  EXPECT_EQ(stop_positions1.size(), 2);
  EXPECT_EQ(stop_positions1[0].size(), 1);
  EXPECT_EQ(stop_positions1[0][0], 1200);
  EXPECT_TRUE(stop_positions1[1].empty());
  EXPECT_THROW((void)simulator.set_stop_positions_of_tr(1000, {1200}),
               cda_rail::exceptions::TrainNotExistentException);
  EXPECT_THROW(
      (void)simulator.set_stop_positions_of_tr(tr1, {1200, 1250, 1300}),
      cda_rail::exceptions::InvalidInputException);
  simulator.set_stop_positions_of_tr(tr1, {1250});
  const auto& stop_positions2 = simulator.get_stop_positions_of_tr(tr1);
  EXPECT_EQ(stop_positions2.size(), 1);
  EXPECT_EQ(stop_positions2[0], 1250);

  EXPECT_THROW((void)simulator.get_stop_positions_of_tr(1000),
               cda_rail::exceptions::TrainNotExistentException);

  simulator.set_stop_positions_of_tr(tr1, {});
  const auto& stop_positions3 = simulator.get_stop_positions_of_tr(tr1);
  EXPECT_TRUE(stop_positions3.empty());
  const auto& stop_positions4 = simulator.get_stop_positions_of_tr(tr2);
  EXPECT_TRUE(stop_positions4.empty());

  EXPECT_THROW((void)simulator.append_stop_position_to_tr(tr1, -100),
               cda_rail::exceptions::InvalidInputException);

  simulator.append_stop_position_to_tr(tr1, 1300);
  const auto& stop_positions5 = simulator.get_stop_positions_of_tr(tr1);
  EXPECT_EQ(stop_positions5.size(), 1);
  EXPECT_EQ(stop_positions5[0], 1300);
  const auto& stop_positions6 = simulator.get_stop_positions_of_tr(tr2);
  EXPECT_TRUE(stop_positions6.empty());

  EXPECT_THROW((void)simulator.append_stop_position_to_tr(1000, 500),
               cda_rail::exceptions::TrainNotExistentException);
  EXPECT_THROW((void)simulator.append_stop_position_to_tr(tr1, 200),
               cda_rail::exceptions::ConsistencyException);
  simulator.append_stop_position_to_tr(tr1, 1500);
  const auto& stop_positions7 = simulator.get_stop_positions_of_tr(tr1);
  EXPECT_EQ(stop_positions7.size(), 2);
  EXPECT_EQ(stop_positions7[0], 1300);
  EXPECT_EQ(stop_positions7[1], 1500);
  EXPECT_THROW((void)simulator.append_stop_position_to_tr(tr1, 1600),
               cda_rail::exceptions::ConsistencyException);

  EXPECT_THROW((void)simulator.append_stop_position_to_tr(tr2, 1600),
               cda_rail::exceptions::ConsistencyException);

  simulator.set_train_edges_of_tr(tr1, {});
  simulator.set_stop_positions_of_tr(tr1, {});
  EXPECT_THROW((void)simulator.append_current_stop_position_of_tr(tr1),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_THROW((void)simulator.append_current_stop_position_of_tr(1000),
               cda_rail::exceptions::TrainNotExistentException);
  simulator.append_train_edge_to_tr(tr1, l0_l1);
  EXPECT_THROW((void)simulator.append_current_stop_position_of_tr(tr1),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_THROW((void)simulator.append_stop_edge_to_tr(tr1, g00_g01),
               cda_rail::exceptions::ConsistencyException);
  simulator.set_train_edges_of_tr(
      tr1, {l0_l1, l1_l2, l2_l3, l3_g00, g00_g01, g01_r2, r2_r1});
  EXPECT_THROW((void)simulator.append_current_stop_position_of_tr(tr1),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_THROW((void)simulator.append_stop_edge_to_tr(1000, g00_g01),
               cda_rail::exceptions::TrainNotExistentException);
  simulator.append_stop_edge_to_tr(tr1, g00_g01);
  const auto& stop_positions8 = simulator.get_stop_positions_of_tr(tr1);
  EXPECT_EQ(stop_positions8.size(), 1);
  EXPECT_EQ(stop_positions8[0], 1310);
  simulator.append_current_stop_position_of_tr(tr1);
  const auto& stop_positions9 = simulator.get_stop_positions_of_tr(tr1);
  EXPECT_EQ(stop_positions9.size(), 2);
  EXPECT_EQ(stop_positions9[0], 1310);
  EXPECT_EQ(stop_positions9[1], 1320);
  EXPECT_THROW((void)simulator.append_current_stop_position_of_tr(tr1),
               cda_rail::exceptions::ConsistencyException);
}

TEST(GreedySimulator, ValidStopPos) {
  Network    network;
  const auto v4 = network.add_vertex("v4", cda_rail::VertexType::TTD);
  const auto v0 = network.add_vertex("v0", cda_rail::VertexType::TTD);
  const auto v2 = network.add_vertex("v2", cda_rail::VertexType::TTD);
  const auto v1 = network.add_vertex("v1", cda_rail::VertexType::TTD);
  const auto v3 = network.add_vertex("v3", cda_rail::VertexType::TTD);

  const auto v0_v1 = network.add_edge(v0, v1, 50, 10);
  const auto v3_v4 = network.add_edge(v3, v4, 75, 10);
  const auto v2_v3 = network.add_edge(v2, v3, 50, 10);
  const auto v1_v2 = network.add_edge(v1, v2, 30, 10);

  network.add_successor(v0_v1, v1_v2);
  network.add_successor(v1_v2, v2_v3);
  network.add_successor(v2_v3, v3_v4);

  Timetable  timetable;
  const auto tr2 = timetable.add_train("Train2", 50, 10, 1, 1, true, 0, 0, v0,
                                       360, 0, v4, network);
  const auto tr1 = timetable.add_train("Train1", 75, 10, 1, 1, true, 0, 0, v0,
                                       360, 0, v4, network);

  timetable.add_empty_station("Station1");
  timetable.add_track_to_station("Station1", v3_v4, network);
  timetable.add_track_to_station("Station1", v2_v3, network);
  timetable.add_empty_station("Station2");
  timetable.add_track_to_station("Station2", v0_v1, network);
  timetable.add_track_to_station("Station2", v1_v2, network);

  timetable.insert_stop("Train1", "Station2", 60, 60);
  timetable.insert_stop("Train1", "Station1", 120, 60);
  timetable.insert_stop("Train2", "Station2", 120, 60);

  RouteMap                                                    routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);
  cda_rail::simulator::GreedySimulator simulator(instance, {});

  EXPECT_FALSE(simulator.is_current_pos_valid_stop_position(tr1));
  EXPECT_FALSE(simulator.is_current_pos_valid_stop_position(tr2));

  simulator.append_train_edge_to_tr(tr1, v0_v1);
  simulator.append_train_edge_to_tr(tr2, v0_v1);
  EXPECT_FALSE(simulator.is_current_pos_valid_stop_position(tr1));
  EXPECT_TRUE(simulator.is_current_pos_valid_stop_position(tr2));

  simulator.append_train_edge_to_tr(tr1, v1_v2);
  simulator.append_train_edge_to_tr(tr2, v1_v2);
  EXPECT_TRUE(simulator.is_current_pos_valid_stop_position(tr1));
  EXPECT_TRUE(simulator.is_current_pos_valid_stop_position(tr2));

  simulator.append_stop_edge_to_tr(tr1, v1_v2);
  simulator.append_stop_edge_to_tr(tr2, v0_v1);
  EXPECT_FALSE(simulator.is_current_pos_valid_stop_position(tr1));
  EXPECT_FALSE(simulator.is_current_pos_valid_stop_position(tr2));

  simulator.append_train_edge_to_tr(tr1, v2_v3);
  simulator.append_train_edge_to_tr(tr2, v2_v3);
  EXPECT_FALSE(simulator.is_current_pos_valid_stop_position(tr1));
  EXPECT_FALSE(simulator.is_current_pos_valid_stop_position(tr2));

  simulator.append_train_edge_to_tr(tr1, v3_v4);
  simulator.append_train_edge_to_tr(tr2, v3_v4);
  EXPECT_TRUE(simulator.is_current_pos_valid_stop_position(tr1));
  EXPECT_FALSE(simulator.is_current_pos_valid_stop_position(tr2));

  simulator.append_stop_edge_to_tr(tr1, v3_v4);
  EXPECT_FALSE(simulator.is_current_pos_valid_stop_position(tr1));
}

TEST(GreedySimulator, TrainsOnPath) {
  Network network;

  auto const v0  = network.add_vertex("v0", cda_rail::VertexType::TTD);
  auto const v1  = network.add_vertex("v1", cda_rail::VertexType::TTD);
  auto const v2  = network.add_vertex("v2", cda_rail::VertexType::TTD);
  auto const v3a = network.add_vertex("v3a", cda_rail::VertexType::TTD);
  auto const v3b = network.add_vertex("v3b", cda_rail::VertexType::TTD);
  auto const v4b = network.add_vertex("v4b", cda_rail::VertexType::TTD);

  auto const v0_v1   = network.add_edge(v0, v1, 50, 10);
  auto const v1_v2   = network.add_edge(v1, v2, 50, 10);
  auto const v2_v3a  = network.add_edge(v2, v3a, 50, 10);
  auto const v2_v3b  = network.add_edge(v2, v3b, 50, 10);
  auto const v3b_v4b = network.add_edge(v3b, v4b, 50, 10);

  auto const v1_v0  = network.add_edge(v1, v0, 50, 10);
  auto const v2_v1  = network.add_edge(v2, v1, 50, 10);
  auto const v3a_v2 = network.add_edge(v3a, v2, 50, 10);
  auto const v3b_v2 = network.add_edge(v3b, v2, 50, 10);
  // 3b->4b has no reverse edge

  network.add_successor(v0_v1, v1_v2);
  network.add_successor(v1_v2, v2_v3a);
  network.add_successor(v1_v2, v2_v3b);
  network.add_successor(v2_v3b, v3b_v4b);
  network.add_successor(v3a_v2, v2_v1);
  network.add_successor(v3b_v2, v2_v1);
  network.add_successor(v2_v1, v1_v0);

  Timetable  timetable;
  const auto tr1a = timetable.add_train("Train1a", 50, 10, 1, 1, true, 0, 0, v0,
                                        360, 0, v3a, network);
  const auto tr1b = timetable.add_train("Train1b", 50, 10, 1, 1, true, 0, 0, v0,
                                        360, 0, v3a, network);
  const auto tr2  = timetable.add_train("Train2", 50, 10, 1, 1, true, 0, 0, v0,
                                        360, 0, v4b, network);
  const auto tr3a = timetable.add_train("Train3a", 50, 10, 1, 1, true, 0, 0,
                                        v3a, 360, 0, v0, network);
  const auto tr3b = timetable.add_train("Train3b", 50, 10, 1, 1, true, 0, 0,
                                        v3a, 360, 0, v0, network);

  RouteMap                                                    routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);

  cda_rail::simulator::GreedySimulator simulator(instance, {});
  simulator.set_train_edges_of_tr(tr1a, {v0_v1, v1_v2, v2_v3a});
  simulator.set_train_edges_of_tr(tr1b, {v0_v1, v1_v2});
  simulator.set_train_edges_of_tr(tr2, {v0_v1, v1_v2, v2_v3b, v3b_v4b});
  simulator.set_train_edges_of_tr(tr3a, {v3a_v2, v2_v1, v1_v0});
  simulator.set_train_edges_of_tr(tr3b, {v3a_v2, v2_v1});

  EXPECT_EQ(simulator.trains_on_path({v0_v1, v1_v2, v2_v3a}, false),
            cda_rail::index_set({tr1a}));
  EXPECT_EQ(simulator.trains_on_path({v0_v1, v1_v2, v2_v3a}, true),
            cda_rail::index_set({tr1a, tr3a}));

  EXPECT_EQ(simulator.trains_on_path({v0_v1, v1_v2, v2_v3b, v3b_v4b}, false),
            cda_rail::index_set({tr2}));
  EXPECT_EQ(simulator.trains_on_path({v0_v1, v1_v2, v2_v3b, v3b_v4b}, true),
            cda_rail::index_set({tr2}));

  EXPECT_EQ(simulator.trains_on_path({v0_v1, v1_v2}, false),
            cda_rail::index_set({tr1a, tr1b, tr2}));
  EXPECT_EQ(simulator.trains_on_path({v0_v1, v1_v2}, true),
            cda_rail::index_set({tr1a, tr1b, tr2, tr3a}));

  EXPECT_EQ(simulator.trains_on_path({v1_v2, v2_v3a}, false),
            cda_rail::index_set({tr1a}));
  EXPECT_EQ(simulator.trains_on_path({v1_v2, v2_v3a}, true),
            cda_rail::index_set({tr1a, tr3a, tr3b}));

  EXPECT_EQ(simulator.trains_on_path({v1_v2}, false),
            cda_rail::index_set({tr1a, tr1b, tr2}));
  EXPECT_EQ(simulator.trains_on_path({v1_v2}, true),
            cda_rail::index_set({tr1a, tr1b, tr2, tr3a, tr3b}));

  EXPECT_EQ(simulator.trains_on_path({}, false), cda_rail::index_set({}));
  EXPECT_EQ(simulator.trains_on_path({}, true), cda_rail::index_set({}));

  std::vector<cda_rail::index_vector> tr_edges_patched(
      instance.get_const_train_list().get_number_of_trains());
  tr_edges_patched.at(tr1a) = {v0_v1};
  tr_edges_patched.at(tr1b) = {v0_v1, v1_v2};

  EXPECT_EQ(simulator.trains_on_path({v0_v1}, tr_edges_patched, false),
            cda_rail::index_set({tr1a, tr1b}));
  EXPECT_EQ(simulator.trains_on_path({v0_v1}, tr_edges_patched, true),
            cda_rail::index_set({tr1a, tr1b}));
  EXPECT_EQ(simulator.trains_on_path({v0_v1}, tr_edges_patched, network, false),
            cda_rail::index_set({tr1a, tr1b}));
  EXPECT_EQ(simulator.trains_on_path({v0_v1}, tr_edges_patched, network, true),
            cda_rail::index_set({tr1a, tr1b}));
}

// ---------------------------
// Test private functions
// ---------------------------

TEST(GreedySimulator, BasicPrivateFunctions) {
  // Create instance
  Network     network("SimpleStation", "./data/");
  const auto& ttd_sections = network.unbreakable_sections();
  const auto& l0           = network.get_vertex_index("l0");
  const auto& r0           = network.get_vertex_index("r0");

  const auto& l0_l1   = network.get_edge_index({"l0", "l1"});
  const auto& l1_l2   = network.get_edge_index({"l1", "l2"});
  const auto& l2_l3   = network.get_edge_index({"l2", "l3"});
  const auto& l3_g00  = network.get_edge_index({"l3", "g00"});
  const auto& g00_g01 = network.get_edge_index({"g00", "g01"});
  const auto& g01_r2  = network.get_edge_index({"g01", "r2"});
  const auto& r2_r1   = network.get_edge_index({"r2", "r1"});
  const auto& r1_r0   = network.get_edge_index({"r1", "r0"});
  const auto& r0_r1   = network.get_edge_index({"r0", "r1"});

  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 100, 10, 1, 2, true, 0, 0,
                                       {"l0"}, 360, 10, {"r0"}, network);
  const auto tr2 = timetable.add_train("Train2", 100, 10, 1, 3, false, 30, 10,
                                       {"r0"}, 400, 5, {"l0"}, network);
  const auto tr3 = timetable.add_train("Train3", 100, 10, 1, 4, true, 0, 0,
                                       {"l0"}, 360, 10, {"r0"}, network);
  const auto tr4 = timetable.add_train("Train4", 100, 10, 1, 5, false, 30, 0,
                                       {"l0"}, 400, 10, {"r0"}, network);
  const auto tr5 = timetable.add_train("Train5", 100, 10, 1, 6, true, 120, 0,
                                       {"l0"}, 360, 10, {"r0"}, network);
  EXPECT_TRUE(timetable.check_consistency(network));

  RouteMap routes;

  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);

  cda_rail::simulator::GreedySimulator simulator(instance, ttd_sections);

  simulator.append_train_edge_to_tr(tr1, l0_l1);
  simulator.append_train_edge_to_tr(tr1, l1_l2);
  simulator.append_train_edge_to_tr(tr1, l2_l3);
  simulator.append_train_edge_to_tr(tr1, l3_g00);
  simulator.append_train_edge_to_tr(tr1, g00_g01);
  simulator.append_train_edge_to_tr(tr1, g01_r2);
  simulator.append_train_edge_to_tr(tr1, r2_r1);
  simulator.append_train_edge_to_tr(tr1, r1_r0);
  simulator.append_train_edge_to_tr(tr2, r0_r1);
  simulator.append_train_edge_to_tr(tr3, l0_l1);
  simulator.append_train_edge_to_tr(tr3, l1_l2);
  simulator.append_train_edge_to_tr(tr5, l0_l1);

  EXPECT_EQ(simulator.train_edge_length(tr1), 1820);
  EXPECT_EQ(simulator.train_edge_length(tr2), 500);
  EXPECT_EQ(simulator.train_edge_length(tr3), 1000);
  EXPECT_EQ(simulator.train_edge_length(tr4), 0);
  EXPECT_EQ(simulator.train_edge_length(tr5), 500);

  simulator.set_vertex_orders_of_vertex(r0, {tr1, tr2});
  simulator.set_vertex_orders_of_vertex(l0, {tr1, tr3, tr5});

  // Braking distance
  EXPECT_EQ(simulator.tr_braking_distance(tr1, 0), 0.0);
  EXPECT_EQ(simulator.tr_braking_distance(tr1, -EPS / 2), 0.0);
  EXPECT_EQ(simulator.tr_braking_distance(tr1, 1), 1.0 / 4.0);
  EXPECT_EQ(simulator.tr_braking_distance(tr1, 2), 1.0);
  EXPECT_EQ(simulator.tr_braking_distance(tr1, 3), 9.0 / 4.0);
  EXPECT_EQ(simulator.tr_braking_distance(tr2, 0), 0.0);
  EXPECT_EQ(simulator.tr_braking_distance(tr2, 1), 1.0 / 6.0);
  EXPECT_EQ(simulator.tr_braking_distance(tr2, 2), 2.0 / 3.0);
  EXPECT_EQ(simulator.tr_braking_distance(tr2, 3), 3.0 / 2.0);
  EXPECT_THROW((void)simulator.tr_braking_distance(1000, 1),
               cda_rail::exceptions::TrainNotExistentException);
  EXPECT_THROW((void)simulator.tr_braking_distance(tr1, -1),
               cda_rail::exceptions::InvalidInputException);

  // Trains entering
  const auto& [success_0, entering_tr_0] =
      simulator.get_entering_trains(0, {}, {}, {}, false, 60);
  EXPECT_TRUE(success_0);
  // Expect only tr1
  EXPECT_EQ(entering_tr_0.size(), 1);
  EXPECT_TRUE(entering_tr_0.contains(tr1));

  const auto& [success_30, entering_tr_30] =
      simulator.get_entering_trains(30, {}, {}, {}, false, 60);
  EXPECT_TRUE(success_30);
  // Expect only tr1
  EXPECT_EQ(entering_tr_30.size(), 1);
  EXPECT_TRUE(entering_tr_30.contains(tr1));

  const auto& [success_30b, entering_tr_30b] =
      simulator.get_entering_trains(30, {}, {tr1}, {}, false, 60);
  EXPECT_TRUE(success_30b);
  // Expect only tr2, tr3
  EXPECT_EQ(entering_tr_30b.size(), 2);
  EXPECT_TRUE(entering_tr_30b.contains(tr2));
  EXPECT_TRUE(entering_tr_30b.contains(tr3));

  const auto& [success_30c, entering_tr_30c] =
      simulator.get_entering_trains(30, {tr1}, {}, {}, false, 60);
  EXPECT_TRUE(success_30c);
  // Expect only tr3
  EXPECT_EQ(entering_tr_30c.size(), 1);
  EXPECT_TRUE(entering_tr_30c.contains(tr3));

  const auto& [success_60, entering_tr_60] =
      simulator.get_entering_trains(60, {}, {}, {}, false, 60);
  EXPECT_TRUE(success_60);
  // Expect tr1
  EXPECT_EQ(entering_tr_60.size(), 1);
  EXPECT_TRUE(entering_tr_60.contains(tr1));

  const auto& [success_61, entering_tr_61] =
      simulator.get_entering_trains(61, {}, {}, {}, false, 60);
  EXPECT_FALSE(success_61); // tr1 too late
  EXPECT_EQ(entering_tr_61.size(), 1);
  EXPECT_TRUE(entering_tr_61.contains(tr1));

  const auto& [success_61_t, entering_tr_61_t] =
      simulator.get_entering_trains(61, {}, {}, {}, true, 60);
  EXPECT_TRUE(success_61_t); // tr1 still entering
  EXPECT_EQ(entering_tr_61_t.size(), 1);
  EXPECT_TRUE(entering_tr_61_t.contains(tr1));

  const auto& [success_30_tr1tr2, entering_tr_30_tr1tr2] =
      simulator.get_entering_trains(30, {tr1, tr2}, {}, {}, false, 60);
  EXPECT_TRUE(success_30_tr1tr2);
  // Expect tr3
  EXPECT_EQ(entering_tr_30_tr1tr2.size(), 1);
  EXPECT_TRUE(entering_tr_30_tr1tr2.contains(tr3));

  const auto& [success_30_tr1tr2_l, entering_tr_30_tr1tr2_l] =
      simulator.get_entering_trains(30, {tr2}, {tr1}, {}, false, 60);
  EXPECT_TRUE(success_30_tr1tr2_l);
  // Expect tr3
  EXPECT_EQ(entering_tr_30_tr1tr2_l.size(), 1);
  EXPECT_TRUE(entering_tr_30_tr1tr2_l.contains(tr3));

  const auto& [success_60_tr1tr3, entering_tr_60_tr1tr3] =
      simulator.get_entering_trains(60, {tr3}, {tr1}, {}, false, 60);
  EXPECT_TRUE(success_60_tr1tr3);
  // Expect tr2
  EXPECT_EQ(entering_tr_60_tr1tr3.size(), 1);
  EXPECT_TRUE(entering_tr_60_tr1tr3.contains(tr2));

  const auto& [success_60_tr1tr2tr3, entering_tr_60_tr1tr2tr3] =
      simulator.get_entering_trains(60, {tr2}, {tr1, tr3}, {}, false, 60);
  EXPECT_TRUE(success_60_tr1tr2tr3);
  // Expect no train to enter
  EXPECT_TRUE(entering_tr_60_tr1tr2tr3.empty());

  const auto& [success_120_tr1tr2, entering_tr_120_tr1tr2] =
      simulator.get_entering_trains(60, {tr2}, {tr1}, {}, false, 60);
  EXPECT_TRUE(success_120_tr1tr2);
  // Expect tr3
  EXPECT_EQ(entering_tr_120_tr1tr2.size(), 1);
  EXPECT_TRUE(entering_tr_120_tr1tr2.contains(tr3));

  const auto& [success_120_tr1tr2tr3, entering_tr_120_tr1tr2tr3] =
      simulator.get_entering_trains(120, {tr2, tr3}, {tr1}, {}, false, 60);
  EXPECT_TRUE(success_120_tr1tr2tr3);
  // Expect tr5
  EXPECT_EQ(entering_tr_120_tr1tr2tr3.size(), 1);
  EXPECT_TRUE(entering_tr_120_tr1tr2tr3.contains(tr5));

  const auto& [success_120_tr1tr2tr3b, entering_tr_120_tr1tr2tr3b] =
      simulator.get_entering_trains(120, {tr2, tr3}, {}, {tr1}, false, 60);
  EXPECT_TRUE(success_120_tr1tr2tr3b);
  // Expect tr5
  EXPECT_EQ(entering_tr_120_tr1tr2tr3b.size(), 1);
  EXPECT_TRUE(entering_tr_120_tr1tr2tr3b.contains(tr5));

  // Milestones
  simulator.set_train_edges_of_tr(tr1, {l0_l1, l1_l2, l2_l3, l3_g00, g00_g01});
  simulator.set_train_edges_of_tr(tr2, {});
  simulator.set_train_edges_of_tr(tr3, {l0_l1});
  simulator.set_train_edges_of_tr(tr4, {});
  simulator.set_train_edges_of_tr(tr5, {});

  const auto& milestones_tr1 = simulator.edge_milestones(tr1);
  EXPECT_EQ(milestones_tr1.size(), 6);
  EXPECT_EQ(milestones_tr1[0], 0.0);
  EXPECT_EQ(milestones_tr1[1], 500.0);
  EXPECT_EQ(milestones_tr1[2], 1000.0);
  EXPECT_EQ(milestones_tr1[3], 1005.0);
  EXPECT_EQ(milestones_tr1[4], 1010.0);
  EXPECT_EQ(milestones_tr1[5], 1310.0);

  const auto& milestones_tr2 = simulator.edge_milestones(tr2);
  EXPECT_TRUE(milestones_tr2.empty()); // No edges for tr2

  const auto& milestones_tr3 = simulator.edge_milestones(tr3);
  EXPECT_EQ(milestones_tr3.size(), 2);
  EXPECT_EQ(milestones_tr3[0], 0.0);
  EXPECT_EQ(milestones_tr3[1], 500.0);

  EXPECT_THROW((void)simulator.edge_milestones(1000),
               cda_rail::exceptions::TrainNotExistentException);
}

TEST(GreedySimulator, TrainsOnEdges) {
  // Create instance
  Network     network("SimpleStation", "./data/");
  const auto& ttd_sections = network.unbreakable_sections();
  const auto& l0           = network.get_vertex_index("l0");
  const auto& r0           = network.get_vertex_index("r0");

  const auto& l0_l1   = network.get_edge_index({"l0", "l1"});
  const auto& l1_l2   = network.get_edge_index({"l1", "l2"});
  const auto& l2_l3   = network.get_edge_index({"l2", "l3"});
  const auto& l3_g00  = network.get_edge_index({"l3", "g00"});
  const auto& g00_g01 = network.get_edge_index({"g00", "g01"});

  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 100, 10, 1, 2, true, 0, 0,
                                       {"l0"}, 360, 10, {"r0"}, network);
  const auto tr2 = timetable.add_train("Train2", 100, 10, 1, 3, false, 30, 10,
                                       {"l0"}, 400, 5, {"r0"}, network);
  const auto tr3 = timetable.add_train("Train3", 100, 10, 1, 4, true, 0, 0,
                                       {"l0"}, 360, 10, {"r0"}, network);
  const auto tr4 = timetable.add_train("Train4", 100, 10, 1, 5, false, 30, 0,
                                       {"l0"}, 400, 10, {"r0"}, network);
  const auto tr5 = timetable.add_train("Train5", 100, 10, 1, 6, true, 120, 0,
                                       {"l0"}, 360, 10, {"r0"}, network);
  EXPECT_TRUE(timetable.check_consistency(network));

  RouteMap routes;

  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);

  cda_rail::simulator::GreedySimulator simulator(instance, ttd_sections);

  simulator.append_train_edge_to_tr(tr1, l0_l1);
  simulator.append_train_edge_to_tr(tr1, l1_l2);
  simulator.append_train_edge_to_tr(tr1, l2_l3);
  EXPECT_THROW(simulator.append_train_edge_to_tr(tr2, l1_l2),
               cda_rail::exceptions::ConsistencyException);
  simulator.append_train_edge_to_tr(tr2, l0_l1);
  simulator.append_train_edge_to_tr(tr3, l0_l1);
  simulator.append_train_edge_to_tr(tr3, l1_l2);
  simulator.append_train_edge_to_tr(tr4, l0_l1);

  const auto tr_on_edges = simulator.tr_on_edges();

  EXPECT_EQ(tr_on_edges.size(), network.number_of_edges());
  for (size_t i = 0; i < network.number_of_edges(); ++i) {
    if (i == l0_l1) {
      EXPECT_EQ(tr_on_edges.at(i).size(), 4);
      EXPECT_TRUE(tr_on_edges.at(i).contains(tr1));
      EXPECT_TRUE(tr_on_edges.at(i).contains(tr2));
      EXPECT_TRUE(tr_on_edges.at(i).contains(tr3));
      EXPECT_TRUE(tr_on_edges.at(i).contains(tr4));
    } else if (i == l1_l2) {
      EXPECT_EQ(tr_on_edges.at(i).size(), 2);
      EXPECT_TRUE(tr_on_edges.at(i).contains(tr1));
      EXPECT_TRUE(tr_on_edges.at(i).contains(tr3));
    } else if (i == l2_l3) {
      EXPECT_EQ(tr_on_edges.at(i).size(), 1);
      EXPECT_TRUE(tr_on_edges.at(i).contains(tr1));
    } else {
      EXPECT_TRUE(tr_on_edges.at(i).empty());
    }
  }
}

TEST(GreedySimulator, EdgePositions) {
  // Create instance
  Network     network("SimpleStation", "./data/");
  const auto& l0 = network.get_vertex_index("l0");
  const auto& l1 = network.get_vertex_index("l1");
  const auto& l2 = network.get_vertex_index("l2");
  const auto& r0 = network.get_vertex_index("r0");

  const auto& l0_l1   = network.get_edge_index({"l0", "l1"});
  const auto& l1_l2   = network.get_edge_index({"l1", "l2"});
  const auto& l2_l3   = network.get_edge_index({"l2", "l3"});
  const auto& l3_g00  = network.get_edge_index({"l3", "g00"});
  const auto& l3_g10  = network.get_edge_index({"l3", "g10"});
  const auto& g00_g01 = network.get_edge_index({"g00", "g01"});

  auto const& l0_l1_obj = network.get_edge(l0_l1);
  auto const& l1_l2_obj = network.get_edge(l1_l2);

  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 100, 10, 1, 2, true, 0, 0,
                                       {"l0"}, 360, 10, {"r0"}, network);
  const auto tr2 = timetable.add_train("Train2", 100, 10, 1, 3, false, 30, 10,
                                       {"r0"}, 400, 5, {"l0"}, network);
  const auto tr3 = timetable.add_train("Train3", 100, 10, 1, 4, true, 0, 0,
                                       {"l0"}, 360, 10, {"r0"}, network);
  const auto tr4 = timetable.add_train("Train4", 100, 10, 1, 5, false, 30, 0,
                                       {"l0"}, 400, 10, {"r0"}, network);
  const auto tr5 = timetable.add_train("Train5", 100, 10, 1, 6, true, 120, 0,
                                       {"l0"}, 360, 10, {"r0"}, network);
  EXPECT_TRUE(timetable.check_consistency(network));

  RouteMap routes;

  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);

  cda_rail::simulator::GreedySimulator simulator(
      instance, {{l0_l1, l1_l2}, {l2_l3, l3_g00, l3_g10}});

  // Test TTD sections
  EXPECT_EQ(simulator.get_ttd(l0_l1), 0);
  EXPECT_EQ(simulator.get_ttd(l1_l2), 0);
  EXPECT_EQ(simulator.get_ttd(l2_l3), 1);
  EXPECT_EQ(simulator.get_ttd(l3_g00), 1);
  EXPECT_EQ(simulator.get_ttd(l3_g10), 1);
  EXPECT_FALSE(simulator.get_ttd(g00_g01).has_value());
  EXPECT_THROW((void)simulator.get_ttd(1000),
               cda_rail::exceptions::EdgeNotExistentException);

  simulator.set_vertex_orders_of_vertex(r0, {tr2});
  simulator.set_vertex_orders_of_vertex(l0, {tr1, tr3, tr5});
  simulator.append_train_edge_to_tr(tr1, l0_l1);
  simulator.append_train_edge_to_tr(tr1, l1_l2);
  simulator.append_train_edge_to_tr(tr1, l2_l3);
  simulator.append_train_edge_to_tr(tr1, l3_g00);
  simulator.append_train_edge_to_tr(tr1, g00_g01);
  simulator.append_train_edge_to_tr(tr3, l0_l1);

  // Vertex positions
  auto const vertex_pos1 = simulator.get_vertex_pos(tr1, l0);
  EXPECT_TRUE(vertex_pos1.is_on_route);
  EXPECT_EQ(vertex_pos1.pos, 0);

  auto const vertex_pos2 = simulator.get_vertex_pos(tr1, l1);
  EXPECT_TRUE(vertex_pos2.is_on_route);
  EXPECT_EQ(vertex_pos2.pos, l0_l1_obj.length);

  auto const vertex_pos3 = simulator.get_vertex_pos(tr1, l2);
  EXPECT_TRUE(vertex_pos3.is_on_route);
  EXPECT_EQ(vertex_pos3.pos, l0_l1_obj.length + l1_l2_obj.length);

  auto const vertex_pos4 = simulator.get_vertex_pos(tr1, r0);
  EXPECT_FALSE(vertex_pos4.is_on_route);

  auto const vertex_pos31 = simulator.get_vertex_pos(tr3, l0);
  EXPECT_TRUE(vertex_pos31.is_on_route);
  EXPECT_EQ(vertex_pos31.pos, 0);

  auto const vertex_pos32 = simulator.get_vertex_pos(tr3, l1);
  EXPECT_TRUE(vertex_pos32.is_on_route);
  EXPECT_EQ(vertex_pos32.pos, l0_l1_obj.length);

  auto const vertex_pos33 = simulator.get_vertex_pos(tr3, l2);
  EXPECT_FALSE(vertex_pos33.is_on_route);

  auto const vertex_pos34 = simulator.get_vertex_pos(tr3, r0);
  EXPECT_FALSE(vertex_pos34.is_on_route);

  auto const vertex_pos21 = simulator.get_vertex_pos(tr2, l0);
  EXPECT_FALSE(vertex_pos21.is_on_route);

  // Edge position
  const auto& [occupation, pos] =
      simulator.get_position_on_edge(tr1, {-100, 0}, l0_l1);
  EXPECT_FALSE(occupation.tr_on_edge);
  EXPECT_FALSE(occupation.rear_on_edge);
  EXPECT_FALSE(occupation.front_on_edge);
  EXPECT_EQ(pos.rear, 0);
  EXPECT_EQ(pos.front, 0);

  const auto& [occupation2, pos2] =
      simulator.get_position_on_edge(tr1, {-50, 50}, l0_l1);
  EXPECT_TRUE(occupation2.tr_on_edge);
  EXPECT_FALSE(occupation2.rear_on_edge);
  EXPECT_TRUE(occupation2.front_on_edge);
  EXPECT_EQ(pos2.rear, 0);
  EXPECT_EQ(pos2.front, 50);

  const auto& [occupation3, pos3] =
      simulator.get_position_on_edge(tr1, {-50, 50}, l1_l2);
  EXPECT_FALSE(occupation3.tr_on_edge);
  EXPECT_FALSE(occupation3.rear_on_edge);
  EXPECT_FALSE(occupation3.front_on_edge);
  EXPECT_EQ(pos3.rear, 0);
  EXPECT_EQ(pos3.front, -450);

  const auto& [occupation4, pos4] =
      simulator.get_position_on_edge(tr1, {400, 500}, l0_l1);
  EXPECT_TRUE(occupation4.tr_on_edge);
  EXPECT_TRUE(occupation4.rear_on_edge);
  EXPECT_TRUE(occupation4.front_on_edge);
  EXPECT_EQ(pos4.rear, 400);
  EXPECT_EQ(pos4.front, 500);

  const auto& [occupation4b, pos4b] =
      simulator.get_position_on_edge(tr1, {500, 600}, l0_l1);
  EXPECT_FALSE(occupation4b.tr_on_edge);
  EXPECT_FALSE(occupation4b.rear_on_edge);
  EXPECT_FALSE(occupation4b.front_on_edge);
  EXPECT_EQ(pos4b.rear, 500);
  EXPECT_EQ(pos4b.front, 500);

  const auto& [occupation4c, pos4c] =
      simulator.get_position_on_edge(tr1, {500, 600}, l1_l2);
  EXPECT_TRUE(occupation4c.tr_on_edge);
  EXPECT_TRUE(occupation4c.rear_on_edge);
  EXPECT_TRUE(occupation4c.front_on_edge);
  EXPECT_EQ(pos4c.rear, 0);
  EXPECT_EQ(pos4c.front, 100);

  const auto& [occupation5, pos5] =
      simulator.get_position_on_edge(tr1, {920, 1020}, l0_l1);
  EXPECT_FALSE(occupation5.tr_on_edge);
  EXPECT_FALSE(occupation5.rear_on_edge);
  EXPECT_FALSE(occupation5.front_on_edge);
  EXPECT_EQ(pos5.rear, 920);
  EXPECT_EQ(pos5.front, 500);

  const auto& [occupation6, pos6] =
      simulator.get_position_on_edge(tr1, {920, 1020}, l1_l2);
  EXPECT_TRUE(occupation6.tr_on_edge);
  EXPECT_TRUE(occupation6.rear_on_edge);
  EXPECT_FALSE(occupation6.front_on_edge);
  EXPECT_EQ(pos6.rear, 420);
  EXPECT_EQ(pos6.front, 500);

  const auto& [occupation6b, pos6b] =
      simulator.get_position_on_edge(tr1, {1020, 1120}, l1_l2);
  EXPECT_FALSE(occupation6b.tr_on_edge);
  EXPECT_FALSE(occupation6b.rear_on_edge);
  EXPECT_FALSE(occupation6b.front_on_edge);
  EXPECT_EQ(pos6b.rear, 520);
  EXPECT_EQ(pos6b.front, 500);

  const auto& [occupation6c, pos6c] =
      simulator.get_position_on_edge(tr1, {950, 1020}, l1_l2);
  EXPECT_TRUE(occupation6c.tr_on_edge);
  EXPECT_TRUE(occupation6c.rear_on_edge);
  EXPECT_FALSE(occupation6c.front_on_edge);
  EXPECT_EQ(pos6c.rear, 450);
  EXPECT_EQ(pos6c.front, 500);

  const auto& [occupation7, pos7] =
      simulator.get_position_on_edge(tr1, {920, 1020}, l2_l3);
  EXPECT_TRUE(occupation7.tr_on_edge);
  EXPECT_FALSE(occupation7.rear_on_edge);
  EXPECT_FALSE(occupation7.front_on_edge);
  EXPECT_EQ(pos7.rear, 0);
  EXPECT_EQ(pos7.front, 5);

  const auto& [occupation8, pos8] =
      simulator.get_position_on_edge(tr1, {920, 1020}, l3_g00);
  EXPECT_TRUE(occupation8.tr_on_edge);
  EXPECT_FALSE(occupation8.rear_on_edge);
  EXPECT_FALSE(occupation8.front_on_edge);
  EXPECT_EQ(pos8.rear, 0);
  EXPECT_EQ(pos8.front, 5);

  const auto& [occupation9, pos9] =
      simulator.get_position_on_edge(tr1, {920, 1020}, g00_g01);
  EXPECT_TRUE(occupation9.tr_on_edge);
  EXPECT_FALSE(occupation9.rear_on_edge);
  EXPECT_TRUE(occupation9.front_on_edge);
  EXPECT_EQ(pos9.rear, 0);
  EXPECT_EQ(pos9.front, 10);

  const auto& [occupation10, pos10] = simulator.get_position_on_edge(
      tr1, {0, 100}, l0_l1, {0, 10, 20, 30, 40, 50});
  EXPECT_TRUE(occupation10.tr_on_edge);
  EXPECT_TRUE(occupation10.rear_on_edge);
  EXPECT_FALSE(occupation10.front_on_edge);
  EXPECT_EQ(pos10.rear, 0);
  EXPECT_EQ(pos10.front, 10);

  EXPECT_THROW((void)simulator.get_position_on_edge(tr1, {0, 100}, l0_l1,
                                                    {0, 10, 20, 30, 40}),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_THROW((void)simulator.get_position_on_edge(
                   tr1, {0, 100}, l0_l1, {0, 10, 20, 30, 40, 50, 60}),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_THROW((void)simulator.get_position_on_edge(1000, {0, 100}, l0_l1),
               cda_rail::exceptions::TrainNotExistentException);
  EXPECT_THROW((void)simulator.get_position_on_edge(tr1, {0, 100}, 1000),
               cda_rail::exceptions::EdgeNotExistentException);
  EXPECT_THROW((void)simulator.get_position_on_edge(tr3, {0, 100}, l2_l3),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_THROW((void)simulator.get_position_on_route_edge(1000, {0, 100}, 0),
               cda_rail::exceptions::TrainNotExistentException);
  EXPECT_THROW((void)simulator.get_position_on_route_edge(tr1, {0, 100}, 5),
               cda_rail::exceptions::InvalidInputException);

  // Is on route
  EXPECT_TRUE(simulator.is_on_route(tr1, l3_g00));
  EXPECT_FALSE(simulator.is_on_route(tr1, l3_g10));
  EXPECT_THROW((void)simulator.is_on_route(1000, l3_g00),
               cda_rail::exceptions::TrainNotExistentException);
  EXPECT_THROW((void)simulator.is_on_route(tr1, 1000),
               cda_rail::exceptions::EdgeNotExistentException);

  // Is on TTD
  EXPECT_FALSE(simulator.is_on_ttd(tr1, 1, {900, 1000}));
  EXPECT_TRUE(simulator.is_on_ttd(tr1, 1, {901, 1001}));
  EXPECT_TRUE(simulator.is_on_ttd(tr1, 1, {905, 1005}));
  EXPECT_TRUE(simulator.is_on_ttd(tr1, 1, {906, 1006}));
  EXPECT_TRUE(simulator.is_on_ttd(tr1, 1, {910, 1010}));
  EXPECT_TRUE(simulator.is_on_ttd(tr1, 1, {1000, 1100}));
  EXPECT_TRUE(simulator.is_on_ttd(tr1, 1, {1009, 1109}));
  EXPECT_FALSE(simulator.is_on_ttd(tr1, 1, {1010, 1110}));
  EXPECT_THROW((void)simulator.is_on_ttd(1000, 1, {900, 1000}),
               cda_rail::exceptions::TrainNotExistentException);
  EXPECT_THROW((void)simulator.is_on_ttd(tr1, 2, {900, 1000}),
               cda_rail::exceptions::InvalidInputException);

  // Is behind TTD
  EXPECT_FALSE(simulator.is_behind_ttd(tr1, 1, {900, 1000}));
  EXPECT_FALSE(simulator.is_behind_ttd(tr1, 1, {901, 1001}));
  EXPECT_FALSE(simulator.is_behind_ttd(tr1, 1, {905, 1005}));
  EXPECT_FALSE(simulator.is_behind_ttd(tr1, 1, {906, 1006}));
  EXPECT_FALSE(simulator.is_behind_ttd(tr1, 1, {910, 1010}));
  EXPECT_FALSE(simulator.is_behind_ttd(tr1, 1, {1000, 1100}));
  EXPECT_FALSE(simulator.is_behind_ttd(tr1, 1, {1009, 1109}));
  EXPECT_TRUE(simulator.is_behind_ttd(tr1, 1, {1010, 1110}));
  EXPECT_TRUE(simulator.is_behind_ttd(tr1, 1, {1100, 1200}));
  EXPECT_THROW((void)simulator.is_behind_ttd(1000, 1, {900, 1000}),
               cda_rail::exceptions::TrainNotExistentException);
  EXPECT_THROW((void)simulator.is_behind_ttd(tr1, 2, {900, 1000}),
               cda_rail::exceptions::InvalidInputException);

  // Is on or behind TTD
  EXPECT_FALSE(simulator.is_on_or_behind_ttd(tr1, 1, {900, 1000}));
  EXPECT_TRUE(simulator.is_on_or_behind_ttd(tr1, 1, {901, 1001}));
  EXPECT_TRUE(simulator.is_on_or_behind_ttd(tr1, 1, {905, 1005}));
  EXPECT_TRUE(simulator.is_on_or_behind_ttd(tr1, 1, {906, 1006}));
  EXPECT_TRUE(simulator.is_on_or_behind_ttd(tr1, 1, {910, 1010}));
  EXPECT_TRUE(simulator.is_on_or_behind_ttd(tr1, 1, {1000, 1100}));
  EXPECT_TRUE(simulator.is_on_or_behind_ttd(tr1, 1, {1009, 1109}));
  EXPECT_TRUE(simulator.is_on_or_behind_ttd(tr1, 1, {1010, 1110}));
  EXPECT_TRUE(simulator.is_on_or_behind_ttd(tr1, 1, {1100, 1200}));
  EXPECT_THROW((void)simulator.is_on_or_behind_ttd(1000, 1, {900, 1000}),
               cda_rail::exceptions::TrainNotExistentException);
  EXPECT_THROW((void)simulator.is_on_or_behind_ttd(tr1, 2, {900, 1000}),
               cda_rail::exceptions::InvalidInputException);
}

TEST(GreedySimulator, IsOkToEnter) {
  Network network;
  network.add_vertex("v00", VertexType::TTD);
  network.add_vertex("v01", VertexType::TTD);
  network.add_vertex("v10", VertexType::TTD);
  network.add_vertex("v11", VertexType::TTD);
  network.add_vertex("v2", VertexType::NoBorder);
  network.add_vertex("v3", VertexType::TTD);
  network.add_vertex("v4", VertexType::TTD);

  const auto v2_v3   = network.add_edge({"v2"}, {"v3"}, 10, 55, false);
  const auto v11_v2  = network.add_edge({"v11"}, {"v2"}, 10, 30, false);
  const auto v10_v2  = network.add_edge({"v10"}, {"v2"}, 10, 55, false);
  const auto v01_v11 = network.add_edge({"v01"}, {"v11"}, 101, 30, true);
  const auto v3_v4   = network.add_edge({"v3"}, {"v4"}, 100, 55, true);
  const auto v00_v10 = network.add_edge({"v00"}, {"v10"}, 100, 55, true);

  network.add_successor(v00_v10, v10_v2);
  network.add_successor(v10_v2, v2_v3);
  network.add_successor(v2_v3, v3_v4);
  network.add_successor(v01_v11, v11_v2);
  network.add_successor(v11_v2, v2_v3);

  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 50, 55, 1, 1, true, 0, 15,
                                       {"v01"}, 360, 10, {"v4"}, network);
  const auto tr2 = timetable.add_train("Train2", 50, 55, 1, 2, true, 0, 20,
                                       {"v01"}, 360, 10, {"v4"}, network);
  const auto tr3 = timetable.add_train("Train3", 50, 55, 1, 3, true, 0, 25,
                                       {"v00"}, 360, 10, {"v4"}, network);
  const auto tr4 = timetable.add_train("Train4", 50, 55, 1, 1, true, 0, 15,
                                       {"v01"}, 360, 10, {"v4"}, network);
  const auto tr5 = timetable.add_train("Train5", 50, 55, 1, 3, true, 0, 30,
                                       {"v00"}, 360, 10, {"v4"}, network);
  const auto tr6 = timetable.add_train("Train6", 50, 55, 1, 2, true, 0, 20,
                                       {"v00"}, 360, 10, {"v4"}, network);

  RouteMap routes;

  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);

  cda_rail::simulator::GreedySimulator simulator(instance,
                                                 {{v10_v2, v11_v2, v2_v3}});

  simulator.set_train_edges_of_tr(tr1, {v01_v11, v11_v2, v2_v3, v3_v4});
  simulator.set_train_edges_of_tr(tr2, {v01_v11, v11_v2, v2_v3, v3_v4});
  simulator.set_train_edges_of_tr(tr3, {v00_v10, v10_v2, v2_v3, v3_v4});
  simulator.set_train_edges_of_tr(tr4, {v01_v11, v11_v2, v2_v3, v3_v4});
  simulator.set_train_edges_of_tr(tr5, {v00_v10, v10_v2, v2_v3, v3_v4});
  simulator.set_train_edges_of_tr(tr6, {v00_v10, v10_v2, v2_v3, v3_v4});

  simulator.set_ttd_orders_of_ttd(0, {tr1, tr2, tr3, tr4, tr5, tr6});

  const auto tr_on_edges = simulator.tr_on_edges();

  // tr1: v01 with 15*15/2 = 112.5m braking distance
  // tr2: v01 with 20*20/4 = 100m braking distance
  // tr3: v00 with 25*25/6 = 104.1667m braking distance
  // tr4: v01 with 15*15/2 = 112.5m braking distance
  // tr5: v00 with 30*30/6 = 150m braking distance
  // tr6: v00 with 20*20/4 = 100m braking distance

  std::vector<cda_rail::simulator::GreedySimulator::TrainPosition> train_pos = {
      {-1, -1}, // tr1
      {-1, -1}, // tr2
      {-1, -1}, // tr3
      {-1, -1}, // tr4
      {-1, -1}, // tr5
      {-1, -1}  // tr6
  };
  const std::vector<double> train_velocities(6, 0);

  EXPECT_TRUE(simulator.is_ok_to_enter(tr1, train_pos, train_velocities, {},
                                       tr_on_edges));

  EXPECT_TRUE(simulator.is_ok_to_enter(tr2, train_pos, train_velocities, {},
                                       tr_on_edges));
  train_pos[tr1] = {50, 100};
  EXPECT_FALSE(simulator.is_ok_to_enter(tr2, train_pos, train_velocities, {tr1},
                                        tr_on_edges));
  train_pos[tr1] = {100.1, 150.1};
  EXPECT_TRUE(simulator.is_ok_to_enter(tr2, train_pos, train_velocities, {tr1},
                                       tr_on_edges));
  train_pos[tr1] = {200, 250};
  EXPECT_TRUE(simulator.is_ok_to_enter(tr2, train_pos, train_velocities, {tr1},
                                       tr_on_edges));

  EXPECT_FALSE(simulator.is_ok_to_enter(tr3, train_pos, train_velocities, {tr1},
                                        tr_on_edges));
  train_pos[tr2] = {40, 90};
  EXPECT_FALSE(simulator.is_ok_to_enter(tr3, train_pos, train_velocities,
                                        {tr1, tr2}, tr_on_edges));
  train_pos[tr2] = {55, 105};
  EXPECT_FALSE(simulator.is_ok_to_enter(tr3, train_pos, train_velocities,
                                        {tr1, tr2}, tr_on_edges));
  train_pos[tr2] = {100, 150};
  EXPECT_FALSE(simulator.is_ok_to_enter(tr3, train_pos, train_velocities,
                                        {tr1, tr2}, tr_on_edges));
  train_pos[tr2] = {112, 162};
  EXPECT_FALSE(simulator.is_ok_to_enter(tr3, train_pos, train_velocities,
                                        {tr1, tr2}, tr_on_edges));
  train_pos[tr2] = {120, 170};
  EXPECT_FALSE(simulator.is_ok_to_enter(tr3, train_pos, train_velocities,
                                        {tr1, tr2}, tr_on_edges));
  train_pos[tr2] = {121, 171};
  EXPECT_TRUE(simulator.is_ok_to_enter(tr3, train_pos, train_velocities,
                                       {tr1, tr2}, tr_on_edges));
  train_pos[tr1] = {220, 270};
  train_pos[tr2] = {200, 250};
  EXPECT_TRUE(simulator.is_ok_to_enter(tr3, train_pos, train_velocities, {tr2},
                                       tr_on_edges));

  EXPECT_FALSE(simulator.is_ok_to_enter(tr4, train_pos, train_velocities, {tr2},
                                        tr_on_edges));
  train_pos[tr3] = {40, 90};
  EXPECT_FALSE(simulator.is_ok_to_enter(tr4, train_pos, train_velocities,
                                        {tr2, tr3}, tr_on_edges));
  train_pos[tr3] = {55, 105};
  EXPECT_FALSE(simulator.is_ok_to_enter(tr4, train_pos, train_velocities,
                                        {tr2, tr3}, tr_on_edges));
  train_pos[tr3] = {100, 150};
  EXPECT_FALSE(simulator.is_ok_to_enter(tr4, train_pos, train_velocities,
                                        {tr2, tr3}, tr_on_edges));
  train_pos[tr3] = {112, 162};
  EXPECT_FALSE(simulator.is_ok_to_enter(tr4, train_pos, train_velocities,
                                        {tr2, tr3}, tr_on_edges));
  train_pos[tr3] = {120, 170};
  EXPECT_TRUE(simulator.is_ok_to_enter(tr4, train_pos, train_velocities,
                                       {tr2, tr3}, tr_on_edges));
  train_pos[tr2] = {220, 270};
  train_pos[tr3] = {200, 250};
  EXPECT_TRUE(simulator.is_ok_to_enter(tr4, train_pos, train_velocities, {tr3},
                                       tr_on_edges));

  EXPECT_FALSE(simulator.is_ok_to_enter(tr5, train_pos, train_velocities, {tr3},
                                        tr_on_edges));
  train_pos[tr4] = {40, 90};
  EXPECT_FALSE(simulator.is_ok_to_enter(tr5, train_pos, train_velocities,
                                        {tr3, tr4}, tr_on_edges));
  train_pos[tr4] = {55, 105};
  EXPECT_FALSE(simulator.is_ok_to_enter(tr5, train_pos, train_velocities,
                                        {tr3, tr4}, tr_on_edges));
  train_pos[tr4] = {100, 150};
  EXPECT_FALSE(simulator.is_ok_to_enter(tr5, train_pos, train_velocities,
                                        {tr3, tr4}, tr_on_edges));
  train_pos[tr4] = {101, 151};
  EXPECT_FALSE(simulator.is_ok_to_enter(tr5, train_pos, train_velocities,
                                        {tr3, tr4}, tr_on_edges));
  train_pos[tr4] = {121, 171};
  EXPECT_FALSE(simulator.is_ok_to_enter(tr5, train_pos, train_velocities,
                                        {tr3, tr4}, tr_on_edges));
  train_pos[tr4] = {150.1, 200.1};
  EXPECT_FALSE(simulator.is_ok_to_enter(tr5, train_pos, train_velocities,
                                        {tr3, tr4}, tr_on_edges));
  train_pos[tr4] = {151.1, 201.1};
  EXPECT_TRUE(simulator.is_ok_to_enter(tr5, train_pos, train_velocities,
                                       {tr3, tr4}, tr_on_edges));
  train_pos[tr3] = {220, 270};
  train_pos[tr4] = {200, 250};
  EXPECT_TRUE(simulator.is_ok_to_enter(tr5, train_pos, train_velocities, {tr4},
                                       tr_on_edges));

  train_pos[tr5] = {99.9, 149.9};
  EXPECT_FALSE(simulator.is_ok_to_enter(tr6, train_pos, train_velocities,
                                        {tr4, tr5}, tr_on_edges));
  train_pos[tr5] = {100.1, 150.1};
  EXPECT_TRUE(simulator.is_ok_to_enter(tr6, train_pos, train_velocities,
                                       {tr4, tr5}, tr_on_edges));
}

TEST(GreedySimulator, AbsoluteDistanceMA) {
  Network network;
  network.add_vertex("v00", VertexType::TTD);
  network.add_vertex("v01", VertexType::TTD);
  network.add_vertex("v10", VertexType::TTD);
  network.add_vertex("v11", VertexType::TTD);
  network.add_vertex("v2", VertexType::NoBorder);
  network.add_vertex("v3", VertexType::TTD);
  network.add_vertex("v4", VertexType::TTD);

  const auto v3_v4   = network.add_edge({"v3"}, {"v4"}, 100, 55, true);
  const auto v10_v2  = network.add_edge({"v10"}, {"v2"}, 10, 55, false);
  const auto v11_v2  = network.add_edge({"v11"}, {"v2"}, 10, 30, false);
  const auto v2_v3   = network.add_edge({"v2"}, {"v3"}, 10, 55, false);
  const auto v00_v10 = network.add_edge({"v00"}, {"v10"}, 100, 55, true);
  const auto v01_v11 = network.add_edge({"v01"}, {"v11"}, 101, 30, true);

  network.add_successor(v00_v10, v10_v2);
  network.add_successor(v10_v2, v2_v3);
  network.add_successor(v2_v3, v3_v4);
  network.add_successor(v01_v11, v11_v2);
  network.add_successor(v11_v2, v2_v3);

  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 50, 55, 1, 1, true, 0, 15,
                                       {"v01"}, 360, 10, {"v4"}, network);
  const auto tr2 = timetable.add_train("Train2", 50, 55, 1, 2, true, 0, 20,
                                       {"v00"}, 360, 10, {"v4"}, network);
  const auto tr3 = timetable.add_train("Train3", 50, 55, 1, 3, true, 0, 25,
                                       {"v00"}, 360, 10, {"v4"}, network);

  RouteMap routes;

  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);

  cda_rail::simulator::GreedySimulator simulator(instance,
                                                 {{v10_v2, v11_v2, v2_v3}});

  simulator.set_train_edges_of_tr(tr1, {v01_v11, v11_v2, v2_v3, v3_v4});
  simulator.set_train_edges_of_tr(tr2, {v00_v10, v10_v2, v2_v3, v3_v4});
  simulator.set_train_edges_of_tr(tr3, {v00_v10, v10_v2, v2_v3, v3_v4});

  simulator.set_ttd_orders_of_ttd(0, {tr1, tr2, tr3});

  const auto tr_on_edges = simulator.tr_on_edges();

  std::vector<cda_rail::simulator::GreedySimulator::TrainPosition> train_pos = {
      {-50, 0}, // tr1
      {-50, 0}, // tr2
      {-50, 0}, // tr3
  };
  std::vector<double> train_velocities(3, 0);

  EXPECT_EQ(simulator.get_absolute_distance_ma(tr1, 200, train_pos,
                                               train_velocities, {tr1}, {},
                                               tr_on_edges, true),
            200);
  train_pos[tr1] = {40, 90};
  EXPECT_EQ(simulator.get_absolute_distance_ma(tr1, 200, train_pos,
                                               train_velocities, {tr1}, {},
                                               tr_on_edges, true),
            200);
  EXPECT_EQ(simulator.get_absolute_distance_ma(tr2, 200, train_pos,
                                               train_velocities, {tr1, tr2}, {},
                                               tr_on_edges, true),
            100);
  EXPECT_EQ(simulator.get_absolute_distance_ma(tr2, 200, train_pos,
                                               train_velocities, {tr2}, {tr1},
                                               tr_on_edges, true),
            200);
  train_pos[tr2] = {0, 50};
  EXPECT_EQ(simulator.get_absolute_distance_ma(tr2, 200, train_pos,
                                               train_velocities, {tr1, tr2}, {},
                                               tr_on_edges, true),
            50);
  train_pos[tr1] = {52, 102};
  EXPECT_EQ(simulator.get_absolute_distance_ma(tr2, 200, train_pos,
                                               train_velocities, {tr1, tr2}, {},
                                               tr_on_edges, true),
            50);
  train_pos[tr1] = {90, 140};
  EXPECT_EQ(simulator.get_absolute_distance_ma(tr2, 200, train_pos,
                                               train_velocities, {tr1, tr2}, {},
                                               tr_on_edges, true),
            50);
  train_pos[tr1] = {102, 152};
  EXPECT_EQ(simulator.get_absolute_distance_ma(tr2, 200, train_pos,
                                               train_velocities, {tr1, tr2}, {},
                                               tr_on_edges, true),
            50);
  train_pos[tr1] = {112, 162};
  EXPECT_EQ(simulator.get_absolute_distance_ma(tr2, 200, train_pos,
                                               train_velocities, {tr1, tr2}, {},
                                               tr_on_edges, true),
            50);
  train_pos[tr1] = {120, 170};
  EXPECT_EQ(simulator.get_absolute_distance_ma(tr2, 200, train_pos,
                                               train_velocities, {tr1, tr2}, {},
                                               tr_on_edges, true),
            50);
  train_pos[tr1] = {120, 200};
  EXPECT_EQ(simulator.get_absolute_distance_ma(tr2, 200, train_pos,
                                               train_velocities, {tr1, tr2}, {},
                                               tr_on_edges, true),
            50);
  train_pos[tr1] = {121, 200};
  EXPECT_EQ(simulator.get_absolute_distance_ma(tr2, 200, train_pos,
                                               train_velocities, {tr1, tr2}, {},
                                               tr_on_edges, true),
            70);
  train_pos[tr1] = {121.1, 200};
  EXPECT_EQ(simulator.get_absolute_distance_ma(tr2, 200, train_pos,
                                               train_velocities, {tr1, tr2}, {},
                                               tr_on_edges, true),
            70.1);
  train_pos[tr1] = {150, 200};
  EXPECT_EQ(simulator.get_absolute_distance_ma(tr2, 200, train_pos,
                                               train_velocities, {tr1, tr2}, {},
                                               tr_on_edges, true),
            99);
  EXPECT_EQ(simulator.get_absolute_distance_ma(tr2, 98, train_pos,
                                               train_velocities, {tr1, tr2}, {},
                                               tr_on_edges, true),
            98);
  train_pos[tr1] = {200, 250};
  EXPECT_EQ(simulator.get_absolute_distance_ma(tr3, 200, train_pos,
                                               train_velocities, {tr1, tr3}, {},
                                               tr_on_edges, true),
            100);
  train_pos[tr2] = {50, 100};
  EXPECT_EQ(simulator.get_absolute_distance_ma(
                tr3, 200, train_pos, train_velocities, {tr1, tr2, tr3}, {},
                tr_on_edges, true),
            50);
  train_pos[tr2] = {105, 155};
  EXPECT_EQ(simulator.get_absolute_distance_ma(
                tr3, 200, train_pos, train_velocities, {tr1, tr2, tr3}, {},
                tr_on_edges, true),
            100);
  EXPECT_EQ(simulator.get_absolute_distance_ma(
                tr3, 99, train_pos, train_velocities, {tr1, tr2, tr3}, {},
                tr_on_edges, true),
            99);
  train_pos[tr3] = {50, 100};
  EXPECT_EQ(simulator.get_absolute_distance_ma(
                tr3, 200, train_pos, train_velocities, {tr1, tr2, tr3}, {},
                tr_on_edges, true),
            0);
  train_pos[tr2] = {140, 190};
  EXPECT_EQ(simulator.get_absolute_distance_ma(
                tr3, 200, train_pos, train_velocities, {tr1, tr2, tr3}, {},
                tr_on_edges, true),
            40);

  train_pos[tr1] = {200, 250};
  train_pos[tr2] = {160, 195};
  train_pos[tr3] = {140, 150};
  EXPECT_EQ(simulator.get_absolute_distance_ma(
                tr1, 200, train_pos, train_velocities, {tr1, tr2, tr3}, {},
                tr_on_edges, true),
            200);
  EXPECT_EQ(simulator.get_absolute_distance_ma(
                tr2, 200, train_pos, train_velocities, {tr1, tr2, tr3}, {},
                tr_on_edges, true),
            4);
  EXPECT_EQ(simulator.get_absolute_distance_ma(
                tr3, 200, train_pos, train_velocities, {tr1, tr2, tr3}, {},
                tr_on_edges, true),
            10);

  EXPECT_THROW((void)simulator.get_absolute_distance_ma(
                   tr3, 200, train_pos, train_velocities, {tr1, tr2}, {},
                   tr_on_edges, true),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_THROW((void)simulator.get_absolute_distance_ma(
                   tr3, -1, train_pos, train_velocities, {tr1, tr2, tr3}, {},
                   tr_on_edges, true),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW((void)simulator.get_absolute_distance_ma(
                   1000, 200, train_pos, train_velocities, {tr1, tr2, 1000}, {},
                   tr_on_edges, true),
               cda_rail::exceptions::TrainNotExistentException);
}

TEST(GreedySimulator, FutureSpeedRestrictionConstraints) {
  Network    network;
  const auto v0 = network.add_vertex("v0", VertexType::TTD);
  const auto v1 = network.add_vertex("v1", VertexType::TTD);
  const auto v2 = network.add_vertex("v2", VertexType::TTD);
  const auto v3 = network.add_vertex("v3", VertexType::TTD);
  const auto v4 = network.add_vertex("v4", VertexType::TTD);
  const auto v5 = network.add_vertex("v5", VertexType::TTD);

  const auto v4_v5 = network.add_edge(v4, v5, 1500, 55, true);
  const auto v2_v3 = network.add_edge(v2, v3, 200, 20, true);
  const auto v0_v1 = network.add_edge(v0, v1, 100, 40, true);
  const auto v1_v2 = network.add_edge(v1, v2, 110, 30, true);
  const auto v3_v4 = network.add_edge(v3, v4, 100, 50, true);

  network.add_successor(v0_v1, v1_v2);
  network.add_successor(v1_v2, v2_v3);
  network.add_successor(v2_v3, v3_v4);
  network.add_successor(v3_v4, v4_v5);

  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 200, 51, 3, 2, true, 0, 10, v0,
                                       360, 14, v5, network);

  RouteMap routes;

  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);

  cda_rail::simulator::GreedySimulator simulator(instance, {{}});
  simulator.set_train_edges_of_tr(tr1, {v0_v1, v1_v2, v2_v3, v3_v4, v4_v5});

  const auto& train =
      simulator.get_instance()->get_const_train_list().get_train(tr1);

  const auto ma_data1 = simulator.get_future_max_speed_constraints(
      tr1, train, 0, 10, 500, 400, 10, {}, true);
  const auto ma1 = ma_data1.ma;
  const auto vm1 = ma_data1.max_v;
  EXPECT_EQ(ma1, 310);
  EXPECT_EQ(vm1, 30);
  const auto ma_data1tol = simulator.get_future_max_speed_constraints(
      tr1, train, -cda_rail::EPS / 2.0, 10, 500, 400, 10, {}, true);
  const auto ma1tol = ma_data1tol.ma;
  const auto vm1tol = ma_data1tol.max_v;
  EXPECT_EQ(ma1tol, 310);
  EXPECT_EQ(vm1tol, 30);
  const auto ma_data2 = simulator.get_future_max_speed_constraints(
      tr1, train, 0, 10, 500, 400, 2, {}, true);
  const auto ma2 = ma_data2.ma;
  const auto vm2 = ma_data2.max_v;
  EXPECT_EQ(ma2, 310);
  EXPECT_EQ(vm2, 16);
  const auto ma_data2_0 = simulator.get_future_max_speed_constraints(
      tr1, train, 0, 10, 0, 400, 2, {}, true);
  const auto ma2_0 = ma_data2_0.ma;
  const auto vm2_0 = ma_data2_0.max_v;
  EXPECT_EQ(ma2_0, 0);
  EXPECT_EQ(vm2_0, 16);
  const auto ma_data2_0tol = simulator.get_future_max_speed_constraints(
      tr1, train, 0, 10, -cda_rail::EPS / 2.0, 400, 2, {}, true);
  const auto ma2_0tol = ma_data2_0tol.ma;
  const auto vm2_0tol = ma_data2_0tol.max_v;
  EXPECT_EQ(ma2_0tol, 0);
  EXPECT_EQ(vm2_0tol, 16);
  const auto ma_data3 = simulator.get_future_max_speed_constraints(
      tr1, train, 0, 10, 200, 400, 10, {}, true);
  const auto ma3 = ma_data3.ma;
  const auto vm3 = ma_data3.max_v;
  EXPECT_EQ(ma3, 200);
  EXPECT_EQ(vm3, 30);
  const auto ma_data4 = simulator.get_future_max_speed_constraints(
      tr1, train, 0, 10, 100, 400, 10, {}, true);
  const auto ma4 = ma_data4.ma;
  const auto vm4 = ma_data4.max_v;
  EXPECT_EQ(ma4, 100);
  EXPECT_EQ(vm4, 40);
  const auto ma_data5 = simulator.get_future_max_speed_constraints(
      tr1, train, 0, 10, 50, 400, 10, {}, true);
  const auto ma5 = ma_data5.ma;
  const auto vm5 = ma_data5.max_v;
  EXPECT_EQ(ma5, 50);
  EXPECT_EQ(vm5, 40);

  const auto ma_data6 = simulator.get_future_max_speed_constraints(
      tr1, train, 50, 40, 600, 400, 10, {}, true);
  const auto ma6 = ma_data6.ma;
  const auto vm6 = ma_data6.max_v;
  EXPECT_EQ(ma6, 600);
  EXPECT_EQ(vm6, 20);
  const auto ma_data7 = simulator.get_future_max_speed_constraints(
      tr1, train, 50, 40, 1200, 400, 6, {}, true);
  const auto ma7 = ma_data7.ma;
  const auto vm7 = ma_data7.max_v;
  EXPECT_EQ(ma7, 985);
  EXPECT_EQ(vm7, 20);

  const auto ma_data8 = simulator.get_future_max_speed_constraints(
      tr1, train, 250, 19, 1000, 400, 1, {}, true);
  const auto ma8 = ma_data8.ma;
  const auto vm8 = ma_data8.max_v;
  EXPECT_EQ(ma8, 785);
  EXPECT_EQ(vm8, 20);
  const auto ma_data9 = simulator.get_future_max_speed_constraints(
      tr1, train, 250, 19, 1000, 400, 1, {}, false);
  const auto ma9 = ma_data9.ma;
  const auto vm9 = ma_data9.max_v;
  EXPECT_EQ(ma9, 785);
  EXPECT_EQ(vm9, 20);

  const auto ma_data10 = simulator.get_future_max_speed_constraints(
      tr1, train, 500, 19, 1000, 400, 1, {}, true);
  const auto ma10 = ma_data10.ma;
  const auto vm10 = ma_data10.max_v;
  EXPECT_EQ(ma10, 1000);
  EXPECT_EQ(vm10, 20);
  const auto ma_data11 = simulator.get_future_max_speed_constraints(
      tr1, train, 500, 19, 1000, 400, 1, {}, false);
  const auto ma11 = ma_data11.ma;
  const auto vm11 = ma_data11.max_v;
  EXPECT_EQ(ma11, 1000);
  EXPECT_EQ(vm11, 22);

  const auto ma_data12 = simulator.get_future_max_speed_constraints(
      tr1, train, 0, 0, 1000, 400, 1, {}, true);
  const auto ma12 = ma_data12.ma;
  const auto vm12 = ma_data12.max_v;
  EXPECT_EQ(ma12, 310);
  EXPECT_EQ(vm12, 3);
  const auto ma_data12tol = simulator.get_future_max_speed_constraints(
      tr1, train, 0, -cda_rail::EPS / 2.0, 1000, 400, 1, {}, true);
  const auto ma12tol = ma_data12tol.ma;
  const auto vm12tol = ma_data12tol.max_v;
  EXPECT_EQ(ma12tol, 310);
  EXPECT_EQ(vm12tol, 3);

  // Test exit speed limit at 2010
  // Linear movement 20 -> 14 over 10s: 170m
  // Braking distance: 14*14/4 = 49 to 2059
  // if-case: 2010 - 170 = 1840
  const auto ma_data13prev = simulator.get_future_max_speed_constraints(
      tr1, train, 1600, 20, 1000, 400, 10, {}, false);
  const auto ma13prev = ma_data13prev.ma;
  const auto vm13prev = ma_data13prev.max_v;
  EXPECT_EQ(ma13prev, 459);
  EXPECT_EQ(vm13prev, 50);

  const auto ma_data13 = simulator.get_future_max_speed_constraints(
      tr1, train, 1800, 20, 1000, 400, 10, {}, false);
  const auto ma13 = ma_data13.ma;
  const auto vm13 = ma_data13.max_v;
  EXPECT_EQ(ma13, 259);
  EXPECT_EQ(vm13, 50);

  const auto ma_data13b = simulator.get_future_max_speed_constraints(
      tr1, train, 1800, 20, 100, 400, 10, {}, false);
  const auto ma13b = ma_data13b.ma;
  const auto vm13b = ma_data13b.max_v;
  EXPECT_EQ(ma13b, 100);
  EXPECT_EQ(vm13b, 50);

  const auto ma_data14 = simulator.get_future_max_speed_constraints(
      tr1, train, 1840, 20, 1000, 400, 10, {}, false);
  const auto ma14 = ma_data14.ma;
  const auto vm14 = ma_data14.max_v;
  EXPECT_EQ(ma14, 1000);
  EXPECT_EQ(vm14, 14);

  const auto ma_data15 = simulator.get_future_max_speed_constraints(
      tr1, train, 1841, 20, 1000, 400, 10, {}, false);
  const auto ma15 = ma_data15.ma;
  const auto vm15 = ma_data15.max_v;
  EXPECT_EQ(ma15, 1000);
  EXPECT_EQ(vm15, 14);

  // Stopping on route edge after 510m (no stopping anymore)
  simulator.set_train_edges_of_tr(tr1, {v0_v1, v1_v2, v2_v3, v3_v4});
  const auto ma_data17 = simulator.get_future_max_speed_constraints(
      tr1, train, 400, 20, 1000, 400, 5, {}, false);
  const auto ma17 = ma_data17.ma;
  const auto vm17 = ma_data17.max_v;
  EXPECT_EQ(ma17, 1000);
  EXPECT_EQ(vm17, 20);

  const auto ma_data18 = simulator.get_future_max_speed_constraints(
      tr1, train, 400, 20, 1000, 400, 20, {}, true);
  const auto ma18 = ma_data18.ma;
  const auto vm18 = ma_data18.max_v;
  EXPECT_EQ(ma18, 1000);
  EXPECT_EQ(vm18, 20);

  EXPECT_THROW((void)simulator.get_future_max_speed_constraints(
                   tr1, train, -1, 10, 10, 400, 10, {}, true),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW((void)simulator.get_future_max_speed_constraints(
                   tr1, train, 10, -1, 10, 400, 10, {}, true),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW((void)simulator.get_future_max_speed_constraints(
                   tr1, train, 10, 10, -1, 400, 10, {}, true),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW((void)simulator.get_future_max_speed_constraints(
                   tr1, train, 10, 10, 10, 400, -1, {}, true),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW((void)simulator.get_future_max_speed_constraints(
                   1000, train, 10, 10, 10, 400, 10, {}, true),
               cda_rail::exceptions::TrainNotExistentException);
}

TEST(GreedySimulator, FutureMaxSpeedWithMiddleEdge) {
  Network    network;
  const auto v0 = network.add_vertex("v0", VertexType::TTD);
  const auto v1 = network.add_vertex("v1", VertexType::TTD);
  const auto v2 = network.add_vertex("v2", VertexType::TTD);
  const auto v3 = network.add_vertex("v3", VertexType::TTD);

  const auto v0_v1 = network.add_edge(v0, v1, 100, 40, true);
  const auto v1_v2 = network.add_edge(v1, v2, 50, 30, true);
  const auto v2_v3 = network.add_edge(v2, v3, 10000, 50, true);

  network.add_successor(v0_v1, v1_v2);
  network.add_successor(v1_v2, v2_v3);

  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 100, 60, 3, 2, true, 0, 10, v0,
                                       360, 14, v3, network);

  RouteMap routes;

  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);

  cda_rail::simulator::GreedySimulator simulator(instance, {{}});
  simulator.set_train_edges_of_tr(tr1, {v0_v1, v1_v2, v2_v3});

  const auto& train =
      simulator.get_instance()->get_const_train_list().get_train(tr1);

  const auto ma_data1 = simulator.get_future_max_speed_constraints(
      tr1, train, 160, 60, 500, 400, 10, {}, true);
  EXPECT_EQ(ma_data1.max_v, 30);

  const auto ma_data2 = simulator.get_future_max_speed_constraints(
      tr1, train, 160, 60, 500, 400, 10, {}, false);
  EXPECT_EQ(ma_data2.max_v, 50);
}

TEST(GreedySimulator, FutureMaxSpeedWithBlockedVertices) {
  Network    network;
  const auto v0 = network.add_vertex("v0", VertexType::TTD);
  const auto v1 = network.add_vertex("v1", VertexType::TTD);
  const auto v2 = network.add_vertex("v2", VertexType::TTD);
  const auto v3 = network.add_vertex("v3", VertexType::TTD);

  const auto v0_v1 = network.add_edge(v0, v1, 100, 30, true);
  const auto v1_v2 = network.add_edge(v1, v2, 100, 30, true);
  const auto v2_v3 = network.add_edge(v2, v3, 10000, 30, true);

  network.add_successor(v0_v1, v1_v2);
  network.add_successor(v1_v2, v2_v3);

  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 100, 30, 3, 2, true, 0, 10, v0,
                                       360, 14, v3, network);

  RouteMap routes;

  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);

  cda_rail::simulator::GreedySimulator simulator(instance, {{}});
  simulator.set_train_edges_of_tr(tr1, {v0_v1, v1_v2, v2_v3});

  const auto& train =
      simulator.get_instance()->get_const_train_list().get_train(tr1);

  const auto ma_data1 = simulator.get_future_max_speed_constraints(
      tr1, train, 160, 30, 500, 400, 10, {v1}, true);
  EXPECT_EQ(ma_data1.ma, 500);
  EXPECT_EQ(ma_data1.max_v, 30);

  const auto ma_data2 = simulator.get_future_max_speed_constraints(
      tr1, train, 160, 30, 500, 400, 10, {v1, v2}, true);
  EXPECT_EQ(ma_data2.ma, 40);
  EXPECT_EQ(ma_data2.max_v, 30);
}

TEST(GreedySimulator, EoMDisplacement) {
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance;
  cda_rail::simulator::GreedySimulator simulator(instance, {});

  cda_rail::Train train("Train", 100, 30, 4, 2);

  // After 3 seconds v_1 = 0 + 3*4 = 12
  // x_1 = (0+12)*3/2 = 18
  // bd = 12*12 / 4 = 36
  // x_1 + bd = 18 + 36 = 54
  EXPECT_EQ(cda_rail::max_braking_pos_after_dt_linear_movement(0, 30, 4, 2, 3),
            54);
  EXPECT_EQ(cda_rail::max_braking_pos_after_dt_linear_movement(
                -cda_rail::EPS / 2.0, 30, 4, 2, 3),
            54);
  EXPECT_EQ(simulator.max_displacement(train, 0, 3), 54);
  EXPECT_EQ(simulator.max_displacement(train, -cda_rail::EPS / 2.0, 3), 54);

  // After 3 seconds v_1 = 10 + 3*4 = 22
  // x_1 = (10+22)*3/2 = 48
  // bd = 22*22 / 4 = 121
  // x_1 + bd = 48 + 121 = 169
  EXPECT_EQ(cda_rail::max_braking_pos_after_dt_linear_movement(10, 30, 4, 2, 3),
            169);
  EXPECT_EQ(simulator.max_displacement(train, 10, 3), 169);

  // v_0 = 20
  // After 3 seconds v_1 = 20 + 3*4 = 32 -> v_1 = 30 (capped)
  // x_1 = (20+30)*3/2 = 75
  // bd = 30*30 / 4 = 225
  // x_1 + bd = 75 + 225 = 300
  EXPECT_EQ(cda_rail::max_braking_pos_after_dt_linear_movement(20, 30, 4, 2, 3),
            300);
  EXPECT_EQ(simulator.max_displacement(train, 20, 3), 300);

  // v_0 = 30
  // After 3 seconds v_1 = 30 + 3*4 = 42 -> v_1 = 30 (capped)
  // x_1 = (30+30)*3/2 = 90
  // bd = 30*30 / 4 = 225
  // x_1 + bd = 90 + 225 = 315
  EXPECT_EQ(cda_rail::max_braking_pos_after_dt_linear_movement(30, 30, 4, 2, 3),
            315);
  EXPECT_EQ(cda_rail::max_braking_pos_after_dt_linear_movement(
                30, 30.0 - cda_rail::EPS / 2, 4, 2, 3),
            315);
  EXPECT_EQ(simulator.max_displacement(train, 30, 3), 315);
  train.set_max_speed(30.0 - cda_rail::EPS / 2);
  EXPECT_EQ(simulator.max_displacement(train, 30, 3), 315);

  // dt = 0 -> v_1 = v_0 without movement
  // bd = 10*10 / 4 = 25
  EXPECT_EQ(cda_rail::max_braking_pos_after_dt_linear_movement(10, 30, 4, 2, 0),
            25);
  EXPECT_EQ(cda_rail::max_braking_pos_after_dt_linear_movement(
                10, 30, 4, 2, -cda_rail::EPS / 2),
            25);
  train.set_max_speed(30);
  EXPECT_EQ(simulator.max_displacement(train, 10, 0), 25);
  EXPECT_EQ(simulator.max_displacement(train, 10, -cda_rail::EPS / 2), 25);

  EXPECT_THROW(
      (void)cda_rail::max_braking_pos_after_dt_linear_movement(-1, 30, 4, 2, 3),
      cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW((void)simulator.max_displacement(train, -1, 3),
               cda_rail::exceptions::InvalidInputException);

  EXPECT_THROW(
      (void)cda_rail::max_braking_pos_after_dt_linear_movement(10, 5, 4, 2, 3),
      cda_rail::exceptions::InvalidInputException);
  train.set_max_speed(5);
  EXPECT_THROW((void)simulator.max_displacement(train, 10, 3),
               cda_rail::exceptions::InvalidInputException);

  EXPECT_THROW((void)cda_rail::max_braking_pos_after_dt_linear_movement(
                   10, 30, 4, 2, -1),
               cda_rail::exceptions::InvalidInputException);
  train.set_max_speed(30);
  train.set_acceleration(4);
  train.set_deceleration(2);
  EXPECT_THROW((void)simulator.max_displacement(train, 10, -1),
               cda_rail::exceptions::InvalidInputException);
}

TEST(GreedySimulator, NextStopMA) {
  EXPECT_EQ(cda_rail::simulator::GreedySimulator::get_next_stop_ma(10, 20, 50),
            10);
  EXPECT_EQ(cda_rail::simulator::GreedySimulator::get_next_stop_ma(50, 20, 50),
            30);
}

TEST(GreedySimulator, MAandMaxV) {
  Network    network;
  const auto v0t = network.add_vertex("v0t", VertexType::TTD);
  const auto v0b = network.add_vertex("v0b", VertexType::TTD);
  const auto v1t = network.add_vertex("v1t", VertexType::TTD);
  const auto v1b = network.add_vertex("v1b", VertexType::TTD);
  const auto v2t = network.add_vertex("v2t", VertexType::TTD);
  const auto v2b = network.add_vertex("v2b", VertexType::TTD);
  const auto v3  = network.add_vertex("v3", VertexType::TTD);
  const auto v4  = network.add_vertex("v4", VertexType::TTD);
  const auto v5  = network.add_vertex("v5", VertexType::TTD);
  const auto v6  = network.add_vertex("v6", VertexType::TTD);

  const auto v0t_v1t = network.add_edge({"v0t"}, {"v1t"}, 800, 50, true);
  const auto v0b_v1b = network.add_edge({"v0b"}, {"v1b"}, 100, 5, true);
  const auto v1t_v2t = network.add_edge({"v1t"}, {"v2t"}, 100, 5, true);
  const auto v1b_v2b = network.add_edge({"v1b"}, {"v2b"}, 100, 10, true);
  const auto v2t_v3  = network.add_edge({"v2t"}, {"v3"}, 50, 50, false);
  const auto v2b_v3  = network.add_edge({"v2b"}, {"v3"}, 50, 50, false);
  const auto v3_v4   = network.add_edge({"v3"}, {"v4"}, 50, 50, false);
  const auto v4_v5   = network.add_edge({"v4"}, {"v5"}, 1000, 50, true);
  const auto v5_v6   = network.add_edge({"v5"}, {"v6"}, 100, 50, true);

  network.add_successor(v0t_v1t, v1t_v2t);
  network.add_successor(v0b_v1b, v1b_v2b);
  network.add_successor(v1t_v2t, v2t_v3);
  network.add_successor(v1b_v2b, v2b_v3);
  network.add_successor(v2t_v3, v3_v4);
  network.add_successor(v2b_v3, v3_v4);
  network.add_successor(v3_v4, v4_v5);
  network.add_successor(v4_v5, v5_v6);

  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 10, 50, 4, 2, true, 0, 15,
                                       {"v0t"}, 360, 2, {"v6"}, network);
  const auto tr2 = timetable.add_train("Train2", 10, 50, 7, 14, true, 0, 15,
                                       {"v0t"}, 360, 14, {"v6"}, network);
  const auto tr3 = timetable.add_train("Train3", 10, 50, 6, 12, true, 0, 15,
                                       {"v0t"}, 360, 12, {"v6"}, network);
  const auto tr4 = timetable.add_train("Train4", 10, 50, 5, 10, true, 0, 15,
                                       {"v0b"}, 360, 10, {"v6"}, network);
  const auto tr5 = timetable.add_train("Train5", 10, 50, 4, 8, true, 0, 15,
                                       {"v0b"}, 360, 8, {"v6"}, network);
  const auto tr6 = timetable.add_train("Train6", 10, 50, 3, 6, true, 0, 15,
                                       {"v0t"}, 360, 6, {"v6"}, network);
  const auto tr7 = timetable.add_train("Train7", 20, 50, 2, 4, true, 0, 15,
                                       {"v0b"}, 360, 4, {"v6"}, network);
  const auto tr8 = timetable.add_train("Train8", 10, 50, 8, 16, true, 0, 15,
                                       {"v0t"}, 360, 16, {"v6"}, network);

  timetable.add_empty_station("Station1");
  timetable.add_track_to_station("Station1", v2b_v3, network);

  timetable.insert_stop(tr5, "Station1", 30, 30);

  RouteMap routes;

  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);

  cda_rail::simulator::GreedySimulator simulator(instance,
                                                 {{v2t_v3, v2b_v3, v3_v4}});

  simulator.set_train_edges_of_tr(
      tr1, {v0t_v1t, v1t_v2t, v2t_v3, v3_v4, v4_v5, v5_v6});
  simulator.set_train_edges_of_tr(tr2,
                                  {v0t_v1t, v1t_v2t, v2t_v3, v3_v4, v4_v5});
  simulator.set_train_edges_of_tr(tr3,
                                  {v0t_v1t, v1t_v2t, v2t_v3, v3_v4, v4_v5});
  simulator.set_train_edges_of_tr(tr4,
                                  {v0b_v1b, v1b_v2b, v2b_v3, v3_v4, v4_v5});
  simulator.set_train_edges_of_tr(tr5,
                                  {v0b_v1b, v1b_v2b, v2b_v3, v3_v4, v4_v5});
  simulator.set_train_edges_of_tr(tr6,
                                  {v0t_v1t, v1t_v2t, v2t_v3, v3_v4, v4_v5});
  simulator.set_train_edges_of_tr(tr7,
                                  {v0b_v1b, v1b_v2b, v2b_v3, v3_v4, v4_v5});
  simulator.set_train_edges_of_tr(tr8,
                                  {v0t_v1t, v1t_v2t, v2t_v3, v3_v4, v4_v5});

  simulator.append_stop_edge_to_tr(tr5, v2b_v3);

  simulator.set_ttd_orders({{tr1, tr2, tr3, tr4, tr5, tr6, tr7, tr8}});

  std::vector<cda_rail::simulator::GreedySimulator::TrainPosition> train_pos = {
      {2080, 2100}, // Train1 -> front increased by 10 to incorporate length in
                    // exit headway
      {1970, 1980}, // Train2
      {1090, 1100}, // Train3
      {340, 350},   // Train4
      {240, 250},   // Train5 (stopped at Station1)
      {875, 885},   // Train6
      {90, 110},    // Train7
      {590, 600}    // Train8
  };

  // Check that the ma and max speed constraints are correctly calculated
  // Train 1: Bound by leaving headway -> REMOVED BY REFACTORING
  // Train 2: Bound by final edge
  // Train 3: No bounds -> maximal displacement
  // Train 4: Bound by Train 3
  // Train 5: Bound by stopping at Station1
  // Train 6: Bound by Train 5 in TTD
  // Train 7: Bound by speed limit of edge
  // Train 8: Bound by future speed limit of v1t_v2t

  std::unordered_set<size_t> train_ids   = {tr1, tr2, tr3, tr4,
                                            tr5, tr6, tr7, tr8};
  const auto                 tr_on_edges = simulator.tr_on_edges();

  std::vector<double> train_velocities(
      simulator.get_instance()->get_const_train_list().size(), 0.0);

  train_velocities.at(tr2) = 23;
  // Train 2: Bound by final edge, a = 7, d = 14 (not anymore)
  // REMOVED

  // Train 3: No bounds -> maximal displacement, a = 6, d = 12
  train_velocities.at(tr3) = 10;
  const auto ma_data3      = simulator.get_ma_and_maxv(
      tr3, train_velocities, {}, 400, 2, {}, train_pos, train_ids, {},
      tr_on_edges, true, true);
  const auto ma3    = ma_data3.ma;
  const auto max_v3 = ma_data3.max_v;
  EXPECT_APPROX_EQ_6(ma3, 52.0 + 1.0 / 6.0);
  EXPECT_GE((10.0 + max_v3) * 2.0 / 2.0 + ((max_v3 * max_v3) / (2 * 12)), ma3);
  train_velocities.at(tr3) = 30;
  const auto ma_data3b     = simulator.get_ma_and_maxv(
      tr3, train_velocities, {}, 400, 20, {}, train_pos, train_ids, {},
      tr_on_edges, true, true); // this time limited by tr2
  const auto ma3b    = ma_data3b.ma;
  const auto max_v3b = ma_data3b.max_v;
  EXPECT_EQ(ma3b, 870);
  EXPECT_GE((30.0 + max_v3b) * 20.0 / 2.0 + ((max_v3b * max_v3b) / (2 * 12)),
            ma3b);

  // Train 4: Bound by Train 3, a = 5, d = 10
  train_velocities.at(tr4) = 28;
  const auto ma_data4      = simulator.get_ma_and_maxv(
      tr4, train_velocities, {}, 400, 2, {}, train_pos, train_ids, {},
      tr_on_edges, true, true);
  const auto ma4    = ma_data4.ma;
  const auto max_v4 = ma_data4.max_v;
  EXPECT_EQ(ma4, 40);
  EXPECT_GE((28.0 + max_v4) * 2.0 / 2.0 + ((max_v4 * max_v4) / (2 * 10)), ma4);

  // Train 5: Bound by stopping at Station1, a = 4, d = 8
  train_velocities.at(tr5) = 0;
  const auto ma_data5      = simulator.get_ma_and_maxv(
      tr5, train_velocities, {0}, 400, 2, {}, train_pos, train_ids, {},
      tr_on_edges, true, true);
  const auto ma5    = ma_data5.ma;
  const auto max_v5 = ma_data5.max_v;
  EXPECT_EQ(ma5, 0);
  EXPECT_GE((0.0 + max_v5) * 2.0 / 2.0 + ((max_v5 * max_v5) / (2 * 8)), ma5);
  // Otherwise 90m away from tr4
  train_velocities.at(tr5) = 30;
  const auto ma_data5b     = simulator.get_ma_and_maxv(
      tr5, train_velocities, {}, 400, 2, {}, train_pos, train_ids, {},
      tr_on_edges, true, true);
  const auto ma5b    = ma_data5b.ma;
  const auto max_v5b = ma_data5b.max_v;
  EXPECT_EQ(ma5b, 90);
  EXPECT_GE((30.0 + max_v5b) * 2.0 / 2.0 + ((max_v5b * max_v5b) / (2 * 8)),
            ma5b);

  // Train 6: Bound by Train 5 in TTD, a = 3, d = 6
  // 15m away from TTD
  train_velocities.at(tr6) = 10;
  const auto ma_data6      = simulator.get_ma_and_maxv(
      tr6, train_velocities, {}, 400, 2, {}, train_pos, train_ids, {},
      tr_on_edges, true, true);
  const auto ma6    = ma_data6.ma;
  const auto max_v6 = ma_data6.max_v;
  EXPECT_EQ(ma6, 15);
  EXPECT_GE((10.0 + max_v6) * 2.0 / 2.0 + ((max_v6 * max_v6) / (2 * 6)), ma6);

  // Train 7: Bound by speed limit of edge, a = 2, d = 4
  train_velocities.at(tr7) = 4;
  const auto ma_data7      = simulator.get_ma_and_maxv(
      tr7, train_velocities, {}, 400, 4, {}, train_pos, train_ids, {},
      tr_on_edges, true, true);
  const auto ma7    = ma_data7.ma;
  const auto max_v7 = ma_data7.max_v;
  EXPECT_EQ(max_v7, 5);
  EXPECT_LE((4.0 + max_v7) * 4.0 / 2.0 + ((max_v7 * max_v7) / (2 * 4)), ma7);
  train_velocities.at(tr7) = 4;
  const auto ma_data7b     = simulator.get_ma_and_maxv(
      tr7, train_velocities, {}, 400, 4, {}, train_pos, train_ids, {},
      tr_on_edges, false, true);
  const auto ma7b    = ma_data7b.ma;
  const auto max_v7b = ma_data7b.max_v;
  EXPECT_EQ(max_v7b, 10);
  EXPECT_LE((4.0 + max_v7b) * 4.0 / 2.0 + ((max_v7b * max_v7b) / (2 * 4)),
            ma7b);

  // Train 8: Bound by future speed limit of v1t_v2t, a = 8, d = 16
  // At pos = 800 limit of 5 m/s starts
  // bd = 5 * 5 / (2*16) = 25 / 32 = 0.78125
  // --> ma at 800.78125
  // Train is 200m away from position 800
  train_velocities.at(tr8) = 30;
  const auto ma_data8      = simulator.get_ma_and_maxv(
      tr8, train_velocities, {}, 400, 5, {}, train_pos, train_ids, {},
      tr_on_edges, true, true);
  const auto ma8    = ma_data8.ma;
  const auto max_v8 = ma_data8.max_v;
  EXPECT_EQ(ma8, 200.78125);
  EXPECT_GE((30.0 + max_v8) * 5.0 / 2.0 + ((max_v8 * max_v8) / (2 * 4)), ma8);
}

TEST(GreedySimulator, MAtoV) {
  // v_0 = 5
  // v_1 = 10 after 6 seconds
  // x_1 = (5 + 10) * 6 / 2 = 45
  // d = 4
  // bd = 10 * 10 / (2 * 4) = 12.5
  // x_1 + bd = 45 + 12.5 = 57.5
  EXPECT_EQ(cda_rail::simulator::GreedySimulator::get_v1_from_ma(5, 57.5, 4, 6),
            10);

  EXPECT_EQ(cda_rail::simulator::GreedySimulator::get_v1_from_ma(0, 0, 4, 5),
            0);
  EXPECT_EQ(cda_rail::simulator::GreedySimulator::get_v1_from_ma(
                -cda_rail::EPS / 2.0, 0, 4, 5),
            0);
  EXPECT_EQ(cda_rail::simulator::GreedySimulator::get_v1_from_ma(
                0, -cda_rail::EPS / 2.0, 4, 5),
            0);

  // v_0 = 0
  // v_0 = 6 after 5 seconds
  // x_1 = (0 + 6) * 5 / 2 = 15
  // d = 3
  // bd = 6 * 6 / (2 * 3) = 6
  // x_1 + bd = 15 + 6 = 21
  EXPECT_EQ(cda_rail::simulator::GreedySimulator::get_v1_from_ma(0, 21, 3, 5),
            6);

  // v_0 = 10
  // d = 2
  // bd = 10 * 10 / (2 * 2) = 25
  // t = 10/2 = 5s
  EXPECT_EQ(cda_rail::simulator::GreedySimulator::get_v1_from_ma(10, 25, 2, 5),
            0);
  EXPECT_EQ(cda_rail::simulator::GreedySimulator::get_v1_from_ma(10, 25, 2, 6),
            0);

  EXPECT_THROW((void)cda_rail::simulator::GreedySimulator::get_v1_from_ma(
                   -1, 57.5, 4, 6),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(
      (void)cda_rail::simulator::GreedySimulator::get_v1_from_ma(5, -1, 4, 6),
      cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(
      (void)cda_rail::simulator::GreedySimulator::get_v1_from_ma(5, 57.5, 0, 6),
      cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW((void)cda_rail::simulator::GreedySimulator::get_v1_from_ma(
                   5, 57.5, cda_rail::EPS / 2.0, 6),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW((void)cda_rail::simulator::GreedySimulator::get_v1_from_ma(
                   5, 57.5, 4, -1),
               cda_rail::exceptions::InvalidInputException);
}

TEST(GreedySimulator, MoveTrain) {
  std::vector<cda_rail::simulator::GreedySimulator::TrainPosition> train_pos = {
      {-10, 2},   // Train 0
      {10, 80},   // Train 1
      {250, 300}, // Train 2
      {500, 800}  // Train 3
  };

  // v_0 = 5
  // Stopping within 6m
  // dt = 8 -> theoretical distance = 5 * 8/2 = 20
  EXPECT_TRUE(cda_rail::simulator::GreedySimulator::move_train(0, 5, 0, 6, 8,
                                                               train_pos));
  EXPECT_EQ(train_pos.size(), 4);
  EXPECT_EQ(train_pos[0].rear, -10);
  EXPECT_EQ(train_pos[0].front, 2 + 6);
  EXPECT_EQ(train_pos[1].rear, 10);
  EXPECT_EQ(train_pos[1].front, 80);
  EXPECT_EQ(train_pos[2].rear, 250);
  EXPECT_EQ(train_pos[2].front, 300);
  EXPECT_EQ(train_pos[3].rear, 500);
  EXPECT_EQ(train_pos[3].front, 800);

  // v_0 = 10
  // v_1 = 20 after 4 seconds
  // x_1 = (10 + 20) * 4 / 2 = 60
  EXPECT_TRUE(cda_rail::simulator::GreedySimulator::move_train(1, 10, 20, 100,
                                                               4, train_pos));
  EXPECT_EQ(train_pos.size(), 4);
  EXPECT_EQ(train_pos[0].rear, -10);
  EXPECT_EQ(train_pos[0].front, 2 + 6);
  EXPECT_EQ(train_pos[1].rear, 10);
  EXPECT_EQ(train_pos[1].front, 80 + 60);
  EXPECT_EQ(train_pos[2].rear, 250);
  EXPECT_EQ(train_pos[2].front, 300);
  EXPECT_EQ(train_pos[3].rear, 500);
  EXPECT_EQ(train_pos[3].front, 800);

  // v_0 = 10
  // v_1 = 0 after 5 seconds
  // x_1 = (10 + 0) * 5 / 2 = 25
  EXPECT_TRUE(cda_rail::simulator::GreedySimulator::move_train(2, 10, 0, 150, 5,
                                                               train_pos));
  EXPECT_EQ(train_pos.size(), 4);
  EXPECT_EQ(train_pos[0].rear, -10);
  EXPECT_EQ(train_pos[0].front, 2 + 6);
  EXPECT_EQ(train_pos[1].rear, 10);
  EXPECT_EQ(train_pos[1].front, 80 + 60);
  EXPECT_EQ(train_pos[2].rear, 250);
  EXPECT_EQ(train_pos[2].front, 300 + 25);
  EXPECT_EQ(train_pos[3].rear, 500);
  EXPECT_EQ(train_pos[3].front, 800);

  // v_0 = 0
  // v_1 = 0 after 10 seconds
  // x_1 = (0 + 0) * 10 / 2 = 0
  EXPECT_FALSE(cda_rail::simulator::GreedySimulator::move_train(3, 0, 0, 500,
                                                                10, train_pos));
  EXPECT_EQ(train_pos.size(), 4);
  EXPECT_EQ(train_pos[0].rear, -10);
  EXPECT_EQ(train_pos[0].front, 2 + 6);
  EXPECT_EQ(train_pos[1].rear, 10);
  EXPECT_EQ(train_pos[1].front, 80 + 60);
  EXPECT_EQ(train_pos[2].rear, 250);
  EXPECT_EQ(train_pos[2].front, 300 + 25);
  EXPECT_EQ(train_pos[3].rear, 500);
  EXPECT_EQ(train_pos[3].front, 800);

  EXPECT_THROW((void)cda_rail::simulator::GreedySimulator::move_train(
                   4, 5, 0, 6, 8, train_pos),
               cda_rail::exceptions::TrainNotExistentException);
}

TEST(GreedySimulator, UpdateRearPositions) {
  Network    network;
  const auto v0 = network.add_vertex("v0", VertexType::TTD);
  const auto v1 = network.add_vertex("v1", VertexType::TTD);

  const auto v0_v1 = network.add_edge(v0, v1, 5000, 50, true);

  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 20, 50, 4, 2, true, 0, 15, v0,
                                       360, 2, v1, network);
  const auto tr2 = timetable.add_train("Train2", 12, 50, 7, 14, true, 0, 15, v0,
                                       360, 14, v1, network);
  const auto tr3 = timetable.add_train("Train3", 300, 50, 6, 12, true, 0, 15,
                                       {"v0"}, 360, 12, {"v1"}, network);
  const auto tr4 = timetable.add_train("Train4", 5, 50, 5, 10, true, 0, 15,
                                       {"v0"}, 360, 10, {"v1"}, network);
  const auto tr5 = timetable.add_train("Train5", 15, 50, 4, 8, true, 0, 15,
                                       {"v0"}, 360, 8, {"v1"}, network);
  const auto tr6 = timetable.add_train("Train6", 20, 50, 3, 6, true, 0, 15,
                                       {"v0"}, 360, 6, {"v1"}, network);
  const auto tr7 = timetable.add_train("Train7", 150, 50, 2, 4, true, 0, 15,
                                       {"v0"}, 360, 4, {"v1"}, network);
  const auto tr8 = timetable.add_train("Train8", 9, 50, 8, 16, true, 0, 15,
                                       {"v0"}, 360, 16, {"v1"}, network);

  RouteMap routes;

  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);

  cda_rail::simulator::GreedySimulator simulator(instance, {});

  simulator.set_train_edges_of_tr(tr1, {v0_v1});
  simulator.set_train_edges_of_tr(tr2, {v0_v1});
  simulator.set_train_edges_of_tr(tr3, {v0_v1});
  simulator.set_train_edges_of_tr(tr4, {v0_v1});
  simulator.set_train_edges_of_tr(tr5, {v0_v1});
  simulator.set_train_edges_of_tr(tr6, {v0_v1});
  simulator.set_train_edges_of_tr(tr7, {v0_v1});
  simulator.set_train_edges_of_tr(tr8, {v0_v1});

  std::vector<cda_rail::simulator::GreedySimulator::TrainPosition> train_pos = {
      {2010, 2090}, // Train1, length 20
      {1900, 1980}, // Train2, length 12
      {500, 1100},  // Train3, length 300
      {320, 350},   // Train4, length 5
      {200, 250},   // Train5, length 15
      {775, 885},   // Train6, length 20
      {-50, 110},   // Train7, length 150
      {580, 600}    // Train8, length 9
  };

  simulator.update_rear_positions(train_pos);
  EXPECT_EQ(train_pos.size(), 8);
  // Train 1, length 20
  // Front: 2090
  // Rear: 2090 - 20 = 2070
  EXPECT_EQ(train_pos[0].rear, 2070);
  EXPECT_EQ(train_pos[0].front, 2090);
  // Train 2, length 12
  // Front: 1980
  // Rear: 1980 - 12 = 1968
  EXPECT_EQ(train_pos[1].rear, 1968);
  EXPECT_EQ(train_pos[1].front, 1980);
  // Train 3, length 300
  // Front: 1100
  // Rear: 1100 - 300 = 800
  EXPECT_EQ(train_pos[2].rear, 800);
  EXPECT_EQ(train_pos[2].front, 1100);
  // Train 4, length 5
  // Front: 350
  // Rear: 350 - 5 = 345
  EXPECT_EQ(train_pos[3].rear, 345);
  EXPECT_EQ(train_pos[3].front, 350);
  // Train 5, length 15
  // Front: 250
  // Rear: 250 - 15 = 235
  EXPECT_EQ(train_pos[4].rear, 235);
  EXPECT_EQ(train_pos[4].front, 250);
  // Train 6, length 20
  // Front: 885
  // Rear: 885 - 20 = 865
  EXPECT_EQ(train_pos[5].rear, 865);
  EXPECT_EQ(train_pos[5].front, 885);
  // Train 7, length 150
  // Front: 110
  // Rear: 110 - 150 = -40
  EXPECT_EQ(train_pos[6].rear, -40);
  EXPECT_EQ(train_pos[6].front, 110);
  // Train 8, length 9
  // Front: 600
  // Rear: 600 - 9 = 591
  EXPECT_EQ(train_pos[7].rear, 591);
  EXPECT_EQ(train_pos[7].front, 600);

  train_pos = {
      {2010, 2090}, // Train1, length 20
      {1900, 1980}, // Train2, length 12
      {500, 1100},  // Train3, length 300
      {320, 350},   // Train4, length 5
      {200, 250},   // Train5, length 15
      {775, 885},   // Train6, length 20
      {-50, 110},   // Train7, length 150
      {580, 600},   // Train8, length 9
      {0, 50}       // Additional train
  };

  // Too many trains
  EXPECT_THROW((void)simulator.update_rear_positions(train_pos),
               cda_rail::exceptions::TrainNotExistentException);
}

TEST(GreedySimulator, ReverseEdgeMA) {
  Network    network;
  const auto v0 = network.add_vertex("v0", VertexType::TTD);
  const auto v1 = network.add_vertex("v1", VertexType::TTD);
  const auto v2 = network.add_vertex("v2", VertexType::TTD);
  const auto v3 = network.add_vertex("v3", VertexType::TTD);

  const auto v0_v1 = network.add_edge(v0, v1, 200, 50, true);
  const auto v1_v0 = network.add_edge(v1, v0, 200, 50, true);
  const auto v1_v2 = network.add_edge(v1, v2, 300, 50, true);
  const auto v2_v3 = network.add_edge(v2, v3, 400, 50, true);
  const auto v3_v2 = network.add_edge(v3, v2, 400, 50, true);
  const auto v2_v1 = network.add_edge(v2, v1, 300, 50, true);

  network.add_successor(v0_v1, v1_v2);
  network.add_successor(v1_v2, v2_v3);
  network.add_successor(v3_v2, v2_v1);
  network.add_successor(v2_v1, v1_v0);

  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 10, 50, 1, 2, true, 0, 30, v0,
                                       360, 2, v3, network);
  const auto tr2 = timetable.add_train("Train2", 10, 50, 2, 4, true, 30, 15, v3,
                                       360, 14, v0, network);

  RouteMap                                                    routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);
  cda_rail::simulator::GreedySimulator simulator(instance, {});

  simulator.set_train_edges_of_tr(tr1, {v0_v1, v1_v2, v2_v3});
  simulator.set_train_edges_of_tr(tr2, {v3_v2, v2_v1, v1_v0});
  simulator.set_vertex_orders_of_vertex(v0, {tr1, tr2});
  simulator.set_vertex_orders_of_vertex(v3, {tr2, tr1});

  std::vector<cda_rail::simulator::GreedySimulator::TrainPosition> train_pos = {
      {160, 170}, // Train1, 30m away from conflicting edge
      {340, 350}  // Train2, 50m away from conflicting edge
  };
  std::vector<double> train_velocities(2, 0);

  const auto tr_on_edges = simulator.tr_on_edges();

  // No conflict, Train 1 can move as far as possible
  // v_0 = 0, v_1 = 0+ 2*1 = 2 after 2s
  // x_1 = (0 + 2) * 2 / 2 = 2
  // bd = 2 * 2 / (2*2) = 1
  // ma = x_1 + bd = 2 + 1 = 3
  const auto ma_data1 = simulator.get_ma_and_maxv(
      tr1, train_velocities, {}, 400, 2, {}, train_pos, {tr1, tr2}, {},
      tr_on_edges, true, true);
  const auto ma1    = ma_data1.ma;
  const auto max_v1 = ma_data1.max_v;
  EXPECT_EQ(ma1, 3);
  EXPECT_EQ(max_v1, 2);

  // Similar for Train 2
  // v_0 = 0, v_1 = 0 + 10 * 2 = 20 after 10s
  // x_1 = (0 + 20) * 10 / 2 = 100
  // bd = 20 * 20 / (2*4) = 50
  // ma = x_1 + bd = 100 + 50 = 150
  const auto ma_data2 = simulator.get_ma_and_maxv(
      tr2, train_velocities, {}, 400, 10, {}, train_pos, {tr1, tr2}, {},
      tr_on_edges, true, true);
  const auto ma2    = ma_data2.ma;
  const auto max_v2 = ma_data2.max_v;
  EXPECT_EQ(ma2, 150);
  EXPECT_EQ(max_v2, 20);

  // Now Train 1 has v_0 = 11
  // bd = 11 * 11 / (2*2) = 30.25 -> Entering conflicting edge
  // Hence Train 2 cannot enter and only move 50m.
  // On the other hand, Train 1 is not limited.
  // v_0 = 11, v_1 = 11 + 2 * 1 = 13 after 2s
  // x_1 = (11 + 13) * 2 / 2 = 24
  // bd = 13 * 13 / (2*2) = 42.25
  // ma = x_1 + bd = 24 + 42.25 = 66.25
  train_velocities.at(tr1) = 11;
  const auto ma_data1b     = simulator.get_ma_and_maxv(
      tr1, train_velocities, {}, 400, 2, {}, train_pos, {tr1, tr2}, {},
      tr_on_edges, true, true);
  const auto ma1b    = ma_data1b.ma;
  const auto max_v1b = ma_data1b.max_v;
  EXPECT_EQ(ma1b, 66.25);
  EXPECT_EQ(max_v1b, 13);

  const auto ma_data2b = simulator.get_ma_and_maxv(
      tr2, train_velocities, {}, 400, 10, {}, train_pos, {tr1, tr2}, {},
      tr_on_edges, true, true);
  const auto ma2b    = ma_data2b.ma;
  const auto max_v2b = ma_data2b.max_v;
  EXPECT_EQ(ma2b, 50);
  EXPECT_GE((10.0 + max_v2b) * 10.0 / 2.0 + ((max_v2b * max_v2b) / (2.0 * 4.0)),
            ma2b);

  // Check reverse edge problem for entering trains. Train 1 can reach the
  // conflicting edge.
  train_velocities.at(tr1) = 0;
  train_velocities.at(tr2) = 0;
  EXPECT_TRUE(simulator.is_ok_to_enter(tr1, train_pos, train_velocities, {tr2},
                                       tr_on_edges));

  train_velocities.at(tr2) = 25; // Train 2 has ma into conflicting edge
  EXPECT_FALSE(simulator.is_ok_to_enter(tr1, train_pos, train_velocities, {tr2},
                                        tr_on_edges));
}

TEST(GreedySimulator, ExitVertexOrder) {
  Network    network;
  const auto v0    = network.add_vertex("v0", VertexType::TTD);
  const auto v1    = network.add_vertex("v1", VertexType::TTD);
  const auto v0_v1 = network.add_edge(v0, v1, 5000, 50, true);
  const auto v1_v0 = network.add_edge(v1, v0, 5000, 50, true);

  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 100, 50, 4, 2, true, 0, 15, v0,
                                       198, 40, v1, network);
  const auto tr2 = timetable.add_train("Train2", 100, 50, 2, 1, true, 0, 15, v1,
                                       198, 40, v0, network);

  RouteMap                                                    routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);
  cda_rail::simulator::GreedySimulator simulator(instance, {});

  simulator.set_train_edges_of_tr(tr1, {v0_v1});
  simulator.set_train_edges_of_tr(tr2, {v1_v0});
  simulator.set_vertex_orders_of_vertex(v0, {tr1, tr2});
  simulator.set_vertex_orders_of_vertex(v1, {tr2, tr1});

  // 3500 from final vertex

  EXPECT_EQ(simulator.get_exit_vertex_order_ma(tr1, 1500, 4000, {tr1}, {}),
            3500);
}

// -------------------
// Test simulation
// -------------------

TEST(GreedySimulation, SimpleSimulation) {
  static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
  plog::init(plog::verbose, &console_appender);

  Network    network;
  const auto v0 = network.add_vertex("v0", VertexType::TTD, 60);
  const auto v1 = network.add_vertex("v1", VertexType::TTD, 30);

  const auto v0_v1 = network.add_edge(v0, v1, 5000, 50, true);
  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 100, 50, 4, 2, true, 0, 15, v0,
                                       198, 40, v1, network);
  RouteMap   routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);
  cda_rail::simulator::GreedySimulator simulator(instance, {});

  simulator.set_train_edges_of_tr(tr1, {v0_v1});
  simulator.set_vertex_orders_of_vertex(v0, {tr1});
  simulator.set_vertex_orders_of_vertex(v1, {tr1});

  const auto sim_res = simulator.simulate(6.0, false, true);
  ASSERT_FALSE(sim_res.exit_times.empty());
  PLOGD << "Simulation success: " << (sim_res.success ? "true" : "false")
        << ", Objective value: " << sim_res.exit_times.back() << std::endl;

  EXPECT_TRUE(sim_res.success);
  ASSERT_EQ(sim_res.exit_times.size(), 1);
  EXPECT_GE(sim_res.exit_times[0], 198);
  EXPECT_LE(sim_res.exit_times[0], 198 + 8);
  ASSERT_EQ(sim_res.vertex_headways.size(), 2);
  EXPECT_EQ(sim_res.vertex_headways.at(v0), 60);
  EXPECT_EQ(sim_res.vertex_headways.at(v1), sim_res.exit_times[0] + 30);
}

TEST(GreedySimulation, SimpleSimulationAdditionalTrain) {
  static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
  plog::init(plog::verbose, &console_appender);

  Network    network;
  const auto v0 = network.add_vertex("v0", VertexType::TTD, 60);
  const auto v1 = network.add_vertex("v1", VertexType::TTD, 30);

  const auto v0_v1 = network.add_edge(v0, v1, 5000, 50, true);
  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 100, 50, 4, 2, true, 0, 15, v0,
                                       198, 40, v1, network);
  const auto tr2 = timetable.add_train("Train2", 100, 50, 4, 2, true, 0, 15, v1,
                                       198, 40, v0, network);
  RouteMap   routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);
  cda_rail::simulator::GreedySimulator simulator(instance, {});

  simulator.set_train_edges_of_tr(tr1, {v0_v1});
  simulator.set_vertex_orders_of_vertex(v0, {tr1, tr2});
  simulator.set_vertex_orders_of_vertex(v1, {tr1, tr2});

  const auto sim_res = simulator.simulate(6.0, false, true);
  ASSERT_FALSE(sim_res.exit_times.empty());
  PLOGD << "Simulation success: " << (sim_res.success ? "true" : "false")
        << ", Objective value: " << "(" << sim_res.exit_times[0] << ", "
        << sim_res.exit_times[1] << ")";

  EXPECT_TRUE(sim_res.success);
  ASSERT_EQ(sim_res.exit_times.size(), 2);
  EXPECT_GE(sim_res.exit_times[0], 198);
  EXPECT_LE(sim_res.exit_times[0], 198 + 8);
  EXPECT_EQ(sim_res.exit_times[1], 0);
  ASSERT_EQ(sim_res.vertex_headways.size(), 2);
  EXPECT_EQ(sim_res.vertex_headways.at(v0), 60);
  EXPECT_EQ(sim_res.vertex_headways.at(v1), sim_res.exit_times[0] + 30);
}

TEST(GreedySimulation, SimpleSimulationTwoTrains) {
  static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
  plog::init(plog::verbose, &console_appender);

  Network    network;
  const auto v0 = network.add_vertex("v0", VertexType::TTD, 120);
  const auto v1 = network.add_vertex("v1", VertexType::TTD, 30);

  const auto v0_v1 = network.add_edge(v0, v1, 5000, 50, true);
  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 100, 20, 4, 2, true, 0, 20, v0,
                                       10, 20, v1, network);
  const auto tr2 = timetable.add_train("Train2", 100, 20, 4, 2, true, 0, 20, v0,
                                       10, 20, v1, network);
  RouteMap   routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);
  cda_rail::simulator::GreedySimulator simulator(instance, {});

  simulator.set_train_edges_of_tr(tr1, {v0_v1});
  simulator.set_train_edges_of_tr(tr2, {v0_v1});
  simulator.set_vertex_orders_of_vertex(v0, {tr1, tr2});
  simulator.set_vertex_orders_of_vertex(v1, {tr1, tr2});

  const auto sim_res_impossible = simulator.simulate(6.0, false, true);
  EXPECT_FALSE(sim_res_impossible.success);

  const auto sim_res = simulator.simulate(6.0, true, true);
  ASSERT_EQ(sim_res.exit_times.size(), 2);
  PLOGD << "Simulation success: " << (sim_res.success ? "true" : "false")
        << ", Objective value: " << "(" << sim_res.exit_times[0] << ", "
        << sim_res.exit_times[1] << ")";
  EXPECT_TRUE(sim_res.success);
  EXPECT_GE(sim_res.exit_times[0], 250);
  EXPECT_LE(sim_res.exit_times[0], 250 + 6);
  EXPECT_GE(sim_res.exit_times[1], 120 + 250);
  EXPECT_LE(sim_res.exit_times[1], 120 + 250 + 6);
  ASSERT_EQ(sim_res.vertex_headways.size(), 2);
  EXPECT_EQ(sim_res.vertex_headways.at(v0), 240);
  EXPECT_EQ(sim_res.vertex_headways.at(v1), sim_res.exit_times[1] + 30);
}

TEST(GreedySimulator, DeadlockTest1) {
  static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
  plog::init(plog::verbose, &console_appender);

  Network    network;
  const auto v0 = network.add_vertex("v0", VertexType::TTD, 60);
  const auto v1 = network.add_vertex("v1", VertexType::TTD, 30);

  const auto v0_v1 = network.add_edge(v0, v1, 5000, 50, true);
  const auto v1_v0 = network.add_edge(v1, v0, 5000, 50, true);
  Timetable  timetable;
  const auto tr2 = timetable.add_train("Train2", 100, 50, 4, 2, true, 130, 15,
                                       v1, 198, 40, v0, network);
  const auto tr1 = timetable.add_train("Train1", 100, 50, 4, 2, true, 0, 15, v0,
                                       198, 40, v1, network);

  RouteMap                                                    routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);
  cda_rail::simulator::GreedySimulator simulator(instance, {});

  simulator.set_train_edges_of_tr(tr1, {v0_v1});
  simulator.set_train_edges_of_tr(tr2, {v1_v0});
  simulator.set_vertex_orders_of_vertex(v0, {tr1, tr2});
  simulator.set_vertex_orders_of_vertex(v1, {tr2, tr1});

  const auto sim_res = simulator.simulate(6.0, true, true);
  ASSERT_EQ(sim_res.exit_times.size(), 2);
  PLOGD << "Simulation success: " << (sim_res.success ? "true" : "false")
        << ", Objective value: (" << sim_res.exit_times.at(0) << ", "
        << sim_res.exit_times.at(1) << ")";
  EXPECT_EQ(sim_res.exit_times.at(0), 0);
  EXPECT_EQ(sim_res.exit_times.at(1), 0);
  EXPECT_FALSE(sim_res.success);
  ASSERT_EQ(sim_res.vertex_headways.size(), 2);
  EXPECT_EQ(sim_res.vertex_headways.at(v0), 60);
  EXPECT_EQ(sim_res.vertex_headways.at(v1), 0);
}

TEST(GreedySimulator, DeadlockTest2) {
  static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
  plog::init(plog::verbose, &console_appender);

  Network    network;
  const auto v0 = network.add_vertex("v0", VertexType::TTD, 60);
  const auto v1 = network.add_vertex("v1", VertexType::TTD);
  const auto v2 = network.add_vertex("v2", VertexType::TTD);
  const auto v3 = network.add_vertex("v3", VertexType::TTD, 30);

  const auto v0_v1 = network.add_edge(v0, v1, 500, 50, true);
  const auto v1_v0 = network.add_edge(v1, v0, 500, 50, true);
  const auto v1_v2 = network.add_edge(v1, v2, 600, 50, true);
  const auto v2_v1 = network.add_edge(v2, v1, 600, 50, true);
  const auto v2_v3 = network.add_edge(v2, v3, 1000, 50, true);
  const auto v3_v2 = network.add_edge(v3, v2, 1000, 50, true);

  network.add_successor(v0_v1, v1_v2);
  network.add_successor(v1_v2, v2_v3);
  network.add_successor(v3_v2, v2_v1);
  network.add_successor(v2_v1, v1_v0);

  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 100, 50, 4, 2, true, 0, 15, v0,
                                       198, 40, v3, network);
  const auto tr2 = timetable.add_train("Train2", 100, 50, 4, 2, true, 0, 15, v3,
                                       198, 40, v0, network);

  RouteMap                                                    routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);
  cda_rail::simulator::GreedySimulator simulator(instance, {});

  simulator.set_train_edges_of_tr(tr1, {v0_v1, v1_v2, v2_v3});
  simulator.set_train_edges_of_tr(tr2, {v3_v2, v2_v1, v1_v0});
  simulator.set_vertex_orders_of_vertex(v0, {tr1, tr2});
  simulator.set_vertex_orders_of_vertex(v3, {tr2, tr1});

  const auto sim_res = simulator.simulate(6.0, true, true);
  ASSERT_EQ(sim_res.exit_times.size(), 2);
  PLOGD << "Simulation success: " << (sim_res.success ? "true" : "false")
        << ", Objective value: (" << sim_res.exit_times.at(0) << ", "
        << sim_res.exit_times.at(1) << ")";
  EXPECT_EQ(sim_res.exit_times.at(0), 0);
  EXPECT_EQ(sim_res.exit_times.at(1), 0);
  EXPECT_FALSE(sim_res.success);
  ASSERT_EQ(sim_res.vertex_headways.size(), 4);
  EXPECT_EQ(sim_res.vertex_headways.at(v0), 60);
  EXPECT_EQ(sim_res.vertex_headways.at(v1), 0);
  EXPECT_EQ(sim_res.vertex_headways.at(v2), 0);
  EXPECT_EQ(sim_res.vertex_headways.at(v3), 30);
}

TEST(GreedySimulator, DeadlockTest3) {
  static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
  plog::init(plog::verbose, &console_appender);

  Network    network;
  const auto v0 = network.add_vertex("v0", VertexType::TTD, 60);
  const auto v1 = network.add_vertex("v1", VertexType::TTD);
  const auto v2 = network.add_vertex("v2", VertexType::TTD);
  const auto v3 = network.add_vertex("v3", VertexType::TTD, 30);

  const auto v0_v1 = network.add_edge(v0, v1, 500, 50, true);
  const auto v1_v0 = network.add_edge(v1, v0, 500, 50, true);
  const auto v1_v2 = network.add_edge(v1, v2, 600, 50, true);
  const auto v2_v1 = network.add_edge(v2, v1, 600, 50, true);
  const auto v2_v3 = network.add_edge(v2, v3, 1000, 50, true);
  const auto v3_v2 = network.add_edge(v3, v2, 1000, 50, true);

  network.add_successor(v0_v1, v1_v2);
  network.add_successor(v1_v2, v2_v3);
  network.add_successor(v3_v2, v2_v1);
  network.add_successor(v2_v1, v1_v0);

  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 100, 50, 4, 2, true, 0, 50, v0,
                                       198, 40, v3, network);
  const auto tr2 = timetable.add_train("Train2", 100, 50, 4, 2, true, 30, 50,
                                       v3, 198, 40, v0, network);

  RouteMap                                                    routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);
  cda_rail::simulator::GreedySimulator simulator(instance, {});

  simulator.set_train_edges_of_tr(tr1, {v0_v1});
  simulator.set_train_edges_of_tr(tr2, {v3_v2});
  simulator.set_vertex_orders_of_vertex(v0, {tr1, tr2});
  simulator.set_vertex_orders_of_vertex(v3, {tr2, tr1});

  const auto sim_res = simulator.simulate(6.0, true, true);
  ASSERT_EQ(sim_res.exit_times.size(), 2);
  PLOGD << "Simulation success: " << (sim_res.success ? "true" : "false")
        << ", Objective value: (" << sim_res.exit_times.at(0) << ", "
        << sim_res.exit_times.at(1) << ")";
  // no stopping but instantaneous stop at route end
  const auto time1 = 500.0 / 50.0;
  const auto time2 = 30.0 + 1000.0 / 50.0;
  EXPECT_GE(sim_res.exit_times.at(0), time1 - 3);
  EXPECT_GE(sim_res.exit_times.at(1), time2 - 3);
  EXPECT_LE(sim_res.exit_times.at(0), time1 + 6);
  EXPECT_LE(sim_res.exit_times.at(1), time2 + 6);
  EXPECT_TRUE(sim_res.success);
  ASSERT_EQ(sim_res.vertex_headways.size(), 4);
  EXPECT_EQ(sim_res.vertex_headways.at(v0), 60);
  EXPECT_EQ(sim_res.vertex_headways.at(v1), 0);
  EXPECT_EQ(sim_res.vertex_headways.at(v2), 0);
  EXPECT_EQ(sim_res.vertex_headways.at(v3), 30 + 30);
}

TEST(GreedySimulator, PartialRouteTest) {
  static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
  plog::init(plog::verbose, &console_appender);

  Network    network;
  const auto v0 = network.add_vertex("v0", VertexType::TTD);
  const auto v1 = network.add_vertex("v1", VertexType::TTD);
  const auto v2 = network.add_vertex("v2", VertexType::TTD);

  const auto v0_v1 = network.add_edge(v0, v1, 490, 50, true);
  const auto v1_v2 = network.add_edge(v1, v2, 600, 50, true);

  network.add_successor(v0_v1, v1_v2);

  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 100, 50, 4, 2, true, 0, 50, v0,
                                       198, 50, v2, network);

  RouteMap                                                    routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);
  cda_rail::simulator::GreedySimulator simulator(instance, {});

  simulator.set_train_edges_of_tr(tr1, {v0_v1});
  simulator.set_vertex_orders_of_vertex(v0, {tr1});

  const auto sim_res = simulator.simulate(5.0, true, true, true);
  ASSERT_EQ(sim_res.exit_times.size(), 1);
  PLOGD << "Simulation success: " << (sim_res.success ? "true" : "false")
        << ", Objective value: " << sim_res.exit_times.at(0);
  // no stopping but instantaneous stop at route end (overshooting to 500 after
  // 10s, still stopping at 490 after 10s)
  EXPECT_TRUE(sim_res.success);
  EXPECT_EQ(sim_res.exit_times.at(0), 10.0);
  ASSERT_EQ(sim_res.train_trajectories.size(), 1);
  ASSERT_TRUE(sim_res.train_trajectories.at(0).contains(10.0));
  EXPECT_EQ(sim_res.train_trajectories.at(0).at(10.0).pos, 490.0);
  EXPECT_EQ(sim_res.train_trajectories.at(0).at(10.0).vel, 0);
}

TEST(GreedySimulator, OneStationTest) {
  static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
  plog::init(plog::verbose, &console_appender);

  Network    network;
  const auto v0 = network.add_vertex("v0", VertexType::TTD, 60);
  const auto v1 = network.add_vertex("v1", VertexType::TTD);
  const auto v2 = network.add_vertex("v2", VertexType::TTD);
  const auto v3 = network.add_vertex("v3", VertexType::TTD, 30);

  const auto v0_v1 = network.add_edge(v0, v1, 500, 50, true);
  const auto v1_v0 = network.add_edge(v1, v0, 500, 50, true);
  const auto v1_v2 = network.add_edge(v1, v2, 600, 50, true);
  const auto v2_v1 = network.add_edge(v2, v1, 600, 50, true);
  const auto v2_v3 = network.add_edge(v2, v3, 1000, 50, true);
  const auto v3_v2 = network.add_edge(v3, v2, 1000, 50, true);

  network.add_successor(v0_v1, v1_v2);
  network.add_successor(v1_v2, v2_v3);
  network.add_successor(v3_v2, v2_v1);
  network.add_successor(v2_v1, v1_v0);

  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 100, 50, 4, 2, true, 0, 15, v0,
                                       198, 40, v3, network);
  timetable.add_empty_station("Station1");
  timetable.add_track_to_station("Station1", v1_v2, network);
  timetable.insert_stop(tr1, "Station1", 10, 30);

  RouteMap                                                    routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);
  cda_rail::simulator::GreedySimulator simulator(instance, {});

  simulator.set_train_edges_of_tr(tr1, {v0_v1, v1_v2});
  simulator.append_current_stop_position_of_tr(tr1);
  simulator.set_vertex_orders_of_vertex(v0, {tr1});

  const auto sim_res = simulator.simulate(6.0, false, true);
  ASSERT_EQ(sim_res.exit_times.size(), 1);
  PLOGD << "Simulation success: " << (sim_res.success ? "true" : "false")
        << ", Objective value: " << sim_res.exit_times.at(0);
  const auto time1 = cda_rail::min_travel_time(15, 0, 50, 4, 2, 1100);
  EXPECT_TRUE(sim_res.success);
  EXPECT_GE(sim_res.exit_times.at(0), time1 + 30 - 3);
  EXPECT_LE(sim_res.exit_times.at(0), time1 + 30 + 6);
  ASSERT_EQ(sim_res.stop_times.size(), 1);
  EXPECT_EQ(sim_res.stop_times.at(tr1).size(), 1);
  ASSERT_EQ(sim_res.vertex_headways.size(), 4);
  EXPECT_EQ(sim_res.vertex_headways.at(v0), 60);
  EXPECT_EQ(sim_res.vertex_headways.at(v1), 0);
  EXPECT_EQ(sim_res.vertex_headways.at(v2), 0);
  EXPECT_EQ(sim_res.vertex_headways.at(v3), 0); // Train does not reach v3
}

TEST(GreedySimulator, TwoStationTest) {
  static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
  plog::init(plog::verbose, &console_appender);

  Network    network;
  const auto v0 = network.add_vertex("v0", VertexType::TTD, 60);
  const auto v1 = network.add_vertex("v1", VertexType::TTD);
  const auto v2 = network.add_vertex("v2", VertexType::TTD);
  const auto v3 = network.add_vertex("v3", VertexType::TTD);
  const auto v4 = network.add_vertex("v4", VertexType::TTD, 30);

  const auto v0_v1 = network.add_edge(v0, v1, 500, 50, true);
  const auto v1_v2 = network.add_edge(v1, v2, 600, 50, true);
  const auto v2_v3 = network.add_edge(v2, v3, 1000, 50, true);
  const auto v3_v4 = network.add_edge(v3, v4, 500, 50, true);

  network.add_successor(v0_v1, v1_v2);
  network.add_successor(v1_v2, v2_v3);
  network.add_successor(v2_v3, v3_v4);

  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 100, 50, 4, 2, true, 0, 15, v0,
                                       300, 40, v4, network);
  timetable.add_empty_station("Station1");
  timetable.add_track_to_station("Station1", v1_v2, network);
  timetable.insert_stop(tr1, "Station1", 60, 30);
  timetable.add_empty_station("Station2");
  timetable.add_track_to_station("Station2", v2_v3, network);
  timetable.insert_stop(tr1, "Station2", 90, 60);

  RouteMap                                                    routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);
  cda_rail::simulator::GreedySimulator simulator(instance, {});

  simulator.set_train_edges_of_tr(tr1, {v0_v1, v1_v2, v2_v3});
  simulator.append_stop_edge_to_tr(tr1, v1_v2);
  simulator.append_stop_edge_to_tr(tr1, v2_v3);
  simulator.set_vertex_orders_of_vertex(v0, {tr1});

  const auto sim_res = simulator.simulate(6.0, false, true);
  ASSERT_EQ(sim_res.exit_times.size(), 1);
  PLOGD << "Simulation success: " << (sim_res.success ? "true" : "false")
        << ", Objective value: " << sim_res.exit_times.at(0);
  const auto time1 = cda_rail::min_travel_time(0, 0, 50, 4, 2, 1000);
  EXPECT_TRUE(sim_res.success);
  EXPECT_GE(sim_res.exit_times.at(0), 90 + time1 + 60 - 3);
  EXPECT_LE(sim_res.exit_times.at(0), 90 + time1 + 60 + 6);
  ASSERT_EQ(sim_res.stop_times.size(), 1);
  EXPECT_EQ(sim_res.stop_times.at(tr1).size(), 2);
  ASSERT_EQ(sim_res.vertex_headways.size(), 5);
  EXPECT_EQ(sim_res.vertex_headways.at(v0), 60);
  EXPECT_EQ(sim_res.vertex_headways.at(v1), 0);
  EXPECT_EQ(sim_res.vertex_headways.at(v2), 0);
  EXPECT_EQ(sim_res.vertex_headways.at(v3), 0);
  EXPECT_EQ(sim_res.vertex_headways.at(v4), 0); // Train does not reach v4
}

TEST(GreedySimulator, TrajectorySavingTest) {
  static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
  plog::init(plog::verbose, &console_appender);

  Network    network;
  const auto v0    = network.add_vertex("v0", VertexType::TTD, 60);
  const auto v1    = network.add_vertex("v1", VertexType::TTD, 30);
  const auto v0_v1 = network.add_edge(v0, v1, 1100, 50, true);

  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 100, 50, 4, 2, true, 0, 15, v0,
                                       198, 40, v1, network);

  RouteMap                                                    routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);
  cda_rail::simulator::GreedySimulator simulator(instance, {});

  simulator.set_train_edges_of_tr(tr1, {v0_v1});
  simulator.set_vertex_orders_of_vertex(v0, {tr1});

  const auto sim_res = simulator.simulate(6.0, false, true, true);
  EXPECT_TRUE(sim_res.success);
  ASSERT_EQ(sim_res.train_trajectories.size(), 1);
  EXPECT_FALSE(sim_res.train_trajectories.at(tr1).empty());
}

TEST(GreedySimulator, TwoStationTestWithExit) {
  static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
  plog::init(plog::verbose, &console_appender);

  Network    network;
  const auto v0 = network.add_vertex("v0", VertexType::TTD, 60);
  const auto v1 = network.add_vertex("v1", VertexType::TTD);
  const auto v2 = network.add_vertex("v2", VertexType::TTD);
  const auto v3 = network.add_vertex("v3", VertexType::TTD);
  const auto v4 = network.add_vertex("v4", VertexType::TTD, 30);

  const auto v0_v1 = network.add_edge(v0, v1, 500, 50, true);
  const auto v1_v2 = network.add_edge(v1, v2, 600, 50, true);
  const auto v2_v3 = network.add_edge(v2, v3, 1000, 50, true);
  const auto v3_v4 = network.add_edge(v3, v4, 500, 50, true);

  network.add_successor(v0_v1, v1_v2);
  network.add_successor(v1_v2, v2_v3);
  network.add_successor(v2_v3, v3_v4);

  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 100, 50, 4, 2, true, 0, 15, v0,
                                       300, 40, v4, network);
  timetable.add_empty_station("Station1");
  timetable.add_track_to_station("Station1", v1_v2, network);
  timetable.insert_stop(tr1, "Station1", 60, 30);
  timetable.add_empty_station("Station2");
  timetable.add_track_to_station("Station2", v2_v3, network);
  timetable.insert_stop(tr1, "Station2", 90, 60);

  RouteMap                                                    routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);
  cda_rail::simulator::GreedySimulator simulator(instance, {});

  simulator.set_train_edges_of_tr(tr1, {v0_v1, v1_v2, v2_v3, v3_v4});
  simulator.append_stop_edge_to_tr(tr1, v1_v2);
  simulator.append_stop_edge_to_tr(tr1, v2_v3);
  simulator.set_vertex_orders_of_vertex(v0, {tr1});

  const auto sim_res = simulator.simulate(6.0, false, true);
  ASSERT_EQ(sim_res.exit_times.size(), 1);
  PLOGD << "Simulation success: " << (sim_res.success ? "true" : "false")
        << ", Objective value: " << sim_res.exit_times.at(0);
  const auto time1 = cda_rail::min_travel_time(0, 0, 50, 4, 2, 1000);
  const auto time2 = cda_rail::min_travel_time(0, 40, 50, 4, 2, 600);
  EXPECT_TRUE(sim_res.success);
  EXPECT_GE(sim_res.exit_times.at(0), 300);
  EXPECT_LE(sim_res.exit_times.at(0), 306);
  ASSERT_EQ(sim_res.vertex_headways.size(), 5);
  EXPECT_EQ(sim_res.vertex_headways.at(v0), 60);
  EXPECT_EQ(sim_res.vertex_headways.at(v1), 0);
  EXPECT_EQ(sim_res.vertex_headways.at(v2), 0);
  EXPECT_EQ(sim_res.vertex_headways.at(v3), 0);
  EXPECT_EQ(sim_res.vertex_headways.at(v4), sim_res.exit_times.at(0) + 30);
}

TEST(GreedySimulation, TightEntry) {
  static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
  plog::init(plog::verbose, &console_appender);

  Network    network;
  const auto v0 = network.add_vertex("v0", VertexType::TTD, 6);
  const auto v1 = network.add_vertex("v1", VertexType::TTD, 30);

  const auto v0_v1 = network.add_edge(v0, v1, 5000, 50, true);
  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 100, 15, 4, 2, true, 0, 15, v0,
                                       198, 15, v1, network);
  const auto tr2 = timetable.add_train("Train2", 100, 15, 4, 0.5, true, 12, 15,
                                       v0, 198, 15, v1, network);
  RouteMap   routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);
  cda_rail::simulator::GreedySimulator simulator(instance, {});

  simulator.set_train_edges_of_tr(tr1, {v0_v1});
  simulator.set_train_edges_of_tr(tr2, {v0_v1});
  simulator.set_vertex_orders_of_vertex(v0, {tr1, tr2});
  simulator.set_vertex_orders_of_vertex(v1, {tr1, tr2});

  const auto sim_res = simulator.simulate(6.0, false, true);
  ASSERT_FALSE(sim_res.exit_times.empty());
  PLOGD << "Simulation success: " << (sim_res.success ? "true" : "false")
        << ", Objective value: " << sim_res.exit_times.back() << std::endl;

  EXPECT_FALSE(sim_res.success);
  ASSERT_EQ(sim_res.exit_times.size(), 2);
  EXPECT_EQ(sim_res.exit_times[0], 0);
  EXPECT_EQ(sim_res.exit_times[1], 0);
  ASSERT_EQ(sim_res.vertex_headways.size(), 2);
  EXPECT_EQ(sim_res.vertex_headways.at(v0), 6);
  EXPECT_EQ(sim_res.vertex_headways.at(v1), 0);
}

TEST(GreedySimulation, ExitNetworkSpeedZero) {
  static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
  plog::init(plog::verbose, &console_appender);

  Network    network;
  const auto v0 = network.add_vertex("v0", VertexType::TTD, 60);
  const auto v1 = network.add_vertex("v1", VertexType::TTD, 30);

  const auto v0_v1 = network.add_edge(v0, v1, 5000, 50, true);
  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 100, 50, 4, 2, true, 0, 15, v0,
                                       0, 0, v1, network);
  RouteMap   routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);
  cda_rail::simulator::GreedySimulator simulator(instance, {});

  simulator.set_train_edges_of_tr(tr1, {v0_v1});
  simulator.set_vertex_orders_of_vertex(v0, {tr1});
  simulator.set_vertex_orders_of_vertex(v1, {tr1});

  const auto sim_res = simulator.simulate(6.0, false, true);
  ASSERT_FALSE(sim_res.exit_times.empty());
  PLOGD << "Simulation success: " << (sim_res.success ? "true" : "false")
        << ", Objective value: " << sim_res.exit_times.back() << std::endl;

  const auto time1 = cda_rail::min_travel_time(15, 0, 50, 4, 2, 5100);
  EXPECT_TRUE(sim_res.success);
  ASSERT_EQ(sim_res.exit_times.size(), 1);
  EXPECT_GE(sim_res.exit_times[0], time1 - 3);
  EXPECT_LE(sim_res.exit_times[0], time1 + 6);
  ASSERT_EQ(sim_res.vertex_headways.size(), 2);
  EXPECT_EQ(sim_res.vertex_headways.at(v0), 60);
  EXPECT_EQ(sim_res.vertex_headways.at(v1), sim_res.exit_times[0] + 30);
}

TEST(GreedySimulation, SimpleNetwork) {
  static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
  plog::init(plog::verbose, &console_appender);

  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      "GeneralSimpleNetworkB6Trains", "gen-po", "./data");

  const auto v2c_v3 =
      instance.get_const_network().get_edge_index({"v2c"}, {"v3"});
  const auto v2b_v3 =
      instance.get_const_network().get_edge_index({"v2b"}, {"v3"});
  const auto v3_v2b =
      instance.get_const_network().get_edge_index({"v3"}, {"v2b"});
  const auto v3_v2a =
      instance.get_const_network().get_edge_index({"v3"}, {"v2a"});
  const auto v3_v4 =
      instance.get_const_network().get_edge_index({"v3"}, {"v4"});
  const auto v4_v3 =
      instance.get_const_network().get_edge_index({"v4"}, {"v3"});

  const auto v5_v6 =
      instance.get_const_network().get_edge_index({"v5"}, {"v6"});
  const auto v6_v5 =
      instance.get_const_network().get_edge_index({"v6"}, {"v5"});
  const auto v6_v7a =
      instance.get_const_network().get_edge_index({"v6"}, {"v7a"});
  const auto v6_v7b =
      instance.get_const_network().get_edge_index({"v6"}, {"v7b"});
  const auto v7a_v6 =
      instance.get_const_network().get_edge_index({"v7a"}, {"v6"});
  const auto v7b_v6 =
      instance.get_const_network().get_edge_index({"v7b"}, {"v6"});

  const auto v8a_v9 =
      instance.get_const_network().get_edge_index({"v8a"}, {"v9"});
  const auto v8b_v9 =
      instance.get_const_network().get_edge_index({"v8b"}, {"v9"});
  const auto v9_v8a =
      instance.get_const_network().get_edge_index({"v9"}, {"v8a"});
  const auto v9_v8b =
      instance.get_const_network().get_edge_index({"v9"}, {"v8b"});
  const auto v9_v10 =
      instance.get_const_network().get_edge_index({"v9"}, {"v10"});
  const auto v10_v9 =
      instance.get_const_network().get_edge_index({"v10"}, {"v9"});

  const auto v11_v12 =
      instance.get_const_network().get_edge_index({"v11"}, {"v12"});
  const auto v12_v11 =
      instance.get_const_network().get_edge_index({"v12"}, {"v11"});
  const auto v12_v13c =
      instance.get_const_network().get_edge_index({"v12"}, {"v13c"});
  const auto v12_v13b =
      instance.get_const_network().get_edge_index({"v12"}, {"v13b"});
  const auto v13b_v12 =
      instance.get_const_network().get_edge_index({"v13b"}, {"v12"});
  const auto v13a_v12 =
      instance.get_const_network().get_edge_index({"v13a"}, {"v12"});

  cda_rail::simulator::GreedySimulator simulator(
      instance, {{v2c_v3, v2b_v3, v3_v2b, v3_v2a, v3_v4, v4_v3},
                 {v5_v6, v6_v5, v6_v7a, v6_v7b, v7a_v6, v7b_v6},
                 {v8a_v9, v8b_v9, v9_v8a, v9_v8b, v9_v10, v10_v9},
                 {v11_v12, v12_v11, v12_v13c, v12_v13b, v13b_v12, v13a_v12}});

  const auto v2a_v1a =
      instance.get_const_network().get_edge_index({"v2a"}, {"v1a"});
  const auto v2b_v1b =
      instance.get_const_network().get_edge_index({"v2b"}, {"v1b"});
  const auto v1b_v2b =
      instance.get_const_network().get_edge_index({"v1b"}, {"v2b"});
  const auto v1c_v2c =
      instance.get_const_network().get_edge_index({"v1c"}, {"v2c"});

  const auto v4_v5 =
      instance.get_const_network().get_edge_index({"v4"}, {"v5"});
  const auto v5_v4 =
      instance.get_const_network().get_edge_index({"v5"}, {"v4"});
  const auto v7a_v8a =
      instance.get_const_network().get_edge_index({"v7a"}, {"v8a"});
  const auto v7b_v8b =
      instance.get_const_network().get_edge_index({"v7b"}, {"v8b"});
  const auto v8a_v7a =
      instance.get_const_network().get_edge_index({"v8a"}, {"v7a"});
  const auto v8b_v7b =
      instance.get_const_network().get_edge_index({"v8b"}, {"v7b"});
  const auto v10_v11 =
      instance.get_const_network().get_edge_index({"v10"}, {"v11"});
  const auto v11_v10 =
      instance.get_const_network().get_edge_index({"v11"}, {"v10"});

  const auto v13c_v14c =
      instance.get_const_network().get_edge_index({"v13c"}, {"v14c"});
  const auto v13b_v14b =
      instance.get_const_network().get_edge_index({"v13b"}, {"v14b"});
  const auto v14b_v13b =
      instance.get_const_network().get_edge_index({"v14b"}, {"v13b"});
  const auto v14a_v13a =
      instance.get_const_network().get_edge_index({"v14a"}, {"v13a"});

  const auto tr00 = instance.get_const_train_list().get_train_index("Train0_0");
  const auto tr01 = instance.get_const_train_list().get_train_index("Train0_1");
  const auto tr02 = instance.get_const_train_list().get_train_index("Train0_2");
  const auto tr10 = instance.get_const_train_list().get_train_index("Train1_0");
  const auto tr11 = instance.get_const_train_list().get_train_index("Train1_1");
  const auto tr12 = instance.get_const_train_list().get_train_index("Train1_2");

  const auto v1a  = instance.get_const_network().get_vertex_index("v1a");
  const auto v1b  = instance.get_const_network().get_vertex_index("v1b");
  const auto v1c  = instance.get_const_network().get_vertex_index("v1c");
  const auto v14a = instance.get_const_network().get_vertex_index("v14a");
  const auto v14b = instance.get_const_network().get_vertex_index("v14b");
  const auto v14c = instance.get_const_network().get_vertex_index("v14c");

  simulator.set_train_edges_of_tr(tr00, {v14a_v13a});
  simulator.set_vertex_orders_of_vertex(v14a, {tr00});
  const auto sim_res_init = simulator.simulate(6.0, true, true);
  EXPECT_TRUE(sim_res_init.success);

  simulator.set_train_edges_of_tr(tr00, {v14a_v13a, v13a_v12, v12_v11, v11_v10,
                                         v10_v9, v9_v8a, v8a_v7a, v7a_v6, v6_v5,
                                         v5_v4, v4_v3, v3_v2a, v2a_v1a});
  simulator.set_train_edges_of_tr(
      tr01, {v1c_v2c, v2c_v3, v3_v4, v4_v5, v5_v6, v6_v7b, v7b_v8b, v8b_v9,
             v9_v10, v10_v11, v11_v12, v12_v13c, v13c_v14c});
  simulator.set_train_edges_of_tr(
      tr02, {v1c_v2c, v2c_v3, v3_v4, v4_v5, v5_v6, v6_v7b, v7b_v8b, v8b_v9,
             v9_v10, v10_v11, v11_v12, v12_v13c, v13c_v14c});
  simulator.set_train_edges_of_tr(
      tr10, {v1b_v2b, v2b_v3, v3_v4, v4_v5, v5_v6, v6_v7b, v7b_v8b, v8b_v9,
             v9_v10, v10_v11, v11_v12, v12_v13b, v13b_v14b});
  simulator.set_train_edges_of_tr(tr11, {v14b_v13b, v13b_v12, v12_v11, v11_v10,
                                         v10_v9, v9_v8a, v8a_v7a, v7a_v6, v6_v5,
                                         v5_v4, v4_v3, v3_v2b, v2b_v1b});
  simulator.set_train_edges_of_tr(tr12, {v14b_v13b, v13b_v12, v12_v11, v11_v10,
                                         v10_v9, v9_v8a, v8a_v7a, v7a_v6, v6_v5,
                                         v5_v4, v4_v3, v3_v2b, v2b_v1b});

  simulator.append_stop_edge_to_tr(tr00, v14a_v13a);
  simulator.append_stop_edge_to_tr(tr00, v2a_v1a);
  simulator.append_stop_edge_to_tr(tr01, v1c_v2c);
  simulator.append_stop_edge_to_tr(tr01, v13c_v14c);
  simulator.append_stop_edge_to_tr(tr02, v1c_v2c);
  simulator.append_stop_edge_to_tr(tr02, v13c_v14c);

  simulator.set_vertex_orders_of_vertex(v1a, {tr00});
  simulator.set_vertex_orders_of_vertex(v1b, {tr10, tr11, tr12});
  simulator.set_vertex_orders_of_vertex(v1c, {tr01, tr02});
  simulator.set_vertex_orders_of_vertex(v14a, {tr00});
  simulator.set_vertex_orders_of_vertex(v14b, {tr11, tr12, tr10});
  simulator.set_vertex_orders_of_vertex(v14c, {tr01, tr02});

  simulator.set_ttd_orders_of_ttd(0, {tr01, tr02, tr10, tr00, tr11, tr12});
  simulator.set_ttd_orders_of_ttd(1, {tr01, tr02, tr10, tr00, tr11, tr12});
  simulator.set_ttd_orders_of_ttd(2, {tr00, tr11, tr12, tr01, tr02, tr10});
  simulator.set_ttd_orders_of_ttd(3, {tr00, tr11, tr12, tr01, tr02, tr10});

  const auto sim_res = simulator.simulate(6.0, true, true);
  PLOGD << "Simulation success: " << (sim_res.success ? "true" : "false");
  for (size_t tr = 0; tr < instance.get_const_train_list().size(); ++tr) {
    const auto& tr_name =
        instance.get_const_train_list().get_train(tr).get_name();
    PLOGD << "Objective value for train " << tr_name << ": "
          << sim_res.exit_times.at(tr);
  }
  PLOGD << "Vertex headways at v1a: " << sim_res.vertex_headways.at(v1a)
        << ", v1b: " << sim_res.vertex_headways.at(v1b)
        << ", v1c: " << sim_res.vertex_headways.at(v1c)
        << ", v14a: " << sim_res.vertex_headways.at(v14a)
        << ", v14b: " << sim_res.vertex_headways.at(v14b)
        << ", v14c: " << sim_res.vertex_headways.at(v14c);

  EXPECT_TRUE(sim_res.success);
  EXPECT_EQ(sim_res.vertex_headways.at(v1a), sim_res.exit_times.at(tr00) + 60);
  EXPECT_EQ(sim_res.vertex_headways.at(v1b), sim_res.exit_times.at(tr12) + 60);
  EXPECT_GE(sim_res.vertex_headways.at(v1c), 60 + 60);
  EXPECT_EQ(sim_res.vertex_headways.at(v14a), 60);
  EXPECT_EQ(sim_res.vertex_headways.at(v14b), sim_res.exit_times.at(tr10) + 60);
  EXPECT_EQ(sim_res.vertex_headways.at(v14c), sim_res.exit_times.at(tr02) + 60);
  EXPECT_GE(sim_res.exit_times.at(tr00), 900);
  EXPECT_LE(sim_res.exit_times.at(tr00), 1950);
  EXPECT_GE(sim_res.exit_times.at(tr01), 900);
  EXPECT_LE(sim_res.exit_times.at(tr01), 1950);
  EXPECT_GE(sim_res.exit_times.at(tr02), 900);
  EXPECT_LE(sim_res.exit_times.at(tr02), 1950);
  EXPECT_GE(sim_res.exit_times.at(tr10), 1900);
  EXPECT_LE(sim_res.exit_times.at(tr10), 3450);
  EXPECT_GE(sim_res.exit_times.at(tr11), 1900);
  EXPECT_LE(sim_res.exit_times.at(tr11), 3450);
  EXPECT_GE(sim_res.exit_times.at(tr12), 1900);
  EXPECT_LE(sim_res.exit_times.at(tr12), 3450);
}

TEST(GreedySimulation, FinalState) {
  Network    network;
  const auto v0  = network.add_vertex("v0", VertexType::TTD);
  const auto v1  = network.add_vertex("v1", VertexType::TTD);
  const auto v2  = network.add_vertex("v2", VertexType::TTD);
  const auto v3a = network.add_vertex("v3a", VertexType::TTD);
  const auto v3b = network.add_vertex("v3b", VertexType::TTD);

  const auto v0_v1  = network.add_edge(v0, v1, 1000, 50, true);
  const auto v1_v2  = network.add_edge(v1, v2, 1000, 50, true);
  const auto v2_v3a = network.add_edge(v2, v3a, 1000, 50, true);
  const auto v2_v3b = network.add_edge(v2, v3b, 1000, 50, true);
  network.add_successor(v0_v1, v1_v2);
  network.add_successor(v1_v2, v2_v3a);
  network.add_successor(v1_v2, v2_v3b);

  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 100, 50, 4, 2, true, 0, 15, v0,
                                       198, 40, v3a, network);
  const auto tr2 = timetable.add_train("Train2", 100, 50, 4, 2, true, 0, 15, v0,
                                       198, 40, v3b, network);

  timetable.add_empty_station("Station1");
  timetable.add_track_to_station("Station1", v1_v2, network);
  timetable.insert_stop(tr1, "Station1", 10, 30);

  RouteMap                                                    routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);
  cda_rail::simulator::GreedySimulator simulator(instance, {});

  EXPECT_FALSE(simulator.is_final_state());

  simulator.set_train_edges_of_tr(tr1, {v0_v1, v1_v2, v2_v3a});
  simulator.set_train_edges_of_tr(tr2, {v0_v1, v1_v2, v2_v3b});

  EXPECT_FALSE(simulator.is_final_state());

  simulator.append_stop_edge_to_tr(tr1, v1_v2);

  EXPECT_TRUE(simulator.is_final_state());

  simulator.set_train_edges_of_tr(tr2, {});

  EXPECT_FALSE(simulator.is_final_state());

  simulator.set_train_edges_of_tr(tr2, {v0_v1, v1_v2});

  EXPECT_FALSE(simulator.is_final_state());

  simulator.append_train_edge_to_tr(tr2, v2_v3b);

  EXPECT_TRUE(simulator.is_final_state());
}

TEST(GreedySimulation, ExitEntryZero) {
  Network    network;
  const auto v0 = network.add_vertex("v0", VertexType::TTD, 60);
  const auto v1 = network.add_vertex("v1", VertexType::TTD, 30);

  const auto v0_v1 = network.add_edge(v0, v1, 2000, 20, true);
  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 100, 20, 4, 2, true, 0, 0, v0,
                                       100, 0, v1, network);
  RouteMap   routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);
  cda_rail::simulator::GreedySimulator simulator(instance, {});
  simulator.set_train_edges_of_tr(tr1, {v0_v1});
  simulator.set_vertex_orders_of_vertex(v0, {tr1});
  simulator.set_vertex_orders_of_vertex(v1, {tr1});

  const auto sim_res = simulator.simulate(6.0, false, true);
  const auto time1   = cda_rail::min_travel_time(0, 0, 20, 4, 2, 2100);
  ASSERT_FALSE(sim_res.exit_times.empty());
  PLOGD << "Simulation success: " << (sim_res.success ? "true" : "false")
        << ", Objective value: " << sim_res.exit_times.back()
        << ", expected: " << time1;
  EXPECT_TRUE(sim_res.success);
  ASSERT_EQ(sim_res.exit_times.size(), 1);
  EXPECT_GE(sim_res.exit_times.at(0), time1 - 3);
  EXPECT_LE(sim_res.exit_times.at(0), time1 + 6);
  ASSERT_EQ(sim_res.vertex_headways.size(), 2);
  EXPECT_EQ(sim_res.vertex_headways.at(v0), 60);
  EXPECT_EQ(sim_res.vertex_headways.at(v1), sim_res.exit_times.at(0) + 30);
}

TEST(GreedySimulation, FaultyInstance) {
  static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
  plog::init(plog::verbose, &console_appender);

  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      "GeneralSimpleNetworkB6Trains", "gen-po", "./data");
  const auto ttd_sections = instance.get_const_network().unbreakable_sections();
  cda_rail::simulator::GreedySimulator simulator(instance, ttd_sections);

  simulator.set_train_edges_of_tr(
      0, {21, 23, 24, 25, 26, 27, 29, 31, 33, 34, 35, 37, 39});
  simulator.set_train_edges_of_tr(
      2, {1, 3, 4, 5, 6, 7, 9, 11, 13, 14, 15, 17, 19});
  simulator.set_train_edges_of_tr(3, {0, 2, 4, 5, 6, 7, 9, 11, 13, 14});
  simulator.set_train_edges_of_tr(
      5, {20, 22, 24, 25, 26, 27, 29, 31, 33, 34, 35, 36, 38});
  simulator.set_ttd_orders_of_ttd(0, {0, 2, 5, 3});
  simulator.set_ttd_orders_of_ttd(1, {0, 2, 3, 5});
  simulator.set_ttd_orders_of_ttd(2, {0, 2, 5});
  simulator.set_ttd_orders_of_ttd(3, {0, 2, 3, 5});
  simulator.set_vertex_orders_of_vertex(1, {3, 5});
  simulator.set_vertex_orders_of_vertex(2, {2});
  simulator.set_vertex_orders_of_vertex(21, {0});
  simulator.set_vertex_orders_of_vertex(22, {5});
  simulator.set_stop_positions_of_tr(0, {475});
  simulator.set_stop_positions_of_tr(2, {475});

  EXPECT_NO_THROW((void)simulator.simulate(6.0, false, true));
}

TEST(GreedySimulation, TimeExtractions) {
  // Train enters at speed 24 travels for 18 seconds -> t = 18
  // position = 18 * 24 = 432

  // After 12 seconds: 12 * 24 = 288

  // then decelerating at rate 2 for 12 seconds -> t = 18+12 = 30
  // position = 432 + 24*24/4 = 432 + 144 = 576

  // stopping for 60 seconds -> t = 30 + 60 = 90

  // then accelerating at rate 1 for 12 seconds -> t = 90+12 = 102
  // position = 576 + 12*6 = 576 + 72 = 648

  // then decelerating at rate 2 for 6 seconds -> t = 102 + 6 = 108
  // position = 648 + 12*3 = 648 + 36 = 684

  // sopping for 30 seconds -> t = 138

  // accelerating at rate 1 for 18 seconds -> t = 138 + 18 = 156
  // position = 684 + 18*9 = 684 + 162 = 846

  static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
  plog::init(plog::verbose, &console_appender);

  Network    network;
  const auto v0 = network.add_vertex("v0", VertexType::TTD, 60);
  const auto v1 = network.add_vertex("v1", VertexType::TTD);
  const auto v2 = network.add_vertex("v2", VertexType::TTD);
  const auto v3 = network.add_vertex("v3", VertexType::TTD);
  const auto v4 = network.add_vertex("v4", VertexType::TTD, 30);

  ASSERT_LE(v0, 4);
  ASSERT_LE(v1, 4);
  ASSERT_LE(v2, 4);
  ASSERT_LE(v3, 4);
  ASSERT_LE(v4, 4);

  const auto v0_v1 = network.add_edge(v0, v1, 288, 30, true);
  const auto v1_v2 = network.add_edge(v1, v2, 576 - 288, 30, true);
  const auto v2_v3 = network.add_edge(v2, v3, 684 - 576, 30, true);
  const auto v3_v4 =
      network.add_edge(v3, v4, 846 - 684 - 50, 30, true); // assuming 50m train

  network.add_successor(v0_v1, v1_v2);
  network.add_successor(v1_v2, v2_v3);
  network.add_successor(v2_v3, v3_v4);

  Timetable timetable;
  timetable.add_empty_station("Station1");
  timetable.add_track_to_station("Station1", v1_v2, network);
  timetable.add_empty_station("Station2");
  timetable.add_track_to_station("Station2", v2_v3, network);

  const auto tr1 = timetable.add_train("Train1", 50, 24, 1, 2, true, 0, 24, v0,
                                       100, 18, v4, network);
  timetable.insert_stop(tr1, "Station1", 0, 60);
  timetable.insert_stop(tr1, "Station2", 90, 30);

  // Train 2 is exact copy of train 1 shifted by 100 seconds
  const auto tr2 = timetable.add_train("Train2", 50, 24, 1, 2, true, 0 + 102,
                                       24, v0, 100 + 102, 18, v4, network);
  timetable.insert_stop(tr2, "Station1", 102, 60);
  timetable.insert_stop(tr2, "Station2", 90 + 102, 30);

  ASSERT_LE(tr1, 1);
  ASSERT_LE(tr2, 1);

  RouteMap                                                    routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);
  cda_rail::simulator::GreedySimulator simulator(instance, {});

  // After 12 seconds: 12*24 = 288

  simulator.set_train_edges_of_tr(tr1, {v0_v1});
  simulator.set_vertex_orders_of_vertex(v0, {tr1});

  PLOGD << "-------------------------------------------------------------------"
           "------------";
  PLOGD << "STARTING SIMULATION 1";
  PLOGD << "-------------------------------------------------------------------"
           "------------";

  const auto sim_res = simulator.simulate(6.0, false, true);
  EXPECT_TRUE(sim_res.success);
  ASSERT_EQ(sim_res.exit_times.size(), 2);
  EXPECT_EQ(sim_res.exit_times.at(tr1), 12);
  EXPECT_EQ(sim_res.exit_times.at(tr2), 0);
  ASSERT_EQ(sim_res.stop_times.size(), 2);
  ASSERT_TRUE(sim_res.stop_times.at(tr1).empty());
  ASSERT_TRUE(sim_res.stop_times.at(tr2).empty());
  ASSERT_EQ(sim_res.vertex_headways.size(), 5);
  EXPECT_EQ(sim_res.vertex_headways.at(v0), 60);
  EXPECT_EQ(sim_res.vertex_headways.at(v1), 0);
  EXPECT_EQ(sim_res.vertex_headways.at(v2), 0);
  EXPECT_EQ(sim_res.vertex_headways.at(v3), 0);
  EXPECT_EQ(sim_res.vertex_headways.at(v4), 0);

  // Train enters at speed 24 travels for 18 seconds -> t = 18
  // position = 18 * 24 = 432

  // then decelerating at rate 2 for 12 seconds -> t = 18+12 = 30
  // position = 432 + 24*24/4 = 432 + 144 = 576

  PLOGD << "";
  PLOGD << "-------------------------------------------------------------------"
           "------------";
  PLOGD << "STARTING SIMULATION 2";
  PLOGD << "-------------------------------------------------------------------"
           "------------";

  // 576 / 24 = 24

  simulator.append_train_edge_to_tr(tr1, v1_v2);
  const auto sim_res2 = simulator.simulate(6.0, false, true);
  EXPECT_TRUE(sim_res2.success);
  ASSERT_EQ(sim_res2.exit_times.size(), 2);
  EXPECT_EQ(sim_res2.exit_times.at(tr1), 24);
  EXPECT_EQ(sim_res2.exit_times.at(tr2), 0);
  ASSERT_EQ(sim_res2.stop_times.size(), 2);
  ASSERT_TRUE(sim_res2.stop_times.at(tr1).empty());
  ASSERT_TRUE(sim_res2.stop_times.at(tr2).empty());
  ASSERT_EQ(sim_res2.vertex_headways.size(), 5);
  EXPECT_EQ(sim_res2.vertex_headways.at(v0), 60);
  EXPECT_EQ(sim_res2.vertex_headways.at(v1), 0);
  EXPECT_EQ(sim_res2.vertex_headways.at(v2), 0);
  EXPECT_EQ(sim_res2.vertex_headways.at(v3), 0);
  EXPECT_EQ(sim_res2.vertex_headways.at(v4), 0);

  // stopping for 60 seconds -> t = 30 + 60 = 90

  PLOGD << "";
  PLOGD << "-------------------------------------------------------------------"
           "------------";
  PLOGD << "STARTING SIMULATION 3";
  PLOGD << "-------------------------------------------------------------------"
           "------------";

  simulator.append_current_stop_position_of_tr(tr1);
  const auto sim_res3 = simulator.simulate(6.0, false, true);
  EXPECT_TRUE(sim_res3.success);
  ASSERT_EQ(sim_res3.exit_times.size(), 2);
  EXPECT_EQ(sim_res3.exit_times.at(tr1), 90);
  EXPECT_EQ(sim_res3.exit_times.at(tr2), 0);
  ASSERT_EQ(sim_res3.stop_times.size(), 2);
  ASSERT_EQ(sim_res3.stop_times.at(tr1).size(), 1);
  EXPECT_EQ(sim_res3.stop_times.at(tr1).at(0), 30);
  ASSERT_TRUE(sim_res3.stop_times.at(tr2).empty());
  ASSERT_EQ(sim_res3.vertex_headways.size(), 5);
  EXPECT_EQ(sim_res3.vertex_headways.at(v0), 60);
  EXPECT_EQ(sim_res3.vertex_headways.at(v1), 0);
  EXPECT_EQ(sim_res3.vertex_headways.at(v2), 0);
  EXPECT_EQ(sim_res3.vertex_headways.at(v3), 0);
  EXPECT_EQ(sim_res3.vertex_headways.at(v4), 0);

  // then accelerating at rate 1 for 12 seconds -> t = 90+12 = 102
  // position = 576 + 12*6 = 576 + 72 = 648

  // then decelerating at rate 2 for 6 seconds -> t = 102 + 6 = 108
  // position = 648 + 12*3 = 648 + 36 = 684

  PLOGD << "";
  PLOGD << "-------------------------------------------------------------------"
           "------------";
  PLOGD << "SIMULATION 4 REMOVED";
  PLOGD << "-------------------------------------------------------------------"
           "------------";

  simulator.append_train_edge_to_tr(tr1, v2_v3);

  // stopping for 30 seconds -> t = 138

  PLOGD << "";
  PLOGD << "-------------------------------------------------------------------"
           "------------";
  PLOGD << "STARTING SIMULATION 5";
  PLOGD << "-------------------------------------------------------------------"
           "------------";

  simulator.append_current_stop_position_of_tr(tr1);
  const auto sim_res5 = simulator.simulate(6.0, false, true);
  EXPECT_TRUE(sim_res5.success);
  ASSERT_EQ(sim_res5.exit_times.size(), 2);
  EXPECT_EQ(sim_res5.exit_times.at(tr1), 138);
  EXPECT_EQ(sim_res5.exit_times.at(tr2), 0);
  ASSERT_EQ(sim_res5.stop_times.size(), 2);
  ASSERT_EQ(sim_res5.stop_times.at(tr1).size(), 2);
  EXPECT_EQ(sim_res5.stop_times.at(tr1).at(0), 30);
  EXPECT_EQ(sim_res5.stop_times.at(tr1).at(1), 108);
  ASSERT_TRUE(sim_res5.stop_times.at(tr2).empty());
  ASSERT_EQ(sim_res5.vertex_headways.size(), 5);
  EXPECT_EQ(sim_res5.vertex_headways.at(v0), 60);
  EXPECT_EQ(sim_res5.vertex_headways.at(v1), 0);
  EXPECT_EQ(sim_res5.vertex_headways.at(v2), 0);
  EXPECT_EQ(sim_res5.vertex_headways.at(v3), 0);
  EXPECT_EQ(sim_res5.vertex_headways.at(v4), 0);

  // accelerating at rate 1 for 18 seconds -> t = 138 + 18 = 156
  // position = 684 + 18*9 = 684 + 162 = 846

  PLOGD << "";
  PLOGD << "-------------------------------------------------------------------"
           "------------";
  PLOGD << "STARTING SIMULATION 6";
  PLOGD << "-------------------------------------------------------------------"
           "------------";

  simulator.append_train_edge_to_tr(tr1, v3_v4);
  const auto sim_res6 = simulator.simulate(6.0, false, true);
  EXPECT_TRUE(sim_res6.success);
  ASSERT_EQ(sim_res6.exit_times.size(), 2);
  EXPECT_EQ(sim_res6.exit_times.at(tr1), 156);
  EXPECT_EQ(sim_res6.exit_times.at(tr2), 0);
  ASSERT_EQ(sim_res6.stop_times.size(), 2);
  ASSERT_EQ(sim_res6.stop_times.at(tr1).size(), 2);
  EXPECT_EQ(sim_res6.stop_times.at(tr1).at(0), 30);
  EXPECT_EQ(sim_res6.stop_times.at(tr1).at(1), 108);
  ASSERT_TRUE(sim_res6.stop_times.at(tr2).empty());
  ASSERT_EQ(sim_res6.vertex_headways.size(), 5);
  EXPECT_EQ(sim_res6.vertex_headways.at(v0), 60);
  EXPECT_EQ(sim_res6.vertex_headways.at(v1), 0);
  EXPECT_EQ(sim_res6.vertex_headways.at(v2), 0);
  EXPECT_EQ(sim_res6.vertex_headways.at(v3), 0);
  EXPECT_EQ(sim_res6.vertex_headways.at(v4), 156 + 30);

  // tr2 follows with 102s difference

  PLOGD << "";
  PLOGD << "-------------------------------------------------------------------"
           "------------";
  PLOGD << "STARTING SIMULATION 7";
  PLOGD << "-------------------------------------------------------------------"
           "------------";

  simulator.set_train_edges_of_tr(tr2, {v0_v1});
  simulator.set_vertex_orders_of_vertex(v0, {tr1, tr2});
  const auto sim_res7 = simulator.simulate(6.0, false, true);
  EXPECT_TRUE(sim_res7.success);
  ASSERT_EQ(sim_res7.exit_times.size(), 2);
  EXPECT_EQ(sim_res7.exit_times.at(tr1), 156);
  ASSERT_EQ(sim_res7.stop_times.size(), 2);
  ASSERT_EQ(sim_res7.stop_times.at(tr1).size(), 2);
  EXPECT_EQ(sim_res7.stop_times.at(tr1).at(0), 30);
  EXPECT_EQ(sim_res7.stop_times.at(tr1).at(1), 108);

  EXPECT_EQ(sim_res7.exit_times.at(tr2), 12 + 102);
  ASSERT_TRUE(sim_res7.stop_times.at(tr2).empty());

  ASSERT_EQ(sim_res7.vertex_headways.size(), 5);
  EXPECT_EQ(sim_res7.vertex_headways.at(v0), 60 + 102);
  EXPECT_EQ(sim_res7.vertex_headways.at(v1), 0);
  EXPECT_EQ(sim_res7.vertex_headways.at(v2), 0);
  EXPECT_EQ(sim_res7.vertex_headways.at(v3), 0);
  EXPECT_EQ(sim_res7.vertex_headways.at(v4), 156 + 30);

  PLOGD << "";
  PLOGD << "-------------------------------------------------------------------"
           "------------";
  PLOGD << "STARTING SIMULATION 8";
  PLOGD << "-------------------------------------------------------------------"
           "------------";

  simulator.append_train_edge_to_tr(tr2, v1_v2);
  simulator.append_current_stop_position_of_tr(tr2);
  simulator.append_train_edge_to_tr(tr2, v2_v3);
  simulator.append_current_stop_position_of_tr(tr2);
  simulator.append_train_edge_to_tr(tr2, v3_v4);
  const auto sim_res8 = simulator.simulate(6.0, false, true);
  EXPECT_TRUE(sim_res8.success);
  ASSERT_EQ(sim_res8.exit_times.size(), 2);
  EXPECT_EQ(sim_res8.exit_times.at(tr1), 156);
  ASSERT_EQ(sim_res8.stop_times.size(), 2);
  ASSERT_EQ(sim_res8.stop_times.at(tr1).size(), 2);
  EXPECT_EQ(sim_res8.stop_times.at(tr1).at(0), 30);
  EXPECT_EQ(sim_res8.stop_times.at(tr1).at(1), 108);

  EXPECT_EQ(sim_res8.exit_times.at(tr2), 156 + 102);
  ASSERT_EQ(sim_res8.stop_times.at(tr2).size(), 2);
  EXPECT_EQ(sim_res8.stop_times.at(tr2).at(0), 30 + 102);
  EXPECT_EQ(sim_res8.stop_times.at(tr2).at(1), 108 + 102);

  ASSERT_EQ(sim_res8.vertex_headways.size(), 5);
  EXPECT_EQ(sim_res8.vertex_headways.at(v0), 60 + 102);
  EXPECT_EQ(sim_res8.vertex_headways.at(v1), 0);
  EXPECT_EQ(sim_res8.vertex_headways.at(v2), 0);
  EXPECT_EQ(sim_res8.vertex_headways.at(v3), 0);
  EXPECT_EQ(sim_res8.vertex_headways.at(v4), 156 + 30 + 102);
}

TEST(GreedySimulation, DisappearOnPartialRoute) {
  static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
  plog::init(plog::verbose, &console_appender);

  Network    network;
  const auto v0 = network.add_vertex("v0", VertexType::TTD, 0);
  const auto v1 = network.add_vertex("v1", VertexType::TTD, 0);
  const auto v2 = network.add_vertex("v2", VertexType::TTD, 0);

  const auto v0_v1 = network.add_edge(v0, v1, 5000, 50, true);
  const auto v1_v2 = network.add_edge(v1, v2, 5000, 50, true);

  network.add_successor(v0_v1, v1_v2);

  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 100, 20, 4, 2, true, 0, 20, v0,
                                       500, 20, v2, network);
  const auto tr2 = timetable.add_train("Train2", 100, 50, 4, 4, true, 60, 50,
                                       v0, 260, 50, v2, network);

  RouteMap                                                    routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);
  cda_rail::simulator::GreedySimulator simulator(instance, {});

  simulator.set_train_edges_of_tr(tr1, {v0_v1});
  simulator.set_vertex_orders_of_vertex(v0, {tr1, tr2});
  simulator.set_vertex_orders_of_vertex(v2, {tr2});
  simulator.set_train_edges_of_tr(tr2, {v0_v1, v1_v2});

  const auto sim_res_1 = simulator.simulate(6.0, false, true, false, true);
  const auto sim_res_2 = simulator.simulate(6.0, false, true, false, false);

  EXPECT_FALSE(sim_res_1.success);

  EXPECT_TRUE(sim_res_2.success);
  ASSERT_EQ(sim_res_2.exit_times.size(), 2);
  ASSERT_GE(sim_res_2.exit_times.at(tr1), 5000.0 / 20.0);
  ASSERT_GE(sim_res_2.exit_times.at(tr2),
            sim_res_2.exit_times.at(tr1) +
                (5000.0 / 50.0)); // successful but delayed by tr1
}

TEST(GreedySimulation, ExitTimeConstraintNoSlowingDown) {
  static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
  plog::init(plog::verbose, &console_appender);

  Network    network;
  const auto v0 = network.add_vertex("v0", VertexType::TTD, 0);
  const auto v1 = network.add_vertex("v1", VertexType::TTD, 0);

  const auto v0_v1 = network.add_edge(v0, v1, 5000, 50, true);
  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 250, 50, 4, 2, true, 0, 50, v0,
                                       105, 50, v1, network);
  RouteMap   routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);

  cda_rail::simulator::GreedySimulator simulator(instance, {});

  simulator.set_train_edges_of_tr(tr1, {v0_v1});
  simulator.set_vertex_orders_of_vertex(v0, {tr1});
  simulator.set_vertex_orders_of_vertex(v1, {tr1});

  const auto sim_res = simulator.simulate(5.0, false, true);

  ASSERT_FALSE(sim_res.exit_times.empty());
  PLOGD << "Simulation success: " << (sim_res.success ? "true" : "false")
        << ", Objective value: " << sim_res.exit_times.back() << std::endl;
  EXPECT_TRUE(sim_res.success);
  ASSERT_EQ(sim_res.exit_times.size(), 1);
  EXPECT_EQ(sim_res.exit_times[0], 105);
}

TEST(GreedySimulation, ExitTimeConstraintSlowingDown) {
  static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
  plog::init(plog::verbose, &console_appender);

  Network    network;
  const auto v0 = network.add_vertex("v0", VertexType::TTD, 0);
  const auto v1 = network.add_vertex("v1", VertexType::TTD, 0);

  const auto v0_v1 = network.add_edge(v0, v1, 5000, 50, true);

  Timetable  timetable2;
  const auto tr1 = timetable2.add_train("Train1", 250, 50, 4, 2, true, 0, 50,
                                        v0, 110, 50, v1, network);
  RouteMap   routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance2(
      network, timetable2, routes);

  cda_rail::simulator::GreedySimulator simulator2(instance2, {});

  simulator2.set_train_edges_of_tr(tr1, {v0_v1});
  simulator2.set_vertex_orders_of_vertex(v0, {tr1});
  simulator2.set_vertex_orders_of_vertex(v1, {tr1});

  const auto sim_res2 = simulator2.simulate(5.0, false, true);

  ASSERT_FALSE(sim_res2.exit_times.empty());
  PLOGD << "Simulation 2 success: " << (sim_res2.success ? "true" : "false")
        << ", Objective value: " << sim_res2.exit_times.back() << std::endl;
  EXPECT_TRUE(sim_res2.success);
  ASSERT_EQ(sim_res2.exit_times.size(), 1);
  EXPECT_GE(sim_res2.exit_times[0], 110);
  EXPECT_LE(sim_res2.exit_times[0], 115);
}

TEST(GreedySimulation, ExitTimeConstraintStopping) {
  static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
  plog::init(plog::verbose, &console_appender);

  Network    network;
  const auto v0 = network.add_vertex("v0", VertexType::TTD, 0);
  const auto v1 = network.add_vertex("v1", VertexType::TTD, 0);

  const auto v0_v1 = network.add_edge(v0, v1, 5000, 50, true);

  Timetable  timetable2;
  const auto tr1 = timetable2.add_train("Train1", 250, 50, 4, 2, true, 0, 50,
                                        v0, 200, 50, v1, network);
  RouteMap   routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance2(
      network, timetable2, routes);

  cda_rail::simulator::GreedySimulator simulator2(instance2, {});

  simulator2.set_train_edges_of_tr(tr1, {v0_v1});
  simulator2.set_vertex_orders_of_vertex(v0, {tr1});
  simulator2.set_vertex_orders_of_vertex(v1, {tr1});

  const auto sim_res2 = simulator2.simulate(5.0, false, true);

  ASSERT_FALSE(sim_res2.exit_times.empty());
  PLOGD << "Simulation 2 success: " << (sim_res2.success ? "true" : "false")
        << ", Objective value: " << sim_res2.exit_times.back() << std::endl;
  EXPECT_TRUE(sim_res2.success);
  ASSERT_EQ(sim_res2.exit_times.size(), 1);
  EXPECT_GE(sim_res2.exit_times[0], 200);
  EXPECT_LE(sim_res2.exit_times[0], 205);
}

TEST(GreedySimulator, DesiredOrderInstanceSolution) {
  static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
  plog::init(plog::verbose, &console_appender);

  Network    network;
  auto const v0a = network.add_vertex("v0a", VertexType::TTD);
  auto const v0b = network.add_vertex("v0b", VertexType::TTD);
  auto const v0c = network.add_vertex("v0c", VertexType::TTD);
  auto const v1a = network.add_vertex("v1a", VertexType::TTD);
  auto const v1b = network.add_vertex("v1b", VertexType::TTD);
  auto const v1c = network.add_vertex("v1c", VertexType::TTD);
  auto const v2  = network.add_vertex("v2", VertexType::NoBorder);
  auto const v3  = network.add_vertex("v3", VertexType::TTD);
  auto const v4  = network.add_vertex("v4", VertexType::TTD);

  auto const v01a = network.add_edge({"v0a"}, {"v1a"}, 100, 20, true);
  auto const v01b = network.add_edge({"v0b"}, {"v1b"}, 100, 20, true);
  auto const v01c = network.add_edge({"v0c"}, {"v1c"}, 100, 20, true);

  // Switch
  auto const v12a = network.add_edge({"v1a"}, {"v2"}, 10, 20, false);
  auto const v12b = network.add_edge({"v1b"}, {"v2"}, 10, 20, false);
  auto const v12c = network.add_edge({"v1c"}, {"v2"}, 10, 20, false);
  auto const v23  = network.add_edge({"v2"}, {"v3"}, 10, 20, false);

  auto const v34 = network.add_edge({"v3"}, {"v4"}, 100, 20, true);

  network.add_successor({"v0a", "v1a"}, {"v1a", "v2"});
  network.add_successor({"v0b", "v1b"}, {"v1b", "v2"});
  network.add_successor({"v0c", "v1c"}, {"v1c", "v2"});
  network.add_successor({"v1a", "v2"}, {"v2", "v3"});
  network.add_successor({"v1b", "v2"}, {"v2", "v3"});
  network.add_successor({"v1c", "v2"}, {"v2", "v3"});
  network.add_successor({"v2", "v3"}, {"v3", "v4"});

  // Trains
  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 50, 20, 2, 4, true, 0, 20,
                                       {"v0a"}, 500, 20, {"v4"}, network);
  const auto tr2 = timetable.add_train("Train2", 50, 20, 2, 4, true, 100, 20,
                                       {"v0b"}, 300, 20, {"v4"}, network);
  const auto tr3 = timetable.add_train("Train3", 50, 20, 2, 4, true, 200, 20,
                                       {"v0c"}, 400, 20, {"v4"}, network);

  RouteMap                                                    routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);

  instance.set_train_weight(tr1, 1);
  instance.set_train_weight(tr2, 2);
  instance.set_train_weight(tr3, 3);

  cda_rail::simulator::GreedySimulator simulator(instance,
                                                 {{v12a, v12b, v12c, v23}});
  simulator.set_train_edges_of_tr(tr1, {v01a, v12a, v23, v34});
  simulator.set_train_edges_of_tr(tr2, {v01b, v12b, v23, v34});
  simulator.set_train_edges_of_tr(tr3, {v01c, v12c, v23, v34});
  simulator.set_ttd_orders_of_ttd(0, {tr2, tr3, tr1});
  simulator.set_vertex_orders_of_vertex(v0a, {tr1});
  simulator.set_vertex_orders_of_vertex(v0b, {tr2});
  simulator.set_vertex_orders_of_vertex(v0c, {tr3});
  simulator.set_vertex_orders_of_vertex(v4, {tr2, tr3, tr1});

  auto const sim_res = simulator.simulate(5.0, false, true, true);
  ASSERT_EQ(sim_res.exit_times.size(), 3);
  auto const& tr1_exit = sim_res.exit_times.at(tr1);
  auto const& tr2_exit = sim_res.exit_times.at(tr2);
  auto const& tr3_exit = sim_res.exit_times.at(tr3);

  EXPECT_EQ(tr1_exit, 500.0);
  EXPECT_EQ(tr2_exit, 300.0);
  EXPECT_EQ(tr3_exit, 400.0);

  ASSERT_TRUE(sim_res.train_trajectories.at(tr1).contains(tr1_exit));
  EXPECT_GE(sim_res.train_trajectories.at(tr1).at(tr1_exit).pos,
            100 + 10 + 10 + 100);
  EXPECT_GE(sim_res.train_trajectories.at(tr2).at(tr2_exit).pos,
            100 + 10 + 10 + 100);
  EXPECT_GE(sim_res.train_trajectories.at(tr3).at(tr3_exit).pos,
            100 + 10 + 10 + 100);
}

TEST(GreedySimulator, HeadOnCollision1) {
  static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
  plog::init(plog::verbose, &console_appender);

  // clang-format off
  // (tr1) -> v0a -- v1a -                           - v6a -- v7a
  //                      \                         /
  //          v0b -- v1b - v2 - v3 -- v4 -- v45 - v5 - v6b -- v7b <- (tr2)
  // clang-format on

  Network    network;
  auto const v0a = network.add_vertex("v0a", VertexType::TTD);
  auto const v1a = network.add_vertex("v1a", VertexType::TTD);
  auto const v0b = network.add_vertex("v0b", VertexType::TTD);
  auto const v1b = network.add_vertex("v1b", VertexType::TTD);
  auto const v2  = network.add_vertex("v2", VertexType::NoBorder);
  auto const v3  = network.add_vertex("v3", VertexType::TTD);
  auto const v4  = network.add_vertex("v4", VertexType::TTD);
  auto const v45 = network.add_vertex("v45", VertexType::TTD);
  auto const v5  = network.add_vertex("v5", VertexType::NoBorder);
  auto const v6a = network.add_vertex("v6a", VertexType::TTD);
  auto const v6b = network.add_vertex("v6b", VertexType::TTD);
  auto const v7a = network.add_vertex("v7a", VertexType::TTD);
  auto const v7b = network.add_vertex("v7b", VertexType::TTD);

  auto const v0a_v1a = network.add_edge(v0a, v1a, 1000, 20, true);
  auto const v0b_v1b = network.add_edge(v0b, v1b, 1000, 20, true);
  auto const v1a_v2  = network.add_edge(v1a, v2, 100, 20, false);
  auto const v1b_v2  = network.add_edge(v1b, v2, 100, 20, false);
  auto const v2_v3   = network.add_edge(v2, v3, 100, 20, false);
  auto const v3_v4   = network.add_edge(v3, v4, 1000, 20, true);
  auto const v4_v45  = network.add_edge(v4, v45, 1000, 20, true);
  auto const v45_v5  = network.add_edge(v45, v5, 100, 20, false);
  auto const v5_v6a  = network.add_edge(v5, v6a, 100, 20, false);
  auto const v5_v6b  = network.add_edge(v5, v6b, 100, 20, false);
  auto const v6a_v7a = network.add_edge(v6a, v7a, 1000, 20, true);
  auto const v6b_v7b = network.add_edge(v6b, v7b, 1000, 20, true);

  network.add_successor(v0a_v1a, v1a_v2);
  network.add_successor(v0b_v1b, v1b_v2);
  network.add_successor(v1a_v2, v2_v3);
  network.add_successor(v1b_v2, v2_v3);
  network.add_successor(v2_v3, v3_v4);
  network.add_successor(v3_v4, v4_v45);
  network.add_successor(v4_v45, v45_v5);
  network.add_successor(v45_v5, v5_v6a);
  network.add_successor(v45_v5, v5_v6b);
  network.add_successor(v5_v6a, v6a_v7a);
  network.add_successor(v5_v6b, v6b_v7b);

  // Add all reverse edges
  auto const             num_edges = network.number_of_edges();
  cda_rail::index_vector reverse_edges(num_edges);
  for (size_t i = 0; i < num_edges; ++i) {
    auto const edge     = network.get_edge(i);
    reverse_edges.at(i) = network.add_edge(
        edge.target, edge.source, edge.length, edge.max_speed, edge.breakable);
  }
  for (size_t i = 0; i < num_edges; ++i) {
    auto const& successors = network.get_successors(i);
    for (auto const& successor : successors) {
      network.add_successor(reverse_edges.at(successor), reverse_edges.at(i));
    }
  }

  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 50, 20, 2, 4, true, 0, 10,
                                       {"v0a"}, 500, 20, {"v7a"}, network);
  const auto tr2 = timetable.add_train("Train2", 50, 20, 2, 4, true, 300, 10,
                                       {"v7b"}, 500, 20, {"v0b"}, network);

  RouteMap                                                    routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);

  cda_rail::index_set              ttd1{v1a_v2,
                                        v1b_v2,
                                        v2_v3,
                                        reverse_edges.at(v1a_v2),
                                        reverse_edges.at(v1b_v2),
                                        reverse_edges.at(v2_v3)};
  cda_rail::index_set              ttd2{v45_v5,
                                        v5_v6a,
                                        v5_v6b,
                                        reverse_edges.at(v45_v5),
                                        reverse_edges.at(v5_v6a),
                                        reverse_edges.at(v5_v6b)};
  std::vector<cda_rail::index_set> ttd_sections{ttd1, ttd2};

  cda_rail::simulator::GreedySimulator simulator(instance, ttd_sections);
  simulator.set_train_edges_of_tr(tr1, {v0a_v1a, v1a_v2, v2_v3, v3_v4, v4_v45});
  simulator.set_train_edges_of_tr(
      tr2, {reverse_edges.at(v6b_v7b), reverse_edges.at(v5_v6b),
            reverse_edges.at(v45_v5), reverse_edges.at(v4_v45),
            reverse_edges.at(v3_v4)});
  simulator.set_vertex_orders_of_vertex(v0a, {tr1});
  simulator.set_vertex_orders_of_vertex(v7b, {tr2});
  simulator.set_ttd_orders_of_ttd(0, {tr1});
  simulator.set_ttd_orders_of_ttd(1, {tr2});

  PLOGV << "------------------------------------------";
  PLOGV << "Simulation 1";
  PLOGV << "------------------------------------------";
  auto const sim_res = simulator.simulate(5.0, false, true, false, true);
  EXPECT_FALSE(sim_res.success);

  PLOGV << "------------------------------------------";
  PLOGV << "Simulation 2";
  PLOGV << "------------------------------------------";
  auto const sim_res2 = simulator.simulate(5.0, false, true, false, false);
  EXPECT_TRUE(sim_res2.success);
}

TEST(GreedySimulator, HeadOnCollision2) {
  static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
  plog::init(plog::verbose, &console_appender);

  // clang-format off
  // (tr1) -> v0a -- v1a -                           - v6a -- v7a
  //                      \                         /
  //          v0b -- v1b - v2 - v3 -- v4 -- v45 - v5 - v6b -- v7b <- (tr2)
  // clang-format on

  Network    network;
  auto const v0a = network.add_vertex("v0a", VertexType::TTD);
  auto const v1a = network.add_vertex("v1a", VertexType::TTD);
  auto const v0b = network.add_vertex("v0b", VertexType::TTD);
  auto const v1b = network.add_vertex("v1b", VertexType::TTD);
  auto const v2  = network.add_vertex("v2", VertexType::NoBorder);
  auto const v3  = network.add_vertex("v3", VertexType::TTD);
  auto const v4  = network.add_vertex("v4", VertexType::TTD);
  auto const v45 = network.add_vertex("v45", VertexType::TTD);
  auto const v5  = network.add_vertex("v5", VertexType::NoBorder);
  auto const v6a = network.add_vertex("v6a", VertexType::TTD);
  auto const v6b = network.add_vertex("v6b", VertexType::TTD);
  auto const v7a = network.add_vertex("v7a", VertexType::TTD);
  auto const v7b = network.add_vertex("v7b", VertexType::TTD);

  auto const v0a_v1a = network.add_edge(v0a, v1a, 1000, 20, true);
  auto const v0b_v1b = network.add_edge(v0b, v1b, 1000, 20, true);
  auto const v1a_v2  = network.add_edge(v1a, v2, 100, 20, false);
  auto const v1b_v2  = network.add_edge(v1b, v2, 100, 20, false);
  auto const v2_v3   = network.add_edge(v2, v3, 100, 20, false);
  auto const v3_v4   = network.add_edge(v3, v4, 1000, 20, true);
  auto const v4_v45  = network.add_edge(v4, v45, 1000, 20, true);
  auto const v45_v5  = network.add_edge(v45, v5, 100, 20, false);
  auto const v5_v6a  = network.add_edge(v5, v6a, 100, 20, false);
  auto const v5_v6b  = network.add_edge(v5, v6b, 100, 20, false);
  auto const v6a_v7a = network.add_edge(v6a, v7a, 1000, 20, true);
  auto const v6b_v7b = network.add_edge(v6b, v7b, 1000, 20, true);

  network.add_successor(v0a_v1a, v1a_v2);
  network.add_successor(v0b_v1b, v1b_v2);
  network.add_successor(v1a_v2, v2_v3);
  network.add_successor(v1b_v2, v2_v3);
  network.add_successor(v2_v3, v3_v4);
  network.add_successor(v3_v4, v4_v45);
  network.add_successor(v4_v45, v45_v5);
  network.add_successor(v45_v5, v5_v6a);
  network.add_successor(v45_v5, v5_v6b);
  network.add_successor(v5_v6a, v6a_v7a);
  network.add_successor(v5_v6b, v6b_v7b);

  // Add all reverse edges
  auto const             num_edges = network.number_of_edges();
  cda_rail::index_vector reverse_edges(num_edges);
  for (size_t i = 0; i < num_edges; ++i) {
    auto const edge     = network.get_edge(i);
    reverse_edges.at(i) = network.add_edge(
        edge.target, edge.source, edge.length, edge.max_speed, edge.breakable);
  }
  for (size_t i = 0; i < num_edges; ++i) {
    auto const& successors = network.get_successors(i);
    for (auto const& successor : successors) {
      network.add_successor(reverse_edges.at(successor), reverse_edges.at(i));
    }
  }

  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 50, 20, 2, 4, true, 0, 10,
                                       {"v0a"}, 500, 20, {"v7a"}, network);
  const auto tr2 = timetable.add_train("Train2", 50, 20, 2, 4, true, 0, 10,
                                       {"v7b"}, 500, 20, {"v0b"}, network);

  RouteMap                                                    routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);

  cda_rail::index_set              ttd1{v1a_v2,
                                        v1b_v2,
                                        v2_v3,
                                        reverse_edges.at(v1a_v2),
                                        reverse_edges.at(v1b_v2),
                                        reverse_edges.at(v2_v3)};
  cda_rail::index_set              ttd2{v45_v5,
                                        v5_v6a,
                                        v5_v6b,
                                        reverse_edges.at(v45_v5),
                                        reverse_edges.at(v5_v6a),
                                        reverse_edges.at(v5_v6b)};
  std::vector<cda_rail::index_set> ttd_sections{ttd1, ttd2};

  cda_rail::simulator::GreedySimulator simulator(instance, ttd_sections);
  simulator.set_train_edges_of_tr(tr1, {v0a_v1a, v1a_v2, v2_v3, v3_v4, v4_v45});
  simulator.set_train_edges_of_tr(
      tr2, {reverse_edges.at(v6b_v7b), reverse_edges.at(v5_v6b),
            reverse_edges.at(v45_v5), reverse_edges.at(v4_v45),
            reverse_edges.at(v3_v4)});
  simulator.set_vertex_orders_of_vertex(v0a, {tr1});
  simulator.set_vertex_orders_of_vertex(v7b, {tr2});
  simulator.set_ttd_orders_of_ttd(0, {tr1});
  simulator.set_ttd_orders_of_ttd(1, {tr2});

  PLOGV << "------------------------------------------";
  PLOGV << "Simulation 1";
  PLOGV << "------------------------------------------";
  auto const sim_res = simulator.simulate(5.0, false, true, false, true);
  EXPECT_FALSE(sim_res.success);

  PLOGV << "------------------------------------------";
  PLOGV << "Simulation 2";
  PLOGV << "------------------------------------------";
  auto const sim_res2 = simulator.simulate(5.0, false, true, false, false);
  EXPECT_TRUE(sim_res2.success);
}

// -----------------
// Bugfixing Tests
// -----------------

TEST(GreedySimulator, SimpleNetworkTR1RL) {
  static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
  plog::init(plog::verbose, &console_appender);

  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      "SimpleNetwork", "atmos2023", "./data");

  const auto ttd_sections = instance.get_const_network().unbreakable_sections();
  cda_rail::simulator::GreedySimulator simulator(instance, ttd_sections);

  auto const tr1rl = instance.get_const_train_list().get_train_index("tr1rl");
  EXPECT_EQ(tr1rl, 1);

  simulator.append_train_edge_to_tr(tr1rl, {"v14b", "v13b"});
  simulator.append_train_edge_to_tr(tr1rl, {"v13b", "v12"});
  simulator.append_train_edge_to_tr(tr1rl, {"v12", "v11"});
  simulator.append_train_edge_to_tr(tr1rl, {"v11", "v10"});
  simulator.append_train_edge_to_tr(tr1rl, {"v10", "v9"});
  simulator.append_train_edge_to_tr(tr1rl, {"v9", "v8b"});
  simulator.append_train_edge_to_tr(tr1rl, {"v8b", "v7b"});
  simulator.append_train_edge_to_tr(tr1rl, {"v7b", "v6"});
  simulator.append_train_edge_to_tr(tr1rl, {"v6", "v5"});
  simulator.append_train_edge_to_tr(tr1rl, {"v5", "v4"});
  simulator.append_train_edge_to_tr(tr1rl, {"v4", "v3"});
  simulator.append_train_edge_to_tr(tr1rl, {"v3", "v2b"});
  simulator.append_train_edge_to_tr(tr1rl, {"v2b", "v1b"});

  EXPECT_EQ(ttd_sections.size(), 4);
  for (size_t i = 0; i < ttd_sections.size(); ++i) {
    simulator.set_ttd_orders_of_ttd(i, {tr1rl});
  }
  simulator.set_vertex_orders_of_vertex({"v14b"}, {tr1rl});
  simulator.set_vertex_orders_of_vertex({"v1b"}, {tr1rl});

  auto const sim_res = simulator.simulate();
  EXPECT_TRUE(sim_res.success);
  EXPECT_GE(sim_res.exit_times.at(tr1rl), 960);
  EXPECT_LE(sim_res.exit_times.at(tr1rl), 960 + 6);
}

TEST(GreedySimulator, SimpleNetworkTR1LR) {
  static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
  plog::init(plog::verbose, &console_appender);

  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      "SimpleNetwork", "atmos2023", "./data");

  const auto ttd_sections = instance.get_const_network().unbreakable_sections();
  cda_rail::simulator::GreedySimulator simulator(instance, ttd_sections);

  auto const tr1lr = instance.get_const_train_list().get_train_index("tr1lr");
  EXPECT_EQ(tr1lr, 0);

  simulator.append_train_edge_to_tr(tr1lr, {"v1b", "v2b"});
  simulator.append_train_edge_to_tr(tr1lr, {"v2b", "v3"});
  simulator.append_train_edge_to_tr(tr1lr, {"v3", "v4"});
  simulator.append_train_edge_to_tr(tr1lr, {"v4", "v5"});
  simulator.append_train_edge_to_tr(tr1lr, {"v5", "v6"});
  simulator.append_train_edge_to_tr(tr1lr, {"v6", "v7b"});
  simulator.append_train_edge_to_tr(tr1lr, {"v7b", "v8b"});
  simulator.append_train_edge_to_tr(tr1lr, {"v8b", "v9"});
  simulator.append_train_edge_to_tr(tr1lr, {"v9", "v10"});
  simulator.append_train_edge_to_tr(tr1lr, {"v10", "v11"});
  simulator.append_train_edge_to_tr(tr1lr, {"v11", "v12"});
  simulator.append_train_edge_to_tr(tr1lr, {"v12", "v13b"});
  simulator.append_train_edge_to_tr(tr1lr, {"v13b", "v14b"});

  EXPECT_EQ(ttd_sections.size(), 4);
  for (size_t i = 0; i < ttd_sections.size(); ++i) {
    simulator.set_ttd_orders_of_ttd(i, {tr1lr});
  }
  simulator.set_vertex_orders_of_vertex({"v14b"}, {tr1lr});
  simulator.set_vertex_orders_of_vertex({"v1b"}, {tr1lr});

  auto const sim_res = simulator.simulate();
  EXPECT_TRUE(sim_res.success);
  EXPECT_GE(sim_res.exit_times.at(tr1lr), 960);
  EXPECT_LE(sim_res.exit_times.at(tr1lr), 960 + 6);
}

TEST(GreedySimulator, SimpleNetworkLRTrains) {
  static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
  plog::init(plog::verbose, &console_appender);

  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      "SimpleNetwork", "atmos2023", "./data");

  const auto ttd_sections = instance.get_const_network().unbreakable_sections();
  cda_rail::simulator::GreedySimulator simulator(instance, ttd_sections);

  auto const tr1lr = instance.get_const_train_list().get_train_index("tr1lr");
  auto const tr2lr = instance.get_const_train_list().get_train_index("tr2lr");
  EXPECT_EQ(tr1lr, 0);
  EXPECT_EQ(tr2lr, 2);

  simulator.append_train_edge_to_tr(tr1lr, {"v1b", "v2b"});
  simulator.append_train_edge_to_tr(tr1lr, {"v2b", "v3"});
  simulator.append_train_edge_to_tr(tr1lr, {"v3", "v4"});
  simulator.append_train_edge_to_tr(tr1lr, {"v4", "v5"});
  simulator.append_train_edge_to_tr(tr1lr, {"v5", "v6"});
  simulator.append_train_edge_to_tr(tr1lr, {"v6", "v7b"});
  simulator.append_train_edge_to_tr(tr1lr, {"v7b", "v8b"});
  simulator.append_train_edge_to_tr(tr1lr, {"v8b", "v9"});
  simulator.append_train_edge_to_tr(tr1lr, {"v9", "v10"});
  simulator.append_train_edge_to_tr(tr1lr, {"v10", "v11"});
  simulator.append_train_edge_to_tr(tr1lr, {"v11", "v12"});
  simulator.append_train_edge_to_tr(tr1lr, {"v12", "v13b"});
  simulator.append_train_edge_to_tr(tr1lr, {"v13b", "v14b"});

  simulator.append_train_edge_to_tr(tr2lr, {"v1c", "v2c"});
  simulator.append_train_edge_to_tr(tr2lr, {"v2c", "v3"});
  simulator.append_train_edge_to_tr(tr2lr, {"v3", "v4"});
  simulator.append_train_edge_to_tr(tr2lr, {"v4", "v5"});
  simulator.append_train_edge_to_tr(tr2lr, {"v5", "v6"});
  simulator.append_train_edge_to_tr(tr2lr, {"v6", "v7a"});
  simulator.append_train_edge_to_tr(tr2lr, {"v7a", "v8a"});
  simulator.append_train_edge_to_tr(tr2lr, {"v8a", "v9"});
  simulator.append_train_edge_to_tr(tr2lr, {"v9", "v10"});
  simulator.append_train_edge_to_tr(tr2lr, {"v10", "v11"});
  simulator.append_train_edge_to_tr(tr2lr, {"v11", "v12"});
  simulator.append_train_edge_to_tr(tr2lr, {"v12", "v13c"});
  simulator.append_train_edge_to_tr(tr2lr, {"v13c", "v14c"});

  simulator.set_stop_positions_of_tr(tr2lr, {475, 23000});

  auto const ttd1_edge =
      instance.get_const_network().get_edge_index({"v2b", "v3"});
  auto const ttd2_edge =
      instance.get_const_network().get_edge_index({"v6", "v7b"});
  auto const ttd3_edge =
      instance.get_const_network().get_edge_index({"v8b", "v9"});
  auto const ttd4_edge =
      instance.get_const_network().get_edge_index({"v12", "v13b"});
  EXPECT_EQ(ttd_sections.size(), 4);
  for (size_t i = 0; i < ttd_sections.size(); ++i) {
    if (ttd_sections.at(i).contains(ttd1_edge) ||
        ttd_sections.at(i).contains(ttd2_edge)) {
      simulator.set_ttd_orders_of_ttd(i, {tr1lr, tr2lr});
    } else if (ttd_sections.at(i).contains(ttd3_edge) ||
               ttd_sections.at(i).contains(ttd4_edge)) {
      simulator.set_ttd_orders_of_ttd(i, {tr2lr, tr1lr});
    } else {
      EXPECT_FALSE(true) << "Unexpected TTD section";
    }
  }
  simulator.set_vertex_orders_of_vertex({"v1b"}, {tr1lr});
  simulator.set_vertex_orders_of_vertex({"v14b"}, {tr1lr});
  simulator.set_vertex_orders_of_vertex({"v1c"}, {tr2lr});
  simulator.set_vertex_orders_of_vertex({"v14c"}, {tr2lr});

  auto const sim_res = simulator.simulate();
  EXPECT_TRUE(sim_res.success);
  EXPECT_GE(sim_res.exit_times.at(tr1lr), 960);
  EXPECT_LE(sim_res.exit_times.at(tr1lr), 960 + 6);
  EXPECT_GE(sim_res.exit_times.at(tr2lr), 990);
  EXPECT_LE(sim_res.exit_times.at(tr2lr), 990 + 6);
  EXPECT_TRUE(sim_res.stop_times.at(tr1lr).empty());
  EXPECT_EQ(sim_res.stop_times.at(tr2lr).size(), 2);
}

TEST(GreedySimulator, StopAtRouteEnd) {
  static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
  plog::init(plog::verbose, &console_appender);

  instances::GeneralPerformanceOptimizationInstance instance;

  auto const v0 =
      instance.get_editable_network().add_vertex("v0", VertexType::TTD);
  auto const v1 =
      instance.get_editable_network().add_vertex("v1", VertexType::TTD);

  auto const v0_v1 =
      instance.get_editable_network().add_edge(v0, v1, 100, 10, true);

  instance.add_train("Train1", 20, 10, 2, 2, true, 0, 10, {"v0"}, 0, 10,
                     {"v1"});
  instance.add_empty_station("Station");
  instance.add_track_to_station("Station", v0_v1);
  instance.insert_stop("Train1", "Station", 0, 30);

  cda_rail::simulator::GreedySimulator simulator(instance, {});
  simulator.append_train_edge_to_tr(0, v0_v1);
  simulator.set_stop_positions_of_tr(0, {100});
  simulator.set_vertex_orders({{0}, {0}});

  auto const sim_res = simulator.simulate(5.0);
  EXPECT_TRUE(sim_res.success);
  ASSERT_EQ(sim_res.exit_times.size(), 1);
  ASSERT_EQ(sim_res.stop_times.size(), 1);
  ASSERT_EQ(sim_res.stop_times.at(0).size(), 1);
  EXPECT_GE(sim_res.exit_times.at(0), sim_res.stop_times.at(0).back() + 30);
}

TEST(GreedySimulator, EndAtStop) {
  static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
  plog::init(plog::verbose, &console_appender);

  // Train 1 moves for 5 seconds at speed 10 --> 50
  // Train 1 decelerates for 5 seconds at rate 2 --> stopping
  // pos = 50 + 10/2*5 = 50 + 25 = 75

  // Train 2 stops after 5 seconds at pos 25

  // As soon as it can move again it accelerates over 50m and 10 seconds

  instances::GeneralPerformanceOptimizationInstance instance;

  auto const v0 =
      instance.get_editable_network().add_vertex("v0", VertexType::TTD);
  auto const v1 =
      instance.get_editable_network().add_vertex("v1", VertexType::TTD);
  auto const v2 =
      instance.get_editable_network().add_vertex("v2", VertexType::TTD);
  auto const v3 =
      instance.get_editable_network().add_vertex("v3", VertexType::TTD);

  auto const v0_v1 =
      instance.get_editable_network().add_edge(v0, v1, 75, 10, true);

  auto const tr1 = instance.add_train("Train1", 50, 10, 2, 2, true, 0, 10,
                                      {"v0"}, 0, 10, {"v2"});
  auto const tr2 = instance.add_train("Train2", 50, 10, 1, 2, true, 20, 10,
                                      {"v0"}, 20, 10, {"v2"});
  instance.add_empty_station("Station");
  instance.add_track_to_station("Station", v0_v1);
  instance.insert_stop("Train1", "Station", 0, 30);

  cda_rail::simulator::GreedySimulator simulator(instance, {});
  simulator.append_train_edge_to_tr(tr1, v0_v1);
  simulator.append_train_edge_to_tr(tr2, v0_v1);
  simulator.set_stop_positions_of_tr(tr1, {75});
  simulator.set_vertex_orders_of_vertex(v0, {tr1, tr2});

  PLOGV << "------------------------------------------";
  PLOGV << "Simulation 1";
  PLOGV << "------------------------------------------";
  auto const sim_res_1 = simulator.simulate(2.5, false, false, false, true);

  PLOGV << "------------------------------------------";
  PLOGV << "Simulation 2";
  PLOGV << "------------------------------------------";
  auto const sim_res_2 = simulator.simulate(2.5, false, false, false, false);

  PLOGV << "------------------------------------------";
  PLOGV << "Assertion";
  PLOGV << "------------------------------------------";
  EXPECT_FALSE(sim_res_1.success);
  EXPECT_TRUE(sim_res_2.success);
  ASSERT_EQ(sim_res_2.exit_times.size(), 2);
  EXPECT_EQ(sim_res_2.exit_times.at(tr1), 10 + 30);
  auto const exit_time = sim_res_2.exit_times.at(tr1) + 10;
  EXPECT_GE(sim_res_2.exit_times.at(tr2), exit_time);
  EXPECT_EQ(sim_res_2.exit_times.at(tr2),
            get_first_time_step_after(exit_time, 2.5, true));
  ASSERT_EQ(sim_res_2.stop_times.size(), 2);
  EXPECT_TRUE(sim_res_2.stop_times.at(tr2).empty());
  ASSERT_EQ(sim_res_2.stop_times.at(tr1).size(), 1);
  EXPECT_EQ(sim_res_2.stop_times.at(tr1).at(0), 10);
}

TEST(GreedySimulator, TrainsFollowing) {
  static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
  plog::init(plog::verbose, &console_appender);

  instances::GeneralPerformanceOptimizationInstance instance;

  auto const v0 =
      instance.get_editable_network().add_vertex("v0", VertexType::TTD);
  auto const v1 =
      instance.get_editable_network().add_vertex("v1", VertexType::TTD);
  auto const v2 =
      instance.get_editable_network().add_vertex("v2", VertexType::TTD);
  auto const v3 =
      instance.get_editable_network().add_vertex("v3", VertexType::TTD);

  auto const v0_v1 =
      instance.get_editable_network().add_edge(v0, v1, 100, 10, true);
  auto const v1_v2 =
      instance.get_editable_network().add_edge(v1, v2, 5000, 10, true);
  auto const v2_v3 =
      instance.get_editable_network().add_edge(v2, v3, 100, 10, true);

  instance.get_editable_network().add_successor(v0_v1, v1_v2);
  instance.get_editable_network().add_successor(v1_v2, v2_v3);

  auto const tr1 = instance.add_train("Train1", 20, 10, 2, 2, true, 0, 10,
                                      {"v0"}, 0, 10, {"v3"});
  auto const tr2 = instance.add_train("Train2", 20, 10, 2, 2, true, 60, 10,
                                      {"v0"}, 60, 10, {"v3"});

  cda_rail::simulator::GreedySimulator simulator(instance, {});
  simulator.append_train_edge_to_tr(tr1, v0_v1);
  simulator.append_train_edge_to_tr(tr2, v0_v1);
  simulator.set_vertex_orders_of_vertex(v0, {tr1, tr2});

  auto const sim_res1 = simulator.simulate(5.0, false, false, false, true);
  EXPECT_TRUE(sim_res1.success);
  EXPECT_EQ(sim_res1.exit_times.at(tr1), 10);
  EXPECT_EQ(sim_res1.exit_times.at(tr2), 70);

  auto const sim_res2 = simulator.simulate(5.0, false, false, false, false);
  EXPECT_TRUE(sim_res2.success);
  EXPECT_EQ(sim_res2.exit_times.at(tr1), 10);
  EXPECT_EQ(sim_res2.exit_times.at(tr2), 70);

  simulator.append_train_edge_to_tr(tr2, v1_v2);

  auto const sim_res3 = simulator.simulate(5.0, false, false, false, true);
  EXPECT_FALSE(sim_res3.success);

  auto const sim_res4 = simulator.simulate(5.0, false, false, false, false);
  EXPECT_TRUE(sim_res4.success);
  EXPECT_EQ(sim_res4.exit_times.at(tr1), 10);
  EXPECT_EQ(sim_res4.exit_times.at(tr2), 570);

  simulator.set_train_edges_of_tr(tr2, {v0_v1});
  simulator.set_train_edges_of_tr(tr1, {v0_v1, v1_v2});

  auto const sim_res5 = simulator.simulate(5.0, false, false, false, true);
  EXPECT_TRUE(sim_res5.success);
  EXPECT_EQ(sim_res5.exit_times.at(tr1), 510);
  EXPECT_EQ(sim_res5.exit_times.at(tr2), 70);

  auto const sim_res6 = simulator.simulate(5.0, false, false, false, false);
  EXPECT_TRUE(sim_res6.success);
  EXPECT_EQ(sim_res6.exit_times.at(tr1), 510);
  EXPECT_EQ(sim_res6.exit_times.at(tr2), 70);
}

// NOLINTEND
// (clang-analyzer-deadcode.DeadStores,misc-const-correctness,clang-diagnostic-unused-result)
