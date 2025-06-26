#define TEST_FRIENDS true

#include "CustomExceptions.hpp"
#include "datastructure/GeneralTimetable.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "simulator/GreedySimulator.hpp"

#include "gtest/gtest.h"

using namespace cda_rail;

#define EXPECT_APPROX_EQ_6(a, b) EXPECT_APPROX_EQ(a, b, 1e-6)

#define EXPECT_APPROX_EQ(a, b, c)                                              \
  EXPECT_TRUE(std::abs((a) - (b)) < (c)) << (a) << " !=(approx.) " << (b)

// NOLINTBEGIN
// (clang-analyzer-deadcode.DeadStores,misc-const-correctness,clang-diagnostic-unused-result)

TEST(GreedySimulator, CheckConsistency) {
  // Create instance
  Network     network("./example-networks/SimpleStation/network/");
  const auto& ttd_sections = network.unbreakable_sections();
  const auto& l0_l1        = network.get_edge_index("l0", "l1");
  const auto& l1_l2        = network.get_edge_index("l1", "l2");
  const auto& l2_l3        = network.get_edge_index("l2", "l3");
  const auto& r0_r1        = network.get_edge_index("r0", "r1");
  const auto& r1_r2        = network.get_edge_index("r1", "r2");

  GeneralTimetable<GeneralSchedule<GeneralScheduledStop>> timetable;
  const auto l0  = network.get_vertex_index("l0");
  const auto r0  = network.get_vertex_index("r0");
  const auto tr1 = timetable.add_train("Train1", 100, 10, 1, 1, true, {0, 60},
                                       0, "l0", {360, 420}, 0, "r0", network);
  const auto tr2 = timetable.add_train("Train2", 100, 10, 1, 1, false, {0, 60},
                                       10, r0, {400, 460}, 5, l0, network);
  timetable.add_station("Station1");
  timetable.add_track_to_station("Station1", "g00", "g01", network);
  timetable.add_track_to_station("Station1", "g01", "g00", network);
  timetable.add_track_to_station("Station1", "g10", "g11", network);
  timetable.add_track_to_station("Station1", "g11", "g10", network);
  timetable.add_stop("Train1", "Station1", {60, 120}, {120, 180}, 60);

  timetable.add_station("Station2");
  timetable.add_track_to_station("Station2", "r2", "r1", network);
  timetable.add_stop("Train1", "Station2", {120, 200}, {200, 300}, 60);
  EXPECT_TRUE(timetable.check_consistency(network));

  RouteMap routes;

  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);
  EXPECT_TRUE(instance.check_consistency(false));

  cda_rail::instances::GeneralPerformanceOptimizationInstance instance2(
      network, timetable, routes);
  instance2.add_empty_route("Train1");
  instance2.push_back_edge_to_route("Train1", "l0", "l1");
  EXPECT_FALSE(instance2.check_consistency(false));

  // Test constructors of GreedySimulator

  cda_rail::simulator::GreedySimulator simulator1(
      instance, ttd_sections, {{l0_l1, l1_l2, l2_l3}, {r0_r1, r1_r2}},
      std::vector<std::vector<size_t>>(ttd_sections.size(),
                                       std::vector<size_t>()),
      {}, {{}, {}});
  cda_rail::simulator::GreedySimulator simulator2(
      instance, ttd_sections, {{l0_l1, l1_l2, l2_l3}, {r0_r1, r1_r2}}, {},
      std::vector<std::vector<size_t>>(network.number_of_vertices(),
                                       std::vector<size_t>()),
      {{}, {}});
  cda_rail::simulator::GreedySimulator simulator3(
      instance, ttd_sections, {{l0_l1, l1_l2, l2_l3}},
      std::vector<std::vector<size_t>>(ttd_sections.size(),
                                       std::vector<size_t>()),
      std::vector<std::vector<size_t>>(network.number_of_vertices(),
                                       std::vector<size_t>()),
      {{}, {}});
  cda_rail::simulator::GreedySimulator simulator3b(
      instance, ttd_sections, {{l0_l1, l1_l2, l2_l3}, {r0_r1, r1_r2}},
      std::vector<std::vector<size_t>>(ttd_sections.size(),
                                       std::vector<size_t>()),
      std::vector<std::vector<size_t>>(network.number_of_vertices(),
                                       std::vector<size_t>()),
      {{}, {}, {}});
  cda_rail::simulator::GreedySimulator simulator3c(
      instance, ttd_sections, {{l0_l1, l1_l2, l2_l3}, {r0_r1, r1_r2}},
      std::vector<std::vector<size_t>>(ttd_sections.size(),
                                       std::vector<size_t>()),
      std::vector<std::vector<size_t>>(network.number_of_vertices(),
                                       std::vector<size_t>()),
      {{100, 200}, {}});
  cda_rail::simulator::GreedySimulator simulator3d(
      instance, ttd_sections, {{l0_l1, l1_l2, l2_l3}, {r0_r1, r1_r2}},
      std::vector<std::vector<size_t>>(ttd_sections.size(),
                                       std::vector<size_t>()),
      std::vector<std::vector<size_t>>(network.number_of_vertices(),
                                       std::vector<size_t>()),
      {{}, {100}});
  cda_rail::simulator::GreedySimulator simulator3e(
      instance, ttd_sections, {{l0_l1, l1_l2, l2_l3}, {r0_r1, r1_r2}},
      std::vector<std::vector<size_t>>(ttd_sections.size(),
                                       std::vector<size_t>()),
      std::vector<std::vector<size_t>>(network.number_of_vertices(),
                                       std::vector<size_t>()),
      {{100, 200, 300}, {}});
  cda_rail::simulator::GreedySimulator simulator3f(
      instance, ttd_sections, {{l0_l1, l1_l2, l2_l3}, {r0_r1, r1_r2}},
      std::vector<std::vector<size_t>>(ttd_sections.size(),
                                       std::vector<size_t>()),
      std::vector<std::vector<size_t>>(network.number_of_vertices(),
                                       std::vector<size_t>()),
      {{100}, {}});
  cda_rail::simulator::GreedySimulator simulator3g(
      instance, ttd_sections, {{l0_l1, l1_l2, l2_l3}, {r0_r1, r1_r2}},
      std::vector<std::vector<size_t>>(ttd_sections.size(),
                                       std::vector<size_t>()),
      std::vector<std::vector<size_t>>(network.number_of_vertices(),
                                       std::vector<size_t>()),
      {{200, 100}, {}});
  cda_rail::simulator::GreedySimulator simulator3h(
      instance, ttd_sections, {{l0_l1, l1_l2, l2_l3}, {r0_r1, r1_r2}},
      std::vector<std::vector<size_t>>(ttd_sections.size(),
                                       std::vector<size_t>()),
      std::vector<std::vector<size_t>>(network.number_of_vertices(),
                                       std::vector<size_t>()),
      {{-100, 100}, {}});
  cda_rail::simulator::GreedySimulator simulator4(
      instance, ttd_sections, {{l0_l1, l1_l2, l2_l3}, {r0_r1, r1_r2}},
      std::vector<std::vector<size_t>>(ttd_sections.size(),
                                       std::vector<size_t>()),
      std::vector<std::vector<size_t>>(network.number_of_vertices(),
                                       std::vector<size_t>()),
      {{}, {}});
  simulator4.set_ttd_orders_of_ttd(0, {tr1, tr2});
  cda_rail::simulator::GreedySimulator simulator5(
      instance, ttd_sections, {{l0_l1, l1_l2, l2_l3}, {}},
      std::vector<std::vector<size_t>>(ttd_sections.size(),
                                       std::vector<size_t>()),
      std::vector<std::vector<size_t>>(network.number_of_vertices(),
                                       std::vector<size_t>()),
      {{}, {}});

  cda_rail::simulator::GreedySimulator simulator6(
      instance, ttd_sections, {{l0_l1, l1_l2, 1000}, {r0_r1, r1_r2}},
      std::vector<std::vector<size_t>>(ttd_sections.size(),
                                       std::vector<size_t>()),
      std::vector<std::vector<size_t>>(network.number_of_vertices(),
                                       std::vector<size_t>()),
      {{}, {}});
  cda_rail::simulator::GreedySimulator simulator7(
      instance, ttd_sections, {{l0_l1, l2_l3}, {r0_r1, r1_r2}},
      std::vector<std::vector<size_t>>(ttd_sections.size(),
                                       std::vector<size_t>()),
      std::vector<std::vector<size_t>>(network.number_of_vertices(),
                                       std::vector<size_t>()),
      {{}, {}});
  cda_rail::simulator::GreedySimulator simulator8(
      instance, ttd_sections, {{l0_l1, l1_l2, l2_l3}, {l0_l1, l1_l2}},
      std::vector<std::vector<size_t>>(ttd_sections.size(),
                                       std::vector<size_t>()),
      std::vector<std::vector<size_t>>(network.number_of_vertices(),
                                       std::vector<size_t>()),
      {{}, {}});
  cda_rail::simulator::GreedySimulator simulator9(
      instance, ttd_sections, {{l0_l1, l1_l2, l2_l3}, {r0_r1, r1_r2}},
      std::vector<std::vector<size_t>>(ttd_sections.size(),
                                       std::vector<size_t>()),
      std::vector<std::vector<size_t>>(network.number_of_vertices(),
                                       std::vector<size_t>()),
      {{}, {}});
  simulator9.set_ttd_orders_of_ttd(0, {1000});
  cda_rail::simulator::GreedySimulator simulator10(
      instance, ttd_sections, {{l0_l1, l1_l2, l2_l3}, {r0_r1, r1_r2}},
      std::vector<std::vector<size_t>>(ttd_sections.size(),
                                       std::vector<size_t>()),
      std::vector<std::vector<size_t>>(network.number_of_vertices(),
                                       std::vector<size_t>()),
      {{}, {}});
  cda_rail::simulator::GreedySimulator simulator11(
      instance, ttd_sections, {{l0_l1, l1_l2, l2_l3}, {r0_r1, r1_r2}},
      std::vector<std::vector<size_t>>(ttd_sections.size(),
                                       std::vector<size_t>()),
      std::vector<std::vector<size_t>>(network.number_of_vertices(),
                                       std::vector<size_t>()),
      {{}, {}});
  cda_rail::simulator::GreedySimulator simulator12(
      instance, ttd_sections, {{l0_l1, l1_l2, l2_l3}, {r0_r1, r1_r2}},
      std::vector<std::vector<size_t>>(ttd_sections.size(),
                                       std::vector<size_t>()),
      std::vector<std::vector<size_t>>(network.number_of_vertices(),
                                       std::vector<size_t>()),
      {{}, {}});
  simulator10.set_vertex_orders_of_vertex(l0, {tr1});
  simulator11.set_vertex_orders_of_vertex(l0, {1000});
  simulator12.set_vertex_orders_of_vertex(l0, {tr1, tr2});

  cda_rail::simulator::GreedySimulator simulator_instance2(instance2,
                                                           ttd_sections);

  // Check if consistency is determined correctly
  EXPECT_FALSE(simulator1.check_consistency());
  EXPECT_FALSE(simulator2.check_consistency());
  EXPECT_FALSE(simulator3.check_consistency());
  EXPECT_FALSE(simulator3b.check_consistency());
  EXPECT_TRUE(simulator3c.check_consistency());
  EXPECT_FALSE(simulator3d.check_consistency());
  EXPECT_FALSE(simulator3e.check_consistency());
  EXPECT_TRUE(simulator3f.check_consistency());
  EXPECT_FALSE(simulator3g.check_consistency());
  EXPECT_FALSE(simulator3h.check_consistency());
  EXPECT_TRUE(simulator4.check_consistency());
  EXPECT_TRUE(simulator5.check_consistency());
  EXPECT_FALSE(simulator6.check_consistency());
  EXPECT_FALSE(simulator7.check_consistency());
  EXPECT_FALSE(simulator8.check_consistency());
  EXPECT_FALSE(simulator9.check_consistency());
  EXPECT_TRUE(simulator10.check_consistency());
  EXPECT_FALSE(simulator11.check_consistency());
  EXPECT_FALSE(simulator12.check_consistency());
  EXPECT_FALSE(simulator_instance2.check_consistency());
}

TEST(GreedySimulator, BasicFunctions) {
  // Create instance
  Network     network("./example-networks/SimpleStation/network/");
  const auto& ttd_sections = network.unbreakable_sections();
  const auto& l0_l1        = network.get_edge_index("l0", "l1");
  const auto& l1_l2        = network.get_edge_index("l1", "l2");
  const auto& l2_l3        = network.get_edge_index("l2", "l3");
  const auto& r0_r1        = network.get_edge_index("r0", "r1");
  const auto& r1_r2        = network.get_edge_index("r1", "r2");
  const auto& l3_g00       = network.get_edge_index("l3", "g00");
  const auto& g00_g01      = network.get_edge_index("g00", "g01");
  const auto& g01_r2       = network.get_edge_index("g01", "r2");
  const auto& r2_r1        = network.get_edge_index("r2", "r1");
  const auto& r1_r0        = network.get_edge_index("r1", "r0");

  GeneralTimetable<GeneralSchedule<GeneralScheduledStop>> timetable;
  const auto l0  = network.get_vertex_index("l0");
  const auto r0  = network.get_vertex_index("r0");
  const auto tr1 = timetable.add_train("Train1", 100, 10, 1, 1, true, {0, 60},
                                       0, "l0", {360, 420}, 0, "r0", network);
  const auto tr2 = timetable.add_train("Train2", 100, 10, 1, 1, false, {30, 90},
                                       10, r0, {400, 460}, 5, l0, network);
  timetable.add_station("Station1");
  timetable.add_track_to_station("Station1", "g00", "g01", network);
  timetable.add_track_to_station("Station1", "g01", "g00", network);
  timetable.add_track_to_station("Station1", "g10", "g11", network);
  timetable.add_track_to_station("Station1", "g11", "g10", network);
  timetable.add_stop("Train1", "Station1", {60, 120}, {120, 180}, 60);

  timetable.add_station("Station2");
  timetable.add_track_to_station("Station2", "r2", "r1", network);
  timetable.add_stop("Train1", "Station2", {120, 200}, {200, 300}, 60);
  EXPECT_TRUE(timetable.check_consistency(network));

  RouteMap routes;

  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);
  EXPECT_TRUE(instance.check_consistency(false));

  cda_rail::instances::GeneralPerformanceOptimizationInstance instance2(
      network, timetable, routes);
  instance2.add_empty_route("Train1");
  instance2.push_back_edge_to_route("Train1", "l0", "l1");
  EXPECT_FALSE(instance2.check_consistency(false));

  // Test basic functions of GreedySimulator

  // Train Edges
  cda_rail::simulator::GreedySimulator simulator(instance, ttd_sections);
  EXPECT_THROW(simulator.set_train_edges({{l0_l1}}),
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

  EXPECT_THROW(simulator.get_train_edges_of_tr(1000),
               cda_rail::exceptions::TrainNotExistentException);
  EXPECT_THROW(simulator.set_train_edges_of_tr(1000, {l0_l1}),
               cda_rail::exceptions::TrainNotExistentException);

  // TTD Orders
  EXPECT_THROW(simulator.set_ttd_orders({}),
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
  EXPECT_THROW(simulator.get_ttd_orders_of_ttd(1000),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(simulator.set_ttd_orders_of_ttd(1000, {tr1}),
               cda_rail::exceptions::InvalidInputException);

  // Entry Orders
  EXPECT_THROW(simulator.set_vertex_orders({}),
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
  EXPECT_THROW(simulator.get_vertex_orders_of_vertex(1000),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(simulator.set_vertex_orders_of_vertex(1000, {tr1}),
               cda_rail::exceptions::InvalidInputException);

  // Stop positions
  EXPECT_THROW(simulator.set_stop_positions({{}}),
               cda_rail::exceptions::InvalidInputException);
  simulator.set_stop_positions({{100}, {}});
  const auto& stop_positions1 = simulator.get_stop_positions();
  EXPECT_EQ(stop_positions1.size(), 2);
  EXPECT_EQ(stop_positions1[0].size(), 1);
  EXPECT_EQ(stop_positions1[0][0], 100);
  EXPECT_TRUE(stop_positions1[1].empty());
  EXPECT_THROW(simulator.set_stop_positions_of_tr(1000, {100}),
               cda_rail::exceptions::TrainNotExistentException);
  EXPECT_THROW(simulator.set_stop_positions_of_tr(tr1, {100, 200, 300}),
               cda_rail::exceptions::InvalidInputException);
  simulator.set_stop_positions_of_tr(tr1, {150});
  const auto& stop_positions2 = simulator.get_stop_positions_of_tr(tr1);
  EXPECT_EQ(stop_positions2.size(), 1);
  EXPECT_EQ(stop_positions2[0], 150);

  EXPECT_THROW(simulator.get_stop_positions_of_tr(1000),
               cda_rail::exceptions::TrainNotExistentException);

  simulator.set_stop_positions_of_tr(tr1, {});
  const auto& stop_positions3 = simulator.get_stop_positions_of_tr(tr1);
  EXPECT_TRUE(stop_positions3.empty());
  const auto& stop_positions4 = simulator.get_stop_positions_of_tr(tr2);
  EXPECT_TRUE(stop_positions4.empty());

  EXPECT_THROW(simulator.append_stop_position_to_tr(tr1, -100),
               cda_rail::exceptions::InvalidInputException);

  simulator.append_stop_position_to_tr(tr1, 300);
  const auto& stop_positions5 = simulator.get_stop_positions_of_tr(tr1);
  EXPECT_EQ(stop_positions5.size(), 1);
  EXPECT_EQ(stop_positions5[0], 300);
  const auto& stop_positions6 = simulator.get_stop_positions_of_tr(tr2);
  EXPECT_TRUE(stop_positions6.empty());

  EXPECT_THROW(simulator.append_stop_position_to_tr(1000, 500),
               cda_rail::exceptions::TrainNotExistentException);
  EXPECT_THROW(simulator.append_stop_position_to_tr(tr1, 200),
               cda_rail::exceptions::ConsistencyException);
  simulator.append_stop_position_to_tr(tr1, 400);
  const auto& stop_positions7 = simulator.get_stop_positions_of_tr(tr1);
  EXPECT_EQ(stop_positions7.size(), 2);
  EXPECT_EQ(stop_positions7[0], 300);
  EXPECT_EQ(stop_positions7[1], 400);
  EXPECT_THROW(simulator.append_stop_position_to_tr(tr1, 500),
               cda_rail::exceptions::ConsistencyException);

  EXPECT_THROW(simulator.append_stop_position_to_tr(tr2, 500),
               cda_rail::exceptions::ConsistencyException);

  simulator.set_train_edges_of_tr(tr1, {});
  simulator.set_stop_positions_of_tr(tr1, {});
  EXPECT_THROW(simulator.append_current_stop_position_of_tr(tr1),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_THROW(simulator.append_current_stop_position_of_tr(1000),
               cda_rail::exceptions::TrainNotExistentException);
  simulator.append_train_edge_to_tr(tr1, l0_l1);
  EXPECT_THROW(simulator.append_current_stop_position_of_tr(tr1),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_THROW(simulator.append_stop_edge_to_tr(tr1, g00_g01),
               cda_rail::exceptions::ConsistencyException);
  simulator.set_train_edges_of_tr(
      tr1, {l0_l1, l1_l2, l2_l3, l3_g00, g00_g01, g01_r2, r2_r1});
  EXPECT_THROW(simulator.append_current_stop_position_of_tr(tr1),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_THROW(simulator.append_stop_edge_to_tr(1000, g00_g01),
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
  EXPECT_THROW(simulator.append_current_stop_position_of_tr(tr1),
               cda_rail::exceptions::ConsistencyException);
}

// ---------------------------
// Test private functions
// ---------------------------

TEST(GreedySimulator, BasicPrivateFunctions) {
  // Create instance
  Network     network("./example-networks/SimpleStation/network/");
  const auto& ttd_sections = network.unbreakable_sections();
  const auto& l0           = network.get_vertex_index("l0");
  const auto& r0           = network.get_vertex_index("r0");

  const auto& l0_l1   = network.get_edge_index("l0", "l1");
  const auto& l1_l2   = network.get_edge_index("l1", "l2");
  const auto& l2_l3   = network.get_edge_index("l2", "l3");
  const auto& l3_g00  = network.get_edge_index("l3", "g00");
  const auto& g00_g01 = network.get_edge_index("g00", "g01");
  const auto& g01_r2  = network.get_edge_index("g01", "r2");
  const auto& r2_r1   = network.get_edge_index("r2", "r1");
  const auto& r1_r0   = network.get_edge_index("r1", "r0");
  const auto& r0_r1   = network.get_edge_index("r0", "r1");

  GeneralTimetable<GeneralSchedule<GeneralScheduledStop>> timetable;
  const auto tr1 = timetable.add_train("Train1", 100, 10, 1, 2, true, {0, 60},
                                       0, "l0", {360, 420}, 10, "r0", network);
  const auto tr2 = timetable.add_train("Train2", 100, 10, 1, 3, false, {30, 90},
                                       10, "r0", {400, 460}, 5, "l0", network);
  const auto tr3 = timetable.add_train("Train3", 100, 10, 1, 4, true, {0, 150},
                                       0, "l0", {360, 420}, 10, "r0", network);
  const auto tr4 = timetable.add_train("Train4", 100, 10, 1, 5, false, {30, 90},
                                       0, "l0", {400, 460}, 10, "r0", network);
  const auto tr5 =
      timetable.add_train("Train5", 100, 10, 1, 6, true, {120, 180}, 0, "l0",
                          {360, 420}, 10, "r0", network);
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
  EXPECT_EQ(simulator.braking_distance(tr1, 0), 0.0);
  EXPECT_EQ(simulator.braking_distance(tr1, -EPS / 2), 0.0);
  EXPECT_EQ(simulator.braking_distance(tr1, 1), 1.0 / 4.0);
  EXPECT_EQ(simulator.braking_distance(tr1, 2), 1.0);
  EXPECT_EQ(simulator.braking_distance(tr1, 3), 9.0 / 4.0);
  EXPECT_EQ(simulator.braking_distance(tr2, 0), 0.0);
  EXPECT_EQ(simulator.braking_distance(tr2, 1), 1.0 / 6.0);
  EXPECT_EQ(simulator.braking_distance(tr2, 2), 2.0 / 3.0);
  EXPECT_EQ(simulator.braking_distance(tr2, 3), 3.0 / 2.0);
  EXPECT_THROW(simulator.braking_distance(1000, 1),
               cda_rail::exceptions::TrainNotExistentException);
  EXPECT_THROW(simulator.braking_distance(tr1, -1),
               cda_rail::exceptions::InvalidInputException);

  // Trains entering
  const auto& [success_0, entering_tr_0] =
      simulator.get_entering_trains(0, {}, {}, false);
  EXPECT_TRUE(success_0);
  // Expect only tr1
  EXPECT_EQ(entering_tr_0.size(), 1);
  EXPECT_TRUE(entering_tr_0.contains(tr1));

  const auto& [success_30, entering_tr_30] =
      simulator.get_entering_trains(30, {}, {}, false);
  EXPECT_TRUE(success_30);
  // Expect only tr1
  EXPECT_EQ(entering_tr_30.size(), 1);
  EXPECT_TRUE(entering_tr_30.contains(tr1));

  const auto& [success_30b, entering_tr_30b] =
      simulator.get_entering_trains(30, {}, {tr1}, false);
  EXPECT_TRUE(success_30b);
  // Expect only tr2, tr3
  EXPECT_EQ(entering_tr_30b.size(), 2);
  EXPECT_TRUE(entering_tr_30b.contains(tr2));
  EXPECT_TRUE(entering_tr_30b.contains(tr3));

  const auto& [success_30c, entering_tr_30c] =
      simulator.get_entering_trains(30, {tr1}, {}, false);
  EXPECT_TRUE(success_30c);
  // Expect only tr3
  EXPECT_EQ(entering_tr_30c.size(), 1);
  EXPECT_TRUE(entering_tr_30c.contains(tr3));

  const auto& [success_60, entering_tr_60] =
      simulator.get_entering_trains(60, {}, {}, false);
  EXPECT_TRUE(success_60);
  // Expect tr1
  EXPECT_EQ(entering_tr_60.size(), 1);
  EXPECT_TRUE(entering_tr_60.contains(tr1));

  const auto& [success_61, entering_tr_61] =
      simulator.get_entering_trains(61, {}, {}, false);
  EXPECT_FALSE(success_61); // tr1 too late
  EXPECT_EQ(entering_tr_61.size(), 1);
  EXPECT_TRUE(entering_tr_61.contains(tr1));

  const auto& [success_61_t, entering_tr_61_t] =
      simulator.get_entering_trains(61, {}, {}, true);
  EXPECT_TRUE(success_61_t); // tr1 still entering
  EXPECT_EQ(entering_tr_61_t.size(), 1);
  EXPECT_TRUE(entering_tr_61_t.contains(tr1));

  const auto& [success_30_tr1tr2, entering_tr_30_tr1tr2] =
      simulator.get_entering_trains(30, {tr1, tr2}, {}, false);
  EXPECT_TRUE(success_30_tr1tr2);
  // Expect tr3
  EXPECT_EQ(entering_tr_30_tr1tr2.size(), 1);
  EXPECT_TRUE(entering_tr_30_tr1tr2.contains(tr3));

  const auto& [success_30_tr1tr2_l, entering_tr_30_tr1tr2_l] =
      simulator.get_entering_trains(30, {tr2}, {tr1}, false);
  EXPECT_TRUE(success_30_tr1tr2_l);
  // Expect tr3
  EXPECT_EQ(entering_tr_30_tr1tr2_l.size(), 1);
  EXPECT_TRUE(entering_tr_30_tr1tr2_l.contains(tr3));

  const auto& [success_60_tr1tr3, entering_tr_60_tr1tr3] =
      simulator.get_entering_trains(60, {tr3}, {tr1}, false);
  EXPECT_TRUE(success_60_tr1tr3);
  // Expect tr2
  EXPECT_EQ(entering_tr_60_tr1tr3.size(), 1);
  EXPECT_TRUE(entering_tr_60_tr1tr3.contains(tr2));

  const auto& [success_60_tr1tr2tr3, entering_tr_60_tr1tr2tr3] =
      simulator.get_entering_trains(60, {tr2}, {tr1, tr3}, false);
  EXPECT_TRUE(success_60_tr1tr2tr3);
  // Expect no train to enter
  EXPECT_TRUE(entering_tr_60_tr1tr2tr3.empty());

  const auto& [success_120_tr1tr2, entering_tr_120_tr1tr2] =
      simulator.get_entering_trains(120, {tr2}, {tr1}, false);
  EXPECT_TRUE(success_120_tr1tr2);
  // Expect tr3
  EXPECT_EQ(entering_tr_120_tr1tr2.size(), 1);
  EXPECT_TRUE(entering_tr_120_tr1tr2.contains(tr3));

  const auto& [success_120_tr1tr2tr3, entering_tr_120_tr1tr2tr3] =
      simulator.get_entering_trains(120, {tr2, tr3}, {tr1}, false);
  EXPECT_TRUE(success_120_tr1tr2tr3);
  // Expect tr5
  EXPECT_EQ(entering_tr_120_tr1tr2tr3.size(), 1);
  EXPECT_TRUE(entering_tr_120_tr1tr2tr3.contains(tr5));

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

  EXPECT_THROW(simulator.edge_milestones(1000),
               cda_rail::exceptions::TrainNotExistentException);
}

TEST(GreedySimulator, TrainsOnEdges) {
  // Create instance
  Network     network("./example-networks/SimpleStation/network/");
  const auto& ttd_sections = network.unbreakable_sections();
  const auto& l0           = network.get_vertex_index("l0");
  const auto& r0           = network.get_vertex_index("r0");

  const auto& l0_l1   = network.get_edge_index("l0", "l1");
  const auto& l1_l2   = network.get_edge_index("l1", "l2");
  const auto& l2_l3   = network.get_edge_index("l2", "l3");
  const auto& l3_g00  = network.get_edge_index("l3", "g00");
  const auto& g00_g01 = network.get_edge_index("g00", "g01");

  GeneralTimetable<GeneralSchedule<GeneralScheduledStop>> timetable;
  const auto tr1 = timetable.add_train("Train1", 100, 10, 1, 2, true, {0, 60},
                                       0, "l0", {360, 420}, 10, "r0", network);
  const auto tr2 = timetable.add_train("Train2", 100, 10, 1, 3, false, {30, 90},
                                       10, "r0", {400, 460}, 5, "l0", network);
  const auto tr3 = timetable.add_train("Train3", 100, 10, 1, 4, true, {0, 150},
                                       0, "l0", {360, 420}, 10, "r0", network);
  const auto tr4 = timetable.add_train("Train4", 100, 10, 1, 5, false, {30, 90},
                                       0, "l0", {400, 460}, 10, "r0", network);
  const auto tr5 =
      timetable.add_train("Train5", 100, 10, 1, 6, true, {120, 180}, 0, "l0",
                          {360, 420}, 10, "r0", network);
  EXPECT_TRUE(timetable.check_consistency(network));

  RouteMap routes;

  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);

  cda_rail::simulator::GreedySimulator simulator(instance, ttd_sections);

  simulator.append_train_edge_to_tr(tr1, l0_l1);
  simulator.append_train_edge_to_tr(tr1, l1_l2);
  simulator.append_train_edge_to_tr(tr1, l2_l3);
  simulator.append_train_edge_to_tr(tr2, l0_l1);
  simulator.append_train_edge_to_tr(tr3, l1_l2);
  simulator.append_train_edge_to_tr(tr4, l0_l1);
  simulator.append_train_edge_to_tr(tr4, l3_g00);

  const auto tr_on_edges = simulator.tr_on_edges();

  EXPECT_EQ(tr_on_edges.size(), network.number_of_edges());
  for (size_t i = 0; i < network.number_of_edges(); ++i) {
    if (i == l0_l1) {
      EXPECT_EQ(tr_on_edges.at(i).size(), 3);
      EXPECT_TRUE(tr_on_edges.at(i).contains(tr1));
      EXPECT_TRUE(tr_on_edges.at(i).contains(tr2));
      EXPECT_TRUE(tr_on_edges.at(i).contains(tr4));
    } else if (i == l1_l2) {
      EXPECT_EQ(tr_on_edges.at(i).size(), 2);
      EXPECT_TRUE(tr_on_edges.at(i).contains(tr1));
      EXPECT_TRUE(tr_on_edges.at(i).contains(tr3));
    } else if (i == l2_l3) {
      EXPECT_EQ(tr_on_edges.at(i).size(), 1);
      EXPECT_TRUE(tr_on_edges.at(i).contains(tr1));
    } else if (i == l3_g00) {
      EXPECT_EQ(tr_on_edges.at(i).size(), 1);
      EXPECT_TRUE(tr_on_edges.at(i).contains(tr4));
    } else {
      EXPECT_TRUE(tr_on_edges.at(i).empty());
    }
  }
}

TEST(GreedySimulator, EdgePositions) {
  // Create instance
  Network     network("./example-networks/SimpleStation/network/");
  const auto& l0 = network.get_vertex_index("l0");
  const auto& r0 = network.get_vertex_index("r0");

  const auto& l0_l1   = network.get_edge_index("l0", "l1");
  const auto& l1_l2   = network.get_edge_index("l1", "l2");
  const auto& l2_l3   = network.get_edge_index("l2", "l3");
  const auto& l3_g00  = network.get_edge_index("l3", "g00");
  const auto& l3_g10  = network.get_edge_index("l3", "g10");
  const auto& g00_g01 = network.get_edge_index("g00", "g01");

  GeneralTimetable<GeneralSchedule<GeneralScheduledStop>> timetable;
  const auto tr1 = timetable.add_train("Train1", 100, 10, 1, 2, true, {0, 60},
                                       0, "l0", {360, 420}, 10, "r0", network);
  const auto tr2 = timetable.add_train("Train2", 100, 10, 1, 3, false, {30, 90},
                                       10, "r0", {400, 460}, 5, "l0", network);
  const auto tr3 = timetable.add_train("Train3", 100, 10, 1, 4, true, {0, 150},
                                       0, "l0", {360, 420}, 10, "r0", network);
  const auto tr4 = timetable.add_train("Train4", 100, 10, 1, 5, false, {30, 90},
                                       0, "l0", {400, 460}, 10, "r0", network);
  const auto tr5 =
      timetable.add_train("Train5", 100, 10, 1, 6, true, {120, 180}, 0, "l0",
                          {360, 420}, 10, "r0", network);
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
  EXPECT_THROW(simulator.get_ttd(1000),
               cda_rail::exceptions::EdgeNotExistentException);

  simulator.set_vertex_orders_of_vertex(r0, {tr2});
  simulator.set_vertex_orders_of_vertex(l0, {tr1, tr3, tr5});
  simulator.append_train_edge_to_tr(tr1, l0_l1);
  simulator.append_train_edge_to_tr(tr1, l1_l2);
  simulator.append_train_edge_to_tr(tr1, l2_l3);
  simulator.append_train_edge_to_tr(tr1, l3_g00);
  simulator.append_train_edge_to_tr(tr1, g00_g01);
  simulator.append_train_edge_to_tr(tr3, l0_l1);

  // Edge position
  const auto& [on_edge, occupation, pos] =
      simulator.get_position_on_edge(tr1, {-100, 0}, l0_l1);
  EXPECT_FALSE(on_edge);
  EXPECT_FALSE(occupation.first);
  EXPECT_FALSE(occupation.second);
  EXPECT_EQ(pos.first, 0);
  EXPECT_EQ(pos.second, 0);

  const auto& [on_edge2, occupation2, pos2] =
      simulator.get_position_on_edge(tr1, {-50, 50}, l0_l1);
  EXPECT_TRUE(on_edge2);
  EXPECT_FALSE(occupation2.first);
  EXPECT_TRUE(occupation2.second);
  EXPECT_EQ(pos2.first, 0);
  EXPECT_EQ(pos2.second, 50);

  const auto& [on_edge3, occupation3, pos3] =
      simulator.get_position_on_edge(tr1, {-50, 50}, l1_l2);
  EXPECT_FALSE(on_edge3);
  EXPECT_FALSE(occupation3.first);
  EXPECT_FALSE(occupation3.second);
  EXPECT_EQ(pos3.first, 0);
  EXPECT_EQ(pos3.second, -450);

  const auto& [on_edge4, occupation4, pos4] =
      simulator.get_position_on_edge(tr1, {400, 500}, l0_l1);
  EXPECT_TRUE(on_edge4);
  EXPECT_TRUE(occupation4.first);
  EXPECT_TRUE(occupation4.second);
  EXPECT_EQ(pos4.first, 400);
  EXPECT_EQ(pos4.second, 500);

  const auto& [on_edge4b, occupation4b, pos4b] =
      simulator.get_position_on_edge(tr1, {500, 600}, l0_l1);
  EXPECT_FALSE(on_edge4b);
  EXPECT_FALSE(occupation4b.first);
  EXPECT_FALSE(occupation4b.second);
  EXPECT_EQ(pos4b.first, 500);
  EXPECT_EQ(pos4b.second, 500);

  const auto& [on_edge4c, occupation4c, pos4c] =
      simulator.get_position_on_edge(tr1, {500, 600}, l1_l2);
  EXPECT_TRUE(on_edge4c);
  EXPECT_TRUE(occupation4c.first);
  EXPECT_TRUE(occupation4c.second);
  EXPECT_EQ(pos4c.first, 0);
  EXPECT_EQ(pos4c.second, 100);

  const auto& [on_edge5, occupation5, pos5] =
      simulator.get_position_on_edge(tr1, {920, 1020}, l0_l1);
  EXPECT_FALSE(on_edge5);
  EXPECT_FALSE(occupation5.first);
  EXPECT_FALSE(occupation5.second);
  EXPECT_EQ(pos5.first, 920);
  EXPECT_EQ(pos5.second, 500);

  const auto& [on_edge6, occupation6, pos6] =
      simulator.get_position_on_edge(tr1, {920, 1020}, l1_l2);
  EXPECT_TRUE(on_edge6);
  EXPECT_TRUE(occupation6.first);
  EXPECT_FALSE(occupation6.second);
  EXPECT_EQ(pos6.first, 420);
  EXPECT_EQ(pos6.second, 500);

  const auto& [on_edge6b, occupation6b, pos6b] =
      simulator.get_position_on_edge(tr1, {1020, 1120}, l1_l2);
  EXPECT_FALSE(on_edge6b);
  EXPECT_FALSE(occupation6b.first);
  EXPECT_FALSE(occupation6b.second);
  EXPECT_EQ(pos6b.first, 520);
  EXPECT_EQ(pos6b.second, 500);

  const auto& [on_edge6c, occupation6c, pos6c] =
      simulator.get_position_on_edge(tr1, {950, 1020}, l1_l2);
  EXPECT_TRUE(on_edge6c);
  EXPECT_TRUE(occupation6c.first);
  EXPECT_FALSE(occupation6c.second);
  EXPECT_EQ(pos6c.first, 450);
  EXPECT_EQ(pos6c.second, 500);

  const auto& [on_edge7, occupation7, pos7] =
      simulator.get_position_on_edge(tr1, {920, 1020}, l2_l3);
  EXPECT_TRUE(on_edge7);
  EXPECT_FALSE(occupation7.first);
  EXPECT_FALSE(occupation7.second);
  EXPECT_EQ(pos7.first, 0);
  EXPECT_EQ(pos7.second, 5);

  const auto& [on_edge8, occupation8, pos8] =
      simulator.get_position_on_edge(tr1, {920, 1020}, l3_g00);
  EXPECT_TRUE(on_edge8);
  EXPECT_FALSE(occupation8.first);
  EXPECT_FALSE(occupation8.second);
  EXPECT_EQ(pos8.first, 0);
  EXPECT_EQ(pos8.second, 5);

  const auto& [on_edge9, occupation9, pos9] =
      simulator.get_position_on_edge(tr1, {920, 1020}, g00_g01);
  EXPECT_TRUE(on_edge9);
  EXPECT_FALSE(occupation9.first);
  EXPECT_TRUE(occupation9.second);
  EXPECT_EQ(pos9.first, 0);
  EXPECT_EQ(pos9.second, 10);

  const auto& [on_edge10, occupation10, pos10] = simulator.get_position_on_edge(
      tr1, {0, 100}, l0_l1, {0, 10, 20, 30, 40, 50});
  EXPECT_TRUE(on_edge10);
  EXPECT_TRUE(occupation10.first);
  EXPECT_FALSE(occupation10.second);
  EXPECT_EQ(pos10.first, 0);
  EXPECT_EQ(pos10.second, 10);

  EXPECT_THROW(
      simulator.get_position_on_edge(tr1, {0, 100}, l0_l1, {0, 10, 20, 30, 40}),
      cda_rail::exceptions::ConsistencyException);
  EXPECT_THROW(simulator.get_position_on_edge(tr1, {0, 100}, l0_l1,
                                              {0, 10, 20, 30, 40, 50, 60}),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_THROW(simulator.get_position_on_edge(1000, {0, 100}, l0_l1),
               cda_rail::exceptions::TrainNotExistentException);
  EXPECT_THROW(simulator.get_position_on_edge(tr1, {0, 100}, 1000),
               cda_rail::exceptions::EdgeNotExistentException);
  EXPECT_THROW(simulator.get_position_on_edge(tr3, {0, 100}, l2_l3),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_THROW(simulator.get_position_on_route_edge(1000, {0, 100}, 0),
               cda_rail::exceptions::TrainNotExistentException);
  EXPECT_THROW(simulator.get_position_on_route_edge(tr1, {0, 100}, 5),
               cda_rail::exceptions::InvalidInputException);

  // Is on route
  EXPECT_TRUE(simulator.is_on_route(tr1, l3_g00));
  EXPECT_FALSE(simulator.is_on_route(tr1, l3_g10));
  EXPECT_THROW(simulator.is_on_route(1000, l3_g00),
               cda_rail::exceptions::TrainNotExistentException);
  EXPECT_THROW(simulator.is_on_route(tr1, 1000),
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
  EXPECT_THROW(simulator.is_on_ttd(1000, 1, {900, 1000}),
               cda_rail::exceptions::TrainNotExistentException);
  EXPECT_THROW(simulator.is_on_ttd(tr1, 2, {900, 1000}),
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
  EXPECT_THROW(simulator.is_behind_ttd(1000, 1, {900, 1000}),
               cda_rail::exceptions::TrainNotExistentException);
  EXPECT_THROW(simulator.is_behind_ttd(tr1, 2, {900, 1000}),
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
  EXPECT_THROW(simulator.is_on_or_behind_ttd(1000, 1, {900, 1000}),
               cda_rail::exceptions::TrainNotExistentException);
  EXPECT_THROW(simulator.is_on_or_behind_ttd(tr1, 2, {900, 1000}),
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

  const auto v2_v3   = network.add_edge("v2", "v3", 10, 55, false);
  const auto v11_v2  = network.add_edge("v11", "v2", 10, 30, false);
  const auto v10_v2  = network.add_edge("v10", "v2", 10, 55, false);
  const auto v01_v11 = network.add_edge("v01", "v11", 101, 30, true);
  const auto v3_v4   = network.add_edge("v3", "v4", 100, 55, true);
  const auto v00_v10 = network.add_edge("v00", "v10", 100, 55, true);

  network.add_successor(v00_v10, v10_v2);
  network.add_successor(v10_v2, v2_v3);
  network.add_successor(v2_v3, v3_v4);
  network.add_successor(v01_v11, v11_v2);
  network.add_successor(v11_v2, v2_v3);

  GeneralTimetable<GeneralSchedule<GeneralScheduledStop>> timetable;
  const auto                                              tr1 =
      timetable.add_train("Train1", 50, 55, 1, 1, true, {0, 60}, 15, "v01",
                          {360, 420}, 10, "v4", network);
  const auto tr2 =
      timetable.add_train("Train2", 50, 55, 1, 2, true, {0, 60}, 20, "v01",
                          {360, 420}, 10, "v4", network);
  const auto tr3 =
      timetable.add_train("Train3", 50, 55, 1, 3, true, {0, 60}, 25, "v00",
                          {360, 420}, 10, "v4", network);
  const auto tr4 =
      timetable.add_train("Train4", 50, 55, 1, 1, true, {0, 60}, 15, "v01",
                          {360, 420}, 10, "v4", network);
  const auto tr5 =
      timetable.add_train("Train5", 50, 55, 1, 3, true, {0, 60}, 30, "v00",
                          {360, 420}, 10, "v4", network);
  const auto tr6 =
      timetable.add_train("Train6", 50, 55, 1, 2, true, {0, 60}, 20, "v00",
                          {360, 420}, 10, "v4", network);

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

  std::vector<std::pair<double, double>> train_pos = {
      {-1, -1}, // tr1
      {-1, -1}, // tr2
      {-1, -1}, // tr3
      {-1, -1}, // tr4
      {-1, -1}, // tr5
      {-1, -1}  // tr6
  };
  EXPECT_TRUE(simulator.is_ok_to_enter(tr1, train_pos, {}, tr_on_edges));

  EXPECT_TRUE(simulator.is_ok_to_enter(tr2, train_pos, {}, tr_on_edges));
  train_pos[tr1] = {50, 100};
  EXPECT_FALSE(simulator.is_ok_to_enter(tr2, train_pos, {tr1}, tr_on_edges));
  train_pos[tr1] = {100.1, 150.1};
  EXPECT_TRUE(simulator.is_ok_to_enter(tr2, train_pos, {tr1}, tr_on_edges));
  train_pos[tr1] = {200, 250};
  EXPECT_TRUE(simulator.is_ok_to_enter(tr2, train_pos, {tr1}, tr_on_edges));

  EXPECT_FALSE(simulator.is_ok_to_enter(tr3, train_pos, {tr1}, tr_on_edges));
  train_pos[tr2] = {40, 90};
  EXPECT_FALSE(
      simulator.is_ok_to_enter(tr3, train_pos, {tr1, tr2}, tr_on_edges));
  train_pos[tr2] = {55, 105};
  EXPECT_FALSE(
      simulator.is_ok_to_enter(tr3, train_pos, {tr1, tr2}, tr_on_edges));
  train_pos[tr2] = {100, 150};
  EXPECT_FALSE(
      simulator.is_ok_to_enter(tr3, train_pos, {tr1, tr2}, tr_on_edges));
  train_pos[tr2] = {112, 162};
  EXPECT_FALSE(
      simulator.is_ok_to_enter(tr3, train_pos, {tr1, tr2}, tr_on_edges));
  train_pos[tr2] = {120, 170};
  EXPECT_FALSE(
      simulator.is_ok_to_enter(tr3, train_pos, {tr1, tr2}, tr_on_edges));
  train_pos[tr2] = {121, 171};
  EXPECT_TRUE(
      simulator.is_ok_to_enter(tr3, train_pos, {tr1, tr2}, tr_on_edges));
  train_pos[tr1] = {220, 270};
  train_pos[tr2] = {200, 250};
  EXPECT_TRUE(simulator.is_ok_to_enter(tr3, train_pos, {tr2}, tr_on_edges));

  EXPECT_FALSE(simulator.is_ok_to_enter(tr4, train_pos, {tr2}, tr_on_edges));
  train_pos[tr3] = {40, 90};
  EXPECT_FALSE(
      simulator.is_ok_to_enter(tr4, train_pos, {tr2, tr3}, tr_on_edges));
  train_pos[tr3] = {55, 105};
  EXPECT_FALSE(
      simulator.is_ok_to_enter(tr4, train_pos, {tr2, tr3}, tr_on_edges));
  train_pos[tr3] = {100, 150};
  EXPECT_FALSE(
      simulator.is_ok_to_enter(tr4, train_pos, {tr2, tr3}, tr_on_edges));
  train_pos[tr3] = {112, 162};
  EXPECT_FALSE(
      simulator.is_ok_to_enter(tr4, train_pos, {tr2, tr3}, tr_on_edges));
  train_pos[tr3] = {120, 170};
  EXPECT_TRUE(
      simulator.is_ok_to_enter(tr4, train_pos, {tr2, tr3}, tr_on_edges));
  train_pos[tr2] = {220, 270};
  train_pos[tr3] = {200, 250};
  EXPECT_TRUE(simulator.is_ok_to_enter(tr4, train_pos, {tr3}, tr_on_edges));

  EXPECT_FALSE(simulator.is_ok_to_enter(tr5, train_pos, {tr3}, tr_on_edges));
  train_pos[tr4] = {40, 90};
  EXPECT_FALSE(
      simulator.is_ok_to_enter(tr5, train_pos, {tr3, tr4}, tr_on_edges));
  train_pos[tr4] = {55, 105};
  EXPECT_FALSE(
      simulator.is_ok_to_enter(tr5, train_pos, {tr3, tr4}, tr_on_edges));
  train_pos[tr4] = {100, 150};
  EXPECT_FALSE(
      simulator.is_ok_to_enter(tr5, train_pos, {tr3, tr4}, tr_on_edges));
  train_pos[tr4] = {101, 151};
  EXPECT_FALSE(
      simulator.is_ok_to_enter(tr5, train_pos, {tr3, tr4}, tr_on_edges));
  train_pos[tr4] = {121, 171};
  EXPECT_FALSE(
      simulator.is_ok_to_enter(tr5, train_pos, {tr3, tr4}, tr_on_edges));
  train_pos[tr4] = {150.1, 200.1};
  EXPECT_FALSE(
      simulator.is_ok_to_enter(tr5, train_pos, {tr3, tr4}, tr_on_edges));
  train_pos[tr4] = {151.1, 201.1};
  EXPECT_TRUE(
      simulator.is_ok_to_enter(tr5, train_pos, {tr3, tr4}, tr_on_edges));
  train_pos[tr3] = {220, 270};
  train_pos[tr4] = {200, 250};
  EXPECT_TRUE(simulator.is_ok_to_enter(tr5, train_pos, {tr4}, tr_on_edges));

  train_pos[tr5] = {99.9, 149.9};
  EXPECT_FALSE(
      simulator.is_ok_to_enter(tr6, train_pos, {tr4, tr5}, tr_on_edges));
  train_pos[tr5] = {100.1, 150.1};
  EXPECT_TRUE(
      simulator.is_ok_to_enter(tr6, train_pos, {tr4, tr5}, tr_on_edges));
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

  const auto v3_v4   = network.add_edge("v3", "v4", 100, 55, true);
  const auto v10_v2  = network.add_edge("v10", "v2", 10, 55, false);
  const auto v11_v2  = network.add_edge("v11", "v2", 10, 30, false);
  const auto v2_v3   = network.add_edge("v2", "v3", 10, 55, false);
  const auto v00_v10 = network.add_edge("v00", "v10", 100, 55, true);
  const auto v01_v11 = network.add_edge("v01", "v11", 101, 30, true);

  network.add_successor(v00_v10, v10_v2);
  network.add_successor(v10_v2, v2_v3);
  network.add_successor(v2_v3, v3_v4);
  network.add_successor(v01_v11, v11_v2);
  network.add_successor(v11_v2, v2_v3);

  GeneralTimetable<GeneralSchedule<GeneralScheduledStop>> timetable;
  const auto                                              tr1 =
      timetable.add_train("Train1", 50, 55, 1, 1, true, {0, 60}, 15, "v01",
                          {360, 420}, 10, "v4", network);
  const auto tr2 =
      timetable.add_train("Train2", 50, 55, 1, 2, true, {0, 60}, 20, "v00",
                          {360, 420}, 10, "v4", network);
  const auto tr3 =
      timetable.add_train("Train3", 50, 55, 1, 3, true, {0, 60}, 25, "v00",
                          {360, 420}, 10, "v4", network);

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

  std::vector<std::pair<double, double>> train_pos = {
      {-50, 0}, // tr1
      {-50, 0}, // tr2
      {-50, 0}, // tr3
  };
  std::vector<double> train_velocities(3, 0);

  EXPECT_EQ(simulator.get_absolute_distance_ma(
                tr1, 200, train_pos, train_velocities, {tr1}, {}, tr_on_edges),
            200);
  train_pos[tr1] = {40, 90};
  EXPECT_EQ(simulator.get_absolute_distance_ma(
                tr1, 200, train_pos, train_velocities, {tr1}, {}, tr_on_edges),
            200);
  EXPECT_EQ(simulator.get_absolute_distance_ma(tr2, 200, train_pos,
                                               train_velocities, {tr1, tr2}, {},
                                               tr_on_edges),
            100);
  EXPECT_EQ(simulator.get_absolute_distance_ma(tr2, 200, train_pos,
                                               train_velocities, {tr2}, {tr1},
                                               tr_on_edges),
            200);
  train_pos[tr2] = {0, 50};
  EXPECT_EQ(simulator.get_absolute_distance_ma(tr2, 200, train_pos,
                                               train_velocities, {tr1, tr2}, {},
                                               tr_on_edges),
            50);
  train_pos[tr1] = {52, 102};
  EXPECT_EQ(simulator.get_absolute_distance_ma(tr2, 200, train_pos,
                                               train_velocities, {tr1, tr2}, {},
                                               tr_on_edges),
            50);
  train_pos[tr1] = {90, 140};
  EXPECT_EQ(simulator.get_absolute_distance_ma(tr2, 200, train_pos,
                                               train_velocities, {tr1, tr2}, {},
                                               tr_on_edges),
            50);
  train_pos[tr1] = {102, 152};
  EXPECT_EQ(simulator.get_absolute_distance_ma(tr2, 200, train_pos,
                                               train_velocities, {tr1, tr2}, {},
                                               tr_on_edges),
            50);
  train_pos[tr1] = {112, 162};
  EXPECT_EQ(simulator.get_absolute_distance_ma(tr2, 200, train_pos,
                                               train_velocities, {tr1, tr2}, {},
                                               tr_on_edges),
            50);
  train_pos[tr1] = {120, 170};
  EXPECT_EQ(simulator.get_absolute_distance_ma(tr2, 200, train_pos,
                                               train_velocities, {tr1, tr2}, {},
                                               tr_on_edges),
            50);
  train_pos[tr1] = {120, 200};
  EXPECT_EQ(simulator.get_absolute_distance_ma(tr2, 200, train_pos,
                                               train_velocities, {tr1, tr2}, {},
                                               tr_on_edges),
            50);
  train_pos[tr1] = {121, 200};
  EXPECT_EQ(simulator.get_absolute_distance_ma(tr2, 200, train_pos,
                                               train_velocities, {tr1, tr2}, {},
                                               tr_on_edges),
            70);
  train_pos[tr1] = {121.1, 200};
  EXPECT_EQ(simulator.get_absolute_distance_ma(tr2, 200, train_pos,
                                               train_velocities, {tr1, tr2}, {},
                                               tr_on_edges),
            70.1);
  train_pos[tr1] = {150, 200};
  EXPECT_EQ(simulator.get_absolute_distance_ma(tr2, 200, train_pos,
                                               train_velocities, {tr1, tr2}, {},
                                               tr_on_edges),
            99);
  EXPECT_EQ(simulator.get_absolute_distance_ma(tr2, 98, train_pos,
                                               train_velocities, {tr1, tr2}, {},
                                               tr_on_edges),
            98);
  train_pos[tr1] = {200, 250};
  EXPECT_EQ(simulator.get_absolute_distance_ma(tr3, 200, train_pos,
                                               train_velocities, {tr1, tr3}, {},
                                               tr_on_edges),
            100);
  train_pos[tr2] = {50, 100};
  EXPECT_EQ(
      simulator.get_absolute_distance_ma(tr3, 200, train_pos, train_velocities,
                                         {tr1, tr2, tr3}, {}, tr_on_edges),
      50);
  train_pos[tr2] = {105, 155};
  EXPECT_EQ(
      simulator.get_absolute_distance_ma(tr3, 200, train_pos, train_velocities,
                                         {tr1, tr2, tr3}, {}, tr_on_edges),
      100);
  EXPECT_EQ(
      simulator.get_absolute_distance_ma(tr3, 99, train_pos, train_velocities,
                                         {tr1, tr2, tr3}, {}, tr_on_edges),
      99);
  train_pos[tr3] = {50, 100};
  EXPECT_EQ(
      simulator.get_absolute_distance_ma(tr3, 200, train_pos, train_velocities,
                                         {tr1, tr2, tr3}, {}, tr_on_edges),
      0);
  train_pos[tr2] = {140, 190};
  EXPECT_EQ(
      simulator.get_absolute_distance_ma(tr3, 200, train_pos, train_velocities,
                                         {tr1, tr2, tr3}, {}, tr_on_edges),
      40);

  train_pos[tr1] = {200, 250};
  train_pos[tr2] = {160, 195};
  train_pos[tr3] = {140, 150};
  EXPECT_EQ(
      simulator.get_absolute_distance_ma(tr1, 200, train_pos, train_velocities,
                                         {tr1, tr2, tr3}, {}, tr_on_edges),
      200);
  EXPECT_EQ(
      simulator.get_absolute_distance_ma(tr2, 200, train_pos, train_velocities,
                                         {tr1, tr2, tr3}, {}, tr_on_edges),
      4);
  EXPECT_EQ(
      simulator.get_absolute_distance_ma(tr3, 200, train_pos, train_velocities,
                                         {tr1, tr2, tr3}, {}, tr_on_edges),
      10);

  EXPECT_THROW(simulator.get_absolute_distance_ma(tr3, 200, train_pos,
                                                  train_velocities, {tr1, tr2},
                                                  {}, tr_on_edges),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_THROW(
      simulator.get_absolute_distance_ma(tr3, -1, train_pos, train_velocities,
                                         {tr1, tr2, tr3}, {}, tr_on_edges),
      cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(
      simulator.get_absolute_distance_ma(1000, 200, train_pos, train_velocities,
                                         {tr1, tr2, 1000}, {}, tr_on_edges),
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

  GeneralTimetable<GeneralSchedule<GeneralScheduledStop>> timetable;
  const auto tr1 = timetable.add_train("Train1", 200, 51, 3, 2, true, {0, 60},
                                       10, v0, {360, 420}, 14, v5, network);

  RouteMap routes;

  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);

  cda_rail::simulator::GreedySimulator simulator(instance, {{}});
  simulator.set_train_edges_of_tr(tr1, {v0_v1, v1_v2, v2_v3, v3_v4, v4_v5});

  const auto& train =
      simulator.instance->get_timetable().get_train_list().get_train(tr1);

  const auto [ma1, vm1] = simulator.get_future_max_speed_constraints(
      tr1, train, 0, 10, 500, 10, true);
  EXPECT_EQ(ma1, 310);
  EXPECT_EQ(vm1, 30);
  const auto [ma1tol, vm1tol] = simulator.get_future_max_speed_constraints(
      tr1, train, -cda_rail::EPS / 2.0, 10, 500, 10, true);
  EXPECT_EQ(ma1tol, 310);
  EXPECT_EQ(vm1tol, 30);
  const auto [ma2, vm2] = simulator.get_future_max_speed_constraints(
      tr1, train, 0, 10, 500, 2, true);
  EXPECT_EQ(ma2, 310);
  EXPECT_EQ(vm2, 16);
  const auto [ma2_0, vm2_0] =
      simulator.get_future_max_speed_constraints(tr1, train, 0, 10, 0, 2, true);
  EXPECT_EQ(ma2_0, 0);
  EXPECT_EQ(vm2_0, 16);
  const auto [ma2_0tol, vm2_0tol] = simulator.get_future_max_speed_constraints(
      tr1, train, 0, 10, -cda_rail::EPS / 2.0, 2, true);
  EXPECT_EQ(ma2_0tol, 0);
  EXPECT_EQ(vm2_0tol, 16);
  const auto [ma3, vm3] = simulator.get_future_max_speed_constraints(
      tr1, train, 0, 10, 200, 10, true);
  EXPECT_EQ(ma3, 200);
  EXPECT_EQ(vm3, 30);
  const auto [ma4, vm4] = simulator.get_future_max_speed_constraints(
      tr1, train, 0, 10, 100, 10, true);
  EXPECT_EQ(ma4, 100);
  EXPECT_EQ(vm4, 40);
  const auto [ma5, vm5] = simulator.get_future_max_speed_constraints(
      tr1, train, 0, 10, 50, 10, true);
  EXPECT_EQ(ma5, 50);
  EXPECT_EQ(vm5, 40);

  const auto [ma6, vm6] = simulator.get_future_max_speed_constraints(
      tr1, train, 50, 40, 600, 10, true);
  EXPECT_EQ(ma6, 600);
  EXPECT_EQ(vm6, 20);
  const auto [ma7, vm7] = simulator.get_future_max_speed_constraints(
      tr1, train, 50, 40, 1200, 6, true);
  EXPECT_EQ(ma7, 985);
  EXPECT_EQ(vm7, 20);

  const auto [ma8, vm8] = simulator.get_future_max_speed_constraints(
      tr1, train, 250, 19, 1000, 1, true);
  EXPECT_EQ(ma8, 785);
  EXPECT_EQ(vm8, 20);
  const auto [ma9, vm9] = simulator.get_future_max_speed_constraints(
      tr1, train, 250, 19, 1000, 1, false);
  EXPECT_EQ(ma9, 785);
  EXPECT_EQ(vm9, 20);

  const auto [ma10, vm10] = simulator.get_future_max_speed_constraints(
      tr1, train, 500, 19, 1000, 1, true);
  EXPECT_EQ(ma10, 1000);
  EXPECT_EQ(vm10, 20);
  const auto [ma11, vm11] = simulator.get_future_max_speed_constraints(
      tr1, train, 500, 19, 1000, 1, false);
  EXPECT_EQ(ma11, 1000);
  EXPECT_EQ(vm11, 22);

  const auto [ma12, vm12] = simulator.get_future_max_speed_constraints(
      tr1, train, 0, 0, 1000, 1, true);
  EXPECT_EQ(ma12, 310);
  EXPECT_EQ(vm12, 3);
  const auto [ma12tol, vm12tol] = simulator.get_future_max_speed_constraints(
      tr1, train, 0, -cda_rail::EPS / 2.0, 1000, 1, true);
  EXPECT_EQ(ma12tol, 310);
  EXPECT_EQ(vm12tol, 3);

  // Test exit speed limit at 2010
  // Linear movement 20 -> 14 over 10s: 170m
  // Braking distance: 14*14/4 = 49 to 2059
  // if-case: 2010 - 170 = 1840
  const auto [ma13, vm13] = simulator.get_future_max_speed_constraints(
      tr1, train, 1800, 20, 1000, 10, false);
  EXPECT_EQ(ma13, 259);
  EXPECT_EQ(vm13, 50);

  const auto [ma13b, vm13b] = simulator.get_future_max_speed_constraints(
      tr1, train, 1800, 20, 100, 10, false);
  EXPECT_EQ(ma13b, 100);
  EXPECT_EQ(vm13b, 50);

  const auto [ma14, vm14] = simulator.get_future_max_speed_constraints(
      tr1, train, 1840, 20, 1000, 10, false);
  EXPECT_EQ(ma14, 1000);
  EXPECT_EQ(vm14, 14);

  const auto [ma15, vm15] = simulator.get_future_max_speed_constraints(
      tr1, train, 1841, 20, 1000, 10, false);
  EXPECT_EQ(ma15, 1000);
  EXPECT_EQ(vm15, 14);

  const auto [ma15b, vm15b] = simulator.get_future_max_speed_constraints(
      tr1, train, 1841, 20, 100, 10, false);
  EXPECT_EQ(ma15b, 100);
  EXPECT_EQ(vm15b, 50);

  const auto [ma16, vm16] = simulator.get_future_max_speed_constraints(
      tr1, train, 2100, 20, 1000, 10, false);
  EXPECT_EQ(ma16, 1000);
  EXPECT_EQ(vm16, 14);

  // Stopping on route edge after 510m
  simulator.set_train_edges_of_tr(tr1, {v0_v1, v1_v2, v2_v3, v3_v4});
  const auto [ma17, vm17] = simulator.get_future_max_speed_constraints(
      tr1, train, 400, 20, 1000, 5, false);
  EXPECT_EQ(ma17, 110);
  EXPECT_EQ(vm17, 20);

  const auto [ma18, vm18] = simulator.get_future_max_speed_constraints(
      tr1, train, 400, 20, 1000, 20, true);
  EXPECT_EQ(ma18, 110);
  EXPECT_EQ(vm18, 0);

  EXPECT_THROW(simulator.get_future_max_speed_constraints(tr1, train, -1, 10,
                                                          10, 10, true),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(simulator.get_future_max_speed_constraints(tr1, train, 10, -1,
                                                          10, 10, true),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(simulator.get_future_max_speed_constraints(tr1, train, 10, 10,
                                                          -1, 10, true),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(simulator.get_future_max_speed_constraints(tr1, train, 10, 10,
                                                          10, -1, true),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(simulator.get_future_max_speed_constraints(1000, train, 10, 10,
                                                          10, 10, true),
               cda_rail::exceptions::TrainNotExistentException);
}

TEST(GreedySimulator, EoMDisplacement) {
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance;
  cda_rail::simulator::GreedySimulator simulator(instance, {});

  // After 3 seconds v_1 = 0
  // x_1 = 0
  // bd = 0*0 / 4 = 0
  // x_1 + bd = 0 + 0 = 0
  EXPECT_EQ(cda_rail::max_braking_pos_after_dt_linear_movement(0, 0, 4, 2, 3),
            0);
  EXPECT_EQ(cda_rail::max_braking_pos_after_dt_linear_movement(
                0, -cda_rail::EPS / 2.0, 4, 2, 3),
            0);
  cda_rail::Train train("Train", 100, 0, 4, 2);
  EXPECT_EQ(simulator.max_displacement(train, 0, 3), 0);
  train.max_speed = -cda_rail::EPS / 2.0;
  EXPECT_EQ(simulator.max_displacement(train, 0, 3), 0);

  // After 3 seconds v_1 = 0 + 3*4 = 12
  // x_1 = (0+12)*3/2 = 18
  // bd = 12*12 / 4 = 36
  // x_1 + bd = 18 + 36 = 54
  EXPECT_EQ(cda_rail::max_braking_pos_after_dt_linear_movement(0, 30, 4, 2, 3),
            54);
  EXPECT_EQ(cda_rail::max_braking_pos_after_dt_linear_movement(
                -cda_rail::EPS / 2.0, 30, 4, 2, 3),
            54);
  train.max_speed = 30;
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
  train.max_speed = 30.0 - cda_rail::EPS / 2;
  EXPECT_EQ(simulator.max_displacement(train, 30, 3), 315);

  // dt = 0 -> v_1 = v_0 without movement
  // bd = 10*10 / 4 = 25
  EXPECT_EQ(cda_rail::max_braking_pos_after_dt_linear_movement(10, 30, 4, 2, 0),
            25);
  EXPECT_EQ(cda_rail::max_braking_pos_after_dt_linear_movement(
                10, 30, 4, 2, -cda_rail::EPS / 2),
            25);
  train.max_speed = 30;
  EXPECT_EQ(simulator.max_displacement(train, 10, 0), 25);
  EXPECT_EQ(simulator.max_displacement(train, 10, -cda_rail::EPS / 2), 25);

  EXPECT_THROW(
      cda_rail::max_braking_pos_after_dt_linear_movement(-1, 30, 4, 2, 3),
      cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(simulator.max_displacement(train, -1, 3),
               cda_rail::exceptions::InvalidInputException);

  EXPECT_THROW(
      cda_rail::max_braking_pos_after_dt_linear_movement(10, 5, 4, 2, 3),
      cda_rail::exceptions::InvalidInputException);
  train.max_speed = 5;
  EXPECT_THROW(simulator.max_displacement(train, 10, 3),
               cda_rail::exceptions::InvalidInputException);

  EXPECT_THROW(
      cda_rail::max_braking_pos_after_dt_linear_movement(10, 30, 0, 2, 3),
      cda_rail::exceptions::InvalidInputException);
  train.max_speed    = 30;
  train.acceleration = 0;
  EXPECT_THROW(simulator.max_displacement(train, 10, 3),
               cda_rail::exceptions::InvalidInputException);

  EXPECT_THROW(
      cda_rail::max_braking_pos_after_dt_linear_movement(10, 30, 4, 0, 3),
      cda_rail::exceptions::InvalidInputException);
  train.acceleration = 4;
  train.deceleration = 0;
  EXPECT_THROW(simulator.max_displacement(train, 10, 3),
               cda_rail::exceptions::InvalidInputException);

  EXPECT_THROW(
      cda_rail::max_braking_pos_after_dt_linear_movement(10, 30, 4, 2, -1),
      cda_rail::exceptions::InvalidInputException);
  train.deceleration = 2;
  EXPECT_THROW(simulator.max_displacement(train, 10, -1),
               cda_rail::exceptions::InvalidInputException);
}

TEST(GreedySimulator, NextStopMA) {
  EXPECT_EQ(cda_rail::simulator::GreedySimulator::get_next_stop_ma(10, 20, 50),
            10);
  EXPECT_EQ(cda_rail::simulator::GreedySimulator::get_next_stop_ma(50, 20, 50),
            30);
  EXPECT_EQ(cda_rail::simulator::GreedySimulator::get_next_stop_ma(10, 20, {}),
            10);
}

TEST(GreedySimulator, TimeToExitObjective) {
  // Train : a = 3, d = 4
  // v_0 = 10
  // v_1 = 14 after 5 seconds
  // x_1 = (10 + 14) * 5 / 2 = 60
  EXPECT_EQ(cda_rail::simulator::GreedySimulator::time_to_exit_objective(
                10, 14, 15, 59, 3, 4, 5)
                .second,
            5);
  // From there decelerate for 2s until speed is 14 - 2*4 = 6
  // x_2 = (6+14)* 2/2 = 20
  // x_1 + x_2 = 60 + 20 = 80
  EXPECT_EQ(cda_rail::simulator::GreedySimulator::time_to_exit_objective(
                10, 14, 6, 80, 3, 4, 5)
                .second,
            7);
  EXPECT_EQ(cda_rail::simulator::GreedySimulator::time_to_exit_objective(
                10, 14, 5, 80, 3, 4, 5)
                .second,
            7);
  // From x_2 accelerate for 4s until speed is 6 + 4*3 = 18
  // x_3 = (18+6)* 4/2 = 48
  // x_1 + x_2 + x_3 = 60 + 20 + 48 = 128
  EXPECT_EQ(cda_rail::simulator::GreedySimulator::time_to_exit_objective(
                10, 14, 18, 128, 3, 4, 5)
                .second,
            11);

  // From x_1 accelerate for 2s until speed is 14 + 2*3 = 20
  // x_2 = (20+14)* 2/2 = 34
  // x_1 + x_2 = 60 + 34 = 94
  EXPECT_EQ(cda_rail::simulator::GreedySimulator::time_to_exit_objective(
                10, 14, 20, 94, 3, 4, 5)
                .second,
            7);
  EXPECT_EQ(cda_rail::simulator::GreedySimulator::time_to_exit_objective(
                10, 14, 21, 94, 3, 4, 5)
                .second,
            7);

  // v_0 = 0
  // v_1 = 14 after 5 seconds
  // x_1 = (0 + 14) * 5 / 2 = 35 --> All s are 60-35 = 25 smaller
  EXPECT_EQ(cda_rail::simulator::GreedySimulator::time_to_exit_objective(
                0, 14, 15, 34, 3, 4, 5)
                .second,
            5);
  EXPECT_EQ(cda_rail::simulator::GreedySimulator::time_to_exit_objective(
                0, 14, 6, 55, 3, 4, 5)
                .second,
            7);
  EXPECT_EQ(cda_rail::simulator::GreedySimulator::time_to_exit_objective(
                0, 14, 5, 55, 3, 4, 5)
                .second,
            7);
  EXPECT_EQ(cda_rail::simulator::GreedySimulator::time_to_exit_objective(
                0, 14, 18, 103, 3, 4, 5)
                .second,
            11);
  EXPECT_EQ(cda_rail::simulator::GreedySimulator::time_to_exit_objective(
                0, 14, 20, 69, 3, 4, 5)
                .second,
            7);
  EXPECT_EQ(cda_rail::simulator::GreedySimulator::time_to_exit_objective(
                0, 14, 21, 69, 3, 4, 5)
                .second,
            7);
  EXPECT_EQ(cda_rail::simulator::GreedySimulator::time_to_exit_objective(
                -cda_rail::EPS / 2.0, 14, 15, 34, 3, 4, 5)
                .second,
            5);
  EXPECT_EQ(cda_rail::simulator::GreedySimulator::time_to_exit_objective(
                -cda_rail::EPS / 2.0, 14, 6, 55, 3, 4, 5)
                .second,
            7);
  EXPECT_EQ(cda_rail::simulator::GreedySimulator::time_to_exit_objective(
                -cda_rail::EPS / 2.0, 14, 5, 55, 3, 4, 5)
                .second,
            7);
  EXPECT_EQ(cda_rail::simulator::GreedySimulator::time_to_exit_objective(
                -cda_rail::EPS / 2.0, 14, 18, 103, 3, 4, 5)
                .second,
            11);
  EXPECT_EQ(cda_rail::simulator::GreedySimulator::time_to_exit_objective(
                -cda_rail::EPS / 2.0, 14, 20, 69, 3, 4, 5)
                .second,
            7);
  EXPECT_EQ(cda_rail::simulator::GreedySimulator::time_to_exit_objective(
                -cda_rail::EPS / 2.0, 14, 21, 69, 3, 4, 5)
                .second,
            7);

  // v_0 = 10
  // v_1 = 0 after 5 seconds
  // x_1 = (10 + 0) * 5 / 2 = 25
  EXPECT_EQ(cda_rail::simulator::GreedySimulator::time_to_exit_objective(
                10, 0, 6, 24, 3, 4, 5)
                .second,
            5);
  // It then accelerates for 2s until speed is 0 + 2*3 = 6
  // x_2 = (6 + 0) * 2 / 2 = 6
  // x_1 + x_2 = 25 + 6 = 31
  EXPECT_EQ(cda_rail::simulator::GreedySimulator::time_to_exit_objective(
                10, 0, 6, 31, 3, 4, 5)
                .second,
            std::numeric_limits<double>::infinity());
  EXPECT_EQ(cda_rail::simulator::GreedySimulator::time_to_exit_objective(
                10, 0, 6, 30, 3, 4, 5)
                .second,
            std::numeric_limits<double>::infinity());
  EXPECT_EQ(cda_rail::simulator::GreedySimulator::time_to_exit_objective(
                10, -cda_rail::EPS / 2.0, 6, 31, 3, 4, 5)
                .second,
            std::numeric_limits<double>::infinity());
  EXPECT_EQ(cda_rail::simulator::GreedySimulator::time_to_exit_objective(
                10, -cda_rail::EPS / 2.0, 6, 30, 3, 4, 5)
                .second,
            std::numeric_limits<double>::infinity());

  // s = 0
  EXPECT_EQ(cda_rail::simulator::GreedySimulator::time_to_exit_objective(
                10, 14, 18, 0, 3, 4, 5)
                .second,
            5);
  EXPECT_EQ(cda_rail::simulator::GreedySimulator::time_to_exit_objective(
                10, 14, 18, -cda_rail::EPS / 2.0, 3, 4, 5)
                .second,
            5);

  EXPECT_THROW(cda_rail::simulator::GreedySimulator::time_to_exit_objective(
                   -1, 14, 18, 128, 3, 4, 5),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(cda_rail::simulator::GreedySimulator::time_to_exit_objective(
                   10, -1, 18, 128, 3, 4, 5),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(cda_rail::simulator::GreedySimulator::time_to_exit_objective(
                   10, 14, 0, 80, 3, 4, 5),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(cda_rail::simulator::GreedySimulator::time_to_exit_objective(
                   10, 14, V_MIN / 2.0, 80, 3, 4, 5),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(cda_rail::simulator::GreedySimulator::time_to_exit_objective(
                   10, 14, 18, -1, 3, 4, 5),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(cda_rail::simulator::GreedySimulator::time_to_exit_objective(
                   10, 14, 18, 128, 0, 4, 5),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(cda_rail::simulator::GreedySimulator::time_to_exit_objective(
                   10, 14, 18, 128, EPS / 2.0, 4, 5),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(cda_rail::simulator::GreedySimulator::time_to_exit_objective(
                   10, 14, 18, 128, 3, 0, 5),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(cda_rail::simulator::GreedySimulator::time_to_exit_objective(
                   10, 14, 18, 128, 3, EPS / 2.0, 5),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(cda_rail::simulator::GreedySimulator::time_to_exit_objective(
                   10, 14, 18, 128, 3, 4, 0),
               cda_rail::exceptions::InvalidInputException);
}

TEST(GreedySimulator, ExitHeadwaySpeedConstraint) {
  Network network;
  network.add_vertex("v0", VertexType::TTD);
  network.add_vertex("v1", VertexType::TTD);
  network.add_vertex("v2", VertexType::TTD);
  network.add_vertex("v3", VertexType::TTD);

  const auto v2_v3 = network.add_edge("v2", "v3", 400, 55, true);
  const auto v1_v2 = network.add_edge("v1", "v2", 80, 55, true);
  const auto v0_v1 = network.add_edge("v0", "v1", 20, 55, true);

  network.add_successor(v0_v1, v1_v2);
  network.add_successor(v1_v2, v2_v3);

  GeneralTimetable<GeneralSchedule<GeneralScheduledStop>> timetable;
  const auto tr1 = timetable.add_train("Train1", 50, 24, 3, 4, true, {0, 60},
                                       15, "v0", {360, 420}, 12, "v3", network);

  RouteMap                                                    routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);
  cda_rail::simulator::GreedySimulator simulator(instance, {});

  simulator.append_train_edge_to_tr(tr1, v0_v1);
  simulator.append_train_edge_to_tr(tr1, v1_v2);

  const auto& train1 = simulator.instance->get_train_list().get_train(tr1);

  // Route does not reach exit vertex yet
  EXPECT_EQ(simulator.get_max_speed_exit_headway(tr1, train1, 10, 10, 120, 2),
            16);
  EXPECT_EQ(simulator.get_max_speed_exit_headway(tr1, train1, 10, 0, 120, 2),
            6);
  EXPECT_EQ(simulator.get_max_speed_exit_headway(tr1, train1, 10,
                                                 -cda_rail::EPS / 2.0, 120, 2),
            6);
  EXPECT_EQ(simulator.get_max_speed_exit_headway(tr1, train1, 10, 10, 120, 5),
            24);

  simulator.append_train_edge_to_tr(tr1, v2_v3);

  // v_0 = 0
  // v_n = 12
  // v_n^2-v_0^2 = 12^2 - 0^2 = 144 = 2*a*s = 2*3*s = 6*s
  // s = 144 / 6 = 24 --> pos = 500-24 = 476
  // This takes 12/3 = 4 seconds
  EXPECT_APPROX_EQ(
      simulator.get_max_speed_exit_headway(tr1, train1, 476, 0, 4, 2), 6,
      LINE_SPEED_ACCURACY);
  EXPECT_APPROX_EQ(simulator.get_max_speed_exit_headway(
                       tr1, train1, 476, -cda_rail::EPS / 2.0, 4, 2),
                   6, LINE_SPEED_ACCURACY);
  EXPECT_APPROX_EQ(
      simulator.get_max_speed_exit_headway(tr1, train1, 476, 0, 5, 2), 0,
      LINE_SPEED_ACCURACY);

  // v_0 = 0
  // v_1 = 3 after 5 seconds
  // x_1 = (0 + 3) * 5 / 2 = 7.5
  // v_n = 12 after additional 3 seconds
  // x_2 = (3 + 12) * 3 / 2 = 22.5
  // s = 7.5 + 22.5 = 30 --> pos = 500 - 30 = 470
  EXPECT_APPROX_EQ(
      simulator.get_max_speed_exit_headway(tr1, train1, 470, 0, 8, 5), 3,
      LINE_SPEED_ACCURACY);
  EXPECT_APPROX_EQ(
      simulator.get_max_speed_exit_headway(tr1, train1, 100, 0, 0, 5), 15,
      LINE_SPEED_ACCURACY);

  // v_0 = 0
  // v_1 = 16 after 10 seconds
  // x_1 = (0 + 16) * 10 / 2 = 80
  // v_n = 12 after additional 1 second deceleration
  // x_2 = (16 + 12) * 1 / 2 = 14
  // s = 80 + 14 = 94 --> pos = 500 - 94 = 406
  EXPECT_APPROX_EQ(
      simulator.get_max_speed_exit_headway(tr1, train1, 406, 0, 11, 10), 16,
      LINE_SPEED_ACCURACY);

  // v_0 = 5
  // v_1 = 10 after 4 seconds
  // x_1 = (5 + 10) * 4 / 2 = 30
  // Decelerate for 1 second until speed is 10 - 4 = 6
  // x_2 = (6 + 10) * 1 / 2 = 8
  // Accelerate for 2 seconds until speed is 6 + 2*3 = 12
  // x_3 = (12 + 6) * 2 / 2 = 18
  // s = 30 + 8 + 18 = 56 --> pos = 500 - 56 = 444
  // h = 4 + 1 + 2 = 7
  EXPECT_APPROX_EQ(
      simulator.get_max_speed_exit_headway(tr1, train1, 444, 5, 7, 4), 10,
      LINE_SPEED_ACCURACY);

  // v_0 = 5
  // v_1 = 8 after 2 seconds
  // x_1 = (5 + 8) * 2 / 2 = 13
  // Decelerate for 2 seconds until speed is 8 - 2*4 = 0
  // x_2 = (0 + 8) * 2 / 2 = 8
  // Accelerate for 4 seconds until speed is 0 + 4*3 = 12
  // x_3 = (12 + 0) * 4 / 2 = 24
  // s = 13 + 8 + 24 = 45 --> pos = 500 - 45 = 455
  // t = 2 + 2 + 4 = 8
  EXPECT_APPROX_EQ(
      simulator.get_max_speed_exit_headway(tr1, train1, 455, 5, 8, 2), 8,
      LINE_SPEED_ACCURACY);
  EXPECT_APPROX_EQ(
      simulator.get_max_speed_exit_headway(tr1, train1, 455, 5, 10, 2), 8,
      LINE_SPEED_ACCURACY);

  // v_0 = 10
  // v_1 = 8 after 2 seconds
  // x_1 = (10 + 8) * 2 / 2 = 18
  // -> pos = 500 - 18 = 482
  EXPECT_APPROX_EQ(
      simulator.get_max_speed_exit_headway(tr1, train1, 482, 10, 2, 2), 8,
      LINE_SPEED_ACCURACY);

  EXPECT_THROW(simulator.get_max_speed_exit_headway(tr1, train1, -1, 5, 8, 2),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(simulator.get_max_speed_exit_headway(tr1, train1, 455, -1, 8, 2),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(simulator.get_max_speed_exit_headway(tr1, train1, 455, 5, -1, 2),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(simulator.get_max_speed_exit_headway(tr1, train1, 455, 5, 8, -1),
               cda_rail::exceptions::InvalidInputException);
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

  const auto v0t_v1t = network.add_edge("v0t", "v1t", 800, 50, true);
  const auto v0b_v1b = network.add_edge("v0b", "v1b", 100, 5, true);
  const auto v1t_v2t = network.add_edge("v1t", "v2t", 100, 5, true);
  const auto v1b_v2b = network.add_edge("v1b", "v2b", 100, 10, true);
  const auto v2t_v3  = network.add_edge("v2t", "v3", 50, 50, false);
  const auto v2b_v3  = network.add_edge("v2b", "v3", 50, 50, false);
  const auto v3_v4   = network.add_edge("v3", "v4", 50, 50, false);
  const auto v4_v5   = network.add_edge("v4", "v5", 1000, 50, true);
  const auto v5_v6   = network.add_edge("v5", "v6", 100, 50, true);

  network.add_successor(v0t_v1t, v1t_v2t);
  network.add_successor(v0b_v1b, v1b_v2b);
  network.add_successor(v1t_v2t, v2t_v3);
  network.add_successor(v1b_v2b, v2b_v3);
  network.add_successor(v2t_v3, v3_v4);
  network.add_successor(v2b_v3, v3_v4);
  network.add_successor(v3_v4, v4_v5);
  network.add_successor(v4_v5, v5_v6);

  GeneralTimetable<GeneralSchedule<GeneralScheduledStop>> timetable;
  const auto tr1 = timetable.add_train("Train1", 10, 50, 4, 2, true, {0, 60},
                                       15, "v0t", {360, 420}, 2, "v6", network);
  const auto tr2 =
      timetable.add_train("Train2", 10, 50, 7, 14, true, {0, 60}, 15, "v0t",
                          {360, 420}, 14, "v6", network);
  const auto tr3 =
      timetable.add_train("Train3", 10, 50, 6, 12, true, {0, 60}, 15, "v0t",
                          {360, 420}, 12, "v6", network);
  const auto tr4 =
      timetable.add_train("Train4", 10, 50, 5, 10, true, {0, 60}, 15, "v0b",
                          {360, 420}, 10, "v6", network);
  const auto tr5 = timetable.add_train("Train5", 10, 50, 4, 8, true, {0, 60},
                                       15, "v0b", {360, 420}, 8, "v6", network);
  const auto tr6 = timetable.add_train("Train6", 10, 50, 3, 6, true, {0, 60},
                                       15, "v0t", {360, 420}, 6, "v6", network);
  const auto tr7 = timetable.add_train("Train7", 20, 50, 2, 4, true, {0, 60},
                                       15, "v0b", {360, 420}, 4, "v6", network);
  const auto tr8 =
      timetable.add_train("Train8", 10, 50, 8, 16, true, {0, 60}, 15, "v0t",
                          {360, 420}, 16, "v6", network);

  timetable.add_station("Station1");
  timetable.add_track_to_station("Station1", v2b_v3, network);

  timetable.add_stop(tr5, "Station1", {30, 60}, {60, 90}, 30);

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

  std::vector<std::pair<double, double>> train_pos = {
      {2080, 2090}, // Train1
      {1970, 1980}, // Train2
      {1090, 1100}, // Train3
      {340, 350},   // Train4
      {240, 250},   // Train5 (stopped at Station1)
      {875, 885},   // Train6
      {90, 110},    // Train7
      {590, 600}    // Train8
  };

  // Check that the ma and max speed constraints are correctly calculated
  // Train 1: Bound by leaving headway
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
      simulator.instance->get_train_list().size(), 0.0);

  // Train 1: Bound by leaving headway, a = 4, d = 2
  train_velocities.at(tr1) = 1;
  const auto [ma1, max_v1] =
      simulator.get_ma_and_maxv(tr1, train_velocities, {}, 0, 10, train_pos,
                                train_ids, {}, tr_on_edges, true);
  EXPECT_APPROX_EQ(max_v1, 2, LINE_SPEED_ACCURACY);
  EXPECT_LE((15 + max_v1) * 1.0 / 2.0 + ((max_v1 * max_v1) / (2 * 2)), ma1);
  // in the last second the train decelerates from v_1 = 4 to v_n = 2 -> (4+2) *
  // 1/2 = 3m Before that the train needs to cover 7m, say within 3s
  // --> (v_0 + 4) * 3 / 2 = 7 --> v_0 = 2/3
  // h = 1 + 3 = 4s
  train_velocities.at(tr1) = 2.0 / 3.0;
  const auto [ma1b, max_v1b] =
      simulator.get_ma_and_maxv(tr1, train_velocities, {}, 4, 3, train_pos,
                                train_ids, {}, tr_on_edges, true);
  EXPECT_APPROX_EQ(max_v1b, 4, LINE_SPEED_ACCURACY);
  EXPECT_LE((2.0 / 3.0 + max_v1b) * 3.0 / 2.0 + ((max_v1b * max_v1b) / (2 * 2)),
            ma1b);

  train_velocities.at(tr2) = 23;
  // Train 2: Bound by final edge, a = 7, d = 14
  const auto [ma2, max_v2] =
      simulator.get_ma_and_maxv(tr2, train_velocities, {}, 0, 2, train_pos,
                                train_ids, {}, tr_on_edges, true);
  EXPECT_EQ(ma2, 20);
  EXPECT_EQ(max_v2, 0);
  train_velocities.at(tr2) = 10;
  const auto [ma2b, max_v2b] =
      simulator.get_ma_and_maxv(tr2, train_velocities, {}, 0, 1, train_pos,
                                train_ids, {}, tr_on_edges, true);
  EXPECT_EQ(ma2b, 20);
  EXPECT_GE((10.0 + max_v2b) * 1.0 / 2.0 + ((max_v2b * max_v2b) / (2 * 14)),
            ma2b);

  // Train 3: No bounds -> maximal displacement, a = 6, d = 12
  train_velocities.at(tr3) = 10;
  const auto [ma3, max_v3] =
      simulator.get_ma_and_maxv(tr3, train_velocities, {}, 0, 2, train_pos,
                                train_ids, {}, tr_on_edges, true);
  EXPECT_APPROX_EQ_6(ma3, 52.0 + 1.0 / 6.0);
  EXPECT_GE((10.0 + max_v3) * 2.0 / 2.0 + ((max_v3 * max_v3) / (2 * 12)), ma3);
  train_velocities.at(tr3)   = 30;
  const auto [ma3b, max_v3b] = simulator.get_ma_and_maxv(
      tr3, train_velocities, {}, 0, 20, train_pos, train_ids, {}, tr_on_edges,
      true); // this time limited by tr2
  EXPECT_EQ(ma3b, 870);
  EXPECT_GE((30.0 + max_v3b) * 20.0 / 2.0 + ((max_v3b * max_v3b) / (2 * 12)),
            ma3b);

  // Train 4: Bound by Train 3, a = 5, d = 10
  train_velocities.at(tr4) = 28;
  const auto [ma4, max_v4] =
      simulator.get_ma_and_maxv(tr4, train_velocities, {}, 0, 2, train_pos,
                                train_ids, {}, tr_on_edges, true);
  EXPECT_EQ(ma4, 40);
  EXPECT_GE((28.0 + max_v4) * 2.0 / 2.0 + ((max_v4 * max_v4) / (2 * 10)), ma4);

  // Train 5: Bound by stopping at Station1, a = 4, d = 8
  train_velocities.at(tr5) = 0;
  const auto [ma5, max_v5] =
      simulator.get_ma_and_maxv(tr5, train_velocities, {250}, 0, 2, train_pos,
                                train_ids, {}, tr_on_edges, true);
  EXPECT_EQ(ma5, 0);
  EXPECT_GE((0.0 + max_v5) * 2.0 / 2.0 + ((max_v5 * max_v5) / (2 * 8)), ma5);
  // Otherwise 90m away from tr4
  train_velocities.at(tr5) = 30;
  const auto [ma5b, max_v5b] =
      simulator.get_ma_and_maxv(tr5, train_velocities, {}, 0, 2, train_pos,
                                train_ids, {}, tr_on_edges, true);
  EXPECT_EQ(ma5b, 90);
  EXPECT_GE((30.0 + max_v5b) * 2.0 / 2.0 + ((max_v5b * max_v5b) / (2 * 8)),
            ma5b);

  // Train 6: Bound by Train 5 in TTD, a = 3, d = 6
  // 15m away from TTD
  train_velocities.at(tr6) = 10;
  const auto [ma6, max_v6] =
      simulator.get_ma_and_maxv(tr6, train_velocities, {}, 0, 2, train_pos,
                                train_ids, {}, tr_on_edges, true);
  EXPECT_EQ(ma6, 15);
  EXPECT_GE((10.0 + max_v6) * 2.0 / 2.0 + ((max_v6 * max_v6) / (2 * 6)), ma6);

  // Train 7: Bound by speed limit of edge, a = 2, d = 4
  train_velocities.at(tr7) = 4;
  const auto [ma7, max_v7] =
      simulator.get_ma_and_maxv(tr7, train_velocities, {}, 0, 4, train_pos,
                                train_ids, {}, tr_on_edges, true);
  EXPECT_EQ(max_v7, 5);
  EXPECT_LE((4.0 + max_v7) * 4.0 / 2.0 + ((max_v7 * max_v7) / (2 * 4)), ma7);
  train_velocities.at(tr7) = 4;
  const auto [ma7b, max_v7b] =
      simulator.get_ma_and_maxv(tr7, train_velocities, {}, 0, 4, train_pos,
                                train_ids, {}, tr_on_edges, false);
  EXPECT_EQ(max_v7b, 10);
  EXPECT_LE((4.0 + max_v7b) * 4.0 / 2.0 + ((max_v7b * max_v7b) / (2 * 4)),
            ma7b);

  // Train 8: Bound by future speed limit of v1t_v2t, a = 8, d = 16
  // At pos = 800 limit of 5 m/s starts
  // bd = 5 * 5 / (2*16) = 25 / 32 = 0.78125
  // --> ma at 800.78125
  // Train is 200m away from position 800
  train_velocities.at(tr8) = 30;
  const auto [ma8, max_v8] =
      simulator.get_ma_and_maxv(tr8, train_velocities, {}, 0, 5, train_pos,
                                train_ids, {}, tr_on_edges, true);
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

  EXPECT_THROW(
      cda_rail::simulator::GreedySimulator::get_v1_from_ma(-1, 57.5, 4, 6),
      cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(
      cda_rail::simulator::GreedySimulator::get_v1_from_ma(5, -1, 4, 6),
      cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(
      cda_rail::simulator::GreedySimulator::get_v1_from_ma(5, 57.5, 0, 6),
      cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(cda_rail::simulator::GreedySimulator::get_v1_from_ma(
                   5, 57.5, cda_rail::EPS / 2.0, 6),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(
      cda_rail::simulator::GreedySimulator::get_v1_from_ma(5, 57.5, 4, -1),
      cda_rail::exceptions::InvalidInputException);
}

TEST(GreedySimulator, MoveTrain) {
  std::vector<std::pair<double, double>> train_pos = {
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
  EXPECT_EQ(train_pos[0].first, -10);
  EXPECT_EQ(train_pos[0].second, 2 + 6);
  EXPECT_EQ(train_pos[1].first, 10);
  EXPECT_EQ(train_pos[1].second, 80);
  EXPECT_EQ(train_pos[2].first, 250);
  EXPECT_EQ(train_pos[2].second, 300);
  EXPECT_EQ(train_pos[3].first, 500);
  EXPECT_EQ(train_pos[3].second, 800);

  // v_0 = 10
  // v_1 = 20 after 4 seconds
  // x_1 = (10 + 20) * 4 / 2 = 60
  EXPECT_TRUE(cda_rail::simulator::GreedySimulator::move_train(1, 10, 20, 100,
                                                               4, train_pos));
  EXPECT_EQ(train_pos.size(), 4);
  EXPECT_EQ(train_pos[0].first, -10);
  EXPECT_EQ(train_pos[0].second, 2 + 6);
  EXPECT_EQ(train_pos[1].first, 10);
  EXPECT_EQ(train_pos[1].second, 80 + 60);
  EXPECT_EQ(train_pos[2].first, 250);
  EXPECT_EQ(train_pos[2].second, 300);
  EXPECT_EQ(train_pos[3].first, 500);
  EXPECT_EQ(train_pos[3].second, 800);

  // v_0 = 10
  // v_1 = 0 after 5 seconds
  // x_1 = (10 + 0) * 5 / 2 = 25
  EXPECT_TRUE(cda_rail::simulator::GreedySimulator::move_train(2, 10, 0, 150, 5,
                                                               train_pos));
  EXPECT_EQ(train_pos.size(), 4);
  EXPECT_EQ(train_pos[0].first, -10);
  EXPECT_EQ(train_pos[0].second, 2 + 6);
  EXPECT_EQ(train_pos[1].first, 10);
  EXPECT_EQ(train_pos[1].second, 80 + 60);
  EXPECT_EQ(train_pos[2].first, 250);
  EXPECT_EQ(train_pos[2].second, 300 + 25);
  EXPECT_EQ(train_pos[3].first, 500);
  EXPECT_EQ(train_pos[3].second, 800);

  // v_0 = 0
  // v_1 = 0 after 10 seconds
  // x_1 = (0 + 0) * 10 / 2 = 0
  EXPECT_FALSE(cda_rail::simulator::GreedySimulator::move_train(3, 0, 0, 500,
                                                                10, train_pos));
  EXPECT_EQ(train_pos.size(), 4);
  EXPECT_EQ(train_pos[0].first, -10);
  EXPECT_EQ(train_pos[0].second, 2 + 6);
  EXPECT_EQ(train_pos[1].first, 10);
  EXPECT_EQ(train_pos[1].second, 80 + 60);
  EXPECT_EQ(train_pos[2].first, 250);
  EXPECT_EQ(train_pos[2].second, 300 + 25);
  EXPECT_EQ(train_pos[3].first, 500);
  EXPECT_EQ(train_pos[3].second, 800);

  EXPECT_THROW(cda_rail::simulator::GreedySimulator::move_train(4, 5, 0, 6, 8,
                                                                train_pos),
               cda_rail::exceptions::TrainNotExistentException);
}

TEST(GreedySimulator, UpdateRearPositions) {
  Network    network;
  const auto v0 = network.add_vertex("v0", VertexType::TTD);
  const auto v1 = network.add_vertex("v1", VertexType::TTD);

  const auto v0_v1 = network.add_edge(v0, v1, 5000, 50, true);

  GeneralTimetable<GeneralSchedule<GeneralScheduledStop>> timetable;
  const auto tr1 = timetable.add_train("Train1", 20, 50, 4, 2, true, {0, 60},
                                       15, v0, {360, 420}, 2, v1, network);
  const auto tr2 = timetable.add_train("Train2", 12, 50, 7, 14, true, {0, 60},
                                       15, v0, {360, 420}, 14, v1, network);
  const auto tr3 = timetable.add_train("Train3", 300, 50, 6, 12, true, {0, 60},
                                       15, "v0", {360, 420}, 12, "v1", network);
  const auto tr4 = timetable.add_train("Train4", 5, 50, 5, 10, true, {0, 60},
                                       15, "v0", {360, 420}, 10, "v1", network);
  const auto tr5 = timetable.add_train("Train5", 15, 50, 4, 8, true, {0, 60},
                                       15, "v0", {360, 420}, 8, "v1", network);
  const auto tr6 = timetable.add_train("Train6", 20, 50, 3, 6, true, {0, 60},
                                       15, "v0", {360, 420}, 6, "v1", network);
  const auto tr7 = timetable.add_train("Train7", 150, 50, 2, 4, true, {0, 60},
                                       15, "v0", {360, 420}, 4, "v1", network);
  const auto tr8 = timetable.add_train("Train8", 9, 50, 8, 16, true, {0, 60},
                                       15, "v0", {360, 420}, 16, "v1", network);

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

  std::vector<std::pair<double, double>> train_pos = {
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
  EXPECT_EQ(train_pos[0].first, 2070);
  EXPECT_EQ(train_pos[0].second, 2090);
  // Train 2, length 12
  // Front: 1980
  // Rear: 1980 - 12 = 1968
  EXPECT_EQ(train_pos[1].first, 1968);
  EXPECT_EQ(train_pos[1].second, 1980);
  // Train 3, length 300
  // Front: 1100
  // Rear: 1100 - 300 = 800
  EXPECT_EQ(train_pos[2].first, 800);
  EXPECT_EQ(train_pos[2].second, 1100);
  // Train 4, length 5
  // Front: 350
  // Rear: 350 - 5 = 345
  EXPECT_EQ(train_pos[3].first, 345);
  EXPECT_EQ(train_pos[3].second, 350);
  // Train 5, length 15
  // Front: 250
  // Rear: 250 - 15 = 235
  EXPECT_EQ(train_pos[4].first, 235);
  EXPECT_EQ(train_pos[4].second, 250);
  // Train 6, length 20
  // Front: 885
  // Rear: 885 - 20 = 865
  EXPECT_EQ(train_pos[5].first, 865);
  EXPECT_EQ(train_pos[5].second, 885);
  // Train 7, length 150
  // Front: 110
  // Rear: 110 - 150 = -40
  EXPECT_EQ(train_pos[6].first, -40);
  EXPECT_EQ(train_pos[6].second, 110);
  // Train 8, length 9
  // Front: 600
  // Rear: 600 - 9 = 591
  EXPECT_EQ(train_pos[7].first, 591);
  EXPECT_EQ(train_pos[7].second, 600);

  train_pos = {
      {2010, 2090}, // Train1, length 20
      {1900, 1980}, // Train2, length 12
      {500, 1100},  // Train3, length 300
      {320, 350},   // Train4, length 5
      {200, 250},   // Train5, length 15
      {775, 885},   // Train6, length 20
      {-50, 110},   // Train7, length 150
  };

  // Train 8 is missing
  EXPECT_THROW(simulator.update_rear_positions(train_pos),
               cda_rail::exceptions::InvalidInputException);

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
  EXPECT_THROW(simulator.update_rear_positions(train_pos),
               cda_rail::exceptions::InvalidInputException);
}

TEST(GreedySimulator, ScheduleFeasibility) {
  Network    network;
  const auto v3 = network.add_vertex("v3", VertexType::TTD);
  const auto v1 = network.add_vertex("v1", VertexType::TTD);
  const auto v0 = network.add_vertex("v0", VertexType::TTD);
  const auto v2 = network.add_vertex("v2", VertexType::TTD);

  const auto e2 = network.add_edge(v1, v2, 200, 50, true);
  const auto e3 = network.add_edge(v2, v3, 300, 50, true);
  const auto e1 = network.add_edge(v0, v1, 100, 50, true);

  GeneralTimetable<GeneralSchedule<GeneralScheduledStop>> timetable;
  const auto tr1 = timetable.add_train("Train1", 10, 50, 4, 2, true, {0, 60},
                                       15, v0, {360, 420}, 2, v3, network);
  const auto tr2 = timetable.add_train("Train2", 10, 50, 7, 14, true, {30, 90},
                                       15, v0, {360, 480}, 14, v3, network);
  const auto tr3 =
      timetable.add_train("Train3", 10, 50, 6, 12, true, {120, 180}, 15, v0,
                          {360, 500}, 12, v3, network);

  timetable.add_station("Station1");
  timetable.add_track_to_station("Station1", e2, network);
  timetable.add_station("Station2");
  timetable.add_track_to_station("Station2", e3, network);

  timetable.add_stop(tr1, "Station1", {60, 90}, {90, 120}, 30);
  timetable.add_stop(tr1, "Station2", {120, 150}, {150, 180}, 30);
  timetable.add_stop(tr2, "Station1", {100, 150}, {130, 200}, 30);

  RouteMap                                                    routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);
  cda_rail::simulator::GreedySimulator simulator(instance, {});

  EXPECT_TRUE(simulator.is_feasible_to_schedule(0, {{}, {}, {}},
                                                {{-10, 0}, {-10, 0}, {-10, 0}},
                                                {}, {}, false, false, false));
  EXPECT_FALSE(simulator.is_feasible_to_schedule(60, {{}, {}, {}},
                                                 {{-10, 0}, {-10, 0}, {-10, 0}},
                                                 {}, {}, false, false, false));
  EXPECT_TRUE(simulator.is_feasible_to_schedule(60, {{}, {}, {}},
                                                {{-10, 0}, {-10, 0}, {-10, 0}},
                                                {}, {}, true, false, false));
  EXPECT_TRUE(simulator.is_feasible_to_schedule(
      60, {{}, {}, {}}, {{-10, 0}, {-10, 0}, {-10, 0}}, {tr1}, {}, false, false,
      false));

  simulator.append_train_edge_to_tr(tr1, e1);
  simulator.append_train_edge_to_tr(tr1, e2);
  simulator.append_train_edge_to_tr(tr2, e1);
  simulator.append_train_edge_to_tr(tr3, e1);

  EXPECT_TRUE(simulator.is_feasible_to_schedule(
      89, {{0}, {}, {}}, {{190, 200}, {-10, 0}, {-10, 0}}, {tr1}, {}, false,
      false, false));
  EXPECT_FALSE(simulator.is_feasible_to_schedule(
      90, {{0}, {}, {}}, {{190, 200}, {80, 90}, {-10, 0}}, {tr1, tr2}, {},
      false, false, false));
  EXPECT_TRUE(simulator.is_feasible_to_schedule(
      90, {{0}, {}, {}}, {{190, 200}, {80, 90}, {-10, 0}}, {tr1, tr2}, {},
      false, false, true));
  EXPECT_TRUE(simulator.is_feasible_to_schedule(
      90, {{1}, {}, {}}, {{190, 200}, {80, 90}, {-10, 0}}, {tr1, tr2}, {},
      false, false, false));
  EXPECT_TRUE(simulator.is_feasible_to_schedule(
      149, {{1}, {}, {}}, {{190, 200}, {80, 90}, {-10, 0}}, {tr1, tr2}, {},
      false, false, false));
  EXPECT_FALSE(simulator.is_feasible_to_schedule(
      150, {{1}, {}, {}}, {{190, 200}, {80, 90}, {-10, 0}}, {tr1, tr2}, {},
      false, false, false));
  EXPECT_TRUE(simulator.is_feasible_to_schedule(
      150, {{1}, {}, {}}, {{190, 200}, {80, 90}, {-10, 0}}, {tr1, tr2}, {},
      false, false, true));
  EXPECT_TRUE(simulator.is_feasible_to_schedule(
      150, {{}, {}, {}}, {{190, 200}, {80, 90}, {-10, 0}}, {tr1, tr2}, {},
      false, false, false));
  EXPECT_TRUE(simulator.is_feasible_to_schedule(
      410, {{}, {}, {}}, {{190, 200}, {80, 90}, {-10, 0}}, {tr1, tr2, tr3}, {},
      false, false, false));
  EXPECT_FALSE(simulator.is_feasible_to_schedule(
      420, {{}, {}, {}}, {{190, 200}, {80, 90}, {-10, 0}}, {tr1, tr2, tr3}, {},
      false, false, false));
  EXPECT_TRUE(simulator.is_feasible_to_schedule(
      420, {{}, {}, {}}, {{190, 200}, {80, 90}, {-10, 0}}, {tr1, tr2, tr3}, {},
      false, true, false));
  EXPECT_TRUE(simulator.is_feasible_to_schedule(
      420, {{}, {}, {}}, {{290, 300}, {80, 90}, {-10, 0}}, {tr1, tr2, tr3}, {},
      false, false, false));
  EXPECT_TRUE(simulator.is_feasible_to_schedule(
      430, {{}, {}, {}}, {{290, 300}, {80, 90}, {-10, 0}}, {tr1, tr2, tr3}, {},
      false, false, false));
  EXPECT_FALSE(simulator.is_feasible_to_schedule(
      480, {{}, {}, {}}, {{290, 300}, {80, 90}, {-10, 0}}, {tr1, tr2, tr3}, {},
      false, false, false));
  EXPECT_TRUE(simulator.is_feasible_to_schedule(
      480, {{}, {}, {}}, {{290, 300}, {80, 90}, {-10, 0}}, {tr1, tr2, tr3}, {},
      false, true, false));
  EXPECT_TRUE(simulator.is_feasible_to_schedule(
      480, {{}, {}, {}}, {{290, 300}, {90, 100}, {-10, 0}}, {tr1, tr2, tr3}, {},
      false, false, false));

  simulator.set_train_edges_of_tr(tr1, {e1, e2, e3});
  simulator.set_train_edges_of_tr(tr2, {e1, e2, e3});
  simulator.set_train_edges_of_tr(tr3, {e1, e2, e3});

  EXPECT_FALSE(simulator.is_feasible_to_schedule(
      420, {{}, {}, {}}, {{600, 610}, {490, 500}, {470, 480}}, {tr1, tr2, tr3},
      {}, false, false, false));
  EXPECT_TRUE(simulator.is_feasible_to_schedule(
      420, {{}, {}, {}}, {{600, 610}, {490, 500}, {470, 480}}, {tr1, tr2, tr3},
      {}, false, true, false));
  EXPECT_TRUE(simulator.is_feasible_to_schedule(
      420, {{}, {}, {}}, {{600, 610}, {490, 500}, {470, 480}}, {tr2, tr3},
      {tr1}, false, false, false));
  EXPECT_FALSE(simulator.is_feasible_to_schedule(
      480, {{}, {}, {}}, {{600, 610}, {600, 610}, {470, 480}}, {tr2, tr3},
      {tr1}, false, false, false));
  EXPECT_TRUE(simulator.is_feasible_to_schedule(
      480, {{}, {}, {}}, {{600, 610}, {600, 610}, {470, 480}}, {tr2, tr3},
      {tr1}, false, true, false));
  EXPECT_TRUE(simulator.is_feasible_to_schedule(
      480, {{}, {}, {}}, {{600, 610}, {600, 610}, {470, 480}}, {tr3},
      {tr1, tr2}, false, false, false));

  EXPECT_THROW(simulator.is_feasible_to_schedule(
                   480, {{}, {}, {}, {}}, {{600, 610}, {600, 610}, {470, 480}},
                   {tr3}, {tr1, tr2}, false, false, false),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(simulator.is_feasible_to_schedule(
                   480, {{}, {}}, {{600, 610}, {600, 610}, {470, 480}}, {tr3},
                   {tr1, tr2}, false, false, false),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(simulator.is_feasible_to_schedule(
                   480, {{}, {}, {}},
                   {{600, 610}, {600, 610}, {470, 480}, {0, 1}}, {tr3},
                   {tr1, tr2}, false, false, false),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(simulator.is_feasible_to_schedule(
                   480, {{}, {}, {}}, {{600, 610}, {600, 610}}, {tr3},
                   {tr1, tr2}, false, false, false),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(simulator.is_feasible_to_schedule(
                   -1, {{}, {}, {}}, {{600, 610}, {600, 610}, {470, 480}},
                   {tr3}, {tr1, tr2}, false, false, false),
               cda_rail::exceptions::InvalidInputException);
}

// -------------------
// Test simulation
// -------------------

TEST(GreedySimulation, SimpleSimulation) {
  Network    network;
  const auto v0 = network.add_vertex("v0", VertexType::TTD, 60);
  const auto v1 = network.add_vertex("v1", VertexType::TTD, 30);

  const auto v0_v1 = network.add_edge(v0, v1, 5000, 50, true);
  GeneralTimetable<GeneralSchedule<GeneralScheduledStop>> timetable;
  const auto tr1 = timetable.add_train("Train1", 100, 50, 4, 2, true, {0, 60},
                                       15, v0, {198, 400}, 40, v1, network);
  RouteMap   routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);
  cda_rail::simulator::GreedySimulator simulator(instance, {});

  simulator.set_train_edges_of_tr(tr1, {v0_v1});
  simulator.set_vertex_orders_of_vertex(v0, {tr1});
  simulator.set_vertex_orders_of_vertex(v1, {tr1});

  const auto [success, obj] =
      simulator.simulate(6, false, false, false, true, true);

  std::cout << "Simulation success: " << success
            << ", Objective value: " << obj.back() << std::endl;
}

// NOLINTEND
// (clang-analyzer-deadcode.DeadStores,misc-const-correctness,clang-diagnostic-unused-result)
