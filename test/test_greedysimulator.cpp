#define TEST_FRIENDS true

#include "CustomExceptions.hpp"
#include "datastructure/GeneralTimetable.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "simulator/GreedySimulator.hpp"

#include "gtest/gtest.h"

using namespace cda_rail;

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
      {});
  cda_rail::simulator::GreedySimulator simulator2(
      instance, ttd_sections, {{l0_l1, l1_l2, l2_l3}, {r0_r1, r1_r2}}, {},
      std::vector<std::vector<size_t>>(network.number_of_vertices(),
                                       std::vector<size_t>()));
  cda_rail::simulator::GreedySimulator simulator3(
      instance, ttd_sections, {{l0_l1, l1_l2, l2_l3}},
      std::vector<std::vector<size_t>>(ttd_sections.size(),
                                       std::vector<size_t>()),
      std::vector<std::vector<size_t>>(network.number_of_vertices(),
                                       std::vector<size_t>()));
  cda_rail::simulator::GreedySimulator simulator4(
      instance, ttd_sections, {{l0_l1, l1_l2, l2_l3}, {r0_r1, r1_r2}},
      std::vector<std::vector<size_t>>(ttd_sections.size(),
                                       std::vector<size_t>()),
      std::vector<std::vector<size_t>>(network.number_of_vertices(),
                                       std::vector<size_t>()));
  simulator4.set_ttd_orders_of_ttd(0, {tr1, tr2});
  cda_rail::simulator::GreedySimulator simulator5(
      instance, ttd_sections, {{l0_l1, l1_l2, l2_l3}, {}},
      std::vector<std::vector<size_t>>(ttd_sections.size(),
                                       std::vector<size_t>()),
      std::vector<std::vector<size_t>>(network.number_of_vertices(),
                                       std::vector<size_t>()));

  cda_rail::simulator::GreedySimulator simulator6(
      instance, ttd_sections, {{l0_l1, l1_l2, 1000}, {r0_r1, r1_r2}},
      std::vector<std::vector<size_t>>(ttd_sections.size(),
                                       std::vector<size_t>()),
      std::vector<std::vector<size_t>>(network.number_of_vertices(),
                                       std::vector<size_t>()));
  cda_rail::simulator::GreedySimulator simulator7(
      instance, ttd_sections, {{l0_l1, l2_l3}, {r0_r1, r1_r2}},
      std::vector<std::vector<size_t>>(ttd_sections.size(),
                                       std::vector<size_t>()),
      std::vector<std::vector<size_t>>(network.number_of_vertices(),
                                       std::vector<size_t>()));
  cda_rail::simulator::GreedySimulator simulator8(
      instance, ttd_sections, {{l0_l1, l1_l2, l2_l3}, {l0_l1, l1_l2}},
      std::vector<std::vector<size_t>>(ttd_sections.size(),
                                       std::vector<size_t>()),
      std::vector<std::vector<size_t>>(network.number_of_vertices(),
                                       std::vector<size_t>()));
  cda_rail::simulator::GreedySimulator simulator9(
      instance, ttd_sections, {{l0_l1, l1_l2, l2_l3}, {r0_r1, r1_r2}},
      std::vector<std::vector<size_t>>(ttd_sections.size(),
                                       std::vector<size_t>()),
      std::vector<std::vector<size_t>>(network.number_of_vertices(),
                                       std::vector<size_t>()));
  simulator9.set_ttd_orders_of_ttd(0, {1000});
  cda_rail::simulator::GreedySimulator simulator10(
      instance, ttd_sections, {{l0_l1, l1_l2, l2_l3}, {r0_r1, r1_r2}},
      std::vector<std::vector<size_t>>(ttd_sections.size(),
                                       std::vector<size_t>()),
      std::vector<std::vector<size_t>>(network.number_of_vertices(),
                                       std::vector<size_t>()));
  cda_rail::simulator::GreedySimulator simulator11(
      instance, ttd_sections, {{l0_l1, l1_l2, l2_l3}, {r0_r1, r1_r2}},
      std::vector<std::vector<size_t>>(ttd_sections.size(),
                                       std::vector<size_t>()),
      std::vector<std::vector<size_t>>(network.number_of_vertices(),
                                       std::vector<size_t>()));
  cda_rail::simulator::GreedySimulator simulator12(
      instance, ttd_sections, {{l0_l1, l1_l2, l2_l3}, {r0_r1, r1_r2}},
      std::vector<std::vector<size_t>>(ttd_sections.size(),
                                       std::vector<size_t>()),
      std::vector<std::vector<size_t>>(network.number_of_vertices(),
                                       std::vector<size_t>()));
  simulator10.set_entry_orders_of_vertex(l0, {tr1});
  simulator11.set_entry_orders_of_vertex(l0, {1000});
  simulator12.set_entry_orders_of_vertex(l0, {tr1, tr2});

  cda_rail::simulator::GreedySimulator simulator_instance2(instance2,
                                                           ttd_sections);

  // Check if consistency is determined correctly
  EXPECT_FALSE(simulator1.check_consistency());
  EXPECT_FALSE(simulator2.check_consistency());
  EXPECT_FALSE(simulator3.check_consistency());
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

  GeneralTimetable<GeneralSchedule<GeneralScheduledStop>> timetable;
  const auto l0  = network.get_vertex_index("l0");
  const auto r0  = network.get_vertex_index("r0");
  const auto tr1 = timetable.add_train("Train1", 100, 10, 1, 1, true, {0, 60},
                                       0, "l0", {360, 420}, 0, "r0", network);
  const auto tr2 = timetable.add_train("Train2", 100, 10, 1, 1, false, {30, 90},
                                       10, r0, {400, 460}, 5, l0, network);
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
  EXPECT_THROW(simulator.set_entry_orders({}),
               cda_rail::exceptions::InvalidInputException);
  simulator.set_entry_orders(std::vector<std::vector<size_t>>(
      network.number_of_vertices(), std::vector<size_t>()));
  const auto& entry_orders1 = simulator.get_entry_orders();
  EXPECT_EQ(entry_orders1.size(), network.number_of_vertices());
  for (const auto& orders : entry_orders1) {
    EXPECT_TRUE(orders.empty());
  }
  simulator.set_entry_orders_of_vertex(l0, {tr1});
  const auto& entry_orders2 = simulator.get_entry_orders_of_vertex(l0);
  EXPECT_EQ(entry_orders2.size(), 1);
  EXPECT_EQ(entry_orders2[0], tr1);
  EXPECT_THROW(simulator.get_entry_orders_of_vertex(1000),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(simulator.set_entry_orders_of_vertex(1000, {tr1}),
               cda_rail::exceptions::InvalidInputException);
}

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

  simulator.set_entry_orders_of_vertex(r0, {tr2});
  simulator.set_entry_orders_of_vertex(l0, {tr1, tr3, tr5});

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
  // Expect tr1 and tr2
  EXPECT_EQ(entering_tr_30.size(), 2);
  EXPECT_TRUE(entering_tr_30.contains(tr1));
  EXPECT_TRUE(entering_tr_30.contains(tr2));

  const auto& [success_60, entering_tr_60] =
      simulator.get_entering_trains(60, {}, {}, false);
  EXPECT_TRUE(success_60);
  // Expect tr1, tr2
  EXPECT_EQ(entering_tr_60.size(), 2);
  EXPECT_TRUE(entering_tr_60.contains(tr1));
  EXPECT_TRUE(entering_tr_60.contains(tr2));

  const auto& [success_61, entering_tr_61] =
      simulator.get_entering_trains(61, {}, {}, false);
  EXPECT_FALSE(success_61); // tr1 too late
  EXPECT_EQ(entering_tr_61.size(), 1);
  EXPECT_TRUE(entering_tr_61.contains(tr1));

  const auto& [success_61_t, entering_tr_61_t] =
      simulator.get_entering_trains(61, {}, {}, true);
  EXPECT_TRUE(success_61_t); // tr1 still entering
  EXPECT_EQ(entering_tr_61_t.size(), 2);
  EXPECT_TRUE(entering_tr_61_t.contains(tr1));
  EXPECT_TRUE(entering_tr_61_t.contains(tr2));

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
      simulator.get_entering_trains(60, {tr1, tr3}, {}, false);
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
      simulator.get_entering_trains(120, {tr1, tr2}, {}, false);
  EXPECT_TRUE(success_120_tr1tr2);
  // Expect tr3
  EXPECT_EQ(entering_tr_120_tr1tr2.size(), 1);
  EXPECT_TRUE(entering_tr_120_tr1tr2.contains(tr3));

  const auto& [success_120_tr1tr2tr3, entering_tr_120_tr1tr2tr3] =
      simulator.get_entering_trains(120, {tr1, tr2, tr3}, {}, false);
  EXPECT_TRUE(success_120_tr1tr2tr3);
  // Expect tr5
  EXPECT_EQ(entering_tr_120_tr1tr2tr3.size(), 1);
  EXPECT_TRUE(entering_tr_120_tr1tr2tr3.contains(tr5));

  // Milestones
  simulator.append_train_edge_to_tr(tr1, l0_l1);
  simulator.append_train_edge_to_tr(tr1, l1_l2);
  simulator.append_train_edge_to_tr(tr1, l2_l3);
  simulator.append_train_edge_to_tr(tr1, l3_g00);
  simulator.append_train_edge_to_tr(tr1, g00_g01);
  simulator.append_train_edge_to_tr(tr3, l0_l1);

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

  simulator.set_entry_orders_of_vertex(r0, {tr2});
  simulator.set_entry_orders_of_vertex(l0, {tr1, tr3, tr5});
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

// NOLINTEND
// (clang-analyzer-deadcode.DeadStores,misc-const-correctness,clang-diagnostic-unused-result)
