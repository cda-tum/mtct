#include "simulator/GreedySimulator.hpp"

#include "gtest/gtest.h"

using namespace cda_rail;

TEST(GreedySimulator, CheckConsistency) {
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
