#include "CustomExceptions.hpp"
#include "datastructure/GeneralTimetable.hpp"
#include "datastructure/Route.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"

#include "gtest/gtest.h"
#include <utility>

using namespace cda_rail;

// NOLINTBEGIN (clang-analyzer-deadcode.DeadStores)

TEST(GeneralPerformanceOptimizationInstances,
     GeneralPerformanceOptimizationInstanceConsistency) {
  Network network("./example-networks/SimpleStation/network/");

  GeneralTimetable<GeneralSchedule<GeneralScheduledStop>> timetable;

  const auto l0 = network.get_vertex_index("l0");
  const auto r0 = network.get_vertex_index("r0");

  timetable.add_train("Train1", 100, 10, 1, 1, true, {0, 60}, 0, "l0",
                      {360, 420}, 0, "r0", network);
  timetable.add_train("Train2", 100, 10, 1, 1, false, {0, 60}, 10, l0,
                      {400, 460}, 5, r0, network);

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
  EXPECT_FALSE(instance.check_consistency(true));
  EXPECT_FALSE(instance.check_consistency());

  instance.set_train_weight("Train2", 2);
  instance.set_train_optional("Train1");

  EXPECT_EQ(instance.get_train_weight("Train2"), 2);
  EXPECT_EQ(instance.get_train_optional("Train1"), true);

  instance.set_train_mandatory("Train1");

  EXPECT_EQ(instance.get_train_optional("Train1"), false);

  EXPECT_EQ(instance.get_lambda(), 1);

  instance.set_lambda(2);

  EXPECT_EQ(instance.get_lambda(), 2);

  instance.add_empty_route("Train1");

  instance.push_back_edge_to_route("Train1", "l0", "l1");

  EXPECT_FALSE(instance.check_consistency(false));
  EXPECT_FALSE(instance.check_consistency(true));
  EXPECT_FALSE(instance.check_consistency());

  instance.push_back_edge_to_route("Train1", "l1", "l2");
  instance.push_back_edge_to_route("Train1", "l2", "l3");
  instance.push_back_edge_to_route("Train1", "l3", "g00");
  instance.push_back_edge_to_route("Train1", "g00", "g01");
  instance.push_back_edge_to_route("Train1", "g01", "r2");
  instance.push_back_edge_to_route("Train1", "r2", "r1");
  instance.push_back_edge_to_route("Train1", "r1", "r0");

  EXPECT_TRUE(instance.check_consistency(false));
  EXPECT_FALSE(instance.check_consistency(true));
  EXPECT_FALSE(instance.check_consistency());

  instance.add_empty_route("Train2");
  instance.push_back_edge_to_route("Train2", "l0", "l1");
  instance.push_back_edge_to_route("Train2", "l1", "l2");
  instance.push_back_edge_to_route("Train2", "l2", "l3");
  instance.push_back_edge_to_route("Train2", "l3", "g00");
  instance.push_back_edge_to_route("Train2", "g00", "g01");
  instance.push_back_edge_to_route("Train2", "g01", "r2");
  instance.push_back_edge_to_route("Train2", "r2", "r1");
  instance.push_back_edge_to_route("Train2", "r1", "r0");

  EXPECT_TRUE(instance.check_consistency(false));
  EXPECT_TRUE(instance.check_consistency(true));
  EXPECT_TRUE(instance.check_consistency());
}

TEST(GeneralPerformanceOptimizationInstances,
     GeneralPerformanceOptimizationInstanceExportImport) {
  // Create instance members
  Network network("./example-networks/SimpleStation/network/");

  GeneralTimetable<GeneralSchedule<GeneralScheduledStop>> timetable;

  timetable.add_train("Train1", 100, 10, 1, 1, true, {0, 60}, 0, "l0",
                      {360, 420}, 0, "r0", network);
  timetable.add_train("Train2", 100, 10, 1, 1, false, {0, 60}, 10, "l0",
                      {400, 460}, 5, "r0", network);

  timetable.add_station("Station1");
  timetable.add_track_to_station("Station1", "g00", "g01", network);
  timetable.add_track_to_station("Station1", "g01", "g00", network);
  timetable.add_track_to_station("Station1", "g10", "g11", network);
  timetable.add_track_to_station("Station1", "g11", "g10", network);
  timetable.add_stop("Train1", "Station1", {60, 120}, {120, 180}, 60);

  RouteMap routes;

  // Use above to create instance
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);

  // Make some changes to defaults and add train routes

  instance.set_train_weight("Train2", 2);
  instance.set_train_optional("Train1");
  instance.set_lambda(2);

  instance.add_empty_route("Train1");
  instance.push_back_edge_to_route("Train1", "l0", "l1");
  instance.push_back_edge_to_route("Train1", "l1", "l2");
  instance.push_back_edge_to_route("Train1", "l2", "l3");
  instance.push_back_edge_to_route("Train1", "l3", "g00");
  instance.push_back_edge_to_route("Train1", "g00", "g01");
  instance.push_back_edge_to_route("Train1", "g01", "r2");
  instance.push_back_edge_to_route("Train1", "r2", "r1");
  instance.push_back_edge_to_route("Train1", "r1", "r0");

  instance.add_empty_route("Train2");
  instance.push_back_edge_to_route("Train2", "l0", "l1");
  instance.push_back_edge_to_route("Train2", "l1", "l2");
  instance.push_back_edge_to_route("Train2", "l2", "l3");
  instance.push_back_edge_to_route("Train2", "l3", "g00");
  instance.push_back_edge_to_route("Train2", "g00", "g01");
  instance.push_back_edge_to_route("Train2", "g01", "r2");
  instance.push_back_edge_to_route("Train2", "r2", "r1");
  instance.push_back_edge_to_route("Train2", "r1", "r0");

  // Export and import

  instance.export_instance("./tmp/test-general-instance/");

  cda_rail::instances::GeneralPerformanceOptimizationInstance instance_read(
      "./tmp/test-general-instance/");
  std::filesystem::remove_all("./tmp");

  // Check if imported instance is the same as the original

  EXPECT_TRUE(instance_read.check_consistency());

  const auto l0 = instance_read.const_n().get_vertex_index("l0");
  const auto r0 = instance_read.const_n().get_vertex_index("r0");

  const auto& tr1 = instance_read.get_train_list().get_train("Train1");
  const auto& tr2 = instance_read.get_train_list().get_train("Train2");

  EXPECT_EQ(tr1.name, "Train1");
  EXPECT_EQ(tr1.length, 100);
  EXPECT_EQ(tr1.max_speed, 10);
  EXPECT_EQ(tr1.acceleration, 1);
  EXPECT_EQ(tr1.deceleration, 1);
  EXPECT_EQ(tr1.tim, true);

  EXPECT_EQ(tr2.name, "Train2");
  EXPECT_EQ(tr2.length, 100);
  EXPECT_EQ(tr2.max_speed, 10);
  EXPECT_EQ(tr2.acceleration, 1);
  EXPECT_EQ(tr2.deceleration, 1);
  EXPECT_EQ(tr2.tim, false);

  EXPECT_EQ(instance_read.get_train_weight("Train1"), 1);
  EXPECT_EQ(instance_read.get_train_optional("Train1"), true);
  EXPECT_EQ(instance_read.get_train_weight("Train2"), 2);
  EXPECT_EQ(instance_read.get_train_optional("Train2"), false);
  EXPECT_EQ(instance_read.get_lambda(), 2);

  const auto& tr1_schedule = instance_read.get_schedule("Train1");
  const auto& tr2_schedule = instance_read.get_schedule("Train2");

  EXPECT_EQ(tr1_schedule.get_t_0_range(), (std::pair<int, int>(0, 60)));
  EXPECT_EQ(tr1_schedule.get_t_n_range(), (std::pair<int, int>(360, 420)));
  EXPECT_EQ(tr1_schedule.get_v_0(), 0);
  EXPECT_EQ(tr1_schedule.get_v_n(), 0);
  EXPECT_EQ(tr1_schedule.get_entry(), l0);
  EXPECT_EQ(tr1_schedule.get_exit(), r0);

  EXPECT_EQ(tr2_schedule.get_t_0_range(), (std::pair<int, int>(0, 60)));
  EXPECT_EQ(tr2_schedule.get_t_n_range(), (std::pair<int, int>(400, 460)));
  EXPECT_EQ(tr2_schedule.get_v_0(), 10);
  EXPECT_EQ(tr2_schedule.get_v_n(), 5);
  EXPECT_EQ(tr2_schedule.get_entry(), l0);
  EXPECT_EQ(tr2_schedule.get_exit(), r0);

  EXPECT_EQ(tr1_schedule.get_stops().size(), 1);
  EXPECT_EQ(tr1_schedule.get_stops().at(0).get_station_name(), "Station1");
  EXPECT_EQ(tr1_schedule.get_stops().at(0).get_begin_range(),
            (std::pair<int, int>(60, 120)));
  EXPECT_EQ(tr1_schedule.get_stops().at(0).get_end_range(),
            (std::pair<int, int>(120, 180)));
  EXPECT_EQ(tr1_schedule.get_stops().at(0).get_min_stopping_time(), 60);

  EXPECT_EQ(tr2_schedule.get_stops().size(), 0);

  const auto& tr1_route = instance_read.get_route("Train1");
  const auto& tr2_route = instance_read.get_route("Train2");

  EXPECT_EQ(tr1_route.size(), 8);
  EXPECT_EQ(tr1_route.get_edge(0),
            instance_read.const_n().get_edge_index("l0", "l1"));
  EXPECT_EQ(tr1_route.get_edge(1),
            instance_read.const_n().get_edge_index("l1", "l2"));
  EXPECT_EQ(tr1_route.get_edge(2),
            instance_read.const_n().get_edge_index("l2", "l3"));
  EXPECT_EQ(tr1_route.get_edge(3),
            instance_read.const_n().get_edge_index("l3", "g00"));
  EXPECT_EQ(tr1_route.get_edge(4),
            instance_read.const_n().get_edge_index("g00", "g01"));
  EXPECT_EQ(tr1_route.get_edge(5),
            instance_read.const_n().get_edge_index("g01", "r2"));
  EXPECT_EQ(tr1_route.get_edge(6),
            instance_read.const_n().get_edge_index("r2", "r1"));
  EXPECT_EQ(tr1_route.get_edge(7),
            instance_read.const_n().get_edge_index("r1", "r0"));

  EXPECT_EQ(tr2_route.size(), 8);
  EXPECT_EQ(tr2_route.get_edge(0),
            instance_read.const_n().get_edge_index("l0", "l1"));
  EXPECT_EQ(tr2_route.get_edge(1),
            instance_read.const_n().get_edge_index("l1", "l2"));
  EXPECT_EQ(tr2_route.get_edge(2),
            instance_read.const_n().get_edge_index("l2", "l3"));
  EXPECT_EQ(tr2_route.get_edge(3),
            instance_read.const_n().get_edge_index("l3", "g00"));
  EXPECT_EQ(tr2_route.get_edge(4),
            instance_read.const_n().get_edge_index("g00", "g01"));
  EXPECT_EQ(tr2_route.get_edge(5),
            instance_read.const_n().get_edge_index("g01", "r2"));
  EXPECT_EQ(tr2_route.get_edge(6),
            instance_read.const_n().get_edge_index("r2", "r1"));
  EXPECT_EQ(tr2_route.get_edge(7),
            instance_read.const_n().get_edge_index("r1", "r0"));

  EXPECT_EQ(instance_read.get_station_list().size(), 1);
  const auto& station1 =
      instance_read.get_station_list().get_station("Station1");
  EXPECT_EQ(station1.name, "Station1");
  const auto& station1_tracks = station1.tracks;
  EXPECT_EQ(station1_tracks.size(), 4);
  EXPECT_TRUE(std::find(station1_tracks.begin(), station1_tracks.end(),
                        instance_read.const_n().get_edge_index("g00", "g01")) !=
              station1_tracks.end());
  EXPECT_TRUE(std::find(station1_tracks.begin(), station1_tracks.end(),
                        instance_read.const_n().get_edge_index("g01", "g00")) !=
              station1_tracks.end());
  EXPECT_TRUE(std::find(station1_tracks.begin(), station1_tracks.end(),
                        instance_read.const_n().get_edge_index("g10", "g11")) !=
              station1_tracks.end());
  EXPECT_TRUE(std::find(station1_tracks.begin(), station1_tracks.end(),
                        instance_read.const_n().get_edge_index("g11", "g10")) !=
              station1_tracks.end());
}

TEST(GeneralPerformanceOptimizationInstances,
     SolGeneralPerformanceOptimizationInstanceConsistency) {
  instances::GeneralPerformanceOptimizationInstance instance;

  // Add a simple network to the instance
  const auto v0 = instance.n().add_vertex("v0", cda_rail::VertexType::TTD);
  const auto v1 = instance.n().add_vertex("v1", cda_rail::VertexType::TTD);
  const auto v2 = instance.n().add_vertex("v2", cda_rail::VertexType::TTD);

  const auto v0_v1 = instance.n().add_edge("v0", "v1", 100, 10);
  const auto v1_v2 = instance.n().add_edge("v1", "v2", 200, 20);
  const auto v1_v0 = instance.n().add_edge("v1", "v0", 100, 10);
  const auto v2_v1 = instance.n().add_edge("v2", "v1", 200, 20);

  instance.n().add_successor({"v0", "v1"}, {"v1", "v2"});
  instance.n().add_successor({"v2", "v1"}, {"v1", "v0"});

  const auto tr1 = instance.add_train("tr1", 50, 10, 2, 2, {0, 60}, 0, "v0",
                                      {120, 180}, 5, "v2");
  const auto tr2 = instance.add_train("tr2", 50, 10, 2, 2, {120, 180}, 0, "v2",
                                      {210, 270}, 0, "v0", 2, true);

  // Check the consistency of the instance
  EXPECT_TRUE(instance.check_consistency(false));

  instances::SolGeneralPerformanceOptimizationInstance sol_instance(instance);

  EXPECT_FALSE(sol_instance.check_consistency());

  sol_instance.set_obj(0.5);
  sol_instance.set_status(cda_rail::SolutionStatus::Optimal);

  EXPECT_FALSE(sol_instance.check_consistency());

  sol_instance.add_empty_route("tr1");
  sol_instance.push_back_edge_to_route("tr1", "v0", "v1");
  sol_instance.push_back_edge_to_route("tr1", v1, v2);

  EXPECT_FALSE(sol_instance.check_consistency());

  sol_instance.set_train_routed("tr1");

  EXPECT_FALSE(sol_instance.check_consistency());

  sol_instance.add_train_pos("tr1", 0, 0);

  EXPECT_FALSE(sol_instance.check_consistency());

  sol_instance.add_train_speed("tr1", 0, 10);

  EXPECT_FALSE(sol_instance.check_consistency());

  sol_instance.add_train_pos("tr1", 60, 100);

  EXPECT_FALSE(sol_instance.check_consistency());

  sol_instance.add_train_speed("tr1", 60, 5);

  EXPECT_TRUE(sol_instance.check_consistency());

  sol_instance.set_train_not_routed("tr1");

  EXPECT_FALSE(sol_instance.check_consistency());

  sol_instance.set_train_routed("tr1");

  EXPECT_TRUE(sol_instance.check_consistency());

  sol_instance.set_status(cda_rail::SolutionStatus::Infeasible);
  EXPECT_TRUE(sol_instance.check_consistency());
  sol_instance.set_status(cda_rail::SolutionStatus::Timeout);
  EXPECT_TRUE(sol_instance.check_consistency());
  sol_instance.set_status(cda_rail::SolutionStatus::Optimal);

  sol_instance.set_obj(-1);
  EXPECT_FALSE(sol_instance.check_consistency());
  sol_instance.set_obj(0);

  EXPECT_TRUE(sol_instance.check_consistency());
}

TEST(GeneralPerformanceOptimizationInstances,
     SolGeneralPerformanceOptimizationInstanceExportImport) {
  instances::GeneralPerformanceOptimizationInstance instance;

  // Add a simple network to the instance
  const auto v0 = instance.n().add_vertex("v0", cda_rail::VertexType::TTD);
  const auto v1 = instance.n().add_vertex("v1", cda_rail::VertexType::TTD);
  const auto v2 = instance.n().add_vertex("v2", cda_rail::VertexType::TTD);

  const auto v0_v1 = instance.n().add_edge("v0", "v1", 100, 10);
  const auto v1_v2 = instance.n().add_edge("v1", "v2", 200, 20);
  const auto v1_v0 = instance.n().add_edge("v1", "v0", 100, 10);
  const auto v2_v1 = instance.n().add_edge("v2", "v1", 200, 20);

  instance.n().add_successor({"v0", "v1"}, {"v1", "v2"});
  instance.n().add_successor({"v2", "v1"}, {"v1", "v0"});

  const auto tr1 = instance.add_train("tr1", 50, 10, 2, 2, {0, 60}, 0, "v0",
                                      {120, 180}, 5, "v2");
  const auto tr2 = instance.add_train("tr2", 50, 10, 2, 2, {120, 180}, 0, "v2",
                                      {210, 270}, 0, "v0", 2, true);

  // Check the consistency of the instance
  EXPECT_TRUE(instance.check_consistency(false));

  instances::SolGeneralPerformanceOptimizationInstance sol_instance(instance);

  sol_instance.set_obj(0.5);
  sol_instance.set_status(cda_rail::SolutionStatus::Optimal);

  sol_instance.add_empty_route("tr1");
  sol_instance.push_back_edge_to_route("tr1", "v0", "v1");
  sol_instance.push_back_edge_to_route("tr1", v1, v2);

  sol_instance.set_train_routed("tr1");

  sol_instance.add_train_pos("tr1", 0, 0);
  sol_instance.add_train_pos("tr1", 60, 100);
  sol_instance.add_train_speed("tr1", 0, 10);
  sol_instance.add_train_speed("tr1", 60, 5);

  EXPECT_TRUE(sol_instance.check_consistency());

  sol_instance.export_solution("./tmp/test-sol-instance-1", true);
  sol_instance.export_solution("./tmp/test-sol-instance-2", false);
  const auto sol1_read =
      cda_rail::instances::SolGeneralPerformanceOptimizationInstance::
          import_solution("./tmp/test-sol-instance-1");
  const auto sol2_read =
      cda_rail::instances::SolGeneralPerformanceOptimizationInstance::
          import_solution("./tmp/test-sol-instance-2", instance);
  std::filesystem::remove_all("./tmp");

  EXPECT_TRUE(sol1_read.check_consistency());
  EXPECT_TRUE(sol2_read.check_consistency());

  EXPECT_EQ(sol1_read.get_obj(), 0.5);
  EXPECT_EQ(sol1_read.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_TRUE(sol1_read.get_train_routed("tr1"));
  EXPECT_EQ(sol1_read.get_train_pos("tr1", 0), 0);
  EXPECT_EQ(sol1_read.get_train_pos("tr1", 60), 100);
  EXPECT_EQ(sol1_read.get_train_speed("tr1", 0), 10);
  EXPECT_EQ(sol1_read.get_train_speed("tr1", 60), 5);
  EXPECT_TRUE(sol1_read.get_instance().has_route("tr1"));
  const auto& tr1_route = sol1_read.get_instance().get_route("tr1");
  EXPECT_EQ(tr1_route.size(), 2);
  EXPECT_EQ(tr1_route.get_edge(0),
            sol1_read.get_instance().const_n().get_edge_index("v0", "v1"));
  EXPECT_EQ(tr1_route.get_edge(1),
            sol1_read.get_instance().const_n().get_edge_index("v1", "v2"));
  EXPECT_FALSE(sol1_read.get_train_routed("tr2"));
  EXPECT_FALSE(sol1_read.get_instance().has_route("tr2"));

  EXPECT_EQ(sol2_read.get_obj(), 0.5);
  EXPECT_EQ(sol2_read.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_TRUE(sol2_read.get_train_routed("tr1"));
  EXPECT_EQ(sol2_read.get_train_pos("tr1", 0), 0);
  EXPECT_EQ(sol2_read.get_train_pos("tr1", 60), 100);
  EXPECT_EQ(sol2_read.get_train_speed("tr1", 0), 10);
  EXPECT_EQ(sol2_read.get_train_speed("tr1", 60), 5);
  EXPECT_TRUE(sol2_read.get_instance().has_route("tr1"));
  const auto& tr1_route2 = sol2_read.get_instance().get_route("tr1");
  EXPECT_EQ(tr1_route2.size(), 2);
  EXPECT_EQ(tr1_route2.get_edge(0),
            sol2_read.get_instance().const_n().get_edge_index("v0", "v1"));
  EXPECT_EQ(tr1_route2.get_edge(1),
            sol2_read.get_instance().const_n().get_edge_index("v1", "v2"));
  EXPECT_FALSE(sol2_read.get_train_routed("tr2"));
  EXPECT_FALSE(sol2_read.get_instance().has_route("tr2"));
}

TEST(GeneralPerformanceOptimizationInstances, DiscretizationOfStops1) {
  // Create instance members
  Network network("./example-networks/SimpleStation/network/");

  GeneralTimetable<GeneralSchedule<GeneralScheduledStop>> timetable;

  timetable.add_station("Station1");
  timetable.add_track_to_station("Station1", "g00", "g01", network);

  RouteMap routes;

  // Use above to create instance
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);

  EXPECT_TRUE(instance.check_consistency());

  instance.discretize_stops();

  EXPECT_TRUE(instance.const_n().has_vertex("g00_g01_0"));
  EXPECT_FALSE(instance.const_n().has_vertex("g10_g11_0"));
  EXPECT_FALSE(instance.const_n().has_vertex("g00_g01_1"));
  EXPECT_FALSE(instance.const_n().has_vertex("g11_g10_1"));

  EXPECT_TRUE(instance.const_n().has_edge("g00", "g00_g01_0"));
  EXPECT_TRUE(instance.const_n().has_edge("g00_g01_0", "g01"));
  EXPECT_TRUE(instance.const_n().has_edge("g01", "g00_g01_0"));
  EXPECT_TRUE(instance.const_n().has_edge("g00_g01_0", "g00"));

  const auto& e1 = instance.const_n().get_edge("g00", "g00_g01_0");
  const auto& e2 = instance.const_n().get_edge("g00_g01_0", "g01");
  const auto& e3 = instance.const_n().get_edge("g01", "g00_g01_0");
  const auto& e4 = instance.const_n().get_edge("g00_g01_0", "g00");

  EXPECT_DOUBLE_EQ(e1.length, 150);
  EXPECT_DOUBLE_EQ(e1.min_stop_block_length, 150);
  EXPECT_TRUE(e1.breakable);

  EXPECT_DOUBLE_EQ(e2.length, 150);
  EXPECT_DOUBLE_EQ(e2.min_stop_block_length, 150);
  EXPECT_TRUE(e2.breakable);

  EXPECT_DOUBLE_EQ(e3.length, 150);
  EXPECT_DOUBLE_EQ(e3.min_stop_block_length, 150);
  EXPECT_TRUE(e3.breakable);

  EXPECT_DOUBLE_EQ(e4.length, 150);
  EXPECT_DOUBLE_EQ(e4.min_stop_block_length, 150);
  EXPECT_TRUE(e4.breakable);

  const auto& s1 = instance.get_station_list().get_station("Station1");

  std::vector<size_t> s1_expected = {
      instance.const_n().get_edge_index("g00", "g00_g01_0"),
      instance.const_n().get_edge_index("g00_g01_0", "g01")};

  EXPECT_EQ(s1_expected.size(), s1.tracks.size());

  for (const auto& e : s1_expected) {
    EXPECT_TRUE(std::find(s1.tracks.begin(), s1.tracks.end(), e) !=
                s1.tracks.end())
        << instance.const_n().get_edge(e).source << " to "
        << instance.const_n().get_edge(e).target << " not found in station 1";
  }
}

TEST(GeneralPerformanceOptimizationInstances, DiscretizationOfStops2) {
  // Create instance members
  Network network("./example-networks/SimpleStation/network/");

  GeneralTimetable<GeneralSchedule<GeneralScheduledStop>> timetable;

  timetable.add_station("Station1");
  timetable.add_track_to_station("Station1", "g00", "g01", network);

  timetable.add_station("Station2");
  timetable.add_track_to_station("Station2", "g00", "g01", network);
  timetable.add_track_to_station("Station2", "g10", "g11", network);
  timetable.add_track_to_station("Station2", "g11", "g10", network);

  RouteMap routes;

  // Use above to create instance
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);

  EXPECT_TRUE(instance.check_consistency());

  instance.discretize_stops();

  EXPECT_TRUE(instance.const_n().has_vertex("g00_g01_0"));
  EXPECT_TRUE(instance.const_n().has_vertex("g10_g11_0"));
  EXPECT_FALSE(instance.const_n().has_vertex("g00_g01_1"));
  EXPECT_FALSE(instance.const_n().has_vertex("g11_g10_1"));

  EXPECT_TRUE(instance.const_n().has_edge("g00", "g00_g01_0"));
  EXPECT_TRUE(instance.const_n().has_edge("g00_g01_0", "g01"));
  EXPECT_TRUE(instance.const_n().has_edge("g01", "g00_g01_0"));
  EXPECT_TRUE(instance.const_n().has_edge("g00_g01_0", "g00"));

  EXPECT_TRUE(instance.const_n().has_edge("g10", "g10_g11_0"));
  EXPECT_TRUE(instance.const_n().has_edge("g10_g11_0", "g11"));
  EXPECT_TRUE(instance.const_n().has_edge("g11", "g10_g11_0"));
  EXPECT_TRUE(instance.const_n().has_edge("g10_g11_0", "g10"));

  const auto& e1 = instance.const_n().get_edge("g00", "g00_g01_0");
  const auto& e2 = instance.const_n().get_edge("g00_g01_0", "g01");
  const auto& e3 = instance.const_n().get_edge("g01", "g00_g01_0");
  const auto& e4 = instance.const_n().get_edge("g00_g01_0", "g00");

  const auto& e5 = instance.const_n().get_edge("g10", "g10_g11_0");
  const auto& e6 = instance.const_n().get_edge("g10_g11_0", "g11");
  const auto& e7 = instance.const_n().get_edge("g11", "g10_g11_0");
  const auto& e8 = instance.const_n().get_edge("g10_g11_0", "g10");

  EXPECT_DOUBLE_EQ(e1.length, 150);
  EXPECT_DOUBLE_EQ(e1.min_stop_block_length, 150);
  EXPECT_TRUE(e1.breakable);

  EXPECT_DOUBLE_EQ(e2.length, 150);
  EXPECT_DOUBLE_EQ(e2.min_stop_block_length, 150);
  EXPECT_TRUE(e2.breakable);

  EXPECT_DOUBLE_EQ(e3.length, 150);
  EXPECT_DOUBLE_EQ(e3.min_stop_block_length, 150);
  EXPECT_TRUE(e3.breakable);

  EXPECT_DOUBLE_EQ(e4.length, 150);
  EXPECT_DOUBLE_EQ(e4.min_stop_block_length, 150);
  EXPECT_TRUE(e4.breakable);

  EXPECT_DOUBLE_EQ(e5.length, 150);
  EXPECT_DOUBLE_EQ(e5.min_stop_block_length, 150);
  EXPECT_TRUE(e5.breakable);

  EXPECT_DOUBLE_EQ(e6.length, 150);
  EXPECT_DOUBLE_EQ(e6.min_stop_block_length, 150);
  EXPECT_TRUE(e6.breakable);

  EXPECT_DOUBLE_EQ(e7.length, 150);
  EXPECT_DOUBLE_EQ(e7.min_stop_block_length, 150);
  EXPECT_TRUE(e7.breakable);

  EXPECT_DOUBLE_EQ(e8.length, 150);
  EXPECT_DOUBLE_EQ(e8.min_stop_block_length, 150);
  EXPECT_TRUE(e8.breakable);

  const auto& s1 = instance.get_station_list().get_station("Station1");
  const auto& s2 = instance.get_station_list().get_station("Station2");

  std::vector<size_t> s1_expected = {
      instance.const_n().get_edge_index("g00", "g00_g01_0"),
      instance.const_n().get_edge_index("g00_g01_0", "g01")};

  std::vector<size_t> s2_expected = {
      instance.const_n().get_edge_index("g00", "g00_g01_0"),
      instance.const_n().get_edge_index("g00_g01_0", "g01"),
      instance.const_n().get_edge_index("g10", "g10_g11_0"),
      instance.const_n().get_edge_index("g10_g11_0", "g11"),
      instance.const_n().get_edge_index("g11", "g10_g11_0"),
      instance.const_n().get_edge_index("g10_g11_0", "g10")};

  EXPECT_EQ(s1_expected.size(), s1.tracks.size());
  EXPECT_EQ(s2_expected.size(), s2.tracks.size());

  for (const auto& e : s1_expected) {
    EXPECT_TRUE(std::find(s1.tracks.begin(), s1.tracks.end(), e) !=
                s1.tracks.end())
        << instance.const_n().get_edge(e).source << " to "
        << instance.const_n().get_edge(e).target << " not found in station 1";
  }

  for (const auto& e : s2_expected) {
    EXPECT_TRUE(std::find(s2.tracks.begin(), s2.tracks.end(), e) !=
                s2.tracks.end())
        << instance.const_n().get_edge(e).source << " to "
        << instance.const_n().get_edge(e).target << " not found in station 2";
  }
}

// NOLINTEND (clang-analyzer-deadcode.DeadStores)
