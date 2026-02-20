#include "Definitions.hpp"
#include "EOMHelper.hpp"
#include "datastructure/GeneralTimetable.hpp"
#include "datastructure/Route.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"

#include "gtest/gtest.h"
#include <cmath>
#include <tuple>
#include <utility>

using namespace cda_rail;

#define EXPECT_APPROX_EQ(a, b) EXPECT_APPROX_EQ_2(a, b, 1e-6)

#define EXPECT_APPROX_EQ_2(a, b, c)                                            \
  EXPECT_TRUE(std::abs((a) - (b)) < (c)) << (a) << " !=(approx.) " << (b)

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

  const auto v0_v1 = instance.n().add_edge("v0", "v1", 100, 10, false);
  const auto v1_v2 = instance.n().add_edge("v1", "v2", 200, 20, false);
  const auto v1_v0 = instance.n().add_edge("v1", "v0", 100, 10, false);
  const auto v2_v1 = instance.n().add_edge("v2", "v1", 200, 20, false);

  instance.n().add_successor({"v0", "v1"}, {"v1", "v2"});
  instance.n().add_successor({"v2", "v1"}, {"v1", "v0"});

  const auto tr1 = instance.add_train("tr1", 50, 10, 2, 2, {0, 60}, 10, "v0",
                                      {120, 180}, 6, "v2");
  const auto tr2 = instance.add_train("tr2", 50, 20, 2, 2, {120, 180}, 0, "v2",
                                      {210, 270}, 0, "v0", 2, true);

  // Check the consistency of the instance
  EXPECT_TRUE(instance.check_consistency(false));

  instances::SolGeneralPerformanceOptimizationInstance<
      instances::GeneralPerformanceOptimizationInstance>
      sol_instance(instance);

  EXPECT_FALSE(sol_instance.check_consistency());

  sol_instance.set_obj(0.5);
  sol_instance.set_status(cda_rail::SolutionStatus::Optimal);
  sol_instance.set_solution_found();

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

  sol_instance.add_train_pos("tr1", 10, 100);

  EXPECT_FALSE(sol_instance.check_consistency());

  sol_instance.add_train_speed("tr1", 10, 10);

  EXPECT_TRUE(sol_instance.check_consistency());

  sol_instance.set_train_not_routed("tr1");

  EXPECT_FALSE(sol_instance.check_consistency());

  sol_instance.set_train_routed("tr1");

  EXPECT_TRUE(sol_instance.check_consistency());

  sol_instance.add_train_pos("tr1", 20, 200);
  sol_instance.add_train_speed("tr1", 20, 10);

  // Constant speed of 10 for first 200 meters
  // Then decelerate at rate 2 for 2 seconds to reach speed 6
  // Distance travelled is 8*2 = 16
  // Distance remaining 100 - 16 = 84
  // Time for this is 84/6 = 14
  // Hence, exits at time 20 + 2 + 14 = 36, pos 300, speed 6
  sol_instance.add_train_pos("tr1", 36, 300);
  sol_instance.add_train_speed("tr1", 36, 6);

  EXPECT_TRUE(sol_instance.check_consistency());

  EXPECT_APPROX_EQ(sol_instance.get_train_pos("tr1", 0), 0);
  EXPECT_APPROX_EQ(sol_instance.get_train_pos("tr1", 10), 100);
  EXPECT_APPROX_EQ(sol_instance.get_train_pos("tr1", 20), 200);
  EXPECT_APPROX_EQ(sol_instance.get_train_pos("tr1", 36), 300);

  EXPECT_APPROX_EQ(sol_instance.get_train_speed("tr1", 0), 10);
  EXPECT_APPROX_EQ(sol_instance.get_train_speed("tr1", 10), 10);
  EXPECT_APPROX_EQ(sol_instance.get_train_speed("tr1", 20), 10);
  EXPECT_APPROX_EQ(sol_instance.get_train_speed("tr1", 36), 6);

  // Time 0 with speed 10 at position 0
  const auto posvel1 = sol_instance.get_approximate_train_pos_and_vel("tr1", 0);
  EXPECT_TRUE(posvel1.has_value());
  const auto [pos1, vel1] = posvel1.value();
  EXPECT_APPROX_EQ(pos1, 0);
  EXPECT_APPROX_EQ(vel1, 10);

  const auto [pos1_lb, pos1_ub, v1_lb, v1_ub] =
      sol_instance.get_exact_pos_and_vel_bounds("tr1", 0);
  EXPECT_APPROX_EQ(pos1_lb, 0);
  EXPECT_APPROX_EQ(pos1_ub, 0);
  EXPECT_APPROX_EQ(v1_lb, 10);
  EXPECT_APPROX_EQ(v1_ub, 10);

  // Time 5 with speed 10 at position 50
  const auto posvel2 = sol_instance.get_approximate_train_pos_and_vel("tr1", 5);
  EXPECT_TRUE(posvel2.has_value());
  const auto [pos2, vel2] = posvel2.value();
  EXPECT_APPROX_EQ(pos2, 50);
  EXPECT_APPROX_EQ(vel2, 10);

  const auto [pos2_lb, pos2_ub, v2_lb, v2_ub] =
      sol_instance.get_exact_pos_and_vel_bounds("tr1", 5);
  EXPECT_APPROX_EQ(min_travel_time_from_start(10, 10, 10, 2, 2, 100, pos2_ub),
                   5);
  EXPECT_APPROX_EQ(
      max_travel_time_from_start_no_stopping(10, 10, V_MIN, 2, 2, 100, pos2_lb),
      5);
  EXPECT_APPROX_EQ(v2_lb, V_MIN);
  EXPECT_APPROX_EQ(v2_ub, 10);

  // Time 10 with speed 10 at position 100
  const auto posvel3 =
      sol_instance.get_approximate_train_pos_and_vel("tr1", 10);
  EXPECT_TRUE(posvel3.has_value());
  const auto [pos3, vel3] = posvel3.value();
  EXPECT_APPROX_EQ(pos3, 100);
  EXPECT_APPROX_EQ(vel3, 10);

  // Time 15 with speed 10 at position 150
  const auto posvel4 =
      sol_instance.get_approximate_train_pos_and_vel("tr1", 15);
  EXPECT_TRUE(posvel4.has_value());
  const auto [pos4, vel4] = posvel4.value();
  EXPECT_APPROX_EQ(pos4, 150);
  EXPECT_APPROX_EQ(vel4, 10);

  // Time 20 with speed 10 at position 200
  const auto posvel5 =
      sol_instance.get_approximate_train_pos_and_vel("tr1", 20);
  EXPECT_TRUE(posvel5.has_value());
  const auto [pos5, vel5] = posvel5.value();
  EXPECT_APPROX_EQ(pos5, 200);
  EXPECT_APPROX_EQ(vel5, 10);

  // Time 21 with speed 8 at position 209
  const auto posvel6 =
      sol_instance.get_approximate_train_pos_and_vel("tr1", 21);
  EXPECT_TRUE(posvel6.has_value());
  const auto [pos6, vel6] = posvel6.value();
  EXPECT_APPROX_EQ_2(pos6, 209, 10 * cda_rail::LINE_SPEED_ACCURACY);
  EXPECT_APPROX_EQ_2(vel6, 8, 2 * cda_rail::LINE_SPEED_ACCURACY);

  // Time 22 with speed 6 at position 216
  const auto posvel7 =
      sol_instance.get_approximate_train_pos_and_vel("tr1", 22);
  EXPECT_TRUE(posvel7.has_value());
  const auto [pos7, vel7] = posvel7.value();
  EXPECT_APPROX_EQ_2(pos7, 216, 10 * cda_rail::LINE_SPEED_ACCURACY);
  EXPECT_APPROX_EQ_2(vel7, 6, 2 * cda_rail::LINE_SPEED_ACCURACY);

  // Time 25 with speed 6 at position 216+3*6 = 234
  const auto posvel8 =
      sol_instance.get_approximate_train_pos_and_vel("tr1", 25);
  EXPECT_TRUE(posvel8.has_value());
  const auto [pos8, vel8] = posvel8.value();
  EXPECT_APPROX_EQ_2(pos8, 234, 10 * cda_rail::LINE_SPEED_ACCURACY);
  EXPECT_APPROX_EQ_2(vel8, 6, 2 * cda_rail::LINE_SPEED_ACCURACY);

  // Time 36 with speed 6 at position 300
  const auto posvel9 =
      sol_instance.get_approximate_train_pos_and_vel("tr1", 36);
  EXPECT_TRUE(posvel9.has_value());
  const auto [pos9, vel9] = posvel9.value();
  EXPECT_APPROX_EQ(pos9, 300);
  EXPECT_APPROX_EQ(vel9, 6);

  sol_instance.set_status(cda_rail::SolutionStatus::Infeasible);
  sol_instance.set_solution_not_found();
  EXPECT_TRUE(sol_instance.check_consistency());
  sol_instance.set_status(cda_rail::SolutionStatus::Timeout);
  EXPECT_TRUE(sol_instance.check_consistency());
  sol_instance.set_status(cda_rail::SolutionStatus::Optimal);
  sol_instance.set_solution_found();

  sol_instance.set_obj(-1);
  EXPECT_FALSE(sol_instance.check_consistency());
  sol_instance.set_obj(0);

  EXPECT_TRUE(sol_instance.check_consistency());

  // Test tr_order
  sol_instance.add_empty_route("tr2");
  sol_instance.push_back_edge_to_route("tr2", v1_v2);
  sol_instance.add_train_pos("tr2", 0, 0);
  sol_instance.add_train_speed("tr2", 0, 0);
  sol_instance.add_train_pos("tr2", 5, 0);
  sol_instance.add_train_speed("tr2", 5, 0);
  sol_instance.add_train_pos("tr2", 20, 200);
  sol_instance.add_train_speed("tr2", 20, 20);

  const auto tr2_pos_vel_0 =
      sol_instance.get_approximate_train_pos_and_vel("tr2", 0);
  EXPECT_TRUE(tr2_pos_vel_0.has_value());
  const auto [tr2_pos_0, tr2_vel_0] = tr2_pos_vel_0.value();
  EXPECT_APPROX_EQ(tr2_pos_0, 0);
  EXPECT_APPROX_EQ(tr2_vel_0, 0);

  const auto tr2_pos_vel_2 =
      sol_instance.get_approximate_train_pos_and_vel("tr2", 2);
  EXPECT_TRUE(tr2_pos_vel_2.has_value());
  const auto [tr2_pos_2, tr2_vel_2] = tr2_pos_vel_2.value();
  EXPECT_APPROX_EQ(tr2_pos_2, 0);
  EXPECT_APPROX_EQ(tr2_vel_2, 0);

  const auto tr2_pos_vel_5 =
      sol_instance.get_approximate_train_pos_and_vel("tr2", 5);
  EXPECT_TRUE(tr2_pos_vel_5.has_value());
  const auto [tr2_pos_5, tr2_vel_5] = tr2_pos_vel_5.value();
  EXPECT_APPROX_EQ(tr2_pos_5, 0);
  EXPECT_APPROX_EQ(tr2_vel_5, 0);

  const auto tr2_pos_vel_15 =
      sol_instance.get_approximate_train_pos_and_vel("tr2", 15);
  EXPECT_TRUE(tr2_pos_vel_15.has_value());
  const auto [tr2_pos_15, tr2_vel_15] = tr2_pos_vel_15.value();
  EXPECT_APPROX_EQ(tr2_pos_15, 100);
  EXPECT_APPROX_EQ(tr2_vel_15, 20);

  const auto tr2_pos_vel_20 =
      sol_instance.get_approximate_train_pos_and_vel("tr2", 20);
  EXPECT_TRUE(tr2_pos_vel_20.has_value());
  const auto [tr2_pos_20, tr2_vel_20] = tr2_pos_vel_20.value();
  EXPECT_APPROX_EQ(tr2_pos_20, 200);
  EXPECT_APPROX_EQ(tr2_vel_20, 20);

  const auto [pos_lb_0, pos_ub_0, v_lb_0, v_ub_0] =
      sol_instance.get_exact_pos_and_vel_bounds("tr2", 0);
  EXPECT_APPROX_EQ(pos_lb_0, 0);
  EXPECT_APPROX_EQ(pos_ub_0, 0);
  EXPECT_APPROX_EQ(v_lb_0, 0);
  EXPECT_APPROX_EQ(v_ub_0, 0);

  const auto [pos_lb_2, pos_ub_2, v_lb_1, v_ub_1] =
      sol_instance.get_exact_pos_and_vel_bounds("tr2", 2);
  EXPECT_APPROX_EQ(pos_lb_2, 0);
  EXPECT_APPROX_EQ(pos_ub_2, 0);
  EXPECT_APPROX_EQ(v_lb_1, 0);
  EXPECT_APPROX_EQ(v_ub_1, 0);

  const auto [pos_lb_5, pos_ub_5, v_lb_2, v_ub_2] =
      sol_instance.get_exact_pos_and_vel_bounds("tr2", 5);
  EXPECT_APPROX_EQ(pos_lb_5, 0);
  EXPECT_APPROX_EQ(pos_ub_5, 0);
  EXPECT_APPROX_EQ(v_lb_2, 0);
  EXPECT_APPROX_EQ(v_ub_2, 0);

  const auto tr_order = sol_instance.get_train_order(v0_v1);
  EXPECT_EQ(tr_order.size(), 1);
  EXPECT_EQ(tr_order.at(0), tr1);

  const auto tr_order_2 = sol_instance.get_train_order(v1_v2);
  EXPECT_EQ(tr_order_2.size(), 2);
  EXPECT_EQ(tr_order_2.at(0), tr2);
  EXPECT_EQ(tr_order_2.at(1), tr1);
}

TEST(GeneralPerformanceOptimizationInstances,
     SolGeneralPerformanceOptimizationInstanceTrainOrderWithReverseEdge) {
  instances::GeneralPerformanceOptimizationInstance instance;

  // Add a simple network to the instance
  const auto v0 = instance.n().add_vertex("v0", cda_rail::VertexType::TTD);
  const auto v1 = instance.n().add_vertex("v1", cda_rail::VertexType::TTD);
  const auto v2 = instance.n().add_vertex("v2", cda_rail::VertexType::TTD);

  const auto v0_v1 = instance.n().add_edge("v0", "v1", 100, 10, false);
  const auto v1_v2 = instance.n().add_edge("v1", "v2", 200, 20, false);
  const auto v1_v0 = instance.n().add_edge("v1", "v0", 100, 10, false);
  const auto v2_v1 = instance.n().add_edge("v2", "v1", 200, 20, false);

  instance.n().add_successor({"v0", "v1"}, {"v1", "v2"});
  instance.n().add_successor({"v2", "v1"}, {"v1", "v0"});

  const auto tr1 = instance.add_train("tr1", 50, 10, 2, 2, {0, 160}, 10, "v0",
                                      {30, 270}, 6, "v2");
  const auto tr2 = instance.add_train("tr2", 50, 20, 2, 2, {0, 160}, 0, "v2",
                                      {30, 270}, 0, "v0", 2, true);
  const auto tr3 = instance.add_train("tr3", 50, 10, 2, 2, {0, 160}, 10, "v0",
                                      {30, 270}, 6, "v2");

  // Check the consistency of the instance
  EXPECT_TRUE(instance.check_consistency(false));

  instances::SolGeneralPerformanceOptimizationInstance<
      instances::GeneralPerformanceOptimizationInstance>
      sol_instance(instance);

  sol_instance.add_empty_route("tr1");
  sol_instance.add_empty_route("tr2");
  sol_instance.add_empty_route("tr3");

  sol_instance.push_back_edge_to_route("tr1", v0_v1);
  sol_instance.push_back_edge_to_route("tr1", v1_v2);

  sol_instance.push_back_edge_to_route("tr2", v2_v1);
  sol_instance.push_back_edge_to_route("tr2", v1_v0);

  sol_instance.push_back_edge_to_route("tr3", v0, v1);
  sol_instance.push_back_edge_to_route("tr3", v1, v2);

  sol_instance.add_train_pos("tr1", 0, 0);
  sol_instance.add_train_pos("tr1", 10, 100);
  sol_instance.add_train_pos("tr1", 20, 200);
  sol_instance.add_train_pos("tr1", 30, 300);
  sol_instance.add_train_speed("tr1", 0, 10);
  sol_instance.add_train_speed("tr1", 10, 10);
  sol_instance.add_train_speed("tr1", 20, 10);
  sol_instance.add_train_speed("tr1", 30, 10);

  sol_instance.add_train_pos("tr2", 40, 0);
  sol_instance.add_train_pos("tr2", 50, 100);
  sol_instance.add_train_pos("tr2", 60, 200);
  sol_instance.add_train_pos("tr2", 70, 300);
  sol_instance.add_train_speed("tr2", 40, 10);
  sol_instance.add_train_speed("tr2", 50, 10);
  sol_instance.add_train_speed("tr2", 60, 10);
  sol_instance.add_train_speed("tr2", 70, 10);

  sol_instance.add_train_pos("tr3", 80, 0);
  sol_instance.add_train_pos("tr3", 90, 100);
  sol_instance.add_train_pos("tr3", 100, 200);
  sol_instance.add_train_pos("tr3", 110, 300);
  sol_instance.add_train_speed("tr3", 80, 10);
  sol_instance.add_train_speed("tr3", 90, 10);
  sol_instance.add_train_speed("tr3", 100, 10);
  sol_instance.add_train_speed("tr3", 110, 10);

  // Check Train Orders
  const auto tr_order_v0_v1 = sol_instance.get_train_order(v0_v1);
  EXPECT_EQ(tr_order_v0_v1.size(), 2);
  EXPECT_EQ(tr_order_v0_v1.at(0), tr1);
  EXPECT_EQ(tr_order_v0_v1.at(1), tr3);

  const auto tr_order_v1_v2 = sol_instance.get_train_order(v1_v2);
  EXPECT_EQ(tr_order_v1_v2.size(), 2);
  EXPECT_EQ(tr_order_v1_v2.at(0), tr1);
  EXPECT_EQ(tr_order_v1_v2.at(1), tr3);

  const auto tr_order_v1_v0 = sol_instance.get_train_order(v1_v0);
  EXPECT_EQ(tr_order_v1_v0.size(), 1);
  EXPECT_EQ(tr_order_v1_v0.at(0), tr2);

  const auto tr_order_v2_v1 = sol_instance.get_train_order(v2_v1);
  EXPECT_EQ(tr_order_v2_v1.size(), 1);
  EXPECT_EQ(tr_order_v2_v1.at(0), tr2);

  const auto tr_order_rev_v0_v1 =
      sol_instance.get_train_order_with_reverse(v0_v1);
  EXPECT_EQ(tr_order_rev_v0_v1.size(), 3);
  EXPECT_EQ(tr_order_rev_v0_v1.at(0).first, tr1);
  EXPECT_EQ(tr_order_rev_v0_v1.at(1).first, tr2);
  EXPECT_EQ(tr_order_rev_v0_v1.at(2).first, tr3);
  EXPECT_TRUE(tr_order_rev_v0_v1.at(0).second);
  EXPECT_FALSE(tr_order_rev_v0_v1.at(1).second);
  EXPECT_TRUE(tr_order_rev_v0_v1.at(2).second);

  const auto tr_order_rev_v1_v2 =
      sol_instance.get_train_order_with_reverse(v1_v2);
  EXPECT_EQ(tr_order_rev_v1_v2.size(), 3);
  EXPECT_EQ(tr_order_rev_v1_v2.at(0).first, tr1);
  EXPECT_EQ(tr_order_rev_v1_v2.at(1).first, tr2);
  EXPECT_EQ(tr_order_rev_v1_v2.at(2).first, tr3);
  EXPECT_TRUE(tr_order_rev_v1_v2.at(0).second);
  EXPECT_FALSE(tr_order_rev_v1_v2.at(1).second);
  EXPECT_TRUE(tr_order_rev_v1_v2.at(2).second);

  const auto tr_order_rev_v1_v0 =
      sol_instance.get_train_order_with_reverse(v1_v0);
  EXPECT_EQ(tr_order_rev_v1_v0.size(), 3);
  EXPECT_EQ(tr_order_rev_v1_v0.at(0).first, tr1);
  EXPECT_EQ(tr_order_rev_v1_v0.at(1).first, tr2);
  EXPECT_EQ(tr_order_rev_v1_v0.at(2).first, tr3);
  EXPECT_FALSE(tr_order_rev_v1_v0.at(0).second);
  EXPECT_TRUE(tr_order_rev_v1_v0.at(1).second);
  EXPECT_FALSE(tr_order_rev_v1_v0.at(2).second);

  const auto tr_order_rev_v2_v1 =
      sol_instance.get_train_order_with_reverse(v2_v1);
  EXPECT_EQ(tr_order_rev_v2_v1.size(), 3);
  EXPECT_EQ(tr_order_rev_v2_v1.at(0).first, tr1);
  EXPECT_EQ(tr_order_rev_v2_v1.at(1).first, tr2);
  EXPECT_EQ(tr_order_rev_v2_v1.at(2).first, tr3);
  EXPECT_FALSE(tr_order_rev_v2_v1.at(0).second);
  EXPECT_TRUE(tr_order_rev_v2_v1.at(1).second);
  EXPECT_FALSE(tr_order_rev_v2_v1.at(2).second);
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

  instances::SolGeneralPerformanceOptimizationInstance<
      instances::GeneralPerformanceOptimizationInstance>
      sol_instance(instance);

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
      cda_rail::instances::SolGeneralPerformanceOptimizationInstance<
          instances::GeneralPerformanceOptimizationInstance>::
          import_solution("./tmp/test-sol-instance-1");
  const auto sol2_read =
      cda_rail::instances::SolGeneralPerformanceOptimizationInstance<
          instances::GeneralPerformanceOptimizationInstance>::
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

  // Discretize stop edges

  instance.discretize_stops();

  // All stop edges should have been separated once ...

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

  // ... in the middle at 150m having carried over the properties of the
  // original edge

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

TEST(GeneralPerformanceOptimizationInstances, StopVertices) {
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance;

  // Create network
  size_t v0  = instance.n().add_vertex("v0", cda_rail::VertexType::TTD);
  size_t v1  = instance.n().add_vertex("v1", cda_rail::VertexType::TTD);
  size_t v21 = instance.n().add_vertex("v21", cda_rail::VertexType::TTD);
  size_t v22 = instance.n().add_vertex("v22", cda_rail::VertexType::TTD);
  size_t v31 = instance.n().add_vertex("v31", cda_rail::VertexType::TTD);
  size_t v32 = instance.n().add_vertex("v32", cda_rail::VertexType::TTD);
  size_t v41 = instance.n().add_vertex("v41", cda_rail::VertexType::TTD);
  size_t v42 = instance.n().add_vertex("v42", cda_rail::VertexType::TTD);
  size_t v43 = instance.n().add_vertex("v43", cda_rail::VertexType::TTD);
  size_t v51 = instance.n().add_vertex("v51", cda_rail::VertexType::TTD);
  size_t v52 = instance.n().add_vertex("v52", cda_rail::VertexType::TTD);
  size_t v53 = instance.n().add_vertex("v53", cda_rail::VertexType::TTD);
  size_t v6  = instance.n().add_vertex("v6", cda_rail::VertexType::TTD);
  size_t v7  = instance.n().add_vertex("v7", cda_rail::VertexType::TTD);

  // Edges to add
  const std::vector<std::tuple<size_t, size_t, double>> to_add = {
      {v0, v1, 50},   {v1, v21, 10},  {v1, v22, 10},  {v21, v31, 50},
      {v22, v32, 50}, {v31, v41, 20}, {v32, v42, 10}, {v32, v43, 10},
      {v41, v51, 50}, {v41, v42, 10}, {v42, v52, 60}, {v43, v53, 50},
      {v51, v6, 10},  {v52, v6, 10},  {v53, v6, 10},  {v6, v7, 100}};

  std::unordered_map<int, size_t> edge_map;

  // Add bidirectional edges of specified length
  for (const auto& [source, target, length] : to_add) {
    edge_map.insert_or_assign(
        100 * source + target,
        instance.n().add_edge(source, target, length, 50));
    edge_map.insert_or_assign(
        100 * target + source,
        instance.n().add_edge(target, source, length, 50));
  }

  EXPECT_EQ(edge_map.size(), 2 * to_add.size());

  // Add successors
  instance.n().add_successor({v0, v1}, {v1, v21});
  instance.n().add_successor({v0, v1}, {v1, v22});
  instance.n().add_successor({v1, v21}, {v21, v31});
  instance.n().add_successor({v1, v22}, {v22, v32});
  instance.n().add_successor({v21, v31}, {v31, v41});
  instance.n().add_successor({v22, v32}, {v32, v42});
  instance.n().add_successor({v22, v32}, {v32, v43});
  instance.n().add_successor({v31, v41}, {v41, v51});
  instance.n().add_successor({v32, v42}, {v42, v41});
  instance.n().add_successor({v32, v42}, {v42, v52});
  instance.n().add_successor({v32, v43}, {v43, v53});
  instance.n().add_successor({v41, v51}, {v51, v6});
  instance.n().add_successor({v42, v52}, {v52, v6});
  instance.n().add_successor({v43, v53}, {v53, v6});
  instance.n().add_successor({v51, v6}, {v6, v7});
  instance.n().add_successor({v52, v6}, {v6, v7});
  instance.n().add_successor({v53, v6}, {v6, v7});
  // And all reverse successors
  instance.n().add_successor({v21, v1}, {v1, v0});
  instance.n().add_successor({v22, v1}, {v1, v0});
  instance.n().add_successor({v31, v21}, {v21, v1});
  instance.n().add_successor({v32, v22}, {v22, v1});
  instance.n().add_successor({v41, v31}, {v31, v21});
  instance.n().add_successor({v42, v32}, {v32, v22});
  instance.n().add_successor({v43, v32}, {v32, v22});
  instance.n().add_successor({v51, v41}, {v41, v31});
  instance.n().add_successor({v41, v42}, {v42, v32});
  instance.n().add_successor({v52, v42}, {v42, v32});
  instance.n().add_successor({v53, v43}, {v43, v32});
  instance.n().add_successor({v6, v51}, {v51, v41});
  instance.n().add_successor({v6, v52}, {v52, v42});
  instance.n().add_successor({v6, v53}, {v53, v43});
  instance.n().add_successor({v7, v6}, {v6, v51});
  instance.n().add_successor({v7, v6}, {v6, v52});
  instance.n().add_successor({v7, v6}, {v6, v53});

  // Add station bidirectional edges v21-v31-v41-v51, v22-v32-v42-v52,
  // v22-v32-v43-v53
  instance.add_station("Station1");
  instance.add_track_to_station("Station1", v21, v31);
  instance.add_track_to_station("Station1", v31, v41);
  instance.add_track_to_station("Station1", v41, v51);
  instance.add_track_to_station("Station1", v22, v32);
  instance.add_track_to_station("Station1", v32, v42);
  instance.add_track_to_station("Station1", v42, v52);
  instance.add_track_to_station("Station1", v32, v43);
  instance.add_track_to_station("Station1", v43, v53);
  instance.add_track_to_station("Station1", v31, v21);
  instance.add_track_to_station("Station1", v41, v31);
  instance.add_track_to_station("Station1", v51, v41);
  instance.add_track_to_station("Station1", v32, v22);
  instance.add_track_to_station("Station1", v42, v32);
  instance.add_track_to_station("Station1", v52, v42);
  instance.add_track_to_station("Station1", v43, v32);
  instance.add_track_to_station("Station1", v53, v43);

  // Add trains of various length
  instance.add_train("Train100", 100, 50, 1, 1, {0, 60}, 10, v0, {300, 360}, 5,
                     v7);
  instance.add_train("Train60", 60, 50, 1, 1, {0, 60}, 10, v0, {300, 360}, 5,
                     v7);
  instance.add_train("Train30", 30, 50, 1, 1, {0, 60}, 10, v0, {300, 360}, 5,
                     v7);

  const auto tr100stops =
      instance.possible_stop_vertices("Train100", "Station1");
  // Expect 5 stop vertices
  EXPECT_EQ(tr100stops.size(), 5);
  std::unordered_map<size_t, size_t> tr100stop_map;
  for (size_t i = 0; i < tr100stops.size(); ++i) {
    tr100stop_map.insert_or_assign(tr100stops[i].first, i);
  }
  // Expect stop map to have keys v21, v51, v22, v52, and v53
  EXPECT_EQ(tr100stop_map.size(), 5);
  EXPECT_EQ(tr100stop_map.count(v21), 1);
  EXPECT_EQ(tr100stop_map.count(v51), 1);
  EXPECT_EQ(tr100stop_map.count(v22), 1);
  EXPECT_EQ(tr100stop_map.count(v52), 1);
  EXPECT_EQ(tr100stop_map.count(v53), 1);

  // At v21, the stop path is v21-v31-v41-v51 using the reverse edges
  const auto& [stop_100_21_v, stop_100_21_p] = tr100stops[tr100stop_map[v21]];
  EXPECT_EQ(stop_100_21_v, v21);
  EXPECT_EQ(stop_100_21_p.size(), 1);
  EXPECT_TRUE(std::find(stop_100_21_p.begin(), stop_100_21_p.end(),
                        std::vector<size_t>({edge_map.at(v21 + 100 * v31),
                                             edge_map.at(v31 + 100 * v41),
                                             edge_map.at(v41 + 100 * v51)})) !=
              stop_100_21_p.end());

  // At v51, the stop path is v51-v41-v31-v21 using the reverse edges
  const auto& [stop_100_51_v, stop_100_51_p] = tr100stops[tr100stop_map[v51]];
  EXPECT_EQ(stop_100_51_v, v51);
  EXPECT_EQ(stop_100_51_p.size(), 1);
  EXPECT_TRUE(std::find(stop_100_51_p.begin(), stop_100_51_p.end(),
                        std::vector<size_t>({edge_map.at(v51 + 100 * v41),
                                             edge_map.at(v41 + 100 * v31),
                                             edge_map.at(v31 + 100 * v21)})) !=
              stop_100_51_p.end());

  // At v22, the stop path is v22-v32-v42-v52 or v22-v32-v43-v53 using the
  // reverse edges
  const auto& [stop_100_22_v, stop_100_22_p] = tr100stops[tr100stop_map[v22]];
  EXPECT_EQ(stop_100_22_v, v22);
  EXPECT_EQ(stop_100_22_p.size(), 2);
  EXPECT_TRUE(std::find(stop_100_22_p.begin(), stop_100_22_p.end(),
                        std::vector<size_t>({edge_map.at(v22 + 100 * v32),
                                             edge_map.at(v32 + 100 * v42),
                                             edge_map.at(v42 + 100 * v52)})) !=
              stop_100_22_p.end());
  EXPECT_TRUE(std::find(stop_100_22_p.begin(), stop_100_22_p.end(),
                        std::vector<size_t>({edge_map.at(v22 + 100 * v32),
                                             edge_map.at(v32 + 100 * v43),
                                             edge_map.at(v43 + 100 * v53)})) !=
              stop_100_22_p.end());

  // At v52, the stop path is v52-v42-v32-v22 using the reverse edges
  const auto& [stop_100_52_v, stop_100_52_p] = tr100stops[tr100stop_map[v52]];
  EXPECT_EQ(stop_100_52_v, v52);
  EXPECT_EQ(stop_100_52_p.size(), 1);
  EXPECT_TRUE(std::find(stop_100_52_p.begin(), stop_100_52_p.end(),
                        std::vector<size_t>({edge_map.at(v52 + 100 * v42),
                                             edge_map.at(v42 + 100 * v32),
                                             edge_map.at(v32 + 100 * v22)})) !=
              stop_100_52_p.end());

  // At v53, the stop path is v53-v43-v32-v22 using the reverse edges
  const auto& [stop_100_53_v, stop_100_53_p] = tr100stops[tr100stop_map[v53]];
  EXPECT_EQ(stop_100_53_v, v53);
  EXPECT_EQ(stop_100_53_p.size(), 1);
  EXPECT_TRUE(std::find(stop_100_53_p.begin(), stop_100_53_p.end(),
                        std::vector<size_t>({edge_map.at(v53 + 100 * v43),
                                             edge_map.at(v43 + 100 * v32),
                                             edge_map.at(v32 + 100 * v22)})) !=
              stop_100_53_p.end());

  const auto tr60stops = instance.possible_stop_vertices("Train60", "Station1");
  // Expect v21, v31, v41, v51, v22, v32, v42, v52, v43, v53
  // i.e. 10 stop vertices
  EXPECT_EQ(tr60stops.size(), 10);
  std::unordered_map<size_t, size_t> tr60stop_map;
  for (size_t i = 0; i < tr60stops.size(); ++i) {
    tr60stop_map.insert_or_assign(tr60stops[i].first, i);
  }
  EXPECT_EQ(tr60stop_map.size(), 10);
  EXPECT_EQ(tr60stop_map.count(v21), 1);
  EXPECT_EQ(tr60stop_map.count(v31), 1);
  EXPECT_EQ(tr60stop_map.count(v41), 1);
  EXPECT_EQ(tr60stop_map.count(v51), 1);
  EXPECT_EQ(tr60stop_map.count(v22), 1);
  EXPECT_EQ(tr60stop_map.count(v32), 1);
  EXPECT_EQ(tr60stop_map.count(v42), 1);
  EXPECT_EQ(tr60stop_map.count(v52), 1);
  EXPECT_EQ(tr60stop_map.count(v43), 1);
  EXPECT_EQ(tr60stop_map.count(v53), 1);

  // At v21, the stop path is v21-v31-v41 using the reverse edges
  const auto& [stop_60_21_v, stop_60_21_p] = tr60stops[tr60stop_map[v21]];
  EXPECT_EQ(stop_60_21_v, v21);
  EXPECT_EQ(stop_60_21_p.size(), 1);
  EXPECT_TRUE(std::find(stop_60_21_p.begin(), stop_60_21_p.end(),
                        std::vector<size_t>({edge_map.at(v21 + 100 * v31),
                                             edge_map.at(v31 + 100 * v41)})) !=
              stop_60_21_p.end());

  // At v31, the stop path is v31-v41-v51 using the reverse edges
  const auto& [stop_60_31_v, stop_60_31_p] = tr60stops[tr60stop_map[v31]];
  EXPECT_EQ(stop_60_31_v, v31);
  EXPECT_EQ(stop_60_31_p.size(), 1);
  EXPECT_TRUE(std::find(stop_60_31_p.begin(), stop_60_31_p.end(),
                        std::vector<size_t>({edge_map.at(v31 + 100 * v41),
                                             edge_map.at(v41 + 100 * v51)})) !=
              stop_60_31_p.end());

  // At v41, the stop path is v41-v31-v21 using the reverse edges
  const auto& [stop_60_41_v, stop_60_41_p] = tr60stops[tr60stop_map[v41]];
  EXPECT_EQ(stop_60_41_v, v41);
  EXPECT_EQ(stop_60_41_p.size(), 1);
  EXPECT_TRUE(std::find(stop_60_41_p.begin(), stop_60_41_p.end(),
                        std::vector<size_t>({edge_map.at(v41 + 100 * v31),
                                             edge_map.at(v31 + 100 * v21)})) !=
              stop_60_41_p.end());

  // At v51, the stop path is v51-v41-v31 using the reverse edges
  const auto& [stop_60_51_v, stop_60_51_p] = tr60stops[tr60stop_map[v51]];
  EXPECT_EQ(stop_60_51_v, v51);
  EXPECT_EQ(stop_60_51_p.size(), 1);
  EXPECT_TRUE(std::find(stop_60_51_p.begin(), stop_60_51_p.end(),
                        std::vector<size_t>({edge_map.at(v51 + 100 * v41),
                                             edge_map.at(v41 + 100 * v31)})) !=
              stop_60_51_p.end());

  // At v22, the stop path is v22-v32-v42 or v22-v32-v43 using the reverse edges
  const auto& [stop_60_22_v, stop_60_22_p] = tr60stops[tr60stop_map[v22]];
  EXPECT_EQ(stop_60_22_v, v22);
  EXPECT_EQ(stop_60_22_p.size(), 2);
  EXPECT_TRUE(std::find(stop_60_22_p.begin(), stop_60_22_p.end(),
                        std::vector<size_t>({edge_map.at(v22 + 100 * v32),
                                             edge_map.at(v32 + 100 * v42)})) !=
              stop_60_22_p.end());
  EXPECT_TRUE(std::find(stop_60_22_p.begin(), stop_60_22_p.end(),
                        std::vector<size_t>({edge_map.at(v22 + 100 * v32),
                                             edge_map.at(v32 + 100 * v43)})) !=
              stop_60_22_p.end());

  // At v32, the stop path is v32-v42-v52 or v32-v43-v53 using the reverse edges
  const auto& [stop_60_32_v, stop_60_32_p] = tr60stops[tr60stop_map[v32]];
  EXPECT_EQ(stop_60_32_v, v32);
  EXPECT_EQ(stop_60_32_p.size(), 2);
  EXPECT_TRUE(std::find(stop_60_32_p.begin(), stop_60_32_p.end(),
                        std::vector<size_t>({edge_map.at(v32 + 100 * v42),
                                             edge_map.at(v42 + 100 * v52)})) !=
              stop_60_32_p.end());
  EXPECT_TRUE(std::find(stop_60_32_p.begin(), stop_60_32_p.end(),
                        std::vector<size_t>({edge_map.at(v32 + 100 * v43),
                                             edge_map.at(v43 + 100 * v53)})) !=
              stop_60_32_p.end());

  // At v42, the stop path is v42-v32-v22 or v42-v52 using the reverse edges
  const auto& [stop_60_42_v, stop_60_42_p] = tr60stops[tr60stop_map[v42]];
  EXPECT_EQ(stop_60_42_v, v42);
  EXPECT_EQ(stop_60_42_p.size(), 2);
  EXPECT_TRUE(std::find(stop_60_42_p.begin(), stop_60_42_p.end(),
                        std::vector<size_t>({edge_map.at(v42 + 100 * v32),
                                             edge_map.at(v32 + 100 * v22)})) !=
              stop_60_42_p.end());
  EXPECT_TRUE(std::find(stop_60_42_p.begin(), stop_60_42_p.end(),
                        std::vector<size_t>({edge_map.at(v42 + 100 * v52)})) !=
              stop_60_42_p.end());

  // At v52, the stop path is v52-v42 using the reverse edges
  const auto& [stop_60_52_v, stop_60_52_p] = tr60stops[tr60stop_map[v52]];
  EXPECT_EQ(stop_60_52_v, v52);
  EXPECT_EQ(stop_60_52_p.size(), 1);
  EXPECT_TRUE(std::find(stop_60_52_p.begin(), stop_60_52_p.end(),
                        std::vector<size_t>({edge_map.at(v52 + 100 * v42)})) !=
              stop_60_52_p.end());

  // At v43 the stop path is v43-v32-v22 using the reverse edges
  const auto& [stop_60_43_v, stop_60_43_p] = tr60stops[tr60stop_map[v43]];
  EXPECT_EQ(stop_60_43_v, v43);
  EXPECT_EQ(stop_60_43_p.size(), 1);
  EXPECT_TRUE(std::find(stop_60_43_p.begin(), stop_60_43_p.end(),
                        std::vector<size_t>({edge_map.at(v43 + 100 * v32),
                                             edge_map.at(v32 + 100 * v22)})) !=
              stop_60_43_p.end());

  // At v53 the stop path is v53-v43-v32 using the reverse edges
  const auto& [stop_60_53_v, stop_60_53_p] = tr60stops[tr60stop_map[v53]];
  EXPECT_EQ(stop_60_53_v, v53);
  EXPECT_EQ(stop_60_53_p.size(), 1);
  EXPECT_TRUE(std::find(stop_60_53_p.begin(), stop_60_53_p.end(),
                        std::vector<size_t>({edge_map.at(v53 + 100 * v43),
                                             edge_map.at(v43 + 100 * v32)})) !=
              stop_60_53_p.end());

  const auto tr30stops = instance.possible_stop_vertices("Train30", "Station1");
  // Expect v21, v31, v41, v51, v22, v32, v42, v52, v43, v53
  // i.e. 10 stop vertices
  EXPECT_EQ(tr30stops.size(), 10);
  std::unordered_map<size_t, size_t> tr30stop_map;
  for (size_t i = 0; i < tr30stops.size(); ++i) {
    tr30stop_map.insert_or_assign(tr30stops[i].first, i);
  }
  EXPECT_EQ(tr30stop_map.size(), 10);
  EXPECT_EQ(tr30stop_map.count(v21), 1);
  EXPECT_EQ(tr30stop_map.count(v31), 1);
  EXPECT_EQ(tr30stop_map.count(v41), 1);
  EXPECT_EQ(tr30stop_map.count(v51), 1);
  EXPECT_EQ(tr30stop_map.count(v22), 1);
  EXPECT_EQ(tr30stop_map.count(v32), 1);
  EXPECT_EQ(tr30stop_map.count(v42), 1);
  EXPECT_EQ(tr30stop_map.count(v52), 1);
  EXPECT_EQ(tr30stop_map.count(v43), 1);
  EXPECT_EQ(tr30stop_map.count(v53), 1);

  // At v21, the stop path is v21-v31 using the reverse edges
  const auto& [stop_30_21_v, stop_30_21_p] = tr30stops[tr30stop_map[v21]];
  EXPECT_EQ(stop_30_21_v, v21);
  EXPECT_EQ(stop_30_21_p.size(), 1);
  EXPECT_TRUE(std::find(stop_30_21_p.begin(), stop_30_21_p.end(),
                        std::vector<size_t>({edge_map.at(v21 + 100 * v31)})) !=
              stop_30_21_p.end());

  // At v31, the stop path is v31-v21 or v31-v41-v51 using the reverse edges
  const auto& [stop_30_31_v, stop_30_31_p] = tr30stops[tr30stop_map[v31]];
  EXPECT_EQ(stop_30_31_v, v31);
  EXPECT_EQ(stop_30_31_p.size(), 2);
  EXPECT_TRUE(std::find(stop_30_31_p.begin(), stop_30_31_p.end(),
                        std::vector<size_t>({edge_map.at(v31 + 100 * v21)})) !=
              stop_30_31_p.end());
  EXPECT_TRUE(std::find(stop_30_31_p.begin(), stop_30_31_p.end(),
                        std::vector<size_t>({edge_map.at(v31 + 100 * v41),
                                             edge_map.at(v41 + 100 * v51)})) !=
              stop_30_31_p.end());

  // At v41, the stop path is v41-v31-v21 or v41-v51 using the reverse edges
  const auto& [stop_30_41_v, stop_30_41_p] = tr30stops[tr30stop_map[v41]];
  EXPECT_EQ(stop_30_41_v, v41);
  EXPECT_EQ(stop_30_41_p.size(), 2);
  EXPECT_TRUE(std::find(stop_30_41_p.begin(), stop_30_41_p.end(),
                        std::vector<size_t>({edge_map.at(v41 + 100 * v31),
                                             edge_map.at(v31 + 100 * v21)})) !=
              stop_30_41_p.end());
  EXPECT_TRUE(std::find(stop_30_41_p.begin(), stop_30_41_p.end(),
                        std::vector<size_t>({edge_map.at(v41 + 100 * v51)})) !=
              stop_30_41_p.end());

  // At v51, the stop path is v51-v41 using the reverse edges
  const auto& [stop_30_51_v, stop_30_51_p] = tr30stops[tr30stop_map[v51]];
  EXPECT_EQ(stop_30_51_v, v51);
  EXPECT_EQ(stop_30_51_p.size(), 1);
  EXPECT_TRUE(std::find(stop_30_51_p.begin(), stop_30_51_p.end(),
                        std::vector<size_t>({edge_map.at(v51 + 100 * v41)})) !=
              stop_30_51_p.end());

  // At v22, the stop path is v22-v32 using the reverse edges
  const auto& [stop_30_22_v, stop_30_22_p] = tr30stops[tr30stop_map[v22]];
  EXPECT_EQ(stop_30_22_v, v22);
  EXPECT_EQ(stop_30_22_p.size(), 1);
  EXPECT_TRUE(std::find(stop_30_22_p.begin(), stop_30_22_p.end(),
                        std::vector<size_t>({edge_map.at(v22 + 100 * v32)})) !=
              stop_30_22_p.end());

  // At v32, the stop path is v32-v22 or v32-v42-v52 or v32-v43-v53 using the
  // reverse edges
  const auto& [stop_30_32_v, stop_30_32_p] = tr30stops[tr30stop_map[v32]];
  EXPECT_EQ(stop_30_32_v, v32);
  EXPECT_EQ(stop_30_32_p.size(), 3);
  EXPECT_TRUE(std::find(stop_30_32_p.begin(), stop_30_32_p.end(),
                        std::vector<size_t>({edge_map.at(v32 + 100 * v22)})) !=
              stop_30_32_p.end());
  EXPECT_TRUE(std::find(stop_30_32_p.begin(), stop_30_32_p.end(),
                        std::vector<size_t>({edge_map.at(v32 + 100 * v42),
                                             edge_map.at(v42 + 100 * v52)})) !=
              stop_30_32_p.end());
  EXPECT_TRUE(std::find(stop_30_32_p.begin(), stop_30_32_p.end(),
                        std::vector<size_t>({edge_map.at(v32 + 100 * v43),
                                             edge_map.at(v43 + 100 * v53)})) !=
              stop_30_32_p.end());

  // At v42, the stop path is v42-v32-v22 or v42-v52 using the reverse edges
  const auto& [stop_30_42_v, stop_30_42_p] = tr30stops[tr30stop_map[v42]];
  EXPECT_EQ(stop_30_42_v, v42);
  EXPECT_EQ(stop_30_42_p.size(), 2);
  EXPECT_TRUE(std::find(stop_30_42_p.begin(), stop_30_42_p.end(),
                        std::vector<size_t>({edge_map.at(v42 + 100 * v32),
                                             edge_map.at(v32 + 100 * v22)})) !=
              stop_30_42_p.end());
  EXPECT_TRUE(std::find(stop_30_42_p.begin(), stop_30_42_p.end(),
                        std::vector<size_t>({edge_map.at(v42 + 100 * v52)})) !=
              stop_30_42_p.end());

  // At v52, the stop path is v52-v42 using the reverse edges
  const auto& [stop_30_52_v, stop_30_52_p] = tr30stops[tr30stop_map[v52]];
  EXPECT_EQ(stop_30_52_v, v52);
  EXPECT_EQ(stop_30_52_p.size(), 1);
  EXPECT_TRUE(std::find(stop_30_52_p.begin(), stop_30_52_p.end(),
                        std::vector<size_t>({edge_map.at(v52 + 100 * v42)})) !=
              stop_30_52_p.end());

  // At v43 the stop path is v43-v32-v22 or v43-v53 using the reverse edges
  const auto& [stop_30_43_v, stop_30_43_p] = tr30stops[tr30stop_map[v43]];
  EXPECT_EQ(stop_30_43_v, v43);
  EXPECT_EQ(stop_30_43_p.size(), 2);
  EXPECT_TRUE(std::find(stop_30_43_p.begin(), stop_30_43_p.end(),
                        std::vector<size_t>({edge_map.at(v43 + 100 * v32),
                                             edge_map.at(v32 + 100 * v22)})) !=
              stop_30_43_p.end());
  EXPECT_TRUE(std::find(stop_30_43_p.begin(), stop_30_43_p.end(),
                        std::vector<size_t>({edge_map.at(v43 + 100 * v53)})) !=
              stop_30_43_p.end());

  // At v53 the stop path is v53-v43 using the reverse edges
  const auto& [stop_30_53_v, stop_30_53_p] = tr30stops[tr30stop_map[v53]];
  EXPECT_EQ(stop_30_53_v, v53);
  EXPECT_EQ(stop_30_53_p.size(), 1);
  EXPECT_TRUE(std::find(stop_30_53_p.begin(), stop_30_53_p.end(),
                        std::vector<size_t>({edge_map.at(v53 + 100 * v43)})) !=
              stop_30_53_p.end());
}

TEST(GeneralPerformanceOptimizationInstances, LeavingTimes) {
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance;

  // Create simple network
  const auto v0 = instance.n().add_vertex("v0", cda_rail::VertexType::TTD);
  const auto v1 = instance.n().add_vertex("v1", cda_rail::VertexType::TTD);

  const auto e01 = instance.n().add_edge(v0, v1, 1000, 20);

  // Add train and schedule
  const auto tr1 = instance.add_train("Train1", 56, 50, 1.5, 2, {0, 60}, 20, v0,
                                      {300, 360}, 8, v1);
  // Train 2 stops on exit
  // It will be stopped.
  // Then accelerate to 8 m/s in 8 seconds travelling 4*8=32 meters
  // Then remain at constant speed for 2 seconds traveling 2*8=16 meters
  // Then decelerate for 4 seconds to 0 m/s traveling 4*4=16 meters
  // In total 32+16+16=64 meters -> length
  const auto tr2 = instance.add_train("Train2", 64, 8, 1, 2, {120, 180}, 20, v0,
                                      {500, 560}, 0, v1);

  // Train 3 stops on exit
  // It will be stopped.
  // Then accelerate to 20 m/s at rate 2 for 10 seconds
  // traveling 10*10=100 meters
  // Then remain at constant speed for 2 seconds traveling 2*20=40 meters
  // Then decelerate at rate 4 for 5 seconds to 0 m/s
  // traveling 10*5=50 meters
  // In total 100+40+50=190 meters -> length
  // In time 10+2+5=17 seconds
  const auto tr3 = instance.add_train("Train3", 190, 50, 2, 4, {0, 60}, 20, v0,
                                      {300, 360}, 0, v1);

  // Train 4 exits with speed 10
  // It arrives at exit node with speed 5
  // Then accelerate at rate 1 for 5 seconds to 10 m/s
  // traveling 7.5*5=37.5 meters
  // Then remain constant for 2.25 seconds
  // traveling 2.25*10=22.5 meters
  // Total distance 37.5+22.5=60 meters
  // Total time 5+2.25=7.25 seconds
  const auto tr4 = instance.add_train("Train4", 60, 50, 1, 2, {0, 60}, 20, v0,
                                      {300, 360}, 10, v1);

  // Leaving times of Train1
  EXPECT_EQ(instance.get_approximate_leaving_time(tr1), 7);
  EXPECT_APPROX_EQ(instance.get_maximal_leaving_time(tr1, 10),
                   cda_rail::max_travel_time_no_stopping(10, 8, cda_rail::V_MIN,
                                                         1.5, 2, 56));

  // Leaving times of Train2
  EXPECT_APPROX_EQ(
      instance.get_maximal_leaving_time(tr2, 8),
      cda_rail::max_travel_time_no_stopping(8, 0, cda_rail::V_MIN, 1, 2, 64));
  EXPECT_APPROX_EQ(instance.get_minimal_leaving_time(tr2, 0), 14);

  // Leaving times for Train3
  EXPECT_APPROX_EQ(
      instance.get_maximal_leaving_time(tr3, 10),
      cda_rail::max_travel_time_no_stopping(10, 0, cda_rail::V_MIN, 2, 4, 190));
  EXPECT_APPROX_EQ(instance.get_minimal_leaving_time(tr3, 0), 17);

  // Leaving times for Train4
  EXPECT_APPROX_EQ(
      instance.get_maximal_leaving_time(tr4, 5),
      cda_rail::max_travel_time_no_stopping(5, 10, cda_rail::V_MIN, 1, 2, 60));
  EXPECT_APPROX_EQ(instance.get_minimal_leaving_time(tr4, 5), 7.25);
}

TEST(GeneralPerformanceOptimizationInstances, RASPaths) {
  const std::vector<std::string> paths{"toy", "practical"};

  for (const auto& p : paths) {
    const std::string instance_path =
        "./example-networks-gen-po-ras/" + p + "/";
    const auto instance =
        cda_rail::instances::GeneralPerformanceOptimizationInstance(
            instance_path);

    for (size_t tr = 0; tr < instance.get_train_list().size(); ++tr) {
      double min_time = 0;

      const auto tr_schedule = instance.get_schedule(tr);
      const auto entry       = tr_schedule.get_entry();
      const auto exit        = tr_schedule.get_exit();
      const auto entry_edges = instance.const_n().out_edges(entry);
      const auto tr_obj      = instance.get_train_list().get_train(tr);
      const auto entry_obj   = instance.const_n().get_vertex(entry);
      const auto exit_obj    = instance.const_n().get_vertex(exit);
      EXPECT_EQ(instance.const_n().neighbors(entry).size(), 1)
          << "Instance " << p << ": Train " << tr_obj.name << ", entry vertex "
          << entry_obj.name << " does not have exactly one neighbor";
      EXPECT_EQ(instance.const_n().neighbors(exit).size(), 1)
          << "Instance " << p << ": Train " << tr_obj.name << ", exit vertex "
          << exit_obj.name << " does not have exactly one neighbor";
      EXPECT_EQ(entry_edges.size(), 1)
          << "Instance " << p << ": Train " << tr_obj.name
          << " does not have exactly one entry edge at entry vertex "
          << entry_obj.name;
      const auto entry_edge = entry_edges[0];
      const auto p_len      = instance.const_n().shortest_path(
          entry_edge, exit, false, true, true, tr_obj.max_speed);
      EXPECT_TRUE(p_len.has_value())
          << "Instance " << p << ": No path for train " << tr_obj.name
          << " from " << entry_obj.name << " to " << exit_obj.name;
      min_time += p_len.value_or(0);

      std::vector<size_t> last_edges   = {entry_edge};
      std::string         last_station = "Entry " + entry_obj.name;
      for (const auto& stop : tr_schedule.get_stops()) {
        const auto station_name = stop.get_station_name();
        const auto station =
            instance.get_station_list().get_station(station_name);
        const auto p_station_len =
            instance.const_n().shortest_path_between_sets(
                last_edges, station.tracks, true, true, true, tr_obj.max_speed);
        EXPECT_TRUE(p_station_len.has_value())
            << "Instance " << p << ": No path for train " << tr_obj.name
            << " from " << last_station << " to " << station_name;
        last_edges   = station.tracks;
        last_station = station_name;
        min_time += p_station_len.value_or(0);
        min_time += stop.get_min_stopping_time();
      }
      const auto p_exit_len = instance.const_n().shortest_path_between_sets(
          last_edges, {exit}, false, true, true, tr_obj.max_speed);
      EXPECT_TRUE(p_exit_len.has_value())
          << "Instance " << p << ": No path for train " << tr_obj.name
          << " from " << last_station << " to exit " << exit_obj.name;
      min_time += p_exit_len.value_or(0);

      EXPECT_LE(min_time + 1 * 60 * 60, tr_schedule.get_t_n_range().second -
                                            tr_schedule.get_t_0_range().second)
          << "Instance " << p << ": Train " << tr_obj.name
          << " cannot reach exit in scheduled time with 1h buffer"
          << " (min time: " << min_time << ", scheduled time: "
          << tr_schedule.get_t_n_range().second -
                 tr_schedule.get_t_0_range().second
          << ")";
    }
  }
}

TEST(GeneralPerformanceOptimizationInstances, Overlaps) {
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance;

  // Create simple network with parallel edges
  const auto v0 = instance.n().add_vertex("v0", cda_rail::VertexType::TTD);
  const auto v1 = instance.n().add_vertex("v1", cda_rail::VertexType::TTD);
  const auto v2 = instance.n().add_vertex("v2", cda_rail::VertexType::NoBorder);
  const auto v3 = instance.n().add_vertex("v3", cda_rail::VertexType::TTD);
  const auto v4 = instance.n().add_vertex("v4", cda_rail::VertexType::TTD);

  const auto e01 = instance.n().add_edge(v0, v1, 100, 20, true);
  const auto e12 = instance.n().add_edge(v1, v2, 10, 20, false);
  const auto e23 = instance.n().add_edge(v2, v3, 20, 20, false);
  const auto e24 = instance.n().add_edge(v2, v4, 30, 20, false);
  const auto e42 = instance.n().add_edge(v4, v2, 30, 20, false);
  const auto e21 = instance.n().add_edge(v2, v1, 10, 20, false);
  const auto e10 = instance.n().add_edge(v1, v0, 100, 20, true);

  instance.n().add_successor(e01, e12);
  instance.n().add_successor(e12, e23);
  instance.n().add_successor(e12, e24);
  instance.n().add_successor(e42, e21);
  instance.n().add_successor(e21, e10);

  const auto tr1 = instance.add_train("tr1", 50, 20, 1, 2, {0, 60}, 20, v0,
                                      {300, 360}, 0, v3);
  const auto tr2 = instance.add_train("tr2", 50, 20, 1, 2, {0, 60}, 20, v0,
                                      {300, 360}, 0, v4);
  const auto tr3 = instance.add_train("tr3", 50, 20, 1, 2, {0, 60}, 20, v4,
                                      {300, 360}, 0, v0);

  instance.add_empty_route("tr1");
  instance.add_empty_route("tr2");
  instance.add_empty_route("tr3");

  instance.push_back_edge_to_route("tr1", e01);
  instance.push_back_edge_to_route("tr1", e12);
  instance.push_back_edge_to_route("tr1", e23);
  instance.push_back_edge_to_route("tr2", e01);
  instance.push_back_edge_to_route("tr2", e12);
  instance.push_back_edge_to_route("tr2", e24);
  instance.push_back_edge_to_route("tr3", e42);
  instance.push_back_edge_to_route("tr3", e21);
  instance.push_back_edge_to_route("tr3", e10);

  const auto tr12_parallel = instance.get_parallel_overlaps("tr1", "tr2");
  EXPECT_EQ(tr12_parallel.size(), 1);
  const auto& [tr12_parallel_1, tr12_parallel_2, tr12_parallel_e] =
      tr12_parallel.at(0);
  EXPECT_EQ(tr12_parallel_1.first, 0);
  EXPECT_EQ(tr12_parallel_1.second, 110);
  EXPECT_EQ(tr12_parallel_2.first, 0);
  EXPECT_EQ(tr12_parallel_2.second, 110);
  EXPECT_EQ(tr12_parallel_e, std::unordered_set<size_t>({e01, e12}));

  const auto tr13_parallel = instance.get_parallel_overlaps("tr1", "tr3");
  EXPECT_TRUE(tr13_parallel.empty());

  const auto tr12_ttd = instance.get_ttd_overlaps("tr1", "tr2");
  EXPECT_EQ(tr12_ttd.size(), 1);
  const auto& [tr12_ttd_1, tr12_ttd_2, tr12_ttd_e] = tr12_ttd.at(0);
  EXPECT_EQ(tr12_ttd_1.first, 100);
  EXPECT_EQ(tr12_ttd_1.second, 130);
  EXPECT_EQ(tr12_ttd_2.first, 100);
  EXPECT_EQ(tr12_ttd_2.second, 140);
  EXPECT_EQ(tr12_ttd_e, std::unordered_set<size_t>({e12, e23, e24}));

  const auto tr13_ttd = instance.get_ttd_overlaps("tr1", "tr3");
  EXPECT_EQ(tr13_ttd.size(), 1);
  const auto& [tr13_ttd_1, tr13_ttd_2, tr13_ttd_e] = tr13_ttd.at(0);
  EXPECT_EQ(tr13_ttd_1.first, 100);
  EXPECT_EQ(tr13_ttd_1.second, 130);
  EXPECT_EQ(tr13_ttd_2.first, 0);
  EXPECT_EQ(tr13_ttd_2.second, 40);
  EXPECT_EQ(tr13_ttd_e, std::unordered_set<size_t>({e12, e23, e24}));

  const auto tr12_reverse = instance.get_reverse_overlaps("tr1", "tr2");
  EXPECT_TRUE(tr12_reverse.empty());

  const auto tr13_reverse = instance.get_reverse_overlaps("tr1", "tr3");
  EXPECT_EQ(tr13_reverse.size(), 1);
  const auto& [tr13_reverse_1, tr13_reverse_2, tr13_reverse_e] =
      tr13_reverse.at(0);
  EXPECT_EQ(tr13_reverse_1.first, 0);
  EXPECT_EQ(tr13_reverse_1.second, 110);
  EXPECT_EQ(tr13_reverse_2.first, 30);
  EXPECT_EQ(tr13_reverse_2.second, 140);
  EXPECT_EQ(tr13_reverse_e, std::unordered_set<size_t>({e01, e12}));

  const auto tr12_crossing = instance.get_crossing_overlaps("tr1", "tr2");
  EXPECT_EQ(tr12_crossing.size(), 1);
  // equal to ttd12 conflict
  const auto& [tr12_crossing_1, tr12_crossing_2, tr12_crossing_e] =
      tr12_crossing.at(0);
  EXPECT_EQ(tr12_crossing_1.first, 100);
  EXPECT_EQ(tr12_crossing_1.second, 130);
  EXPECT_EQ(tr12_crossing_2.first, 100);
  EXPECT_EQ(tr12_crossing_2.second, 140);
  EXPECT_EQ(tr12_crossing_e, std::unordered_set<size_t>({e12, e23, e24}));

  const auto tr13_crossing = instance.get_crossing_overlaps("tr1", "tr3");
  EXPECT_EQ(tr13_crossing.size(), 1);
  // equal to ttd13 and reverse13 merged
  const auto& [tr13_crossing_1, tr13_crossing_2, tr13_crossing_e] =
      tr13_crossing.at(0);
  EXPECT_EQ(tr13_crossing_1.first, 0);
  EXPECT_EQ(tr13_crossing_1.second, 130);
  EXPECT_EQ(tr13_crossing_2.first, 0);
  EXPECT_EQ(tr13_crossing_2.second, 140);
  EXPECT_EQ(tr13_crossing_e,
            std::unordered_set<size_t>({e01, e12, e12, e23, e24}));
}

// NOLINTEND (clang-analyzer-deadcode.DeadStores)
