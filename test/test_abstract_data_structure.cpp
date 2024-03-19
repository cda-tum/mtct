#include "CustomExceptions.hpp"
#include "datastructure/GeneralTimetable.hpp"
#include "datastructure/Route.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"

#include "gtest/gtest.h"
#include <utility>

using namespace cda_rail;

TEST(GeneralAbstractDataStructure, GeneralScheduledStopExceptions) {
  EXPECT_THROW(cda_rail::GeneralScheduledStop({10, 9}, {12, 15}, 1, "Test1"),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(cda_rail::GeneralScheduledStop({0, 5}, {12, 9}, 1, "Test2"),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(cda_rail::GeneralScheduledStop({0, 5}, {12, 15}, 0, "Test3"),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(cda_rail::GeneralScheduledStop({-1, 5}, {12, 15}, 1, ""),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(cda_rail::GeneralScheduledStop({0, 5}, {-1, 15}, 1, "Test4"),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(cda_rail::GeneralScheduledStop({10, 12}, {0, 5}, 1, "Test5"),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(cda_rail::GeneralScheduledStop({0, 1}, {2, 3}, 5, "Test6"),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_NO_THROW(cda_rail::GeneralScheduledStop({0, 1}, {2, 3}, 1, "Test7"));
  EXPECT_NO_THROW(cda_rail::GeneralScheduledStop({0, 2}, {1, 3}, 1, "Test8"));
}

TEST(GeneralAbstractDataStructure, GeneralScheduledStopConstructor) {
  cda_rail::GeneralScheduledStop stop({0, 2}, {3, 4}, 2, "Test");
  EXPECT_EQ(stop.get_begin_range(), (std::pair<int, int>(0, 2)));
  EXPECT_EQ(stop.get_end_range(), (std::pair<int, int>(3, 4)));
  EXPECT_EQ(stop.get_min_stopping_time(), 2);
  EXPECT_EQ(stop.get_station_name(), "Test");
}

TEST(GeneralAbstractDataStructure, GeneralSchedulesStopForcedStoppingInterval) {
  cda_rail::GeneralScheduledStop stop1({0, 2}, {3, 4}, 1, "Test");
  EXPECT_EQ(stop1.get_forced_stopping_interval(), (std::pair<int, int>(2, 3)));

  cda_rail::GeneralScheduledStop stop2({0, 2}, {3, 4}, 2, "Test");
  EXPECT_EQ(stop2.get_forced_stopping_interval(), (std::pair<int, int>(2, 3)));

  cda_rail::GeneralScheduledStop stop3({0, 2}, {3, 4}, 3, "Test");
  EXPECT_EQ(stop3.get_forced_stopping_interval(), (std::pair<int, int>(1, 3)));

  cda_rail::GeneralScheduledStop stop4({0, 2}, {3, 4}, 4, "Test");
  EXPECT_EQ(stop4.get_forced_stopping_interval(), (std::pair<int, int>(0, 4)));

  cda_rail::GeneralScheduledStop stop5({0, 5}, {0, 5}, 1, "Test");
  EXPECT_EQ(stop5.get_forced_stopping_interval(), (std::pair<int, int>(4, 1)));
}

TEST(GeneralAbstractDataStructure, GeneralScheduledStopConflicts) {
  cda_rail::GeneralScheduledStop stop1({0, 2}, {3, 4}, 1, "Test");
  cda_rail::GeneralScheduledStop stop2({5, 6}, {7, 8}, 1, "Test");

  EXPECT_TRUE(stop1.conflicts(stop2));
  EXPECT_TRUE(stop2.conflicts(stop1));

  cda_rail::GeneralScheduledStop stop3({4, 5}, {10, 11}, 1, "Test1");
  cda_rail::GeneralScheduledStop stop4({0, 1}, {7, 8}, 1, "Test2");
  cda_rail::GeneralScheduledStop stop5({0, 1}, {2, 3}, 1, "Test3");
  cda_rail::GeneralScheduledStop stop6({0, 5}, {0, 5}, 1, "Test4");

  EXPECT_TRUE(stop3.conflicts(stop4));
  EXPECT_TRUE(stop4.conflicts(stop3));

  EXPECT_FALSE(stop3.conflicts(stop5));
  EXPECT_FALSE(stop5.conflicts(stop3));

  EXPECT_FALSE(stop3.conflicts(stop6));
  EXPECT_FALSE(stop6.conflicts(stop3));

  EXPECT_TRUE(stop4.conflicts(stop5));
  EXPECT_TRUE(stop5.conflicts(stop4));

  EXPECT_FALSE(stop4.conflicts(stop6));
  EXPECT_FALSE(stop6.conflicts(stop4));

  EXPECT_FALSE(stop5.conflicts(stop6));
  EXPECT_FALSE(stop6.conflicts(stop5));
}

TEST(GeneralAbstractDataStructure, GeneralTimetable) {
  Network network("./example-networks/SimpleStation/network/");

  GeneralTimetable<GeneralSchedule<GeneralScheduledStop>> timetable;

  const auto l0 = network.get_vertex_index("l0");
  const auto r0 = network.get_vertex_index("r0");

  const auto tr1 = timetable.add_train("Train1", 100, 10, 1, 1, true, {0, 60},
                                       0, "l0", {360, 420}, 0, "r0", network);
  const auto tr2 = timetable.add_train("Train2", 100, 10, 1, 1, false, {0, 60},
                                       10, l0, {400, 460}, 5, r0, network);

  EXPECT_EQ(timetable.get_train_list().get_train_index("Train1"), tr1);
  EXPECT_EQ(timetable.get_train_list().get_train_index("Train2"), tr2);
  EXPECT_EQ(timetable.get_train_list().get_train("Train1").name, "Train1");
  EXPECT_EQ(timetable.get_train_list().get_train("Train1").length, 100);
  EXPECT_EQ(timetable.get_train_list().get_train("Train1").max_speed, 10);
  EXPECT_EQ(timetable.get_train_list().get_train("Train1").acceleration, 1);
  EXPECT_EQ(timetable.get_train_list().get_train("Train1").deceleration, 1);
  EXPECT_EQ(timetable.get_train_list().get_train("Train1").tim, true);

  EXPECT_EQ(timetable.get_train_list().get_train("Train2").name, "Train2");
  EXPECT_EQ(timetable.get_train_list().get_train("Train2").length, 100);
  EXPECT_EQ(timetable.get_train_list().get_train("Train2").max_speed, 10);
  EXPECT_EQ(timetable.get_train_list().get_train("Train2").acceleration, 1);
  EXPECT_EQ(timetable.get_train_list().get_train("Train2").deceleration, 1);
  EXPECT_EQ(timetable.get_train_list().get_train("Train2").tim, false);

  EXPECT_EQ(timetable.get_schedule(tr1).get_t_0_range(),
            (std::pair<int, int>(0, 60)));
  EXPECT_EQ(timetable.get_schedule(tr1).get_t_n_range(),
            (std::pair<int, int>(360, 420)));
  EXPECT_EQ(timetable.get_schedule(tr1).get_v_0(), 0);
  EXPECT_EQ(timetable.get_schedule(tr1).get_v_n(), 0);

  EXPECT_EQ(timetable.get_schedule("Train2").get_t_0_range(),
            (std::pair<int, int>(0, 60)));
  EXPECT_EQ(timetable.get_schedule("Train2").get_t_n_range(),
            (std::pair<int, int>(400, 460)));
  EXPECT_EQ(timetable.get_schedule("Train2").get_v_0(), 10);
  EXPECT_EQ(timetable.get_schedule("Train2").get_v_n(), 5);

  EXPECT_TRUE(timetable.check_consistency(network));

  timetable.add_station("Station1");
  timetable.add_track_to_station("Station1", "g00", "g01", network);
  timetable.add_track_to_station("Station1", "g01", "g00", network);
  timetable.add_track_to_station("Station1", "g10", "g11", network);
  timetable.add_track_to_station("Station1", "g11", "g10", network);

  EXPECT_TRUE(timetable.check_consistency(network));

  timetable.add_stop("Train1", "Station1", {60, 120}, {120, 180}, 60);

  EXPECT_TRUE(timetable.check_consistency(network));
  EXPECT_EQ(
      timetable.get_schedule("Train1").get_stops().at(0).get_station_name(),
      "Station1");
  EXPECT_EQ(
      timetable.get_schedule("Train1").get_stops().at(0).get_begin_range(),
      (std::pair<int, int>(60, 120)));
  EXPECT_EQ(timetable.get_schedule("Train1").get_stops().at(0).get_end_range(),
            (std::pair<int, int>(120, 180)));
  EXPECT_EQ(timetable.get_schedule("Train1")
                .get_stops()
                .at(0)
                .get_min_stopping_time(),
            60);

  EXPECT_THROW(
      timetable.add_stop("Train1", "Station1", {180, 240}, {240, 300}, 60),
      exceptions::ConsistencyException);

  timetable.add_stop("Train2", "Station1", {400, 460}, {460, 520}, 60);

  EXPECT_FALSE(timetable.check_consistency(network));

  timetable.remove_stop("Train2", "Station1");

  EXPECT_TRUE(timetable.check_consistency(network));
}

TEST(GeneralAbstractDataStructure, GeneralTimetableExportImport) {
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

  // Write timetable to directory
  timetable.export_timetable("./tmp/test-general-timetable/", network);

  GeneralTimetable<GeneralSchedule<GeneralScheduledStop>> timetable_read(
      "./tmp/test-general-timetable/", network);

  std::filesystem::remove_all("./tmp");

  const auto tr1 = timetable_read.get_train_list().get_train_index("Train1");
  const auto tr2 = timetable_read.get_train_list().get_train_index("Train2");

  EXPECT_TRUE(timetable_read.check_consistency(network));

  EXPECT_EQ(timetable_read.get_train_list().get_train_index("Train1"), tr1);
  EXPECT_EQ(timetable_read.get_train_list().get_train_index("Train2"), tr2);
  EXPECT_EQ(timetable_read.get_train_list().get_train("Train1").name, "Train1");
  EXPECT_EQ(timetable_read.get_train_list().get_train("Train1").length, 100);
  EXPECT_EQ(timetable_read.get_train_list().get_train("Train1").max_speed, 10);
  EXPECT_EQ(timetable_read.get_train_list().get_train("Train1").acceleration,
            1);
  EXPECT_EQ(timetable_read.get_train_list().get_train("Train1").deceleration,
            1);
  EXPECT_EQ(timetable_read.get_train_list().get_train("Train1").tim, true);

  EXPECT_EQ(timetable_read.get_train_list().get_train("Train2").name, "Train2");
  EXPECT_EQ(timetable_read.get_train_list().get_train("Train2").length, 100);
  EXPECT_EQ(timetable_read.get_train_list().get_train("Train2").max_speed, 10);
  EXPECT_EQ(timetable_read.get_train_list().get_train("Train2").acceleration,
            1);
  EXPECT_EQ(timetable_read.get_train_list().get_train("Train2").deceleration,
            1);
  EXPECT_EQ(timetable_read.get_train_list().get_train("Train2").tim, false);

  EXPECT_EQ(timetable_read.get_schedule(tr1).get_t_0_range(),
            (std::pair<int, int>(0, 60)));
  EXPECT_EQ(timetable_read.get_schedule(tr1).get_t_n_range(),
            (std::pair<int, int>(360, 420)));
  EXPECT_EQ(timetable_read.get_schedule(tr1).get_v_0(), 0);
  EXPECT_EQ(timetable_read.get_schedule(tr1).get_v_n(), 0);

  EXPECT_EQ(timetable_read.get_schedule("Train2").get_t_0_range(),
            (std::pair<int, int>(0, 60)));
  EXPECT_EQ(timetable_read.get_schedule("Train2").get_t_n_range(),
            (std::pair<int, int>(400, 460)));
  EXPECT_EQ(timetable_read.get_schedule("Train2").get_v_0(), 10);
  EXPECT_EQ(timetable_read.get_schedule("Train2").get_v_n(), 5);

  EXPECT_EQ(timetable_read.get_schedule("Train1")
                .get_stops()
                .at(0)
                .get_station_name(),
            "Station1");
  EXPECT_EQ(
      timetable_read.get_schedule("Train1").get_stops().at(0).get_begin_range(),
      (std::pair<int, int>(60, 120)));
  EXPECT_EQ(
      timetable_read.get_schedule("Train1").get_stops().at(0).get_end_range(),
      (std::pair<int, int>(120, 180)));
  EXPECT_EQ(timetable_read.get_schedule("Train1")
                .get_stops()
                .at(0)
                .get_min_stopping_time(),
            60);
}

TEST(GeneralAbstractDataStructure, ParseSchedule) {
  cda_rail::ScheduledStop stop1(0, 4, "Test");
  cda_rail::ScheduledStop stop2(6, 8, "Test");

  cda_rail::Schedule schedule(0, 10, 1, 20, 5, 2, {stop1, stop2});

  const auto general_schedule = schedule.parse_to_general_schedule();
  EXPECT_EQ(general_schedule.get_t_0_range(), (std::pair<int, int>(0, 0)));
  EXPECT_EQ(general_schedule.get_t_n_range(), (std::pair<int, int>(20, 20)));
  EXPECT_EQ(general_schedule.get_v_0(), 10);
  EXPECT_EQ(general_schedule.get_v_n(), 5);
  EXPECT_EQ(general_schedule.get_entry(), 1);
  EXPECT_EQ(general_schedule.get_exit(), 2);
  EXPECT_EQ(general_schedule.get_stops().size(), 2);
  EXPECT_EQ(general_schedule.get_stops().at(0).get_begin_range(),
            (std::pair<int, int>(0, 0)));
  EXPECT_EQ(general_schedule.get_stops().at(0).get_end_range(),
            (std::pair<int, int>(4, 4)));
  EXPECT_EQ(general_schedule.get_stops().at(0).get_min_stopping_time(), 4);
  EXPECT_EQ(general_schedule.get_stops().at(0).get_station_name(), "Test");
  EXPECT_EQ(typeid(general_schedule.get_stops().at(0)),
            typeid(cda_rail::GeneralScheduledStop));
  EXPECT_EQ(general_schedule.get_stops().at(1).get_begin_range(),
            (std::pair<int, int>(6, 6)));
  EXPECT_EQ(general_schedule.get_stops().at(1).get_end_range(),
            (std::pair<int, int>(8, 8)));
  EXPECT_EQ(general_schedule.get_stops().at(1).get_min_stopping_time(), 2);
  EXPECT_EQ(general_schedule.get_stops().at(1).get_station_name(), "Test");
  EXPECT_EQ(typeid(general_schedule.get_stops().at(1)),
            typeid(cda_rail::GeneralScheduledStop));
}

TEST(GeneralAbstractDataStructure, ParseTimetable) {
  auto network = cda_rail::Network::import_network(
      "./example-networks/SimpleStation/network/");
  cda_rail::Timetable timetable;

  timetable.add_station("Station1");
  timetable.add_track_to_station("Station1", "g00", "g01", network);

  timetable.add_station("Station2");
  timetable.add_track_to_station("Station2", "g10", "g11", network);

  const auto l0 = network.get_vertex_index("l0");
  const auto r0 = network.get_vertex_index("r0");

  const auto tr1 = timetable.add_train("tr1", 100, 83.33, 2, 1, 0, 0, l0, 300,
                                       20, r0, network);
  timetable.add_stop(tr1, "Station1", 0, 60);
  timetable.add_stop(tr1, "Station2", 120, 180);

  const auto tr2 = timetable.add_train("tr2", 100, 83.33, 2, 1, 0, 0, l0, 300,
                                       20, r0, network);
  timetable.add_stop(tr2, "Station1", 100, 160);

  const auto general_timetable = timetable.parse_to_general_timetable();
  EXPECT_EQ(general_timetable.get_station_list().get_station_names().size(), 2);
  EXPECT_EQ(general_timetable.get_station_list().get_station_names().at(0),
            "Station1");
  EXPECT_EQ(general_timetable.get_station_list().get_station_names().at(1),
            "Station2");
  EXPECT_EQ(general_timetable.get_train_list().size(), 2);
  EXPECT_EQ(general_timetable.get_train_list().get_train_index("tr1"), tr1);
  EXPECT_EQ(general_timetable.get_train_list().get_train_index("tr2"), tr2);
  EXPECT_EQ(general_timetable.get_train_list().get_train("tr1").name, "tr1");
  EXPECT_EQ(general_timetable.get_train_list().get_train("tr1").length, 100);
  EXPECT_EQ(general_timetable.get_train_list().get_train("tr1").max_speed,
            83.33);
  EXPECT_EQ(general_timetable.get_train_list().get_train("tr1").acceleration,
            2);
  EXPECT_EQ(general_timetable.get_train_list().get_train("tr1").deceleration,
            1);
  EXPECT_EQ(general_timetable.get_train_list().get_train("tr1").tim, true);
  EXPECT_EQ(general_timetable.get_train_list().get_train("tr2").name, "tr2");
  EXPECT_EQ(general_timetable.get_train_list().get_train("tr2").length, 100);
  EXPECT_EQ(general_timetable.get_train_list().get_train("tr2").max_speed,
            83.33);
  EXPECT_EQ(general_timetable.get_train_list().get_train("tr2").acceleration,
            2);
  EXPECT_EQ(general_timetable.get_train_list().get_train("tr2").deceleration,
            1);
  EXPECT_EQ(general_timetable.get_train_list().get_train("tr2").tim, true);
  EXPECT_EQ(general_timetable.get_schedule(tr1).get_t_0_range(),
            (std::pair<int, int>(0, 0)));
  EXPECT_EQ(general_timetable.get_schedule(tr1).get_t_n_range(),
            (std::pair<int, int>(300, 300)));
  EXPECT_EQ(general_timetable.get_schedule(tr1).get_v_0(), 0);
  EXPECT_EQ(general_timetable.get_schedule(tr1).get_v_n(), 20);
  EXPECT_EQ(general_timetable.get_schedule(tr1).get_entry(), l0);
  EXPECT_EQ(general_timetable.get_schedule(tr1).get_exit(), r0);
  EXPECT_EQ(general_timetable.get_schedule(tr1).get_stops().size(), 2);
  EXPECT_EQ(
      general_timetable.get_schedule(tr1).get_stops().at(0).get_station_name(),
      "Station1");
  EXPECT_EQ(
      general_timetable.get_schedule(tr1).get_stops().at(0).get_begin_range(),
      (std::pair<int, int>(0, 0)));
  EXPECT_EQ(
      general_timetable.get_schedule(tr1).get_stops().at(0).get_end_range(),
      (std::pair<int, int>(60, 60)));
  EXPECT_EQ(general_timetable.get_schedule(tr1)
                .get_stops()
                .at(0)
                .get_min_stopping_time(),
            60);
  EXPECT_EQ(
      general_timetable.get_schedule(tr1).get_stops().at(1).get_station_name(),
      "Station2");
  EXPECT_EQ(
      general_timetable.get_schedule(tr1).get_stops().at(1).get_begin_range(),
      (std::pair<int, int>(120, 120)));
  EXPECT_EQ(
      general_timetable.get_schedule(tr1).get_stops().at(1).get_end_range(),
      (std::pair<int, int>(180, 180)));
  EXPECT_EQ(general_timetable.get_schedule(tr1)
                .get_stops()
                .at(1)
                .get_min_stopping_time(),
            60);
  EXPECT_EQ(general_timetable.get_schedule(tr2).get_t_0_range(),
            (std::pair<int, int>(0, 0)));
  EXPECT_EQ(general_timetable.get_schedule(tr2).get_t_n_range(),
            (std::pair<int, int>(300, 300)));
  EXPECT_EQ(general_timetable.get_schedule(tr2).get_v_0(), 0);
  EXPECT_EQ(general_timetable.get_schedule(tr2).get_v_n(), 20);
  EXPECT_EQ(general_timetable.get_schedule(tr2).get_entry(), l0);
  EXPECT_EQ(general_timetable.get_schedule(tr2).get_exit(), r0);
  EXPECT_EQ(general_timetable.get_schedule(tr2).get_stops().size(), 1);
  EXPECT_EQ(
      general_timetable.get_schedule(tr2).get_stops().at(0).get_station_name(),
      "Station1");
  EXPECT_EQ(
      general_timetable.get_schedule(tr2).get_stops().at(0).get_begin_range(),
      (std::pair<int, int>(100, 100)));
  EXPECT_EQ(
      general_timetable.get_schedule(tr2).get_stops().at(0).get_end_range(),
      (std::pair<int, int>(160, 160)));
  EXPECT_EQ(general_timetable.get_schedule(tr2)
                .get_stops()
                .at(0)
                .get_min_stopping_time(),
            60);

  EXPECT_EQ(typeid(general_timetable),
            typeid(cda_rail::GeneralTimetable<
                   cda_rail::GeneralSchedule<cda_rail::GeneralScheduledStop>>));
}
