#include "CustomExceptions.hpp"
#include "datastructure/GeneralTimetable.hpp"

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

  EXPECT_THROW(
      timetable.add_stop("Train1", "Station1", {180, 240}, {240, 300}, 60),
      exceptions::ConsistencyException);

  timetable.add_stop("Train2", "Station1", {400, 460}, {460, 520}, 60);

  EXPECT_FALSE(timetable.check_consistency(network));

  timetable.remove_stop("Train2", "Station1");

  EXPECT_TRUE(timetable.check_consistency(network));
}
