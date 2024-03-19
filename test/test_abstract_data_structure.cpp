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

struct EdgeTarget {
  std::string source;
  std::string target;
  double      length;
  double      max_speed;
  bool        breakable;
  double      min_block_length;
  double      min_stop_block_length = 100;
};

void check_instance_import_general_cast(
    const cda_rail::instances::GeneralPerformanceOptimizationInstance&
        instance) {
  const auto& network = instance.const_n();

  // Check vertices properties
  std::vector<std::string> vertex_names = {
      "l0", "l1", "l2", "l3", "r0", "r1", "r2", "g00", "g01", "g10", "g11"};
  std::vector<cda_rail::VertexType> type = {
      cda_rail::VertexType::TTD,      cda_rail::VertexType::TTD,
      cda_rail::VertexType::TTD,      cda_rail::VertexType::NoBorder,
      cda_rail::VertexType::TTD,      cda_rail::VertexType::TTD,
      cda_rail::VertexType::NoBorder, cda_rail::VertexType::TTD,
      cda_rail::VertexType::TTD,      cda_rail::VertexType::TTD,
      cda_rail::VertexType::TTD};

  EXPECT_EQ(network.number_of_vertices(), vertex_names.size());

  for (size_t i = 0; i < vertex_names.size(); i++) {
    std::string      v_name = vertex_names[i];
    cda_rail::Vertex v      = network.get_vertex(v_name);
    EXPECT_EQ(v.name, v_name);
    EXPECT_EQ(v.type, type[i]);
  }

  // Check edges properties
  std::vector<EdgeTarget> edge_targets;
  edge_targets.push_back({"l0", "l1", 500, 27.77777777777778, true, 10});
  edge_targets.push_back({"l1", "l2", 500, 27.77777777777778, true, 10});
  edge_targets.push_back({"l2", "l3", 5, 27.77777777777778, false, 0});
  edge_targets.push_back({"l3", "g00", 5, 27.77777777777778, false, 0});
  edge_targets.push_back({"l3", "g10", 5, 27.77777777777778, false, 0});
  edge_targets.push_back({"g00", "g01", 300, 27.77777777777778, true, 10, 150});
  edge_targets.push_back({"g10", "g11", 300, 27.77777777777778, true, 10, 150});
  edge_targets.push_back({"g01", "r2", 5, 27.77777777777778, false, 0});
  edge_targets.push_back({"g11", "r2", 5, 27.77777777777778, false, 0});
  edge_targets.push_back({"r2", "r1", 5, 27.77777777777778, false, 0});
  edge_targets.push_back({"r1", "r0", 500, 27.77777777777778, true, 10});
  edge_targets.push_back({"r0", "r1", 500, 27.77777777777778, true, 10});
  edge_targets.push_back({"r1", "r2", 5, 27.77777777777778, false, 0});
  edge_targets.push_back({"r2", "g01", 5, 27.77777777777778, false, 0});
  edge_targets.push_back({"r2", "g11", 5, 27.77777777777778, false, 0});
  edge_targets.push_back({"g01", "g00", 300, 27.77777777777778, true, 10, 150});
  edge_targets.push_back({"g11", "g10", 300, 27.77777777777778, true, 10, 150});
  edge_targets.push_back({"g00", "l3", 5, 27.77777777777778, false, 0});
  edge_targets.push_back({"g10", "l3", 5, 27.77777777777778, false, 0});
  edge_targets.push_back({"l3", "l2", 5, 27.77777777777778, false, 0});
  edge_targets.push_back({"l2", "l1", 500, 27.77777777777778, true, 10});
  edge_targets.push_back({"l1", "l0", 500, 27.77777777777778, true, 10});

  EXPECT_EQ(network.number_of_edges(), edge_targets.size());
  for (const auto& edge : edge_targets) {
    cda_rail::Edge e = network.get_edge(edge.source, edge.target);
    EXPECT_EQ(network.get_vertex(e.source).name, edge.source);
    EXPECT_EQ(network.get_vertex(e.target).name, edge.target);
    EXPECT_EQ(e.length, edge.length);
    EXPECT_EQ(e.max_speed, edge.max_speed);
    EXPECT_EQ(e.breakable, edge.breakable);
    EXPECT_EQ(e.min_block_length, edge.min_block_length);
    EXPECT_EQ(e.min_stop_block_length, edge.min_stop_block_length);
  }

  // Check successors
  std::vector<size_t> successors_target;

  // l0,l1
  successors_target.clear();
  successors_target.emplace_back(network.get_edge_index("l1", "l2"));
  EXPECT_EQ(network.get_successors("l0", "l1"), successors_target);

  // l1,l2
  successors_target.clear();
  successors_target.emplace_back(network.get_edge_index("l2", "l3"));
  EXPECT_EQ(network.get_successors("l1", "l2"), successors_target);

  // l2,l3
  successors_target.clear();
  successors_target.emplace_back(network.get_edge_index("l3", "g00"));
  successors_target.emplace_back(network.get_edge_index("l3", "g10"));
  auto successors_l2_l3 = network.get_successors("l2", "l3");
  std::sort(successors_target.begin(), successors_target.end());
  std::sort(successors_l2_l3.begin(), successors_l2_l3.end());
  EXPECT_EQ(successors_l2_l3, successors_target);

  // l3,g00
  successors_target.clear();
  successors_target.emplace_back(network.get_edge_index("g00", "g01"));
  EXPECT_EQ(network.get_successors("l3", "g00"), successors_target);

  // l3,g10
  successors_target.clear();
  successors_target.emplace_back(network.get_edge_index("g10", "g11"));
  EXPECT_EQ(network.get_successors("l3", "g10"), successors_target);

  // g00,g01
  successors_target.clear();
  successors_target.emplace_back(network.get_edge_index("g01", "r2"));
  EXPECT_EQ(network.get_successors("g00", "g01"), successors_target);

  // g10,g11
  successors_target.clear();
  successors_target.emplace_back(network.get_edge_index("g11", "r2"));
  EXPECT_EQ(network.get_successors("g10", "g11"), successors_target);

  // g01,r2
  successors_target.clear();
  successors_target.emplace_back(network.get_edge_index("r2", "r1"));
  EXPECT_EQ(network.get_successors("g01", "r2"), successors_target);

  // g11,r2
  successors_target.clear();
  successors_target.emplace_back(network.get_edge_index("r2", "r1"));
  EXPECT_EQ(network.get_successors("g11", "r2"), successors_target);

  // r2,r1
  successors_target.clear();
  successors_target.emplace_back(network.get_edge_index("r1", "r0"));
  EXPECT_EQ(network.get_successors("r2", "r1"), successors_target);

  // r1,r0
  successors_target.clear();
  EXPECT_EQ(network.get_successors("r1", "r0"), successors_target);

  // r0,r1
  successors_target.clear();
  successors_target.emplace_back(network.get_edge_index("r1", "r2"));
  EXPECT_EQ(network.get_successors("r0", "r1"), successors_target);

  // r1,r2
  successors_target.clear();
  successors_target.emplace_back(network.get_edge_index("r2", "g01"));
  successors_target.emplace_back(network.get_edge_index("r2", "g11"));
  auto successors_r1_r2 = network.get_successors("r1", "r2");
  std::sort(successors_target.begin(), successors_target.end());
  std::sort(successors_r1_r2.begin(), successors_r1_r2.end());
  EXPECT_EQ(successors_r1_r2, successors_target);

  // r2,g01
  successors_target.clear();
  successors_target.emplace_back(network.get_edge_index("g01", "g00"));
  EXPECT_EQ(network.get_successors("r2", "g01"), successors_target);

  // r2,g11
  successors_target.clear();
  successors_target.emplace_back(network.get_edge_index("g11", "g10"));
  EXPECT_EQ(network.get_successors("r2", "g11"), successors_target);

  // g01,g00
  successors_target.clear();
  successors_target.emplace_back(network.get_edge_index("g00", "l3"));
  EXPECT_EQ(network.get_successors("g01", "g00"), successors_target);

  // g11,g10
  successors_target.clear();
  successors_target.emplace_back(network.get_edge_index("g10", "l3"));
  EXPECT_EQ(network.get_successors("g11", "g10"), successors_target);

  // g00,l3
  successors_target.clear();
  successors_target.emplace_back(network.get_edge_index("l3", "l2"));
  EXPECT_EQ(network.get_successors("g00", "l3"), successors_target);

  // g10,l3
  successors_target.clear();
  successors_target.emplace_back(network.get_edge_index("l3", "l2"));
  EXPECT_EQ(network.get_successors("g10", "l3"), successors_target);

  // l3,l2
  successors_target.clear();
  successors_target.emplace_back(network.get_edge_index("l2", "l1"));
  EXPECT_EQ(network.get_successors("l3", "l2"), successors_target);

  // l2,l1
  successors_target.clear();
  successors_target.emplace_back(network.get_edge_index("l1", "l0"));
  EXPECT_EQ(network.get_successors("l2", "l1"), successors_target);

  // l1,l0
  successors_target.clear();
  EXPECT_EQ(network.get_successors("l1", "l0"), successors_target);

  // Check timetable
  const auto& stations = instance.get_station_list();
  EXPECT_EQ(stations.size(), 1);
  EXPECT_TRUE(stations.has_station("Central"));

  // Check if the station is imported correctly
  const auto& station = stations.get_station("Central");
  EXPECT_EQ(station.name, "Central");
  EXPECT_EQ(station.tracks.size(), 4);
  std::vector<size_t> track_ids{network.get_edge_index("g00", "g01"),
                                network.get_edge_index("g10", "g11"),
                                network.get_edge_index("g01", "g00"),
                                network.get_edge_index("g11", "g10")};
  auto                tracks_read = station.tracks;
  std::sort(tracks_read.begin(), tracks_read.end());
  std::sort(track_ids.begin(), track_ids.end());
  EXPECT_EQ(tracks_read, track_ids);

  const auto& trains = instance.get_train_list();
  // Check if the all trains are imported
  EXPECT_EQ(trains.size(), 3);
  EXPECT_TRUE(trains.has_train("tr1"));
  EXPECT_TRUE(trains.has_train("tr2"));
  EXPECT_TRUE(trains.has_train("tr3"));
  // Check if the train tr1 is imported correctly
  auto tr1 = trains.get_train("tr1");
  EXPECT_EQ(tr1.name, "tr1");
  EXPECT_EQ(tr1.length, 100);
  EXPECT_EQ(tr1.max_speed, 83.33);
  EXPECT_EQ(tr1.acceleration, 2);
  EXPECT_EQ(tr1.deceleration, 1);
  // Check if the train tr2 is imported correctly
  auto tr2 = trains.get_train("tr2");
  EXPECT_EQ(tr2.name, "tr2");
  EXPECT_EQ(tr2.length, 100);
  EXPECT_EQ(tr2.max_speed, 27.78);
  EXPECT_EQ(tr2.acceleration, 2);
  EXPECT_EQ(tr2.deceleration, 1);
  // Check if the train tr3 is imported correctly
  auto tr3 = trains.get_train("tr3");
  EXPECT_EQ(tr3.name, "tr3");
  EXPECT_EQ(tr3.length, 250);
  EXPECT_EQ(tr3.max_speed, 20);
  EXPECT_EQ(tr3.acceleration, 2);
  EXPECT_EQ(tr3.deceleration, 1);

  // Check the schedule of tr1
  const auto& tr1_schedule = instance.get_schedule("tr1");
  EXPECT_EQ(tr1_schedule.get_t_0_range(), (std::pair<int, int>(120, 120)));
  EXPECT_EQ(tr1_schedule.get_v_0(), 0);
  EXPECT_EQ(tr1_schedule.get_t_n_range(), (std::pair<int, int>(645, 645)));
  EXPECT_EQ(tr1_schedule.get_v_n(), 16.67);
  EXPECT_EQ(network.get_vertex(tr1_schedule.get_entry()).name, "l0");
  EXPECT_EQ(network.get_vertex(tr1_schedule.get_exit()).name, "r0");
  EXPECT_EQ(tr1_schedule.get_stops().size(), 1);
  const auto& stop = tr1_schedule.get_stops()[0];
  EXPECT_EQ(stop.get_begin_range(), (std::pair<int, int>(240, 240)));
  EXPECT_EQ(stop.get_end_range(), (std::pair<int, int>(300, 300)));
  EXPECT_EQ(stop.get_min_stopping_time(), 300 - 240);
  EXPECT_EQ(stations.get_station(stop.get_station_name()).name, "Central");

  // Check the schedule of tr2
  const auto& tr2_schedule = instance.get_schedule("tr2");
  EXPECT_EQ(tr2_schedule.get_t_0_range(), (std::pair<int, int>(0, 0)));
  EXPECT_EQ(tr2_schedule.get_v_0(), 0);
  EXPECT_EQ(tr2_schedule.get_t_n_range(), (std::pair<int, int>(420, 420)));
  EXPECT_EQ(tr2_schedule.get_v_n(), 16.67);
  EXPECT_EQ(network.get_vertex(tr2_schedule.get_entry()).name, "l0");
  EXPECT_EQ(network.get_vertex(tr2_schedule.get_exit()).name, "r0");
  EXPECT_EQ(tr2_schedule.get_stops().size(), 1);
  const auto& stop2 = tr2_schedule.get_stops()[0];
  EXPECT_EQ(stop2.get_begin_range(), (std::pair<int, int>(120, 120)));
  EXPECT_EQ(stop2.get_end_range(), (std::pair<int, int>(300, 300)));
  EXPECT_EQ(stop2.get_min_stopping_time(), 300 - 120);
  EXPECT_EQ(stations.get_station(stop2.get_station_name()).name, "Central");

  // Check the schedule of tr3
  const auto& tr3_schedule = instance.get_schedule("tr3");
  EXPECT_EQ(tr3_schedule.get_t_0_range(), (std::pair<int, int>(0, 0)));
  EXPECT_EQ(tr3_schedule.get_v_0(), 0);
  EXPECT_EQ(tr3_schedule.get_t_n_range(), (std::pair<int, int>(420, 420)));
  EXPECT_EQ(tr3_schedule.get_v_n(), 16.67);
  EXPECT_EQ(network.get_vertex(tr3_schedule.get_entry()).name, "r0");
  EXPECT_EQ(network.get_vertex(tr3_schedule.get_exit()).name, "l0");
  EXPECT_EQ(tr3_schedule.get_stops().size(), 1);
  const auto& stop3 = tr3_schedule.get_stops()[0];
  EXPECT_EQ(stop3.get_begin_range(), (std::pair<int, int>(180, 180)));
  EXPECT_EQ(stop3.get_end_range(), (std::pair<int, int>(300, 300)));
  EXPECT_EQ(stop3.get_min_stopping_time(), 300 - 180);
  EXPECT_EQ(stations.get_station(stop3.get_station_name()).name, "Central");

  // Check the route  map
  // Check if the route consists of three trains with names "tr1", "tr2" and
  // "tr3"
  EXPECT_EQ(instance.route_map_size(), 3);
  EXPECT_TRUE(instance.has_route("tr1"));
  EXPECT_TRUE(instance.has_route("tr2"));
  EXPECT_TRUE(instance.has_route("tr3"));

  // Check if the route for tr1 consists of eight edges passing vertices
  // l0-l1-l2-l3-g00-g01-r2-r1-r0 in this order.
  const auto& route = instance.get_route("tr1");
  EXPECT_EQ(route.size(), 8);
  EXPECT_EQ(network.get_vertex(route.get_edge(0, network).source).name, "l0");
  EXPECT_EQ(network.get_vertex(route.get_edge(0, network).target).name, "l1");
  EXPECT_EQ(network.get_vertex(route.get_edge(1, network).source).name, "l1");
  EXPECT_EQ(network.get_vertex(route.get_edge(1, network).target).name, "l2");
  EXPECT_EQ(network.get_vertex(route.get_edge(2, network).source).name, "l2");
  EXPECT_EQ(network.get_vertex(route.get_edge(2, network).target).name, "l3");
  EXPECT_EQ(network.get_vertex(route.get_edge(3, network).source).name, "l3");
  EXPECT_EQ(network.get_vertex(route.get_edge(3, network).target).name, "g00");
  EXPECT_EQ(network.get_vertex(route.get_edge(4, network).source).name, "g00");
  EXPECT_EQ(network.get_vertex(route.get_edge(4, network).target).name, "g01");
  EXPECT_EQ(network.get_vertex(route.get_edge(5, network).source).name, "g01");
  EXPECT_EQ(network.get_vertex(route.get_edge(5, network).target).name, "r2");
  EXPECT_EQ(network.get_vertex(route.get_edge(6, network).source).name, "r2");
  EXPECT_EQ(network.get_vertex(route.get_edge(6, network).target).name, "r1");
  EXPECT_EQ(network.get_vertex(route.get_edge(7, network).source).name, "r1");
  EXPECT_EQ(network.get_vertex(route.get_edge(7, network).target).name, "r0");

  // Check if the route for tr2 consists of eight edges passing vertices
  // l0-l1-l2-l3-g00-g01-r2-r1-r0 in this order.
  const auto& route2 = instance.get_route("tr2");
  EXPECT_EQ(route2.size(), 8);
  EXPECT_EQ(network.get_vertex(route2.get_edge(0, network).source).name, "l0");
  EXPECT_EQ(network.get_vertex(route2.get_edge(0, network).target).name, "l1");
  EXPECT_EQ(network.get_vertex(route2.get_edge(1, network).source).name, "l1");
  EXPECT_EQ(network.get_vertex(route2.get_edge(1, network).target).name, "l2");
  EXPECT_EQ(network.get_vertex(route2.get_edge(2, network).source).name, "l2");
  EXPECT_EQ(network.get_vertex(route2.get_edge(2, network).target).name, "l3");
  EXPECT_EQ(network.get_vertex(route2.get_edge(3, network).source).name, "l3");
  EXPECT_EQ(network.get_vertex(route2.get_edge(3, network).target).name, "g00");
  EXPECT_EQ(network.get_vertex(route2.get_edge(4, network).source).name, "g00");
  EXPECT_EQ(network.get_vertex(route2.get_edge(4, network).target).name, "g01");
  EXPECT_EQ(network.get_vertex(route2.get_edge(5, network).source).name, "g01");
  EXPECT_EQ(network.get_vertex(route2.get_edge(5, network).target).name, "r2");
  EXPECT_EQ(network.get_vertex(route2.get_edge(6, network).source).name, "r2");
  EXPECT_EQ(network.get_vertex(route2.get_edge(6, network).target).name, "r1");
  EXPECT_EQ(network.get_vertex(route2.get_edge(7, network).source).name, "r1");
  EXPECT_EQ(network.get_vertex(route2.get_edge(7, network).target).name, "r0");

  // Check if the route for tr3 consists of eight edges passing vertices
  // r0-r1-r2-g11-g10-l3-l2-l1 in this order.
  const auto& route3 = instance.get_route("tr3");
  EXPECT_EQ(route3.size(), 8);
  EXPECT_EQ(network.get_vertex(route3.get_edge(0, network).source).name, "r0");
  EXPECT_EQ(network.get_vertex(route3.get_edge(0, network).target).name, "r1");
  EXPECT_EQ(network.get_vertex(route3.get_edge(1, network).source).name, "r1");
  EXPECT_EQ(network.get_vertex(route3.get_edge(1, network).target).name, "r2");
  EXPECT_EQ(network.get_vertex(route3.get_edge(2, network).source).name, "r2");
  EXPECT_EQ(network.get_vertex(route3.get_edge(2, network).target).name, "g11");
  EXPECT_EQ(network.get_vertex(route3.get_edge(3, network).source).name, "g11");
  EXPECT_EQ(network.get_vertex(route3.get_edge(3, network).target).name, "g10");
  EXPECT_EQ(network.get_vertex(route3.get_edge(4, network).source).name, "g10");
  EXPECT_EQ(network.get_vertex(route3.get_edge(4, network).target).name, "l3");
  EXPECT_EQ(network.get_vertex(route3.get_edge(5, network).source).name, "l3");
  EXPECT_EQ(network.get_vertex(route3.get_edge(5, network).target).name, "l2");
  EXPECT_EQ(network.get_vertex(route3.get_edge(6, network).source).name, "l2");
  EXPECT_EQ(network.get_vertex(route3.get_edge(6, network).target).name, "l1");
  EXPECT_EQ(network.get_vertex(route3.get_edge(7, network).source).name, "l1");
  EXPECT_EQ(network.get_vertex(route3.get_edge(7, network).target).name, "l0");

  // Check consistency
  EXPECT_TRUE(instance.check_consistency());
  EXPECT_TRUE(instance.check_consistency(true));
  EXPECT_TRUE(instance.check_consistency(false));

  // Check if max_t is correct
  EXPECT_EQ(instance.max_t(), 645);
}

TEST(GeneralAbstractDataStructure, VSSGenerationTimetableParse) {
  const auto instance =
      cda_rail::instances::VSSGenerationTimetable::import_instance(
          "./example-networks/SimpleStation/");

  const auto general_instance =
      cda_rail::instances::GeneralPerformanceOptimizationInstance::
          cast_from_vss_generation(instance);
  check_instance_import_general_cast(general_instance);
}
