#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Timetable.hpp"

#include "gmock/gmock-spec-builders.h"
#include "gtest/gtest.h"

TEST(TimetableFunctionality, ReadTimetable) {
  auto network = cda_rail::Network::import_network("SimpleStation", "./data/");
  auto timetable = cda_rail::Timetable::import_timetable(
      "./example-networks/SimpleStation/timetable/", network);

  // Check if the timetable has the correct stations
  const auto& stations = timetable.get_station_list();
  EXPECT_EQ(stations.size(), 1);
  EXPECT_TRUE(stations.has_station("Central"));

  // Check if the station is imported correctly
  const auto& station = stations.get_station("Central");
  EXPECT_EQ(station.name, "Central");
  EXPECT_EQ(station.tracks.size(), 4);
  const cda_rail::index_set track_ids_target{
      network.get_edge_index({"g00"}, {"g01"}),
      network.get_edge_index({"g10"}, {"g11"}),
      network.get_edge_index({"g01"}, {"g00"}),
      network.get_edge_index({"g11"}, {"g10"})};
  EXPECT_EQ(station.tracks, track_ids_target);

  // Check if the timetable has the correct trains
  const auto& trains = timetable.get_train_list();
  // Check if the all trains are imported
  EXPECT_EQ(trains.size(), 3);
  EXPECT_TRUE(trains.has_train("tr1"));
  EXPECT_TRUE(trains.has_train("tr2"));
  EXPECT_TRUE(trains.has_train("tr3"));
  // Check if the train tr1 is imported correctly
  auto tr1 = trains.get_train("tr1");
  EXPECT_EQ(tr1.get_name(), "tr1");
  EXPECT_EQ(tr1.get_length(), 100);
  EXPECT_EQ(tr1.get_max_speed(), 83.33);
  EXPECT_EQ(tr1.get_acceleration(), 2);
  EXPECT_EQ(tr1.get_deceleration(), 1);
  // Check if the train tr2 is imported correctly
  auto tr2 = trains.get_train("tr2");
  EXPECT_EQ(tr2.get_name(), "tr2");
  EXPECT_EQ(tr2.get_length(), 100);
  EXPECT_EQ(tr2.get_max_speed(), 27.78);
  EXPECT_EQ(tr2.get_acceleration(), 2);
  EXPECT_EQ(tr2.get_deceleration(), 1);
  // Check if the train tr3 is imported correctly
  auto tr3 = trains.get_train("tr3");
  EXPECT_EQ(tr3.get_name(), "tr3");
  EXPECT_EQ(tr3.get_length(), 250);
  EXPECT_EQ(tr3.get_max_speed(), 20);
  EXPECT_EQ(tr3.get_acceleration(), 2);
  EXPECT_EQ(tr3.get_deceleration(), 1);

  // Check the schedule of tr1
  const auto& tr1_schedule = timetable.get_schedule("tr1");
  EXPECT_EQ(tr1_schedule.get_entry_time(), 120);
  EXPECT_EQ(tr1_schedule.get_initial_velocity(), 0);
  EXPECT_EQ(tr1_schedule.get_exit_time(), 645);
  EXPECT_EQ(tr1_schedule.get_exit_velocity(), 16.67);
  EXPECT_EQ(network.get_vertex(tr1_schedule.get_entry_vertex()).name, "l0");
  EXPECT_EQ(network.get_vertex(tr1_schedule.get_exit_vertex()).name, "r0");
  EXPECT_EQ(tr1_schedule.get_stops().size(), 1);
  const auto& stop = tr1_schedule.get_stops()[0];
  EXPECT_EQ(stop.get_service_time(), 240);
  EXPECT_EQ(stop.get_earliest_departure(), 300);
  EXPECT_EQ(stations.get_station(stop.get_station().name).name, "Central");

  // Check the schedule of tr2
  const auto& tr2_schedule = timetable.get_schedule("tr2");
  EXPECT_EQ(tr2_schedule.get_entry_time(), 0);
  EXPECT_EQ(tr2_schedule.get_initial_velocity(), 0);
  EXPECT_EQ(tr2_schedule.get_exit_time(), 420);
  EXPECT_EQ(tr2_schedule.get_exit_velocity(), 16.67);
  EXPECT_EQ(network.get_vertex(tr2_schedule.get_entry_vertex()).name, "l0");
  EXPECT_EQ(network.get_vertex(tr2_schedule.get_exit_vertex()).name, "r0");
  EXPECT_EQ(tr2_schedule.get_stops().size(), 1);
  const auto& stop2 = tr2_schedule.get_stops()[0];
  EXPECT_EQ(stop2.get_service_time(), 120);
  EXPECT_EQ(stop2.get_earliest_departure(), 300);
  EXPECT_EQ(stations.get_station(stop2.get_station().name).name, "Central");

  // Check the schedule of tr3
  const auto& tr3_schedule = timetable.get_schedule("tr3");
  EXPECT_EQ(tr3_schedule.get_entry_time(), 0);
  EXPECT_EQ(tr3_schedule.get_initial_velocity(), 0);
  EXPECT_EQ(tr3_schedule.get_exit_time(), 420);
  EXPECT_EQ(tr3_schedule.get_exit_velocity(), 16.67);
  EXPECT_EQ(network.get_vertex(tr3_schedule.get_entry_vertex()).name, "r0");
  EXPECT_EQ(network.get_vertex(tr3_schedule.get_exit_vertex()).name, "l0");
  EXPECT_EQ(tr3_schedule.get_stops().size(), 1);
  const auto& stop3 = tr3_schedule.get_stops()[0];
  EXPECT_EQ(stop3.get_service_time(), 180);
  EXPECT_EQ(stop3.get_earliest_departure(), 300);
  EXPECT_EQ(stations.get_station(stop3.get_station().name).name, "Central");

  EXPECT_EQ(timetable.latest_exit_time(), 645);

  EXPECT_TRUE(timetable.check_consistency(network));
}

TEST(TimetableFunctionality, WriteTimetable) {
  auto network = cda_rail::Network::import_network("SimpleStation", "./data/");
  cda_rail::Timetable timetable;

  timetable.add_train("tr1", 100, 83.33, 2, 1, 0, 0, "l0", 300, 20, "r0",
                      network);
  timetable.add_train("tr2", 100, 27.78, 2, 1, 0, 0, "r0", 300, 20, "l0",
                      network);

  EXPECT_EQ(timetable.get_schedule("tr1").get_entry_time(), 0);
  EXPECT_EQ(timetable.get_schedule("tr1").get_exit_time(), 300);
  EXPECT_EQ(timetable.get_schedule("tr2").get_entry_time(), 0);
  EXPECT_EQ(timetable.get_schedule("tr2").get_exit_time(), 300);

  timetable.add_empty_station("Station1");
  timetable.add_empty_station("Station2");

  timetable.add_track_to_station("Station1", {"g00", "g01"}, network);
  timetable.add_track_to_station("Station1", {"g10", "g11"}, network);
  timetable.add_track_to_station("Station1", {"g01", "g00"}, network);
  timetable.add_track_to_station("Station1", {"g11", "g10"}, network);
  timetable.add_track_to_station("Station2", {"r1", "r0"}, network);

  timetable.insert_stop("tr1", "Station1", 100, 160);
  timetable.insert_stop("tr1", "Station2", 200, 260);
  timetable.insert_stop("tr2", "Station1", 160, 220);

  // Check if the timetable is as expected
  // Check if the timetable has the correct stations
  const auto& stations = timetable.get_station_list();
  EXPECT_EQ(stations.size(), 2);
  EXPECT_TRUE(stations.has_station("Station1"));
  EXPECT_TRUE(stations.has_station("Station2"));

  // Check if the stations are imported correctly
  const auto& st1 = stations.get_station("Station1");
  EXPECT_EQ(st1.name, "Station1");
  EXPECT_EQ(st1.tracks.size(), 4);
  const cda_rail::index_set s1_expected_tracks{
      network.get_edge_index({"g00"}, {"g01"}),
      network.get_edge_index({"g10"}, {"g11"}),
      network.get_edge_index({"g01"}, {"g00"}),
      network.get_edge_index({"g11"}, {"g10"})};
  EXPECT_EQ(st1.tracks, s1_expected_tracks);
  const auto& st2 = stations.get_station("Station2");
  EXPECT_EQ(st2.name, "Station2");
  EXPECT_EQ(st2.tracks.size(), 1);
  const cda_rail::index_set s2_expected_tracks{
      network.get_edge_index({"r1"}, {"r0"})};
  EXPECT_EQ(st2.tracks, s2_expected_tracks);

  // Check if the timetable has the correct trains
  const auto& trains = timetable.get_train_list();
  EXPECT_EQ(trains.size(), 2);
  EXPECT_TRUE(trains.has_train("tr1"));
  EXPECT_TRUE(trains.has_train("tr2"));

  // Check if the train tr1 is saved correctly
  auto tr1 = trains.get_train("tr1");
  EXPECT_EQ(tr1.get_name(), "tr1");
  EXPECT_EQ(tr1.get_length(), 100);
  EXPECT_EQ(tr1.get_max_speed(), 83.33);
  EXPECT_EQ(tr1.get_acceleration(), 2);
  EXPECT_EQ(tr1.get_deceleration(), 1);
  // Check if the train tr2 is saved correctly
  auto tr2 = trains.get_train("tr2");
  EXPECT_EQ(tr2.get_name(), "tr2");
  EXPECT_EQ(tr2.get_length(), 100);
  EXPECT_EQ(tr2.get_max_speed(), 27.78);
  EXPECT_EQ(tr2.get_acceleration(), 2);
  EXPECT_EQ(tr2.get_deceleration(), 1);

  // Check if the schedule of tr1 is saved correctly
  const auto& tr1_schedule = timetable.get_schedule("tr1");
  EXPECT_EQ(tr1_schedule.get_entry_time(), 0);
  EXPECT_EQ(tr1_schedule.get_initial_velocity(), 0);
  EXPECT_EQ(tr1_schedule.get_exit_time(), 300);
  EXPECT_EQ(tr1_schedule.get_exit_velocity(), 20);
  EXPECT_EQ(network.get_vertex(tr1_schedule.get_entry_vertex()).name, "l0");
  EXPECT_EQ(network.get_vertex(tr1_schedule.get_exit_vertex()).name, "r0");
  EXPECT_EQ(tr1_schedule.get_stops().size(), 2);
  const auto& stop1 = tr1_schedule.get_stops()[0];
  EXPECT_EQ(stop1.get_service_time(), 100);
  EXPECT_EQ(stop1.get_earliest_departure(), 160);
  EXPECT_EQ(stations.get_station(stop1.get_station().name).name, "Station1");
  const auto& stop2 = tr1_schedule.get_stops()[1];
  EXPECT_EQ(stop2.get_service_time(), 200);
  EXPECT_EQ(stop2.get_earliest_departure(), 260);
  EXPECT_EQ(stations.get_station(stop2.get_station().name).name, "Station2");

  // Check if the schedule of tr2 is saved correctly
  const auto& tr2_schedule = timetable.get_schedule("tr2");
  EXPECT_EQ(tr2_schedule.get_entry_time(), 0);
  EXPECT_EQ(tr2_schedule.get_initial_velocity(), 0);
  EXPECT_EQ(tr2_schedule.get_exit_time(), 300);
  EXPECT_EQ(tr2_schedule.get_exit_velocity(), 20);
  EXPECT_EQ(network.get_vertex(tr2_schedule.get_entry_vertex()).name, "r0");
  EXPECT_EQ(network.get_vertex(tr2_schedule.get_exit_vertex()).name, "l0");
  EXPECT_EQ(tr2_schedule.get_stops().size(), 1);
  const auto& stop3 = tr2_schedule.get_stops()[0];
  EXPECT_EQ(stop3.get_service_time(), 160);
  EXPECT_EQ(stop3.get_earliest_departure(), 220);
  EXPECT_EQ(stations.get_station(stop3.get_station().name).name, "Station1");

  // Write timetable to directory
  timetable.export_timetable("./tmp/test-timetable/", network);

  // Read timetable from directory
  auto timetable_read =
      cda_rail::Timetable::import_timetable("./tmp/test-timetable/", network);

  // Delete temporary files
  std::filesystem::remove_all("./tmp");

  // Check if the timetable is as expected
  // Check if the timetable has the correct stations
  const auto& stations_read = timetable_read.get_station_list();
  EXPECT_EQ(stations_read.size(), 2);
  EXPECT_TRUE(stations_read.has_station("Station1"));
  EXPECT_TRUE(stations_read.has_station("Station2"));

  // Check if the stations are imported correctly
  const auto& st1_read = stations_read.get_station("Station1");
  EXPECT_EQ(st1_read.name, "Station1");
  EXPECT_EQ(st1_read.tracks.size(), 4);
  EXPECT_EQ(st1_read.tracks, s1_expected_tracks);
  const auto& st2_read = stations_read.get_station("Station2");
  EXPECT_EQ(st2_read.name, "Station2");
  EXPECT_EQ(st2_read.tracks.size(), 1);
  EXPECT_EQ(st2_read.tracks, s2_expected_tracks);

  // Check if the timetable has the correct trains
  const auto& trains_read = timetable_read.get_train_list();
  EXPECT_EQ(trains_read.size(), 2);
  EXPECT_TRUE(trains_read.has_train("tr1"));
  EXPECT_TRUE(trains_read.has_train("tr2"));

  // Check if the train tr1 is saved correctly
  auto tr1_read = trains_read.get_train("tr1");
  EXPECT_EQ(tr1_read.get_name(), "tr1");
  EXPECT_EQ(tr1_read.get_length(), 100);
  EXPECT_EQ(tr1_read.get_max_speed(), 83.33);
  EXPECT_EQ(tr1_read.get_acceleration(), 2);
  EXPECT_EQ(tr1_read.get_deceleration(), 1);
  // Check if the train tr2 is saved correctly
  auto tr2_read = trains_read.get_train("tr2");
  EXPECT_EQ(tr2_read.get_name(), "tr2");
  EXPECT_EQ(tr2_read.get_length(), 100);
  EXPECT_EQ(tr2_read.get_max_speed(), 27.78);
  EXPECT_EQ(tr2_read.get_acceleration(), 2);
  EXPECT_EQ(tr2_read.get_deceleration(), 1);

  // Check if the schedule of tr1 is saved correctly
  const auto& tr1_schedule_read = timetable_read.get_schedule("tr1");
  EXPECT_EQ(tr1_schedule_read.get_entry_time(), 0);
  EXPECT_EQ(tr1_schedule_read.get_initial_velocity(), 0);
  EXPECT_EQ(tr1_schedule_read.get_exit_time(), 300);
  EXPECT_EQ(tr1_schedule_read.get_exit_velocity(), 20);
  EXPECT_EQ(network.get_vertex(tr1_schedule_read.get_entry_vertex()).name,
            "l0");
  EXPECT_EQ(network.get_vertex(tr1_schedule_read.get_exit_vertex()).name, "r0");
  EXPECT_EQ(tr1_schedule_read.get_stops().size(), 2);
  const auto& stop1_read = tr1_schedule_read.get_stops()[0];
  EXPECT_EQ(stop1_read.get_service_time(), 100);
  EXPECT_EQ(stop1_read.get_earliest_departure(), 160);
  EXPECT_EQ(stations_read.get_station(stop1_read.get_station().name).name,
            "Station1");
  const auto& stop2_read = tr1_schedule_read.get_stops()[1];
  EXPECT_EQ(stop2_read.get_service_time(), 200);
  EXPECT_EQ(stop2_read.get_earliest_departure(), 260);
  EXPECT_EQ(stations_read.get_station(stop2_read.get_station().name).name,
            "Station2");

  // Check if the schedule of tr2 is saved correctly
  const auto& tr2_schedule_read = timetable_read.get_schedule("tr2");
  EXPECT_EQ(tr2_schedule_read.get_entry_time(), 0);
  EXPECT_EQ(tr2_schedule_read.get_initial_velocity(), 0);
  EXPECT_EQ(tr2_schedule_read.get_exit_time(), 300);
  EXPECT_EQ(tr2_schedule_read.get_exit_velocity(), 20);
  EXPECT_EQ(network.get_vertex(tr2_schedule_read.get_entry_vertex()).name,
            "r0");
  EXPECT_EQ(network.get_vertex(tr2_schedule_read.get_exit_vertex()).name, "l0");
  EXPECT_EQ(tr2_schedule_read.get_stops().size(), 1);
  const auto& stop3_read = tr2_schedule_read.get_stops()[0];
  EXPECT_EQ(stop3_read.get_service_time(), 160);
  EXPECT_EQ(stop3_read.get_earliest_departure(), 220);
  EXPECT_EQ(stations_read.get_station(stop3_read.get_station().name).name,
            "Station1");
}

TEST(TimetableFunctionality, TimetableConsistency) {
  auto network  = cda_rail::Network::import_network("SimpleStation", "./data/");
  auto network1 = cda_rail::Network::import_network("SimpleStation", "./data/");
  network1.add_edge({"l0"}, {"r1"}, 100, 10, false);
  auto network2 = cda_rail::Network::import_network("SimpleStation", "./data/");
  network2.add_edge({"r0"}, {"l1"}, 100, 10, false);
  // NOLINTNEXTLINE(misc-const-correctness)
  cda_rail::Network network3;
  cda_rail::Network network4;
  network4.add_vertex("l0", cda_rail::VertexType::TTD);
  network4.add_vertex("l1", cda_rail::VertexType::TTD);
  network4.add_vertex("l2", cda_rail::VertexType::TTD);
  network4.add_vertex("l3", cda_rail::VertexType::TTD);
  network4.add_vertex("r0", cda_rail::VertexType::TTD);
  network4.add_vertex("r1", cda_rail::VertexType::TTD);
  network4.add_vertex("r2", cda_rail::VertexType::TTD);
  network4.add_vertex("r3", cda_rail::VertexType::TTD);
  network4.add_vertex("g00", cda_rail::VertexType::TTD);
  network4.add_vertex("g01", cda_rail::VertexType::TTD);
  network4.add_vertex("g10", cda_rail::VertexType::TTD);
  network4.add_vertex("g11", cda_rail::VertexType::TTD);
  cda_rail::Timetable timetable;

  timetable.add_empty_station("Station1");
  timetable.add_track_to_station("Station1", {"g00", "g01"}, network);

  const auto l0 = network.get_vertex_index("l0");
  const auto r0 = network.get_vertex_index("r0");

  const auto tr1 = timetable.add_train("tr1", 100, 83.33, 2, 1, 0, 0, l0, 300,
                                       20, r0, network);
  timetable.insert_stop(tr1, "Station1", 0, 60);

  EXPECT_TRUE(timetable.check_consistency(network));
  EXPECT_FALSE(timetable.check_consistency(network1));
  EXPECT_FALSE(timetable.check_consistency(network2));
  EXPECT_FALSE(timetable.check_consistency(network3));
  EXPECT_FALSE(timetable.check_consistency(network4));
}

TEST(TimetableFunctionality, TimetableExceptions) {
  auto network = cda_rail::Network::import_network("SimpleStation", "./data/");
  cda_rail::Timetable timetable;

  const auto l0 = network.get_vertex_index("l0");
  const auto r0 = network.get_vertex_index("r0");

  const auto v_x = 100 * (l0 + r0);

  const auto tr1 = timetable.add_train("tr1", 100, 83.33, 2, 1, 0, 0, l0, 300,
                                       20, r0, network);
  EXPECT_THROW((void)timetable.add_train("tr1", 100, 83.33, 2, 1, 0, 0, l0, 300,
                                         20, r0, network),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_THROW((void)timetable.add_train("tr2", 100, 83.33, 2, 1, 0, 0, v_x,
                                         300, 20, r0, network),
               cda_rail::exceptions::VertexNotExistentException);
  EXPECT_THROW((void)timetable.add_train("tr2", 100, 83.33, 2, 1, 0, 0, l0, 300,
                                         20, v_x, network),
               cda_rail::exceptions::VertexNotExistentException);

  EXPECT_THROW((void)timetable.get_schedule(10),
               cda_rail::exceptions::TrainNotExistentException);

  timetable.add_empty_station("Station1");
  timetable.add_empty_station("Station2");

  timetable.add_track_to_station("Station1", {"g00", "g01"}, network);
  timetable.add_track_to_station("Station1", {"g10", "g11"}, network);
  timetable.add_track_to_station("Station1", {"g01", "g00"}, network);
  timetable.add_track_to_station("Station1", {"g11", "g10"}, network);
  timetable.add_track_to_station("Station2", {"r1", "r0"}, network);

  EXPECT_THROW((void)timetable.insert_stop(tr1 + 10, "Station1", 0, 60),
               cda_rail::exceptions::TrainNotExistentException);
  EXPECT_THROW((void)timetable.insert_stop(tr1, "Station3", 0, 60),
               cda_rail::exceptions::StationNotExistentException);
  EXPECT_THROW((void)timetable.insert_stop(tr1, "Station1", -1, 60),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW((void)timetable.insert_stop(tr1, "Station1", 0, -1),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW((void)timetable.insert_stop(tr1, "Station1", 60, 0),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW((void)timetable.insert_stop(tr1, "Station1", 60, 60),
               cda_rail::exceptions::InvalidInputException);

  timetable.insert_stop(tr1, "Station1", 0, 60);
  EXPECT_THROW((void)timetable.insert_stop(tr1, "Station1", 0, 60),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_THROW((void)timetable.insert_stop(tr1, "Station2", 30, 90),
               cda_rail::exceptions::ConsistencyException);

  EXPECT_THROW((void)timetable.time_index_interval(tr1 + 10, 15, true),
               cda_rail::exceptions::TrainNotExistentException);
}
