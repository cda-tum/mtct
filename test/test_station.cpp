#include "datastructure/Station.hpp"

#include "gtest/gtest.h"

TEST(Functionality, ReadStation) {
  auto network  = cda_rail::Network::import_network("SimpleStation", "./data/");
  auto stations = cda_rail::StationList::import_stations(
      "./example-networks/SimpleStation/timetable/", network);

  // Check if the station is imported correctly
  EXPECT_EQ(stations.size(), 1);
  EXPECT_TRUE(stations.has_station("Central"));

  // Check if the station is imported correctly
  const auto& station = stations.get_station("Central");
  EXPECT_EQ(station.name, "Central");
  EXPECT_EQ(station.tracks.size(), 4);
  cda_rail::index_set track_ids{network.get_edge_index({"g00"}, {"g01"}),
                                network.get_edge_index({"g10"}, {"g11"}),
                                network.get_edge_index({"g01"}, {"g00"}),
                                network.get_edge_index({"g11"}, {"g10"})};
  EXPECT_EQ(station.tracks, track_ids);
}

TEST(Functionality, WriteStations) {
  auto network = cda_rail::Network::import_network("SimpleStation", "./data/");
  cda_rail::StationList stations;

  stations.add_empty_station("S1");
  stations.add_empty_station("S2");

  const auto& l0_l1 = network.get_edge_index({"l0"}, {"l1"});

  stations.add_track_to_station("S1", "l0", "l1", network);
  stations.add_track_to_station("S1", "l0", "l1", network);
  stations.add_track_to_station("S2", "l0", "l1", network);
  stations.add_track_to_station("S2", "l1", "l2", network);

  EXPECT_THROW((void)stations.get_station("S3"),
               cda_rail::exceptions::StationNotExistentException);
  EXPECT_THROW((void)stations.add_track_to_station("S3", l0_l1, network),
               cda_rail::exceptions::StationNotExistentException);
  EXPECT_THROW((void)stations.add_track_to_station("S1", 100, network),
               cda_rail::exceptions::EdgeNotExistentException);

  stations.export_stations("./tmp/write_stations_test", network);
  auto stations_read = cda_rail::StationList::import_stations(
      "./tmp/write_stations_test", network);

  std::filesystem::remove_all("./tmp");

  EXPECT_EQ(stations_read.size(), 2);
  EXPECT_TRUE(stations_read.has_station("S1"));
  EXPECT_TRUE(stations_read.has_station("S2"));

  const auto& s1 = stations_read.get_station("S1");
  EXPECT_EQ(s1.name, "S1");
  EXPECT_EQ(s1.tracks.size(), 1);
  const cda_rail::index_set s1_tracks{network.get_edge_index({"l0"}, {"l1"})};
  EXPECT_EQ(s1.tracks, s1_tracks);

  const auto& s2 = stations_read.get_station("S2");
  EXPECT_EQ(s2.name, "S2");
  EXPECT_EQ(s2.tracks.size(), 2);
  cda_rail::index_set s2_tracks_target{network.get_edge_index({"l0"}, {"l1"}),
                                       network.get_edge_index({"l1"}, {"l2"})};
  EXPECT_EQ(s2.tracks, s2_tracks_target);
}
