#include "datastructure/Station.hpp"

#include "gtest/gtest.h"

namespace {

cda_rail::Network build_linear_test_network() {
  cda_rail::Network network{"StationUnitTestNetwork"};

  const auto v0 = network.add_vertex("v0", cda_rail::VertexType::TTD);
  const auto v1 = network.add_vertex("v1", cda_rail::VertexType::TTD);
  const auto v2 = network.add_vertex("v2", cda_rail::VertexType::TTD);
  const auto v3 = network.add_vertex("v3", cda_rail::VertexType::TTD);

  const auto e01 = network.add_edge(v0, v1, 100, 20, false, 10, 10);
  const auto e12 = network.add_edge(v1, v2, 100, 20, false, 10, 10);
  const auto e23 = network.add_edge(v2, v3, 100, 20, false, 10, 10);

  network.add_successor(e01, e12);
  network.add_successor(e12, e23);

  return network;
}

} // namespace

// Station class

TEST(Station, EqualityDependsOnNameAndTracks) {
  const cda_rail::Station s1{"Central", {1, 2}};
  const cda_rail::Station s2{"Central", {2, 1}};
  const cda_rail::Station s3{"West", {1, 2}};
  const cda_rail::Station s4{"Central", {1, 3}};

  EXPECT_EQ(s1, s2);
  EXPECT_NE(s1, s3);
  EXPECT_NE(s1, s4);
}

TEST(Station, IsFullyInStationChecksSubsetSemantics) {
  const cda_rail::Station station{"S", {1, 2, 3}};

  EXPECT_TRUE(station.is_fully_in_station({}));
  EXPECT_TRUE(station.is_fully_in_station({1, 3}));
  EXPECT_FALSE(station.is_fully_in_station({1, 4}));
}

TEST(Station, GetStopTracksReturnsEmptyWhenStationHasNoTracks) {
  const auto              network = build_linear_test_network();
  const cda_rail::Station station{"S", {}};

  EXPECT_TRUE(station.get_stop_tracks(1.0, network).empty());
}

TEST(Station, GetStopTracksRespectsEdgesToConsider) {
  const auto              network = build_linear_test_network();
  const auto              e01     = network.get_edge_index({"v0"}, {"v1"});
  const auto              e12     = network.get_edge_index({"v1"}, {"v2"});
  const cda_rail::Station station{"S", {e01, e12}};

  const auto stop_tracks = station.get_stop_tracks(1.0, network, {e12});

  ASSERT_FALSE(stop_tracks.empty());
  for (const auto& [stop_edge, paths] : stop_tracks) {
    EXPECT_EQ(stop_edge, e12);
    EXPECT_FALSE(paths.empty());
  }
}

// Statin list class

TEST(StationList, DefaultConstructionCreatesEmptyList) {
  const cda_rail::StationList stations;

  EXPECT_EQ(stations.size(), 0);
  EXPECT_EQ(stations.cbegin(), stations.cend());
}

TEST(StationList, AddEmptyStationCreatesStationAndPreventsDuplicates) {
  cda_rail::StationList stations;

  stations.add_empty_station("S");

  EXPECT_EQ(stations.size(), 1);
  EXPECT_TRUE(stations.has_station("S"));
  EXPECT_THROW(stations.add_empty_station("S"),
               cda_rail::exceptions::ConsistencyException);
}

TEST(StationList, GetStationReturnsStationAndThrowsForUnknownName) {
  cda_rail::StationList stations;
  stations.add_empty_station("S");

  EXPECT_EQ(stations.get_station("S").name, "S");
  EXPECT_THROW((void)stations.get_station("Missing"),
               cda_rail::exceptions::StationNotExistentException);
}

TEST(StationList, GetStationNamesReturnsAllAddedNames) {
  cda_rail::StationList stations;
  stations.add_empty_station("S1");
  stations.add_empty_station("S2");

  const auto names = stations.get_station_names();

  EXPECT_EQ(names.size(), 2);
  EXPECT_TRUE(names.contains("S1"));
  EXPECT_TRUE(names.contains("S2"));
}

TEST(StationList, AddTrackByIndexIsIdempotentAndRequiresExistingStation) {
  const auto            network = build_linear_test_network();
  const auto            e01     = network.get_edge_index({"v0"}, {"v1"});
  cda_rail::StationList stations;
  stations.add_empty_station("S");

  stations.add_track_to_station("S", e01);
  stations.add_track_to_station("S", e01);

  EXPECT_EQ(stations.get_station("S").tracks.size(), 1);
  EXPECT_THROW(stations.add_track_to_station("Missing", e01),
               cda_rail::exceptions::StationNotExistentException);
}

TEST(StationList, AddTrackByEdgeInputValidatesEdgeAndStation) {
  const auto            network = build_linear_test_network();
  const auto            e01     = network.get_edge_index({"v0"}, {"v1"});
  cda_rail::StationList stations;
  stations.add_empty_station("S");

  stations.add_track_to_station("S", {"v0", "v1"}, network);

  EXPECT_TRUE(stations.get_station("S").tracks.contains(e01));
  EXPECT_THROW(stations.add_track_to_station("S", {"v3", "v0"}, network),
               cda_rail::exceptions::EdgeNotExistentException);
  EXPECT_THROW(stations.add_track_to_station("Missing", {"v0", "v1"}, network),
               cda_rail::exceptions::StationNotExistentException);
}

TEST(StationList,
     IsFullyInStationDelegatesToStationAndThrowsForUnknownStation) {
  const auto            network = build_linear_test_network();
  const auto            e01     = network.get_edge_index({"v0"}, {"v1"});
  const auto            e12     = network.get_edge_index({"v1"}, {"v2"});
  cda_rail::StationList stations;
  stations.add_empty_station("S");
  stations.add_track_to_station("S", e01);

  EXPECT_TRUE(stations.is_fully_in_station("S", {e01}));
  EXPECT_FALSE(stations.is_fully_in_station("S", {e01, e12}));
  EXPECT_THROW((void)stations.is_fully_in_station("Missing", {e01}),
               cda_rail::exceptions::StationNotExistentException);
}

TEST(StationList, UpdateAfterDiscretizationReplacesMappedTracks) {
  const auto            network = build_linear_test_network();
  const auto            e01     = network.get_edge_index({"v0"}, {"v1"});
  const auto            e12     = network.get_edge_index({"v1"}, {"v2"});
  const auto            e23     = network.get_edge_index({"v2"}, {"v3"});
  cda_rail::StationList stations;
  stations.add_empty_station("S");
  stations.add_track_to_station("S", e01);
  stations.add_track_to_station("S", e12);

  stations.update_after_discretization({{e01, {e23}}});

  const auto& tracks = stations.get_station("S").tracks;
  EXPECT_FALSE(tracks.contains(e01));
  EXPECT_TRUE(tracks.contains(e12));
  EXPECT_TRUE(tracks.contains(e23));
}

TEST(StationList, GetStopTracksForwardsToStationAndThrowsForUnknownStation) {
  const auto            network = build_linear_test_network();
  cda_rail::StationList stations;
  stations.add_empty_station("S");
  stations.add_track_to_station("S", {"v0", "v1"}, network);
  stations.add_track_to_station("S", {"v1", "v2"}, network);

  const auto expected = stations.get_station("S").get_stop_tracks(1.0, network);
  const auto actual   = stations.get_stop_tracks("S", 1.0, network);

  EXPECT_EQ(actual, expected);
  EXPECT_THROW((void)stations.get_stop_tracks("Missing", 1.0, network),
               cda_rail::exceptions::StationNotExistentException);
}

// Old tests

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

  stations.add_track_to_station("S1", {"l0", "l1"}, network);
  stations.add_track_to_station("S1", {"l0", "l1"}, network);
  stations.add_track_to_station("S2", {"l0", "l1"}, network);
  stations.add_track_to_station("S2", {"l1", "l2"}, network);

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
