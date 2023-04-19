#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Timetable.hpp"
#include "Definitions.hpp"
#include "gtest/gtest.h"
#include <algorithm>
#include "nlohmann/json.hpp"
#include <unordered_set>
#include "datastructure/Route.hpp"

using json = nlohmann::json;

struct EdgeTarget {
    std::string source;
    std::string target;
    double length;
    double max_speed;
    bool breakable;
    double min_block_length;
};

TEST(Functionality, ReadNetwork) {
    cda_rail::Network network = cda_rail::Network::import_network("./example-networks/Fig11/network/");

    // Check vertices properties
    std::vector<std::string> vertex_names = {"l0", "l1", "l2", "l3", "r0", "r1", "r2", "g00", "g01", "g10", "g11"};
    std::vector<int> type = {2,2,2,0,2,2,0,2,2,2,2};

    EXPECT_TRUE(network.number_of_vertices() == vertex_names.size());

    for (int i = 0; i < vertex_names.size(); i++) {
        std::string v_name = vertex_names[i];
        cda_rail::Vertex v = network.get_vertex(v_name);
        EXPECT_TRUE(v.name == v_name);
        EXPECT_TRUE(v.type == type[i]);
    }

    // Check edges properties
    std::vector<EdgeTarget> edge_targets;
    edge_targets.push_back({"l0", "l1", 500, 27.77777777777778, true, 0});
    edge_targets.push_back({"l1", "l2", 500, 27.77777777777778, true, 0});
    edge_targets.push_back({"l2", "l3", 5, 27.77777777777778, false, 0});
    edge_targets.push_back({"l3", "g00", 5, 27.77777777777778, false, 0});
    edge_targets.push_back({"l3", "g10", 5, 27.77777777777778, false, 0});
    edge_targets.push_back({"g00", "g01", 300, 27.77777777777778, true, 0});
    edge_targets.push_back({"g10", "g11", 300, 27.77777777777778, true, 0});
    edge_targets.push_back({"g01", "r2", 5, 27.77777777777778, false, 0});
    edge_targets.push_back({"g11", "r2", 5, 27.77777777777778, false, 0});
    edge_targets.push_back({"r2", "r1", 5, 27.77777777777778, false, 0});
    edge_targets.push_back({"r1", "r0", 500, 27.77777777777778, true, 0});
    edge_targets.push_back({"r0", "r1", 500, 27.77777777777778, true, 0});
    edge_targets.push_back({"r1", "r2", 5, 27.77777777777778, false, 0});
    edge_targets.push_back({"r2", "g01", 5, 27.77777777777778, false, 0});
    edge_targets.push_back({"r2", "g11", 5, 27.77777777777778, false, 0});
    edge_targets.push_back({"g01", "g00", 300, 27.77777777777778, true, 0});
    edge_targets.push_back({"g11", "g10", 300, 27.77777777777778, true, 0});
    edge_targets.push_back({"g00", "l3", 5, 27.77777777777778, false, 0});
    edge_targets.push_back({"g10", "l3", 5, 27.77777777777778, false, 0});
    edge_targets.push_back({"l3", "l2", 5, 27.77777777777778, false, 0});
    edge_targets.push_back({"l2", "l1", 500, 27.77777777777778, true, 0});
    edge_targets.push_back({"l1", "l0", 500, 27.77777777777778, true, 0});

    EXPECT_TRUE(network.number_of_edges() == edge_targets.size());
    for (const auto& edge : edge_targets) {
        cda_rail::Edge e = network.get_edge(edge.source, edge.target);
        EXPECT_TRUE(network.get_vertex(e.source).name == edge.source);
        EXPECT_TRUE(network.get_vertex(e.target).name == edge.target);
        EXPECT_TRUE(e.length == edge.length);
        EXPECT_TRUE(e.max_speed == edge.max_speed);
        EXPECT_TRUE(e.breakable == edge.breakable);
        EXPECT_TRUE(e.min_block_length == edge.min_block_length);
    }

    // Check successors
    std::unordered_set<int> successors_target;

    // l0,l1
    successors_target.clear();
    successors_target.insert(network.get_edge_index("l1", "l2"));
    EXPECT_TRUE(network.get_successors("l0", "l1") == successors_target);

    // l1,l2
    successors_target.clear();
    successors_target.insert(network.get_edge_index("l2", "l3"));
    EXPECT_TRUE(network.get_successors("l1", "l2") == successors_target);

    // l2,l3
    successors_target.clear();
    successors_target.insert(network.get_edge_index("l3", "g00"));
    successors_target.insert(network.get_edge_index("l3", "g10"));
    EXPECT_TRUE(network.get_successors("l2", "l3") == successors_target);

    // l3,g00
    successors_target.clear();
    successors_target.insert(network.get_edge_index("g00", "g01"));
    EXPECT_TRUE(network.get_successors("l3", "g00") == successors_target);

    // l3,g10
    successors_target.clear();
    successors_target.insert(network.get_edge_index("g10", "g11"));
    EXPECT_TRUE(network.get_successors("l3", "g10") == successors_target);

    // g00,g01
    successors_target.clear();
    successors_target.insert(network.get_edge_index("g01", "r2"));
    EXPECT_TRUE(network.get_successors("g00", "g01") == successors_target);

    // g10,g11
    successors_target.clear();
    successors_target.insert(network.get_edge_index("g11", "r2"));
    EXPECT_TRUE(network.get_successors("g10", "g11") == successors_target);

    // g01,r2
    successors_target.clear();
    successors_target.insert(network.get_edge_index("r2", "r1"));
    EXPECT_TRUE(network.get_successors("g01", "r2") == successors_target);

    // g11,r2
    successors_target.clear();
    successors_target.insert(network.get_edge_index("r2", "r1"));
    EXPECT_TRUE(network.get_successors("g11", "r2") == successors_target);

    // r2,r1
    successors_target.clear();
    successors_target.insert(network.get_edge_index("r1", "r0"));
    EXPECT_TRUE(network.get_successors("r2", "r1") == successors_target);

    // r1,r0
    successors_target.clear();
    EXPECT_TRUE(network.get_successors("r1", "r0") == successors_target);

    // r0,r1
    successors_target.clear();
    successors_target.insert(network.get_edge_index("r1", "r2"));
    EXPECT_TRUE(network.get_successors("r0", "r1") == successors_target);

    // r1,r2
    successors_target.clear();
    successors_target.insert(network.get_edge_index("r2", "g01"));
    successors_target.insert(network.get_edge_index("r2", "g11"));
    EXPECT_TRUE(network.get_successors("r1", "r2") == successors_target);

    // r2,g01
    successors_target.clear();
    successors_target.insert(network.get_edge_index("g01", "g00"));
    EXPECT_TRUE(network.get_successors("r2", "g01") == successors_target);

    // r2,g11
    successors_target.clear();
    successors_target.insert(network.get_edge_index("g11", "g10"));
    EXPECT_TRUE(network.get_successors("r2", "g11") == successors_target);

    // g01,g00
    successors_target.clear();
    successors_target.insert(network.get_edge_index("g00", "l3"));
    EXPECT_TRUE(network.get_successors("g01", "g00") == successors_target);

    // g11,g10
    successors_target.clear();
    successors_target.insert(network.get_edge_index("g10", "l3"));
    EXPECT_TRUE(network.get_successors("g11", "g10") == successors_target);

    // g00,l3
    successors_target.clear();
    successors_target.insert(network.get_edge_index("l3", "l2"));
    EXPECT_TRUE(network.get_successors("g00", "l3") == successors_target);

    // g10,l3
    successors_target.clear();
    successors_target.insert(network.get_edge_index("l3", "l2"));
    EXPECT_TRUE(network.get_successors("g10", "l3") == successors_target);

    // l3,l2
    successors_target.clear();
    successors_target.insert(network.get_edge_index("l2", "l1"));
    EXPECT_TRUE(network.get_successors("l3", "l2") == successors_target);

    // l2,l1
    successors_target.clear();
    successors_target.insert(network.get_edge_index("l1", "l0"));
    EXPECT_TRUE(network.get_successors("l2", "l1") == successors_target);

    // l1,l0
    successors_target.clear();
    EXPECT_TRUE(network.get_successors("l1", "l0") == successors_target);
}

TEST(Functionality, WriteNetwork) {
    cda_rail::Network network;
    network.add_vertex("v0", 0);
    network.add_vertex("v1", 1);
    network.add_vertex("v2", 2);

    network.add_edge("v0", "v1", 1, 2, true, 0);
    network.add_edge("v1","v2", 3, 4, false, 1.5);
    network.add_edge("v1","v0", 1, 2, true, 0);
    network.add_edge("v2", "v0", 10, 20, false, 2);

    network.add_successor(network.get_edge_index("v0", "v1"), network.get_edge_index("v1", "v2"));
    network.add_successor(network.get_edge_index("v0", "v1"), network.get_edge_index("v1", "v0"));
    network.add_successor(network.get_edge_index("v2","v0"), network.get_edge_index("v0", "v1"));

    network.export_network("./tmp/write_network_test");

    auto network_read = cda_rail::Network::import_network("./tmp/write_network_test");

    // Delete created directory and everything in it
    std::filesystem::remove_all("./tmp");


    // check if both networks are equivalent

    // check vertices
    EXPECT_TRUE(network.number_of_vertices() == network_read.number_of_vertices());
    for (int i = 0; i < network.number_of_vertices(); ++i) {
        EXPECT_TRUE(network_read.has_vertex(network.get_vertex(i).name));
        EXPECT_TRUE(network_read.get_vertex(network.get_vertex(i).name).type == network.get_vertex(i).type);
    }

    // check edges
    EXPECT_TRUE(network.number_of_edges() == network_read.number_of_edges());
    for (int i = 0; i < network.number_of_edges(); ++i) {
        const auto& source_vertex = network.get_vertex(network.get_edge(i).source);
        const auto& target_vertex = network.get_vertex(network.get_edge(i).target);
        EXPECT_TRUE(network_read.has_edge(source_vertex.name, target_vertex.name));
        const auto& edge_read = network_read.get_edge(source_vertex.name, target_vertex.name);
        EXPECT_TRUE(edge_read.breakable == network.get_edge(i).breakable);
        EXPECT_TRUE(edge_read.length == network.get_edge(i).length);
        EXPECT_TRUE(edge_read.max_speed == network.get_edge(i).max_speed);
        EXPECT_TRUE(edge_read.min_block_length == network.get_edge(i).min_block_length);
    }

    // check successors
    for (int i = 0; i < network.number_of_edges(); ++i) {
        const auto& successors_target = network.get_successors(i);
        std::unordered_set<int> successors_target_transformed;
        for (auto successor : successors_target) {
            const auto& e = network.get_edge(successor);
            std::string source = network.get_vertex(e.source).name;
            std::string target = network.get_vertex(e.target).name;
            successors_target_transformed.insert(network_read.get_edge_index(source, target));
        }
        const auto& e = network.get_edge(i);
        std::string source = network.get_vertex(e.source).name;
        std::string target = network.get_vertex(e.target).name;
        EXPECT_TRUE(network_read.get_successors(source, target) == successors_target_transformed);
    }
}

TEST(Functionality, ReadTrains) {
    auto trains = cda_rail::TrainList::import_trains("./example-networks/Fig11/timetable/");

    // Check if the all trains are imported
    EXPECT_TRUE(trains.size() == 3);
    EXPECT_TRUE(trains.has_train("tr1"));
    EXPECT_TRUE(trains.has_train("tr2"));
    EXPECT_TRUE(trains.has_train("tr3"));

    // Check if the train tr1 is imported correctly
    auto tr1 = trains.get_train("tr1");
    EXPECT_TRUE(tr1.name == "tr1");
    EXPECT_TRUE(tr1.length == 100);
    EXPECT_TRUE(tr1.max_speed == 83.33);
    EXPECT_TRUE(tr1.acceleration == 2);
    EXPECT_TRUE(tr1.deceleration == 1);

    // Check if the train tr2 is imported correctly
    auto tr2 = trains.get_train("tr2");
    EXPECT_TRUE(tr2.name == "tr2");
    EXPECT_TRUE(tr2.length == 100);
    EXPECT_TRUE(tr2.max_speed == 27.78);
    EXPECT_TRUE(tr2.acceleration == 2);
    EXPECT_TRUE(tr2.deceleration == 1);

    // Check if the train tr3 is imported correctly
    auto tr3 = trains.get_train("tr3");
    EXPECT_TRUE(tr3.name == "tr3");
    EXPECT_TRUE(tr3.length == 250);
    EXPECT_TRUE(tr3.max_speed == 20);
    EXPECT_TRUE(tr3.acceleration == 2);
    EXPECT_TRUE(tr3.deceleration == 1);
}

TEST(Functionality, WriteTrains) {
    // Create a train list
    auto trains = cda_rail::TrainList();
    trains.add_train("tr1", 100, 83.33, 2, 1);
    trains.add_train("tr2", 100, 27.78, 2, 1);
    trains.add_train("tr3", 250, 20, 2, 1);

    // Write the train list to a file
    trains.export_trains("./tmp/write_trains_test");

    // Read the train list from the file
    auto trains_read = cda_rail::TrainList::import_trains("./tmp/write_trains_test");

    // Delete created directory and everything in it
    std::filesystem::remove_all("./tmp");

    // Check if the all trains are imported
    EXPECT_TRUE(trains_read.size() == 3);
    EXPECT_TRUE(trains_read.has_train("tr1"));
    EXPECT_TRUE(trains_read.has_train("tr2"));
    EXPECT_TRUE(trains_read.has_train("tr3"));

    // Check if the train tr1 is imported correctly
    auto tr1 = trains_read.get_train("tr1");
    EXPECT_TRUE(tr1.name == "tr1");
    EXPECT_TRUE(tr1.length == 100);
    EXPECT_TRUE(tr1.max_speed == 83.33);
    EXPECT_TRUE(tr1.acceleration == 2);
    EXPECT_TRUE(tr1.deceleration == 1);

    // Check if the train tr2 is imported correctly
    auto tr2 = trains_read.get_train("tr2");
    EXPECT_TRUE(tr2.name == "tr2");
    EXPECT_TRUE(tr2.length == 100);
    EXPECT_TRUE(tr2.max_speed == 27.78);
    EXPECT_TRUE(tr2.acceleration == 2);
    EXPECT_TRUE(tr2.deceleration == 1);

    // Check if the train tr3 is imported correctly
    auto tr3 = trains_read.get_train("tr3");
    EXPECT_TRUE(tr3.name == "tr3");
    EXPECT_TRUE(tr3.length == 250);
    EXPECT_TRUE(tr3.max_speed == 20);
    EXPECT_TRUE(tr3.acceleration == 2);
    EXPECT_TRUE(tr3.deceleration == 1);
}

TEST(Functionality, IsDirectory) {
    EXPECT_TRUE(cda_rail::is_directory_and_create("./tmp/is_directory"));
    EXPECT_TRUE(cda_rail::is_directory_and_create("./tmp/is_directory"));
    std::filesystem::remove_all("./tmp");
    EXPECT_TRUE(cda_rail::is_directory_and_create("./tmp/is_directory/"));
    EXPECT_TRUE(cda_rail::is_directory_and_create("./tmp/is_directory/"));
    std::filesystem::remove_all("./tmp");
    EXPECT_TRUE(cda_rail::is_directory_and_create("./tmp/"));
    EXPECT_TRUE(cda_rail::is_directory_and_create("./tmp/"));
    std::filesystem::remove_all("./tmp");
    EXPECT_TRUE(cda_rail::is_directory_and_create(R"(.\tmp\is_directory\)"));
    EXPECT_TRUE(cda_rail::is_directory_and_create(R"(.\tmp\is_directory\)"));
    std::filesystem::remove_all("./tmp");
    EXPECT_TRUE(cda_rail::is_directory_and_create(R"(.\tmp\is_directory)"));
    EXPECT_TRUE(cda_rail::is_directory_and_create(R"(.\tmp\is_directory)"));
    std::filesystem::remove_all("./tmp");
    EXPECT_TRUE(cda_rail::is_directory_and_create(R"(.\tmp\)"));
    EXPECT_TRUE(cda_rail::is_directory_and_create(R"(.\tmp\)"));
    std::filesystem::remove_all("./tmp");
    EXPECT_TRUE(cda_rail::is_directory_and_create(R"(.\tmp)"));
    EXPECT_TRUE(cda_rail::is_directory_and_create(R"(.\tmp)"));
    std::filesystem::remove_all("./tmp");
}

TEST(Functionality, ReadStation) {
    auto network = cda_rail::Network::import_network("./example-networks/Fig11/network/");
    auto stations = cda_rail::StationList::import_stations("./example-networks/Fig11/timetable/", network);

    // Check if the station is imported correctly
    EXPECT_TRUE(stations.size() == 1);
    EXPECT_TRUE(stations.has_station("Central"));

    // Check if the station is imported correctly
    auto& station = stations.get_station("Central");
    EXPECT_TRUE(station.name == "Central");
    EXPECT_TRUE(station.tracks.size() == 4);
    std::unordered_set<int> track_ids{network.get_edge_index("g00", "g01"),
                                      network.get_edge_index("g10", "g11"),
                                      network.get_edge_index("g01", "g00"),
                                      network.get_edge_index("g11", "g10")};
    EXPECT_TRUE(station.tracks == track_ids);
}

TEST(Functionality, WriteStations) {
    auto network = cda_rail::Network::import_network("./example-networks/Fig11/network/");
    cda_rail::StationList stations;

    stations.add_station("S1");
    stations.add_station("S2");

    stations.add_track_to_station("S1", "l0", "l1", network);
    stations.add_track_to_station("S2", "l0", "l1", network);
    stations.add_track_to_station("S2", "l1", "l2", network);

    stations.export_stations("./tmp/write_stations_test", network);
    auto stations_read = cda_rail::StationList::import_stations("./tmp/write_stations_test", network);

    std::filesystem::remove_all("./tmp");

    EXPECT_TRUE(stations_read.size() == 2);
    EXPECT_TRUE(stations_read.has_station("S1"));
    EXPECT_TRUE(stations_read.has_station("S2"));

    auto& s1 = stations_read.get_station("S1");
    EXPECT_TRUE(s1.name == "S1");
    EXPECT_TRUE(s1.tracks.size() == 1);
    std::unordered_set<int> s1_tracks{network.get_edge_index("l0", "l1")};
    EXPECT_TRUE(s1.tracks == s1_tracks);

    auto& s2 = stations_read.get_station("S2");
    EXPECT_TRUE(s2.name == "S2");
    EXPECT_TRUE(s2.tracks.size() == 2);
    std::unordered_set<int> s2_tracks{network.get_edge_index("l0", "l1"),
                                      network.get_edge_index("l1", "l2")};
    EXPECT_TRUE(s2.tracks == s2_tracks);
}

TEST(Functionality, ReadTimetable) {
    auto network = cda_rail::Network::import_network("./example-networks/Fig11/network/");
    auto timetable = cda_rail::Timetable::import_timetable("./example-networks/Fig11/timetable/", network);

    // Check if the timetable has the correct stations
    auto& stations = timetable.get_station_list();
    EXPECT_TRUE(stations.size() == 1);
    EXPECT_TRUE(stations.has_station("Central"));

    // Check if the station is imported correctly
    auto& station = stations.get_station("Central");
    EXPECT_TRUE(station.name == "Central");
    EXPECT_TRUE(station.tracks.size() == 4);
    std::unordered_set<int> track_ids{network.get_edge_index("g00", "g01"),
                                      network.get_edge_index("g10", "g11"),
                                      network.get_edge_index("g01", "g00"),
                                      network.get_edge_index("g11", "g10")};
    EXPECT_TRUE(station.tracks == track_ids);

    // Check if the timetable has the correct trains
    auto& trains = timetable.get_train_list();
    // Check if the all trains are imported
    EXPECT_TRUE(trains.size() == 3);
    EXPECT_TRUE(trains.has_train("tr1"));
    EXPECT_TRUE(trains.has_train("tr2"));
    EXPECT_TRUE(trains.has_train("tr3"));
    // Check if the train tr1 is imported correctly
    auto tr1 = trains.get_train("tr1");
    EXPECT_TRUE(tr1.name == "tr1");
    EXPECT_TRUE(tr1.length == 100);
    EXPECT_TRUE(tr1.max_speed == 83.33);
    EXPECT_TRUE(tr1.acceleration == 2);
    EXPECT_TRUE(tr1.deceleration == 1);
    // Check if the train tr2 is imported correctly
    auto tr2 = trains.get_train("tr2");
    EXPECT_TRUE(tr2.name == "tr2");
    EXPECT_TRUE(tr2.length == 100);
    EXPECT_TRUE(tr2.max_speed == 27.78);
    EXPECT_TRUE(tr2.acceleration == 2);
    EXPECT_TRUE(tr2.deceleration == 1);
    // Check if the train tr3 is imported correctly
    auto tr3 = trains.get_train("tr3");
    EXPECT_TRUE(tr3.name == "tr3");
    EXPECT_TRUE(tr3.length == 250);
    EXPECT_TRUE(tr3.max_speed == 20);
    EXPECT_TRUE(tr3.acceleration == 2);
    EXPECT_TRUE(tr3.deceleration == 1);

    // Check the schedule of tr1
    auto& tr1_schedule = timetable.get_schedule("tr1");
    EXPECT_TRUE(tr1_schedule.t_0 == 120);
    EXPECT_TRUE(tr1_schedule.v_0 == 0);
    EXPECT_TRUE(tr1_schedule.t_n == 640);
    EXPECT_TRUE(tr1_schedule.v_n == 16.67);
    EXPECT_TRUE(network.get_vertex(tr1_schedule.entry).name == "l0");
    EXPECT_TRUE(network.get_vertex(tr1_schedule.exit).name == "r0");
    EXPECT_TRUE(tr1_schedule.stops.size() == 1);
    auto& stop = tr1_schedule.stops[0];
    EXPECT_TRUE(stop.begin == 240);
    EXPECT_TRUE(stop.end == 300);
    EXPECT_TRUE(stations.get_station(stop.station).name == "Central");

    // Check the schedule of tr2
    auto& tr2_schedule = timetable.get_schedule("tr2");
    EXPECT_TRUE(tr2_schedule.t_0 == 0);
    EXPECT_TRUE(tr2_schedule.v_0 == 0);
    EXPECT_TRUE(tr2_schedule.t_n == 420);
    EXPECT_TRUE(tr2_schedule.v_n == 16.67);
    EXPECT_TRUE(network.get_vertex(tr2_schedule.entry).name == "l0");
    EXPECT_TRUE(network.get_vertex(tr2_schedule.exit).name == "r0");
    EXPECT_TRUE(tr2_schedule.stops.size() == 1);
    auto& stop2 = tr2_schedule.stops[0];
    EXPECT_TRUE(stop2.begin == 120);
    EXPECT_TRUE(stop2.end == 300);
    EXPECT_TRUE(stations.get_station(stop2.station).name == "Central");

    // Check the schedule of tr3
    auto& tr3_schedule = timetable.get_schedule("tr3");
    EXPECT_TRUE(tr3_schedule.t_0 == 0);
    EXPECT_TRUE(tr3_schedule.v_0 == 0);
    EXPECT_TRUE(tr3_schedule.t_n == 420);
    EXPECT_TRUE(tr3_schedule.v_n == 16.67);
    EXPECT_TRUE(network.get_vertex(tr3_schedule.entry).name == "r0");
    EXPECT_TRUE(network.get_vertex(tr3_schedule.exit).name == "l0");
    EXPECT_TRUE(tr3_schedule.stops.size() == 1);
    auto& stop3 = tr3_schedule.stops[0];
    EXPECT_TRUE(stop3.begin == 180);
    EXPECT_TRUE(stop3.end == 300);
    EXPECT_TRUE(stations.get_station(stop3.station).name == "Central");
}

TEST(Functionality, WriteTimetable) {
    auto network = cda_rail::Network::import_network("./example-networks/Fig11/network/");
    cda_rail::Timetable timetable;

    timetable.add_train("tr1", 100, 83.33, 2, 1,
                        0, 0, "l0",
                        300, 20, "r0",
                        network);
    timetable.add_train("tr2", 100, 27.78, 2, 1,
                        0, 0, "r0",
                        300, 20, "l0",
                        network);

    timetable.add_station("Station1");
    timetable.add_station("Station2");

    timetable.add_track_to_station("Station1", "g00", "g01", network);
    timetable.add_track_to_station("Station1", "g10", "g11", network);
    timetable.add_track_to_station("Station1", "g01", "g00", network);
    timetable.add_track_to_station("Station1", "g11", "g10", network);
    timetable.add_track_to_station("Station2", "r1", "r0", network);

    timetable.add_stop("tr1", "Station1", 100, 160);
    timetable.add_stop("tr1", "Station2", 200, 260);
    timetable.add_stop("tr2", "Station1", 160, 220);

    // Check if the timetable is as expected
    // Check if the timetable has the correct stations
    auto& stations = timetable.get_station_list();
    EXPECT_TRUE(stations.size() == 2);
    EXPECT_TRUE(stations.has_station("Station1"));
    EXPECT_TRUE(stations.has_station("Station2"));

    // Check if the stations are imported correctly
    auto& st1 = stations.get_station("Station1");
    EXPECT_TRUE(st1.name == "Station1");
    EXPECT_TRUE(st1.tracks.size() == 4);
    std::unordered_set<int> s1_expected_tracks = {network.get_edge_index("g00", "g01"),
                                                 network.get_edge_index("g10", "g11"),
                                                 network.get_edge_index("g01", "g00"),
                                                 network.get_edge_index("g11", "g10")};
    EXPECT_TRUE(st1.tracks == s1_expected_tracks);
    auto& st2 = stations.get_station("Station2");
    EXPECT_TRUE(st2.name == "Station2");
    EXPECT_TRUE(st2.tracks.size() == 1);
    std::unordered_set<int> s2_expected_tracks = {network.get_edge_index("r1", "r0")};
    EXPECT_TRUE(st2.tracks == s2_expected_tracks);

    // Check if the timetable has the correct trains
    auto& trains = timetable.get_train_list();
    EXPECT_TRUE(trains.size() == 2);
    EXPECT_TRUE(trains.has_train("tr1"));
    EXPECT_TRUE(trains.has_train("tr2"));

    // Check if the train tr1 is saved correctly
    auto tr1 = trains.get_train("tr1");
    EXPECT_TRUE(tr1.name == "tr1");
    EXPECT_TRUE(tr1.length == 100);
    EXPECT_TRUE(tr1.max_speed == 83.33);
    EXPECT_TRUE(tr1.acceleration == 2);
    EXPECT_TRUE(tr1.deceleration == 1);
    // Check if the train tr2 is saved correctly
    auto tr2 = trains.get_train("tr2");
    EXPECT_TRUE(tr2.name == "tr2");
    EXPECT_TRUE(tr2.length == 100);
    EXPECT_TRUE(tr2.max_speed == 27.78);
    EXPECT_TRUE(tr2.acceleration == 2);
    EXPECT_TRUE(tr2.deceleration == 1);

    // Check if the schedule of tr1 is saved correctly
    auto& tr1_schedule = timetable.get_schedule("tr1");
    EXPECT_TRUE(tr1_schedule.t_0 == 0);
    EXPECT_TRUE(tr1_schedule.v_0 == 0);
    EXPECT_TRUE(tr1_schedule.t_n == 300);
    EXPECT_TRUE(tr1_schedule.v_n == 20);
    EXPECT_TRUE(network.get_vertex(tr1_schedule.entry).name == "l0");
    EXPECT_TRUE(network.get_vertex(tr1_schedule.exit).name == "r0");
    EXPECT_TRUE(tr1_schedule.stops.size() == 2);
    auto& stop1 = tr1_schedule.stops[0];
    EXPECT_TRUE(stop1.begin == 100);
    EXPECT_TRUE(stop1.end == 160);
    EXPECT_TRUE(stations.get_station(stop1.station).name == "Station1");
    auto& stop2 = tr1_schedule.stops[1];
    EXPECT_TRUE(stop2.begin == 200);
    EXPECT_TRUE(stop2.end == 260);
    EXPECT_TRUE(stations.get_station(stop2.station).name == "Station2");

    // Check if the schedule of tr2 is saved correctly
    auto& tr2_schedule = timetable.get_schedule("tr2");
    EXPECT_TRUE(tr2_schedule.t_0 == 0);
    EXPECT_TRUE(tr2_schedule.v_0 == 0);
    EXPECT_TRUE(tr2_schedule.t_n == 300);
    EXPECT_TRUE(tr2_schedule.v_n == 20);
    EXPECT_TRUE(network.get_vertex(tr2_schedule.entry).name == "r0");
    EXPECT_TRUE(network.get_vertex(tr2_schedule.exit).name == "l0");
    EXPECT_TRUE(tr2_schedule.stops.size() == 1);
    auto& stop3 = tr2_schedule.stops[0];
    EXPECT_TRUE(stop3.begin == 160);
    EXPECT_TRUE(stop3.end == 220);
    EXPECT_TRUE(stations.get_station(stop3.station).name == "Station1");

    // Write timetable to directory
    timetable.export_timetable("./tmp/test-timetable/", network);

    // Read timetable from directory
    auto timetable_read = cda_rail::Timetable::import_timetable("./tmp/test-timetable/", network);

    // Delete temporary files
    std::filesystem::remove_all("./tmp");

    // Check if the timetable is as expected
    // Check if the timetable has the correct stations
    auto& stations_read = timetable_read.get_station_list();
    EXPECT_TRUE(stations_read.size() == 2);
    EXPECT_TRUE(stations_read.has_station("Station1"));
    EXPECT_TRUE(stations_read.has_station("Station2"));

    // Check if the stations are imported correctly
    auto& st1_read = stations_read.get_station("Station1");
    EXPECT_TRUE(st1_read.name == "Station1");
    EXPECT_TRUE(st1_read.tracks.size() == 4);
    EXPECT_TRUE(st1_read.tracks == s1_expected_tracks);
    auto& st2_read = stations_read.get_station("Station2");
    EXPECT_TRUE(st2_read.name == "Station2");
    EXPECT_TRUE(st2_read.tracks.size() == 1);
    EXPECT_TRUE(st2_read.tracks == s2_expected_tracks);

    // Check if the timetable has the correct trains
    auto& trains_read = timetable_read.get_train_list();
    EXPECT_TRUE(trains_read.size() == 2);
    EXPECT_TRUE(trains_read.has_train("tr1"));
    EXPECT_TRUE(trains_read.has_train("tr2"));

    // Check if the train tr1 is saved correctly
    auto tr1_read = trains_read.get_train("tr1");
    EXPECT_TRUE(tr1_read.name == "tr1");
    EXPECT_TRUE(tr1_read.length == 100);
    EXPECT_TRUE(tr1_read.max_speed == 83.33);
    EXPECT_TRUE(tr1_read.acceleration == 2);
    EXPECT_TRUE(tr1_read.deceleration == 1);
    // Check if the train tr2 is saved correctly
    auto tr2_read = trains_read.get_train("tr2");
    EXPECT_TRUE(tr2_read.name == "tr2");
    EXPECT_TRUE(tr2_read.length == 100);
    EXPECT_TRUE(tr2_read.max_speed == 27.78);
    EXPECT_TRUE(tr2_read.acceleration == 2);
    EXPECT_TRUE(tr2_read.deceleration == 1);

    // Check if the schedule of tr1 is saved correctly
    auto& tr1_schedule_read = timetable_read.get_schedule("tr1");
    EXPECT_TRUE(tr1_schedule_read.t_0 == 0);
    EXPECT_TRUE(tr1_schedule_read.v_0 == 0);
    EXPECT_TRUE(tr1_schedule_read.t_n == 300);
    EXPECT_TRUE(tr1_schedule_read.v_n == 20);
    EXPECT_TRUE(network.get_vertex(tr1_schedule_read.entry).name == "l0");
    EXPECT_TRUE(network.get_vertex(tr1_schedule_read.exit).name == "r0");
    EXPECT_TRUE(tr1_schedule_read.stops.size() == 2);
    auto& stop1_read = tr1_schedule_read.stops[0];
    EXPECT_TRUE(stop1_read.begin == 100);
    EXPECT_TRUE(stop1_read.end == 160);
    EXPECT_TRUE(stations_read.get_station(stop1_read.station).name == "Station1");
    auto& stop2_read = tr1_schedule_read.stops[1];
    EXPECT_TRUE(stop2_read.begin == 200);
    EXPECT_TRUE(stop2_read.end == 260);
    EXPECT_TRUE(stations_read.get_station(stop2_read.station).name == "Station2");

    // Check if the schedule of tr2 is saved correctly
    auto& tr2_schedule_read = timetable_read.get_schedule("tr2");
    EXPECT_TRUE(tr2_schedule_read.t_0 == 0);
    EXPECT_TRUE(tr2_schedule_read.v_0 == 0);
    EXPECT_TRUE(tr2_schedule_read.t_n == 300);
    EXPECT_TRUE(tr2_schedule_read.v_n == 20);
    EXPECT_TRUE(network.get_vertex(tr2_schedule_read.entry).name == "r0");
    EXPECT_TRUE(network.get_vertex(tr2_schedule_read.exit).name == "l0");
    EXPECT_TRUE(tr2_schedule_read.stops.size() == 1);
    auto& stop3_read = tr2_schedule_read.stops[0];
    EXPECT_TRUE(stop3_read.begin == 160);
    EXPECT_TRUE(stop3_read.end == 220);
    EXPECT_TRUE(stations_read.get_station(stop3_read.station).name == "Station1");
}


TEST(Functionality, RouteMap) {
    auto network = cda_rail::Network::import_network("./example-networks/Fig11/network/");
    auto train_list = cda_rail::TrainList();

    train_list.add_train("tr1", 100, 83.33, 2, 1);
    train_list.add_train("tr2", 100, 27.78, 2, 1);

    auto route_map = cda_rail::RouteMap();

    EXPECT_ANY_THROW(route_map.add_empty_route("tr3", train_list));

    route_map.add_empty_route("tr1", train_list);
    route_map.push_back_edge("tr1", "l1", "l2", network);
    EXPECT_ANY_THROW(route_map.push_back_edge("tr1", "l0", "l2", network));
    EXPECT_ANY_THROW(route_map.push_back_edge("tr1", "l0", "l1", network));
    route_map.push_back_edge("tr1", "l2", "l3", network);
    EXPECT_ANY_THROW(route_map.push_front_edge("tr1", "l0", "l2", network));
    EXPECT_ANY_THROW(route_map.push_front_edge("tr1", "l3", "g00", network));
    route_map.push_front_edge("tr1", "l0", "l1", network);

    // Check if route consists of three edges passing vertices l0-l1-l2-l3 in this order.
    auto& route = route_map.get_route("tr1");
    EXPECT_TRUE(route.size() == 3);
    EXPECT_TRUE(network.get_vertex(route.get_edge(0, network).source).name == "l0");
    EXPECT_TRUE(network.get_vertex(route.get_edge(0, network).target).name == "l1");
    EXPECT_TRUE(network.get_vertex(route.get_edge(1, network).source).name == "l1");
    EXPECT_TRUE(network.get_vertex(route.get_edge(1, network).target).name == "l2");
    EXPECT_TRUE(network.get_vertex(route.get_edge(2, network).source).name == "l2");
    EXPECT_TRUE(network.get_vertex(route.get_edge(2, network).target).name == "l3");

    // Check if the consistency checking works as expected
    EXPECT_TRUE(route_map.check_consistency(train_list, network, false));
    EXPECT_FALSE(route_map.check_consistency(train_list, network, true));
    EXPECT_FALSE(route_map.check_consistency(train_list, network));

    route_map.add_empty_route("tr2");
    route_map.push_back_edge("tr2", "r0", "r1", network);
    route_map.push_back_edge("tr2", "r1", "r2", network);

    // Check if route consists of two edges passing vertices r0-r1-r2 in this order.
    auto& route2 = route_map.get_route("tr2");
    EXPECT_TRUE(route2.size() == 2);
    EXPECT_TRUE(network.get_vertex(route2.get_edge(0, network).source).name == "r0");
    EXPECT_TRUE(network.get_vertex(route2.get_edge(0, network).target).name == "r1");
    EXPECT_TRUE(network.get_vertex(route2.get_edge(1, network).source).name == "r1");
    EXPECT_TRUE(network.get_vertex(route2.get_edge(1, network).target).name == "r2");

    // Check if the consistency checking works as expected
    EXPECT_TRUE(route_map.check_consistency(train_list, network, false));
    EXPECT_TRUE(route_map.check_consistency(train_list, network, true));
    EXPECT_TRUE(route_map.check_consistency(train_list, network));
}

TEST(Functionality, ImportRouteMap) {
    auto network = cda_rail::Network::import_network("./example-networks/Fig11/network/");
    auto train_list = cda_rail::TrainList::import_trains("./example-networks/Fig11/timetable/");
    auto route_map = cda_rail::RouteMap::import_routes("./example-networks/Fig11/routes/", network);

    // Check if the route consists of thee trains with names "tr1", "tr2" and "tr3"
    EXPECT_TRUE(route_map.size() == 3);
    EXPECT_TRUE(route_map.has_route("tr1"));
    EXPECT_TRUE(route_map.has_route("tr2"));
    EXPECT_TRUE(route_map.has_route("tr3"));

    // Check if the route for tr1 consists of eight edges passing vertices l0-l1-l2-l3-g00-g01-r2-r1-r0 in this order.
    auto& route = route_map.get_route("tr1");
    EXPECT_TRUE(route.size() == 8);
    EXPECT_TRUE(network.get_vertex(route.get_edge(0, network).source).name == "l0");
    EXPECT_TRUE(network.get_vertex(route.get_edge(0, network).target).name == "l1");
    EXPECT_TRUE(network.get_vertex(route.get_edge(1, network).source).name == "l1");
    EXPECT_TRUE(network.get_vertex(route.get_edge(1, network).target).name == "l2");
    EXPECT_TRUE(network.get_vertex(route.get_edge(2, network).source).name == "l2");
    EXPECT_TRUE(network.get_vertex(route.get_edge(2, network).target).name == "l3");
    EXPECT_TRUE(network.get_vertex(route.get_edge(3, network).source).name == "l3");
    EXPECT_TRUE(network.get_vertex(route.get_edge(3, network).target).name == "g00");
    EXPECT_TRUE(network.get_vertex(route.get_edge(4, network).source).name == "g00");
    EXPECT_TRUE(network.get_vertex(route.get_edge(4, network).target).name == "g01");
    EXPECT_TRUE(network.get_vertex(route.get_edge(5, network).source).name == "g01");
    EXPECT_TRUE(network.get_vertex(route.get_edge(5, network).target).name == "r2");
    EXPECT_TRUE(network.get_vertex(route.get_edge(6, network).source).name == "r2");
    EXPECT_TRUE(network.get_vertex(route.get_edge(6, network).target).name == "r1");
    EXPECT_TRUE(network.get_vertex(route.get_edge(7, network).source).name == "r1");
    EXPECT_TRUE(network.get_vertex(route.get_edge(7, network).target).name == "r0");

    // Check if the route for tr2 consists of eight edges passing vertices l0-l1-l2-l3-g00-g01-r2-r1-r0 in this order.
    auto& route2 = route_map.get_route("tr2");
    EXPECT_TRUE(route2.size() == 8);
    EXPECT_TRUE(network.get_vertex(route2.get_edge(0, network).source).name == "l0");
    EXPECT_TRUE(network.get_vertex(route2.get_edge(0, network).target).name == "l1");
    EXPECT_TRUE(network.get_vertex(route2.get_edge(1, network).source).name == "l1");
    EXPECT_TRUE(network.get_vertex(route2.get_edge(1, network).target).name == "l2");
    EXPECT_TRUE(network.get_vertex(route2.get_edge(2, network).source).name == "l2");
    EXPECT_TRUE(network.get_vertex(route2.get_edge(2, network).target).name == "l3");
    EXPECT_TRUE(network.get_vertex(route2.get_edge(3, network).source).name == "l3");
    EXPECT_TRUE(network.get_vertex(route2.get_edge(3, network).target).name == "g00");
    EXPECT_TRUE(network.get_vertex(route2.get_edge(4, network).source).name == "g00");
    EXPECT_TRUE(network.get_vertex(route2.get_edge(4, network).target).name == "g01");
    EXPECT_TRUE(network.get_vertex(route2.get_edge(5, network).source).name == "g01");
    EXPECT_TRUE(network.get_vertex(route2.get_edge(5, network).target).name == "r2");
    EXPECT_TRUE(network.get_vertex(route2.get_edge(6, network).source).name == "r2");
    EXPECT_TRUE(network.get_vertex(route2.get_edge(6, network).target).name == "r1");
    EXPECT_TRUE(network.get_vertex(route2.get_edge(7, network).source).name == "r1");
    EXPECT_TRUE(network.get_vertex(route2.get_edge(7, network).target).name == "r0");

    // Check if the route for tr3 consists of eight edges passing vertices r0-r1-r2-g11-g10-l3-l2-l1 in this order.
    auto& route3 = route_map.get_route("tr3");
    EXPECT_TRUE(route3.size() == 8);
    EXPECT_TRUE(network.get_vertex(route3.get_edge(0, network).source).name == "r0");
    EXPECT_TRUE(network.get_vertex(route3.get_edge(0, network).target).name == "r1");
    EXPECT_TRUE(network.get_vertex(route3.get_edge(1, network).source).name == "r1");
    EXPECT_TRUE(network.get_vertex(route3.get_edge(1, network).target).name == "r2");
    EXPECT_TRUE(network.get_vertex(route3.get_edge(2, network).source).name == "r2");
    EXPECT_TRUE(network.get_vertex(route3.get_edge(2, network).target).name == "g11");
    EXPECT_TRUE(network.get_vertex(route3.get_edge(3, network).source).name == "g11");
    EXPECT_TRUE(network.get_vertex(route3.get_edge(3, network).target).name == "g10");
    EXPECT_TRUE(network.get_vertex(route3.get_edge(4, network).source).name == "g10");
    EXPECT_TRUE(network.get_vertex(route3.get_edge(4, network).target).name == "l3");
    EXPECT_TRUE(network.get_vertex(route3.get_edge(5, network).source).name == "l3");
    EXPECT_TRUE(network.get_vertex(route3.get_edge(5, network).target).name == "l2");
    EXPECT_TRUE(network.get_vertex(route3.get_edge(6, network).source).name == "l2");
    EXPECT_TRUE(network.get_vertex(route3.get_edge(6, network).target).name == "l1");
    EXPECT_TRUE(network.get_vertex(route3.get_edge(7, network).source).name == "l1");
    EXPECT_TRUE(network.get_vertex(route3.get_edge(7, network).target).name == "l0");


    // Check imported consistency
    EXPECT_TRUE(route_map.check_consistency(train_list, network, false));
    EXPECT_TRUE(route_map.check_consistency(train_list, network, true));
    EXPECT_TRUE(route_map.check_consistency(train_list, network));
}

TEST(Functionality, ExportRouteMap) {
    auto network = cda_rail::Network::import_network("./example-networks/Fig11/network/");
    auto train_list = cda_rail::TrainList();
    train_list.add_train("tr1", 100, 83.33, 2, 1);
    train_list.add_train("tr2", 100, 27.78, 2, 1);
    auto route_map = cda_rail::RouteMap();
    route_map.add_empty_route("tr1", train_list);
    route_map.push_back_edge("tr1", "l1", "l2", network);
    route_map.push_back_edge("tr1", "l2", "l3", network);
    route_map.push_front_edge("tr1", "l0", "l1", network);
    route_map.add_empty_route("tr2");
    route_map.push_back_edge("tr2", "r0", "r1", network);
    route_map.push_back_edge("tr2", "r1", "r2", network);

    // Export and import route map
    route_map.export_routes("./tmp/write_route_map_test", network);
    auto route_map_read = cda_rail::RouteMap::import_routes("./tmp/write_route_map_test", network);
    std::filesystem::remove_all("./tmp");

    // Check if the route map is the same as the original one
    // Check if the route map contains two routes for tr1 and tr2
    EXPECT_TRUE(route_map_read.size() == 2);
    EXPECT_TRUE(route_map_read.has_route("tr1"));
    EXPECT_TRUE(route_map_read.has_route("tr2"));

    // Check if the route for tr1 consists of three edges passing vertices l0-l1-l2-l3 in this order.
    auto& route1 = route_map_read.get_route("tr1");
    EXPECT_TRUE(route1.size() == 3);
    EXPECT_TRUE(network.get_vertex(route1.get_edge(0, network).source).name == "l0");
    EXPECT_TRUE(network.get_vertex(route1.get_edge(0, network).target).name == "l1");
    EXPECT_TRUE(network.get_vertex(route1.get_edge(1, network).source).name == "l1");
    EXPECT_TRUE(network.get_vertex(route1.get_edge(1, network).target).name == "l2");
    EXPECT_TRUE(network.get_vertex(route1.get_edge(2, network).source).name == "l2");
    EXPECT_TRUE(network.get_vertex(route1.get_edge(2, network).target).name == "l3");

    // Check if the route for tr2 consists of two edges passing vertices r0-r1-r2 in this order.
    auto& route2 = route_map_read.get_route("tr2");
    EXPECT_TRUE(route2.size() == 2);
    EXPECT_TRUE(network.get_vertex(route2.get_edge(0, network).source).name == "r0");
    EXPECT_TRUE(network.get_vertex(route2.get_edge(0, network).target).name == "r1");
    EXPECT_TRUE(network.get_vertex(route2.get_edge(1, network).source).name == "r1");
    EXPECT_TRUE(network.get_vertex(route2.get_edge(1, network).target).name == "r2");

    // Check imported consistency
    EXPECT_TRUE(route_map_read.check_consistency(train_list, network, false));
    EXPECT_TRUE(route_map_read.check_consistency(train_list, network, true));
    EXPECT_TRUE(route_map_read.check_consistency(train_list, network));
}