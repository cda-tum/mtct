#include "probleminstances/VSSGenerationTimetable.hpp"
#include "gtest/gtest.h"
#include "Definitions.hpp"

struct EdgeTarget {
    std::string source;
    std::string target;
    double length;
    double max_speed;
    bool breakable;
    double min_block_length;
};

TEST(Functionality, VSSGenerationTimetabbleInstanceImport) {
    auto instance = cda_rail::instances::VSSGenerationTimetable::import_instance("./example-networks/Fig11/");

    // Expected network
    const auto& network = instance.n();

    // Check vertices properties
    std::vector<std::string> vertex_names = {"l0", "l1", "l2", "l3", "r0", "r1", "r2", "g00", "g01", "g10", "g11"};
    std::vector<cda_rail::VertexType> type = {cda_rail::VertexType::TTD,
                                              cda_rail::VertexType::TTD,
                                              cda_rail::VertexType::TTD,
                                              cda_rail::VertexType::NO_BORDER,
                                              cda_rail::VertexType::TTD,
                                              cda_rail::VertexType::TTD,
                                              cda_rail::VertexType::NO_BORDER,
                                              cda_rail::VertexType::TTD,
                                              cda_rail::VertexType::TTD,
                                              cda_rail::VertexType::TTD,
                                              cda_rail::VertexType::TTD};

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
    std::vector<int> successors_target;

    // l0,l1
    successors_target.clear();
    successors_target.emplace_back(network.get_edge_index("l1", "l2"));
    EXPECT_TRUE(network.get_successors("l0", "l1") == successors_target);

    // l1,l2
    successors_target.clear();
    successors_target.emplace_back(network.get_edge_index("l2", "l3"));
    EXPECT_TRUE(network.get_successors("l1", "l2") == successors_target);

    // l2,l3
    successors_target.clear();
    successors_target.emplace_back(network.get_edge_index("l3", "g00"));
    successors_target.emplace_back(network.get_edge_index("l3", "g10"));
    auto successors_l2_l3 = network.get_successors("l2", "l3");
    std::sort(successors_target.begin(), successors_target.end());
    std::sort(successors_l2_l3.begin(), successors_l2_l3.end());
    EXPECT_TRUE(successors_l2_l3 == successors_target);

    // l3,g00
    successors_target.clear();
    successors_target.emplace_back(network.get_edge_index("g00", "g01"));
    EXPECT_TRUE(network.get_successors("l3", "g00") == successors_target);

    // l3,g10
    successors_target.clear();
    successors_target.emplace_back(network.get_edge_index("g10", "g11"));
    EXPECT_TRUE(network.get_successors("l3", "g10") == successors_target);

    // g00,g01
    successors_target.clear();
    successors_target.emplace_back(network.get_edge_index("g01", "r2"));
    EXPECT_TRUE(network.get_successors("g00", "g01") == successors_target);

    // g10,g11
    successors_target.clear();
    successors_target.emplace_back(network.get_edge_index("g11", "r2"));
    EXPECT_TRUE(network.get_successors("g10", "g11") == successors_target);

    // g01,r2
    successors_target.clear();
    successors_target.emplace_back(network.get_edge_index("r2", "r1"));
    EXPECT_TRUE(network.get_successors("g01", "r2") == successors_target);

    // g11,r2
    successors_target.clear();
    successors_target.emplace_back(network.get_edge_index("r2", "r1"));
    EXPECT_TRUE(network.get_successors("g11", "r2") == successors_target);

    // r2,r1
    successors_target.clear();
    successors_target.emplace_back(network.get_edge_index("r1", "r0"));
    EXPECT_TRUE(network.get_successors("r2", "r1") == successors_target);

    // r1,r0
    successors_target.clear();
    EXPECT_TRUE(network.get_successors("r1", "r0") == successors_target);

    // r0,r1
    successors_target.clear();
    successors_target.emplace_back(network.get_edge_index("r1", "r2"));
    EXPECT_TRUE(network.get_successors("r0", "r1") == successors_target);

    // r1,r2
    successors_target.clear();
    successors_target.emplace_back(network.get_edge_index("r2", "g01"));
    successors_target.emplace_back(network.get_edge_index("r2", "g11"));
    auto successors_r1_r2 = network.get_successors("r1", "r2");
    std::sort(successors_target.begin(), successors_target.end());
    std::sort(successors_r1_r2.begin(), successors_r1_r2.end());
    EXPECT_TRUE(successors_r1_r2 == successors_target);

    // r2,g01
    successors_target.clear();
    successors_target.emplace_back(network.get_edge_index("g01", "g00"));
    EXPECT_TRUE(network.get_successors("r2", "g01") == successors_target);

    // r2,g11
    successors_target.clear();
    successors_target.emplace_back(network.get_edge_index("g11", "g10"));
    EXPECT_TRUE(network.get_successors("r2", "g11") == successors_target);

    // g01,g00
    successors_target.clear();
    successors_target.emplace_back(network.get_edge_index("g00", "l3"));
    EXPECT_TRUE(network.get_successors("g01", "g00") == successors_target);

    // g11,g10
    successors_target.clear();
    successors_target.emplace_back(network.get_edge_index("g10", "l3"));
    EXPECT_TRUE(network.get_successors("g11", "g10") == successors_target);

    // g00,l3
    successors_target.clear();
    successors_target.emplace_back(network.get_edge_index("l3", "l2"));
    EXPECT_TRUE(network.get_successors("g00", "l3") == successors_target);

    // g10,l3
    successors_target.clear();
    successors_target.emplace_back(network.get_edge_index("l3", "l2"));
    EXPECT_TRUE(network.get_successors("g10", "l3") == successors_target);

    // l3,l2
    successors_target.clear();
    successors_target.emplace_back(network.get_edge_index("l2", "l1"));
    EXPECT_TRUE(network.get_successors("l3", "l2") == successors_target);

    // l2,l1
    successors_target.clear();
    successors_target.emplace_back(network.get_edge_index("l1", "l0"));
    EXPECT_TRUE(network.get_successors("l2", "l1") == successors_target);

    // l1,l0
    successors_target.clear();
    EXPECT_TRUE(network.get_successors("l1", "l0") == successors_target);

    // Check timetable
    const auto& stations = instance.get_station_list();
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

    const auto& trains = instance.get_train_list();
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
    auto& tr1_schedule = instance.get_schedule("tr1");
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
    auto& tr2_schedule = instance.get_schedule("tr2");
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
    auto& tr3_schedule = instance.get_schedule("tr3");
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

    // Check the route  map
    // Check if the route consists of three trains with names "tr1", "tr2" and "tr3"
    EXPECT_TRUE(instance.route_map_size() == 3);
    EXPECT_TRUE(instance.has_route("tr1"));
    EXPECT_TRUE(instance.has_route("tr2"));
    EXPECT_TRUE(instance.has_route("tr3"));

    // Check if the route for tr1 consists of eight edges passing vertices l0-l1-l2-l3-g00-g01-r2-r1-r0 in this order.
    auto& route = instance.get_route("tr1");
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
    auto& route2 = instance.get_route("tr2");
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
    auto& route3 = instance.get_route("tr3");
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

    // Check consistency
    EXPECT_TRUE(instance.check_consistency());
    EXPECT_TRUE(instance.check_consistency(true));
    EXPECT_TRUE(instance.check_consistency(false));
}

TEST(Functionality, VSSGenerationTimetableExport) {
    cda_rail::instances::VSSGenerationTimetable instance;

    // Add a simple network to the instance
    instance.n().add_vertex("v0", cda_rail::VertexType::TTD);
    instance.n().add_vertex("v1", cda_rail::VertexType::VSS);
    instance.n().add_vertex("v2", cda_rail::VertexType::NO_BORDER);

    instance.n().add_edge("v0", "v1", 100, 10, true, 10);
    instance.n().add_edge("v1", "v2", 200, 20, false);
    instance.n().add_edge("v1", "v0", 100, 10, true, 10);
    instance.n().add_edge("v2", "v1", 200, 20, false);

    instance.n().add_successor({"v0", "v1"}, {"v1", "v2"});
    instance.n().add_successor({"v2", "v1"}, {"v1", "v0"});

    // Add a simple timetable to the instance
    instance.add_train("tr1", 50, 10, 2, 2, 0, 0, "v0", 600, 5, "v2");
    instance.add_station("s0");
    instance.add_track_to_station("s0", "v0", "v1");
    instance.add_station("s1");
    instance.add_track_to_station("s1", "v1", "v2");
    instance.add_track_to_station("s1", "v2", "v1");
    instance.add_stop("tr1", "s1", 200, 260);
    instance.add_stop("tr1", "s0", 60, 120);

    EXPECT_TRUE(instance.get_station_list().get_station(instance.get_schedule("tr1").stops.at(0).station).name == "s0");

    // Add route to instance
    instance.add_empty_route("tr1");
    instance.push_back_edge_to_route("tr1", "v0", "v1");
    instance.push_back_edge_to_route("tr1", "v1", "v2");

    // Check for consistency
    EXPECT_TRUE(instance.check_consistency());

    // Export the instance
    instance.export_instance("./tmp/vss_generation_timetable_export_test");

    // Import the instance and delete tmp folder
    auto instance_read = cda_rail::instances::VSSGenerationTimetable::import_instance("./tmp/vss_generation_timetable_export_test");
    std::filesystem::remove_all("./tmp");

    // Check if the imported instance is still consistent
    EXPECT_TRUE(instance_read.check_consistency());

    // Check if the imported instance is the same as the original instance
    // check vertices
    const auto& network = instance.n();
    const auto& network_read = instance_read.n();
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
        std::vector<int> successors_target_transformed;
        for (auto successor : successors_target) {
            const auto& e = network.get_edge(successor);
            std::string source = network.get_vertex(e.source).name;
            std::string target = network.get_vertex(e.target).name;
            successors_target_transformed.emplace_back(network_read.get_edge_index(source, target));
        }
        const auto& e = network.get_edge(i);
        std::string source = network.get_vertex(e.source).name;
        std::string target = network.get_vertex(e.target).name;
        auto successors_target_read = network_read.get_successors(source, target);
        std::sort(successors_target_transformed.begin(), successors_target_transformed.end());
        std::sort(successors_target_read.begin(), successors_target_read.end());
        EXPECT_TRUE(successors_target_transformed == successors_target_read);
    }

    // Check if the imported timetable is the same as the original timetable
    // Check if the timetable has the correct stations
    auto& stations_read = instance_read.get_station_list();
    EXPECT_TRUE(stations_read.size() == 2);
    EXPECT_TRUE(stations_read.has_station("s0"));
    EXPECT_TRUE(stations_read.has_station("s1"));

    // Check if the stations are imported correctly
    auto& st1_read = stations_read.get_station("s0");
    EXPECT_TRUE(st1_read.name == "s0");
    EXPECT_TRUE(st1_read.tracks.size() == 1);
    EXPECT_TRUE(network_read.get_edge(*st1_read.tracks.begin()).source == network_read.get_vertex_index("v0"));
    EXPECT_TRUE(network_read.get_edge(*st1_read.tracks.begin()).target == network_read.get_vertex_index("v1"));
    auto& st2_read = stations_read.get_station("s1");
    EXPECT_TRUE(st2_read.name == "s1");
    EXPECT_TRUE(st2_read.tracks.size() == 2);
    EXPECT_TRUE(st2_read.tracks == std::unordered_set<int>({network_read.get_edge_index("v1", "v2"),
                                                            network_read.get_edge_index("v2", "v1")}));

    // Check if the timetable has the correct trains
    auto& trains_read = instance_read.get_train_list();
    EXPECT_TRUE(trains_read.size() == 1);
    EXPECT_TRUE(trains_read.has_train("tr1"));

    // Check if the train tr1 is saved correctly
    auto tr1_read = trains_read.get_train("tr1");
    EXPECT_TRUE(tr1_read.name == "tr1");
    EXPECT_TRUE(tr1_read.length == 50);
    EXPECT_TRUE(tr1_read.max_speed == 10);
    EXPECT_TRUE(tr1_read.acceleration == 2);
    EXPECT_TRUE(tr1_read.deceleration == 2);

    // Check if the schedule of tr1 is saved correctly
    auto& tr1_schedule_read = instance_read.get_schedule("tr1");
    EXPECT_TRUE(tr1_schedule_read.t_0 == 0);
    EXPECT_TRUE(tr1_schedule_read.v_0 == 0);
    EXPECT_TRUE(tr1_schedule_read.t_n == 600);
    EXPECT_TRUE(tr1_schedule_read.v_n == 5);
    EXPECT_TRUE(network.get_vertex(tr1_schedule_read.entry).name == "v0");
    EXPECT_TRUE(network.get_vertex(tr1_schedule_read.exit).name == "v2");
    EXPECT_TRUE(tr1_schedule_read.stops.size() == 2);
    auto& stop1_read = tr1_schedule_read.stops[0];
    EXPECT_TRUE(stop1_read.begin == 60);
    EXPECT_TRUE(stop1_read.end == 120);
    EXPECT_TRUE(stations_read.get_station(stop1_read.station).name == "s0");
    auto& stop2_read = tr1_schedule_read.stops[1];
    EXPECT_TRUE(stop2_read.begin == 200);
    EXPECT_TRUE(stop2_read.end == 260);
    EXPECT_TRUE(stations_read.get_station(stop2_read.station).name == "s1");

    // Check if the imported instance has the same route map as the original instance
    // Check if the route for tr1 consists of two edges passing v0-v1-v2 in this order
    const auto& route_read = instance_read.get_route("tr1");
    EXPECT_TRUE(route_read.size() == 2);
    EXPECT_TRUE(network_read.get_vertex(route_read.get_edge(0, network).source).name == "v0");
    EXPECT_TRUE(network_read.get_vertex(route_read.get_edge(0, network).target).name == "v1");
    EXPECT_TRUE(network_read.get_vertex(route_read.get_edge(1, network).source).name == "v1");
    EXPECT_TRUE(network_read.get_vertex(route_read.get_edge(1, network).target).name == "v2");
}