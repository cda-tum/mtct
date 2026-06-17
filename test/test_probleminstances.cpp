#include "Definitions.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Timetable.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"

#include "gtest/gtest.h"
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

#define EXPECT_APPROX_EQ_2(a, b) EXPECT_APPROX_EQ(a, b, 1e-2)

#define EXPECT_APPROX_EQ(a, b, c)                                              \
  EXPECT_TRUE(std::abs((a) - (b)) < (c)) << (a) << " !=(approx.) " << (b)

using std::size_t;

// NOLINTBEGIN(clang-diagnostic-unused-result,google-readability-function-size,readability-function-size)

namespace {

struct EdgeTarget {
  std::string source;
  std::string target;
  double      length;
  double      max_speed;
  bool        breakable;
  double      min_block_length;
  double      min_stop_block_length = 100;
};

void check_instance_import(
    const cda_rail::instances::GeneralPerformanceOptimizationInstance&
        instance) {
  const auto& network = instance.get_const_network();

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
    const std::string&      v_name = vertex_names.at(i);
    const cda_rail::Vertex& v      = network.get_vertex({v_name});
    EXPECT_EQ(v.name, v_name);
    EXPECT_EQ(v.type, type.at(i));
  }

  // Check edges properties
  std::vector<EdgeTarget> edge_targets;
  edge_targets.push_back({.source           = "l0",
                          .target           = "l1",
                          .length           = 500,
                          .max_speed        = 27.7778,
                          .breakable        = true,
                          .min_block_length = 10});
  edge_targets.push_back({.source           = "l1",
                          .target           = "l2",
                          .length           = 500,
                          .max_speed        = 27.7778,
                          .breakable        = true,
                          .min_block_length = 10});
  edge_targets.push_back({.source           = "l2",
                          .target           = "l3",
                          .length           = 5,
                          .max_speed        = 27.7778,
                          .breakable        = false,
                          .min_block_length = 0});
  edge_targets.push_back({.source           = "l3",
                          .target           = "g00",
                          .length           = 5,
                          .max_speed        = 27.7778,
                          .breakable        = false,
                          .min_block_length = 0});
  edge_targets.push_back({.source           = "l3",
                          .target           = "g10",
                          .length           = 5,
                          .max_speed        = 27.7778,
                          .breakable        = false,
                          .min_block_length = 0});
  edge_targets.push_back({.source                = "g00",
                          .target                = "g01",
                          .length                = 300,
                          .max_speed             = 27.7778,
                          .breakable             = true,
                          .min_block_length      = 10,
                          .min_stop_block_length = 150});
  edge_targets.push_back({.source                = "g10",
                          .target                = "g11",
                          .length                = 300,
                          .max_speed             = 27.7778,
                          .breakable             = true,
                          .min_block_length      = 10,
                          .min_stop_block_length = 150});
  edge_targets.push_back({.source           = "g01",
                          .target           = "r2",
                          .length           = 5,
                          .max_speed        = 27.7778,
                          .breakable        = false,
                          .min_block_length = 0});
  edge_targets.push_back({.source           = "g11",
                          .target           = "r2",
                          .length           = 5,
                          .max_speed        = 27.7778,
                          .breakable        = false,
                          .min_block_length = 0});
  edge_targets.push_back({.source           = "r2",
                          .target           = "r1",
                          .length           = 5,
                          .max_speed        = 27.7778,
                          .breakable        = false,
                          .min_block_length = 0});
  edge_targets.push_back({.source           = "r1",
                          .target           = "r0",
                          .length           = 500,
                          .max_speed        = 27.7778,
                          .breakable        = true,
                          .min_block_length = 10});
  edge_targets.push_back({.source           = "r0",
                          .target           = "r1",
                          .length           = 500,
                          .max_speed        = 27.7778,
                          .breakable        = true,
                          .min_block_length = 10});
  edge_targets.push_back({.source           = "r1",
                          .target           = "r2",
                          .length           = 5,
                          .max_speed        = 27.7778,
                          .breakable        = false,
                          .min_block_length = 0});
  edge_targets.push_back({.source           = "r2",
                          .target           = "g01",
                          .length           = 5,
                          .max_speed        = 27.7778,
                          .breakable        = false,
                          .min_block_length = 0});
  edge_targets.push_back({.source           = "r2",
                          .target           = "g11",
                          .length           = 5,
                          .max_speed        = 27.7778,
                          .breakable        = false,
                          .min_block_length = 0});
  edge_targets.push_back({.source                = "g01",
                          .target                = "g00",
                          .length                = 300,
                          .max_speed             = 27.7778,
                          .breakable             = true,
                          .min_block_length      = 10,
                          .min_stop_block_length = 150});
  edge_targets.push_back({.source                = "g11",
                          .target                = "g10",
                          .length                = 300,
                          .max_speed             = 27.7778,
                          .breakable             = true,
                          .min_block_length      = 10,
                          .min_stop_block_length = 150});
  edge_targets.push_back({.source           = "g00",
                          .target           = "l3",
                          .length           = 5,
                          .max_speed        = 27.7778,
                          .breakable        = false,
                          .min_block_length = 0});
  edge_targets.push_back({.source           = "g10",
                          .target           = "l3",
                          .length           = 5,
                          .max_speed        = 27.7778,
                          .breakable        = false,
                          .min_block_length = 0});
  edge_targets.push_back({.source           = "l3",
                          .target           = "l2",
                          .length           = 5,
                          .max_speed        = 27.7778,
                          .breakable        = false,
                          .min_block_length = 0});
  edge_targets.push_back({.source           = "l2",
                          .target           = "l1",
                          .length           = 500,
                          .max_speed        = 27.7778,
                          .breakable        = true,
                          .min_block_length = 10});
  edge_targets.push_back({.source           = "l1",
                          .target           = "l0",
                          .length           = 500,
                          .max_speed        = 27.7778,
                          .breakable        = true,
                          .min_block_length = 10});

  EXPECT_EQ(network.number_of_edges(), edge_targets.size());
  for (const auto& edge : edge_targets) {
    const cda_rail::Edge e = network.get_edge({edge.source, edge.target});
    EXPECT_EQ(network.get_vertex(e.source).name, edge.source);
    EXPECT_EQ(network.get_vertex(e.target).name, edge.target);
    EXPECT_EQ(e.length, edge.length);
    EXPECT_APPROX_EQ_2(e.max_speed, edge.max_speed);
    EXPECT_EQ(e.breakable, edge.breakable);
    EXPECT_EQ(e.min_block_length, edge.min_block_length);
    EXPECT_EQ(e.min_stop_block_length, edge.min_stop_block_length);
  }

  // Check successors
  cda_rail::index_set successors_target;

  // l0,l1
  successors_target.clear();
  successors_target.insert(network.get_edge_index({"l1"}, {"l2"}));
  EXPECT_EQ(network.get_successors({"l0", "l1"}), successors_target);

  // l1,l2
  successors_target.clear();
  successors_target.insert(network.get_edge_index({"l2"}, {"l3"}));
  EXPECT_EQ(network.get_successors({"l1", "l2"}), successors_target);

  // l2,l3
  successors_target.clear();
  successors_target.insert(network.get_edge_index({"l3"}, {"g00"}));
  successors_target.insert(network.get_edge_index({"l3"}, {"g10"}));
  auto successors_l2_l3 = network.get_successors({"l2", "l3"});
  EXPECT_EQ(successors_l2_l3, successors_target);

  // l3,g00
  successors_target.clear();
  successors_target.insert(network.get_edge_index({"g00"}, {"g01"}));
  EXPECT_EQ(network.get_successors({"l3", "g00"}), successors_target);

  // l3,g10
  successors_target.clear();
  successors_target.insert(network.get_edge_index({"g10"}, {"g11"}));
  EXPECT_EQ(network.get_successors({"l3", "g10"}), successors_target);

  // g00,g01
  successors_target.clear();
  successors_target.insert(network.get_edge_index({"g01"}, {"r2"}));
  EXPECT_EQ(network.get_successors({"g00", "g01"}), successors_target);

  // g10,g11
  successors_target.clear();
  successors_target.insert(network.get_edge_index({"g11"}, {"r2"}));
  EXPECT_EQ(network.get_successors({"g10", "g11"}), successors_target);

  // g01,r2
  successors_target.clear();
  successors_target.insert(network.get_edge_index({"r2"}, {"r1"}));
  EXPECT_EQ(network.get_successors({"g01", "r2"}), successors_target);

  // g11,r2
  successors_target.clear();
  successors_target.insert(network.get_edge_index({"r2"}, {"r1"}));
  EXPECT_EQ(network.get_successors({"g11", "r2"}), successors_target);

  // r2,r1
  successors_target.clear();
  successors_target.insert(network.get_edge_index({"r1"}, {"r0"}));
  EXPECT_EQ(network.get_successors({"r2", "r1"}), successors_target);

  // r1,r0
  successors_target.clear();
  EXPECT_EQ(network.get_successors({"r1", "r0"}), successors_target);

  // r0,r1
  successors_target.clear();
  successors_target.insert(network.get_edge_index({"r1"}, {"r2"}));
  EXPECT_EQ(network.get_successors({"r0", "r1"}), successors_target);

  // r1,r2
  successors_target.clear();
  successors_target.insert(network.get_edge_index({"r2"}, {"g01"}));
  successors_target.insert(network.get_edge_index({"r2"}, {"g11"}));
  auto successors_r1_r2 = network.get_successors({"r1", "r2"});
  EXPECT_EQ(successors_r1_r2, successors_target);

  // r2,g01
  successors_target.clear();
  successors_target.insert(network.get_edge_index({"g01"}, {"g00"}));
  EXPECT_EQ(network.get_successors({"r2", "g01"}), successors_target);

  // r2,g11
  successors_target.clear();
  successors_target.insert(network.get_edge_index({"g11"}, {"g10"}));
  EXPECT_EQ(network.get_successors({"r2", "g11"}), successors_target);

  // g01,g00
  successors_target.clear();
  successors_target.insert(network.get_edge_index({"g00"}, {"l3"}));
  EXPECT_EQ(network.get_successors({"g01", "g00"}), successors_target);

  // g11,g10
  successors_target.clear();
  successors_target.insert(network.get_edge_index({"g10"}, {"l3"}));
  EXPECT_EQ(network.get_successors({"g11", "g10"}), successors_target);

  // g00,l3
  successors_target.clear();
  successors_target.insert(network.get_edge_index({"l3"}, {"l2"}));
  EXPECT_EQ(network.get_successors({"g00", "l3"}), successors_target);

  // g10,l3
  successors_target.clear();
  successors_target.insert(network.get_edge_index({"l3"}, {"l2"}));
  EXPECT_EQ(network.get_successors({"g10", "l3"}), successors_target);

  // l3,l2
  successors_target.clear();
  successors_target.insert(network.get_edge_index({"l2"}, {"l1"}));
  EXPECT_EQ(network.get_successors({"l3", "l2"}), successors_target);

  // l2,l1
  successors_target.clear();
  successors_target.insert(network.get_edge_index({"l1"}, {"l0"}));
  EXPECT_EQ(network.get_successors({"l2", "l1"}), successors_target);

  // l1,l0
  successors_target.clear();
  EXPECT_EQ(network.get_successors({"l1", "l0"}), successors_target);

  // Check timetable
  const auto& stations = instance.get_const_station_list();
  EXPECT_EQ(stations.size(), 1);
  EXPECT_TRUE(stations.has_station("Central"));

  // Check if the station is imported correctly
  const auto& station = stations.get_station("Central");
  EXPECT_EQ(station.name, "Central");
  EXPECT_EQ(station.tracks.size(), 4);
  cda_rail::index_set const track_ids{network.get_edge_index({"g00"}, {"g01"}),
                                      network.get_edge_index({"g10"}, {"g11"}),
                                      network.get_edge_index({"g01"}, {"g00"}),
                                      network.get_edge_index({"g11"}, {"g10"})};
  auto                      tracks_read = station.tracks;
  EXPECT_EQ(tracks_read, track_ids);

  const auto& trains = instance.get_const_train_list();
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
  const auto& tr1_schedule = instance.get_const_schedule("tr1");
  EXPECT_EQ(tr1_schedule.get_entry_time(), 120);
  EXPECT_EQ(tr1_schedule.get_initial_velocity(), 0);
  EXPECT_EQ(tr1_schedule.get_exit_time(), 645);
  EXPECT_EQ(tr1_schedule.get_exit_velocity(), 16.67);
  EXPECT_EQ(network.get_vertex(tr1_schedule.get_entry_vertex()).name, "l0");
  EXPECT_EQ(network.get_vertex(tr1_schedule.get_exit_vertex()).name, "r0");
  EXPECT_EQ(tr1_schedule.get_stops().size(), 1);
  const auto& stop = tr1_schedule.get_stops().at(0);
  EXPECT_EQ(stop.get_service_time(), 240);
  EXPECT_EQ(stop.get_earliest_departure(), 300);
  EXPECT_EQ(stations.get_station(stop.get_station().name).name, "Central");

  EXPECT_FALSE(0 >= stop.get_service_time() &&
               0 <= stop.get_earliest_departure());
  EXPECT_FALSE(100 >= stop.get_service_time() &&
               100 <= stop.get_earliest_departure());
  EXPECT_FALSE(310 >= stop.get_service_time() &&
               310 <= stop.get_earliest_departure());
  EXPECT_TRUE(240 >= stop.get_service_time() &&
              240 <= stop.get_earliest_departure());
  EXPECT_TRUE(260 >= stop.get_service_time() &&
              260 <= stop.get_earliest_departure());
  EXPECT_TRUE(300 >= stop.get_service_time() &&
              300 <= stop.get_earliest_departure());

  // Check the schedule of tr2
  const auto& tr2_schedule = instance.get_const_schedule("tr2");
  EXPECT_EQ(tr2_schedule.get_entry_time(), 0);
  EXPECT_EQ(tr2_schedule.get_initial_velocity(), 0);
  EXPECT_EQ(tr2_schedule.get_exit_time(), 420);
  EXPECT_EQ(tr2_schedule.get_exit_velocity(), 16.67);
  EXPECT_EQ(network.get_vertex(tr2_schedule.get_entry_vertex()).name, "l0");
  EXPECT_EQ(network.get_vertex(tr2_schedule.get_exit_vertex()).name, "r0");
  EXPECT_EQ(tr2_schedule.get_stops().size(), 1);
  const auto& stop2 = tr2_schedule.get_stops().at(0);
  EXPECT_EQ(stop2.get_service_time(), 120);
  EXPECT_EQ(stop2.get_earliest_departure(), 300);
  EXPECT_EQ(stations.get_station(stop2.get_station().name).name, "Central");

  EXPECT_FALSE(0 >= stop2.get_service_time() &&
               0 <= stop2.get_earliest_departure());
  EXPECT_FALSE(100 >= stop2.get_service_time() &&
               100 <= stop2.get_earliest_departure());
  EXPECT_FALSE(310 >= stop2.get_service_time() &&
               310 <= stop2.get_earliest_departure());
  EXPECT_TRUE(120 >= stop2.get_service_time() &&
              120 <= stop2.get_earliest_departure());
  EXPECT_TRUE(150 >= stop2.get_service_time() &&
              150 <= stop2.get_earliest_departure());
  EXPECT_TRUE(300 >= stop2.get_service_time() &&
              300 <= stop2.get_earliest_departure());

  // Check the schedule of tr3
  const auto& tr3_schedule = instance.get_const_schedule("tr3");
  EXPECT_EQ(tr3_schedule.get_entry_time(), 0);
  EXPECT_EQ(tr3_schedule.get_initial_velocity(), 0);
  EXPECT_EQ(tr3_schedule.get_exit_time(), 420);
  EXPECT_EQ(tr3_schedule.get_exit_velocity(), 16.67);
  EXPECT_EQ(network.get_vertex(tr3_schedule.get_entry_vertex()).name, "r0");
  EXPECT_EQ(network.get_vertex(tr3_schedule.get_exit_vertex()).name, "l0");
  EXPECT_EQ(tr3_schedule.get_stops().size(), 1);
  const auto& stop3 = tr3_schedule.get_stops().at(0);
  EXPECT_EQ(stop3.get_service_time(), 180);
  EXPECT_EQ(stop3.get_earliest_departure(), 300);
  EXPECT_EQ(stations.get_station(stop3.get_station().name).name, "Central");

  // Check the route  map
  // Check if the route consists of three trains with names "tr1", "tr2" and
  // "tr3"
  EXPECT_EQ(instance.get_const_routes().size(), 3);
  EXPECT_TRUE(instance.get_const_routes().has_route("tr1"));
  EXPECT_TRUE(instance.get_const_routes().has_route("tr2"));
  EXPECT_TRUE(instance.get_const_routes().has_route("tr3"));

  // Check if the route for tr1 consists of eight edges passing vertices
  // l0-l1-l2-l3-g00-g01-r2-r1-r0 in this order.
  const auto& route = instance.get_const_routes().get_route("tr1");
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
  const auto& route2 = instance.get_const_routes().get_route("tr2");
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
  const auto& route3 = instance.get_const_routes().get_route("tr3");
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
}
} // namespace

TEST(GenPOInstance, GenPOInstanceImport1) {
  cda_rail::instances::GeneralPerformanceOptimizationInstance const instance(
      "SimpleStation", "atmos2023", "./data");

  check_instance_import(instance);
}

TEST(GenPOInstance, GenPOTimetableExport) {
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance;
  instance.set_instance_name("VSSGenerationTimetableExportTest");
  instance.set_instance_subdirectory("tmp_subdirectory");
  instance.get_editable_network().set_network_name(
      "VSSGenerationTimetableExportTestNetwork");

  // Add a simple network to the instance
  instance.get_editable_network().add_vertex("v0", cda_rail::VertexType::TTD);
  instance.get_editable_network().add_vertex("v1", cda_rail::VertexType::VSS);
  instance.get_editable_network().add_vertex("v2",
                                             cda_rail::VertexType::NoBorder);

  instance.get_editable_network().add_edge({"v0"}, {"v1"}, 100, 10, true, 10);
  instance.get_editable_network().add_edge({"v1"}, {"v2"}, 200, 20, false);
  instance.get_editable_network().add_edge({"v1"}, {"v0"}, 100, 10, true, 10);
  instance.get_editable_network().add_edge({"v2"}, {"v1"}, 200, 20, false);

  instance.get_editable_network().add_successor({"v0", "v1"}, {"v1", "v2"});
  instance.get_editable_network().add_successor({"v2", "v1"}, {"v1", "v0"});

  // Add a simple timetable to the instance
  instance.add_train("tr1", 50, 10, 2, 2, 0, 0, {"v0"}, 600, 5, {"v2"});
  instance.add_empty_station("s0");
  instance.add_track_to_station("s0", {"v0", "v1"});
  instance.add_empty_station("s1");
  instance.add_track_to_station("s1", {"v1", "v2"});
  instance.add_track_to_station("s1", {"v2", "v1"});
  instance.insert_stop("tr1", "s1", 200, 60);
  instance.insert_stop("tr1", "s0", 60, 60);

  EXPECT_EQ(

      instance.get_const_schedule("tr1").get_stops().at(0).get_station().name,
      "s0");

  // Add route to instance
  instance.add_empty_route("tr1");
  instance.push_back_edge_to_route("tr1", {"v0", "v1"});
  instance.push_back_edge_to_route("tr1", {"v1", "v2"});

  // Check for consistency
  EXPECT_TRUE(instance.check_consistency());

  // Export the instance
  instance.export_instance("./tmp", true);

  // Import the instance and delete tmp folder
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance_read(
      "VSSGenerationTimetableExportTest", "tmp_subdirectory", "./tmp");
  std::filesystem::remove_all("./tmp");

  // Check if the imported instance is still consistent
  EXPECT_TRUE(instance_read.check_consistency());

  // Check if the imported instance is the same as the original instance
  // check vertices
  const auto& network      = instance.get_editable_network();
  const auto& network_read = instance_read.get_editable_network();
  EXPECT_EQ(network.number_of_vertices(), network_read.number_of_vertices());
  for (size_t i = 0; i < network.number_of_vertices(); ++i) {
    EXPECT_TRUE(network_read.has_vertex(network.get_vertex(i).name));
    EXPECT_EQ(network_read.get_vertex({network.get_vertex(i).name}).type,
              network.get_vertex(i).type);
  }

  // check edges
  EXPECT_EQ(network.number_of_edges(), network_read.number_of_edges());
  for (size_t i = 0; i < network.number_of_edges(); ++i) {
    const auto& source_vertex = network.get_vertex(network.get_edge(i).source);
    const auto& target_vertex = network.get_vertex(network.get_edge(i).target);
    EXPECT_TRUE(
        network_read.has_edge({source_vertex.name}, {target_vertex.name}));
    const auto& edge_read =
        network_read.get_edge({source_vertex.name, target_vertex.name});
    EXPECT_EQ(edge_read.breakable, network.get_edge(i).breakable);
    EXPECT_EQ(edge_read.length, network.get_edge(i).length);
    EXPECT_EQ(edge_read.max_speed, network.get_edge(i).max_speed);
    EXPECT_EQ(edge_read.min_block_length, network.get_edge(i).min_block_length);
  }

  // check successors
  for (size_t i = 0; i < network.number_of_edges(); ++i) {
    const auto&         successors_target = network.get_successors(i);
    cda_rail::index_set successors_target_transformed;
    for (auto successor : successors_target) {
      const auto&       e      = network.get_edge(successor);
      const std::string source = network.get_vertex(e.source).name;
      const std::string target = network.get_vertex(e.target).name;
      successors_target_transformed.insert(
          network_read.get_edge_index({source}, {target}));
    }
    const auto&       e         = network.get_edge(i);
    const std::string source    = network.get_vertex(e.source).name;
    const std::string target    = network.get_vertex(e.target).name;
    auto successors_target_read = network_read.get_successors({source, target});
    EXPECT_EQ(successors_target_transformed, successors_target_read);
  }

  // Check if the imported timetable is the same as the original timetable
  // Check if the timetable has the correct stations
  const auto& stations_read = instance_read.get_const_station_list();
  EXPECT_EQ(stations_read.size(), 2);
  EXPECT_TRUE(stations_read.has_station("s0"));
  EXPECT_TRUE(stations_read.has_station("s1"));

  // Check if the stations are imported correctly
  const auto& st1_read = stations_read.get_station("s0");
  EXPECT_EQ(st1_read.name, "s0");
  EXPECT_EQ(st1_read.tracks.size(), 1);
  EXPECT_EQ(network_read.get_edge(*st1_read.tracks.begin()).source,
            network_read.get_vertex_index("v0"));
  EXPECT_EQ(network_read.get_edge(*st1_read.tracks.begin()).target,
            network_read.get_vertex_index("v1"));
  const auto& st2_read = stations_read.get_station("s1");
  EXPECT_EQ(st2_read.name, "s1");
  EXPECT_EQ(st2_read.tracks.size(), 2);
  auto tracks_st2_read = st2_read.tracks;
  auto tracks_st2_target =
      cda_rail::index_set({network_read.get_edge_index({"v1", "v2"}),
                           network_read.get_edge_index({"v2", "v1"})});
  EXPECT_EQ(tracks_st2_read, tracks_st2_target);

  // Check if the timetable has the correct trains
  const auto& trains_read = instance_read.get_const_train_list();
  EXPECT_EQ(trains_read.size(), 1);
  EXPECT_TRUE(trains_read.has_train("tr1"));

  // Check if the train tr1 is saved correctly
  const auto& tr1_read = trains_read.get_train("tr1");
  EXPECT_EQ(tr1_read.get_name(), "tr1");
  EXPECT_EQ(tr1_read.get_length(), 50);
  EXPECT_EQ(tr1_read.get_max_speed(), 10);
  EXPECT_EQ(tr1_read.get_acceleration(), 2);
  EXPECT_EQ(tr1_read.get_deceleration(), 2);

  // Check if the schedule of tr1 is saved correctly
  const auto& tr1_schedule_read = instance_read.get_const_schedule("tr1");
  EXPECT_EQ(tr1_schedule_read.get_entry_time(), 0);
  EXPECT_EQ(tr1_schedule_read.get_initial_velocity(), 0);
  EXPECT_EQ(tr1_schedule_read.get_exit_time(), 600);
  EXPECT_EQ(tr1_schedule_read.get_exit_velocity(), 5);
  EXPECT_EQ(network.get_vertex(tr1_schedule_read.get_entry_vertex()).name,
            "v0");
  EXPECT_EQ(network.get_vertex(tr1_schedule_read.get_exit_vertex()).name, "v2");
  EXPECT_EQ(tr1_schedule_read.get_stops().size(), 2);
  const auto& stop1_read = tr1_schedule_read.get_stops().at(0);
  EXPECT_EQ(stop1_read.get_service_time(), 60);
  EXPECT_EQ(stop1_read.get_earliest_departure(), 120);
  EXPECT_EQ(stations_read.get_station(stop1_read.get_station().name).name,
            "s0");
  const auto& stop2_read = tr1_schedule_read.get_stops().at(1);
  EXPECT_EQ(stop2_read.get_service_time(), 200);
  EXPECT_EQ(stop2_read.get_earliest_departure(), 260);
  EXPECT_EQ(stations_read.get_station(stop2_read.get_station().name).name,
            "s1");

  // Check if the imported instance has the same route map as the original
  // instance Check if the route for tr1 consists of two edges passing v0-v1-v2
  // in this order
  const auto& route_read = instance_read.get_const_routes().get_route("tr1");
  EXPECT_EQ(route_read.size(), 2);
  EXPECT_EQ(
      network_read.get_vertex(route_read.get_edge(0, network).source).name,
      "v0");
  EXPECT_EQ(
      network_read.get_vertex(route_read.get_edge(0, network).target).name,
      "v1");
  EXPECT_EQ(
      network_read.get_vertex(route_read.get_edge(1, network).source).name,
      "v1");
  EXPECT_EQ(
      network_read.get_vertex(route_read.get_edge(1, network).target).name,
      "v2");

  // Check tr1 length
  EXPECT_EQ(instance_read.route_length("tr1"), 300);
}

TEST(GenPOInstance, Stammstrecke) {
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance;

  // Pasing -> Laim
  const auto pasing_entry = instance.get_editable_network().add_vertex(
      "PasingEntry", cda_rail::VertexType::TTD);
  const auto pasing_exit = instance.get_editable_network().add_vertex(
      "PasingExit", cda_rail::VertexType::TTD);
  const auto pasing_switch_1 = instance.get_editable_network().add_vertex(
      "PasingSwitch1", cda_rail::VertexType::TTD);
  const auto pasing_switch_2 = instance.get_editable_network().add_vertex(
      "PasingSwitch2", cda_rail::VertexType::TTD);
  const auto laim_1l = instance.get_editable_network().add_vertex(
      "Laim1L", cda_rail::VertexType::TTD);
  const auto laim_3l = instance.get_editable_network().add_vertex(
      "Laim3L", cda_rail::VertexType::TTD);
  const auto laim_1r = instance.get_editable_network().add_vertex(
      "Laim1R", cda_rail::VertexType::TTD);
  const auto laim_entry = instance.get_editable_network().add_vertex(
      "LaimEntry", cda_rail::VertexType::TTD);
  const auto laim_3r = instance.get_editable_network().add_vertex(
      "Laim3R", cda_rail::VertexType::TTD);
  const auto laim_switch_nymphenburg =
      instance.get_editable_network().add_vertex("LaimSwitchNymphenburg",
                                                 cda_rail::VertexType::TTD);
  const auto laim_exit_nymphenburg = instance.get_editable_network().add_vertex(
      "LaimExitNymphenburg", cda_rail::VertexType::TTD);

  const auto e1lr = instance.get_editable_network().add_edge(
      pasing_entry, pasing_switch_1, 280, 120 / 3.6, true, 50);
  const auto e1rl = instance.get_editable_network().add_edge(
      pasing_switch_2, pasing_exit, 160, 120 / 3.6, true, 50);
  const auto e2lr = instance.get_editable_network().add_edge(
      pasing_switch_1, laim_1l, 2812, 120 / 3.6, true, 50);
  const auto e2rl = instance.get_editable_network().add_edge(
      laim_switch_nymphenburg, pasing_switch_2, 2562, 120 / 3.6, true, 50);
  const auto e3rl = instance.get_editable_network().add_edge(
      laim_3l, laim_switch_nymphenburg, 370, 120 / 3.6, true, 50);
  const auto e2rl_exit = instance.get_editable_network().add_edge(
      laim_switch_nymphenburg, laim_exit_nymphenburg, 30, 100 / 3.6, false, 50);
  const auto e3lr = instance.get_editable_network().add_edge(
      laim_1l, laim_1r, 210, 120 / 3.6, true, 25);
  const auto e4rl = instance.get_editable_network().add_edge(
      laim_3r, laim_3l, 210, 120 / 3.6, true, 25);

  const auto switch_e1 = instance.get_editable_network().add_edge(
      pasing_switch_1, pasing_switch_2, 120, 80 / 3.6, false, 50);
  const auto switch_e2 = instance.get_editable_network().add_edge(
      pasing_switch_2, pasing_switch_1, 120, 80 / 3.6, false, 50);
  instance.get_editable_network().add_successor(switch_e1, e1rl);
  instance.get_editable_network().add_successor(switch_e2, e2lr);

  instance.get_editable_network().add_successor(e1lr, e2lr);
  instance.get_editable_network().add_successor(e2lr, e3lr);
  instance.get_editable_network().add_successor(e4rl, e3rl);
  instance.get_editable_network().add_successor(e3rl, e2rl);
  instance.get_editable_network().add_successor(e2rl, e1rl);
  instance.get_editable_network().add_successor(e3rl, e2rl_exit);

  instance.add_empty_station("Laim");
  instance.add_track_to_station("Laim", e3lr);
  instance.add_track_to_station("Laim", e4rl);

  // Laim -> Hirschgarten
  const auto laim_switch_hirschgarten =
      instance.get_editable_network().add_vertex("LaimSwitchHirschgarten",
                                                 cda_rail::VertexType::TTD);
  const auto hirschgarten_1l = instance.get_editable_network().add_vertex(
      "Hirschgarten1L", cda_rail::VertexType::TTD);
  const auto hirschgarten_2l = instance.get_editable_network().add_vertex(
      "Hirschgarten2L", cda_rail::VertexType::TTD);
  const auto hirschgarten_1r = instance.get_editable_network().add_vertex(
      "Hirschgarten1R", cda_rail::VertexType::TTD);
  const auto hirschgarten_2r = instance.get_editable_network().add_vertex(
      "Hirschgarten2R", cda_rail::VertexType::TTD);

  const auto e4lr = instance.get_editable_network().add_edge(
      laim_1r, laim_switch_hirschgarten, 200, 100 / 3.6, true, 50);
  const auto e4lr_entry = instance.get_editable_network().add_edge(
      laim_entry, laim_switch_hirschgarten, 200, 100 / 3.6, true, 50);
  const auto e5lr = instance.get_editable_network().add_edge(
      laim_switch_hirschgarten, hirschgarten_1l, 692, 100 / 3.6, true, 50);
  const auto e6lr = instance.get_editable_network().add_edge(
      hirschgarten_1l, hirschgarten_1r, 205, 100 / 3.6, true, 25);
  const auto e5rl = instance.get_editable_network().add_edge(
      hirschgarten_2l, laim_3r, 892, 100 / 3.6, true, 50);
  const auto e6rl = instance.get_editable_network().add_edge(
      hirschgarten_2r, hirschgarten_2l, 205, 100 / 3.6, true, 25);

  instance.get_editable_network().add_successor(e3lr, e4lr);
  instance.get_editable_network().add_successor(e5rl, e4rl);

  instance.get_editable_network().add_successor(e4lr, e5lr);
  instance.get_editable_network().add_successor(e4lr_entry, e5lr);
  instance.get_editable_network().add_successor(e5lr, e6lr);
  instance.get_editable_network().add_successor(e6rl, e5rl);

  instance.add_empty_station("Hirschgarten");
  instance.add_track_to_station("Hirschgarten", e6lr);
  instance.add_track_to_station("Hirschgarten", e6rl);

  // Hirschgarten -> Donnersbergerbruecke
  const auto donnersbergerbruecke_1l =
      instance.get_editable_network().add_vertex("Donnersbergerbruecke1L",
                                                 cda_rail::VertexType::TTD);
  const auto donnersbergerbruecke_2l =
      instance.get_editable_network().add_vertex("Donnersbergerbruecke2L",
                                                 cda_rail::VertexType::TTD);
  const auto donnersbergerbruecke_1r =
      instance.get_editable_network().add_vertex("Donnersbergerbruecke1R",
                                                 cda_rail::VertexType::TTD);
  const auto donnersbergerbruecke_2r =
      instance.get_editable_network().add_vertex("Donnersbergerbruecke2R",
                                                 cda_rail::VertexType::TTD);

  const auto e7lr = instance.get_editable_network().add_edge(
      hirschgarten_1r, donnersbergerbruecke_1l, 1095, 100 / 3.6, true, 50);
  const auto e8lr = instance.get_editable_network().add_edge(
      donnersbergerbruecke_1l, donnersbergerbruecke_1r, 205, 100 / 3.6, true,
      25);
  const auto e7rl = instance.get_editable_network().add_edge(
      donnersbergerbruecke_2l, hirschgarten_2r, 1095, 100 / 3.6, true, 50);
  const auto e8rl = instance.get_editable_network().add_edge(
      donnersbergerbruecke_2r, donnersbergerbruecke_2l, 205, 100 / 3.6, true,
      25);

  instance.get_editable_network().add_successor(e6lr, e7lr);
  instance.get_editable_network().add_successor(e7rl, e6rl);

  instance.get_editable_network().add_successor(e7lr, e8lr);
  instance.get_editable_network().add_successor(e8rl, e7rl);

  instance.add_empty_station("Donnersbergerbruecke");
  instance.add_track_to_station("Donnersbergerbruecke", e8lr);
  instance.add_track_to_station("Donnersbergerbruecke", e8rl);

  // Donnersbergerbruecke -> Hackerbruecke
  const auto hackerbruecke_switch_1 =
      instance.get_editable_network().add_vertex("HackerbrueckeSwitch1",
                                                 cda_rail::VertexType::TTD);
  const auto hackerbruecke_switch_2 =
      instance.get_editable_network().add_vertex("HackerbrueckeSwitch2",
                                                 cda_rail::VertexType::TTD);
  const auto hackerbruecke_switch_3 =
      instance.get_editable_network().add_vertex("HackerbrueckeSwitch3",
                                                 cda_rail::VertexType::TTD);
  const auto hackerbruecke_switch_4 =
      instance.get_editable_network().add_vertex("HackerbrueckeSwitch4",
                                                 cda_rail::VertexType::TTD);
  const auto hackerbruecke_switch_c =
      instance.get_editable_network().add_vertex(
          "HackerbrueckeSwitchC", cda_rail::VertexType::NoBorder);
  const auto hackerbruecke_1l = instance.get_editable_network().add_vertex(
      "Hackerbruecke1L", cda_rail::VertexType::TTD);
  const auto hackerbruecke_2l = instance.get_editable_network().add_vertex(
      "Hackerbruecke2L", cda_rail::VertexType::TTD);
  const auto hackerbruecke_1r = instance.get_editable_network().add_vertex(
      "Hackerbruecke1R", cda_rail::VertexType::TTD);
  const auto hackerbruecke_2r = instance.get_editable_network().add_vertex(
      "Hackerbruecke2R", cda_rail::VertexType::TTD);
  const auto hackerbruecke_switch_entry =
      instance.get_editable_network().add_vertex("HackerbrueckeSwitchEntry",
                                                 cda_rail::VertexType::TTD);
  const auto hackerbruecke_switch_exit =
      instance.get_editable_network().add_vertex("HackerbrueckeSwitchExit",
                                                 cda_rail::VertexType::TTD);
  const auto donnersberger_entry = instance.get_editable_network().add_vertex(
      "DonnersbergerEntry", cda_rail::VertexType::TTD);
  const auto donnersberger_exit = instance.get_editable_network().add_vertex(
      "DonnersbergerExit", cda_rail::VertexType::TTD);

  const auto e9lr_a = instance.get_editable_network().add_edge(
      donnersbergerbruecke_1r, hackerbruecke_switch_entry, 210, 100 / 3.6, true,
      50);
  const auto e9lr_b = instance.get_editable_network().add_edge(
      hackerbruecke_switch_entry, hackerbruecke_switch_1, 294, 100 / 3.6, true,
      50);
  const auto e9lr_entry = instance.get_editable_network().add_edge(
      donnersberger_entry, hackerbruecke_switch_entry, 210, 100 / 3.6, true,
      50);
  const auto e10lr = instance.get_editable_network().add_edge(
      hackerbruecke_switch_1, hackerbruecke_switch_2, 150, 100 / 3.6, false,
      50);
  const auto e11lr = instance.get_editable_network().add_edge(
      hackerbruecke_switch_2, hackerbruecke_1l, 40, 100 / 3.6, false, 50);
  const auto e12lr = instance.get_editable_network().add_edge(
      hackerbruecke_1l, hackerbruecke_1r, 207, 100 / 3.6, true, 25);
  const auto e9rl_a = instance.get_editable_network().add_edge(
      hackerbruecke_switch_exit, donnersbergerbruecke_2r, 210, 100 / 3.6, true,
      50);
  const auto e9rl_b = instance.get_editable_network().add_edge(
      hackerbruecke_switch_4, hackerbruecke_switch_exit, 294, 100 / 3.6, true,
      50);
  const auto e9rl_exit = instance.get_editable_network().add_edge(
      hackerbruecke_switch_exit, donnersberger_exit, 210, 100 / 3.6, true, 50);
  const auto e10rl = instance.get_editable_network().add_edge(
      hackerbruecke_switch_3, hackerbruecke_switch_4, 150, 100 / 3.6, false,
      50);
  const auto e11rl = instance.get_editable_network().add_edge(
      hackerbruecke_2l, hackerbruecke_switch_3, 40, 100 / 3.6, false, 50);
  const auto e12rl = instance.get_editable_network().add_edge(
      hackerbruecke_2r, hackerbruecke_2l, 207, 100 / 3.6, true, 25);

  instance.get_editable_network().add_successor(e9rl_b, e9rl_exit);
  instance.get_editable_network().add_successor(e9lr_entry, e9lr_b);

  const auto switch_e3 = instance.get_editable_network().add_edge(
      hackerbruecke_switch_1, hackerbruecke_switch_c, 75, 80 / 3.6, false, 50);
  const auto switch_e4 = instance.get_editable_network().add_edge(
      hackerbruecke_switch_2, hackerbruecke_switch_c, 75, 80 / 3.6, false, 50);
  const auto switch_e5 = instance.get_editable_network().add_edge(
      hackerbruecke_switch_3, hackerbruecke_switch_c, 75, 80 / 3.6, false, 50);
  const auto switch_e6 = instance.get_editable_network().add_edge(
      hackerbruecke_switch_4, hackerbruecke_switch_c, 75, 80 / 3.6, false, 50);
  const auto switch_e7 = instance.get_editable_network().add_edge(
      hackerbruecke_switch_c, hackerbruecke_switch_1, 75, 80 / 3.6, false, 50);
  const auto switch_e8 = instance.get_editable_network().add_edge(
      hackerbruecke_switch_c, hackerbruecke_switch_2, 75, 80 / 3.6, false, 50);
  const auto switch_e9 = instance.get_editable_network().add_edge(
      hackerbruecke_switch_c, hackerbruecke_switch_3, 75, 80 / 3.6, false, 50);
  const auto switch_e10 = instance.get_editable_network().add_edge(
      hackerbruecke_switch_c, hackerbruecke_switch_4, 75, 80 / 3.6, false, 50);

  instance.get_editable_network().add_successor(switch_e3, switch_e9);
  instance.get_editable_network().add_successor(switch_e4, switch_e10);
  instance.get_editable_network().add_successor(switch_e5, switch_e7);
  instance.get_editable_network().add_successor(switch_e6, switch_e8);
  instance.get_editable_network().add_successor(switch_e8, e11lr);
  instance.get_editable_network().add_successor(switch_e10, e9rl_b);
  instance.get_editable_network().add_successor(e9lr_b, switch_e3);
  instance.get_editable_network().add_successor(e11rl, switch_e5);

  instance.get_editable_network().add_successor(e8lr, e9lr_a);
  instance.get_editable_network().add_successor(e9rl_a, e8rl);

  instance.get_editable_network().add_successor(e9lr_a, e9lr_b);
  instance.get_editable_network().add_successor(e9lr_b, e10lr);
  instance.get_editable_network().add_successor(e10lr, e11lr);
  instance.get_editable_network().add_successor(e11lr, e12lr);
  instance.get_editable_network().add_successor(e12rl, e11rl);
  instance.get_editable_network().add_successor(e11rl, e10rl);
  instance.get_editable_network().add_successor(e10rl, e9rl_b);
  instance.get_editable_network().add_successor(e9rl_b, e9rl_a);

  instance.add_empty_station("Hackerbruecke");
  instance.add_track_to_station("Hackerbruecke", e12lr);
  instance.add_track_to_station("Hackerbruecke", e12rl);

  // Hackerbruecke -> Hbf
  const auto hbf_1l = instance.get_editable_network().add_vertex(
      "Hbf1L", cda_rail::VertexType::TTD);
  const auto hbf_2l = instance.get_editable_network().add_vertex(
      "Hbf2L", cda_rail::VertexType::TTD);
  const auto hbf_1r = instance.get_editable_network().add_vertex(
      "Hbf1R", cda_rail::VertexType::TTD);
  const auto hbf_2r = instance.get_editable_network().add_vertex(
      "Hbf2R", cda_rail::VertexType::TTD);

  const auto e13lr = instance.get_editable_network().add_edge(
      hackerbruecke_1r, hbf_1l, 591, 80 / 3.6, true, 50);
  const auto e14lr = instance.get_editable_network().add_edge(
      hbf_1l, hbf_1r, 210, 80 / 3.6, true, 25);
  const auto e13rl = instance.get_editable_network().add_edge(
      hbf_2l, hackerbruecke_2r, 591, 80 / 3.6, true, 50);
  const auto e14rl = instance.get_editable_network().add_edge(
      hbf_2r, hbf_2l, 210, 80 / 3.6, true, 25);

  instance.get_editable_network().add_successor(e12lr, e13lr);
  instance.get_editable_network().add_successor(e13rl, e12rl);

  instance.get_editable_network().add_successor(e13lr, e14lr);
  instance.get_editable_network().add_successor(e14rl, e13rl);

  instance.add_empty_station("Hbf");
  instance.add_track_to_station("Hbf", e14lr);
  instance.add_track_to_station("Hbf", e14rl);

  // Hbf -> Karlsplatz
  const auto karlsplatz_1l = instance.get_editable_network().add_vertex(
      "Karlsplatz1L", cda_rail::VertexType::TTD);
  const auto karlsplatz_2l = instance.get_editable_network().add_vertex(
      "Karlsplatz2L", cda_rail::VertexType::TTD);
  const auto karlsplatz_1r = instance.get_editable_network().add_vertex(
      "Karlsplatz1R", cda_rail::VertexType::TTD);
  const auto karlsplatz_2r = instance.get_editable_network().add_vertex(
      "Karlsplatz2R", cda_rail::VertexType::TTD);

  const auto e15lr = instance.get_editable_network().add_edge(
      hbf_1r, karlsplatz_1l, 292, 80 / 3.6, true, 50);
  const auto e16lr = instance.get_editable_network().add_edge(
      karlsplatz_1l, karlsplatz_1r, 206, 80 / 3.6, true, 25);
  const auto e15rl = instance.get_editable_network().add_edge(
      karlsplatz_2l, hbf_2r, 292, 80 / 3.6, true, 50);
  const auto e16rl = instance.get_editable_network().add_edge(
      karlsplatz_2r, karlsplatz_2l, 206, 80 / 3.6, true, 25);

  instance.get_editable_network().add_successor(e14lr, e15lr);
  instance.get_editable_network().add_successor(e15rl, e14rl);

  instance.get_editable_network().add_successor(e15lr, e16lr);
  instance.get_editable_network().add_successor(e16rl, e15rl);

  instance.add_empty_station("Karlsplatz");
  instance.add_track_to_station("Karlsplatz", e16lr);
  instance.add_track_to_station("Karlsplatz", e16rl);

  // Karlsplatz -> Marienplatz
  const auto marienplatz_1l = instance.get_editable_network().add_vertex(
      "Marienplatz1L", cda_rail::VertexType::TTD);
  const auto marienplatz_2l = instance.get_editable_network().add_vertex(
      "Marienplatz2L", cda_rail::VertexType::TTD);
  const auto marienplatz_1r = instance.get_editable_network().add_vertex(
      "Marienplatz1R", cda_rail::VertexType::TTD);
  const auto marienplatz_2r = instance.get_editable_network().add_vertex(
      "Marienplatz2R", cda_rail::VertexType::TTD);

  const auto e17lr = instance.get_editable_network().add_edge(
      karlsplatz_1r, marienplatz_1l, 494, 80 / 3.6, true, 50);
  const auto e18lr = instance.get_editable_network().add_edge(
      marienplatz_1l, marienplatz_1r, 205, 80 / 3.6, true, 25);
  const auto e17rl = instance.get_editable_network().add_edge(
      marienplatz_2l, karlsplatz_2r, 494, 80 / 3.6, true, 50);
  const auto e18rl = instance.get_editable_network().add_edge(
      marienplatz_2r, marienplatz_2l, 205, 80 / 3.6, true, 25);

  instance.get_editable_network().add_successor(e16lr, e17lr);
  instance.get_editable_network().add_successor(e17rl, e16rl);

  instance.get_editable_network().add_successor(e17lr, e18lr);
  instance.get_editable_network().add_successor(e18rl, e17rl);

  instance.add_empty_station("Marienplatz");
  instance.add_track_to_station("Marienplatz", e18lr);
  instance.add_track_to_station("Marienplatz", e18rl);

  // Marienplatz -> Isartor
  const auto isartor_switch_lr = instance.get_editable_network().add_vertex(
      "IsartorSwitchLR", cda_rail::VertexType::TTD);
  const auto isartor_switch_rl = instance.get_editable_network().add_vertex(
      "IsartorSwitchRL", cda_rail::VertexType::TTD);
  const auto isartor_1l = instance.get_editable_network().add_vertex(
      "Isartor1L", cda_rail::VertexType::TTD);
  const auto isartor_2l = instance.get_editable_network().add_vertex(
      "Isartor2L", cda_rail::VertexType::TTD);
  const auto isartor_1r = instance.get_editable_network().add_vertex(
      "Isartor1R", cda_rail::VertexType::TTD);
  const auto isartor_2r = instance.get_editable_network().add_vertex(
      "Isartor2R", cda_rail::VertexType::TTD);

  const auto e19lr = instance.get_editable_network().add_edge(
      marienplatz_1r, isartor_switch_lr, 393, 80 / 3.6, true, 50);
  const auto e20lr = instance.get_editable_network().add_edge(
      isartor_switch_lr, isartor_1l, 100, 80 / 3.6, false, 50);
  const auto e21lr = instance.get_editable_network().add_edge(
      isartor_1l, isartor_1r, 209, 80 / 3.6, true, 25);
  const auto e19rl = instance.get_editable_network().add_edge(
      isartor_switch_rl, marienplatz_2r, 343, 80 / 3.6, true, 50);
  const auto e20rl = instance.get_editable_network().add_edge(
      isartor_2l, isartor_switch_rl, 150, 80 / 3.6, false, 50);
  const auto e21rl = instance.get_editable_network().add_edge(
      isartor_2r, isartor_2l, 209, 80 / 3.6, true, 25);

  instance.get_editable_network().add_successor(e18lr, e19lr);
  instance.get_editable_network().add_successor(e19rl, e18rl);

  instance.get_editable_network().add_successor(e19lr, e20lr);
  instance.get_editable_network().add_successor(e20lr, e21lr);
  instance.get_editable_network().add_successor(e21rl, e20rl);
  instance.get_editable_network().add_successor(e20rl, e19rl);

  instance.add_empty_station("Isartor");
  instance.add_track_to_station("Isartor", e21lr);
  instance.add_track_to_station("Isartor", e21rl);

  // Isartor -> Rosenheimer Platz
  const auto isartor_switch_r_lr = instance.get_editable_network().add_vertex(
      "IsartorSwitch_R_LR", cda_rail::VertexType::TTD);
  const auto isartor_switch_r_rl = instance.get_editable_network().add_vertex(
      "IsartorSwitch_R_RL", cda_rail::VertexType::TTD);
  const auto rosenheimer_1l = instance.get_editable_network().add_vertex(
      "Rosenheimer1L", cda_rail::VertexType::TTD);
  const auto rosenheimer_2l = instance.get_editable_network().add_vertex(
      "Rosenheimer2L", cda_rail::VertexType::TTD);
  const auto rosenheimer_1r = instance.get_editable_network().add_vertex(
      "Rosenheimer1R", cda_rail::VertexType::TTD);
  const auto rosenheimer_2r = instance.get_editable_network().add_vertex(
      "Rosenheimer2R", cda_rail::VertexType::TTD);

  const auto e22lr = instance.get_editable_network().add_edge(
      isartor_1r, isartor_switch_r_lr, 100, 80 / 3.6, false, 50);
  const auto e23lr = instance.get_editable_network().add_edge(
      isartor_switch_r_lr, rosenheimer_1l, 592, 80 / 3.6, true, 50);
  const auto e24lr = instance.get_editable_network().add_edge(
      rosenheimer_1l, rosenheimer_1r, 206, 80 / 3.6, true, 25);
  const auto e22rl = instance.get_editable_network().add_edge(
      isartor_switch_r_rl, isartor_2r, 150, 80 / 3.6, false, 50);
  const auto e23rl = instance.get_editable_network().add_edge(
      rosenheimer_2l, isartor_switch_r_rl, 542, 80 / 3.6, true, 50);
  const auto e24rl = instance.get_editable_network().add_edge(
      rosenheimer_2r, rosenheimer_2l, 206, 80 / 3.6, true, 25);

  const auto switch_it_1 = instance.get_editable_network().add_edge(
      isartor_switch_lr, isartor_switch_rl, 50, 60 / 3.6, false, 50);
  const auto switch_it_2 = instance.get_editable_network().add_edge(
      isartor_switch_rl, isartor_switch_lr, 50, 60 / 3.6, false, 50);
  const auto switch_it_3 = instance.get_editable_network().add_edge(
      isartor_switch_r_lr, isartor_switch_r_rl, 50, 60 / 3.6, false, 50);
  const auto switch_it_4 = instance.get_editable_network().add_edge(
      isartor_switch_r_rl, isartor_switch_r_lr, 50, 60 / 3.6, false, 50);
  const auto tmp_1 = instance.get_editable_network().add_edge(
      isartor_switch_r_lr, isartor_1r, 100, 80 / 3.6, false, 50);
  const auto tmp_2 = instance.get_editable_network().add_edge(
      isartor_1r, isartor_1l, 209, 80 / 3.6, true, 25);
  const auto tmp_3 = instance.get_editable_network().add_edge(
      isartor_1l, isartor_switch_lr, 100, 80 / 3.6, false, 50);
  instance.get_editable_network().add_successor(switch_it_2, e20lr);
  instance.get_editable_network().add_successor(e22lr, switch_it_3);
  instance.get_editable_network().add_successor(e23rl, switch_it_4);
  instance.get_editable_network().add_successor(switch_it_4, tmp_1);
  instance.get_editable_network().add_successor(tmp_1, tmp_2);
  instance.get_editable_network().add_successor(tmp_2, tmp_3);
  instance.get_editable_network().add_successor(tmp_3, switch_it_1);

  instance.get_editable_network().add_successor(e21lr, e22lr);
  instance.get_editable_network().add_successor(e22rl, e21rl);

  instance.get_editable_network().add_successor(e22lr, e23lr);
  instance.get_editable_network().add_successor(e23lr, e24lr);
  instance.get_editable_network().add_successor(e24rl, e23rl);
  instance.get_editable_network().add_successor(e23rl, e22rl);

  instance.add_empty_station("Rosenheimer Platz");
  instance.add_track_to_station("Rosenheimer Platz", e24lr);
  instance.add_track_to_station("Rosenheimer Platz", e24rl);

  // Rosenheimer Platz -> Ostbahnhof
  const auto ost_switch4_lr = instance.get_editable_network().add_vertex(
      "OstSwitch4_LR", cda_rail::VertexType::TTD);
  const auto ost_switch5_lr = instance.get_editable_network().add_vertex(
      "OstSwitch5_LR", cda_rail::VertexType::TTD);
  const auto ost_switch1_rl = instance.get_editable_network().add_vertex(
      "OstSwitch1_RL", cda_rail::VertexType::TTD);
  const auto ost_switch2_rl = instance.get_editable_network().add_vertex(
      "OstSwitch2_RL", cda_rail::VertexType::TTD);
  const auto ost_switch3_rl = instance.get_editable_network().add_vertex(
      "OstSwitch3_RL", cda_rail::VertexType::TTD);
  const auto ost_1_entry = instance.get_editable_network().add_vertex(
      "Ost1Entry", cda_rail::VertexType::TTD);
  const auto ost_2_entry = instance.get_editable_network().add_vertex(
      "Ost2Entry", cda_rail::VertexType::TTD);
  const auto ost_3_entry = instance.get_editable_network().add_vertex(
      "Ost3Entry", cda_rail::VertexType::TTD);
  const auto ost_4_exit = instance.get_editable_network().add_vertex(
      "Ost4Exit", cda_rail::VertexType::TTD);
  const auto ost_5_exit = instance.get_editable_network().add_vertex(
      "Ost5Exit", cda_rail::VertexType::TTD);

  const auto e25lr = instance.get_editable_network().add_edge(
      rosenheimer_1r, ost_switch5_lr, 792, 80 / 3.6, true, 50);
  const auto e26lr_4 = instance.get_editable_network().add_edge(
      ost_switch5_lr, ost_switch4_lr, 60, 80 / 3.6, false, 50);
  const auto e27lr_4 = instance.get_editable_network().add_edge(
      ost_switch4_lr, ost_4_exit, 40, 80 / 3.6, false, 50);
  const auto e26lr_5 = instance.get_editable_network().add_edge(
      ost_switch5_lr, ost_5_exit, 100, 80 / 3.6, false, 50);
  const auto e25rl = instance.get_editable_network().add_edge(
      ost_switch1_rl, rosenheimer_2r, 752, 80 / 3.6, true, 50);
  const auto e26rl_1 = instance.get_editable_network().add_edge(
      ost_1_entry, ost_switch1_rl, 140, 80 / 3.6, false, 50);
  const auto e26rl_23 = instance.get_editable_network().add_edge(
      ost_switch2_rl, ost_switch1_rl, 40, 80 / 3.6, false, 50);
  const auto e27rl_2 = instance.get_editable_network().add_edge(
      ost_2_entry, ost_switch2_rl, 100, 80 / 3.6, false, 50);
  const auto e27rl_3 = instance.get_editable_network().add_edge(
      ost_switch3_rl, ost_switch2_rl, 60, 80 / 3.6, false, 50);
  const auto e28rl_3 = instance.get_editable_network().add_edge(
      ost_3_entry, ost_switch3_rl, 40, 80 / 3.6, false, 50);

  instance.get_editable_network().add_successor(e24lr, e25lr);
  instance.get_editable_network().add_successor(e25rl, e24rl);

  instance.get_editable_network().add_successor(e25lr, e26lr_4);
  instance.get_editable_network().add_successor(e26lr_4, e27lr_4);
  instance.get_editable_network().add_successor(e25lr, e26lr_5);
  instance.get_editable_network().add_successor(e26rl_1, e25rl);
  instance.get_editable_network().add_successor(e27rl_2, e26rl_23);
  instance.get_editable_network().add_successor(e26rl_23, e25rl);
  instance.get_editable_network().add_successor(e28rl_3, e27rl_3);
  instance.get_editable_network().add_successor(e27rl_3, e26rl_23);

  // NOLINTBEGIN(bugprone-swapped-arguments)

  // Add S2 Petershausen
  instance.add_train("S2Petershausen", 135, 140 / 3.6, 1, 0.9, 0, 0,
                     ost_2_entry, 17.25 * 60, 20, laim_exit_nymphenburg);
  instance.insert_stop("S2Petershausen", "Rosenheimer Platz", 1.5 * 60, 30);
  instance.insert_stop("S2Petershausen", "Isartor", 3.5 * 60, 30);
  instance.insert_stop("S2Petershausen", "Marienplatz", 5.25 * 60, 30);
  instance.insert_stop("S2Petershausen", "Karlsplatz", 7 * 60, 30);
  instance.insert_stop("S2Petershausen", "Hbf", 8.5 * 60, 30);
  instance.insert_stop("S2Petershausen", "Hackerbruecke", 10.25 * 60, 30);
  instance.insert_stop("S2Petershausen", "Donnersbergerbruecke", 12 * 60, 30);
  instance.insert_stop("S2Petershausen", "Hirschgarten", 14 * 60, 30);
  instance.insert_stop("S2Petershausen", "Laim", 16 * 60, 30);
  instance.add_empty_route("S2Petershausen");
  instance.push_back_edge_to_route("S2Petershausen", e27rl_2);
  instance.push_back_edge_to_route("S2Petershausen", e26rl_23);
  instance.push_back_edge_to_route("S2Petershausen", e25rl);
  instance.push_back_edge_to_route("S2Petershausen", e24rl);
  instance.push_back_edge_to_route("S2Petershausen", e23rl);
  instance.push_back_edge_to_route("S2Petershausen", e22rl);
  instance.push_back_edge_to_route("S2Petershausen", e21rl);
  instance.push_back_edge_to_route("S2Petershausen", e20rl);
  instance.push_back_edge_to_route("S2Petershausen", e19rl);
  instance.push_back_edge_to_route("S2Petershausen", e18rl);
  instance.push_back_edge_to_route("S2Petershausen", e17rl);
  instance.push_back_edge_to_route("S2Petershausen", e16rl);
  instance.push_back_edge_to_route("S2Petershausen", e15rl);
  instance.push_back_edge_to_route("S2Petershausen", e14rl);
  instance.push_back_edge_to_route("S2Petershausen", e13rl);
  instance.push_back_edge_to_route("S2Petershausen", e12rl);
  instance.push_back_edge_to_route("S2Petershausen", e11rl);
  instance.push_back_edge_to_route("S2Petershausen", e10rl);
  instance.push_back_edge_to_route("S2Petershausen", e9rl_b);
  instance.push_back_edge_to_route("S2Petershausen", e9rl_a);
  instance.push_back_edge_to_route("S2Petershausen", e8rl);
  instance.push_back_edge_to_route("S2Petershausen", e7rl);
  instance.push_back_edge_to_route("S2Petershausen", e6rl);
  instance.push_back_edge_to_route("S2Petershausen", e5rl);
  instance.push_back_edge_to_route("S2Petershausen", e4rl);
  instance.push_back_edge_to_route("S2Petershausen", e3rl);
  instance.push_back_edge_to_route("S2Petershausen", e2rl_exit);

  // Add S6Tutzing
  instance.add_train("S6Tutzing", 202, 140 / 3.6, 1, 0.9, 1.5 * 60, 0,
                     ost_1_entry, 20.5 * 60, 0, pasing_exit);
  instance.insert_stop("S6Tutzing", "Rosenheimer Platz", 3 * 60, 30);
  instance.insert_stop("S6Tutzing", "Isartor", 5 * 60, 30);
  instance.insert_stop("S6Tutzing", "Marienplatz", 6.75 * 60, 30);
  instance.insert_stop("S6Tutzing", "Karlsplatz", 8.5 * 60, 30);
  instance.insert_stop("S6Tutzing", "Hbf", 10 * 60, 30);
  instance.insert_stop("S6Tutzing", "Hackerbruecke", 11.75 * 60, 30);
  instance.insert_stop("S6Tutzing", "Donnersbergerbruecke", 13.5 * 60, 30);
  instance.insert_stop("S6Tutzing", "Hirschgarten", 15.5 * 60, 30);
  instance.insert_stop("S6Tutzing", "Laim", 17.5 * 60, 30);
  instance.add_empty_route("S6Tutzing");
  instance.push_back_edge_to_route("S6Tutzing", e26rl_1);
  instance.push_back_edge_to_route("S6Tutzing", e25rl);
  instance.push_back_edge_to_route("S6Tutzing", e24rl);
  instance.push_back_edge_to_route("S6Tutzing", e23rl);
  instance.push_back_edge_to_route("S6Tutzing", e22rl);
  instance.push_back_edge_to_route("S6Tutzing", e21rl);
  instance.push_back_edge_to_route("S6Tutzing", e20rl);
  instance.push_back_edge_to_route("S6Tutzing", e19rl);
  instance.push_back_edge_to_route("S6Tutzing", e18rl);
  instance.push_back_edge_to_route("S6Tutzing", e17rl);
  instance.push_back_edge_to_route("S6Tutzing", e16rl);
  instance.push_back_edge_to_route("S6Tutzing", e15rl);
  instance.push_back_edge_to_route("S6Tutzing", e14rl);
  instance.push_back_edge_to_route("S6Tutzing", e13rl);
  instance.push_back_edge_to_route("S6Tutzing", e12rl);
  instance.push_back_edge_to_route("S6Tutzing", e11rl);
  instance.push_back_edge_to_route("S6Tutzing", e10rl);
  instance.push_back_edge_to_route("S6Tutzing", e9rl_b);
  instance.push_back_edge_to_route("S6Tutzing", e9rl_a);
  instance.push_back_edge_to_route("S6Tutzing", e8rl);
  instance.push_back_edge_to_route("S6Tutzing", e7rl);
  instance.push_back_edge_to_route("S6Tutzing", e6rl);
  instance.push_back_edge_to_route("S6Tutzing", e5rl);
  instance.push_back_edge_to_route("S6Tutzing", e4rl);
  instance.push_back_edge_to_route("S6Tutzing", e3rl);
  instance.push_back_edge_to_route("S6Tutzing", e2rl);
  instance.push_back_edge_to_route("S6Tutzing", e1rl);

  // Add S7 Wolfratshausen
  instance.add_train("S7Wolfratshausen", 135, 140 / 3.6, 1, 0.9, 2.75 * 60, 0,
                     ost_3_entry, 14.75 * 60, 0, donnersberger_exit);
  instance.insert_stop("S7Wolfratshausen", "Rosenheimer Platz", 4.25 * 60, 30);
  instance.insert_stop("S7Wolfratshausen", "Isartor", 6.25 * 60, 30);
  instance.insert_stop("S7Wolfratshausen", "Marienplatz", 8 * 60, 30);
  instance.insert_stop("S7Wolfratshausen", "Karlsplatz", 9.75 * 60, 30);
  instance.insert_stop("S7Wolfratshausen", "Hbf", 11.25 * 60, 30);
  instance.insert_stop("S7Wolfratshausen", "Hackerbruecke", 13 * 60, 30);
  instance.add_empty_route("S7Wolfratshausen");
  instance.push_back_edge_to_route("S7Wolfratshausen", e28rl_3);
  instance.push_back_edge_to_route("S7Wolfratshausen", e27rl_3);
  instance.push_back_edge_to_route("S7Wolfratshausen", e26rl_23);
  instance.push_back_edge_to_route("S7Wolfratshausen", e25rl);
  instance.push_back_edge_to_route("S7Wolfratshausen", e24rl);
  instance.push_back_edge_to_route("S7Wolfratshausen", e23rl);
  instance.push_back_edge_to_route("S7Wolfratshausen", e22rl);
  instance.push_back_edge_to_route("S7Wolfratshausen", e21rl);
  instance.push_back_edge_to_route("S7Wolfratshausen", e20rl);
  instance.push_back_edge_to_route("S7Wolfratshausen", e19rl);
  instance.push_back_edge_to_route("S7Wolfratshausen", e18rl);
  instance.push_back_edge_to_route("S7Wolfratshausen", e17rl);
  instance.push_back_edge_to_route("S7Wolfratshausen", e16rl);
  instance.push_back_edge_to_route("S7Wolfratshausen", e15rl);
  instance.push_back_edge_to_route("S7Wolfratshausen", e14rl);
  instance.push_back_edge_to_route("S7Wolfratshausen", e13rl);
  instance.push_back_edge_to_route("S7Wolfratshausen", e12rl);
  instance.push_back_edge_to_route("S7Wolfratshausen", e11rl);
  instance.push_back_edge_to_route("S7Wolfratshausen", e10rl);
  instance.push_back_edge_to_route("S7Wolfratshausen", e9rl_b);
  instance.push_back_edge_to_route("S7Wolfratshausen", e9rl_exit);

  // Add S8 Germering
  instance.add_train("S8Germering", 135, 140 / 3.6, 1, 0.9, 4.25 * 60, 0,
                     ost_1_entry, 23.25 * 60, 0, pasing_exit);
  instance.insert_stop("S8Germering", "Rosenheimer Platz", 5.75 * 60, 30);
  instance.insert_stop("S8Germering", "Isartor", 7.75 * 60, 30);
  instance.insert_stop("S8Germering", "Marienplatz", 9.5 * 60, 30);
  instance.insert_stop("S8Germering", "Karlsplatz", 11.25 * 60, 30);
  instance.insert_stop("S8Germering", "Hbf", 12.75 * 60, 30);
  instance.insert_stop("S8Germering", "Hackerbruecke", 14.5 * 60, 30);
  instance.insert_stop("S8Germering", "Donnersbergerbruecke", 16.25 * 60, 30);
  instance.insert_stop("S8Germering", "Hirschgarten", 18.25 * 60, 30);
  instance.insert_stop("S8Germering", "Laim", 20.25 * 60, 30);
  instance.add_empty_route("S8Germering");
  instance.push_back_edge_to_route("S8Germering", e26rl_1);
  instance.push_back_edge_to_route("S8Germering", e25rl);
  instance.push_back_edge_to_route("S8Germering", e24rl);
  instance.push_back_edge_to_route("S8Germering", e23rl);
  instance.push_back_edge_to_route("S8Germering", e22rl);
  instance.push_back_edge_to_route("S8Germering", e21rl);
  instance.push_back_edge_to_route("S8Germering", e20rl);
  instance.push_back_edge_to_route("S8Germering", e19rl);
  instance.push_back_edge_to_route("S8Germering", e18rl);
  instance.push_back_edge_to_route("S8Germering", e17rl);
  instance.push_back_edge_to_route("S8Germering", e16rl);
  instance.push_back_edge_to_route("S8Germering", e15rl);
  instance.push_back_edge_to_route("S8Germering", e14rl);
  instance.push_back_edge_to_route("S8Germering", e13rl);
  instance.push_back_edge_to_route("S8Germering", e12rl);
  instance.push_back_edge_to_route("S8Germering", e11rl);
  instance.push_back_edge_to_route("S8Germering", e10rl);
  instance.push_back_edge_to_route("S8Germering", e9rl_b);
  instance.push_back_edge_to_route("S8Germering", e9rl_a);
  instance.push_back_edge_to_route("S8Germering", e8rl);
  instance.push_back_edge_to_route("S8Germering", e7rl);
  instance.push_back_edge_to_route("S8Germering", e6rl);
  instance.push_back_edge_to_route("S8Germering", e5rl);
  instance.push_back_edge_to_route("S8Germering", e4rl);
  instance.push_back_edge_to_route("S8Germering", e3rl);
  instance.push_back_edge_to_route("S8Germering", e2rl);
  instance.push_back_edge_to_route("S8Germering", e1rl);

  // S3 Mammendorf
  instance.add_train("S3Mammendorf", 135, 140 / 3.6, 1, 0.9, 5.5 * 60, 0,
                     ost_3_entry, 24.75 * 60, 0, pasing_exit);
  instance.insert_stop("S3Mammendorf", "Rosenheimer Platz", 7 * 60, 30);
  instance.insert_stop("S3Mammendorf", "Isartor", 9 * 60, 30);
  instance.insert_stop("S3Mammendorf", "Marienplatz", 10.75 * 60, 30);
  instance.insert_stop("S3Mammendorf", "Karlsplatz", 12.5 * 60, 30);
  instance.insert_stop("S3Mammendorf", "Hbf", 14.5 * 60, 30);
  instance.insert_stop("S3Mammendorf", "Hackerbruecke", 16.25 * 60, 30);
  instance.insert_stop("S3Mammendorf", "Donnersbergerbruecke", 18 * 60, 30);
  instance.insert_stop("S3Mammendorf", "Hirschgarten", 20 * 60, 30);
  instance.insert_stop("S3Mammendorf", "Laim", 22 * 60, 30);
  instance.add_empty_route("S3Mammendorf");
  instance.push_back_edge_to_route("S3Mammendorf", e28rl_3);
  instance.push_back_edge_to_route("S3Mammendorf", e27rl_3);
  instance.push_back_edge_to_route("S3Mammendorf", e26rl_23);
  instance.push_back_edge_to_route("S3Mammendorf", e25rl);
  instance.push_back_edge_to_route("S3Mammendorf", e24rl);
  instance.push_back_edge_to_route("S3Mammendorf", e23rl);
  instance.push_back_edge_to_route("S3Mammendorf", e22rl);
  instance.push_back_edge_to_route("S3Mammendorf", e21rl);
  instance.push_back_edge_to_route("S3Mammendorf", e20rl);
  instance.push_back_edge_to_route("S3Mammendorf", e19rl);
  instance.push_back_edge_to_route("S3Mammendorf", e18rl);
  instance.push_back_edge_to_route("S3Mammendorf", e17rl);
  instance.push_back_edge_to_route("S3Mammendorf", e16rl);
  instance.push_back_edge_to_route("S3Mammendorf", e15rl);
  instance.push_back_edge_to_route("S3Mammendorf", e14rl);
  instance.push_back_edge_to_route("S3Mammendorf", e13rl);
  instance.push_back_edge_to_route("S3Mammendorf", e12rl);
  instance.push_back_edge_to_route("S3Mammendorf", e11rl);
  instance.push_back_edge_to_route("S3Mammendorf", e10rl);
  instance.push_back_edge_to_route("S3Mammendorf", e9rl_b);
  instance.push_back_edge_to_route("S3Mammendorf", e9rl_a);
  instance.push_back_edge_to_route("S3Mammendorf", e8rl);
  instance.push_back_edge_to_route("S3Mammendorf", e7rl);
  instance.push_back_edge_to_route("S3Mammendorf", e6rl);
  instance.push_back_edge_to_route("S3Mammendorf", e5rl);
  instance.push_back_edge_to_route("S3Mammendorf", e4rl);
  instance.push_back_edge_to_route("S3Mammendorf", e3rl);
  instance.push_back_edge_to_route("S3Mammendorf", e2rl);
  instance.push_back_edge_to_route("S3Mammendorf", e1rl);

  // S2 Dachau
  instance.add_train("S2Dachau", 202, 140 / 3.6, 1, 0.9, 7 * 60, 0, ost_2_entry,
                     24.75 * 60, 20, laim_exit_nymphenburg);
  instance.insert_stop("S2Dachau", "Rosenheimer Platz", 8.5 * 60, 30);
  instance.insert_stop("S2Dachau", "Isartor", 10.5 * 60, 30);
  instance.insert_stop("S2Dachau", "Marienplatz", 12.25 * 60, 30);
  instance.insert_stop("S2Dachau", "Karlsplatz", 14 * 60, 30);
  instance.insert_stop("S2Dachau", "Hbf", 16 * 60, 30);
  instance.insert_stop("S2Dachau", "Hackerbruecke", 17.75 * 60, 30);
  instance.insert_stop("S2Dachau", "Donnersbergerbruecke", 19.5 * 60, 30);
  instance.insert_stop("S2Dachau", "Hirschgarten", 21.5 * 60, 30);
  instance.insert_stop("S2Dachau", "Laim", 23.5 * 60, 30);
  instance.add_empty_route("S2Dachau");
  instance.push_back_edge_to_route("S2Dachau", e27rl_2);
  instance.push_back_edge_to_route("S2Dachau", e26rl_23);
  instance.push_back_edge_to_route("S2Dachau", e25rl);
  instance.push_back_edge_to_route("S2Dachau", e24rl);
  instance.push_back_edge_to_route("S2Dachau", e23rl);
  instance.push_back_edge_to_route("S2Dachau", e22rl);
  instance.push_back_edge_to_route("S2Dachau", e21rl);
  instance.push_back_edge_to_route("S2Dachau", e20rl);
  instance.push_back_edge_to_route("S2Dachau", e19rl);
  instance.push_back_edge_to_route("S2Dachau", e18rl);
  instance.push_back_edge_to_route("S2Dachau", e17rl);
  instance.push_back_edge_to_route("S2Dachau", e16rl);
  instance.push_back_edge_to_route("S2Dachau", e15rl);
  instance.push_back_edge_to_route("S2Dachau", e14rl);
  instance.push_back_edge_to_route("S2Dachau", e13rl);
  instance.push_back_edge_to_route("S2Dachau", e12rl);
  instance.push_back_edge_to_route("S2Dachau", e11rl);
  instance.push_back_edge_to_route("S2Dachau", e10rl);
  instance.push_back_edge_to_route("S2Dachau", e9rl_b);
  instance.push_back_edge_to_route("S2Dachau", e9rl_a);
  instance.push_back_edge_to_route("S2Dachau", e8rl);
  instance.push_back_edge_to_route("S2Dachau", e7rl);
  instance.push_back_edge_to_route("S2Dachau", e6rl);
  instance.push_back_edge_to_route("S2Dachau", e5rl);
  instance.push_back_edge_to_route("S2Dachau", e4rl);
  instance.push_back_edge_to_route("S2Dachau", e3rl);
  instance.push_back_edge_to_route("S2Dachau", e2rl_exit);

  // s4 Geltendorf
  instance.add_train("S4Geltendorf", 202, 140 / 3.6, 1, 0.9, 8.5 * 60, 0,
                     ost_1_entry, 28 * 60, 0, pasing_exit);
  instance.insert_stop("S4Geltendorf", "Rosenheimer Platz", 10 * 60, 30);
  instance.insert_stop("S4Geltendorf", "Isartor", 12 * 60, 30);
  instance.insert_stop("S4Geltendorf", "Marienplatz", 13.75 * 60, 30);
  instance.insert_stop("S4Geltendorf", "Karlsplatz", 15.5 * 60, 30);
  instance.insert_stop("S4Geltendorf", "Hbf", 17.5 * 60, 30);
  instance.insert_stop("S4Geltendorf", "Hackerbruecke", 19.25 * 60, 30);
  instance.insert_stop("S4Geltendorf", "Donnersbergerbruecke", 21 * 60, 30);
  instance.insert_stop("S4Geltendorf", "Hirschgarten", 23 * 60, 30);
  instance.insert_stop("S4Geltendorf", "Laim", 25 * 60, 30);
  instance.add_empty_route("S4Geltendorf");
  instance.push_back_edge_to_route("S4Geltendorf", e26rl_1);
  instance.push_back_edge_to_route("S4Geltendorf", e25rl);
  instance.push_back_edge_to_route("S4Geltendorf", e24rl);
  instance.push_back_edge_to_route("S4Geltendorf", e23rl);
  instance.push_back_edge_to_route("S4Geltendorf", e22rl);
  instance.push_back_edge_to_route("S4Geltendorf", e21rl);
  instance.push_back_edge_to_route("S4Geltendorf", e20rl);
  instance.push_back_edge_to_route("S4Geltendorf", e19rl);
  instance.push_back_edge_to_route("S4Geltendorf", e18rl);
  instance.push_back_edge_to_route("S4Geltendorf", e17rl);
  instance.push_back_edge_to_route("S4Geltendorf", e16rl);
  instance.push_back_edge_to_route("S4Geltendorf", e15rl);
  instance.push_back_edge_to_route("S4Geltendorf", e14rl);
  instance.push_back_edge_to_route("S4Geltendorf", e13rl);
  instance.push_back_edge_to_route("S4Geltendorf", e12rl);
  instance.push_back_edge_to_route("S4Geltendorf", e11rl);
  instance.push_back_edge_to_route("S4Geltendorf", e10rl);
  instance.push_back_edge_to_route("S4Geltendorf", e9rl_b);
  instance.push_back_edge_to_route("S4Geltendorf", e9rl_a);
  instance.push_back_edge_to_route("S4Geltendorf", e8rl);
  instance.push_back_edge_to_route("S4Geltendorf", e7rl);
  instance.push_back_edge_to_route("S4Geltendorf", e6rl);
  instance.push_back_edge_to_route("S4Geltendorf", e5rl);
  instance.push_back_edge_to_route("S4Geltendorf", e4rl);
  instance.push_back_edge_to_route("S4Geltendorf", e3rl);
  instance.push_back_edge_to_route("S4Geltendorf", e2rl);
  instance.push_back_edge_to_route("S4Geltendorf", e1rl);

  // S1 Freising

  instance.add_train("S1Freising", 135, 140 / 3.6, 1, 0.9, 10 * 60, 0,
                     ost_2_entry, 27.75 * 60, 20, laim_exit_nymphenburg);
  instance.insert_stop("S1Freising", "Rosenheimer Platz", 11.5 * 60, 30);
  instance.insert_stop("S1Freising", "Isartor", 13.5 * 60, 30);
  instance.insert_stop("S1Freising", "Marienplatz", 15.25 * 60, 30);
  instance.insert_stop("S1Freising", "Karlsplatz", 17 * 60, 30);
  instance.insert_stop("S1Freising", "Hbf", 19 * 60, 30);
  instance.insert_stop("S1Freising", "Hackerbruecke", 20.75 * 60, 30);
  instance.insert_stop("S1Freising", "Donnersbergerbruecke", 22.5 * 60, 30);
  instance.insert_stop("S1Freising", "Hirschgarten", 24.5 * 60, 30);
  instance.insert_stop("S1Freising", "Laim", 26.5 * 60, 30);
  instance.add_empty_route("S1Freising");
  instance.push_back_edge_to_route("S1Freising", e27rl_2);
  instance.push_back_edge_to_route("S1Freising", e26rl_23);
  instance.push_back_edge_to_route("S1Freising", e25rl);
  instance.push_back_edge_to_route("S1Freising", e24rl);
  instance.push_back_edge_to_route("S1Freising", e23rl);
  instance.push_back_edge_to_route("S1Freising", e22rl);
  instance.push_back_edge_to_route("S1Freising", e21rl);
  instance.push_back_edge_to_route("S1Freising", e20rl);
  instance.push_back_edge_to_route("S1Freising", e19rl);
  instance.push_back_edge_to_route("S1Freising", e18rl);
  instance.push_back_edge_to_route("S1Freising", e17rl);
  instance.push_back_edge_to_route("S1Freising", e16rl);
  instance.push_back_edge_to_route("S1Freising", e15rl);
  instance.push_back_edge_to_route("S1Freising", e14rl);
  instance.push_back_edge_to_route("S1Freising", e13rl);
  instance.push_back_edge_to_route("S1Freising", e12rl);
  instance.push_back_edge_to_route("S1Freising", e11rl);
  instance.push_back_edge_to_route("S1Freising", e10rl);
  instance.push_back_edge_to_route("S1Freising", e9rl_b);
  instance.push_back_edge_to_route("S1Freising", e9rl_a);
  instance.push_back_edge_to_route("S1Freising", e8rl);
  instance.push_back_edge_to_route("S1Freising", e7rl);
  instance.push_back_edge_to_route("S1Freising", e6rl);
  instance.push_back_edge_to_route("S1Freising", e5rl);
  instance.push_back_edge_to_route("S1Freising", e4rl);
  instance.push_back_edge_to_route("S1Freising", e3rl);
  instance.push_back_edge_to_route("S1Freising", e2rl_exit);

  /**
  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  !!!!!!!!!! REVERSE DIRECTION !!!!!!!!!!!!!!!!!!!!!!!!!!!!
  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  **/

  // S7 Aying
  instance.add_train("S7Aying", 202, 140 / 3.6, 1, 0.9, 5.75 * 60, 0,
                     donnersberger_entry, 18.5 * 60, 0, ost_4_exit);
  instance.insert_stop("S7Aying", "Hackerbruecke", 7 * 60, 30);
  instance.insert_stop("S7Aying", "Hbf", 9 * 60, 30);
  instance.insert_stop("S7Aying", "Karlsplatz", 11 * 60, 30);
  instance.insert_stop("S7Aying", "Marienplatz", 12.75 * 60, 30);
  instance.insert_stop("S7Aying", "Isartor", 14.5 * 60, 30);
  instance.insert_stop("S7Aying", "Rosenheimer Platz", 16.5 * 60, 30);
  instance.add_empty_route("S7Aying");
  instance.push_back_edge_to_route("S7Aying", e9lr_entry);
  instance.push_back_edge_to_route("S7Aying", e9lr_b);
  instance.push_back_edge_to_route("S7Aying", e10lr);
  instance.push_back_edge_to_route("S7Aying", e11lr);
  instance.push_back_edge_to_route("S7Aying", e12lr);
  instance.push_back_edge_to_route("S7Aying", e13lr);
  instance.push_back_edge_to_route("S7Aying", e14lr);
  instance.push_back_edge_to_route("S7Aying", e15lr);
  instance.push_back_edge_to_route("S7Aying", e16lr);
  instance.push_back_edge_to_route("S7Aying", e17lr);
  instance.push_back_edge_to_route("S7Aying", e18lr);
  instance.push_back_edge_to_route("S7Aying", e19lr);
  instance.push_back_edge_to_route("S7Aying", e20lr);
  instance.push_back_edge_to_route("S7Aying", e21lr);
  instance.push_back_edge_to_route("S7Aying", e22lr);
  instance.push_back_edge_to_route("S7Aying", e23lr);
  instance.push_back_edge_to_route("S7Aying", e24lr);
  instance.push_back_edge_to_route("S7Aying", e25lr);
  instance.push_back_edge_to_route("S7Aying", e26lr_4);
  instance.push_back_edge_to_route("S7Aying", e27lr_4);

  // S6 Ebersberg
  instance.add_train("S6Ebersberg", 135, 140 / 3.6, 1, 0.9, 0 * 60, 0,
                     pasing_entry, 19.75 * 60, 0, ost_5_exit);
  instance.insert_stop("S6Ebersberg", "Laim", 2.5 * 60, 30);
  instance.insert_stop("S6Ebersberg", "Hirschgarten", 4.5 * 60, 30);
  instance.insert_stop("S6Ebersberg", "Donnersbergerbruecke", 6.5 * 60, 30);
  instance.insert_stop("S6Ebersberg", "Hackerbruecke", 8.25 * 60, 30);
  instance.insert_stop("S6Ebersberg", "Hbf", 10.25 * 60, 30);
  instance.insert_stop("S6Ebersberg", "Karlsplatz", 12.25 * 60, 30);
  instance.insert_stop("S6Ebersberg", "Marienplatz", 14 * 60, 30);
  instance.insert_stop("S6Ebersberg", "Isartor", 15.75 * 60, 30);
  instance.insert_stop("S6Ebersberg", "Rosenheimer Platz", 17.75 * 60, 30);
  instance.add_empty_route("S6Ebersberg");
  instance.push_back_edge_to_route("S6Ebersberg", e1lr);
  instance.push_back_edge_to_route("S6Ebersberg", e2lr);
  instance.push_back_edge_to_route("S6Ebersberg", e3lr);
  instance.push_back_edge_to_route("S6Ebersberg", e4lr);
  instance.push_back_edge_to_route("S6Ebersberg", e5lr);
  instance.push_back_edge_to_route("S6Ebersberg", e6lr);
  instance.push_back_edge_to_route("S6Ebersberg", e7lr);
  instance.push_back_edge_to_route("S6Ebersberg", e8lr);
  instance.push_back_edge_to_route("S6Ebersberg", e9lr_a);
  instance.push_back_edge_to_route("S6Ebersberg", e9lr_b);
  instance.push_back_edge_to_route("S6Ebersberg", e10lr);
  instance.push_back_edge_to_route("S6Ebersberg", e11lr);
  instance.push_back_edge_to_route("S6Ebersberg", e12lr);
  instance.push_back_edge_to_route("S6Ebersberg", e13lr);
  instance.push_back_edge_to_route("S6Ebersberg", e14lr);
  instance.push_back_edge_to_route("S6Ebersberg", e15lr);
  instance.push_back_edge_to_route("S6Ebersberg", e16lr);
  instance.push_back_edge_to_route("S6Ebersberg", e17lr);
  instance.push_back_edge_to_route("S6Ebersberg", e18lr);
  instance.push_back_edge_to_route("S6Ebersberg", e19lr);
  instance.push_back_edge_to_route("S6Ebersberg", e20lr);
  instance.push_back_edge_to_route("S6Ebersberg", e21lr);
  instance.push_back_edge_to_route("S6Ebersberg", e22lr);
  instance.push_back_edge_to_route("S6Ebersberg", e23lr);
  instance.push_back_edge_to_route("S6Ebersberg", e24lr);
  instance.push_back_edge_to_route("S6Ebersberg", e25lr);
  instance.push_back_edge_to_route("S6Ebersberg", e26lr_5);

  // S2 Erding
  instance.add_train("S2Erding", 135, 140 / 3.6, 1, 0.9, 4.5 * 60, 0,
                     laim_entry, 21.25 * 60, 0, ost_5_exit);
  instance.insert_stop("S2Erding", "Hirschgarten", 6 * 60, 30);
  instance.insert_stop("S2Erding", "Donnersbergerbruecke", 8 * 60, 30);
  instance.insert_stop("S2Erding", "Hackerbruecke", 9.75 * 60, 30);
  instance.insert_stop("S2Erding", "Hbf", 11.75 * 60, 30);
  instance.insert_stop("S2Erding", "Karlsplatz", 13.75 * 60, 30);
  instance.insert_stop("S2Erding", "Marienplatz", 15.5 * 60, 30);
  instance.insert_stop("S2Erding", "Isartor", 17.25 * 60, 30);
  instance.insert_stop("S2Erding", "Rosenheimer Platz", 19.25 * 60, 30);
  instance.add_empty_route("S2Erding");
  instance.push_back_edge_to_route("S2Erding", e4lr_entry);
  instance.push_back_edge_to_route("S2Erding", e5lr);
  instance.push_back_edge_to_route("S2Erding", e6lr);
  instance.push_back_edge_to_route("S2Erding", e7lr);
  instance.push_back_edge_to_route("S2Erding", e8lr);
  instance.push_back_edge_to_route("S2Erding", e9lr_a);
  instance.push_back_edge_to_route("S2Erding", e9lr_b);
  instance.push_back_edge_to_route("S2Erding", e10lr);
  instance.push_back_edge_to_route("S2Erding", e11lr);
  instance.push_back_edge_to_route("S2Erding", e12lr);
  instance.push_back_edge_to_route("S2Erding", e13lr);
  instance.push_back_edge_to_route("S2Erding", e14lr);
  instance.push_back_edge_to_route("S2Erding", e15lr);
  instance.push_back_edge_to_route("S2Erding", e16lr);
  instance.push_back_edge_to_route("S2Erding", e17lr);
  instance.push_back_edge_to_route("S2Erding", e18lr);
  instance.push_back_edge_to_route("S2Erding", e19lr);
  instance.push_back_edge_to_route("S2Erding", e20lr);
  instance.push_back_edge_to_route("S2Erding", e21lr);
  instance.push_back_edge_to_route("S2Erding", e22lr);
  instance.push_back_edge_to_route("S2Erding", e23lr);
  instance.push_back_edge_to_route("S2Erding", e24lr);
  instance.push_back_edge_to_route("S2Erding", e25lr);
  instance.push_back_edge_to_route("S2Erding", e26lr_5);

  // S3 Deisenhofen
  instance.add_train("S3Deisenhofen", 135, 140 / 3.6, 1, 0.9, 2.75 * 60, 0,
                     pasing_entry, 22.5 * 60, 0, ost_4_exit);
  instance.insert_stop("S3Deisenhofen", "Laim", 5.25 * 60, 30);
  instance.insert_stop("S3Deisenhofen", "Hirschgarten", 7.25 * 60, 30);
  instance.insert_stop("S3Deisenhofen", "Donnersbergerbruecke", 9.25 * 60, 30);
  instance.insert_stop("S3Deisenhofen", "Hackerbruecke", 11 * 60, 30);
  instance.insert_stop("S3Deisenhofen", "Hbf", 13 * 60, 30);
  instance.insert_stop("S3Deisenhofen", "Karlsplatz", 15 * 60, 30);
  instance.insert_stop("S3Deisenhofen", "Marienplatz", 16.75 * 60, 30);
  instance.insert_stop("S3Deisenhofen", "Isartor", 18.5 * 60, 30);
  instance.insert_stop("S3Deisenhofen", "Rosenheimer Platz", 20.5 * 60, 30);
  instance.add_empty_route("S3Deisenhofen");
  instance.push_back_edge_to_route("S3Deisenhofen", e1lr);
  instance.push_back_edge_to_route("S3Deisenhofen", e2lr);
  instance.push_back_edge_to_route("S3Deisenhofen", e3lr);
  instance.push_back_edge_to_route("S3Deisenhofen", e4lr);
  instance.push_back_edge_to_route("S3Deisenhofen", e5lr);
  instance.push_back_edge_to_route("S3Deisenhofen", e6lr);
  instance.push_back_edge_to_route("S3Deisenhofen", e7lr);
  instance.push_back_edge_to_route("S3Deisenhofen", e8lr);
  instance.push_back_edge_to_route("S3Deisenhofen", e9lr_a);
  instance.push_back_edge_to_route("S3Deisenhofen", e9lr_b);
  instance.push_back_edge_to_route("S3Deisenhofen", e10lr);
  instance.push_back_edge_to_route("S3Deisenhofen", e11lr);
  instance.push_back_edge_to_route("S3Deisenhofen", e12lr);
  instance.push_back_edge_to_route("S3Deisenhofen", e13lr);
  instance.push_back_edge_to_route("S3Deisenhofen", e14lr);
  instance.push_back_edge_to_route("S3Deisenhofen", e15lr);
  instance.push_back_edge_to_route("S3Deisenhofen", e16lr);
  instance.push_back_edge_to_route("S3Deisenhofen", e17lr);
  instance.push_back_edge_to_route("S3Deisenhofen", e18lr);
  instance.push_back_edge_to_route("S3Deisenhofen", e19lr);
  instance.push_back_edge_to_route("S3Deisenhofen", e20lr);
  instance.push_back_edge_to_route("S3Deisenhofen", e21lr);
  instance.push_back_edge_to_route("S3Deisenhofen", e22lr);
  instance.push_back_edge_to_route("S3Deisenhofen", e23lr);
  instance.push_back_edge_to_route("S3Deisenhofen", e24lr);
  instance.push_back_edge_to_route("S3Deisenhofen", e25lr);
  instance.push_back_edge_to_route("S3Deisenhofen", e26lr_4);
  instance.push_back_edge_to_route("S3Deisenhofen", e27lr_4);

  // S8 Airport
  instance.add_train("S8Airport", 202, 140 / 3.6, 1, 0.9, 4.25 * 60, 0,
                     pasing_entry, 24 * 60, 0, ost_5_exit);
  instance.insert_stop("S8Airport", "Laim", 6.75 * 60, 30);
  instance.insert_stop("S8Airport", "Hirschgarten", 8.75 * 60, 30);
  instance.insert_stop("S8Airport", "Donnersbergerbruecke", 10.75 * 60, 30);
  instance.insert_stop("S8Airport", "Hackerbruecke", 12.5 * 60, 30);
  instance.insert_stop("S8Airport", "Hbf", 14.5 * 60, 30);
  instance.insert_stop("S8Airport", "Karlsplatz", 16.5 * 60, 30);
  instance.insert_stop("S8Airport", "Marienplatz", 18.25 * 60, 30);
  instance.insert_stop("S8Airport", "Isartor", 20 * 60, 30);
  instance.insert_stop("S8Airport", "Rosenheimer Platz", 22 * 60, 30);
  instance.add_empty_route("S8Airport");
  instance.push_back_edge_to_route("S8Airport", e1lr);
  instance.push_back_edge_to_route("S8Airport", e2lr);
  instance.push_back_edge_to_route("S8Airport", e3lr);
  instance.push_back_edge_to_route("S8Airport", e4lr);
  instance.push_back_edge_to_route("S8Airport", e5lr);
  instance.push_back_edge_to_route("S8Airport", e6lr);
  instance.push_back_edge_to_route("S8Airport", e7lr);
  instance.push_back_edge_to_route("S8Airport", e8lr);
  instance.push_back_edge_to_route("S8Airport", e9lr_a);
  instance.push_back_edge_to_route("S8Airport", e9lr_b);
  instance.push_back_edge_to_route("S8Airport", e10lr);
  instance.push_back_edge_to_route("S8Airport", e11lr);
  instance.push_back_edge_to_route("S8Airport", e12lr);
  instance.push_back_edge_to_route("S8Airport", e13lr);
  instance.push_back_edge_to_route("S8Airport", e14lr);
  instance.push_back_edge_to_route("S8Airport", e15lr);
  instance.push_back_edge_to_route("S8Airport", e16lr);
  instance.push_back_edge_to_route("S8Airport", e17lr);
  instance.push_back_edge_to_route("S8Airport", e18lr);
  instance.push_back_edge_to_route("S8Airport", e19lr);
  instance.push_back_edge_to_route("S8Airport", e20lr);
  instance.push_back_edge_to_route("S8Airport", e21lr);
  instance.push_back_edge_to_route("S8Airport", e22lr);
  instance.push_back_edge_to_route("S8Airport", e23lr);
  instance.push_back_edge_to_route("S8Airport", e24lr);
  instance.push_back_edge_to_route("S8Airport", e25lr);
  instance.push_back_edge_to_route("S8Airport", e26lr_5);

  // S1 Leuchtenbergring
  instance.add_train("S1Leuchtenbergring", 202, 140 / 3.6, 1, 0.9, 8.75 * 60, 0,
                     laim_entry, 25.5 * 60, 0, ost_4_exit);
  instance.insert_stop("S1Leuchtenbergring", "Hirschgarten", 10.25 * 60, 30);
  instance.insert_stop("S1Leuchtenbergring", "Donnersbergerbruecke", 12.25 * 60,
                       30);
  instance.insert_stop("S1Leuchtenbergring", "Hackerbruecke", 14 * 60, 30);
  instance.insert_stop("S1Leuchtenbergring", "Hbf", 16 * 60, 30);
  instance.insert_stop("S1Leuchtenbergring", "Karlsplatz", 18 * 60, 30);
  instance.insert_stop("S1Leuchtenbergring", "Marienplatz", 19.75 * 60, 30);
  instance.insert_stop("S1Leuchtenbergring", "Isartor", 21.5 * 60, 30);
  instance.insert_stop("S1Leuchtenbergring", "Rosenheimer Platz", 23.5 * 60,
                       30);
  instance.add_empty_route("S1Leuchtenbergring");
  instance.push_back_edge_to_route("S1Leuchtenbergring", e4lr_entry);
  instance.push_back_edge_to_route("S1Leuchtenbergring", e5lr);
  instance.push_back_edge_to_route("S1Leuchtenbergring", e6lr);
  instance.push_back_edge_to_route("S1Leuchtenbergring", e7lr);
  instance.push_back_edge_to_route("S1Leuchtenbergring", e8lr);
  instance.push_back_edge_to_route("S1Leuchtenbergring", e9lr_a);
  instance.push_back_edge_to_route("S1Leuchtenbergring", e9lr_b);
  instance.push_back_edge_to_route("S1Leuchtenbergring", e10lr);
  instance.push_back_edge_to_route("S1Leuchtenbergring", e11lr);
  instance.push_back_edge_to_route("S1Leuchtenbergring", e12lr);
  instance.push_back_edge_to_route("S1Leuchtenbergring", e13lr);
  instance.push_back_edge_to_route("S1Leuchtenbergring", e14lr);
  instance.push_back_edge_to_route("S1Leuchtenbergring", e15lr);
  instance.push_back_edge_to_route("S1Leuchtenbergring", e16lr);
  instance.push_back_edge_to_route("S1Leuchtenbergring", e17lr);
  instance.push_back_edge_to_route("S1Leuchtenbergring", e18lr);
  instance.push_back_edge_to_route("S1Leuchtenbergring", e19lr);
  instance.push_back_edge_to_route("S1Leuchtenbergring", e20lr);
  instance.push_back_edge_to_route("S1Leuchtenbergring", e21lr);
  instance.push_back_edge_to_route("S1Leuchtenbergring", e22lr);
  instance.push_back_edge_to_route("S1Leuchtenbergring", e23lr);
  instance.push_back_edge_to_route("S1Leuchtenbergring", e24lr);
  instance.push_back_edge_to_route("S1Leuchtenbergring", e25lr);
  instance.push_back_edge_to_route("S1Leuchtenbergring", e26lr_4);
  instance.push_back_edge_to_route("S1Leuchtenbergring", e27lr_4);

  // S4 Grafing
  instance.add_train("S4Grafing", 135, 140 / 3.6, 1, 0.9, 7.25 * 60, 0,
                     pasing_entry, 27 * 60, 0, ost_5_exit);
  instance.insert_stop("S4Grafing", "Laim", 9.75 * 60, 30);
  instance.insert_stop("S4Grafing", "Hirschgarten", 11.75 * 60, 30);
  instance.insert_stop("S4Grafing", "Donnersbergerbruecke", 13.75 * 60, 30);
  instance.insert_stop("S4Grafing", "Hackerbruecke", 15.5 * 60, 30);
  instance.insert_stop("S4Grafing", "Hbf", 17.5 * 60, 30);
  instance.insert_stop("S4Grafing", "Karlsplatz", 19.5 * 60, 30);
  instance.insert_stop("S4Grafing", "Marienplatz", 21.25 * 60, 30);
  instance.insert_stop("S4Grafing", "Isartor", 23 * 60, 30);
  instance.insert_stop("S4Grafing", "Rosenheimer Platz", 25 * 60, 30);
  instance.add_empty_route("S4Grafing");
  instance.push_back_edge_to_route("S4Grafing", e1lr);
  instance.push_back_edge_to_route("S4Grafing", e2lr);
  instance.push_back_edge_to_route("S4Grafing", e3lr);
  instance.push_back_edge_to_route("S4Grafing", e4lr);
  instance.push_back_edge_to_route("S4Grafing", e5lr);
  instance.push_back_edge_to_route("S4Grafing", e6lr);
  instance.push_back_edge_to_route("S4Grafing", e7lr);
  instance.push_back_edge_to_route("S4Grafing", e8lr);
  instance.push_back_edge_to_route("S4Grafing", e9lr_a);
  instance.push_back_edge_to_route("S4Grafing", e9lr_b);
  instance.push_back_edge_to_route("S4Grafing", e10lr);
  instance.push_back_edge_to_route("S4Grafing", e11lr);
  instance.push_back_edge_to_route("S4Grafing", e12lr);
  instance.push_back_edge_to_route("S4Grafing", e13lr);
  instance.push_back_edge_to_route("S4Grafing", e14lr);
  instance.push_back_edge_to_route("S4Grafing", e15lr);
  instance.push_back_edge_to_route("S4Grafing", e16lr);
  instance.push_back_edge_to_route("S4Grafing", e17lr);
  instance.push_back_edge_to_route("S4Grafing", e18lr);
  instance.push_back_edge_to_route("S4Grafing", e19lr);
  instance.push_back_edge_to_route("S4Grafing", e20lr);
  instance.push_back_edge_to_route("S4Grafing", e21lr);
  instance.push_back_edge_to_route("S4Grafing", e22lr);
  instance.push_back_edge_to_route("S4Grafing", e23lr);
  instance.push_back_edge_to_route("S4Grafing", e24lr);
  instance.push_back_edge_to_route("S4Grafing", e25lr);
  instance.push_back_edge_to_route("S4Grafing", e26lr_5);

  // S2 Ost
  instance.add_train("S2Ost", 135, 140 / 3.6, 1, 0.9, 11.75 * 60, 0, laim_entry,
                     28.5 * 60, 0, ost_5_exit);
  instance.insert_stop("S2Ost", "Hirschgarten", 13.25 * 60, 30);
  instance.insert_stop("S2Ost", "Donnersbergerbruecke", 15.25 * 60, 30);
  instance.insert_stop("S2Ost", "Hackerbruecke", 17 * 60, 30);
  instance.insert_stop("S2Ost", "Hbf", 19 * 60, 30);
  instance.insert_stop("S2Ost", "Karlsplatz", 21 * 60, 30);
  instance.insert_stop("S2Ost", "Marienplatz", 22.75 * 60, 30);
  instance.insert_stop("S2Ost", "Isartor", 24.5 * 60, 30);
  instance.insert_stop("S2Ost", "Rosenheimer Platz", 26.5 * 60, 30);
  instance.add_empty_route("S2Ost");
  instance.push_back_edge_to_route("S2Ost", e4lr_entry);
  instance.push_back_edge_to_route("S2Ost", e5lr);
  instance.push_back_edge_to_route("S2Ost", e6lr);
  instance.push_back_edge_to_route("S2Ost", e7lr);
  instance.push_back_edge_to_route("S2Ost", e8lr);
  instance.push_back_edge_to_route("S2Ost", e9lr_a);
  instance.push_back_edge_to_route("S2Ost", e9lr_b);
  instance.push_back_edge_to_route("S2Ost", e10lr);
  instance.push_back_edge_to_route("S2Ost", e11lr);
  instance.push_back_edge_to_route("S2Ost", e12lr);
  instance.push_back_edge_to_route("S2Ost", e13lr);
  instance.push_back_edge_to_route("S2Ost", e14lr);
  instance.push_back_edge_to_route("S2Ost", e15lr);
  instance.push_back_edge_to_route("S2Ost", e16lr);
  instance.push_back_edge_to_route("S2Ost", e17lr);
  instance.push_back_edge_to_route("S2Ost", e18lr);
  instance.push_back_edge_to_route("S2Ost", e19lr);
  instance.push_back_edge_to_route("S2Ost", e20lr);
  instance.push_back_edge_to_route("S2Ost", e21lr);
  instance.push_back_edge_to_route("S2Ost", e22lr);
  instance.push_back_edge_to_route("S2Ost", e23lr);
  instance.push_back_edge_to_route("S2Ost", e24lr);
  instance.push_back_edge_to_route("S2Ost", e25lr);
  instance.push_back_edge_to_route("S2Ost", e26lr_5);

  // NOLINTEND(bugprone-swapped-arguments)

  EXPECT_TRUE(instance.check_consistency(true));
  EXPECT_TRUE(
      instance.get_editable_network().is_consistent_for_transformation());

  auto pairs = instance.get_editable_network().all_edge_pairs_shortest_paths();

  auto pasing_ost4 = pairs.at(e1lr).at(e27lr_4) +
                     instance.get_editable_network().get_edge(e1lr).length;
  auto pasing_ost5 = pairs.at(e1lr).at(e26lr_5) +
                     instance.get_editable_network().get_edge(e1lr).length;
  auto laim_ost4 = pairs.at(e4lr_entry).at(e27lr_4) +
                   instance.get_editable_network().get_edge(e4lr_entry).length;
  auto laim_ost5 = pairs.at(e4lr_entry).at(e26lr_5) +
                   instance.get_editable_network().get_edge(e4lr_entry).length;
  auto donnersberger_ost4 =
      pairs.at(e9lr_entry).at(e27lr_4) +
      instance.get_editable_network().get_edge(e9lr_entry).length;
  auto donnersberger_ost5 =
      pairs.at(e9lr_entry).at(e26lr_5) +
      instance.get_editable_network().get_edge(e9lr_entry).length;
  auto ost1_pasing = pairs.at(e26rl_1).at(e1rl) +
                     instance.get_editable_network().get_edge(e26rl_1).length;
  auto ost2_pasing = pairs.at(e27rl_2).at(e1rl) +
                     instance.get_editable_network().get_edge(e27rl_2).length;
  auto ost3_pasing = pairs.at(e28rl_3).at(e1rl) +
                     instance.get_editable_network().get_edge(e28rl_3).length;
  auto ost1_laim   = pairs.at(e26rl_1).at(e2rl_exit) +
                     instance.get_editable_network().get_edge(e26rl_1).length;
  auto ost2_laim   = pairs.at(e27rl_2).at(e2rl_exit) +
                     instance.get_editable_network().get_edge(e27rl_2).length;
  auto ost3_laim   = pairs.at(e28rl_3).at(e2rl_exit) +
                     instance.get_editable_network().get_edge(e28rl_3).length;
  auto ost1_donnersberger =
      pairs.at(e26rl_1).at(e9rl_a) +
      instance.get_editable_network().get_edge(e26rl_1).length;
  auto ost2_donnersberger =
      pairs.at(e27rl_2).at(e9rl_a) +
      instance.get_editable_network().get_edge(e27rl_2).length;
  auto ost3_donnersberger =
      pairs.at(e28rl_3).at(e9rl_a) +
      instance.get_editable_network().get_edge(e28rl_3).length;

  constexpr int full_expected          = 11090;
  constexpr int laim_expected_lr       = 7788;
  constexpr int laim_expected_rl       = laim_expected_lr + 210 + 370 + 30;
  constexpr int donnersberger_expected = 5391;

  EXPECT_EQ(pasing_ost4, full_expected);
  EXPECT_EQ(pasing_ost5, full_expected);
  EXPECT_EQ(laim_ost4, laim_expected_lr);
  EXPECT_EQ(laim_ost5, laim_expected_lr);
  EXPECT_EQ(donnersberger_ost4, donnersberger_expected);
  EXPECT_EQ(donnersberger_ost5, donnersberger_expected);
  EXPECT_EQ(ost1_pasing, full_expected);
  EXPECT_EQ(ost2_pasing, full_expected);
  EXPECT_EQ(ost3_pasing, full_expected);
  EXPECT_EQ(ost1_laim, laim_expected_rl);
  EXPECT_EQ(ost2_laim, laim_expected_rl);
  EXPECT_EQ(ost3_laim, laim_expected_rl);
  EXPECT_EQ(ost1_donnersberger, donnersberger_expected);
  EXPECT_EQ(ost2_donnersberger, donnersberger_expected);
  EXPECT_EQ(ost3_donnersberger, donnersberger_expected);
}

TEST(ProblemInstanceWithSchedulesAndRoutes, Helper) {
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance;

  instance.get_editable_network().add_vertex("v0", cda_rail::VertexType::TTD);
  instance.get_editable_network().add_vertex("v1", cda_rail::VertexType::TTD);
  instance.get_editable_network().add_vertex("v2",
                                             cda_rail::VertexType::NoBorder);
  instance.get_editable_network().add_vertex("v3a", cda_rail::VertexType::TTD);
  instance.get_editable_network().add_vertex("v3b", cda_rail::VertexType::TTD);
  instance.get_editable_network().add_vertex("v4a", cda_rail::VertexType::TTD);
  instance.get_editable_network().add_vertex("v4b", cda_rail::VertexType::TTD);
  instance.get_editable_network().add_vertex("v5a", cda_rail::VertexType::TTD);
  instance.get_editable_network().add_vertex("v5b", cda_rail::VertexType::TTD);

  auto const e01 =
      instance.get_editable_network().add_edge({"v0"}, {"v1"}, 100, 100, true);
  auto const e12 =
      instance.get_editable_network().add_edge({"v1"}, {"v2"}, 50, 100, false);
  auto const e23a =
      instance.get_editable_network().add_edge({"v2"}, {"v3a"}, 50, 100, false);
  auto const e23b =
      instance.get_editable_network().add_edge({"v2"}, {"v3b"}, 50, 100, false);
  auto const e34a = instance.get_editable_network().add_edge({"v3a"}, {"v4a"},
                                                             500, 100, true);
  auto const e34b = instance.get_editable_network().add_edge({"v3b"}, {"v4b"},
                                                             500, 100, true);
  auto const e45a = instance.get_editable_network().add_edge({"v4a"}, {"v5a"},
                                                             500, 100, true);
  [[maybe_unused]] auto const e45b = instance.get_editable_network().add_edge(
      {"v4b"}, {"v5b"}, 500, 100, true);

  instance.get_editable_network().add_successor({{"v0"}, {"v1"}},
                                                {{"v1"}, {"v2"}});
  instance.get_editable_network().add_successor({{"v1"}, {"v2"}},
                                                {{"v2"}, {"v3a"}});
  instance.get_editable_network().add_successor({{"v1"}, {"v2"}},
                                                {{"v2"}, {"v3b"}});
  instance.get_editable_network().add_successor({{"v2"}, {"v3a"}},
                                                {{"v3a"}, {"v4a"}});
  instance.get_editable_network().add_successor({{"v2"}, {"v3b"}},
                                                {{"v3b"}, {"v4b"}});
  instance.get_editable_network().add_successor({{"v3a"}, {"v4a"}},
                                                {{"v4a"}, {"v5a"}});
  instance.get_editable_network().add_successor({{"v3b"}, {"v4b"}},
                                                {{"v4b"}, {"v5b"}});

  auto const tr1 = instance.add_train("tr1", 100, 100, 2, 2, 0, 10, {"v0"}, 200,
                                      10, {"v5a"});
  auto const tr2 = instance.add_train("tr2", 100, 100, 2, 2, 0, 10, {"v0"}, 200,
                                      10, {"v5b"});

  instance.add_empty_station("Station");
  instance.add_track_to_station("Station", {"v3a", "v4a"});
  instance.add_track_to_station("Station", {"v3b", "v4b"});
  instance.add_track_to_station("Station", {"v4a", "v5a"});
  instance.add_track_to_station("Station", {"v4b", "v5b"});

  EXPECT_FALSE(instance.has_route_for_every_train());

  EXPECT_ANY_THROW((void)instance.edges_used_by_train("tr1", true, true));
  auto const tr1_edges = instance.edges_used_by_train("tr1", true, false);
  EXPECT_EQ(tr1_edges.size(), instance.get_const_network().number_of_edges());
  for (size_t e_idx = 0; e_idx < instance.get_const_network().number_of_edges();
       ++e_idx) {
    EXPECT_TRUE(tr1_edges.contains(e_idx))
        << "Edge " << e_idx << " ("
        << instance.get_const_network().get_edge_name(e_idx)
        << ") not found in tr_edges";
  }

  instance.add_empty_route("tr1");
  instance.push_back_edge_to_route("tr1", {"v0", "v1"});
  instance.push_back_edge_to_route("tr1", {"v1", "v2"});
  instance.push_back_edge_to_route("tr1", {"v2", "v3a"});
  instance.push_back_edge_to_route("tr1", {"v3a", "v4a"});
  instance.push_back_edge_to_route("tr1", {"v4a", "v5a"});

  EXPECT_FALSE(instance.has_route_for_every_train());

  auto const tr1_edges_2 = instance.edges_used_by_train("tr1", true, true);
  cda_rail::index_set const expected_tr1_edges_2{e01, e12, e23a, e34a, e45a};
  EXPECT_EQ(tr1_edges_2, expected_tr1_edges_2);

  auto const tr1_edges_2b = instance.edges_used_by_train("tr1", false);
  EXPECT_EQ(tr1_edges_2b.size(),
            instance.get_const_network().number_of_edges());
  for (size_t e_idx = 0; e_idx < instance.get_const_network().number_of_edges();
       ++e_idx) {
    EXPECT_TRUE(tr1_edges_2b.contains(e_idx))
        << "Edge " << e_idx << " ("
        << instance.get_const_network().get_edge_name(e_idx)
        << ") not found in tr_edges";
  }

  auto const tr1_sections_2 = instance.sections_used_by_train(
      "tr1", {{e12, e23a, e23b}, {e34b}}, true, true);
  cda_rail::index_set const expected_tr1_sections_2{0};
  EXPECT_EQ(tr1_sections_2, expected_tr1_sections_2);

  EXPECT_ANY_THROW(
      (void)instance.trains_in_section({e12, e23a, e23b}, true, true));
  auto const tr_in_section_2 =
      instance.trains_in_section({e12, e23a, e23b}, true, false);
  cda_rail::index_set const expected_tr_in_section_2{tr1, tr2};

  instance.add_empty_route("tr2");
  instance.push_back_edge_to_route("tr2", {"v0", "v1"});
  instance.push_back_edge_to_route("tr2", {"v1", "v2"});
  instance.push_back_edge_to_route("tr2", {"v2", "v3b"});
  instance.push_back_edge_to_route("tr2", {"v3b", "v4b"});
  instance.push_back_edge_to_route("tr2", {"v4b", "v5b"});

  EXPECT_TRUE(instance.has_route_for_every_train());

  auto const tr_on_sec1 = instance.trains_in_section({e23a, e23b}, true, true);
  auto const tr_on_sec2 = instance.trains_in_section({e34b}, true, true);
  EXPECT_EQ(tr_on_sec1, cda_rail::index_set({tr1, tr2}));
  EXPECT_EQ(tr_on_sec2, cda_rail::index_set({tr2}));

  auto const tr_on_edges1 = instance.all_trains_on_edge(e12, true);
  auto const tr_on_edges2 = instance.all_trains_on_edge(e23a, true);
  auto const tr_on_edges3 = instance.all_trains_on_edge(e23b, true);
  EXPECT_EQ(tr_on_edges1, cda_rail::index_set({tr1, tr2}));
  EXPECT_EQ(tr_on_edges2, cda_rail::index_set({tr1}));
  EXPECT_EQ(tr_on_edges3, cda_rail::index_set({tr2}));
}
// NOLINTEND(clang-diagnostic-unused-result,google-readability-function-size,readability-function-size)
