#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "VSSModel.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Route.hpp"
#include "datastructure/Station.hpp"
#include "datastructure/Timetable.hpp"
#include "datastructure/Train.hpp"
#include "nlohmann/json_fwd.hpp"

#include "gtest/gtest.h"
#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

using json = nlohmann::json;
using std::size_t;

struct EdgeTarget {
  std::string source;
  std::string target;
  double      length;
  double      max_speed;
  bool        breakable;
  double      min_block_length;
  double      min_stop_block_length = 100;
};

// NOLINTBEGIN(clang-diagnostic-unused-result,clang-analyzer-deadcode.DeadStores,bugprone-unchecked-optional-access)

TEST(Functionality, NetworkTTDIntersection) {
  const std::vector<std::pair<size_t, size_t>> expected({{0, 1}, {2, 4}});
  const auto actual = cda_rail::Network::get_intersecting_ttd(
      {0, 1, 2, 3, 4}, {{1, 2, 5}, {6, 9, 10}, {11, 4, 10}});
  EXPECT_EQ(actual, expected);
}

TEST(Functionality, NetworkFunctions) {
  cda_rail::Network network;
  const auto v0 = network.add_vertex("v0", cda_rail::VertexType::NoBorder);
  const auto v1 = network.add_vertex("v1", cda_rail::VertexType::VSS);
  const auto v2 = network.add_vertex("v2", cda_rail::VertexType::TTD);

  const auto e0 = network.add_edge("v0", "v1", 1, 2, false, 0);
  const auto e1 = network.add_edge("v1", "v2", 3, 4, true, 1.5);
  const auto e2 = network.add_edge("v1", "v0", 1, 2, false, 0);
  const auto e3 = network.add_edge("v2", "v0", 10, 20, true, 2, 5);

  network.add_successor(network.get_edge_index("v0", "v1"),
                        network.get_edge_index("v1", "v2"));
  network.add_successor(network.get_edge_index("v2", "v0"),
                        network.get_edge_index("v0", "v1"));

  // Check edge name
  EXPECT_EQ(network.get_edge_name(e0), "v0-v1");
  EXPECT_EQ(network.get_edge_name(e1), "v1-v2");
  EXPECT_EQ(network.get_edge_name(e2), "v1-v0");
  EXPECT_EQ(network.get_edge_name(e3), "v2-v0");

  EXPECT_EQ(network.get_edge_name(v0, v2), "v0-v2");
  EXPECT_EQ(network.get_edge_name(v0, v2, false), "v0-v2");
  EXPECT_THROW(network.get_edge_name(v0, v2, true),
               cda_rail::exceptions::EdgeNotExistentException);
  EXPECT_THROW(network.get_edge_name(v0, v0 + v1 + v2),
               cda_rail::exceptions::VertexNotExistentException);
  EXPECT_THROW(network.get_edge_name(v0, v0 + v1 + v2, false),
               cda_rail::exceptions::VertexNotExistentException);
  EXPECT_THROW(network.get_edge_name(v0, v0 + v1 + v2, true),
               cda_rail::exceptions::VertexNotExistentException);
  EXPECT_EQ(network.get_edge_name(v0, v1), "v0-v1");
  EXPECT_EQ(network.get_edge_name(v0, v1, false), "v0-v1");
  EXPECT_EQ(network.get_edge_name(v0, v1, true), "v0-v1");

  EXPECT_EQ(network.get_edge_name("v0", "v2"), "v0-v2");
  EXPECT_EQ(network.get_edge_name("v0", "v2", false), "v0-v2");
  EXPECT_THROW(network.get_edge_name("v0", "v2", true),
               cda_rail::exceptions::EdgeNotExistentException);
  EXPECT_EQ(network.get_edge_name("v0", "temp"), "v0-temp");
  EXPECT_EQ(network.get_edge_name("v0", "temp", false), "v0-temp");
  EXPECT_THROW(network.get_edge_name("v0", "temp", true),
               cda_rail::exceptions::VertexNotExistentException);
  EXPECT_EQ(network.get_edge_name("temp", "v0"), "temp-v0");
  EXPECT_EQ(network.get_edge_name("temp", "v0", false), "temp-v0");
  EXPECT_THROW(network.get_edge_name("temp", "v0", true),
               cda_rail::exceptions::VertexNotExistentException);
  EXPECT_EQ(network.get_edge_name("v0", "v1"), "v0-v1");
  EXPECT_EQ(network.get_edge_name("v0", "v1", false), "v0-v1");
  EXPECT_EQ(network.get_edge_name("v0", "v1", true), "v0-v1");

  // Check vertices used by edges
  const auto vertices1 = network.vertices_used_by_edges({e0, e1, e2});
  // Expect all three vertices v0, v1, v2
  EXPECT_EQ(vertices1.size(), 3);
  EXPECT_TRUE(std::find(vertices1.begin(), vertices1.end(), v0) !=
              vertices1.end());
  EXPECT_TRUE(std::find(vertices1.begin(), vertices1.end(), v1) !=
              vertices1.end());
  EXPECT_TRUE(std::find(vertices1.begin(), vertices1.end(), v2) !=
              vertices1.end());

  const auto vertices2 = network.vertices_used_by_edges({e0, e2});
  // Expect only v0 and v1
  EXPECT_EQ(vertices2.size(), 2);
  EXPECT_TRUE(std::find(vertices2.begin(), vertices2.end(), v0) !=
              vertices2.end());
  EXPECT_TRUE(std::find(vertices2.begin(), vertices2.end(), v1) !=
              vertices2.end());

  // check vertex indices
  EXPECT_EQ(network.get_vertex_index("v0"), v0);
  EXPECT_EQ(network.get_vertex_index("v1"), v1);
  EXPECT_EQ(network.get_vertex_index("v2"), v2);

  // check edge indices
  EXPECT_EQ(network.get_edge_index("v0", "v1"), e0);
  EXPECT_EQ(network.get_edge_index("v1", "v2"), e1);
  EXPECT_EQ(network.get_edge_index("v1", "v0"), e2);
  EXPECT_EQ(network.get_edge_index("v2", "v0"), e3);

  // get Vertex tests
  EXPECT_EQ(network.get_vertex(0).name, "v0");
  EXPECT_EQ(network.get_vertex("v0").name, "v0");
  EXPECT_EQ(network.get_vertex_index("v0"), 0);

  // get Edge tests
  EXPECT_EQ(network.get_edge(0).source, 0);
  EXPECT_EQ(network.get_edge(0).target, 1);
  EXPECT_EQ(network.get_edge(0, 1).source, 0);
  EXPECT_EQ(network.get_edge(0, 1).target, 1);
  EXPECT_EQ(network.get_edge("v0", "v1").source, 0);
  EXPECT_EQ(network.get_edge("v0", "v1").target, 1);
  EXPECT_EQ(network.get_edge_index(0, 1), 0);
  EXPECT_EQ(network.get_edge_index("v0", "v1"), 0);

  // has vertex tests
  EXPECT_TRUE(network.has_vertex(0));
  EXPECT_FALSE(network.has_vertex(3));
  EXPECT_TRUE(network.has_vertex("v0"));
  EXPECT_FALSE(network.has_vertex("v3"));

  // has edge tests
  EXPECT_TRUE(network.has_edge(0));
  EXPECT_FALSE(network.has_edge(4));
  EXPECT_TRUE(network.has_edge(0, 1));
  EXPECT_FALSE(network.has_edge(0, 2));
  EXPECT_TRUE(network.has_edge("v0", "v1"));
  EXPECT_FALSE(network.has_edge("v0", "v2"));

  // Maximum number of VSS test
  EXPECT_EQ(network.max_vss_on_edge(e0), 0);
  EXPECT_EQ(network.max_vss_on_edge(e1), 2);
  EXPECT_EQ(network.max_vss_on_edge(e2), 0);
  EXPECT_EQ(network.max_vss_on_edge(e3), 5);

  // change vertex name tests
  network.change_vertex_name(0, "v0_tmp");
  EXPECT_EQ(network.get_vertex(0).name, "v0_tmp");
  EXPECT_EQ(network.get_vertex("v0_tmp").name, "v0_tmp");
  EXPECT_EQ(network.get_vertex_index("v0_tmp"), 0);
  EXPECT_FALSE(network.has_vertex("v0"));
  EXPECT_TRUE(network.has_vertex("v0_tmp"));
  network.change_vertex_name("v0_tmp", "v0");
  EXPECT_EQ(network.get_vertex(0).name, "v0");
  EXPECT_EQ(network.get_vertex("v0").name, "v0");
  EXPECT_EQ(network.get_vertex_index("v0"), 0);
  EXPECT_FALSE(network.has_vertex("v0_tmp"));
  EXPECT_TRUE(network.has_vertex("v0"));

  // change vertex headway tests
  EXPECT_EQ(network.get_vertex("v0").headway, 0);
  network.change_vertex_headway("v0", 10);
  EXPECT_EQ(network.get_vertex("v0").headway, 10);
  const auto v0_index = network.get_vertex_index("v0");
  network.change_vertex_headway(v0_index, 0);
  EXPECT_EQ(network.get_vertex("v0").headway, 0);

  // change edge properties tests
  network.change_edge_length(0, 2);
  EXPECT_EQ(network.get_edge(0).length, 2);
  network.change_edge_max_speed(0, 3);
  EXPECT_EQ(network.get_edge(0).max_speed, 3);
  network.change_edge_min_block_length(0, 4);
  EXPECT_EQ(network.get_edge(0).min_block_length, 4);
  network.change_edge_length(0, 1, 5);
  EXPECT_EQ(network.get_edge(0).length, 5);
  network.change_edge_max_speed(0, 1, 6);
  EXPECT_EQ(network.get_edge(0).max_speed, 6);
  network.change_edge_min_block_length(0, 1, 7);
  EXPECT_EQ(network.get_edge(0).min_block_length, 7);
  network.change_edge_length("v0", "v1", 8);
  EXPECT_EQ(network.get_edge(0).length, 8);
  network.change_edge_max_speed("v0", "v1", 9);
  EXPECT_EQ(network.get_edge(0).max_speed, 9);
  network.change_edge_min_block_length("v0", "v1", 10);
  EXPECT_EQ(network.get_edge(0).min_block_length, 10);
  network.set_edge_breakable(1);
  EXPECT_TRUE(network.get_edge(1).breakable);
  network.set_edge_unbreakable(1, 2);
  EXPECT_FALSE(network.get_edge(1).breakable);
  network.set_edge_breakable("v1", "v2");
  EXPECT_TRUE(network.get_edge(1).breakable);

  EXPECT_EQ(network.get_edge(e0).min_stop_block_length, 100);
  EXPECT_EQ(network.get_edge(e1).min_stop_block_length, 100);
  EXPECT_EQ(network.get_edge(e2).min_stop_block_length, 100);
  EXPECT_EQ(network.get_edge(e3).min_stop_block_length, 5);

  network.change_edge_min_stop_block_length(e0, 2);
  EXPECT_EQ(network.get_edge(e0).min_stop_block_length, 2);

  // out and in edges tests
  const std::vector<size_t> expected_out{1, 2};
  const std::vector<size_t> expected_in{0};
  const std::vector<size_t> expected_neighbors{0, 2};
  auto                      out_edges_1 = network.out_edges(1);
  std::sort(out_edges_1.begin(), out_edges_1.end());
  EXPECT_EQ(out_edges_1, expected_out);
  auto out_edges_v1 = network.out_edges("v1");
  std::sort(out_edges_v1.begin(), out_edges_v1.end());
  EXPECT_EQ(out_edges_v1, expected_out);
  auto in_edges_1 = network.in_edges(1);
  std::sort(in_edges_1.begin(), in_edges_1.end());
  EXPECT_EQ(in_edges_1, expected_in);
  auto in_edges_v1 = network.in_edges("v1");
  std::sort(in_edges_v1.begin(), in_edges_v1.end());
  EXPECT_EQ(in_edges_v1, expected_in);
  auto neighbors_1 = network.neighbors(1);
  std::sort(neighbors_1.begin(), neighbors_1.end());
  EXPECT_EQ(neighbors_1, expected_neighbors);
  auto neighbors_v1 = network.neighbors("v1");
  std::sort(neighbors_v1.begin(), neighbors_v1.end());
  EXPECT_EQ(neighbors_v1, expected_neighbors);

  // successor tests
  const std::vector<size_t> expected_successors{1};
  EXPECT_EQ(network.get_successors(0), expected_successors);
  EXPECT_EQ(network.get_successors(0, 1), expected_successors);
  EXPECT_EQ(network.get_successors("v0", "v1"), expected_successors);

  // Vertex and edge numbers
  EXPECT_EQ(network.number_of_vertices(), 3);
  EXPECT_EQ(network.number_of_edges(), 4);

  // Valid successor
  EXPECT_TRUE(network.is_valid_successor(0, 1));
  EXPECT_FALSE(network.is_valid_successor(0, 2));
}

TEST(Functionality, NetworkPredecessor) {
  // Create network

  cda_rail::Network network;

  const auto v_0 = network.add_vertex("v0", cda_rail::VertexType::NoBorder);
  const auto v_1 = network.add_vertex("v1", cda_rail::VertexType::NoBorder);
  const auto v_2 = network.add_vertex("v2", cda_rail::VertexType::NoBorder);
  const auto v_3 = network.add_vertex("v3", cda_rail::VertexType::NoBorder);
  const auto v_4 = network.add_vertex("v4", cda_rail::VertexType::NoBorder);
  const auto v_5 = network.add_vertex("v5", cda_rail::VertexType::NoBorder);

  const auto e_0_1 = network.add_edge(v_0, v_1, 100, 10);
  const auto e_1_2 = network.add_edge(v_1, v_2, 50, 10);
  const auto e_2_3 = network.add_edge(v_2, v_3, 50, 10);
  const auto e_3_2 = network.add_edge(v_3, v_2, 50, 10);
  const auto e_3_4 = network.add_edge(v_3, v_4, 100, 10);
  const auto e_4_3 = network.add_edge(v_4, v_3, 100, 10);
  const auto e_1_3 = network.add_edge(v_1, v_3, 100, 10);
  const auto e_3_0 = network.add_edge(v_3, v_0, 400, 10);
  const auto e_4_5 = network.add_edge(v_4, v_5, 10, 10);

  network.add_successor(e_0_1, e_1_2);
  network.add_successor(e_0_1, e_1_3);
  network.add_successor(e_1_2, e_2_3);
  network.add_successor(e_2_3, e_3_4);
  network.add_successor(e_1_3, e_3_4);
  network.add_successor(e_4_3, e_3_2);
  network.add_successor(e_1_3, e_3_0);
  network.add_successor(e_2_3, e_3_0);
  network.add_successor(e_3_0, e_0_1);
  network.add_successor(e_3_4, e_4_5);

  // Predecessors of e_0_1 are e_3_0
  const auto predecessors_0_1 = network.get_predecessors(e_0_1);
  EXPECT_EQ(predecessors_0_1.size(), 1);
  EXPECT_TRUE(std::find(predecessors_0_1.begin(), predecessors_0_1.end(),
                        e_3_0) != predecessors_0_1.end())
      << "e_3_0 is not a predecessor of e_0_1";

  // Predecessors of e_1_2 is e_0_1
  const auto predecessors_1_2 = network.get_predecessors(e_1_2);
  EXPECT_EQ(predecessors_1_2.size(), 1);
  EXPECT_TRUE(std::find(predecessors_1_2.begin(), predecessors_1_2.end(),
                        e_0_1) != predecessors_1_2.end())
      << "e_0_1 is not a predecessor of e_1_2";

  // Predecessors of e_2_3 is e_1_2
  const auto predecessors_2_3 = network.get_predecessors(e_2_3);
  EXPECT_EQ(predecessors_2_3.size(), 1);
  EXPECT_TRUE(std::find(predecessors_2_3.begin(), predecessors_2_3.end(),
                        e_1_2) != predecessors_2_3.end())
      << "e_1_2 is not a predecessor of e_2_3";

  // Predecessors of e_3_2 is e_4_3
  const auto predecessors_3_2 = network.get_predecessors(e_3_2);
  EXPECT_EQ(predecessors_3_2.size(), 1);
  EXPECT_TRUE(std::find(predecessors_3_2.begin(), predecessors_3_2.end(),
                        e_4_3) != predecessors_3_2.end())
      << "e_4_3 is not a predecessor of e_3_2";

  // Predecessors of e_3_4 are e_2_3 and e_1_3
  const auto predecessors_3_4 = network.get_predecessors(e_3_4);
  EXPECT_EQ(predecessors_3_4.size(), 2);
  EXPECT_TRUE(std::find(predecessors_3_4.begin(), predecessors_3_4.end(),
                        e_2_3) != predecessors_3_4.end())
      << "e_2_3 is not a predecessor of e_3_4";
  EXPECT_TRUE(std::find(predecessors_3_4.begin(), predecessors_3_4.end(),
                        e_1_3) != predecessors_3_4.end())
      << "e_1_3 is not a predecessor of e_3_4";

  // Predecessors of e_4_3 are None
  const auto predecessors_4_3 = network.get_predecessors(e_4_3);
  EXPECT_EQ(predecessors_4_3.size(), 0);

  // Predecessors of e_1_3 is e_0_1
  const auto predecessors_1_3 = network.get_predecessors(e_1_3);
  EXPECT_EQ(predecessors_1_3.size(), 1);
  EXPECT_TRUE(std::find(predecessors_1_3.begin(), predecessors_1_3.end(),
                        e_0_1) != predecessors_1_3.end())
      << "e_0_1 is not a predecessor of e_1_3";

  // Predecessors of e_3_0 are e_1_3 and e_2_3
  const auto predecessors_3_0 = network.get_predecessors(e_3_0);
  EXPECT_EQ(predecessors_3_0.size(), 2);
  EXPECT_TRUE(std::find(predecessors_3_0.begin(), predecessors_3_0.end(),
                        e_1_3) != predecessors_3_0.end())
      << "e_1_3 is not a predecessor of e_3_0";
  EXPECT_TRUE(std::find(predecessors_3_0.begin(), predecessors_3_0.end(),
                        e_2_3) != predecessors_3_0.end())
      << "e_2_3 is not a predecessor of e_3_0";

  // Predecessors of e_4_5 is e_3_4
  const auto predecessors_4_5 = network.get_predecessors(e_4_5);
  EXPECT_EQ(predecessors_4_5.size(), 1);
  EXPECT_TRUE(std::find(predecessors_4_5.begin(), predecessors_4_5.end(),
                        e_3_4) != predecessors_4_5.end())
      << "e_3_4 is not a predecessor of e_4_5";
}

TEST(Functionality, NetworkForwardPathsFromVertex) {
  // Create network

  cda_rail::Network network;

  const auto v_0 = network.add_vertex("v0", cda_rail::VertexType::NoBorder);
  const auto v_1 = network.add_vertex("v1", cda_rail::VertexType::NoBorder);
  const auto v_2 = network.add_vertex("v2", cda_rail::VertexType::NoBorder);
  const auto v_3 = network.add_vertex("v3", cda_rail::VertexType::NoBorder);
  const auto v_4 = network.add_vertex("v4", cda_rail::VertexType::NoBorder);
  const auto v_5 = network.add_vertex("v5", cda_rail::VertexType::NoBorder);

  const auto e_0_1 = network.add_edge(v_0, v_1, 100, 10);
  const auto e_1_2 = network.add_edge(v_1, v_2, 50, 10);
  const auto e_2_3 = network.add_edge(v_2, v_3, 50, 10);
  const auto e_3_2 = network.add_edge(v_3, v_2, 50, 10);
  const auto e_3_4 = network.add_edge(v_3, v_4, 100, 10);
  const auto e_4_3 = network.add_edge(v_4, v_3, 100, 10);
  const auto e_1_3 = network.add_edge(v_1, v_3, 100, 10);
  const auto e_3_0 = network.add_edge(v_3, v_0, 400, 10);
  const auto e_4_5 = network.add_edge(v_4, v_5, 10, 10);

  network.add_successor(e_0_1, e_1_2);
  network.add_successor(e_0_1, e_1_3);
  network.add_successor(e_1_2, e_2_3);
  network.add_successor(e_2_3, e_3_4);
  network.add_successor(e_1_3, e_3_4);
  network.add_successor(e_4_3, e_3_2);
  network.add_successor(e_1_3, e_3_0);
  network.add_successor(e_2_3, e_3_0);
  network.add_successor(e_3_0, e_0_1);
  network.add_successor(e_3_4, e_4_5);

  // Forward paths from v_0 with length 50 is e_0_1
  const auto forward_paths_0 =
      network.all_paths_of_length_starting_in_vertex(v_0, 50);
  EXPECT_EQ(forward_paths_0.size(), 1);
  EXPECT_TRUE(std::find(forward_paths_0.begin(), forward_paths_0.end(),
                        std::vector<size_t>{e_0_1}) != forward_paths_0.end())
      << "e_0_1 is not in the forward paths from v_0 with length 50";

  // Forward paths from v_0 with length 100 is e_0_1
  const auto forward_paths_1 =
      network.all_paths_of_length_starting_in_vertex(v_0, 100);
  EXPECT_EQ(forward_paths_1.size(), 1);
  EXPECT_TRUE(std::find(forward_paths_1.begin(), forward_paths_1.end(),
                        std::vector<size_t>{e_0_1}) != forward_paths_1.end())
      << "e_0_1 is not in the forward paths from v_0 with length 100";

  // Forward paths from v_0 with length 150 are (e_0_1,e_1_2) and (e_0_1, e_1_3)
  const auto forward_paths_2 =
      network.all_paths_of_length_starting_in_vertex(v_0, 150);
  EXPECT_EQ(forward_paths_2.size(), 2);
  EXPECT_TRUE(std::find(forward_paths_2.begin(), forward_paths_2.end(),
                        std::vector<size_t>{e_0_1, e_1_2}) !=
              forward_paths_2.end())
      << "(e_0_1, e_1_2) is not in the forward paths from v_0 with length 150";
  EXPECT_TRUE(std::find(forward_paths_2.begin(), forward_paths_2.end(),
                        std::vector<size_t>{e_0_1, e_1_3}) !=
              forward_paths_2.end())
      << "(e_0_1, e_1_3) is not in the forward paths from v_0 with length 150";

  // Forward paths from v_0 with length 180 are (e_0_1,e_1_2,e_2_3) and (e_0_1,
  // e_1_3)
  const auto forward_paths_3 =
      network.all_paths_of_length_starting_in_vertex(v_0, 180);
  EXPECT_EQ(forward_paths_3.size(), 2);
  EXPECT_TRUE(std::find(forward_paths_3.begin(), forward_paths_3.end(),
                        std::vector<size_t>{e_0_1, e_1_2, e_2_3}) !=
              forward_paths_3.end())
      << "(e_0_1, e_1_2, e_2_3) is not in the forward paths from v_0 with "
         "length 180";
  EXPECT_TRUE(std::find(forward_paths_3.begin(), forward_paths_3.end(),
                        std::vector<size_t>{e_0_1, e_1_3}) !=
              forward_paths_3.end())
      << "(e_0_1, e_1_3) is not in the forward paths from v_0 with length 180";

  // Forward paths from v_0 with length 300 are (e_0_1,e_1_2,e_2_3,e_3_4),
  // (e_0_1, e_1_2, e_2_3, e_3_0), (e_0_1, e_1_3, e_3_4), and (e_0_1, e_1_3,
  // e_3_0)
  const auto forward_paths_4 =
      network.all_paths_of_length_starting_in_vertex(v_0, 300);
  EXPECT_EQ(forward_paths_4.size(), 4);
  EXPECT_TRUE(std::find(forward_paths_4.begin(), forward_paths_4.end(),
                        std::vector<size_t>{e_0_1, e_1_2, e_2_3, e_3_4}) !=
              forward_paths_4.end())
      << "(e_0_1, e_1_2, e_2_3, e_3_4) is not in the forward paths from v_0 "
         "with length 300";
  EXPECT_TRUE(std::find(forward_paths_4.begin(), forward_paths_4.end(),
                        std::vector<size_t>{e_0_1, e_1_2, e_2_3, e_3_0}) !=
              forward_paths_4.end())
      << "(e_0_1, e_1_2, e_2_3, e_3_0) is not in the forward paths from v_0 "
         "with length 300";
  EXPECT_TRUE(std::find(forward_paths_4.begin(), forward_paths_4.end(),
                        std::vector<size_t>{e_0_1, e_1_3, e_3_4}) !=
              forward_paths_4.end())
      << "(e_0_1, e_1_3, e_3_4) is not in the forward paths from v_0 with "
         "length 300";
  EXPECT_TRUE(std::find(forward_paths_4.begin(), forward_paths_4.end(),
                        std::vector<size_t>{e_0_1, e_1_3, e_3_0}) !=
              forward_paths_4.end())
      << "(e_0_1, e_1_3, e_3_0) is not in the forward paths from v_0 with "
         "length 300";

  // Forward paths from v_0 with length 400 are (e_0_1, e_1_2, e_2_3, e_3_0) and
  // (e_0_1, e_1_3, e_3_0)
  const auto forward_paths_5 =
      network.all_paths_of_length_starting_in_vertex(v_0, 400);
  EXPECT_EQ(forward_paths_5.size(), 2);
  EXPECT_TRUE(std::find(forward_paths_5.begin(), forward_paths_5.end(),
                        std::vector<size_t>{e_0_1, e_1_2, e_2_3, e_3_0}) !=
              forward_paths_5.end())
      << "(e_0_1, e_1_2, e_2_3, e_3_0) is not in the forward paths from v_0 "
         "with length 400";
  EXPECT_TRUE(std::find(forward_paths_5.begin(), forward_paths_5.end(),
                        std::vector<size_t>{e_0_1, e_1_3, e_3_0}) !=
              forward_paths_5.end())
      << "(e_0_1, e_1_3, e_3_0) is not in the forward paths from v_0 with "
         "length 400";

  // Forward paths from v_0 with length 600 are (e_0_1, e_1_2, e_2_3, e_3_0) and
  // (e_0_1, e_1_3, e_3_0)
  const auto forward_paths_6 =
      network.all_paths_of_length_starting_in_vertex(v_0, 600);
  EXPECT_EQ(forward_paths_6.size(), 2);
  EXPECT_TRUE(std::find(forward_paths_6.begin(), forward_paths_6.end(),
                        std::vector<size_t>{e_0_1, e_1_2, e_2_3, e_3_0}) !=
              forward_paths_6.end())
      << "(e_0_1, e_1_2, e_2_3, e_3_0) is not in the forward paths from v_0 "
         "with length 600";
  EXPECT_TRUE(std::find(forward_paths_6.begin(), forward_paths_6.end(),
                        std::vector<size_t>{e_0_1, e_1_3, e_3_0}) !=
              forward_paths_6.end())
      << "(e_0_1, e_1_3, e_3_0) is not in the forward paths from v_0 with "
         "length 600";

  // Forward paths from v_0 with length 601 are None, due to cycle
  const auto forward_paths_7 =
      network.all_paths_of_length_starting_in_vertex(v_0, 601);
  EXPECT_EQ(forward_paths_7.size(), 0);

  // Forward paths from v_1 with length 25 are e_1_2 and e_1_3
  const auto forward_paths_8 =
      network.all_paths_of_length_starting_in_vertex(v_1, 25);
  EXPECT_EQ(forward_paths_8.size(), 2);
  EXPECT_TRUE(std::find(forward_paths_8.begin(), forward_paths_8.end(),
                        std::vector<size_t>{e_1_2}) != forward_paths_8.end())
      << "e_1_2 is not in the forward paths from v_1 with length 25";
  EXPECT_TRUE(std::find(forward_paths_8.begin(), forward_paths_8.end(),
                        std::vector<size_t>{e_1_3}) != forward_paths_8.end())
      << "e_1_3 is not in the forward paths from v_1 with length 25";

  // Forward paths from v_1 with length 250 are (e_1_2, e_2_3, e_3_4), (e_1_2,
  // e_2_3, e_3_0), (e_1_3, e_3_4), and (e_1_3, e_3_0)
  const auto forward_paths_9 =
      network.all_paths_of_length_starting_in_vertex(v_1, 150);
  EXPECT_EQ(forward_paths_9.size(), 4);
  EXPECT_TRUE(std::find(forward_paths_9.begin(), forward_paths_9.end(),
                        std::vector<size_t>{e_1_2, e_2_3, e_3_4}) !=
              forward_paths_9.end())
      << "(e_1_2, e_2_3, e_3_4) is not in the forward paths from v_1 with "
         "length 150";
  EXPECT_TRUE(std::find(forward_paths_9.begin(), forward_paths_9.end(),
                        std::vector<size_t>{e_1_2, e_2_3, e_3_0}) !=
              forward_paths_9.end())
      << "(e_1_2, e_2_3, e_3_0) is not in the forward paths from v_1 with "
         "length 150";
  EXPECT_TRUE(std::find(forward_paths_9.begin(), forward_paths_9.end(),
                        std::vector<size_t>{e_1_3, e_3_4}) !=
              forward_paths_9.end())
      << "(e_1_3, e_3_4) is not in the forward paths from v_1 with length 150";
  EXPECT_TRUE(std::find(forward_paths_9.begin(), forward_paths_9.end(),
                        std::vector<size_t>{e_1_3, e_3_0}) !=
              forward_paths_9.end())
      << "(e_1_3, e_3_0) is not in the forward paths from v_1 with length 150";

  // Forward paths from v_3 with length 25 are e_3_2, e_3_0 and e_3_4
  const auto forward_paths_10 =
      network.all_paths_of_length_starting_in_vertex(v_3, 25);
  EXPECT_EQ(forward_paths_10.size(), 3);
  EXPECT_TRUE(std::find(forward_paths_10.begin(), forward_paths_10.end(),
                        std::vector<size_t>{e_3_2}) != forward_paths_10.end())
      << "e_3_2 is not in the forward paths from v_3 with length 25";
  EXPECT_TRUE(std::find(forward_paths_10.begin(), forward_paths_10.end(),
                        std::vector<size_t>{e_3_0}) != forward_paths_10.end())
      << "e_3_0 is not in the forward paths from v_3 with length 25";
  EXPECT_TRUE(std::find(forward_paths_10.begin(), forward_paths_10.end(),
                        std::vector<size_t>{e_3_4}) != forward_paths_10.end())
      << "e_3_4 is not in the forward paths from v_3 with length 25";

  // Forward paths from v_3 with length 105 are (e_3_4,e_4_5) and e_3_0
  const auto forward_paths_11 =
      network.all_paths_of_length_starting_in_vertex(v_3, 105);
  EXPECT_EQ(forward_paths_11.size(), 2);
  EXPECT_TRUE(std::find(forward_paths_11.begin(), forward_paths_11.end(),
                        std::vector<size_t>{e_3_4, e_4_5}) !=
              forward_paths_11.end())
      << "(e_3_4, e_4_5) is not in the forward paths from v_3 with length 105";
  EXPECT_TRUE(std::find(forward_paths_11.begin(), forward_paths_11.end(),
                        std::vector<size_t>{e_3_0}) != forward_paths_11.end())
      << "e_3_0 is not in the forward paths from v_3 with length 105";

  // Forward paths from v_5 with length 1 are None
  const auto forward_paths_12 =
      network.all_paths_of_length_starting_in_vertex(v_5, 1);
  EXPECT_EQ(forward_paths_12.size(), 0);

  // Forward paths from v_1 with length 601 is None
  const auto forward_paths_13 =
      network.all_paths_of_length_starting_in_vertex(v_1, 601);
  EXPECT_EQ(forward_paths_13.size(), 0);

  // Forward paths from v_4 with length 50 and exit node v5, where only v4_v5
  // and v1_v2 are considered is v4_v5
  const auto forward_paths_14 = network.all_paths_of_length_starting_in_vertex(
      v_4, 50, v_5, {e_1_2, e_4_5});
  EXPECT_EQ(forward_paths_14.size(), 1);
  EXPECT_TRUE(std::find(forward_paths_14.begin(), forward_paths_14.end(),
                        std::vector<size_t>{e_4_5}) != forward_paths_14.end())
      << "e_4_5 is not in the forward paths from v_4 with length 50 and exit "
         "node v5";

  EXPECT_THROW(
      network.all_paths_of_length_starting_in_vertex(v_3, 0, v_5, {}, false),
      cda_rail::exceptions::InvalidInputException);

  const auto forward_paths_15 =
      network.all_paths_of_length_starting_in_vertex(v_3, 0, v_5, {}, true);
  EXPECT_EQ(forward_paths_15.size(), 3);
  EXPECT_TRUE(std::find(forward_paths_15.begin(), forward_paths_15.end(),
                        std::vector<size_t>{e_3_2}) != forward_paths_15.end())
      << "e_3_2 is not in the forward paths from v_3 with length 0";
  EXPECT_TRUE(std::find(forward_paths_15.begin(), forward_paths_15.end(),
                        std::vector<size_t>{e_3_0}) != forward_paths_15.end())
      << "e_3_0 is not in the forward paths from v_3 with length 0";
  EXPECT_TRUE(std::find(forward_paths_15.begin(), forward_paths_15.end(),
                        std::vector<size_t>{e_3_4}) != forward_paths_15.end())
      << "e_3_4 is not in the forward paths from v_3 with length 0";
}

TEST(Functionality, NetworkForwardPathsFromEdge) {
  // Create network

  cda_rail::Network network;

  const auto v_0 = network.add_vertex("v0", cda_rail::VertexType::NoBorder);
  const auto v_1 = network.add_vertex("v1", cda_rail::VertexType::NoBorder);
  const auto v_2 = network.add_vertex("v2", cda_rail::VertexType::NoBorder);
  const auto v_3 = network.add_vertex("v3", cda_rail::VertexType::NoBorder);
  const auto v_4 = network.add_vertex("v4", cda_rail::VertexType::NoBorder);
  const auto v_5 = network.add_vertex("v5", cda_rail::VertexType::NoBorder);

  const auto e_0_1 = network.add_edge(v_0, v_1, 100, 10);
  const auto e_1_2 = network.add_edge(v_1, v_2, 50, 10);
  const auto e_2_3 = network.add_edge(v_2, v_3, 50, 10);
  const auto e_3_2 = network.add_edge(v_3, v_2, 50, 10);
  const auto e_3_4 = network.add_edge(v_3, v_4, 100, 10);
  const auto e_4_3 = network.add_edge(v_4, v_3, 100, 10);
  const auto e_1_3 = network.add_edge(v_1, v_3, 100, 10);
  const auto e_3_0 = network.add_edge(v_3, v_0, 400, 10);
  const auto e_4_5 = network.add_edge(v_4, v_5, 10, 10);

  network.add_successor(e_0_1, e_1_2);
  network.add_successor(e_0_1, e_1_3);
  network.add_successor(e_1_2, e_2_3);
  network.add_successor(e_2_3, e_3_4);
  network.add_successor(e_1_3, e_3_4);
  network.add_successor(e_4_3, e_3_2);
  network.add_successor(e_1_3, e_3_0);
  network.add_successor(e_2_3, e_3_0);
  network.add_successor(e_3_0, e_0_1);
  network.add_successor(e_3_4, e_4_5);

  // Forward paths from e_0_1 with length 50 is e_0_1
  const auto forward_paths_0 =
      network.all_paths_of_length_starting_in_edge(e_0_1, 50);
  EXPECT_EQ(forward_paths_0.size(), 1);
  EXPECT_TRUE(std::find(forward_paths_0.begin(), forward_paths_0.end(),
                        std::vector<size_t>{e_0_1}) != forward_paths_0.end())
      << "e_0_1 is not in the forward paths from e_0_1 with length 50";

  // Forward paths from e_0_1 with length 125 are (e_0_1, e_1_2) and (e_0_1,
  // e_1_3)
  const auto forward_paths_1 =
      network.all_paths_of_length_starting_in_edge(e_0_1, 125);
  EXPECT_EQ(forward_paths_1.size(), 2);
  EXPECT_TRUE(std::find(forward_paths_1.begin(), forward_paths_1.end(),
                        std::vector<size_t>{e_0_1, e_1_2}) !=
              forward_paths_1.end())
      << "(e_0_1, e_1_2) is not in the forward paths from e_0_1 with length "
         "125";
  EXPECT_TRUE(std::find(forward_paths_1.begin(), forward_paths_1.end(),
                        std::vector<size_t>{e_0_1, e_1_3}) !=
              forward_paths_1.end())
      << "(e_0_1, e_1_3) is not in the forward paths from e_0_1 with length "
         "125";

  // Forward paths from e_1_2 with length 110 are (e_1_2, e_2_3, e_3_4) and
  // (e_1_2, e_2_3, e_3_0)
  const auto forward_paths_2 =
      network.all_paths_of_length_starting_in_edge(e_1_2, 110);
  EXPECT_EQ(forward_paths_2.size(), 2);
  EXPECT_TRUE(std::find(forward_paths_2.begin(), forward_paths_2.end(),
                        std::vector<size_t>{e_1_2, e_2_3, e_3_4}) !=
              forward_paths_2.end())
      << "(e_1_2, e_2_3, e_3_4) is not in the forward paths from e_1_2 with "
         "length 110";
  EXPECT_TRUE(std::find(forward_paths_2.begin(), forward_paths_2.end(),
                        std::vector<size_t>{e_1_2, e_2_3, e_3_0}) !=
              forward_paths_2.end())
      << "(e_1_2, e_2_3, e_3_0) is not in the forward paths from e_1_2 with "
         "length 110";

  // Forward paths from e_1_3 with length 110 are (e_1_3, e_3_4) and (e_1_3,
  // e_3_0)
  const auto forward_paths_3 =
      network.all_paths_of_length_starting_in_edge(e_1_3, 110);
  EXPECT_EQ(forward_paths_3.size(), 2);
  EXPECT_TRUE(std::find(forward_paths_3.begin(), forward_paths_3.end(),
                        std::vector<size_t>{e_1_3, e_3_4}) !=
              forward_paths_3.end())
      << "(e_1_3, e_3_4) is not in the forward paths from e_1_3 with length "
         "110";
  EXPECT_TRUE(std::find(forward_paths_3.begin(), forward_paths_3.end(),
                        std::vector<size_t>{e_1_3, e_3_0}) !=
              forward_paths_3.end())
      << "(e_1_3, e_3_0) is not in the forward paths from e_1_3 with length "
         "110";

  // Forward path from e_1_2 with length 601 is None
  const auto forward_paths_4 =
      network.all_paths_of_length_starting_in_edge(e_1_2, 601);
  EXPECT_EQ(forward_paths_4.size(), 0);

  // Test exit node, expect 3-4-5
  const auto forward_paths_5 =
      network.all_paths_of_length_starting_in_edge(e_3_4, 200, v_5);
  EXPECT_EQ(forward_paths_5.size(), 1);
  EXPECT_TRUE(std::find(forward_paths_5.begin(), forward_paths_5.end(),
                        std::vector<size_t>{e_3_4, e_4_5}) !=
              forward_paths_5.end())
      << "(e_3_4, e_4_5) is not in the forward paths from e_3_4 with length "
         "200";
}

TEST(Functionality, NetworkBackwardPathsFromVertex) {
  // Create network

  cda_rail::Network network;

  const auto v_0 = network.add_vertex("v0", cda_rail::VertexType::NoBorder);
  const auto v_1 = network.add_vertex("v1", cda_rail::VertexType::NoBorder);
  const auto v_2 = network.add_vertex("v2", cda_rail::VertexType::NoBorder);
  const auto v_3 = network.add_vertex("v3", cda_rail::VertexType::NoBorder);
  const auto v_4 = network.add_vertex("v4", cda_rail::VertexType::NoBorder);
  const auto v_5 = network.add_vertex("v5", cda_rail::VertexType::NoBorder);

  const auto e_0_1 = network.add_edge(v_0, v_1, 100, 10);
  const auto e_1_2 = network.add_edge(v_1, v_2, 50, 10);
  const auto e_2_3 = network.add_edge(v_2, v_3, 50, 10);
  const auto e_3_2 = network.add_edge(v_3, v_2, 50, 10);
  const auto e_3_4 = network.add_edge(v_3, v_4, 100, 10);
  const auto e_4_3 = network.add_edge(v_4, v_3, 100, 10);
  const auto e_1_3 = network.add_edge(v_1, v_3, 100, 10);
  const auto e_3_0 = network.add_edge(v_3, v_0, 400, 10);
  const auto e_4_5 = network.add_edge(v_4, v_5, 10, 10);

  network.add_successor(e_0_1, e_1_2);
  network.add_successor(e_0_1, e_1_3);
  network.add_successor(e_1_2, e_2_3);
  network.add_successor(e_2_3, e_3_4);
  network.add_successor(e_1_3, e_3_4);
  network.add_successor(e_4_3, e_3_2);
  network.add_successor(e_1_3, e_3_0);
  network.add_successor(e_2_3, e_3_0);
  network.add_successor(e_3_0, e_0_1);
  network.add_successor(e_3_4, e_4_5);

  // Backward paths from v_0 with length 50 is e_3_0
  const auto backward_paths_0 =
      network.all_paths_of_length_ending_in_vertex(v_0, 50);
  EXPECT_EQ(backward_paths_0.size(), 1);
  EXPECT_TRUE(std::find(backward_paths_0.begin(), backward_paths_0.end(),
                        std::vector<size_t>{e_3_0}) != backward_paths_0.end())
      << "e_3_0 is not in the backward paths from v_0 with length 50";

  // Backward paths from v_0 with length 475 are (e_3_0, e_1_3) and (e_3_0,
  // e_2_3, e_1_2)
  const auto backward_paths_1 =
      network.all_paths_of_length_ending_in_vertex(v_0, 475);
  EXPECT_EQ(backward_paths_1.size(), 2);
  EXPECT_TRUE(std::find(backward_paths_1.begin(), backward_paths_1.end(),
                        std::vector<size_t>{e_3_0, e_1_3}) !=
              backward_paths_1.end())
      << "(e_3_0, e_1_3) is not in the backward paths from v_0 with length 475";
  EXPECT_TRUE(std::find(backward_paths_1.begin(), backward_paths_1.end(),
                        std::vector<size_t>{e_3_0, e_2_3, e_1_2}) !=
              backward_paths_1.end())
      << "(e_3_0, e_2_3, e_1_2) is not in the backward paths from v_0 with "
         "length 475";

  // Backward paths from v_2 with length 100 is (e_3_2, e_4_3) and (e_1_2,
  // e_0_1)
  const auto backward_paths_2 =
      network.all_paths_of_length_ending_in_vertex(v_2, 100);
  EXPECT_EQ(backward_paths_2.size(), 2);
  EXPECT_TRUE(std::find(backward_paths_2.begin(), backward_paths_2.end(),
                        std::vector<size_t>{e_3_2, e_4_3}) !=
              backward_paths_2.end())
      << "(e_3_2, e_4_3) is not in the backward paths from v_2 with length 100";
  EXPECT_TRUE(std::find(backward_paths_2.begin(), backward_paths_2.end(),
                        std::vector<size_t>{e_1_2, e_0_1}) !=
              backward_paths_2.end())
      << "(e_1_2, e_0_1) is not in the backward paths from v_2 with length 100";

  // Backward paths from v_0 with length 601 is None
  const auto backward_paths_3 =
      network.all_paths_of_length_ending_in_vertex(v_0, 601);
  EXPECT_EQ(backward_paths_3.size(), 0);

  // Backward paths from v_4 with length 701 is None
  const auto backward_paths_4 =
      network.all_paths_of_length_ending_in_vertex(v_4, 701);
  EXPECT_EQ(backward_paths_4.size(), 0);
}

TEST(Functionality, NetworkBackwardPathsFromEdge) {
  // Create network

  cda_rail::Network network;

  const auto v_0 = network.add_vertex("v0", cda_rail::VertexType::NoBorder);
  const auto v_1 = network.add_vertex("v1", cda_rail::VertexType::NoBorder);
  const auto v_2 = network.add_vertex("v2", cda_rail::VertexType::NoBorder);
  const auto v_3 = network.add_vertex("v3", cda_rail::VertexType::NoBorder);
  const auto v_4 = network.add_vertex("v4", cda_rail::VertexType::NoBorder);
  const auto v_5 = network.add_vertex("v5", cda_rail::VertexType::NoBorder);

  const auto e_0_1 = network.add_edge(v_0, v_1, 100, 10);
  const auto e_1_2 = network.add_edge(v_1, v_2, 50, 10);
  const auto e_2_3 = network.add_edge(v_2, v_3, 50, 10);
  const auto e_3_2 = network.add_edge(v_3, v_2, 50, 10);
  const auto e_3_4 = network.add_edge(v_3, v_4, 100, 10);
  const auto e_4_3 = network.add_edge(v_4, v_3, 100, 10);
  const auto e_1_3 = network.add_edge(v_1, v_3, 100, 10);
  const auto e_3_0 = network.add_edge(v_3, v_0, 400, 10);
  const auto e_4_5 = network.add_edge(v_4, v_5, 10, 10);

  network.add_successor(e_0_1, e_1_2);
  network.add_successor(e_0_1, e_1_3);
  network.add_successor(e_1_2, e_2_3);
  network.add_successor(e_2_3, e_3_4);
  network.add_successor(e_1_3, e_3_4);
  network.add_successor(e_4_3, e_3_2);
  network.add_successor(e_1_3, e_3_0);
  network.add_successor(e_2_3, e_3_0);
  network.add_successor(e_3_0, e_0_1);
  network.add_successor(e_3_4, e_4_5);

  // Backward paths from e_3_0 with length 50 is e_3_0
  const auto backward_paths_0 =
      network.all_paths_of_length_ending_in_edge(e_3_0, 50);
  EXPECT_EQ(backward_paths_0.size(), 1);
  EXPECT_TRUE(std::find(backward_paths_0.begin(), backward_paths_0.end(),
                        std::vector<size_t>{e_3_0}) != backward_paths_0.end())
      << "e_3_0 is not in the backward paths from e_3_0 with length 50";

  // Backward paths from e_3_0 with length 475 are (e_3_0, e_1_3) and (e_3_0,
  // e_2_3, e_1_2)
  const auto backward_paths_1 =
      network.all_paths_of_length_ending_in_edge(e_3_0, 475);
  EXPECT_EQ(backward_paths_1.size(), 2);
  EXPECT_TRUE(std::find(backward_paths_1.begin(), backward_paths_1.end(),
                        std::vector<size_t>{e_3_0, e_1_3}) !=
              backward_paths_1.end())
      << "(e_3_0, e_1_3) is not in the backward paths from e_3_0 with length "
         "475";
  EXPECT_TRUE(std::find(backward_paths_1.begin(), backward_paths_1.end(),
                        std::vector<size_t>{e_3_0, e_2_3, e_1_2}) !=
              backward_paths_1.end())
      << "(e_3_0, e_2_3, e_1_2) is not in the backward paths from e_3_0 with "
         "length 475";

  // Backward paths from e_3_4 with length 700 are (e_3_4, e_1_3, e_0_1, e_3_0)
  // and (e_3_4, e_2_3, e_1_2, e_0_1, e_3_0)
  const auto backward_paths_2 =
      network.all_paths_of_length_ending_in_edge(e_3_4, 700);
  EXPECT_EQ(backward_paths_2.size(), 2);
  EXPECT_TRUE(std::find(backward_paths_2.begin(), backward_paths_2.end(),
                        std::vector<size_t>{e_3_4, e_1_3, e_0_1, e_3_0}) !=
              backward_paths_2.end())
      << "(e_3_4, e_1_3, e_0_1, e_3_0) is not in the backward paths from e_3_4 "
         "with length 700";
  EXPECT_TRUE(std::find(backward_paths_2.begin(), backward_paths_2.end(),
                        std::vector<size_t>{e_3_4, e_2_3, e_1_2, e_0_1,
                                            e_3_0}) != backward_paths_2.end())
      << "(e_3_4, e_2_3, e_1_2, e_0_1, e_3_0) is not in the backward paths "
         "from e_3_4 with length 700";

  // Backward paths from e_3_4 with length 701 is None
  const auto backward_paths_3 =
      network.all_paths_of_length_ending_in_edge(e_3_4, 701);
  EXPECT_EQ(backward_paths_3.size(), 0);

  // Backward paths from e_1_2 with length 151 is (e_1_2, e_0_1, e_3_0)
  const auto backward_paths_4 =
      network.all_paths_of_length_ending_in_edge(e_1_2, 151);
  EXPECT_EQ(backward_paths_4.size(), 1);
  EXPECT_TRUE(std::find(backward_paths_4.begin(), backward_paths_4.end(),
                        std::vector<size_t>{e_1_2, e_0_1, e_3_0}) !=
              backward_paths_4.end())
      << "(e_1_2, e_0_1, e_3_0) is not in the backward paths from e_1_2 with "
         "length 151";

  // Backward paths from e_3_2 with length 151 is None
  const auto backward_paths_5 =
      network.all_paths_of_length_ending_in_edge(e_3_2, 151);
  EXPECT_EQ(backward_paths_5.size(), 0);
}

TEST(Functionality, NetworkSections) {
  cda_rail::Network network;

  // Add vertices
  network.add_vertex("v0", cda_rail::VertexType::TTD);
  network.add_vertex("v1", cda_rail::VertexType::NoBorder);
  network.add_vertex("v20", cda_rail::VertexType::TTD);
  network.add_vertex("v21", cda_rail::VertexType::NoBorder);
  network.add_vertex("v30", cda_rail::VertexType::NoBorder);
  network.add_vertex("v31", cda_rail::VertexType::VSS);
  network.add_vertex("v4", cda_rail::VertexType::TTD);
  network.add_vertex("v5", cda_rail::VertexType::VSS);
  network.add_vertex("v6", cda_rail::VertexType::NoBorderVSS);
  network.add_vertex("v7", cda_rail::VertexType::TTD);

  // Add edges v0 -> v1 -> v20 -> v30 -> v4 -> v5 -> v6 -> v7
  // All unbreakable other properties not relevant
  const auto v0_v1   = network.add_edge("v0", "v1", 1, 1, false);
  const auto v1_v20  = network.add_edge("v1", "v20", 1, 1, false);
  const auto v20_v30 = network.add_edge("v20", "v30", 1, 1, false);
  const auto v30_v4  = network.add_edge("v30", "v4", 1, 1, false);
  const auto v4_v5   = network.add_edge("v4", "v5", 1, 1, false);
  const auto v5_v6   = network.add_edge("v5", "v6", 1, 1, false);
  const auto v6_v7   = network.add_edge("v6", "v7", 1, 1, false);

  // Add edges v7 -> v6 -> v5 -> v4 -> v31 -> v21 -> v1 -> v0
  // v4 -> v31 breakable, all other unbreakable
  const auto v7_v6 = network.add_edge("v7", "v6", 1, 1, false);
  const auto v6_v5 = network.add_edge("v6", "v5", 1, 1, false);
  const auto v5_v4 = network.add_edge("v5", "v4", 1, 1, false);
  network.add_edge("v4", "v31", 1, 1, true);
  const auto v31_v21 = network.add_edge("v31", "v21", 1, 1, false);
  const auto v21_v1  = network.add_edge("v21", "v1", 1, 1, false);
  const auto v1_v0   = network.add_edge("v1", "v0", 1, 1, false);

  auto no_border_vss_sections = network.no_border_vss_sections();

  // Check non_border_vss_sections
  // There should be 1 section
  EXPECT_EQ(no_border_vss_sections.size(), 1);
  // The section should contain 4 edges, namely v5 -> v6 -> v7 and the reverse
  EXPECT_EQ(no_border_vss_sections[0].size(), 4);
  EXPECT_TRUE(std::find(no_border_vss_sections[0].begin(),
                        no_border_vss_sections[0].end(),
                        v5_v6) != no_border_vss_sections[0].end());
  EXPECT_TRUE(std::find(no_border_vss_sections[0].begin(),
                        no_border_vss_sections[0].end(),
                        v6_v7) != no_border_vss_sections[0].end());
  EXPECT_TRUE(std::find(no_border_vss_sections[0].begin(),
                        no_border_vss_sections[0].end(),
                        v7_v6) != no_border_vss_sections[0].end());
  EXPECT_TRUE(std::find(no_border_vss_sections[0].begin(),
                        no_border_vss_sections[0].end(),
                        v6_v5) != no_border_vss_sections[0].end());

  const std::pair<size_t, size_t> pair1 = std::make_pair(v5_v6, v6_v5);
  const std::pair<size_t, size_t> pair2 = std::make_pair(v6_v7, v7_v6);
  EXPECT_TRUE(network.common_vertex(pair1, pair2) ==
              network.get_vertex_index("v6"));

  auto unbreakable_sections = network.unbreakable_sections();

  // Check unbreakable_sections
  // There should be 3 sections
  EXPECT_EQ(unbreakable_sections.size(), 3);
  // One section should contain v0_v1, one should contain v20_v30, and one
  // should contain v4_v5

  std::optional<size_t> s0;
  std::optional<size_t> s1;
  std::optional<size_t> s2;
  for (size_t i = 0; i < unbreakable_sections.size(); i++) {
    if (std::find(unbreakable_sections[i].begin(),
                  unbreakable_sections[i].end(),
                  v0_v1) != unbreakable_sections[i].end()) {
      s0 = i;
    }
    if (std::find(unbreakable_sections[i].begin(),
                  unbreakable_sections[i].end(),
                  v20_v30) != unbreakable_sections[i].end()) {
      s1 = i;
    }
    if (std::find(unbreakable_sections[i].begin(),
                  unbreakable_sections[i].end(),
                  v4_v5) != unbreakable_sections[i].end()) {
      s2 = i;
    }
  }
  // s0, s1 and s2 should be all different and within [0, 2]
  EXPECT_TRUE(s0.has_value());
  EXPECT_TRUE(s1.has_value());
  EXPECT_TRUE(s2.has_value());
  EXPECT_NE(s0, s1);
  EXPECT_NE(s0, s2);
  EXPECT_NE(s1, s2);
  EXPECT_GE(s0, 0);
  EXPECT_GE(s1, 0);
  EXPECT_GE(s2, 0);
  EXPECT_LE(s0, 2);
  EXPECT_LE(s1, 2);
  EXPECT_LE(s2, 2);

  const auto s0_val = s0.value();
  const auto s1_val = s1.value();
  const auto s2_val = s2.value();

  // Test breakable special case
  network.set_edge_breakable(v0_v1);
  const auto& unbreakable_v0_v1_var =
      network.get_unbreakable_section_containing_edge(v0_v1);
  EXPECT_EQ(unbreakable_v0_v1_var.size(), 0);
  network.set_edge_unbreakable(v0_v1);

  // Section s0 should contain 5 edges, namely v0 -> v1, v1 -> v20, v31 -> v21,
  // v21 -> v1, v1 -> v0
  const auto& unbreakable_v0_v1 =
      network.get_unbreakable_section_containing_edge(v0_v1);
  EXPECT_EQ(unbreakable_v0_v1.size(), 5);
  EXPECT_TRUE(std::find(unbreakable_v0_v1.begin(), unbreakable_v0_v1.end(),
                        v0_v1) != unbreakable_v0_v1.end());
  EXPECT_TRUE(std::find(unbreakable_v0_v1.begin(), unbreakable_v0_v1.end(),
                        v1_v20) != unbreakable_v0_v1.end());
  EXPECT_TRUE(std::find(unbreakable_v0_v1.begin(), unbreakable_v0_v1.end(),
                        v31_v21) != unbreakable_v0_v1.end());
  EXPECT_TRUE(std::find(unbreakable_v0_v1.begin(), unbreakable_v0_v1.end(),
                        v21_v1) != unbreakable_v0_v1.end());
  EXPECT_TRUE(std::find(unbreakable_v0_v1.begin(), unbreakable_v0_v1.end(),
                        v1_v0) != unbreakable_v0_v1.end());
  EXPECT_TRUE(network.is_on_same_unbreakable_section(v0_v1, v1_v20));
  EXPECT_TRUE(network.is_on_same_unbreakable_section(v1_v20, v0_v1));
  EXPECT_TRUE(network.is_on_same_unbreakable_section(v0_v1, v31_v21));
  EXPECT_TRUE(network.is_on_same_unbreakable_section(v0_v1, v21_v1));
  EXPECT_TRUE(network.is_on_same_unbreakable_section(v0_v1, v1_v0));
  EXPECT_FALSE(network.is_on_same_unbreakable_section(v0_v1, v30_v4));

  EXPECT_EQ(unbreakable_sections[s0_val].size(), 5);
  EXPECT_TRUE(std::find(unbreakable_sections[s0_val].begin(),
                        unbreakable_sections[s0_val].end(),
                        v0_v1) != unbreakable_sections[s0_val].end());
  EXPECT_TRUE(std::find(unbreakable_sections[s0_val].begin(),
                        unbreakable_sections[s0_val].end(),
                        v1_v20) != unbreakable_sections[s0_val].end());
  EXPECT_TRUE(std::find(unbreakable_sections[s0_val].begin(),
                        unbreakable_sections[s0_val].end(),
                        v31_v21) != unbreakable_sections[s0_val].end());
  EXPECT_TRUE(std::find(unbreakable_sections[s0_val].begin(),
                        unbreakable_sections[s0_val].end(),
                        v21_v1) != unbreakable_sections[s0_val].end());
  EXPECT_TRUE(std::find(unbreakable_sections[s0_val].begin(),
                        unbreakable_sections[s0_val].end(),
                        v1_v0) != unbreakable_sections[s0_val].end());

  // Section s1 should contain 2 edges, namely v20 -> v30 -> v4
  EXPECT_EQ(unbreakable_sections[s1_val].size(), 2);
  EXPECT_TRUE(std::find(unbreakable_sections[s1_val].begin(),
                        unbreakable_sections[s1_val].end(),
                        v20_v30) != unbreakable_sections[s1_val].end());
  EXPECT_TRUE(std::find(unbreakable_sections[s1_val].begin(),
                        unbreakable_sections[s1_val].end(),
                        v30_v4) != unbreakable_sections[s1_val].end());

  // Section s2 should contain 2 edges, namely v4 -> v5 and the reverse
  const auto& unbreakable_v4_v5 =
      network.get_unbreakable_section_containing_edge(v4_v5);
  EXPECT_EQ(unbreakable_v4_v5.size(), 2);
  EXPECT_TRUE(std::find(unbreakable_v4_v5.begin(), unbreakable_v4_v5.end(),
                        v4_v5) != unbreakable_v4_v5.end());
  EXPECT_TRUE(std::find(unbreakable_v4_v5.begin(), unbreakable_v4_v5.end(),
                        v5_v4) != unbreakable_v4_v5.end());
  EXPECT_TRUE(network.is_on_same_unbreakable_section(v4_v5, v5_v4));
  EXPECT_TRUE(network.is_on_same_unbreakable_section(v5_v4, v4_v5));
  EXPECT_FALSE(network.is_on_same_unbreakable_section(v4_v5, v0_v1));

  EXPECT_EQ(unbreakable_sections[s0_val].size(), 5);
  EXPECT_TRUE(std::find(unbreakable_sections[s0_val].begin(),
                        unbreakable_sections[s0_val].end(),
                        v0_v1) != unbreakable_sections[s0_val].end());
  EXPECT_TRUE(std::find(unbreakable_sections[s0_val].begin(),
                        unbreakable_sections[s0_val].end(),
                        v1_v20) != unbreakable_sections[s0_val].end());
  EXPECT_TRUE(std::find(unbreakable_sections[s0_val].begin(),
                        unbreakable_sections[s0_val].end(),
                        v31_v21) != unbreakable_sections[s0_val].end());
  EXPECT_TRUE(std::find(unbreakable_sections[s0_val].begin(),
                        unbreakable_sections[s0_val].end(),
                        v21_v1) != unbreakable_sections[s0_val].end());
  EXPECT_TRUE(std::find(unbreakable_sections[s0_val].begin(),
                        unbreakable_sections[s0_val].end(),
                        v1_v0) != unbreakable_sections[s0_val].end());

  EXPECT_EQ(unbreakable_sections[s2_val].size(), 2);
  EXPECT_TRUE(std::find(unbreakable_sections[s2_val].begin(),
                        unbreakable_sections[s2_val].end(),
                        v4_v5) != unbreakable_sections[s2_val].end());
  EXPECT_TRUE(std::find(unbreakable_sections[s2_val].begin(),
                        unbreakable_sections[s2_val].end(),
                        v5_v4) != unbreakable_sections[s2_val].end());
}

TEST(Functionality, NetworkConsistency) {
  cda_rail::Network network;

  // Add vertices
  network.add_vertex("v0", cda_rail::VertexType::TTD);
  network.add_vertex("v1", cda_rail::VertexType::NoBorderVSS);
  network.add_vertex("v2", cda_rail::VertexType::TTD);
  network.add_vertex("v3", cda_rail::VertexType::VSS);

  network.add_edge("v0", "v1", 100, 100, false);
  network.add_edge("v1", "v2", 100, 100, false);
  network.add_edge("v1", "v3", 100, 100, false);

  EXPECT_FALSE(network.is_consistent_for_transformation());

  network.change_vertex_type("v1", cda_rail::VertexType::NoBorder);

  EXPECT_TRUE(network.is_consistent_for_transformation());

  network.add_vertex("v4", cda_rail::VertexType::NoBorder);
  network.add_vertex("v5", cda_rail::VertexType::NoBorderVSS);
  network.add_vertex("v6", cda_rail::VertexType::VSS);

  network.add_edge("v2", "v4", 100, 100, false);
  network.add_edge("v4", "v5", 100, 100, false);
  network.add_edge("v5", "v6", 100, 100, false);

  EXPECT_FALSE(network.is_consistent_for_transformation());

  network.change_vertex_type("v5", cda_rail::VertexType::NoBorder);

  EXPECT_TRUE(network.is_consistent_for_transformation());

  network.add_vertex("v7", cda_rail::VertexType::TTD);

  network.add_edge("v6", "v7", 100, 100, true, 0);

  EXPECT_FALSE(network.is_consistent_for_transformation());

  network.change_edge_min_block_length("v6", "v7", 1);

  EXPECT_TRUE(network.is_consistent_for_transformation());

  network.change_vertex_type("v7", cda_rail::VertexType::NoBorder);

  EXPECT_FALSE(network.is_consistent_for_transformation());

  network.change_vertex_type("v7", cda_rail::VertexType::VSS);

  EXPECT_TRUE(network.is_consistent_for_transformation());

  network.add_vertex("v8", cda_rail::VertexType::TTD);

  network.add_edge("v7", "v8", 100, 100, false);
  network.add_edge("v8", "v7", 50, 50, false);

  EXPECT_FALSE(network.is_consistent_for_transformation());

  network.change_edge_length("v8", "v7", 100);

  EXPECT_TRUE(network.is_consistent_for_transformation());

  network.set_edge_breakable("v8", "v7");

  EXPECT_FALSE(network.is_consistent_for_transformation());
}

TEST(Functionality, ReadNetwork) {
  const cda_rail::Network network = cda_rail::Network::import_network(
      "./example-networks/SimpleStation/network/");

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
    const std::string       v_name = vertex_names[i];
    const cda_rail::Vertex& v      = network.get_vertex(v_name);
    EXPECT_EQ(v.name, v_name);
    EXPECT_EQ(v.type, type[i]);
    EXPECT_DOUBLE_EQ(v.headway, 0.0);
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
    const cda_rail::Edge e = network.get_edge(edge.source, edge.target);
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
  std::sort(successors_target.begin(), successors_target.end());
  auto successors_l2_l3 = network.get_successors("l2", "l3");
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
  std::sort(successors_target.begin(), successors_target.end());
  auto successors_r1_r2 = network.get_successors("r1", "r2");
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
}

TEST(Functionality, WriteNetwork) {
  cda_rail::Network network;
  network.add_vertex("v0", cda_rail::VertexType::NoBorder);
  network.add_vertex("v1", cda_rail::VertexType::VSS, 5);
  network.add_vertex("v2", cda_rail::VertexType::TTD);

  network.add_edge("v0", "v1", 1, 2, true, 0);
  network.add_edge("v1", "v2", 3, 4, false, 1.5);
  network.add_edge("v1", "v0", 1, 2, true, 0);
  network.add_edge("v2", "v0", 10, 20, false, 2, 5);

  network.add_successor(network.get_edge_index("v0", "v1"),
                        network.get_edge_index("v1", "v2"));
  network.add_successor(network.get_edge_index("v0", "v1"),
                        network.get_edge_index("v1", "v0"));
  network.add_successor(network.get_edge_index("v2", "v0"),
                        network.get_edge_index("v0", "v1"));

  network.export_network("./tmp/write_network_test");

  auto network_read =
      cda_rail::Network::import_network("./tmp/write_network_test");

  // Delete created directory and everything in it
  std::filesystem::remove_all("./tmp");

  // check if both networks are equivalent

  // check vertices
  EXPECT_EQ(network.number_of_vertices(), network_read.number_of_vertices());
  for (size_t i = 0; i < network.number_of_vertices(); ++i) {
    EXPECT_TRUE(network_read.has_vertex(network.get_vertex(i).name));
    EXPECT_EQ(network_read.get_vertex(network.get_vertex(i).name).type,
              network.get_vertex(i).type);
    EXPECT_DOUBLE_EQ(
        network_read.get_vertex(network.get_vertex(i).name).headway,
        network.get_vertex(i).headway);
  }

  // check edges
  EXPECT_EQ(network.number_of_edges(), network_read.number_of_edges());
  for (size_t i = 0; i < network.number_of_edges(); ++i) {
    const auto& source_vertex = network.get_vertex(network.get_edge(i).source);
    const auto& target_vertex = network.get_vertex(network.get_edge(i).target);
    EXPECT_TRUE(network_read.has_edge(source_vertex.name, target_vertex.name));
    const auto& edge_read =
        network_read.get_edge(source_vertex.name, target_vertex.name);
    EXPECT_EQ(edge_read.breakable, network.get_edge(i).breakable);
    EXPECT_EQ(edge_read.length, network.get_edge(i).length);
    EXPECT_EQ(edge_read.max_speed, network.get_edge(i).max_speed);
    EXPECT_EQ(edge_read.min_block_length, network.get_edge(i).min_block_length);
    EXPECT_EQ(edge_read.min_stop_block_length,
              network.get_edge(i).min_stop_block_length);
  }

  // check successors
  for (size_t i = 0; i < network.number_of_edges(); ++i) {
    const auto&         successors_target = network.get_successors(i);
    std::vector<size_t> successors_target_transformed;
    for (auto successor : successors_target) {
      const auto&       e      = network.get_edge(successor);
      const std::string source = network.get_vertex(e.source).name;
      const std::string target = network.get_vertex(e.target).name;
      successors_target_transformed.emplace_back(
          network_read.get_edge_index(source, target));
    }
    const auto&       e      = network.get_edge(i);
    const std::string source = network.get_vertex(e.source).name;
    const std::string target = network.get_vertex(e.target).name;
    auto              successors_target_transformed_read =
        network_read.get_successors(source, target);
    std::sort(successors_target_transformed.begin(),
              successors_target_transformed.end());
    std::sort(successors_target_transformed_read.begin(),
              successors_target_transformed_read.end());
    EXPECT_EQ(successors_target_transformed,
              successors_target_transformed_read);
  }
}

TEST(Functionality, NetworkEdgeSeparation) {
  cda_rail::Network network;
  // Add vertices
  const auto v00 = network.add_vertex("v00", cda_rail::VertexType::TTD);
  network.add_vertex("v01", cda_rail::VertexType::TTD);
  const auto v1 = network.add_vertex("v1", cda_rail::VertexType::TTD);
  network.add_vertex("v2", cda_rail::VertexType::TTD);
  network.add_vertex("v30", cda_rail::VertexType::TTD);
  network.add_vertex("v31", cda_rail::VertexType::TTD);

  // Add edges
  const auto v00_v1 = network.add_edge("v00", "v1", 100, 100, false);
  const auto v01_v1 = network.add_edge("v01", "v1", 100, 100, false);
  const auto v1_v2  = network.add_edge("v1", "v2", 44, 100, true, 10);
  const auto v2_v30 = network.add_edge("v2", "v30", 100, 100, false);
  const auto v2_v31 = network.add_edge("v2", "v31", 100, 100, false);

  // Add successors
  network.add_successor(v00_v1, v1_v2);
  network.add_successor(v01_v1, v1_v2);
  network.add_successor(v1_v2, v2_v30);
  network.add_successor(v1_v2, v2_v31);

  // Separate edge v1_v2 uniformly
  auto new_edges =
      network.separate_edge("v1", "v2", &cda_rail::vss::functions::uniform);

  // There are 4 new forward edges and no new reverse edges
  EXPECT_EQ(new_edges.first.size(), 4);
  EXPECT_EQ(new_edges.second.size(), 0);

  // Check if the vertices are correct
  // All old vertices still exist. Additionally, vertices v1_v2_0 to v1_v2_2
  // have been added with type NoBorder Hence, in total there are 9 vertices.
  EXPECT_EQ(network.number_of_vertices(), 9);
  EXPECT_TRUE(network.has_vertex("v1_v2_0"));
  EXPECT_TRUE(network.has_vertex("v1_v2_1"));
  EXPECT_TRUE(network.has_vertex("v1_v2_2"));
  EXPECT_EQ(network.get_vertex("v1_v2_0").type,
            cda_rail::VertexType::NoBorderVSS);
  EXPECT_EQ(network.get_vertex("v1_v2_1").type,
            cda_rail::VertexType::NoBorderVSS);
  EXPECT_EQ(network.get_vertex("v1_v2_2").type,
            cda_rail::VertexType::NoBorderVSS);
  EXPECT_TRUE(network.has_vertex("v00"));
  EXPECT_TRUE(network.has_vertex("v01"));
  EXPECT_TRUE(network.has_vertex("v1"));
  EXPECT_TRUE(network.has_vertex("v2"));
  EXPECT_TRUE(network.has_vertex("v30"));
  EXPECT_TRUE(network.has_vertex("v31"));
  EXPECT_EQ(network.get_vertex("v00").type, cda_rail::VertexType::TTD);
  EXPECT_EQ(network.get_vertex("v01").type, cda_rail::VertexType::TTD);
  EXPECT_EQ(network.get_vertex("v1").type, cda_rail::VertexType::TTD);
  EXPECT_EQ(network.get_vertex("v2").type, cda_rail::VertexType::TTD);
  EXPECT_EQ(network.get_vertex("v30").type, cda_rail::VertexType::TTD);
  EXPECT_EQ(network.get_vertex("v31").type, cda_rail::VertexType::TTD);

  // Check if the edges are correct
  // v00/v01 ->(len: 100) v1 ->(len: 11) v1_v2_0 -> (len: 11) v1_v2_1 -> (len:
  // 11) v1_v2_2 -> (len:11) v2 -> (len: 100) v30/v31
  EXPECT_EQ(network.number_of_edges(), 8);
  EXPECT_TRUE(network.has_edge("v00", "v1"));
  EXPECT_TRUE(network.has_edge("v01", "v1"));
  EXPECT_TRUE(network.has_edge("v1", "v1_v2_0"));
  EXPECT_TRUE(network.has_edge("v1_v2_0", "v1_v2_1"));
  EXPECT_TRUE(network.has_edge("v1_v2_1", "v1_v2_2"));
  EXPECT_TRUE(network.has_edge("v1_v2_2", "v2"));
  EXPECT_TRUE(network.has_edge("v2", "v30"));
  EXPECT_TRUE(network.has_edge("v2", "v31"));
  EXPECT_EQ(network.get_edge("v00", "v1").length, 100);
  EXPECT_EQ(network.get_edge("v01", "v1").length, 100);
  EXPECT_EQ(network.get_edge("v1", "v1_v2_0").length, 11);
  EXPECT_EQ(network.get_edge("v1_v2_0", "v1_v2_1").length, 11);
  EXPECT_EQ(network.get_edge("v1_v2_1", "v1_v2_2").length, 11);
  EXPECT_EQ(network.get_edge("v1_v2_2", "v2").length, 11);
  EXPECT_EQ(network.get_edge("v2", "v30").length, 100);
  EXPECT_EQ(network.get_edge("v2", "v31").length, 100);
  // The other properties are unchanged, except that all edges are unbreakable,
  // i.e., the breakable property is false
  EXPECT_FALSE(network.get_edge("v00", "v1").breakable);
  EXPECT_FALSE(network.get_edge("v01", "v1").breakable);
  EXPECT_FALSE(network.get_edge("v1", "v1_v2_0").breakable);
  EXPECT_FALSE(network.get_edge("v1_v2_0", "v1_v2_1").breakable);
  EXPECT_FALSE(network.get_edge("v1_v2_1", "v1_v2_2").breakable);
  EXPECT_FALSE(network.get_edge("v1_v2_2", "v2").breakable);
  EXPECT_FALSE(network.get_edge("v2", "v30").breakable);
  EXPECT_FALSE(network.get_edge("v2", "v31").breakable);
  EXPECT_EQ(network.get_edge("v00", "v1").max_speed, 100);
  EXPECT_EQ(network.get_edge("v01", "v1").max_speed, 100);
  EXPECT_EQ(network.get_edge("v1", "v1_v2_0").max_speed, 100);
  EXPECT_EQ(network.get_edge("v1_v2_0", "v1_v2_1").max_speed, 100);
  EXPECT_EQ(network.get_edge("v1_v2_1", "v1_v2_2").max_speed, 100);
  EXPECT_EQ(network.get_edge("v1_v2_2", "v2").max_speed, 100);
  EXPECT_EQ(network.get_edge("v2", "v30").max_speed, 100);
  EXPECT_EQ(network.get_edge("v2", "v31").max_speed, 100);
  // The edge v1 -> v2 does not exist anymore
  EXPECT_FALSE(network.has_edge("v1", "v2"));

  // The new edges are v1 -> v1_v2_0 -> v1_v2_1 -> v1_v2_2 -> v2 in this order
  EXPECT_EQ(network.get_edge_index("v1", "v1_v2_0"), new_edges.first[0]);
  EXPECT_EQ(network.get_edge_index("v1_v2_0", "v1_v2_1"), new_edges.first[1]);
  EXPECT_EQ(network.get_edge_index("v1_v2_1", "v1_v2_2"), new_edges.first[2]);
  EXPECT_EQ(network.get_edge_index("v1_v2_2", "v2"), new_edges.first[3]);

  // The last index of the new edges is identical to the old index of v1->v2
  EXPECT_EQ(new_edges.first.back(), v1_v2);

  // v00 has no incoming edges
  EXPECT_EQ(network.in_edges("v00").size(), 0);
  // v01 has no incoming edges
  EXPECT_EQ(network.in_edges("v01").size(), 0);
  // v1 has two incoming edges, namely from v00 and v01
  const auto& in_edges_v1 = network.in_edges("v1");
  EXPECT_EQ(in_edges_v1.size(), 2);
  EXPECT_TRUE(std::find(in_edges_v1.begin(), in_edges_v1.end(),
                        network.get_edge_index("v00", "v1")) !=
              in_edges_v1.end());
  EXPECT_TRUE(std::find(in_edges_v1.begin(), in_edges_v1.end(),
                        network.get_edge_index("v01", "v1")) !=
              in_edges_v1.end());
  // v1_v2_0 has one incoming edge, namely from v1
  const auto& in_edges_v1_v2_0 = network.in_edges("v1_v2_0");
  EXPECT_EQ(in_edges_v1_v2_0.size(), 1);
  EXPECT_TRUE(std::find(in_edges_v1_v2_0.begin(), in_edges_v1_v2_0.end(),
                        network.get_edge_index("v1", "v1_v2_0")) !=
              in_edges_v1_v2_0.end());
  // v1_v2_1 has one incoming edge, namely from v1_v2_0
  const auto& in_edges_v1_v2_1 = network.in_edges("v1_v2_1");
  EXPECT_EQ(in_edges_v1_v2_1.size(), 1);
  EXPECT_TRUE(std::find(in_edges_v1_v2_1.begin(), in_edges_v1_v2_1.end(),
                        network.get_edge_index("v1_v2_0", "v1_v2_1")) !=
              in_edges_v1_v2_1.end());
  // v1_v2_2 has one incoming edge, namely from v1_v2_1
  const auto& in_edges_v1_v2_2 = network.in_edges("v1_v2_2");
  EXPECT_EQ(in_edges_v1_v2_2.size(), 1);
  EXPECT_TRUE(std::find(in_edges_v1_v2_2.begin(), in_edges_v1_v2_2.end(),
                        network.get_edge_index("v1_v2_1", "v1_v2_2")) !=
              in_edges_v1_v2_2.end());
  // v2 has one incoming edge, namely from v1_v2_2
  const auto& in_edges_v2 = network.in_edges("v2");
  EXPECT_EQ(in_edges_v2.size(), 1);
  EXPECT_TRUE(std::find(in_edges_v2.begin(), in_edges_v2.end(),
                        network.get_edge_index("v1_v2_2", "v2")) !=
              in_edges_v2.end());
  // v30 has one incoming edge, namely from v2
  const auto& in_edges_v30 = network.in_edges("v30");
  EXPECT_EQ(in_edges_v30.size(), 1);
  EXPECT_TRUE(std::find(in_edges_v30.begin(), in_edges_v30.end(),
                        network.get_edge_index("v2", "v30")) !=
              in_edges_v30.end());
  // v31 has one incoming edge, namely from v2
  const auto& in_edges_v31 = network.in_edges("v31");
  EXPECT_EQ(in_edges_v31.size(), 1);
  EXPECT_TRUE(std::find(in_edges_v31.begin(), in_edges_v31.end(),
                        network.get_edge_index("v2", "v31")) !=
              in_edges_v31.end());
  // v00 has one outgoing edge, namely to v1
  const auto& out_edges_v00 = network.out_edges("v00");
  EXPECT_EQ(out_edges_v00.size(), 1);
  EXPECT_TRUE(std::find(out_edges_v00.begin(), out_edges_v00.end(),
                        network.get_edge_index("v00", "v1")) !=
              out_edges_v00.end());
  // v01 has one outgoing edge, namely to v1
  const auto& out_edges_v01 = network.out_edges("v01");
  EXPECT_EQ(out_edges_v01.size(), 1);
  EXPECT_TRUE(std::find(out_edges_v01.begin(), out_edges_v01.end(),
                        network.get_edge_index("v01", "v1")) !=
              out_edges_v01.end());
  // v1 has one outgoing edge, namely to v1_v2_0
  const auto& out_edges_v1 = network.out_edges("v1");
  EXPECT_EQ(out_edges_v1.size(), 1);
  EXPECT_TRUE(std::find(out_edges_v1.begin(), out_edges_v1.end(),
                        network.get_edge_index("v1", "v1_v2_0")) !=
              out_edges_v1.end());
  // v1_v2_0 has one outgoing edge, namely to v1_v2_1
  const auto& out_edges_v1_v2_0 = network.out_edges("v1_v2_0");
  EXPECT_EQ(out_edges_v1_v2_0.size(), 1);
  EXPECT_TRUE(std::find(out_edges_v1_v2_0.begin(), out_edges_v1_v2_0.end(),
                        network.get_edge_index("v1_v2_0", "v1_v2_1")) !=
              out_edges_v1_v2_0.end());
  // v1_v2_1 has one outgoing edge, namely to v1_v2_2
  const auto& out_edges_v1_v2_1 = network.out_edges("v1_v2_1");
  EXPECT_EQ(out_edges_v1_v2_1.size(), 1);
  EXPECT_TRUE(std::find(out_edges_v1_v2_1.begin(), out_edges_v1_v2_1.end(),
                        network.get_edge_index("v1_v2_1", "v1_v2_2")) !=
              out_edges_v1_v2_1.end());
  // v1_v2_2 has one outgoing edge, namely to v2
  const auto& out_edges_v1_v2_2 = network.out_edges("v1_v2_2");
  EXPECT_EQ(out_edges_v1_v2_2.size(), 1);
  EXPECT_TRUE(std::find(out_edges_v1_v2_2.begin(), out_edges_v1_v2_2.end(),
                        network.get_edge_index("v1_v2_2", "v2")) !=
              out_edges_v1_v2_2.end());
  // v2 has two outgoing edges, namely to v30 and v31
  const auto& out_edges_v2 = network.out_edges("v2");
  EXPECT_EQ(out_edges_v2.size(), 2);
  EXPECT_TRUE(std::find(out_edges_v2.begin(), out_edges_v2.end(),
                        network.get_edge_index("v2", "v30")) !=
              out_edges_v2.end());
  EXPECT_TRUE(std::find(out_edges_v2.begin(), out_edges_v2.end(),
                        network.get_edge_index("v2", "v31")) !=
              out_edges_v2.end());
  // v30 has no outgoing edges
  EXPECT_TRUE(network.out_edges("v30").empty());
  // v31 has no outgoing edges
  EXPECT_TRUE(network.out_edges("v31").empty());

  // The successors are essentially the same as the outgoing edges in this case
  // v00->v1 has one successor, namely v1->v1_v2_0
  const auto& successors_v00_v1 = network.get_successors("v00", "v1");
  EXPECT_EQ(successors_v00_v1.size(), 1);
  EXPECT_TRUE(std::find(successors_v00_v1.begin(), successors_v00_v1.end(),
                        network.get_edge_index("v1", "v1_v2_0")) !=
              successors_v00_v1.end());
  // v01->v1 has one successor, namely v1->v1_v2_0
  const auto& successors_v01_v1 = network.get_successors("v01", "v1");
  EXPECT_EQ(successors_v01_v1.size(), 1);
  EXPECT_TRUE(std::find(successors_v01_v1.begin(), successors_v01_v1.end(),
                        network.get_edge_index("v1", "v1_v2_0")) !=
              successors_v01_v1.end());
  // v1->v1_v2_0 has one successor, namely v1_v2_0->v1_v2_1
  const auto& successors_v1_v1_v2_0 = network.get_successors("v1", "v1_v2_0");
  EXPECT_EQ(successors_v1_v1_v2_0.size(), 1);
  EXPECT_TRUE(std::find(successors_v1_v1_v2_0.begin(),
                        successors_v1_v1_v2_0.end(),
                        network.get_edge_index("v1_v2_0", "v1_v2_1")) !=
              successors_v1_v1_v2_0.end());
  // v1_v2_0->v1_v2_1 has one successor, namely v1_v2_1->v1_v2_2
  const auto& successors_v1_v2_0_v1_v2_1 =
      network.get_successors("v1_v2_0", "v1_v2_1");
  EXPECT_EQ(successors_v1_v2_0_v1_v2_1.size(), 1);
  EXPECT_TRUE(std::find(successors_v1_v2_0_v1_v2_1.begin(),
                        successors_v1_v2_0_v1_v2_1.end(),
                        network.get_edge_index("v1_v2_1", "v1_v2_2")) !=
              successors_v1_v2_0_v1_v2_1.end());
  // v1_v2_1->v1_v2_2 has one successor, namely v1_v2_2->v2
  const auto& successors_v1_v2_1_v1_v2_2 =
      network.get_successors("v1_v2_1", "v1_v2_2");
  EXPECT_EQ(successors_v1_v2_1_v1_v2_2.size(), 1);
  EXPECT_TRUE(std::find(successors_v1_v2_1_v1_v2_2.begin(),
                        successors_v1_v2_1_v1_v2_2.end(),
                        network.get_edge_index("v1_v2_2", "v2")) !=
              successors_v1_v2_1_v1_v2_2.end());
  // v1_v2_2->v2 has two successors, namely v2->v30 and v2->v31
  const auto& successors_v1_v2_2_v2 = network.get_successors("v1_v2_2", "v2");
  EXPECT_EQ(successors_v1_v2_2_v2.size(), 2);
  EXPECT_TRUE(std::find(successors_v1_v2_2_v2.begin(),
                        successors_v1_v2_2_v2.end(),
                        network.get_edge_index("v2", "v30")) !=
              successors_v1_v2_2_v2.end());
  EXPECT_TRUE(std::find(successors_v1_v2_2_v2.begin(),
                        successors_v1_v2_2_v2.end(),
                        network.get_edge_index("v2", "v31")) !=
              successors_v1_v2_2_v2.end());
  // v2->v30 has no successors
  EXPECT_TRUE(network.get_successors("v2", "v30").empty());
  // v2->v31 has no successors
  EXPECT_TRUE(network.get_successors("v2", "v31").empty());

  // Check the mapping
  // Reminder v1 ->(len: 11) v1_v2_0 -> (len: 11) v1_v2_1 -> (len: 11) v1_v2_2
  // -> (len:11) v2
  std::pair<size_t, double> expected_pair = {v00_v1, 0};
  EXPECT_EQ(network.get_old_edge(v00, v1), expected_pair);
  expected_pair = {v1_v2, 0};
  EXPECT_EQ(network.get_old_edge("v1", "v1_v2_0"), expected_pair);
  expected_pair = {v1_v2, 11};
  EXPECT_EQ(network.get_old_edge("v1_v2_0", "v1_v2_1"), expected_pair);
  expected_pair = {v1_v2, 22};
  EXPECT_EQ(network.get_old_edge("v1_v2_1", "v1_v2_2"), expected_pair);
  expected_pair = {v1_v2, 33};
  EXPECT_EQ(network.get_old_edge("v1_v2_2", "v2"), expected_pair);
}

TEST(Functionality, NetworkExceptions) {
  cda_rail::Network network;
  const auto        v1 = network.add_vertex("v1", cda_rail::VertexType::TTD);
  EXPECT_THROW(network.add_vertex("v1", cda_rail::VertexType::TTD),
               cda_rail::exceptions::InvalidInputException);
  const auto v2 = network.add_vertex("v2", cda_rail::VertexType::TTD);

  EXPECT_THROW(network.add_edge(v1, v1, 300, 50, true, 5),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(network.add_edge(10, v2, 300, 50, true, 5),
               cda_rail::exceptions::VertexNotExistentException);
  EXPECT_THROW(network.add_edge(v1, 10, 300, 50, true, 5),
               cda_rail::exceptions::VertexNotExistentException);
  const auto v1_v2 = network.add_edge(v1, v2, 300, 50, true, 5);
  EXPECT_THROW(network.add_edge(v1, v2, 300, 50, true, 5),
               cda_rail::exceptions::InvalidInputException);

  const auto v3    = network.add_vertex("v3", cda_rail::VertexType::TTD);
  const auto v4    = network.add_vertex("v4", cda_rail::VertexType::TTD);
  const auto v2_v3 = network.add_edge("v2", "v3", 300, 50, true, 5);
  const auto v3_v4 = network.add_edge(v3, v4, 300, 50, true, 5);

  EXPECT_THROW(network.add_successor(v1_v2, 10),
               cda_rail::exceptions::EdgeNotExistentException);
  EXPECT_THROW(network.add_successor(10, v2_v3),
               cda_rail::exceptions::EdgeNotExistentException);
  EXPECT_THROW(network.add_successor(v1_v2, v3_v4),
               cda_rail::exceptions::ConsistencyException);

  network.add_successor(v1_v2, v2_v3);
  network.add_successor(v1_v2, v2_v3);
  network.add_successor(v2_v3, v3_v4);

  EXPECT_THROW(network.get_vertex_index("v9"),
               cda_rail::exceptions::VertexNotExistentException);
  EXPECT_THROW(network.get_vertex(10),
               cda_rail::exceptions::VertexNotExistentException);

  EXPECT_THROW(network.get_edge(10),
               cda_rail::exceptions::EdgeNotExistentException);
  EXPECT_THROW(network.get_edge(10, v2),
               cda_rail::exceptions::VertexNotExistentException);
  EXPECT_THROW(network.get_edge(v1, 10),
               cda_rail::exceptions::VertexNotExistentException);
  EXPECT_THROW(network.get_edge(v1, v3),
               cda_rail::exceptions::EdgeNotExistentException);
  EXPECT_THROW(network.get_edge_index(10, v2),
               cda_rail::exceptions::VertexNotExistentException);
  EXPECT_THROW(network.get_edge_index(v1, 10),
               cda_rail::exceptions::VertexNotExistentException);
  EXPECT_THROW(network.get_edge_index(v1, v3),
               cda_rail::exceptions::EdgeNotExistentException);
  EXPECT_THROW(network.has_edge(10, v2),
               cda_rail::exceptions::VertexNotExistentException);
  EXPECT_THROW(network.has_edge(v1, 10),
               cda_rail::exceptions::VertexNotExistentException);
  EXPECT_THROW(network.has_edge("v9", "v2"),
               cda_rail::exceptions::VertexNotExistentException);
  EXPECT_THROW(network.has_edge("v1", "v9"),
               cda_rail::exceptions::VertexNotExistentException);

  EXPECT_THROW(network.change_vertex_name(10, "v2"),
               cda_rail::exceptions::VertexNotExistentException);
  EXPECT_THROW(network.change_vertex_name(v1, "v2"),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(network.change_edge_length(10, 100),
               cda_rail::exceptions::EdgeNotExistentException);
  EXPECT_THROW(network.change_edge_max_speed(10, 100),
               cda_rail::exceptions::EdgeNotExistentException);
  EXPECT_THROW(network.change_edge_min_block_length(10, 100),
               cda_rail::exceptions::EdgeNotExistentException);
  EXPECT_THROW(network.set_edge_breakable(10),
               cda_rail::exceptions::EdgeNotExistentException);
  EXPECT_THROW(network.set_edge_unbreakable(10),
               cda_rail::exceptions::EdgeNotExistentException);

  EXPECT_THROW(network.out_edges(10),
               cda_rail::exceptions::VertexNotExistentException);
  EXPECT_THROW(network.in_edges(10),
               cda_rail::exceptions::VertexNotExistentException);
  EXPECT_THROW(network.neighbors(10),
               cda_rail::exceptions::VertexNotExistentException);
  EXPECT_THROW(network.is_adjustable(10),
               cda_rail::exceptions::VertexNotExistentException);
  EXPECT_THROW(network.change_vertex_type(10, cda_rail::VertexType::TTD),
               cda_rail::exceptions::VertexNotExistentException);

  EXPECT_THROW(network.get_successors(10),
               cda_rail::exceptions::EdgeNotExistentException);
  EXPECT_THROW(network.is_valid_successor(10, v2_v3),
               cda_rail::exceptions::EdgeNotExistentException);
  EXPECT_THROW(network.is_valid_successor(v1_v2, 10),
               cda_rail::exceptions::EdgeNotExistentException);
  EXPECT_THROW(network.get_reverse_edge_index(10),
               cda_rail::exceptions::EdgeNotExistentException);
}

TEST(Functionality, NetworkVertexIsAdjustable) {
  cda_rail::Network network;

  const auto v1 = network.add_vertex("v1", cda_rail::VertexType::TTD);
  const auto v2 = network.add_vertex("v2", cda_rail::VertexType::TTD);
  const auto v3 = network.add_vertex("v3", cda_rail::VertexType::NoBorder);
  const auto v4 = network.add_vertex("v4", cda_rail::VertexType::NoBorder);
  const auto v5 = network.add_vertex("v5", cda_rail::VertexType::TTD);
  const auto v6 = network.add_vertex("v6", cda_rail::VertexType::TTD);

  network.add_edge(v1, v2, 100, 100, false);
  network.add_edge(v2, v3, 100, 100, false);
  network.add_edge(v3, v4, 100, 100, false);
  network.add_edge(v4, v5, 100, 100, false);
  network.add_edge(v4, v6, 100, 100, false);

  EXPECT_FALSE(network.is_adjustable(v1));
  EXPECT_FALSE(network.is_adjustable(v2));
  EXPECT_TRUE(network.is_adjustable(v3));
  EXPECT_FALSE(network.is_adjustable(v4));
  EXPECT_FALSE(network.is_adjustable(v5));
  EXPECT_FALSE(network.is_adjustable(v6));

  EXPECT_THROW(network.is_adjustable(v1 + v2 + v3 + v4 + v5 + v6),
               cda_rail::exceptions::VertexNotExistentException);
}

TEST(Functionality, SortPairs) {
  cda_rail::Network network;
  // Add vertices
  network.add_vertex("v0", cda_rail::VertexType::TTD);
  network.add_vertex("v1", cda_rail::VertexType::NoBorderVSS);
  network.add_vertex("v2", cda_rail::VertexType::NoBorderVSS);
  network.add_vertex("v3", cda_rail::VertexType::NoBorderVSS);
  network.add_vertex("v4", cda_rail::VertexType::TTD);
  network.add_vertex("v5", cda_rail::VertexType::TTD);

  // Add edges
  const auto v0_v1 = network.add_edge("v0", "v1", 100, 100, false);
  const auto v2_v1 = network.add_edge("v2", "v1", 100, 100, false);
  const auto v1_v0 = network.add_edge("v1", "v0", 100, 100, false);
  const auto v1_v2 = network.add_edge("v1", "v2", 100, 100, false);
  const auto v2_v3 = network.add_edge("v2", "v3", 100, 100, false);
  const auto v3_v4 = network.add_edge("v3", "v4", 100, 100, false);
  const auto v4_v3 = network.add_edge("v4", "v3", 100, 100, false);
  network.add_edge("v4", "v5", 100, 100, false);
  network.add_edge("v5", "v4", 100, 100, false);

  // Edge pairs
  const std::vector<size_t> to_combine = {v3_v4, v4_v3, v2_v1, v1_v2,
                                          v1_v0, v0_v1, v2_v3};
  const auto& combined_edges = network.combine_reverse_edges(to_combine, true);

  // Check correctness
  std::vector<std::pair<std::optional<size_t>, std::optional<size_t>>>
      expected_combined_edges = {
          {v0_v1, v1_v0}, {v1_v2, v2_v1}, {v2_v3, {}}, {v3_v4, v4_v3}};
  EXPECT_EQ(combined_edges.size(), expected_combined_edges.size());
  size_t expected_index = 0;
  size_t expected_incr  = 1;
  if (combined_edges[0] != expected_combined_edges[0]) {
    expected_index = combined_edges.size() - 1;
    expected_incr  = -1;
  }
  for (const auto& combined_edge : combined_edges) {
    EXPECT_EQ(combined_edge, expected_combined_edges[expected_index]);
    expected_index += expected_incr;
  }
}

TEST(Functionality, NetworkEdgeSeparationReverse) {
  cda_rail::Network network;
  // Add vertices
  network.add_vertex("v00", cda_rail::VertexType::TTD);
  network.add_vertex("v01", cda_rail::VertexType::TTD);
  network.add_vertex("v1", cda_rail::VertexType::TTD);
  network.add_vertex("v2", cda_rail::VertexType::TTD);
  network.add_vertex("v30", cda_rail::VertexType::TTD);
  network.add_vertex("v31", cda_rail::VertexType::TTD);

  // Add edges
  const auto v00_v1 = network.add_edge("v00", "v1", 100, 100, false);
  const auto v01_v1 = network.add_edge("v01", "v1", 100, 100, false);
  const auto v1_v2  = network.add_edge("v1", "v2", 44, 100, true, 10);
  const auto v2_v30 = network.add_edge("v2", "v30", 100, 100, false);
  const auto v2_v31 = network.add_edge("v2", "v31", 100, 100, false);
  // Add reverse edges
  const auto v1_v00 = network.add_edge("v1", "v00", 100, 100, false);
  const auto v1_v01 = network.add_edge("v1", "v01", 100, 100, false);
  const auto v2_v1  = network.add_edge("v2", "v1", 44, 100, true, 10);
  const auto v30_v2 = network.add_edge("v30", "v2", 100, 100, false);
  const auto v31_v2 = network.add_edge("v31", "v2", 100, 100, false);

  // Add successors
  network.add_successor(v00_v1, v1_v2);
  network.add_successor(v01_v1, v1_v2);
  network.add_successor(v1_v2, v2_v30);
  network.add_successor(v1_v2, v2_v31);
  // Add reverse successors
  network.add_successor(v31_v2, v2_v1);
  network.add_successor(v30_v2, v2_v1);
  network.add_successor(v2_v1, v1_v00);
  network.add_successor(v2_v1, v1_v01);

  // Separate edge v1_v2 uniformly
  auto new_edges =
      network.separate_edge("v1", "v2", &cda_rail::vss::functions::uniform);

  // There are 4 new edges forward and 4 new edges reverse
  EXPECT_EQ(new_edges.first.size(), 4);
  EXPECT_EQ(new_edges.second.size(), 4);

  // Check if the vertices are correct
  // All old vertices still exist. Additionally, vertices v1_v2_0 to v1_v2_2
  // have been added with type NoBorder Hence, in total there are 9 vertices.
  EXPECT_EQ(network.number_of_vertices(), 9);
  EXPECT_TRUE(network.has_vertex("v1_v2_0"));
  EXPECT_TRUE(network.has_vertex("v1_v2_1"));
  EXPECT_TRUE(network.has_vertex("v1_v2_2"));
  EXPECT_EQ(network.get_vertex("v1_v2_0").type,
            cda_rail::VertexType::NoBorderVSS);
  EXPECT_EQ(network.get_vertex("v1_v2_1").type,
            cda_rail::VertexType::NoBorderVSS);
  EXPECT_EQ(network.get_vertex("v1_v2_2").type,
            cda_rail::VertexType::NoBorderVSS);
  EXPECT_TRUE(network.has_vertex("v00"));
  EXPECT_TRUE(network.has_vertex("v01"));
  EXPECT_TRUE(network.has_vertex("v1"));
  EXPECT_TRUE(network.has_vertex("v2"));
  EXPECT_TRUE(network.has_vertex("v30"));
  EXPECT_TRUE(network.has_vertex("v31"));
  EXPECT_EQ(network.get_vertex("v00").type, cda_rail::VertexType::TTD);
  EXPECT_EQ(network.get_vertex("v01").type, cda_rail::VertexType::TTD);
  EXPECT_EQ(network.get_vertex("v1").type, cda_rail::VertexType::TTD);
  EXPECT_EQ(network.get_vertex("v2").type, cda_rail::VertexType::TTD);
  EXPECT_EQ(network.get_vertex("v30").type, cda_rail::VertexType::TTD);
  EXPECT_EQ(network.get_vertex("v31").type, cda_rail::VertexType::TTD);

  // Check if the edges are correct
  EXPECT_EQ(network.number_of_edges(), 16);
  // v00/v01 ->(len: 100) v1 ->(len: 11) v1_v2_0 -> (len: 11) v1_v2_1 -> (len:
  // 11) v1_v2_2 -> (len:11) v2 -> (len: 100) v30/v31
  EXPECT_TRUE(network.has_edge("v00", "v1"));
  EXPECT_TRUE(network.has_edge("v01", "v1"));
  EXPECT_TRUE(network.has_edge("v1", "v1_v2_0"));
  EXPECT_TRUE(network.has_edge("v1_v2_0", "v1_v2_1"));
  EXPECT_TRUE(network.has_edge("v1_v2_1", "v1_v2_2"));
  EXPECT_TRUE(network.has_edge("v1_v2_2", "v2"));
  EXPECT_TRUE(network.has_edge("v2", "v30"));
  EXPECT_TRUE(network.has_edge("v2", "v31"));
  EXPECT_EQ(network.get_edge("v00", "v1").length, 100);
  EXPECT_EQ(network.get_edge("v01", "v1").length, 100);
  EXPECT_EQ(network.get_edge("v1", "v1_v2_0").length, 11);
  EXPECT_EQ(network.get_edge("v1_v2_0", "v1_v2_1").length, 11);
  EXPECT_EQ(network.get_edge("v1_v2_1", "v1_v2_2").length, 11);
  EXPECT_EQ(network.get_edge("v1_v2_2", "v2").length, 11);
  EXPECT_EQ(network.get_edge("v2", "v30").length, 100);
  EXPECT_EQ(network.get_edge("v2", "v31").length, 100);
  // The other properties are unchanged, except that all edges are unbreakable,
  // i.e., the breakable property is false
  EXPECT_FALSE(network.get_edge("v00", "v1").breakable);
  EXPECT_FALSE(network.get_edge("v01", "v1").breakable);
  EXPECT_FALSE(network.get_edge("v1", "v1_v2_0").breakable);
  EXPECT_FALSE(network.get_edge("v1_v2_0", "v1_v2_1").breakable);
  EXPECT_FALSE(network.get_edge("v1_v2_1", "v1_v2_2").breakable);
  EXPECT_FALSE(network.get_edge("v1_v2_2", "v2").breakable);
  EXPECT_FALSE(network.get_edge("v2", "v30").breakable);
  EXPECT_FALSE(network.get_edge("v2", "v31").breakable);
  EXPECT_EQ(network.get_edge("v00", "v1").max_speed, 100);
  EXPECT_EQ(network.get_edge("v01", "v1").max_speed, 100);
  EXPECT_EQ(network.get_edge("v1", "v1_v2_0").max_speed, 100);
  EXPECT_EQ(network.get_edge("v1_v2_0", "v1_v2_1").max_speed, 100);
  EXPECT_EQ(network.get_edge("v1_v2_1", "v1_v2_2").max_speed, 100);
  EXPECT_EQ(network.get_edge("v1_v2_2", "v2").max_speed, 100);
  EXPECT_EQ(network.get_edge("v2", "v30").max_speed, 100);
  EXPECT_EQ(network.get_edge("v2", "v31").max_speed, 100);
  // The edge v1 -> v2 does not exist anymore
  EXPECT_FALSE(network.has_edge("v1", "v2"));

  // The new edges are v1 -> v1_v2_0 -> v1_v2_1 -> v1_v2_2 -> v2 in this order
  EXPECT_EQ(network.get_edge_index("v1", "v1_v2_0"), new_edges.first[0]);
  EXPECT_EQ(network.get_edge_index("v1_v2_0", "v1_v2_1"), new_edges.first[1]);
  EXPECT_EQ(network.get_edge_index("v1_v2_1", "v1_v2_2"), new_edges.first[2]);
  EXPECT_EQ(network.get_edge_index("v1_v2_2", "v2"), new_edges.first[3]);

  // The last index of the new edges is identical to the old index of v1->v2
  EXPECT_EQ(new_edges.first.back(), v1_v2);

  // Check if the reverse edges are correct
  // v30/v31 ->(len: 100) v2 ->(len: 11) v1_v2_2 -> (len: 11) v1_v2_1 -> (len:
  // 11) v1_v2_0 -> (len:11) v1 -> (len: 100) v00/v01
  EXPECT_TRUE(network.has_edge("v30", "v2"));
  EXPECT_TRUE(network.has_edge("v31", "v2"));
  EXPECT_TRUE(network.has_edge("v2", "v1_v2_2"));
  EXPECT_TRUE(network.has_edge("v1_v2_2", "v1_v2_1"));
  EXPECT_TRUE(network.has_edge("v1_v2_1", "v1_v2_0"));
  EXPECT_TRUE(network.has_edge("v1_v2_0", "v1"));
  EXPECT_TRUE(network.has_edge("v1", "v00"));
  EXPECT_TRUE(network.has_edge("v1", "v01"));
  EXPECT_EQ(network.get_edge("v30", "v2").length, 100);
  EXPECT_EQ(network.get_edge("v31", "v2").length, 100);
  EXPECT_EQ(network.get_edge("v2", "v1_v2_2").length, 11);
  EXPECT_EQ(network.get_edge("v1_v2_2", "v1_v2_1").length, 11);
  EXPECT_EQ(network.get_edge("v1_v2_1", "v1_v2_0").length, 11);
  EXPECT_EQ(network.get_edge("v1_v2_0", "v1").length, 11);
  EXPECT_EQ(network.get_edge("v1", "v00").length, 100);
  EXPECT_EQ(network.get_edge("v1", "v01").length, 100);
  // The other properties are unchanged, except that all edges are unbreakable,
  // i.e., the breakable property is false
  EXPECT_FALSE(network.get_edge("v30", "v2").breakable);
  EXPECT_FALSE(network.get_edge("v31", "v2").breakable);
  EXPECT_FALSE(network.get_edge("v2", "v1_v2_2").breakable);
  EXPECT_FALSE(network.get_edge("v1_v2_2", "v1_v2_1").breakable);
  EXPECT_FALSE(network.get_edge("v1_v2_1", "v1_v2_0").breakable);
  EXPECT_FALSE(network.get_edge("v1_v2_0", "v1").breakable);
  EXPECT_FALSE(network.get_edge("v1", "v00").breakable);
  EXPECT_FALSE(network.get_edge("v1", "v01").breakable);
  EXPECT_EQ(network.get_edge("v30", "v2").max_speed, 100);
  EXPECT_EQ(network.get_edge("v31", "v2").max_speed, 100);
  EXPECT_EQ(network.get_edge("v2", "v1_v2_2").max_speed, 100);
  EXPECT_EQ(network.get_edge("v1_v2_2", "v1_v2_1").max_speed, 100);
  EXPECT_EQ(network.get_edge("v1_v2_1", "v1_v2_0").max_speed, 100);
  EXPECT_EQ(network.get_edge("v1_v2_0", "v1").max_speed, 100);
  EXPECT_EQ(network.get_edge("v1", "v00").max_speed, 100);
  EXPECT_EQ(network.get_edge("v1", "v01").max_speed, 100);
  // The edge v2 -> v1 does not exist anymore
  EXPECT_FALSE(network.has_edge("v2", "v1"));

  // The new edges are v2 -> v1_v2_2 -> v1_v2_1 -> v1_v2_0 -> v1 in this order
  EXPECT_EQ(network.get_edge_index("v2", "v1_v2_2"), new_edges.second[0]);
  EXPECT_EQ(network.get_edge_index("v1_v2_2", "v1_v2_1"), new_edges.second[1]);
  EXPECT_EQ(network.get_edge_index("v1_v2_1", "v1_v2_0"), new_edges.second[2]);
  EXPECT_EQ(network.get_edge_index("v1_v2_0", "v1"), new_edges.second[3]);

  // The last index of the new edges is identical to the old index of v2->v1
  EXPECT_EQ(new_edges.second.back(), v2_v1);

  // v00 has one incoming edge, namely v1 -> v00
  const auto& v00_incoming = network.in_edges("v00");
  EXPECT_EQ(v00_incoming.size(), 1);
  EXPECT_TRUE(std::find(v00_incoming.begin(), v00_incoming.end(),
                        network.get_edge_index("v1", "v00")) !=
              v00_incoming.end());
  // v01 has one incoming edge, namely v1 -> v01
  const auto& v01_incoming = network.in_edges("v01");
  EXPECT_EQ(v01_incoming.size(), 1);
  EXPECT_TRUE(std::find(v01_incoming.begin(), v01_incoming.end(),
                        network.get_edge_index("v1", "v01")) !=
              v01_incoming.end());
  // v1 has three incoming edges, namely from v00, v01 and v1_v2_0
  const auto& v1_incoming = network.in_edges("v1");
  EXPECT_EQ(v1_incoming.size(), 3);
  EXPECT_TRUE(std::find(v1_incoming.begin(), v1_incoming.end(),
                        network.get_edge_index("v00", "v1")) !=
              v1_incoming.end());
  EXPECT_TRUE(std::find(v1_incoming.begin(), v1_incoming.end(),
                        network.get_edge_index("v01", "v1")) !=
              v1_incoming.end());
  EXPECT_TRUE(std::find(v1_incoming.begin(), v1_incoming.end(),
                        network.get_edge_index("v1_v2_0", "v1")) !=
              v1_incoming.end());
  // v1_v2_0 has two incoming edges, namely from v1 and v1_v2_1
  const auto& v1_v2_0_incoming = network.in_edges("v1_v2_0");
  EXPECT_EQ(v1_v2_0_incoming.size(), 2);
  EXPECT_TRUE(std::find(v1_v2_0_incoming.begin(), v1_v2_0_incoming.end(),
                        network.get_edge_index("v1", "v1_v2_0")) !=
              v1_v2_0_incoming.end());
  EXPECT_TRUE(std::find(v1_v2_0_incoming.begin(), v1_v2_0_incoming.end(),
                        network.get_edge_index("v1_v2_1", "v1_v2_0")) !=
              v1_v2_0_incoming.end());
  // v1_v2_1 has two incoming edges, namely from v1_v2_0 and v1_v2_2
  const auto& v1_v2_1_incoming = network.in_edges("v1_v2_1");
  EXPECT_EQ(v1_v2_1_incoming.size(), 2);
  EXPECT_TRUE(std::find(v1_v2_1_incoming.begin(), v1_v2_1_incoming.end(),
                        network.get_edge_index("v1_v2_0", "v1_v2_1")) !=
              v1_v2_1_incoming.end());
  EXPECT_TRUE(std::find(v1_v2_1_incoming.begin(), v1_v2_1_incoming.end(),
                        network.get_edge_index("v1_v2_2", "v1_v2_1")) !=
              v1_v2_1_incoming.end());
  // v1_v2_2 has two incoming edges, namely from v1_v2_1 and v2
  const auto& v1_v2_2_incoming = network.in_edges("v1_v2_2");
  EXPECT_EQ(v1_v2_2_incoming.size(), 2);
  EXPECT_TRUE(std::find(v1_v2_2_incoming.begin(), v1_v2_2_incoming.end(),
                        network.get_edge_index("v1_v2_1", "v1_v2_2")) !=
              v1_v2_2_incoming.end());
  EXPECT_TRUE(std::find(v1_v2_2_incoming.begin(), v1_v2_2_incoming.end(),
                        network.get_edge_index("v2", "v1_v2_2")) !=
              v1_v2_2_incoming.end());
  // v2 has three incoming edges, namely from v1_v2_2, v30 and v31
  const auto& v2_incoming = network.in_edges("v2");
  EXPECT_EQ(v2_incoming.size(), 3);
  EXPECT_TRUE(std::find(v2_incoming.begin(), v2_incoming.end(),
                        network.get_edge_index("v1_v2_2", "v2")) !=
              v2_incoming.end());
  EXPECT_TRUE(std::find(v2_incoming.begin(), v2_incoming.end(),
                        network.get_edge_index("v30", "v2")) !=
              v2_incoming.end());
  EXPECT_TRUE(std::find(v2_incoming.begin(), v2_incoming.end(),
                        network.get_edge_index("v31", "v2")) !=
              v2_incoming.end());
  // v30 has one incoming edge, namely from v2
  const auto& v30_incoming = network.in_edges("v30");
  EXPECT_EQ(v30_incoming.size(), 1);
  EXPECT_TRUE(std::find(v30_incoming.begin(), v30_incoming.end(),
                        network.get_edge_index("v2", "v30")) !=
              v30_incoming.end());
  // v31 has one incoming edge, namely from v2
  const auto& v31_incoming = network.in_edges("v31");
  EXPECT_EQ(v31_incoming.size(), 1);
  EXPECT_TRUE(std::find(v31_incoming.begin(), v31_incoming.end(),
                        network.get_edge_index("v2", "v31")) !=
              v31_incoming.end());

  // v00 has one outgoing edge, namely to v1
  const auto& v00_outgoing = network.out_edges("v00");
  EXPECT_EQ(v00_outgoing.size(), 1);
  EXPECT_TRUE(std::find(v00_outgoing.begin(), v00_outgoing.end(),
                        network.get_edge_index("v00", "v1")) !=
              v00_outgoing.end());
  // v01 has one outgoing edge, namely to v1
  const auto& v01_outgoing = network.out_edges("v01");
  EXPECT_EQ(v01_outgoing.size(), 1);
  EXPECT_TRUE(std::find(v01_outgoing.begin(), v01_outgoing.end(),
                        network.get_edge_index("v01", "v1")) !=
              v01_outgoing.end());
  // v1 has three outgoing edges, namely to v00, v01 and v1_v2_0
  const auto& v1_outgoing = network.out_edges("v1");
  EXPECT_EQ(v1_outgoing.size(), 3);
  EXPECT_TRUE(std::find(v1_outgoing.begin(), v1_outgoing.end(),
                        network.get_edge_index("v1", "v00")) !=
              v1_outgoing.end());
  EXPECT_TRUE(std::find(v1_outgoing.begin(), v1_outgoing.end(),
                        network.get_edge_index("v1", "v01")) !=
              v1_outgoing.end());
  EXPECT_TRUE(std::find(v1_outgoing.begin(), v1_outgoing.end(),
                        network.get_edge_index("v1", "v1_v2_0")) !=
              v1_outgoing.end());
  // v1_v2_0 has two outgoing edges, namely to v1 and v1_v2_1
  const auto& v1_v2_0_outgoing = network.out_edges("v1_v2_0");
  EXPECT_EQ(v1_v2_0_outgoing.size(), 2);
  EXPECT_TRUE(std::find(v1_v2_0_outgoing.begin(), v1_v2_0_outgoing.end(),
                        network.get_edge_index("v1_v2_0", "v1")) !=
              v1_v2_0_outgoing.end());
  EXPECT_TRUE(std::find(v1_v2_0_outgoing.begin(), v1_v2_0_outgoing.end(),
                        network.get_edge_index("v1_v2_0", "v1_v2_1")) !=
              v1_v2_0_outgoing.end());
  // v1_v2_1 has two outgoing edges, namely to v1_v2_0 and v1_v2_2
  const auto& v1_v2_1_outgoing = network.out_edges("v1_v2_1");
  EXPECT_EQ(v1_v2_1_outgoing.size(), 2);
  EXPECT_TRUE(std::find(v1_v2_1_outgoing.begin(), v1_v2_1_outgoing.end(),
                        network.get_edge_index("v1_v2_1", "v1_v2_0")) !=
              v1_v2_1_outgoing.end());
  EXPECT_TRUE(std::find(v1_v2_1_outgoing.begin(), v1_v2_1_outgoing.end(),
                        network.get_edge_index("v1_v2_1", "v1_v2_2")) !=
              v1_v2_1_outgoing.end());
  // v1_v2_2 has two outgoing edges, namely to v1_v2_1 and v2
  const auto& v1_v2_2_outgoing = network.out_edges("v1_v2_2");
  EXPECT_EQ(v1_v2_2_outgoing.size(), 2);
  EXPECT_TRUE(std::find(v1_v2_2_outgoing.begin(), v1_v2_2_outgoing.end(),
                        network.get_edge_index("v1_v2_2", "v1_v2_1")) !=
              v1_v2_2_outgoing.end());
  EXPECT_TRUE(std::find(v1_v2_2_outgoing.begin(), v1_v2_2_outgoing.end(),
                        network.get_edge_index("v1_v2_2", "v2")) !=
              v1_v2_2_outgoing.end());
  // v2 has three outgoing edges, namely to v1_v2_2, v30 and v31
  const auto& v2_outgoing = network.out_edges("v2");
  EXPECT_EQ(v2_outgoing.size(), 3);
  EXPECT_TRUE(std::find(v2_outgoing.begin(), v2_outgoing.end(),
                        network.get_edge_index("v2", "v1_v2_2")) !=
              v2_outgoing.end());
  EXPECT_TRUE(std::find(v2_outgoing.begin(), v2_outgoing.end(),
                        network.get_edge_index("v2", "v30")) !=
              v2_outgoing.end());
  EXPECT_TRUE(std::find(v2_outgoing.begin(), v2_outgoing.end(),
                        network.get_edge_index("v2", "v31")) !=
              v2_outgoing.end());
  // v30 has one outgoing edge, namely to v2
  const auto& v30_outgoing = network.out_edges("v30");
  EXPECT_EQ(v30_outgoing.size(), 1);
  EXPECT_TRUE(std::find(v30_outgoing.begin(), v30_outgoing.end(),
                        network.get_edge_index("v30", "v2")) !=
              v30_outgoing.end());
  // v31 has one outgoing edge, namely to v2
  const auto& v31_outgoing = network.out_edges("v31");
  EXPECT_EQ(v31_outgoing.size(), 1);
  EXPECT_TRUE(std::find(v31_outgoing.begin(), v31_outgoing.end(),
                        network.get_edge_index("v31", "v2")) !=
              v31_outgoing.end());

  // Check the successors, they should disallow turning around
  // Successors of v00->v1 is the edge to v1_v2_0
  const auto& v00_v1_successors = network.get_successors("v00", "v1");
  EXPECT_EQ(v00_v1_successors.size(), 1);
  EXPECT_TRUE(std::find(v00_v1_successors.begin(), v00_v1_successors.end(),
                        network.get_edge_index("v1", "v1_v2_0")) !=
              v00_v1_successors.end());
  // Successors of v01->v1 is the edge to v1_v2_0
  const auto& v01_v1_successors = network.get_successors("v01", "v1");
  EXPECT_EQ(v01_v1_successors.size(), 1);
  EXPECT_TRUE(std::find(v01_v1_successors.begin(), v01_v1_successors.end(),
                        network.get_edge_index("v1", "v1_v2_0")) !=
              v01_v1_successors.end());
  // Successors of v1->v1_v2_0 is the edge to v1_v2_1
  const auto& v1_v1_v2_0_successors = network.get_successors("v1", "v1_v2_0");
  EXPECT_EQ(v1_v1_v2_0_successors.size(), 1);
  EXPECT_TRUE(std::find(v1_v1_v2_0_successors.begin(),
                        v1_v1_v2_0_successors.end(),
                        network.get_edge_index("v1_v2_0", "v1_v2_1")) !=
              v1_v1_v2_0_successors.end());
  // Successors of v1_v2_0->v1_v2_1 is the edge to v1_v2_2
  const auto& v1_v2_0_v1_v2_1_successors =
      network.get_successors("v1_v2_0", "v1_v2_1");
  EXPECT_EQ(v1_v2_0_v1_v2_1_successors.size(), 1);
  EXPECT_TRUE(std::find(v1_v2_0_v1_v2_1_successors.begin(),
                        v1_v2_0_v1_v2_1_successors.end(),
                        network.get_edge_index("v1_v2_1", "v1_v2_2")) !=
              v1_v2_0_v1_v2_1_successors.end());
  // Successors of v1_v2_1->v1_v2_2 is the edge to v2
  const auto& v1_v2_1_v1_v2_2_successors =
      network.get_successors("v1_v2_1", "v1_v2_2");
  EXPECT_EQ(v1_v2_1_v1_v2_2_successors.size(), 1);
  EXPECT_TRUE(std::find(v1_v2_1_v1_v2_2_successors.begin(),
                        v1_v2_1_v1_v2_2_successors.end(),
                        network.get_edge_index("v1_v2_2", "v2")) !=
              v1_v2_1_v1_v2_2_successors.end());
  // Successors of v1_v2_2->v2 are the edges to v30 and v31
  const auto& v1_v2_2_v2_successors = network.get_successors("v1_v2_2", "v2");
  EXPECT_EQ(v1_v2_2_v2_successors.size(), 2);
  EXPECT_TRUE(std::find(v1_v2_2_v2_successors.begin(),
                        v1_v2_2_v2_successors.end(),
                        network.get_edge_index("v2", "v30")) !=
              v1_v2_2_v2_successors.end());
  EXPECT_TRUE(std::find(v1_v2_2_v2_successors.begin(),
                        v1_v2_2_v2_successors.end(),
                        network.get_edge_index("v2", "v31")) !=
              v1_v2_2_v2_successors.end());
  // Successors of v2->v30 and v2->v31 are empty
  EXPECT_TRUE(network.get_successors("v2", "v30").empty());
  // Successors of v30->v2 is the edge to v1_v2_2
  const auto& v30_v2_successors = network.get_successors("v30", "v2");
  EXPECT_EQ(v30_v2_successors.size(), 1);
  EXPECT_TRUE(std::find(v30_v2_successors.begin(), v30_v2_successors.end(),
                        network.get_edge_index("v2", "v1_v2_2")) !=
              v30_v2_successors.end());
  // Successors of v31->v2 is the edge to v1_v2_2
  const auto& v31_v2_successors = network.get_successors("v31", "v2");
  EXPECT_EQ(v31_v2_successors.size(), 1);
  EXPECT_TRUE(std::find(v31_v2_successors.begin(), v31_v2_successors.end(),
                        network.get_edge_index("v2", "v1_v2_2")) !=
              v31_v2_successors.end());
  // Successors of v2->v1_v2_2 is the edge to v1_v2_1
  const auto& v2_v1_v2_2_successors = network.get_successors("v2", "v1_v2_2");
  EXPECT_EQ(v2_v1_v2_2_successors.size(), 1);
  EXPECT_TRUE(std::find(v2_v1_v2_2_successors.begin(),
                        v2_v1_v2_2_successors.end(),
                        network.get_edge_index("v1_v2_2", "v1_v2_1")) !=
              v2_v1_v2_2_successors.end());
  // Successors of v1_v2_2->v1_v2_1 is the edge to v1_v2_0
  const auto& v1_v2_2_v1_v2_1_successors =
      network.get_successors("v1_v2_2", "v1_v2_1");
  EXPECT_EQ(v1_v2_2_v1_v2_1_successors.size(), 1);
  EXPECT_TRUE(std::find(v1_v2_2_v1_v2_1_successors.begin(),
                        v1_v2_2_v1_v2_1_successors.end(),
                        network.get_edge_index("v1_v2_1", "v1_v2_0")) !=
              v1_v2_2_v1_v2_1_successors.end());
  // Successors of v1_v2_1->v1_v2_0 is the edge to v1
  const auto& v1_v2_1_v1_v2_0_successors =
      network.get_successors("v1_v2_1", "v1_v2_0");
  EXPECT_EQ(v1_v2_1_v1_v2_0_successors.size(), 1);
  EXPECT_TRUE(std::find(v1_v2_1_v1_v2_0_successors.begin(),
                        v1_v2_1_v1_v2_0_successors.end(),
                        network.get_edge_index("v1_v2_0", "v1")) !=
              v1_v2_1_v1_v2_0_successors.end());
  // Successors of v1_v2_0->v1 are the edges to v00 and v01
  const auto& v1_v2_0_v1_successors = network.get_successors("v1_v2_0", "v1");
  EXPECT_EQ(v1_v2_0_v1_successors.size(), 2);
  EXPECT_TRUE(std::find(v1_v2_0_v1_successors.begin(),
                        v1_v2_0_v1_successors.end(),
                        network.get_edge_index("v1", "v00")) !=
              v1_v2_0_v1_successors.end());
  EXPECT_TRUE(std::find(v1_v2_0_v1_successors.begin(),
                        v1_v2_0_v1_successors.end(),
                        network.get_edge_index("v1", "v01")) !=
              v1_v2_0_v1_successors.end());
  // Successors of v1->v00 and v1->v01 are empty
  EXPECT_TRUE(network.get_successors("v1", "v00").empty());
  EXPECT_TRUE(network.get_successors("v1", "v01").empty());

  // Check mapping
  // Reminder: v1 ->(len: 11) v1_v2_0 -> (len: 11) v1_v2_1 -> (len: 11) v1_v2_2
  // -> (len:11) v2
  std::pair<size_t, double> expected_pair = {v1_v2, 0};
  EXPECT_EQ(network.get_old_edge("v1", "v1_v2_0"), expected_pair);
  expected_pair = {v1_v2, 11};
  EXPECT_EQ(network.get_old_edge("v1_v2_0", "v1_v2_1"), expected_pair);
  expected_pair = {v1_v2, 22};
  EXPECT_EQ(network.get_old_edge("v1_v2_1", "v1_v2_2"), expected_pair);
  expected_pair = {v1_v2, 33};
  EXPECT_EQ(network.get_old_edge("v1_v2_2", "v2"), expected_pair);
  expected_pair = {v2_v1, 0};
  EXPECT_EQ(network.get_old_edge("v2", "v1_v2_2"), expected_pair);
  expected_pair = {v2_v1, 11};
  EXPECT_EQ(network.get_old_edge("v1_v2_2", "v1_v2_1"), expected_pair);
  expected_pair = {v2_v1, 22};
  EXPECT_EQ(network.get_old_edge("v1_v2_1", "v1_v2_0"), expected_pair);
  expected_pair = {v2_v1, 33};
  EXPECT_EQ(network.get_old_edge("v1_v2_0", "v1"), expected_pair);
}

TEST(Functionality, NetworkVerticesByType) {
  cda_rail::Network network;
  // Add vertices of each type NoBorder (1x), TTD (2x), VSS (3x), NoBorderVSS
  // (4x)
  const auto v1  = network.add_vertex("v1", cda_rail::VertexType::NoBorder);
  const auto v2  = network.add_vertex("v2", cda_rail::VertexType::TTD);
  const auto v3  = network.add_vertex("v3", cda_rail::VertexType::TTD);
  const auto v4  = network.add_vertex("v4", cda_rail::VertexType::VSS);
  const auto v5  = network.add_vertex("v5", cda_rail::VertexType::VSS);
  const auto v6  = network.add_vertex("v6", cda_rail::VertexType::VSS);
  const auto v7  = network.add_vertex("v7", cda_rail::VertexType::NoBorderVSS);
  const auto v8  = network.add_vertex("v8", cda_rail::VertexType::NoBorderVSS);
  const auto v9  = network.add_vertex("v9", cda_rail::VertexType::NoBorderVSS);
  const auto v10 = network.add_vertex("v10", cda_rail::VertexType::NoBorderVSS);

  // Check if the vertices are in the correct sets
  auto no_border = network.get_vertices_by_type(cda_rail::VertexType::NoBorder);
  EXPECT_EQ(no_border.size(), 1);
  EXPECT_TRUE(std::find(no_border.begin(), no_border.end(), v1) !=
              no_border.end());

  auto ttd = network.get_vertices_by_type(cda_rail::VertexType::TTD);
  EXPECT_EQ(ttd.size(), 2);
  EXPECT_TRUE(std::find(ttd.begin(), ttd.end(), v2) != ttd.end());
  EXPECT_TRUE(std::find(ttd.begin(), ttd.end(), v3) != ttd.end());

  auto vss = network.get_vertices_by_type(cda_rail::VertexType::VSS);
  EXPECT_EQ(vss.size(), 3);
  EXPECT_TRUE(std::find(vss.begin(), vss.end(), v4) != vss.end());
  EXPECT_TRUE(std::find(vss.begin(), vss.end(), v5) != vss.end());
  EXPECT_TRUE(std::find(vss.begin(), vss.end(), v6) != vss.end());

  auto no_border_vss =
      network.get_vertices_by_type(cda_rail::VertexType::NoBorderVSS);
  EXPECT_EQ(no_border_vss.size(), 4);
  EXPECT_TRUE(std::find(no_border_vss.begin(), no_border_vss.end(), v7) !=
              no_border_vss.end());
  EXPECT_TRUE(std::find(no_border_vss.begin(), no_border_vss.end(), v8) !=
              no_border_vss.end());
  EXPECT_TRUE(std::find(no_border_vss.begin(), no_border_vss.end(), v9) !=
              no_border_vss.end());
  EXPECT_TRUE(std::find(no_border_vss.begin(), no_border_vss.end(), v10) !=
              no_border_vss.end());
}

TEST(Functionality, NetworkVertexSpeed) {
  cda_rail::Network network;

  // Add vertices
  const auto v1  = network.add_vertex("v1", cda_rail::VertexType::TTD);
  const auto v2  = network.add_vertex("v2", cda_rail::VertexType::TTD);
  const auto v3  = network.add_vertex("v3", cda_rail::VertexType::TTD);
  const auto v41 = network.add_vertex("v41", cda_rail::VertexType::VSS);
  const auto v42 = network.add_vertex("v42", cda_rail::VertexType::VSS);
  const auto v5  = network.add_vertex("v5", cda_rail::VertexType::VSS);
  const auto v6  = network.add_vertex("v6", cda_rail::VertexType::VSS);
  const auto v7  = network.add_vertex("v7", cda_rail::VertexType::VSS);
  const auto v8  = network.add_vertex("v8", cda_rail::VertexType::VSS);

  // Add edges
  const auto e_1_2  = network.add_edge(v1, v2, 10, 30, false);
  const auto e_2_3  = network.add_edge(v2, v3, 20, 40, false);
  const auto e_3_41 = network.add_edge(v3, v41, 30, 50, false);
  const auto e_41_5 = network.add_edge(v41, v5, 40, 50, false);
  const auto e_5_6  = network.add_edge(v5, v6, 50, 40, false);
  const auto e_5_7  = network.add_edge(v5, v7, 60, 50, false);
  const auto e_7_8  = network.add_edge(v7, v8, 70, 20, false);
  const auto e_8_7  = network.add_edge(v8, v7, 70, 20, false);
  const auto e_7_42 = network.add_edge(v7, v42, 90, 30, false);
  const auto e_42_3 = network.add_edge(v42, v3, 100, 40, false);
  const auto e_5_41 = network.add_edge(v5, v41, 40, 50, false);
  const auto e_41_3 = network.add_edge(v41, v3, 30, 50, false);

  // Check velocity maximal speeds
  EXPECT_DOUBLE_EQ(network.maximal_vertex_speed(v1), 30);
  EXPECT_DOUBLE_EQ(network.maximal_vertex_speed("v2"), 30);
  EXPECT_DOUBLE_EQ(network.maximal_vertex_speed(v3), 40);
  EXPECT_DOUBLE_EQ(network.maximal_vertex_speed("v41"), 50);
  EXPECT_DOUBLE_EQ(network.maximal_vertex_speed(v5), 50);
  EXPECT_DOUBLE_EQ(network.maximal_vertex_speed("v6"), 40);
  EXPECT_DOUBLE_EQ(network.maximal_vertex_speed(v7), 30);
  EXPECT_DOUBLE_EQ(network.maximal_vertex_speed("v8"), 20);
  EXPECT_DOUBLE_EQ(network.maximal_vertex_speed(v42), 30);

  EXPECT_DOUBLE_EQ(network.maximal_vertex_speed(v7, {e_5_7, e_1_2, e_7_8}), 20);

  EXPECT_DOUBLE_EQ(network.minimal_neighboring_edge_length(v1), 10);
  EXPECT_DOUBLE_EQ(network.minimal_neighboring_edge_length("v2"), 10);
  EXPECT_DOUBLE_EQ(network.minimal_neighboring_edge_length(v3), 20);
  EXPECT_DOUBLE_EQ(network.minimal_neighboring_edge_length("v41"), 30);
  EXPECT_DOUBLE_EQ(network.minimal_neighboring_edge_length(v5), 40);
  EXPECT_DOUBLE_EQ(network.minimal_neighboring_edge_length("v6"), 50);
  EXPECT_DOUBLE_EQ(network.minimal_neighboring_edge_length(v7), 60);
  EXPECT_DOUBLE_EQ(network.minimal_neighboring_edge_length("v8"), 70);
  EXPECT_DOUBLE_EQ(network.minimal_neighboring_edge_length(v42), 90);

  EXPECT_DOUBLE_EQ(
      network.minimal_neighboring_edge_length(v7, {e_7_8, e_1_2, e_7_42}), 70);
  EXPECT_DOUBLE_EQ(network.minimal_neighboring_edge_length(v7, {e_1_2}),
                   std::numeric_limits<double>::infinity());
}

TEST(Functionality, ReverseIndices) {
  cda_rail::Network network;
  network.add_vertex("v1", cda_rail::VertexType::TTD);
  network.add_vertex("v2", cda_rail::VertexType::TTD);
  network.add_vertex("v3", cda_rail::VertexType::TTD);
  network.add_vertex("v4", cda_rail::VertexType::TTD);

  const auto e12 = network.add_edge("v1", "v2", 100, 10, false);
  const auto e23 = network.add_edge("v2", "v3", 100, 10, false);
  const auto e34 = network.add_edge("v3", "v4", 100, 10, false);
  const auto e43 = network.add_edge("v4", "v3", 100, 10, false);
  const auto e21 = network.add_edge("v2", "v1", 100, 10, false);

  // Check if the reverse indices are correct
  EXPECT_EQ(network.get_reverse_edge_index(e12), e21);
  EXPECT_EQ(network.get_reverse_edge_index(e23), std::optional<size_t>());
  EXPECT_EQ(network.get_reverse_edge_index(e34), e43);
  EXPECT_EQ(network.get_reverse_edge_index(e43), e34);
  EXPECT_EQ(network.get_reverse_edge_index(e21), e12);

  const std::vector edges          = {e12, e23, e34, e43, e21};
  const auto        edges_combined = network.combine_reverse_edges(edges);
  // Expect three edge sets
  EXPECT_EQ(edges_combined.size(), 3);
  // Expect the following pairs to exist: (min(e12, e21), max(e12, e21)), (e23,
  // -1), and (min(e34, e43), max(e34, e43))
  EXPECT_TRUE(
      std::find(edges_combined.begin(), edges_combined.end(),
                std::make_pair(std::optional<size_t>(std::min(e12, e21)),
                               std::optional<size_t>(std::max(e12, e21)))) !=
      edges_combined.end());
  EXPECT_TRUE(std::find(edges_combined.begin(), edges_combined.end(),
                        std::make_pair(std::optional<size_t>(e23),
                                       std::optional<size_t>())) !=
              edges_combined.end());
  EXPECT_TRUE(
      std::find(edges_combined.begin(), edges_combined.end(),
                std::make_pair(std::optional<size_t>(std::min(e34, e43)),
                               std::optional<size_t>(std::max(e34, e43)))) !=
      edges_combined.end());
}

TEST(Functionality, InverseEdges) {
  cda_rail::Network network;

  network.add_vertex("v1", cda_rail::VertexType::TTD);
  network.add_vertex("v2", cda_rail::VertexType::TTD);
  network.add_vertex("v3", cda_rail::VertexType::TTD);
  network.add_vertex("v4", cda_rail::VertexType::TTD);

  const auto e12 = network.add_edge("v1", "v2", 100, 10, false);
  const auto e23 = network.add_edge("v2", "v3", 100, 10, false);
  const auto e34 = network.add_edge("v3", "v4", 100, 10, false);
  const auto e32 = network.add_edge("v3", "v2", 100, 10, false);

  // Check if the inverse edges are correct

  // inverse of e12 and e23 is e34 and e32
  const auto inv_1 = network.inverse_edges({e12, e23});
  EXPECT_EQ(inv_1.size(), 2);
  EXPECT_TRUE(std::find(inv_1.begin(), inv_1.end(), e34) != inv_1.end());
  EXPECT_TRUE(std::find(inv_1.begin(), inv_1.end(), e32) != inv_1.end());

  // inverse of e23 and e32 only considering e12, e23 and e34 is e12 and e34
  const auto inv_2 = network.inverse_edges({e23, e32}, {e12, e23, e34});
  EXPECT_EQ(inv_2.size(), 2);
  EXPECT_TRUE(std::find(inv_2.begin(), inv_2.end(), e12) != inv_2.end());
  EXPECT_TRUE(std::find(inv_2.begin(), inv_2.end(), e34) != inv_2.end());
}

TEST(Functionality, ShortestPaths) {
  // Initialize empty network
  cda_rail::Network network;

  // Add 6 vertices
  const auto v1 = network.add_vertex("v1", cda_rail::VertexType::TTD);
  const auto v2 = network.add_vertex("v2", cda_rail::VertexType::TTD);
  const auto v3 = network.add_vertex("v3", cda_rail::VertexType::TTD);
  const auto v4 = network.add_vertex("v4", cda_rail::VertexType::TTD);
  const auto v5 = network.add_vertex("v5", cda_rail::VertexType::TTD);
  const auto v6 = network.add_vertex("v6", cda_rail::VertexType::TTD);

  // Add the following edges
  // v1 v2 of length 100
  const auto v1_v2 = network.add_edge("v1", "v2", 100, 10, false);
  // v2 v3 in both directions of length 200
  const auto v2_v3 = network.add_edge("v2", "v3", 200, 10, false);
  const auto v3_v2 = network.add_edge("v3", "v2", 200, 10, false);
  // v3 v4 in both directions of length 300
  const auto v3_v4 = network.add_edge("v3", "v4", 300, 10, false);
  const auto v4_v3 = network.add_edge("v4", "v3", 300, 10, false);
  // v4 v5 in both directions of length 400
  const auto v4_v5 = network.add_edge("v4", "v5", 400, 10, false);
  const auto v5_v4 = network.add_edge("v5", "v4", 400, 10, false);
  // v4 v1 of length 500
  const auto v4_v1 = network.add_edge("v4", "v1", 500, 10, false);
  // v3 v5 of length 500
  const auto v3_v5 = network.add_edge("v3", "v5", 500, 10, false);
  // v5 v6 in both directions of length 1000
  const auto v5_v6 = network.add_edge("v5", "v6", 1000, 10, false);
  const auto v6_v5 = network.add_edge("v6", "v5", 1000, 10, false);

  // Add successor edges
  network.add_successor(v1_v2, v2_v3);
  network.add_successor(v2_v3, v3_v4);
  network.add_successor(v2_v3, v3_v5);
  network.add_successor(v3_v4, v4_v5);
  network.add_successor(v3_v4, v4_v1);
  network.add_successor(v4_v3, v3_v2);
  network.add_successor(v4_v5, v5_v6);
  network.add_successor(v5_v4, v4_v3);
  network.add_successor(v4_v1, v1_v2);
  network.add_successor(v3_v5, v5_v6);
  network.add_successor(v6_v5, v5_v4);

  const auto shortest_paths = network.all_edge_pairs_shortest_paths();

  // Check if the shortest paths are correct
  // Starting from v1_v2, we reach
  // v1_v2 in 0
  // v2_v3 in 200
  // v3_v4 in 500
  // v3_v5 in 700
  // v4_v5 in 900
  // v5_v6 in 1700
  // v4_v1 in 1000
  // all other edges are not reachable
  EXPECT_EQ(shortest_paths[v1_v2][v1_v2], 0);
  EXPECT_EQ(shortest_paths[v1_v2][v2_v3], 200);
  EXPECT_EQ(shortest_paths[v1_v2][v3_v4], 500);
  EXPECT_EQ(shortest_paths[v1_v2][v3_v5], 700);
  EXPECT_EQ(shortest_paths[v1_v2][v4_v5], 900);
  EXPECT_EQ(shortest_paths[v1_v2][v5_v6], 1700);
  EXPECT_EQ(shortest_paths[v1_v2][v4_v1], 1000);
  EXPECT_EQ(shortest_paths[v1_v2][v3_v2], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v1_v2][v4_v3], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v1_v2][v5_v4], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v1_v2][v6_v5], cda_rail::INF);

  EXPECT_EQ(network.shortest_path(v1_v2, v2), 0);
  EXPECT_EQ(network.shortest_path(v1_v2, v3), 200);
  EXPECT_EQ(network.shortest_path(v1_v2, v4), 500);
  EXPECT_EQ(network.shortest_path(v1_v2, v5), 700);
  EXPECT_EQ(network.shortest_path(v1_v2, v6), 1700);
  EXPECT_EQ(network.shortest_path(v1_v2, v1), 1000);

  // Starting from v2_v3, we reach
  // v2_v3 in 0
  // v3_v4 in 300
  // v3_v5 in 500
  // v4_v5 in 700
  // v5_v6 in 1500
  // v4_v1 in 800
  // v1_v2 in 900
  // all other edges are not reachable
  EXPECT_EQ(shortest_paths[v2_v3][v2_v3], 0);
  EXPECT_EQ(shortest_paths[v2_v3][v3_v4], 300);
  EXPECT_EQ(shortest_paths[v2_v3][v3_v5], 500);
  EXPECT_EQ(shortest_paths[v2_v3][v4_v5], 700);
  EXPECT_EQ(shortest_paths[v2_v3][v5_v6], 1500);
  EXPECT_EQ(shortest_paths[v2_v3][v4_v1], 800);
  EXPECT_EQ(shortest_paths[v2_v3][v1_v2], 900);
  EXPECT_EQ(shortest_paths[v2_v3][v3_v2], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v2_v3][v4_v3], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v2_v3][v5_v4], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v2_v3][v6_v5], cda_rail::INF);

  EXPECT_EQ(network.shortest_path(v2_v3, v1), 800);
  EXPECT_EQ(network.shortest_path(v2_v3, v2), 900);
  EXPECT_EQ(network.shortest_path(v2_v3, v3), 0);
  EXPECT_EQ(network.shortest_path(v2_v3, v4), 300);
  EXPECT_EQ(network.shortest_path(v2_v3, v5), 500);
  EXPECT_EQ(network.shortest_path(v2_v3, v6), 1500);

  // Starting from v3_v4, we reach
  // v3_v4 in 0
  // v4_v5 in 400
  // v5_v6 in 1400
  // v4_v1 in 500
  // v1_v2 in 600
  // v2_v3 in 800
  // v3_v5 in 1300
  // all other edges are not reachable
  EXPECT_EQ(shortest_paths[v3_v4][v3_v4], 0);
  EXPECT_EQ(shortest_paths[v3_v4][v4_v5], 400);
  EXPECT_EQ(shortest_paths[v3_v4][v5_v6], 1400);
  EXPECT_EQ(shortest_paths[v3_v4][v4_v1], 500);
  EXPECT_EQ(shortest_paths[v3_v4][v1_v2], 600);
  EXPECT_EQ(shortest_paths[v3_v4][v2_v3], 800);
  EXPECT_EQ(shortest_paths[v3_v4][v3_v5], 1300);
  EXPECT_EQ(shortest_paths[v3_v4][v3_v2], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v3_v4][v4_v3], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v3_v4][v5_v4], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v3_v4][v6_v5], cda_rail::INF);

  EXPECT_EQ(network.shortest_path(v3_v4, v1), 500);
  EXPECT_EQ(network.shortest_path(v3_v4, v2), 600);
  EXPECT_EQ(network.shortest_path(v3_v4, v3), 800);
  EXPECT_EQ(network.shortest_path(v3_v4, v4), 0);
  EXPECT_EQ(network.shortest_path(v3_v4, v5), 400);
  EXPECT_EQ(network.shortest_path(v3_v4, v6), 1400);

  // Starting from v3_v5, we reach
  // v3_v5 in 0
  // v5_v6 in 1000
  // all other edges are not reachable
  EXPECT_EQ(shortest_paths[v3_v5][v3_v5], 0);
  EXPECT_EQ(shortest_paths[v3_v5][v5_v6], 1000);
  EXPECT_EQ(shortest_paths[v3_v5][v3_v4], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v3_v5][v4_v5], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v3_v5][v4_v1], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v3_v5][v1_v2], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v3_v5][v2_v3], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v3_v5][v4_v3], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v3_v5][v5_v4], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v3_v5][v6_v5], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v3_v5][v3_v2], cda_rail::INF);

  EXPECT_FALSE(network.shortest_path(v3_v5, v1).has_value());
  EXPECT_FALSE(network.shortest_path(v3_v5, v2).has_value());
  EXPECT_FALSE(network.shortest_path(v3_v5, v3).has_value());
  EXPECT_FALSE(network.shortest_path(v3_v5, v4).has_value());
  EXPECT_EQ(network.shortest_path(v3_v5, v5), 0);
  EXPECT_EQ(network.shortest_path(v3_v5, v6), 1000);

  // Starting from v4_v5, we reach
  // v4_v5 in 0
  // v5_v6 in 1000
  // all other edges are not reachable
  EXPECT_EQ(shortest_paths[v4_v5][v4_v5], 0);
  EXPECT_EQ(shortest_paths[v4_v5][v5_v6], 1000);
  EXPECT_EQ(shortest_paths[v4_v5][v3_v4], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v4_v5][v3_v5], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v4_v5][v4_v1], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v4_v5][v1_v2], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v4_v5][v2_v3], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v4_v5][v4_v3], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v4_v5][v5_v4], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v4_v5][v6_v5], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v4_v5][v3_v2], cda_rail::INF);

  EXPECT_FALSE(network.shortest_path(v4_v5, v1).has_value());
  EXPECT_FALSE(network.shortest_path(v4_v5, v2).has_value());
  EXPECT_FALSE(network.shortest_path(v4_v5, v3).has_value());
  EXPECT_FALSE(network.shortest_path(v4_v5, v4).has_value());
  EXPECT_EQ(network.shortest_path(v4_v5, v5), 0);
  EXPECT_EQ(network.shortest_path(v4_v5, v6), 1000);

  // Starting from v5_v6, we reach
  // v5_v6 in 0
  // all other edges are not reachable
  EXPECT_EQ(shortest_paths[v5_v6][v5_v6], 0);
  EXPECT_EQ(shortest_paths[v5_v6][v3_v4], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v5_v6][v3_v5], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v5_v6][v4_v5], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v5_v6][v4_v1], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v5_v6][v1_v2], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v5_v6][v2_v3], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v5_v6][v4_v3], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v5_v6][v5_v4], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v5_v6][v6_v5], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v5_v6][v3_v2], cda_rail::INF);

  EXPECT_FALSE(network.shortest_path(v5_v6, v1).has_value());
  EXPECT_FALSE(network.shortest_path(v5_v6, v2).has_value());
  EXPECT_FALSE(network.shortest_path(v5_v6, v3).has_value());
  EXPECT_FALSE(network.shortest_path(v5_v6, v4).has_value());
  EXPECT_FALSE(network.shortest_path(v5_v6, v5).has_value());
  EXPECT_EQ(network.shortest_path(v5_v6, v6), 0);

  // Starting from v4_v1, we reach
  // v4_v1 in 0
  // v1_v2 in 100
  // v2_v3 in 300
  // v3_v4 in 600
  // v4_v5 in 1000
  // v3_v5 in 800
  // v5_v6 in 1800
  // all other edges are not reachable
  EXPECT_EQ(shortest_paths[v4_v1][v4_v1], 0);
  EXPECT_EQ(shortest_paths[v4_v1][v1_v2], 100);
  EXPECT_EQ(shortest_paths[v4_v1][v2_v3], 300);
  EXPECT_EQ(shortest_paths[v4_v1][v3_v4], 600);
  EXPECT_EQ(shortest_paths[v4_v1][v4_v5], 1000);
  EXPECT_EQ(shortest_paths[v4_v1][v3_v5], 800);
  EXPECT_EQ(shortest_paths[v4_v1][v5_v6], 1800);
  EXPECT_EQ(shortest_paths[v4_v1][v3_v2], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v4_v1][v4_v3], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v4_v1][v5_v4], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v4_v1][v6_v5], cda_rail::INF);

  EXPECT_EQ(network.shortest_path(v4_v1, v1), 0);
  EXPECT_EQ(network.shortest_path(v4_v1, v2), 100);
  EXPECT_EQ(network.shortest_path(v4_v1, v3), 300);
  EXPECT_EQ(network.shortest_path(v4_v1, v4), 600);
  EXPECT_EQ(network.shortest_path(v4_v1, v5), 800);
  EXPECT_EQ(network.shortest_path(v4_v1, v6), 1800);

  // Starting from v6_v5, we reach
  // v6_v5 in 0
  // v5_v4 in 400
  // v4_v3 in 700
  // v3_v2 in 900
  // all other edges are not reachable
  EXPECT_EQ(shortest_paths[v6_v5][v6_v5], 0);
  EXPECT_EQ(shortest_paths[v6_v5][v5_v4], 400);
  EXPECT_EQ(shortest_paths[v6_v5][v4_v3], 700);
  EXPECT_EQ(shortest_paths[v6_v5][v3_v2], 900);
  EXPECT_EQ(shortest_paths[v6_v5][v3_v4], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v6_v5][v3_v5], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v6_v5][v4_v5], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v6_v5][v4_v1], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v6_v5][v1_v2], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v6_v5][v2_v3], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v6_v5][v5_v6], cda_rail::INF);

  EXPECT_FALSE(network.shortest_path(v6_v5, v1).has_value());
  EXPECT_EQ(network.shortest_path(v6_v5, v2), 900);
  EXPECT_EQ(network.shortest_path(v6_v5, v3), 700);
  EXPECT_EQ(network.shortest_path(v6_v5, v4), 400);
  EXPECT_EQ(network.shortest_path(v6_v5, v5), 0);
  EXPECT_FALSE(network.shortest_path(v6_v5, v6).has_value());

  // Starting from v5_v4, we reach
  // v5_v4 in 0
  // v4_v3 in 300
  // v3_v2 in 500
  // all other edges are not reachable
  EXPECT_EQ(shortest_paths[v5_v4][v5_v4], 0);
  EXPECT_EQ(shortest_paths[v5_v4][v4_v3], 300);
  EXPECT_EQ(shortest_paths[v5_v4][v3_v2], 500);
  EXPECT_EQ(shortest_paths[v5_v4][v3_v4], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v5_v4][v3_v5], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v5_v4][v4_v5], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v5_v4][v4_v1], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v5_v4][v1_v2], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v5_v4][v2_v3], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v5_v4][v5_v6], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v5_v4][v6_v5], cda_rail::INF);

  EXPECT_FALSE(network.shortest_path(v5_v4, v1).has_value());
  EXPECT_EQ(network.shortest_path(v5_v4, v2), 500);
  EXPECT_EQ(network.shortest_path(v5_v4, v3), 300);
  EXPECT_EQ(network.shortest_path(v5_v4, v4), 0);
  EXPECT_FALSE(network.shortest_path(v5_v4, v5).has_value());
  EXPECT_FALSE(network.shortest_path(v5_v4, v6).has_value());

  // Starting from v4_v3, we reach
  // v4_v3 in 0
  // v3_v2 in 200
  // all other edges are not reachable
  EXPECT_EQ(shortest_paths[v4_v3][v4_v3], 0);
  EXPECT_EQ(shortest_paths[v4_v3][v3_v2], 200);
  EXPECT_EQ(shortest_paths[v4_v3][v3_v4], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v4_v3][v3_v5], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v4_v3][v4_v5], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v4_v3][v4_v1], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v4_v3][v1_v2], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v4_v3][v2_v3], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v4_v3][v5_v4], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v4_v3][v5_v6], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v4_v3][v6_v5], cda_rail::INF);

  EXPECT_FALSE(network.shortest_path(v4_v3, v1).has_value());
  EXPECT_EQ(network.shortest_path(v4_v3, v2), 200);
  EXPECT_EQ(network.shortest_path(v4_v3, v3), 0);
  EXPECT_FALSE(network.shortest_path(v4_v3, v4).has_value());
  EXPECT_FALSE(network.shortest_path(v4_v3, v5).has_value());
  EXPECT_FALSE(network.shortest_path(v4_v3, v6).has_value());

  // Starting from v3_v2, we reach
  // v3_v2 in 0
  // all other edges are not reachable
  EXPECT_EQ(shortest_paths[v3_v2][v3_v2], 0);
  EXPECT_EQ(shortest_paths[v3_v2][v3_v4], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v3_v2][v3_v5], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v3_v2][v4_v3], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v3_v2][v4_v5], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v3_v2][v4_v1], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v3_v2][v1_v2], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v3_v2][v2_v3], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v3_v2][v5_v4], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v3_v2][v5_v6], cda_rail::INF);
  EXPECT_EQ(shortest_paths[v3_v2][v6_v5], cda_rail::INF);

  EXPECT_FALSE(network.shortest_path(v3_v2, v1).has_value());
  EXPECT_EQ(network.shortest_path(v3_v2, v2), 0);
  EXPECT_FALSE(network.shortest_path(v3_v2, v3).has_value());
  EXPECT_FALSE(network.shortest_path(v3_v2, v4).has_value());
  EXPECT_FALSE(network.shortest_path(v3_v2, v5).has_value());
  EXPECT_FALSE(network.shortest_path(v3_v2, v6).has_value());

  // Shortest paths
  const auto [shortest_paths_1_val, shortest_paths_1_path] =
      network.shortest_path_using_edges(v1_v2, v6);
  EXPECT_TRUE(shortest_paths_1_val.has_value());
  EXPECT_EQ(shortest_paths_1_val.value(), 1700);
  EXPECT_EQ(shortest_paths_1_path.size(), 4);
  EXPECT_EQ(shortest_paths_1_path,
            std::vector<size_t>({v1_v2, v2_v3, v3_v5, v5_v6}));

  const auto [shortest_paths_1b_val, shortest_paths_1b_path] =
      network.shortest_path_using_edges(v1_v2, v6, false);
  EXPECT_TRUE(shortest_paths_1b_val.has_value());
  EXPECT_EQ(shortest_paths_1b_val.value(), 1700);
  EXPECT_EQ(shortest_paths_1b_path.size(), 4);
  EXPECT_EQ(shortest_paths_1b_path,
            std::vector<size_t>({v1_v2, v2_v3, v3_v5, v5_v6}));

  const auto [shortest_paths_2_val, shortest_paths_2_path] =
      network.shortest_path_using_edges(
          v1_v2, v6, true,
          {v1_v2, v2_v3, v3_v4, v4_v3, v4_v5, v5_v4, v5_v6, v6_v5});
  EXPECT_TRUE(shortest_paths_2_val.has_value());
  EXPECT_EQ(shortest_paths_2_val.value(), 1900);
  EXPECT_EQ(shortest_paths_2_path.size(), 5);
  EXPECT_EQ(shortest_paths_2_path,
            std::vector<size_t>({v1_v2, v2_v3, v3_v4, v4_v5, v5_v6}));

  const auto [shortest_paths_2b_val, shortest_paths_2b_path] =
      network.shortest_path_using_edges(
          v1_v2, v6, false,
          {v1_v2, v2_v3, v3_v4, v4_v3, v4_v5, v5_v4, v5_v6, v6_v5});
  EXPECT_TRUE(shortest_paths_2b_val.has_value());
  EXPECT_EQ(shortest_paths_2b_val.value(), 1900);
  EXPECT_EQ(shortest_paths_2b_path.size(), 5);
  EXPECT_EQ(shortest_paths_2b_path,
            std::vector<size_t>({v1_v2, v2_v3, v3_v4, v4_v5, v5_v6}));

  const auto [shortest_paths_3_val, shortest_paths_3_path] =
      network.shortest_path_using_edges(v5_v4, v1, true);
  EXPECT_FALSE(shortest_paths_3_val.has_value());
  EXPECT_TRUE(shortest_paths_3_path.empty());

  const auto [shortest_paths_3b_val, shortest_paths_3b_path] =
      network.shortest_path_using_edges(v5_v4, v1, false);
  EXPECT_TRUE(shortest_paths_3b_val.has_value());
  EXPECT_EQ(shortest_paths_3b_val.value(), 500);
  EXPECT_EQ(shortest_paths_3b_path.size(), 2);
  EXPECT_EQ(shortest_paths_3b_path, std::vector<size_t>({v5_v4, v4_v1}));

  const auto [shortest_paths_4_val, shortest_paths_4_path] =
      network.shortest_path_using_edges(v1_v2, v2);
  EXPECT_TRUE(shortest_paths_4_val.has_value());
  EXPECT_EQ(shortest_paths_4_val.value(), 0);
  EXPECT_EQ(shortest_paths_4_path.size(), 1);
  EXPECT_EQ(shortest_paths_4_path, std::vector<size_t>({v1_v2}));
}

TEST(Functionality, QuickestPaths) {
  cda_rail::Network network;

  // Add 5 vertices
  const auto v1 = network.add_vertex("v1", cda_rail::VertexType::TTD);
  const auto v2 = network.add_vertex("v2", cda_rail::VertexType::TTD);
  const auto v3 = network.add_vertex("v3", cda_rail::VertexType::TTD);
  const auto v4 = network.add_vertex("v4", cda_rail::VertexType::TTD);
  const auto v5 = network.add_vertex("v5", cda_rail::VertexType::TTD);

  // Add v1 -> v2 -> v3 -> v4 edges with max speed 10
  const auto v1_v2 = network.add_edge("v1", "v2", 100, 10, false);
  const auto v2_v3 = network.add_edge("v2", "v3", 200, 10, false);
  const auto v3_v4 = network.add_edge("v3", "v4", 100, 10, false);

  // Add v2 -> v5 -> v3 each of lengths 200 with max speed 40
  const auto v2_v5 = network.add_edge("v2", "v5", 200, 40, false);
  const auto v5_v3 = network.add_edge("v5", "v3", 200, 40, false);

  // Add successors
  network.add_successor(v1_v2, v2_v3);
  network.add_successor(v1_v2, v2_v5);
  network.add_successor(v2_v3, v3_v4);
  network.add_successor(v2_v5, v5_v3);
  network.add_successor(v5_v3, v3_v4);

  // Calculate shortest path
  const auto shortest_dist =
      network.shortest_path(v1_v2, v4, false, false, false);
  EXPECT_TRUE(shortest_dist.has_value());
  // 200+100
  EXPECT_EQ(shortest_dist.value(), 300);

  const auto shortest_dist_edge =
      network.shortest_path(v1_v2, v3_v4, true, false, false);
  EXPECT_TRUE(shortest_dist_edge.has_value());
  EXPECT_EQ(shortest_dist_edge.value(), 300);

  const auto shortest_dist_zero =
      network.shortest_path(v1_v2, v2, false, false, false);
  EXPECT_TRUE(shortest_dist_zero.has_value());
  EXPECT_EQ(shortest_dist_zero.value(), 0);

  const auto shortest_dist_zero_edge =
      network.shortest_path(v1_v2, v1_v2, true, false, false);
  EXPECT_TRUE(shortest_dist_zero_edge.has_value());
  EXPECT_EQ(shortest_dist_zero_edge.value(), 0);

  // Calculate quickest path
  const auto quickest_dist_1 =
      network.shortest_path(v1_v2, v4, false, false, true);
  EXPECT_TRUE(quickest_dist_1.has_value());
  // 200/40 + 200/40 + 100/10 = 20
  EXPECT_EQ(quickest_dist_1.value(), 20);
  const auto quickest_dist_1b =
      network.shortest_path(v1_v2, v4, false, true, true);
  // 100/10 more -> 30
  EXPECT_TRUE(quickest_dist_1b.has_value());
  EXPECT_EQ(quickest_dist_1b.value(), 30);

  const auto quickest_dist_1_edge =
      network.shortest_path(v1_v2, v3_v4, true, false, true);
  EXPECT_TRUE(quickest_dist_1_edge.has_value());
  EXPECT_EQ(quickest_dist_1_edge.value(), 20);

  // Calculate quickest path with max speed 25
  const auto quickest_dist_2 =
      network.shortest_path(v1_v2, v4, false, false, true, 25);
  EXPECT_TRUE(quickest_dist_2.has_value());
  // 200/25 + 200/25 + 100/10 = 26
  EXPECT_EQ(quickest_dist_2.value(), 26);

  const auto quickest_dist_2_edge =
      network.shortest_path(v1_v2, v3_v4, true, false, true, 25);
  EXPECT_TRUE(quickest_dist_2_edge.has_value());
  EXPECT_EQ(quickest_dist_2_edge.value(), 26);

  // Calculate quickest path with max speed 15
  const auto quickest_dist_3 =
      network.shortest_path(v1_v2, v4, false, false, true, 15);
  EXPECT_TRUE(quickest_dist_3.has_value());
  // 200/10 + 100/10 = 30
  EXPECT_EQ(quickest_dist_3.value(), 30);

  const auto quickest_dist_3_edge =
      network.shortest_path(v1_v2, v3_v4, true, false, true, 15);
  EXPECT_TRUE(quickest_dist_3_edge.has_value());
  EXPECT_EQ(quickest_dist_3_edge.value(), 30);

  const auto& quickest_dist_4 =
      network.shortest_path(v1_v2, v3, false, false, true, 10);
  EXPECT_TRUE(quickest_dist_4.has_value());
  // 200/10 = 20
  EXPECT_EQ(quickest_dist_4.value(), 20);

  const auto& quickest_dist_4_edge =
      network.shortest_path(v1_v2, v5_v3, true, false, true, 10);
  EXPECT_TRUE(quickest_dist_4_edge.has_value());
  // 200/10 + 200/10 = 40
  EXPECT_EQ(quickest_dist_4_edge.value(), 40);

  EXPECT_THROW(network.shortest_path(v1_v2, v4, false, false, true, -1),
               cda_rail::exceptions::InvalidInputException);
}

TEST(Functionality, ShortestPathsBetweenSets) {
  cda_rail::Network network;

  // Add 6 vertices
  const auto v1 = network.add_vertex("v1", cda_rail::VertexType::TTD);
  const auto v2 = network.add_vertex("v2", cda_rail::VertexType::TTD);
  const auto v3 = network.add_vertex("v3", cda_rail::VertexType::TTD);
  const auto v4 = network.add_vertex("v4", cda_rail::VertexType::TTD);
  const auto v5 = network.add_vertex("v5", cda_rail::VertexType::TTD);
  const auto v6 = network.add_vertex("v6", cda_rail::VertexType::TTD);

  // Add v4 -> v3 -> v2 -> v1 edges with lengths 100 each
  const auto v4_v3 = network.add_edge("v4", "v3", 100, 10, false);
  const auto v3_v2 = network.add_edge("v3", "v2", 100, 10, false);
  const auto v2_v1 = network.add_edge("v2", "v1", 100, 10, false);

  // Add v3 -> v4 -> v5 -> v6 edges with lengths 100 each (except v4 -> v5 which
  // is 200)
  const auto v3_v4 = network.add_edge("v3", "v4", 100, 10, false);
  const auto v4_v5 = network.add_edge("v4", "v5", 200, 10, false);
  const auto v5_v6 = network.add_edge("v5", "v6", 100, 10, false);

  // Add v2 -> v5 edge with length 500
  const auto v2_v5 = network.add_edge("v2", "v5", 500, 10, false);

  // Add successors
  // v4 -> v3 -> v2 -> v1
  network.add_successor(v4_v3, v3_v2);
  network.add_successor(v3_v2, v2_v1);
  // v3 -> v4 -> v5 -> v6
  network.add_successor(v3_v4, v4_v5);
  network.add_successor(v4_v5, v5_v6);
  // v3 -> v2 -> v5 -> v6
  network.add_successor(v3_v2, v2_v5);
  network.add_successor(v2_v5, v5_v6);

  // Calculate shortest paths between sets
  const auto shortest_dist_1 = network.shortest_path(v4_v3, v6, false);
  // 100 + 500 + 100 = 700
  EXPECT_TRUE(shortest_dist_1.has_value());
  EXPECT_EQ(shortest_dist_1.value(), 700);
  const auto shortest_dist_1_edge = network.shortest_path(v4_v3, v5_v6, true);
  EXPECT_TRUE(shortest_dist_1_edge.has_value());
  EXPECT_EQ(shortest_dist_1_edge.value(), 700);

  const auto shortest_dist_2 =
      network.shortest_path_between_sets({v4_v3}, {v6, v1}, false);
  // 100 + 100 = 200
  EXPECT_TRUE(shortest_dist_2.has_value());
  EXPECT_EQ(shortest_dist_2.value(), 200);
  const auto shortest_dist_2b =
      network.shortest_path_between_sets({v4_v3}, {v6, v1}, false, true);
  // 100 more for first edge -> 300
  EXPECT_TRUE(shortest_dist_2b.has_value());
  EXPECT_EQ(shortest_dist_2b.value(), 300);
  const auto shortest_dist_2_edge =
      network.shortest_path_between_sets({v4_v3}, {v5_v6, v2_v1}, true);
  EXPECT_TRUE(shortest_dist_2_edge.has_value());
  EXPECT_EQ(shortest_dist_2_edge.value(), 200);

  const auto shortest_dist_3 =
      network.shortest_path_between_sets({v4_v3, v3_v4}, {v6}, false);
  // 200 + 100 = 300
  EXPECT_TRUE(shortest_dist_3.has_value());
  EXPECT_EQ(shortest_dist_3.value(), 300);
  const auto shortest_dist_3b =
      network.shortest_path_between_sets({v4_v3, v3_v4}, {v6}, false, true);
  // 100 more for first edge -> 400
  EXPECT_TRUE(shortest_dist_3b.has_value());
  EXPECT_EQ(shortest_dist_3b.value(), 400);
  const auto shortest_dist_3_edge =
      network.shortest_path_between_sets({v4_v3, v3_v4}, {v5_v6}, true);
  EXPECT_TRUE(shortest_dist_3_edge.has_value());
  EXPECT_EQ(shortest_dist_3_edge.value(), 300);

  const auto shortest_dist_4 =
      network.shortest_path_between_sets({v4_v3, v3_v4}, {v1, v6}, false);
  // 100 + 100 = 200
  EXPECT_TRUE(shortest_dist_4.has_value());
  EXPECT_EQ(shortest_dist_4.value(), 200);
  const auto shortest_dist_4_edge =
      network.shortest_path_between_sets({v4_v3, v3_v4}, {v2_v1, v5_v6}, true);
  EXPECT_TRUE(shortest_dist_4_edge.has_value());
  EXPECT_EQ(shortest_dist_4_edge.value(), 200);

  EXPECT_THROW(network.shortest_path_between_sets({}, {0}),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(network.shortest_path_between_sets({0}, {}),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(network.shortest_path_between_sets({1000}, {v1}, false),
               cda_rail::exceptions::EdgeNotExistentException);
  EXPECT_THROW(network.shortest_path_between_sets({v2_v1}, {1000}, false),
               cda_rail::exceptions::VertexNotExistentException);
  EXPECT_THROW(network.shortest_path_between_sets({v2_v1}, {1000}, true),
               cda_rail::exceptions::EdgeNotExistentException);
}

TEST(Functionality, ReadTrains) {
  auto trains = cda_rail::TrainList::import_trains(
      "./example-networks/SimpleStation/timetable/");

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
  EXPECT_TRUE(tr1.tim);

  // Check if the train tr2 is imported correctly
  auto tr2 = trains.get_train("tr2");
  EXPECT_EQ(tr2.name, "tr2");
  EXPECT_EQ(tr2.length, 100);
  EXPECT_EQ(tr2.max_speed, 27.78);
  EXPECT_EQ(tr2.acceleration, 2);
  EXPECT_EQ(tr2.deceleration, 1);
  EXPECT_TRUE(tr2.tim);

  // Check if the train tr3 is imported correctly
  auto tr3 = trains.get_train("tr3");
  EXPECT_EQ(tr3.name, "tr3");
  EXPECT_EQ(tr3.length, 250);
  EXPECT_EQ(tr3.max_speed, 20);
  EXPECT_EQ(tr3.acceleration, 2);
  EXPECT_EQ(tr3.deceleration, 1);
  EXPECT_TRUE(tr3.tim);
}

TEST(Functionality, WriteTrains) {
  // Create a train list
  auto       trains    = cda_rail::TrainList();
  const auto tr1_index = trains.add_train("tr1", 100, 83.33, 2, 1);
  const auto tr2_index = trains.add_train("tr2", 100, 27.78, 2, 1);
  const auto tr3_index = trains.add_train("tr3", 250, 20, 2, 1);

  // check the train indices
  EXPECT_EQ(trains.get_train_index("tr1"), tr1_index);
  EXPECT_EQ(trains.get_train_index("tr2"), tr2_index);
  EXPECT_EQ(trains.get_train_index("tr3"), tr3_index);

  // Write the train list to a file
  trains.export_trains("./tmp/write_trains_test");

  // Read the train list from the file
  auto trains_read =
      cda_rail::TrainList::import_trains("./tmp/write_trains_test");

  // Delete created directory and everything in it
  std::filesystem::remove_all("./tmp");

  // Check if the all trains are imported
  EXPECT_EQ(trains_read.size(), 3);
  EXPECT_TRUE(trains_read.has_train("tr1"));
  EXPECT_TRUE(trains_read.has_train("tr2"));
  EXPECT_TRUE(trains_read.has_train("tr3"));

  // Check if the train tr1 is imported correctly
  auto tr1 = trains_read.get_train("tr1");
  EXPECT_EQ(tr1.name, "tr1");
  EXPECT_EQ(tr1.length, 100);
  EXPECT_EQ(tr1.max_speed, 83.33);
  EXPECT_EQ(tr1.acceleration, 2);
  EXPECT_EQ(tr1.deceleration, 1);
  EXPECT_TRUE(tr1.tim);

  // Check if the train tr2 is imported correctly
  auto tr2 = trains_read.get_train("tr2");
  EXPECT_EQ(tr2.name, "tr2");
  EXPECT_EQ(tr2.length, 100);
  EXPECT_EQ(tr2.max_speed, 27.78);
  EXPECT_EQ(tr2.acceleration, 2);
  EXPECT_EQ(tr2.deceleration, 1);
  EXPECT_TRUE(tr2.tim);

  // Check if the train tr3 is imported correctly
  auto tr3 = trains_read.get_train("tr3");
  EXPECT_EQ(tr3.name, "tr3");
  EXPECT_EQ(tr3.length, 250);
  EXPECT_EQ(tr3.max_speed, 20);
  EXPECT_EQ(tr3.acceleration, 2);
  EXPECT_EQ(tr3.deceleration, 1);
  EXPECT_TRUE(tr3.tim);
}

TEST(Functionality, EditTrains) {
  // Create a train list
  auto       trains    = cda_rail::TrainList();
  const auto tr1_index = trains.add_train("tr1", 100, 83.33, 2, 1, false);

  EXPECT_EQ(trains.get_train("tr1").length, 100);
  EXPECT_EQ(trains.get_train(tr1_index).max_speed, 83.33);
  EXPECT_EQ(trains.get_train("tr1").acceleration, 2);
  EXPECT_EQ(trains.get_train(tr1_index).deceleration, 1);
  EXPECT_FALSE(trains.get_train("tr1").tim);

  trains.editable_tr("tr1").length           = 200;
  trains.editable_tr(tr1_index).max_speed    = 50;
  trains.editable_tr("tr1").acceleration     = 3;
  trains.editable_tr(tr1_index).deceleration = 2;
  trains.editable_tr("tr1").tim              = true;

  EXPECT_EQ(trains.get_train("tr1").length, 200);
  EXPECT_EQ(trains.get_train(tr1_index).max_speed, 50);
  EXPECT_EQ(trains.get_train("tr1").acceleration, 3);
  EXPECT_EQ(trains.get_train(tr1_index).deceleration, 2);
  EXPECT_TRUE(trains.get_train("tr1").tim);
}

TEST(Functionality, TrainExceptions) {
  // Create a train list
  auto trains = cda_rail::TrainList();
  trains.add_train("tr1", 100, 83.33, 2, 1, false);
  EXPECT_THROW(trains.add_train("tr1", 100, 83.33, 2, 1, false),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_THROW(trains.get_train_index("tr2"),
               cda_rail::exceptions::TrainNotExistentException);
  EXPECT_THROW(trains.get_train(10),
               cda_rail::exceptions::TrainNotExistentException);
  EXPECT_THROW(trains.editable_tr(10),
               cda_rail::exceptions::TrainNotExistentException);
  EXPECT_THROW(trains.get_train("tr2"),
               cda_rail::exceptions::TrainNotExistentException);
  EXPECT_THROW(trains.editable_tr("tr2"),
               cda_rail::exceptions::TrainNotExistentException);
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
  auto network = cda_rail::Network::import_network(
      "./example-networks/SimpleStation/network/");
  auto stations = cda_rail::StationList::import_stations(
      "./example-networks/SimpleStation/timetable/", network);

  // Check if the station is imported correctly
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
  auto                station_tracks = station.tracks;
  std::sort(station_tracks.begin(), station_tracks.end());
  std::sort(track_ids.begin(), track_ids.end());
  EXPECT_EQ(station_tracks, track_ids);
}

TEST(Functionality, WriteStations) {
  auto network = cda_rail::Network::import_network(
      "./example-networks/SimpleStation/network/");
  cda_rail::StationList stations;

  stations.add_station("S1");
  stations.add_station("S2");

  const auto& l0_l1 = network.get_edge_index("l0", "l1");

  stations.add_track_to_station("S1", "l0", "l1", network);
  stations.add_track_to_station("S1", "l0", "l1", network);
  stations.add_track_to_station("S2", "l0", "l1", network);
  stations.add_track_to_station("S2", "l1", "l2", network);

  EXPECT_THROW(stations.get_station("S3"),
               cda_rail::exceptions::StationNotExistentException);
  EXPECT_THROW(stations.add_track_to_station("S3", l0_l1, network),
               cda_rail::exceptions::StationNotExistentException);
  EXPECT_THROW(stations.add_track_to_station("S1", 100, network),
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
  const std::vector<size_t> s1_tracks{network.get_edge_index("l0", "l1")};
  EXPECT_EQ(s1.tracks, s1_tracks);

  const auto& s2 = stations_read.get_station("S2");
  EXPECT_EQ(s2.name, "S2");
  EXPECT_EQ(s2.tracks.size(), 2);
  std::vector<size_t> s2_tracks_target{network.get_edge_index("l0", "l1"),
                                       network.get_edge_index("l1", "l2")};
  auto                s2_tracks = s2.tracks;
  std::sort(s2_tracks.begin(), s2_tracks.end());
  std::sort(s2_tracks_target.begin(), s2_tracks_target.end());
  EXPECT_EQ(s2_tracks, s2_tracks_target);
}

TEST(Functionality, ReadTimetable) {
  auto network = cda_rail::Network::import_network(
      "./example-networks/SimpleStation/network/");
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
  std::vector<size_t> track_ids_target{network.get_edge_index("g00", "g01"),
                                       network.get_edge_index("g10", "g11"),
                                       network.get_edge_index("g01", "g00"),
                                       network.get_edge_index("g11", "g10")};
  auto                track_ids = station.tracks;
  std::sort(track_ids.begin(), track_ids.end());
  std::sort(track_ids_target.begin(), track_ids_target.end());
  EXPECT_EQ(track_ids, track_ids_target);

  // Check if the timetable has the correct trains
  const auto& trains = timetable.get_train_list();
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
  const auto& tr1_schedule = timetable.get_schedule("tr1");
  EXPECT_EQ(tr1_schedule.get_t_0(), 120);
  EXPECT_EQ(tr1_schedule.get_v_0(), 0);
  EXPECT_EQ(tr1_schedule.get_t_n(), 645);
  EXPECT_EQ(tr1_schedule.get_v_n(), 16.67);
  EXPECT_EQ(network.get_vertex(tr1_schedule.get_entry()).name, "l0");
  EXPECT_EQ(network.get_vertex(tr1_schedule.get_exit()).name, "r0");
  EXPECT_EQ(tr1_schedule.get_stops().size(), 1);
  const auto& stop = tr1_schedule.get_stops()[0];
  EXPECT_EQ(stop.arrival(), 240);
  EXPECT_EQ(stop.departure(), 300);
  EXPECT_EQ(stations.get_station(stop.get_station_name()).name, "Central");

  // Check the schedule of tr2
  const auto& tr2_schedule = timetable.get_schedule("tr2");
  EXPECT_EQ(tr2_schedule.get_t_0(), 0);
  EXPECT_EQ(tr2_schedule.get_v_0(), 0);
  EXPECT_EQ(tr2_schedule.get_t_n(), 420);
  EXPECT_EQ(tr2_schedule.get_v_n(), 16.67);
  EXPECT_EQ(network.get_vertex(tr2_schedule.get_entry()).name, "l0");
  EXPECT_EQ(network.get_vertex(tr2_schedule.get_exit()).name, "r0");
  EXPECT_EQ(tr2_schedule.get_stops().size(), 1);
  const auto& stop2 = tr2_schedule.get_stops()[0];
  EXPECT_EQ(stop2.arrival(), 120);
  EXPECT_EQ(stop2.departure(), 300);
  EXPECT_EQ(stations.get_station(stop2.get_station_name()).name, "Central");

  // Check the schedule of tr3
  const auto& tr3_schedule = timetable.get_schedule("tr3");
  EXPECT_EQ(tr3_schedule.get_t_0(), 0);
  EXPECT_EQ(tr3_schedule.get_v_0(), 0);
  EXPECT_EQ(tr3_schedule.get_t_n(), 420);
  EXPECT_EQ(tr3_schedule.get_v_n(), 16.67);
  EXPECT_EQ(network.get_vertex(tr3_schedule.get_entry()).name, "r0");
  EXPECT_EQ(network.get_vertex(tr3_schedule.get_exit()).name, "l0");
  EXPECT_EQ(tr3_schedule.get_stops().size(), 1);
  const auto& stop3 = tr3_schedule.get_stops()[0];
  EXPECT_EQ(stop3.arrival(), 180);
  EXPECT_EQ(stop3.departure(), 300);
  EXPECT_EQ(stations.get_station(stop3.get_station_name()).name, "Central");

  EXPECT_EQ(timetable.max_t(), 645);

  EXPECT_TRUE(timetable.check_consistency(network));
}

TEST(Functionality, WriteTimetable) {
  auto network = cda_rail::Network::import_network(
      "./example-networks/SimpleStation/network/");
  cda_rail::Timetable timetable;

  timetable.add_train("tr1", 100, 83.33, 2, 1, 0, 0, "l0", 300, 20, "r0",
                      network);
  timetable.add_train("tr2", 100, 27.78, 2, 1, 0, 0, "r0", 300, 20, "l0",
                      network);

  const std::pair<int, int> time_interval_expected{0, 300};

  EXPECT_EQ(timetable.time_interval("tr1"), time_interval_expected);
  EXPECT_EQ(timetable.time_interval("tr2"), time_interval_expected);

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
  const auto& stations = timetable.get_station_list();
  EXPECT_EQ(stations.size(), 2);
  EXPECT_TRUE(stations.has_station("Station1"));
  EXPECT_TRUE(stations.has_station("Station2"));

  // Check if the stations are imported correctly
  const auto& st1 = stations.get_station("Station1");
  EXPECT_EQ(st1.name, "Station1");
  EXPECT_EQ(st1.tracks.size(), 4);
  std::vector<size_t> s1_expected_tracks = {
      network.get_edge_index("g00", "g01"),
      network.get_edge_index("g10", "g11"),
      network.get_edge_index("g01", "g00"),
      network.get_edge_index("g11", "g10")};
  auto st1_tracks = st1.tracks;
  std::sort(st1_tracks.begin(), st1_tracks.end());
  std::sort(s1_expected_tracks.begin(), s1_expected_tracks.end());
  EXPECT_EQ(st1_tracks, s1_expected_tracks);
  const auto& st2 = stations.get_station("Station2");
  EXPECT_EQ(st2.name, "Station2");
  EXPECT_EQ(st2.tracks.size(), 1);
  const std::vector<size_t> s2_expected_tracks = {
      network.get_edge_index("r1", "r0")};
  EXPECT_EQ(st2.tracks, s2_expected_tracks);

  // Check if the timetable has the correct trains
  const auto& trains = timetable.get_train_list();
  EXPECT_EQ(trains.size(), 2);
  EXPECT_TRUE(trains.has_train("tr1"));
  EXPECT_TRUE(trains.has_train("tr2"));

  // Check if the train tr1 is saved correctly
  auto tr1 = trains.get_train("tr1");
  EXPECT_EQ(tr1.name, "tr1");
  EXPECT_EQ(tr1.length, 100);
  EXPECT_EQ(tr1.max_speed, 83.33);
  EXPECT_EQ(tr1.acceleration, 2);
  EXPECT_EQ(tr1.deceleration, 1);
  // Check if the train tr2 is saved correctly
  auto tr2 = trains.get_train("tr2");
  EXPECT_EQ(tr2.name, "tr2");
  EXPECT_EQ(tr2.length, 100);
  EXPECT_EQ(tr2.max_speed, 27.78);
  EXPECT_EQ(tr2.acceleration, 2);
  EXPECT_EQ(tr2.deceleration, 1);

  // Check if the schedule of tr1 is saved correctly
  const auto& tr1_schedule = timetable.get_schedule("tr1");
  EXPECT_EQ(tr1_schedule.get_t_0(), 0);
  EXPECT_EQ(tr1_schedule.get_v_0(), 0);
  EXPECT_EQ(tr1_schedule.get_t_n(), 300);
  EXPECT_EQ(tr1_schedule.get_v_n(), 20);
  EXPECT_EQ(network.get_vertex(tr1_schedule.get_entry()).name, "l0");
  EXPECT_EQ(network.get_vertex(tr1_schedule.get_exit()).name, "r0");
  EXPECT_EQ(tr1_schedule.get_stops().size(), 2);
  const auto& stop1 = tr1_schedule.get_stops()[0];
  EXPECT_EQ(stop1.arrival(), 100);
  EXPECT_EQ(stop1.departure(), 160);
  EXPECT_EQ(stations.get_station(stop1.get_station_name()).name, "Station1");
  const auto& stop2 = tr1_schedule.get_stops()[1];
  EXPECT_EQ(stop2.arrival(), 200);
  EXPECT_EQ(stop2.departure(), 260);
  EXPECT_EQ(stations.get_station(stop2.get_station_name()).name, "Station2");

  // Check if the schedule of tr2 is saved correctly
  const auto& tr2_schedule = timetable.get_schedule("tr2");
  EXPECT_EQ(tr2_schedule.get_t_0(), 0);
  EXPECT_EQ(tr2_schedule.get_v_0(), 0);
  EXPECT_EQ(tr2_schedule.get_t_n(), 300);
  EXPECT_EQ(tr2_schedule.get_v_n(), 20);
  EXPECT_EQ(network.get_vertex(tr2_schedule.get_entry()).name, "r0");
  EXPECT_EQ(network.get_vertex(tr2_schedule.get_exit()).name, "l0");
  EXPECT_EQ(tr2_schedule.get_stops().size(), 1);
  const auto& stop3 = tr2_schedule.get_stops()[0];
  EXPECT_EQ(stop3.arrival(), 160);
  EXPECT_EQ(stop3.departure(), 220);
  EXPECT_EQ(stations.get_station(stop3.get_station_name()).name, "Station1");

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
  auto st1_read_tracks = st1_read.tracks;
  std::sort(st1_read_tracks.begin(), st1_read_tracks.end());
  EXPECT_EQ(st1_read_tracks, s1_expected_tracks);
  const auto& st2_read = stations_read.get_station("Station2");
  EXPECT_EQ(st2_read.name, "Station2");
  EXPECT_EQ(st2_read.tracks.size(), 1);
  auto st2_read_tracks = st2_read.tracks;
  std::sort(st2_read_tracks.begin(), st2_read_tracks.end());
  EXPECT_EQ(st2_read_tracks, s2_expected_tracks);

  // Check if the timetable has the correct trains
  const auto& trains_read = timetable_read.get_train_list();
  EXPECT_EQ(trains_read.size(), 2);
  EXPECT_TRUE(trains_read.has_train("tr1"));
  EXPECT_TRUE(trains_read.has_train("tr2"));

  // Check if the train tr1 is saved correctly
  auto tr1_read = trains_read.get_train("tr1");
  EXPECT_EQ(tr1_read.name, "tr1");
  EXPECT_EQ(tr1_read.length, 100);
  EXPECT_EQ(tr1_read.max_speed, 83.33);
  EXPECT_EQ(tr1_read.acceleration, 2);
  EXPECT_EQ(tr1_read.deceleration, 1);
  // Check if the train tr2 is saved correctly
  auto tr2_read = trains_read.get_train("tr2");
  EXPECT_EQ(tr2_read.name, "tr2");
  EXPECT_EQ(tr2_read.length, 100);
  EXPECT_EQ(tr2_read.max_speed, 27.78);
  EXPECT_EQ(tr2_read.acceleration, 2);
  EXPECT_EQ(tr2_read.deceleration, 1);

  // Check if the schedule of tr1 is saved correctly
  const auto& tr1_schedule_read = timetable_read.get_schedule("tr1");
  EXPECT_EQ(tr1_schedule_read.get_t_0(), 0);
  EXPECT_EQ(tr1_schedule_read.get_v_0(), 0);
  EXPECT_EQ(tr1_schedule_read.get_t_n(), 300);
  EXPECT_EQ(tr1_schedule_read.get_v_n(), 20);
  EXPECT_EQ(network.get_vertex(tr1_schedule_read.get_entry()).name, "l0");
  EXPECT_EQ(network.get_vertex(tr1_schedule_read.get_exit()).name, "r0");
  EXPECT_EQ(tr1_schedule_read.get_stops().size(), 2);
  const auto& stop1_read = tr1_schedule_read.get_stops()[0];
  EXPECT_EQ(stop1_read.arrival(), 100);
  EXPECT_EQ(stop1_read.departure(), 160);
  EXPECT_EQ(stations_read.get_station(stop1_read.get_station_name()).name,
            "Station1");
  const auto& stop2_read = tr1_schedule_read.get_stops()[1];
  EXPECT_EQ(stop2_read.arrival(), 200);
  EXPECT_EQ(stop2_read.departure(), 260);
  EXPECT_EQ(stations_read.get_station(stop2_read.get_station_name()).name,
            "Station2");

  // Check if the schedule of tr2 is saved correctly
  const auto& tr2_schedule_read = timetable_read.get_schedule("tr2");
  EXPECT_EQ(tr2_schedule_read.get_t_0(), 0);
  EXPECT_EQ(tr2_schedule_read.get_v_0(), 0);
  EXPECT_EQ(tr2_schedule_read.get_t_n(), 300);
  EXPECT_EQ(tr2_schedule_read.get_v_n(), 20);
  EXPECT_EQ(network.get_vertex(tr2_schedule_read.get_entry()).name, "r0");
  EXPECT_EQ(network.get_vertex(tr2_schedule_read.get_exit()).name, "l0");
  EXPECT_EQ(tr2_schedule_read.get_stops().size(), 1);
  const auto& stop3_read = tr2_schedule_read.get_stops()[0];
  EXPECT_EQ(stop3_read.arrival(), 160);
  EXPECT_EQ(stop3_read.departure(), 220);
  EXPECT_EQ(stations_read.get_station(stop3_read.get_station_name()).name,
            "Station1");
}

TEST(Functionality, TimetableConsistency) {
  auto network = cda_rail::Network::import_network(
      "./example-networks/SimpleStation/network/");
  auto network1 = cda_rail::Network::import_network(
      "./example-networks/SimpleStation/network/");
  network1.add_edge("l0", "r1", 100, 10, false);
  auto network2 = cda_rail::Network::import_network(
      "./example-networks/SimpleStation/network/");
  network2.add_edge("r0", "l1", 100, 10, false);
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

  timetable.add_station("Station1");
  timetable.add_track_to_station("Station1", "g00", "g01", network);

  const auto l0 = network.get_vertex_index("l0");
  const auto r0 = network.get_vertex_index("r0");

  const auto tr1 = timetable.add_train("tr1", 100, 83.33, 2, 1, 0, 0, l0, 300,
                                       20, r0, network);
  timetable.add_stop(tr1, "Station1", 0, 60);

  EXPECT_TRUE(timetable.check_consistency(network));
  EXPECT_FALSE(timetable.check_consistency(network1));
  EXPECT_FALSE(timetable.check_consistency(network2));
  EXPECT_FALSE(timetable.check_consistency(network3));
  EXPECT_FALSE(timetable.check_consistency(network4));
}

TEST(Functionality, TimetableExceptions) {
  auto network = cda_rail::Network::import_network(
      "./example-networks/SimpleStation/network/");
  cda_rail::Timetable timetable;

  const auto l0 = network.get_vertex_index("l0");
  const auto r0 = network.get_vertex_index("r0");

  const auto v_x = 100 * (l0 + r0);

  const auto tr1 = timetable.add_train("tr1", 100, 83.33, 2, 1, 0, 0, l0, 300,
                                       20, r0, network);
  EXPECT_THROW(timetable.add_train("tr1", 100, 83.33, 2, 1, 0, 0, l0, 300, 20,
                                   r0, network),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_THROW(timetable.add_train("tr2", 100, 83.33, 2, 1, 0, 0, v_x, 300, 20,
                                   r0, network),
               cda_rail::exceptions::VertexNotExistentException);
  EXPECT_THROW(timetable.add_train("tr2", 100, 83.33, 2, 1, 0, 0, l0, 300, 20,
                                   v_x, network),
               cda_rail::exceptions::VertexNotExistentException);

  EXPECT_THROW(timetable.get_schedule(10),
               cda_rail::exceptions::TrainNotExistentException);

  timetable.add_station("Station1");
  timetable.add_station("Station2");

  timetable.add_track_to_station("Station1", "g00", "g01", network);
  timetable.add_track_to_station("Station1", "g10", "g11", network);
  timetable.add_track_to_station("Station1", "g01", "g00", network);
  timetable.add_track_to_station("Station1", "g11", "g10", network);
  timetable.add_track_to_station("Station2", "r1", "r0", network);

  EXPECT_THROW(timetable.add_stop(tr1 + 10, "Station1", 0, 60),
               cda_rail::exceptions::TrainNotExistentException);
  EXPECT_THROW(timetable.add_stop(tr1, "Station3", 0, 60),
               cda_rail::exceptions::StationNotExistentException);
  EXPECT_THROW(timetable.add_stop(tr1, "Station1", -1, 60),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(timetable.add_stop(tr1, "Station1", 0, -1),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(timetable.add_stop(tr1, "Station1", 60, 0),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(timetable.add_stop(tr1, "Station1", 60, 60),
               cda_rail::exceptions::InvalidInputException);

  timetable.add_stop(tr1, "Station1", 0, 60);
  EXPECT_THROW(timetable.add_stop(tr1, "Station1", 0, 60),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_THROW(timetable.add_stop(tr1, "Station2", 30, 90),
               cda_rail::exceptions::ConsistencyException);

  EXPECT_THROW(timetable.time_index_interval(tr1 + 10, 15, true),
               cda_rail::exceptions::TrainNotExistentException);
}

TEST(Functionality, RouteMap) {
  auto network = cda_rail::Network::import_network(
      "./example-networks/SimpleStation/network/");
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

  // Check if route consists of three edges passing vertices l0-l1-l2-l3 in this
  // order.
  const auto& route = route_map.get_route("tr1");
  EXPECT_EQ(route.size(), 3);
  EXPECT_EQ(network.get_vertex(route.get_edge(0, network).source).name, "l0");
  EXPECT_EQ(network.get_vertex(route.get_edge(0, network).target).name, "l1");
  EXPECT_EQ(network.get_vertex(route.get_edge(1, network).source).name, "l1");
  EXPECT_EQ(network.get_vertex(route.get_edge(1, network).target).name, "l2");
  EXPECT_EQ(network.get_vertex(route.get_edge(2, network).source).name, "l2");
  EXPECT_EQ(network.get_vertex(route.get_edge(2, network).target).name, "l3");

  EXPECT_EQ(route.length(network), 1005);

  // Check if the consistency checking works as expected
  EXPECT_TRUE(route_map.check_consistency(train_list, network, false));
  EXPECT_FALSE(route_map.check_consistency(train_list, network, true));
  EXPECT_FALSE(route_map.check_consistency(train_list, network));

  route_map.add_empty_route("tr2");
  route_map.push_back_edge("tr2", "r0", "r1", network);
  route_map.push_back_edge("tr2", "r1", "r2", network);

  // Check if route consists of two edges passing vertices r0-r1-r2 in this
  // order.
  const auto& route2 = route_map.get_route("tr2");
  EXPECT_EQ(route2.size(), 2);
  EXPECT_EQ(network.get_vertex(route2.get_edge(0, network).source).name, "r0");
  EXPECT_EQ(network.get_vertex(route2.get_edge(0, network).target).name, "r1");
  EXPECT_EQ(network.get_vertex(route2.get_edge(1, network).source).name, "r1");
  EXPECT_EQ(network.get_vertex(route2.get_edge(1, network).target).name, "r2");

  EXPECT_EQ(route2.length(network), 505);

  // Check route map length
  EXPECT_EQ(route_map.length("tr1", network), 1005);
  EXPECT_EQ(route_map.length("tr2", network), 505);

  // Check if the consistency checking works as expected
  EXPECT_TRUE(route_map.check_consistency(train_list, network, false));
  EXPECT_TRUE(route_map.check_consistency(train_list, network, true));
  EXPECT_TRUE(route_map.check_consistency(train_list, network));
}

TEST(Functionality, ImportRouteMap) {
  auto network = cda_rail::Network::import_network(
      "./example-networks/SimpleStation/network/");
  auto train_list = cda_rail::TrainList::import_trains(
      "./example-networks/SimpleStation/timetable/");
  auto route_map = cda_rail::RouteMap::import_routes(
      "./example-networks/SimpleStation/routes/", network);

  // Check if the route consists of three trains with names "tr1", "tr2" and
  // "tr3"
  EXPECT_EQ(route_map.size(), 3);
  EXPECT_TRUE(route_map.has_route("tr1"));
  EXPECT_TRUE(route_map.has_route("tr2"));
  EXPECT_TRUE(route_map.has_route("tr3"));

  // Check if the route for tr1 consists of eight edges passing vertices
  // l0-l1-l2-l3-g00-g01-r2-r1-r0 in this order.
  const auto& route = route_map.get_route("tr1");
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
  const auto& route2 = route_map.get_route("tr2");
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
  const auto& route3 = route_map.get_route("tr3");
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

  // Check imported consistency
  EXPECT_TRUE(route_map.check_consistency(train_list, network, false));
  EXPECT_TRUE(route_map.check_consistency(train_list, network, true));
  EXPECT_TRUE(route_map.check_consistency(train_list, network));
}

TEST(Functionality, ExportRouteMap) {
  auto network = cda_rail::Network::import_network(
      "./example-networks/SimpleStation/network/");
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
  auto route_map_read =
      cda_rail::RouteMap::import_routes("./tmp/write_route_map_test", network);
  std::filesystem::remove_all("./tmp");

  // Check if the route map is the same as the original one
  // Check if the route map contains two routes for tr1 and tr2
  EXPECT_EQ(route_map_read.size(), 2);
  EXPECT_TRUE(route_map_read.has_route("tr1"));
  EXPECT_TRUE(route_map_read.has_route("tr2"));

  // Check if the route for tr1 consists of three edges passing vertices
  // l0-l1-l2-l3 in this order.
  const auto& route1 = route_map_read.get_route("tr1");
  EXPECT_EQ(route1.size(), 3);
  EXPECT_EQ(network.get_vertex(route1.get_edge(0, network).source).name, "l0");
  EXPECT_EQ(network.get_vertex(route1.get_edge(0, network).target).name, "l1");
  EXPECT_EQ(network.get_vertex(route1.get_edge(1, network).source).name, "l1");
  EXPECT_EQ(network.get_vertex(route1.get_edge(1, network).target).name, "l2");
  EXPECT_EQ(network.get_vertex(route1.get_edge(2, network).source).name, "l2");
  EXPECT_EQ(network.get_vertex(route1.get_edge(2, network).target).name, "l3");

  // Check if the route for tr2 consists of two edges passing vertices r0-r1-r2
  // in this order.
  const auto& route2 = route_map_read.get_route("tr2");
  EXPECT_EQ(route2.size(), 2);
  EXPECT_EQ(network.get_vertex(route2.get_edge(0, network).source).name, "r0");
  EXPECT_EQ(network.get_vertex(route2.get_edge(0, network).target).name, "r1");
  EXPECT_EQ(network.get_vertex(route2.get_edge(1, network).source).name, "r1");
  EXPECT_EQ(network.get_vertex(route2.get_edge(1, network).target).name, "r2");

  // Check imported consistency
  EXPECT_TRUE(route_map_read.check_consistency(train_list, network, false));
  EXPECT_TRUE(route_map_read.check_consistency(train_list, network, true));
  EXPECT_TRUE(route_map_read.check_consistency(train_list, network));
}

TEST(Functionality, RouteMapHelper) {
  cda_rail::Network network;
  network.add_vertex("v0", cda_rail::VertexType::TTD);
  const auto v1 = network.add_vertex("v1", cda_rail::VertexType::TTD);
  const auto v2 = network.add_vertex("v2", cda_rail::VertexType::TTD);
  network.add_vertex("v3", cda_rail::VertexType::TTD);

  network.add_edge("v0", "v1", 10, 5, false);
  const auto v1_v2 = network.add_edge("v1", "v2", 20, 5, false);
  const auto v2_v3 = network.add_edge("v2", "v3", 30, 5, false);
  const auto v3_v2 = network.add_edge("v3", "v2", 30, 5, false);
  const auto v2_v1 = network.add_edge("v2", "v1", 20, 5, false);
  network.add_edge("v1", "v0", 10, 5, false);

  network.add_successor({"v0", "v1"}, {"v1", "v2"});
  network.add_successor({"v1", "v2"}, {"v2", "v3"});

  cda_rail::RouteMap route_map;

  EXPECT_THROW(route_map.remove_first_edge("tr1"),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_THROW(route_map.remove_last_edge("tr1"),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_THROW(route_map.get_route("tr1"),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_THROW(route_map.push_back_edge("tr1", v1_v2, network),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_THROW(route_map.push_back_edge("tr1", v1, v2, network),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_THROW(route_map.push_back_edge("tr1", "v1", "v2", network),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_THROW(route_map.push_front_edge("tr1", v1_v2, network),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_THROW(route_map.push_front_edge("tr1", v1, v2, network),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_THROW(route_map.push_front_edge("tr1", "v1", "v2", network),
               cda_rail::exceptions::ConsistencyException);

  route_map.add_empty_route("tr1");
  route_map.push_back_edge("tr1", "v0", "v1", network);
  route_map.push_back_edge("tr1", "v1", "v2", network);
  route_map.push_back_edge("tr1", "v2", "v3", network);

  EXPECT_THROW(route_map.add_empty_route("tr1"),
               cda_rail::exceptions::InvalidInputException);

  const auto& tr1_map    = route_map.get_route("tr1");
  const auto  tr1_e1_pos = tr1_map.edge_pos("v0", "v1", network);
  const std::pair<double, double> expected_tr1_e1_pos = {0, 10};
  EXPECT_EQ(tr1_e1_pos, expected_tr1_e1_pos);
  const auto tr1_e2_pos = tr1_map.edge_pos(v1, v2, network);
  const std::pair<double, double> expected_tr1_e2_pos = {10, 30};
  EXPECT_EQ(tr1_e2_pos, expected_tr1_e2_pos);
  const auto                      tr1_e3_pos = tr1_map.edge_pos(v2_v3, network);
  const std::pair<double, double> expected_tr1_e3_pos = {30, 60};
  EXPECT_EQ(tr1_e3_pos, expected_tr1_e3_pos);

  const auto station_pos =
      tr1_map.edge_pos({v1_v2, v2_v1, v2_v3, v3_v2}, network);
  const std::pair<double, double> expected_station_pos = {10, 60};
  EXPECT_EQ(station_pos, expected_station_pos);

  EXPECT_EQ(tr1_map.length(network), 60);

  EXPECT_THROW(route_map.remove_route("nonexistingtrain"),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_TRUE(route_map.has_route("tr1"));
  route_map.remove_route("tr1");
  EXPECT_FALSE(route_map.has_route("tr1"));
}

TEST(Functionality, Iterators) {
  // Create a train list
  auto trains = cda_rail::TrainList();
  trains.add_train("tr1", 100, 83.33, 2, 1);
  trains.add_train("tr2", 100, 27.78, 2, 1);
  trains.add_train("tr3", 250, 20, 2, 1);

  // Check range based for loop
  int i = 0;
  for (const auto& train : trains) {
    EXPECT_EQ(&train, &trains.get_train(i));
    i++;
  }

  // Create route map
  auto route_map = cda_rail::RouteMap();

  route_map.add_empty_route("tr1");
  route_map.add_empty_route("tr2");

  // Check range based for loop
  for (const auto& [name, route] : route_map) {
    EXPECT_EQ(&route, &route_map.get_route(name));
  }

  // Create stations
  cda_rail::StationList stations;
  stations.add_station("S1");
  stations.add_station("S2");

  // Check range based for loop
  for (const auto& [name, station] : stations) {
    EXPECT_EQ(&station, &stations.get_station(name));
  }

  // Create timetable
  auto network = cda_rail::Network::import_network(
      "./example-networks/SimpleStation/network/");
  cda_rail::Timetable timetable;

  timetable.add_train("tr1", 100, 83.33, 2, 1, 0, 0, "l0", 300, 20, "r0",
                      network);
  timetable.add_train("tr2", 100, 27.78, 2, 1, 0, 0, "r0", 300, 20, "l0",
                      network);

  // Check range based for loop
  // for (const auto& [name, schedule] : timetable) {
  //    EXPECT_EQ(&schedule, &timetable.get_schedule(name));
  //}
}

TEST(Functionality, IsFullyInStation) {
  cda_rail::StationList stations;

  stations.add_station("Station1");

  stations.add_track_to_station("Station1", 0);
  stations.add_track_to_station("Station1", 1);
  stations.add_track_to_station("Station1", 3);

  EXPECT_TRUE(stations.is_fully_in_station("Station1", {0}));
  EXPECT_TRUE(stations.is_fully_in_station("Station1", {1}));
  EXPECT_TRUE(stations.is_fully_in_station("Station1", {0, 1}));
  EXPECT_TRUE(stations.is_fully_in_station("Station1", {0, 1, 3}));
  EXPECT_FALSE(stations.is_fully_in_station("Station1", {0, 2}));
}

TEST(Functionality, NetworkRouteLen) {
  cda_rail::Network network;

  network.add_vertex("v0", cda_rail::VertexType::TTD);
  network.add_vertex("v1", cda_rail::VertexType::TTD);
  network.add_vertex("v2", cda_rail::VertexType::TTD);
  network.add_vertex("v3", cda_rail::VertexType::TTD);

  const auto e1 = network.add_edge("v1", "v2", 20, 5, false);
  const auto e2 = network.add_edge("v2", "v3", 30, 5, false);
  const auto e0 = network.add_edge("v0", "v1", 10, 5, false);

  EXPECT_EQ(network.length_of_path({e0, e1, e2}), 60);
  EXPECT_EQ(network.length_of_path({e1, e2}), 50);
  EXPECT_EQ(network.length_of_path({e0, e1}), 30);
  EXPECT_EQ(network.length_of_path({e2}), 30);
  EXPECT_EQ(network.length_of_path({e1}), 20);
  EXPECT_EQ(network.length_of_path({e0}), 10);
  EXPECT_EQ(network.length_of_path({}), 0);
  EXPECT_THROW(network.length_of_path({e0, e1, 1000, e2}),
               cda_rail::exceptions::EdgeNotExistentException);
}

TEST(Functionality, NetworkNextTTD) {
  cda_rail::Network network;
  const auto        v0 = network.add_vertex("v0", cda_rail::VertexType::TTD);
  const auto        v1 = network.add_vertex("v1", cda_rail::VertexType::TTD);
  const auto v2  = network.add_vertex("v2", cda_rail::VertexType::NoBorder);
  const auto v3a = network.add_vertex("v3a", cda_rail::VertexType::TTD);
  const auto v3b = network.add_vertex("v3b", cda_rail::VertexType::TTD);
  const auto v4a = network.add_vertex("v4a", cda_rail::VertexType::TTD);
  const auto v4b = network.add_vertex("v4b", cda_rail::VertexType::TTD);
  const auto v5  = network.add_vertex("v5", cda_rail::VertexType::NoBorder);
  const auto v6  = network.add_vertex("v6", cda_rail::VertexType::TTD);
  const auto v7  = network.add_vertex("v7", cda_rail::VertexType::NoBorder);
  const auto v8a = network.add_vertex("v8a", cda_rail::VertexType::TTD);
  const auto v8b = network.add_vertex("v8b", cda_rail::VertexType::TTD);
  const auto v9a = network.add_vertex("v9a", cda_rail::VertexType::TTD);
  const auto v9b = network.add_vertex("v9b", cda_rail::VertexType::TTD);

  const auto v0_v1   = network.add_edge(v0, v1, 100, 20, true);
  const auto v1_v2   = network.add_edge(v1, v2, 10, 20, false);
  const auto v2_v3a  = network.add_edge(v2, v3a, 10, 20, false);
  const auto v2_v3b  = network.add_edge(v2, v3b, 10, 20, false);
  const auto v3a_v4a = network.add_edge(v3a, v4a, 100, 20, true);
  const auto v3b_v4b = network.add_edge(v3b, v4b, 100, 20, true);
  const auto v4a_v5  = network.add_edge(v4a, v5, 10, 20, false);
  const auto v4b_v5  = network.add_edge(v4b, v5, 10, 20, false);
  const auto v5_v6   = network.add_edge(v5, v6, 10, 20, false);
  const auto v6_v7   = network.add_edge(v6, v7, 10, 20, false);
  const auto v7_v8a  = network.add_edge(v7, v8a, 10, 20, false);
  const auto v7_v8b  = network.add_edge(v7, v8b, 10, 20, false);
  const auto v8a_v9a = network.add_edge(v8a, v9a, 100, 20, true);
  const auto v8b_v9b = network.add_edge(v8b, v9b, 100, 20, true);

  network.add_successor(v0_v1, v1_v2);
  network.add_successor(v1_v2, v2_v3a);
  network.add_successor(v1_v2, v2_v3b);
  network.add_successor(v2_v3a, v3a_v4a);
  network.add_successor(v2_v3b, v3b_v4b);
  network.add_successor(v3a_v4a, v4a_v5);
  network.add_successor(v3b_v4b, v4b_v5);
  network.add_successor(v4a_v5, v5_v6);
  network.add_successor(v4b_v5, v5_v6);
  network.add_successor(v5_v6, v6_v7);
  network.add_successor(v6_v7, v7_v8a);
  network.add_successor(v6_v7, v7_v8b);
  network.add_successor(v7_v8a, v8a_v9a);
  network.add_successor(v7_v8b, v8b_v9b);

  const auto& ttd_sections = network.unbreakable_sections();
  EXPECT_EQ(ttd_sections.size(), 3);

  const auto routing1 =
      network.all_paths_ending_at_ttd(v0_v1, ttd_sections, v9a);
  // (v1_v2, v2_v3a, v3a_v3b) and (v1_v2, v2_v3b, v3b_c4b) are two different
  // paths
  EXPECT_EQ(routing1.size(), 2);
  EXPECT_TRUE(std::find(routing1.begin(), routing1.end(),
                        std::vector<size_t>{v1_v2, v2_v3a, v3a_v4a}) !=
              routing1.end());
  EXPECT_TRUE(std::find(routing1.begin(), routing1.end(),
                        std::vector<size_t>{v1_v2, v2_v3b, v3b_v4b}) !=
              routing1.end());

  const auto routing2 =
      network.all_paths_ending_at_ttd(v2_v3a, ttd_sections, v9a);
  // Expect only (v3a_v4a) as the only path
  EXPECT_EQ(routing2.size(), 1);
  EXPECT_EQ(routing2.at(0), std::vector<size_t>({v3a_v4a}));

  const auto routing3 =
      network.all_paths_ending_at_ttd(v3a_v4a, ttd_sections, v9b);
  // Expect only (v4a_v5, v5_v6)
  EXPECT_EQ(routing3.size(), 1);
  EXPECT_EQ(routing3.at(0), std::vector<size_t>({v4a_v5, v5_v6}));

  const auto routing4 =
      network.all_paths_ending_at_ttd(v5_v6, ttd_sections, v9b);
  // Expect only (v6_v7, v7_v8b, v8b_v9b)
  EXPECT_EQ(routing4.size(), 1);
  EXPECT_EQ(routing4.at(0), std::vector<size_t>({v6_v7, v7_v8b, v8b_v9b}));
}

TEST(NetworkFunctionality, TrackIndex) {
  cda_rail::Network network;
  const auto v0 = network.add_vertex("v0", cda_rail::VertexType::NoBorder);
  const auto v1 = network.add_vertex("v1", cda_rail::VertexType::VSS);
  const auto v2 = network.add_vertex("v2", cda_rail::VertexType::TTD);

  const auto e0 = network.add_edge("v0", "v1", 1, 2, false, 0);
  const auto e1 = network.add_edge("v1", "v2", 3, 4, true, 1.5);
  const auto e2 = network.add_edge("v1", "v0", 1, 2, false, 0);

  EXPECT_EQ(network.get_track_index(e1), e1);
  EXPECT_EQ(network.get_track_index(e0), std::min(e0, e2));
  EXPECT_EQ(network.get_track_index(e2), std::min(e0, e2));
}

TEST(RouteFunctionality, FirstPosOnEdge) {
  cda_rail::Network network;
  const auto v0 = network.add_vertex("v0", cda_rail::VertexType::NoBorder);
  const auto v1 = network.add_vertex("v1", cda_rail::VertexType::VSS);
  const auto v2 = network.add_vertex("v2", cda_rail::VertexType::TTD);

  const auto e0 = network.add_edge("v0", "v1", 1, 2, false, 0);
  const auto e1 = network.add_edge("v1", "v2", 3, 4, true, 1.5);
  const auto e2 = network.add_edge("v1", "v0", 1, 2, false, 0);

  network.add_successor(e0, e1);

  cda_rail::Route route;
  route.push_back_edge(e0, network);
  route.push_back_edge(e1, network);

  const auto p1 = route.get_first_pos_on_edges({e0, e1, e2}, network);
  const auto p2 = route.get_first_pos_on_edges({e1}, network);
  const auto p3 = route.get_first_pos_on_edges({e2}, network);

  EXPECT_TRUE(p1.has_value());
  EXPECT_TRUE(p2.has_value());
  EXPECT_FALSE(p3.has_value());
  EXPECT_EQ(p1.value_or(-1), 0);
  EXPECT_EQ(p2.value_or(-1), 1);
}

TEST(RouteFunctionality, LastPosOnEdge) {
  cda_rail::Network network;
  const auto v0 = network.add_vertex("v0", cda_rail::VertexType::NoBorder);
  const auto v1 = network.add_vertex("v1", cda_rail::VertexType::VSS);
  const auto v2 = network.add_vertex("v2", cda_rail::VertexType::TTD);

  const auto e0 = network.add_edge("v0", "v1", 1, 2, false, 0);
  const auto e1 = network.add_edge("v1", "v2", 3, 4, true, 1.5);
  const auto e2 = network.add_edge("v1", "v0", 1, 2, false, 0);

  network.add_successor(e0, e1);

  cda_rail::Route route;
  route.push_back_edge(e0, network);
  route.push_back_edge(e1, network);

  const auto p1 = route.get_last_pos_on_edges({e0, e1, e2}, network);
  const auto p2 = route.get_last_pos_on_edges({e1}, network);
  const auto p3 = route.get_last_pos_on_edges({e2}, network);
  const auto p4 = route.get_last_pos_on_edges({e0}, network);

  EXPECT_TRUE(p1.has_value());
  EXPECT_TRUE(p2.has_value());
  EXPECT_FALSE(p3.has_value());
  EXPECT_TRUE(p4.has_value());
  EXPECT_EQ(p1.value_or(-1), 4);
  EXPECT_EQ(p2.value_or(-1), 4);
  EXPECT_EQ(p4.value_or(-1), 1);
}

TEST(RouteMapFunctionality, EmptyConflicts) {
  auto network = cda_rail::Network::import_network(
      "./example-networks/SimpleStation/network/");
  auto train_list = cda_rail::TrainList();

  train_list.add_train("tr1", 100, 83.33, 2, 1);
  train_list.add_train("tr2", 100, 27.78, 2, 1);

  auto route_map = cda_rail::RouteMap();

  EXPECT_ANY_THROW(route_map.add_empty_route("tr3", train_list));

  route_map.add_empty_route("tr1", train_list);
  route_map.add_empty_route("tr2", train_list);
  EXPECT_TRUE(route_map.get_reverse_overlaps("tr1", "tr2", network).empty());
  EXPECT_TRUE(route_map.get_crossing_overlaps("tr1", "tr2", network).empty());
  EXPECT_TRUE(route_map.get_parallel_overlaps("tr1", "tr2", network).empty());
  EXPECT_TRUE(route_map.get_ttd_overlaps("tr1", "tr2", network).empty());
}

// NOLINTEND(clang-diagnostic-unused-result,clang-analyzer-deadcode.DeadStores,bugprone-unchecked-optional-access)
