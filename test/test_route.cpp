#include "datastructure/Route.hpp"

#include "gmock/gmock-spec-builders.h"
#include "gtest/gtest.h"

TEST(RouteFunctionality, FirstPosOnEdge) {
  cda_rail::Network network;
  const auto v0 = network.add_vertex("v0", cda_rail::VertexType::NoBorder);
  const auto v1 = network.add_vertex("v1", cda_rail::VertexType::VSS);
  const auto v2 = network.add_vertex("v2", cda_rail::VertexType::TTD);

  const auto e0 = network.add_edge({"v0"}, {"v1"}, 1, 2, false, 0);
  const auto e1 = network.add_edge({"v1"}, {"v2"}, 3, 4, true, 1.5);
  const auto e2 = network.add_edge({"v1"}, {"v0"}, 1, 2, false, 0);

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

  const auto e0 = network.add_edge({"v0"}, {"v1"}, 1, 2, false, 0);
  const auto e1 = network.add_edge({"v1"}, {"v2"}, 3, 4, true, 1.5);
  const auto e2 = network.add_edge({"v1"}, {"v0"}, 1, 2, false, 0);

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
