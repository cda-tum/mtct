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

namespace {

struct RouteTestNetwork {
  cda_rail::Network network;
  size_t            e0{};
  size_t            e1{};
  size_t            e2{};
  size_t            e_alt{};
};

RouteTestNetwork make_route_test_network() {
  cda_rail::Network network;
  network.add_vertex("v0", cda_rail::VertexType::NoBorder);
  network.add_vertex("v1", cda_rail::VertexType::VSS);
  network.add_vertex("v2", cda_rail::VertexType::TTD);
  network.add_vertex("v3", cda_rail::VertexType::NoBorder);

  const auto e0    = network.add_edge({"v0"}, {"v1"}, 1.0, 10.0, true, 1.0);
  const auto e1    = network.add_edge({"v1"}, {"v2"}, 2.0, 10.0, true, 1.0);
  const auto e2    = network.add_edge({"v2"}, {"v3"}, 3.0, 10.0, true, 1.0);
  const auto e_alt = network.add_edge({"v0"}, {"v2"}, 4.0, 10.0, true, 1.0);

  network.add_successor(e0, e1);
  network.add_successor(e1, e2);

  return RouteTestNetwork{std::move(network), e0, e1, e2, e_alt};
}

} // namespace

TEST(RouteFunctionality, EmptyRouteHasSizeZeroAndNoEdges) {
  cda_rail::Route route;

  EXPECT_TRUE(route.empty());
  EXPECT_EQ(route.size(), 0);
  EXPECT_TRUE(route.get_edges().empty());
}

TEST(RouteFunctionality, PushBackEdgeAddsEdgeAndContainsById) {
  auto [network, e0, e1, e2, e_alt] = make_route_test_network();
  cda_rail::Route route;

  route.push_back_edge(e0, network);

  EXPECT_FALSE(route.empty());
  EXPECT_EQ(route.size(), 1);
  EXPECT_EQ(route.get_edge_id(0), e0);
  EXPECT_TRUE(route.contains_edge(e0));
}

TEST(RouteFunctionality, ContainsEdgeSupportsInputAndOptionalOverloads) {
  auto [network, e0, e1, e2, e_alt] = make_route_test_network();
  cda_rail::Route route;
  route.push_back_edge(e0, network);

  EXPECT_TRUE(route.contains_edge({"v0", "v1"}, network));
  EXPECT_TRUE(route.contains_edge(std::optional<size_t>{e0}));
  EXPECT_FALSE(route.contains_edge(std::optional<size_t>{e1}));
  EXPECT_FALSE(route.contains_edge(std::optional<size_t>{}));
}

TEST(RouteFunctionality, LengthIsSumOfRouteEdgeLengths) {
  auto [network, e0, e1, e2, e_alt] = make_route_test_network();
  cda_rail::Route route;
  route.push_back_edge(e0, network);
  route.push_back_edge(e1, network);
  route.push_back_edge(e2, network);

  EXPECT_DOUBLE_EQ(route.length(network), 6.0);
}

TEST(RouteFunctionality, PushFrontEdgePrependsWhenSuccessorRelationIsValid) {
  auto [network, e0, e1, e2, e_alt] = make_route_test_network();
  cda_rail::Route route;
  route.push_back_edge(e1, network);

  route.push_front_edge(e0, network);

  EXPECT_EQ(route.get_edges(), (cda_rail::index_vector{e0, e1}));
}

TEST(RouteFunctionality, PushBackEdgeRejectsNonSuccessor) {
  auto [network, e0, e1, e2, e_alt] = make_route_test_network();
  cda_rail::Route route;
  route.push_back_edge(e0, network);

  EXPECT_THROW(route.push_back_edge(e_alt, network),
               cda_rail::exceptions::ConsistencyException);
}

TEST(RouteFunctionality, PushFrontEdgeRejectsNonPredecessor) {
  auto [network, e0, e1, e2, e_alt] = make_route_test_network();
  cda_rail::Route route;
  route.push_back_edge(e1, network);

  EXPECT_THROW(route.push_front_edge(e_alt, network),
               cda_rail::exceptions::ConsistencyException);
}

TEST(RouteFunctionality, RemoveFirstEdgeRemovesLeadingEdge) {
  auto [network, e0, e1, e2, e_alt] = make_route_test_network();
  cda_rail::Route route;
  route.push_back_edge(e0, network);
  route.push_back_edge(e1, network);

  route.remove_first_edge();

  EXPECT_EQ(route.get_edges(), (cda_rail::index_vector{e1}));
}

TEST(RouteFunctionality, RemoveLastEdgeRemovesTrailingEdge) {
  auto [network, e0, e1, e2, e_alt] = make_route_test_network();
  cda_rail::Route route;
  route.push_back_edge(e0, network);
  route.push_back_edge(e1, network);

  route.remove_last_edge();

  EXPECT_EQ(route.get_edges(), (cda_rail::index_vector{e0}));
}

TEST(RouteFunctionality, RemoveFirstEdgeThrowsOnEmptyRoute) {
  cda_rail::Route route;
  EXPECT_THROW(route.remove_first_edge(),
               cda_rail::exceptions::ConsistencyException);
}

TEST(RouteFunctionality, RemoveLastEdgeThrowsOnEmptyRoute) {
  cda_rail::Route route;
  EXPECT_THROW(route.remove_last_edge(),
               cda_rail::exceptions::ConsistencyException);
}

TEST(RouteFunctionality, EdgePosOnRouteReturnsStartAndEndPositions) {
  auto [network, e0, e1, e2, e_alt] = make_route_test_network();
  cda_rail::Route route;
  route.push_back_edge(e0, network);
  route.push_back_edge(e1, network);
  route.push_back_edge(e2, network);

  const auto [start_pos, end_pos] = route.edge_pos_on_route(e1, network);

  EXPECT_DOUBLE_EQ(start_pos, 1.0);
  EXPECT_DOUBLE_EQ(end_pos, 3.0);
}

TEST(RouteFunctionality, EdgePosOnRouteThrowsWhenEdgeIsNotOnRoute) {
  auto [network, e0, e1, e2, e_alt] = make_route_test_network();
  cda_rail::Route route;
  route.push_back_edge(e0, network);
  route.push_back_edge(e1, network);

  EXPECT_THROW(
      { static_cast<void>(route.edge_pos_on_route(e2, network)); },
      cda_rail::exceptions::ConsistencyException);
}

TEST(RouteFunctionality, EdgeSetPosOnRouteReturnsCoveredInterval) {
  auto [network, e0, e1, e2, e_alt] = make_route_test_network();
  cda_rail::Route route;
  route.push_back_edge(e0, network);
  route.push_back_edge(e1, network);
  route.push_back_edge(e2, network);

  const auto [start_pos, end_pos] =
      route.edge_set_pos_on_route({e2, e0}, network);

  EXPECT_DOUBLE_EQ(start_pos, 0.0);
  EXPECT_DOUBLE_EQ(end_pos, 6.0);
}

TEST(RouteFunctionality, EdgeSetPosOnRouteThrowsWhenNoGivenEdgeIsOnRoute) {
  auto [network, e0, e1, e2, e_alt] = make_route_test_network();
  cda_rail::Route route;
  route.push_back_edge(e0, network);

  EXPECT_THROW(
      { static_cast<void>(route.edge_set_pos_on_route({e1, e2}, network)); },
      cda_rail::exceptions::ConsistencyException);
}

TEST(RouteFunctionality, GetFirstPosOnEdgesReturnsStartOfFirstMatchingEdge) {
  auto [network, e0, e1, e2, e_alt] = make_route_test_network();
  cda_rail::Route route;
  route.push_back_edge(e0, network);
  route.push_back_edge(e1, network);

  const auto pos = route.get_first_pos_on_edges({e1}, network);

  ASSERT_TRUE(pos.has_value());
  EXPECT_DOUBLE_EQ(pos.value(), 1.0);
}

TEST(RouteFunctionality, GetLastPosOnEdgesReturnsEndOfLastMatchingEdge) {
  auto [network, e0, e1, e2, e_alt] = make_route_test_network();
  cda_rail::Route route;
  route.push_back_edge(e0, network);
  route.push_back_edge(e1, network);
  route.push_back_edge(e2, network);

  const auto pos = route.get_last_pos_on_edges({e0, e2}, network);

  ASSERT_TRUE(pos.has_value());
  EXPECT_DOUBLE_EQ(pos.value(), 6.0);
}

TEST(RouteFunctionality, GetFirstAndLastPosOnEdgesReturnNoValueForEmptyRoute) {
  auto [network, e0, e1, e2, e_alt] = make_route_test_network();
  cda_rail::Route route;

  EXPECT_FALSE(route.get_first_pos_on_edges({e0}, network).has_value());
  EXPECT_FALSE(route.get_last_pos_on_edges({e0}, network).has_value());
}

TEST(RouteFunctionality, GetEdgeIdAtPosReturnsEdgeContainingPosition) {
  auto [network, e0, e1, e2, e_alt] = make_route_test_network();
  cda_rail::Route route;
  route.push_back_edge(e0, network);
  route.push_back_edge(e1, network);
  route.push_back_edge(e2, network);

  EXPECT_EQ(route.get_edge_id_at_pos(0.0, network), e0);
  EXPECT_EQ(route.get_edge_id_at_pos(1.5, network), e1);
  EXPECT_EQ(route.get_edge_id_at_pos(6.0, network), e2);
}

TEST(RouteFunctionality, GetEdgeIdAtPosThrowsForInvalidPosition) {
  auto [network, e0, e1, e2, e_alt] = make_route_test_network();
  cda_rail::Route route;
  route.push_back_edge(e0, network);
  route.push_back_edge(e1, network);

  EXPECT_THROW(
      { static_cast<void>(route.get_edge_id_at_pos(-0.1, network)); },
      cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(
      { static_cast<void>(route.get_edge_id_at_pos(10.0, network)); },
      cda_rail::exceptions::ConsistencyException);
}

TEST(RouteFunctionality, GetEdgeIdReturnsEdgeAtGivenRouteIndex) {
  auto [network, e0, e1, e2, e_alt] = make_route_test_network();
  cda_rail::Route route;
  route.push_back_edge(e0, network);
  route.push_back_edge(e1, network);

  EXPECT_EQ(route.get_edge_id(1), e1);
}

TEST(RouteFunctionality, GetEdgeIdThrowsForOutOfRangeIndex) {
  auto [network, e0, e1, e2, e_alt] = make_route_test_network();
  cda_rail::Route route;
  route.push_back_edge(e0, network);

  EXPECT_THROW(
      { static_cast<void>(route.get_edge_id(1)); },
      cda_rail::exceptions::InvalidInputException);
}

TEST(RouteFunctionality, GetEdgeReturnsNetworkEdgeAtGivenRouteIndex) {
  auto [network, e0, e1, e2, e_alt] = make_route_test_network();
  cda_rail::Route route;
  route.push_back_edge(e0, network);

  const auto& edge = route.get_edge(0, network);

  EXPECT_EQ(edge.source, network.get_edge(e0).source);
  EXPECT_EQ(edge.target, network.get_edge(e0).target);
  EXPECT_DOUBLE_EQ(edge.length, network.get_edge(e0).length);
}

TEST(RouteFunctionality, GetEdgeThrowsForOutOfRangeIndex) {
  auto [network, e0, e1, e2, e_alt] = make_route_test_network();
  cda_rail::Route route;
  route.push_back_edge(e0, network);

  EXPECT_THROW(
      { static_cast<void>(route.get_edge(1, network)); },
      cda_rail::exceptions::InvalidInputException);
}

TEST(RouteFunctionality, CheckConsistencyReturnsTrueForValidRoute) {
  auto [network, e0, e1, e2, e_alt] = make_route_test_network();
  cda_rail::Route route;
  route.push_back_edge(e0, network);
  route.push_back_edge(e1, network);

  EXPECT_TRUE(route.check_consistency(network));
}

TEST(RouteFunctionality, CheckConsistencyReturnsFalseForDisconnectedRoute) {
  auto [network, e0, e1, e2, e_alt] = make_route_test_network();
  cda_rail::Route route;
  route.push_back_edge(e0, network);
  route.update_after_discretization({{e0, {e0, e_alt}}});

  EXPECT_FALSE(route.check_consistency(network));
}

TEST(RouteFunctionality, UpdateAfterDiscretizationReplacesMappedEdgesInOrder) {
  auto [network, e0, e1, e2, e_alt] = make_route_test_network();
  cda_rail::Route route;
  route.push_back_edge(e0, network);
  route.push_back_edge(e1, network);

  route.update_after_discretization({{e0, {e0, e_alt}}});

  EXPECT_EQ(route.get_edges(), (cda_rail::index_vector{e0, e_alt, e1}));
}
