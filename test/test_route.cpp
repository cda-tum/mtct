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

// Route Map

TEST(RouteMapFunctionality, RouteMap) {
  auto network = cda_rail::Network::import_network("SimpleStation", "./data/");
  auto train_list = cda_rail::TrainList();

  train_list.add_train("tr1", 100, 83.33, 2, 1);
  train_list.add_train("tr2", 100, 27.78, 2, 1);

  auto route_map = cda_rail::RouteMap();

  EXPECT_ANY_THROW(route_map.add_empty_route("tr3", train_list));

  route_map.add_empty_route("tr1", train_list);
  route_map.push_back_edge("tr1", {"l1", "l2"}, network);
  EXPECT_ANY_THROW(route_map.push_back_edge("tr1", {"l0", "l2"}, network));
  EXPECT_ANY_THROW(route_map.push_back_edge("tr1", {"l0", "l1"}, network));
  route_map.push_back_edge("tr1", {"l2", "l3"}, network);
  EXPECT_ANY_THROW(route_map.push_front_edge("tr1", {"l0", "l2"}, network));
  EXPECT_ANY_THROW(route_map.push_front_edge("tr1", {"l3", "g00"}, network));
  route_map.push_front_edge("tr1", {"l0", "l1"}, network);

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
  route_map.push_back_edge("tr2", {"r0", "r1"}, network);
  route_map.push_back_edge("tr2", {"r1", "r2"}, network);

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
  EXPECT_EQ(route_map.route_length("tr1", network), 1005);
  EXPECT_EQ(route_map.route_length("tr2", network), 505);

  // Check if the consistency checking works as expected
  EXPECT_TRUE(route_map.check_consistency(train_list, network, false));
  EXPECT_TRUE(route_map.check_consistency(train_list, network, true));
  EXPECT_TRUE(route_map.check_consistency(train_list, network));
}

TEST(RouteMapFunctionality, ImportRouteMap) {
  auto network = cda_rail::Network::import_network("SimpleStation", "./data/");
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

TEST(RouteMapFunctionality, ExportRouteMap) {
  auto network = cda_rail::Network::import_network("SimpleStation", "./data/");
  auto train_list = cda_rail::TrainList();
  train_list.add_train("tr1", 100, 83.33, 2, 1);
  train_list.add_train("tr2", 100, 27.78, 2, 1);
  auto route_map = cda_rail::RouteMap();
  route_map.add_empty_route("tr1", train_list);
  route_map.push_back_edge("tr1", {"l1", "l2"}, network);
  route_map.push_back_edge("tr1", {"l2", "l3"}, network);
  route_map.push_front_edge("tr1", {"l0", "l1"}, network);
  route_map.add_empty_route("tr2");
  route_map.push_back_edge("tr2", {"r0", "r1"}, network);
  route_map.push_back_edge("tr2", {"r1", "r2"}, network);

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

TEST(RouteMapFunctionality, RouteMapHelper) {
  cda_rail::Network network;
  network.add_vertex("v0", cda_rail::VertexType::TTD);
  const auto v1 = network.add_vertex("v1", cda_rail::VertexType::TTD);
  const auto v2 = network.add_vertex("v2", cda_rail::VertexType::TTD);
  network.add_vertex("v3", cda_rail::VertexType::TTD);

  network.add_edge({"v0"}, {"v1"}, 10, 5, false);
  const auto v1_v2 = network.add_edge({"v1"}, {"v2"}, 20, 5, false);
  const auto v2_v3 = network.add_edge({"v2"}, {"v3"}, 30, 5, false);
  const auto v3_v2 = network.add_edge({"v3"}, {"v2"}, 30, 5, false);
  const auto v2_v1 = network.add_edge({"v2"}, {"v1"}, 20, 5, false);
  network.add_edge({"v1"}, {"v0"}, 10, 5, false);

  network.add_successor({"v0", "v1"}, {"v1", "v2"});
  network.add_successor({"v1", "v2"}, {"v2", "v3"});

  cda_rail::RouteMap route_map;

  EXPECT_THROW((void)route_map.remove_first_edge("tr1"),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_THROW((void)route_map.remove_last_edge("tr1"),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_THROW((void)route_map.get_route("tr1"),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_THROW((void)route_map.push_back_edge("tr1", v1_v2, network),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_THROW((void)route_map.push_back_edge("tr1", {v1, v2}, network),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_THROW((void)route_map.push_back_edge("tr1", {"v1", "v2"}, network),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_THROW((void)route_map.push_front_edge("tr1", v1_v2, network),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_THROW((void)route_map.push_front_edge("tr1", {v1, v2}, network),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_THROW((void)route_map.push_front_edge("tr1", {"v1", "v2"}, network),
               cda_rail::exceptions::ConsistencyException);

  route_map.add_empty_route("tr1");
  route_map.push_back_edge("tr1", {"v0", "v1"}, network);
  route_map.push_back_edge("tr1", {"v1", "v2"}, network);
  route_map.push_back_edge("tr1", {"v2", "v3"}, network);

  EXPECT_THROW((void)route_map.add_empty_route("tr1"),
               cda_rail::exceptions::InvalidInputException);

  const auto& tr1_map    = route_map.get_route("tr1");
  const auto  tr1_e1_pos = tr1_map.edge_pos_on_route({"v0", "v1"}, network);
  const std::pair<double, double> expected_tr1_e1_pos = {0, 10};
  EXPECT_EQ(tr1_e1_pos.source, expected_tr1_e1_pos.first);
  EXPECT_EQ(tr1_e1_pos.target, expected_tr1_e1_pos.second);
  const auto tr1_e2_pos = tr1_map.edge_pos_on_route({v1, v2}, network);
  const std::pair<double, double> expected_tr1_e2_pos = {10, 30};
  EXPECT_EQ(tr1_e2_pos.source, expected_tr1_e2_pos.first);
  EXPECT_EQ(tr1_e2_pos.target, expected_tr1_e2_pos.second);
  const auto tr1_e3_pos = tr1_map.edge_pos_on_route(v2_v3, network);
  const std::pair<double, double> expected_tr1_e3_pos = {30, 60};
  EXPECT_EQ(tr1_e3_pos.source, expected_tr1_e3_pos.first);
  EXPECT_EQ(tr1_e3_pos.target, expected_tr1_e3_pos.second);

  const auto station_pos =
      tr1_map.edge_set_pos_on_route({v1_v2, v2_v1, v2_v3, v3_v2}, network);
  const std::pair<double, double> expected_station_pos = {10, 60};
  EXPECT_EQ(station_pos.source, expected_station_pos.first);
  EXPECT_EQ(station_pos.target, expected_station_pos.second);

  EXPECT_EQ(tr1_map.length(network), 60);

  EXPECT_THROW((void)route_map.remove_route("nonexistingtrain"),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_TRUE(route_map.has_route("tr1"));
  route_map.remove_route("tr1");
  EXPECT_FALSE(route_map.has_route("tr1"));
}

TEST(RouteMapFunctionality, EmptyConflicts) {
  auto network = cda_rail::Network::import_network("SimpleStation", "./data/");
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
