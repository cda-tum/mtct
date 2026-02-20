#include "datastructure/Route.hpp"

#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Train.hpp"
#include "nlohmann/json.hpp"
#include "nlohmann/json_fwd.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <numeric>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

using json = nlohmann::json;
using std::size_t;

void cda_rail::Route::push_back_edge(size_t         edge_index,
                                     const Network& network) {
  /**
   * Adds the edge to the end of the route.
   * Throws an error if the edge does not exist in the network or is not a valid
   * successor of the last edge.
   *
   * @param edge_index The index of the edge to add.
   * @param network The network to which the edge belongs.
   */

  if (!network.has_edge(edge_index)) {
    throw exceptions::EdgeNotExistentException(edge_index);
  }
  if (!edges.empty() && !network.is_valid_successor(edges.back(), edge_index)) {
    throw exceptions::ConsistencyException("Edge is not a valid successor.");
  }
  edges.emplace_back(edge_index);
}

void cda_rail::Route::push_front_edge(size_t         edge_index,
                                      const Network& network) {
  /**
   * Adds the edge to the beginning of the route.
   * Throws an error if the edge does not exist in the network or is not a valid
   * predecessor of the first edge.
   *
   * @param edge_index The index of the edge to add.
   * @param network The network to which the edge belongs.
   */

  if (!network.has_edge(edge_index)) {
    throw exceptions::EdgeNotExistentException(edge_index);
  }
  if (!edges.empty() &&
      !network.is_valid_successor(edge_index, edges.front())) {
    throw exceptions::ConsistencyException("Edge is not a valid predecessor.");
  }
  edges.insert(edges.begin(), edge_index);
}

void cda_rail::Route::remove_first_edge() {
  /**
   * Removes the first edge from the route.
   * Throws an error if the route is empty.
   */

  if (edges.empty()) {
    throw exceptions::ConsistencyException("Route is empty.");
  }
  edges.erase(edges.begin());
}

void cda_rail::Route::remove_last_edge() {
  /**
   * Removes the last edge from the route.
   * Throws an error if the route is empty.
   */

  if (edges.empty()) {
    throw exceptions::ConsistencyException("Route is empty.");
  }
  edges.pop_back();
}

size_t cda_rail::Route::get_edge(size_t route_index) const {
  /**
   * Returns the edge at the given index.
   * Throws an error if the index is out of range.
   *
   * @param route_index The index of the edge to return.
   * @return The edge index at the given index.
   */

  if (route_index >= edges.size()) {
    throw exceptions::InvalidInputException("Index out of range.");
  }
  return edges[route_index];
}

const cda_rail::Edge& cda_rail::Route::get_edge(size_t         route_index,
                                                const Network& network) const {
  /**
   * Returns the edge at the given index.
   * Throws an error if the index is out of range.
   *
   * @param route_index The index of the edge to return.
   * @param network The network to which the edge belongs.
   * @return The edge at the given index.
   */

  if (route_index >= edges.size()) {
    throw exceptions::InvalidInputException("Index out of range.");
  }
  return network.get_edge(edges[route_index]);
}

bool cda_rail::Route::check_consistency(const Network& network) const {
  /**
   * Asserts if the route is valid for the given network.
   * A route is valid if all edges exist in the network and if all edges are
   * valid successors of the previous edge. An empty route is valid. Returns
   * true if the route is valid, false otherwise. Throws an error if an edge
   * does not exist in the network.
   *
   * @param network The network to which the route belongs.
   * @return True if the route is valid, false otherwise.
   */

  if (edges.empty()) {
    return true;
  }
  for (size_t i = 0; i < edges.size() - 1; i++) {
    if (!network.is_valid_successor(edges[i], edges[i + 1])) {
      return false;
    }
  }
  return true;
}

double cda_rail::Route::length(const Network& network) const {
  /***
   * Returns the length of the route, i.e., the sum of the lengths of all edges.
   * Throws an error if an edge does not exist in the network.
   *
   * @param network The network to which the route belongs.
   * @return The length of the route.
   */

  return std::accumulate(edges.begin(), edges.end(), 0.0,
                         [&network](double sum, size_t edge) {
                           return sum + network.get_edge(edge).length;
                         });
}

void cda_rail::Route::update_after_discretization(
    const std::vector<std::pair<size_t, cda_rail::index_vector>>& new_edges) {
  /**
   * This method updates the route after the discretization of the network
   * accordingly. For every pair (v, {v_1, ..., v_n}), v is replaced by v_1,
   * ..., v_n.
   *
   * @param new_edges The new edges of the network.
   */

  cda_rail::index_vector edges_updated;
  for (const auto& old_edge : edges) {
    bool replaced = false;
    for (const auto& [track, new_tracks] : new_edges) {
      if (old_edge == track) {
        edges_updated.insert(edges_updated.end(), new_tracks.begin(),
                             new_tracks.end());
        replaced = true;
        break;
      }
    }
    if (!replaced) {
      edges_updated.emplace_back(old_edge);
    }
  }

  edges = std::move(edges_updated);
}

std::pair<double, double>
cda_rail::Route::edge_pos(size_t edge, const Network& network) const {
  /**
   * Returns the position of the given edge in the route, i.e., the distance
   * from the route start to the source and target respectively.
   *
   * @param edge The edge to get the position of.
   * @param network The network to which the edge belongs.
   * @return The position of the edge in the route.
   */

  if (!network.has_edge(edge)) {
    throw exceptions::EdgeNotExistentException(edge);
  }

  std::pair<double, double> return_pos = {0, 0};
  bool                      edge_found = false;
  for (const auto i : edges) {
    return_pos.second += network.get_edge(i).length;
    if (i == edge) {
      edge_found = true;
      break;
    }
    return_pos.first += network.get_edge(i).length;
  }

  if (!edge_found) {
    throw exceptions::ConsistencyException("Edge does not exist in route.");
  }

  return return_pos;
}

std::pair<double, double>
cda_rail::Route::edge_pos(const cda_rail::index_vector& edges_to_consider,
                          const Network&                network) const {
  /**
   * Returns the minimal start and maximal end position of the given
   * edges_to_consider in the route. Throws an error only if none of the
   * edges_to_consider exists in the route.
   *
   * @param edges_to_consider The edges to consider.
   * @param network The network to which the edges belong.
   *
   * @return Start and end location of edge within the route.
   */

  std::pair<double, double> return_pos = {length(network) + 1, -1};

  for (const auto& edge : edges_to_consider) {
    if (!contains_edge(edge)) {
      continue;
    }
    const auto [start_pos, end_pos] = edge_pos(edge, network);
    return_pos.first                = std::min(return_pos.first, start_pos);
    return_pos.second               = std::max(return_pos.second, end_pos);
  }

  if (return_pos.first > return_pos.second) {
    throw exceptions::ConsistencyException(
        "None of the edges_to_consider exists in the route.");
  }

  return return_pos;
}

size_t
cda_rail::Route::get_edge_at_pos(double                   pos,
                                 const cda_rail::Network& network) const {
  round_towards_zero(pos, GRB_EPS);
  if (pos < 0) {
    throw exceptions::InvalidInputException("Position must be non-negative.");
  }

  double current_pos = 0;
  for (const auto& edge : edges) {
    const auto edge_length = network.get_edge(edge).length;
    if (current_pos + edge_length > pos) {
      return edge;
    }
    current_pos += edge_length;
  }
  if (std::abs(current_pos - pos) < GRB_EPS) {
    return edges.back();
  }
  throw exceptions::ConsistencyException("Position is not on the route.");
}

void cda_rail::RouteMap::add_empty_route(const std::string& train_name) {
  /**
   * Adds an empty route for the given train.
   * Throws an error if the train already has a route.
   *
   * @param train_name The name of the train.
   */

  if (routes.find(train_name) != routes.end()) {
    throw exceptions::InvalidInputException("Train already has a route.");
  }
  routes[train_name] = Route();
}

void cda_rail::RouteMap::add_empty_route(const std::string& train_name,
                                         const TrainList&   trains) {
  /**
   * Adds an empty route for the given train.
   * Throws an error if the train already has a route.
   * Throws an error if the train does not exist.
   *
   * @param train_name The name of the train.
   * @param trains The list of trains.
   */

  if (!trains.has_train(train_name)) {
    throw exceptions::TrainNotExistentException(train_name);
  }
  add_empty_route(train_name);
}

void cda_rail::RouteMap::remove_first_edge(const std::string& train_name) {
  /**
   * Removes the first edge from the route of the given train.
   * Throws an error if the train does not have a route.
   *
   * @param train_name The name of the train.
   */

  if (routes.find(train_name) == routes.end()) {
    throw exceptions::ConsistencyException("Train does not have a route.");
  }
  routes[train_name].remove_first_edge();
}

void cda_rail::RouteMap::remove_last_edge(const std::string& train_name) {
  /**
   * Removes the last edge from the route of the given train.
   * Throws an error if the train does not have a route.
   *
   * @param train_name The name of the train.
   */

  if (routes.find(train_name) == routes.end()) {
    throw exceptions::ConsistencyException("Train does not have a route.");
  }
  routes[train_name].remove_last_edge();
}

const cda_rail::Route&
cda_rail::RouteMap::get_route(const std::string& train_name) const {
  /**
   * Returns the route of the given train.
   * Throws an error if the train does not have a route.
   *
   * @param train_name The name of the train.
   *
   * @return The route of the given train.
   */

  if (routes.find(train_name) == routes.end()) {
    throw exceptions::ConsistencyException("Train does not have a route.");
  }
  return routes.at(train_name);
}

bool cda_rail::RouteMap::check_consistency(
    const TrainList& trains, const Network& network,
    bool every_train_must_have_route) const {
  /**
   * Asserts if the route map is valid for the given trains and network.
   * A route map is valid if all routes are valid for the given network and if
   * all trains with routes exist. If every_train_must_have_route is true, then
   * the route map is only valid if all trains have routes.
   *
   * @param trains The list of trains.
   * @param network The network to which the routes belong.
   * @param every_train_must_have_route If true, then the route map is only
   * valid if all trains have routes.
   * @return True if the route map is valid, false otherwise.
   */

  if (every_train_must_have_route && routes.size() != trains.size()) {
    return false;
  }

  return std::ranges::all_of(routes, [&trains, &network](const auto& route) {
    return trains.has_train(route.first) &&
           route.second.check_consistency(network);
  });
}

void cda_rail::RouteMap::export_routes(const std::filesystem::path& p,
                                       const Network& network) const {
  /**
   * Exports the routes to a routes.json file within the given path.
   * The form is {train: [[e1_0, e1_1], [e2_0, e2_1], ...]} where the edge names
   * are used.
   *
   * @param p The path to the directory in which the routes.json file should be
   * created.
   * @param network The network to which the routes belong.
   */

  if (!is_directory_and_create(p)) {
    throw exceptions::ExportException("Could not create directory " +
                                      p.string());
  }

  json j;
  for (const auto& [name, route] : routes) {
    std::vector<std::pair<std::string, std::string>> route_edges;
    for (int i = 0; i < route.size(); i++) {
      const auto& edge = route.get_edge(i, network);
      route_edges.emplace_back(network.get_vertex(edge.source).name,
                               network.get_vertex(edge.target).name);
    }
    j[name] = route_edges;
  }

  std::ofstream file(p / "routes.json");
  file << j << '\n';
}

void cda_rail::RouteMap::push_back_edge(const std::string& train_name,
                                        size_t             edge_index,
                                        const Network&     network) {
  /**
   * Adds the edge to the end of the route of the given train.
   * Throws an error if the train does not have a route.
   *
   * @param train_name The name of the train.
   * @param edge_index The index of the edge in the network.
   * @param network The network to which the routes belong.
   */

  if (routes.find(train_name) == routes.end()) {
    throw exceptions::ConsistencyException("Train does not have a route.");
  }
  routes[train_name].push_back_edge(edge_index, network);
}

void cda_rail::RouteMap::push_back_edge(const std::string& train_name,
                                        size_t source, size_t target,
                                        const Network& network) {
  /**
   * Adds the edge to the end of the route of the given train.
   * Throws an error if the train does not have a route.
   *
   * @param train_name The name of the train.
   * @param source The index of the source station in the network.
   * @param target The index of the target station in the network.
   * @param network The network to which the routes belong.
   */

  if (routes.find(train_name) == routes.end()) {
    throw exceptions::ConsistencyException("Train does not have a route.");
  }
  routes[train_name].push_back_edge(source, target, network);
}

void cda_rail::RouteMap::push_back_edge(const std::string& train_name,
                                        const std::string& source,
                                        const std::string& target,
                                        const Network&     network) {
  /**
   * Adds the edge to the end of the route of the given train.
   * Throws an error if the train does not have a route.
   *
   * @param train_name The name of the train.
   * @param source The name of the source station.
   * @param target The name of the target station.
   * @param network The network to which the routes belong.
   */

  if (routes.find(train_name) == routes.end()) {
    throw exceptions::ConsistencyException("Train does not have a route.");
  }
  routes[train_name].push_back_edge(source, target, network);
}

void cda_rail::RouteMap::push_front_edge(const std::string& train_name,
                                         size_t             edge_index,
                                         const Network&     network) {
  /**
   * Adds the edge to the front of the route of the given train.
   * Throws an error if the train does not have a route.
   *
   * @param train_name The name of the train.
   * @param edge_index The index of the edge in the network.
   * @param network The network to which the routes belong.
   */

  if (routes.find(train_name) == routes.end()) {
    throw exceptions::ConsistencyException("Train does not have a route.");
  }
  routes[train_name].push_front_edge(edge_index, network);
}

void cda_rail::RouteMap::push_front_edge(const std::string& train_name,
                                         size_t source, size_t target,
                                         const Network& network) {
  /**
   * Adds the edge to the front of the route of the given train.
   * Throws an error if the train does not have a route.
   *
   * @param train_name The name of the train.
   * @param source The index of the source station in the network.
   * @param target The index of the target station in the network.
   * @param network The network to which the routes belong.
   */

  if (routes.find(train_name) == routes.end()) {
    throw exceptions::ConsistencyException("Train does not have a route.");
  }
  routes[train_name].push_front_edge(source, target, network);
}

void cda_rail::RouteMap::push_front_edge(const std::string& train_name,
                                         const std::string& source,
                                         const std::string& target,
                                         const Network&     network) {
  /**
   * Adds the edge to the front of the route of the given train.
   * Throws an error if the train does not have a route.
   *
   * @param train_name The name of the train.
   * @param source The name of the source station.
   * @param target The name of the target station.
   * @param network The network to which the routes belong.
   */

  if (routes.find(train_name) == routes.end()) {
    throw exceptions::ConsistencyException("Train does not have a route.");
  }
  routes[train_name].push_front_edge(source, target, network);
}

cda_rail::RouteMap::RouteMap(const std::filesystem::path& p,
                             const Network&               network) {
  /**
   * Constructs the object and imports the routes from a routes.json file within
   * the given path. The form is {train: [[e1_0, e1_1], [e2_0, e2_1], ...]}
   * where the edge names are used.
   *
   * @param p The path to the directory in which the routes.json file should be
   * created.
   * @param network The network to which the routes belong.
   */

  if (!std::filesystem::exists(p)) {
    throw exceptions::ImportException("Path does not exist.");
  }
  if (!std::filesystem::is_directory(p)) {
    throw exceptions::ImportException("Path is not a directory.");
  }

  std::ifstream file(p / "routes.json");
  json          data = json::parse(file);

  for (const auto& [name, route] : data.items()) {
    this->add_empty_route(name);
    for (auto& edge : route) {
      this->push_back_edge(name, edge[0].get<std::string>(),
                           edge[1].get<std::string>(), network);
    }
  }
}

double cda_rail::RouteMap::length(const std::string& train_name,
                                  const Network&     network) const {
  /**
   * Returns the length of the route of the given train.
   *
   * @param train_name The name of the train.
   * @param network The network to which the routes belong.
   *
   * @return The length of the route of the given train.
   */

  return get_route(train_name).length(network);
}

void cda_rail::RouteMap::update_after_discretization(
    const std::vector<std::pair<size_t, cda_rail::index_vector>>& new_edges) {
  /**
   * This method updates the routes after the discretization of the network
   * accordingly. For every pair (v, {v_1, ..., v_n}), v is replaced by v_1,
   * ..., v_n.
   *
   * @param new_edges The new edges of the network.
   */

  for (auto& [train_name, route] : routes) {
    route.update_after_discretization(new_edges);
  }
}

void cda_rail::RouteMap::remove_route(const std::string& train_name) {
  if (routes.find(train_name) == routes.end()) {
    throw exceptions::ConsistencyException("Train does not have a route.");
  }
  routes.erase(train_name);
}

std::optional<double>
cda_rail::Route::get_first_pos_on_edges(const std::vector<size_t>& edge_indices,
                                        const Network& network) const {
  /**
   * This functions returns the position at the beginning of the first route
   * edge that is part of edge_indices.
   *
   * @param edge_indices The edge indices to consider.
   * @param network The network to which the route belongs.
   * @return The position at the beginning of the first route edge that is part
   * of edge_indices
   */

  double position = 0.0;
  for (const auto& edge : edges) {
    if (std::ranges::contains(edge_indices, edge)) {
      return position;
    }
    position += network.get_edge(edge).length;
  }
  return {};
}

std::optional<double>
cda_rail::Route::get_last_pos_on_edges(const std::vector<size_t>& edge_indices,
                                       const Network& network) const {
  /**
   * This functions returns the position at the end of the last route edge that
   * is part of edge_indices.
   *
   * @param edge_indices The edge indices to consider.
   * @param network The network to which the route belongs.
   * @return The position at the end of the last route edge that is part of
   * edge_indices.
   */

  double                position = 0.0;
  std::optional<double> return_position;
  for (const auto& edge : edges) {
    position += network.get_edge(edge).length;
    if (std::ranges::contains(edge_indices, edge)) {
      return_position = position;
    }
  }
  return return_position;
}

std::vector<cda_rail::ConflictPair>
cda_rail::RouteMap::get_parallel_overlaps(const std::string& train1,
                                          const std::string& train2,
                                          const Network&     network) const {
  /**
   * This functions returns all parallel overlaps between the routes of train1
   * and train2. An overlap is an interval in which both trains are on the same
   * tracks. The return information returns the start and end positions of the
   * overlap ob both routes (in m). E.g., if train1 travels on e1, e2, e3, e4
   * and train2 travels on e0, e2, e3, e5; then the overlap is on e2 and e3,
   * i.e., the return value is {(length(e1), length(e1)+length(e2)+length(e3)),
   * (length(e0), length(e0)+length(e2)+length(e3))}.
   *
   * @param train1 The name of the first train.
   * @param train2 The name of the second train.
   * @param network The network to which the routes belong.
   * @return A vector of ConflictPairs representing the overlaps.
   */

  std::vector<ConflictPair> result;

  // Get both routes
  const auto& route1 = get_route(train1);
  const auto& route2 = get_route(train2);

  const auto& edges1 = route1.get_edges();
  const auto& edges2 = route2.get_edges();

  if (edges1.empty() || edges2.empty()) {
    return result;
  }

  // Find all consecutive overlaps
  size_t i1   = 0;
  double pos1 = 0.0;
  while (i1 < edges1.size()) {
    // Check if current edge in route1 exists in route2
    const size_t edge1 = edges1.at(i1);
    if (const auto edge2_index = std::ranges::find(edges2, edge1);
        edge2_index != edges2.end()) {
      // Found an overlapping edge, now find the full overlap
      size_t i2 = std::distance(edges2.begin(), edge2_index);
      std::unordered_set<size_t> edges_in_overlap;

      assert(i1 < edges1.size() && i2 < edges2.size() &&
             edges1.at(i1) == edges2.at(i2));

      // Calculate start position
      double       pos2       = route2.edge_pos(edge1, network).first;
      const double start_pos1 = pos1;
      const double start_pos2 = pos2;

      // Extend the overlap as long as edges match
      while (i1 < edges1.size() && i2 < edges2.size() &&
             edges1.at(i1) == edges2.at(i2)) {
        edges_in_overlap.insert(network.get_track_index(edges1.at(i1)));
        pos1 += network.get_edge(edges1.at(i1)).length;
        pos2 += network.get_edge(edges2.at(i2)).length;
        ++i1;
        ++i2;
      }

      // Store the found overlap
      result.emplace_back(std::make_pair(start_pos1, pos1),
                          std::make_pair(start_pos2, pos2), edges_in_overlap);
    } else {
      // No overlap, move to the next edge in route1
      pos1 += network.get_edge(edge1).length;
      ++i1;
    }
  }

  return result;
}

std::vector<cda_rail::ConflictPair>
cda_rail::RouteMap::get_ttd_overlaps(const std::string& train1,
                                     const std::string& train2,
                                     const Network&     network) const {
  /**
   * This functions returns all ttd overlaps between the routes of train1
   * and train2. An overlap is an interval in which both trains are on the same
   * TTD section. The return information returns the start and end positions of
   * the overlap ob both routes (in m), similarly to parallel overlaps.
   *
   * @param train1 The name of the first train.
   * @param train2 The name of the second train.
   * @param network The network to which the routes belong.
   * @return A vector of ConflictPairs representing the overlaps.
   */

  std::vector<ConflictPair> result;

  // Get both routes
  const auto& route1 = get_route(train1);
  const auto& route2 = get_route(train2);
  const auto& edges1 = route1.get_edges();

  std::unordered_set<size_t> blacklist;
  for (const auto& edge1 : edges1) {
    if (blacklist.contains(edge1)) {
      continue;
    }

    if (const auto& e1_obj = network.get_edge(edge1); e1_obj.breakable) {
      continue;
    }
    const auto& ttd_sec =
        network.get_unbreakable_section_containing_edge(edge1);
    assert(!ttd_sec.empty());
    std::unordered_set<size_t> ttd_tracks;
    for (const auto& ttd_sec_edge : ttd_sec) {
      ttd_tracks.insert(network.get_track_index(ttd_sec_edge));
      blacklist.insert(ttd_sec_edge);
    }
    const auto start_pos2 = route2.get_first_pos_on_edges(ttd_sec, network);
    const auto end_pos2   = route2.get_last_pos_on_edges(ttd_sec, network);
    assert(start_pos2.has_value() == end_pos2.has_value());
    if (start_pos2.has_value() && end_pos2.has_value()) {
      const auto start_pos1 = route1.get_first_pos_on_edges(ttd_sec, network);
      const auto end_pos1   = route1.get_last_pos_on_edges(ttd_sec, network);
      assert(start_pos1.has_value() && end_pos1.has_value());
      result.emplace_back(std::make_pair(start_pos1.value(), end_pos1.value()),
                          std::make_pair(start_pos2.value(), end_pos2.value()),
                          ttd_tracks);
    }
  }

  return result;
}

std::vector<cda_rail::ConflictPair>
cda_rail::RouteMap::get_reverse_overlaps(const std::string& train1,
                                         const std::string& train2,
                                         const Network&     network) const {
  /**
   * This functions returns all reverse overlaps between the routes of train1
   * and train2, i.e., parts where both trains travel on the same track but in
   * different direction. The return information returns the start and end
   * positions of the overlap ob both routes (in m). E.g., if train1 travels on
   * e1, e2, e3, e4 and train2 travels on e5, e3', e2', e0; (e3' and e2' being
   * the reverse indices of e3 and r2), then the overlap is on e2 and e3, i.e.,
   * the return value is {(length(e1), length(e1)+length(e2)+length(e3)),
   * (length(e5), length(e5)+length(e2')+length(e3'))}.
   *
   * @param train1 The name of the first train.
   * @param train2 The name of the second train.
   * @param network The network to which the routes belong.
   * @return A vector of ConflictPairs representing the overlaps.
   */

  std::vector<ConflictPair> result;

  // Get both routes
  const auto& route1 = get_route(train1);
  const auto& route2 = get_route(train2);

  const auto& edges1 = route1.get_edges();
  const auto& edges2 = route2.get_edges();

  if (edges1.empty() || edges2.empty()) {
    return result;
  }

  // Find all consecutive overlaps
  size_t i1   = 0;
  double pos1 = 0.0;
  while (i1 < edges1.size()) {
    // Check if current edge in route1 exists in route2
    const size_t edge1 = edges1.at(i1);
    if (const auto edge1_reverse = network.get_reverse_edge_index(edge1);
        (edge1_reverse.has_value() &&
         std::ranges::contains(edges2, edge1_reverse.value()))) {
      const auto edge2_index = std::ranges::find(edges2, edge1_reverse.value());
      // Found an overlapping edge, now find the full overlap
      size_t i2 = std::distance(edges2.begin(), edge2_index);
      std::unordered_set<size_t> edges_in_overlap;

      assert(i1 < edges1.size() && i2 < edges2.size() &&
             edges2.at(i2) == edge1_reverse.value());

      // Calculate start position
      double pos2 = route2.edge_pos(edge1_reverse.value(), network).second;
      const double start_pos1 = pos1;
      const double end_pos2   = pos2;

      // Extend the overlap as long as edges match
      bool i2_at_end = false;
      while (!i2_at_end && i1 < edges1.size() &&
             network.get_reverse_edge_index(edges1.at(i1)).has_value() &&
             network.get_reverse_edge_index(edges1.at(i1)).value_or(-1) ==
                 edges2.at(i2)) {
        assert(network.get_track_index(edges1.at(i1)) ==
               network.get_track_index(edges2.at(i2)));
        edges_in_overlap.insert(network.get_track_index(edges1.at(i1)));
        pos1 += network.get_edge(edges1.at(i1)).length;
        pos2 -= network.get_edge(edges2.at(i2)).length;
        ++i1;
        i2_at_end = (i2 == 0);
        if (!i2_at_end) {
          --i2;
        }
      }

      // Store the found overlap
      result.emplace_back(std::make_pair(start_pos1, pos1),
                          std::make_pair(pos2, end_pos2), edges_in_overlap);
    } else {
      // No overlap, move to the next edge in route1
      pos1 += network.get_edge(edge1).length;
      ++i1;
    }
  }

  return result;
}

std::vector<cda_rail::ConflictPair>
cda_rail::RouteMap::get_crossing_overlaps(const std::string& train1,
                                          const std::string& train2,
                                          const Network&     network) const {
  /**
   * combine ttd and reverse conflicts
   */
  const auto ttd_conflicts     = get_ttd_overlaps(train1, train2, network);
  const auto reverse_conflicts = get_reverse_overlaps(train1, train2, network);

  // concatenate conflicts and order by train 1 start
  std::vector<ConflictPair> conflicts;
  conflicts.reserve(ttd_conflicts.size() + reverse_conflicts.size());
  conflicts.insert(conflicts.end(), ttd_conflicts.begin(), ttd_conflicts.end());
  conflicts.insert(conflicts.end(), reverse_conflicts.begin(),
                   reverse_conflicts.end());
  std::ranges::sort(conflicts,
                    [](const ConflictPair& a, const ConflictPair& b) {
                      const auto& [a1, a2, a3] = a;
                      const auto& [b1, b2, b3] = b;
                      return a1.first < b1.first;
                    });

  // for any two conflicts, unite them if they are directly next to each other
  // or overlap, i.e., if train1_end of prev >= train1_start of succ AND if
  // train2_end of succ >= train_2 start of prev Then unite them into one
  // conflict
  std::vector<ConflictPair> result;
  if (conflicts.empty()) {
    return result;
  }

  result.emplace_back(conflicts.at(0));
  for (size_t i = 1; i < conflicts.size(); ++i) {
    auto& [prev1, prev2, prev_edges] = result.back();

    if (const auto& [succ1, succ2, succ_edges] = conflicts.at(i);
        prev1.second >= succ1.first && succ2.second >= prev2.first) {
      prev1.second = std::max(prev1.second, succ1.second);
      prev2.first  = std::min(prev2.first, succ2.first);
      prev_edges.insert(succ_edges.begin(), succ_edges.end());
    } else {
      result.emplace_back(conflicts.at(i));
    }
  }

  return result;
}
