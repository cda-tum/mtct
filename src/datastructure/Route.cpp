#include "datastructure/Route.hpp"

#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "nlohmann/json.hpp"

#include <fstream>

using json = nlohmann::json;

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
    const std::vector<std::pair<size_t, std::vector<size_t>>>& new_edges) {
  /**
   * This method updates the route after the discretization of the network
   * accordingly. For every pair (v, {v_1, ..., v_n}), v is replaced by v_1,
   * ..., v_n.
   *
   * @param new_edges The new edges of the network.
   */

  std::vector<size_t> edges_updated;
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
cda_rail::Route::edge_pos(const std::vector<size_t>& edges_to_consider,
                          const Network&             network) const {
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
  if (std::abs(pos) < GRB_EPS) {
    pos = 0;
  }
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

  return std::all_of(routes.begin(), routes.end(),
                     [&trains, &network](const auto& route) {
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
  file << j << std::endl;
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
    const std::vector<std::pair<size_t, std::vector<size_t>>>& new_edges) {
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
