#include "datastructure/Route.hpp"

#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "GeneralHelper.hpp"
#include "StringHelper.hpp"
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
#include <functional>
#include <iterator>
#include <limits>
#include <numeric>
#include <optional>
#include <ranges>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

using json = nlohmann::json;
using std::size_t;

// -----------------------------
// ROUTE
// -----------------------------

/**
 * @brief Calculates the total length of the route.
 *
 * @return double The sum of lengths of all edges in the route.
 */

double cda_rail::Route::length(const Network& network) const {
  const auto edge_lengths =
      m_edges | std::views::transform([&network](size_t edge) {
        return network.get_edge(edge).length;
      });
  return std::accumulate(edge_lengths.begin(), edge_lengths.end(), 0.0,
                         std::plus{});
}

/**
 * @brief Determines the position of an edge within the route.
 *
 * @param edge The edge to locate.
 * @return An `EdgePosition` where `source` is the distance from the route's start to the edge's start,
 *         and `target` is the distance to the edge's end.
 *
 * @throws EdgeNotExistentException if the edge does not exist in the network.
 * @throws ConsistencyException if the edge is not part of this route.
 */
cda_rail::Route::EdgePosition
cda_rail::Route::edge_pos_on_route(Network::EdgeInput const& edge,
                                   const Network&            network) const {
  auto const edge_id = edge.resolve(&network);

  if (!network.has_edge(edge_id)) {
    throw exceptions::EdgeNotExistentException(edge_id);
  }

  const auto edge_it = std::ranges::find(m_edges, edge_id);
  if (edge_it == m_edges.end()) {
    throw exceptions::ConsistencyException("Edge does not exist in route.");
  }

  const auto prefix_edges =
      std::ranges::subrange(m_edges.begin(), edge_it) |
      std::views::transform([&network](size_t route_edge) {
        return network.get_edge(route_edge).length;
      });

  const double source_pos = std::accumulate(
      prefix_edges.begin(), prefix_edges.end(), 0.0, std::plus{});

  return {.source = source_pos,
          .target = source_pos + network.get_edge(*edge_it).length};
}

/**
 * @brief Determines the position span on the route that covers a set of edges.
 *
 * @param edges_to_consider Edge IDs to locate on the route.
 * @return EdgePosition spanning from the start position of the first considered edge to the end position of the last considered edge on the route.
 * @throws exceptions::ConsistencyException if none of the provided edges exist on the route.
 */
cda_rail::Route::EdgePosition cda_rail::Route::edge_set_pos_on_route(
    const cda_rail::index_vector& edges_to_consider,
    const Network&                network) const {
  const std::unordered_set<size_t> considered_edges(edges_to_consider.begin(),
                                                    edges_to_consider.end());

  EdgePosition return_pos{.source = std::numeric_limits<double>::infinity(),
                          .target = -1.0};
  double       current_pos = 0.0;

  for (const size_t route_edge : m_edges) {
    const double edge_length = network.get_edge(route_edge).length;
    if (considered_edges.contains(route_edge)) {
      return_pos.source = std::min(return_pos.source, current_pos);
      return_pos.target =
          std::max(return_pos.target, current_pos + edge_length);
    }
    current_pos += edge_length;
  }

  if (return_pos.source > return_pos.target) {
    throw exceptions::ConsistencyException(
        "None of the edges_to_consider exists in the route.");
  }

  return return_pos;
}

/**
 * @brief Determines the starting position of the first edge from a set that appears on the route.
 *
 * @param edge_indices Set of edge identifiers to search for.
 * @param network The network providing edge metadata.
 *
 * @return The starting position of the first matching edge, or `std::nullopt` if no edges from the set are found on the route.
 */
std::optional<double>
cda_rail::Route::get_first_pos_on_edges(const cda_rail::index_set& edge_indices,
                                        const Network& network) const {
  return get_pos_on_edges_impl(edge_indices, network, true);
}

/**
 * @brief Determines the end position of the last matching edge on the route.
 *
 * @return The position where the last edge from `edge_indices` ends, or empty if no matching edges exist.
 */
std::optional<double>
cda_rail::Route::get_last_pos_on_edges(const cda_rail::index_set& edge_indices,
                                       const Network& network) const {
  return get_pos_on_edges_impl(edge_indices, network, false);
}

/**
 * @brief Finds the edge at the given position along the route.
 *
 * @return size_t The id of the edge at that position.
 * 
 * @throws ConsistencyException If the route is empty or the position does not fall on any edge.
 * @throws InvalidInputException If pos is negative.
 */
size_t
cda_rail::Route::get_edge_id_at_pos(double                   pos,
                                    const cda_rail::Network& network) const {
  round_small_numbers_to_zero_inplace(pos, GRB_EPS);
  if (m_edges.empty()) {
    throw exceptions::ConsistencyException("Route is empty.");
  }
  if (pos < 0) {
    throw exceptions::InvalidInputException("Position must be non-negative.");
  }

  double current_pos = 0;
  for (const auto& edge : m_edges) {
    const auto edge_length = network.get_edge(edge).length;
    if (current_pos + edge_length > pos) {
      return edge;
    }
    current_pos += edge_length;
  }
  if (std::abs(current_pos - pos) < GRB_EPS) {
    return m_edges.back();
  }
  throw exceptions::ConsistencyException("Position is not on the route.");
}

/**
 * @brief Retrieves the edge id at a position along the route.
 *
 * @param route_index The position along the route.
 * @return size_t The edge id at the given position.
 * @throws InvalidInputException if the index is out of range.
 */
size_t cda_rail::Route::get_edge_id(size_t route_index) const {
  is_route_index_valid(route_index);
  return m_edges.at(route_index);
}

/**
 * @brief Retrieves the edge at a position in the route.
 *
 * @param route_index Position of the edge within the route.
 * @return const Edge& Const reference to the edge at the specified position.
 * @throws InvalidInputException if route_index is out of range.
 */
const cda_rail::Edge& cda_rail::Route::get_edge(size_t         route_index,
                                                const Network& network) const {
  is_route_index_valid(route_index);
  return network.get_edge(m_edges.at(route_index));
}

/**
 * @brief Appends an edge to the end of the route.
 *
 * If the route is non-empty, validates that the new edge is a valid successor
 * of the last edge.
 *
 * @param new_edge The edge to append.
 * @throws ConsistencyException if the route is non-empty and the edge is not a
 * valid successor of the last edge.
 */

void cda_rail::Route::push_back_edge(Network::EdgeInput const& new_edge,
                                     const Network&            network) {
  auto const edge_index = new_edge.resolve(&network);
  if (!m_edges.empty() &&
      !network.is_valid_successor(m_edges.back(), edge_index)) {
    throw exceptions::ConsistencyException("Edge is not a valid successor.");
  }
  m_edges.emplace_back(edge_index);
}

/**
 * @brief Adds an edge to the beginning of the route.
 *
 * @param new_edge The edge to prepend.
 *
 * @throws ConsistencyException if the new edge is not a valid predecessor of the first edge in the route.
 */
void cda_rail::Route::push_front_edge(Network::EdgeInput const& new_edge,
                                      const Network&            network) {
  auto const edge_index = new_edge.resolve(&network);

  if (!m_edges.empty() &&
      !network.is_valid_successor(edge_index, m_edges.front())) {
    throw exceptions::ConsistencyException("Edge is not a valid predecessor.");
  }
  m_edges.insert(m_edges.begin(), edge_index);
}

/**
 * @brief Removes the first edge from the route.
 * @throws ConsistencyException if the route is empty.
 */
void cda_rail::Route::remove_first_edge() {
  if (!has_edges()) {
    throw exceptions::ConsistencyException("Route is empty.");
  }
  m_edges.erase(m_edges.begin());
}

/**
 * @brief Removes the last edge from the route.
 *
 * @throws exceptions::ConsistencyException If the route is empty.
 */
void cda_rail::Route::remove_last_edge() {
  if (!has_edges()) {
    throw exceptions::ConsistencyException("Route is empty.");
  }
  m_edges.pop_back();
}

/**
 * @brief Validates that all route edges exist in the network and form valid consecutive connections.
 *
 * Verifies that each edge in the route exists in the network and that all consecutive edge pairs
 * form valid successor pairs according to the network's connectivity rules.
 *
 * @param network The network to validate edges against.
 *
 * @return `true` if all consecutive edge pairs are valid successors, `false` otherwise.
 *
 * @throws EdgeNotExistentException if any edge in the route does not exist in the network.
 */

bool cda_rail::Route::check_consistency(const Network& network) const {
  for (const auto edge : m_edges) {
    if (!network.has_edge(edge)) {
      throw exceptions::EdgeNotExistentException(edge);
    }
  }
  return std::ranges::adjacent_find(
             m_edges, [&network](size_t edge, size_t successor) {
               return !network.is_valid_successor(edge, successor);
             }) == m_edges.end();
}

/**
 * @brief Updates the route's edges by replacing each edge with its discretized subdivision.
 *
 * @param new_edges Maps old edge ids to their replacement edge sequences.
 */
void cda_rail::Route::update_after_discretization(
    const std::vector<std::pair<size_t, cda_rail::index_vector>>& new_edges) {
  std::unordered_map<size_t, const cda_rail::index_vector*> replacement_map;
  replacement_map.reserve(new_edges.size());
  for (const auto& [old_edge, replacement_edges] : new_edges) {
    replacement_map[old_edge] = &replacement_edges;
  }

  cda_rail::index_vector edges_updated;
  for (const size_t old_edge : m_edges) {
    if (const auto replacement_it = replacement_map.find(old_edge);
        replacement_it != replacement_map.end()) {
      const auto& replacement_edges = *replacement_it->second;
      edges_updated.insert(edges_updated.end(), replacement_edges.begin(),
                           replacement_edges.end());
      continue;
    }
    edges_updated.emplace_back(old_edge);
  }

  m_edges = std::move(edges_updated);
}

/**
 * @brief Finds the position of edges from a given set on the route.
 *
 * @param edge_indices The set of edge ids to locate.
 * @param network The network providing edge lengths.
 * @param first_match If true, returns the position of the first matching edge; if false, returns the position of the last.
 * @return The start position of the first matching edge if `first_match` is true; the end position of the last matching edge if `first_match` is false. Returns empty if no edges from the set are found on the route.
 */
std::optional<double>
cda_rail::Route::get_pos_on_edges_impl(const cda_rail::index_set& edge_indices,
                                       const Network&             network,
                                       bool first_match) const {
  const std::unordered_set<size_t> edge_set(edge_indices.begin(),
                                            edge_indices.end());

  double                position = 0.0;
  std::optional<double> last_position;

  for (const size_t edge : m_edges) {
    const double edge_length = network.get_edge(edge).length;
    if (edge_set.contains(edge)) {
      if (first_match) {
        return position;
      }
      last_position = position + edge_length;
    }
    position += edge_length;
  }

  return last_position;
}

/**
 * @brief Validates that an index is within the route's valid range.
 * @throws InvalidInputException if the index is out of range.
 */
void cda_rail::Route::is_route_index_valid(size_t route_index) const {
  if (route_index >= m_edges.size()) {
    throw exceptions::InvalidInputException("Index out of range.");
  }
}

/**
 * @brief Determines whether the route contains any edges.
 *
 * @return bool `true` if the route contains at least one edge, `false` otherwise.
 */
bool cda_rail::Route::has_edges() const { return !m_edges.empty(); }

// -----------------------------
// ROUTE MAP
// -----------------------------

/**
 * @brief Retrieves the route for a train.
 *
 * @return const cda_rail::Route& The route for the specified train.
 *
 * @throws cda_rail::exceptions::ConsistencyException if the train does not have a route.
 */
const cda_rail::Route&
cda_rail::RouteMap::get_route(const std::string& train_name) const {
  throw_if_train_has_no_route(train_name);
  return m_routes.at(train_name);
}

/**
 * @brief Computes the total length of a train's route.
 *
 * @param train_name The name of the train.
 * @param network The network containing edge information.
 * @return The sum of lengths of all edges in the train's route.
 *
 * @throw ConsistencyException If the train has no route.
 */
double cda_rail::RouteMap::route_length(const std::string& train_name,
                                        const Network&     network) const {
  return get_route(train_name).length(network);
}

/**
 * @brief Identifies overlapping segments where two routes traverse the same sequence of edges in the same order.
 *
 * @param train1 Name of the first train.
 * @param train2 Name of the second train.
 * @param network The network containing edge and track information.
 * @return A vector of conflict pairs, each representing a contiguous segment where both routes share the same edges in sequence. Each pair specifies position ranges on both routes and the set of track indices involved.
 */

std::vector<cda_rail::ConflictPair>
cda_rail::RouteMap::get_parallel_overlaps(const std::string& train1,
                                          const std::string& train2,
                                          const Network&     network) const {
  std::vector<ConflictPair> result;

  const auto& route1 = get_route(train1);
  const auto& route2 = get_route(train2);
  const auto& edges1 = route1.get_edges();
  const auto& edges2 = route2.get_edges();

  if (edges1.empty() || edges2.empty()) {
    return result;
  }

  size_t i1   = 0;
  double pos1 = 0.0;
  while (i1 < edges1.size()) {
    const size_t edge1 = edges1.at(i1);
    if (const auto edge2_it = std::ranges::find(edges2, edge1);
        edge2_it != edges2.end()) {
      size_t                     i2 = std::distance(edges2.begin(), edge2_it);
      std::unordered_set<size_t> edges_in_overlap;

      assert(i1 < edges1.size() && i2 < edges2.size() &&
             edges1.at(i1) == edges2.at(i2));

      double       pos2       = route2.edge_pos_on_route(edge1, network).source;
      const double start_pos1 = pos1;
      const double start_pos2 = pos2;

      while (i1 < edges1.size() && i2 < edges2.size() &&
             edges1.at(i1) == edges2.at(i2)) {
        edges_in_overlap.insert(network.get_track_index(edges1.at(i1)));
        pos1 += network.get_edge(edges1.at(i1)).length;
        pos2 += network.get_edge(edges2.at(i2)).length;
        ++i1;
        ++i2;
      }

      result.emplace_back(ConflictPair{.pos1  = {start_pos1, pos1},
                                       .pos2  = {start_pos2, pos2},
                                       .edges = edges_in_overlap});
    } else {
      pos1 += network.get_edge(edge1).length;
      ++i1;
    }
  }

  return result;
}

/**
 * @brief Identifies overlaps where two train routes share unbreakable track sections.
 *
 * For each unbreakable section traversed by the first train's route, determines
 * whether the second train's route also traverses the same section. Returns all
 * shared unbreakable sections as conflict pairs.
 *
 * @param train1 Name of the first train.
 * @param train2 Name of the second train.
 * @return A vector of conflict pairs, each recording the position intervals on
 *         both routes and the track indices of the shared unbreakable section.
 */
std::vector<cda_rail::ConflictPair>
cda_rail::RouteMap::get_ttd_overlaps(const std::string& train1,
                                     const std::string& train2,
                                     const Network&     network) const {
  std::vector<ConflictPair> result;

  const auto& route1 = get_route(train1);
  const auto& route2 = get_route(train2);
  const auto& edges1 = route1.get_edges();

  std::unordered_set<size_t> visited_edges;
  for (const auto& edge1 : edges1) {
    if (visited_edges.contains(edge1)) {
      continue;
    }
    if (network.get_edge(edge1).breakable) {
      continue;
    }

    const auto& ttd_sec =
        network.get_unbreakable_section_containing_edge(edge1);
    assert(!ttd_sec.empty());

    std::unordered_set<size_t> ttd_tracks;
    for (const auto ttd_sec_edge : ttd_sec) {
      ttd_tracks.insert(network.get_track_index(ttd_sec_edge));
      visited_edges.insert(ttd_sec_edge);
    }

    const auto start_pos2 = route2.get_first_pos_on_edges(ttd_sec, network);
    const auto end_pos2   = route2.get_last_pos_on_edges(ttd_sec, network);
    assert(start_pos2.has_value() == end_pos2.has_value());

    if (start_pos2.has_value() && end_pos2.has_value()) {
      const auto start_pos1 = route1.get_first_pos_on_edges(ttd_sec, network);
      const auto end_pos1   = route1.get_last_pos_on_edges(ttd_sec, network);
      assert(start_pos1.has_value() && end_pos1.has_value());
      result.emplace_back(
          ConflictPair{.pos1  = {start_pos1.value(), end_pos1.value()},
                       .pos2  = {start_pos2.value(), end_pos2.value()},
                       .edges = ttd_tracks});
    }
  }

  return result;
}

/**
 * @brief Finds overlapping route sections where trains travel in opposite directions.
 *
 * Identifies all segments where an edge in one train's route has a reverse-direction
 * counterpart in the other's route, tracking position intervals and tracks involved.
 *
 * @return Vector of conflict pairs, each containing position intervals on both routes
 *         and the tracks traversed in the overlap.
 */
std::vector<cda_rail::ConflictPair>
cda_rail::RouteMap::get_reverse_overlaps(const std::string& train1,
                                         const std::string& train2,
                                         const Network&     network) const {
  std::vector<ConflictPair> result;

  const auto& route1 = get_route(train1);
  const auto& route2 = get_route(train2);
  const auto& edges1 = route1.get_edges();
  const auto& edges2 = route2.get_edges();

  if (edges1.empty() || edges2.empty()) {
    return result;
  }

  size_t i1   = 0;
  double pos1 = 0.0;
  while (i1 < edges1.size()) {
    const size_t edge1         = edges1.at(i1);
    const auto   edge1_reverse = network.get_reverse_edge_index(edge1);

    if (edge1_reverse.has_value() &&
        std::ranges::contains(edges2, edge1_reverse.value())) {
      const auto edge2_it = std::ranges::find(edges2, edge1_reverse.value());
      size_t     i2       = std::distance(edges2.begin(), edge2_it);
      std::unordered_set<size_t> edges_in_overlap;

      assert(i1 < edges1.size() && i2 < edges2.size() &&
             edges2.at(i2) == edge1_reverse.value());

      double pos2 =
          route2.edge_pos_on_route(edge1_reverse.value(), network).target;
      const double start_pos1 = pos1;
      const double end_pos2   = pos2;

      bool i2_at_end = false;
      while (!i2_at_end && i1 < edges1.size()) {
        const auto rev = network.get_reverse_edge_index(edges1.at(i1));
        if (!rev.has_value() || rev.value() != edges2.at(i2)) {
          break;
        }
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

      result.emplace_back(ConflictPair{.pos1  = {start_pos1, pos1},
                                       .pos2  = {pos2, end_pos2},
                                       .edges = edges_in_overlap});
    } else {
      pos1 += network.get_edge(edge1).length;
      ++i1;
    }
  }

  return result;
}

/**
 * @brief Identifies crossing overlaps between two trains by merging time-table and reverse-direction overlaps.
 *
 * Computes all time-table-dependent and reverse-direction conflicts between the two routes,
 * then consolidates overlapping regions along train1's route into single intervals.
 *
 * @param train1 Identifier of the first train.
 * @param train2 Identifier of the second train.
 * @param network The rail network.
 *
 * @return A vector of merged conflict pairs, where overlapping regions are consolidated
 *         into single intervals sorted by position along train1's route.
 */
std::vector<cda_rail::ConflictPair>
cda_rail::RouteMap::get_crossing_overlaps(const std::string& train1,
                                          const std::string& train2,
                                          const Network&     network) const {
  auto       conflicts         = get_ttd_overlaps(train1, train2, network);
  const auto reverse_conflicts = get_reverse_overlaps(train1, train2, network);
  conflicts.insert(conflicts.end(), reverse_conflicts.begin(),
                   reverse_conflicts.end());
  std::ranges::sort(conflicts, {},
                    [](const ConflictPair& c) { return c.pos1.first; });

  std::vector<ConflictPair> result;
  if (conflicts.empty()) {
    return result;
  }

  result.emplace_back(conflicts.front());
  for (size_t i = 1; i < conflicts.size(); ++i) {
    auto&       prev = result.back();
    const auto& succ = conflicts.at(i);

    if (prev.pos1.second >= succ.pos1.first &&
        prev.pos2.second >= succ.pos2.first &&
        succ.pos2.second >= prev.pos2.first) {
      prev.pos1.second = std::max(prev.pos1.second, succ.pos1.second);
      prev.pos2.first  = std::min(prev.pos2.first, succ.pos2.first);
      prev.pos2.second = std::max(prev.pos2.second, succ.pos2.second);
      prev.edges.insert(succ.edges.begin(), succ.edges.end());
    } else {
      result.emplace_back(succ);
    }
  }

  return result;
}

/**
 * @brief Registers an empty route for a train.
 *
 * @throws InvalidInputException if the train already has a route.
 */

void cda_rail::RouteMap::add_empty_route(const std::string& train_name) {
  if (!m_routes.try_emplace(train_name).second) {
    throw exceptions::InvalidInputException("Train already has a route.");
  }
}

/**
 * @brief Adds an empty route for a train, validating it exists in the train list.
 *
 * @throws TrainNotExistentException if the train does not exist in trains.
 */
void cda_rail::RouteMap::add_empty_route(const std::string& train_name,
                                         const TrainList&   trains) {
  if (!trains.has_train(train_name)) {
    throw exceptions::TrainNotExistentException(train_name);
  }
  add_empty_route(train_name);
}

/**
 * @brief Appends an edge to the end of a train's route.
 *
 * @param train_name Name of the train whose route to extend.
 * @param new_edge The edge to append.
 * @param network The network to resolve and validate the edge.
 *
 * @throws ConsistencyException if the train has no route.
 * @throws ConsistencyException if the new edge is not a valid successor of the current last edge.
 */
void cda_rail::RouteMap::push_back_edge(const std::string&        train_name,
                                        Network::EdgeInput const& new_edge,
                                        const Network&            network) {
  throw_if_train_has_no_route(train_name);
  m_routes.at(train_name).push_back_edge(new_edge, network);
}

/**
 * @brief Adds an edge to the front of a train's route.
 *
 * @param train_name Name of the train whose route is to be modified.
 * @param new_edge The edge to prepend, specified by source and target vertex names.
 * @param network The network in which the edge and train route exist.
 */
void cda_rail::RouteMap::push_front_edge(const std::string&        train_name,
                                         Network::EdgeInput const& new_edge,
                                         const Network&            network) {
  throw_if_train_has_no_route(train_name);
  m_routes.at(train_name).push_front_edge(new_edge, network);
}

/**
 * @brief Removes the first edge from a train's route.
 */
```
void cda_rail::RouteMap::remove_first_edge(const std::string& train_name) {
  throw_if_train_has_no_route(train_name);
  m_routes.at(train_name).remove_first_edge();
}

/**
 * @brief Removes the last edge from a train's route.
 *
 * @param train_name The name of the train.
 *
 * @throws ConsistencyException if the train has no route or the route is empty.
 */
void cda_rail::RouteMap::remove_last_edge(const std::string& train_name) {
  throw_if_train_has_no_route(train_name);
  m_routes.at(train_name).remove_last_edge();
}

/**
 * @brief Removes the route for a train.
 *
 * @param train_name The name of the train.
 *
 * @throws ConsistencyException if the train does not have a route.
 */
void cda_rail::RouteMap::remove_route(const std::string& train_name) {
  throw_if_train_has_no_route(train_name);
  m_routes.erase(train_name);
}

/**
 * @brief Exports all routes to a JSON file in the specified directory.
 *
 * Writes `routes.json` with each train name mapped to an array of edge endpoints
 * (source and target vertex names).
 *
 * @throws ExportException if the directory cannot be created.
 */

void cda_rail::RouteMap::export_routes(const std::filesystem::path& p,
                                       const Network& network) const {
  if (!is_directory_and_create(p)) {
    throw exceptions::ExportException("Could not create directory " +
                                      p.string());
  }

  json j;
  for (const auto& [name, route] : m_routes) {
    const auto edge_pairs =
        route.get_edges() | std::views::transform([&network](size_t edge_id) {
          const auto& edge = network.get_edge(edge_id);
          return std::pair{network.get_vertex(edge.source).name,
                           network.get_vertex(edge.target).name};
        });
    // NOLINTNEXTLINE(*-pro-bounds-avoid-unchecked-container-access)
    j[name] = std::vector(edge_pairs.begin(), edge_pairs.end());
  }

  std::ofstream file(p / "routes.json");
  file << j << '\n';
}

/**
 * @brief Loads routes from a JSON file in the specified directory.
 *
 * Reads "routes.json" from the given path and constructs routes for each
 * train in the file. Each route is defined as an ordered sequence of edges
 * specified by pairs of vertex names.
 *
 * @param p Directory path containing "routes.json".
 * @param network Network used to resolve and validate edge endpoints.
 *
 * @throws ImportException If the path does not exist or is not a directory.
 */
cda_rail::RouteMap::RouteMap(const std::filesystem::path& p,
                             const Network&               network) {
  if (!std::filesystem::exists(p)) {
    throw exceptions::ImportException("Path does not exist.");
  }
  if (!std::filesystem::is_directory(p)) {
    throw exceptions::ImportException("Path is not a directory.");
  }

  std::ifstream file(p / "routes.json");
  const json    data = json::parse(file);

  for (const auto& [name, route] : data.items()) {
    add_empty_route(name);
    for (const auto& edge : route) {
      push_back_edge(
          name, {edge.at(0).get<std::string>(), edge.at(1).get<std::string>()},
          network);
    }
  }
}

/**
 * @brief Verifies that routes are consistent with the trains and network.
 *
 * @param every_train_must_have_route If `true`, every train must have a route.
 * @return `true` if all routes are valid and consistent, `false` otherwise.
 */

bool cda_rail::RouteMap::check_consistency(
    const TrainList& trains, const Network& network,
    bool every_train_must_have_route) const {
  if (every_train_must_have_route && m_routes.size() != trains.size()) {
    return false;
  }

  return std::ranges::all_of(m_routes, [&trains, &network](const auto& route) {
    return trains.has_train(route.first) &&
           route.second.check_consistency(network);
  });
}

/**
 * @brief Applies edge discretization to all routes.
 *
 * @param new_edges Mapping from original edge id to replacement edges after discretization.
 */
void cda_rail::RouteMap::update_after_discretization(
    const std::vector<std::pair<size_t, cda_rail::index_vector>>& new_edges) {
  for (auto& route : m_routes | std::views::values) {
    route.update_after_discretization(new_edges);
  }
}

/**
 * @brief Validates that a train has a route.
 *
 * @throws ConsistencyException if the train does not have a route.
 */

void cda_rail::RouteMap::throw_if_train_has_no_route(
    std::string const& train_name) const {
  if (!m_routes.contains(train_name)) {
    auto const error_message = concatenate_string_views(
        {"Train ", train_name, " does not have a route."});
    throw exceptions::ConsistencyException(error_message);
  }
}
