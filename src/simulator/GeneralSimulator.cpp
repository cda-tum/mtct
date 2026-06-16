#include "simulator/GeneralSimulator.hpp"

#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "EOMHelper.hpp"
#include "StringHelper.hpp"
#include "probleminstances/GeneralProblemInstance.hpp"

#include <algorithm>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

/**
 * @brief Constructs a GeneralSimulator with a problem instance and TTD sections.
 *
 * Validates that all edges referenced in the TTD sections exist in the instance's network,
 * and initializes internal containers for train routes, orderings, and stop positions.
 *
 * @param instance Problem instance containing the network, trains, and schedule.
 * @param ttd_sections TTD sections, each containing sets of edge indices.
 *
 * @throws EdgeNotExistentException if any edge in a TTD section does not exist in the network.
 */
cda_rail::simulator::GeneralSimulator::GeneralSimulator(
    std::shared_ptr<
        const cda_rail::instances::GeneralProblemInstanceWithScheduleAndRoutes>
                                     instance,
    std::vector<cda_rail::index_set> ttd_sections)
    : m_instance(std::move(instance)), m_ttd_sections(std::move(ttd_sections)) {
  check_ttd_sections(m_ttd_sections);

  m_train_edges.resize(m_instance->get_const_train_list().size());
  m_ttd_orders.resize(m_ttd_sections.size());
  m_vertex_orders.resize(m_instance->get_const_network().number_of_vertices());
  m_stop_positions.resize(m_instance->get_const_train_list().size());
}

/**
 * @brief Initializes a GeneralSimulator with routes, orderings, and stop positions.
 *
 * All inputs are validated against the problem instance before storage.
 *
 * @throws EdgeNotExistentException If TTD sections or train edges reference non-existent edges.
 * @throws InvalidInputException If data sizes or contents are inconsistent with the instance.
 * @throws ConsistencyException If stop positions do not align with scheduled stops or routes.
 */
cda_rail::simulator::GeneralSimulator::GeneralSimulator(
    std::shared_ptr<
        const cda_rail::instances::GeneralProblemInstanceWithScheduleAndRoutes>
                                        instance,
    std::vector<cda_rail::index_set>    ttd_sections,
    std::vector<cda_rail::index_vector> train_edges,
    std::vector<cda_rail::index_vector> ttd_orders,
    std::vector<cda_rail::index_vector> vertex_orders,
    std::vector<std::vector<double>>    stop_positions)
    : m_instance(std::move(instance)), m_ttd_sections(std::move(ttd_sections)),
      m_train_edges(std::move(train_edges)),
      m_ttd_orders(std::move(ttd_orders)),
      m_vertex_orders(std::move(vertex_orders)),
      m_stop_positions(std::move(stop_positions)) {
  check_ttd_sections(m_ttd_sections);
  check_train_edges(m_train_edges);
  check_ttd_orders(m_ttd_orders);
  check_vertex_orders(m_vertex_orders);
  check_stop_positions(m_stop_positions);
}

/**
           * @brief Initializes the simulator with a problem instance and time-distance-diagram sections.
           *
           * @param instance Problem instance with schedule and routes.
           * @param ttd_sections Edge sets defining time-distance-diagram sections.
           */
          cda_rail::simulator::GeneralSimulator::GeneralSimulator(
    instances::GeneralProblemInstanceWithScheduleAndRoutes& instance,
    std::vector<cda_rail::index_set>                        ttd_sections)
    : GeneralSimulator(
          std::make_shared<const cda_rail::instances::
                               GeneralProblemInstanceWithScheduleAndRoutes>(
              instance),
          std::move(ttd_sections)) {}

/**
           * @brief Initializes a simulator with the given problem instance, TTD sections, train routes, orderings, and stop positions.
           *
           * Wraps the provided instance reference in a shared pointer and delegates initialization to the main constructor,
           * which validates all inputs and initializes internal state.
           *
           * @param instance The problem instance (by reference).
           * @param ttd_sections Train-timing-dependent sections, specified as edge sets.
           * @param train_edges Ordered edge sequences for each train.
           * @param ttd_orders Train orderings for each TTD section.
           * @param vertex_orders Train orderings for each vertex.
           * @param stop_positions Stop positions (distances along route) for each train.
           */
          cda_rail::simulator::GeneralSimulator::GeneralSimulator(
    instances::GeneralProblemInstanceWithScheduleAndRoutes& instance,
    std::vector<cda_rail::index_set>                        ttd_sections,
    std::vector<cda_rail::index_vector>                     train_edges,
    std::vector<cda_rail::index_vector>                     ttd_orders,
    std::vector<cda_rail::index_vector>                     vertex_orders,
    std::vector<std::vector<double>>                        stop_positions)
    : GeneralSimulator(
          std::make_shared<const cda_rail::instances::
                               GeneralProblemInstanceWithScheduleAndRoutes>(
              instance),
          std::move(ttd_sections), std::move(train_edges),
          std::move(ttd_orders), std::move(vertex_orders),
          std::move(stop_positions)) {}

/**
 * @brief Retrieves the edges routed for a train.
 *
 * @param trainId The ID of the train.
 * @return Constant reference to the edge sequence for the specified train.
 * @throws if the specified train does not exist.
 */
const cda_rail::index_vector&
cda_rail::simulator::GeneralSimulator::get_train_edges_of_tr(
    size_t const trainId) const {
  m_instance->get_const_train_list().throw_if_train_not_exist(trainId);
  return m_train_edges.at(trainId);
}

/**
 * @brief Retrieves the train ordering for a specified TTD section.
 *
 * @param ttdIndex Index of the TTD section.
 * @return const cda_rail::index_vector& Ordered train indices for the TTD section.
 * @throws cda_rail::exceptions::InvalidInputException if ttdIndex is out of bounds.
 */
const cda_rail::index_vector&
cda_rail::simulator::GeneralSimulator::get_ttd_orders_of_ttd(
    size_t const ttdIndex) const {
  if (ttdIndex >= m_ttd_orders.size()) {
    throw cda_rail::exceptions::InvalidInputException(
        "TTD index out of bounds.");
  }
  return m_ttd_orders.at(ttdIndex);
}

/**
 * @brief Identifies which TTD section an edge belongs to.
 *
 * @param edge The edge to query.
 * @return The index of the TTD section containing the edge, or empty if the edge is not part of any TTD section.
 */
std::optional<size_t> cda_rail::simulator::GeneralSimulator::get_ttd(
    Network::EdgeInput const& edge) const {
  size_t const edge_id = m_instance->get_const_network().get_edge_index(edge);
  for (size_t ttd_index = 0; ttd_index < m_ttd_sections.size(); ++ttd_index) {
    const auto& ttd_section = m_ttd_sections.at(ttd_index);
    if (std::ranges::contains(ttd_section, edge_id)) {
      return ttd_index; // Found the TTD section containing the edge
    }
  }
  return {}; // Edge is not part of any TTD section
}

/**
 * @brief Retrieves the train ordering for a vertex.
 *
 * @return const cda_rail::index_vector& The vector of train indices ordered at the given vertex.
 */
const cda_rail::index_vector&
cda_rail::simulator::GeneralSimulator::get_vertex_orders_of_vertex(
    Network::VertexInput const& vertex) const {
  return m_vertex_orders.at(
      m_instance->get_const_network().resolve_vertex_index(vertex));
}

/**
 * @brief Retrieves the stop positions of a train.
 *
 * @param trainId The train identifier.
 * @return const std::vector<double>& A const reference to the train's stop positions.
 */
const std::vector<double>&
cda_rail::simulator::GeneralSimulator::get_stop_positions_of_tr(
    size_t const trainId) const {
  m_instance->get_const_train_list().throw_if_train_not_exist(trainId);
  return m_stop_positions.at(trainId);
}

/**
 * @brief Calculates the total length of all edges in a train's route.
 *
 * @param tr The train ID.
 * @return double The cumulative distance covered by the train's assigned edges.
 *
 * @throws std::exception if the train ID does not exist.
 */
double cda_rail::simulator::GeneralSimulator::train_edge_length(
    size_t const tr) const {
  m_instance->get_const_train_list().throw_if_train_not_exist(tr);
  return m_instance->get_const_network().length_of_path(m_train_edges.at(tr));
}

/**
 * @brief Gets the cumulative distance to the end of an edge in a train's route.
 *
 * @param train_id Identifier of the train.
 * @param edge_id Identifier of the edge.
 * @return double The cumulative distance from the route start to the end of the specified edge.
 *
 * @throws EdgeNotExistentException if the edge does not exist in the network.
 * @throws ConsistencyException if the edge is not found in the train's route.
 */
double
cda_rail::simulator::GeneralSimulator::get_edge_position(size_t train_id,
                                                         size_t edge_id) const {
  if (!m_instance->get_const_network().has_edge(edge_id)) {
    throw cda_rail::exceptions::EdgeNotExistentException(edge_id);
  }
  const auto& tr_edges = get_train_edges_of_tr(train_id);
  double      pos      = 0.0;
  for (const auto& edge : tr_edges) {
    pos += m_instance->get_const_network().get_edge(edge).length;
    if (edge == edge_id) {
      return pos;
    }
  }
  throw cda_rail::exceptions::ConsistencyException(
      "Edge " + std::to_string(edge_id) + " not found in train " +
      std::to_string(train_id) + "'s route.");
}

/**
 * @brief Retrieves the edge at a given distance along a train's route.
 *
 * @param train_id The train identifier.
 * @param position Distance along the route.
 * @return size_t The edge index at the given position.
 * @throws ConsistencyException if the train has no edges or the position exceeds the route length.
 */
size_t cda_rail::simulator::GeneralSimulator::get_edge_at_position(
    size_t train_id, double position) const {
  m_instance->get_const_train_list().throw_if_train_not_exist(train_id);
  exceptions::throw_if_negative(position, "Position");
  if (m_train_edges.at(train_id).empty()) {
    throw cda_rail::exceptions::ConsistencyException(
        "Train " + std::to_string(train_id) + " has no edges in its route.");
  }

  double current_pos = 0.0;
  for (auto const& edge : m_train_edges.at(train_id)) {
    current_pos += m_instance->get_const_network().get_edge(edge).length;
    if (current_pos >= position) {
      return edge;
    }
  }

  throw cda_rail::exceptions::ConsistencyException(
      "Position " + std::to_string(position) + " is not on any edge of train " +
      std::to_string(train_id) + ".");
}

/**
 * @brief Determines whether a route end is valid for the next stop.
 *
 * @param edges The edges forming the potential route end.
 * @return true if the route end is valid for the next stop, false otherwise.
 */
bool cda_rail::simulator::GeneralSimulator::is_route_end_valid_stop_pos(
    size_t tr, const cda_rail::index_vector& edges) const {
  m_instance->get_const_train_list().throw_if_train_not_exist(tr);

  const auto& tr_length =
      m_instance->get_const_train_list().get_train(tr).get_length();
  const auto& tr_schedule = m_instance->get_const_schedule(tr).get_stops();
  if (m_stop_positions.at(tr).size() >= tr_schedule.size()) {
    // All stops have been set, hence, no further stop is possible
    return false;
  }
  const auto& next_station =
      tr_schedule.at(m_stop_positions.at(tr).size()).get_station();

  double len = 0;
  for (auto it = edges.rbegin(); (len < tr_length) && (it != edges.rend());
       ++it) {
    if (!std::ranges::contains(next_station.tracks, *it)) {
      // Track does not belong to the next station
      return false;
    }
    len += m_instance->get_const_network().get_edge(*it).length;
  }

  return len >= tr_length;
}

/**
 * @brief Sets the edge routes for all trains.
 *
 * @param tr_edges Edge sequences for each train, indexed by train ID.
 *
 * @throws InvalidInputException if the number of routes does not match the number of trains.
 * @throws ConsistencyException if any route does not form a valid path.
 * @throws EdgeNotExistentException if any edge does not exist in the network.
 */
void cda_rail::simulator::GeneralSimulator::set_train_edges(
    std::vector<cda_rail::index_vector> tr_edges) {
  check_train_edges(tr_edges);
  m_train_edges = std::move(tr_edges);
}

/**
 * @brief Sets the ordered edge sequence for a train.
 *
 * @param train_id The train identifier.
 * @param edges The new ordered sequence of edge indices.
 *
 * @throws InvalidInputException if train_id does not exist.
 * @throws ConsistencyException if the edges do not form a valid path.
 * @throws EdgeNotExistentException if any edge does not exist in the network.
 */
void cda_rail::simulator::GeneralSimulator::set_train_edges_of_tr(
    size_t train_id, cda_rail::index_vector edges) {
  check_train_edges_for_tr(train_id, edges);
  m_train_edges.at(train_id) = std::move(edges);
}

/**
 * @brief Appends an edge to a train's route with successor validation.
 */
void cda_rail::simulator::GeneralSimulator::append_train_edge_to_tr(
    size_t train_id, size_t edge) {
  m_instance->get_const_train_list().throw_if_train_not_exist(train_id);
  if (m_train_edges.at(train_id).empty()) {
    check_train_edges_for_tr(train_id, cda_rail::index_vector{edge});
  } else {
    auto const& last_edge = m_train_edges.at(train_id).back();
    m_instance->get_const_network().throw_if_not_valid_successor(last_edge,
                                                                 edge);
  }
  m_train_edges.at(train_id).push_back(edge);
}

/**
 * @brief Sets the train ordering for all TTD sections.
 *
 * @param orders Train orderings indexed by TTD section, where each inner vector
 *               represents the order in which trains traverse that section.
 */
void cda_rail::simulator::GeneralSimulator::set_ttd_orders(
    std::vector<cda_rail::index_vector> orders) {
  check_ttd_orders(orders);
  m_ttd_orders = std::move(orders);
}

/**
 * @brief Sets the train ordering for a specific TTD section.
 *
 * @param ttd_index The index of the TTD section.
 * @param orders Vector of train IDs in the desired order for this TTD section.
 *
 * @throws InvalidInputException if ttd_index is out of bounds, if any train ID in orders does not exist, or if orders contains duplicate train IDs.
 */
void cda_rail::simulator::GeneralSimulator::set_ttd_orders_of_ttd(
    size_t ttd_index, cda_rail::index_vector orders) {
  if (ttd_index >= m_ttd_orders.size()) {
    throw cda_rail::exceptions::InvalidInputException(
        "TTD index out of bounds.");
  }
  check_if_all_tr_valid(orders);
  check_if_all_tr_unique(orders);
  m_ttd_orders.at(ttd_index) = std::move(orders);
}

/**
 * @brief Replaces the vertex orderings for all network vertices.
 *
 * @param orders Vertex orderings indexed by vertex, each containing an ordered list of train IDs.
 */
void cda_rail::simulator::GeneralSimulator::set_vertex_orders(
    std::vector<cda_rail::index_vector> orders) {
  check_vertex_orders(orders);
  m_vertex_orders = std::move(orders);
}

/**
 * @brief Sets the train order for a given vertex.
 *
 * @param vertex The vertex for which to set the train order.
 * @param orders The ordered sequence of train IDs. All trains must exist and be unique.
 *
 * @throws InvalidInputException if any train ID does not exist or if there are duplicates.
 */
void cda_rail::simulator::GeneralSimulator::set_vertex_orders_of_vertex(
    cda_rail::Network::VertexInput const& vertex,
    cda_rail::index_vector                orders) {
  check_if_all_tr_valid(orders);
  check_if_all_tr_unique(orders);
  m_vertex_orders.at(m_instance->get_const_network().resolve_vertex_index(
      vertex)) = std::move(orders);
}

/**
 * @brief Sets stop positions for all trains.
 *
 * @param positions A vector of stop position sequences, one per train. Each inner vector contains
 * cumulative distances along the train's route where stops occur.
 */
void cda_rail::simulator::GeneralSimulator::set_stop_positions(
    std::vector<std::vector<double>> positions) {
  check_stop_positions(positions);
  m_stop_positions = std::move(positions);
}

/**
 * @brief Sets the stop positions for a train.
 */
void cda_rail::simulator::GeneralSimulator::set_stop_positions_of_tr(
    size_t train_id, std::vector<double> positions) {
  check_stop_positions_for_tr(train_id, positions);
  m_stop_positions.at(train_id) = std::move(positions);
}

/**
 * @brief Appends a stop position for a train along its route.
 *
 * Adds a position at which the train will stop. The position must be non-negative,
 * must be non-decreasing relative to previously added stops, and the total stops
 * cannot exceed the train's scheduled stop count.
 *
 * @param position The cumulative distance along the train's route.
 *
 * @throws ConsistencyException if all scheduled stops are already set or if 
 *         position is less than the last recorded stop position.
 */
void cda_rail::simulator::GeneralSimulator::append_stop_position_to_tr(
    size_t train_id, double position) {
  m_instance->get_const_train_list().throw_if_train_not_exist(train_id);
  cda_rail::exceptions::throw_if_negative(position, "Stop position");

  if (m_stop_positions.at(train_id).size() >=
      m_instance->get_const_schedule(train_id).get_stops().size()) {
    throw cda_rail::exceptions::ConsistencyException(concatenate_string_views(
        {"All scheduled stops for train ",
         m_instance->get_const_train_list().get_train(train_id).get_name(),
         " are already set."}));
  }

  if (!m_stop_positions.at(train_id).empty() &&
      position < m_stop_positions.at(train_id).back()) {
    throw cda_rail::exceptions::ConsistencyException(concatenate_string_views(
        {"Stop positions must be non-decreasing for train ",
         m_instance->get_const_train_list().get_train(train_id).get_name(),
         ". Last position is ",
         std::to_string(m_stop_positions.at(train_id).back()),
         ", new position is ", std::to_string(position), "."}));
  }

  m_stop_positions.at(train_id).push_back(position);
}

/**
 * @brief Appends a stop position for a train at a specified edge.
 *
 * Validates that the edge is a valid track for the next scheduled station,
 * then appends the corresponding stop position.
 *
 * @param train_id The index of the train.
 * @param edge The index of the edge where the stop should be appended.
 * @throws ConsistencyException if all scheduled stops are already set,
 *   or if the edge is not a valid track for the next scheduled station.
 */
void cda_rail::simulator::GeneralSimulator::append_stop_edge_to_tr(
    size_t train_id, size_t edge) {
  const auto& stop_positions_of_tr = get_stop_positions_of_tr(train_id);
  const auto& tr_stops = m_instance->get_const_schedule(train_id).get_stops();
  if (stop_positions_of_tr.size() >= tr_stops.size()) {
    throw cda_rail::exceptions::ConsistencyException(concatenate_string_views(
        {"All scheduled stops for train ",
         m_instance->get_const_train_list().get_train(train_id).get_name(),
         " are already set."}));
  }

  const auto& next_stop =
      tr_stops.at(stop_positions_of_tr.size()).get_station();
  if (!std::ranges::contains(next_stop.tracks, edge)) {
    throw cda_rail::exceptions::ConsistencyException(concatenate_string_views(
        {"Edge ", m_instance->get_const_network().get_edge_name(edge),
         " is not a valid stop edge for train ",
         m_instance->get_const_train_list().get_train(train_id).get_name(),
         ". Next stop is ", next_stop.name, "."}));
  }
  append_stop_position_to_tr(train_id, get_edge_position(train_id, edge));
}

/**
 * @brief Appends a stop position at the train's current route endpoint.
 *
 * @param train_id The identifier of the train.
 *
 * @throws ConsistencyException if the train's route is empty.
 */
void cda_rail::simulator::GeneralSimulator::append_current_stop_position_of_tr(
    size_t train_id) {
  const auto& tr_edges = get_train_edges_of_tr(train_id);
  if (tr_edges.empty()) {
    throw cda_rail::exceptions::ConsistencyException(concatenate_string_views(
        {"Train ",
         m_instance->get_const_train_list().get_train(train_id).get_name(),
         " has no edges in its route. Cannot append current stop position."}));
  }
  append_stop_edge_to_tr(train_id, tr_edges.back());
}

/**
 * @brief Determines whether the simulation has reached a terminal state.
 *
 * @return `true` if all trains have non-empty routes, all scheduled stops are assigned, and each train's route ends at its scheduled exit vertex; `false` otherwise.
 */
bool cda_rail::simulator::GeneralSimulator::is_final_state() const {
  for (size_t tr = 0; tr < m_instance->get_const_train_list().size(); ++tr) {
    if (m_train_edges.at(tr).empty()) {
      return false;
    }
    if (m_instance->get_const_schedule(tr).get_stops().size() !=
        m_stop_positions.at(tr).size()) {
      return false; // Not all stops have been set for the train
    }
    if (m_instance->get_const_network()
            .get_edge(m_train_edges.at(tr).back())
            .target != m_instance->get_const_schedule(tr).get_exit_vertex()) {
      return false; // Train has not reached the end of its route
    }
  }
  return true; // All trains have reached the end of their route and all stops
  // have been set
}

/**
 * @brief Computes the braking distance for a train at a given velocity.
 *
 * Returns 0.0 for non-positive velocities.
 *
 * @param tr The train ID.
 * @param v The velocity.
 * @return double The braking distance; 0.0 for velocities ≤ 0.
 */
double
cda_rail::simulator::GeneralSimulator::tr_braking_distance(size_t tr,
                                                           double v) const {
  /**
   * Calculates the braking distance for a train with id `tr` at velocity `v`.
   *
   * @param tr: The id of the train for which the braking distance is
   * calculated.
   * @param v: The velocity at which the braking distance is calculated.
   *
   * @return: The braking distance for the train at the given velocity.
   */
  m_instance->get_const_train_list().throw_if_train_not_exist(tr);
  cda_rail::exceptions::throw_if_less_than(v, -EPS, "Velocity");

  if (v <= 0) {
    return 0.0; // No braking distance if the train is not moving
  }
  return cda_rail::braking_distance(
      v, m_instance->get_const_train_list().get_train(tr).get_deceleration());
}

/**
 * @brief Computes cumulative distances along a train's routed edges.
 *
 * @param tr The ID of the train.
 * @return A vector of milestones where element `i` represents the cumulative
 * distance from the route start to the beginning of edge `i`. The first element
 * is always 0.0. Returns an empty vector if the train has no routed edges.
 */
std::vector<double>
cda_rail::simulator::GeneralSimulator::edge_milestones(size_t tr) const {
  /**
   * This function returns the individual milestones, i.e., the distance on
   * the individual route to each edges starting point. The last value is the
   * distance to the exit node.
   *
   * @param tr: The id of the train for which the milestones are calculated.
   * @return: A vector of doubles with the milestones for each edge of the
   * train.
   */
  m_instance->get_const_train_list().throw_if_train_not_exist(tr);

  const auto& edges = m_train_edges.at(tr);
  if (edges.empty()) {
    return {}; // No edges, no milestones
  }
  std::vector<double> milestones;
  milestones.reserve(edges.size() + 1);
  milestones.emplace_back(0.0); // First milestone is always 0
  for (const auto& edge_id : edges) {
    const auto& edge = m_instance->get_const_network().get_edge(edge_id);
    milestones.emplace_back(milestones.back() + edge.length);
  }
  return milestones;
}

/**
 * @brief Determines if an edge is part of a train's route.
 *
 * @param tr Train identifier.
 * @param edge_id Edge identifier.
 * @return `true` if the edge is on the train's route, `false` otherwise.
 *
 * @throw cda_rail::exceptions::EdgeNotExistentException if the edge does not exist in the network.
 */
bool cda_rail::simulator::GeneralSimulator::is_on_route(size_t tr,
                                                        size_t edge_id) const {
  m_instance->get_const_train_list().throw_if_train_not_exist(tr);
  if (!m_instance->get_const_network().has_edge(edge_id)) {
    throw cda_rail::exceptions::EdgeNotExistentException(edge_id);
  }

  const auto& tr_edges = m_train_edges.at(tr);
  return std::ranges::contains(tr_edges, edge_id);
}

/**
 * @brief Maps trains to each network edge.
 *
 * @return A vector of unordered sets, indexed by edge ID. Each set contains the train
 * indices routed on that edge.
 */
std::vector<std::unordered_set<size_t>>
cda_rail::simulator::GeneralSimulator::tr_on_edges() const {
  /**
   * This function returns a vector of unordered sets, where each set
   * contains the indices of trains that are routed on a specific edge.
   */

  std::vector<std::unordered_set<size_t>> trains_on_edges(
      m_instance->get_const_network().number_of_edges());

  for (size_t tr = 0; tr < m_instance->get_const_train_list().size(); ++tr) {
    const auto& edges = m_train_edges.at(tr);
    for (const auto& edge_id : edges) {
      trains_on_edges.at(edge_id).insert(tr);
    }
  }
  return trains_on_edges;
}

/**
 * @brief Validates that all edges in the given TTD sections exist in the network.
 *
 * @param ttd_sections Vector of TTD sections, each containing edge IDs.
 * @throws EdgeNotExistentException if any edge does not exist in the network.
 */
void cda_rail::simulator::GeneralSimulator::check_ttd_sections(
    std::vector<cda_rail::index_set> const& ttd_sections) const {
  for (auto const& section : ttd_sections) {
    for (auto const& edge : section) {
      if (!m_instance->get_const_network().has_edge(edge)) {
        throw cda_rail::exceptions::EdgeNotExistentException(edge);
      }
    }
  }
}

/**
 * @brief Validates that a sequence of edges is compatible with a train's schedule entry and edge successors.
 *
 * @param tr Train ID.
 * @param edges Sequence of edge indices to validate.
 * @throws ConsistencyException if validation fails.
 */
void cda_rail::simulator::GeneralSimulator::check_train_edges_for_tr(
    size_t tr, cda_rail::index_vector const& edges) const {
  m_instance->get_const_train_list().throw_if_train_not_exist(tr);

  if (edges.empty()) {
    return;
  }

  auto const& first_edge = edges.front();
  auto const& entry = m_instance->get_const_schedule(tr).get_entry_vertex();
  if (entry != m_instance->get_const_network().get_edge(first_edge).source) {
    throw cda_rail::exceptions::ConsistencyException(concatenate_string_views(
        {"The first edge (",
         m_instance->get_const_network().get_edge_name(first_edge),
         ") for train ",
         m_instance->get_const_train_list().get_train(tr).get_name(),
         " does not start at the entry vertex (",
         m_instance->get_const_network().get_vertex(entry).name,
         ") specified in the schedule."}));
  }

  for (size_t i = 1; i < edges.size(); ++i) {
    auto const& prev_edge = edges.at(i - 1);
    auto const& curr_edge = edges.at(i);
    m_instance->get_const_network().throw_if_not_valid_successor(prev_edge,
                                                                 curr_edge);
  }
}

/**
 * @brief Validates train edge routes for correct count and connectivity.
 *
 * Ensures the train edges vector size matches the number of trains in the instance 
 * and that each train's edge sequence forms a valid connected path according to 
 * the network topology and schedule.
 *
 * @param train_edges Vector of edge sequences, one per train.
 *
 * @throws InvalidInputException if the vector size does not match the number of trains.
 */
void cda_rail::simulator::GeneralSimulator::check_train_edges(
    std::vector<cda_rail::index_vector> const& train_edges) const {
  if (train_edges.size() != m_instance->get_const_train_list().size()) {
    throw cda_rail::exceptions::InvalidInputException(
        "Size of train edges vector does not match the number of trains in "
        "the instance.");
  }

  for (size_t tr = 0; tr < train_edges.size(); ++tr) {
    check_train_edges_for_tr(tr, train_edges.at(tr));
  }
}

/**
 * @brief Ensures all train identifiers are valid.
 *
 * Throws if any train identifier does not exist in the instance.
 */
void cda_rail::simulator::GeneralSimulator::check_if_all_tr_valid(
    cda_rail::index_vector const& train_ids) const {
  for (auto const& tr : train_ids) {
    m_instance->get_const_train_list().throw_if_train_not_exist(tr);
  }
}

/**
 * @brief Validates that all train IDs are unique.
 *
 * @throws InvalidInputException if any train ID appears more than once.
 */
void cda_rail::simulator::GeneralSimulator::check_if_all_tr_unique(
    cda_rail::index_vector const& train_ids) {
  std::unordered_set<size_t> seen_trains;
  for (auto const& tr : train_ids) {
    if (!seen_trains.insert(tr).second) {
      throw cda_rail::exceptions::InvalidInputException(
          "Train order contains duplicate train IDs.");
    }
  }
}

/**
 * @brief Validates TTD section orderings.
 *
 * Ensures that the number of orderings matches the number of TTD sections, and that each ordering contains only valid, unique train IDs.
 *
 * @param ttd_orders Orderings of trains for each TTD section.
 *
 * @throws InvalidInputException if the size of ttd_orders does not match the number of TTD sections, or if any ordering contains invalid or duplicate train IDs.
 */
void cda_rail::simulator::GeneralSimulator::check_ttd_orders(
    std::vector<cda_rail::index_vector> const& ttd_orders) const {
  if (ttd_orders.size() != m_ttd_sections.size()) {
    throw cda_rail::exceptions::InvalidInputException(
        "Size of ttd_orders does not match number of ttd sections in "
        "instance.");
  }
  for (auto const& ttd_order : ttd_orders) {
    check_if_all_tr_valid(ttd_order);
    check_if_all_tr_unique(ttd_order);
  }
}

/**
 * @brief Ensures vertex orders match the network size and contain valid, unique train IDs.
 *
 * @throws InvalidInputException If the size does not match the number of vertices,
 *         or if any ordering contains invalid or duplicate train IDs.
 */
void cda_rail::simulator::GeneralSimulator::check_vertex_orders(
    std::vector<cda_rail::index_vector> const& vertex_orders) const {
  if (vertex_orders.size() !=
      m_instance->get_const_network().number_of_vertices()) {
    throw cda_rail::exceptions::InvalidInputException(
        "Size of vertex_orders does not match number of vertices in "
        "instance.");
  }
  for (auto const& vertex_order : vertex_orders) {
    check_if_all_tr_valid(vertex_order);
    check_if_all_tr_unique(vertex_order);
  }
}

/**
 * @brief Validates that stop positions for a train conform to its schedule.
 *
 * Verifies that the given positions are non-decreasing, do not exceed the
 * number of scheduled stops, and each position falls on a valid track for
 * its corresponding scheduled station.
 *
 * @param tr Train identifier.
 * @param positions Stop position values along the train route.
 * @throws InvalidInputException If the number of positions exceeds scheduled
 *         stops, positions are not sorted in non-decreasing order, or any
 *         position falls outside its scheduled station's valid tracks.
 */
void cda_rail::simulator::GeneralSimulator::check_stop_positions_for_tr(
    size_t tr, std::vector<double> const& positions) const {
  m_instance->get_const_train_list().throw_if_train_not_exist(tr);

  auto const& scheduled_stops_tr =
      m_instance->get_const_schedule(tr).get_stops();
  if (positions.size() > scheduled_stops_tr.size()) {
    throw cda_rail::exceptions::InvalidInputException(concatenate_string_views(
        {"Number of stop positions for train ",
         m_instance->get_const_train_list().get_train(tr).get_name(),
         " exceeds the number of scheduled stops for this train."}));
  }
  if (!std::ranges::is_sorted(positions)) {
    throw cda_rail::exceptions::InvalidInputException(concatenate_string_views(
        {"Stop positions for train ",
         m_instance->get_const_train_list().get_train(tr).get_name(),
         " are not sorted in non-decreasing order."}));
  }
  for (size_t stop_idx = 0; stop_idx < positions.size(); ++stop_idx) {
    auto const& station = scheduled_stops_tr.at(stop_idx).get_station();
    auto const  tr_edge = get_edge_at_position(tr, positions.at(stop_idx));
    if (!std::ranges::contains(station.tracks, tr_edge)) {
      throw cda_rail::exceptions::InvalidInputException(
          concatenate_string_views(
              {"Stop position for train ",
               m_instance->get_const_train_list().get_train(tr).get_name(),
               " at stop index ", std::to_string(stop_idx),
               " is located on edge ",
               m_instance->get_const_network().get_edge_name(tr_edge),
               " which is not part of ", station.name}));
    }
  }
}

/**
 * @brief Validates that stop positions are structured correctly for all trains.
 *
 * Checks that the outer vector has one entry per train in the instance
 * and that each train's stop positions are valid.
 *
 * @param stop_positions Stop positions for each train.
 *
 * @throws InvalidInputException if the number of entries does not match the number of trains.
 */
void cda_rail::simulator::GeneralSimulator::check_stop_positions(
    std::vector<std::vector<double>> const& stop_positions) const {
  if (stop_positions.size() != m_instance->get_const_train_list().size()) {
    throw cda_rail::exceptions::InvalidInputException(
        "Size of stop_positions does not match number of trains in "
        "instance.");
  }

  for (size_t tr = 0; tr < m_instance->get_const_train_list().size(); ++tr) {
    check_stop_positions_for_tr(tr, stop_positions.at(tr));
  }
}
