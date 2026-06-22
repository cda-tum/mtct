#include "simulator/GeneralSimulator.hpp"

#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "EOMHelper.hpp"
#include "StringHelper.hpp"
#include "probleminstances/GeneralProblemInstance.hpp"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <functional>
#include <iterator>
#include <memory>
#include <numeric>
#include <optional>
#include <ranges>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

bool cda_rail::simulator::SimulatorState::operator==(
    const SimulatorState& other) const {
  return train_edges == other.train_edges && ttd_orders == other.ttd_orders &&
         vertex_orders == other.vertex_orders &&
         stop_positions == other.stop_positions;
}

bool cda_rail::simulator::SimulatorState::operator>(
    const SimulatorState& other) const {
  auto get_obj = [](const auto& edges) {
    const auto edge_sizes =
        edges | std::views::transform([](const auto& e) { return e.size(); });
    return std::accumulate(edge_sizes.begin(), edge_sizes.end(), 0.0,
                           std::plus<>{});
  };

  return get_obj(train_edges) > get_obj(other.train_edges);
}

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

cda_rail::simulator::GeneralSimulator::GeneralSimulator(
    instances::GeneralProblemInstanceWithScheduleAndRoutes& instance,
    std::vector<cda_rail::index_set>                        ttd_sections)
    : GeneralSimulator(
          std::make_shared<const cda_rail::instances::
                               GeneralProblemInstanceWithScheduleAndRoutes>(
              instance),
          std::move(ttd_sections)) {}

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

const cda_rail::index_vector&
cda_rail::simulator::GeneralSimulator::get_train_edges_of_tr(
    size_t const trainId) const {
  m_instance->get_const_train_list().throw_if_train_not_exist(trainId);
  return m_train_edges.at(trainId);
}

const cda_rail::index_vector&
cda_rail::simulator::GeneralSimulator::get_ttd_orders_of_ttd(
    size_t const ttdIndex) const {
  if (ttdIndex >= m_ttd_orders.size()) {
    throw cda_rail::exceptions::InvalidInputException(
        "TTD index out of bounds.");
  }
  return m_ttd_orders.at(ttdIndex);
}

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

const cda_rail::index_vector&
cda_rail::simulator::GeneralSimulator::get_vertex_orders_of_vertex(
    Network::VertexInput const& vertex) const {
  return m_vertex_orders.at(
      m_instance->get_const_network().resolve_vertex_index(vertex));
}

const std::vector<double>&
cda_rail::simulator::GeneralSimulator::get_stop_positions_of_tr(
    size_t const trainId) const {
  m_instance->get_const_train_list().throw_if_train_not_exist(trainId);
  return m_stop_positions.at(trainId);
}

double cda_rail::simulator::GeneralSimulator::train_edge_length(
    size_t const tr) const {
  m_instance->get_const_train_list().throw_if_train_not_exist(tr);
  return m_instance->get_const_network().length_of_path(m_train_edges.at(tr));
}

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

void cda_rail::simulator::GeneralSimulator::set_simulator_state(
    SimulatorState state) {
  set_train_edges(std::move(state.train_edges));
  set_stop_positions(std::move(state.stop_positions));
  set_ttd_orders(std::move(state.ttd_orders));
  set_vertex_orders(std::move(state.vertex_orders));
}

void cda_rail::simulator::GeneralSimulator::set_train_edges(
    std::vector<cda_rail::index_vector> tr_edges) {
  check_train_edges(tr_edges);
  m_train_edges = std::move(tr_edges);
}

void cda_rail::simulator::GeneralSimulator::set_train_edges_of_tr(
    size_t train_id, cda_rail::index_vector edges) {
  check_train_edges_for_tr(train_id, edges);
  m_train_edges.at(train_id) = std::move(edges);
}

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

void cda_rail::simulator::GeneralSimulator::set_ttd_orders(
    std::vector<cda_rail::index_vector> orders) {
  check_ttd_orders(orders);
  m_ttd_orders = std::move(orders);
}

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

void cda_rail::simulator::GeneralSimulator::set_vertex_orders(
    std::vector<cda_rail::index_vector> orders) {
  check_vertex_orders(orders);
  m_vertex_orders = std::move(orders);
}

void cda_rail::simulator::GeneralSimulator::set_vertex_orders_of_vertex(
    cda_rail::Network::VertexInput const& vertex,
    cda_rail::index_vector                orders) {
  check_if_all_tr_valid(orders);
  check_if_all_tr_unique(orders);
  m_vertex_orders.at(m_instance->get_const_network().resolve_vertex_index(
      vertex)) = std::move(orders);
}

void cda_rail::simulator::GeneralSimulator::set_stop_positions(
    std::vector<std::vector<double>> positions) {
  check_stop_positions(positions);
  m_stop_positions = std::move(positions);
}

void cda_rail::simulator::GeneralSimulator::set_stop_positions_of_tr(
    size_t train_id, std::vector<double> positions) {
  check_stop_positions_for_tr(train_id, positions);
  m_stop_positions.at(train_id) = std::move(positions);
}

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

double
cda_rail::simulator::GeneralSimulator::tr_braking_distance(size_t tr,
                                                           double v) const {
  m_instance->get_const_train_list().throw_if_train_not_exist(tr);
  cda_rail::exceptions::throw_if_less_than(v, -EPS, "Velocity");

  if (v <= 0) {
    return 0.0; // No braking distance if the train is not moving
  }
  return cda_rail::braking_distance(
      v, m_instance->get_const_train_list().get_train(tr).get_deceleration());
}

std::vector<double>
cda_rail::simulator::GeneralSimulator::edge_milestones(size_t tr) const {
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

bool cda_rail::simulator::GeneralSimulator::is_on_route(size_t tr,
                                                        size_t edge_id) const {
  m_instance->get_const_train_list().throw_if_train_not_exist(tr);
  if (!m_instance->get_const_network().has_edge(edge_id)) {
    throw cda_rail::exceptions::EdgeNotExistentException(edge_id);
  }

  const auto& tr_edges = m_train_edges.at(tr);
  return std::ranges::contains(tr_edges, edge_id);
}

std::vector<std::unordered_set<size_t>>
cda_rail::simulator::GeneralSimulator::tr_on_edges() const {
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

cda_rail::index_set
cda_rail::simulator::GeneralSimulator::trains_on_path_helper(
    cda_rail::index_vector const&              edges,
    std::vector<cda_rail::index_vector> const& tr_edges) {
  // Set of trains where edges is exact subsequence of tr_edges.at(tr)
  if (edges.empty()) {
    return {};
  }

  cda_rail::index_set retval;
  auto                matching_indices =
      std::views::iota(size_t{0}, tr_edges.size()) |
      std::views::filter([&tr_edges, &edges](size_t const tr) {
        return std::ranges::contains_subrange(tr_edges.at(tr), edges);
      });

  std::ranges::copy(matching_indices, std::inserter(retval, retval.end()));
  return retval;
}

cda_rail::index_set cda_rail::simulator::GeneralSimulator::trains_on_path(
    cda_rail::index_vector const&              edges,
    std::vector<cda_rail::index_vector> const& tr_edges, Network const& network,
    bool also_reverse_path) {
  auto retval = trains_on_path_helper(edges, tr_edges);
  if (also_reverse_path) {
    if (auto const reverse_path = network.get_reverse_path(edges);
        reverse_path.has_value()) {
      auto const tr_on_reverse_path =
          trains_on_path_helper(reverse_path.value(), tr_edges);
      std::ranges::copy(tr_on_reverse_path,
                        std::inserter(retval, retval.end()));
    }
  }
  return retval;
}

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

void cda_rail::simulator::GeneralSimulator::check_if_all_tr_valid(
    cda_rail::index_vector const& train_ids) const {
  for (auto const& tr : train_ids) {
    m_instance->get_const_train_list().throw_if_train_not_exist(tr);
  }
}

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

std::size_t std::hash<cda_rail::simulator::SimulatorState>::operator()(
    const cda_rail::simulator::SimulatorState& state) const noexcept {
  // Based on boost::hash_combine implementation

  size_t seed         = 0;
  auto   hash_combine = [&seed](size_t h) {
    seed ^= h + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  };

  hash_combine(std::hash<size_t>{}(state.train_edges.size()));
  for (const auto& vec : state.train_edges) {
    hash_combine(std::hash<size_t>{}(vec.size()));
    for (const size_t v : vec) {
      hash_combine(std::hash<size_t>{}(v));
    }
  }
  hash_combine(std::hash<size_t>{}(state.ttd_orders.size()));
  for (const auto& vec : state.ttd_orders) {
    hash_combine(std::hash<size_t>{}(vec.size()));
    for (const size_t v : vec) {
      hash_combine(std::hash<size_t>{}(v));
    }
  }
  hash_combine(std::hash<size_t>{}(state.vertex_orders.size()));
  for (const auto& vec : state.vertex_orders) {
    hash_combine(std::hash<size_t>{}(vec.size()));
    for (const size_t v : vec) {
      hash_combine(std::hash<size_t>{}(v));
    }
  }
  hash_combine(std::hash<size_t>{}(state.stop_positions.size()));
  for (const auto& vec : state.stop_positions) {
    hash_combine(std::hash<size_t>{}(vec.size()));
    for (const double v : vec) {
      hash_combine(std::hash<double>{}(v));
    }
  }

  return seed;
}
