#include "datastructure/RailwayNetwork.hpp"

#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "GeneralHelper.hpp"
#include "VSSModel.hpp"
#include "nlohmann/json.hpp"
#include "nlohmann/json_fwd.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <functional>
#include <ios>
#include <limits>
#include <numeric>
#include <optional>
#include <queue>
#include <ranges>
#include <sstream>
#include <stack>
#include <string>
#include <tinyxml2.h>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

using json = nlohmann::json;

// -----------------------------
// CONSTRUCTORS
// -----------------------------

cda_rail::Network::Network(std::string_view const       networkName,
                           const std::filesystem::path& working_directory) {
  auto const path = working_directory / "networks" / networkName;
  if (!std::filesystem::exists(path)) {
    throw exceptions::ImportException("Path " + path.string() +
                                      " does not exist");
  }
  if (!std::filesystem::is_directory(path)) {
    throw exceptions::ImportException("Path " + path.string() +
                                      " is not a directory");
  }
  this->read_graphml(path);
  this->read_successors(path);
}

// -----------------------------
// IMPORT / EXPORT
// -----------------------------

void cda_rail::Network::export_network(
    const std::filesystem::path& working_directory) const {
  auto const path = working_directory / "networks" / m_network_name;
  if (!is_directory_and_create(path)) {
    throw exceptions::ExportException("Could not create directory " +
                                      path.string());
  }
  export_graphml(path);
  export_successors_cpp(path);
  export_successors_python(path);
}

// -----------------------------
// GETTER
// -----------------------------

bool cda_rail::Network::has_edge_helper(size_t source_id,
                                        size_t target_id) const {
  if (!has_vertex(source_id)) {
    throw exceptions::VertexNotExistentException(source_id);
  }
  if (!has_vertex(target_id)) {
    throw exceptions::VertexNotExistentException(target_id);
  }
  return std::ranges::any_of(m_edges, [&source_id, &target_id](const auto& e) {
    return e.source == source_id && e.target == target_id;
  });
}

bool cda_rail::Network::is_valid_successor_helper(size_t e0, size_t e1) const {
  if (!has_edge(e0)) {
    throw exceptions::EdgeNotExistentException(e0);
  }
  if (!has_edge(e1)) {
    throw exceptions::EdgeNotExistentException(e1);
  }
  return m_edges[e0].target == m_edges[e1].source &&
         m_successors.at(e0).contains(e1);
}

bool cda_rail::Network::is_adjustable_helper(size_t vertex_id) const {
  if (!has_vertex(vertex_id)) {
    throw exceptions::VertexNotExistentException(vertex_id);
  }
  return m_vertices[vertex_id].type == VertexType::NoBorder &&
         neighbors(vertex_id).size() == 2;
}

bool cda_rail::Network::is_consistent_for_transformation() const {
  for (size_t i = 0; i < number_of_edges(); ++i) {
    const auto& edge     = get_edge(i);
    const auto  src_type = get_vertex(edge.source).type;
    const auto  tgt_type = get_vertex(edge.target).type;

    if (edge.breakable && edge.min_block_length <= 0) {
      return false;
    }
    if (edge.breakable && (src_type == VertexType::NoBorder ||
                           tgt_type == VertexType::NoBorder)) {
      return false;
    }
    if (has_edge(edge.target, edge.source)) {
      const auto& rev = get_edge({edge.target, edge.source});
      if (edge.breakable != rev.breakable || edge.length != rev.length) {
        return false;
      }
    }
  }

  for (size_t i = 0; i < number_of_vertices(); ++i) {
    if (get_vertex(i).type != VertexType::NoBorderVSS) {
      continue;
    }
    const auto& nbrs = neighbors(i);
    if (nbrs.size() > 2) {
      return false;
    }
    if (std::ranges::any_of(nbrs, [this](size_t j) {
          return get_vertex(j).type == VertexType::NoBorder;
        })) {
      return false;
    }
  }
  return true;
}

cda_rail::index_vector
cda_rail::Network::get_vertices_by_type(VertexType type) const {
  auto indices = std::views::iota(size_t{0}, m_vertices.size()) |
                 std::views::filter([this, type](size_t i) {
                   return m_vertices[i].type == type;
                 });
  return {indices.begin(), indices.end()};
}

std::pair<size_t, double>
cda_rail::Network::get_old_edge_helper(size_t new_edge) const {
  if (!has_edge(new_edge)) {
    throw exceptions::EdgeNotExistentException(new_edge);
  }
  auto it = m_new_edge_to_old_edge_after_transform.find(new_edge);
  return it != m_new_edge_to_old_edge_after_transform.end()
             ? it->second
             : std::pair{new_edge, 0.0};
}

size_t cda_rail::Network::get_vertex_index(std::string_view const name) const {
  if (!has_vertex(name)) {
    throw exceptions::VertexNotExistentException(name);
  }
  return m_vertex_name_to_index.at(std::string{name});
}

const cda_rail::Vertex&
cda_rail::Network::get_vertex_helper(size_t index) const {
  if (!has_vertex(index)) {
    throw exceptions::VertexNotExistentException(index);
  }
  return m_vertices[index];
}

size_t cda_rail::Network::get_edge_index_helper(size_t source_id,
                                                size_t target_id) const {
  if (!has_vertex(source_id)) {
    throw exceptions::VertexNotExistentException(source_id);
  }
  if (!has_vertex(target_id)) {
    throw exceptions::VertexNotExistentException(target_id);
  }
  for (size_t i = 0; i < m_edges.size(); ++i) {
    if (m_edges[i].source == source_id && m_edges[i].target == target_id) {
      return i;
    }
  }
  throw exceptions::EdgeNotExistentException(get_vertex(source_id).name,
                                             get_vertex(target_id).name);
}

const cda_rail::Edge& cda_rail::Network::get_edge_helper(size_t index) const {
  if (!has_edge(index)) {
    throw exceptions::EdgeNotExistentException(index);
  }
  return m_edges[index];
}

std::string cda_rail::Network::get_edge_name_helper(
    std::string_view v1, std::string_view v2, bool const checkExistence) const {
  if (checkExistence && !has_vertex(v1)) {
    throw exceptions::VertexNotExistentException(v1);
  }
  if (checkExistence && !has_vertex(v2)) {
    throw exceptions::VertexNotExistentException(v2);
  }
  if (checkExistence && !has_edge(v1, v2)) {
    throw exceptions::EdgeNotExistentException(v1, v2);
  }
  return std::string(v1).append("-").append(v2);
}

cda_rail::index_set cda_rail::Network::out_edges_helper(size_t index) const {
  if (!has_vertex(index)) {
    throw exceptions::VertexNotExistentException(index);
  }
  cda_rail::index_set result;
  for (size_t i = 0; i < m_edges.size(); ++i) {
    if (m_edges[i].source == index) {
      result.insert(i);
    }
  }
  return result;
}

cda_rail::index_set
cda_rail::Network::in_edges_helper(size_t const index) const {
  if (!has_vertex(index)) {
    throw exceptions::VertexNotExistentException(index);
  }
  cda_rail::index_set result;
  for (size_t i = 0; i < m_edges.size(); ++i) {
    if (m_edges[i].target == index) {
      result.insert(i);
    }
  }
  return result;
}

cda_rail::index_set
cda_rail::Network::neighboring_edges_helper(size_t const index) const {
  auto       result = in_edges(index);
  const auto outs   = out_edges(index);
  result.insert(outs.begin(), outs.end());
  return result;
}

cda_rail::index_set
cda_rail::Network::get_predecessors_helper(size_t index) const {
  if (!has_edge(index)) {
    throw exceptions::EdgeNotExistentException(index);
  }
  cda_rail::index_set result;
  for (const auto e1 : in_edges(get_edge(index).source)) {
    if (is_valid_successor(e1, index)) {
      result.insert(e1);
    }
  }
  return result;
}

const cda_rail::index_set&
cda_rail::Network::get_successors_helper(size_t index) const {
  if (!has_edge(index)) {
    throw exceptions::EdgeNotExistentException(index);
  }
  return m_successors[index];
}

cda_rail::index_set cda_rail::Network::neighbors_helper(size_t v_index) const {
  if (!has_vertex(v_index)) {
    throw exceptions::VertexNotExistentException(v_index);
  }
  cda_rail::index_set result;
  for (const auto e_idx : neighboring_edges(v_index)) {
    result.insert(other_vertex(e_idx, v_index));
  }
  return result;
}

std::optional<size_t>
cda_rail::Network::common_vertex_helper(size_t e_idx_1, size_t e_idx_2) const {
  const auto& e1 = get_edge(e_idx_1);
  const auto& e2 = get_edge(e_idx_2);

  std::optional<size_t> shared;
  for (const size_t v : {e1.source, e1.target}) {
    if (v == e2.source || v == e2.target) {
      if (shared.has_value()) {
        throw cda_rail::exceptions::ConsistencyException(
            concatenate_string_views({"Edges", std::to_string(e_idx_1), " and ",
                                      std::to_string(e_idx_2),
                                      " have more than one common vertex"}));
      }
      shared = v;
    }
  }
  return shared;
}

std::optional<size_t>
cda_rail::Network::get_reverse_edge_index_helper(size_t edge_index) const {
  if (!has_edge(edge_index)) {
    throw exceptions::EdgeNotExistentException(edge_index);
  }
  const auto& edge = get_edge(edge_index);
  if (has_edge(edge.target, edge.source)) {
    return get_edge_index(edge.target, edge.source);
  }
  return {};
}

std::vector<std::pair<std::optional<size_t>, std::optional<size_t>>>
cda_rail::Network::combine_reverse_edges(
    const cda_rail::index_vector& edges_to_consider, bool sort) const {
  if (!std::ranges::all_of(edges_to_consider,
                           [this](size_t i) { return has_edge(i); })) {
    throw exceptions::EdgeNotExistentException();
  }

  std::vector<std::pair<std::optional<size_t>, std::optional<size_t>>> result;
  for (const auto edge_index : edges_to_consider) {
    const auto rev = get_reverse_edge_index(edge_index);
    if (!rev.has_value() ||
        !std::ranges::contains(edges_to_consider, rev.value())) {
      result.emplace_back(edge_index, std::nullopt);
    } else if (rev.value() > edge_index) {
      result.emplace_back(edge_index, rev);
    }
  }

  return sort ? sort_edge_pairs(result) : result;
}

size_t cda_rail::Network::get_track_index_helper(size_t edge_index) const {
  const auto rev = get_reverse_edge_index(edge_index);
  return rev.has_value() ? std::min(edge_index, rev.value()) : edge_index;
}

cda_rail::index_vector cda_rail::Network::breakable_edges() const {
  auto view =
      std::views::iota(size_t{0}, number_of_edges()) |
      std::views::filter([this](size_t i) { return get_edge(i).breakable; });
  return {view.begin(), view.end()};
}

cda_rail::index_vector cda_rail::Network::relevant_breakable_edges() const {
  cda_rail::index_vector result;
  for (size_t i = 0; i < number_of_edges(); ++i) {
    if (!get_edge(i).breakable) {
      continue;
    }
    const auto rev = get_reverse_edge_index(i);
    if (!rev.has_value() || rev.value() > i) {
      result.emplace_back(i);
    }
  }
  return result;
}

std::vector<cda_rail::index_set>
cda_rail::Network::unbreakable_sections() const {
  std::vector<cda_rail::index_set> result;

  // Single-edge unbreakable sections between two TTD/VSS border vertices
  const auto is_border = [this](size_t v) {
    const auto t = get_vertex(v).type;
    return t == VertexType::TTD || t == VertexType::VSS;
  };

  for (size_t i = 0; i < number_of_edges(); ++i) {
    const auto& edge = get_edge(i);
    if (edge.breakable) {
      continue;
    }
    const auto rev_idx = get_reverse_edge_index(i);
    // Only process canonical direction (no reverse, or we have the smaller
    // index)
    if (rev_idx.has_value() && rev_idx.value() < i) {
      continue;
    }
    if (!is_border(edge.source) || !is_border(edge.target)) {
      continue;
    }
    cda_rail::index_set section{i};
    if (rev_idx.has_value()) {
      section.insert(rev_idx.value());
    }
    result.emplace_back(std::move(section));
  }

  // Multi-edge sections through NoBorder vertices
  std::unordered_set<size_t> start_vertices;
  for (size_t i = 0; i < number_of_vertices(); ++i) {
    if (get_vertex(i).type == VertexType::NoBorder && !neighbors(i).empty()) {
      start_vertices.emplace(i);
    }
  }
  dfs_inplace(result, start_vertices, VertexType::NoBorder);

  return result;
}

std::vector<cda_rail::index_set>
cda_rail::Network::no_border_vss_sections() const {
  std::unordered_set<size_t> start_vertices;
  for (size_t i = 0; i < number_of_vertices(); ++i) {
    if (get_vertex(i).type == VertexType::NoBorderVSS &&
        !neighbors(i).empty()) {
      start_vertices.emplace(i);
    }
  }
  std::vector<cda_rail::index_set> result;
  dfs_inplace(result, start_vertices, VertexType::NoBorderVSS,
              {VertexType::NoBorder});
  return result;
}

cda_rail::index_set
cda_rail::Network::get_unbreakable_section_containing_edge_helper(
    size_t e) const {
  const auto& edge_object = get_edge(e);
  if (edge_object.breakable) {
    return {};
  }

  cda_rail::index_set result{e};
  if (const auto rev = get_reverse_edge_index(e); rev.has_value()) {
    result.insert(rev.value());
  }

  // BFS: expand through non-border vertices collecting all connected edges
  const auto is_border = [this](size_t v) {
    const auto t = get_vertex(v).type;
    return t == VertexType::TTD || t == VertexType::VSS;
  };

  std::queue<size_t>         to_visit;
  std::unordered_set<size_t> visited;
  to_visit.push(edge_object.source);
  to_visit.push(edge_object.target);

  while (!to_visit.empty()) {
    const auto v = to_visit.front();
    to_visit.pop();

    if (!visited.insert(v).second || is_border(v)) {
      continue;
    }

    for (const auto nb : neighbors(v)) {
      if (!visited.contains(nb)) {
        to_visit.push(nb);
      }
      if (has_edge(v, nb)) {
        result.insert(get_edge_index(v, nb));
      }
      if (has_edge(nb, v)) {
        result.insert(get_edge_index(nb, v));
      }
    }
  }
  return result;
}

bool cda_rail::Network::is_on_same_unbreakable_section_helper(size_t e1,
                                                              size_t e2) const {
  return get_unbreakable_section_containing_edge(e1).contains(e2);
}

cda_rail::index_set cda_rail::Network::vertices_used_by_edges(
    const cda_rail::index_set& edges_tmp) const {
  cda_rail::index_set used;
  for (const auto e : edges_tmp) {
    used.insert(get_edge(e).source);
    used.insert(get_edge(e).target);
  }
  return used;
}

std::vector<std::pair<size_t, size_t>> cda_rail::Network::get_intersecting_ttd(
    const cda_rail::index_vector&           edges_to_consider,
    const std::vector<cda_rail::index_set>& ttd) {
  std::vector<std::pair<size_t, size_t>> result;
  for (size_t ttd_idx = 0; ttd_idx < ttd.size(); ++ttd_idx) {
    // Find the first edge in edges_to_consider that belongs to this TTD section
    for (size_t edge_idx = 0; edge_idx < edges_to_consider.size(); ++edge_idx) {
      if (ttd[ttd_idx].contains(edges_to_consider[edge_idx])) {
        result.emplace_back(ttd_idx, edge_idx);
        break;
      }
    }
  }
  return result;
}

cda_rail::index_set cda_rail::Network::edge_set_complement(
    const cda_rail::index_set& edge_indices) const {
  cda_rail::index_set all_edges;
  for (size_t i = 0; i < number_of_edges(); ++i) {
    all_edges.insert(i);
  }
  return edge_set_complement(edge_indices, all_edges);
}

cda_rail::index_set cda_rail::Network::edge_set_complement(
    const cda_rail::index_set& edge_indices,
    const cda_rail::index_set& edges_to_consider) const {
  const auto edge_exists = [this](size_t i) { return has_edge(i); };
  if (!std::ranges::all_of(edge_indices, edge_exists)) {
    throw exceptions::EdgeNotExistentException();
  }
  if (!std::ranges::all_of(edges_to_consider, edge_exists)) {
    throw exceptions::EdgeNotExistentException();
  }

  cda_rail::index_set result;
  for (const auto e : edges_to_consider) {
    if (!edge_indices.contains(e)) {
      result.insert(e);
    }
  }
  return result;
}

double cda_rail::Network::maximal_vertex_speed_helper(
    size_t vertex_id, const cda_rail::index_set& edges_to_consider) const {
  // Collect relevant neighboring edges
  const auto          all_n_edges = neighboring_edges(vertex_id);
  cda_rail::index_set n_edges;
  if (edges_to_consider.empty()) {
    n_edges = all_n_edges;
  } else {
    for (const auto e : all_n_edges) {
      if (edges_to_consider.contains(e)) {
        n_edges.insert(e);
      }
    }
  }

  if (n_edges.empty()) {
    return 0;
  }
  if (neighbors(vertex_id).size() == 1) {
    return get_edge(*n_edges.cbegin()).max_speed;
  }

  // Find the second-highest speed across distinct neighboring vertices
  double                max_speed        = 0;
  double                second_max_speed = 0;
  std::optional<size_t> max_speed_neighbor;

  for (const auto e : n_edges) {
    const auto& edge = get_edge(e);
    const auto  nb   = other_vertex(e, vertex_id);
    if (edge.max_speed > max_speed) {
      second_max_speed   = max_speed;
      max_speed          = edge.max_speed;
      max_speed_neighbor = nb;
    } else if (edge.max_speed > second_max_speed &&
               max_speed_neighbor.has_value() &&
               nb != max_speed_neighbor.value()) {
      second_max_speed = edge.max_speed;
    }
  }
  return second_max_speed;
}

double cda_rail::Network::minimal_neighboring_edge_length_helper(
    size_t v, const cda_rail::index_set& edges_to_consider) const {
  const auto          all_n_edges = neighboring_edges(v);
  cda_rail::index_set n_edges;
  if (edges_to_consider.empty()) {
    n_edges = all_n_edges;
  } else {
    for (const auto e : all_n_edges) {
      if (edges_to_consider.contains(e)) {
        n_edges.insert(e);
      }
    }
  }

  if (n_edges.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  return get_edge(
             *std::ranges::min_element(
                 n_edges, {}, [this](size_t i) { return get_edge(i).length; }))
      .length;
}

size_t cda_rail::Network::max_vss_on_edge_helper(size_t index) const {
  const auto& edge = get_edge(index);
  if (!edge.breakable || edge.min_block_length == 0) {
    return 0;
  }
  return static_cast<size_t>(std::floor(edge.length / edge.min_block_length));
}

// -----------------------------
// SETTER
// -----------------------------

void cda_rail::Network::change_vertex_name_helper(
    size_t index, std::string_view const new_name) {
  if (!has_vertex(index)) {
    throw exceptions::VertexNotExistentException(index);
  }
  if (has_vertex(new_name)) {
    throw exceptions::InvalidInputException("Vertex already exists");
  }
  m_vertex_name_to_index.erase(m_vertices.at(index).name);
  m_vertices.at(index).name = new_name;
  m_vertex_name_to_index.emplace(new_name, index);
}

void cda_rail::Network::change_vertex_type_helper(size_t     index,
                                                  VertexType new_type) {
  if (!has_vertex(index)) {
    throw exceptions::VertexNotExistentException(index);
  }
  m_vertices[index].type = new_type;
}

void cda_rail::Network::change_vertex_headway_helper(size_t index,
                                                     double new_headway) {
  if (!has_vertex(index)) {
    throw exceptions::VertexNotExistentException(index);
  }
  m_vertices[index].headway = new_headway;
}

void cda_rail::Network::change_edge_length_helper(size_t index,
                                                  double new_length) {
  if (!has_edge(index)) {
    throw exceptions::EdgeNotExistentException(index);
  }
  m_edges[index].length = new_length;
}

void cda_rail::Network::change_edge_max_speed_helper(size_t index,
                                                     double new_max_speed) {
  if (!has_edge(index)) {
    throw exceptions::EdgeNotExistentException(index);
  }
  m_edges[index].max_speed = new_max_speed;
}

void cda_rail::Network::change_edge_min_block_length_helper(
    size_t index, double new_min_block_length) {
  if (!has_edge(index)) {
    throw exceptions::EdgeNotExistentException(index);
  }
  m_edges[index].min_block_length = new_min_block_length;
}

void cda_rail::Network::change_edge_min_stop_block_length_helper(
    size_t index, double new_min_stop_block_length) {
  if (!has_edge(index)) {
    throw exceptions::EdgeNotExistentException(index);
  }
  m_edges[index].min_stop_block_length = new_min_stop_block_length;
}

void cda_rail::Network::change_edge_breakable_helper(size_t const index,
                                                     bool const   value) {
  if (!has_edge(index)) {
    throw exceptions::EdgeNotExistentException(index);
  }
  m_edges.at(index).breakable = value;
}

size_t cda_rail::Network::add_vertex(Vertex vertex) {
  if (has_vertex(vertex.name)) {
    throw exceptions::InvalidInputException("Vertex already exists");
  }
  m_vertices.emplace_back(std::move(vertex));
  m_vertex_name_to_index[m_vertices.back().name] = m_vertices.size() - 1;
  return m_vertices.size() - 1;
}

size_t cda_rail::Network::add_edge_helper(
    size_t source, size_t target, double length, double max_speed,
    std::optional<bool> const&   breakable,
    std::optional<double> const& min_block_length,
    std::optional<double> const& min_stop_block_length) {
  check_new_edge_requirements(source, target);
  m_edges.emplace_back(source, target, length, max_speed, breakable,
                       min_block_length, min_stop_block_length);
  m_successors.emplace_back();
  return m_edges.size() - 1;
}

void cda_rail::Network::add_successor_helper(size_t edge_in, size_t edge_out) {
  if (!has_edge(edge_in)) {
    throw exceptions::EdgeNotExistentException(edge_in);
  }
  if (!has_edge(edge_out)) {
    throw exceptions::EdgeNotExistentException(edge_out);
  }
  if (m_edges[edge_in].target != m_edges[edge_out].source) {
    throw exceptions::ConsistencyException("Edge " + std::to_string(edge_out) +
                                           " is not adjacent to " +
                                           std::to_string(edge_in));
  }
  m_successors[edge_in].insert(edge_out);
}

std::vector<std::pair<size_t, cda_rail::index_vector>>
cda_rail::Network::separate_stop_edges(const cda_rail::index_set& stop_edges) {
  if (!is_consistent_for_transformation()) {
    throw exceptions::ConsistencyException();
  }

  std::vector<std::pair<size_t, cda_rail::index_vector>> result;
  for (const size_t i : stop_edges) {
    const auto& edge = get_edge(i);
    if (!edge.breakable || 2 * edge.min_stop_block_length > edge.length) {
      continue;
    }
    // Only process canonical direction to avoid double-processing reverse edges
    const auto rev = get_reverse_edge_index(i);
    if (rev.has_value() && rev.value() < i &&
        stop_edges.contains(rev.value())) {
      continue;
    }

    auto [fst, snd] = separate_stop_edge(i);
    if (!fst.empty()) {
      result.emplace_back(fst.back(), fst);
    }
    if (!snd.empty()) {
      result.emplace_back(snd.back(), snd);
    }
  }
  return result;
}

std::vector<std::pair<size_t, cda_rail::index_vector>>
cda_rail::Network::discretize(const vss::SeparationFunction& sep_func) {
  if (!is_consistent_for_transformation()) {
    throw exceptions::ConsistencyException();
  }

  std::vector<std::pair<size_t, cda_rail::index_vector>> result;
  for (const size_t i : relevant_breakable_edges()) {
    auto [fst, snd] = separate_edge(i, sep_func);
    if (!fst.empty()) {
      result.emplace_back(fst.back(), fst);
    }
    if (!snd.empty()) {
      result.emplace_back(snd.back(), snd);
    }
  }
  return result;
}

// ------------------------
// Path Finding Algorithms
// ------------------------

std::vector<cda_rail::index_vector>
cda_rail::Network::all_paths_ending_at_ttd_helper(
    size_t e_0, const std::vector<cda_rail::index_set>& ttd_sections,
    std::optional<size_t> exit_node) const {
  std::vector<cda_rail::index_vector> result;
  for (const auto successor : get_successors(e_0)) {
    for (const auto& path : all_paths_ending_at_ttd_recursive_helper(
             successor, ttd_sections, exit_node, {}, true)) {
      result.push_back(path);
    }
  }
  return result;
}

double
cda_rail::Network::length_of_path(const cda_rail::index_vector& path) const {
  return std::transform_reduce(path.begin(), path.end(), 0.0, std::plus{},
                               [this](size_t i) { return get_edge(i).length; });
}

std::vector<std::vector<double>>
cda_rail::Network::all_edge_pairs_shortest_paths() const {
  const size_t                     n = number_of_edges();
  std::vector<std::vector<double>> dist(n, std::vector<double>(n, INF));

  for (size_t u = 0; u < n; ++u) {
    dist[u][u] = 0;
    for (const auto v : get_successors(u)) {
      dist[u][v] = get_edge(v).length;
    }
  }

  // Floyd-Warshall
  for (size_t k = 0; k < n; ++k) {
    for (size_t i = 0; i < n; ++i) {
      if (dist[i][k] == INF) {
        continue;
      } // minor optimization
      for (size_t j = 0; j < n; ++j) {
        if (const double via = dist[i][k] + dist[k][j]; via < dist[i][j]) {
          dist[i][j] = via;
        }
      }
    }
  }
  return dist;
}

std::optional<double> cda_rail::Network::shortest_path_helper(
    size_t source_edge_id, size_t target_id, bool target_is_edge,
    bool include_first_edge, bool use_minimal_time, double max_v) const {
  return shortest_path_using_edges_helper(source_edge_id, target_id, true, {},
                                          target_is_edge, include_first_edge,
                                          use_minimal_time, max_v)
      .first;
}

std::pair<std::optional<double>, cda_rail::index_vector>
cda_rail::Network::shortest_path_between_sets_using_edges_helper(
    cda_rail::index_set source_edge_ids, cda_rail::index_set target_ids,
    bool only_use_valid_successors, cda_rail::index_set edges_to_use,
    bool target_is_edge, bool include_first_edge, bool use_minimal_time,
    double max_v) const {
  // Validate input
  if (source_edge_ids.empty()) {
    throw exceptions::InvalidInputException(
        "Source edge IDs must not be empty");
  }
  if (target_ids.empty()) {
    throw exceptions::InvalidInputException("Target IDs must not be empty");
  }
  for (const auto id : source_edge_ids) {
    if (!has_edge(id)) {
      throw exceptions::EdgeNotExistentException(id);
    }
  }
  for (const auto id : target_ids) {
    if (target_is_edge && !has_edge(id)) {
      throw exceptions::EdgeNotExistentException(id);
    }
    if (!target_is_edge && !has_vertex(id)) {
      throw exceptions::VertexNotExistentException(id);
    }
  }
  if (use_minimal_time && max_v <= 0) {
    throw exceptions::InvalidInputException(
        "Maximum speed must be strictly positive if minimal time is used");
  }

  // Check if a source already reaches a target (zero cost when not including
  // first edge)
  if (!include_first_edge) {
    for (const auto src : source_edge_ids) {
      const auto& e = get_edge(src);
      for (const auto tgt : target_ids) {
        if ((target_is_edge && src == tgt) ||
            (!target_is_edge && e.target == tgt)) {
          return {0, {src}};
        }
      }
    }
  }

  const size_t           n = number_of_edges();
  std::vector<double>    distances(n, INF);
  std::vector<bool>      visited(n, false);
  cda_rail::index_vector predecessors(n, std::numeric_limits<size_t>::max());

  // Min-heap: (distance, edge_id)
  std::priority_queue<std::pair<double, size_t>,
                      std::vector<std::pair<double, size_t>>, std::greater<>>
      pq;

  for (const auto src : source_edge_ids) {
    const double d =
        include_first_edge
            ? delta_dist_helper(get_edge(src), max_v, use_minimal_time)
            : 0.0;
    if (d < distances[src]) {
      distances[src] = d;
      pq.emplace(d, src);
    }
  }

  while (!pq.empty()) {
    auto [dist, edge_id] = pq.top();
    pq.pop();

    if (visited[edge_id]) {
      continue;
    }
    visited[edge_id] = true;

    const auto& edge = get_edge(edge_id);

    // Check if we reached a target
    if (target_ids.contains(target_is_edge ? edge_id : edge.target)) {
      // Reconstruct path by walking predecessor chain
      cda_rail::index_vector path{edge_id};
      while (!source_edge_ids.contains(path.back()) &&
             predecessors[path.back()] != std::numeric_limits<size_t>::max()) {
        const size_t pred = predecessors[path.back()];
        if (std::ranges::contains(path, pred)) {
          throw exceptions::ConsistencyException("Cycle in path");
        }
        path.emplace_back(pred);
      }
      std::ranges::reverse(path);
      return {dist, path};
    }

    const auto& raw_successors = only_use_valid_successors
                                     ? get_successors(edge_id)
                                     : out_edges(edge.target);

    for (const auto succ : raw_successors) {
      if (!edges_to_use.empty() && !edges_to_use.contains(succ)) {
        continue;
      }
      const auto& succ_edge = get_edge(succ);
      // Skip U-turn (reverse edge)
      if (succ_edge.source == edge.target && succ_edge.target == edge.source) {
        continue;
      }
      const double new_dist =
          dist + delta_dist_helper(succ_edge, max_v, use_minimal_time);
      if (new_dist < distances[succ]) {
        distances[succ]    = new_dist;
        predecessors[succ] = edge_id;
        pq.emplace(new_dist, succ);
      }
    }
  }

  return {std::nullopt, {}};
}
