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

  for (const auto& edge : m_edges) {
    if (edge.source == source_id && edge.target == target_id) {
      return true;
    }
  }
  return false;
}

bool cda_rail::Network::is_valid_successor_helper(size_t e0, size_t e1) const {
  if (!has_edge(e0)) {
    throw exceptions::EdgeNotExistentException(e0);
  }
  if (!has_edge(e1)) {
    throw exceptions::EdgeNotExistentException(e1);
  }

  if (m_edges[e0].target != m_edges[e1].source) {
    return false;
  }
  return (std::ranges::contains(m_successors.at(e0), e1));
}

bool cda_rail::Network::is_adjustable_helper(size_t vertex_id) const {
  if (!has_vertex(vertex_id)) {
    throw exceptions::VertexNotExistentException(vertex_id);
  }

  if (m_vertices[vertex_id].type != VertexType::NoBorder) {
    return false;
  }

  if (neighbors(vertex_id).size() != 2) {
    return false;
  }

  return true;
}

bool cda_rail::Network::is_consistent_for_transformation() const {
  for (size_t i = 0; i < number_of_edges(); ++i) {
    const auto& edge = get_edge(i);

    if (edge.breakable && edge.min_block_length <= 0) {
      return false;
    }

    if (edge.breakable &&
        (get_vertex(edge.source).type == VertexType::NoBorder ||
         get_vertex(edge.target).type == VertexType::NoBorder)) {
      return false;
    }

    if (has_edge(edge.target, edge.source)) {
      const auto& reverse_edge = get_edge({edge.target, edge.source});
      if (edge.breakable != reverse_edge.breakable ||
          edge.length != reverse_edge.length) {
        return false;
      }
    }
  }

  for (size_t i = 0; i < number_of_vertices(); ++i) {
    if (get_vertex(i).type == VertexType::NoBorderVSS) {
      const auto& v_neighbors = neighbors(i);

      if (v_neighbors.size() > 2) {
        return false;
      }

      if (std::ranges::any_of(v_neighbors, [this](const auto& j) {
            return this->get_vertex(j).type == VertexType::NoBorder;
          })) {
        return false;
      }
    }
  }

  return true;
}

cda_rail::index_vector
cda_rail::Network::get_vertices_by_type(VertexType type) const {
  cda_rail::index_vector ret_val;
  for (size_t i = 0; i < m_vertices.size(); ++i) {
    if (m_vertices[i].type == type) {
      ret_val.emplace_back(i);
    }
  }
  return ret_val;
}

std::pair<size_t, double>
cda_rail::Network::get_old_edge_helper(size_t new_edge) const {
  if (!has_edge(new_edge)) {
    throw exceptions::EdgeNotExistentException(new_edge);
  }
  // If new_edge_to_old_edge_after_transform has no(!) key new_edge, return
  // (new_edge, 0)
  if (m_new_edge_to_old_edge_after_transform.find(new_edge) ==
      m_new_edge_to_old_edge_after_transform.end()) {
    return {new_edge, 0};
  }
  return m_new_edge_to_old_edge_after_transform.at(new_edge);
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
  for (size_t i = 0; i < m_edges.size(); i++) {
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
  cda_rail::index_set out_edges;
  for (size_t i = 0; i < m_edges.size(); ++i) {
    if (m_edges[i].source == index) {
      out_edges.insert(i);
    }
  }
  return out_edges;
}

cda_rail::index_set
cda_rail::Network::in_edges_helper(size_t const index) const {
  if (!has_vertex(index)) {
    throw exceptions::VertexNotExistentException(index);
  }
  cda_rail::index_set in_edges;
  for (size_t i = 0; i < m_edges.size(); ++i) {
    if (m_edges[i].target == index) {
      in_edges.insert(i);
    }
  }
  return in_edges;
}

cda_rail::index_set
cda_rail::Network::neighboring_edges_helper(size_t const index) const {
  auto       ret_val      = in_edges(index);
  const auto edges_to_add = out_edges(index);
  ret_val.insert(edges_to_add.begin(), edges_to_add.end());
  return ret_val;
}

cda_rail::index_set
cda_rail::Network::get_predecessors_helper(size_t index) const {
  if (!has_edge(index)) {
    throw exceptions::EdgeNotExistentException(index);
  }
  cda_rail::index_set ret_val;

  for (const auto& e_1 : in_edges(get_edge(index).source)) {
    if (is_valid_successor(e_1, index)) {
      ret_val.insert(e_1);
    }
  }

  return ret_val;
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
  cda_rail::index_set neighbors;

  for (auto const  edges_to_consider = neighboring_edges(v_index);
       const auto& e_idx : edges_to_consider) {
    neighbors.insert(other_vertex(e_idx, v_index));
  }

  return neighbors;
}

std::optional<size_t>
cda_rail::Network::common_vertex_helper(size_t e_idx_1, size_t e_idx_2) const {
  auto const& e1_obj = get_edge(e_idx_1);
  auto const& e2_obj = get_edge(e_idx_2);

  std::unordered_set<size_t> intersection;
  if (e1_obj.source == e2_obj.source || e1_obj.source == e2_obj.target) {
    intersection.insert(e1_obj.source);
  }
  if (e1_obj.target == e2_obj.source || e1_obj.target == e2_obj.target) {
    intersection.insert(e1_obj.target);
  }

  if (intersection.empty()) {
    return {};
  }
  if (intersection.size() == 1) {
    return *intersection.begin();
  }
  throw cda_rail::exceptions::ConsistencyException(concatenate_string_views(
      {"Edges", std::to_string(e_idx_1), " and ", std::to_string(e_idx_2),
       " have more than one common vertex"}));
}

std::optional<size_t>
cda_rail::Network::get_reverse_edge_index_helper(size_t edge_index) const {
  if (!has_edge(edge_index)) {
    throw exceptions::EdgeNotExistentException(edge_index);
  }

  if (const auto& edge = get_edge(edge_index);
      has_edge(edge.target, edge.source)) {
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

  std::vector<std::pair<std::optional<size_t>, std::optional<size_t>>> ret_val;
  for (const auto& edge_index : edges_to_consider) {
    const auto reverse_edge_index = get_reverse_edge_index(edge_index);
    if (!reverse_edge_index.has_value() ||
        !std::ranges::contains(edges_to_consider, reverse_edge_index.value())) {
      ret_val.emplace_back(edge_index, std::nullopt);
    } else if (reverse_edge_index > edge_index) {
      ret_val.emplace_back(edge_index, reverse_edge_index);
    }
  }

  if (sort) {
    return sort_edge_pairs(ret_val);
  }
  return ret_val;
}

size_t cda_rail::Network::get_track_index_helper(size_t edge_index) const {
  const auto reverse_edge_index = get_reverse_edge_index(edge_index);
  if (reverse_edge_index.has_value()) {
    return std::min(edge_index, reverse_edge_index.value());
  }
  return edge_index;
}

cda_rail::index_vector cda_rail::Network::breakable_edges() const {
  cda_rail::index_vector ret_val;
  for (size_t i = 0; i < number_of_edges(); ++i) {
    if (get_edge(i).breakable) {
      ret_val.emplace_back(i);
    }
  }
  return ret_val;
}

cda_rail::index_vector cda_rail::Network::relevant_breakable_edges() const {
  cda_rail::index_vector ret_val;
  for (size_t i = 0; i < number_of_edges(); ++i) {
    const auto& edge = get_edge(i);
    // add edge only if reverse edge does not exist or has larger index
    if (edge.breakable) {
      if (auto const reverse_edge_index = get_reverse_edge_index(i);
          reverse_edge_index.has_value() && reverse_edge_index.value() > i) {
        ret_val.emplace_back(i);
      }
    }
  }
  return ret_val;
}

std::vector<cda_rail::index_set>
cda_rail::Network::unbreakable_sections() const {
  std::vector<cda_rail::index_set> ret_val;

  // Add all one edge sections
  for (size_t i = 0; i < number_of_edges(); ++i) {
    const auto& edge = get_edge(i);
    if (!edge.breakable &&
        ((!has_edge(edge.target, edge.source) ||
          get_edge_index(edge.target, edge.source) > i)) &&
        (get_vertex(edge.source).type == VertexType::TTD ||
         get_vertex(edge.source).type == VertexType::VSS) &&
        (get_vertex(edge.target).type == VertexType::TTD ||
         get_vertex(edge.target).type == VertexType::VSS)) {
      ret_val.emplace_back();
      ret_val.back().insert(i);
      if (has_edge(edge.target, edge.source)) {
        ret_val.back().insert(get_edge_index(edge.target, edge.source));
      }
    }
  }

  // Get possible start vertices for DFS
  std::unordered_set<size_t> vertices_to_visit;
  for (size_t i = 0; i < number_of_vertices(); ++i) {
    if (get_vertex(i).type == VertexType::NoBorder && !neighbors(i).empty()) {
      vertices_to_visit.emplace(i);
    }
  }

  dfs_inplace(ret_val, vertices_to_visit, VertexType::NoBorder);

  return ret_val;
}

std::vector<cda_rail::index_set>
cda_rail::Network::no_border_vss_sections() const {
  // Get possible start vertices for DFS
  std::unordered_set<size_t> vertices_to_visit;
  for (size_t i = 0; i < number_of_vertices(); ++i) {
    if (get_vertex(i).type == VertexType::NoBorderVSS &&
        !neighbors(i).empty()) {
      vertices_to_visit.emplace(i);
    }
  }

  std::vector<cda_rail::index_set> ret_val;
  dfs_inplace(ret_val, vertices_to_visit, VertexType::NoBorderVSS,
              {VertexType::NoBorder});

  return ret_val;
}

cda_rail::index_set
cda_rail::Network::get_unbreakable_section_containing_edge_helper(
    size_t e) const {
  const auto& edge_object = get_edge(e);
  if (edge_object.breakable) {
    return {};
  }

  cda_rail::index_set ret_val;
  ret_val.insert(e);
  const auto reverse_e = get_reverse_edge_index(e);
  if (reverse_e.has_value()) {
    ret_val.insert(reverse_e.value());
  }

  std::queue<size_t>     vertices_to_visit;
  cda_rail::index_vector visited_vertices;
  vertices_to_visit.push(edge_object.source);
  vertices_to_visit.push(edge_object.target);

  while (!vertices_to_visit.empty()) {
    const auto& current_v = vertices_to_visit.front();

    if (std::ranges::contains(visited_vertices, current_v)) {
      // Vertex already visited
      vertices_to_visit.pop();
      continue;
    }

    if (get_vertex(current_v).type != VertexType::TTD &&
        get_vertex(current_v).type != VertexType::VSS) {
      // This vertex can be used to further extend the section
      const auto& n_vertices = neighbors(current_v);
      for (const auto& v_tmp : n_vertices) {
        if (!std::ranges::contains(visited_vertices, v_tmp)) {
          vertices_to_visit.push(v_tmp);
        }

        // Possibly add new edges
        if (has_edge(current_v, v_tmp)) {
          const auto& e_tmp = get_edge_index(current_v, v_tmp);
          if (!std::ranges::contains(ret_val, e_tmp)) {
            ret_val.insert(e_tmp);
          }
        }
        if (has_edge(v_tmp, current_v)) {
          const auto& e_tmp = get_edge_index(v_tmp, current_v);
          if (!std::ranges::contains(ret_val, e_tmp)) {
            ret_val.insert(e_tmp);
          }
        }
      }
    }

    // Update visited vertices
    visited_vertices.push_back(current_v);
    vertices_to_visit.pop();
  }

  return ret_val;
}

bool cda_rail::Network::is_on_same_unbreakable_section_helper(size_t e1,
                                                              size_t e2) const {
  const auto section_tmp = get_unbreakable_section_containing_edge(e1);
  return std::ranges::contains(section_tmp, e2);
}

cda_rail::index_set cda_rail::Network::vertices_used_by_edges(
    const cda_rail::index_set& edges_tmp) const {
  std::unordered_set<size_t> used_vertices;
  for (const auto& edge : edges_tmp) {
    used_vertices.insert(get_edge(edge).source);
    used_vertices.insert(get_edge(edge).target);
  }
  return used_vertices;
}

std::vector<std::pair<size_t, size_t>> cda_rail::Network::get_intersecting_ttd(
    const cda_rail::index_vector&           edges_to_consider,
    const std::vector<cda_rail::index_set>& ttd) {
  std::vector<std::pair<size_t, size_t>> ret_val;

  for (size_t ttd_index = 0; ttd_index < ttd.size(); ++ttd_index) {
    bool        intersection_found = false;
    const auto& ttd_section        = ttd.at(ttd_index);
    for (size_t edge_index = 0;
         !intersection_found && edge_index < edges_to_consider.size();
         ++edge_index) {
      if (std::ranges::contains(ttd_section,
                                edges_to_consider.at(edge_index))) {
        ret_val.emplace_back(ttd_index, edge_index);
        intersection_found = true;
      }
    }
  }

  return ret_val;
}

cda_rail::index_set cda_rail::Network::edge_set_complement(
    const cda_rail::index_set& edge_indices) const {
  cda_rail::index_set edges_to_consider;
  edges_to_consider.reserve(number_of_edges());

  auto edges_view = std::views::iota(size_t{0}, number_of_edges());
  edges_to_consider.insert(edges_view.begin(), edges_view.end());

  return edge_set_complement(edge_indices, edges_to_consider);
}

cda_rail::index_set cda_rail::Network::edge_set_complement(
    const cda_rail::index_set& edge_indices,
    const cda_rail::index_set& edges_to_consider) const {
  if (!std::ranges::all_of(edge_indices,
                           [this](size_t i) { return has_edge(i); })) {
    throw exceptions::EdgeNotExistentException();
  }
  if (!std::ranges::all_of(edges_to_consider,
                           [this](size_t i) { return has_edge(i); })) {
    throw exceptions::EdgeNotExistentException();
  }

  cda_rail::index_set ret_val;

  auto diff_view =
      edges_to_consider | std::views::filter([&edge_indices](size_t i) {
        return !edge_indices.contains(i);
      });

  for (auto const edge : diff_view) {
    ret_val.insert(edge);
  }

  return ret_val;
}

double cda_rail::Network::maximal_vertex_speed_helper(
    size_t vertex_id, const cda_rail::index_set& edges_to_consider) const {
  const auto& n_edges_tmp = neighboring_edges(vertex_id);
  auto        n_edges =
      edges_to_consider.empty() ? n_edges_tmp : cda_rail::index_set();
  for (const auto& e : n_edges_tmp) {
    if (std::ranges::contains(edges_to_consider, e)) {
      n_edges.insert(e);
    }
  }

  if (n_edges.empty()) {
    return 0;
  }

  if (neighbors(vertex_id).size() == 1) {
    return get_edge(*n_edges.cbegin()).max_speed;
  }

  double                max_speed = 0;
  std::optional<size_t> max_speed_neighboring_vertex;
  double                second_max_speed = 0;
  for (const auto& e : n_edges) {
    const auto& edge = get_edge(e);
    if (edge.max_speed > max_speed) {
      second_max_speed             = max_speed;
      max_speed                    = edge.max_speed;
      max_speed_neighboring_vertex = other_vertex(e, vertex_id);
    } else if (edge.max_speed > second_max_speed &&
               max_speed_neighboring_vertex.has_value() &&
               // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
               other_vertex(e, vertex_id) !=
                   max_speed_neighboring_vertex.value()) {
      second_max_speed = edge.max_speed;
    }
  }
  return second_max_speed;
}

double cda_rail::Network::minimal_neighboring_edge_length_helper(
    size_t v, const cda_rail::index_set& edges_to_consider) const {
  const auto n_edges_tmp = neighboring_edges(v);
  auto       n_edges =
      edges_to_consider.empty() ? n_edges_tmp : cda_rail::index_set();
  for (const auto& e : n_edges_tmp) {
    if (std::ranges::contains(edges_to_consider, e)) {
      n_edges.insert(e);
    }
  }

  if (n_edges.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  const auto min_edge_index = *std::min_element(
      n_edges.begin(), n_edges.end(), [this](size_t a, size_t b) {
        return get_edge(a).length < get_edge(b).length;
      });
  return get_edge(min_edge_index).length;
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

  // If successors[edges] already contains edge_out, do nothing
  if (std::ranges::contains(m_successors.at(edge_in), edge_out)) {
    return;
  }

  m_successors[edge_in].insert(edge_out);
}

std::vector<std::pair<size_t, cda_rail::index_vector>>
cda_rail::Network::separate_stop_edges(
    const cda_rail::index_vector& stop_edges) {
  std::vector<std::pair<size_t, cda_rail::index_vector>> ret_val;
  for (size_t const i : stop_edges) {
    const auto edge_object = get_edge(i);
    if (2 * edge_object.min_stop_block_length > edge_object.length) {
      continue;
    }
    auto separated_edges = separate_stop_edge(i);
    if (!separated_edges.first.empty()) {
      ret_val.emplace_back(separated_edges.first.back(), separated_edges.first);
    }
    if (!separated_edges.second.empty()) {
      ret_val.emplace_back(separated_edges.second.back(),
                           separated_edges.second);
    }
  }
  return ret_val;
}

std::vector<std::pair<size_t, cda_rail::index_vector>>
cda_rail::Network::discretize(const vss::SeparationFunction& sep_func) {
  std::vector<std::pair<size_t, cda_rail::index_vector>> ret_val;
  for (size_t const i : relevant_breakable_edges()) {
    auto separated_edges = separate_edge(i, sep_func);
    if (!separated_edges.first.empty()) {
      ret_val.emplace_back(separated_edges.first.back(), separated_edges.first);
    }
    if (!separated_edges.second.empty()) {
      ret_val.emplace_back(separated_edges.second.back(),
                           separated_edges.second);
    }
  }
  return ret_val;
}

// ------------------------
// Path Finding Algorithms
// ------------------------

std::vector<cda_rail::index_vector> cda_rail::Network::all_paths_ending_at_ttd(
    size_t e_0, const std::vector<cda_rail::index_set>& ttd_sections,
    std::optional<size_t> exit_node) const {
  std::vector<cda_rail::index_vector> ret_val;

  const auto possible_successors = get_successors(e_0);
  for (const auto& successor : possible_successors) {
    const auto successor_paths =
        all_paths_ending_at_ttd(successor, ttd_sections, exit_node, {}, true);
    for (const auto& successor_path : successor_paths) {
      ret_val.emplace_back();
      ret_val.back().insert(ret_val.back().end(), successor_path.begin(),
                            successor_path.end());
    }
  }
  return ret_val;
}

double
cda_rail::Network::length_of_path(const cda_rail::index_vector& path) const {
  double len = 0.0;
  for (const auto& edge_index : path) {
    len += get_edge(edge_index).length;
  }
  return len;
}

std::vector<std::vector<double>>
cda_rail::Network::all_edge_pairs_shortest_paths() const {
  std::vector<std::vector<double>> ret_val(
      number_of_edges(), std::vector<double>(number_of_edges(), INF));

  for (size_t u = 0; u < number_of_edges(); ++u) {
    for (size_t v = 0; v < number_of_edges(); ++v) {
      if (u == v) {
        ret_val[u][v] = 0;
      } else if (is_valid_successor(u, v)) {
        ret_val[u][v] = get_edge(v).length;
      }
    }
  }

  // Floyd-Warshall iterations
  for (size_t k = 0; k < number_of_edges(); ++k) {
    for (size_t i = 0; i < number_of_edges(); ++i) {
      for (size_t j = 0; j < number_of_edges(); ++j) {
        ret_val[i][j] = std::min(ret_val[i][j], ret_val[i][k] + ret_val[k][j]);
      }
    }
  }

  return ret_val;
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
  for (const auto& source_edge_id : source_edge_ids) {
    if (!has_edge(source_edge_id)) {
      throw exceptions::EdgeNotExistentException(source_edge_id);
    }
  }
  for (const auto& target_id : target_ids) {
    if (target_is_edge && !has_edge(target_id)) {
      throw exceptions::EdgeNotExistentException(target_id);
    }
    if (!target_is_edge && !has_vertex(target_id)) {
      throw exceptions::VertexNotExistentException(target_id);
    }
  }
  if (use_minimal_time && max_v <= 0) {
    throw exceptions::InvalidInputException(
        "Maximum speed must be strictly positive if minimal time is used");
  }

  // If source edge already leads to the target, then the distance is 0
  if (!include_first_edge) {
    for (const auto& source_edge_id : source_edge_ids) {
      const auto& source_edge = get_edge(source_edge_id);
      for (const auto& target_id : target_ids) {
        if (target_is_edge && source_edge_id == target_id) {
          return {0, {source_edge_id}};
        }
        if (!target_is_edge && source_edge.target == target_id) {
          return {0, {source_edge_id}};
        }
      }
    }
  }

  // Initialize vectors and queues for Dijkstra
  std::vector<double>    distances(number_of_edges(), INF);
  std::vector<bool>      visited(number_of_edges(), false);
  cda_rail::index_vector predecessors(number_of_edges(),
                                      std::numeric_limits<size_t>::max());
  // Priority queue where the element with the smallest .first is returned
  std::priority_queue<std::pair<double, size_t>,
                      std::vector<std::pair<double, size_t>>, std::greater<>>
      pq;
  for (const auto& source_edge_id : source_edge_ids) {
    const double initial_dist =
        (include_first_edge ? delta_dist_helper(get_edge(source_edge_id), max_v,
                                                use_minimal_time)
                            : 0.0);
    pq.emplace(initial_dist, source_edge_id);
    distances[source_edge_id] = initial_dist;
  }

  // Dijkstra
  while (!pq.empty()) {
    auto [dist, edge_id] = pq.top();
    pq.pop();

    if (visited[edge_id]) {
      // Probably relict from later update due to shorter path
      continue;
    }
    visited[edge_id] = true;

    const auto& edge = get_edge(edge_id);

    const bool target_found = std::ranges::contains(
        target_ids, target_is_edge ? edge_id : edge.target);

    if (target_found) {
      cda_rail::index_vector path;
      path.emplace_back(edge_id);
      while (!std::ranges::contains(source_edge_ids, path.back()) &&
             predecessors[path.back()] != std::numeric_limits<size_t>::max()) {
        const size_t predecessor = predecessors[path.back()];
        if (std::ranges::contains(path, predecessor)) {
          throw exceptions::ConsistencyException("Cycle in path");
        }
        path.emplace_back(predecessor);
      }
      std::reverse(path.begin(), path.end());
      return {dist, path};
    }

    const auto&         possible_successors = only_use_valid_successors
                                                  ? get_successors(edge_id)
                                                  : out_edges(edge.target);
    cda_rail::index_set cleaned_successors;
    if (edges_to_use.empty()) {
      cleaned_successors = possible_successors;
    } else {
      for (const auto& successor : possible_successors) {
        if (std::ranges::contains(edges_to_use, successor)) {
          cleaned_successors.insert(successor);
        }
      }
    }

    for (const auto& successor : cleaned_successors) {
      const auto& successor_edge = get_edge(successor);
      if (successor_edge.source == edge.target &&
          successor_edge.target == edge.source) {
        // Skip reverse edge
        continue;
      }
      if (dist + successor_edge.length < distances[successor]) {
        // Update entry in priority queue
        const auto delta_dist =
            delta_dist_helper(successor_edge, max_v, use_minimal_time);
        distances[successor]    = dist + delta_dist;
        predecessors[successor] = edge_id;
        pq.emplace(distances[successor], successor);
      }
    }
  }

  return {std::nullopt, {}};
}
