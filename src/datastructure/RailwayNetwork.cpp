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
#include <sstream>
#include <stack>
#include <string>
#include <tinyxml2.h>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

using json = nlohmann::json;

// Constructor

cda_rail::Network::Network(const std::filesystem::path& working_directory,
                           std::string_view const       networkName) {
  /**
   * Construct object and read network from path. This includes the graph and
   * successors. The network is stored in the networks subfolder of the working
   * directory. For this a folder with the networkName is created.
   *
   * @param working_directory The working directory of the program
   * @param neworkName The network name
   * @return Network
   */

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

size_t cda_rail::Network::add_vertex(Vertex vertex) {
  /**
   * Add vertex to network
   * @param vertex The vertex object
   *
   * @return Index of vertex
   */
  if (has_vertex(vertex.name)) {
    throw exceptions::InvalidInputException("Vertex already exists");
  }
  m_vertices.emplace_back(std::move(vertex));
  m_vertex_name_to_index[m_vertices.back().name] = m_vertices.size() - 1;
  return m_vertices.size() - 1;
}

size_t cda_rail::Network::add_edge(size_t source, size_t target, double length,
                                   double max_speed, bool breakable,
                                   double min_block_length,
                                   double min_stop_block_length) {
  /**
   * Add edge to network
   * @param source Source vertex
   * @param target Target vertex
   * @param length Length of edge
   * @param max_speed Maximum speed on edge
   * @param breakable Whether edge is breakable
   * @param min_block_length Minimum block length on edge
   *
   * @return Index of edge
   */

  check_new_edge_requirements(source, target);

  m_edges.emplace_back(source, target, length, max_speed, breakable,
                       min_block_length, min_stop_block_length);
  m_successors.emplace_back();
  return m_edges.size() - 1;
}

size_t cda_rail::Network::add_edge(
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
  /**
   * Add successor to edge, but only if the edges are adjacent to each other
   *
   * @param edge_in Edge to add successor to
   * @param edge_out Edge to add as successor
   */
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

const cda_rail::Vertex& cda_rail::Network::get_vertex(size_t index) const {
  /**
   * Get vertex by index
   *
   * @param index Index of vertex
   *
   * @return Vertex
   */
  if (!has_vertex(index)) {
    throw exceptions::VertexNotExistentException(index);
  }
  return m_vertices[index];
}

size_t cda_rail::Network::get_vertex_index(std::string_view const name) const {
  /**
   * Get vertex index by name
   *
   * @param name Name of vertex
   *
   * @return Index of vertex
   */
  if (!has_vertex(name)) {
    throw exceptions::VertexNotExistentException(name);
  }
  return m_vertex_name_to_index.at(std::string{name});
}

const cda_rail::Edge& cda_rail::Network::get_edge(size_t index) const {
  /**
   * Get edge by index
   *
   * @param index Index of edge
   *
   * @return Edge
   */
  if (!has_edge(index)) {
    throw exceptions::EdgeNotExistentException(index);
  }
  return m_edges[index];
}

const cda_rail::Edge& cda_rail::Network::get_edge(size_t source_id,
                                                  size_t target_id) const {
  /**
   * Get edge by source and target index
   *
   * @param source_id Index of source vertex
   * @param target_id Index of target vertex
   *
   * @return Edge
   */
  if (!has_vertex(source_id)) {
    throw exceptions::VertexNotExistentException(source_id);
  }
  if (!has_vertex(target_id)) {
    throw exceptions::VertexNotExistentException(target_id);
  }
  for (const auto& edge : m_edges) {
    if (edge.source == source_id && edge.target == target_id) {
      return edge;
    }
  }
  throw exceptions::EdgeNotExistentException(source_id, target_id);
}

size_t cda_rail::Network::get_edge_index(size_t source_id,
                                         size_t target_id) const {
  /**
   * Get edge index by source and target index
   *
   * @param source_id Index of source vertex
   * @param target_id Index of target vertex
   *
   * @return Index of edge
   */
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

void cda_rail::Network::change_vertex_name(size_t                 index,
                                           std::string_view const new_name) {
  /**
   * Change vertex name
   *
   * @param index Index of vertex
   * @param new_name New name of vertex
   */
  if (!has_vertex(index)) {
    throw exceptions::VertexNotExistentException(index);
  }
  if (has_vertex(new_name)) {
    throw exceptions::InvalidInputException("Vertex already exists");
  }
  m_vertex_name_to_index.erase(m_vertices.at(index).name);
  m_vertices.at(index).name                            = new_name;
  m_vertex_name_to_index.at(m_vertices.at(index).name) = index;
}

void cda_rail::Network::change_edge_length_helper(size_t index,
                                                  double new_length) {
  /**
   * Change edge length
   *
   * @param index Index of edge
   * @param new_length New length of edge
   */
  if (!has_edge(index)) {
    throw exceptions::EdgeNotExistentException(index);
  }
  m_edges[index].length = new_length;
}

void cda_rail::Network::change_edge_max_speed_helper(size_t index,
                                                     double new_max_speed) {
  /**
   * Change edge max speed
   *
   * @param index Index of edge
   * @param new_max_speed New max speed of edge
   */
  if (!has_edge(index)) {
    throw exceptions::EdgeNotExistentException(index);
  }
  m_edges[index].max_speed = new_max_speed;
}

void cda_rail::Network::change_edge_min_block_length_helper(
    size_t index, double new_min_block_length) {
  /**
   * Change edge min block length
   *
   * @param index Index of edge
   * @param new_min_block_length New min block length of edge
   */
  if (!has_edge(index)) {
    throw exceptions::EdgeNotExistentException(index);
  }
  m_edges[index].min_block_length = new_min_block_length;
}

void cda_rail::Network::change_edge_min_stop_block_length_helper(
    size_t index, double new_min_stop_block_length) {
  /**
   * Change edge min stop block length
   *
   * @param index Index of edge
   * @param new_min_stop_block_length New min block length of edge
   */
  if (!has_edge(index)) {
    throw exceptions::EdgeNotExistentException(index);
  }
  m_edges[index].min_stop_block_length = new_min_stop_block_length;
}

void cda_rail::Network::set_edge_breakable_helper(size_t index) {
  /**
   * Sets an edge to be breakable
   *
   * @param index Index of edge
   */
  if (!has_edge(index)) {
    throw exceptions::EdgeNotExistentException(index);
  }
  m_edges[index].breakable = true;
}

void cda_rail::Network::set_edge_unbreakable_helper(size_t index) {
  /**
   * Sets an edge to be unbreakable
   *
   * @param index Index of edge
   */
  if (!has_edge(index)) {
    throw exceptions::EdgeNotExistentException(index);
  }
  m_edges[index].breakable = false;
}

cda_rail::index_set cda_rail::Network::out_edges(size_t index) const {
  /**
   * Gets all edges leaving a given vertex
   *
   * @param index Index of vertex
   *
   * @return Vector of indices of edges leaving the vertex
   */
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

cda_rail::index_set cda_rail::Network::in_edges(size_t const index) const {
  /**
   * Gets all edges entering a given vertex
   *
   * @param index Index of vertex
   *
   * @return Vector of indices of edges entering the vertex
   */
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

const cda_rail::index_set&
cda_rail::Network::get_successors(size_t index) const {
  /**
   * Gets all successors of a given edge
   *
   * @param index Index of edge
   *
   * @return Vector of indices of successors
   */
  if (!has_edge(index)) {
    throw exceptions::EdgeNotExistentException(index);
  }
  return m_successors[index];
}

cda_rail::index_set cda_rail::Network::get_predecessors(size_t index) const {
  /**
   * Gets all predecessors of a given edge
   *
   * @param index Index of edge
   *
   * @return Vector of indices of predecessors
   */
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

void cda_rail::Network::export_network(
    const std::filesystem::path& working_directory) const {
  /** Export the network to a given directory. This includes the graphml and
   * successors files. The network is stored in the networks subfolder in a
   * folder named after the network name.
   *
   * @param working_directory: The working directory of the program.
   */

  auto const path = working_directory / "networks" / m_network_name;
  if (!is_directory_and_create(path)) {
    throw exceptions::ExportException("Could not create directory " +
                                      path.string());
  }

  export_graphml(path);

  export_successors_cpp(path);
  export_successors_python(path);
}

bool cda_rail::Network::is_valid_successor(size_t e0, size_t e1) const {
  /**
   * Checks if the edge e1 is a valid successor of the edge e0, this includes
   * the following:
   * - The source of e1 is the target of e0, i.e., they are adjacent
   * - e1 is a valid successor of e0 according to the successors map (not all
   * edges on turnouts e.g.)
   *
   * @param e0: The edge to check the successors of.
   * @param e1: The edge to check if it is a successor of e0.
   *
   * @return: True if e1 is a valid successor of e0, false otherwise.
   */
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

cda_rail::index_vector cda_rail::Network::neighbors(size_t index) const {
  /**
   * Get all neighbors of a given vertex
   *
   * @param index Index of vertex
   *
   * @return Vector of vertex indices of neighbors
   */
  if (!has_vertex(index)) {
    throw exceptions::VertexNotExistentException(index);
  }
  cda_rail::index_vector neighbors;
  auto                   e_out = out_edges(index);
  auto                   e_in  = in_edges(index);
  for (auto e : e_out) {
    if (!std::ranges::contains(neighbors, get_edge(e).target)) {
      neighbors.emplace_back(get_edge(e).target);
    }
  }
  for (auto e : e_in) {
    if (!std::ranges::contains(neighbors, get_edge(e).source)) {
      neighbors.emplace_back(get_edge(e).source);
    }
  }
  return neighbors;
}

bool cda_rail::Network::is_adjustable(size_t vertex_id) const {
  /**
   * Checks if a given vertex type is adjustable (if applicable for a certain
   * algorithmic approach). A vertex is adjustable if all of the following
   * conditions hold:
   * - The vertex is of type NoBorder
   * - The vertex has exactly two neighboring vertices
   * In all other cases the vertex is not adjustable.
   *
   * @param vertex_id: The id of the vertex to check.
   * @return: True if the vertex is adjustable, false otherwise.
   */

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

cda_rail::index_vector cda_rail::Network::breakable_edges() const {
  /**
   * Returns indices of all breakable edges.
   *
   * @return Vector of indices of breakable edges.
   */

  cda_rail::index_vector ret_val;
  for (size_t i = 0; i < number_of_edges(); ++i) {
    if (get_edge(i).breakable) {
      ret_val.emplace_back(i);
    }
  }
  return ret_val;
}

cda_rail::index_vector cda_rail::Network::relevant_breakable_edges() const {
  /**
   * Returns indices of all breakable edges, but only once per direction.
   *
   * @return Vector of indices of breakable edges.
   */

  cda_rail::index_vector ret_val;
  for (size_t i = 0; i < number_of_edges(); ++i) {
    const auto& edge = get_edge(i);
    // add edge only if reverse edge does not exist or has larger index
    if (edge.breakable && (!has_edge(edge.target, edge.source) ||
                           get_edge_index(edge.target, edge.source) > i)) {
      ret_val.emplace_back(i);
    }
  }
  return ret_val;
}

std::vector<std::pair<size_t, cda_rail::index_vector>>
cda_rail::Network::discretize(const vss::SeparationFunction& sep_func) {
  /**
   * Discretizes the graphs edges to allow for VSS borders only at specified
   * positions / vertices.
   *
   * @param separation_type Type of separation.
   * @return Vector of pairs. The first element of the pair is the edge index of
   * the original edge. The second element is a vector of edge indices of the
   * new edges.
   */

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

bool cda_rail::Network::is_consistent_for_transformation() const {
  /**
   * Checks if the graph passes consistency checks needed for the
   * transformations to work correctly.
   * - If an edge is breakable it has a minimal block length that is strictly
   * positive.
   * - If an edge is breakable both it's source and target are type VSS or TTD
   * - For bidirectional edges, the breakable and length attributes are the same
   * for both directions.
   * - Vertices of type NoBorderVSS have at most two neighbors. These
   * neighbors are not of type NoBorder.
   *
   * @return True if the graph is consistent, false otherwise.
   */

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
      const auto& reverse_edge = get_edge(edge.target, edge.source);
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

std::vector<cda_rail::index_vector>
cda_rail::Network::unbreakable_sections() const {
  /**
   * Returns a vector of vectors of edge indices. Each vector of edge indices
   * represents an unbreakable section.
   *
   * @return Vector of vectors of edge indices. Each vector represents an
   * unbreakable section.
   */

  std::vector<cda_rail::index_vector> ret_val;

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
      ret_val.back().emplace_back(i);
      if (has_edge(edge.target, edge.source)) {
        ret_val.back().emplace_back(get_edge_index(edge.target, edge.source));
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

std::vector<cda_rail::index_vector>
cda_rail::Network::no_border_vss_sections() const {
  /**
   * Returns a vector of vectors of edge indices. Each vector of edge indices
   * represents a no border vss section. I.e., it includes sections containing
   * NoBorderVSS vertices that can be altered by an algorithm. These vertices
   * have likely been created by a transformation in advance.
   *
   * @return Vector of vectors of edge indices. Each vector represents a no
   * border vss section.
   */

  // Get possible start vertices for DFS
  std::unordered_set<size_t> vertices_to_visit;
  for (size_t i = 0; i < number_of_vertices(); ++i) {
    if (get_vertex(i).type == VertexType::NoBorderVSS &&
        !neighbors(i).empty()) {
      vertices_to_visit.emplace(i);
    }
  }

  std::vector<cda_rail::index_vector> ret_val;
  dfs_inplace(ret_val, vertices_to_visit, VertexType::NoBorderVSS,
              {VertexType::NoBorder});

  return ret_val;
}

void cda_rail::Network::change_vertex_type(size_t index, VertexType new_type) {
  /**
   * Changes the type of a specific vertex.
   *
   * @param index: Index of the vertex to change
   * @param new_type: New type of the vertex
   */

  if (!has_vertex(index)) {
    throw exceptions::VertexNotExistentException(index);
  }
  m_vertices[index].type = new_type;
}

std::vector<std::pair<std::optional<size_t>, std::optional<size_t>>>
cda_rail::Network::combine_reverse_edges(
    const cda_rail::index_vector& edges_to_consider, bool sort) const {
  /**
   * Given a vector of edges_to_consider, this function combines
   * edges_to_consider that are the reverse of each other. If no reverse of an
   * edge exists, the second element will be empty. A pair is only
   * added once, i.e., the first index is always smaller than the second for
   * uniqueness. The order of the edges_to_consider is not preserved. It throws
   * an error, if one of the edges_to_consider does not exist.
   *
   * @param edges: Vector of edge indices
   * @param sort: If true, the pairs are sorted by the neighboring relation.
   * This only works if the edges_to_consider correspond to a simple path!
   * @return: Vector of pairs of edge indices
   */

  if (!std::ranges::all_of(edges_to_consider,
                           [this](size_t i) { return has_edge(i); })) {
    throw exceptions::EdgeNotExistentException();
  }

  std::vector<std::pair<std::optional<size_t>, std::optional<size_t>>> ret_val;
  for (const auto& edge_index : edges_to_consider) {
    const auto reverse_edge_index = get_reverse_edge_index(edge_index);
    if (!reverse_edge_index.has_value() || reverse_edge_index > edge_index) {
      ret_val.emplace_back(edge_index, reverse_edge_index);
    }
  }

  if (sort) {
    return sort_edge_pairs(ret_val);
  }
  return ret_val;
}

std::optional<size_t>
cda_rail::Network::get_reverse_edge_index(size_t edge_index) const {
  /**
   * Gets the reverse index of an edge. Empty if the reverse edge does not
   * exist. Throws an error if the edge does not exist.
   *
   * @param edge: Index of the edge
   * @return: Index of the reverse edge, empty if it does not exist
   */

  if (!has_edge(edge_index)) {
    throw exceptions::EdgeNotExistentException(edge_index);
  }

  if (const auto& edge = get_edge(edge_index);
      has_edge(edge.target, edge.source)) {
    return get_edge_index(edge.target, edge.source);
  }
  return {};
}

size_t cda_rail::Network::get_track_index(size_t edge_index) const {
  const auto reverse_edge_index = get_reverse_edge_index(edge_index);
  if (reverse_edge_index.has_value()) {
    return std::min(edge_index, reverse_edge_index.value());
  }
  return edge_index;
}

cda_rail::index_vector
cda_rail::Network::get_vertices_by_type(VertexType type) const {
  /**
   * Returns a vector of all vertices of a specific type.
   *
   * @param type: Type of the vertices
   * @return: Vector of vertex indices
   */

  cda_rail::index_vector ret_val;
  for (size_t i = 0; i < m_vertices.size(); ++i) {
    if (m_vertices[i].type == type) {
      ret_val.emplace_back(i);
    }
  }
  return ret_val;
}

std::optional<size_t> cda_rail::Network::common_vertex(
    const std::pair<std::optional<size_t>, std::optional<size_t>>& pair1,
    const std::pair<std::optional<size_t>, std::optional<size_t>>& pair2)
    const {
  /**
   * Returns the common vertex of two edge pairs, if it exists. Otherwise
   * optional is empty. Throws an error if the pairs are not reverse of each
   * other.
   *
   * @param pair1: First pair of edge indices
   * @param pair2: Second pair of edge indices
   *
   * @return: Index of the common vertex, empty if it does not exist
   */

  if (!pair1.first.has_value() || !pair2.first.has_value()) {
    throw exceptions::InvalidInputException("Pairs first entry is empty");
  }

  if (!has_edge(pair1.first.value()) || !has_edge(pair2.first.value())) {
    throw exceptions::EdgeNotExistentException();
  }

  if (get_reverse_edge_index(pair1.first) != pair1.second) {
    throw exceptions::ConsistencyException(
        "First pair is not reverse of each other");
  }
  if (get_reverse_edge_index(pair2.first) != pair2.second) {
    throw exceptions::ConsistencyException(
        "Second pair is not reverse of each other");
  }

  std::optional<size_t> ret_val;
  const auto&           edge1 = get_edge(pair1.first.value());
  if (const auto& edge2 = get_edge(pair2.first.value());
      edge1.source == edge2.source || edge1.source == edge2.target) {
    ret_val = edge1.source;
  } else if (edge1.target == edge2.source || edge1.target == edge2.target) {
    ret_val = edge1.target;
  }

  return ret_val;
}

cda_rail::index_vector cda_rail::Network::inverse_edges(
    const cda_rail::index_vector& edge_indices,
    const cda_rail::index_vector& edges_to_consider) const {
  /**
   * Returns a vector of edge indices that are the inverse of the given edge
   * indices, i.e., return edges_to_consider - edge_indices
   *
   * @param edge_indices: Vector of edge indices
   * @param edges_to_consider: Vector of edge indices to consider
   * @return: Vector of edge indices that are the inverse of the given edge
   * indices
   */

  if (!std::ranges::all_of(edge_indices,
                           [this](size_t i) { return has_edge(i); })) {
    throw exceptions::EdgeNotExistentException();
  }
  if (!std::ranges::all_of(edges_to_consider,
                           [this](size_t i) { return has_edge(i); })) {
    throw exceptions::EdgeNotExistentException();
  }

  cda_rail::index_vector ret_val;
  for (const auto& edge_index : edges_to_consider) {
    if (!std::ranges::contains(edge_indices, edge_index)) {
      ret_val.emplace_back(edge_index);
    }
  }

  return ret_val;
}

int cda_rail::Network::max_vss_on_edge(size_t index) const {
  /**
   * Returns how many vss can be placed on a certain edge at most.
   *
   * @param index: Index of the edge
   *
   * @return: Number of vss that can be placed on the edge at most
   */

  const auto& edge = get_edge(index);
  if (!edge.breakable || edge.min_block_length == 0) {
    return 0;
  }
  return static_cast<int>(std::floor(edge.length / edge.min_block_length));
}

std::vector<std::vector<double>>
cda_rail::Network::all_edge_pairs_shortest_paths() const {
  /**
   * Calculates all shortest paths between all edges.
   * Given e0 = (v0, v1) and e1 = (v2, v3), the distance refers to the distance
   * between v1 and v3 by only using valid successors. If v0 or v2 are of
   * interest the value has to be post-processed accordingly. The distance is
   * std::numeric_limits<double>::max()/3 if no path exists. This methods uses
   * the Floyd-Warshall algorithm.
   *
   * @return: Matrix of distances between all edges
   */
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

std::optional<double>
cda_rail::Network::shortest_path(size_t source_edge_id, size_t target_id,
                                 bool target_is_edge, bool include_first_edge,
                                 bool use_minimal_time, double max_v) const {
  /**
   * Calculates the shortest path from a source edge e to a target vertex w.
   * If e = (u,v), then the length of the shortest path between v and w is
   * returned. However, only valid successors of e can be used as a first edge.
   * If no path exists, the optional has no value.
   *
   * @param source_edge_id: Index of the source edge e.
   * @param target_id: Index of the target vertex or edge.
   * @param target_is_edge: If true, the target is an edge, otherwise it is a
   * vertex.
   * @param use_minimal_time: If true, the minimal time is used instead of the
   * distance
   * @param max_v: Maximum speed of the train considered.
   */

  return shortest_path_using_edges(source_edge_id, target_id, true, {},
                                   target_is_edge, include_first_edge,
                                   use_minimal_time, max_v)
      .first;
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

cda_rail::index_set cda_rail::Network::vertices_used_by_edges(
    const cda_rail::index_set& edges_tmp) const {
  std::unordered_set<size_t> used_vertices;
  for (const auto& edge : edges_tmp) {
    used_vertices.insert(get_edge(edge).source);
    used_vertices.insert(get_edge(edge).target);
  }
  return used_vertices;
}

double cda_rail::Network::maximal_vertex_speed(
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

cda_rail::index_set
cda_rail::Network::neighboring_edges(size_t const index) const {
  auto       ret_val      = in_edges(index);
  const auto edges_to_add = out_edges(index);
  ret_val.insert(edges_to_add.begin(), edges_to_add.end());
  return ret_val;
}

double cda_rail::Network::minimal_neighboring_edge_length(
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

std::vector<std::pair<size_t, size_t>> cda_rail::Network::get_intersecting_ttd(
    const cda_rail::index_vector&              edges_to_consider,
    const std::vector<cda_rail::index_vector>& ttd) {
  /**
   * Returns the intersecting ttd sections, together with the entering, i.e.,
   * first edge. The edge is returned using the index within the path.
   */

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

std::pair<size_t, double>
cda_rail::Network::get_old_edge(size_t new_edge) const {
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

void cda_rail::Network::change_vertex_headway(size_t index,
                                              double new_headway) {
  if (!has_vertex(index)) {
    throw exceptions::VertexNotExistentException(index);
  }
  m_vertices[index].headway = new_headway;
}

cda_rail::index_vector
cda_rail::Network::get_unbreakable_section_containing_edge(size_t e) const {
  /**
   * This functions returns the unbreakable section that contains edge e as a
   * vector of edge indices.
   */

  const auto& edge_object = get_edge(e);
  if (edge_object.breakable) {
    return {};
  }

  cda_rail::index_vector ret_val;
  ret_val.emplace_back(e);
  const auto reverse_e = get_reverse_edge_index(e);
  if (reverse_e.has_value()) {
    ret_val.push_back(reverse_e.value());
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
            ret_val.push_back(e_tmp);
          }
        }
        if (has_edge(v_tmp, current_v)) {
          const auto& e_tmp = get_edge_index(v_tmp, current_v);
          if (!std::ranges::contains(ret_val, e_tmp)) {
            ret_val.push_back(e_tmp);
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

bool cda_rail::Network::is_on_same_unbreakable_section(size_t e1,
                                                       size_t e2) const {
  /**
   * This function returns true if, and only if, two edges are within the same
   * unbreakable section.
   */

  const auto section_tmp = get_unbreakable_section_containing_edge(e1);
  return std::ranges::contains(section_tmp, e2);
}

std::pair<std::optional<double>, cda_rail::index_vector>
cda_rail::Network::shortest_path_between_sets_using_edges(
    cda_rail::index_vector source_edge_ids, cda_rail::index_vector target_ids,
    bool only_use_valid_successors, cda_rail::index_vector edges_to_use,
    bool target_is_edge, bool include_first_edge, bool use_minimal_time,
    double max_v) const {
  /**
   * Calculates the shortest path from a source edge e to a target vertex w.
   * If e = (u,v), then the length of the shortest path between v and w is
   * returned. If no path exists, the optional has no value, and the vector is
   * empty. If only_use_valid_successors is true, only valid successors of e can
   * be used as a first edge and for all preceding edges. If edges_to_use is not
   * empty, only these edges are used, otherwise all edges are used.
   *
   * @param source_edge_ids: Index of the source edge e. Train can start on any
   * of the specified edges.
   * @param target_ids: Index of the target. Train can end on any of the
   * specified objects.
   * @param only_use_valid_successors: If true, only valid successors of the
   * source edge are used as first edge and for all preceding edges.
   * @param edges_to_use: If not empty, only these edges are used, otherwise all
   * edges are used.
   * @param target_is_edge: If true, the target is an edge, otherwise it is a
   * vertex.
   * @param include_first_edge: If true, the first edge is included in the
   * length, otherwise it is not.
   * @param use_minimal_time: If true, the minimal time is used instead of the
   * distance.
   * @param max_v: Maximum speed of the train considered.
   *
   * @return: A pair containing the distance and the path as a vector of edge
   */

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

double
cda_rail::Network::length_of_path(const cda_rail::index_vector& path) const {
  /**
   * Calculates the length of a path given by a vector of edge indices.
   * @param path: Vector of edge indices
   * @return: Length of the path
   */

  double len = 0.0;
  for (const auto& edge_index : path) {
    len += get_edge(edge_index).length;
  }
  return len;
}

std::vector<cda_rail::index_vector> cda_rail::Network::all_paths_ending_at_ttd(
    size_t e_0, const std::vector<cda_rail::index_vector>& ttd_sections,
    std::optional<size_t> exit_node) const {
  /**
   * All paths starting after(!) e_0 and ending in a TTD section.
   */
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

std::string cda_rail::Network::get_edge_name(const std::string_view v1,
                                             const std::string_view v2,
                                             bool const checkExistence) const {
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
