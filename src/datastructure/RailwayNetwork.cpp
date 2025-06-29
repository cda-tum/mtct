#include "datastructure/RailwayNetwork.hpp"

#include "CustomExceptions.hpp"
#include "Definitions.hpp"
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

void cda_rail::Network::get_keys(tinyxml2::XMLElement* graphml_body,
                                 std::string& breakable, std::string& length,
                                 std::string& max_speed,
                                 std::string& min_block_length,
                                 std::string& min_stop_block_length,
                                 std::string& type, std::string& headway) {
  /**
   * Get keys from graphml file
   * @param graphml_body Body of graphml file
   * @param breakable Breakable key
   * @param length Length key
   * @param max_speed Max speed key
   * @param min_block_length Min block length key
   * @param min_stop_block_length Min stop block length key
   * @param type Type key
   * @param headway Headway key
   *
   * The variables are passed by reference and are modified in place.
   */

  tinyxml2::XMLElement* graphml_key = graphml_body->FirstChildElement("key");
  while (graphml_key != nullptr) {
    if (graphml_key->Attribute("attr.name") == std::string("breakable")) {
      breakable = graphml_key->Attribute("id");
    } else if (graphml_key->Attribute("attr.name") ==
               std::string("min_block_length")) {
      min_block_length = graphml_key->Attribute("id");
    } else if (graphml_key->Attribute("attr.name") ==
               std::string("max_speed")) {
      max_speed = graphml_key->Attribute("id");
    } else if (graphml_key->Attribute("attr.name") == std::string("length")) {
      length = graphml_key->Attribute("id");
    } else if (graphml_key->Attribute("attr.name") == std::string("type")) {
      type = graphml_key->Attribute("id");
    } else if (graphml_key->Attribute("attr.name") ==
               std::string("min_stop_block_length")) {
      min_stop_block_length = graphml_key->Attribute("id");
    } else if (graphml_key->Attribute("attr.name") == std::string("headway")) {
      headway = graphml_key->Attribute("id");
    }
    graphml_key = graphml_key->NextSiblingElement("key");
  }
}

void cda_rail::Network::add_vertices_from_graphml(
    const tinyxml2::XMLElement* graphml_node, const std::string& type,
    const std::string& headway) {
  /**
   * Add vertices from graphml file
   * @param graphml_node Node of graphml file
   * @param network Network object
   * @param type Type key
   * @param headway Headway key
   *
   * The vertices are added to the network object in place.
   */

  while (graphml_node != nullptr) {
    const tinyxml2::XMLElement* graphml_data =
        graphml_node->FirstChildElement("data");
    std::string const     name = graphml_node->Attribute("id");
    std::optional<int>    v_type;
    std::optional<double> headway_value;
    while (graphml_data != nullptr) {
      if (graphml_data->Attribute("key") == type) {
        v_type = std::stoi(graphml_data->GetText());
      } else if (!headway.empty() &&
                 graphml_data->Attribute("key") == headway) {
        headway_value = std::stod(graphml_data->GetText());
      }
      graphml_data = graphml_data->NextSiblingElement("data");
    }
    if (!v_type.has_value()) {
      throw exceptions::ImportException("graphml");
    }
    if (headway_value.has_value()) {
      this->add_vertex(name, static_cast<VertexType>(v_type.value()),
                       headway_value.value());
    } else {
      this->add_vertex(name, static_cast<VertexType>(v_type.value()));
    }
    graphml_node = graphml_node->NextSiblingElement("node");
  }
}

void cda_rail::Network::add_edges_from_graphml(
    const tinyxml2::XMLElement* graphml_edge, const std::string& breakable,
    const std::string& length, const std::string& max_speed,
    const std::string& min_block_length,
    const std::string& min_stop_block_length) {
  /**
   * Add edges from graphml file
   * @param graphml_edge Edge of graphml file
   * @param network Network object
   * @param breakable Breakable key
   * @param length Length key
   * @param max_speed Max speed key
   * @param min_block_length Min block length key
   *
   * The edges are added to the network object in place.
   */

  while (graphml_edge != nullptr) {
    const tinyxml2::XMLElement* graphml_data =
        graphml_edge->FirstChildElement("data");
    std::string const     source_name = graphml_edge->Attribute("source");
    std::string const     target_name = graphml_edge->Attribute("target");
    std::optional<double> e_length;
    std::optional<double> e_max_speed;
    std::optional<bool>   e_breakable;
    std::optional<double> e_min_block_length;
    std::optional<double> e_min_stop_block_length;
    while (graphml_data != nullptr) {
      if (graphml_data->Attribute("key") == breakable) {
        std::string tmp = graphml_data->GetText();
        to_bool_optional(tmp, e_breakable);
      } else if (graphml_data->Attribute("key") == min_block_length) {
        e_min_block_length = std::stod(graphml_data->GetText());
      } else if (graphml_data->Attribute("key") == max_speed) {
        e_max_speed = std::stod(graphml_data->GetText());
      } else if (graphml_data->Attribute("key") == length) {
        e_length = std::stod(graphml_data->GetText());
      } else if (!min_stop_block_length.empty() &&
                 graphml_data->Attribute("key") == min_stop_block_length) {
        e_min_stop_block_length = std::stod(graphml_data->GetText());
      }
      graphml_data = graphml_data->NextSiblingElement("data");
    }
    if (!e_length.has_value() || !e_max_speed.has_value() ||
        !e_breakable.has_value() || !e_min_block_length.has_value()) {
      throw exceptions::ImportException("graphml");
    }
    if (e_min_stop_block_length.has_value()) {
      this->add_edge(source_name, target_name, e_length.value(),
                     e_max_speed.value(), e_breakable.value(),
                     e_min_block_length.value(),
                     e_min_stop_block_length.value());
    } else {
      this->add_edge(source_name, target_name, e_length.value(),
                     e_max_speed.value(), e_breakable.value(),
                     e_min_block_length.value());
    }
    graphml_edge = graphml_edge->NextSiblingElement("edge");
  }
}

void cda_rail::Network::read_graphml(const std::filesystem::path& p) {
  /**
   * Read network graph from XML file into the object
   * @param path Path to XML file
   */

  tinyxml2::XMLDocument graph_xml;
  graph_xml.LoadFile((p / "tracks.graphml").string().c_str());
  if (graph_xml.Error()) {
    throw exceptions::ImportException("graphml");
  }

  tinyxml2::XMLElement* graphml_body = graph_xml.FirstChildElement("graphml");

  std::string breakable;
  std::string length;
  std::string max_speed;
  std::string min_block_length;
  std::string min_stop_block_length;
  std::string type;
  std::string headway;
  Network::get_keys(graphml_body, breakable, length, max_speed,
                    min_block_length, min_stop_block_length, type, headway);
  if (breakable.empty() || length.empty() || max_speed.empty() ||
      min_block_length.empty() || type.empty()) {
    throw exceptions::ImportException("graphml");
  }

  const tinyxml2::XMLElement* graphml_graph =
      graphml_body->FirstChildElement("graph");
  if ((graphml_graph->Attribute("edgedefault")) != std::string("directed")) {
    throw exceptions::InvalidInputException("Graph is not directed");
  }

  const tinyxml2::XMLElement* graphml_node =
      graphml_graph->FirstChildElement("node");
  this->add_vertices_from_graphml(graphml_node, type, headway);

  const tinyxml2::XMLElement* graphml_edge =
      graphml_graph->FirstChildElement("edge");
  this->add_edges_from_graphml(graphml_edge, breakable, length, max_speed,
                               min_block_length, min_stop_block_length);
}

void cda_rail::Network::read_successors(const std::filesystem::path& p) {
  /**
   * Read successors from path
   * @param path Path to successors file
   */

  std::ifstream f((p / "successors_cpp.json"));
  json          data = json::parse(f);

  for (const auto& [key, val] : data.items()) {
    std::string source_name;
    std::string target_name;
    extract_vertices_from_key(key, source_name, target_name);
    auto const edge_id_in = get_edge_index(source_name, target_name);
    for (auto& tuple : val) {
      auto const edge_id_out = get_edge_index(tuple[0].get<std::string>(),
                                              tuple[1].get<std::string>());
      add_successor(edge_id_in, edge_id_out);
    }
  }
}

size_t cda_rail::Network::add_vertex(const std::string& name, VertexType type,
                                     double headway) {
  /**
   * Add vertex to network
   * @param name Name of vertex
   * @param type Type of vertex
   * @param headway Headway of vertex
   *
   * @return Index of vertex
   */
  if (has_vertex(name)) {
    throw exceptions::InvalidInputException("Vertex already exists");
  }
  vertices.emplace_back(name, type, headway);
  vertex_name_to_index[name] = vertices.size() - 1;
  return vertex_name_to_index[name];
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
  if (source == target) {
    throw exceptions::InvalidInputException("Source and target are the same");
  }
  if (!has_vertex(source)) {
    throw exceptions::VertexNotExistentException(source);
  }
  if (!has_vertex(target)) {
    throw exceptions::VertexNotExistentException(target);
  }
  if (has_edge(source, target)) {
    throw exceptions::InvalidInputException("Edge already exists");
  }
  edges.emplace_back(source, target, length, max_speed, breakable,
                     min_block_length, min_stop_block_length);
  successors.emplace_back();
  return edges.size() - 1;
}

void cda_rail::Network::add_successor(size_t edge_in, size_t edge_out) {
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
  if (edges[edge_in].target != edges[edge_out].source) {
    throw exceptions::ConsistencyException("Edge " + std::to_string(edge_out) +
                                           " is not adjacent to " +
                                           std::to_string(edge_in));
  }

  // If successors[edges] already contains edge_out, do nothing
  if (std::find(successors[edge_in].begin(), successors[edge_in].end(),
                edge_out) != successors[edge_in].end()) {
    return;
  }

  successors[edge_in].emplace_back(edge_out);
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
  return vertices[index];
}

size_t cda_rail::Network::get_vertex_index(const std::string& name) const {
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
  return vertex_name_to_index.at(name);
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
  return edges[index];
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
  for (const auto& edge : edges) {
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
  for (size_t i = 0; i < edges.size(); i++) {
    if (edges[i].source == source_id && edges[i].target == target_id) {
      return i;
    }
  }
  throw exceptions::EdgeNotExistentException(source_id, target_id);
}

bool cda_rail::Network::has_edge(size_t source_id, size_t target_id) const {
  /**
   * Check if edge exists by source and target index
   *
   * @param source_id Index of source vertex
   * @param target_id Index of target vertex
   *
   * @return True if edge exists, false otherwise
   */
  if (!has_vertex(source_id)) {
    throw exceptions::VertexNotExistentException(source_id);
  }
  if (!has_vertex(target_id)) {
    throw exceptions::VertexNotExistentException(target_id);
  }
  return std::any_of(
      edges.begin(), edges.end(), [source_id, target_id](const Edge& edge) {
        return edge.source == source_id && edge.target == target_id;
      });
}

bool cda_rail::Network::has_edge(const std::string& source_name,
                                 const std::string& target_name) const {
  /**
   * Check if edge exists by source and target name
   *
   * @param source_name Name of source vertex
   * @param target_name Name of target vertex
   *
   * @return True if edge exists, false otherwise
   */
  if (!has_vertex(source_name)) {
    throw exceptions::VertexNotExistentException(source_name);
  }
  if (!has_vertex(target_name)) {
    throw exceptions::VertexNotExistentException(target_name);
  }
  return has_edge(get_vertex_index(source_name), get_vertex_index(target_name));
}

void cda_rail::Network::change_vertex_name(size_t             index,
                                           const std::string& new_name) {
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
  vertex_name_to_index.erase(vertices[index].name);
  vertices[index].name           = new_name;
  vertex_name_to_index[new_name] = index;
}

void cda_rail::Network::change_edge_length(size_t index, double new_length) {
  /**
   * Change edge length
   *
   * @param index Index of edge
   * @param new_length New length of edge
   */
  if (!has_edge(index)) {
    throw exceptions::EdgeNotExistentException(index);
  }
  edges[index].length = new_length;
}

void cda_rail::Network::change_edge_max_speed(size_t index,
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
  edges[index].max_speed = new_max_speed;
}

void cda_rail::Network::change_edge_min_block_length(
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
  edges[index].min_block_length = new_min_block_length;
}

void cda_rail::Network::change_edge_min_stop_block_length(
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
  edges[index].min_stop_block_length = new_min_stop_block_length;
}

void cda_rail::Network::set_edge_breakable(size_t index) {
  /**
   * Sets an edge to be breakable
   *
   * @param index Index of edge
   */
  if (!has_edge(index)) {
    throw exceptions::EdgeNotExistentException(index);
  }
  edges[index].breakable = true;
}

void cda_rail::Network::set_edge_unbreakable(size_t index) {
  /**
   * Sets an edge to be unbreakable
   *
   * @param index Index of edge
   */
  if (!has_edge(index)) {
    throw exceptions::EdgeNotExistentException(index);
  }
  edges[index].breakable = false;
}

std::vector<size_t> cda_rail::Network::out_edges(size_t index) const {
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
  std::vector<size_t> out_edges;
  for (size_t i = 0; i < edges.size(); ++i) {
    if (edges[i].source == index) {
      out_edges.emplace_back(i);
    }
  }
  return out_edges;
}

std::vector<size_t> cda_rail::Network::in_edges(size_t index) const {
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
  std::vector<size_t> in_edges;
  for (size_t i = 0; i < edges.size(); ++i) {
    if (edges[i].target == index) {
      in_edges.emplace_back(i);
    }
  }
  return in_edges;
}

const std::vector<size_t>&
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
  return successors[index];
}

std::vector<size_t> cda_rail::Network::get_predecessors(size_t index) const {
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
  std::vector<size_t> ret_val;

  for (const auto& e_1 : in_edges(get_edge(index).source)) {
    if (is_valid_successor(e_1, index)) {
      ret_val.push_back(e_1);
    }
  }

  return ret_val;
}

void cda_rail::to_bool_optional(std::string& s, std::optional<bool>& b) {
  /**
   * Converts a string to an optional bool
   *
   * @param s String to convert
   * @param b Optional bool to write to, used as return value
   */
  std::transform(s.begin(), s.end(), s.begin(), ::tolower);
  bool tmp = false;
  if (!!(std::istringstream(s) >> std::boolalpha >> tmp)) {
    b = tmp;
  }
}

void cda_rail::Network::export_graphml(const std::filesystem::path& p) const {
  /**
   * Export the network to a GraphML file in the given path.
   * @param path: The path to the directory.
   */

  // Open file
  std::ofstream file(p / "tracks.graphml");

  // Write the header.
  file << "<?xml version='1.0' encoding='UTF-8'?>" << '\n';
  file
      << R"(<graphml xmlns="http://graphml.graphdrawing.org/xmlns" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="http://graphml.graphdrawing.org/xmlns http://graphml.graphdrawing.org/xmlns/1.0/graphml.xsd">)"
      << '\n';

  // Write the key relations
  std::string const breakable             = "d0";
  std::string const length                = "d1";
  std::string const max_speed             = "d2";
  std::string const min_block_length      = "d3";
  std::string const min_stop_block_length = "d4";
  std::string const type                  = "d5";
  std::string const headway               = "d6";
  file << "<key id=\"" << breakable
       << R"(" for="edge" attr.name="breakable" attr.type="boolean"/>)" << '\n';
  file << "<key id=\"" << length
       << R"(" for="edge" attr.name="length" attr.type="double"/>)" << '\n';
  file << "<key id=\"" << max_speed
       << R"(" for="edge" attr.name="max_speed" attr.type="double"/>)" << '\n';
  file << "<key id=\"" << min_block_length
       << R"(" for="edge" attr.name="min_block_length" attr.type="double"/>)"
       << '\n';
  file
      << "<key id=\"" << min_stop_block_length
      << R"(" for="edge" attr.name="min_stop_block_length" attr.type="double"/>)"
      << '\n';
  file << "<key id=\"" << type
       << R"(" for="vertex" attr.name="type" attr.type="long"/>)" << '\n';
  file << "<key id=\"" << headway
       << R"(" for="vertex" attr.name="headway" attr.type="double"/>)" << '\n';

  // Write the graph header
  file << "<graph edgedefault=\"directed\">" << '\n';

  // Write the vertices
  for (const auto& vertex : vertices) {
    file << "<node id=\"" << vertex.name << "\">" << '\n';
    file << "<data key=\"" << type << "\">" << static_cast<int>(vertex.type)
         << "</data>" << '\n';
    file << "<data key=\"" << headway << "\">" << vertex.headway << "</data>"
         << '\n';
    file << "</node>" << '\n';
  }

  // Write the edges
  for (const auto& edge : edges) {
    file << "<edge source=\"" << vertices[edge.source].name << "\" target=\""
         << vertices[edge.target].name << "\">" << '\n';
    file << "<data key=\"" << breakable << "\">" << std::boolalpha
         << edge.breakable << "</data>" << '\n';
    file << "<data key=\"" << length << "\">" << edge.length << "</data>"
         << '\n';
    file << "<data key=\"" << max_speed << "\">" << edge.max_speed << "</data>"
         << '\n';
    file << "<data key=\"" << min_block_length << "\">" << edge.min_block_length
         << "</data>" << '\n';
    file << "<data key=\"" << min_stop_block_length << "\">"
         << edge.min_stop_block_length << "</data>" << '\n';
    file << "</edge>" << '\n';
  }

  // Write the footer
  file << "</graph>" << '\n';
  file << "</graphml>" << '\n';

  // Close the file.
  file.close();
}

void cda_rail::Network::export_successors_cpp(
    const std::filesystem::path& p) const {
  /**
   * Export the successors to successors_cpp.json for C++ in the given
   * directory.
   * @param path: The path to the directory.
   */

  json j;
  for (size_t i = 0; i < number_of_edges(); ++i) {
    const auto&                                      edge = get_edge(i);
    std::vector<std::pair<std::string, std::string>> successor_edges_export;
    for (const auto& successor : successors[i]) {
      const auto& successor_edge = get_edge(successor);
      successor_edges_export.emplace_back(vertices[successor_edge.source].name,
                                          vertices[successor_edge.target].name);
    }
    j["('" + vertices[edge.source].name + "', '" + vertices[edge.target].name +
      "')"] = successor_edges_export;
  }

  std::ofstream file(p / "successors_cpp.json");
  file << j << '\n';
}

void cda_rail::Network::export_successors_python(
    const std::filesystem::path& p) const {
  /**
   * Export the successors to successors.txt for Python in the given directory.
   * @param path: The path to the directory.
   */

  std::ofstream file(p / "successors.txt");

  file << "{";
  bool first_key = true;
  for (size_t i = 0; i < number_of_edges(); ++i) {
    const auto& edge = get_edge(i);

    if (first_key) {
      first_key = false;
    } else {
      file << ", ";
    }

    file << "('" << vertices[edge.source].name << "', '"
         << vertices[edge.target].name << "'): ";
    write_successor_set_to_file(file, i);
  }
  file << "}" << '\n';
}

void cda_rail::Network::write_successor_set_to_file(std::ofstream& file,
                                                    size_t         i) const {
  /**
   * Write the successor set to the given file.
   * @param file: The file to write to.
   * @param i: The index of the edge.
   */

  // If there are no successors, write set(), otherwise write all successors.
  if (get_successors(i).empty()) {
    file << "set()";
  } else {
    file << "{";
    bool first_element = true;
    for (const auto successor : get_successors(i)) {
      // Write comma if not first element
      if (first_element) {
        first_element = false;
      } else {
        file << ", ";
      }

      const auto& successor_edge = get_edge(successor);
      file << "('" << vertices[successor_edge.source].name << "', '"
           << vertices[successor_edge.target].name << "')";
    }
    file << "}";
  }
}

void cda_rail::Network::export_network(const std::filesystem::path& p) const {
  /** Export the network to a given directory. This includes the graphml and
   * successors files.
   * @param path: The path to the directory.
   */

  if (!is_directory_and_create(p)) {
    throw exceptions::ExportException("Could not create directory " +
                                      p.string());
  }

  export_graphml(p);

  export_successors_cpp(p);
  export_successors_python(p);
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

  if (edges[e0].target != edges[e1].source) {
    return false;
  }
  return (std::find(successors[e0].begin(), successors[e0].end(), e1) !=
          successors[e0].end());
}

std::vector<size_t> cda_rail::Network::neighbors(size_t index) const {
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
  std::vector<size_t> neighbors;
  auto                e_out = out_edges(index);
  auto                e_in  = in_edges(index);
  for (auto e : e_out) {
    if (std::find(neighbors.begin(), neighbors.end(), get_edge(e).target) ==
        neighbors.end()) {
      neighbors.emplace_back(get_edge(e).target);
    }
  }
  for (auto e : e_in) {
    if (std::find(neighbors.begin(), neighbors.end(), get_edge(e).source) ==
        neighbors.end()) {
      neighbors.emplace_back(get_edge(e).source);
    }
  }
  return neighbors;
}

cda_rail::Network::Network(const std::filesystem::path& p) {
  /**
   * Construct object and read network from path. This includes the graph and
   * successors.
   * @param p Path to network directory
   * @return Network
   */

  if (!std::filesystem::exists(p)) {
    throw exceptions::ImportException("Path does not exist");
  }
  if (!std::filesystem::is_directory(p)) {
    throw exceptions::ImportException("Path is not a directory");
  }

  this->read_graphml(p);
  this->read_successors(p);
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

  if (vertices[vertex_id].type != VertexType::NoBorder) {
    return false;
  }

  if (neighbors(vertex_id).size() != 2) {
    return false;
  }

  return true;
}

std::pair<std::vector<size_t>, std::vector<size_t>>
cda_rail::Network::separate_edge_at(
    size_t edge_index, const std::vector<double>& distances_from_source,
    bool new_edge_breakable) {
  /**
   * This function separates an edge at given distances from the source vertex.
   * If the reverse edge exists it is separated analogously. In particular the
   * following is done:
   * - New vertices are created at the given distances from the source vertex
   * and edges in between them.
   * - The successors are updated accordingly.
   * - The previous edge(s) are removed (implicitly).
   *
   * @param edge_index: The index of the edge to separate.
   * @param distances_from_source: The distances from the source vertex at which
   * to separate the edge.
   * @return: A pair of vectors containing the indices of the new edges and
   * reverse edges, where the latter might have size 0.
   */

  if (!has_edge(edge_index)) {
    throw exceptions::EdgeNotExistentException(edge_index);
  }

  if (distances_from_source.empty()) {
    throw exceptions::InvalidInputException("Distances are not specified");
  }

  if (!std::is_sorted(distances_from_source.begin(),
                      distances_from_source.end())) {
    throw exceptions::ConsistencyException("Distances are not sorted");
  }

  const auto edge = get_edge(edge_index);

  if (distances_from_source.front() <= 0 ||
      distances_from_source.back() >= edge.length) {
    throw exceptions::ConsistencyException(
        "Distances are not strictly between 0 and the length of the edge");
  }

  std::vector<size_t> new_vertices;
  for (size_t i = 0; i < distances_from_source.size(); ++i) {
    std::string const vertex_name = get_vertex(edge.source).name + "_" +
                                    get_vertex(edge.target).name + "_" +
                                    std::to_string(i);
    new_vertices.emplace_back(add_vertex(vertex_name, VertexType::NoBorderVSS));
  }

  std::pair<std::vector<size_t>, std::vector<size_t>> return_edges;
  auto& new_edges = return_edges.first;
  new_edges.emplace_back(add_edge(edge.source, new_vertices.front(),
                                  distances_from_source.front(), edge.max_speed,
                                  new_edge_breakable, edge.min_block_length,
                                  edge.min_stop_block_length));
  update_new_old_edge(new_edges.back(), edge_index, 0);
  for (size_t i = 1; i < distances_from_source.size(); ++i) {
    new_edges.emplace_back(add_edge(
        new_vertices[i - 1], new_vertices[i],
        distances_from_source[i] - distances_from_source[i - 1], edge.max_speed,
        new_edge_breakable, edge.min_block_length, edge.min_stop_block_length));
    update_new_old_edge(new_edges.back(), edge_index,
                        distances_from_source[i - 1]);
  }
  change_edge_length(edge_index, edge.length - distances_from_source.back());
  update_new_old_edge(edge_index, edge_index, distances_from_source.back());
  if (!new_edge_breakable) {
    set_edge_unbreakable(edge_index);
  }
  edges[edge_index].source = new_vertices.back();
  new_edges.emplace_back(edge_index);

  // Update successors, i.e.,
  // - For every incoming edge into edge.source, replace edge_index by
  // new_edges.front() if applicable
  // - For every new_edge (except the last) add the next new_edge as single
  // successor
  // - For the last new edge add the same successors as edge_index had (this has
  // already been done implicitly)
  for (const auto& incoming_edge_index : in_edges(edge.source)) {
    std::replace(successors[incoming_edge_index].begin(),
                 successors[incoming_edge_index].end(), edge_index,
                 new_edges.front());
  }
  for (size_t i = 0; i < new_edges.size() - 1; ++i) {
    add_successor(new_edges[i], new_edges[i + 1]);
  }

  auto& new_reverse_edges = return_edges.second;
  if (has_edge(edge.target, edge.source)) {
    // Check if reverse edge has same length
    const auto reverse_edge_index = get_edge_index(edge.target, edge.source);
    const auto reverse_edge       = get_edge(reverse_edge_index);
    if (reverse_edge.length != edge.length) {
      throw exceptions::ConsistencyException(
          "Reverse edge has different length");
    }

    new_reverse_edges.emplace_back(add_edge(
        edge.target, new_vertices.back(),
        edge.length - distances_from_source.back(), reverse_edge.max_speed,
        new_edge_breakable, reverse_edge.min_block_length,
        reverse_edge.min_stop_block_length));
    update_new_old_edge(new_reverse_edges.back(), reverse_edge_index, 0);
    for (size_t i = distances_from_source.size() - 1; i > 0; --i) {
      new_reverse_edges.emplace_back(add_edge(
          new_vertices[i], new_vertices[i - 1],
          distances_from_source[i] - distances_from_source[i - 1],
          reverse_edge.max_speed, new_edge_breakable,
          reverse_edge.min_block_length, reverse_edge.min_stop_block_length));
      update_new_old_edge(new_reverse_edges.back(), reverse_edge_index,
                          reverse_edge.length - distances_from_source[i]);
    }
    change_edge_length(reverse_edge_index, distances_from_source.front());
    update_new_old_edge(reverse_edge_index, reverse_edge_index,
                        reverse_edge.length - distances_from_source.front());
    if (!new_edge_breakable) {
      set_edge_unbreakable(reverse_edge_index);
    }
    edges[reverse_edge_index].source = new_vertices.front();
    new_reverse_edges.emplace_back(reverse_edge_index);

    for (const auto& incoming_edge_index : in_edges(edge.target)) {
      std::replace(successors[incoming_edge_index].begin(),
                   successors[incoming_edge_index].end(), reverse_edge_index,
                   new_reverse_edges.front());
    }
    for (size_t i = 0; i < new_reverse_edges.size() - 1; ++i) {
      add_successor(new_reverse_edges[i], new_reverse_edges[i + 1]);
    }
  }

  return return_edges;
}

std::pair<std::vector<size_t>, std::vector<size_t>>
cda_rail::Network::separate_edge_private_helper(
    size_t edge_index, double min_length,
    const vss::SeparationFunction& sep_func, bool new_edge_breakable) {
  /**
   * Separates an edge (and possibly its reverse edge) according to the given
   * number of new vertices.
   *
   * @param edge_index Index of the edge to separate.
   * @param separation_type Type of separation.
   *
   * @return Pair of vectors of indices of new edges and reverse edges.
   */

  if (!is_consistent_for_transformation()) {
    throw exceptions::ConsistencyException();
  }
  if (!get_edge(edge_index).breakable) {
    throw exceptions::ConsistencyException("Edge is not breakable");
  }

  // Get edge to separate
  const auto& edge = get_edge(edge_index);
  // Get number of new vertices
  const auto number_of_blocks =
      vss::functions::max_n_blocks(sep_func, min_length / edge.length);

  // Calculate distances
  std::vector<double> distances_from_source;
  distances_from_source.reserve(number_of_blocks - 1);
  for (int i = 0; i < number_of_blocks - 1; ++i) {
    distances_from_source.emplace_back(edge.length *
                                       sep_func(i, number_of_blocks));
  }

  return separate_edge_at(edge_index, distances_from_source,
                          new_edge_breakable);
}

std::vector<size_t> cda_rail::Network::breakable_edges() const {
  /**
   * Returns indices of all breakable edges.
   *
   * @return Vector of indices of breakable edges.
   */

  std::vector<size_t> ret_val;
  for (size_t i = 0; i < number_of_edges(); ++i) {
    if (get_edge(i).breakable) {
      ret_val.emplace_back(i);
    }
  }
  return ret_val;
}

std::vector<size_t> cda_rail::Network::relevant_breakable_edges() const {
  /**
   * Returns indices of all breakable edges, but only once per direction.
   *
   * @return Vector of indices of breakable edges.
   */

  std::vector<size_t> ret_val;
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

std::vector<std::pair<size_t, std::vector<size_t>>>
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

  std::vector<std::pair<size_t, std::vector<size_t>>> ret_val;
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

      if (std::any_of(v_neighbors.begin(), v_neighbors.end(),
                      [this](const auto& j) {
                        return this->get_vertex(j).type == VertexType::NoBorder;
                      })) {
        return false;
      }
    }
  }

  return true;
}

std::vector<std::vector<size_t>>
cda_rail::Network::unbreakable_sections() const {
  /**
   * Returns a vector of vectors of edge indices. Each vector of edge indices
   * represents an unbreakable section.
   *
   * @return Vector of vectors of edge indices. Each vector represents an
   * unbreakable section.
   */

  std::vector<std::vector<size_t>> ret_val;

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

  dfs(ret_val, vertices_to_visit, VertexType::NoBorder);

  return ret_val;
}

std::vector<std::vector<size_t>>
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

  std::vector<std::vector<size_t>> ret_val;
  dfs(ret_val, vertices_to_visit, VertexType::NoBorderVSS,
      {VertexType::NoBorder});

  return ret_val;
}

void cda_rail::Network::dfs(std::vector<std::vector<size_t>>& ret_val,
                            std::unordered_set<size_t>&       vertices_to_visit,
                            const VertexType&                 section_type,
                            const std::vector<VertexType>& error_types) const {
  /**
   * Performs DFS on the graph to find sections whose inner vertices are of the
   * specified type.
   *
   * @param ret_val: Vector of vectors of edge indices. Each vector represents a
   * section. Used as return value.
   * @param vertices_to_visit: Set of vertices to visit.
   * @param section_type: Type that defines sections, i.e., all inner vertices
   * of a section are of this type.
   * @param error_types: Types that are not allowed to be in a section.
   */

  while (!vertices_to_visit.empty()) {
    ret_val.emplace_back();

    std::stack<int> stack;
    stack.emplace(*vertices_to_visit.begin());
    std::vector<int> visited_vertices;

    // DFS
    while (!stack.empty()) {
      int const current_vertex = stack.top();
      stack.pop();
      visited_vertices.emplace_back(current_vertex);
      if (vertices_to_visit.find(current_vertex) != vertices_to_visit.end()) {
        vertices_to_visit.erase(current_vertex);
      }

      const auto neighbor_vertices = neighbors(current_vertex);
      for (const auto& neighbor : neighbor_vertices) {
        if (get_vertex(neighbor).type == section_type &&
            std::find(visited_vertices.begin(), visited_vertices.end(),
                      neighbor) == visited_vertices.end()) {
          stack.emplace(neighbor);
        }
        if (std::find(error_types.begin(), error_types.end(),
                      get_vertex(neighbor).type) != error_types.end()) {
          throw exceptions::ConsistencyException(
              "This should never happen, but I found error type vertex");
        }

        if (has_edge(current_vertex, neighbor)) {
          const auto edge_index = get_edge_index(current_vertex, neighbor);
          // If edge is breakable throw error
          if (get_edge(edge_index).breakable) {
            throw exceptions::ConsistencyException(
                "This should never happen, but I found a breakable edge in an "
                "unbreakable section");
          }

          if (std::find(ret_val.back().begin(), ret_val.back().end(),
                        edge_index) == ret_val.back().end()) {
            ret_val.back().emplace_back(edge_index);
          }
        }

        if (has_edge(neighbor, current_vertex)) {
          const auto edge_index = get_edge_index(neighbor, current_vertex);

          if (get_edge(edge_index).breakable) {
            throw exceptions::ConsistencyException(
                "This should never happen, but I found a breakable edge in an "
                "unbreakable section");
          }

          if (std::find(ret_val.back().begin(), ret_val.back().end(),
                        edge_index) == ret_val.back().end()) {
            ret_val.back().emplace_back(edge_index);
          }
        }
      }
    }
  }
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
  vertices[index].type = new_type;
}

std::vector<std::pair<std::optional<size_t>, std::optional<size_t>>>
cda_rail::Network::combine_reverse_edges(
    const std::vector<size_t>& edges_to_consider, bool sort) const {
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

  if (!std::all_of(edges_to_consider.begin(), edges_to_consider.end(),
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

std::vector<size_t>
cda_rail::Network::get_vertices_by_type(VertexType type) const {
  /**
   * Returns a vector of all vertices of a specific type.
   *
   * @param type: Type of the vertices
   * @return: Vector of vertex indices
   */

  std::vector<size_t> ret_val;
  for (size_t i = 0; i < vertices.size(); ++i) {
    if (vertices[i].type == type) {
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

std::vector<std::pair<std::optional<size_t>, std::optional<size_t>>>
cda_rail::Network::sort_edge_pairs(
    std::vector<std::pair<std::optional<size_t>, std::optional<size_t>>>&
        edge_pairs) const {
  /**
   * Sort a vector of edge pairs so that neighboring pairs are adjacent
   *
   * @param edges: Vector of edge pairs
   *
   * @return: Sorted vector of edge pairs
   */

  if (!std::all_of(
          edge_pairs.begin(), edge_pairs.end(),
          [](const auto& edge_pair) { return edge_pair.first.has_value(); })) {
    throw exceptions::InvalidInputException("Edge pair first entry is empty");
  }
  if (!std::all_of(edge_pairs.begin(), edge_pairs.end(),
                   [this](const auto& edge_pair) {
                     return has_edge(edge_pair.first.value());
                   })) {
    throw exceptions::EdgeNotExistentException();
  }

  if (!std::all_of(edge_pairs.begin(), edge_pairs.end(),
                   [this](const auto& edge_pair) {
                     return get_reverse_edge_index(edge_pair.first.value()) ==
                            edge_pair.second;
                   })) {
    throw exceptions::ConsistencyException(
        "Pairs are not reverse of each other");
  }

  std::unordered_map<size_t, std::unordered_set<size_t>> vertex_neighbors;
  for (size_t i = 0; i < edge_pairs.size(); ++i) {
    const auto& edge_pair = edge_pairs[i];
    // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
    const auto& edge = get_edge(edge_pair.first.value());
    vertex_neighbors[edge.source].emplace(i);
    vertex_neighbors[edge.target].emplace(i);
  }

  size_t j = 0;
  for (j = 0; j < number_of_vertices(); ++j) {
    if (vertex_neighbors[j].size() == 1) {
      break;
    }
  }

  std::vector<std::pair<std::optional<size_t>, std::optional<size_t>>> ret_val;
  while (j < number_of_vertices()) {
    if (vertex_neighbors[j].empty()) {
      break;
    }
    if (vertex_neighbors[j].size() > 1) {
      throw exceptions::ConsistencyException(
          "Something went wrong, vertex has more than one neighbor still.");
    }

    const auto  edge_pair_index = *vertex_neighbors[j].begin();
    const auto& edge_pair       = edge_pairs[edge_pair_index];

    // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
    const auto& edge = get_edge(edge_pair.first.value());
    vertex_neighbors[edge.source].erase(edge_pair_index);
    vertex_neighbors[edge.target].erase(edge_pair_index);

    if (edge.source != j && edge.target == j) {
      ret_val.emplace_back(edge_pair.second, edge_pair.first);
      j = edge.source;
    } else if (edge.target != j && edge.source == j) {
      ret_val.emplace_back(edge_pair.first, edge_pair.second);
      j = edge.target;
    } else {
      throw exceptions::ConsistencyException(
          "Something went wrong, source and target are not as expected.");
    }
  }

  for (size_t i = 0; i < number_of_vertices(); ++i) {
    if (!vertex_neighbors[i].empty()) {
      throw exceptions::ConsistencyException(
          "Something went wrong, not everything was processed.");
    }
  }

  return ret_val;
}

std::vector<size_t> cda_rail::Network::inverse_edges(
    const std::vector<size_t>& edge_indices,
    const std::vector<size_t>& edges_to_consider) const {
  /**
   * Returns a vector of edge indices that are the inverse of the given edge
   * indices, i.e., return edges_to_consider - edge_indices
   *
   * @param edge_indices: Vector of edge indices
   * @param edges_to_consider: Vector of edge indices to consider
   * @return: Vector of edge indices that are the inverse of the given edge
   * indices
   */

  if (!std::all_of(edge_indices.begin(), edge_indices.end(),
                   [this](size_t i) { return has_edge(i); })) {
    throw exceptions::EdgeNotExistentException();
  }
  if (!std::all_of(edges_to_consider.begin(), edges_to_consider.end(),
                   [this](size_t i) { return has_edge(i); })) {
    throw exceptions::EdgeNotExistentException();
  }

  std::vector<size_t> ret_val;
  for (const auto& edge_index : edges_to_consider) {
    if (std::find(edge_indices.begin(), edge_indices.end(), edge_index) ==
        edge_indices.end()) {
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

std::vector<std::pair<size_t, std::vector<size_t>>>
cda_rail::Network::separate_stop_edges(const std::vector<size_t>& stop_edges) {
  std::vector<std::pair<size_t, std::vector<size_t>>> ret_val;
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

std::vector<std::vector<size_t>> cda_rail::Network::all_routes_of_given_length(
    std::optional<size_t> v_0, std::optional<size_t> e_0, double desired_length,
    bool reverse_direction, std::optional<size_t> exit_node,
    std::vector<size_t> edges_used_by_train) const {
  /**
   * Finds all routes from a specified starting point in the specified
   * direction. The routes are of a specified length, i.e., at least that long,
   * however removing the last edge results in a route that is too short.
   *
   * @param v_0: The index of the starting vertex. If specified, e_0 should be
   * empty.
   * @param e_0: The index of the starting edge. If specified, v_0 should be
   * empty.
   * @param desired_length: The desired length of the routes.
   * @param reverse_direction: If true, the routes are in the reverse direction.
   * Default is false, i.e., in edge order.
   */

  if (v_0.has_value() && e_0.has_value()) {
    throw exceptions::InvalidInputException("Both v_0 and e_0 are specified");
  }
  if (!v_0.has_value() && !e_0.has_value()) {
    throw exceptions::InvalidInputException(
        "Neither v_0 nor e_0 are specified");
  }

  if (v_0.has_value() && !has_vertex(v_0.value())) {
    throw exceptions::VertexNotExistentException(v_0.value());
  }
  if (e_0.has_value() && !has_edge(e_0.value())) {
    throw exceptions::EdgeNotExistentException(e_0.value());
  }

  if (desired_length <= 0) {
    throw exceptions::InvalidInputException(
        "Desired length is not strictly positive");
  }

  // NOLINTBEGIN(readability-avoid-nested-conditional-operator)
  const std::vector<size_t> edges_to_consider_tmp =
      v_0.has_value()
          ? (reverse_direction ? in_edges(v_0.value()) : out_edges(v_0.value()))
          : std::vector<size_t>{e_0.value()};
  // NOLINTEND(readability-avoid-nested-conditional-operator)

  auto edges_to_consider = edges_used_by_train.empty() ? edges_to_consider_tmp
                                                       : std::vector<size_t>();

  if (!edges_used_by_train.empty()) {
    for (const auto& e : edges_to_consider_tmp) {
      if (std::find(edges_used_by_train.begin(), edges_used_by_train.end(),
                    e) != edges_used_by_train.end()) {
        edges_to_consider.emplace_back(e);
      }
    }
  }

  std::vector<std::vector<size_t>> ret_val;

  for (const auto& e_index : edges_to_consider) {
    if (!reverse_direction && exit_node.has_value() &&
        get_edge(e_index).target == exit_node.value()) {
      ret_val.emplace_back(1, e_index);
      continue;
    }

    const auto& e_len = get_edge(e_index).length;

    if (e_len >= desired_length) {
      ret_val.emplace_back(1, e_index);
      continue;
    }

    const auto next_edges =
        reverse_direction ? get_predecessors(e_index) : get_successors(e_index);

    for (const auto& e_next_index : next_edges) {
      const auto paths_e_next = all_routes_of_given_length(
          std::nullopt, e_next_index, desired_length - e_len, reverse_direction,
          exit_node, edges_used_by_train);
      for (const auto& path_e_next : paths_e_next) {
        // check for cycle
        const auto edges_r = reverse_direction
                                 ? in_edges(get_edge(e_index).target)
                                 : out_edges(get_edge(e_index).source);
        if (std::any_of(
                edges_r.begin(), edges_r.end(), [&path_e_next](const auto& e) {
                  return std::find(path_e_next.begin(), path_e_next.end(), e) !=
                         path_e_next.end();
                })) {
          continue;
        }

        std::vector<size_t> path;
        path.emplace_back(e_index);
        path.insert(path.end(), path_e_next.begin(), path_e_next.end());
        ret_val.push_back(path);
      }
    }
  }

  return ret_val;
}

std::vector<size_t> cda_rail::Network::vertices_used_by_edges(
    const std::vector<size_t>& edges_tmp) const {
  std::unordered_set<size_t> used_vertices;
  for (const auto& edge : edges_tmp) {
    used_vertices.insert(get_edge(edge).source);
    used_vertices.insert(get_edge(edge).target);
  }
  return {used_vertices.begin(), used_vertices.end()};
}

double cda_rail::Network::maximal_vertex_speed(
    size_t v, const std::vector<size_t>& edges_to_consider) const {
  const auto& n_edges_tmp = neighboring_edges(v);
  auto        n_edges =
      edges_to_consider.empty() ? n_edges_tmp : std::vector<size_t>();
  for (const auto& e : n_edges_tmp) {
    if (std::find(edges_to_consider.begin(), edges_to_consider.end(), e) !=
        edges_to_consider.end()) {
      n_edges.emplace_back(e);
    }
  }

  if (n_edges.empty()) {
    return 0;
  }

  if (neighbors(v).size() == 1) {
    return get_edge(n_edges.front()).max_speed;
  }

  double                max_speed = 0;
  std::optional<size_t> max_speed_neighboring_vertex;
  double                second_max_speed = 0;
  for (const auto& e : n_edges) {
    const auto& edge = get_edge(e);
    if (edge.max_speed > max_speed) {
      second_max_speed             = max_speed;
      max_speed                    = edge.max_speed;
      max_speed_neighboring_vertex = other_vertex(e, v);
    } else if (edge.max_speed > second_max_speed &&
               max_speed_neighboring_vertex.has_value() &&
               // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
               other_vertex(e, v) != max_speed_neighboring_vertex.value()) {
      second_max_speed = edge.max_speed;
    }
  }
  return second_max_speed;
}

std::vector<size_t> cda_rail::Network::neighboring_edges(size_t index) const {
  auto       ret_val      = in_edges(index);
  const auto edges_to_add = out_edges(index);
  ret_val.insert(ret_val.end(), edges_to_add.begin(), edges_to_add.end());
  return ret_val;
}

double cda_rail::Network::minimal_neighboring_edge_length(
    size_t v, const std::vector<size_t>& edges_to_consider) const {
  const auto n_edges_tmp = neighboring_edges(v);
  auto       n_edges =
      edges_to_consider.empty() ? n_edges_tmp : std::vector<size_t>();
  for (const auto& e : n_edges_tmp) {
    if (std::find(edges_to_consider.begin(), edges_to_consider.end(), e) !=
        edges_to_consider.end()) {
      n_edges.emplace_back(e);
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
    const std::vector<size_t>&              edges_to_consider,
    const std::vector<std::vector<size_t>>& ttd) {
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
      if (std::find(ttd_section.begin(), ttd_section.end(),
                    edges_to_consider.at(edge_index)) != ttd_section.end()) {
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
  if (new_edge_to_old_edge_after_transform.find(new_edge) ==
      new_edge_to_old_edge_after_transform.end()) {
    return {new_edge, 0};
  }
  return new_edge_to_old_edge_after_transform.at(new_edge);
}

void cda_rail::Network::update_new_old_edge(size_t new_edge, size_t old_edge,
                                            double position) {
  /**
   * Updates the mapping accordingly
   */
  std::pair<size_t, double> old_edge_position = {old_edge, position};
  if (new_edge_to_old_edge_after_transform.find(old_edge) !=
      new_edge_to_old_edge_after_transform.end()) {
    const auto& old_edge_position_before =
        new_edge_to_old_edge_after_transform.at(old_edge);
    old_edge_position = {old_edge_position_before.first,
                         old_edge_position_before.second + position};
  }
  new_edge_to_old_edge_after_transform[new_edge] = old_edge_position;
}

void cda_rail::Network::change_vertex_headway(size_t index,
                                              double new_headway) {
  if (!has_vertex(index)) {
    throw exceptions::VertexNotExistentException(index);
  }
  vertices[index].headway = new_headway;
}

std::vector<size_t>
cda_rail::Network::get_unbreakable_section_containing_edge(size_t e) const {
  /**
   * This functions returns the unbreakable section that contains edge e as a
   * vector of edge indices.
   */

  const auto& edge_object = get_edge(e);
  if (edge_object.breakable) {
    return {};
  }

  std::vector<size_t> ret_val;
  ret_val.emplace_back(e);
  const auto reverse_e = get_reverse_edge_index(e);
  if (reverse_e.has_value()) {
    ret_val.push_back(reverse_e.value());
  }

  std::queue<size_t>  vertices_to_visit;
  std::vector<size_t> visited_vertices;
  vertices_to_visit.push(edge_object.source);
  vertices_to_visit.push(edge_object.target);

  while (!vertices_to_visit.empty()) {
    const auto& current_v = vertices_to_visit.front();

    if (std::find(visited_vertices.begin(), visited_vertices.end(),
                  current_v) != visited_vertices.end()) {
      // Vertex already visited
      vertices_to_visit.pop();
      continue;
    }

    if (get_vertex(current_v).type != VertexType::TTD &&
        get_vertex(current_v).type != VertexType::VSS) {
      // This vertex can be used to further extend the section
      const auto& n_vertices = neighbors(current_v);
      for (const auto& v_tmp : n_vertices) {
        if (std::find(visited_vertices.begin(), visited_vertices.end(),
                      v_tmp) == visited_vertices.end()) {
          vertices_to_visit.push(v_tmp);
        }

        // Possibly add new edges
        if (has_edge(current_v, v_tmp)) {
          const auto& e_tmp = get_edge_index(current_v, v_tmp);
          if (std::find(ret_val.begin(), ret_val.end(), e_tmp) ==
              ret_val.end()) {
            ret_val.push_back(e_tmp);
          }
        }
        if (has_edge(v_tmp, current_v)) {
          const auto& e_tmp = get_edge_index(v_tmp, current_v);
          if (std::find(ret_val.begin(), ret_val.end(), e_tmp) ==
              ret_val.end()) {
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
  return std::find(section_tmp.begin(), section_tmp.end(), e2) !=
         section_tmp.end();
}

std::pair<std::optional<double>, std::vector<size_t>>
cda_rail::Network::shortest_path_between_sets_using_edges(
    std::vector<size_t> source_edge_ids, std::vector<size_t> target_ids,
    bool only_use_valid_successors, std::vector<size_t> edges_to_use,
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
  std::vector<double> distances(number_of_edges(), INF);
  std::vector<bool>   visited(number_of_edges(), false);
  std::vector<size_t> predecessors(number_of_edges(),
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

    const bool target_found =
        std::find(target_ids.begin(), target_ids.end(),
                  target_is_edge ? edge_id : edge.target) != target_ids.end();

    if (target_found) {
      std::vector<size_t> path;
      path.emplace_back(edge_id);
      while (std::find(source_edge_ids.begin(), source_edge_ids.end(),
                       path.back()) == source_edge_ids.end() &&
             predecessors[path.back()] != std::numeric_limits<size_t>::max()) {
        const size_t predecessor = predecessors[path.back()];
        if (std::find(path.begin(), path.end(), predecessor) != path.end()) {
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
    std::vector<size_t> cleaned_successors;
    if (edges_to_use.empty()) {
      cleaned_successors = possible_successors;
    } else {
      for (const auto& successor : possible_successors) {
        if (std::find(edges_to_use.begin(), edges_to_use.end(), successor) !=
            edges_to_use.end()) {
          cleaned_successors.emplace_back(successor);
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
cda_rail::Network::length_of_path(const std::vector<size_t>& path) const {
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

double cda_rail::Network::delta_dist_helper(const Edge& successor_edge,
                                            double      max_v,
                                            bool        use_minimal_time) {
  double delta_dist = successor_edge.length;
  if (use_minimal_time) {
    // If minimal time is used, calculate time needed with maximal speed
    const double vel = std::min(max_v, successor_edge.max_speed);
    if (vel <= 0) {
      throw exceptions::InvalidInputException(
          "Maximum speed of every edge must be strictly positive if "
          "minimal time is used");
    }
    delta_dist /= vel;
  }
  return delta_dist;
}
