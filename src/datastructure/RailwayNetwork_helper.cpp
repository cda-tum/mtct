#include "datastructure/RailwayNetwork.hpp"
#include "nlohmann/json.hpp"
#include "nlohmann/json_fwd.hpp"

#include <stack>
#include <variant>

using json = nlohmann::json;

// Import helper

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

  std::optional<std::string> breakable;
  std::optional<std::string> length;
  std::optional<std::string> max_speed;
  std::optional<std::string> min_block_length;
  std::optional<std::string> min_stop_block_length;
  std::optional<std::string> type;
  std::optional<std::string> headway;
  Network::get_keys_inplace(graphml_body, breakable, length, max_speed,
                            min_block_length, min_stop_block_length, type,
                            headway);
  if (!breakable.has_value() || !length.has_value() || !max_speed.has_value() ||
      !min_block_length.has_value() || !type.has_value()) {
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

void cda_rail::Network::get_keys_inplace(
    tinyxml2::XMLElement* graphml_body, std::optional<std::string>& breakable,
    std::optional<std::string>& length, std::optional<std::string>& max_speed,
    std::optional<std::string>& min_block_length,
    std::optional<std::string>& min_stop_block_length,
    std::optional<std::string>& type, std::optional<std::string>& headway) {
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
    const tinyxml2::XMLElement*       graphml_node,
    const std::optional<std::string>& type,
    const std::optional<std::string>& headway) {
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
    std::optional<std::pair<double, double>> pos;
    while (graphml_data != nullptr) {
      if (type.has_value() && graphml_data->Attribute("key") == type.value()) {
        v_type = std::stoi(graphml_data->GetText());
      } else if (headway.has_value() &&
                 graphml_data->Attribute("key") == headway.value()) {
        headway_value = std::stod(graphml_data->GetText());
      }
      graphml_data = graphml_data->NextSiblingElement("data");
    }
    if (!v_type.has_value()) {
      throw exceptions::ImportException("graphml");
    }
    this->add_vertex(
        {name, static_cast<VertexType>(v_type.value()), headway_value});
    graphml_node = graphml_node->NextSiblingElement("node");
  }
}

void cda_rail::Network::extract_vertices_from_key_inplace(
    const std::string& key, std::string& source_name,
    std::string& target_name) {
  size_t const first_quote  = key.find_first_of('\'');
  size_t const second_quote = key.find_first_of('\'', first_quote + 1);
  source_name = key.substr(first_quote + 1, second_quote - first_quote - 1);

  size_t const third_quote  = key.find_first_of('\'', second_quote + 1);
  size_t const fourth_quote = key.find_first_of('\'', third_quote + 1);
  target_name = key.substr(third_quote + 1, fourth_quote - third_quote - 1);
}

void cda_rail::Network::add_edges_from_graphml(
    const tinyxml2::XMLElement*       graphml_edge,
    const std::optional<std::string>& breakable,
    const std::optional<std::string>& length,
    const std::optional<std::string>& max_speed,
    const std::optional<std::string>& min_block_length,
    const std::optional<std::string>& min_stop_block_length) {
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
      if (breakable.has_value() &&
          graphml_data->Attribute("key") == breakable.value()) {
        std::string tmp = graphml_data->GetText();
        to_bool_optional_inplace(tmp, e_breakable);
      } else if (min_block_length.has_value() &&
                 graphml_data->Attribute("key") == min_block_length.value()) {
        e_min_block_length = std::stod(graphml_data->GetText());
      } else if (max_speed.has_value() &&
                 graphml_data->Attribute("key") == max_speed.value()) {
        e_max_speed = std::stod(graphml_data->GetText());
      } else if (length.has_value() &&
                 graphml_data->Attribute("key") == length.value()) {
        e_length = std::stod(graphml_data->GetText());
      } else if (min_stop_block_length.has_value() &&
                 graphml_data->Attribute("key") ==
                     min_stop_block_length.value()) {
        e_min_stop_block_length = std::stod(graphml_data->GetText());
      }
      graphml_data = graphml_data->NextSiblingElement("data");
    }
    if (!e_length.has_value() || !e_max_speed.has_value()) {
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
    extract_vertices_from_key_inplace(key, source_name, target_name);
    auto const edge_id_in = get_edge_index(source_name, target_name);
    for (auto& tuple : val) {
      auto const edge_id_out = get_edge_index(tuple[0].get<std::string>(),
                                              tuple[1].get<std::string>());
      add_successor(edge_id_in, edge_id_out);
    }
  }
}

// Export helper

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
  for (const auto& vertex : m_vertices) {
    file << "<node id=\"" << vertex.name << "\">" << '\n';
    file << "<data key=\"" << type << "\">" << static_cast<int>(vertex.type)
         << "</data>" << '\n';
    file << "<data key=\"" << headway << "\">" << vertex.headway << "</data>"
         << '\n';
    file << "</node>" << '\n';
  }

  // Write the edges
  for (const auto& edge : m_edges) {
    file << "<edge source=\"" << m_vertices.at(edge.source).name
         << "\" target=\"" << m_vertices.at(edge.target).name << "\">" << '\n';
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

    file << "('" << m_vertices[edge.source].name << "', '"
         << m_vertices[edge.target].name << "'): ";
    write_successor_set_to_file(file, i);
  }
  file << "}" << '\n';
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
    for (const auto& successor : m_successors.at(i)) {
      const auto& successor_edge = get_edge(successor);
      successor_edges_export.emplace_back(
          m_vertices.at(successor_edge.source).name,
          m_vertices.at(successor_edge.target).name);
    }
    j["('" + m_vertices.at(edge.source).name + "', '" +
      m_vertices.at(edge.target).name + "')"] = successor_edges_export;
  }

  std::ofstream file(p / "successors_cpp.json");
  file << j << '\n';
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
      file << "('" << m_vertices[successor_edge.source].name << "', '"
           << m_vertices[successor_edge.target].name << "')";
    }
    file << "}";
  }
}

// edge separation/changing helper

std::pair<cda_rail::index_vector, cda_rail::index_vector>
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

std::pair<cda_rail::index_vector, cda_rail::index_vector>
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

  cda_rail::index_vector new_vertices;
  for (size_t i = 0; i < distances_from_source.size(); ++i) {
    std::string const vertex_name = get_vertex(edge.source).name + "_" +
                                    get_vertex(edge.target).name + "_" +
                                    std::to_string(i);
    new_vertices.emplace_back(
        add_vertex({vertex_name, VertexType::NoBorderVSS}));
  }

  std::pair<cda_rail::index_vector, cda_rail::index_vector> return_edges;
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
  m_edges[edge_index].source = new_vertices.back();
  new_edges.emplace_back(edge_index);

  // Update successors, i.e.,
  // - For every incoming edge into edge.source, replace edge_index by
  // new_edges.front() if applicable
  // - For every new_edge (except the last) add the next new_edge as single
  // successor
  // - For the last new edge add the same successors as edge_index had (this has
  // already been done implicitly)
  for (const auto& incoming_edge_index : in_edges(edge.source)) {
    m_successors[incoming_edge_index].erase(edge_index);
    m_successors[incoming_edge_index].insert(new_edges.front());
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
    m_edges[reverse_edge_index].source = new_vertices.front();
    new_reverse_edges.emplace_back(reverse_edge_index);

    for (const auto& incoming_edge_index : in_edges(edge.target)) {
      m_successors[incoming_edge_index].erase(reverse_edge_index);
      m_successors[incoming_edge_index].insert(new_reverse_edges.front());
    }
    for (size_t i = 0; i < new_reverse_edges.size() - 1; ++i) {
      add_successor(new_reverse_edges[i], new_reverse_edges[i + 1]);
    }
  }

  return return_edges;
}

void cda_rail::Network::update_new_old_edge(size_t new_edge, size_t old_edge,
                                            double position) {
  /**
   * Updates the mapping accordingly
   */
  std::pair<size_t, double> old_edge_position = {old_edge, position};
  if (m_new_edge_to_old_edge_after_transform.contains(old_edge)) {
    const auto& old_edge_position_before =
        m_new_edge_to_old_edge_after_transform.at(old_edge);
    old_edge_position = {old_edge_position_before.first,
                         old_edge_position_before.second + position};
  }
  m_new_edge_to_old_edge_after_transform[new_edge] = old_edge_position;
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

  if (!std::ranges::all_of(edge_pairs, [](const auto& edge_pair) {
        return edge_pair.first.has_value();
      })) {
    throw exceptions::InvalidInputException("Edge pair first entry is empty");
  }
  if (!std::ranges::all_of(edge_pairs, [this](const auto& edge_pair) {
        return has_edge(edge_pair.first.value());
      })) {
    throw exceptions::EdgeNotExistentException();
  }

  if (!std::ranges::all_of(edge_pairs, [this](const auto& edge_pair) {
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

// validity helper

void cda_rail::Network::check_new_edge_requirements(size_t const source,
                                                    size_t const target) const {
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
}

// path finding algorithm helper

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

void cda_rail::Network::dfs_inplace(
    std::vector<cda_rail::index_vector>& ret_val,
    std::unordered_set<size_t>&          vertices_to_visit,
    const VertexType&                    section_type,
    const std::vector<VertexType>&       error_types) const {
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
            !std::ranges::contains(visited_vertices, neighbor)) {
          stack.emplace(neighbor);
        }
        if (std::ranges::contains(error_types, get_vertex(neighbor).type)) {
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

          if (!std::ranges::contains(ret_val.back(), edge_index)) {
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

          if (!std::ranges::contains(ret_val.back(), edge_index)) {
            ret_val.back().emplace_back(edge_index);
          }
        }
      }
    }
  }
}

std::vector<cda_rail::index_vector>
cda_rail::Network::all_routes_of_given_length(
    std::optional<size_t> v_0, std::optional<size_t> e_0, double desired_length,
    bool reverse_direction, std::optional<size_t> exit_node,
    cda_rail::index_set edges_used_by_train,
    bool                return_successors_if_zero) const {
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

  if (return_successors_if_zero && desired_length == 0 && v_0.has_value()) {
    const auto neighboring_edges =
        reverse_direction ? in_edges(v_0.value()) : out_edges(v_0.value());
    std::vector<cda_rail::index_vector> ret_val;
    ret_val.reserve(neighboring_edges.size());
    for (const auto& s : neighboring_edges) {
      ret_val.emplace_back(1, s);
    }
    return ret_val;
  }

  if (desired_length <= 0) {
    throw exceptions::InvalidInputException(
        "Desired length is not strictly positive");
  }

  // NOLINTBEGIN(readability-avoid-nested-conditional-operator)
  const cda_rail::index_set edges_to_consider_tmp =
      v_0.has_value()
          ? (reverse_direction ? in_edges(v_0.value()) : out_edges(v_0.value()))
          : cda_rail::index_set{e_0.value()};
  // NOLINTEND(readability-avoid-nested-conditional-operator)

  auto edges_to_consider = edges_used_by_train.empty() ? edges_to_consider_tmp
                                                       : cda_rail::index_set();

  if (!edges_used_by_train.empty()) {
    for (const auto& e : edges_to_consider_tmp) {
      if (edges_used_by_train.contains(e)) {
        edges_to_consider.insert(e);
      }
    }
  }

  std::vector<cda_rail::index_vector> ret_val;

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
        if (std::ranges::any_of(edges_r, [&path_e_next](const auto& e) {
              return std::ranges::contains(path_e_next, e);
            })) {
          continue;
        }

        cda_rail::index_vector path;
        path.emplace_back(e_index);
        path.insert(path.end(), path_e_next.begin(), path_e_next.end());
        ret_val.push_back(path);
      }
    }
  }

  return ret_val;
}

std::vector<cda_rail::index_vector> cda_rail::Network::all_paths_ending_at_ttd(
    size_t e_0, const std::vector<cda_rail::index_vector>& ttd_sections,
    std::optional<size_t> exit_node, std::optional<size_t> safe_ttd,
    bool first_edge) const {
  /**
   * Finds all paths starting at edge e_0 and ending at a TTD section, which is
   * not safe_ttd.
   */
  std::vector<cda_rail::index_vector> ret_val;

  if (!has_edge(e_0)) {
    throw exceptions::EdgeNotExistentException(e_0);
  }
  for (size_t ttd_idx = 0; ttd_idx < ttd_sections.size(); ++ttd_idx) {
    if (safe_ttd.has_value() && ttd_idx == safe_ttd.value()) {
      continue;
    }
    const auto& ttd_section = ttd_sections.at(ttd_idx);
    if (std::ranges::contains(ttd_section, e_0)) {
      if (!first_edge) {
        // Edge is in TTD section, but not the first edge
        return {{}};
      }
      safe_ttd = ttd_idx;
    }
  }

  const auto& e_0_edge = get_edge(e_0);
  if (exit_node.has_value() && e_0_edge.target == exit_node.value()) {
    // Edge is already at exit node
    return {{e_0}};
  }

  const auto possible_successors = get_successors(e_0);
  for (const auto& successor : possible_successors) {
    const auto successor_paths = all_paths_ending_at_ttd(
        successor, ttd_sections, exit_node, safe_ttd, false);
    for (const auto& successor_path : successor_paths) {
      ret_val.emplace_back();
      ret_val.back().emplace_back(e_0);
      ret_val.back().insert(ret_val.back().end(), successor_path.begin(),
                            successor_path.end());
    }
  }

  return ret_val;
}

// ----------------
// HELPER STRUCTS
// ----------------

size_t cda_rail::Network::VertexInput::resolve(
    cda_rail::Network const* const network) const {
  if (const auto* idx = std::get_if<size_t>(&m_data)) {
    return *idx;
  }

  if (const auto* int_pair = std::get_if<std::string_view>(&m_data)) {
    return network->get_vertex_index(*int_pair);
  }

  // Should never be reached
  throw std::runtime_error("Invalid VertexInput variant");
};

size_t
cda_rail::Network::EdgeInput::resolve(Network const* const network) const {
  if (const auto* idx = std::get_if<size_t>(&m_data)) {
    return *idx;
  }

  if (const auto* int_pair = std::get_if<std::pair<size_t, size_t>>(&m_data)) {
    return network->get_edge_index(int_pair->first, int_pair->second);
  }

  if (const auto* str_pair =
          std::get_if<std::pair<std::string_view, std::string_view>>(&m_data)) {
    return network->get_edge_index(str_pair->first, str_pair->second);
  }

  // Should never be reached
  throw std::runtime_error("Invalid EdgeInput variant");
};
