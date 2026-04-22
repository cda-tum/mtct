#include "datastructure/RailwayNetwork.hpp"
#include "nlohmann/json.hpp"
#include "nlohmann/json_fwd.hpp"

#include <functional>
#include <stack>
#include <variant>

using json = nlohmann::json;

// Import helper

void cda_rail::Network::read_graphml(const std::filesystem::path& p) {
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
  if (graphml_graph->Attribute("edgedefault") != std::string("directed")) {
    throw exceptions::InvalidInputException("Graph is not directed");
  }

  add_vertices_from_graphml(graphml_graph->FirstChildElement("node"), type,
                            headway);
  add_edges_from_graphml(graphml_graph->FirstChildElement("edge"), breakable,
                         length, max_speed, min_block_length,
                         min_stop_block_length);
}

void cda_rail::Network::get_keys_inplace(
    tinyxml2::XMLElement* graphml_body, std::optional<std::string>& breakable,
    std::optional<std::string>& length, std::optional<std::string>& max_speed,
    std::optional<std::string>& min_block_length,
    std::optional<std::string>& min_stop_block_length,
    std::optional<std::string>& type, std::optional<std::string>& headway) {
  const std::unordered_map<std::string, std::optional<std::string>*> key_map = {
      {"breakable", &breakable},
      {"min_block_length", &min_block_length},
      {"max_speed", &max_speed},
      {"length", &length},
      {"type", &type},
      {"min_stop_block_length", &min_stop_block_length},
      {"headway", &headway}};

  for (auto* key = graphml_body->FirstChildElement("key"); key != nullptr;
       key       = key->NextSiblingElement("key")) {
    if (const char* attr_name = key->Attribute("attr.name")) {
      if (auto it = key_map.find(attr_name); it != key_map.end()) {
        *it->second = key->Attribute("id");
      }
    }
  }
}

void cda_rail::Network::add_vertices_from_graphml(
    const tinyxml2::XMLElement*       graphml_node,
    const std::optional<std::string>& type,
    const std::optional<std::string>& headway) {
  for (auto const* node = graphml_node; node != nullptr;
       node             = node->NextSiblingElement("node")) {
    std::optional<int>    v_type;
    std::optional<double> headway_value;

    // Build a map from key-id → parser lambda for this node's data elements
    std::unordered_map<std::string, std::function<void(const char*)>> parsers;
    if (type.has_value()) {
      parsers.emplace(type.value(), [&v_type](const char* text) {
        v_type = std::stoi(text);
      });
    }
    if (headway.has_value()) {
      parsers.emplace(headway.value(), [&headway_value](const char* text) {
        headway_value = std::stod(text);
      });
    }

    for (auto* data = node->FirstChildElement("data"); data != nullptr;
         data       = data->NextSiblingElement("data")) {
      if (auto it = parsers.find(data->Attribute("key")); it != parsers.end()) {
        it->second(data->GetText());
      }
    }
    if (!v_type.has_value()) {
      throw exceptions::ImportException("graphml");
    }
    add_vertex({node->Attribute("id"), static_cast<VertexType>(v_type.value()),
                headway_value});
  }
}

void cda_rail::Network::extract_vertices_from_key_inplace(
    const std::string& key, std::string& source_name,
    std::string& target_name) {
  const size_t q1 = key.find_first_of('\'');
  const size_t q2 = key.find_first_of('\'', q1 + 1);
  source_name     = key.substr(q1 + 1, q2 - q1 - 1);

  const size_t q3 = key.find_first_of('\'', q2 + 1);
  const size_t q4 = key.find_first_of('\'', q3 + 1);
  target_name     = key.substr(q3 + 1, q4 - q3 - 1);
}

void cda_rail::Network::add_edges_from_graphml(
    const tinyxml2::XMLElement*       graphml_edge,
    const std::optional<std::string>& breakable,
    const std::optional<std::string>& length,
    const std::optional<std::string>& max_speed,
    const std::optional<std::string>& min_block_length,
    const std::optional<std::string>& min_stop_block_length) {
  for (auto const* cur = graphml_edge; cur != nullptr;
       cur             = cur->NextSiblingElement("edge")) {
    std::optional<double> e_length;
    std::optional<double> e_max_speed;
    std::optional<bool>   e_breakable;
    std::optional<double> e_min_block_length;
    std::optional<double> e_min_stop_block_length;

    // Map each key-id to a parser that fills the corresponding optional
    std::unordered_map<std::string, std::function<void(const char*)>> parsers;
    if (breakable.has_value()) {
      parsers.emplace(breakable.value(), [&e_breakable](const char* text) {
        std::string tmp = text;
        to_bool_optional_inplace(tmp, e_breakable);
      });
    }
    if (length.has_value()) {
      parsers.emplace(length.value(), [&e_length](const char* text) {
        e_length = std::stod(text);
      });
    }
    if (max_speed.has_value()) {
      parsers.emplace(max_speed.value(), [&e_max_speed](const char* text) {
        e_max_speed = std::stod(text);
      });
    }
    if (min_block_length.has_value()) {
      parsers.emplace(min_block_length.value(),
                      [&e_min_block_length](const char* text) {
                        e_min_block_length = std::stod(text);
                      });
    }
    if (min_stop_block_length.has_value()) {
      parsers.emplace(min_stop_block_length.value(),
                      [&e_min_stop_block_length](const char* text) {
                        e_min_stop_block_length = std::stod(text);
                      });
    }

    for (auto* data = cur->FirstChildElement("data"); data != nullptr;
         data       = data->NextSiblingElement("data")) {
      if (auto it = parsers.find(data->Attribute("key")); it != parsers.end()) {
        it->second(data->GetText());
      }
    }
    if (!e_length.has_value() || !e_max_speed.has_value()) {
      throw exceptions::ImportException("graphml");
    }
    add_edge({cur->Attribute("source")}, {cur->Attribute("target")},
             e_length.value(), e_max_speed.value(), e_breakable,
             e_min_block_length, e_min_stop_block_length);
  }
}

void cda_rail::Network::read_successors(const std::filesystem::path& p) {
  std::ifstream f(p / "successors_cpp.json");
  const json    data = json::parse(f);

  for (const auto& [key, val] : data.items()) {
    std::string source_name;
    std::string target_name;
    extract_vertices_from_key_inplace(key, source_name, target_name);
    const auto edge_in = get_edge_index({source_name}, {target_name});
    for (const auto& tuple : val) {
      add_successor(edge_in, get_edge_index({tuple[0].get<std::string>()},
                                            {tuple[1].get<std::string>()}));
    }
  }
}

// Export helper

void cda_rail::Network::export_graphml(const std::filesystem::path& p) const {
  std::ofstream file(p / "tracks.graphml");

  // Key IDs
  constexpr auto breakable_key             = "d0";
  constexpr auto length_key                = "d1";
  constexpr auto max_speed_key             = "d2";
  constexpr auto min_block_length_key      = "d3";
  constexpr auto min_stop_block_length_key = "d4";
  constexpr auto type_key                  = "d5";
  constexpr auto headway_key               = "d6";

  file
      << "<?xml version='1.0' encoding='UTF-8'?>\n"
      << R"(<graphml xmlns="http://graphml.graphdrawing.org/xmlns" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="http://graphml.graphdrawing.org/xmlns http://graphml.graphdrawing.org/xmlns/1.0/graphml.xsd">)"
      << '\n'
      << "<key id=\"" << breakable_key
      << R"(" for="edge" attr.name="breakable" attr.type="boolean"/>)" << '\n'
      << "<key id=\"" << length_key
      << R"(" for="edge" attr.name="length" attr.type="double"/>)" << '\n'
      << "<key id=\"" << max_speed_key
      << R"(" for="edge" attr.name="max_speed" attr.type="double"/>)" << '\n'
      << "<key id=\"" << min_block_length_key
      << R"(" for="edge" attr.name="min_block_length" attr.type="double"/>)"
      << '\n'
      << "<key id=\"" << min_stop_block_length_key
      << R"(" for="edge" attr.name="min_stop_block_length" attr.type="double"/>)"
      << '\n'
      << "<key id=\"" << type_key
      << R"(" for="vertex" attr.name="type" attr.type="long"/>)" << '\n'
      << "<key id=\"" << headway_key
      << R"(" for="vertex" attr.name="headway" attr.type="double"/>)" << '\n'
      << "<graph edgedefault=\"directed\">\n";

  for (const auto& vertex : m_vertices) {
    file << "<node id=\"" << vertex.name << "\">\n"
         << "<data key=\"" << type_key << "\">" << static_cast<int>(vertex.type)
         << "</data>\n"
         << "<data key=\"" << headway_key << "\">" << vertex.headway
         << "</data>\n"
         << "</node>\n";
  }

  for (const auto& edge : m_edges) {
    file << "<edge source=\"" << m_vertices[edge.source].name << "\" target=\""
         << m_vertices[edge.target].name << "\">\n"
         << "<data key=\"" << breakable_key << "\">" << std::boolalpha
         << edge.breakable << "</data>\n"
         << "<data key=\"" << length_key << "\">" << edge.length << "</data>\n"
         << "<data key=\"" << max_speed_key << "\">" << edge.max_speed
         << "</data>\n"
         << "<data key=\"" << min_block_length_key << "\">"
         << edge.min_block_length << "</data>\n"
         << "<data key=\"" << min_stop_block_length_key << "\">"
         << edge.min_stop_block_length << "</data>\n"
         << "</edge>\n";
  }

  file << "</graph>\n</graphml>\n";
}

void cda_rail::Network::export_successors_python(
    const std::filesystem::path& p) const {
  std::ofstream file(p / "successors.txt");
  file << "{";
  bool first = true;
  for (size_t i = 0; i < number_of_edges(); ++i) {
    if (!first) {
      file << ", ";
    }
    first            = false;
    const auto& edge = get_edge(i);
    file << "('" << m_vertices[edge.source].name << "', '"
         << m_vertices[edge.target].name << "'): ";
    write_successor_set_to_file(file, i);
  }
  file << "}\n";
}

void cda_rail::Network::export_successors_cpp(
    const std::filesystem::path& p) const {
  json j;
  for (size_t i = 0; i < number_of_edges(); ++i) {
    const auto&                                      edge = get_edge(i);
    std::vector<std::pair<std::string, std::string>> succ_list;
    for (const auto succ : m_successors[i]) {
      const auto& se = get_edge(succ);
      succ_list.emplace_back(m_vertices[se.source].name,
                             m_vertices[se.target].name);
    }
    j["('" + m_vertices[edge.source].name + "', '" +
      m_vertices[edge.target].name + "')"] = succ_list;
  }
  std::ofstream file(p / "successors_cpp.json");
  file << j << '\n';
}

void cda_rail::Network::write_successor_set_to_file(std::ofstream& file,
                                                    size_t         i) const {
  const auto& succs = get_successors(i);
  if (succs.empty()) {
    file << "set()";
    return;
  }
  file << "{";
  bool first = true;
  for (const auto succ : succs) {
    if (!first) {
      file << ", ";
    }
    first          = false;
    const auto& se = get_edge(succ);
    file << "('" << m_vertices[se.source].name << "', '"
         << m_vertices[se.target].name << "')";
  }
  file << "}";
}

// edge separation/changing helper

std::pair<cda_rail::index_vector, cda_rail::index_vector>
cda_rail::Network::separate_edge_private_helper(
    size_t edge_index, double min_length,
    const vss::SeparationFunction& sep_func, bool new_edge_breakable) {
  if (!get_edge(edge_index).breakable) {
    throw exceptions::ConsistencyException("Edge is not breakable");
  }

  const auto& edge = get_edge(edge_index);
  const auto  n_blocks =
      vss::functions::max_n_blocks(sep_func, min_length / edge.length);

  std::vector<double> distances;
  distances.reserve(n_blocks - 1);
  for (int i = 0; i < n_blocks - 1; ++i) {
    distances.emplace_back(edge.length * sep_func(i, n_blocks));
  }

  return separate_edge_at(edge_index, distances, new_edge_breakable);
}

std::pair<cda_rail::index_vector, cda_rail::index_vector>
cda_rail::Network::separate_edge_at(
    size_t edge_index, const std::vector<double>& distances_from_source,
    bool new_edge_breakable) {
  if (!has_edge(edge_index)) {
    throw exceptions::EdgeNotExistentException(edge_index);
  }
  if (distances_from_source.empty()) {
    throw exceptions::InvalidInputException("Distances are not specified");
  }
  if (!std::ranges::is_sorted(distances_from_source)) {
    throw exceptions::ConsistencyException("Distances are not sorted");
  }

  // Copy edge (referenced edge is modified below)
  const auto edge = get_edge(edge_index);

  if (distances_from_source.front() <= 0 ||
      distances_from_source.back() >= edge.length) {
    throw exceptions::ConsistencyException(
        "Distances are not strictly between 0 and the length of the edge");
  }

  // Create intermediate NoBorderVSS vertices
  cda_rail::index_vector new_vertices;
  new_vertices.reserve(distances_from_source.size());
  for (size_t i = 0; i < distances_from_source.size(); ++i) {
    new_vertices.emplace_back(
        add_vertex({concatenate_string_views({get_vertex(edge.source).name, "-",
                                              get_vertex(edge.target).name, "_",
                                              std::to_string(i)}),
                    VertexType::NoBorderVSS}));
  }

  std::pair<cda_rail::index_vector, cda_rail::index_vector> return_edges;
  auto& new_edges = return_edges.first;

  // First sub-edge: source → first new vertex
  new_edges.emplace_back(add_edge(edge.source, new_vertices.front(),
                                  distances_from_source.front(), edge.max_speed,
                                  new_edge_breakable, edge.min_block_length,
                                  edge.min_stop_block_length));
  update_new_old_edge(new_edges.back(), edge_index, 0);

  // Intermediate sub-edges
  for (size_t i = 1; i < distances_from_source.size(); ++i) {
    new_edges.emplace_back(add_edge(
        new_vertices[i - 1], new_vertices[i],
        distances_from_source[i] - distances_from_source[i - 1], edge.max_speed,
        new_edge_breakable, edge.min_block_length, edge.min_stop_block_length));
    update_new_old_edge(new_edges.back(), edge_index,
                        distances_from_source[i - 1]);
  }

  // Reuse original edge index for the last sub-edge
  change_edge_length(edge_index, edge.length - distances_from_source.back());
  update_new_old_edge(edge_index, edge_index, distances_from_source.back());
  if (!new_edge_breakable) {
    set_edge_unbreakable(edge_index);
  }
  m_edges[edge_index].source = new_vertices.back();
  new_edges.emplace_back(edge_index);

  // Update predecessor successors to point to the first new sub-edge
  for (const auto inc : in_edges(edge.source)) {
    if (m_successors[inc].contains(edge_index)) {
      m_successors[inc].erase(edge_index);
      m_successors[inc].insert(new_edges.front());
    }
  }
  // Chain new sub-edges as successors
  for (size_t i = 0; i + 1 < new_edges.size(); ++i) {
    add_successor(new_edges[i], new_edges[i + 1]);
  }

  // Handle reverse edge if it exists
  auto& new_reverse_edges = return_edges.second;
  if (has_edge(edge.target, edge.source)) {
    const auto rev_idx  = get_edge_index(edge.target, edge.source);
    const auto rev_edge = get_edge(rev_idx);
    if (rev_edge.length != edge.length) {
      throw exceptions::ConsistencyException(
          "Reverse edge has different length");
    }

    // First reverse sub-edge: target → last new vertex
    new_reverse_edges.emplace_back(
        add_edge(edge.target, new_vertices.back(),
                 edge.length - distances_from_source.back(), rev_edge.max_speed,
                 new_edge_breakable, rev_edge.min_block_length,
                 rev_edge.min_stop_block_length));
    update_new_old_edge(new_reverse_edges.back(), rev_idx, 0);

    // Intermediate reverse sub-edges (in reverse order)
    for (size_t i = distances_from_source.size() - 1; i > 0; --i) {
      new_reverse_edges.emplace_back(
          add_edge(new_vertices[i], new_vertices[i - 1],
                   distances_from_source[i] - distances_from_source[i - 1],
                   rev_edge.max_speed, new_edge_breakable,
                   rev_edge.min_block_length, rev_edge.min_stop_block_length));
      update_new_old_edge(new_reverse_edges.back(), rev_idx,
                          rev_edge.length - distances_from_source[i]);
    }

    // Reuse original reverse edge index for the last reverse sub-edge
    change_edge_length(rev_idx, distances_from_source.front());
    update_new_old_edge(rev_idx, rev_idx,
                        rev_edge.length - distances_from_source.front());
    if (!new_edge_breakable) {
      set_edge_unbreakable(rev_idx);
    }
    m_edges[rev_idx].source = new_vertices.front();
    new_reverse_edges.emplace_back(rev_idx);

    for (const auto inc : in_edges(edge.target)) {
      if (m_successors[inc].contains(rev_idx)) {
        m_successors[inc].erase(rev_idx);
        m_successors[inc].insert(new_reverse_edges.front());
      }
    }
    for (size_t i = 0; i + 1 < new_reverse_edges.size(); ++i) {
      add_successor(new_reverse_edges[i], new_reverse_edges[i + 1]);
    }
  }

  return return_edges;
}

void cda_rail::Network::update_new_old_edge(size_t new_edge, size_t old_edge,
                                            double position) {
  // If old_edge was itself derived from a prior transformation, chain the
  // mapping
  auto accumulated_position = position;
  if (auto it = m_new_edge_to_old_edge_after_transform.find(old_edge);
      it != m_new_edge_to_old_edge_after_transform.end()) {
    old_edge             = it->second.first;
    accumulated_position = it->second.second + position;
  }
  m_new_edge_to_old_edge_after_transform[new_edge] = {old_edge,
                                                      accumulated_position};
}

std::vector<std::pair<std::optional<size_t>, std::optional<size_t>>>
cda_rail::Network::sort_edge_pairs(
    std::vector<std::pair<std::optional<size_t>, std::optional<size_t>>>&
        edge_pairs) const {
  if (!std::ranges::all_of(edge_pairs,
                           [](const auto& p) { return p.first.has_value(); })) {
    throw exceptions::InvalidInputException("Edge pair first entry is empty");
  }
  if (!std::ranges::all_of(edge_pairs, [this](const auto& p) {
        return has_edge(p.first.value());
      })) {
    throw exceptions::EdgeNotExistentException();
  }
  if (!std::ranges::all_of(edge_pairs, [this](const auto& p) {
        return get_reverse_edge_index(p.first.value()) == p.second;
      })) {
    throw exceptions::ConsistencyException(
        "Pairs are not reverse of each other");
  }

  // Build vertex → edge-pair-index adjacency
  std::unordered_map<size_t, std::unordered_set<size_t>> vertex_neighbors;
  for (size_t i = 0; i < edge_pairs.size(); ++i) {
    const auto& e = get_edge(edge_pairs[i].first.value()); // NOLINT
    vertex_neighbors[e.source].emplace(i);
    vertex_neighbors[e.target].emplace(i);
  }

  // Find a degree-1 vertex to start from (end of path)
  auto start_it =
      std::ranges::find_if(std::views::iota(size_t{0}, number_of_vertices()),
                           [&vertex_neighbors](size_t v) {
                             return vertex_neighbors[v].size() == 1;
                           });
  size_t j =
      (start_it != std::views::iota(size_t{0}, number_of_vertices()).end())
          ? *start_it
          : number_of_vertices();

  std::vector<std::pair<std::optional<size_t>, std::optional<size_t>>> result;
  while (j < number_of_vertices()) {
    if (vertex_neighbors[j].empty()) {
      break;
    }
    if (vertex_neighbors[j].size() > 1) {
      throw exceptions::ConsistencyException(
          "Something went wrong, vertex has more than one neighbor still.");
    }

    const auto  pair_idx = *vertex_neighbors[j].begin();
    const auto& ep       = edge_pairs[pair_idx];
    const auto& e        = get_edge(ep.first.value()); // NOLINT

    vertex_neighbors[e.source].erase(pair_idx);
    vertex_neighbors[e.target].erase(pair_idx);

    if (e.target == j) {
      result.emplace_back(ep.second, ep.first);
      j = e.source;
    } else if (e.source == j) {
      result.emplace_back(ep.first, ep.second);
      j = e.target;
    } else {
      throw exceptions::ConsistencyException(
          "Something went wrong, source and target are not as expected.");
    }
  }

  if (std::ranges::any_of(std::views::iota(size_t{0}, number_of_vertices()),
                          [&vertex_neighbors](size_t i) {
                            return !vertex_neighbors[i].empty();
                          })) {
    throw exceptions::ConsistencyException(
        "Something went wrong, not everything was processed.");
  }

  return result;
}

// validity helper

void cda_rail::Network::check_new_edge_requirements(size_t source,
                                                    size_t target) const {
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

double cda_rail::Network::delta_dist_helper(const Edge& edge, double max_v,
                                            bool use_minimal_time) {
  if (!use_minimal_time) {
    return edge.length;
  }
  const double vel = std::min(max_v, edge.max_speed);
  if (vel <= 0) {
    throw exceptions::InvalidInputException(
        "Maximum speed of every edge must be strictly positive if "
        "minimal time is used");
  }
  return edge.length / vel;
}

void cda_rail::Network::dfs_inplace(
    std::vector<cda_rail::index_set>&     ret_val,
    std::unordered_set<size_t>&           vertices_to_visit,
    const VertexType&                     section_type,
    const std::unordered_set<VertexType>& error_types) const {
  while (!vertices_to_visit.empty()) {
    ret_val.emplace_back();
    auto& section = ret_val.back();

    std::stack<size_t>         stack;
    std::unordered_set<size_t> visited;
    stack.emplace(*vertices_to_visit.begin());

    while (!stack.empty()) {
      const size_t v = stack.top();
      stack.pop();

      if (!visited.insert(v).second) {
        continue; // already visited
      }
      vertices_to_visit.erase(v);

      for (const auto nb : neighbors(v)) {
        if (error_types.contains(get_vertex(nb).type)) {
          throw exceptions::ConsistencyException(
              "This should never happen, but I found error type vertex");
        }
        if (get_vertex(nb).type == section_type && !visited.contains(nb)) {
          stack.emplace(nb);
        }
        // Collect incident edges in both directions
        for (const auto [src, tgt] : {std::pair{v, nb}, std::pair{nb, v}}) {
          if (has_edge(src, tgt)) {
            const auto idx = get_edge_index(src, tgt);
            if (get_edge(idx).breakable) {
              throw exceptions::ConsistencyException(
                  "This should never happen, but I found a breakable edge in "
                  "an unbreakable section");
            }
            section.insert(idx);
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
  if (v_0.has_value() == e_0.has_value()) {
    throw exceptions::InvalidInputException(
        v_0.has_value() ? "Both v_0 and e_0 are specified"
                        : "Neither v_0 nor e_0 are specified");
  }
  if (v_0.has_value() && !has_vertex(*v_0)) {
    throw exceptions::VertexNotExistentException(*v_0);
  }
  if (e_0.has_value() && !has_edge(*e_0)) {
    throw exceptions::EdgeNotExistentException(*e_0);
  }

  // Special case: zero length from a vertex
  if (return_successors_if_zero && desired_length == 0 && v_0.has_value()) {
    const auto adj = reverse_direction ? in_edges(*v_0) : out_edges(*v_0);
    std::vector<cda_rail::index_vector> result;
    result.reserve(adj.size());
    for (const auto s : adj) {
      result.emplace_back(1, s);
    }
    return result;
  }

  if (desired_length <= 0) {
    throw exceptions::InvalidInputException(
        "Desired length is not strictly positive");
  }

  // Collect starting edges (filtered by edges_used_by_train if specified)
  cda_rail::index_set start_edges_raw =
      v_0.has_value() ? (reverse_direction ? in_edges(*v_0) : out_edges(*v_0))
                      : cda_rail::index_set{*e_0};

  cda_rail::index_set start_edges;
  if (edges_used_by_train.empty()) {
    start_edges = std::move(start_edges_raw);
  } else {
    for (const auto e : start_edges_raw) {
      if (edges_used_by_train.contains(e)) {
        start_edges.insert(e);
      }
    }
  }

  std::vector<cda_rail::index_vector> result;

  for (const auto e_idx : start_edges) {
    // Early exit at designated node
    if (!reverse_direction && exit_node.has_value() &&
        get_edge(e_idx).target == *exit_node) {
      result.emplace_back(1, e_idx);
      continue;
    }

    const double e_len = get_edge(e_idx).length;

    if (e_len >= desired_length) {
      result.emplace_back(1, e_idx);
      continue;
    }

    const auto next_edges =
        reverse_direction ? get_predecessors(e_idx) : get_successors(e_idx);

    for (const auto e_next : next_edges) {
      const auto sub_paths = all_routes_of_given_length(
          std::nullopt, e_next, desired_length - e_len, reverse_direction,
          exit_node, edges_used_by_train);

      for (const auto& sub_path : sub_paths) {
        // Cycle check
        const auto boundary_edges = reverse_direction
                                        ? in_edges(get_edge(e_idx).target)
                                        : out_edges(get_edge(e_idx).source);
        if (std::ranges::any_of(boundary_edges, [&sub_path](const auto e) {
              return std::ranges::contains(sub_path, e);
            })) {
          continue;
        }
        cda_rail::index_vector path{e_idx};
        path.insert(path.end(), sub_path.begin(), sub_path.end());
        result.push_back(std::move(path));
      }
    }
  }

  return result;
}

std::vector<cda_rail::index_vector>
cda_rail::Network::all_paths_ending_at_ttd_recursive_helper(
    size_t e_0, const std::vector<cda_rail::index_set>& ttd_sections,
    std::optional<size_t> exit_node, std::optional<size_t> safe_ttd,
    bool first_edge) const {
  if (!has_edge(e_0)) {
    throw exceptions::EdgeNotExistentException(e_0);
  }

  // Check TTD membership: stop if we enter a new TTD section
  for (size_t ttd_idx = 0; ttd_idx < ttd_sections.size(); ++ttd_idx) {
    if (safe_ttd.has_value() && ttd_idx == *safe_ttd) {
      continue;
    }
    if (ttd_sections[ttd_idx].contains(e_0)) {
      if (!first_edge) {
        return {{}};
      }
      safe_ttd = ttd_idx;
    }
  }

  const auto& e0_edge = get_edge(e_0);
  if (exit_node.has_value() && e0_edge.target == *exit_node) {
    return {{e_0}};
  }

  std::vector<cda_rail::index_vector> result;
  for (const auto succ : get_successors(e_0)) {
    for (const auto& sub_path : all_paths_ending_at_ttd_recursive_helper(
             succ, ttd_sections, exit_node, safe_ttd, false)) {
      cda_rail::index_vector path{e_0};
      path.insert(path.end(), sub_path.begin(), sub_path.end());
      result.push_back(std::move(path));
    }
  }
  return result;
}

// ----------------
// HELPER STRUCTS
// ----------------

size_t cda_rail::Network::VertexInput::resolve(
    cda_rail::Network const* const network) const {
  if (const auto* idx = std::get_if<size_t>(&m_data)) {
    return *idx;
  }
  if (const auto* name = std::get_if<std::string_view>(&m_data)) {
    return network->get_vertex_index(*name);
  }
  if (const auto* vertex = std::get_if<Vertex>(&m_data)) {
    const auto idx = network->get_vertex_index(vertex->name);
    if (network->get_vertex(idx) != *vertex) {
      throw exceptions::ConsistencyException(
          "Vertex objects with matching names have different properties.");
    }
    return idx;
  }
  throw std::runtime_error("Invalid VertexInput variant");
}

size_t
cda_rail::Network::EdgeInput::resolve(Network const* const network) const {
  if (const auto* idx = std::get_if<size_t>(&m_data)) {
    return *idx;
  }
  if (const auto* pair = std::get_if<std::pair<size_t, size_t>>(&m_data)) {
    return network->get_edge_index(pair->first, pair->second);
  }
  if (const auto* str_pair =
          std::get_if<std::pair<std::string_view, std::string_view>>(&m_data)) {
    return network->get_edge_index(str_pair->first, str_pair->second);
  }
  if (const auto* edge = std::get_if<Edge>(&m_data)) {
    const auto idx = network->get_edge_index(edge->source, edge->target);
    if (network->get_edge(idx) != *edge) {
      throw exceptions::ConsistencyException(
          "Edge objects with matching source and target have different "
          "properties.");
    }
    return idx;
  }
  throw std::runtime_error("Invalid EdgeInput variant");
}
