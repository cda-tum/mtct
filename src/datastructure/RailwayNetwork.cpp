#include "datastructure/RailwayNetwork.hpp"
#include "Definitions.hpp"
#include <string>
#include <tinyxml2.h>
#include "nlohmann/json.hpp"
#include <filesystem>
#include <fstream>
#include <optional>
#include <algorithm>

using json = nlohmann::json;

void cda_rail::Network::extract_vertices_from_key(const std::string &key, std::string &source_name,
                                                  std::string &target_name) {
    /**
     * Extract source and target names from key.
     * @param key Key
     * @param source_name Source name
     * @param target_name Target name
     *
     * The variables are passed by reference and are modified in place.
     */

    // Extract source, which is between the first and second single quote
    size_t first_quote = key.find_first_of("'");
    size_t second_quote = key.find_first_of("'", first_quote + 1);
    source_name = key.substr(first_quote + 1, second_quote - first_quote - 1);

    // Extract target, which is between the third and fourth single quote
    size_t third_quote = key.find_first_of("'", second_quote + 1);
    size_t fourth_quote = key.find_first_of("'", third_quote + 1);
    target_name = key.substr(third_quote + 1, fourth_quote - third_quote - 1);
}

void cda_rail::Network::get_keys(tinyxml2::XMLElement *graphml_body, std::string &breakable, std::string &length,
                                 std::string &max_speed, std::string &min_block_length, std::string &type) {
    /**
     * Get keys from graphml file
     * @param graphml_body Body of graphml file
     * @param breakable Breakable key
     * @param length Length key
     * @param max_speed Max speed key
     * @param min_block_length Min block length key
     * @param type Type key
     *
     * The variables are passed by reference and are modified in place.
     */

    tinyxml2::XMLElement* graphml_key = graphml_body->FirstChildElement("key");
    while (graphml_key) {
        if (graphml_key->Attribute("attr.name") == std::string("breakable")) {
            breakable = graphml_key->Attribute("id");
        }
        else if (graphml_key->Attribute("attr.name") == std::string("min_block_length")) {
            min_block_length = graphml_key->Attribute("id");
        }
        else if (graphml_key->Attribute("attr.name") == std::string("max_speed")) {
            max_speed = graphml_key->Attribute("id");
        }
        else if (graphml_key->Attribute("attr.name") == std::string("length")) {
            length = graphml_key->Attribute("id");
        }
        else if (graphml_key->Attribute("attr.name") == std::string("type")) {
            type = graphml_key->Attribute("id");
        }
        graphml_key = graphml_key->NextSiblingElement("key");
    }
}

void cda_rail::Network::add_vertices_from_graphml(const tinyxml2::XMLElement *graphml_node,
                                                  const std::string &type) {
    /**
     * Add vertices from graphml file
     * @param graphml_node Node of graphml file
     * @param network Network object
     * @param type Type key
     *
     * The vertices are added to the network object in place.
     */

    while (graphml_node) {
        const tinyxml2::XMLElement* graphml_data = graphml_node->FirstChildElement("data");
        std::string name = graphml_node->Attribute("id");
        std::optional<int> v_type;
        while (graphml_data) {
            if (graphml_data->Attribute("key") == type) {
                v_type = std::stoi(graphml_data->GetText());
            }
            graphml_data = graphml_data->NextSiblingElement("data");
        }
        if (!v_type.has_value()) {
            throw std::runtime_error("Error reading graphml file");
        }
        this->add_vertex(name, static_cast<VertexType>(v_type.value()));
        graphml_node = graphml_node->NextSiblingElement("node");
    }
}

void cda_rail::Network::add_edges_from_graphml(const tinyxml2::XMLElement *graphml_edge,
                                               const std::string &breakable, const std::string &length,
                                               const std::string &max_speed, const std::string &min_block_length) {
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

    while (graphml_edge) {
        const tinyxml2::XMLElement* graphml_data = graphml_edge->FirstChildElement("data");
        std::string source_name = graphml_edge->Attribute("source");
        std::string target_name = graphml_edge->Attribute("target");
        std::optional<double> e_length;
        std::optional<double> e_max_speed;
        std::optional<bool> e_breakable;
        std::optional<double> e_min_block_length;
        while (graphml_data) {
            if (graphml_data->Attribute("key") == breakable) {
                std::string tmp = graphml_data->GetText();
                cda_rail::to_bool_optional(tmp, e_breakable);
            }
            else if (graphml_data->Attribute("key") == min_block_length) {
                e_min_block_length = std::stod(graphml_data->GetText());
            }
            else if (graphml_data->Attribute("key") == max_speed) {
                e_max_speed = std::stod(graphml_data->GetText());
            }
            else if (graphml_data->Attribute("key") == length) {
                e_length = std::stod(graphml_data->GetText());
            }
            graphml_data = graphml_data->NextSiblingElement("data");
        }
        if (!e_length.has_value() || !e_max_speed.has_value() || !e_breakable.has_value() || !e_min_block_length.has_value()) {
            throw std::runtime_error("Error reading graphml file");
        }
        this->add_edge(source_name, target_name, e_length.value(), e_max_speed.value(), e_breakable.value(), e_min_block_length.value());
        graphml_edge = graphml_edge->NextSiblingElement("edge");
    }
}

void cda_rail::Network::read_graphml(const std::filesystem::path &p) {
    /**
     * Read network graph from XML file into the object
     * @param path Path to XML file
     */

    // Open graphml file
    tinyxml2::XMLDocument graph_xml;
    graph_xml.LoadFile((p / "tracks.graphml").string().c_str());
    if (graph_xml.Error()) {
        throw std::runtime_error("Error reading graphml file");
    }

    // Get graphml body
    tinyxml2::XMLElement* graphml_body = graph_xml.FirstChildElement("graphml");

    // Get keys
    std::string breakable = "";
    std::string length = "";
    std::string max_speed = "";
    std::string min_block_length = "";
    std::string type = "";
    cda_rail::Network::get_keys(graphml_body, breakable, length, max_speed, min_block_length, type);
    if ((breakable == "") || (length == "") || (max_speed == "") || (min_block_length == "") || (type == "")) {
        throw std::runtime_error("Error reading graphml file");
    }

    // Get graph
    const tinyxml2::XMLElement* graphml_graph = graphml_body->FirstChildElement("graph");
    if ((graphml_graph->Attribute("edgedefault")) != std::string("directed")) {
        throw std::runtime_error("Graph is not directed. Not all properties present.");
    }

    // Get vertices
    const tinyxml2::XMLElement* graphml_node = graphml_graph->FirstChildElement("node");
    this->add_vertices_from_graphml(graphml_node, type);

    // Get edges
    const tinyxml2::XMLElement* graphml_edge = graphml_graph->FirstChildElement("edge");
    this->add_edges_from_graphml(graphml_edge, breakable, length, max_speed, min_block_length);
}

void cda_rail::Network::read_successors(const std::filesystem::path &p) {
    /**
     * Read successors from path
     * @param path Path to successors file
     */

    // Open file
    std::ifstream f((p / "successors_cpp.json"));
    json data = json::parse(f);

    // Parse json
    for (auto& [key, val] : data.items()) {
        std::string source_name;
        std::string target_name;
        cda_rail::Network::extract_vertices_from_key(key, source_name, target_name);
        int edge_id_in= get_edge_index(source_name, target_name);
        for (auto& tuple : val) {
            int edge_id_out = get_edge_index(tuple[0].get<std::string>(), tuple[1].get<std::string>());
            add_successor(edge_id_in, edge_id_out);
        }
    }
}

int cda_rail::Network::add_vertex(const std::string &name, VertexType type) {
    /**
     * Add vertex to network
     * @param name Name of vertex
     * @param type Type of vertex
     */
    if (has_vertex(name)) {
        throw std::domain_error("Vertex already exists");
    }
    vertices.emplace_back(name, type);
    vertex_name_to_index[name] = vertices.size() - 1;
    return vertex_name_to_index[name];
}

int cda_rail::Network::add_edge(int source, int target, double length, double max_speed,
                                 bool breakable, double min_block_length) {
    /**
     * Add edge to network
     * @param source Source vertex
     * @param target Target vertex
     * @param length Length of edge
     * @param max_speed Maximum speed on edge
     * @param breakable Whether edge is breakable
     * @param min_block_length Minimum block length on edge
     */
    if (source == target) {
        throw std::invalid_argument("Source and target are the same");
    }
    if (!has_vertex(source)) {
        throw std::out_of_range("Source vertex does not exist");
    }
    if (!has_vertex(target)) {
        throw std::out_of_range("Target vertex does not exist");
    }
    if (has_edge(source, target)) {
        throw std::domain_error("Edge already exists");
    }
    edges.emplace_back(source, target, length, max_speed, breakable, min_block_length);
    successors.emplace_back();
    return edges.size() - 1;
}

void cda_rail::Network::add_successor(int edge_in, int edge_out) {
    /**
     * Add successor to edge, but only if the edges are adjacent to each other
     *
     * @param edge_in Edge to add successor to
     * @param edge_out Edge to add as successor
     */
    if (!has_edge(edge_in)) {
        throw std::out_of_range("Edge in does not exist");
    }
    if (!has_edge(edge_out)) {
        throw std::out_of_range("Edge out does not exist");
    }
    if (edges[edge_in].target != edges[edge_out].source) {
        throw std::invalid_argument("Edges are not adjacent");
    }

    // If successors[edges] already contains edge_out, do nothing
    if (std::find(successors[edge_in].begin(), successors[edge_in].end(), edge_out) != successors[edge_in].end()) {
        return;
    }

    successors[edge_in].emplace_back(edge_out);
}

const cda_rail::Vertex &cda_rail::Network::get_vertex(int index) const {
    if (!has_vertex(index)) {
        throw std::out_of_range("Vertex does not exist");
    }
    return vertices[index];
}

int cda_rail::Network::get_vertex_index(const std::string &name) const {
    if (!has_vertex(name)) {
        throw std::out_of_range("Vertex does not exist");
    }
    return vertex_name_to_index.at(name);
}

const cda_rail::Edge &cda_rail::Network::get_edge(int index) const {
    if (!has_edge(index)) {
        throw std::out_of_range("Edge does not exist");
    }
    return edges[index];
}

const cda_rail::Edge &cda_rail::Network::get_edge(int source_id, int target_id) const {
    if (!has_vertex(source_id)) {
        throw std::out_of_range("Source vertex does not exist");
    }
    if (!has_vertex(target_id)) {
        throw std::out_of_range("Target vertex does not exist");
    }
    for (const auto &edge : edges) {
        if (edge.source == source_id && edge.target == target_id) {
            return edge;
        }
    }
    throw std::out_of_range("Edge does not exist");
}

int cda_rail::Network::get_edge_index(int source_id, int target_id) const {
    if (!has_vertex(source_id)) {
        throw std::out_of_range("Source vertex does not exist");
    }
    if (!has_vertex(target_id)) {
        throw std::out_of_range("Target vertex does not exist");
    }
    for (int i = 0; i < edges.size(); i++) {
        if (edges[i].source == source_id && edges[i].target == target_id) {
            return i;
        }
    }
    throw std::out_of_range("Edge does not exist");
}

bool cda_rail::Network::has_edge(int source_id, int target_id) const {
    if (!has_vertex(source_id)) {
        throw std::out_of_range("Source vertex does not exist");
    }
    if (!has_vertex(target_id)) {
        throw std::out_of_range("Target vertex does not exist");
    }
    return std::any_of(edges.begin(), edges.end(),
                       [source_id, target_id](const Edge &edge) {
                            return edge.source == source_id && edge.target == target_id;
                        });
}

bool cda_rail::Network::has_edge(const std::string &source_name, const std::string &target_name) const {
    if (!has_vertex(source_name)) {
        throw std::out_of_range("Source vertex does not exist");
    }
    if (!has_vertex(target_name)) {
        throw std::out_of_range("Target vertex does not exist");
    }
    return has_edge(get_vertex_index(source_name), get_vertex_index(target_name));
}

void cda_rail::Network::change_vertex_name(int index, const std::string &new_name) {
    if (!has_vertex(index)) {
        throw std::out_of_range("Vertex does not exist");
    }
    if (has_vertex(new_name)) {
        throw std::domain_error("Vertex with new name already exists");
    }
    vertex_name_to_index.erase(vertices[index].name);
    vertices[index].name = new_name;
    vertex_name_to_index[new_name] = index;
}

void cda_rail::Network::change_edge_property(int index, double value, const std::string &property) {
    if (!has_edge(index)) {
        throw std::out_of_range("Edge does not exist");
    }
    if (property == "length") {
        edges[index].length = value;
    } else if (property == "max_speed") {
        edges[index].max_speed = value;
    } else if (property == "min_block_length") {
        edges[index].min_block_length = value;
    } else {
        throw std::domain_error("Property does not exist");
    }
}

void cda_rail::Network::change_edge_breakable(int index, bool value) {
    if (!has_edge(index)) {
        throw std::out_of_range("Edge does not exist");
    }
    edges[index].breakable = value;
}

std::vector<int> cda_rail::Network::out_edges(int index) const {
    if (!has_vertex(index)) {
        throw std::out_of_range("Vertex does not exist");
    }
    std::vector<int> out_edges;
    for (int i = 0; i < edges.size(); ++i) {
        if (edges[i].source == index) {
            out_edges.emplace_back(i);
        }
    }
    return out_edges;
}

std::vector<int> cda_rail::Network::in_edges(int index) const {
    if (!has_vertex(index)) {
        throw std::out_of_range("Vertex does not exist");
    }
    std::vector<int> in_edges;
    for (int i = 0; i < edges.size(); ++i) {
        if (edges[i].target == index) {
            in_edges.emplace_back(i);
        }
    }
    return in_edges;
}

const std::vector<int> &cda_rail::Network::get_successors(int index) const {
    if (!has_edge(index)) {
        throw std::out_of_range("Edge does not exist");
    }
    return successors[index];
}

void cda_rail::to_bool_optional(std::string &s, std::optional<bool> &b) {
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    bool tmp;
    if (!!(std::istringstream(s) >> std::boolalpha >> tmp)) {
        b = tmp;
    }
}

void cda_rail::Network::export_graphml(const std::filesystem::path &p) const {
    /**
     * Export the network to a GraphML file in the given path.
     * @param path: The path to the directory.
     */

    // Open the file.
    std::ofstream file(p / "tracks.graphml");

    // Write the header.
    file << "<?xml version='1.0' encoding='UTF-8'?>" << std::endl;
    file << R"(<graphml xmlns="http://graphml.graphdrawing.org/xmlns" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="http://graphml.graphdrawing.org/xmlns http://graphml.graphdrawing.org/xmlns/1.0/graphml.xsd">)"
         << std::endl;

    // Write the key relations
    std::string breakable = "d0";
    std::string length = "d1";
    std::string max_speed = "d2";
    std::string min_block_length = "d3";
    std::string type = "d4";
    file << "<key id=\"" << breakable << R"(" for="edge" attr.name="breakable" attr.type="boolean"/>)" << std::endl;
    file << "<key id=\"" << length << R"(" for="edge" attr.name="length" attr.type="double"/>)" << std::endl;
    file << "<key id=\"" << max_speed << R"(" for="edge" attr.name="max_speed" attr.type="double"/>)" << std::endl;
    file << "<key id=\"" << min_block_length << R"(" for="edge" attr.name="min_block_length" attr.type="double"/>)"
         << std::endl;
    file << "<key id=\"" << type << R"(" for="edge" attr.name="type" attr.type="long"/>)" << std::endl;

    // Write the graph header
    file << "<graph edgedefault=\"directed\">" << std::endl;

    // Write the vertices
    for (const auto &vertex : vertices) {
        file << "<node id=\"" << vertex.name << "\">" << std::endl;
        file << "<data key=\"" << type << "\">" << static_cast<int>(vertex.type) << "</data>" << std::endl;
        file << "</node>" << std::endl;
    }

    // Write the edges
    for (const auto &edge : edges) {
        file << "<edge source=\"" << vertices[edge.source].name << "\" target=\"" << vertices[edge.target].name << "\">"
             << std::endl;
        file << "<data key=\"" << breakable << "\">" << std::boolalpha << edge.breakable << "</data>" << std::endl;
        file << "<data key=\"" << length << "\">" << edge.length << "</data>" << std::endl;
        file << "<data key=\"" << max_speed << "\">" << edge.max_speed << "</data>" << std::endl;
        file << "<data key=\"" << min_block_length << "\">" << edge.min_block_length << "</data>" << std::endl;
        file << "</edge>" << std::endl;
    }

    // Write the footer
    file << "</graph>" << std::endl;
    file << "</graphml>" << std::endl;

    // Close the file.
    file.close();
}

void cda_rail::Network::export_successors_cpp(const std::filesystem::path &p) const {
    /**
     * Export the successors to successors_cpp.json for C++ in the given directory.
     * @param path: The path to the directory.
     */

    json j;
    for (int i = 0; i < number_of_edges(); ++i) {
        const auto& edge = get_edge(i);
        std::vector<std::pair<std::string, std::string>> successor_edges_export;
        for (const auto& successor : successors[i]) {
            const auto& successor_edge = get_edge(successor);
            successor_edges_export.emplace_back(vertices[successor_edge.source].name, vertices[successor_edge.target].name);
        }
        j["('" + vertices[edge.source].name + "', '" + vertices[edge.target].name + "')"] = successor_edges_export;
    }

    // Open and write the file for C++.
    std::ofstream file(p / "successors_cpp.json");
    file << j << std::endl;
}

void cda_rail::Network::export_successors_python(const std::filesystem::path &p) const {
    /**
     * Export the successors to successors.txt for Python in the given directory.
     * @param path: The path to the directory.
     */

    // Open the file for Python.
    std::ofstream file(p / "successors.txt");

    // Write the beginning of the file.
    file << "{";

    // Write the successors
    bool first_key = true;
    for (int i = 0; i < number_of_edges(); ++i) {
        const auto& edge = get_edge(i);
        // Write comma if not first key
        if (first_key) {
            first_key = false;
        } else {
            file << ", ";
        }

        // Write the key and set
        file << "('" << vertices[edge.source].name << "', '" << vertices[edge.target].name << "'): ";
        write_successor_set_to_file(file, i);
    }

    // Write the end of the file.
    file << "}" << std::endl;
}

void cda_rail::Network::write_successor_set_to_file(std::ofstream& file, int i) const {
    /**
     * Write the successor set to the given file.
     * @param file: The file to write to.
     * @param i: The index of the edge.
     */

    // If there are no successors, write set(), otherwise write all successors.
    if (get_successors(i).empty()) {
        // Empty set
        file << "set()";
    } else {
        // Write the beginning of the set.
        file << "{";

        // Write the elements of the set.
        bool first_element = true;
        for (const auto successor : get_successors(i)) {
            // Write comma if not first element
            if (first_element) {
                first_element = false;
            } else {
                file << ", ";
            }

            // Write the element
            const auto& successor_edge = get_edge(successor);
            file << "('" << vertices[successor_edge.source].name << "', '" << vertices[successor_edge.target].name << "')";
        }

        // Write the end of the set.
        file << "}";
    }
}

void cda_rail::Network::export_network(const std::filesystem::path &p) const {
    /** Export the network to a given directory. This includes the graphml and successors files.
     * @param path: The path to the directory.
     */

    // Create the directory if it does not exist.
    if (!cda_rail::is_directory_and_create(p)) {
        throw std::runtime_error("Could not create directory " + p.string());
    }

    // Export the graphml file.
    export_graphml(p);

    // Export the successors files.
    export_successors_cpp(p);
    export_successors_python(p);
}

bool cda_rail::Network::is_valid_successor(int e0, int e1) const {
    /**
     * Checks if the edge e1 is a valid successor of the edge e0, this includes the following:
     * - The source of e1 is the target of e0, i.e., they are adjacent
     * - e1 is a valid successor of e0 according to the successors map (not all edges on turnouts e.g.)
     *
     * @param e0: The edge to check the successors of.
     * @param e1: The edge to check if it is a successor of e0.
     * @return: True if e1 is a valid successor of e0, false otherwise.
     */
     if (!has_edge(e0)) {
        throw std::out_of_range("Edge does not exist");
     }
     if (!has_edge(e1)) {
         throw std::out_of_range("EDge does not exist");
     }

     if (edges[e0].target != edges[e1].source) {
         return false;
     }
     return (std::find(successors[e0].begin(), successors[e0].end(), e1) != successors[e0].end());
}

std::vector<int> cda_rail::Network::neighbors(int index) const {
    if (!has_vertex(index)) {
        throw std::out_of_range("Vertex does not exist");
    }
    std::vector<int> neighbors;
    auto e_out = out_edges(index);
    auto e_in = in_edges(index);
    for (auto e : e_out) {
        // If e is not in neighbors, emplace back
        if (std::find(neighbors.begin(), neighbors.end(), get_edge(e).target) == neighbors.end()) {
            neighbors.emplace_back(get_edge(e).target);
        }
    }
    for (auto e : e_in) {
        // If e is not in neighbors, emplace back
        if (std::find(neighbors.begin(), neighbors.end(), get_edge(e).source) == neighbors.end()) {
            neighbors.emplace_back(get_edge(e).source);
        }
    }
    return neighbors;
}

cda_rail::Network::Network(const std::filesystem::path &p) {
    /**
     * Construct object and read network from path. This includes the graph and successors.
     * @param p Path to network directory
     * @return Network
     */

    // Check if path exists
    if (!std::filesystem::exists(p)) {
        throw std::runtime_error("Path does not exist");
    }
    if (!std::filesystem::is_directory(p)) {
        throw std::runtime_error("Path is not a directory");
    }

    // Read graph
    this->read_graphml(p);

    // Read successors
    this->read_successors(p);
}
