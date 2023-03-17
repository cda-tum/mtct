#include "RailwayNetwork.hpp"
#include <string>
#include "parsing/tinyxml2.h"
#include <filesystem>
#include <optional>

cda_rail::Network cda_rail::Network::read_network(const std::string& path) {
    /**
     * Read network from XML file
     * @param path Path to XML file
     * @return Network object
     */

    // Open graphml file
    std::filesystem::path p(path);
    tinyxml2::XMLDocument graph_xml;
    graph_xml.LoadFile((p / "tracks.graphml").string().c_str());
    if (graph_xml.Error()) {
        throw std::runtime_error("Error reading graphml file");
    }

    // Get graphml body
    tinyxml2::XMLElement* graphml_body = graph_xml.FirstChildElement("graphml");

    // Get keys
    tinyxml2::XMLElement* graphml_key = graphml_body->FirstChildElement("key");
    std::string breakable = "";
    std::string min_block_length = "";
    std::string max_speed = "";
    std::string length = "";
    std::string type = "";

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

    if (breakable == "" or min_block_length == "" or max_speed == "" or length == "" or type == "") {
        throw std::runtime_error("Error reading graphml file");
    }

    // Get graph
    const tinyxml2::XMLElement* graphml_graph = graphml_body->FirstChildElement("graph");
    if ((graphml_graph->Attribute("edgedefault")) != std::string("directed")) {
        throw std::runtime_error("Graph is not directed. Not all properties present.");
    }

    // Initialize network
    auto network = cda_rail::Network();

    // Get vertices
    const tinyxml2::XMLElement* graphml_node = graphml_graph->FirstChildElement("node");
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
        if (not v_type.has_value()) {
            throw std::runtime_error("Error reading graphml file");
        }
        network.add_vertex(name, v_type.value());
        graphml_node = graphml_node->NextSiblingElement("node");
    }

    // Get edges
    const tinyxml2::XMLElement* graphml_edge = graphml_graph->FirstChildElement("edge");
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
                e_breakable = std::stoi(graphml_data->GetText());
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
        if (not e_length.has_value() or not e_max_speed.has_value() or not e_breakable.has_value() or not e_min_block_length.has_value()) {
            throw std::runtime_error("Error reading graphml file");
        }
        network.add_edge(source_name, target_name, e_length.value(), e_max_speed.value(), e_breakable.value(), e_min_block_length.value());
        graphml_edge = graphml_edge->NextSiblingElement("edge");
    }

    return network;
}

void cda_rail::Network::add_vertex(const std::string &name, const int type) {
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
}

void cda_rail::Network::add_edge(const int source, const int target, const double length, const double max_speed,
                                 const bool breakable, const double min_block_length) {
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
    if (not has_vertex(source)) {
        throw std::out_of_range("Source vertex does not exist");
    }
    if (not has_vertex(target)) {
        throw std::out_of_range("Target vertex does not exist");
    }
    if (has_edge(source, target)) {
        throw std::domain_error("Edge already exists");
    }
    cda_rail::Edge e = {source, target, length, max_speed, breakable, min_block_length};
    edges.push_back(e);
    successors.emplace_back();
}

void cda_rail::Network::add_edge(const std::string &source_name, const std::string &target_name, const double length,
                                 const double max_speed, const bool breakable, const double min_block_length) {
    /**
     * Add edge to network
     * @param source_name Name of source vertex
     * @param target_name Name of target vertex
     * @param length Length of edge
     * @param max_speed Maximum speed on edge
     * @param breakable Whether edge is breakable
     * @param min_block_length Minimum block length on edge
     */
    if (not has_vertex(source_name)) {
        throw std::out_of_range("Source vertex does not exist");
    }
    if (not has_vertex(target_name)) {
        throw std::out_of_range("Target vertex does not exist");
    }
    add_edge(get_vertex_index(source_name), get_vertex_index(target_name), length, max_speed, breakable,
             min_block_length);
}

void cda_rail::Network::add_successor(const int edge_in, const int edge_out) {
    /**
     * Add successor to edge
     * @param edge_in Edge to add successor to
     * @param edge_out Edge to add as successor
     */
    if (not has_edge(edge_in)) {
        throw std::out_of_range("Edge in does not exist");
    }
    if (not has_edge(edge_out)) {
        throw std::out_of_range("Edge out does not exist");
    }
    successors[edge_in].insert(edge_out);
}

const cda_rail::Vertex &cda_rail::Network::get_vertex(const int index) const {
    if (not has_vertex(index)) {
        throw std::out_of_range("Vertex does not exist");
    }
    return vertices[index];
}

const cda_rail::Vertex &cda_rail::Network::get_vertex(const std::string &name) const {
    if (not has_vertex(name)) {
        throw std::out_of_range("Vertex does not exist");
    }
    return vertices[get_vertex_index(name)];
}

int cda_rail::Network::get_vertex_index(const std::string &name) const {
    if (not has_vertex(name)) {
        throw std::out_of_range("Vertex does not exist");
    }
    return vertex_name_to_index.at(name);
}

const cda_rail::Edge &cda_rail::Network::get_edge(const int index) const {
    if (not has_edge(index)) {
        throw std::out_of_range("Edge does not exist");
    }
    return edges[index];
}

const cda_rail::Edge &cda_rail::Network::get_edge(const int source_id, const int target_id) const {
    if (not has_vertex(source_id)) {
        throw std::out_of_range("Source vertex does not exist");
    }
    if (not has_vertex(target_id)) {
        throw std::out_of_range("Target vertex does not exist");
    }
    for (const auto &edge : edges) {
        if (edge.source == source_id and edge.target == target_id) {
            return edge;
        }
    }
    throw std::out_of_range("Edge does not exist");
}

const cda_rail::Edge &cda_rail::Network::get_edge(const std::string &source_name,
                                                  const std::string &target_name) const {
    if (not has_vertex(source_name)) {
        throw std::out_of_range("Source vertex does not exist");
    }
    if (not has_vertex(target_name)) {
        throw std::out_of_range("Target vertex does not exist");
    }
    return get_edge(get_vertex_index(source_name), get_vertex_index(target_name));
}

int cda_rail::Network::get_edge_index(const int source_id, const int target_id) const {
    if (not has_vertex(source_id)) {
        throw std::out_of_range("Source vertex does not exist");
    }
    if (not has_vertex(target_id)) {
        throw std::out_of_range("Target vertex does not exist");
    }
    for (int i = 0; i < edges.size(); i++) {
        if (edges[i].source == source_id and edges[i].target == target_id) {
            return i;
        }
    }
    throw std::out_of_range("Edge does not exist");
}

int cda_rail::Network::get_edge_index(const std::string &source_name, const std::string &target_name) const {
    if (not has_vertex(source_name)) {
        throw std::out_of_range("Source vertex does not exist");
    }
    if (not has_vertex(target_name)) {
        throw std::out_of_range("Target vertex does not exist");
    }
    return get_edge_index(get_vertex_index(source_name), get_vertex_index(target_name));
}

bool cda_rail::Network::has_vertex(const int index) const {
    return (index >= 0 and index < vertices.size());
}

bool cda_rail::Network::has_vertex(const std::string &name) const {
    return vertex_name_to_index.find(name) != vertex_name_to_index.end();
}

bool cda_rail::Network::has_edge(const int id) const {
    return (id >= 0 and id < edges.size());
}

bool cda_rail::Network::has_edge(const int source_id, const int target_id) const {
    if (not has_vertex(source_id)) {
        throw std::out_of_range("Source vertex does not exist");
    }
    if (not has_vertex(target_id)) {
        throw std::out_of_range("Target vertex does not exist");
    }
    for (const auto &edge : edges) {
        if (edge.source == source_id and edge.target == target_id) {
            return true;
        }
    }
    return false;
}

bool cda_rail::Network::has_edge(const std::string &source_name, const std::string &target_name) const {
    if (not has_vertex(source_name)) {
        throw std::out_of_range("Source vertex does not exist");
    }
    if (not has_vertex(target_name)) {
        throw std::out_of_range("Target vertex does not exist");
    }
    return has_edge(get_vertex_index(source_name), get_vertex_index(target_name));
}

void cda_rail::Network::change_vertex_name(const int index, const std::string &new_name) {
    if (not has_vertex(index)) {
        throw std::out_of_range("Vertex does not exist");
    }
    if (has_vertex(new_name)) {
        throw std::domain_error("Vertex with new name already exists");
    }
    vertex_name_to_index.erase(vertices[index].name);
    vertices[index].name = new_name;
    vertex_name_to_index[new_name] = index;
}

void cda_rail::Network::change_vertex_name(const std::string &old_name, const std::string &new_name) {
    if (not has_vertex(old_name)) {
        throw std::out_of_range("Vertex does not exist");
    }
    if (has_vertex(new_name)) {
        throw std::domain_error("Vertex with new name already exists");
    }
    change_vertex_name(get_vertex_index(old_name), new_name);
}

void cda_rail::Network::change_edge_property(const int index, const double value, const std::string &property) {
    if (not has_edge(index)) {
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

void cda_rail::Network::change_edge_property(const int source_id, const int target_id, const double value,
                                             const std::string &property) {
    if (not has_edge(source_id, target_id)) {
        throw std::out_of_range("Edge does not exist");
    }
    change_edge_property(get_edge_index(source_id, target_id), value, property);
}

void cda_rail::Network::change_edge_property(const std::string &source_name, const std::string &target_name,
                                             const double value, const std::string &property) {
    if (not has_edge(source_name, target_name)) {
        throw std::out_of_range("Edge does not exist");
    }
    change_edge_property(get_edge_index(source_name, target_name), value, property);
}

void cda_rail::Network::change_edge_breakable(const int index, const bool value) {
    if (not has_edge(index)) {
        throw std::out_of_range("Edge does not exist");
    }
    edges[index].breakable = value;
}

void cda_rail::Network::change_edge_breakable(const int source_id, const int target_id, const bool value) {
    if (not has_edge(source_id, target_id)) {
        throw std::out_of_range("Edge does not exist");
    }
    change_edge_breakable(get_edge_index(source_id, target_id), value);
}

void cda_rail::Network::change_edge_breakable(const std::string &source_name, const std::string &target_name,
                                              const bool value) {
    if (not has_edge(source_name, target_name)) {
        throw std::out_of_range("Edge does not exist");
    }
    change_edge_breakable(get_edge_index(source_name, target_name), value);
}

const std::unordered_set<int> &cda_rail::Network::out_edges(const int index) const {
    if (not has_vertex(index)) {
        throw std::out_of_range("Vertex does not exist");
    }
    std::unordered_set<int> out_edges;
    for (int i = 0; i < edges.size(); ++i) {
        if (edges[i].source == index) {
            out_edges.insert(i);
        }
    }
    return out_edges;
}

const std::unordered_set<int> &cda_rail::Network::out_edges(const std::string &name) const {
    if (not has_vertex(name)) {
        throw std::out_of_range("Vertex does not exist");
    }
    return out_edges(get_vertex_index(name));
}

const std::unordered_set<int> &cda_rail::Network::in_edges(const int index) const {
    if (not has_vertex(index)) {
        throw std::out_of_range("Vertex does not exist");
    }
    std::unordered_set<int> in_edges;
    for (int i = 0; i < edges.size(); ++i) {
        if (edges[i].target == index) {
            in_edges.insert(i);
        }
    }
    return in_edges;
}

const std::unordered_set<int> &cda_rail::Network::in_edges(const std::string &name) const {
    if (not has_vertex(name)) {
        throw std::out_of_range("Vertex does not exist");
    }
    return in_edges(get_vertex_index(name));
}
