#include "RailwayNetwork.hpp"
#include "gtest/gtest.h"
#include "parsing/tinyxml2.h"
#include <iostream>
#include <filesystem>
#include <unordered_map>
#include <algorithm>
#include <cctype>
#include "parsing/json.hpp"
#include <fstream>

using json = nlohmann::json;

TEST(Functionaliy, SimpleTest) {
    cda_rail::Network g;
    EXPECT_TRUE(g.edges.size() == 0); // dummy test
}

/**
TEST(Functionality, ReadXML) {
    tinyxml2::XMLDocument graph_xml;
    graph_xml.LoadFile("./example-networks/Fig11/network/tracks.graphml");
    std::cout << "Current path is "  << std::filesystem::current_path() << std::endl;
    std::cout << "Try loading example-networks/Fig11/network/tracks.graphml" << std::endl;
    std::cout << "XML loading message: " << ((graph_xml.ErrorID() == 0) ? "OK" : graph_xml.ErrorStr()) << std::endl;
    EXPECT_TRUE(graph_xml.ErrorID() == 0);
}

TEST(Functionality, ParseXML) {
    tinyxml2::XMLDocument graph_xml;
    graph_xml.LoadFile("./example-networks/Fig11/network/tracks.graphml");
    EXPECT_TRUE(graph_xml.ErrorID() == 0);

    tinyxml2::XMLElement* graphml_body = graph_xml.FirstChildElement("graphml");

    tinyxml2::XMLElement* graphml_key = graphml_body->FirstChildElement("key");
    std::string breakable, min_block_length, max_speed, length, type;
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
    std::cout << "breakable: " << breakable << std::endl
        << "min_block_length: " << min_block_length << std::endl
        << "max_speed: " << max_speed << std::endl
        << "length: " << length << std::endl
        << "type: " << type << std::endl;

    tinyxml2::XMLElement* graphml_graph = graphml_body->FirstChildElement("graph");
    std::cout << "Graph type: " << graphml_graph->Attribute("edgedefault") << std::endl;
    EXPECT_TRUE((graphml_graph->Attribute("edgedefault")) == std::string("directed"));

    cda_rail::Network nx = cda_rail::Network();
    std::unordered_map<std::string, int> index_map = std::unordered_map<std::string, int>();

    tinyxml2::XMLElement* graphml_node = graphml_graph->FirstChildElement("node");
    while (graphml_node) {
        cda_rail::Vertex v = cda_rail::Vertex();
        v.name = graphml_node->Attribute("id");
        tinyxml2::XMLElement* graphml_data = graphml_node->FirstChildElement("data");
        while (graphml_data) {
            if (graphml_data->Attribute("key") == type) {
                v.type = std::stoi(graphml_data->GetText());
            }
            graphml_data = graphml_data->NextSiblingElement("data");
        }
        nx.vertices.push_back(v);
        index_map[v.name] = nx.vertices.size() - 1;
        std::cout << "Vector has " << nx.vertices.size() << " elements and a capacity of " << nx.vertices.capacity() << std::endl;
        graphml_node = graphml_node->NextSiblingElement("node");
    }

    for (int i = 0; i < nx.vertices.size(); ++i) {
        std::cout << "Vertex " << i << ": " << nx.vertices[i].name << " mapped to " << index_map[nx.vertices[i].name] << std::endl;
        EXPECT_TRUE(i == index_map[nx.vertices[i].name]);
    }

    tinyxml2::XMLElement* graphml_edge = graphml_graph->FirstChildElement("edge");
    while (graphml_edge) {
        cda_rail::Edge e = cda_rail::Edge();
        e.source = &nx.vertices[index_map[graphml_edge->Attribute("source")]];
        e.target = &nx.vertices[index_map[graphml_edge->Attribute("target")]];

        tinyxml2::XMLElement* graphml_data = graphml_edge->FirstChildElement("data");
        while (graphml_data) {
            if (graphml_data->Attribute("key") == length) {
                e.length = std::stod(graphml_data->GetText());
            }
            else if (graphml_data->Attribute("key") == max_speed) {
                e.max_speed = std::stod(graphml_data->GetText());
            }
            else if (graphml_data->Attribute("key") == min_block_length) {
                e.min_block_length = std::stod(graphml_data->GetText());
            }
            else if (graphml_data->Attribute("key") == breakable) {
                std::string breakable_str = graphml_data->GetText();
                std::transform(breakable_str.begin(), breakable_str.end(), breakable_str.begin(), ::tolower);
                std:std::istringstream is(breakable_str);
                is >> std::boolalpha >> e.breakable;
            }
            graphml_data = graphml_data->NextSiblingElement("data");
        }
        nx.edges.push_back(e);
        graphml_edge = graphml_edge->NextSiblingElement("edge");
    }
}

TEST(Functionality, JSON) {
    tinyxml2::XMLDocument graph_xml;
    graph_xml.LoadFile("./example-networks/Fig11/network/tracks.graphml");
    EXPECT_TRUE(graph_xml.ErrorID() == 0);

    tinyxml2::XMLElement* graphml_body = graph_xml.FirstChildElement("graphml");

    tinyxml2::XMLElement* graphml_key = graphml_body->FirstChildElement("key");
    std::string breakable, min_block_length, max_speed, length, type;
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

    tinyxml2::XMLElement* graphml_graph = graphml_body->FirstChildElement("graph");
    EXPECT_TRUE((graphml_graph->Attribute("edgedefault")) == std::string("directed"));

    cda_rail::Network nx = cda_rail::Network();
    std::unordered_map<std::string, int> vertex_index_map = std::unordered_map<std::string, int>();

    tinyxml2::XMLElement* graphml_node = graphml_graph->FirstChildElement("node");
    while (graphml_node) {
        cda_rail::Vertex v = cda_rail::Vertex();
        v.name = graphml_node->Attribute("id");
        tinyxml2::XMLElement* graphml_data = graphml_node->FirstChildElement("data");
        while (graphml_data) {
            if (graphml_data->Attribute("key") == type) {
                v.type = std::stoi(graphml_data->GetText());
            }
            graphml_data = graphml_data->NextSiblingElement("data");
        }
        nx.vertices.push_back(v);
        vertex_index_map[v.name] = nx.vertices.size() - 1;
        graphml_node = graphml_node->NextSiblingElement("node");
    }

    for (int i = 0; i < nx.vertices.size(); ++i) {
        EXPECT_TRUE(i == vertex_index_map[nx.vertices[i].name]);
    }

    std::unordered_map<std::string, int> edge_index_map = std::unordered_map<std::string, int>();
    tinyxml2::XMLElement* graphml_edge = graphml_graph->FirstChildElement("edge");
    while (graphml_edge) {
        cda_rail::Edge e = cda_rail::Edge();
        e.source = &nx.vertices[vertex_index_map[graphml_edge->Attribute("source")]];
        e.target = &nx.vertices[vertex_index_map[graphml_edge->Attribute("target")]];

        tinyxml2::XMLElement* graphml_data = graphml_edge->FirstChildElement("data");
        while (graphml_data) {
            if (graphml_data->Attribute("key") == length) {
                e.length = std::stod(graphml_data->GetText());
            }
            else if (graphml_data->Attribute("key") == max_speed) {
                e.max_speed = std::stod(graphml_data->GetText());
            }
            else if (graphml_data->Attribute("key") == min_block_length) {
                e.min_block_length = std::stod(graphml_data->GetText());
            }
            else if (graphml_data->Attribute("key") == breakable) {
                std::string breakable_str = graphml_data->GetText();
                std::transform(breakable_str.begin(), breakable_str.end(), breakable_str.begin(), ::tolower);
                std:std::istringstream is(breakable_str);
                is >> std::boolalpha >> e.breakable;
            }
            graphml_data = graphml_data->NextSiblingElement("data");
        }
        nx.edges.push_back(e);
        edge_index_map[e.toString()] = nx.edges.size() - 1;
        graphml_edge = graphml_edge->NextSiblingElement("edge");
    }

    std::ifstream f("./example-networks/Fig11/network/successors_cpp_2.json");
    json data = json::parse(f);
    for (auto& [key, val] : data.items()) {
        std::cout << "key: " << key << ", value: " << val << " of size " << val.size() << std::endl;
        for (auto& tuple : val) {
            std::string edge_str = "('" + tuple[0].get<std::string>() + "', '" + tuple[1].get<std::string>() + "')";
            std::cout << "edge_str: " << edge_str << std::endl;
        }
    }
}
 **/