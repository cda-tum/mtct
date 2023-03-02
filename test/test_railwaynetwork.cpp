#include "RailwayNetwork.hpp"
#include "gtest/gtest.h"
#include "xml/tinyxml2.h"
#include <iostream>
#include <filesystem>

TEST(Functionaliy, SimpleTest) {
    cda_rail::Network g;
    EXPECT_TRUE(g.edges.size() == 0); // dummy test
}

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
}