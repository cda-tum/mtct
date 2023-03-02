#include "RailwayNetwork.hpp"
#include "gtest/gtest.h"
#include "xml/tinyxml2.h"
#include <iostream>
#include <filesystem>

TEST(Functionaliy, SimpleTest) {
    cda_rail::Network g;
    EXPECT_TRUE(g.edges.size() == 0); // dummy test
}

TEST(Functionality, XML) {
    tinyxml2::XMLDocument graph_xml;
    graph_xml.LoadFile("./example-networks/Fig11/network/tracks.gexf");
    std::cout << "Current path is "  << std::filesystem::current_path() << std::endl;
    std::cout << "Try loading example-networks/Fig11/network/tracks.gexf" << std::endl;
    std::cout << "XML loading message: " << ((graph_xml.ErrorID() == 0) ? "OK" : graph_xml.ErrorStr()) << std::endl;
    EXPECT_TRUE(graph_xml.ErrorID() == 0);
}