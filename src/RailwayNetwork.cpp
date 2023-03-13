#include "RailwayNetwork.hpp"
#include <string>

std::string cda_rail::Edge::toString() const {
    return "('" + source->name + "', '" + target->name + "')";
}

cda_rail::Network cda_rail::Network::read_network(std::string path) {
    return cda_rail::Network();
}