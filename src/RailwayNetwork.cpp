#include "RailwayNetwork.hpp"
#include <string>

std::string cda_rail::Edge::toString() const {
    return "('" + source->name + "', '" + target->name + "')";
}