#pragma once
#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Timetable.hpp"

namespace cda_rail::instances{
    class VSSGenerationTimetable {
        private:
            cda_rail::Network network;
            cda_rail::Timetable timetable;
    };
}