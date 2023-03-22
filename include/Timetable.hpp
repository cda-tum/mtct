#pragma once
#include <string>
#include <unordered_set>

namespace cda_rail {
    struct Station {
        /**
         * Station object
         * @param name Name of the station
         * @param tracks Unordered set of edges that define the station.
         */
        std::string name;
        std::unordered_set<int> tracks;
    };
}