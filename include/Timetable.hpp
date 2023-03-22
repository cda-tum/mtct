#pragma once
#include <string>
#include <unordered_set>
#include <unordered_map>
#include "Train.hpp"

namespace cda_rail {
    struct Station {
        /**
         * Station object
         * @param name Name of the station
         * @param tracks Unordered set of edges that define the station.
         */
        std::string name;
        std::unordered_set<int> tracks = {};
    };

    struct Interval {
        /**
         * Interval struct in seconds.
         */
        int begin;
        int end;
    };

    class Timetable {
        /**
         * Timetable class
         */
        private:
            std::vector<Station> stations;
            std::vector<Train> trains;
            std::unordered_map<std::string, int> station_name_to_index;
            std::unordered_map<std::string, int> train_name_to_index;
    };
}