#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include "datastructure/RailwayNetwork.hpp"
#include <filesystem>

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

    class StationList {
        /**
         * StationList class
         */
        private:
            std::unordered_map<std::string, Station> stations;

        public:
            void add_station(const std::string& name, const std::unordered_set<int>& tracks);
            void add_station(const std::string& name);

            [[nodiscard]] bool has_station(const std::string& name) const;
            [[nodiscard]] const Station& get_station(const std::string& name) const;

            [[nodiscard]] int size() const;
            [[nodiscard]] std::vector<std::string> get_station_names() const;

            void add_track_to_station(const std::string& name, int track, const cda_rail::Network& network);
            void add_track_to_station(const std::string& name, int source, int target, const cda_rail::Network& network);
            void add_track_to_station(const std::string& name, const std::string& source, const std::string& target, const cda_rail::Network& network);

            void export_stations(const std::string& path, const cda_rail::Network& network) const;
            void export_stations(const char* path, const cda_rail::Network& network) const;
            void export_stations(const std::filesystem::path& p, const cda_rail::Network& network) const;
            [[nodiscard]] static cda_rail::StationList import_stations(const std::string& path, const cda_rail::Network& network);
            [[nodiscard]] static cda_rail::StationList import_stations(const char* path, const cda_rail::Network& network);
            [[nodiscard]] static cda_rail::StationList import_stations(const std::filesystem::path& p, const cda_rail::Network& network);
    };
}
