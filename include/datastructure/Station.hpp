#pragma once
#include <string>
#include <vector>
#include <unordered_map>
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
        std::vector<int> tracks = {};
    };

    class StationList {
        /**
         * StationList class
         */
        private:
            std::unordered_map<std::string, Station> stations;

        public:
            // Constructors
            StationList() = default;
            StationList(const std::filesystem::path& p, const cda_rail::Network& network);
            StationList(const std::string& path, const cda_rail::Network& network) : StationList(std::filesystem::path(path), network) {};
            StationList(const char* path, const cda_rail::Network& network) : StationList(std::filesystem::path(path), network) {};

            // Rule of 5
            StationList(const StationList& other) = default;
            StationList(StationList&& other) = default;
            StationList& operator=(const StationList& other) = default;
            StationList& operator=(StationList&& other) = default;
            ~StationList() = default;

            void add_station(const std::string& name) {stations[name] = cda_rail::Station{name};};

            [[nodiscard]] bool has_station(const std::string& name) const {return stations.find(name) != stations.end();};
            [[nodiscard]] const Station& get_station(const std::string& name) const;

            [[nodiscard]] int size() const {return stations.size();};
            [[nodiscard]] std::vector<std::string> get_station_names() const;

            void add_track_to_station(const std::string& name, int track, const cda_rail::Network& network);
            void add_track_to_station(const std::string& name, int source, int target, const cda_rail::Network& network) {
                add_track_to_station(name, network.get_edge_index(source, target), network);
            };
            void add_track_to_station(const std::string& name, const std::string& source, const std::string& target, const cda_rail::Network& network) {
                add_track_to_station(name, network.get_edge_index(source, target), network);
            };

            void export_stations(const std::string& path, const cda_rail::Network& network) const;
            void export_stations(const char* path, const cda_rail::Network& network) const {export_stations(std::filesystem::path(path), network);};
            void export_stations(const std::filesystem::path& p, const cda_rail::Network& network) const;
            [[nodiscard]] static cda_rail::StationList import_stations(const std::string& path, const cda_rail::Network& network) {return StationList(path, network);};
            [[nodiscard]] static cda_rail::StationList import_stations(const char* path, const cda_rail::Network& network) {return StationList(path, network);};
            [[nodiscard]] static cda_rail::StationList import_stations(const std::filesystem::path& p, const cda_rail::Network& network) {return StationList(p, network);};
    };
}
