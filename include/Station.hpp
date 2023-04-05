#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include "RailwayNetwork.hpp"
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
            std::vector<Station> stations;
            std::unordered_map<std::string, int> station_name_to_index;

        public:
            void add_station(const std::string& name, const std::unordered_set<int>& tracks);
            void add_station(const std::string& name);

            [[nodiscard]] bool has_station(const std::string& name) const;
            [[nodiscard]] bool has_station(int index) const;

            [[nodiscard]] int get_station_index(const std::string& name) const;
            [[nodiscard]] const Station& get_station(int index) const;
            [[nodiscard]] const Station& get_station(const std::string& name) const;

            [[nodiscard]] const cda_rail::Network& get_network() const;

            void add_track_to_station(int station_index, int track, const cda_rail::Network& network);
            void add_track_to_station(const std::string& name, int track, const cda_rail::Network& network);
            void add_track_to_station(int station_index, int source, int target, const cda_rail::Network& network);
            void add_track_to_station(const std::string& name, int source, int target, const cda_rail::Network& network);
            void add_track_to_station(int station_index, const std::string& source, const std::string& target, const cda_rail::Network& network);
            void add_track_to_station(const std::string& name, const std::string& source, const std::string& target, const cda_rail::Network& network);

            void export_stations(const std::string& path, const cda_rail::Network& network) const;
            void export_stations(const std::filesystem::path& p, const cda_rail::Network& network) const;
            [[nodiscard]] static cda_rail::StationList import_stations(const std::string& path, const cda_rail::Network& network);
            [[nodiscard]] static cda_rail::StationList import_stations(const std::filesystem::path& p, const cda_rail::Network& network);
    };
}
