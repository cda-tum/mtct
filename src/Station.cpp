#include "Station.hpp"
#include "RailwayNetwork.hpp"
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>
#include <unordered_map>
#include "nlohmann/json.hpp"

using json = nlohmann::json;

cda_rail::StationList::StationList(const cda_rail::Network &network): network(network) {};

void cda_rail::StationList::add_station(const std::string &name, const std::unordered_set<int> &tracks) {
    stations.push_back(cda_rail::Station{name, tracks});
    station_name_to_index[name] = stations.size() - 1;
}

void cda_rail::StationList::add_station(const std::string &name) {
    stations.push_back(cda_rail::Station{name});
    station_name_to_index[name] = stations.size() - 1;
}

bool cda_rail::StationList::has_station(const std::string &name) const {
    return station_name_to_index.find(name) != station_name_to_index.end();
}

bool cda_rail::StationList::has_station(int index) const {
    return (index >= 0 && index < stations.size());
}

int cda_rail::StationList::get_station_index(const std::string &name) const {
    if (!has_station(name)) {
        throw std::out_of_range("Station does not exist.");
    }
    return station_name_to_index.at(name);
}

const cda_rail::Station &cda_rail::StationList::get_station(int index) const {
    if (!has_station(index)) {
        throw std::out_of_range("Station does not exist.");
    }
    return stations.at(index);
}

const cda_rail::Station &cda_rail::StationList::get_station(const std::string &name) const {
    return get_station(get_station_index(name));
}

void cda_rail::StationList::add_track_to_station(int station_index, int track) {
    if (!has_station(station_index)) {
        throw std::out_of_range("Station does not exist.");
    }
    if (!network.has_edge(track)) {
        throw std::out_of_range("Track does not exist.");
    }
    stations.at(station_index).tracks.insert(track);
}

void cda_rail::StationList::add_track_to_station(const std::string &name, int track) {
    if (!has_station(name)) {
        throw std::out_of_range("Station does not exist.");
    }
    if (!network.has_edge(track)) {
        throw std::out_of_range("Track does not exist.");
    }
    add_track_to_station(get_station_index(name), track);
}

void
cda_rail::StationList::add_track_to_station(int station_index, int source, int target) {
    if (!has_station(station_index)) {
        throw std::out_of_range("Station does not exist.");
    }
    if (!network.has_edge(source, target)) {
        throw std::out_of_range("Track does not exist.");
    }
    add_track_to_station(station_index, network.get_edge_index(source, target));
}

void
cda_rail::StationList::add_track_to_station(const std::string &name, int source, int target) {
    if (!has_station(name)) {
        throw std::out_of_range("Station does not exist.");
    }
    if (!network.has_edge(source, target)) {
        throw std::out_of_range("Track does not exist.");
    }
    add_track_to_station(get_station_index(name), network.get_edge_index(source, target));
}

void cda_rail::StationList::add_track_to_station(int station_index, const std::string &source, const std::string &target) {
    if (!has_station(station_index)) {
        throw std::out_of_range("Station does not exist.");
    }
    if (!network.has_edge(source, target)) {
        throw std::out_of_range("Track does not exist.");
    }
    add_track_to_station(station_index, network.get_edge_index(source, target));
}

void cda_rail::StationList::export_stations(const std::string &path) const {
    /**
     * This method exports all stations to a file. The file is a json file with the following structure:
     * {"station1_name": [["edge1_0", "edge1_1"], ["edge2_0", "edge2_1"], ...], "station2_name": ...}
     * Hereby the names stored in the network reference are used for the edges.
     *
     * @param path The path to the file directory to export to.
     * @param network The network reference to use for the edge names.
     */

    std::filesystem::path p(path);
    if (!std::filesystem::exists(p)) {
        throw std::invalid_argument("Path does not exist.");
    }
    if (!std::filesystem::is_directory(p)) {
        throw std::invalid_argument("Path is not a directory.");
    }

    json j;
    for (const auto& station : stations) {
        std::vector<std::pair<std::string, std::string>> edges;
        for (const auto& track : station.tracks) {
            const auto& edge = network.get_edge(track);
            edges.emplace_back(network.get_vertex(edge.source).name, network.get_vertex(edge.target).name);
        }
        j[station.name] = edges;
    }

    std::ofstream file(p / "stations.json");
    file << j << std::endl;
}
