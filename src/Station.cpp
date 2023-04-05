#include "Station.hpp"
#include "RailwayNetwork.hpp"
#include "Definitions.hpp"
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>
#include <unordered_map>
#include "nlohmann/json.hpp"

using json = nlohmann::json;

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

void cda_rail::StationList::add_track_to_station(int station_index, int track,
                                                 const cda_rail::Network &network) {
    if (!has_station(station_index)) {
        throw std::out_of_range("Station does not exist.");
    }
    if (!network.has_edge(track)) {
        throw std::out_of_range("Track does not exist.");
    }
    stations.at(station_index).tracks.insert(track);
}

void cda_rail::StationList::add_track_to_station(const std::string &name, int track,
                                                 const cda_rail::Network &network) {
    if (!has_station(name)) {
        throw std::out_of_range("Station does not exist.");
    }
    if (!network.has_edge(track)) {
        throw std::out_of_range("Track does not exist.");
    }
    add_track_to_station(get_station_index(name), track, network);
}

void
cda_rail::StationList::add_track_to_station(int station_index, int source, int target,
                                            const cda_rail::Network &network) {
    if (!has_station(station_index)) {
        throw std::out_of_range("Station does not exist.");
    }
    if (!network.has_edge(source, target)) {
        throw std::out_of_range("Track does not exist.");
    }
    add_track_to_station(station_index, network.get_edge_index(source, target), network);
}

void
cda_rail::StationList::add_track_to_station(const std::string &name, int source, int target,
                                            const cda_rail::Network &network) {
    if (!has_station(name)) {
        throw std::out_of_range("Station does not exist.");
    }
    if (!network.has_edge(source, target)) {
        throw std::out_of_range("Track does not exist.");
    }
    add_track_to_station(get_station_index(name), network.get_edge_index(source, target), network);
}

void cda_rail::StationList::add_track_to_station(int station_index, const std::string &source, const std::string &target,
                                                 const cda_rail::Network &network) {
    if (!has_station(station_index)) {
        throw std::out_of_range("Station does not exist.");
    }
    if (!network.has_edge(source, target)) {
        throw std::out_of_range("Track does not exist.");
    }
    add_track_to_station(station_index, network.get_edge_index(source, target), network);
}

void cda_rail::StationList::add_track_to_station(const std::string &name, const std::string &source, const std::string &target,
                                                 const cda_rail::Network &network) {
    if (!has_station(name)) {
        throw std::out_of_range("Station does not exist.");
    }
    if (!network.has_edge(source, target)) {
        throw std::out_of_range("Track does not exist.");
    }
    add_track_to_station(get_station_index(name), network.get_edge_index(source, target), network);
}

void cda_rail::StationList::export_stations(const std::string &path, const cda_rail::Network &network) const {
    /**
     * This method exports all stations to a file. The file is a json file with the following structure:
     * {"station1_name": [["edge1_0", "edge1_1"], ["edge2_0", "edge2_1"], ...], "station2_name": ...}
     * Hereby the names stored in the network reference are used for the edges.
     *
     * @param path The path to the file directory to export to.
     * @param network The network reference to use for the edge names.
     */

    std::filesystem::path p(path);
    export_stations(p, network);
}

void cda_rail::StationList::export_stations(const std::filesystem::path &p, const cda_rail::Network &network) const {
    if (!cda_rail::is_directory_and_create(p)) {
        throw std::runtime_error("Could not create directory " + p.string());
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

cda_rail::StationList cda_rail::StationList::import_stations(const std::string &path, const cda_rail::Network &network) {
    return import_stations(std::filesystem::path(path), network);
}

cda_rail::StationList cda_rail::StationList::import_stations(const std::filesystem::path &p, const cda_rail::Network &network) {
    /**
     * This method imports all stations from a file. The file is a json file with the structure described in export_stations.
     * Hereby the names stored in the network reference are used for the edges.
     *
     * @param path The path to the file directory to import from.
     * @param network The network reference to use for the edge names.
     */

    if (!std::filesystem::exists(p)) {
        throw std::invalid_argument("Path does not exist.");
    }
    if (!std::filesystem::is_directory(p)) {
        throw std::invalid_argument("Path is not a directory.");
    }

    std::ifstream file(p / "stations.json");
    json data = json::parse(file);

    StationList stations;
    for (const auto& [name, edges] : data.items()) {
        stations.add_station(name);
        for (const auto& edge : edges) {
            stations.add_track_to_station(name, edge[0].get<std::string>(), edge[1].get<std::string>(), network);
        }
    }

    return stations;
}

void cda_rail::StationList::export_stations(const char *path, const cda_rail::Network &network) const {
    export_stations(std::filesystem::path(path), network);
}

cda_rail::StationList cda_rail::StationList::import_stations(const char *path, const cda_rail::Network &network) {
    return import_stations(std::filesystem::path(path), network);
}

int cda_rail::StationList::size() const {
    return stations.size();
}
