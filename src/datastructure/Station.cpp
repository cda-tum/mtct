#include "datastructure/Station.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "Definitions.hpp"
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>
#include <unordered_map>
#include "nlohmann/json.hpp"

using json = nlohmann::json;

const cda_rail::Station &cda_rail::StationList::get_station(const std::string &name) const {
    if (!has_station(name)) {
        throw std::out_of_range("Station does not exist.");
    }
    return stations.at(name);
}

void cda_rail::StationList::add_track_to_station(const std::string &name, int track,
                                                 const cda_rail::Network &network) {
    if (!has_station(name)) {
        throw std::out_of_range("Station does not exist.");
    }
    if (!network.has_edge(track)) {
        throw std::out_of_range("Track does not exist.");
    }
    stations.at(name).tracks.insert(track);
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
    for (const auto& [name, station] : stations) {
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

std::vector<std::string> cda_rail::StationList::get_station_names() const {
    /**
     * This method returns a vector of all station names, i.e., the keys of stations.
     *
     * @return A vector of all station names.
     */

    std::vector<std::string> names;
    for (const auto& [name, station] : stations) {
        names.emplace_back(name);
    }
    return names;
}

cda_rail::StationList::StationList(const std::filesystem::path &p, const cda_rail::Network &network) {
    /**
     * This method constructs the object and imports all stations from a file. The file is a json file with the structure described in export_stations.
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

    for (const auto& [name, edges] : data.items()) {
        this->add_station(name);
        for (const auto& edge : edges) {
            this->add_track_to_station(name, edge[0].get<std::string>(), edge[1].get<std::string>(), network);
        }
    }
}
