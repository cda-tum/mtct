#include "datastructure/Station.hpp"

#include "Definitions.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "nlohmann/json.hpp"

#include <filesystem>
#include <fstream>
#include <string>
#include <unordered_map>

using json = nlohmann::json;

const cda_rail::Station&
cda_rail::StationList::get_station(const std::string& name) const {
  if (!has_station(name)) {
    throw std::invalid_argument("Station does not exist.");
  }
  return stations.at(name);
}

void cda_rail::StationList::add_track_to_station(
    const std::string& name, size_t track, const cda_rail::Network& network) {
  if (!has_station(name)) {
    throw std::invalid_argument("Station does not exist.");
  }
  if (!network.has_edge(track)) {
    throw std::invalid_argument("Track does not exist.");
  }
  // If stations.at(name).tracks already contains track, nothing happens.
  if (std::find(stations.at(name).tracks.begin(),
                stations.at(name).tracks.end(),
                track) != stations.at(name).tracks.end()) {
    return;
  }
  stations.at(name).tracks.emplace_back(track);
}

void cda_rail::StationList::export_stations(
    const std::string& path, const cda_rail::Network& network) const {
  /**
   * This method exports all stations to a file. The file is a json file with
   * the following structure:
   * {"station1_name": [["edge1_0", "edge1_1"], ["edge2_0", "edge2_1"], ...],
   * "station2_name": ...} Hereby the names stored in the network reference are
   * used for the edges.
   *
   * @param path The path to the file directory to export to.
   * @param network The network reference to use for the edge names.
   */

  std::filesystem::path const p(path);
  export_stations(p, network);
}

void cda_rail::StationList::export_stations(
    const std::filesystem::path& p, const cda_rail::Network& network) const {
  if (!cda_rail::is_directory_and_create(p)) {
    throw std::runtime_error("Could not create directory " + p.string());
  }

  json j;
  for (const auto& [name, station] : stations) {
    std::vector<std::pair<std::string, std::string>> edges;
    for (const auto& track : station.tracks) {
      const auto& edge = network.get_edge(track);
      edges.emplace_back(network.get_vertex(edge.source).name,
                         network.get_vertex(edge.target).name);
    }
    j[station.name] = edges;
  }

  std::ofstream file(p / "stations.json");
  file << j << std::endl;
}

std::vector<std::string> cda_rail::StationList::get_station_names() const {
  /**
   * This method returns a vector of all station names, i.e., the keys of
   * stations.
   *
   * @return A vector of all station names.
   */

  std::vector<std::string> names;
  names.reserve(stations.size());
  for (const auto& [name, station] : stations) {
    names.emplace_back(name);
  }
  return names;
}

cda_rail::StationList::StationList(const std::filesystem::path& p,
                                   const cda_rail::Network&     network) {
  /**
   * This method constructs the object and imports all stations from a file. The
   * file is a json file with the structure described in export_stations. Hereby
   * the names stored in the network reference are used for the edges.
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
  json          data = json::parse(file);

  for (const auto& [name, edges] : data.items()) {
    this->add_station(name);
    for (const auto& edge : edges) {
      this->add_track_to_station(name, edge[0].get<std::string>(),
                                 edge[1].get<std::string>(), network);
    }
  }
}

void cda_rail::StationList::update_after_discretization(
    const std::vector<std::pair<size_t, std::vector<size_t>>>& new_edges) {
  /**
   * This method updates the timetable after the discretization of the network
   * accordingly. For every pair (v, {v_1, ..., v_n}), v is replaced by v_1,
   * ..., v_n. Concretely, the following changes are made:
   * - For every station, the tracks are replaced by the new edges if
   * applicable.
   *
   * @param new_edges The new edges of the network as returned from
   * cda_rail::Network::discretize.
   */

  for (auto& [name, station] : stations) {
    auto&      tracks = station.tracks;
    const auto size   = tracks.size();
    for (size_t i = 0; i < size; ++i) {
      for (const auto& [track, new_tracks] : new_edges) {
        if (tracks[i] == track) {
          tracks[i] = new_tracks[0];
          for (size_t j = 1; j < new_tracks.size(); ++j) {
            tracks.emplace_back(new_tracks[j]);
          }
        }
      }
    }
  }
}
