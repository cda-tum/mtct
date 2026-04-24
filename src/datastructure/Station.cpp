#include "datastructure/Station.hpp"

#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "GeneralHelper.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "nlohmann/json.hpp"
#include "nlohmann/json_fwd.hpp"

#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

using json = nlohmann::json;

/*
 * STATION
 */

std::vector<std::pair<size_t, std::vector<cda_rail::index_vector>>>
cda_rail::Station::get_stop_tracks(
    double const tr_len, cda_rail::Network const& network,
    cda_rail::index_set const& edges_to_consider) const {
  auto station_tracks_to_consider =
      edges_to_consider.empty() ? tracks : cda_rail::index_set{};
  for (auto const& tmp_e : edges_to_consider) {
    if (tracks.contains(tmp_e)) {
      station_tracks_to_consider.insert(tmp_e);
    }
  }

  std::vector<std::pair<size_t, std::vector<cda_rail::index_vector>>> ret_val;
  for (const auto& e : station_tracks_to_consider) {
    const auto stop_paths = network.all_paths_of_length_ending_in_edge(
        e, tr_len, {}, station_tracks_to_consider);
    if (!stop_paths.empty()) {
      ret_val.emplace_back(e, stop_paths);
    }
  }

  return ret_val;
}

bool cda_rail::Station::is_fully_in_station(
    cda_rail::index_set const& edges) const {
  auto const& station_tracks =
      tracks; // needed because struct element cannot be captured directly

  return std::ranges::all_of(edges, [&station_tracks](size_t edge) {
    return std::ranges::contains(station_tracks, edge);
  });
}

/*
 * STATION LIST
 */

// CONSTRUCTOR

cda_rail::StationList::StationList(std::filesystem::path const& p,
                                   Network const&               network) {
  if (!std::filesystem::exists(p)) {
    throw exceptions::ImportException("Path does not exist.");
  }
  if (!std::filesystem::is_directory(p)) {
    throw exceptions::ImportException("Path is not a directory.");
  }

  std::ifstream file(p / "stations.json");

  for (json data = json::parse(file);
       const auto& [name, edges] : data.items()) {
    this->add_empty_station(name);
    for (const auto& edge : edges) {
      this->add_track_to_station(name, edge.at(0).get<std::string>(),
                                 edge.at(1).get<std::string>(), network);
    }
  }
}

// GETTER

std::shared_ptr<cda_rail::Station>
cda_rail::StationList::get_station_ptr(const std::string& name) {
  if (!has_station(name)) {
    throw exceptions::StationNotExistentException(name);
  }
  return stations.at(name);
}

const cda_rail::Station&
cda_rail::StationList::get_station(const std::string& name) const {
  if (!has_station(name)) {
    throw exceptions::StationNotExistentException(name);
  }
  return *stations.at(name);
}

std::unordered_set<std::string>
cda_rail::StationList::get_station_names() const {
  std::unordered_set<std::string> names;
  names.reserve(stations.size());
  for (const auto& name : stations | std::views::keys) {
    names.insert(name);
  }
  return names;
}

// EDITING

void cda_rail::StationList::add_empty_station(std::string const& name) {
  if (has_station(name)) {
    throw exceptions::ConsistencyException("Station with name '" + name +
                                           "' already exists.");
  }
  stations.emplace(name, std::make_shared<Station>(Station{.name = name}));
};

void cda_rail::StationList::add_track_to_station(const std::string& name,
                                                 size_t const       track) {
  // If stations.at(name).tracks already contains track, nothing happens.
  // get_station throws error if the station name does not exist
  if (get_station(name).tracks.contains(track)) {
    return;
  }
  stations.at(name)->tracks.insert(track);
}

// EXPORT

void cda_rail::StationList::export_stations(const std::filesystem::path& p,
                                            const Network& network) const {
  if (!is_directory_and_create(p)) {
    throw exceptions::ExportException("Could not create directory " +
                                      p.string());
  }

  json j;
  for (const auto& station : stations | std::views::values) {
    std::vector<std::pair<std::string, std::string>> edges;
    for (const auto& track : station->tracks) {
      const auto& edge = network.get_edge(track);
      edges.emplace_back(network.get_vertex(edge.source).name,
                         network.get_vertex(edge.target).name);
    }
    j[station->name] = edges;
  }

  std::ofstream file(p / "stations.json");
  file << j << '\n';
}

// HELPER

void cda_rail::StationList::update_after_discretization(
    const std::vector<std::pair<size_t, cda_rail::index_set>>& new_edges) {
  for (auto& station : stations | std::views::values) {
    auto& tracks = station->tracks;
    for (const auto& [track, new_tracks] : new_edges) {
      if (tracks.contains(track)) {
        tracks.erase(track);
        tracks.insert(new_tracks.begin(), new_tracks.end());
      }
    }
  }
}

bool cda_rail::StationList::is_fully_in_station(
    std::string const& station_name, cda_rail::index_set const& edges) const {
  if (!has_station(station_name)) {
    throw exceptions::StationNotExistentException(station_name);
  }

  return get_station(station_name).is_fully_in_station(edges);
}
