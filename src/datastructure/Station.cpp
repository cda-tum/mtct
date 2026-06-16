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
#include <memory>
#include <ranges>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

using json = nlohmann::json;

/**
 * @brief Finds station tracks reachable via paths of specified length.
 *
 * @param trLen The length of paths to consider.
 * @param network The network instance for path queries.
 * @param edges_to_consider If non-empty, restricts evaluation to edges present
 *                           in both this set and the station's tracks;
 *                           otherwise, evaluates all station tracks.
 * @return A vector of pairs, each containing a track index and the paths of
 *         length `trLen` ending at that track. Tracks without such paths are
 *         excluded.
 */

std::vector<std::pair<size_t, std::vector<cda_rail::index_vector>>>
cda_rail::Station::get_stop_tracks(
    double const trLen, cda_rail::Network const& network,
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
        e, trLen, {}, station_tracks_to_consider);
    if (!stop_paths.empty()) {
      ret_val.emplace_back(e, stop_paths);
    }
  }

  return ret_val;
}

/**
 * @brief Determines if all provided edges are tracks of this station.
 *
 * @param edges The edges to verify.
 * @return `true` if all edges are contained in this station's tracks, `false`
 * otherwise.
 */
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

/**
 * @brief Initializes the station list by loading stations and tracks from a
 * JSON file.
 *
 * @param p Directory path containing `stations.json`.
 * @param network Network instance to resolve vertex names in the JSON to track
 * edges.
 * @throw exceptions::ImportException if the path does not exist or is not a
 * directory.
 */

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
      this->add_track_to_station(
          name, {edge.at(0).get<std::string>(), edge.at(1).get<std::string>()},
          network);
    }
  }
}

/**
 * @brief Obtains a mutable shared pointer to a station by name.
 *
 * @return std::shared_ptr<cda_rail::Station> A shared pointer to the Station
 * with the given name.
 * @throws exceptions::StationNotExistentException if the station does not
 * exist.
 */

std::shared_ptr<cda_rail::Station>
cda_rail::StationList::get_station_ptr(const std::string& name) {
  if (!has_station(name)) {
    throw exceptions::StationNotExistentException(name);
  }
  return stations.at(name);
}

/**
 * @brief Retrieves a station by name.
 *
 * @param name The name of the station to retrieve.
 * @return const cda_rail::Station& A const reference to the station.
 * @throws exceptions::StationNotExistentException If no station with the given
 * name exists.
 */
const cda_rail::Station&
cda_rail::StationList::get_station(const std::string& name) const {
  if (!has_station(name)) {
    throw exceptions::StationNotExistentException(name);
  }
  return *stations.at(name);
}

/**
 * @brief Retrieves all station names in the station list.
 *
 * @return std::unordered_set<std::string> An unordered set containing all
 * station names.
 */
std::unordered_set<std::string>
cda_rail::StationList::get_station_names() const {
  std::unordered_set<std::string> names;
  names.reserve(stations.size());
  for (const auto& name : stations | std::views::keys) {
    names.insert(name);
  }
  return names;
}

/**
 * @brief Adds a new empty station with the given name.
 *
 * @param name The name of the station to add.
 *
 * @throws ConsistencyException If a station with the given name already exists.
 */

void cda_rail::StationList::add_empty_station(std::string const& name) {
  if (has_station(name)) {
    throw exceptions::ConsistencyException("Station with name '" + name +
                                           "' already exists.");
  }
  stations.emplace(name, std::make_shared<Station>(Station{.name = name}));
};

/**
 * @brief Adds a track to a station.
 *
 * @throws exceptions::StationNotExistentException if the station does not
 * exist.
 */
void cda_rail::StationList::add_track_to_station(const std::string& name,
                                                 size_t const       track) {
  // If stations.at(name).tracks already contains track, nothing happens.
  // get_station throws error if the station name does not exist
  if (get_station(name).tracks.contains(track)) {
    return;
  }
  stations.at(name)->tracks.insert(track);
}

/**
 * @brief Serializes stations to a JSON file.
 *
 * Writes each station's tracks to a JSON object as pairs of vertex names
 * representing edges. The JSON is written to `p / "stations.json"`.
 *
 * @param p Directory path where the stations.json file will be created.
 * @param network Network used to resolve track indices to vertex names.
 *
 * @throws exceptions::ExportException if the directory cannot be created.
 */

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

namespace {
// refactoring in anonymous namespace to prevent duplication in code
template <typename StationContainer, typename ReplacementContainer>
/**
 * @brief Updates tracks in all stations to reflect edge discretization.
 *
 * For each station, replaces any tracks that appear in @p new_edges with their
 * corresponding replacement tracks. Tracks not affected by discretization
 * remain unchanged.
 *
 * @param stations Container of station shared pointers.
 * @param new_edges Vector of track index replacements, where each pair maps an
 * old track to its replacement tracks.
 */
void update_after_discretization_impl(
    StationContainer&                                           stations,
    const std::vector<std::pair<size_t, ReplacementContainer>>& new_edges) {
  for (auto& station : stations | std::views::values) {
    for (const auto& [track, new_tracks] : new_edges) {
      if (station->tracks.contains(track)) {
        station->tracks.erase(track);
        station->tracks.insert(new_tracks.begin(), new_tracks.end());
      }
    }
  }
}
} // namespace

void cda_rail::StationList::update_after_discretization(
    const std::vector<std::pair<size_t, cda_rail::index_set>>& new_edges) {
  update_after_discretization_impl(stations, new_edges);
}

/**
 * @brief Updates each station's tracks after discretization.
 *
 * For each station, removes tracks that have been replaced during
 * discretization and inserts their replacement tracks.
 *
 * @param new_edges Mapping from old track indices to their new replacement
 * tracks.
 */
void cda_rail::StationList::update_after_discretization(
    const std::vector<std::pair<size_t, cda_rail::index_vector>>& new_edges) {
  update_after_discretization_impl(stations, new_edges);
}

/**
 * @brief Checks whether all edges are contained in a specified station.
 *
 * @param station_name Name of the station to check.
 * @param edges Set of edge indices.
 * @return `true` if all edges are in the station, `false` otherwise.
 * @throws StationNotExistentException if the station does not exist.
 */
bool cda_rail::StationList::is_fully_in_station(
    std::string const& station_name, cda_rail::index_set const& edges) const {
  if (!has_station(station_name)) {
    throw exceptions::StationNotExistentException(station_name);
  }

  return get_station(station_name).is_fully_in_station(edges);
}
