#pragma once
#include "CustomExceptions.hpp"
#include "DeepConstSharedPtr.hpp"
#include "Definitions.hpp"
#include "datastructure/RailwayNetwork.hpp"

#include <cstddef>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace cda_rail {
/**
 * @brief Represents a station as a named set of network tracks (edge indices).
 *
 * A station is identified by its `name` and contains all tracks on which trains
 * are considered to be within that station.
 */
struct Station {
  /**
   * @brief Station identifier.
   */
  std::string name{};

  /**
   * @brief Set of track indices (network edges) that belong to this station.
   */
  cda_rail::index_set tracks{};

  /**
   * @brief Computes station tracks that can host a stop for a given train
   * length.
   *
   * For each eligible stop edge, the method returns all valid in-station paths
   * ending in that edge whose total length can accommodate `tr_len`.
   *
   * @param tr_len Train length.
   * @param network Network containing the station edges.
   * @param edges_to_consider Optional candidate edges. If empty, all station
   * edges are considered.
   * @return Vector of pairs `(stop_edge, stop_paths)`, where `stop_paths`
   * contains all valid paths ending in `stop_edge`.
   */
  [[nodiscard]] std::vector<
      std::pair<size_t, std::vector<cda_rail::index_vector>>>
  get_stop_tracks(double tr_len, Network const& network,
                  cda_rail::index_set const& edges_to_consider = {}) const;

  /**
   * @brief Checks whether all provided edges belong to this station.
   *
   * @param edges Edges to validate.
   * @return `true` if every edge in `edges` is part of `tracks`; otherwise
   * `false`.
   */
  [[nodiscard]] bool
  is_fully_in_station(cda_rail::index_set const& edges) const;
};

/**
 * @brief Container and persistence API for named stations.
 *
 * `StationList` owns all stations and provides lookup, mutation,
 * import/export, and helper operations.
 */
class StationList {
private:
  // DeepConstSharedPtr propagates constness (without using experimental)
  std::unordered_map<std::string, DeepConstSharedPtr<Station>> stations;

public:
  // Constructors
  /**
   * @brief Creates an empty station list.
   */
  StationList() = default;

  /**
   * @brief Loads stations from `stations.json` in the given directory.
   *
   * @param p Directory containing `stations.json`.
   * @param network Network used to resolve edge endpoints from stored vertex
   * names.
   * @throws exceptions::ImportException If `p` does not exist or is not a
   * directory.
   */
  StationList(std::filesystem::path const& p, Network const& network);

  /**
   * @brief Convenience overload forwarding to the filesystem path constructor.
   *
   * @param path Directory containing `stations.json`.
   * @param network Network used to resolve edge endpoints.
   */
  StationList(std::string const& path, Network const& network)
      : StationList(std::filesystem::path(path), network) {};

  /**
   * @brief Convenience overload forwarding to the filesystem path constructor.
   *
   * @param path Directory containing `stations.json`.
   * @param network Network used to resolve edge endpoints.
   */
  StationList(char const* const path, Network const& network)
      : StationList(std::filesystem::path(path), network) {};

  // Rule of 0 suffices

  // Iterators (for range-based for loops) that do not allow modification of the
  // underlying data
  [[nodiscard]] constexpr auto cbegin() const { return stations.cbegin(); };
  [[nodiscard]] constexpr auto cend() const { return stations.cend(); };
  [[nodiscard]] size_t         size() const { return stations.size(); };

  /*
   * GETTER
   */
  /**
   * @brief Checks whether a station with the given name exists.
   *
   * @param name Station name.
   * @return `true` if present; otherwise `false`.
   */
  [[nodiscard]] bool has_station(const std::string& name) const {
    return stations.contains(name);
  };

  /**
   * @brief Retrieves a station by name.
   *
   * @param name Station name.
   * @return Immutable reference to the requested station.
   * @throws exceptions::StationNotExistentException If no station with `name`
   * exists.
   */
  [[nodiscard]] Station const& get_station(const std::string& name) const;

  /**
   * @brief Returns the set of all station names.
   *
   * @return Unordered set containing each station name exactly once.
   */
  [[nodiscard]] std::unordered_set<std::string> get_station_names() const;

  /**
   * @brief Checks whether all given edges belong to a named station.
   *
   * @param station_name Station to check.
   * @param edges Edge indices to validate.
   * @return `true` if all edges are part of the station; otherwise `false`.
   * @throws exceptions::StationNotExistentException If `station_name` is
   * unknown.
   */
  [[nodiscard]] bool
  is_fully_in_station(std::string const&         station_name,
                      cda_rail::index_set const& edges) const;

  /*
   * SETTER / EDITING
   */

  /**
   * @brief Adds a new station without tracks.
   *
   * @param name Name of the station to create.
   * @throws exceptions::ConsistencyException If a station with `name` already
   * exists.
   */
  void add_empty_station(std::string const& name);

  /**
   * @brief Adds a track index to an existing station.
   *
   * If the track already belongs to the station, the call has no effect.
   *
   * @param name Station name.
   * @param track Track edge index.
   * @throws exceptions::StationNotExistentException If `name` is unknown.
   */
  void add_track_to_station(const std::string& name, size_t track);

  /**
   * @brief Adds a track index to a station after validating it in the network.
   *
   * @param name Station name.
   * @param track Track edge index.
   * @param network Network used for edge existence validation.
   * @throws exceptions::EdgeNotExistentException If `track` is not in
   * `network`.
   * @throws exceptions::StationNotExistentException If `name` is unknown.
   */
  void add_track_to_station(const std::string& name, size_t const track,
                            const Network& network) {
    if (!network.has_edge(track)) {
      throw exceptions::EdgeNotExistentException(track);
    }
    add_track_to_station(name, track);
  };
  /**
   * @brief Adds a track specified by source and target vertex indices.
   *
   * @param name Station name.
   * @param source Source vertex index.
   * @param target Target vertex index.
   * @param network Network used to resolve the edge index.
   */
  void add_track_to_station(const std::string& name, size_t const source,
                            size_t const target, const Network& network) {
    add_track_to_station(name, network.get_edge_index(source, target), network);
  };

  /**
   * @brief Adds a track specified by source and target vertex names.
   *
   * @param name Station name.
   * @param source Source vertex name.
   * @param target Target vertex name.
   * @param network Network used to resolve the edge index.
   */
  void add_track_to_station(const std::string& name, const std::string& source,
                            const std::string& target, const Network& network) {
    add_track_to_station(name, network.get_edge_index(source, target), network);
  };

  /*
   * EXPORT/IMPORT
   */

  /**
   * @brief Exports all stations to `stations.json` in the target directory.
   *
   * Edge endpoints are written using vertex names from `network`.
   *
   * @param p Target directory.
   * @param network Network used to resolve edge endpoint names.
   * @throws exceptions::ExportException If the target directory cannot be
   * created.
   */
  void export_stations(const std::filesystem::path& p,
                       const Network&               network) const;

  /**
   * @brief Convenience overload forwarding to the filesystem path export.
   *
   * @param path Target directory.
   * @param network Network used to resolve edge endpoint names.
   */
  void export_stations(std::string const& path, const Network& network) const {
    std::filesystem::path const p(path);
    export_stations(p, network);
  };

  /**
   * @brief Convenience overload forwarding to the filesystem path export.
   *
   * @param path Target directory.
   * @param network Network used to resolve edge endpoint names.
   */
  void export_stations(char const* const path, const Network& network) const {
    export_stations(std::filesystem::path(path), network);
  };

  /**
   * @brief Imports stations from a directory path string.
   *
   * @param path Directory containing `stations.json`.
   * @param network Network used to resolve edge endpoints.
   * @return Imported station list.
   */
  [[nodiscard]] static StationList import_stations(const std::string& path,
                                                   const Network&     network) {
    return {path, network};
  };

  /**
   * @brief Imports stations from a C-string path.
   *
   * @param path Directory containing `stations.json`.
   * @param network Network used to resolve edge endpoints.
   * @return Imported station list.
   */
  [[nodiscard]] static StationList import_stations(const char*    path,
                                                   const Network& network) {
    return {path, network};
  };

  /**
   * @brief Imports stations from a filesystem path.
   *
   * @param p Directory containing `stations.json`.
   * @param network Network used to resolve edge endpoints.
   * @return Imported station list.
   */
  [[nodiscard]] static StationList
  import_stations(const std::filesystem::path& p, const Network& network) {
    return {p, network};
  };

  /*
   * HELPER
   */

  /**
   * @brief Updates station tracks after network discretization.
   *
   * For each pair `(old_edge, replacement_edges)`, every occurrence of
   * `old_edge` in a station is replaced by all edges in `replacement_edges`.
   *
   * @param new_edges Mapping from original edge index to replacement edge set.
   */
  void update_after_discretization(
      const std::vector<std::pair<size_t, cda_rail::index_set>>& new_edges);

  /**
   * @brief Computes possible stop tracks for a named station.
   *
   * This forwards to `Station::get_stop_tracks` of the resolved station.
   *
   * @param name Station name.
   * @param tr_len Train length.
   * @param network Network containing the station edges.
   * @param edges_to_consider Optional candidate edges. If empty, all station
   * edges are considered.
   * @return Vector of pairs `(stop_edge, stop_paths)` as returned by
   * `Station::get_stop_tracks`.
   * @throws exceptions::StationNotExistentException If `name` is unknown.
   */
  [[nodiscard]] std::vector<
      std::pair<size_t, std::vector<cda_rail::index_vector>>>
  get_stop_tracks(const std::string& name, double tr_len,
                  const Network&             network,
                  const cda_rail::index_set& edges_to_consider = {}) const {
    return get_station(name).get_stop_tracks(tr_len, network,
                                             edges_to_consider);
  }
};
} // namespace cda_rail
