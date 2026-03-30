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
struct Station {
  /**
   * Station object
   * @param name Name of the station
   * @param tracks Unordered set of edges that define the station.
   */
  std::string         name{};
  cda_rail::index_set tracks{};

  /**
   * This method returns the tracks of the station on which a train of length
   * tr_len can stop at the target vertex.
   * * @param tr_len The length of the train.
   * * @param network The network to which the station belongs.
   * * @param edges_to_consider The edges to consider. Default: {}, then all
   * edges
   *
   * @return A vector of pairs:
   * - The first element of the pair is the index of a possible stop edge
   * - The second element lists all possible stop paths ending in that edge
   * Note: The train has to use one of the stop paths if it stops at the (end of
   * the) edge
   */
  [[nodiscard]] std::vector<
      std::pair<size_t, std::vector<cda_rail::index_vector>>>
  get_stop_tracks(double tr_len, Network const& network,
                  cda_rail::index_set const& edges_to_consider = {}) const;

  [[nodiscard]] bool
  is_fully_in_station(cda_rail::index_set const& edges) const;
};

class StationList {
  /**
   * StationList class
   */
private:
  // DeepConstSharedPtr propagates constness (without using experimental)
  std::unordered_map<std::string, DeepConstSharedPtr<Station>> stations;

public:
  // Constructors
  StationList() = default;
  StationList(std::filesystem::path const& p, Network const& network);
  StationList(std::string const& path, Network const& network)
      : StationList(std::filesystem::path(path), network) {};
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
  [[nodiscard]] bool has_station(const std::string& name) const {
    return stations.contains(name);
  };
  [[nodiscard]] Station const& get_station(const std::string& name) const;
  [[nodiscard]] std::unordered_set<std::string> get_station_names() const;

  [[nodiscard]] bool
  is_fully_in_station(std::string const&         station_name,
                      cda_rail::index_set const& edges) const;

  /*
   * SETTER / EDITING
   */

  void add_empty_station(std::string const& name);

  void add_track_to_station(const std::string& name, size_t track);
  void add_track_to_station(const std::string& name, size_t const track,
                            const Network& network) {
    if (!network.has_edge(track)) {
      throw exceptions::EdgeNotExistentException(track);
    }
    add_track_to_station(name, track);
  };
  void add_track_to_station(const std::string& name, size_t const source,
                            size_t const target, const Network& network) {
    add_track_to_station(name, network.get_edge_index(source, target), network);
  };
  void add_track_to_station(const std::string& name, const std::string& source,
                            const std::string& target, const Network& network) {
    add_track_to_station(name, network.get_edge_index(source, target), network);
  };

  /*
   * EXPORT/IMPORT
   */

  void export_stations(const std::filesystem::path& p,
                       const Network&               network) const;
  void export_stations(std::string const& path, const Network& network) const {
    std::filesystem::path const p(path);
    export_stations(p, network);
  };
  void export_stations(char const* const path, const Network& network) const {
    export_stations(std::filesystem::path(path), network);
  };
  [[nodiscard]] static StationList import_stations(const std::string& path,
                                                   const Network&     network) {
    return {path, network};
  };
  [[nodiscard]] static StationList import_stations(const char*    path,
                                                   const Network& network) {
    return {path, network};
  };
  [[nodiscard]] static StationList
  import_stations(const std::filesystem::path& p, const Network& network) {
    return {p, network};
  };

  /*
   * HELPER
   */

  void update_after_discretization(
      const std::vector<std::pair<size_t, cda_rail::index_set>>& new_edges);

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
