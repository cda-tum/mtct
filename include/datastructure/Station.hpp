#pragma once
#include "CustomExceptions.hpp"
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
  std::string name;
  // NOLINTNEXTLINE(readability-redundant-member-init)
  cda_rail::index_vector tracks = {};

  [[nodiscard]] std::vector<
      std::pair<size_t, std::vector<cda_rail::index_vector>>>
  get_stop_tracks(double tr_len, const Network& network,
                  const cda_rail::index_vector& edges_to_consider = {}) const;
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
  StationList(const std::filesystem::path& p, const Network& network);
  StationList(const std::string& path, const Network& network)
      : StationList(std::filesystem::path(path), network) {};
  StationList(const char* path, const Network& network)
      : StationList(std::filesystem::path(path), network) {};

  // Rule of 5
  StationList(const StationList& other)            = default;
  StationList(StationList&& other)                 = default;
  StationList& operator=(const StationList& other) = default;
  StationList& operator=(StationList&& other)      = default;
  ~StationList()                                   = default;

  [[nodiscard]] bool is_fully_in_station(const std::string&     station_name,
                                         cda_rail::index_vector edges) const;

  // Iterators (for range-based for loops) that do not allow modification of the
  // underlying data
  [[nodiscard]] auto begin() const { return stations.begin(); };
  [[nodiscard]] auto end() const { return stations.end(); };

  void add_station(const std::string& name) { stations[name] = Station{name}; };

  [[nodiscard]] bool has_station(const std::string& name) const {
    return stations.find(name) != stations.end();
  };
  [[nodiscard]] const Station& get_station(const std::string& name) const;

  [[nodiscard]] size_t size() const { return stations.size(); };
  [[nodiscard]] std::vector<std::string> get_station_names() const;

  void add_track_to_station(const std::string& name, size_t track);
  void add_track_to_station(const std::string& name, size_t track,
                            const Network& network) {
    if (!network.has_edge(track)) {
      throw exceptions::EdgeNotExistentException(track);
    }
    add_track_to_station(name, track);
  };
  void add_track_to_station(const std::string& name, size_t source,
                            size_t target, const Network& network) {
    add_track_to_station(name, network.get_edge_index(source, target), network);
  };
  void add_track_to_station(const std::string& name, const std::string& source,
                            const std::string& target, const Network& network) {
    add_track_to_station(name, network.get_edge_index(source, target), network);
  };

  void export_stations(const std::string& path, const Network& network) const;
  void export_stations(const char* path, const Network& network) const {
    export_stations(std::filesystem::path(path), network);
  };
  void export_stations(const std::filesystem::path& p,
                       const Network&               network) const;
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

  void update_after_discretization(
      const std::vector<std::pair<size_t, cda_rail::index_vector>>& new_edges);

  [[nodiscard]] std::vector<
      std::pair<size_t, std::vector<cda_rail::index_vector>>>
  get_stop_tracks(const std::string& name, double tr_len,
                  const Network&                network,
                  const cda_rail::index_vector& edges_to_consider = {}) const {
    return get_station(name).get_stop_tracks(tr_len, network,
                                             edges_to_consider);
  }
};
} // namespace cda_rail
