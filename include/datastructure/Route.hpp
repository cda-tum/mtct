#pragma once
#include "Definitions.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Train.hpp"

#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace cda_rail {

struct ConflictPair {
  std::pair<double, double>  pos1;
  std::pair<double, double>  pos2;
  std::unordered_set<size_t> edges;
};

class Route {
  struct EdgePosition {
    double source{};
    double target{};
  };

private:
  cda_rail::index_vector m_edges{};

public:
  // -----------------------------
  // CONSTRUCTORS
  // -----------------------------
  // Only default constructors (empty)

  // -----------------------------
  // GETTER
  // -----------------------------
  [[nodiscard]] size_t size() const { return m_edges.size(); };
  [[nodiscard]] double length(const Network& network) const;
  [[nodiscard]] bool   empty() const { return m_edges.empty(); };

  [[nodiscard]] const cda_rail::index_vector& get_edges() const {
    return m_edges;
  };

  [[nodiscard]] EdgePosition edge_pos_on_route(Network::EdgeInput const& edge,
                                               const Network& network) const;
  [[nodiscard]] EdgePosition
  edge_set_pos_on_route(const cda_rail::index_vector& edges_to_consider,
                        const Network&                network) const;

  [[nodiscard]] std::optional<double>
  get_first_pos_on_edges(const cda_rail::index_vector& edge_indices,
                         const Network&                network) const;
  [[nodiscard]] std::optional<double>
  get_last_pos_on_edges(const cda_rail::index_vector& edge_indices,
                        const Network&                network) const;

  [[nodiscard]] size_t get_edge_id_at_pos(double         pos,
                                          const Network& network) const;

  [[nodiscard]] size_t      get_edge_id(size_t route_index) const;
  [[nodiscard]] const Edge& get_edge(size_t         route_index,
                                     const Network& network) const;

  [[nodiscard]] bool contains_edge(size_t const edgeIndex) const {
    return std::ranges::contains(m_edges, edgeIndex);
  };
  [[nodiscard]] bool contains_edge(Network::EdgeInput const& edge_input,
                                   const Network&            network) const {
    return contains_edge(edge_input.resolve(&network));
  };
  [[nodiscard]] bool
  contains_edge(std::optional<size_t> const edgeIndex) const {
    return edgeIndex.has_value() && contains_edge(edgeIndex.value());
  }

  // -----------------------------
  // EDITING FUNCTIONS
  // -----------------------------

  void push_back_edge(Network::EdgeInput const& new_edge,
                      const Network&            network);
  void push_front_edge(Network::EdgeInput const& new_edge,
                       const Network&            network);

  void remove_first_edge();
  void remove_last_edge();

  // -----------------------------
  // HELPER
  // -----------------------------
  [[nodiscard]] bool check_consistency(const Network& network) const;
  void               update_after_discretization(
      const std::vector<std::pair<size_t, cda_rail::index_vector>>& new_edges);
};

#if 0
// TODO: For testing purposes

class RouteMap {
private:
  std::unordered_map<std::string, Route> routes;

public:
  // Constructors
  RouteMap() = default;
  RouteMap(const std::filesystem::path& p, const Network& network);
  RouteMap(const std::string& path, const Network& network)
      : RouteMap(std::filesystem::path(path), network) {};
  RouteMap(const char* path, const Network& network)
      : RouteMap(std::filesystem::path(path), network) {};

  // Rule of 5
  RouteMap(const RouteMap& other)            = default;
  RouteMap(RouteMap&& other)                 = default;
  RouteMap& operator=(const RouteMap& other) = default;
  RouteMap& operator=(RouteMap&& other)      = default;
  ~RouteMap()                                = default;

  // Iterators (for range-based for loops) that do not allow modification of the
  // underlying data
  [[nodiscard]] auto begin() const { return routes.begin(); };
  [[nodiscard]] auto end() const { return routes.end(); };

  void add_empty_route(const std::string& train_name);
  void add_empty_route(const std::string& train_name, const TrainList& trains);

  void push_back_edge(const std::string& train_name, size_t edge_index,
                      const Network& network);
  void push_back_edge(const std::string& train_name, size_t source,
                      size_t target, const Network& network);
  void push_back_edge(const std::string& train_name, const std::string& source,
                      const std::string& target, const Network& network);

  void push_front_edge(const std::string& train_name, size_t edge_index,
                       const Network& network);
  void push_front_edge(const std::string& train_name, size_t source,
                       size_t target, const Network& network);
  void push_front_edge(const std::string& train_name, const std::string& source,
                       const std::string& target, const Network& network);

  void remove_first_edge(const std::string& train_name);
  void remove_last_edge(const std::string& train_name);

  void remove_route(const std::string& train_name);

  [[nodiscard]] bool has_route(const std::string& train_name) const {
    return routes.contains(train_name);
  };
  [[nodiscard]] size_t       size() const { return routes.size(); };
  [[nodiscard]] bool         empty() const { return routes.empty(); };
  [[nodiscard]] const Route& get_route(const std::string& train_name) const;

  [[nodiscard]] double length(const std::string& train_name,
                              const Network&     network) const;
  [[nodiscard]] std::pair<double, double>
  edge_pos(const std::string& train_name, size_t edge,
           const Network& network) const {
    return get_route(train_name).edge_pos(edge, network);
  };
  [[nodiscard]] std::pair<double, double>
  edge_pos(const std::string& train_name, size_t source, size_t target,
           const Network& network) const {
    return get_route(train_name).edge_pos(source, target, network);
  };
  [[nodiscard]] std::pair<double, double>
  edge_pos(const std::string& train_name, const std::string& source,
           const std::string& target, const Network& network) const {
    return get_route(train_name).edge_pos(source, target, network);
  };
  [[nodiscard]] std::pair<double, double>
  edge_pos(const std::string& train_name, const cda_rail::index_vector& edges,
           const Network& network) const {
    return get_route(train_name).edge_pos(edges, network);
  };

  // Overlap functions
  [[nodiscard]] std::vector<ConflictPair>
  get_parallel_overlaps(const std::string& train1, const std::string& train2,
                        const Network& network) const;
  [[nodiscard]] std::vector<ConflictPair>
  get_ttd_overlaps(const std::string& train1, const std::string& train2,
                   const Network& network) const;
  [[nodiscard]] std::vector<ConflictPair>
  get_reverse_overlaps(const std::string& train1, const std::string& train2,
                       const Network& network) const;
  [[nodiscard]] std::vector<ConflictPair>
  get_crossing_overlaps(const std::string& train1, const std::string& train2,
                        const Network& network) const;

  [[nodiscard]] bool
  check_consistency(const TrainList& trains, const Network& network,
                    bool every_train_must_have_route = true) const;

  void export_routes(const std::filesystem::path& p,
                     const Network&               network) const;
  void export_routes(const std::string& path, const Network& network) const {
    export_routes(std::filesystem::path(path), network);
  };
  void export_routes(const char* path, const Network& network) const {
    export_routes(std::filesystem::path(path), network);
  };

  [[nodiscard]] static RouteMap import_routes(const std::filesystem::path& p,
                                              const Network& network) {
    return {p, network};
  };
  [[nodiscard]] static RouteMap import_routes(const std::string& path,
                                              const Network&     network) {
    return {path, network};
  };
  [[nodiscard]] static RouteMap import_routes(const char*    path,
                                              const Network& network) {
    return {path, network};
  };

  void update_after_discretization(
      const std::vector<std::pair<size_t, cda_rail::index_vector>>& new_edges);
};

#endif

} // namespace cda_rail
