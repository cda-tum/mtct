#pragma once
#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Train.hpp"

#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace cda_rail {
class Route {
private:
  std::vector<size_t> edges;

public:
  void push_back_edge(size_t edge_index, const Network& network);
  void push_back_edge(size_t source, size_t target, const Network& network) {
    push_back_edge(network.get_edge_index(source, target), network);
  };
  void push_back_edge(const std::string& source, const std::string& target,
                      const Network& network) {
    push_back_edge(network.get_edge_index(source, target), network);
  };

  void push_front_edge(size_t edge_index, const Network& network);
  void push_front_edge(size_t source, size_t target, const Network& network) {
    push_front_edge(network.get_edge_index(source, target), network);
  };
  void push_front_edge(const std::string& source, const std::string& target,
                       const Network& network) {
    push_front_edge(network.get_edge_index(source, target), network);
  };

  void remove_first_edge();
  void remove_last_edge();

  [[nodiscard]] double length(const Network& network) const;
  [[nodiscard]] std::pair<double, double>
  edge_pos(size_t edge, const Network& network) const;
  [[nodiscard]] std::pair<double, double>
  edge_pos(size_t source, size_t target, const Network& network) const {
    return edge_pos(network.get_edge_index(source, target), network);
  };
  [[nodiscard]] std::pair<double, double>
  edge_pos(const std::string& source, const std::string& target,
           const Network& network) const {
    return edge_pos(network.get_edge_index(source, target), network);
  };

  [[nodiscard]] std::pair<double, double>
  edge_pos(const std::vector<size_t>& edges_to_consider,
           const Network&             network) const;

  [[nodiscard]] size_t get_edge_at_pos(double         pos,
                                       const Network& network) const;

  [[nodiscard]] size_t      get_edge(size_t route_index) const;
  [[nodiscard]] const Edge& get_edge(size_t         route_index,
                                     const Network& network) const;
  [[nodiscard]] size_t      size() const { return edges.size(); };
  [[nodiscard]] bool        empty() const { return edges.empty(); };
  [[nodiscard]] const std::vector<size_t>& get_edges() const { return edges; };

  [[nodiscard]] bool contains_edge(size_t edge_index) const {
    return std::ranges::contains(edges, edge_index);
  };
  [[nodiscard]] bool contains_edge(std::optional<size_t> edge_index) const {
    return edge_index.has_value() && contains_edge(edge_index.value());
  }

  [[nodiscard]] bool check_consistency(const Network& network) const;

  void update_after_discretization(
      const std::vector<std::pair<size_t, std::vector<size_t>>>& new_edges);
};

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
    return routes.find(train_name) != routes.end();
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
  edge_pos(const std::string& train_name, const std::vector<size_t>& edges,
           const Network& network) const {
    return get_route(train_name).edge_pos(edges, network);
  };

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
      const std::vector<std::pair<size_t, std::vector<size_t>>>& new_edges);
};
} // namespace cda_rail
