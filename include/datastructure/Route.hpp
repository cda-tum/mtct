#pragma once
#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Train.hpp"

#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

namespace cda_rail {
class Route {
private:
  std::vector<int> edges;

public:
  void push_back_edge(int edge_index, const cda_rail::Network& network);
  void push_back_edge(int source, int target,
                      const cda_rail::Network& network) {
    push_back_edge(network.get_edge_index(source, target), network);
  };
  void push_back_edge(const std::string& source, const std::string& target,
                      const cda_rail::Network& network) {
    push_back_edge(network.get_edge_index(source, target), network);
  };

  void push_front_edge(int edge_index, const cda_rail::Network& network);
  void push_front_edge(int source, int target,
                       const cda_rail::Network& network) {
    push_front_edge(network.get_edge_index(source, target), network);
  };
  void push_front_edge(const std::string& source, const std::string& target,
                       const cda_rail::Network& network) {
    push_front_edge(network.get_edge_index(source, target), network);
  };

  void remove_first_edge();
  void remove_last_edge();

  [[nodiscard]] double length(const cda_rail::Network& network) const;
  [[nodiscard]] std::pair<double, double>
  edge_pos(int edge, const cda_rail::Network& network) const;
  [[nodiscard]] std::pair<double, double>
  edge_pos(int source, int target, const cda_rail::Network& network) const {
    return edge_pos(network.get_edge_index(source, target), network);
  };
  [[nodiscard]] std::pair<double, double>
  edge_pos(const std::string& source, const std::string& target,
           const cda_rail::Network& network) const {
    return edge_pos(network.get_edge_index(source, target), network);
  };

  [[nodiscard]] std::pair<double, double>
  edge_pos(const std::vector<int>&  edges_to_consider,
           const cda_rail::Network& network) const;

  [[nodiscard]] int get_edge(int route_index) const;
  [[nodiscard]] const cda_rail::Edge&
  get_edge(int route_index, const cda_rail::Network& network) const;
  [[nodiscard]] int                     size() const { return edges.size(); };
  [[nodiscard]] bool                    empty() const { return edges.empty(); };
  [[nodiscard]] const std::vector<int>& get_edges() const { return edges; };

  [[nodiscard]] bool contains_edge(int edge_index) const {
    return std::find(edges.begin(), edges.end(), edge_index) != edges.end();
  };

  [[nodiscard]] bool check_consistency(const cda_rail::Network& network) const;

  void update_after_discretization(
      const std::vector<std::pair<int, std::vector<int>>>& new_edges);
};

class RouteMap {
private:
  std::unordered_map<std::string, Route> routes;

public:
  // Constructors
  RouteMap() = default;
  RouteMap(const std::filesystem::path& p, const cda_rail::Network& network);
  RouteMap(const std::string& path, const cda_rail::Network& network)
      : RouteMap(std::filesystem::path(path), network){};
  RouteMap(const char* path, const cda_rail::Network& network)
      : RouteMap(std::filesystem::path(path), network){};

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
  void add_empty_route(const std::string&         train_name,
                       const cda_rail::TrainList& trains);

  void push_back_edge(const std::string& train_name, int edge_index,
                      const cda_rail::Network& network);
  void push_back_edge(const std::string& train_name, int source, int target,
                      const cda_rail::Network& network);
  void push_back_edge(const std::string& train_name, const std::string& source,
                      const std::string&       target,
                      const cda_rail::Network& network);

  void push_front_edge(const std::string& train_name, int edge_index,
                       const cda_rail::Network& network);
  void push_front_edge(const std::string& train_name, int source, int target,
                       const cda_rail::Network& network);
  void push_front_edge(const std::string& train_name, const std::string& source,
                       const std::string&       target,
                       const cda_rail::Network& network);

  void remove_first_edge(const std::string& train_name);
  void remove_last_edge(const std::string& train_name);

  [[nodiscard]] bool has_route(const std::string& train_name) const {
    return routes.find(train_name) != routes.end();
  };
  [[nodiscard]] int          size() const { return routes.size(); };
  [[nodiscard]] bool         empty() const { return routes.empty(); };
  [[nodiscard]] const Route& get_route(const std::string& train_name) const;

  [[nodiscard]] double length(const std::string&       train_name,
                              const cda_rail::Network& network) const;
  [[nodiscard]] std::pair<double, double>
  edge_pos(const std::string& train_name, int edge,
           const cda_rail::Network& network) const {
    return get_route(train_name).edge_pos(edge, network);
  };
  [[nodiscard]] std::pair<double, double>
  edge_pos(const std::string& train_name, int source, int target,
           const cda_rail::Network& network) const {
    return get_route(train_name).edge_pos(source, target, network);
  };
  [[nodiscard]] std::pair<double, double>
  edge_pos(const std::string& train_name, const std::string& source,
           const std::string& target, const cda_rail::Network& network) const {
    return get_route(train_name).edge_pos(source, target, network);
  };
  [[nodiscard]] std::pair<double, double>
  edge_pos(const std::string& train_name, const std::vector<int>& edges,
           const cda_rail::Network& network) const {
    return get_route(train_name).edge_pos(edges, network);
  };

  [[nodiscard]] bool
  check_consistency(const cda_rail::TrainList& trains,
                    const cda_rail::Network&   network,
                    bool every_train_must_have_route = true) const;

  void export_routes(const std::filesystem::path& p,
                     const cda_rail::Network&     network) const;
  void export_routes(const std::string&       path,
                     const cda_rail::Network& network) const {
    export_routes(std::filesystem::path(path), network);
  };
  void export_routes(const char* path, const cda_rail::Network& network) const {
    export_routes(std::filesystem::path(path), network);
  };

  [[nodiscard]] static RouteMap
  import_routes(const std::filesystem::path& p,
                const cda_rail::Network&     network) {
    return {p, network};
  };
  [[nodiscard]] static RouteMap
  import_routes(const std::string& path, const cda_rail::Network& network) {
    return {path, network};
  };
  [[nodiscard]] static RouteMap
  import_routes(const char* path, const cda_rail::Network& network) {
    return {path, network};
  };

  void update_after_discretization(
      const std::vector<std::pair<int, std::vector<int>>>& new_edges);
};
} // namespace cda_rail
