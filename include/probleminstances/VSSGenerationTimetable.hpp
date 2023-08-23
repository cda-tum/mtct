#pragma once
#include "VSSModel.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Route.hpp"
#include "datastructure/Timetable.hpp"

namespace cda_rail::instances {
class VSSGenerationTimetable {
private:
  Network   network;
  Timetable timetable;
  RouteMap  routes;

public:
  // Constructors
  VSSGenerationTimetable() = default;
  explicit VSSGenerationTimetable(const std::filesystem::path& p,
                                  bool every_train_must_have_route = true);
  explicit VSSGenerationTimetable(const std::string& path,
                                  bool every_train_must_have_route = true)
      : VSSGenerationTimetable(std::filesystem::path(path),
                               every_train_must_have_route){};
  explicit VSSGenerationTimetable(const char* path,
                                  bool every_train_must_have_route = true)
      : VSSGenerationTimetable(std::filesystem::path(path),
                               every_train_must_have_route){};

  // Rule of 5
  VSSGenerationTimetable(const VSSGenerationTimetable& other) = default;
  VSSGenerationTimetable(VSSGenerationTimetable&& other)      = default;
  VSSGenerationTimetable&
  operator=(const VSSGenerationTimetable& other)                    = default;
  VSSGenerationTimetable& operator=(VSSGenerationTimetable&& other) = default;
  ~VSSGenerationTimetable()                                         = default;

  // Network functions, i.e., network is accessible via n() as a reference
  [[nodiscard]] Network&       n() { return network; };
  [[nodiscard]] const Network& const_n() const { return network; };

  // Timetable functions
  size_t add_train(const std::string& name, int length, double max_speed,
                   double acceleration, double deceleration, int t_0,
                   double v_0, size_t entry, int t_n, double v_n, size_t exit) {
    return timetable.add_train(name, length, max_speed, acceleration,
                               deceleration, t_0, v_0, entry, t_n, v_n, exit,
                               network);
  };
  size_t add_train(const std::string& name, int length, double max_speed,
                   double acceleration, double deceleration, int t_0,
                   double v_0, const std::string& entry, int t_n, double v_n,
                   const std::string& exit) {
    return timetable.add_train(name, length, max_speed, acceleration,
                               deceleration, t_0, v_0, entry, t_n, v_n, exit,
                               network);
  };

  void add_station(const std::string& name) { timetable.add_station(name); };

  void add_track_to_station(const std::string& name, size_t track) {
    timetable.add_track_to_station(name, track, network);
  };
  void add_track_to_station(const std::string& name, size_t source,
                            size_t target) {
    timetable.add_track_to_station(name, source, target, network);
  };
  void add_track_to_station(const std::string& name, const std::string& source,
                            const std::string& target) {
    timetable.add_track_to_station(name, source, target, network);
  };

  void add_stop(size_t train_index, const std::string& station_name, int begin,
                int end, bool sort = true) {
    timetable.add_stop(train_index, station_name, begin, end, sort);
  };
  void add_stop(const std::string& train_name, const std::string& station_name,
                int begin, int end, bool sort = true) {
    timetable.add_stop(train_name, station_name, begin, end, sort);
  };

  [[nodiscard]] const StationList& get_station_list() const {
    return timetable.get_station_list();
  };
  [[nodiscard]] const TrainList& get_train_list() const {
    return timetable.get_train_list();
  };
  [[nodiscard]] const Schedule& get_schedule(size_t index) const {
    return timetable.get_schedule(index);
  };
  [[nodiscard]] const Schedule&
  get_schedule(const std::string& train_name) const {
    return timetable.get_schedule(train_name);
  };

  [[nodiscard]] int                 max_t() const { return timetable.max_t(); };
  [[nodiscard]] std::pair<int, int> time_interval(size_t train_index) const {
    return timetable.time_interval(train_index);
  };
  [[nodiscard]] std::pair<int, int>
  time_interval(const std::string& train_name) const {
    return timetable.time_interval(train_name);
  };

  void sort_stops() { timetable.sort_stops(); };

  // RouteMap functions
  void add_empty_route(const std::string& train_name) {
    routes.add_empty_route(train_name, get_train_list());
  };

  void push_back_edge_to_route(const std::string& train_name,
                               size_t             edge_index) {
    routes.push_back_edge(train_name, edge_index, network);
  };
  void push_back_edge_to_route(const std::string& train_name, size_t source,
                               size_t target) {
    routes.push_back_edge(train_name, source, target, network);
  };
  void push_back_edge_to_route(const std::string& train_name,
                               const std::string& source,
                               const std::string& target) {
    routes.push_back_edge(train_name, source, target, network);
  };

  void push_front_edge_to_route(const std::string& train_name,
                                size_t             edge_index) {
    routes.push_front_edge(train_name, edge_index, network);
  };
  void push_front_edge_to_route(const std::string& train_name, size_t source,
                                size_t target) {
    routes.push_front_edge(train_name, source, target, network);
  };
  void push_front_edge_to_route(const std::string& train_name,
                                const std::string& source,
                                const std::string& target) {
    routes.push_front_edge(train_name, source, target, network);
  };

  void remove_first_edge_from_route(const std::string& train_name) {
    routes.remove_first_edge(train_name);
  };
  void remove_last_edge_from_route(const std::string& train_name) {
    routes.remove_last_edge(train_name);
  };

  [[nodiscard]] bool has_route(const std::string& train_name) const {
    return routes.has_route(train_name);
  };
  [[nodiscard]] size_t       route_map_size() const { return routes.size(); };
  [[nodiscard]] const Route& get_route(const std::string& train_name) const {
    return routes.get_route(train_name);
  };

  [[nodiscard]] double route_length(const std::string& train_name) const {
    return routes.length(train_name, network);
  };
  [[nodiscard]] std::pair<double, double>
  route_edge_pos(const std::string& train_name, size_t edge) const {
    return routes.edge_pos(train_name, edge, network);
  };
  [[nodiscard]] std::pair<double, double>
  route_edge_pos(const std::string& train_name, size_t source,
                 size_t target) const {
    return routes.edge_pos(train_name, source, target, network);
  };
  [[nodiscard]] std::pair<double, double>
  route_edge_pos(const std::string& train_name, const std::string& source,
                 const std::string& target) const {
    return routes.edge_pos(train_name, source, target, network);
  };
  [[nodiscard]] std::pair<double, double>
  route_edge_pos(const std::string&         train_name,
                 const std::vector<size_t>& edges) const {
    return routes.edge_pos(train_name, edges, network);
  };

  // General Consistency Check
  [[nodiscard]] bool
  check_consistency(bool every_train_must_have_route = true) const {
    return (timetable.check_consistency(network) &&
            routes.check_consistency(get_train_list(), network,
                                     every_train_must_have_route));
  };

  // Export and import functions
  void export_instance(const std::filesystem::path& p) const;
  void export_instance(const std::string& path) const {
    export_instance(std::filesystem::path(path));
  };
  void export_instance(const char* path) const {
    export_instance(std::filesystem::path(path));
  };

  [[nodiscard]] static VSSGenerationTimetable
  import_instance(const std::filesystem::path& p,
                  bool every_train_must_have_route = true) {
    return VSSGenerationTimetable(p, every_train_must_have_route);
  };
  [[nodiscard]] static VSSGenerationTimetable
  import_instance(const std::string& path,
                  bool               every_train_must_have_route = true) {
    return VSSGenerationTimetable(path, every_train_must_have_route);
  };
  [[nodiscard]] static VSSGenerationTimetable
  import_instance(const char* path, bool every_train_must_have_route = true) {
    return VSSGenerationTimetable(path, every_train_must_have_route);
  };

  // Transformation functions
  void discretize(
      const vss::SeparationFunction& sep_func = &vss::functions::uniform);

  // Helper
  [[nodiscard]] std::vector<size_t>
  trains_in_section(const std::vector<size_t>& section) const;
  [[nodiscard]] std::vector<size_t> trains_at_t(int t) const;
  [[nodiscard]] std::vector<size_t>
  trains_at_t(int t, const std::vector<size_t>& trains_to_consider) const;
  [[nodiscard]] bool                has_route_for_every_train() const;
  [[nodiscard]] std::vector<size_t> trains_on_edge(size_t edge_id,
                                                   bool   fixed_routes) const;
  [[nodiscard]] std::vector<size_t>
  trains_on_edge(size_t edge_id, bool fixed_routes,
                 const std::vector<size_t>& trains_to_consider) const;
  [[nodiscard]] std::vector<size_t>
  edges_used_by_train(size_t train_id, bool fixed_routes) const {
    return edges_used_by_train(get_train_list().get_train(train_id).name,
                               fixed_routes);
  };
  [[nodiscard]] std::vector<size_t>
  edges_used_by_train(const std::string& train_name, bool fixed_routes) const;
};
} // namespace cda_rail::instances
