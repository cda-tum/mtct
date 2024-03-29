#pragma once
#include "CustomExceptions.hpp"
#include "VSSModel.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Route.hpp"
#include "datastructure/Timetable.hpp"

#include <filesystem>
#include <optional>

namespace cda_rail::instances {
class VSSGenerationTimetable {
private:
  Network   network;
  Timetable timetable;
  RouteMap  routes;

  friend class SolVSSGenerationTimetable;

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
  size_t add_train(const std::string& name, int length, double max_speed,
                   double acceleration, double deceleration, bool tim, int t_0,
                   double v_0, size_t entry, int t_n, double v_n, size_t exit) {
    return timetable.add_train(name, length, max_speed, acceleration,
                               deceleration, tim, t_0, v_0, entry, t_n, v_n,
                               exit, network);
  };
  size_t add_train(const std::string& name, int length, double max_speed,
                   double acceleration, double deceleration, bool tim, int t_0,
                   double v_0, const std::string& entry, int t_n, double v_n,
                   const std::string& exit) {
    return timetable.add_train(name, length, max_speed, acceleration,
                               deceleration, tim, t_0, v_0, entry, t_n, v_n,
                               exit, network);
  };

  Train& editable_tr(size_t index) { return timetable.editable_tr(index); };
  Train& editable_tr(const std::string& name) {
    return timetable.editable_tr(name);
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

  [[nodiscard]] std::pair<size_t, size_t>
  time_index_interval(size_t train_index, int dt,
                      bool tn_inclusive = true) const {
    return timetable.time_index_interval(train_index, dt, tn_inclusive);
  }
  [[nodiscard]] std::pair<size_t, size_t>
  time_index_interval(const std::string& train_name, int dt,
                      bool tn_inclusive = true) const {
    return timetable.time_index_interval(train_name, dt, tn_inclusive);
  }

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

class SolVSSGenerationTimetable {
private:
  VSSGenerationTimetable           instance;
  std::vector<std::vector<double>> vss_pos;

  int                              dt = -1;
  std::vector<std::vector<double>> train_pos;
  std::vector<std::vector<double>> train_speed;

  SolutionStatus status        = SolutionStatus::Unknown;
  double         obj           = -1;
  double         mip_obj       = -1;
  bool           postprocessed = false;
  bool           has_sol       = false;

  void initialize_vectors();

public:
  // Constructor
  explicit SolVSSGenerationTimetable(VSSGenerationTimetable instance, int dt);
  explicit SolVSSGenerationTimetable(
      const std::filesystem::path&                 p,
      const std::optional<VSSGenerationTimetable>& instance =
          std::optional<VSSGenerationTimetable>());
  SolVSSGenerationTimetable() = delete;

  // Getter
  [[nodiscard]] const VSSGenerationTimetable& get_instance() const {
    return instance;
  };

  [[nodiscard]] const std::vector<double>& get_vss_pos(size_t edge_id) const {
    if (!instance.const_n().has_edge(edge_id)) {
      throw cda_rail::exceptions::EdgeNotExistentException(edge_id);
    }
    return vss_pos.at(edge_id);
  };
  [[nodiscard]] const std::vector<double>& get_vss_pos(size_t source,
                                                       size_t target) const {
    return get_vss_pos(instance.const_n().get_edge_index(source, target));
  };
  [[nodiscard]] const std::vector<double>&
  get_vss_pos(const std::string& source, const std::string& target) const {
    return get_vss_pos(instance.const_n().get_edge_index(source, target));
  };

  [[nodiscard]] double get_train_pos(size_t train_id, int time) const;
  [[nodiscard]] double get_train_pos(const std::string& train_name,
                                     int                time) const {
    return get_train_pos(instance.get_train_list().get_train_index(train_name),
                         time);
  }
  [[nodiscard]] std::vector<double>
  get_valid_border_stops(size_t train_id) const;
  [[nodiscard]] std::vector<double>
  get_valid_border_stops(const std::string& train_name) const {
    return get_valid_border_stops(
        instance.get_train_list().get_train_index(train_name));
  }

  [[nodiscard]] double get_train_speed(size_t train_id, int time) const;
  [[nodiscard]] double get_train_speed(const std::string& train_name,
                                       int                time) const {
    return get_train_speed(
        instance.get_train_list().get_train_index(train_name), time);
  }

  [[nodiscard]] SolutionStatus get_status() const { return status; };
  [[nodiscard]] double         get_obj() const { return obj; };
  [[nodiscard]] double         get_mip_obj() const { return mip_obj; };
  [[nodiscard]] bool get_postprocessed() const { return postprocessed; };
  [[nodiscard]] int  get_dt() const { return dt; };
  [[nodiscard]] bool has_solution() const { return has_sol; };
  void set_status(SolutionStatus new_status) { status = new_status; };
  void set_obj(double new_obj) { obj = new_obj; };
  void set_mip_obj(double new_mip_obj) { mip_obj = new_mip_obj; };
  void set_postprocessed(bool new_postprocessed) {
    postprocessed = new_postprocessed;
  };
  void set_solution_found() { has_sol = true; };
  void set_solution_not_found() { has_sol = false; };

  // RouteMap functions
  void reset_routes() {
    for (const auto& tr : instance.get_train_list()) {
      if (instance.has_route(tr.name)) {
        instance.routes.remove_route(tr.name);
      }
    }
  }
  void add_empty_route(const std::string& train_name) {
    instance.add_empty_route(train_name);
  };

  void push_back_edge_to_route(const std::string& train_name,
                               size_t             edge_index) {
    instance.push_back_edge_to_route(train_name, edge_index);
  };
  void push_back_edge_to_route(const std::string& train_name, size_t source,
                               size_t target) {
    instance.push_back_edge_to_route(train_name, source, target);
  };
  void push_back_edge_to_route(const std::string& train_name,
                               const std::string& source,
                               const std::string& target) {
    instance.push_back_edge_to_route(train_name, source, target);
  };

  void push_front_edge_to_route(const std::string& train_name,
                                size_t             edge_index) {
    instance.push_front_edge_to_route(train_name, edge_index);
  };
  void push_front_edge_to_route(const std::string& train_name, size_t source,
                                size_t target) {
    instance.push_front_edge_to_route(train_name, source, target);
  };
  void push_front_edge_to_route(const std::string& train_name,
                                const std::string& source,
                                const std::string& target) {
    instance.push_front_edge_to_route(train_name, source, target);
  };

  void remove_first_edge_from_route(const std::string& train_name) {
    instance.remove_first_edge_from_route(train_name);
  };
  void remove_last_edge_from_route(const std::string& train_name) {
    instance.remove_last_edge_from_route(train_name);
  };

  void add_vss_pos(size_t edge_id, double pos, bool reverse_edge = true);
  void add_vss_pos(size_t source, size_t target, double pos,
                   bool reverse_edge = true) {
    add_vss_pos(instance.const_n().get_edge_index(source, target), pos,
                reverse_edge);
  };
  void add_vss_pos(const std::string& source, const std::string& target,
                   double pos, bool reverse_edge = true) {
    add_vss_pos(instance.const_n().get_edge_index(source, target), pos,
                reverse_edge);
  };

  void set_vss_pos(size_t edge_id, std::vector<double> pos);
  void set_vss_pos(size_t source, size_t target, std::vector<double> pos) {
    set_vss_pos(instance.const_n().get_edge_index(source, target),
                std::move(pos));
  };
  void set_vss_pos(const std::string& source, const std::string& target,
                   std::vector<double> pos) {
    set_vss_pos(instance.const_n().get_edge_index(source, target),
                std::move(pos));
  };

  void reset_vss_pos(size_t edge_id);
  void reset_vss_pos(size_t source, size_t target) {
    reset_vss_pos(instance.const_n().get_edge_index(source, target));
  };
  void reset_vss_pos(const std::string& source, const std::string& target) {
    reset_vss_pos(instance.const_n().get_edge_index(source, target));
  };

  void add_train_pos(size_t train_id, int time, double pos);
  void add_train_pos(const std::string& train_name, int time, double pos) {
    add_train_pos(instance.get_train_list().get_train_index(train_name), time,
                  pos);
  };

  void add_train_speed(size_t train_id, int time, double speed);
  void add_train_speed(const std::string& train_name, int time, double speed) {
    add_train_speed(instance.get_train_list().get_train_index(train_name), time,
                    speed);
  };

  [[nodiscard]] bool check_consistency() const;

  void export_solution(const std::filesystem::path& p,
                       bool export_instance = true) const;
  void export_solution(const std::string& path,
                       bool               export_instance = true) const {
    export_solution(std::filesystem::path(path), export_instance);
  };
  void export_solution(const char* path, bool export_instance = true) const {
    export_solution(std::filesystem::path(path), export_instance);
  };

  [[nodiscard]] static SolVSSGenerationTimetable
  import_solution(const std::filesystem::path&                 p,
                  const std::optional<VSSGenerationTimetable>& instance =
                      std::optional<VSSGenerationTimetable>()) {
    return SolVSSGenerationTimetable(p, instance);
  };
  [[nodiscard]] static SolVSSGenerationTimetable
  import_solution(const std::string&                           path,
                  const std::optional<VSSGenerationTimetable>& instance =
                      std::optional<VSSGenerationTimetable>()) {
    return import_solution(std::filesystem::path(path), instance);
  };
  [[nodiscard]] static SolVSSGenerationTimetable
  import_solution(const char*                                  path,
                  const std::optional<VSSGenerationTimetable>& instance =
                      std::optional<VSSGenerationTimetable>()) {
    return import_solution(std::filesystem::path(path), instance);
  };
};
} // namespace cda_rail::instances
