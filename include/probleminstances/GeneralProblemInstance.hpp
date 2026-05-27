#pragma once

#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Route.hpp"
#include "datastructure/Station.hpp"
#include "datastructure/Timetable.hpp"
#include "datastructure/Train.hpp"
#include "nlohmann/json.hpp"
#include "nlohmann/json_fwd.hpp"
#include "plog/Util.h"

#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

using json = nlohmann::json;

namespace cda_rail::instances {

class GeneralProblemInstance {
  std::string m_instance_name{"UnnamedInstance"};
  std::string m_instance_subdirectory{"UnnamedSubdirectory"};

protected:
  GeneralProblemInstance() = default;
  GeneralProblemInstance(std::string_view const instanceName,
                         std::string_view const instanceSubdirectory)
      : m_instance_name(instanceName),
        m_instance_subdirectory(instanceSubdirectory) {
    exceptions::throw_if_invalid_folder_name(instanceName);
    exceptions::throw_if_invalid_folder_name(instanceSubdirectory);
  };

public:
  virtual void
  export_instance(const std::filesystem::path& workingDirectory) const = 0;

  virtual void export_instance(const std::string& workingDirectory) const {
    export_instance(std::filesystem::path(workingDirectory));
  };
  virtual void export_instance(char const* const workingDirectory) const {
    export_instance(std::filesystem::path(workingDirectory));
  };

  [[nodiscard]] virtual bool check_consistency() const = 0;

  virtual ~GeneralProblemInstance() = default;

  // rule of 5 (because of virtual destructor)
  GeneralProblemInstance(const GeneralProblemInstance&)            = default;
  GeneralProblemInstance& operator=(const GeneralProblemInstance&) = default;
  GeneralProblemInstance(GeneralProblemInstance&&) noexcept        = default;
  GeneralProblemInstance&
  operator=(GeneralProblemInstance&&) noexcept = default;

  // Getter
  [[nodiscard]] const std::string& get_instance_name() const {
    return m_instance_name;
  }
  [[nodiscard]] const std::string& get_instance_subdirectory() const {
    return m_instance_subdirectory;
  }
};

class GeneralProblemInstanceWithScheduleAndRoutes
    : public GeneralProblemInstance {
  friend class SolGeneralProblemInstanceWithScheduleAndRoutes;

  Network   m_network{};
  Timetable m_timetable{};
  RouteMap  m_routes{};

protected:
  GeneralProblemInstanceWithScheduleAndRoutes() = default;
  explicit GeneralProblemInstanceWithScheduleAndRoutes(Network network)
      : m_network(std::move(network)) {};
  explicit GeneralProblemInstanceWithScheduleAndRoutes(Network   network,
                                                       Timetable timetable,
                                                       RouteMap  routes)
      : m_network(std::move(network)), m_timetable(std::move(timetable)),
        m_routes(std::move(routes)) {};
  GeneralProblemInstanceWithScheduleAndRoutes(
      std::string_view const       instanceName,
      std::string_view const       instanceSubdirectory,
      std::filesystem::path const& workingDirectory);
  GeneralProblemInstanceWithScheduleAndRoutes(
      std::string_view const instanceName,
      std::string_view const instanceSubdirectory,
      std::string const&     workingDirectory)
      : GeneralProblemInstanceWithScheduleAndRoutes(
            instanceName, instanceSubdirectory,
            std::filesystem::path(workingDirectory)) {};
  GeneralProblemInstanceWithScheduleAndRoutes(
      std::string_view const instanceName,
      std::string_view const instanceSubdirectory,
      char const* const      workingDirectory)
      : GeneralProblemInstanceWithScheduleAndRoutes(
            instanceName, instanceSubdirectory,
            std::filesystem::path(workingDirectory)) {};

  [[nodiscard]] auto& get_editable_network() { return m_network; };
  [[nodiscard]] auto& get_editable_timetable() { return m_timetable; };
  [[nodiscard]] auto& get_editable_routes() { return m_routes; };

public:
  // --------------------
  // EXPORT
  // --------------------

  virtual void export_instance(std::filesystem::path const& workingDirectory,
                               bool const                   saveNetwork) const;
  void         export_instance(
      std::filesystem::path const& workingDirectory) const override {
    export_instance(workingDirectory, false);
  }

  // ---------------------
  // GETTER
  // ---------------------

  // Simple Getters

  [[nodiscard]] const auto& get_const_network() const { return m_network; };
  [[nodiscard]] const auto& get_const_timetable() const { return m_timetable; };
  [[nodiscard]] const auto& get_const_routes() const { return m_routes; };

  Train& editable_train(size_t const index) {
    return m_timetable.editable_train(index);
  }
  Train& editable_train(const std::string& name) {
    return m_timetable.editable_train(name);
  };

  [[nodiscard]] const StationList& get_const_station_list() const {
    return this->get_const_timetable().get_station_list();
  };
  [[nodiscard]] const TrainList& get_const_train_list() const {
    return this->get_const_timetable().get_train_list();
  };
  [[nodiscard]] const auto& get_const_schedule(size_t index) const {
    return this->get_const_timetable().get_schedule(index);
  };
  [[nodiscard]] const auto&
  get_const_schedule(const std::string& train_name) const {
    return this->get_const_timetable().get_schedule(train_name);
  };

  // Route getter

  [[nodiscard]] bool has_route_for_every_train() const;

  [[nodiscard]] double route_length(const std::string& train_name) const {
    return m_routes.route_length(train_name, this->get_const_network());
  };

  // Stop track getters

  [[nodiscard]] std::vector<
      std::pair<size_t, std::vector<cda_rail::index_vector>>>
  get_stop_tracks(size_t tr, const std::string& station_name,
                  const cda_rail::index_set& edges_to_consider = {}) const {
    return this->get_const_timetable().get_stop_tracks(
        tr, station_name, this->get_const_network(), edges_to_consider);
  };

  [[nodiscard]] std::optional<double>
  get_last_stop_position_on_route(size_t             tr_id,
                                  const std::string& station_name) const;

  [[nodiscard]] std::vector<
      std::pair<size_t, std::vector<cda_rail::index_vector>>>
  possible_stop_vertices(
      size_t tr, const std::string& station_name,
      const cda_rail::index_set& edges_to_consider = {}) const;
  [[nodiscard]] std::vector<
      std::pair<size_t, std::vector<cda_rail::index_vector>>>
  possible_stop_vertices(
      const std::string& train_name, const std::string& station_name,
      const cda_rail::index_set& edges_to_consider = {}) const {
    return possible_stop_vertices(
        get_const_timetable().get_train_list().get_train_index(train_name),
        station_name, edges_to_consider);
  };

  // Train usage functions

  [[nodiscard]] cda_rail::index_set
  edges_used_by_train(const std::string& train_name, bool fixed_routes,
                      bool error_if_no_route = true) const;
  [[nodiscard]] cda_rail::index_set
  edges_used_by_train(size_t train_id, bool fixed_routes,
                      bool error_if_no_route = true) const {
    return edges_used_by_train(
        get_const_train_list().get_train(train_id).get_name(), fixed_routes,
        error_if_no_route);
  }

  [[nodiscard]] cda_rail::index_set
  vertices_used_by_train(const std::string& tr_name, bool fixed_routes,
                         bool error_if_no_route = true) const;
  [[nodiscard]] cda_rail::index_set
  vertices_used_by_train(size_t tr_id, bool fixed_routes,
                         bool error_if_no_route = true) const {
    return vertices_used_by_train(
        get_const_train_list().get_train(tr_id).get_name(), fixed_routes,
        error_if_no_route);
  }

  [[nodiscard]] cda_rail::index_set
  sections_used_by_train(const std::string&                      tr_name,
                         const std::vector<cda_rail::index_set>& sections,
                         bool                                    fixed_routes,
                         bool error_if_no_route = true) const;
  [[nodiscard]] cda_rail::index_set sections_used_by_train(
      size_t tr_id, const std::vector<cda_rail::index_set>& sections,
      bool fixed_routes, bool error_if_no_route = true) const {
    return sections_used_by_train(
        get_const_train_list().get_train(tr_id).get_name(), sections,
        fixed_routes, error_if_no_route);
  }

  [[nodiscard]] cda_rail::index_set
  trains_in_section(const cda_rail::index_set& section, bool fix_routes = true,
                    bool error_if_no_route = true) const;

  [[nodiscard]] cda_rail::index_set
  trains_on_edge(size_t edge_id, bool fixed_routes,
                 const cda_rail::index_set& trains_to_consider,
                 bool                       error_if_not_route = true) const;
  [[nodiscard]] cda_rail::index_set
  all_trains_on_edge(size_t edge_id, bool fixed_routes = true,
                     bool error_if_not_route = true) const;

  // --------------------
  // Editing functions
  // --------------------

  size_t add_train(std::string const& train_name, double length,
                   double max_speed, double acceleration, double deceleration,
                   bool tim, double entry_time, double initial_velocity,
                   Network::VertexInput const& entry_vertex, double exit_time,
                   double                      exit_velocity,
                   Network::VertexInput const& exit_vertex) {
    return m_timetable.add_train(
        train_name, length, max_speed, acceleration, deceleration, tim,
        entry_time, initial_velocity, entry_vertex, exit_time, exit_velocity,
        exit_vertex, this->get_const_network());
  }
  size_t add_train(std::string const& train_name, double length,
                   double max_speed, double acceleration, double deceleration,
                   double entry_time, double initial_velocity,
                   Network::VertexInput const& entry_vertex, double exit_time,
                   double                      exit_velocity,
                   Network::VertexInput const& exit_vertex) {
    return m_timetable.add_train(train_name, length, max_speed, acceleration,
                                 deceleration, entry_time, initial_velocity,
                                 entry_vertex, exit_time, exit_velocity,
                                 exit_vertex, this->get_const_network());
  }

  void add_empty_station(const std::string& name) {
    m_timetable.add_empty_station(name);
  };
  void add_track_to_station(const std::string&        name,
                            Network::EdgeInput const& track) {
    m_timetable.add_track_to_station(name, track, this->get_const_network());
  }

  void insert_stop(size_t train_index, std::string const& station_name,
                   double service_time, double service_duration) {
    m_timetable.insert_stop(train_index, station_name, service_time,
                            service_duration);
  }
  void insert_stop(std::string const& train_name,
                   std::string const& station_name, double service_time,
                   double service_duration) {
    m_timetable.insert_stop(train_name, station_name, service_time,
                            service_duration);
  }

  // RouteMap functions
  void add_empty_route(const std::string& train_name) {
    m_routes.add_empty_route(train_name, get_const_train_list());
  }
  void push_back_edge_to_route(const std::string&        train_name,
                               Network::EdgeInput const& edge_input) {
    m_routes.push_back_edge(train_name, edge_input, this->get_const_network());
  }
  void push_front_edge_to_route(const std::string&        train_name,
                                Network::EdgeInput const& edge_input) {
    m_routes.push_front_edge(train_name, edge_input, this->get_const_network());
  }
  void remove_first_edge_from_route(const std::string& train_name) {
    m_routes.remove_first_edge(train_name);
  }
  void remove_last_edge_from_route(const std::string& train_name) {
    m_routes.remove_last_edge(train_name);
  }

  [[nodiscard]] Route::EdgePosition
  route_edge_pos(const std::string&        train_name,
                 Network::EdgeInput const& edge) const {
    return get_const_routes()
        .get_route(train_name)
        .edge_pos_on_route(edge, this->get_const_network());
  }
  [[nodiscard]] Route::EdgePosition
  route_edge_pos(const std::string&            train_name,
                 const cda_rail::index_vector& edges) const {
    return get_const_routes()
        .get_route(train_name)
        .edge_set_pos_on_route(edges, this->get_const_network());
  }

  // --------------------
  // Overlap detection
  // --------------------

  [[nodiscard]] std::vector<ConflictPair>
  get_parallel_overlaps(const std::string& train1,
                        const std::string& train2) const {
    return get_const_routes().get_parallel_overlaps(train1, train2,
                                                    this->get_const_network());
  }
  [[nodiscard]] std::vector<ConflictPair>
  get_ttd_overlaps(const std::string& train1, const std::string& train2) const {
    return get_const_routes().get_ttd_overlaps(train1, train2,
                                               this->get_const_network());
  }
  [[nodiscard]] std::vector<ConflictPair>
  get_reverse_overlaps(const std::string& train1,
                       const std::string& train2) const {
    return get_const_routes().get_reverse_overlaps(train1, train2,
                                                   this->get_const_network());
  }
  [[nodiscard]] std::vector<ConflictPair>
  get_crossing_overlaps(const std::string& train1,
                        const std::string& train2) const {
    return get_const_routes().get_crossing_overlaps(train1, train2,
                                                    this->get_const_network());
  }

  // ---------------
  // Consistency
  // ---------------

  [[nodiscard]] bool check_consistency() const override {
    return check_consistency(true);
  }

  [[nodiscard]] virtual bool
  check_consistency(bool every_train_must_have_route) const;
};

template <typename T> class SolGeneralProblemInstance {
  static_assert(std::is_base_of_v<GeneralProblemInstance, T>,
                "T must be a child of GeneralProblemInstance");

protected:
  T              instance;
  SolutionStatus status  = SolutionStatus::Unknown;
  double         obj     = -1;
  bool           has_sol = false;

  SolGeneralProblemInstance() = default;
  explicit SolGeneralProblemInstance(const T& instance) : instance(instance) {};
  SolGeneralProblemInstance(const T& instance, SolutionStatus status,
                            double obj, bool has_sol)
      : instance(instance), status(status), obj(obj), has_sol(has_sol) {};

  void export_general_solution_data(const std::filesystem::path& p,
                                    bool export_instance,
                                    bool export_data) const {
    if (!is_directory_and_create(p / "solution")) {
      throw exceptions::ExportException("Could not create directory " +
                                        p.string());
    }

    if (export_instance) {
      instance.export_instance(p / "instance");
    }

    if (export_data) {
      json data;
      data["status"]       = static_cast<int>(status);
      data["obj"]          = obj;
      data["has_solution"] = has_sol;
      std::ofstream data_file(p / "solution" / "data.json");
      data_file << data << '\n';
      data_file.close();
    }
  };

  [[nodiscard]] json get_general_solution_data() const {
    json data;
    data["status"]       = static_cast<int>(status);
    data["obj"]          = obj;
    data["has_solution"] = has_sol;
    return data;
  };

  void set_general_solution_data(const json& data) {
    this->status  = static_cast<SolutionStatus>(data["status"].get<int>());
    this->obj     = data["obj"].get<double>();
    this->has_sol = data["has_solution"].get<bool>();
  };

  [[nodiscard]] bool check_general_solution_data_consistency() const {
    if (status == SolutionStatus::Unknown) {
      return false;
    }
    if (status == SolutionStatus::Infeasible ||
        status == SolutionStatus::Timeout) {
      return true;
    }
    return obj + EPS >= 0;
  }

public:
  [[nodiscard]] const T&       get_instance() const { return instance; };
  [[nodiscard]] SolutionStatus get_status() const { return status; };
  [[nodiscard]] double         get_obj() const { return obj; };
  [[nodiscard]] bool           has_solution() const { return has_sol; };
  void set_status(SolutionStatus new_status) { status = new_status; };
  void set_obj(double new_obj) { obj = new_obj; };
  void set_solution_found() { has_sol = true; };
  void set_solution_not_found() { has_sol = false; };

  virtual void export_solution(const std::filesystem::path& p,
                               bool export_instance) const = 0;

  void export_solution(const std::filesystem::path& p) const {
    export_solution(p, true);
  };

  void export_solution(const std::string& path,
                       bool               export_instance = true) const {
    export_solution(std::filesystem::path(path), export_instance);
  };
  void export_solution(const char* path, bool export_instance = true) const {
    export_solution(std::filesystem::path(path), export_instance);
  };

  [[nodiscard]] virtual bool check_consistency() const = 0;

  virtual ~SolGeneralProblemInstance() = default;
};

template <typename T>
class SolGeneralProblemInstanceWithScheduleAndRoutes
    : public SolGeneralProblemInstance<T> {
  static_assert(std::is_base_of_v<GeneralProblemInstance, T>,
                "T must be a child of GeneralProblemInstance");

protected:
  void export_general_solution_data_with_routes(const std::filesystem::path& p,
                                                bool export_instance,
                                                bool export_data) const {
    if (!is_directory_and_create(p / "solution")) {
      throw exceptions::ExportException("Could not create directory " +
                                        p.string());
    }

    if (!export_instance) {
      this->get_instance().const_routes().export_routes(
          p / "instance" / "routes", this->get_instance().get_const_network());
    }

    SolGeneralProblemInstance<T>::export_general_solution_data(
        p, export_instance, export_data);
  }

public:
  SolGeneralProblemInstanceWithScheduleAndRoutes() = default;
  explicit SolGeneralProblemInstanceWithScheduleAndRoutes(const T& instance)
      : SolGeneralProblemInstance<T>(instance) {};
  SolGeneralProblemInstanceWithScheduleAndRoutes(const T&       instance,
                                                 SolutionStatus status,
                                                 double obj, bool has_sol)
      : SolGeneralProblemInstance<T>(instance, status, obj, has_sol) {};

  // RouteMap functions
  void reset_routes() {
    for (const auto& tr : this->instance.get_train_list()) {
      if (this->instance.has_route(tr.name)) {
        this->instance.routes.remove_route(tr.name);
      }
    }
  }
  void add_empty_route(const std::string& train_name) {
    this->instance.add_empty_route(train_name);
  };

  void push_back_edge_to_route(const std::string& train_name,
                               size_t             edge_index) {
    this->instance.push_back_edge_to_route(train_name, edge_index);
  };
  void push_back_edge_to_route(const std::string& train_name, size_t source,
                               size_t target) {
    this->instance.push_back_edge_to_route(train_name, source, target);
  };
  void push_back_edge_to_route(const std::string& train_name,
                               const std::string& source,
                               const std::string& target) {
    this->instance.push_back_edge_to_route(train_name, source, target);
  };

  void push_front_edge_to_route(const std::string& train_name,
                                size_t             edge_index) {
    this->instance.push_front_edge_to_route(train_name, edge_index);
  };
  void push_front_edge_to_route(const std::string& train_name, size_t source,
                                size_t target) {
    this->instance.push_front_edge_to_route(train_name, source, target);
  };
  void push_front_edge_to_route(const std::string& train_name,
                                const std::string& source,
                                const std::string& target) {
    this->instance.push_front_edge_to_route(train_name, source, target);
  };

  void remove_first_edge_from_route(const std::string& train_name) {
    this->instance.remove_first_edge_from_route(train_name);
  };
  void remove_last_edge_from_route(const std::string& train_name) {
    this->instance.remove_last_edge_from_route(train_name);
  };
};
} // namespace cda_rail::instances
