#pragma once

#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "StringHelper.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Route.hpp"
#include "datastructure/Timetable.hpp"
#include "datastructure/Train.hpp"
#include "nlohmann/json_fwd.hpp"

#include <cstddef>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace cda_rail::instances {

class GeneralProblemInstance {
  std::string m_instance_name{"UnnamedInstance"};
  std::string m_instance_subdirectory{"UnnamedSubdirectory"};

protected:
  /**
   * @brief Initializes the instance with empty name and subdirectory.
   */
  GeneralProblemInstance() = default;
  /**
   * @brief Initializes the instance with a name and subdirectory.
   *
   * @param instanceName The name of the instance; must be a valid folder name.
   * @param instanceSubdirectory The subdirectory path; must be a valid folder
   * name.
   *
   * @throws If either parameter is not a valid folder name.
   */
  GeneralProblemInstance(std::string_view const instanceName,
                         std::string_view const instanceSubdirectory)
      : m_instance_name(instanceName),
        m_instance_subdirectory(instanceSubdirectory) {
    exceptions::throw_if_invalid_folder_name(instanceName);
    exceptions::throw_if_invalid_folder_name(instanceSubdirectory);
  };

public:
  /**
   * @brief Exports the instance to the specified working directory.
   *
   * @param working_directory Base export directory.
   */
  virtual void
  export_instance(const std::filesystem::path& working_directory) const = 0;

  /**
   * @brief Exports the instance to the specified working directory.
   *
   * @param working_directory Path to the directory where the instance is
   * exported.
   */
  virtual void export_instance(const std::string& working_directory) const {
    export_instance(std::filesystem::path(working_directory));
  };
  /**
   * @brief Exports the instance to the specified working directory.
   *
   * @param workingDirectory Path where the instance will be exported.
   */
  virtual void export_instance(char const* const workingDirectory) const {
    export_instance(std::filesystem::path(workingDirectory));
  };

  /**
   * @brief Checks whether the instance is internally consistent.
   *
   * @return `true` if the instance is consistent, otherwise `false`.
   */
  [[nodiscard]] virtual bool check_consistency() const = 0;

  /** @brief Virtual destructor. */
  virtual ~GeneralProblemInstance() = default;

  // rule of 5 (because of virtual destructor)
  /** @brief Copy constructor. */
  GeneralProblemInstance(const GeneralProblemInstance&) = default;
  /** @brief Copy assignment operator. */
  GeneralProblemInstance& operator=(const GeneralProblemInstance&) = default;
  /** @brief Move constructor. */
  GeneralProblemInstance(GeneralProblemInstance&&) noexcept = default;
  /** @brief Move assignment operator. */
  GeneralProblemInstance&
  operator=(GeneralProblemInstance&&) noexcept = default;

  /**
   * @brief Gets the instance name.
   * @return const std::string& The instance name.
   */
  [[nodiscard]] const std::string& get_instance_name() const {
    return m_instance_name;
  }
  /**
   * @brief Provides the instance subdirectory.
   * @return Const reference to the instance subdirectory string.
   */
  [[nodiscard]] const std::string& get_instance_subdirectory() const {
    return m_instance_subdirectory;
  }

  /**
   * @brief Sets the instance name.
   * @param instanceName The new instance name; must be a valid folder name.
   * @throws If the instance name is not a valid folder name.
   */
  void set_instance_name(std::string_view const instanceName) {
    exceptions::throw_if_invalid_folder_name(instanceName);
    m_instance_name = instanceName;
  }
  /**
   * @brief Sets the instance subdirectory.
   *
   * @param instanceSubdirectory The new subdirectory name.
   *
   * @throws std::invalid_argument If the subdirectory is not a valid folder
   * name.
   */
  void set_instance_subdirectory(std::string_view const instanceSubdirectory) {
    exceptions::throw_if_invalid_folder_name(instanceSubdirectory);
    m_instance_subdirectory = instanceSubdirectory;
  }

protected:
  struct FeasibilityCheck {
    bool        is_obviously_infeasible{false};
    std::string reason{};
  };

public:
  [[nodiscard]] virtual FeasibilityCheck
  is_obviously_infeasible(bool late_entry_allowed) const = 0;
};

class GeneralProblemInstanceWithScheduleAndRoutes
    : public GeneralProblemInstance {
  friend class SolGeneralProblemInstanceWithScheduleAndRoutes;

  Network   m_network{};
  Timetable m_timetable{};
  RouteMap  m_routes{};

protected:
  /** @brief Constructs an empty problem instance with schedule and routes. */
  GeneralProblemInstanceWithScheduleAndRoutes() = default;
  /**
   * @brief Initializes the instance with a network.
   *
   * @param network The network for this instance.
   */
  explicit GeneralProblemInstanceWithScheduleAndRoutes(Network network)
      : m_network(std::move(network)) {};
  /**
   * @brief Initializes the instance with a network, timetable, and routes.
   */
  explicit GeneralProblemInstanceWithScheduleAndRoutes(Network   network,
                                                       Timetable timetable,
                                                       RouteMap  routes)
      : m_network(std::move(network)), m_timetable(std::move(timetable)),
        m_routes(std::move(routes)) {};
  /**
   * @brief Loads a full problem instance from disk.
   *
   * @param instance_name Instance name.
   * @param instance_subdirectory Instance subdirectory.
   * @param working_directory Base working directory.
   */
  GeneralProblemInstanceWithScheduleAndRoutes(
      std::string_view instance_name, std::string_view instance_subdirectory,
      std::filesystem::path const& working_directory);
  /**
   * @brief Constructs a problem instance by loading from a working directory.
   *
   * @param instanceName The name of the problem instance.
   * @param instanceSubdirectory The subdirectory within the working directory
   * containing instance data.
   * @param working_directory The root working directory path.
   */
  GeneralProblemInstanceWithScheduleAndRoutes(
      std::string_view const instanceName,
      std::string_view const instanceSubdirectory,
      std::string const&     working_directory)
      : GeneralProblemInstanceWithScheduleAndRoutes(
            instanceName, instanceSubdirectory,
            std::filesystem::path(working_directory)) {};
  /**
   * @brief Constructs an instance by loading data from the working directory.
   *
   * @param instanceName Name of the instance.
   * @param instanceSubdirectory Subdirectory within the working directory
   * containing instance data.
   * @param workingDirectory Base directory path.
   */
  GeneralProblemInstanceWithScheduleAndRoutes(
      std::string_view const instanceName,
      std::string_view const instanceSubdirectory,
      char const* const      workingDirectory)
      : GeneralProblemInstanceWithScheduleAndRoutes(
            instanceName, instanceSubdirectory,
            std::filesystem::path(workingDirectory)) {};

public:
  // Rule of 5
  GeneralProblemInstanceWithScheduleAndRoutes(
      const GeneralProblemInstanceWithScheduleAndRoutes&) = default;
  GeneralProblemInstanceWithScheduleAndRoutes&
  operator=(const GeneralProblemInstanceWithScheduleAndRoutes&) = default;
  /**
   * @brief Move constructor.
   */
  GeneralProblemInstanceWithScheduleAndRoutes(
      GeneralProblemInstanceWithScheduleAndRoutes&&) noexcept = default;
  GeneralProblemInstanceWithScheduleAndRoutes&
  operator=(GeneralProblemInstanceWithScheduleAndRoutes&&) noexcept = default;
  ~GeneralProblemInstanceWithScheduleAndRoutes() override           = default;

  using GeneralProblemInstance::export_instance;

  // --------------------
  // HELPER
  // --------------------
  /**
   * @brief Checks whether a route end is a valid stopping position.
   *
   * @param tr Train index.
   * @param edges Current route edges.
   * @param next_stop_id Index of the next scheduled stop.
   * @return `true` if the route end can serve as a stop position.
   */
  [[nodiscard]] bool
  is_route_end_valid_stop_pos(size_t tr, const cda_rail::index_vector& edges,
                              size_t next_stop_id) const;

  // --------------------
  // EXPORT
  // --------------------

  /**
   * @brief Exports the instance and optionally its network.
   *
   * @param working_directory Base export directory.
   * @param save_network Whether to export the network alongside the instance.
   */
  virtual void export_instance(std::filesystem::path const& working_directory,
                               bool                         save_network) const;
  /**
   * @brief Exports the instance to the specified working directory without
   * saving the network.
   *
   * @param working_directory The target directory for the export.
   */
  void export_instance(
      std::filesystem::path const& working_directory) const override {
    export_instance(working_directory, false);
  }

  // ---------------------
  // GETTER
  // ---------------------

  /**
   * @brief Provides read-only access to the network.
   *
   * @return const Network& A const reference to the network.
   */

  [[nodiscard]] const auto& get_const_network() const { return m_network; };
  /**
   * @brief Accesses the timetable.
   * @return Const reference to the timetable.
   */
  [[nodiscard]] const auto& get_const_timetable() const { return m_timetable; };
  /**
   * @brief Provides read-only access to the routes.
   *
   * @return A const reference to the RouteMap.
   */
  [[nodiscard]] const auto& get_const_routes() const { return m_routes; };

  /**
   * @brief Provides editable access to the underlying network.
   * @return Network& A reference to the underlying network.
   */
  [[nodiscard]] auto& get_editable_network() { return m_network; };

protected:
  /**
   * @brief Provides editable access to the timetable.
   * @return Reference to the timetable.
   */
  [[nodiscard]] auto& get_editable_timetable() { return m_timetable; };
  /**
   * @brief Provides editable access to the route map.
   *
   * @return RouteMap& Reference to the route map.
   */
  [[nodiscard]] auto& get_editable_routes() { return m_routes; };

public:
  /**
   * @brief Gets a train by its index in the schedule.
   *
   * @return Train& A reference to the editable train at the given index.
   */
  Train& editable_train(size_t const index) {
    return m_timetable.editable_train(index);
  }
  /**
   * @brief Retrieves mutable access to a train by name.
   *
   * @return Train& Mutable reference to the train with the given name.
   */
  Train& editable_train(const std::string& name) {
    return m_timetable.editable_train(name);
  };

  /**
   * @brief Provides the list of all stations.
   * @return Const reference to the StationList.
   */
  [[nodiscard]] const StationList& get_const_station_list() const {
    return this->get_const_timetable().get_station_list();
  };
  /**
   * @brief Provides const access to the list of trains in the instance.
   *
   * @return const TrainList& Reference to the train list.
   */
  [[nodiscard]] const TrainList& get_const_train_list() const {
    return this->get_const_timetable().get_train_list();
  };
  /**
   * @brief Retrieves the schedule for a train.
   *
   * @param index The index of the train.
   * @return The schedule for the train at the specified index.
   */
  [[nodiscard]] const auto& get_const_schedule(size_t index) const {
    return this->get_const_timetable().get_schedule(index);
  };
  /**
   * @brief Retrieves the schedule for a specified train.
   *
   * @param train_name Name of the train.
   * @return Const reference to the train's schedule.
   */
  [[nodiscard]] const auto&
  get_const_schedule(const std::string& train_name) const {
    return this->get_const_timetable().get_schedule(train_name);
  };

  // Route getter

  /**
   * @brief Checks whether every train has an assigned route.
   *
   * @return `true` if each train has a route, otherwise `false`.
   */
  [[nodiscard]] bool has_route_for_every_train() const;

  /**
   * @brief Computes the total length of a train's route.
   *
   * @param train_name Name of the train.
   * @return double The total length of the train's route.
   */
  [[nodiscard]] double route_length(const std::string& train_name) const {
    return m_routes.route_length(train_name, this->get_const_network());
  };

  /**
   * @brief Retrieves the stop tracks for a train at a specified station.
   *
   * @param tr Train index.
   * @param station_name Name of the station.
   * @param edges_to_consider Optional set of edge IDs to filter the results. If
   * empty, all edges are considered.
   * @return A vector of pairs, each containing a track ID and its corresponding
   * stop edge sequences.
   */

  [[nodiscard]] std::vector<
      std::pair<size_t, std::vector<cda_rail::index_vector>>>
  get_stop_tracks(size_t tr, const std::string& station_name,
                  const cda_rail::index_set& edges_to_consider = {}) const {
    return this->get_const_timetable().get_stop_tracks(
        tr, station_name, this->get_const_network(), edges_to_consider);
  };

  /**
   * @brief Returns the last reachable stop position of a station on a route.
   *
   * @param tr_id Train index.
   * @param station_name Name of the station.
   * @return Last route position on which the train can stop at the station, or
   *         `std::nullopt` if no such position exists on the current route.
   */
  [[nodiscard]] std::optional<double>
  get_last_stop_position_on_route(size_t             tr_id,
                                  const std::string& station_name) const;

  /**
   * @brief Computes feasible stop-edge paths for a train at a station.
   *
   * @param tr Train index.
   * @param station_name Name of the station.
   * @param edges_to_consider Optional subset of edges to consider.
   * @return Possible stop vertices, organized by track.
   */
  [[nodiscard]] std::vector<
      std::pair<size_t, std::vector<cda_rail::index_vector>>>
  possible_stop_vertices(
      size_t tr, const std::string& station_name,
      const cda_rail::index_set& edges_to_consider = {}) const;
  /**
   * @brief Finds possible vertices where a train can stop at a station.
   *
   * @param train_name The name of the train.
   * @param station_name The name of the station.
   * @param edges_to_consider Optional set of edges to consider.
   * @return Possible stop vertices, organized by track.
   */
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

  /**
   * @brief Returns the edges used by a train.
   *
   * @param train_name Name of the train.
   * @param fixed_routes Whether to use fixed routes.
   * @param error_if_no_route Whether to throw if no route exists.
   * @return Set of used edge indices.
   */
  [[nodiscard]] cda_rail::index_set
  edges_used_by_train(const std::string& train_name, bool fixed_routes,
                      bool error_if_no_route = true) const;
  /**
   * @brief Retrieves the edges used by a train identified by index.
   *
   * @param train_id Index of the train.
   * @param fixed_routes Whether to use fixed routes.
   * @param error_if_no_route If true, throws when the train has no assigned
   * route.
   * @return An index set of edges used by the train.
   */
  [[nodiscard]] cda_rail::index_set
  edges_used_by_train(size_t train_id, bool fixed_routes,
                      bool error_if_no_route = true) const {
    return edges_used_by_train(
        get_const_train_list().get_train(train_id).get_name(), fixed_routes,
        error_if_no_route);
  }

  /**
   * @brief Returns the vertices used by a train.
   *
   * @param tr_name Name of the train.
   * @param fixed_routes Whether to use fixed routes.
   * @param error_if_no_route Whether to throw if no route exists.
   * @return Set of used vertex indices.
   */
  [[nodiscard]] cda_rail::index_set
  vertices_used_by_train(const std::string& tr_name, bool fixed_routes,
                         bool error_if_no_route = true) const;
  /**
   * @brief Retrieves all vertices used by a train identified by its index.
   *
   * @param tr_id Train index.
   * @param fixed_routes Whether to consider only fixed routes.
   * @param error_if_no_route If `true`, throw if the train has no route;
   * otherwise, return an empty set.
   * @return Set of vertex indices used by the train.
   */
  [[nodiscard]] cda_rail::index_set
  vertices_used_by_train(size_t tr_id, bool fixed_routes,
                         bool error_if_no_route = true) const {
    return vertices_used_by_train(
        get_const_train_list().get_train(tr_id).get_name(), fixed_routes,
        error_if_no_route);
  }

  /**
   * @brief Returns the queried sections used by a train.
   *
   * @param tr_name Name of the train.
   * @param sections Sections to test.
   * @param fixed_routes Whether to use fixed routes.
   * @param error_if_no_route Whether to throw if no route exists.
   * @return Set of section indices used by the train.
   */
  [[nodiscard]] cda_rail::index_set
  sections_used_by_train(const std::string&                      tr_name,
                         const std::vector<cda_rail::index_set>& sections,
                         bool                                    fixed_routes,
                         bool error_if_no_route = true) const;
  /**
   * @brief Retrieves sections used by a train specified by index.
   *
   * @param sections Sections to consider when determining usage.
   * @param fixed_routes If `true`, uses the fixed route; if `false`, includes
   * all possible routes.
   * @param error_if_no_route If `true`, throws if the train has no route; if
   * `false`, returns an empty set.
   * @return Sections used by the train.
   */
  [[nodiscard]] cda_rail::index_set sections_used_by_train(
      size_t tr_id, const std::vector<cda_rail::index_set>& sections,
      bool fixed_routes, bool error_if_no_route = true) const {
    return sections_used_by_train(
        get_const_train_list().get_train(tr_id).get_name(), sections,
        fixed_routes, error_if_no_route);
  }

  /**
   * @brief Returns the trains that use a section.
   *
   * @param section Section represented as an edge set.
   * @param fix_routes Whether to use fixed routes.
   * @param error_if_no_route Whether to throw for trains without routes.
   * @return Set of train indices using the section.
   */
  [[nodiscard]] cda_rail::index_set
  trains_in_section(const cda_rail::index_set& section, bool fix_routes = true,
                    bool error_if_no_route = true) const;

  /**
   * @brief Identifies which trains use the specified edge.
   *
   * @param fixed_routes Whether to consider fixed routes.
   * @param trains_to_consider The set of trains to examine.
   * @param error_if_not_route If `true`, raises an error for trains without a
   * route; if `false`, skips them.
   * @return Index set of trains from `trains_to_consider` that use the edge.
   */
  [[nodiscard]] cda_rail::index_set
  trains_on_edge(size_t edge_id, bool fixed_routes,
                 const cda_rail::index_set& trains_to_consider,
                 bool                       error_if_not_route = true) const {
    return trains_on_edge(this->get_const_routes(), edge_id, fixed_routes,
                          trains_to_consider, error_if_not_route);
  }
  /**
   * @brief Identifies which trains from a subset use the specified edge.
   *
   * @param route_map Route map to inspect.
   * @param edge_id Edge index.
   * @param fixed_routes Whether to restrict the check to fixed routes.
   * @param trains_to_consider Subset of train indices to test.
   * @param error_if_not_route Whether to treat missing routes as errors.
   * @return Index set of trains from `trains_to_consider` that use the edge.
   */
  [[nodiscard]] cda_rail::index_set
  trains_on_edge(RouteMap const& route_map, size_t edge_id, bool fixed_routes,
                 const cda_rail::index_set& trains_to_consider,
                 bool                       error_if_not_route = true) const;
  /**
   * @brief Returns the indices of all trains using a given edge.
   *
   * @param edge_id The edge to query.
   * @param fixed_routes If `true`, queries fixed routes; otherwise queries
   * solution routes.
   * @param error_if_not_route If `true`, throws an error if the edge is not in
   * any train's route.
   * @return An index set of train indices using the edge.
   */
  [[nodiscard]] cda_rail::index_set
  all_trains_on_edge(size_t edge_id, bool fixed_routes = true,
                     bool error_if_not_route = true) const {
    return all_trains_on_edge(this->get_const_routes(), edge_id, fixed_routes,
                              error_if_not_route);
  }
  /**
   * @brief Returns all trains whose routes use the specified edge.
   *
   * @param route_map Route map to inspect.
   * @param edge_id Edge index.
   * @param fixed_routes Whether to restrict the check to fixed routes.
   * @param error_if_not_route Whether to treat missing routes as errors.
   * @return Index set of all trains using the edge under the chosen route map.
   */
  [[nodiscard]] cda_rail::index_set
  all_trains_on_edge(RouteMap const& route_map, size_t edge_id,
                     bool fixed_routes       = true,
                     bool error_if_not_route = true) const;

  // --------------------
  // Editing functions
  /**
   * @brief Adds a train with the specified properties to the timetable.
   *
   * @param train_name Name identifier for the train.
   * @param length Length of the train.
   * @param max_speed Maximum speed.
   * @param acceleration Acceleration rate.
   * @param deceleration Deceleration rate.
   * @param tim Whether train integrity monitoring is available.
   * @param entry_time Time when the train enters the network.
   * @param initial_velocity Velocity upon entry.
   * @param entry_vertex Vertex where the train enters.
   * @param exit_time Time when the train exits the network.
   * @param exit_velocity Velocity upon exit.
   * @param exit_vertex Vertex where the train exits.
   * @return Index of the newly added train.
   */

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
  /**
   * @brief Adds a train to the timetable.
   *
   * @param train_name Name identifier for the train.
   * @param length Length of the train.
   * @param max_speed Maximum speed.
   * @param acceleration Acceleration rate.
   * @param deceleration Deceleration rate.
   * @param entry_time Time when the train enters the network.
   * @param initial_velocity Velocity upon entry.
   * @param entry_vertex Vertex where the train enters.
   * @param exit_time Time when the train exits the network.
   * @param exit_velocity Velocity upon exit.
   * @param exit_vertex Vertex where the train exits.
   * @return Index of the newly added train in the timetable.
   */
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

  /**
   * @brief Adds an empty station to the timetable.
   *
   * @param name The name of the station to add.
   */
  void add_empty_station(const std::string& name) {
    m_timetable.add_empty_station(name);
  };
  /**
   * @brief Adds a track to a station.
   *
   * @param name The station name.
   * @param track The edge input to add as a track.
   */
  void add_track_to_station(const std::string&        name,
                            Network::EdgeInput const& track) {
    m_timetable.add_track_to_station(name, track, this->get_const_network());
  }

  /**
   * @brief Inserts a stop into a train's schedule.
   *
   * @param train_index Index of the train.
   * @param station_name Name of the station.
   * @param service_time Service time at the station.
   * @param service_duration Duration of the service at the station.
   */
  void insert_stop(size_t train_index, std::string const& station_name,
                   double service_time, double service_duration) {
    m_timetable.insert_stop(train_index, station_name, service_time,
                            service_duration);
  }
  /**
   * @brief Inserts a stop for a train at a specified station.
   *
   * @param service_time The scheduled time for the service stop.
   * @param service_duration The duration of the service stop.
   */
  void insert_stop(std::string const& train_name,
                   std::string const& station_name, double service_time,
                   double service_duration) {
    m_timetable.insert_stop(train_name, station_name, service_time,
                            service_duration);
  }

  /**
   * @brief Adds an empty route for a train.
   *
   * @param train_name Name of the train for which to add an empty route.
   */
  void add_empty_route(const std::string& train_name) {
    m_routes.add_empty_route(train_name, get_const_train_list());
  }
  /**
   * @brief Appends an edge to the end of a train's route.
   *
   * @param train_name The name of the train whose route to modify.
   * @param edge_input The edge to append to the route.
   */
  void push_back_edge_to_route(const std::string&        train_name,
                               Network::EdgeInput const& edge_input) {
    m_routes.push_back_edge(train_name, edge_input, this->get_const_network());
  }
  /**
   * @brief Adds an edge to the front of a train's route.
   *
   * @param train_name Name of the train.
   * @param edge_input The edge to add to the front of the route.
   */
  void push_front_edge_to_route(const std::string&        train_name,
                                Network::EdgeInput const& edge_input) {
    m_routes.push_front_edge(train_name, edge_input, this->get_const_network());
  }
  /**
   * @brief Removes the first edge from a train's route.
   *
   * @param train_name The name of the train.
   */
  void remove_first_edge_from_route(const std::string& train_name) {
    m_routes.remove_first_edge(train_name);
  }
  /**
   * @brief Removes the last edge from a train's route.
   *
   * @param train_name The name of the train whose route should be modified.
   */
  void remove_last_edge_from_route(const std::string& train_name) {
    m_routes.remove_last_edge(train_name);
  }

  /**
   * @brief Determines the position of an edge on a train's route.
   *
   * @param train_name The name of the train.
   * @param edge The edge to locate on the route.
   * @return The position of the edge on the route.
   */
  [[nodiscard]] Route::EdgePosition
  route_edge_pos(const std::string&        train_name,
                 Network::EdgeInput const& edge) const {
    return get_const_routes()
        .get_route(train_name)
        .edge_pos_on_route(edge, this->get_const_network());
  }
  /**
   * @brief Determines the position of edges on a train's route.
   *
   * @param train_name Name of the train.
   * @param edges Collection of edge indices to locate.
   * @return Route::EdgePosition The position of the edges on the route.
   */
  [[nodiscard]] Route::EdgePosition
  route_edge_pos(const std::string&            train_name,
                 const cda_rail::index_vector& edges) const {
    return get_const_routes()
        .get_route(train_name)
        .edge_set_pos_on_route(edges, this->get_const_network());
  }

  // --------------------
  // Overlap detection
  /**
   * @brief Identifies parallel overlaps between two trains.
   *
   * @param train1 Name of the first train.
   * @param train2 Name of the second train.
   * @return A vector of conflict pairs representing the parallel overlaps
   * between the trains.
   */

  [[nodiscard]] std::vector<ConflictPair>
  get_parallel_overlaps(const std::string& train1,
                        const std::string& train2) const {
    return get_const_routes().get_parallel_overlaps(train1, train2,
                                                    this->get_const_network());
  }
  /**
   * @brief Identifies time-to-distance overlaps between two trains.
   *
   * @param train1 Name of the first train.
   * @param train2 Name of the second train.
   * @return std::vector<ConflictPair> Conflicts where the trains overlap in
   * time and distance.
   */
  [[nodiscard]] std::vector<ConflictPair>
  get_ttd_overlaps(const std::string& train1, const std::string& train2) const {
    return get_const_routes().get_ttd_overlaps(train1, train2,
                                               this->get_const_network());
  }
  /**
   * @brief Identifies sections where two trains travel in opposite directions.
   *
   * @param train1 Name of the first train.
   * @param train2 Name of the second train.
   * @return A vector of conflict pairs representing sections where the trains
   * traverse the same edges in opposite directions.
   */
  [[nodiscard]] std::vector<ConflictPair>
  get_reverse_overlaps(const std::string& train1,
                       const std::string& train2) const {
    return get_const_routes().get_reverse_overlaps(train1, train2,
                                                   this->get_const_network());
  }
  /**
   * @brief Identifies crossing conflicts between two trains' routes.
   *
   * @param train1 Name of the first train.
   * @param train2 Name of the second train.
   * @return std::vector<ConflictPair> containing all crossing conflicts between
   * the two trains' routes.
   */
  [[nodiscard]] std::vector<ConflictPair>
  get_crossing_overlaps(const std::string& train1,
                        const std::string& train2) const {
    return get_const_routes().get_crossing_overlaps(train1, train2,
                                                    this->get_const_network());
  }

  // ---------------
  // Consistency
  /**
   * @brief Checks that the instance is internally consistent, requiring all
   * trains to have routes.
   *
   * @return `true` if the instance is consistent, `false` otherwise.
   */

  [[nodiscard]] bool check_consistency() const override {
    return check_consistency(true);
  }

  /**
   * @brief Checks consistency while optionally allowing trains without routes.
   *
   * @param every_train_must_have_route Whether every train must have a route.
   * @return `true` if the instance is consistent under the requested rule.
   */
  [[nodiscard]] virtual bool
  check_consistency(bool every_train_must_have_route) const;

  // Helper
  [[nodiscard]] FeasibilityCheck
  is_obviously_infeasible(bool late_entry_allowed) const override;
};

class SolGeneralProblemInstance {
private:
  std::shared_ptr<GeneralProblemInstance> m_instance;
  SolutionStatus                          m_status{SolutionStatus::Unknown};
  double                                  m_obj{-1};
  double                                  m_lb{-1};
  bool                                    m_has_sol{false};

protected:
  /**
   * @brief Creates a solution wrapper around the given problem instance.
   */
  explicit SolGeneralProblemInstance(
      std::shared_ptr<GeneralProblemInstance> instance_ptr)
      : m_instance(std::move(instance_ptr)) {};
  /**
   * @brief Constructs a solution instance with problem instance and solution
   * metadata.
   *
   * @param instance_ptr Problem instance.
   * @param status Solution status.
   * @param obj Objective value.
   * @param has_sol Whether a solution exists.
   */
  SolGeneralProblemInstance(
      std::shared_ptr<GeneralProblemInstance> instance_ptr,
      SolutionStatus status, double obj, bool has_sol)
      : m_instance(std::move(instance_ptr)), m_status(status), m_obj(obj),
        m_has_sol(has_sol) {};

  /**
   * @brief Serializes solution-wide metadata to JSON.
   *
   * @return JSON object containing status, objective, and availability flags.
   */
  [[nodiscard]] nlohmann::json get_general_solution_data() const;
  /**
   * @brief Loads solution-wide metadata from JSON.
   *
   * @param data JSON object containing general solution fields.
   */
  void set_general_solution_data(const nlohmann::json& data);

  /**
   * @brief Retrieves the underlying problem instance pointer.
   *
   * @return Const pointer to the underlying `GeneralProblemInstance`.
   */
  [[nodiscard]] GeneralProblemInstance const*
  get_uncast_instance_pointer() const {
    return m_instance.get();
  };

public:
  /**
   * @brief Computes the file path for exporting a solution.
   *
   * Constructs a path of the form
   * `working_directory/solutions/<solutionSubdirectory>/<instance_subdirectory>/<instance_name>`,
   * with `-<parameter_identifier>` appended to the instance name if provided.
   *
   * @param parameter_identifier Optional identifier appended to the instance
   * name.
   * @return The constructed solution export path.
   */
  [[nodiscard]] std::filesystem::path get_export_path(
      const std::filesystem::path&      working_directory,
      std::string_view const            solutionSubdirectory,
      std::optional<std::string> const& parameter_identifier) const {
    exceptions::throw_if_invalid_folder_name(solutionSubdirectory);
    if (parameter_identifier.has_value()) {
      exceptions::throw_if_invalid_folder_name(parameter_identifier.value());
    }
    auto const adjusted_name =
        parameter_identifier.has_value()
            ? concatenate_string_views({get_instance()->get_instance_name(),
                                        "-", parameter_identifier.value()})
            : get_instance()->get_instance_name();
    return {working_directory / "solutions" / solutionSubdirectory /
            get_instance()->get_instance_subdirectory() / adjusted_name};
  }

  /**
   * @brief Provides access to the problem instance.
   * @return GeneralProblemInstance const* A const pointer to the underlying
   * problem instance.
   */
  [[nodiscard]] virtual GeneralProblemInstance const* get_instance() const {
    return m_instance.get();
  };
  /**
   * @brief Retrieves the current solution status.
   *
   * @return SolutionStatus The solution state.
   */
  [[nodiscard]] SolutionStatus get_status() const { return m_status; };
  /**
   * @brief Gets the objective value of the solution.
   *
   * @return double The objective value. -1 indicates no objective value has
   * been set.
   */
  [[nodiscard]] double get_obj() const { return m_obj; };
  /**
   * @brief Gets the lower bound on the objective value of the solution.
   *
   * @return double The lower bound.
   */
  [[nodiscard]] double get_lower_bound() const { return m_lb; };
  /**
   * @brief Indicates whether a solution has been found.
   *
   * @return bool `true` if a solution has been found, `false` otherwise.
   */
  [[nodiscard]] bool has_solution() const { return m_has_sol; };
  /**
   * @brief Sets the solution status.
   */
  void set_status(SolutionStatus new_status) { m_status = new_status; };
  /**
   * @brief Sets the objective value of the solution.
   *
   * @param new_obj The new objective value.
   */
  void set_obj(double new_obj) { m_obj = new_obj; };
  /**
   * @brief Sets the loewr bound of the objective value of the solution.
   *
   * @param new_obj The lower bound.
   */
  void set_lower_bound(double new_lb) { m_lb = new_lb; };
  /**
   * @brief Marks the solution as found.
   */
  void set_solution_found() { m_has_sol = true; };
  /**
   * @brief Marks that no solution has been found.
   */
  void set_solution_not_found() { m_has_sol = false; };

  virtual void
  load_solution(const std::filesystem::path&      working_directory,
                std::string_view                  solution_subdirectory,
                std::optional<std::string> const& parameter_identifier);

  /**
   * @brief Loads a solution from the specified working directory and solution
   * subdirectory.
   *
   * @param working_directory The base working directory path.
   * @param solution_subdirectory The subdirectory containing the solution.
   */
  virtual void load_solution(const std::filesystem::path& working_directory,
                             std::string_view solution_subdirectory) {
    load_solution(working_directory, solution_subdirectory, {});
  }

  /**
   * @brief Loads a solution from the specified path.
   *
   * @param path Working directory path.
   * @param solutionSubdirectory Subdirectory containing solution files.
   * @param parameter_identifier Optional identifier appended to the instance
   * name when locating solution files.
   */
  void
  load_solution(const std::string&                path,
                std::string_view const            solutionSubdirectory,
                std::optional<std::string> const& parameter_identifier = {}) {
    load_solution(std::filesystem::path(path), solutionSubdirectory,
                  parameter_identifier);
  }
  /**
   * @brief Loads a solution from the specified location.
   *
   * @param path Working directory path.
   * @param solutionSubdirectory Subdirectory containing solutions.
   * @param parameter_identifier Optional identifier to append to the instance
   * name when constructing the solution path.
   */
  void
  load_solution(const char* path, std::string_view const solutionSubdirectory,
                std::optional<std::string> const& parameter_identifier = {}) {
    load_solution(std::filesystem::path(path), solutionSubdirectory,
                  parameter_identifier);
  }

  virtual void
  export_solution(const std::filesystem::path&      working_directory,
                  std::string_view                  solution_subdirectory,
                  std::optional<std::string> const& parameter_identifier) const;

  /**
   * @brief Exports the solution without a parameter identifier.
   *
   * @param working_directory The root directory for solution exports.
   * @param solution_subdirectory The subdirectory path for organizing
   * solutions.
   */
  virtual void export_solution(const std::filesystem::path& working_directory,
                               std::string_view solution_subdirectory) const {
    export_solution(working_directory, solution_subdirectory, {});
  }

  /**
   * @brief Exports the solution to the specified directory.
   *
   * @param path Directory path where the solution will be exported.
   * @param solutionSubdirectory Solution subdirectory name.
   * @param parameter_identifier Optional parameter identifier appended to the
   * export path.
   */
  void export_solution(
      const std::string& path, std::string_view const solutionSubdirectory,
      std::optional<std::string> const& parameter_identifier = {}) const {
    export_solution(std::filesystem::path(path), solutionSubdirectory,
                    parameter_identifier);
  };
  /**
   * @brief Exports the solution to the specified directory and subdirectory.
   *
   * @param path Directory path where the solution will be exported.
   * @param solutionSubdirectory Subdirectory name for organizing solutions.
   * @param parameter_identifier Optional identifier appended to the solution
   * name.
   */
  void export_solution(
      const char* path, std::string_view const solutionSubdirectory,
      std::optional<std::string> const& parameter_identifier = {}) const {
    export_solution(std::filesystem::path(path), solutionSubdirectory,
                    parameter_identifier);
  };

  /**
   * @brief Checks whether the stored solution metadata is consistent.
   *
   * @return `true` if the solution state is consistent, otherwise `false`.
   */
  [[nodiscard]] virtual bool check_consistency() const;

  /** @brief Virtual destructor. */
  virtual ~SolGeneralProblemInstance() = default;

  // Rule of 5 due to virtual deconstructor
  SolGeneralProblemInstance(SolGeneralProblemInstance const&) = default;
  SolGeneralProblemInstance&
  operator=(SolGeneralProblemInstance const&)                     = default;
  SolGeneralProblemInstance(SolGeneralProblemInstance&&) noexcept = default;
  SolGeneralProblemInstance&
  operator=(SolGeneralProblemInstance&&) noexcept = default;
};

class SolGeneralProblemInstanceWithScheduleAndRoutes
    : public SolGeneralProblemInstance {
  RouteMap m_solution_routes{};

  /**
   * @brief Accesses the underlying instance with its specific type.
   * @return Const pointer to the underlying instance as
   * `GeneralProblemInstanceWithScheduleAndRoutes const*`.
   */
  [[nodiscard]] GeneralProblemInstanceWithScheduleAndRoutes const*
  cast_instance() const {
    return dynamic_cast<GeneralProblemInstanceWithScheduleAndRoutes const*>(
        get_uncast_instance_pointer());
  }

protected:
  /**
   * @brief Initializes a solution wrapper for a schedule-and-routes problem
   * instance.
   *
   * @param instance_ptr The problem instance to wrap.
   */
  explicit SolGeneralProblemInstanceWithScheduleAndRoutes(
      std::shared_ptr<GeneralProblemInstanceWithScheduleAndRoutes> instance_ptr)
      : SolGeneralProblemInstance(std::move(instance_ptr)),
        m_solution_routes(cast_instance()->get_const_routes()) {}
  /**
   * @brief Initializes a solution instance with an existing problem instance
   * and solution metadata.
   */
  SolGeneralProblemInstanceWithScheduleAndRoutes(
      std::shared_ptr<GeneralProblemInstanceWithScheduleAndRoutes> instance_ptr,
      SolutionStatus status, double obj, bool has_sol)
      : SolGeneralProblemInstance(std::move(instance_ptr), status, obj,
                                  has_sol),
        m_solution_routes(cast_instance()->get_const_routes()) {};

public:
  using SolGeneralProblemInstance::export_solution;
  using SolGeneralProblemInstance::load_solution;

  /**
   * @brief Creates a solution wrapper from a problem instance.
   */
  explicit SolGeneralProblemInstanceWithScheduleAndRoutes(
      GeneralProblemInstanceWithScheduleAndRoutes const& instance)
      : SolGeneralProblemInstanceWithScheduleAndRoutes(
            std::make_shared<GeneralProblemInstanceWithScheduleAndRoutes>(
                instance)) {};
  /**
   * @brief Initializes a solution wrapper around a copy of the provided
   * instance.
   *
   * @param instance Problem instance to wrap.
   * @param status Solution status.
   * @param obj Objective value of the solution.
   * @param has_sol Whether a solution exists.
   */
  SolGeneralProblemInstanceWithScheduleAndRoutes(
      GeneralProblemInstanceWithScheduleAndRoutes const& instance,
      SolutionStatus status, double obj, bool has_sol)
      : SolGeneralProblemInstanceWithScheduleAndRoutes(
            std::make_shared<GeneralProblemInstanceWithScheduleAndRoutes>(
                instance),
            status, obj, has_sol) {};

  // Rule of 5
  SolGeneralProblemInstanceWithScheduleAndRoutes(
      const SolGeneralProblemInstanceWithScheduleAndRoutes&) = default;
  SolGeneralProblemInstanceWithScheduleAndRoutes&
  operator=(const SolGeneralProblemInstanceWithScheduleAndRoutes&) = default;
  SolGeneralProblemInstanceWithScheduleAndRoutes(
      SolGeneralProblemInstanceWithScheduleAndRoutes&&) noexcept = default;
  SolGeneralProblemInstanceWithScheduleAndRoutes& operator=(
      SolGeneralProblemInstanceWithScheduleAndRoutes&&) noexcept = default;
  ~SolGeneralProblemInstanceWithScheduleAndRoutes() override     = default;

  /** @brief Loads the route-aware part of a solution from disk. */
  void load_solution(
      const std::filesystem::path&      working_directory,
      std::string_view                  solution_subdirectory,
      std::optional<std::string> const& parameter_identifier) override;

  /**
   * @brief Loads a solution from the specified working directory and solution
   * subdirectory.
   *
   * @param working_directory The root directory containing solutions.
   * @param solutionSubdirectory The solution subdirectory within the working
   * directory.
   */
  void load_solution(const std::filesystem::path& working_directory,
                     std::string_view const solutionSubdirectory) override {
    load_solution(working_directory, solutionSubdirectory, {});
  }

  /**
   * @brief Accesses the underlying schedule-and-routes problem instance.
   *
   * @return GeneralProblemInstanceWithScheduleAndRoutes const* Const pointer to
   * the underlying instance.
   */
  [[nodiscard]] GeneralProblemInstanceWithScheduleAndRoutes const*
  get_instance() const override {
    return cast_instance();
  }

  /**
   * @brief Retrieves the solution routes.
   *
   * @return const RouteMap& The solution routes.
   */
  [[nodiscard]] RouteMap const& get_const_solution_routes() const {
    return m_solution_routes;
  }

  // RouteMap functions
  /** @brief Resets solution routes to the instance's fixed routes. */
  void reset_routes();
  /**
   * @brief Adds an empty solution route for a train.
   *
   * @param train_name Name of the train.
   */
  void add_empty_route(const std::string& train_name);

  /**
   * @brief Appends an edge to the end of a train's solution route.
   *
   * @param train_name Name of the train.
   * @param edge_input The edge to append.
   */
  void push_back_edge_to_route(const std::string&        train_name,
                               Network::EdgeInput const& edge_input) {
    m_solution_routes.push_back_edge(train_name, edge_input,
                                     get_instance()->get_const_network());
  }
  /**
   * @brief Adds an edge to the front of the solution route for a train.
   *
   * @param train_name Name of the train whose solution route is modified.
   * @param edge_input The edge to prepend to the route.
   */
  void push_front_edge_to_route(const std::string&        train_name,
                                Network::EdgeInput const& edge_input) {
    m_solution_routes.push_front_edge(train_name, edge_input,
                                      get_instance()->get_const_network());
  }

  /**
   * @brief Removes the first edge from a train's route in the solution.
   *
   * @param train_name The name of the train whose route is to be modified.
   */
  void remove_first_edge_from_route(const std::string& train_name) {
    m_solution_routes.remove_first_edge(train_name);
  }
  /**
   * @brief Removes the last edge from a train's route in the solution.
   *
   * @param train_name Name of the train.
   */
  void remove_last_edge_from_route(const std::string& train_name) {
    m_solution_routes.remove_last_edge(train_name);
  }

  /**
   * @brief Exports the solution without saving the underlying instance.
   *
   * @param working_directory The base directory for solution exports.
   * @param solutionSubdirectory The subdirectory for this solution.
   * @param parameter_identifier Optional identifier to include in the export
   * path.
   */
  void export_solution(
      const std::filesystem::path&      working_directory,
      std::string_view const            solutionSubdirectory,
      std::optional<std::string> const& parameter_identifier) const override {
    export_solution(working_directory, solutionSubdirectory, false,
                    parameter_identifier);
  }
  /**
   * @brief Exports the solution without saving the problem instance.
   *
   * Exports only the solution data (routes) to the specified directory without
   * exporting the underlying problem instance.
   *
   * @param working_directory Root working directory for the solution export.
   * @param solutionSubdirectory Subdirectory within the solutions folder for
   * this solution.
   */
  void
  export_solution(const std::filesystem::path& working_directory,
                  std::string_view const solutionSubdirectory) const override {
    export_solution(working_directory, solutionSubdirectory, false, {});
  }

  virtual void
  export_solution(const std::filesystem::path& working_directory,
                  std::string_view solution_subdirectory, bool save_instance,
                  std::optional<std::string> const& parameter_identifier) const;

  /**
   * @brief Checks whether the solution routes are consistent with the instance.
   *
   * @return `true` if the solution is consistent, otherwise `false`.
   */
  [[nodiscard]] bool check_consistency() const override;
};
} // namespace cda_rail::instances
