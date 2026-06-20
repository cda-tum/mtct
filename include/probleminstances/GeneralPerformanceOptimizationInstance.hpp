#pragma once

#include "Definitions.hpp"
#include "GeneralProblemInstance.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Route.hpp"
#include "datastructure/Timetable.hpp"
#include "nlohmann/json_fwd.hpp"

#include <cassert>
#include <cstddef>
#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

using json = nlohmann::json;

namespace cda_rail::instances {

class GeneralPerformanceOptimizationInstance
    : public GeneralProblemInstanceWithScheduleAndRoutes {
  friend class SolGeneralPerformanceOptimizationInstance;
  // friend class SolVSSGeneralPerformanceOptimizationInstance;

  std::vector<double> m_train_weights{};
  double m_station_delay_weight{1}; // add (average) station delays with given
                                    // weight to objective function

public:
  // -------------------
  // CONSTRUCTOR
  /**
   * @brief Constructs an instance with default values.
   */
  GeneralPerformanceOptimizationInstance() = default;
  /**
   * @brief Constructs an instance from a network.
   *
   * @param network The network for this instance.
   */
  explicit GeneralPerformanceOptimizationInstance(Network network)
      : GeneralProblemInstanceWithScheduleAndRoutes(std::move(network)) {
    initialize_vectors();
  }
  /**
   * @brief Constructs an instance with a network, timetable, and routes.
   *
   * Initializes all train weights to 1.
   */
  GeneralPerformanceOptimizationInstance(Network network, Timetable timetable,
                                         RouteMap routes)
      : GeneralProblemInstanceWithScheduleAndRoutes(
            std::move(network), std::move(timetable), std::move(routes)) {
    initialize_vectors();
  }
  GeneralPerformanceOptimizationInstance(
      std::string_view instance_name, std::string_view instance_subdirectory,
      std::filesystem::path const& working_directory);
  /**
   * @brief Constructs an instance from a named subdirectory.
   *
   * @param instanceName The name of the instance to load.
   * @param instanceSubdirectory The subdirectory containing the instance data.
   * @param working_directory The working directory containing the instance
   * subdirectory.
   */
  GeneralPerformanceOptimizationInstance(
      std::string_view const instanceName,
      std::string_view const instanceSubdirectory,
      std::string const&     working_directory)
      : GeneralPerformanceOptimizationInstance(
            instanceName, instanceSubdirectory,
            std::filesystem::path(working_directory)) {}
  /**
   * @brief Constructs an instance by loading from a specified directory
   * structure.
   *
   * @param instanceName Name identifying the instance to load.
   * @param instanceSubdirectory Subdirectory (within `workingDirectory`)
   * containing instance data.
   * @param workingDirectory Root directory path for loading instance data.
   */
  GeneralPerformanceOptimizationInstance(
      std::string_view const instanceName,
      std::string_view const instanceSubdirectory,
      char const* const      workingDirectory)
      : GeneralPerformanceOptimizationInstance(
            instanceName, instanceSubdirectory,
            std::filesystem::path(workingDirectory)) {}

  // ------------------
  // GETTER
  /**
   * @brief Accesses the per-train weights.
   *
   * @return A const reference to the vector of per-train weights, indexed by
   * train.
   */

  [[nodiscard]] const auto& get_train_weights() const {
    return m_train_weights;
  }
  /**
   * @brief Gets the station delay weight.
   *
   * @return double The weight applied to station delays in the objective
   * function.
   */
  [[nodiscard]] double get_station_delay_weight() const {
    return m_station_delay_weight;
  }

  /**
   * @brief Retrieves the weight assigned to a specific train.
   *
   * @param train_index Index of the train.
   * @return double The weight for the specified train.
   */
  [[nodiscard]] double get_train_weight(size_t train_index) const {
    get_const_timetable().get_train_list().throw_if_train_not_exist(
        train_index);
    return m_train_weights.at(train_index);
  }
  /**
   * @brief Retrieves the weight of a train by name.
   *
   * @param train_name The name of the train.
   * @return double The weight assigned to the train.
   */
  [[nodiscard]] double get_train_weight(const std::string& train_name) const {
    return get_train_weight(
        get_const_timetable().get_train_list().get_train_index(train_name));
  }
  /**
   * @brief Returns the weight of a train by its name.
   *
   * @param train_name The name of the train.
   * @return double The weight of the specified train.
   */
  [[nodiscard]] double get_train_weight(const char* train_name) const {
    return get_train_weight(
        get_const_timetable().get_train_list().get_train_index(train_name));
  }

  // Objective

  [[nodiscard]] double
  get_objective_val(const std::vector<double>&              tr_exit_times,
                    const std::vector<std::vector<double>>& stop_times,
                    bool throw_error_if_not_all_stops_specified = true) const;

  // -------------------
  // EDITING
  // -------------------

  /**
   * @brief Sets the station-delay weight.
   *
   * @param new_weight The weight value to assign.
   */

  void set_station_delay_weight(double new_weight) {
    m_station_delay_weight = new_weight;
  }

  /**
   * @brief Assigns a weight to a specific train.
   *
   * @param train_index Index of the train.
   * @param weight The weight value.
   */
  void set_train_weight(size_t train_index, double weight) {
    get_const_timetable().get_train_list().throw_if_train_not_exist(
        train_index);
    m_train_weights.at(train_index) = weight;
  }
  /**
   * @brief Sets the weight for a train by its name.
   * @param train_name Name of the train.
   * @param weight Weight to assign to the train.
   */
  void set_train_weight(const std::string& train_name, double weight) {
    set_train_weight(
        get_const_timetable().get_train_list().get_train_index(train_name),
        weight);
  }
  /**
   * @brief Sets the weight for a train by name.
   *
   * @param train_name The name of the train.
   * @param weight The new weight value.
   */
  void set_train_weight(const char* train_name, double weight) {
    set_train_weight(
        get_const_timetable().get_train_list().get_train_index(train_name),
        weight);
  }

  /**
   * @brief Adds a train with an associated weight to the instance.
   *
   * @param tim Whether the train operates under timetable constraints.
   * @param tr_weight Weight coefficient for this train in the objective
   * (default 1).
   * @return Index of the newly added train.
   */

  size_t add_train(std::string const& train_name, double length,
                   double max_speed, double acceleration, double deceleration,
                   bool tim, double entry_time, double initial_velocity,
                   Network::VertexInput const& entry_vertex, double exit_time,
                   double                      exit_velocity,
                   Network::VertexInput const& exit_vertex,
                   double                      tr_weight = 1) {
    auto index = GeneralProblemInstanceWithScheduleAndRoutes::add_train(
        train_name, length, max_speed, acceleration, deceleration, tim,
        entry_time, initial_velocity, entry_vertex, exit_time, exit_velocity,
        exit_vertex);
    m_train_weights.emplace_back(tr_weight);
    return index;
  }
  /**
   * @brief Adds a train to the instance with specified characteristics and
   * weight.
   *
   * @param train_name Name of the train.
   * @param length Length of the train.
   * @param max_speed Maximum speed of the train.
   * @param acceleration Acceleration capability of the train.
   * @param deceleration Deceleration capability of the train.
   * @param entry_time Time at which the train enters the network.
   * @param initial_velocity Initial velocity of the train.
   * @param entry_vertex Entry vertex for the train.
   * @param exit_time Time at which the train exits the network.
   * @param exit_velocity Exit velocity of the train.
   * @param exit_vertex Exit vertex for the train.
   * @param tr_weight Per-train weight factor for objective calculation
   * (default: 1).
   * @return The index of the added train.
   */
  size_t add_train(std::string const& train_name, double length,
                   double max_speed, double acceleration, double deceleration,
                   double entry_time, double initial_velocity,
                   Network::VertexInput const& entry_vertex, double exit_time,
                   double                      exit_velocity,
                   Network::VertexInput const& exit_vertex,
                   double                      tr_weight = 1) {
    return add_train(train_name, length, max_speed, acceleration, deceleration,
                     true, entry_time, initial_velocity, entry_vertex,
                     exit_time, exit_velocity, exit_vertex, tr_weight);
  }

  // ---------------------
  // EXPORT
  // ---------------------

  using GeneralProblemInstanceWithScheduleAndRoutes::export_instance;

  void export_instance(const std::filesystem::path& working_directory,
                       bool save_network) const override;

  // ---------------------
  // TRANSFORMATION
  // ---------------------

  // Transformation functions
  void discretize_stops();

  /**
   * @brief Checks if the instance is consistent.
   *
   * @return `true` if the instance is consistent and every train has a route,
   * `false` otherwise.
   */
  [[nodiscard]] bool check_consistency() const override {
    return check_consistency(true);
  }

  [[nodiscard]] bool
  check_consistency(bool every_train_must_have_route) const override;

  // -----------------
  // HELPER
  // -----------------

  [[nodiscard]] double get_approximate_leaving_time(size_t train) const;
  [[nodiscard]] double get_maximal_leaving_time(size_t train, double v) const;
  [[nodiscard]] double get_minimal_leaving_time(size_t train, double v) const;
  /**
   * @brief Gets the approximate leaving time for a train by name.
   * @param tr_name The name of the train.
   * @return double The approximate leaving time of the train.
   */
  [[nodiscard]] double
  get_approximate_leaving_time(const std::string& tr_name) const {
    return get_approximate_leaving_time(
        this->get_const_timetable().get_train_list().get_train_index(tr_name));
  };
  /**
   * @brief Determines the maximum departure time for a train at a specified
   * velocity.
   *
   * @param v Velocity constraint.
   * @return Maximum departure time.
   */
  [[nodiscard]] double get_maximal_leaving_time(const std::string& tr_name,
                                                double             v) const {
    return get_maximal_leaving_time(
        this->get_const_timetable().get_train_list().get_train_index(tr_name),
        v);
  };
  /**
   * @brief Gets the minimal leaving time for a train.
   *
   * @param v Constraint value for the calculation.
   * @return double The minimal leaving time for the specified train.
   */
  [[nodiscard]] double get_minimal_leaving_time(const std::string& tr_name,
                                                double             v) const {
    return get_minimal_leaving_time(
        this->get_const_timetable().get_train_list().get_train_index(tr_name),
        v);
  };

private:
  /**
   * @brief Initializes per-train weights to default values.
   *
   * Each train is assigned a weight of 1.
   */
  void initialize_vectors() {
    m_train_weights.resize(
        this->get_const_timetable().get_train_list().get_number_of_trains(), 1);
  }
};

class SolGeneralPerformanceOptimizationInstance
    : public SolGeneralProblemInstanceWithScheduleAndRoutes {
private:
  std::vector<std::map<double, double>> m_train_pos;
  std::vector<std::map<double, double>> m_train_speed;

  void initialize_vectors();

public:
  using SolGeneralProblemInstanceWithScheduleAndRoutes::export_solution;
  using SolGeneralProblemInstanceWithScheduleAndRoutes::load_solution;

  /**
   * @brief Initializes a solution object for a performance optimization problem
   * instance.
   *
   * @param instance The problem instance to associate with this solution.
   */
  explicit SolGeneralPerformanceOptimizationInstance(
      const GeneralPerformanceOptimizationInstance& instance)
      : SolGeneralProblemInstanceWithScheduleAndRoutes(
            std::make_shared<GeneralPerformanceOptimizationInstance>(
                instance)) {
    this->initialize_vectors();
  }
  /**
   * @brief Constructs a solution instance with metadata for the given
   * optimization problem.
   *
   * @param instance The optimization problem instance to solve.
   * @param status The solution status.
   * @param obj The objective value.
   * @param has_sol Whether a valid solution has been obtained.
   */
  SolGeneralPerformanceOptimizationInstance(
      const GeneralPerformanceOptimizationInstance& instance,
      SolutionStatus status, double obj, bool has_sol)
      : SolGeneralProblemInstanceWithScheduleAndRoutes(
            std::make_shared<GeneralPerformanceOptimizationInstance>(instance),
            status, obj, has_sol) {
    this->initialize_vectors();
  }

  /**
   * @brief Constructs a copy of an existing solution instance.
   */
  SolGeneralPerformanceOptimizationInstance(
      SolGeneralPerformanceOptimizationInstance const&) = default;
  SolGeneralPerformanceOptimizationInstance&
  operator=(SolGeneralPerformanceOptimizationInstance const&) = default;
  SolGeneralPerformanceOptimizationInstance(
      SolGeneralPerformanceOptimizationInstance&&) noexcept = default;
  SolGeneralPerformanceOptimizationInstance&
  operator=(SolGeneralPerformanceOptimizationInstance&&) noexcept = default;
  /**
   * @brief Virtual destructor.
   */
  ~SolGeneralPerformanceOptimizationInstance() override = default;

  // Import / Export

  void load_solution(
      const std::filesystem::path&      working_directory,
      std::string_view                  solution_subdirectory,
      std::optional<std::string> const& parameter_identifier) override;

  /**
   * @brief Exports the solution without saving the instance.
   */
  void export_solution(
      const std::filesystem::path&      working_directory,
      std::string_view const            solutionSubdirectory,
      std::optional<std::string> const& parameter_identifier) const override {
    export_solution(working_directory, solutionSubdirectory, false,
                    parameter_identifier);
  }

  void export_solution(
      const std::filesystem::path& working_directory,
      std::string_view solution_subdirectory, bool save_instance,
      std::optional<std::string> const& parameter_identifier) const override;

  /**
   * @brief Retrieves the underlying problem instance.
   *
   * @return Pointer to the underlying GeneralPerformanceOptimizationInstance.
   */
  [[nodiscard]] GeneralPerformanceOptimizationInstance const*
  get_instance() const override {
    return dynamic_cast<GeneralPerformanceOptimizationInstance const*>(
        get_uncast_instance_pointer());
  }

  // Problem Specific Getters

  [[nodiscard]] double get_train_pos(const std::string& tr_name,
                                     double             t) const;

  struct EdgeTimeBound {
    size_t edge_id;
    double previous_time_step;
    double next_time_step;
  };
  [[nodiscard]] EdgeTimeBound
  get_edge_and_time_bounds(const std::string& tr_name, double t) const;

  struct PosBound {
    double lb;
    double ub;
  };
  struct VelBound {
    double lb;
    double ub;
  };
  struct PosVelBound {
    PosBound pos;
    VelBound vel;
  };
  [[nodiscard]] PosVelBound
  get_exact_pos_and_vel_bounds(const std::string& tr_name, double t) const;

  struct PosVel {
    double pos;
    double vel;
  };
  [[nodiscard]] std::optional<PosVel>
  get_approximate_train_pos_and_vel(const std::string& tr_name, double t) const;

  [[nodiscard]] double get_train_speed(const std::string& tr_name,
                                       double             t) const;

  [[nodiscard]] std::vector<double>
  get_train_times(const std::string& tr_name) const;

  [[nodiscard]] cda_rail::index_vector get_train_order(size_t edge_index) const;

  struct TrainDirection {
    size_t train_id;
    bool   original_direction;
  };
  [[nodiscard]] std::vector<TrainDirection>
  get_train_order_with_reverse(size_t edge_index) const;

  [[nodiscard]] double get_time_at_pos(const std::string& tr_name, double pos,
                                       bool lb = false) const;

  // Add train timing information

  void add_train_pos(const std::string& tr_name, double t, double pos);
  void add_train_speed(const std::string& tr_name, double t, double speed);

  // Check solution consistency

  [[nodiscard]] bool check_consistency() const override;
};
} // namespace cda_rail::instances
