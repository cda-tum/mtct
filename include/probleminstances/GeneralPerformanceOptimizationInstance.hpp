#pragma once

#include "Definitions.hpp"
#include "GeneralProblemInstance.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Route.hpp"
#include "datastructure/Timetable.hpp"
#include "nlohmann/json_fwd.hpp"

#include <cassert>
#include <cstdarg>
#include <cstddef>
#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

// for some reason only works if imported after cstdarg

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
  // -------------------

  GeneralPerformanceOptimizationInstance() = default;
  explicit GeneralPerformanceOptimizationInstance(Network network)
      : GeneralProblemInstanceWithScheduleAndRoutes(std::move(network)) {
    initialize_vectors();
  }
  GeneralPerformanceOptimizationInstance(Network network, Timetable timetable,
                                         RouteMap routes)
      : GeneralProblemInstanceWithScheduleAndRoutes(
            std::move(network), std::move(timetable), std::move(routes)) {
    initialize_vectors();
  }
  GeneralPerformanceOptimizationInstance(
      std::string_view instance_name, std::string_view instance_subdirectory,
      std::filesystem::path const& working_directory);
  GeneralPerformanceOptimizationInstance(
      std::string_view const instanceName,
      std::string_view const instanceSubdirectory,
      std::string const&     working_directory)
      : GeneralPerformanceOptimizationInstance(
            instanceName, instanceSubdirectory,
            std::filesystem::path(working_directory)) {}
  GeneralPerformanceOptimizationInstance(
      std::string_view const instanceName,
      std::string_view const instanceSubdirectory,
      char const* const      workingDirectory)
      : GeneralPerformanceOptimizationInstance(
            instanceName, instanceSubdirectory,
            std::filesystem::path(workingDirectory)) {}

  // ------------------
  // GETTER
  // ------------------

  [[nodiscard]] const auto& get_train_weights() const {
    return m_train_weights;
  }
  [[nodiscard]] double get_station_delay_weight() const {
    return m_station_delay_weight;
  }

  [[nodiscard]] double get_train_weight(size_t train_index) const {
    get_const_timetable().get_train_list().throw_if_train_not_exist(
        train_index);
    return m_train_weights.at(train_index);
  }
  [[nodiscard]] double get_train_weight(const std::string& train_name) const {
    return get_train_weight(
        get_const_timetable().get_train_list().get_train_index(train_name));
  }
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

  // Setter

  void set_station_delay_weight(double new_weight) {
    m_station_delay_weight = new_weight;
  }

  void set_train_weight(size_t train_index, double weight) {
    get_const_timetable().get_train_list().throw_if_train_not_exist(
        train_index);
    m_train_weights.at(train_index) = weight;
  }
  void set_train_weight(const std::string& train_name, double weight) {
    set_train_weight(
        get_const_timetable().get_train_list().get_train_index(train_name),
        weight);
  }
  void set_train_weight(const char* train_name, double weight) {
    set_train_weight(
        get_const_timetable().get_train_list().get_train_index(train_name),
        weight);
  }

  // Instance addition

  size_t add_train(std::string const& train_name, double length,
                   double max_speed, double acceleration, double deceleration,
                   bool tim, double entry_time, double initial_velocity,
                   Network::VertexInput const& entry_vertex, double exit_time,
                   double                      exit_velocity,
                   Network::VertexInput const& exit_vertex,
                   double                      tr_weight = 1) {
    m_train_weights.emplace_back(tr_weight);
    return GeneralProblemInstanceWithScheduleAndRoutes::add_train(
        train_name, length, max_speed, acceleration, deceleration, tim,
        entry_time, initial_velocity, entry_vertex, exit_time, exit_velocity,
        exit_vertex);
  }
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
  [[nodiscard]] double
  get_approximate_leaving_time(const std::string& tr_name) const {
    return get_approximate_leaving_time(
        this->get_const_timetable().get_train_list().get_train_index(tr_name));
  };
  [[nodiscard]] double get_maximal_leaving_time(const std::string& tr_name,
                                                double             v) const {
    return get_maximal_leaving_time(
        this->get_const_timetable().get_train_list().get_train_index(tr_name),
        v);
  };
  [[nodiscard]] double get_minimal_leaving_time(const std::string& tr_name,
                                                double             v) const {
    return get_minimal_leaving_time(
        this->get_const_timetable().get_train_list().get_train_index(tr_name),
        v);
  };

private:
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

  // Constructor
  explicit SolGeneralPerformanceOptimizationInstance(
      const GeneralPerformanceOptimizationInstance& instance)
      : SolGeneralProblemInstanceWithScheduleAndRoutes(
            std::make_shared<GeneralPerformanceOptimizationInstance>(
                instance)) {
    this->initialize_vectors();
  }
  SolGeneralPerformanceOptimizationInstance(
      const GeneralPerformanceOptimizationInstance& instance,
      SolutionStatus status, double obj, bool has_sol)
      : SolGeneralProblemInstanceWithScheduleAndRoutes(
            std::make_shared<GeneralPerformanceOptimizationInstance>(instance),
            status, obj, has_sol) {
    this->initialize_vectors();
  }

  // Rule of 5
  SolGeneralPerformanceOptimizationInstance(
      SolGeneralPerformanceOptimizationInstance const&) = default;
  SolGeneralPerformanceOptimizationInstance&
  operator=(SolGeneralPerformanceOptimizationInstance const&) = default;
  SolGeneralPerformanceOptimizationInstance(
      SolGeneralPerformanceOptimizationInstance&&) noexcept = default;
  SolGeneralPerformanceOptimizationInstance&
  operator=(SolGeneralPerformanceOptimizationInstance&&) noexcept = default;
  ~SolGeneralPerformanceOptimizationInstance() override           = default;

  // Import / Export

  void load_solution(
      const std::filesystem::path&      working_directory,
      std::string_view                  solution_subdirectory,
      std::optional<std::string> const& parameter_identifier) override;

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

  // Additional Getter
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

  [[nodiscard]] double get_time_at_pos(const std::string& tr_name,
                                       double             pos) const;

  // Add train timing information

  void add_train_pos(const std::string& tr_name, double t, double pos);
  void add_train_speed(const std::string& tr_name, double t, double speed);

  // Check solution consistency

  [[nodiscard]] bool check_consistency() const override;
};

// NOLINTNEXTLINE(readability-avoid-unconditional-preprocessor-if)
#if 0
class SolVSSGeneralPerformanceOptimizationInstance
    : public SolGeneralPerformanceOptimizationInstance {
  std::vector<std::vector<double>> m_vss_pos;

  void initialize_vss_vector();

public:
  using SolGeneralPerformanceOptimizationInstance::export_solution;
  using SolGeneralPerformanceOptimizationInstance::load_solution;

  explicit SolVSSGeneralPerformanceOptimizationInstance(
      const GeneralPerformanceOptimizationInstance& instance)
      : SolGeneralPerformanceOptimizationInstance(instance) {
    this->initialize_vss_vector();
  }
  SolVSSGeneralPerformanceOptimizationInstance(
      const GeneralPerformanceOptimizationInstance& instance,
      SolutionStatus status, double obj, bool has_sol)
      : SolGeneralPerformanceOptimizationInstance(instance, status, obj,
                                                  has_sol) {
    this->initialize_vss_vector();
  }

  void add_vss_pos(cda_rail::Network::EdgeInput const& edge_input, double pos,
                   bool reverse_edge = true);

  void set_vss_pos(cda_rail::Network::EdgeInput const& edge_input,
                   std::vector<double>                 pos);

  void reset_vss_pos(cda_rail::Network::EdgeInput const& edge_input);

  void export_solution(const std::filesystem::path&      workingDirectory,
                       std::string_view const            solutionSubdirectory,
                       bool                              export_instance,
                       std::optional<std::string> const& parameter_identifier =
                           {}) const override;

  [[nodiscard]] bool check_consistency() const override;

  void load_solution(
      const std::filesystem::path&      workingDirectory,
      std::string_view const            solutionSubdirectory,
      std::optional<std::string> const& parameter_identifier = {}) override;
};
#endif
} // namespace cda_rail::instances
