#pragma once

#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "EOMHelper.hpp"
#include "GeneralProblemInstance.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Route.hpp"
#include "datastructure/Timetable.hpp"
#include "nlohmann/json.hpp"
#include "nlohmann/json_fwd.hpp"

#include <algorithm>
#include <cassert>
#include <cstdarg>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

// for some reason only works if imported after cstdarg
#include "plog/Util.h"

using json = nlohmann::json;

namespace cda_rail::instances {

class GeneralPerformanceOptimizationInstance
    : public GeneralProblemInstanceWithScheduleAndRoutes {
  friend class SolGeneralPerformanceOptimizationInstance;
  // friend class SolVSSGeneralPerformanceOptimizationInstance;

  std::vector<double> m_train_weights{};
  std::vector<bool>   m_train_optional{};
  double m_station_delay_weight{1}; // add (average) station delays with given
                                    // weight to objective function
  double m_lambda{1}; // Minutes of delay (of a weight one train) that are
  // "equal" to scheduling another weight one train

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
      std::string_view const       instanceName,
      std::string_view const       instanceSubdirectory,
      std::filesystem::path const& workingDirectory);
  GeneralPerformanceOptimizationInstance(
      std::string_view const instanceName,
      std::string_view const instanceSubdirectory,
      std::string const&     workingDirectory)
      : GeneralPerformanceOptimizationInstance(
            instanceName, instanceSubdirectory,
            std::filesystem::path(workingDirectory)) {}
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
  };
  [[nodiscard]] const auto& get_train_optional() const {
    return m_train_optional;
  }
  [[nodiscard]] double get_station_delay_weight() const {
    return m_station_delay_weight;
  }
  [[nodiscard]] double get_lambda() const { return m_lambda; };

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

  [[nodiscard]] bool get_train_optional(size_t train_index) const {
    get_const_timetable().get_train_list().throw_if_train_not_exist(
        train_index);
    return m_train_optional.at(train_index);
  }
  [[nodiscard]] bool get_train_optional(const std::string& train_name) const {
    return get_train_optional(
        get_const_timetable().get_train_list().get_train_index(train_name));
  }
  [[nodiscard]] bool get_train_optional(const char* train_name) const {
    return get_train_optional(
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
  };
  void set_lambda(double new_lambda) { m_lambda = new_lambda; };

  void set_train_weight(size_t train_index, double weight) {
    get_const_timetable().get_train_list().throw_if_train_not_exist(
        train_index);
    m_train_weights.at(train_index) = weight;
  };
  void set_train_weight(const std::string& train_name, double weight) {
    set_train_weight(
        get_const_timetable().get_train_list().get_train_index(train_name),
        weight);
  };
  void set_train_weight(const char* train_name, double weight) {
    set_train_weight(
        get_const_timetable().get_train_list().get_train_index(train_name),
        weight);
  };

  void set_train_optionality_value(size_t train_index, bool val) {
    get_const_timetable().get_train_list().throw_if_train_not_exist(
        train_index);
    m_train_optional.at(train_index) = val;
  };
  void set_train_optionality_value(const std::string& train_name, bool val) {
    set_train_optionality_value(
        get_const_timetable().get_train_list().get_train_index(train_name),
        val);
  };
  void set_train_optionality_value(const char* train_name, bool val) {
    set_train_optionality_value(
        get_const_timetable().get_train_list().get_train_index(train_name),
        val);
  };
  void set_train_optional(size_t train_index) {
    set_train_optionality_value(train_index, true);
  };
  void set_train_optional(const std::string& train_name) {
    set_train_optionality_value(
        get_const_timetable().get_train_list().get_train_index(train_name),
        true);
  };
  void set_train_optional(const char* train_name) {
    set_train_optionality_value(
        get_const_timetable().get_train_list().get_train_index(train_name),
        true);
  };
  void set_train_mandatory(size_t train_index) {
    set_train_optionality_value(train_index, false);
  };
  void set_train_mandatory(const std::string& train_name) {
    set_train_optionality_value(
        get_const_timetable().get_train_list().get_train_index(train_name),
        false);
  };
  void set_train_mandatory(const char* train_name) {
    set_train_optionality_value(
        get_const_timetable().get_train_list().get_train_index(train_name),
        false);
  };

  // Instance addition

  size_t add_train(std::string const& train_name, double length,
                   double max_speed, double acceleration, double deceleration,
                   bool tim, double entry_time, double initial_velocity,
                   Network::VertexInput const& entry_vertex, double exit_time,
                   double                      exit_velocity,
                   Network::VertexInput const& exit_vertex,
                   double tr_weight = 1, bool tr_optional = false) {
    m_train_weights.emplace_back(tr_weight);
    m_train_optional.emplace_back(tr_optional);
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
                   double tr_weight = 1, bool tr_optional = false) {
    return add_train(train_name, length, max_speed, acceleration, deceleration,
                     true, entry_time, initial_velocity, entry_vertex,
                     exit_time, exit_velocity, exit_vertex, tr_weight,
                     tr_optional);
  }

  // ---------------------
  // EXPORT
  // ---------------------

  using GeneralProblemInstanceWithScheduleAndRoutes::export_instance;

  void export_instance(const std::filesystem::path& workingDirectory,
                       bool const                   saveNetwork) const override;

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
    m_train_optional.resize(
        this->get_const_timetable().get_train_list().get_number_of_trains(),
        false);
  }
};

class SolGeneralPerformanceOptimizationInstance
    : public SolGeneralProblemInstanceWithScheduleAndRoutes {
private:
  std::vector<std::map<double, double>> m_train_pos;
  std::vector<std::map<double, double>> m_train_speed;
  std::vector<bool>                     m_train_routed;

  void initialize_vectors();

public:
  using SolGeneralProblemInstanceWithScheduleAndRoutes::export_solution;
  using SolGeneralProblemInstanceWithScheduleAndRoutes::load_solution;

  // Constructor
  explicit SolGeneralPerformanceOptimizationInstance(
      const GeneralPerformanceOptimizationInstance& instance)
      : SolGeneralProblemInstanceWithScheduleAndRoutes(
            std::make_unique<GeneralPerformanceOptimizationInstance>(
                instance)) {
    this->initialize_vectors();
  }
  SolGeneralPerformanceOptimizationInstance(
      const GeneralPerformanceOptimizationInstance& instance,
      SolutionStatus status, double obj, bool has_sol)
      : SolGeneralProblemInstanceWithScheduleAndRoutes(
            std::make_unique<GeneralPerformanceOptimizationInstance>(instance),
            status, obj, has_sol) {
    this->initialize_vectors();
  }

  // Import / Export

  void load_solution(const std::filesystem::path& workingDirectory,
                     std::string_view const solutionSubdirectory) override;

  void
  export_solution(const std::filesystem::path& workingDirectory,
                  std::string_view const solutionSubdirectory) const override {
    export_solution(workingDirectory, solutionSubdirectory, false);
  }

  virtual void export_solution(const std::filesystem::path& workingDirectory,
                               std::string_view const solutionSubdirectory,
                               bool                   export_instance) const;

  // Additional Getter
  [[nodiscard]] GeneralPerformanceOptimizationInstance const*
  get_instance() const override {
    return dynamic_cast<GeneralPerformanceOptimizationInstance const*>(
        SolGeneralProblemInstance::get_instance());
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

  [[nodiscard]] bool get_train_routed(const std::string& tr_name) const;

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
  void set_train_routed(const std::string& tr_name) {
    set_train_routed_value(tr_name, true);
  }
  void set_train_not_routed(const std::string& tr_name) {
    set_train_routed_value(tr_name, false);
  }
  void set_train_routed_value(const std::string& tr_name, bool val);

  // Check solution consistency

  [[nodiscard]] bool check_consistency() const override;
};

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

  void export_solution(const std::filesystem::path& workingDirectory,
                       std::string_view const       solutionSubdirectory,
                       bool export_instance) const override;

  [[nodiscard]] bool check_consistency() const override;

  void load_solution(const std::filesystem::path& workingDirectory,
                     std::string_view const solutionSubdirectory) override;
};

} // namespace cda_rail::instances
