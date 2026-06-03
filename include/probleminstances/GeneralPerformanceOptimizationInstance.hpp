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

  using GeneralProblemInstance::export_instance;

  void
  export_instance(const std::filesystem::path& workingDirectory) const override;

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

  void export_solution(const std::filesystem::path& workingDirectory,
                       std::string_view const       solutionSubdirectory,
                       bool                         export_instance) const;

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

#if 0

template <typename T>
class SolVSSGeneralPerformanceOptimizationInstance
    : public SolGeneralPerformanceOptimizationInstance<T> {
  static_assert(
      std::is_base_of_v<GeneralPerformanceOptimizationInstance, T>,
      "T must be derived from GeneralPerformanceOptimizationInstance");

  std::vector<std::vector<double>> vss_pos;

public:
  SolVSSGeneralPerformanceOptimizationInstance() = default;
  explicit SolVSSGeneralPerformanceOptimizationInstance(
      const GeneralPerformanceOptimizationInstance& instance)
      : SolGeneralPerformanceOptimizationInstance<T>(instance) {
    vss_pos = std::vector<std::vector<double>>(
        this->instance.const_n().number_of_edges());
  };
  SolVSSGeneralPerformanceOptimizationInstance(
      const GeneralPerformanceOptimizationInstance& instance,
      SolutionStatus status, double obj, bool has_sol)
      : SolGeneralPerformanceOptimizationInstance<T>(instance, status, obj,
                                                     has_sol) {
    vss_pos = std::vector<std::vector<double>>(
        this->instance.const_n().number_of_edges());
  };
  explicit SolVSSGeneralPerformanceOptimizationInstance(
      const std::filesystem::path& p,
      const std::optional<T>&      instance = std::optional<T>())
      : SolGeneralPerformanceOptimizationInstance<T>(p, instance) {
    vss_pos = std::vector<std::vector<double>>(
        this->instance.const_n().number_of_edges());
  };

  void add_vss_pos(size_t edge_id, double pos, bool reverse_edge = true) {
    // Add VSS position on edge. Also on reverse edge if true.

    if (!this->instance.const_n().has_edge(edge_id)) {
      throw exceptions::EdgeNotExistentException(edge_id);
    }

    const auto& edge = this->instance.const_n().get_edge(edge_id);

    if (pos <= EPS || pos + EPS >= edge.length) {
      throw exceptions::ConsistencyException(
          "VSS position " + std::to_string(pos) + " is not on edge " +
          std::to_string(edge_id));
    }

    vss_pos.at(edge_id).emplace_back(pos);
    std::sort(vss_pos.at(edge_id).begin(), vss_pos.at(edge_id).end());

    if (reverse_edge) {
      const auto reverse_edge_index =
          this->instance.const_n().get_reverse_edge_index(edge_id);
      if (reverse_edge_index.has_value()) {
        vss_pos.at(reverse_edge_index.value()).emplace_back(edge.length - pos);
        std::sort(vss_pos.at(reverse_edge_index.value()).begin(),
                  vss_pos.at(reverse_edge_index.value()).end());
      }
    }
  };
  void add_vss_pos(size_t source, size_t target, double pos,
                   bool reverse_edge = true) {
    add_vss_pos(this->instance.const_n().get_edge_index(source, target), pos,
                reverse_edge);
  };
  void add_vss_pos(const std::string& source, const std::string& target,
                   double pos, bool reverse_edge = true) {
    add_vss_pos(this->instance.const_n().get_edge_index(source, target), pos,
                reverse_edge);
  };

  void set_vss_pos(size_t edge_id, std::vector<double> pos) {
    if (!this->instance.const_n().has_edge(edge_id)) {
      throw exceptions::EdgeNotExistentException(edge_id);
    }

    const auto& edge = this->instance.const_n().get_edge(edge_id);

    for (const auto& p : pos) {
      if (p <= EPS || p + EPS >= edge.length) {
        throw exceptions::ConsistencyException(
            "VSS position " + std::to_string(p) + " is not on edge " +
            std::to_string(edge_id));
      }
    }

    vss_pos.at(edge_id) = std::move(pos);
  };
  void set_vss_pos(size_t source, size_t target, std::vector<double> pos) {
    set_vss_pos(this->instance.const_n().get_edge_index(source, target),
                std::move(pos));
  };
  void set_vss_pos(const std::string& source, const std::string& target,
                   std::vector<double> pos) {
    set_vss_pos(this->instance.const_n().get_edge_index(source, target),
                std::move(pos));
  };

  void reset_vss_pos(size_t edge_id) {
    if (!this->instance.const_n().has_edge(edge_id)) {
      throw exceptions::EdgeNotExistentException(edge_id);
    }

    vss_pos.at(edge_id).clear();
  };
  void reset_vss_pos(size_t source, size_t target) {
    reset_vss_pos(this->instance.const_n().get_edge_index(source, target));
  };
  void reset_vss_pos(const std::string& source, const std::string& target) {
    reset_vss_pos(this->const_n().get_edge_index(source, target));
  };

  void export_solution(const std::filesystem::path& p,
                       bool export_instance) const override {
    SolGeneralPerformanceOptimizationInstance<T>::export_solution(
        p, export_instance);

    // NOLINTNEXTLINE(misc-const-correctness)
    json vss_pos_json;
    for (size_t edge_id = 0;
         edge_id < this->instance.const_n().number_of_edges(); ++edge_id) {
      const auto& edge = this->instance.const_n().get_edge(edge_id);
      const auto& v0   = this->instance.const_n().get_vertex(edge.source).name;
      const auto& v1   = this->instance.const_n().get_vertex(edge.target).name;
      vss_pos_json["('" + v0 + "', '" + v1 + "')"] = vss_pos.at(edge_id);
    }

    std::ofstream vss_pos_file(p / "solution" / "vss_pos.json");
    vss_pos_file << vss_pos_json << '\n';
    vss_pos_file.close();
  };
  [[nodiscard]] bool check_consistency() const override {
    if (!SolGeneralPerformanceOptimizationInstance<T>::check_consistency()) {
      return false;
    }
    for (size_t edge_id = 0; edge_id < vss_pos.size(); ++edge_id) {
      const auto& edge = this->instance.const_n().get_edge(edge_id);
      if (!edge.breakable && !vss_pos.at(edge_id).empty()) {
        return false;
      }
      for (const auto& pos : vss_pos.at(edge_id)) {
        if (pos + EPS < 0 || pos > edge.length + EPS) {
          return false;
        }
      }
    }
    return true;
  };

  [[nodiscard]] static SolVSSGeneralPerformanceOptimizationInstance
  import_solution(const std::filesystem::path& p,
                  const std::optional<T>&      instance = std::optional<T>()) {
    auto sol = SolVSSGeneralPerformanceOptimizationInstance(p, instance);
    if (!sol.check_consistency()) {
      throw exceptions::ConsistencyException(
          "Imported solution object is not consistent");
    }
    return sol;
  };
  [[nodiscard]] static SolVSSGeneralPerformanceOptimizationInstance
  import_solution(const std::string&      path,
                  const std::optional<T>& instance = std::optional<T>()) {
    return import_solution(std::filesystem::path(path), instance);
  };
  [[nodiscard]] static SolVSSGeneralPerformanceOptimizationInstance
  import_solution(const char*             path,
                  const std::optional<T>& instance = std::optional<T>()) {
    return import_solution(std::filesystem::path(path), instance);
  };
};

#endif

} // namespace cda_rail::instances
