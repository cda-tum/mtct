#pragma once

#include "Definitions.hpp"
#include "EOMHelper.hpp"
#include "GeneralProblemInstance.hpp"
#include "VSSGenerationTimetable.hpp"
#include "datastructure/GeneralTimetable.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Route.hpp"
#include "nlohmann/json.hpp"

#include <cassert>
#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

using json = nlohmann::json;

namespace cda_rail::instances {

class GeneralPerformanceOptimizationInstance
    : public GeneralProblemInstanceWithScheduleAndRoutes<
          GeneralTimetable<GeneralSchedule<GeneralScheduledStop>>> {
  template <typename S> friend class SolGeneralPerformanceOptimizationInstance;
  template <typename S>
  friend class SolVSSGeneralPerformanceOptimizationInstance;

  void initialize_vectors() {
    train_weights =
        std::vector<double>(this->get_timetable().get_train_list().size(), 1);
    train_optional =
        std::vector<bool>(this->get_timetable().get_train_list().size(), false);
  }

  std::vector<double> train_weights;
  std::vector<bool>   train_optional;
  double lambda = 1; // Minutes of delay (of a weight one train) that are
                     // "equal" to scheduling another weight one train

public:
  GeneralPerformanceOptimizationInstance() = default;
  explicit GeneralPerformanceOptimizationInstance(const Network& network)
      : GeneralProblemInstanceWithScheduleAndRoutes<
            GeneralTimetable<GeneralSchedule<GeneralScheduledStop>>>(network) {
    initialize_vectors();
  };
  explicit GeneralPerformanceOptimizationInstance(
      const Network&                                                 network,
      const GeneralTimetable<GeneralSchedule<GeneralScheduledStop>>& timetable,
      const RouteMap&                                                routes)
      : GeneralProblemInstanceWithScheduleAndRoutes<
            GeneralTimetable<GeneralSchedule<GeneralScheduledStop>>>(
            network, timetable, routes) {
    initialize_vectors();
  };
  explicit GeneralPerformanceOptimizationInstance(
      const std::filesystem::path& path)
      : GeneralProblemInstanceWithScheduleAndRoutes<
            GeneralTimetable<GeneralSchedule<GeneralScheduledStop>>>(path) {
    initialize_vectors();

    std::ifstream file(path / "problem_data.json");
    json          j = json::parse(file);
    for (const auto& [train_name, weight] : j["train_weights"].items()) {
      set_train_weight(train_name, static_cast<double>(weight));
    }
    for (const auto& [train_name, optional] : j["train_optional"].items()) {
      set_train_optionality_value(train_name, static_cast<bool>(optional));
    }
    lambda = static_cast<double>(j["lambda"]);
  };

  static GeneralPerformanceOptimizationInstance
  cast_from_vss_generation(const VSSGenerationTimetable& vss_gen);
  [[nodiscard]] VSSGenerationTimetable
  cast_to_vss_generation(bool throw_error = true) const;

  using T = GeneralTimetable<GeneralSchedule<GeneralScheduledStop>>;

  template <typename TrainN = std::string, typename EntryN = std::string,
            typename ExitN = std::string>
  size_t add_train(const TrainN& name, int length, double max_speed,
                   double acceleration, double deceleration,
                   decltype(T::time_type()) t_0, double v_0,
                   const EntryN& entry, decltype(T::time_type()) t_n,
                   double v_n, const ExitN& exit, double tr_weight = 1,
                   bool tr_optional = false) {
    train_weights.push_back(tr_weight);
    train_optional.push_back(tr_optional);
    return GeneralProblemInstanceWithScheduleAndRoutes<T>::add_train(
        name, length, max_speed, acceleration, deceleration, t_0, v_0, entry,
        t_n, v_n, exit);
  }

  [[nodiscard]] const auto& get_train_weights() const { return train_weights; };
  [[nodiscard]] const auto& get_train_optional() const {
    return train_optional;
  };
  [[nodiscard]] double get_lambda() const { return lambda; };

  [[nodiscard]] double get_train_weight(size_t train_index) {
    if (!this->get_timetable().get_train_list().has_train(train_index)) {
      throw std::invalid_argument("Train index out of bounds");
    }
    return train_weights.at(train_index);
  }
  [[nodiscard]] double get_train_weight(const std::string& train_name) {
    return get_train_weight(
        this->get_timetable().get_train_list().get_train_index(train_name));
  }
  [[nodiscard]] double get_train_weight(const char* train_name) {
    return get_train_weight(
        this->get_timetable().get_train_list().get_train_index(train_name));
  }

  [[nodiscard]] bool get_train_optional(size_t train_index) {
    if (!this->get_timetable().get_train_list().has_train(train_index)) {
      throw std::invalid_argument("Train index out of bounds");
    }
    return train_optional.at(train_index);
  }
  [[nodiscard]] bool get_train_optional(const std::string& train_name) {
    return get_train_optional(
        this->get_timetable().get_train_list().get_train_index(train_name));
  }
  [[nodiscard]] bool get_train_optional(const char* train_name) {
    return get_train_optional(
        this->get_timetable().get_train_list().get_train_index(train_name));
  }

  void set_lambda(double new_lambda) { lambda = new_lambda; };

  void set_train_weight(size_t train_index, double weight) {
    if (!this->get_timetable().get_train_list().has_train(train_index)) {
      throw std::invalid_argument("Train index out of bounds");
    }
    train_weights[train_index] = weight;
  };
  void set_train_weight(const std::string& train_name, double weight) {
    set_train_weight(
        this->get_timetable().get_train_list().get_train_index(train_name),
        weight);
  };
  void set_train_weight(const char* train_name, double weight) {
    set_train_weight(
        this->get_timetable().get_train_list().get_train_index(train_name),
        weight);
  };

  void set_train_optionality_value(size_t train_index, bool val) {
    if (!this->get_timetable().get_train_list().has_train(train_index)) {
      throw std::invalid_argument("Train index out of bounds");
    }
    train_optional[train_index] = val;
  };
  void set_train_optionality_value(const std::string& train_name, bool val) {
    set_train_optionality_value(
        this->get_timetable().get_train_list().get_train_index(train_name),
        val);
  };
  void set_train_optionality_value(const char* train_name, bool val) {
    set_train_optionality_value(
        this->get_timetable().get_train_list().get_train_index(train_name),
        val);
  };
  void set_train_optional(size_t train_index) {
    set_train_optionality_value(train_index, true);
  };
  void set_train_optional(const std::string& train_name) {
    set_train_optionality_value(
        this->get_timetable().get_train_list().get_train_index(train_name),
        true);
  };
  void set_train_optional(const char* train_name) {
    set_train_optionality_value(
        this->get_timetable().get_train_list().get_train_index(train_name),
        true);
  };
  void set_train_mandatory(size_t train_index) {
    set_train_optionality_value(train_index, false);
  };
  void set_train_mandatory(const std::string& train_name) {
    set_train_optionality_value(
        this->get_timetable().get_train_list().get_train_index(train_name),
        false);
  };
  void set_train_mandatory(const char* train_name) {
    set_train_optionality_value(
        this->get_timetable().get_train_list().get_train_index(train_name),
        false);
  };

  // Transformation functions
  void discretize_stops();

  using GeneralProblemInstance::export_instance;

  void export_instance(const std::filesystem::path& path) const override {
    GeneralProblemInstanceWithScheduleAndRoutes<GeneralTimetable<
        GeneralSchedule<GeneralScheduledStop>>>::export_instance(path);

    json j;
    for (size_t i = 0; i < train_weights.size(); ++i) {
      j["train_weights"]
       [this->get_timetable().get_train_list().get_train(i).name] =
           train_weights[i];
      j["train_optional"]
       [this->get_timetable().get_train_list().get_train(i).name] =
           train_optional[i];
    }
    j["lambda"] = lambda;

    std::ofstream file(path / "problem_data.json");
    file << j << std::endl;
  };

  [[nodiscard]] bool check_consistency() const override {
    return check_consistency(true);
  }

  [[nodiscard]] bool
  check_consistency(bool every_train_must_have_route) const override {
    if (!GeneralProblemInstanceWithScheduleAndRoutes<
            GeneralTimetable<GeneralSchedule<GeneralScheduledStop>>>::
            check_consistency(every_train_must_have_route)) {
      return false;
    }
    const auto num_tr = this->get_timetable().get_train_list().size();
    if (train_weights.size() != num_tr || train_optional.size() != num_tr) {
      return false;
    }
    return (lambda >= 0 &&
            std::all_of(train_weights.begin(), train_weights.end(),
                        [](double w) { return w >= 0; }));
  };

  [[nodiscard]] double get_approximate_leaving_time(size_t train) const;
  [[nodiscard]] double get_maximal_leaving_time(size_t train, double v) const;
  [[nodiscard]] double get_minimal_leaving_time(size_t train, double v) const;
  [[nodiscard]] double
  get_approximate_leaving_time(const std::string& tr_name) const {
    return get_approximate_leaving_time(
        this->get_timetable().get_train_list().get_train_index(tr_name));
  };
  [[nodiscard]] double get_maximal_leaving_time(const std::string& tr_name,
                                                double             v) const {
    return get_maximal_leaving_time(
        this->get_timetable().get_train_list().get_train_index(tr_name), v);
  };
  [[nodiscard]] double get_minimal_leaving_time(const std::string& tr_name,
                                                double             v) const {
    return get_minimal_leaving_time(
        this->get_timetable().get_train_list().get_train_index(tr_name), v);
  };
};

template <typename T>
class SolGeneralPerformanceOptimizationInstance
    : public SolGeneralProblemInstanceWithScheduleAndRoutes<T> {
  static_assert(
      std::is_base_of<GeneralPerformanceOptimizationInstance, T>::value,
      "T must be derived from GeneralPerformanceOptimizationInstance");
  std::vector<std::map<double, double>> train_pos;
  std::vector<std::map<double, double>> train_speed;
  std::vector<bool>                     train_routed;

  void initialize_vectors() {
    train_pos.reserve(this->instance.get_timetable().get_train_list().size());
    train_speed.reserve(this->instance.get_timetable().get_train_list().size());
    train_routed = std::vector<bool>(
        this->instance.get_timetable().get_train_list().size(), false);
    for (size_t tr = 0; tr < this->instance.get_train_list().size(); ++tr) {
      train_pos.emplace_back();
      train_speed.emplace_back();
    }
  };

public:
  SolGeneralPerformanceOptimizationInstance() = default;
  explicit SolGeneralPerformanceOptimizationInstance(const T& instance)
      : SolGeneralProblemInstanceWithScheduleAndRoutes<T>(instance) {
    this->initialize_vectors();
  };
  SolGeneralPerformanceOptimizationInstance(const T&       instance,
                                            SolutionStatus status, double obj,
                                            bool has_sol)
      : SolGeneralProblemInstanceWithScheduleAndRoutes<T>(instance, status, obj,
                                                          has_sol) {
    this->initialize_vectors();
  };
  explicit SolGeneralPerformanceOptimizationInstance(
      const std::filesystem::path& p,
      const std::optional<T>&      instance = std::optional<T>()) {
    if (!std::filesystem::exists(p)) {
      throw exceptions::ImportException("Path does not exist");
    }
    if (!std::filesystem::is_directory(p)) {
      throw exceptions::ImportException("Path is not a directory");
    }

    bool const import_routes = instance.has_value();
    if (instance.has_value()) {
      this->instance = instance.value();
    } else {
      this->instance = GeneralPerformanceOptimizationInstance(p / "instance");
    }

    if (import_routes) {
      this->instance.editable_routes() =
          RouteMap(p / "instance" / "routes", this->instance.const_n());
    }

    std::ifstream data_file(p / "solution" / "data.json");
    json          data = json::parse(data_file);
    SolGeneralProblemInstanceWithScheduleAndRoutes<
        GeneralPerformanceOptimizationInstance>::
        set_general_solution_data(data);

    this->initialize_vectors();

    // Read train_pos
    std::ifstream train_pos_file(p / "solution" / "train_pos.json");
    json          train_pos_json = json::parse(train_pos_file);
    for (const auto& [tr_name, tr_pos_json] : train_pos_json.items()) {
      for (const auto& [idx, pos_pair] : tr_pos_json.items()) {
        const auto [t, pos] =
            pos_pair.template get<std::pair<double, double>>();
        this->add_train_pos(tr_name, t, pos);
      }
    }

    // Read train_speed
    std::ifstream train_speed_file(p / "solution" / "train_speed.json");
    json          train_speed_json = json::parse(train_speed_file);
    for (const auto& [tr_name, tr_speed_json] : train_speed_json.items()) {
      for (const auto& [idx, speed_pair] : tr_speed_json.items()) {
        const auto [t, speed] =
            speed_pair.template get<std::pair<double, double>>();
        this->add_train_speed(tr_name, t, speed);
      }
    }

    // Read train_routed
    std::ifstream train_routed_file(p / "solution" / "train_routed.json");
    json          train_routed_json = json::parse(train_routed_file);
    for (const auto& [tr_name, routed] : train_routed_json.items()) {
      this->train_routed[this->instance.get_train_list().get_train_index(
          tr_name)] = routed.template get<bool>();
    }
  };

  [[nodiscard]] double get_train_pos(const std::string& tr_name,
                                     double             t) const {
    if (!this->instance.get_train_list().has_train(tr_name)) {
      throw exceptions::TrainNotExistentException(tr_name);
    }
    const auto tr_id = this->instance.get_train_list().get_train_index(tr_name);
    if (train_pos.at(tr_id).count(t) > 0) {
      return train_pos.at(tr_id).at(t);
    }
    throw exceptions::ConsistencyException("No position for train " + tr_name +
                                           " at time " + std::to_string(t));
  };
  [[nodiscard]] std::tuple<size_t, double, double>
  get_edge_and_time_bounds(const std::string& tr_name, double t) const {
    if (!this->instance.get_train_list().has_train(tr_name)) {
      throw exceptions::TrainNotExistentException(tr_name);
    }
    const auto tr_id = this->instance.get_train_list().get_train_index(tr_name);
    const auto& tr_pos = train_pos.at(tr_id);

    double t0 = -1;
    double t1 = -1;
    for (const auto& [time, pos] : tr_pos) {
      if (time == t) {
        t0 = time;
        t1 = time;
        break;
      }
      if (time < t) {
        t0 = time;
      } else if (t0 >= 0) {
        t1 = time;
        break;
      } else {
        throw exceptions::ConsistencyException(
            "Train " + tr_name + " not present at time " + std::to_string(t));
      }
    }
    if (t1 < 0) {
      throw exceptions::ConsistencyException(
          "Train " + tr_name + " not present at time " + std::to_string(t));
    }

    assert(t >= t0);
    assert(t <= t1);
    const auto pos0 = get_train_pos(tr_name, t0);

    const Route& route = this->get_instance().const_routes().get_route(tr_name);
    const Network& n   = this->get_instance().const_n();
    const auto     r_len = route.length(n);
    return {route.get_edge_at_pos(std::min(pos0 + GRB_EPS, r_len), n), t0, t1};
  };
  [[nodiscard]] std::tuple<double, double, double, double>
  get_exact_pos_and_vel_bounds(const std::string& tr_name, double t) const {
    const auto [edge, t1, t2] = get_edge_and_time_bounds(tr_name, t);
    assert(t >= t1);
    assert(t <= t2);

    const auto v1   = get_train_speed(tr_name, t1);
    const auto v2   = get_train_speed(tr_name, t2);
    const auto pos1 = get_train_pos(tr_name, t1);
    const auto pos2 = get_train_pos(tr_name, t2);

    const Route& tr_route         = this->instance.get_route(tr_name);
    const auto&  r_len            = tr_route.length(this->instance.const_n());
    const bool   tr_leaving_route = pos2 >= r_len + GRB_EPS;

    if (std::abs(pos2 - pos1) < GRB_EPS) {
      return {std::min(pos1, pos2), std::max(pos1, pos2), std::min(v1, v2),
              std::max(v1, v2)};
    }

    const auto& edge_obj = this->instance.const_n().get_edge(edge);
    const auto& tr_obj   = this->instance.get_train_list().get_train(tr_name);

    const auto max_speed = tr_leaving_route
                               ? tr_obj.max_speed
                               : std::min(edge_obj.max_speed, tr_obj.max_speed);

    const auto max_t =
        max_travel_time(v1, v2, V_MIN, tr_obj.acceleration, tr_obj.deceleration,
                        pos2 - pos1, edge_obj.breakable);
    const auto min_t = min_travel_time(v1, v2, max_speed, tr_obj.acceleration,
                                       tr_obj.deceleration, pos2 - pos1);
    double     ub    = pos1;
    double     lb    = pos1;
    double     v_lb  = 0;
    double     v_ub  = max_speed;

    if (max_t >= std::numeric_limits<double>::infinity()) {
      const auto t_to_stop = v1 / tr_obj.deceleration;
      const auto rel_t     = std::min(t_to_stop, t - t1);
      lb += v1 * rel_t - 0.5 * tr_obj.deceleration * rel_t * rel_t;
      v_lb = v1 - tr_obj.deceleration * rel_t;
    } else {
      const auto min_speed = minimal_line_speed(
          v1, v2, V_MIN, tr_obj.acceleration, tr_obj.deceleration, pos2 - pos1);
      lb += pos_on_edge_at_time(v1, v2, min_speed, tr_obj.acceleration,
                                tr_obj.deceleration, pos2 - pos1, t - t1);
      v_lb = vel_on_edge_at_time(v1, v2, min_speed, tr_obj.acceleration,
                                 tr_obj.deceleration, pos2 - pos1, t - t1);
    }

    if (t >= t1 + min_t) {
      ub += pos2 - pos1;
    } else {
      const auto max_line_speed =
          maximal_line_speed(v1, v2, max_speed, tr_obj.acceleration,
                             tr_obj.deceleration, pos2 - pos1);
      ub += pos_on_edge_at_time(v1, v2, max_line_speed, tr_obj.acceleration,
                                tr_obj.deceleration, pos2 - pos1, t - t1);
      v_ub = vel_on_edge_at_time(v1, v2, max_line_speed, tr_obj.acceleration,
                                 tr_obj.deceleration, pos2 - pos1, t - t1);
    }
    return {lb, ub, v_lb, v_ub};
  };
  [[nodiscard]] std::optional<std::pair<double, double>>
  get_approximate_train_pos_and_vel(const std::string& tr_name,
                                    double             t) const {
    const auto [edge, t1, t2] = get_edge_and_time_bounds(tr_name, t);
    assert(t >= t1);
    assert(t <= t2);

    const auto pos_1 = get_train_pos(tr_name, t1);
    const auto v1    = get_train_speed(tr_name, t1);

    if (t1 == t2) {
      return std::make_pair(pos_1, v1);
    }

    const auto pos_2 = get_train_pos(tr_name, t2);
    const auto v2    = get_train_speed(tr_name, t2);

    const auto& edge_obj  = this->instance.const_n().get_edge(edge);
    const auto& tr_obj    = this->instance.get_train_list().get_train(tr_name);
    const auto  max_speed = std::min(tr_obj.max_speed, edge_obj.max_speed);
    const auto  dist_travelled = pos_2 - pos_1;

    if (std::abs(dist_travelled) < GRB_EPS) {
      // Train stopped
      return std::make_pair(pos_1, 0);
    }

    const auto v_line =
        get_line_speed(v1, v2, V_MIN, max_speed, tr_obj.acceleration,
                       tr_obj.deceleration, dist_travelled, t2 - t1);
    if (v_line <= 0) {
      return std::nullopt;
    }

    const auto tr_pos =
        get_train_pos(tr_name, t1) +
        pos_on_edge_at_time(v1, v2, v_line, tr_obj.acceleration,
                            tr_obj.deceleration, dist_travelled, t - t1);
    const auto tr_vel =
        vel_on_edge_at_time(v1, v2, v_line, tr_obj.acceleration,
                            tr_obj.deceleration, dist_travelled, t - t1);

    return std::make_pair(tr_pos, tr_vel);
  };
  [[nodiscard]] double get_train_speed(const std::string& tr_name,
                                       double             t) const {
    if (!this->instance.get_train_list().has_train(tr_name)) {
      throw exceptions::TrainNotExistentException(tr_name);
    }
    const auto tr_id = this->instance.get_train_list().get_train_index(tr_name);
    if (train_speed.at(tr_id).count(t) > 0) {
      return train_speed.at(tr_id).at(t);
    }
    throw exceptions::ConsistencyException("No speed for train " + tr_name +
                                           " at time " + std::to_string(t));
  };
  [[nodiscard]] bool get_train_routed(const std::string& tr_name) const {
    if (!this->instance.get_train_list().has_train(tr_name)) {
      throw exceptions::TrainNotExistentException(tr_name);
    }
    return train_routed.at(
        this->instance.get_train_list().get_train_index(tr_name));
  };
  [[nodiscard]] std::vector<double>
  get_train_times(const std::string& tr_name) const {
    if (!this->instance.get_train_list().has_train(tr_name)) {
      throw exceptions::TrainNotExistentException(tr_name);
    }
    const auto tr_id = this->instance.get_train_list().get_train_index(tr_name);
    const auto& tr_speed_map = train_speed.at(tr_id);
    // Return keys of the map
    std::vector<double> times;
    for (const auto& [t, _] : tr_speed_map) {
      times.push_back(t);
    }
    // Sort
    std::sort(times.begin(), times.end());
    return times;
  };
  [[nodiscard]] std::vector<size_t> get_train_order(size_t edge_index) const {
    std::vector<size_t> tr_on_edge =
        this->get_instance().trains_on_edge(edge_index, true);
    std::map<size_t, double> tr_times;
    for (const auto& tr : tr_on_edge) {
      const Train& tr_object =
          this->get_instance().get_train_list().get_train(tr);
      const double e_pos =
          this->get_instance().route_edge_pos(tr_object.name, edge_index).first;
      const auto time_at_e_pos = get_time_at_pos(tr_object.name, e_pos);
      tr_times.insert({tr, time_at_e_pos});
    }
    std::sort(tr_on_edge.begin(), tr_on_edge.end(),
              [&tr_times](size_t tr1, size_t tr2) {
                return tr_times.at(tr1) < tr_times.at(tr2);
              });
    return tr_on_edge;
  };
  [[nodiscard]] double get_time_at_pos(const std::string& tr_name,
                                       double             pos) const {
    if (!this->instance.get_train_list().has_train(tr_name)) {
      throw exceptions::TrainNotExistentException(tr_name);
    }
    const auto tr_times = get_train_times(tr_name);
    for (const auto& t : tr_times) {
      if (std::abs(get_train_pos(tr_name, t) - pos) < GRB_EPS) {
        return t;
      }
    }
    throw exceptions::ConsistencyException(
        "No time for train " + tr_name + " at position " + std::to_string(pos));
  };

  void add_train_pos(const std::string& tr_name, double t, double pos) {
    if (!this->instance.get_train_list().has_train(tr_name)) {
      throw exceptions::TrainNotExistentException(tr_name);
    }
    if (pos + EPS < 0) {
      throw exceptions::ConsistencyException("Position must be non-negative");
    }
    if (t + EPS < 0) {
      throw exceptions::ConsistencyException("Time must be non-negative");
    }

    const auto tr_id = this->instance.get_train_list().get_train_index(tr_name);
    if (train_pos.at(tr_id).count(t) > 0) {
      train_pos.at(tr_id).at(t) = pos;
    } else {
      train_pos.at(tr_id).insert({t, pos});
    }
  };
  void add_train_speed(const std::string& tr_name, double t, double speed) {
    if (!this->instance.get_train_list().has_train(tr_name)) {
      throw exceptions::TrainNotExistentException(tr_name);
    }
    if (speed + EPS < 0) {
      throw exceptions::ConsistencyException("Speed must be non-negative");
    }
    if (t + EPS < 0) {
      throw exceptions::ConsistencyException("Time must be non-negative");
    }

    const auto tr_id = this->instance.get_train_list().get_train_index(tr_name);
    if (train_speed.at(tr_id).count(t) > 0) {
      train_speed.at(tr_id).at(t) = speed;
    } else {
      train_speed.at(tr_id).insert({t, speed});
    }
  };
  void set_train_routed(const std::string& tr_name) {
    set_train_routed_value(tr_name, true);
  };
  void set_train_not_routed(const std::string& tr_name) {
    set_train_routed_value(tr_name, false);
  };
  void set_train_routed_value(const std::string& tr_name, bool val) {
    if (!this->instance.get_train_list().has_train(tr_name)) {
      throw exceptions::TrainNotExistentException(tr_name);
    }
    train_routed.at(this->instance.get_train_list().get_train_index(tr_name)) =
        val;
  };

  void export_solution(const std::filesystem::path& p,
                       bool export_instance) const override {
    /**
     * This method exports the solution object to a specific path. This includes
     * the following:
     * - If export_instance is true, the instance is exported to the path p /
     * instance
     * - If export_instance is false, the routes are exported to the path p /
     * instance / routes
     * - dt, status, obj, and postprocessed are exported to p / solution /
     * data.json
     * - train_pos and train_speed are exported to p / solution / train_pos.json
     * and p / solution / train_speed.json The method throws a
     * ConsistencyException if the solution is not consistent.
     *
     * @param p the path to the folder where the solution should be exported
     * @param export_instance whether the instance should be exported next to
     * the solution
     */

    if (!check_consistency()) {
      throw exceptions::ConsistencyException();
    }

    if (!is_directory_and_create(p / "solution")) {
      throw exceptions::ExportException("Could not create directory " +
                                        p.string());
    }

    SolGeneralProblemInstanceWithScheduleAndRoutes<
        T>::export_general_solution_data_with_routes(p, export_instance, true);

    json train_pos_json;
    json train_speed_json;
    json train_routed_json;
    for (size_t tr_id = 0; tr_id < this->instance.get_train_list().size();
         ++tr_id) {
      const auto& train = this->instance.get_train_list().get_train(tr_id);
      train_pos_json[train.name]    = train_pos.at(tr_id);
      train_speed_json[train.name]  = train_speed.at(tr_id);
      train_routed_json[train.name] = train_routed.at(tr_id);
    }

    std::ofstream train_pos_file(p / "solution" / "train_pos.json");
    train_pos_file << train_pos_json << std::endl;
    train_pos_file.close();

    std::ofstream train_speed_file(p / "solution" / "train_speed.json");
    train_speed_file << train_speed_json << std::endl;
    train_speed_file.close();

    std::ofstream train_routed_file(p / "solution" / "train_routed.json");
    train_routed_file << train_routed_json << std::endl;
    train_routed_file.close();
  };
  [[nodiscard]] bool check_consistency() const override {
    if (!SolGeneralProblemInstanceWithScheduleAndRoutes<
            T>::check_general_solution_data_consistency()) {
      return false;
    }
    if (!this->instance.check_consistency(false)) {
      return false;
    }

    if (!this->has_sol) {
      return true;
    }

    for (auto tr_id = 0; tr_id < train_routed.size(); tr_id++) {
      const auto& tr_name =
          this->instance.get_train_list().get_train(tr_id).name;
      if (train_routed.at(tr_id) && !this->instance.has_route(tr_name)) {
        return false;
      }
      if (!train_routed.at(tr_id) &&
          !this->instance.get_train_optional().at(tr_id)) {
        return false;
      }
      if (train_routed.at(tr_id) && train_pos.at(tr_id).size() < 2) {
        // At least two points of information are needed to recover the timing
        return false;
      }

      if (train_pos.size() != train_speed.size()) {
        return false;
      }

      for (const auto& [t, pos] : train_pos.at(tr_id)) {
        if (train_speed.at(tr_id).count(t) != 1) {
          return false;
        }
      }
    }

    for (const auto& train_pos_vec : train_pos) {
      for (const auto& [t, pos] : train_pos_vec) {
        if (pos + EPS < 0) {
          return false;
        }
      }
    }
    for (size_t tr_id = 0; tr_id < train_speed.size(); ++tr_id) {
      const auto& train = this->instance.get_train_list().get_train(tr_id);
      for (const auto& [t, v] : train_speed.at(tr_id)) {
        if (v + EPS < 0 || v > train.max_speed + EPS) {
          return false;
        }
      }
    }
    return true;
  };

  [[nodiscard]] static SolGeneralPerformanceOptimizationInstance
  import_solution(const std::filesystem::path& p,
                  const std::optional<T>&      instance = std::optional<T>()) {
    auto sol = SolGeneralPerformanceOptimizationInstance(p, instance);
    if (!sol.check_consistency()) {
      throw exceptions::ConsistencyException(
          "Imported solution object is not consistent");
    }
    return sol;
  };
  [[nodiscard]] static SolGeneralPerformanceOptimizationInstance
  import_solution(const std::string&      path,
                  const std::optional<T>& instance = std::optional<T>()) {
    return import_solution(std::filesystem::path(path), instance);
  };
  [[nodiscard]] static SolGeneralPerformanceOptimizationInstance
  import_solution(const char*             path,
                  const std::optional<T>& instance = std::optional<T>()) {
    return import_solution(std::filesystem::path(path), instance);
  };
};

template <typename T>
class SolVSSGeneralPerformanceOptimizationInstance
    : public SolGeneralPerformanceOptimizationInstance<T> {
  static_assert(
      std::is_base_of<GeneralPerformanceOptimizationInstance, T>::value,
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

    json vss_pos_json;
    for (size_t edge_id = 0;
         edge_id < this->instance.const_n().number_of_edges(); ++edge_id) {
      const auto& edge = this->instance.const_n().get_edge(edge_id);
      const auto& v0   = this->instance.const_n().get_vertex(edge.source).name;
      const auto& v1   = this->instance.const_n().get_vertex(edge.target).name;
      vss_pos_json["('" + v0 + "', '" + v1 + "')"] = vss_pos.at(edge_id);
    }

    std::ofstream vss_pos_file(p / "solution" / "vss_pos.json");
    vss_pos_file << vss_pos_json << std::endl;
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

} // namespace cda_rail::instances
