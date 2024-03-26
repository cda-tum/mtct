#pragma once

#include "Definitions.hpp"
#include "GeneralProblemInstance.hpp"
#include "VSSGenerationTimetable.hpp"
#include "datastructure/GeneralTimetable.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Route.hpp"
#include "nlohmann/json.hpp"

#include <filesystem>
#include <fstream>
#include <map>
#include <string>
#include <type_traits>
#include <vector>

using json = nlohmann::json;

namespace cda_rail::instances {

class GeneralPerformanceOptimizationInstance
    : public GeneralProblemInstanceWithScheduleAndRoutes<
          GeneralTimetable<GeneralSchedule<GeneralScheduledStop>>> {
  friend class SolGeneralPerformanceOptimizationInstance;

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

class SolGeneralPerformanceOptimizationInstance
    : public SolGeneralProblemInstanceWithScheduleAndRoutes<
          GeneralPerformanceOptimizationInstance> {
  std::vector<std::map<double, double>> train_pos;
  std::vector<std::map<double, double>> train_speed;
  std::vector<bool>                     train_routed;

  void initialize_vectors() {
    train_pos.reserve(instance.get_timetable().get_train_list().size());
    train_speed.reserve(instance.get_timetable().get_train_list().size());
    train_routed = std::vector<bool>(
        instance.get_timetable().get_train_list().size(), false);
    for (size_t tr = 0; tr < this->instance.get_train_list().size(); ++tr) {
      train_pos.emplace_back();
      train_speed.emplace_back();
    }
  };

public:
  SolGeneralPerformanceOptimizationInstance() = default;
  explicit SolGeneralPerformanceOptimizationInstance(
      const GeneralPerformanceOptimizationInstance& instance)
      : SolGeneralProblemInstanceWithScheduleAndRoutes<
            GeneralPerformanceOptimizationInstance>(instance) {
    this->initialize_vectors();
  };
  SolGeneralPerformanceOptimizationInstance(
      const GeneralPerformanceOptimizationInstance& instance,
      SolutionStatus status, double obj, bool has_sol)
      : SolGeneralProblemInstanceWithScheduleAndRoutes<
            GeneralPerformanceOptimizationInstance>(instance, status, obj,
                                                    has_sol) {
    this->initialize_vectors();
  };
  explicit SolGeneralPerformanceOptimizationInstance(
      const std::filesystem::path&                                 p,
      const std::optional<GeneralPerformanceOptimizationInstance>& instance =
          std::optional<GeneralPerformanceOptimizationInstance>());

  [[nodiscard]] double get_train_pos(const std::string& tr_name,
                                     double             t) const;
  [[nodiscard]] double get_train_speed(const std::string& tr_name,
                                       double             t) const;
  [[nodiscard]] bool   get_train_routed(const std::string& tr_name) const;

  void add_train_pos(const std::string& tr_name, double t, double pos);
  void add_train_speed(const std::string& tr_name, double t, double speed);
  void set_train_routed(const std::string& tr_name) {
    set_train_routed_value(tr_name, true);
  }
  void set_train_not_routed(const std::string& tr_name) {
    set_train_routed_value(tr_name, false);
  }
  void set_train_routed_value(const std::string& tr_name, bool val);

  void               export_solution(const std::filesystem::path& p,
                                     bool export_instance) const override;
  [[nodiscard]] bool check_consistency() const override;

  [[nodiscard]] static SolGeneralPerformanceOptimizationInstance
  import_solution(
      const std::filesystem::path&                                 p,
      const std::optional<GeneralPerformanceOptimizationInstance>& instance =
          std::optional<GeneralPerformanceOptimizationInstance>()) {
    auto sol = SolGeneralPerformanceOptimizationInstance(p, instance);
    if (!sol.check_consistency()) {
      throw exceptions::ConsistencyException(
          "Imported solution object is not consistent");
    }
    return sol;
  };
  [[nodiscard]] static SolGeneralPerformanceOptimizationInstance
  import_solution(
      const std::string&                                           path,
      const std::optional<GeneralPerformanceOptimizationInstance>& instance =
          std::optional<GeneralPerformanceOptimizationInstance>()) {
    return import_solution(std::filesystem::path(path), instance);
  };
  [[nodiscard]] static SolGeneralPerformanceOptimizationInstance
  import_solution(
      const char*                                                  path,
      const std::optional<GeneralPerformanceOptimizationInstance>& instance =
          std::optional<GeneralPerformanceOptimizationInstance>()) {
    return import_solution(std::filesystem::path(path), instance);
  };
};

class SolVSSGeneralPerformanceOptimizationInstance
    : public SolGeneralPerformanceOptimizationInstance {
  // TODO
};

} // namespace cda_rail::instances
