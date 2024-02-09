#pragma once

#include "GeneralProblemInstance.hpp"
#include "datastructure/GeneralTimetable.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Route.hpp"
#include "nlohmann/json.hpp"

#include <filesystem>
#include <fstream>
#include <string>
#include <type_traits>
#include <vector>

using json = nlohmann::json;

namespace cda_rail::instances {

template <typename T>
class GeneralPerformanceOptimizationInstance
    : public GeneralProblemInstanceWithScheduleAndRoutes<GeneralTimetable<T>> {
  static_assert(std::is_base_of_v<BaseGeneralSchedule, T>,
                "T must be a child of BaseGeneralSchedule");
  static_assert(HasTimeType<T>::value, "T must have a time_type() method");
  using TimeType = decltype(T::time_type());

  friend class SolGeneralPerformanceOptimizationInstance<T>;

  void initialize_vectors() {
    train_weights = std::vector<double>(timetable.get_train_list().size(), 1);
    train_optional =
        std::vector<bool>(timetable.get_train_list().size(), false);
  }

protected:
  std::vector<double> train_weights;
  std::vector<bool>   train_optional;
  double lambda = 1; // Minutes of delay (of a weight one train) that are
                     // "equal" to scheduling another weight one train

public:
  GeneralPerformanceOptimizationInstance() = default;
  explicit GeneralPerformanceOptimizationInstance(Network             network,
                                                  GeneralTimetable<T> timetable,
                                                  RouteMap            routes)
      : GeneralProblemInstanceWithScheduleAndRoutes<GeneralTimetable<T>>(
            network, timetable, routes) {
    initialize_vectors();
  };
  explicit GeneralPerformanceOptimizationInstance(
      const std::filesystem::path& path)
      : GeneralProblemInstanceWithScheduleAndRoutes<GeneralTimetable<T>>(path) {
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

  using GeneralProblemInstance::export_instance;

  void export_instance(const std::filesystem::path& path) const override {
    GeneralProblemInstanceWithScheduleAndRoutes<
        GeneralTimetable<T>>::export_instance(path);

    json j;
    for (size_t i = 0; i < train_weights.size(); ++i) {
      j["train_weights"]
       [this->get_timetable().get_train_list().get_train(i).get_name()] =
           train_weights[i];
      j["train_optional"]
       [this->get_timetable().get_train_list().get_train(i).get_name()] =
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
    if (!GeneralProblemInstanceWithScheduleAndRoutes<GeneralTimetable<T>>::
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
};

template <typename T>
class SolGeneralPerformanceOptimizationInstance
    : public SolGeneralProblemInstance<
          GeneralPerformanceOptimizationInstance<T>> {
  static_assert(std::is_base_of_v<BaseGeneralSchedule, T>,
                "T must be a child of BaseGeneralSchedule");

public:
  SolGeneralPerformanceOptimizationInstance() = default;
  explicit SolGeneralPerformanceOptimizationInstance(
      GeneralPerformanceOptimizationInstance<T> instance)
      : SolGeneralProblemInstance<GeneralPerformanceOptimizationInstance<T>>(
            std::move(instance)){};
  SolGeneralPerformanceOptimizationInstance(
      GeneralPerformanceOptimizationInstance<T> instance, SolutionStatus status,
      double obj, bool has_sol)
      : SolGeneralProblemInstance<GeneralPerformanceOptimizationInstance<T>>(
            std::move(instance), status, obj, has_sol){};
};

} // namespace cda_rail::instances
