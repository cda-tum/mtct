#pragma once

#include "GeneralProblemInstance.hpp"
#include "datastructure/GeneralTimetable.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "nlohmann/json.hpp"

#include <filesystem>
#include <fstream>
#include <string>
#include <type_traits>
#include <vector>

using json = nlohmann::json;

namespace cda_rail::instances {

template <typename, typename = void> struct HasTimeType : std::false_type {};

template <typename T>
struct HasTimeType<T, std::void_t<decltype(std::declval<T>().time_type())>>
    : std::true_type {};

template <typename T>
class GeneralPerformanceOptimizationInstance : public GeneralProblemInstance {
  static_assert(std::is_base_of_v<BaseGeneralSchedule, T>,
                "T must be a child of BaseGeneralSchedule");
  static_assert(HasTimeType<T>::value, "T must have a time_type() method");
  using TimeType = decltype(T::time_type());

  void initialize_vectors() {
    train_weights = std::vector<double>(timetable.get_train_list().size(), 1);
    train_optional =
        std::vector<bool>(timetable.get_train_list().size(), false);
  }

  void set_train_optional_value(size_t train_index, bool val) {
    if (!timetable.get_train_list().has_train(train_index)) {
      throw std::invalid_argument("Train index out of bounds");
    }
    train_optional[train_index] = val;
  };
  void set_train_optional_value(const std::string& train_name, bool val) {
    set_train_optional_value(
        timetable.get_train_list().get_train_index(train_name), val);
  };
  void set_train_weight(const char* train_name, bool val) {
    set_train_optional_value(
        timetable.get_train_list().get_train_index(train_name), val);
  };

protected:
  GeneralTimetable<T> timetable;
  std::vector<double> train_weights;
  std::vector<bool>   train_optional;
  double lambda = 1; // Minutes of delay (of a weight one train) that are
                     // "equal" to scheduling another weight one train

public:
  GeneralPerformanceOptimizationInstance() = default;
  explicit GeneralPerformanceOptimizationInstance(Network             network,
                                                  GeneralTimetable<T> timetable)
      : GeneralProblemInstance(std::move(network)),
        timetable(std::move(timetable)) {
    initialize_vectors();
  };
  explicit GeneralPerformanceOptimizationInstance(
      const std::filesystem::path& path)
      : GeneralProblemInstance(path),
        timetable(GeneralTimetable<T>(path / "Timetable")) {
    initialize_vectors();

    std::ifstream file(path / "problem_data.json");
    json          j = json::parse(file);
    for (const auto& [train_name, weight] : j["train_weights"].items()) {
      set_train_weight(train_name, static_cast<double>(weight));
    }
    for (const auto& [train_name, optional] : j["train_optional"].items()) {
      set_train_optional_value(train_name, static_cast<bool>(optional));
    }
    lambda = static_cast<double>(j["lambda"]);
  };

  [[nodiscard]] const auto& get_timetable() const { return timetable; };
  [[nodiscard]] const auto& get_train_weights() const { return train_weights; };
  [[nodiscard]] const auto& get_train_optional() const {
    return train_optional;
  };
  [[nodiscard]] double get_lambda() const { return lambda; };

  [[nodiscard]] double get_train_weight(size_t train_index) {
    if (!timetable.get_train_list().has_train(train_index)) {
      throw std::invalid_argument("Train index out of bounds");
    }
    return train_weights.at(train_index);
  }
  [[nodiscard]] double get_train_weight(const std::string& train_name) {
    return get_train_weight(
        timetable.get_train_list().get_train_index(train_name));
  }
  [[nodiscard]] double get_train_weight(const char* train_name) {
    return get_train_weight(
        timetable.get_train_list().get_train_index(train_name));
  }

  [[nodiscard]] bool get_train_optional(size_t train_index) {
    if (!timetable.get_train_list().has_train(train_index)) {
      throw std::invalid_argument("Train index out of bounds");
    }
    return train_optional.at(train_index);
  }
  [[nodiscard]] bool get_train_optional(const std::string& train_name) {
    return get_train_optional(
        timetable.get_train_list().get_train_index(train_name));
  }
  [[nodiscard]] bool get_train_optional(const char* train_name) {
    return get_train_optional(
        timetable.get_train_list().get_train_index(train_name));
  }

  void set_train_weight(size_t train_index, double weight) {
    if (!timetable.get_train_list().has_train(train_index)) {
      throw std::invalid_argument("Train index out of bounds");
    }
    train_weights[train_index] = weight;
  };
  void set_train_weight(const std::string& train_name, double weight) {
    set_train_weight(timetable.get_train_list().get_train_index(train_name),
                     weight);
  };
  void set_train_weight(const char* train_name, double weight) {
    set_train_weight(timetable.get_train_list().get_train_index(train_name),
                     weight);
  };

  void set_train_optionality_value(size_t train_index, bool val) {
    if (!timetable.get_train_list().has_train(train_index)) {
      throw std::invalid_argument("Train index out of bounds");
    }
    train_optional[train_index] = val;
  };
  void set_train_optionality_value(const std::string& train_name, bool val) {
    set_train_optionality_value(
        timetable.get_train_list().get_train_index(train_name), val);
  };
  void set_train_optionality_value(const char* train_name, bool val) {
    set_train_optionality_value(
        timetable.get_train_list().get_train_index(train_name), val);
  };
  void set_train_optional(size_t train_index) {
    set_train_optionality_value(train_index, true);
  };
  void set_train_optional(const std::string& train_name) {
    set_train_optionality_value(
        timetable.get_train_list().get_train_index(train_name), true);
  };
  void set_train_optional(const char* train_name) {
    set_train_optionality_value(
        timetable.get_train_list().get_train_index(train_name), true);
  };
  void set_train_mandatory(size_t train_index) {
    set_train_optionality_value(train_index, false);
  };
  void set_train_mandatory(const std::string& train_name) {
    set_train_optionality_value(
        timetable.get_train_list().get_train_index(train_name), false);
  };
  void set_train_mandatory(const char* train_name) {
    set_train_optionality_value(
        timetable.get_train_list().get_train_index(train_name), false);
  };

  Train& editable_tr(size_t index) { return timetable.editable_tr(index); };
  Train& editable_tr(const std::string& name) {
    return timetable.editable_tr(name);
  };

  template <typename... Args> size_t add_train(Args... args) {
    return timetable.add_train(args..., network);
  }

  void add_station(const std::string& name) { timetable.add_station(name); };

  template <typename... Args> size_t add_stop(Args... args) {
    return timetable.add_stop(args...);
  }

  [[nodiscard]] const StationList& get_station_list() const {
    return timetable.get_station_list();
  };
  [[nodiscard]] const TrainList& get_train_list() const {
    return timetable.get_train_list();
  };
  [[nodiscard]] const auto& get_schedule(size_t index) const {
    return timetable.get_schedule(index);
  };
  [[nodiscard]] const auto& get_schedule(const std::string& train_name) const {
    return timetable.get_schedule(train_name);
  };

  using GeneralProblemInstance::export_instance;

  void export_instance(const std::filesystem::path& path) const override {
    if (!is_directory_and_create(path)) {
      throw std::invalid_argument("Path is not a directory");
    }
    timetable.export_timetable(path / "Timetable");
    export_network(path);

    json j;
    for (size_t i = 0; i < train_weights.size(); ++i) {
      j["train_weights"][timetable.get_train_list().get_train(i).get_name()] =
          train_weights[i];
      j["train_optional"][timetable.get_train_list().get_train(i).get_name()] =
          train_optional[i];
    }
    j["lambda"] = lambda;

    std::ofstream file(path / "problem_data.json");
    file << j << std::endl;
  };

  [[nodiscard]] bool check_consistency() const override {
    if (!timetable.check_consistency(network)) {
      return false;
    }
    const auto num_tr = timetable.get_train_list().size();
    if (train_weights.size() != num_tr || train_optional.size() != num_tr) {
      return false;
    }
    return lambda >= 0 &&
           std::all_of(train_weights.begin(), train_weights.end(),
                       [](double w) { return w >= 0; });
  };
};
} // namespace cda_rail::instances
