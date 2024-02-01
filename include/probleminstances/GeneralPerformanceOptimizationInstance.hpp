#pragma once

#include "GeneralProblemInstance.hpp"
#include "datastructure/GeneralTimetable.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "nlohmann/json.hpp"

#include <filesystem>
#include <fstream>
#include <type_traits>
#include <vector>

using json = nlohmann::json;

namespace cda_rail::instances {

template <typename T>
class GeneralPerformanceOptimizationInstance : public GeneralProblemInstance {
  static_assert(std::is_base_of_v<BaseGeneralSchedule, T>,
                "T must be a child of BaseGeneralSchedule");

  void initialize_vectors() {
    train_weights = std::vector<double>(timetable.get_train_list().size(), 1);
    train_optional =
        std::vector<bool>(timetable.get_train_list().size(), false);
  }

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

  [[nodiscard]] const auto& get_timetable() const { return timetable; };
  [[nodiscard]] const auto& get_train_weights() const { return train_weights; };

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
};
} // namespace cda_rail::instances
