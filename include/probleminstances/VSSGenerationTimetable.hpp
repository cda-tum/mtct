#pragma once
#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "GeneralProblemInstance.hpp"
#include "VSSModel.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Route.hpp"
#include "datastructure/Timetable.hpp"

#include <filesystem>
#include <string>
#include <utility>

namespace cda_rail::instances {
class VSSGenerationTimetable
    : public GeneralProblemInstanceWithScheduleAndRoutes<Timetable> {
  friend class SolVSSGenerationTimetable;

public:
  // Constructors
  VSSGenerationTimetable() = default;
  explicit VSSGenerationTimetable(const std::filesystem::path& p)
      : GeneralProblemInstanceWithScheduleAndRoutes<Timetable>(p) {};
  explicit VSSGenerationTimetable(const std::string& path)
      : VSSGenerationTimetable(std::filesystem::path(path)) {};
  explicit VSSGenerationTimetable(const char* path)
      : VSSGenerationTimetable(std::filesystem::path(path)) {};
  explicit VSSGenerationTimetable(const Network& n, const Timetable& tt,
                                  const RouteMap& rm)
      : GeneralProblemInstanceWithScheduleAndRoutes<Timetable>(n, tt, rm) {};
  ~VSSGenerationTimetable() override = default;

  [[nodiscard]] std::pair<size_t, size_t>
  time_index_interval(size_t train_index, int dt,
                      bool tn_inclusive = true) const {
    return this->get_timetable().time_index_interval(train_index, dt,
                                                     tn_inclusive);
  }
  [[nodiscard]] std::pair<size_t, size_t>
  time_index_interval(const std::string& train_name, int dt,
                      bool tn_inclusive = true) const {
    return this->get_timetable().time_index_interval(train_name, dt,
                                                     tn_inclusive);
  }

  // Export and import functions
  using GeneralProblemInstanceWithScheduleAndRoutes<Timetable>::export_instance;

  [[nodiscard]] static VSSGenerationTimetable
  import_instance(const std::filesystem::path& p,
                  bool every_train_must_have_route = true) {
    const VSSGenerationTimetable return_instance(p);
    if (!return_instance.check_consistency(every_train_must_have_route)) {
      throw exceptions::ConsistencyException(
          "Imported instance object is not consistent");
    }
    return return_instance;
  };
  [[nodiscard]] static VSSGenerationTimetable
  import_instance(const std::string& path,
                  bool               every_train_must_have_route = true) {
    return import_instance(std::filesystem::path(path),
                           every_train_must_have_route);
  };
  [[nodiscard]] static VSSGenerationTimetable
  import_instance(const char* path, bool every_train_must_have_route = true) {
    return import_instance(std::filesystem::path(path),
                           every_train_must_have_route);
  };

  // Transformation functions
  void discretize(
      const vss::SeparationFunction& sep_func = &vss::functions::uniform);

  // Helper

  [[nodiscard]] cda_rail::index_vector trains_at_t(int t) const;
  [[nodiscard]] cda_rail::index_vector
  trains_at_t(int t, const cda_rail::index_vector& trains_to_consider) const;
};
} // namespace cda_rail::instances
