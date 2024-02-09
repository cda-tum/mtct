#pragma once
#include "CustomExceptions.hpp"
#include "GeneralProblemInstance.hpp"
#include "VSSModel.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Route.hpp"
#include "datastructure/Timetable.hpp"

#include <filesystem>
#include <optional>

namespace cda_rail::instances {
class VSSGenerationTimetable
    : public GeneralProblemInstanceWithScheduleAndRoutes<Timetable> {
  friend class SolVSSGenerationTimetable;

public:
  // Constructors
  VSSGenerationTimetable() = default;
  explicit VSSGenerationTimetable(const std::filesystem::path& p)
      : GeneralProblemInstanceWithScheduleAndRoutes<Timetable>(p){};
  explicit VSSGenerationTimetable(const std::string& path)
      : VSSGenerationTimetable(std::filesystem::path(path)){};
  explicit VSSGenerationTimetable(const char* path)
      : VSSGenerationTimetable(std::filesystem::path(path)){};
  ~VSSGenerationTimetable() = default;

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
    VSSGenerationTimetable return_instance(p);
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
  [[nodiscard]] std::vector<size_t>
  trains_in_section(const std::vector<size_t>& section) const;
  [[nodiscard]] std::vector<size_t> trains_at_t(int t) const;
  [[nodiscard]] std::vector<size_t>
  trains_at_t(int t, const std::vector<size_t>& trains_to_consider) const;
  [[nodiscard]] bool                has_route_for_every_train() const;
  [[nodiscard]] std::vector<size_t> trains_on_edge(size_t edge_id,
                                                   bool   fixed_routes) const;
  [[nodiscard]] std::vector<size_t>
  trains_on_edge(size_t edge_id, bool fixed_routes,
                 const std::vector<size_t>& trains_to_consider) const;
  [[nodiscard]] std::vector<size_t>
  edges_used_by_train(size_t train_id, bool fixed_routes) const {
    return edges_used_by_train(get_train_list().get_train(train_id).name,
                               fixed_routes);
  };
  [[nodiscard]] std::vector<size_t>
  edges_used_by_train(const std::string& train_name, bool fixed_routes) const;
};

class SolVSSGenerationTimetable
    : public SolGeneralProblemInstanceWithScheduleAndRoutes<
          VSSGenerationTimetable> {
private:
  std::vector<std::vector<double>> vss_pos;

  int                              dt = -1;
  std::vector<std::vector<double>> train_pos;
  std::vector<std::vector<double>> train_speed;

  double mip_obj       = -1;
  bool   postprocessed = false;

  void initialize_vectors();

public:
  // Constructor
  explicit SolVSSGenerationTimetable(const VSSGenerationTimetable& instance,
                                     int                           dt);
  explicit SolVSSGenerationTimetable(
      const std::filesystem::path&                 p,
      const std::optional<VSSGenerationTimetable>& instance =
          std::optional<VSSGenerationTimetable>());
  SolVSSGenerationTimetable() = delete;

  // Getter
  [[nodiscard]] const std::vector<double>& get_vss_pos(size_t edge_id) const {
    if (!instance.const_n().has_edge(edge_id)) {
      throw cda_rail::exceptions::EdgeNotExistentException(edge_id);
    }
    return vss_pos.at(edge_id);
  };
  [[nodiscard]] const std::vector<double>& get_vss_pos(size_t source,
                                                       size_t target) const {
    return get_vss_pos(instance.const_n().get_edge_index(source, target));
  };
  [[nodiscard]] const std::vector<double>&
  get_vss_pos(const std::string& source, const std::string& target) const {
    return get_vss_pos(instance.const_n().get_edge_index(source, target));
  };

  [[nodiscard]] double get_train_pos(size_t train_id, int time) const;
  [[nodiscard]] double get_train_pos(const std::string& train_name,
                                     int                time) const {
    return get_train_pos(instance.get_train_list().get_train_index(train_name),
                         time);
  }
  [[nodiscard]] std::vector<double>
  get_valid_border_stops(size_t train_id) const;
  [[nodiscard]] std::vector<double>
  get_valid_border_stops(const std::string& train_name) const {
    return get_valid_border_stops(
        instance.get_train_list().get_train_index(train_name));
  }

  [[nodiscard]] double get_train_speed(size_t train_id, int time) const;
  [[nodiscard]] double get_train_speed(const std::string& train_name,
                                       int                time) const {
    return get_train_speed(
        instance.get_train_list().get_train_index(train_name), time);
  }

  [[nodiscard]] double get_mip_obj() const { return mip_obj; };
  [[nodiscard]] bool   get_postprocessed() const { return postprocessed; };
  [[nodiscard]] int    get_dt() const { return dt; };

  void set_mip_obj(double new_mip_obj) { mip_obj = new_mip_obj; };
  void set_postprocessed(bool new_postprocessed) {
    postprocessed = new_postprocessed;
  };

  void add_vss_pos(size_t edge_id, double pos, bool reverse_edge = true);
  void add_vss_pos(size_t source, size_t target, double pos,
                   bool reverse_edge = true) {
    add_vss_pos(instance.const_n().get_edge_index(source, target), pos,
                reverse_edge);
  };
  void add_vss_pos(const std::string& source, const std::string& target,
                   double pos, bool reverse_edge = true) {
    add_vss_pos(instance.const_n().get_edge_index(source, target), pos,
                reverse_edge);
  };

  void set_vss_pos(size_t edge_id, std::vector<double> pos);
  void set_vss_pos(size_t source, size_t target, std::vector<double> pos) {
    set_vss_pos(instance.const_n().get_edge_index(source, target),
                std::move(pos));
  };
  void set_vss_pos(const std::string& source, const std::string& target,
                   std::vector<double> pos) {
    set_vss_pos(instance.const_n().get_edge_index(source, target),
                std::move(pos));
  };

  void reset_vss_pos(size_t edge_id);
  void reset_vss_pos(size_t source, size_t target) {
    reset_vss_pos(instance.const_n().get_edge_index(source, target));
  };
  void reset_vss_pos(const std::string& source, const std::string& target) {
    reset_vss_pos(instance.const_n().get_edge_index(source, target));
  };

  void add_train_pos(size_t train_id, int time, double pos);
  void add_train_pos(const std::string& train_name, int time, double pos) {
    add_train_pos(instance.get_train_list().get_train_index(train_name), time,
                  pos);
  };

  void add_train_speed(size_t train_id, int time, double speed);
  void add_train_speed(const std::string& train_name, int time, double speed) {
    add_train_speed(instance.get_train_list().get_train_index(train_name), time,
                    speed);
  };

  [[nodiscard]] bool check_consistency() const override;

  void export_solution(const std::filesystem::path& p,
                       bool export_instance) const override;

  [[nodiscard]] static SolVSSGenerationTimetable
  import_solution(const std::filesystem::path&                 p,
                  const std::optional<VSSGenerationTimetable>& instance =
                      std::optional<VSSGenerationTimetable>()) {
    auto sol = SolVSSGenerationTimetable(p, instance);
    if (!sol.check_consistency()) {
      throw exceptions::ConsistencyException(
          "Imported solution object is not consistent");
    }
    return sol;
  };
  [[nodiscard]] static SolVSSGenerationTimetable
  import_solution(const std::string&                           path,
                  const std::optional<VSSGenerationTimetable>& instance =
                      std::optional<VSSGenerationTimetable>()) {
    return import_solution(std::filesystem::path(path), instance);
  };
  [[nodiscard]] static SolVSSGenerationTimetable
  import_solution(const char*                                  path,
                  const std::optional<VSSGenerationTimetable>& instance =
                      std::optional<VSSGenerationTimetable>()) {
    return import_solution(std::filesystem::path(path), instance);
  };
};
} // namespace cda_rail::instances
