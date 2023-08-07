#pragma once
#include "MultiArray.hpp"
#include "gurobi_c++.h"
#include "probleminstances/VSSGenerationTimetable.hpp"
#include "unordered_map"

#include <filesystem>
#include <optional>
#include <string>

namespace cda_rail::solver::mip_based {
class VSSGenTimetableSolver {
private:
  instances::VSSGenerationTimetable instance;

  // Instance variables
  int                              dt                     = -1;
  int                              num_t                  = -1;
  size_t                           num_tr                 = -1;
  size_t                           num_edges              = -1;
  size_t                           num_vertices           = -1;
  size_t                           num_breakable_sections = -1;
  std::vector<std::vector<size_t>> unbreakable_sections;
  std::vector<std::vector<size_t>> no_border_vss_sections;
  std::vector<std::pair<int, int>> train_interval;
  std::vector<std::pair<std::optional<size_t>, std::optional<size_t>>>
                                     breakable_edges_pairs;
  std::vector<size_t>                no_border_vss_vertices;
  std::vector<size_t>                relevant_edges;
  std::vector<size_t>                breakable_edges;
  bool                               fix_routes       = false;
  VSSModel                           vss_model        = VSSModel::CONTINUOUS;
  std::vector<SeparationType>        separation_types = {};
  bool                               include_train_dynamics = false;
  bool                               include_braking_curves = false;
  bool                               use_pwl                = false;
  bool                               use_schedule_cuts      = false;
  std::unordered_map<size_t, size_t> breakable_edge_indices;
  std::vector<std::pair<std::vector<size_t>, std::vector<size_t>>>
      fwd_bwd_sections;

  // Gurobi variables
  std::optional<GRBEnv>                               env;
  std::optional<GRBModel>                             model;
  std::unordered_map<std::string, MultiArray<GRBVar>> vars;

  // Variable functions
  void create_general_variables();
  void create_fixed_routes_variables();
  void create_free_routes_variables();
  void create_discretized_variables();
  void create_non_discretized_variables();
  void create_brakelen_variables();

  // Constraint functions
  void create_general_constraints();
  void create_fixed_routes_constraints();
  void create_free_routes_constraints();
  void create_discretized_constraints();
  void create_non_discretized_constraints();
  void create_acceleration_constraints();
  void create_brakelen_constraints();

  // Helper functions for constraints
  void create_general_boundary_constraints();

  void create_general_schedule_constraints();
  void create_unbreakable_sections_constraints();
  void create_general_speed_constraints();
  void create_reverse_occupation_constraints();

  void create_fixed_routes_position_constraints();
  void create_boundary_fixed_routes_constraints();
  void create_fixed_routes_occupation_constraints();
  void create_fixed_route_schedule_constraints();
  void create_fixed_routes_impossibility_cuts();
  void create_fixed_routes_no_overlap_entry_exit_constraints();

  void create_non_discretized_general_constraints();
  void create_non_discretized_position_constraints();
  void create_non_discretized_free_route_constraints();
  void create_non_discretized_fixed_route_constraints();

  void create_free_routes_position_constraints();
  void create_free_routes_overlap_constraints();
  void create_boundary_free_routes_constraints();
  void create_free_routes_occupation_constraints();
  void create_free_routes_impossibility_cuts();
  void create_free_routes_no_overlap_entry_exit_constraints();

  // Objective
  void set_objective();

  // Helper functions
  [[nodiscard]] std::vector<size_t>
                       unbreakable_section_indices(size_t train_index) const;
  void                 calculate_fwd_bwd_sections();
  void                 calculate_fwd_bwd_sections_discretized();
  void                 calculate_fwd_bwd_sections_non_discretized();
  [[nodiscard]] double get_max_brakelen(const size_t& tr) const;

  [[nodiscard]] std::pair<std::vector<std::vector<size_t>>,
                          std::vector<std::vector<size_t>>>
  common_entry_exit_vertices() const;

  struct TemporaryImpossibilityStruct {
    bool                to_use;
    int                 t_before;
    int                 t_after;
    double              v_before;
    double              v_after;
    std::vector<size_t> edges_before;
    std::vector<size_t> edges_after;
  };
  [[nodiscard]] TemporaryImpossibilityStruct
  get_temporary_impossibility_struct(const size_t& tr, const int& t) const;

  [[nodiscard]] double
  max_distance_travelled(const size_t& tr, const int& time_steps,
                         const double& v0, const double& a_max,
                         const bool& braking_distance) const;

public:
  // Constructors
  explicit VSSGenTimetableSolver(instances::VSSGenerationTimetable instance);
  explicit VSSGenTimetableSolver(const std::filesystem::path& instance_path);
  explicit VSSGenTimetableSolver(const std::string& instance_path);
  explicit VSSGenTimetableSolver(const char* instance_path);

  // Methods
  int solve(int delta_t = 15, bool fix_routes_input = true,
            VSSModel vss_model_input = VSSModel::CONTINUOUS,
            const std::vector<SeparationType>& separation_types_input = {},
            bool include_train_dynamics_input                         = true,
            bool include_braking_curves_input                         = true,
            bool use_pwl_input = false, bool use_schedule_cuts_input = true,
            int time_limit = -1, bool debug = false,
            bool               export_to_file = false,
            const std::string& file_name      = "model");
};
} // namespace cda_rail::solver::mip_based
