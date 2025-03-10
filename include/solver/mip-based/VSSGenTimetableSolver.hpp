#pragma once
#include "Definitions.hpp"
#include "GeneralMIPSolver.hpp"
#include "VSSModel.hpp"
#include "gurobi_c++.h"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "probleminstances/VSSGenerationTimetable.hpp"
#include "solver/GeneralSolver.hpp"
#include "unordered_map"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace cda_rail::solver::mip_based {

enum class UpdateStrategy : std::uint8_t { Fixed = 0, Relative = 1 };

struct SolverStrategy {
  bool                         iterative_approach = false;
  cda_rail::OptimalityStrategy optimality_strategy =
      cda_rail::OptimalityStrategy::Optimal;
  UpdateStrategy update_strategy = UpdateStrategy::Fixed;
  double         initial_value   = 1;
  double         update_value    = 2;
  bool           include_cuts    = true;
};

struct ModelDetail {
  int  delta_t        = 15;
  bool fix_routes     = true;
  bool train_dynamics = true;
  bool braking_curves = true;
};

struct ModelDetailMBInformation {
  int  delta_t                    = 15;
  bool train_dynamics             = true;
  bool braking_curves             = true;
  bool fix_stop_positions         = true;
  bool fix_exact_positions        = true;
  bool fix_exact_velocities       = true;
  bool hint_approximate_positions = true;
  bool fix_order_on_edges         = true;
};

struct ModelSettings {
  // NOLINTNEXTLINE(readability-redundant-member-init)
  vss::Model model_type        = vss::Model();
  bool       use_pwl           = false;
  bool       use_schedule_cuts = true;
};

class VSSGenTimetableSolver
    : public GeneralMIPSolver<instances::VSSGenerationTimetable,
                              instances::SolVSSGenerationTimetable> {
  friend class VSSGenTimetableSolverWithMovingBlockInformation;

private:
  // Instance variables
  int                                    dt                     = -1;
  size_t                                 num_t                  = 0;
  size_t                                 num_tr                 = 0;
  size_t                                 num_edges              = 0;
  size_t                                 num_vertices           = 0;
  size_t                                 num_breakable_sections = 0;
  std::vector<std::vector<size_t>>       unbreakable_sections;
  std::vector<std::vector<size_t>>       no_border_vss_sections;
  std::vector<std::pair<size_t, size_t>> train_interval;
  std::vector<std::pair<std::optional<size_t>, std::optional<size_t>>>
                      breakable_edges_pairs;
  std::vector<size_t> no_border_vss_vertices;
  std::vector<size_t> relevant_edges;
  std::vector<size_t> breakable_edges;
  bool                fix_routes = false;
  vss::Model          vss_model  = vss::Model(vss::ModelType::Continuous);
  bool                include_train_dynamics     = false;
  bool                include_braking_curves     = false;
  bool                use_pwl                    = false;
  bool                use_schedule_cuts          = false;
  bool                iterative_vss              = false;
  OptimalityStrategy  optimality_strategy        = OptimalityStrategy::Optimal;
  UpdateStrategy      iterative_update_strategy  = UpdateStrategy::Fixed;
  double              iterative_initial_value    = 1;
  double              iterative_update_value     = 2;
  bool                iterative_include_cuts     = true;
  bool                iterative_include_cuts_tmp = true;
  bool                postprocess                = false;
  ExportOption        export_option              = ExportOption::NoExport;
  std::vector<size_t> max_vss_per_edge_in_iteration;
  std::unordered_map<size_t, size_t> breakable_edge_indices;
  std::vector<std::pair<std::vector<size_t>, std::vector<size_t>>>
      fwd_bwd_sections;

  // Variable functions
  void create_variables();
  void create_general_variables();
  void create_fixed_routes_variables();
  void create_free_routes_variables();
  void create_discretized_variables();
  void create_non_discretized_variables();
  void create_brakelen_variables();
  void create_only_stop_at_vss_variables();
  void create_non_discretized_only_stop_at_vss_variables();

  // Constraint functions
  void create_constraints();
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
  void create_non_discretized_fraction_constraints();
  void create_non_discretized_alt_fraction_constraints();
  void create_non_discretized_general_only_stop_at_vss_constraints();
  void create_non_discretized_free_routes_only_stop_at_vss_constraints();
  void create_non_discretized_fixed_routes_only_stop_at_vss_constraints();

  void create_free_routes_position_constraints();
  void create_free_routes_overlap_constraints();
  void create_boundary_free_routes_constraints();
  void create_free_routes_occupation_constraints();
  void create_free_routes_impossibility_cuts();
  void create_free_routes_no_overlap_entry_exit_constraints();

  // Objective
  void set_objective();

  // Helper functions
  void set_timeout(int time_limit);
  [[nodiscard]] std::optional<instances::SolVSSGenerationTimetable>
  optimize(const std::optional<instances::VSSGenerationTimetable>& old_instance,
           int                                                     time_limit);
  void export_lp_if_applicable(const SolutionSettings& solution_settings);
  void export_solution_if_applicable(
      const std::optional<cda_rail::instances::SolVSSGenerationTimetable>&
                              sol_object,
      const SolutionSettings& solution_settings);
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
    size_t              t_before;
    size_t              t_after;
    double              v_before;
    double              v_after;
    std::vector<size_t> edges_before;
    std::vector<size_t> edges_after;
  };
  [[nodiscard]] TemporaryImpossibilityStruct
  get_temporary_impossibility_struct(const size_t& tr, const size_t& t) const;

  [[nodiscard]] double
  max_distance_travelled(const size_t& tr, const size_t& time_steps,
                         const double& v0, const double& a_max,
                         const bool& braking_distance) const;

  virtual void cleanup() override;

  [[nodiscard]] instances::SolVSSGenerationTimetable
  extract_solution(bool postprocess, bool full_model,
                   const std::optional<instances::VSSGenerationTimetable>&
                       old_instance) const;

  bool update_vss(size_t relevant_edge_index, double obj_ub,
                  GRBLinExpr& cut_expr);
  void update_max_vss_on_edge(size_t relevant_edge_index, size_t new_max_vss,
                              GRBLinExpr& cut_expr);
  [[nodiscard]] std::optional<instances::VSSGenerationTimetable>
  initialize_variables(const ModelDetail&      model_detail,
                       const ModelSettings&    model_settings,
                       const SolverStrategy&   solver_strategy,
                       const SolutionSettings& solution_settings,
                       int time_limit, bool debug_input);

protected:
  void solve_init_vss_gen_timetable(int time_limit, bool debug_input) {
    this->solve_init_general_mip(time_limit, debug_input);
  };

public:
  // Constructors
  explicit VSSGenTimetableSolver(
      const instances::VSSGenerationTimetable& instance);
  explicit VSSGenTimetableSolver(const std::filesystem::path& instance_path);
  explicit VSSGenTimetableSolver(const std::string& instance_path);
  explicit VSSGenTimetableSolver(const char* instance_path);

  // Methods
  [[nodiscard]] instances::SolVSSGenerationTimetable
  solve(const ModelDetail&      model_detail,
        const ModelSettings&    model_settings    = {},
        const SolverStrategy&   solver_strategy   = {},
        const SolutionSettings& solution_settings = {}, int time_limit = -1,
        bool debug_input = false);

  using GeneralSolver::solve;
  [[nodiscard]] virtual instances::SolVSSGenerationTimetable
  solve(int time_limit, bool debug_input) override {
    return solve({}, {}, {}, {}, time_limit, debug_input);
  }
};

class VSSGenTimetableSolverWithMovingBlockInformation
    : public VSSGenTimetableSolver {
private:
  instances::SolGeneralPerformanceOptimizationInstance<
      instances::GeneralPerformanceOptimizationInstance>
       moving_block_solution;
  bool fix_orders_on_edges        = true;
  bool fix_stop_positions         = true;
  bool fix_exact_positions        = true;
  bool fix_exact_velocities       = true;
  bool hint_approximate_positions = true;

  // Additional functions
  void include_additional_information();
  void fix_oder_on_edges();
  void fix_stop_positions_constraints();
  void fix_exact_positions_and_velocities_constraints();
  void hint_approximate_positions_constraints();

  virtual void cleanup() override;

public:
  explicit VSSGenTimetableSolverWithMovingBlockInformation(
      const instances::SolGeneralPerformanceOptimizationInstance<
          instances::GeneralPerformanceOptimizationInstance>&
           moving_block_solution_tmp,
      bool throw_error = true)
      : VSSGenTimetableSolver(
            moving_block_solution_tmp.get_instance().cast_to_vss_generation(
                throw_error)),
        moving_block_solution(moving_block_solution_tmp) {};
  explicit VSSGenTimetableSolverWithMovingBlockInformation(
      const std::filesystem::path& sol_path)
      : VSSGenTimetableSolverWithMovingBlockInformation(
            instances::SolGeneralPerformanceOptimizationInstance<
                instances::GeneralPerformanceOptimizationInstance>(sol_path)) {
        };
  explicit VSSGenTimetableSolverWithMovingBlockInformation(
      const std::string& sol_path)
      : VSSGenTimetableSolverWithMovingBlockInformation(
            std::filesystem::path(sol_path)) {};
  explicit VSSGenTimetableSolverWithMovingBlockInformation(const char* sol_path)
      : VSSGenTimetableSolverWithMovingBlockInformation(
            std::filesystem::path(sol_path)) {};

  [[nodiscard]] instances::SolVSSGenerationTimetable
  solve(const ModelDetailMBInformation& model_detail_mb_information,
        const ModelSettings&            model_settings  = {},
        const SolverStrategy&           solver_strategy = {},
        const SolutionSettings& solution_settings = {}, int time_limit = -1,
        bool debug_input = false);

  using GeneralSolver::solve;
  [[nodiscard]] virtual instances::SolVSSGenerationTimetable
  solve(int time_limit, bool debug_input) override {
    return solve({}, {}, {}, {}, time_limit, debug_input);
  }
};

} // namespace cda_rail::solver::mip_based
