#pragma once
#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "GeneralMIPSolver.hpp"
#include "VSSModel.hpp"
#include "datastructure/Timetable.hpp"
#include "gurobi_c++.h"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "solver/GeneralSolver.hpp"
#include "unordered_map"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace cda_rail::solver::mip_based {
using std::size_t;

enum class UpdateStrategy : std::uint8_t { Fixed = 0, Relative = 1 };

constexpr std::string update_strategy_to_string(UpdateStrategy strategy) {
  switch (strategy) {
  case UpdateStrategy::Fixed:
    return "Fixed";
  case UpdateStrategy::Relative:
    return "Relative";
  default:
    throw cda_rail::exceptions::ConsistencyException("Unknown update strategy");
  }
}

constexpr std::string
optimality_strategy_to_string(OptimalityStrategy strategy) {
  switch (strategy) {
  case OptimalityStrategy::Optimal:
    return "Optimal";
  case OptimalityStrategy::TradeOff:
    return "TradeOff";
  case OptimalityStrategy::Feasible:
    return "Feasible";
  default:
    throw cda_rail::exceptions::ConsistencyException(
        "Unknown optimality strategy");
  }
}

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
  double delta_t        = 15;
  bool   fix_routes     = true;
  bool   train_dynamics = true;
  bool   braking_curves = true;
};

struct ModelDetailMBInformation {
  double delta_t                    = 15;
  bool   train_dynamics             = true;
  bool   braking_curves             = true;
  bool   fix_stop_positions         = true;
  bool   fix_exact_positions        = true;
  bool   fix_exact_velocities       = true;
  bool   hint_approximate_positions = true;
  bool   fix_order_on_edges         = true;
};

struct ModelSettings {
  // NOLINTNEXTLINE(readability-redundant-member-init)
  vss::Model model_type{};
  bool       use_pwl{false};
  bool       use_schedule_cuts{true};
};

class VSSGenTimetableSolver
    : public GeneralMIPSolver<
          instances::GeneralPerformanceOptimizationInstance,
          instances::SolVSSGeneralPerformanceOptimizationInstance> {
  friend class VSSGenTimetableSolverWithMovingBlockInformation;

private:
  // Instance variables
  double                                 dt{-1};
  size_t                                 num_t{0};
  size_t                                 num_tr{0};
  size_t                                 num_edges{0};
  size_t                                 num_vertices{0};
  size_t                                 num_breakable_sections{0};
  std::vector<cda_rail::index_vector>    unbreakable_sections;
  std::vector<cda_rail::index_vector>    no_border_vss_sections;
  std::vector<std::pair<size_t, size_t>> train_interval;
  std::vector<std::pair<std::optional<size_t>, std::optional<size_t>>>
                         breakable_edges_pairs;
  cda_rail::index_vector no_border_vss_vertices;
  cda_rail::index_vector relevant_edges;
  cda_rail::index_vector breakable_edges;
  bool                   fix_routes{false};
  vss::Model             vss_model{
      vss::ModelType::Continuous}; // This line is correct and intended.
                                   // vss_model is of type vss::Model
  bool                   include_train_dynamics{false};
  bool                   include_braking_curves{false};
  bool                   use_pwl{false};
  bool                   use_schedule_cuts{false};
  bool                   iterative_vss{false};
  OptimalityStrategy     optimality_strategy{OptimalityStrategy::Optimal};
  UpdateStrategy         iterative_update_strategy{UpdateStrategy::Fixed};
  double                 iterative_initial_value{1.0};
  double                 iterative_update_value{2.0};
  bool                   iterative_include_cuts{true};
  bool                   iterative_include_cuts_tmp{true};
  bool                   postprocess{false};
  cda_rail::index_vector max_vss_per_edge_in_iteration{};
  std::unordered_map<size_t, size_t> breakable_edge_indices{};
  std::vector<std::pair<cda_rail::index_vector, cda_rail::index_vector>>
      fwd_bwd_sections{};

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
  [[nodiscard]] std::optional<
      instances::SolVSSGeneralPerformanceOptimizationInstance>
  optimize(const std::optional<
               instances::GeneralPerformanceOptimizationInstance>& old_instance,
           int                                                     time_limit);
  /**
   * @brief Exports the MIP model itself if solution_settings ask for it.
   *
   * The model is written into the standard solution directory of the given
   * solution, i.e., to the very same place the solution itself is exported to,
   * using solution_settings.model_name as file name. This happens
   * independently of the export option, i.e., the model can be exported
   * without the solution and vice versa.
   *
   * @param sol_object The solution defining the export directory
   * @param solution_settings Settings describing where and what to export
   */
  void export_lp_model_if_applicable(
      const instances::SolVSSGeneralPerformanceOptimizationInstance& sol_object,
      const SolutionSettingsVSSGen& solution_settings);
  /**
   * @brief Collects the solver settings written to solver_data.json.
   *
   * @param solution_settings The export settings used
   * @param time_limit The time limit in seconds passed to the solver
   * @return The settings as further data of the solver data export
   */
  [[nodiscard]] FurtherData
  get_further_data(const SolutionSettingsVSSGen& solution_settings,
                   int                           time_limit) const;
  [[nodiscard]] cda_rail::index_vector
                       unbreakable_section_indices(size_t train_index) const;
  void                 calculate_fwd_bwd_sections();
  void                 calculate_fwd_bwd_sections_discretized();
  void                 calculate_fwd_bwd_sections_non_discretized();
  [[nodiscard]] double get_max_brakelen(const size_t& tr) const;

  [[nodiscard]] std::pair<std::vector<cda_rail::index_vector>,
                          std::vector<cda_rail::index_vector>>
  common_entry_exit_vertices() const;

  struct TemporaryImpossibilityStruct {
    bool                   to_use;
    size_t                 t_before;
    size_t                 t_after;
    double                 v_before;
    double                 v_after;
    cda_rail::index_vector edges_before;
    cda_rail::index_vector edges_after;
  };
  [[nodiscard]] TemporaryImpossibilityStruct
  get_temporary_impossibility_struct(const size_t& tr, const size_t& t) const;

  /**
   * @brief Time step indices during which a train is serviced at a stop.
   *
   * The stop itself is given in continuous time, the model only knows time step
   * indices. The first index is the last time step at or before the start of
   * the service, the second index the first time step at or after the earliest
   * departure.
   *
   * @param stop The scheduled stop in question.
   * @return Pair of time step indices (t0, t1) belonging to the stop.
   */
  [[nodiscard]] std::pair<size_t, size_t>
  stop_time_indices(const ScheduledStop& stop) const;

  [[nodiscard]] double
  max_distance_travelled(const size_t& tr, const size_t& time_steps,
                         const double& v0, const double& a_max,
                         const bool& braking_distance) const;

  [[nodiscard]] instances::SolVSSGeneralPerformanceOptimizationInstance
  extract_solution(
      bool postprocess, bool full_model,
      const std::optional<instances::GeneralPerformanceOptimizationInstance>&
          old_instance) const;

  bool update_vss(size_t relevant_edge_index, double obj_ub,
                  GRBLinExpr& cut_expr);
  void update_max_vss_on_edge(size_t relevant_edge_index, size_t new_max_vss,
                              GRBLinExpr& cut_expr);
  [[nodiscard]] std::optional<instances::GeneralPerformanceOptimizationInstance>
  initialize_variables(const ModelDetail&            model_detail,
                       const ModelSettings&          model_settings,
                       const SolverStrategy&         solver_strategy,
                       const SolutionSettingsVSSGen& solution_settings,
                       int time_limit, bool debug_input,
                       bool overwrite_severity);

protected:
  void cleanup() override;

  void solve_init_vss_gen_timetable(bool debug_input, bool overwrite_severity) {
    this->solve_init_general_mip(debug_input, overwrite_severity);
  };

public:
  ~VSSGenTimetableSolver() override = default;
  // Constructors
  explicit VSSGenTimetableSolver(
      const instances::GeneralPerformanceOptimizationInstance& instance)
      : GeneralMIPSolver(instance) {};
  template <typename... Args>
  explicit VSSGenTimetableSolver(Args&&... args)
    requires(!IsSingleInstanceArgument<Args...>::value)
      : GeneralMIPSolver(std::forward<Args>(args)...) {}

  // Methods
  [[nodiscard]] instances::SolVSSGeneralPerformanceOptimizationInstance solve(
      const ModelDetail& model_detail, const ModelSettings& model_settings = {},
      const SolverStrategy&         solver_strategy   = {},
      const SolutionSettingsVSSGen& solution_settings = {}, int time_limit = -1,
      bool debug_input = false, bool overwrite_severity = true);

  using GeneralSolver::solve;
  [[nodiscard]] instances::SolVSSGeneralPerformanceOptimizationInstance
  solve(int time_limit, bool debug_input, bool overwrite_severity) override {
    return solve({}, {}, {}, {}, time_limit, debug_input, overwrite_severity);
  }
};

class VSSGenTimetableSolverWithMovingBlockInformation
    : public VSSGenTimetableSolver {
private:
  instances::SolGeneralPerformanceOptimizationInstance m_moving_block_solution;
  bool m_fix_orders_on_edges{true};
  bool m_fix_stop_positions{true};
  bool m_fix_exact_positions{true};
  bool m_fix_exact_velocities{true};
  bool m_hint_approximate_positions{true};

  // Additional functions
  void include_additional_information();
  void fix_oder_on_edges();
  void fix_stop_positions_constraints();
  void fix_exact_positions_and_velocities_constraints();
  void hint_approximate_positions_constraints();

protected:
  void cleanup() override;

public:
  ~VSSGenTimetableSolverWithMovingBlockInformation() override = default;
  // Constructors
  explicit VSSGenTimetableSolverWithMovingBlockInformation(
      const instances::SolGeneralPerformanceOptimizationInstance&
          moving_block_solution)
      : VSSGenTimetableSolver(*moving_block_solution.get_instance()),
        m_moving_block_solution(moving_block_solution) {
    // All information extracted from the moving block solution (train orders,
    // positions, velocities, ...) refers to the routes of that solution, which
    // may differ from the routes specified in the instance. Since this solver
    // always uses fixed routes, the instance routes are replaced by the ones
    // of the moving block solution.
    m_instance.set_routes(m_moving_block_solution.get_const_solution_routes());
  };

  // Methods
  [[nodiscard]] instances::SolVSSGeneralPerformanceOptimizationInstance
  solve(const ModelDetailMBInformation& model_detail_mb_information,
        const ModelSettings&            model_settings    = {},
        const SolverStrategy&           solver_strategy   = {},
        const SolutionSettingsVSSGen&   solution_settings = {},
        int time_limit = -1, bool debug_input = false,
        bool overwrite_severity = true);

  using GeneralSolver::solve;
  [[nodiscard]] instances::SolVSSGeneralPerformanceOptimizationInstance
  solve(int time_limit, bool debug_input, bool overwrite_severity) override {
    return solve({}, {}, {}, {}, time_limit, debug_input, overwrite_severity);
  }
};

} // namespace cda_rail::solver::mip_based
