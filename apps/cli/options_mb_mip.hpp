#pragma once

#include "CLI/CLI.hpp"
#include "Definitions.hpp"
#include "options_common.hpp"
#include "solver/mip-based/GenPOMovingBlockMIPSolver.hpp"

#include <string>

namespace cda_rail::cli {

/**
 * @brief Every setting of the moving block MIP solver.
 *
 * The block is shared by `rail_gen_po_moving_block_mip_testing` and the
 * `solve mb-mip` subcommand of `rail_cli`, so that the two cannot drift apart.
 */
struct MbMipSettings {
  // model parameters
  bool                       fix_routes{false};
  double                     max_velocity_delta{5.55}; // 20 km/h
  VelocityRefinementStrategy velocity_refinement_strategy{
      VelocityRefinementStrategy::MinOneStep};
  bool   simplify_headway_constraints{false};
  bool   strengthen_vertex_headway_constraints{false};
  bool   late_entry_possible{false};
  bool   use_minimum_time_bounds{true};
  double max_exit_delay{solver::mip_based::DEFAULT_MAX_DELAY};
  double max_station_delay{solver::mip_based::DEFAULT_MAX_DELAY};
  double max_delay{solver::mip_based::DEFAULT_MAX_DELAY};

  // solver parameters
  bool use_indicator_constraints{false};
  bool use_lazy_constraints{true};
  bool include_reverse_headways{false};
  bool include_higher_velocities_in_edge_expr{false};
  solver::mip_based::LazyConstraintSelectionStrategy
      lazy_constraint_selection_strategy{
          solver::mip_based::LazyConstraintSelectionStrategy::OnlyViolated};
  solver::mip_based::LazyTrainSelectionStrategy lazy_train_selection_strategy{
      solver::mip_based::LazyTrainSelectionStrategy::OnlyAdjacent};
  double abs_mip_gap{10};

  SolvingSettings solving{};
  ExportSettings  exporting{};
};

/** @brief Adds every option of the moving block MIP solver to @p app. */
void add_mb_mip_options(CLI::App& app, MbMipSettings& settings);

/**
 * @brief Applies the settings that depend on each other after parsing.
 *
 * Resolves `--max-delay` into the two individual delays and generates the
 * parameter identifier if `--generate-parameter-identifier` was given.
 */
void finalize_mb_mip_settings(CLI::App& app, MbMipSettings& settings);

/** @brief The identifier `--generate-parameter-identifier` produces. */
[[nodiscard]] std::string
generate_mb_mip_identifier(const MbMipSettings& settings);

/** @brief Logs the settings, as the apps do before they solve. */
void log_mb_mip_settings(const MbMipSettings& settings,
                         const std::string&   working_directory);

/** @brief The model settings handed to the solver. */
[[nodiscard]] solver::mip_based::ModelDetailMovingBlock
mb_mip_model_detail(const MbMipSettings& settings);
/** @brief The solver strategy handed to the solver. */
[[nodiscard]] solver::mip_based::SolverStrategyMovingBlock
mb_mip_solver_strategy(const MbMipSettings& settings);
/**
 * @brief The solution settings handed to the solver.
 *
 * @param working_directory Used if no separate export working directory was
 *        given.
 */
[[nodiscard]] solver::mip_based::SolutionSettingsMovingBlock
mb_mip_solution_settings(const MbMipSettings& settings,
                         const std::string&   working_directory);

} // namespace cda_rail::cli
