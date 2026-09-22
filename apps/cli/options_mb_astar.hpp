#pragma once

#include "CLI/CLI.hpp"
#include "options_common.hpp"
#include "simulator/GreedyHeuristic.hpp"
#include "solver/astar-based/GenPOMovingBlockAStarSolver.hpp"

#include <string>

namespace cda_rail::cli {

/**
 * @brief Every setting of the moving block A* solver.
 *
 * Shared by `rail_gen_po_moving_block_astar_testing` and the `solve mb-astar`
 * subcommand of `rail_cli`.
 */
struct MbAStarSettings {
  double                                 dt{6};
  bool                                   late_entry_possible{false};
  bool                                   limit_speed_by_leaving_edges{true};
  bool                                   consider_earliest_exit{true};
  bool                                   time_aware_state_transitions{false};
  double                                 a_star_weight{1.0};
  solver::astar_based::NextStateStrategy next_state_strategy{
      solver::astar_based::NextStateStrategy::SingleEdge};
  simulator::RemainingTimeHeuristicType remaining_time_heuristic_type{
      simulator::RemainingTimeHeuristicType::Simple};

  SolvingSettings solving{};
  ExportSettings  exporting{};
};

/** @brief Adds every option of the moving block A* solver to @p app. */
void add_mb_astar_options(CLI::App& app, MbAStarSettings& settings);
/** @brief Generates the parameter identifier if it was asked for. */
void finalize_mb_astar_settings(MbAStarSettings& settings);
/** @brief The identifier `--generate-parameter-identifier` produces. */
[[nodiscard]] std::string
generate_mb_astar_identifier(const MbAStarSettings& settings);
/** @brief Logs the settings, as the apps do before they solve. */
void log_mb_astar_settings(const MbAStarSettings& settings,
                           const std::string&     working_directory);

/** @brief The model settings handed to the solver. */
[[nodiscard]] solver::astar_based::ModelDetailMBAStar
mb_astar_model_detail(const MbAStarSettings& settings);
/** @brief The solver strategy handed to the solver. */
[[nodiscard]] solver::astar_based::SolverStrategyMBAStar
mb_astar_solver_strategy(const MbAStarSettings& settings);

} // namespace cda_rail::cli
