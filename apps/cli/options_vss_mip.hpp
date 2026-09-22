#pragma once

#include "CLI/CLI.hpp"
#include "Definitions.hpp"
#include "VSSModel.hpp"
#include "options_common.hpp"
#include "solver/mip-based/GeneralMIPSolver.hpp"
#include "solver/mip-based/VSSGenTimetableSolver.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace cda_rail::cli {

/** @brief The separation functions that can be requested on the command line.
 */
enum class SeparationFunctionType : std::uint8_t { Uniform = 0, Chebyshev = 1 };

/**
 * @brief Every setting of the VSS generation MIP solver.
 *
 * Shared by `rail_vss_generation_timetable_mip_testing` and the
 * `solve vss-mip` subcommand of `rail_cli`. It covers both VSS generation
 * solvers: as soon as a moving block solution subdirectory is given, the
 * solver using moving block information is the one that runs.
 */
struct VssMipSettings {
  // moving block information
  std::string                moving_block_solution_subdirectory{};
  std::optional<std::string> moving_block_working_directory{};
  std::optional<std::string> moving_block_parameter_identifier{};
  bool                       fix_stop_positions{true};
  bool                       fix_exact_positions{true};
  bool                       fix_exact_velocities{true};
  bool                       hint_approximate_positions{true};
  bool                       fix_order_on_edges{true};

  // model parameters
  double         delta_t{15};
  bool           fix_routes{true};
  bool           train_dynamics{true};
  bool           braking_curves{true};
  vss::ModelType vss_model_type{vss::ModelType::Continuous};
  std::vector<SeparationFunctionType> separation_function_types{};
  bool                                only_stop_at_vss{false};
  bool                                use_pwl{false};
  bool                                use_schedule_cuts{true};

  // solver parameters
  bool               iterative_approach{false};
  OptimalityStrategy optimality_strategy{OptimalityStrategy::Optimal};
  solver::mip_based::UpdateStrategyVSSGen iterative_update_strategy{
      solver::mip_based::UpdateStrategyVSSGen::Fixed};
  double iterative_initial_value{1};
  double iterative_update_value{2};
  bool   iterative_include_cuts{true};

  SolvingSettings solving{};
  ExportSettings  exporting{};

  /** @brief Whether a moving block solution is used as a starting point. */
  [[nodiscard]] bool use_moving_block_information() const {
    return !moving_block_solution_subdirectory.empty();
  }
  /** @brief The separation functions belonging to the requested types. */
  [[nodiscard]] std::vector<vss::SeparationFunction>
  separation_functions() const;
};

/** @brief Adds every option of the VSS generation MIP solver to @p app. */
void add_vss_mip_options(CLI::App& app, VssMipSettings& settings);
/** @brief Generates the parameter identifier if it was asked for. */
void finalize_vss_mip_settings(VssMipSettings& settings);
/** @brief The identifier `--generate-parameter-identifier` produces. */
[[nodiscard]] std::string
generate_vss_mip_identifier(const VssMipSettings& settings);
/** @brief Logs the settings, as the apps do before they solve. */
void log_vss_mip_settings(const VssMipSettings& settings,
                          const std::string&    working_directory);

/**
 * @brief The model settings handed to the solver without moving block
 *        information.
 */
[[nodiscard]] solver::mip_based::ModelDetailVSSGen
vss_mip_model_detail(const VssMipSettings& settings);
/** @brief The model settings handed to the solver using it. */
[[nodiscard]] solver::mip_based::ModelDetailMBInformation
vss_mip_model_detail_mb(const VssMipSettings& settings);
/** @brief The VSS model settings handed to the solver. */
[[nodiscard]] solver::mip_based::ModelSettingsVSSGen
vss_mip_model_settings(const VssMipSettings& settings);
/** @brief The solver strategy handed to the solver. */
[[nodiscard]] solver::mip_based::SolverStrategyVSSGen
vss_mip_solver_strategy(const VssMipSettings& settings);
/**
 * @brief The solution settings handed to the solver.
 *
 * @param working_directory Used if no separate export working directory was
 *        given.
 */
[[nodiscard]] solver::mip_based::SolutionSettingsVSSGen
vss_mip_solution_settings(const VssMipSettings& settings,
                          const std::string&    working_directory);

} // namespace cda_rail::cli
