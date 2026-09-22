#pragma once

#include "CLI/CLI.hpp"
#include "Definitions.hpp"
#include "solver/GeneralSolver.hpp"

#include <optional>
#include <string>
#include <string_view>

namespace cda_rail::cli {

/**
 * @brief The instance a solver app reads, given on the command line.
 *
 * `rail_cli` does not use this: there the solvers run on the instance the
 * session already holds.
 */
struct InstanceSettings {
  std::string instance_name;
  std::string instance_subdirectory;
  std::string working_directory;
};

void add_instance_options(CLI::App& app, InstanceSettings& settings);
void log_instance_settings(const InstanceSettings& settings);

/**
 * @brief The export block that all three solver apps share.
 *
 * `export_working_directory` is left empty if the option was not given, in
 * which case the caller substitutes its own working directory.
 */
struct ExportSettings {
  solver::GeneralExportOption export_option{
      solver::GeneralExportOption::NoExport};
  bool                       export_lp_model{false};
  bool                       postprocess{false};
  std::string                model_name{"model"};
  std::string                solution_subdirectory{};
  std::optional<std::string> export_working_directory{};
  std::optional<std::string> parameter_identifier{};
  bool                       generate_identifier{false};
};

/**
 * @brief Which of the optional export options a solver supports.
 *
 * The A* solver builds no model and postprocesses nothing, so it offers
 * neither; only the VSS generation solver can drop unused VSS afterwards.
 */
struct ExportOptionSupport {
  bool lp_model{false};
  bool postprocess{false};
  /**
   * @brief Extension under which the solution of the exported model is
   *        written, which differs between the two MIP solvers.
   */
  std::string_view lp_model_solution_extension{".json"};
};

void add_export_options(CLI::App& app, ExportSettings& settings,
                        ExportOptionSupport support);
void log_export_settings(const ExportSettings& settings,
                         const std::string&    working_directory);

/**
 * @brief Fills the `GeneralSolutionSettings` part of a solver's settings.
 *
 * @param working_directory Used if no separate export working directory was
 *        given.
 */
solver::GeneralSolutionSettings
general_solution_settings(const ExportSettings& settings,
                          const std::string&    working_directory);

/** @brief Time limit and debug output, shared by all three solvers. */
struct SolvingSettings {
  int  time_limit{-1};
  bool debug_output{false};
};

void add_solving_options(CLI::App& app, SolvingSettings& settings);
void log_solving_settings(const SolvingSettings& settings);

/** @brief Logs the status and objective of a solution, as the apps do. */
void log_solution_status(SolutionStatus status, double objective);

} // namespace cda_rail::cli
