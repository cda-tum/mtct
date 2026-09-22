#include "options_common.hpp"

#include "CLI/CLI.hpp"
#include "Definitions.hpp"
#include "StringHelper.hpp"
#include "solver/GeneralSolver.hpp"

#include <plog/Log.h>
#include <string>

// The reinterpret_cast warnings are false positives stemming from the plog
// macros.
// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast)

void cda_rail::cli::add_instance_options(CLI::App&         app,
                                         InstanceSettings& settings) {
  app.add_option(
         "-n,--instance-name", settings.instance_name,
         "Name of the instance to solve. Will load instance in "
         "working_directory/instances/instance_subdirectory/instance_name.")
      ->required()
      ->group("Instance");
  app.add_option(
         "-s,--instance-subdirectory", settings.instance_subdirectory,
         "Subdirectory of the instance to solve. Will load instance in "
         "working_directory/instances/instance_subdirectory/instance_name.")
      ->required()
      ->group("Instance");
  app.add_option(
         "-d,--working-directory", settings.working_directory,
         "Working directory. Will load instance in "
         "working_directory/instances/instance_subdirectory/instance_name.")
      ->required()
      ->group("Instance");
}

void cda_rail::cli::log_instance_settings(const InstanceSettings& settings) {
  PLOGD << "Instance Settings";
  PLOGD << "  Instance name: " << settings.instance_name;
  PLOGD << "  Instance subdirectory: " << settings.instance_subdirectory;
  PLOGD << "  Working directory: " << settings.working_directory;
}

void cda_rail::cli::add_export_options(CLI::App& app, ExportSettings& settings,
                                       ExportOptionSupport const support) {
  auto* export_sol_flag =
      app.add_flag(
             "-o,--export-solution",
             [&settings](int) {
               settings.export_option =
                   solver::GeneralExportOption::ExportSolution;
             },
             "Export the solution.")
          ->group("Export Options");
  auto* export_sol_inst_flag =
      app.add_flag(
             "-i,--export-solution-and-instance",
             [&settings](int) {
               settings.export_option =
                   solver::GeneralExportOption::ExportSolutionWithInstance;
             },
             "Export the solution and the instance.")
          ->group("Export Options");
  CLI::Option* export_lp_flag = nullptr;
  if (support.lp_model) {
    const auto lp_model_description = concatenate_string_views(
        {"Export the MIP model itself (as .mps and, if a solution exists, as ",
         support.lp_model_solution_extension,
         ") into the solution directory."});
    export_lp_flag = app.add_flag("--export-lp-model", settings.export_lp_model,
                                  lp_model_description)
                         ->group("Export Options");
  }

  // The lambda has to capture the option pointers by value: they are looked up
  // whenever an option is checked, which outlives this function.
  CLI::Validator requires_export_option(
      [export_sol_flag, export_sol_inst_flag, export_lp_flag](std::string&) {
        if (export_sol_flag->count() == 0 &&
            export_sol_inst_flag->count() == 0 &&
            (export_lp_flag == nullptr || export_lp_flag->count() == 0)) {
          return export_lp_flag == nullptr
                     ? std::string{"requires either --export-solution or "
                                   "--export-solution-and-instance"}
                     : std::string{"requires either --export-solution, "
                                   "--export-solution-and-instance or "
                                   "--export-lp-model"};
        }
        return std::string{};
      },
      support.lp_model ? " Needs: --export-solution, "
                         "--export-solution-and-instance or --export-lp-model"
                       : " Needs: --export-solution or "
                         "--export-solution-and-instance");
  requires_export_option.non_modifying();

  app.add_option("-b,--export-working-directory",
                 settings.export_working_directory,
                 "Working directory for exporting solutions. If unset, the "
                 "normal working directory is used.")
      ->check(requires_export_option)
      ->group("Export Options");
  auto* solution_subdir_opt =
      app.add_option(
             "-e,--solution-export-subdirectory",
             settings.solution_subdirectory,
             "Subdirectory to export the solution to. Will be created in "
             "export_working_directory/solutions/solution_subdirectory/"
             "instance_subdirectory/instance_name-parameters.")
          ->check(requires_export_option)
          ->group("Export Options");
  if (support.lp_model) {
    app.add_option("--model-name", settings.model_name,
                   "File name (without extension) used when exporting the "
                   "MIP model itself.")
        ->check(requires_export_option)
        ->capture_default_str()
        ->needs(export_lp_flag)
        ->group("Export Options");
  }
  if (support.postprocess) {
    app.add_flag("--postprocess", settings.postprocess,
                 "If this flag is set, the solution is postprocessed to "
                 "remove potentially unused VSS.")
        ->group("Export Options");
  }

  export_sol_flag->needs(solution_subdir_opt);
  export_sol_inst_flag->needs(solution_subdir_opt);
  export_sol_flag->excludes(export_sol_inst_flag);
  export_sol_inst_flag->excludes(export_sol_flag);
  if (export_lp_flag != nullptr) {
    export_lp_flag->needs(solution_subdir_opt);
  }

  auto* parameter_identifier_option =
      app.add_option(
             "-p,--parameter-identifier", settings.parameter_identifier,
             "Optional identifier to distinguish different parameterizations "
             "of the same instance. Will be appended to the instance name in "
             "the export path as instance_name-parameter_identifier. If "
             "empty, no parameter identifier will be appended")
          ->capture_default_str()
          ->group("Export Options");
  app.add_flag("-g,--generate-parameter-identifier",
               settings.generate_identifier,
               "Whether to automatically generate a parameter identifier "
               "based on the parameter settings. If set, the parameter "
               "identifier will be generated as a concatenation of the "
               "parameter names and values. Otherwise the identifier has to "
               "be set explicitly if it should not remain empty.")
      ->excludes(parameter_identifier_option)
      ->group("Export Options");
}

void cda_rail::cli::log_export_settings(const ExportSettings& settings,
                                        const std::string& working_directory) {
  PLOGD << "Export Settings";
  switch (settings.export_option) {
  case solver::GeneralExportOption::NoExport:
    PLOGD << "  Export option: No export";
    break;
  case solver::GeneralExportOption::ExportSolution:
    PLOGD << "  Export option: Export solution";
    break;
  case solver::GeneralExportOption::ExportSolutionWithInstance:
    PLOGD << "  Export option: Export solution with instance";
    break;
  }
  PLOGD << "  Export MIP model: " << (settings.export_lp_model ? "yes" : "no");
  PLOGD << "  MIP model name: " << settings.model_name;
  PLOGD << "  Postprocess solution: " << (settings.postprocess ? "yes" : "no");
  PLOGD << "  Export working directory: "
        << settings.export_working_directory.value_or(working_directory);
  PLOGD << "  Solution export subdirectory: " << settings.solution_subdirectory;
  PLOGD << "  Parameter identifier: "
        << settings.parameter_identifier.value_or("-")
        << (settings.generate_identifier ? " (generated)" : "");
}

cda_rail::solver::GeneralSolutionSettings
cda_rail::cli::general_solution_settings(const ExportSettings& settings,
                                         const std::string& working_directory) {
  return {.export_option = settings.export_option,
          .working_directory =
              settings.export_working_directory.value_or(working_directory),
          .solution_subdirectory = settings.solution_subdirectory,
          .parameter_identifier  = settings.parameter_identifier};
}

void cda_rail::cli::add_solving_options(CLI::App&        app,
                                        SolvingSettings& settings) {
  app.add_option(
         "-t,--time-limit", settings.time_limit,
         "Time limit in seconds for the solver to run. No limit if negative.")
      ->capture_default_str()
      ->group("Additional Solving Parameters");
  app.add_flag("-v,--verbose,--debug", settings.debug_output,
               "Whether to output debug information during the solving "
               "process. Default: no debug output.")
      ->group("Additional Solving Parameters");
}

void cda_rail::cli::log_solving_settings(const SolvingSettings& settings) {
  PLOGD << "Additional Solving Parameters";
  PLOGD << "  Time limit: " << settings.time_limit << " seconds";
  PLOGD << "  Debug output: " << (settings.debug_output ? "yes" : "no");
}

void cda_rail::cli::log_solution_status(SolutionStatus const status,
                                        double const         objective) {
  std::string sol_status{"ERROR"};
  switch (status) {
  case SolutionStatus::Optimal:
    sol_status = "Optimal";
    break;
  case SolutionStatus::Feasible:
    sol_status = "Feasible";
    break;
  case SolutionStatus::Infeasible:
    sol_status = "Infeasible";
    break;
  case SolutionStatus::Timeout:
    sol_status = "Timeout";
    break;
  case SolutionStatus::Unknown:
    sol_status = "Unknown";
    break;
  }
  PLOGI << "Solution status: " << sol_status;
  PLOGI << "Solution objective: " << objective;
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)
