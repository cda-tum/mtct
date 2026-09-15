#include "CLI/CLI.hpp"
#include "Definitions.hpp"
#include "StringHelper.hpp"
#include "plog/Init.h"
#include "plog/Logger.h"
#include "plog/Severity.h"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "solver/GeneralSolver.hpp"
#include "solver/mip-based/GenPOMovingBlockMIPSolver.hpp"
#include "solver/mip-based/GeneralMIPSolver.hpp"

#include <iomanip>
#include <ios>
#include <map>
#include <optional>
#include <plog/Appenders/ColorConsoleAppender.h>
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Log.h>
#include <sstream>
#include <string>

// Settings that also exist in the A* app (gen_po_moving_block_astar_testing)
// use the very same short and long options there:
//   -n -s -d (instance), -l (late entry), -t (time limit), -v (debug),
//   -o -i -b -e -p -g (export).
// The remaining letters are assigned to the MIP specific settings; since the
// two solvers barely share any parameters, a letter may well have a different
// meaning in the A* app. Since all lowercase letters are used up,
// --max-station-delay, --max-delay, --export-lp-model and --model-name are
// long option only.

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,bugprone-exception-escape)

int main(int argc, char** argv) {
  // Only log to console using std::cerr and std::cout respectively unless
  // initialized differently
  if (plog::get() == nullptr) {
    static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
    plog::init(plog::debug, &console_appender);
  }

  CLI::App app{"Moving Block Optimization using a MIP"};
  argv = app.ensure_utf8(argv);
  app.option_defaults()->multi_option_policy(CLI::MultiOptionPolicy::Throw);

  // instance definition
  std::string instance_name{};
  std::string instance_subdirectory{};
  std::string working_directory{};

  app.add_option(
         "-n,--instance-name", instance_name,
         "Name of the instance to solve. Will load instance in "
         "working_directory/instances/instance_subdirectory/instance_name.")
      ->required()
      ->group("Instance");
  app.add_option(
         "-s,--instance-subdirectory", instance_subdirectory,
         "Subdirectory of the instance to solve. Will load instance in "
         "working_directory/instances/instance_subdirectory/instance_name.")
      ->required()
      ->group("Instance");
  app.add_option(
         "-d,--working-directory", working_directory,
         "Working directory. Will load instance in "
         "working_directory/instances/instance_subdirectory/instance_name.")
      ->required()
      ->group("Instance");

  // model parameters
  bool   fix_routes                            = false;
  double max_velocity_delta                    = 5.55; // 20 km/h
  bool   simplify_headway_constraints          = false;
  bool   strengthen_vertex_headway_constraints = false;
  bool   late_entry_possible                   = false;
  double max_exit_delay                        = 1e9;
  double max_station_delay                     = 1e9;
  double max_delay                             = 1e9;

  // solver parameters
  bool   use_indicator_constraints              = false;
  bool   use_lazy_constraints                   = true;
  bool   include_reverse_headways               = false;
  bool   include_higher_velocities_in_edge_expr = false;
  double abs_mip_gap                            = 10;

  int  time_limit{-1};
  bool debug_output{false};

  cda_rail::VelocityRefinementStrategy velocity_refinement_strategy{
      cda_rail::VelocityRefinementStrategy::MinOneStep};
  cda_rail::solver::mip_based::LazyConstraintSelectionStrategy
      lazy_constraint_selection_strategy{
          cda_rail::solver::mip_based::LazyConstraintSelectionStrategy::
              OnlyViolated};
  cda_rail::solver::mip_based::LazyTrainSelectionStrategy
      lazy_train_selection_strategy{
          cda_rail::solver::mip_based::LazyTrainSelectionStrategy::
              OnlyAdjacent};

  // helper maps
  std::map<std::string, cda_rail::VelocityRefinementStrategy> const
      velocity_refinement_strategy_map{
          {"None", cda_rail::VelocityRefinementStrategy::None},
          {"MinOneStep", cda_rail::VelocityRefinementStrategy::MinOneStep}};

  std::map<std::string,
           cda_rail::solver::mip_based::LazyConstraintSelectionStrategy> const
      lazy_constraint_selection_strategy_map{
          {"OnlyViolated", cda_rail::solver::mip_based::
                               LazyConstraintSelectionStrategy::OnlyViolated},
          {"OnlyFirstFound",
           cda_rail::solver::mip_based::LazyConstraintSelectionStrategy::
               OnlyFirstFound},
          {"AllChecked", cda_rail::solver::mip_based::
                             LazyConstraintSelectionStrategy::AllChecked}};

  std::map<std::string,
           cda_rail::solver::mip_based::LazyTrainSelectionStrategy> const
      lazy_train_selection_strategy_map{
          {"OnlyAdjacent", cda_rail::solver::mip_based::
                               LazyTrainSelectionStrategy::OnlyAdjacent},
          {"All",
           cda_rail::solver::mip_based::LazyTrainSelectionStrategy::All}};

  app.add_flag("-f,--fix-routes", fix_routes,
               "If this flag is set, the routes given by the instance are "
               "fixed. Otherwise (by default) the routes are optimized as "
               "well.")
      ->group("Model Parameters");
  app.add_option("-m,--max-velocity-delta", max_velocity_delta,
                 "Maximal velocity difference (in m/s) between two "
                 "consecutive velocity extensions of a train.")
      ->check(CLI::PositiveNumber)
      ->capture_default_str()
      ->group("Model Parameters");
  app.add_option("-r,--velocity-refinement-strategy",
                 velocity_refinement_strategy,
                 "Strategy used to refine the velocity extensions. Currently "
                 "supports 'None' and 'MinOneStep'.")
      ->transform(CLI::CheckedTransformer(velocity_refinement_strategy_map,
                                          CLI::ignore_case))
      ->capture_default_str()
      ->group("Model Parameters");
  app.add_flag("-y,--simplify-headway-constraints",
               simplify_headway_constraints,
               "If this flag is set, simplified (weaker) headway constraints "
               "are used instead of the full ones.")
      ->group("Model Parameters");
  app.add_flag("-q,--strengthen-vertex-headway-constraints",
               strengthen_vertex_headway_constraints,
               "If this flag is set, the vertex headway constraints are "
               "strengthened.")
      ->group("Model Parameters");
  app.add_flag("-l,--allow-late-entry", late_entry_possible,
               "Allow late entry (delays) in the solution (default without "
               "flag is false)")
      ->group("Model Parameters");
  auto* max_exit_delay_opt =
      app.add_option("-x,--max-exit-delay", max_exit_delay,
                     "Maximal delay (in seconds) with which a train is allowed "
                     "to leave the network compared to its scheduled exit "
                     "time.")
          ->check(CLI::NonNegativeNumber)
          ->capture_default_str()
          ->group("Model Parameters");
  auto* max_station_delay_opt =
      app.add_option("--max-station-delay", max_station_delay,
                     "Maximal delay (in seconds) with which a train is allowed "
                     "to be serviced at a station compared to its scheduled "
                     "service time.")
          ->check(CLI::NonNegativeNumber)
          ->capture_default_str()
          ->group("Model Parameters");
  auto* max_delay_opt =
      app.add_option("--max-delay", max_delay,
                     "Maximal delay (in seconds) used for both the exit and "
                     "the station delay at once. Mutually exclusive with the "
                     "individual --max-exit-delay and --max-station-delay "
                     "settings.")
          ->check(CLI::NonNegativeNumber)
          ->group("Model Parameters");

  max_delay_opt->excludes(max_exit_delay_opt);
  max_delay_opt->excludes(max_station_delay_opt);

  app.add_flag("-c,--use-indicator-constraints", use_indicator_constraints,
               "If this flag is set, indicator constraints are used instead "
               "of big-M formulations where possible.")
      ->group("Solver Parameters");
  app.add_flag("!-z,!--no-lazy-constraints,--use-lazy-constraints",
               use_lazy_constraints,
               "Headway constraints are separated lazily by default. If this "
               "flag is negated (-z or --no-lazy-constraints), they are added "
               "to the model upfront and all remaining lazy settings are "
               "ignored.")
      ->group("Solver Parameters");
  app.add_flag("-w,--include-reverse-headways", include_reverse_headways,
               "If this flag is set, headways on reverse edges are separated "
               "as well. Only possible together with lazy constraints using "
               "the 'AllChecked' lazy constraint selection strategy.")
      ->group("Solver Parameters");
  app.add_flag("-u,--include-higher-velocities-in-edge-expr",
               include_higher_velocities_in_edge_expr,
               "If this flag is set, higher velocities are included in the "
               "edge expressions of the lazy headway constraints.")
      ->group("Solver Parameters");
  app.add_option("-j,--lazy-constraint-selection-strategy",
                 lazy_constraint_selection_strategy,
                 "Strategy deciding which violated lazy constraints are "
                 "added. Currently supports 'OnlyViolated', 'OnlyFirstFound', "
                 "and 'AllChecked'.")
      ->transform(CLI::CheckedTransformer(
          lazy_constraint_selection_strategy_map, CLI::ignore_case))
      ->capture_default_str()
      ->group("Solver Parameters");
  app.add_option("-k,--lazy-train-selection-strategy",
                 lazy_train_selection_strategy,
                 "Strategy deciding which train pairs are checked when "
                 "separating lazy constraints. Currently supports "
                 "'OnlyAdjacent' and 'All'.")
      ->transform(CLI::CheckedTransformer(lazy_train_selection_strategy_map,
                                          CLI::ignore_case))
      ->capture_default_str()
      ->group("Solver Parameters");
  app.add_option("-a,--abs-mip-gap", abs_mip_gap,
                 "Absolute MIP gap used as termination criterion of the "
                 "solver.")
      ->check(CLI::NonNegativeNumber)
      ->capture_default_str()
      ->group("Solver Parameters");

  app.add_option(
         "-t,--time-limit", time_limit,
         "Time limit in seconds for the solver to run. No limit if negative.")
      ->capture_default_str()
      ->group("Additional Solving Parameters");
  app.add_flag("-v,--verbose,--debug", debug_output,
               "Whether to output debug information during the solving "
               "process. Default: no debug output.")
      ->group("Additional Solving Parameters");

  // Export options
  cda_rail::solver::GeneralExportOption export_option{
      cda_rail::solver::GeneralExportOption::NoExport};
  bool                       export_lp_model{false};
  std::string                model_name{"model"};
  std::string                solution_subdirectory{};
  std::string                export_working_directory{};
  std::optional<std::string> parameter_identifier{};
  bool                       generate_identifier{false};

  auto* export_sol_flag =
      app.add_flag(
             "-o,--export-solution",
             [&export_option](int) {
               export_option =
                   cda_rail::solver::GeneralExportOption::ExportSolution;
             },
             "Export the solution.")
          ->group("Export Options");
  auto* export_sol_inst_flag =
      app.add_flag(
             "-i,--export-solution-and-instance",
             [&export_option](int) {
               export_option = cda_rail::solver::GeneralExportOption::
                   ExportSolutionWithInstance;
             },
             "Export the solution and the instance.")
          ->group("Export Options");
  auto* export_lp_flag =
      app.add_flag("--export-lp-model", export_lp_model,
                   "Export the MIP model itself (as .mps and, if a solution "
                   "exists, as .json) into the solution directory.")
          ->group("Export Options");

  CLI::Validator requires_export_option(
      [&export_sol_flag, &export_sol_inst_flag, &export_lp_flag](std::string&) {
        if (export_sol_flag->count() == 0 &&
            export_sol_inst_flag->count() == 0 &&
            export_lp_flag->count() == 0) {
          return std::string{"requires either --export-solution, "
                             "--export-solution-and-instance or "
                             "--export-lp-model"};
        }
        return std::string{};
      },
      " Needs: --export-solution, --export-solution-and-instance or "
      "--export-lp-model");
  requires_export_option.non_modifying();

  auto* export_working_directory_opt =
      app.add_option(
             "-b,--export-working-directory", export_working_directory,
             "Working directory for exporting solutions. If unset, the normal "
             "working directory is used.")
          ->check(requires_export_option)
          ->group("Export Options");
  auto* solution_subdir_opt =
      app.add_option(
             "-e,--solution-export-subdirectory", solution_subdirectory,
             "Subdirectory to export the solution to. Will be created in "
             "export_working_directory/solutions/solution_subdirectory/"
             "instance_name-parameters.")
          ->check(requires_export_option)
          ->group("Export Options");
  auto* model_name_opt =
      app.add_option("--model-name", model_name,
                     "File name (without extension) used when exporting the "
                     "MIP model itself.")
          ->check(requires_export_option)
          ->capture_default_str()
          ->group("Export Options");

  export_sol_flag->needs(solution_subdir_opt);
  export_sol_inst_flag->needs(solution_subdir_opt);
  export_lp_flag->needs(solution_subdir_opt);
  export_sol_flag->excludes(export_sol_inst_flag);
  export_sol_inst_flag->excludes(export_sol_flag);
  model_name_opt->needs(export_lp_flag);

  auto* parameter_identifier_option =
      app.add_option(
             "-p,--parameter-identifier", parameter_identifier,
             "Optional identifier to distinguish different parameterizations "
             "of the same instance. Will be appended to the instance name in "
             "the export path as instance_name-parameter_identifier. If empty, "
             "no parameter identifier will be appended")
          ->capture_default_str()
          ->group("Export Options");
  auto* generate_identifier_flag =
      app.add_flag("-g,--generate-parameter-identifier", generate_identifier,
                   "Whether to automatically generate a parameter identifier "
                   "based on the parameter settings. If set, the parameter "
                   "identifier will be generated as a concatenation of the "
                   "parameter names and values. Otherwise the identifier has "
                   "to be set explicitly if it should not remain empty.")
          ->group("Export Options");

  generate_identifier_flag->excludes(parameter_identifier_option);

  CLI11_PARSE(app, argc, argv);

  if (max_delay_opt->count() > 0) {
    max_exit_delay    = max_delay;
    max_station_delay = max_delay;
  }

  if (export_working_directory_opt->count() == 0) {
    export_working_directory = working_directory;
  }

  // ----------------------
  // PRINT SETTINGS
  // ----------------------

  if (generate_identifier) {
    auto format_double = [](double d) {
      std::stringstream ss;
      ss << std::fixed << std::setprecision(6) << d;
      std::string s = ss.str();
      s.erase(s.find_last_not_of('0') + 1, std::string::npos);
      if (s.back() == '.') {
        s.pop_back();
      }
      return s;
    };
    auto bool_to_str = [](bool b) { return b ? "t" : "f"; };

    parameter_identifier = cda_rail::concatenate_string_views(
        {bool_to_str(fix_routes),
         "_",
         format_double(max_velocity_delta),
         "_",
         cda_rail::solver::mip_based::velocity_refinement_strategy_to_string(
             velocity_refinement_strategy),
         "_",
         bool_to_str(simplify_headway_constraints),
         "_",
         bool_to_str(strengthen_vertex_headway_constraints),
         "_",
         bool_to_str(late_entry_possible),
         "_",
         format_double(max_exit_delay),
         "_",
         format_double(max_station_delay),
         "_",
         bool_to_str(use_indicator_constraints),
         "_",
         bool_to_str(use_lazy_constraints),
         "_",
         bool_to_str(include_reverse_headways),
         "_",
         bool_to_str(include_higher_velocities_in_edge_expr),
         "_",
         cda_rail::solver::mip_based::
             lazy_constraint_selection_strategy_to_string(
                 lazy_constraint_selection_strategy),
         "_",
         cda_rail::solver::mip_based::lazy_train_selection_strategy_to_string(
             lazy_train_selection_strategy),
         "_",
         format_double(abs_mip_gap),
         "_",
         std::to_string(time_limit)});
  }

  PLOGD << "The following parameters were passed:";
  PLOGD << "Instance Settings";
  PLOGD << "  Instance name: " << instance_name;
  PLOGD << "  Instance subdirectory: " << instance_subdirectory;
  PLOGD << "  Working directory: " << working_directory;
  PLOGD << "Model Settings";
  PLOGD << "  Fix routes: " << (fix_routes ? "yes" : "no");
  PLOGD << "  Maximal velocity delta: " << max_velocity_delta;
  PLOGD << "  Velocity refinement strategy: "
        << cda_rail::solver::mip_based::velocity_refinement_strategy_to_string(
               velocity_refinement_strategy);
  PLOGD << "  Simplify headway constraints: "
        << (simplify_headway_constraints ? "yes" : "no");
  PLOGD << "  Strengthen vertex headway constraints: "
        << (strengthen_vertex_headway_constraints ? "yes" : "no");
  PLOGD << "  Allow late entry: " << (late_entry_possible ? "yes" : "no");
  PLOGD << "  Maximal exit delay: " << max_exit_delay;
  PLOGD << "  Maximal station delay: " << max_station_delay;
  PLOGD << "Solver Settings";
  PLOGD << "  Use indicator constraints: "
        << (use_indicator_constraints ? "yes" : "no");
  PLOGD << "  Use lazy constraints: " << (use_lazy_constraints ? "yes" : "no");
  PLOGD << "  Include reverse headways: "
        << (include_reverse_headways ? "yes" : "no");
  PLOGD << "  Include higher velocities in edge expressions: "
        << (include_higher_velocities_in_edge_expr ? "yes" : "no");
  PLOGD << "  Lazy constraint selection strategy: "
        << cda_rail::solver::mip_based::
               lazy_constraint_selection_strategy_to_string(
                   lazy_constraint_selection_strategy);
  PLOGD << "  Lazy train selection strategy: "
        << cda_rail::solver::mip_based::lazy_train_selection_strategy_to_string(
               lazy_train_selection_strategy);
  PLOGD << "  Absolute MIP gap: " << abs_mip_gap;
  PLOGD << "Additional Solving Parameters";
  PLOGD << "  Time limit: " << time_limit << " seconds";
  PLOGD << "  Debug output: " << (debug_output ? "yes" : "no");
  PLOGD << "Export Settings";
  switch (export_option) {
  case cda_rail::solver::GeneralExportOption::NoExport:
    PLOGD << "  Export option: No export";
    break;
  case cda_rail::solver::GeneralExportOption::ExportSolution:
    PLOGD << "  Export option: Export solution";
    break;
  case cda_rail::solver::GeneralExportOption::ExportSolutionWithInstance:
    PLOGD << "  Export option: Export solution with instance";
    break;
  }
  PLOGD << "  Export MIP model: " << (export_lp_model ? "yes" : "no");
  PLOGD << "  MIP model name: " << model_name;
  PLOGD << "  Export working directory: " << export_working_directory;
  PLOGD << "  Solution export subdirectory: " << solution_subdirectory;
  PLOGD << "  Parameter identifier: " << parameter_identifier.value_or("-")
        << (generate_identifier ? " (generated)" : "");

  cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(
      instance_name, instance_subdirectory, working_directory);

  cda_rail::solver::mip_based::SolutionSettingsMovingBlock solution_settings;
  solution_settings.export_option         = export_option;
  solution_settings.working_directory     = export_working_directory;
  solution_settings.solution_subdirectory = solution_subdirectory;
  solution_settings.parameter_identifier  = parameter_identifier;
  solution_settings.export_lp_model       = export_lp_model;
  solution_settings.model_name            = model_name;

  // --------------
  // SOLVE
  // --------------

  PLOGI << "Solving...";

  // NOLINTNEXTLINE(clang-diagnostic-unused-result)
  auto const solution = solver.solve(
      {.fix_routes                   = fix_routes,
       .max_velocity_delta           = max_velocity_delta,
       .velocity_refinement_strategy = velocity_refinement_strategy,
       .simplify_headway_constraints = simplify_headway_constraints,
       .strengthen_vertex_headway_constraints =
           strengthen_vertex_headway_constraints,
       .allow_late_entry  = late_entry_possible,
       .max_exit_delay    = max_exit_delay,
       .max_station_delay = max_station_delay},
      {.use_indicator_constraints = use_indicator_constraints,
       .use_lazy_constraints      = use_lazy_constraints,
       .include_reverse_headways  = include_reverse_headways,
       .include_higher_velocities_in_edge_expr =
           include_higher_velocities_in_edge_expr,
       .lazy_constraint_selection_strategy = lazy_constraint_selection_strategy,
       .lazy_train_selection_strategy      = lazy_train_selection_strategy,
       .abs_mip_gap                        = abs_mip_gap},
      solution_settings, time_limit, debug_output, true);

  std::string sol_status{"ERROR"};
  switch (solution.get_status()) {
  case cda_rail::SolutionStatus::Optimal:
    sol_status = "Optimal";
    break;
  case cda_rail::SolutionStatus::Feasible:
    sol_status = "Feasible";
    break;
  case cda_rail::SolutionStatus::Infeasible:
    sol_status = "Infeasible";
    break;
  case cda_rail::SolutionStatus::Timeout:
    sol_status = "Timeout";
    break;
  case cda_rail::SolutionStatus::Unknown:
    sol_status = "Unknown";
    break;
  }
  PLOGI << "Solution status: " << sol_status;
  PLOGI << "Solution objective: " << solution.get_obj();
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,bugprone-exception-escape)
