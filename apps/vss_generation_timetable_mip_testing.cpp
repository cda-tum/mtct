#include "CLI/CLI.hpp"
#include "Definitions.hpp"
#include "StringHelper.hpp"
#include "VSSModel.hpp"
#include "plog/Init.h"
#include "plog/Logger.h"
#include "plog/Severity.h"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "solver/GeneralSolver.hpp"
#include "solver/mip-based/GeneralMIPSolver.hpp"
#include "solver/mip-based/VSSGenTimetableSolver.hpp"

#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <ios>
#include <iostream>
#include <map>
#include <optional>
#include <plog/Appenders/ColorConsoleAppender.h>
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Log.h>
#include <sstream>
#include <string>
#include <vector>

// This app covers both VSS generation solvers, namely VSSGenTimetableSolver
// and VSSGenTimetableSolverWithMovingBlockInformation. The latter is used as
// soon as -m (--moving-block-solution-subdirectory) is given; in that case a
// previously obtained moving block solution is loaded in addition to the
// instance itself and the routes of the instance are superseded by the routes
// of that solution.
//
// Settings that also exist in the two moving block apps
// (gen_po_moving_block_astar_testing and gen_po_moving_block_mip_testing) use
// the very same short and long options there:
//   -n -s -d (instance), -c (time discretization), -f (fix routes),
//   -t (time limit), -v (debug), -o -i -b -e -p -g, --export-lp-model and
//   --model-name (export).
// The remaining letters are assigned to the VSS specific settings; since the
// solvers barely share any parameters, a letter may well have a different
// meaning in the other apps.
//
// Since all lowercase letters are used up, the fine tuning settings
// (--only-stop-at-vss, --use-pwl, --use-schedule-cuts, --postprocess), the
// settings that are only relevant together with -x (--iterative-approach) and
// the remaining export and moving block loading options are long option only.

namespace {
enum class SeparationFunctionType : std::uint8_t { Uniform = 0, Chebyshev = 1 };
} // namespace

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,bugprone-exception-escape)

int main(int argc, char** argv) {
  // Only log to console using std::cerr and std::cout respectively unless
  // initialized differently
  if (plog::get() == nullptr) {
    static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
    plog::init(plog::debug, &console_appender);
  }

  CLI::App app{"VSS Generation Optimization using a MIP"};
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

  // moving block information
  std::string                moving_block_solution_subdirectory{};
  std::string                moving_block_working_directory{};
  std::optional<std::string> moving_block_parameter_identifier{};

  bool fix_stop_positions         = true;
  bool fix_exact_positions        = true;
  bool fix_exact_velocities       = true;
  bool hint_approximate_positions = true;
  bool fix_order_on_edges         = true;

  auto* mb_solution_subdirectory_opt =
      app.add_option(
             "-m,--moving-block-solution-subdirectory",
             moving_block_solution_subdirectory,
             "Subdirectory of a previously obtained moving block solution of "
             "the very same instance. If (and only if) this option is set, the "
             "solver using moving block information is used. The solution is "
             "loaded from moving_block_working_directory/solutions/"
             "moving_block_solution_subdirectory/instance_subdirectory/"
             "instance_name. Note that the routes of the moving block solution "
             "supersede the routes of the instance and that the discrete VSS "
             "model type is not supported in this case.")
          ->group("Moving Block Information");
  auto* mb_working_directory_opt =
      app.add_option("--moving-block-working-directory",
                     moving_block_working_directory,
                     "Working directory from which the moving block solution "
                     "is loaded. If unset, the normal working directory is "
                     "used.")
          ->needs(mb_solution_subdirectory_opt)
          ->group("Moving Block Information");
  app.add_option("--moving-block-parameter-identifier",
                 moving_block_parameter_identifier,
                 "Parameter identifier that was appended to the instance name "
                 "when the moving block solution was exported. If empty, no "
                 "parameter identifier is assumed.")
      ->needs(mb_solution_subdirectory_opt)
      ->capture_default_str()
      ->group("Moving Block Information");

  app.add_flag("!-z,!--free-stop-positions,--fix-stop-positions",
               fix_stop_positions,
               "The positions at which trains stop at a station are fixed to "
               "the ones of the moving block solution by default. If this flag "
               "is negated (-z or --free-stop-positions), they are optimized "
               "again.")
      ->needs(mb_solution_subdirectory_opt)
      ->group("Moving Block Information");
  app.add_flag("!-l,!--free-exact-positions,--fix-exact-positions",
               fix_exact_positions,
               "The exact positions of the trains at every vertex are fixed to "
               "the ones of the moving block solution by default. If this flag "
               "is negated (-l or --free-exact-positions), they are only "
               "bounded by their minimal and maximal positions.")
      ->needs(mb_solution_subdirectory_opt)
      ->group("Moving Block Information");
  app.add_flag("!-y,!--free-exact-velocities,--fix-exact-velocities",
               fix_exact_velocities,
               "The exact velocities of the trains at every vertex are fixed "
               "to the ones of the moving block solution by default. If this "
               "flag is negated (-y or --free-exact-velocities), they are "
               "optimized again.")
      ->needs(mb_solution_subdirectory_opt)
      ->group("Moving Block Information");
  app.add_flag("!-u,!--no-position-hints,--hint-approximate-positions",
               hint_approximate_positions,
               "The approximate positions of the trains at every point in time "
               "are hinted to the solver by default. If this flag is negated "
               "(-u or --no-position-hints), no such hints are given.")
      ->needs(mb_solution_subdirectory_opt)
      ->group("Moving Block Information");
  app.add_flag("!-w,!--free-order-on-edges,--fix-order-on-edges",
               fix_order_on_edges,
               "The order in which the trains traverse the edges is fixed to "
               "the one of the moving block solution by default. If this flag "
               "is negated (-w or --free-order-on-edges), the order is "
               "optimized again.")
      ->needs(mb_solution_subdirectory_opt)
      ->group("Moving Block Information");

  // model parameters
  double delta_t        = 15;
  bool   fix_routes     = true;
  bool   train_dynamics = true;
  bool   braking_curves = true;

  cda_rail::vss::ModelType vss_model_type{cda_rail::vss::ModelType::Continuous};
  std::vector<SeparationFunctionType> separation_function_types{};
  bool                                only_stop_at_vss  = false;
  bool                                use_pwl           = false;
  bool                                use_schedule_cuts = true;

  // solver parameters
  bool                         iterative_approach = false;
  cda_rail::OptimalityStrategy optimality_strategy{
      cda_rail::OptimalityStrategy::Optimal};
  cda_rail::solver::mip_based::UpdateStrategy iterative_update_strategy{
      cda_rail::solver::mip_based::UpdateStrategy::Fixed};
  double iterative_initial_value = 1;
  double iterative_update_value  = 2;
  bool   iterative_include_cuts  = true;

  int  time_limit{-1};
  bool debug_output{false};

  // helper maps
  std::map<std::string, cda_rail::vss::ModelType> const vss_model_type_map{
      {"Discrete", cda_rail::vss::ModelType::Discrete},
      {"Continuous", cda_rail::vss::ModelType::Continuous},
      {"Inferred", cda_rail::vss::ModelType::Inferred},
      {"InferredAlt", cda_rail::vss::ModelType::InferredAlt}};

  std::map<std::string, SeparationFunctionType> const separation_function_map{
      {"Uniform", SeparationFunctionType::Uniform},
      {"Chebyshev", SeparationFunctionType::Chebyshev}};

  std::map<std::string, cda_rail::OptimalityStrategy> const
      optimality_strategy_map{
          {"Optimal", cda_rail::OptimalityStrategy::Optimal},
          {"TradeOff", cda_rail::OptimalityStrategy::TradeOff},
          {"Feasible", cda_rail::OptimalityStrategy::Feasible}};

  std::map<std::string, cda_rail::solver::mip_based::UpdateStrategy> const
      update_strategy_map{
          {"Fixed", cda_rail::solver::mip_based::UpdateStrategy::Fixed},
          {"Relative", cda_rail::solver::mip_based::UpdateStrategy::Relative}};

  auto get_key_by_value = [](const auto& map, const auto& value) {
    for (const auto& [k, v] : map) {
      if (v == value) {
        return k;
      }
    }
    std::cerr << "Internal error: unsupported option value." << '\n';
    std::exit(EXIT_FAILURE);
  };

  app.add_option("-c,--delta-t,--dt,--timestep", delta_t,
                 "Length of the discretized time intervals in seconds.")
      ->check(CLI::PositiveNumber)
      ->capture_default_str()
      ->group("Model Parameters");
  app.add_flag("!-f,!--free-routes,--fix-routes", fix_routes,
               "The routes given by the instance are fixed by default. If this "
               "flag is negated (-f or --free-routes), the routes are "
               "optimized as well. Not applicable together with moving block "
               "information, in which case the routes are always fixed to the "
               "ones of the moving block solution.")
      ->excludes(mb_solution_subdirectory_opt)
      ->group("Model Parameters");
  app.add_flag("!-a,!--no-train-dynamics,--train-dynamics", train_dynamics,
               "The train dynamics (i.e., limited acceleration and "
               "deceleration) are included in the model by default. If this "
               "flag is negated (-a or --no-train-dynamics), they are "
               "omitted.")
      ->group("Model Parameters");
  app.add_flag("!-k,!--no-braking-curves,--braking-curves", braking_curves,
               "The braking curves (i.e., the braking distance depending on "
               "the current speed has to be cleared) are included in the model "
               "by default. If this flag is negated (-k or "
               "--no-braking-curves), they are omitted.")
      ->group("Model Parameters");
  app.add_option("-r,--vss-model-type", vss_model_type,
                 "Denotes how the VSS borders are modelled in the solution "
                 "process. Currently supports 'Discrete', 'Continuous', "
                 "'Inferred', and 'InferredAlt'. 'Discrete' expects exactly "
                 "one, 'Inferred' and 'InferredAlt' expect at least one "
                 "separation function, whereas 'Continuous' expects none.")
      ->transform(CLI::CheckedTransformer(vss_model_type_map, CLI::ignore_case))
      ->capture_default_str()
      ->group("Model Parameters");
  app.add_option("-j,--separation-functions", separation_function_types,
                 "Separation functions used by the VSS model type. Can be "
                 "passed multiple times. Currently supports 'Uniform' and "
                 "'Chebyshev'.")
      ->transform(
          CLI::CheckedTransformer(separation_function_map, CLI::ignore_case))
      ->multi_option_policy(CLI::MultiOptionPolicy::TakeAll)
      ->group("Model Parameters");
  app.add_flag("--only-stop-at-vss", only_stop_at_vss,
               "If this flag is set, trains are only allowed to stop at VSS "
               "borders.")
      ->group("Model Parameters");
  app.add_flag("--use-pwl", use_pwl,
               "If this flag is set, the braking distances are approximated by "
               "piecewise linear functions with a fixed maximal error. "
               "Otherwise (by default) they are modelled as quadratic "
               "functions using Gurobi's ability to solve these by spatial "
               "branching. Only relevant if braking curves are included.")
      ->group("Model Parameters");
  app.add_flag("!--no-schedule-cuts,--use-schedule-cuts", use_schedule_cuts,
               "The formulation is strengthened using cuts implied by the "
               "schedule by default. If this flag is negated "
               "(--no-schedule-cuts), these cuts are omitted.")
      ->group("Model Parameters");

  auto* iterative_approach_flag =
      app.add_flag("-x,--iterative-approach", iterative_approach,
                   "If this flag is set, the number of VSS per edge is "
                   "iteratively increased until optimality is proven instead "
                   "of solving the full model at once.")
          ->group("Solver Parameters");
  app.add_option("-q,--optimality-strategy", optimality_strategy,
                 "Optimality strategy to use. Currently supports 'Optimal', "
                 "'TradeOff', and 'Feasible'.")
      ->transform(
          CLI::CheckedTransformer(optimality_strategy_map, CLI::ignore_case))
      ->capture_default_str()
      ->group("Solver Parameters");
  app.add_option("--iterative-update-strategy", iterative_update_strategy,
                 "Strategy used to update the number of VSS per edge within "
                 "the iterative approach. Currently supports 'Fixed' "
                 "(absolute number) and 'Relative' (fraction of the "
                 "theoretically possible number).")
      ->transform(
          CLI::CheckedTransformer(update_strategy_map, CLI::ignore_case))
      ->needs(iterative_approach_flag)
      ->capture_default_str()
      ->group("Solver Parameters");
  app.add_option("--iterative-initial-value", iterative_initial_value,
                 "Initial number of VSS per edge ('Fixed' update strategy, has "
                 "to be an integer) or initial fraction of the theoretically "
                 "possible number ('Relative' update strategy, has to be "
                 "within (0,1]) used by the iterative approach.")
      ->check(CLI::PositiveNumber)
      ->needs(iterative_approach_flag)
      ->capture_default_str()
      ->group("Solver Parameters");
  app.add_option("--iterative-update-value", iterative_update_value,
                 "Value by which the number of VSS per edge is increased in "
                 "every iteration. Has to be greater than 1 for the 'Fixed' "
                 "and within (0,1) for the 'Relative' update strategy.")
      ->check(CLI::PositiveNumber)
      ->needs(iterative_approach_flag)
      ->capture_default_str()
      ->group("Solver Parameters");
  app.add_flag("!--no-iterative-cuts,--iterative-cuts", iterative_include_cuts,
               "Cuts excluding the already explored search space are added in "
               "every iteration of the iterative approach by default. If this "
               "flag is negated (--no-iterative-cuts), these cuts are "
               "omitted.")
      ->needs(iterative_approach_flag)
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
  bool                       postprocess{false};
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
                   "exists, as .sol) into the solution directory.")
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
             "instance_subdirectory/instance_name-parameters.")
          ->check(requires_export_option)
          ->group("Export Options");
  auto* model_name_opt =
      app.add_option("--model-name", model_name,
                     "File name (without extension) used when exporting the "
                     "MIP model itself.")
          ->check(requires_export_option)
          ->capture_default_str()
          ->group("Export Options");
  app.add_flag("--postprocess", postprocess,
               "If this flag is set, the solution is postprocessed to remove "
               "potentially unused VSS.")
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

  const bool use_moving_block_information =
      mb_solution_subdirectory_opt->count() > 0;

  if (mb_working_directory_opt->count() == 0) {
    moving_block_working_directory = working_directory;
  }
  if (export_working_directory_opt->count() == 0) {
    export_working_directory = working_directory;
  }

  std::vector<cda_rail::vss::SeparationFunction> separation_functions;
  separation_functions.reserve(separation_function_types.size());
  for (const auto& separation_function_type : separation_function_types) {
    switch (separation_function_type) {
    case SeparationFunctionType::Uniform:
      separation_functions.emplace_back(cda_rail::vss::UNIFORM);
      break;
    case SeparationFunctionType::Chebyshev:
      separation_functions.emplace_back(cda_rail::vss::CHEBYSHEV);
      break;
    }
  }

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

    std::string identifier = cda_rail::concatenate_string_views(
        {format_double(delta_t), "_", bool_to_str(train_dynamics), "_",
         bool_to_str(braking_curves), "_",
         get_key_by_value(vss_model_type_map, vss_model_type)});
    for (const auto& separation_function : separation_functions) {
      identifier = cda_rail::concatenate_string_views(
          {identifier, "_", separation_function.get_name()});
    }
    identifier = cda_rail::concatenate_string_views(
        {identifier,
         "_",
         bool_to_str(only_stop_at_vss),
         "_",
         bool_to_str(use_pwl),
         "_",
         bool_to_str(use_schedule_cuts),
         "_",
         bool_to_str(iterative_approach),
         "_",
         get_key_by_value(optimality_strategy_map, optimality_strategy),
         "_",
         get_key_by_value(update_strategy_map, iterative_update_strategy),
         "_",
         format_double(iterative_initial_value),
         "_",
         format_double(iterative_update_value),
         "_",
         bool_to_str(iterative_include_cuts),
         "_",
         bool_to_str(postprocess),
         "_",
         std::to_string(time_limit)});
    // The settings that only exist for one of the two solvers are appended
    // last, so that the identifiers of the settings both of them share keep
    // the very same structure.
    identifier           = use_moving_block_information
                               ? cda_rail::concatenate_string_views(
                                     {identifier, "_", bool_to_str(fix_stop_positions),
                                      "_", bool_to_str(fix_exact_positions), "_",
                                      bool_to_str(fix_exact_velocities), "_",
                                      bool_to_str(hint_approximate_positions), "_",
                                      bool_to_str(fix_order_on_edges)})
                               : cda_rail::concatenate_string_views(
                                     {identifier, "_", bool_to_str(fix_routes)});
    parameter_identifier = identifier;
  }

  // ----------------------
  // PRINT SETTINGS
  // ----------------------

  PLOGD << "The following parameters were passed:";
  PLOGD << "Instance Settings";
  PLOGD << "  Instance name: " << instance_name;
  PLOGD << "  Instance subdirectory: " << instance_subdirectory;
  PLOGD << "  Working directory: " << working_directory;
  PLOGD << "Moving Block Information";
  PLOGD << "  Use moving block information: "
        << (use_moving_block_information ? "yes" : "no");
  if (use_moving_block_information) {
    PLOGD << "  Moving block working directory: "
          << moving_block_working_directory;
    PLOGD << "  Moving block solution subdirectory: "
          << moving_block_solution_subdirectory;
    PLOGD << "  Moving block parameter identifier: "
          << moving_block_parameter_identifier.value_or("-");
    PLOGD << "  Fix stop positions: " << (fix_stop_positions ? "yes" : "no");
    PLOGD << "  Fix exact positions: " << (fix_exact_positions ? "yes" : "no");
    PLOGD << "  Fix exact velocities: "
          << (fix_exact_velocities ? "yes" : "no");
    PLOGD << "  Hint approximate positions: "
          << (hint_approximate_positions ? "yes" : "no");
    PLOGD << "  Fix order on edges: " << (fix_order_on_edges ? "yes" : "no");
  }
  PLOGD << "Model Settings";
  PLOGD << "  Time discretization (delta_t): " << delta_t;
  if (!use_moving_block_information) {
    PLOGD << "  Fix routes: " << (fix_routes ? "yes" : "no");
  }
  PLOGD << "  Train dynamics: " << (train_dynamics ? "yes" : "no");
  PLOGD << "  Braking curves: " << (braking_curves ? "yes" : "no");
  PLOGD << "  VSS model type: "
        << get_key_by_value(vss_model_type_map, vss_model_type);
  for (const auto& separation_function : separation_functions) {
    PLOGD << "  Separation function: " << separation_function.get_name();
  }
  PLOGD << "  Only stop at VSS: " << (only_stop_at_vss ? "yes" : "no");
  PLOGD << "  Use PWL: " << (use_pwl ? "yes" : "no");
  PLOGD << "  Use schedule cuts: " << (use_schedule_cuts ? "yes" : "no");
  PLOGD << "Solver Settings";
  PLOGD << "  Iterative approach: " << (iterative_approach ? "yes" : "no");
  PLOGD << "  Optimality strategy: "
        << get_key_by_value(optimality_strategy_map, optimality_strategy);
  if (iterative_approach) {
    PLOGD << "  Iterative update strategy: "
          << get_key_by_value(update_strategy_map, iterative_update_strategy);
    PLOGD << "  Iterative initial value: " << iterative_initial_value;
    PLOGD << "  Iterative update value: " << iterative_update_value;
    PLOGD << "  Iterative include cuts: "
          << (iterative_include_cuts ? "yes" : "no");
  }
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
  PLOGD << "  Postprocess solution: " << (postprocess ? "yes" : "no");
  PLOGD << "  Export working directory: " << export_working_directory;
  PLOGD << "  Solution export subdirectory: " << solution_subdirectory;
  PLOGD << "  Parameter identifier: " << parameter_identifier.value_or("-")
        << (generate_identifier ? " (generated)" : "");

  const cda_rail::solver::mip_based::ModelSettings model_settings{
      .model_type = cda_rail::vss::Model(vss_model_type, separation_functions,
                                         only_stop_at_vss),
      .use_pwl    = use_pwl,
      .use_schedule_cuts = use_schedule_cuts};
  const cda_rail::solver::mip_based::SolverStrategy solver_strategy{
      .iterative_approach  = iterative_approach,
      .optimality_strategy = optimality_strategy,
      .update_strategy     = iterative_update_strategy,
      .initial_value       = iterative_initial_value,
      .update_value        = iterative_update_value,
      .include_cuts        = iterative_include_cuts};
  cda_rail::solver::mip_based::SolutionSettingsVSSGen solution_settings;
  solution_settings.export_option         = export_option;
  solution_settings.working_directory     = export_working_directory;
  solution_settings.solution_subdirectory = solution_subdirectory;
  solution_settings.parameter_identifier  = parameter_identifier;
  solution_settings.export_lp_model       = export_lp_model;
  solution_settings.model_name            = model_name;
  solution_settings.postprocess           = postprocess;

  // --------------
  // SOLVE
  // --------------

  PLOGI << "Solving...";

  auto const solution = [&]()
      -> cda_rail::instances::SolVSSGeneralPerformanceOptimizationInstance {
    if (use_moving_block_information) {
      const cda_rail::instances::GeneralPerformanceOptimizationInstance
          instance(instance_name, instance_subdirectory, working_directory);
      cda_rail::instances::SolGeneralPerformanceOptimizationInstance
          moving_block_solution(instance);
      moving_block_solution.load_solution(moving_block_working_directory,
                                          moving_block_solution_subdirectory,
                                          moving_block_parameter_identifier);

      cda_rail::solver::mip_based::
          VSSGenTimetableSolverWithMovingBlockInformation solver(
              moving_block_solution);

      return solver.solve(
          {.delta_t                    = delta_t,
           .train_dynamics             = train_dynamics,
           .braking_curves             = braking_curves,
           .fix_stop_positions         = fix_stop_positions,
           .fix_exact_positions        = fix_exact_positions,
           .fix_exact_velocities       = fix_exact_velocities,
           .hint_approximate_positions = hint_approximate_positions,
           .fix_order_on_edges         = fix_order_on_edges},
          model_settings, solver_strategy, solution_settings, time_limit,
          debug_output, true);
    }

    cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
        instance_name, instance_subdirectory, working_directory);

    return solver.solve({.delta_t        = delta_t,
                         .fix_routes     = fix_routes,
                         .train_dynamics = train_dynamics,
                         .braking_curves = braking_curves},
                        model_settings, solver_strategy, solution_settings,
                        time_limit, debug_output, true);
  }();

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
