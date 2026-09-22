#include "options_vss_mip.hpp"

#include "CLI/CLI.hpp"
#include "Definitions.hpp"
#include "Formatting.hpp"
#include "StringHelper.hpp"
#include "VSSModel.hpp"
#include "options_common.hpp"
#include "solver/mip-based/GeneralMIPSolver.hpp"
#include "solver/mip-based/VSSGenTimetableSolver.hpp"

#include <map>
#include <plog/Log.h>
#include <string>
#include <vector>

// Settings that also exist in the two moving block solvers use the very same
// short and long options there:
//   -c (time discretization), -f (fix routes), -t (time limit), -v (debug),
//   -o -i -b -e -p -g, --export-lp-model and --model-name (export).
// The remaining letters are assigned to the VSS specific settings; since the
// solvers barely share any parameters, a letter may well have a different
// meaning elsewhere.
//
// Since all lowercase letters are used up, the fine tuning settings
// (--only-stop-at-vss, --use-pwl, --use-schedule-cuts, --postprocess), the
// settings that are only relevant together with -x (--iterative-approach) and
// the remaining moving block loading options are long option only.

// The reinterpret_cast warnings are false positives stemming from the plog
// macros.
// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast)

namespace {
using cda_rail::cli::SeparationFunctionType;

const std::map<std::string, cda_rail::vss::ModelType>& vss_model_type_map() {
  static const std::map<std::string, cda_rail::vss::ModelType>
      VSS_MODEL_TYPE_MAP{
          {"Discrete", cda_rail::vss::ModelType::Discrete},
          {"Continuous", cda_rail::vss::ModelType::Continuous},
          {"Inferred", cda_rail::vss::ModelType::Inferred},
          {"InferredAlt", cda_rail::vss::ModelType::InferredAlt}};
  return VSS_MODEL_TYPE_MAP;
}

const std::map<std::string, SeparationFunctionType>& separation_function_map() {
  static const std::map<std::string, SeparationFunctionType>
      SEPARATION_FUNCTION_MAP{{"Uniform", SeparationFunctionType::Uniform},
                              {"Chebyshev", SeparationFunctionType::Chebyshev}};
  return SEPARATION_FUNCTION_MAP;
}

const std::map<std::string, cda_rail::OptimalityStrategy>&
optimality_strategy_map() {
  static const std::map<std::string, cda_rail::OptimalityStrategy>
      OPTIMALITY_STRATEGY_MAP{
          {"Optimal", cda_rail::OptimalityStrategy::Optimal},
          {"TradeOff", cda_rail::OptimalityStrategy::TradeOff},
          {"Feasible", cda_rail::OptimalityStrategy::Feasible}};
  return OPTIMALITY_STRATEGY_MAP;
}

const std::map<std::string, cda_rail::solver::mip_based::UpdateStrategyVSSGen>&
update_strategy_map() {
  static const std::map<std::string,
                        cda_rail::solver::mip_based::UpdateStrategyVSSGen>
      UPDATE_STRATEGY_MAP{
          {"Fixed", cda_rail::solver::mip_based::UpdateStrategyVSSGen::Fixed},
          {"Relative",
           cda_rail::solver::mip_based::UpdateStrategyVSSGen::Relative}};
  return UPDATE_STRATEGY_MAP;
}
} // namespace

std::vector<cda_rail::vss::SeparationFunction>
cda_rail::cli::VssMipSettings::separation_functions() const {
  std::vector<vss::SeparationFunction> functions;
  functions.reserve(separation_function_types.size());
  for (const auto& type : separation_function_types) {
    switch (type) {
    case SeparationFunctionType::Uniform:
      functions.emplace_back(vss::UNIFORM);
      break;
    case SeparationFunctionType::Chebyshev:
      functions.emplace_back(vss::CHEBYSHEV);
      break;
    }
  }
  return functions;
}

void cda_rail::cli::add_vss_mip_options(CLI::App&       app,
                                        VssMipSettings& settings) {
  auto* mb_solution_subdirectory_opt =
      app.add_option(
             "-m,--moving-block-solution-subdirectory",
             settings.moving_block_solution_subdirectory,
             "Subdirectory of a previously obtained moving block solution of "
             "the very same instance. If (and only if) this option is set, "
             "the solver using moving block information is used. The solution "
             "is loaded from moving_block_working_directory/solutions/"
             "moving_block_solution_subdirectory/instance_subdirectory/"
             "instance_name. Note that the routes of the moving block "
             "solution supersede the routes of the instance and that the "
             "discrete VSS model type is not supported in this case.")
          ->group("Moving Block Information");
  app.add_option("--moving-block-working-directory",
                 settings.moving_block_working_directory,
                 "Working directory from which the moving block solution is "
                 "loaded. If unset, the normal working directory is used.")
      ->needs(mb_solution_subdirectory_opt)
      ->group("Moving Block Information");
  app.add_option("--moving-block-parameter-identifier",
                 settings.moving_block_parameter_identifier,
                 "Parameter identifier that was appended to the instance name "
                 "when the moving block solution was exported. If empty, no "
                 "parameter identifier is assumed.")
      ->needs(mb_solution_subdirectory_opt)
      ->capture_default_str()
      ->group("Moving Block Information");

  app.add_flag("!-z,!--free-stop-positions,--fix-stop-positions",
               settings.fix_stop_positions,
               "The positions at which trains stop at a station are fixed to "
               "the ones of the moving block solution by default. If this "
               "flag is negated (-z or --free-stop-positions), they are "
               "optimized again.")
      ->needs(mb_solution_subdirectory_opt)
      ->group("Moving Block Information");
  app.add_flag("!-l,!--free-exact-positions,--fix-exact-positions",
               settings.fix_exact_positions,
               "The exact positions of the trains at every vertex are fixed "
               "to the ones of the moving block solution by default. If this "
               "flag is negated (-l or --free-exact-positions), they are only "
               "bounded by their minimal and maximal positions.")
      ->needs(mb_solution_subdirectory_opt)
      ->group("Moving Block Information");
  app.add_flag("!-y,!--free-exact-velocities,--fix-exact-velocities",
               settings.fix_exact_velocities,
               "The exact velocities of the trains at every vertex are fixed "
               "to the ones of the moving block solution by default. If this "
               "flag is negated (-y or --free-exact-velocities), they are "
               "optimized again.")
      ->needs(mb_solution_subdirectory_opt)
      ->group("Moving Block Information");
  app.add_flag("!-u,!--no-position-hints,--hint-approximate-positions",
               settings.hint_approximate_positions,
               "The approximate positions of the trains at every point in "
               "time are hinted to the solver by default. If this flag is "
               "negated (-u or --no-position-hints), no such hints are "
               "given.")
      ->needs(mb_solution_subdirectory_opt)
      ->group("Moving Block Information");
  app.add_flag("!-w,!--free-order-on-edges,--fix-order-on-edges",
               settings.fix_order_on_edges,
               "The order in which the trains traverse the edges is fixed to "
               "the one of the moving block solution by default. If this flag "
               "is negated (-w or --free-order-on-edges), the order is "
               "optimized again.")
      ->needs(mb_solution_subdirectory_opt)
      ->group("Moving Block Information");

  app.add_option("-c,--delta-t,--dt,--timestep", settings.delta_t,
                 "Length of the discretized time intervals in seconds.")
      ->check(CLI::PositiveNumber)
      ->capture_default_str()
      ->group("Model Parameters");
  app.add_flag("!-f,!--free-routes,--fix-routes", settings.fix_routes,
               "The routes given by the instance are fixed by default. If "
               "this flag is negated (-f or --free-routes), the routes are "
               "optimized as well. Not applicable together with moving block "
               "information, in which case the routes are always fixed to the "
               "ones of the moving block solution.")
      ->excludes(mb_solution_subdirectory_opt)
      ->group("Model Parameters");
  app.add_flag("!-a,!--no-train-dynamics,--train-dynamics",
               settings.train_dynamics,
               "The train dynamics (i.e., limited acceleration and "
               "deceleration) are included in the model by default. If this "
               "flag is negated (-a or --no-train-dynamics), they are "
               "omitted.")
      ->group("Model Parameters");
  app.add_flag("!-k,!--no-braking-curves,--braking-curves",
               settings.braking_curves,
               "The braking curves (i.e., the braking distance depending on "
               "the current speed has to be cleared) are included in the "
               "model by default. If this flag is negated (-k or "
               "--no-braking-curves), they are omitted.")
      ->group("Model Parameters");
  app.add_option("-r,--vss-model-type", settings.vss_model_type,
                 "Denotes how the VSS borders are modelled in the solution "
                 "process. Currently supports 'Discrete', 'Continuous', "
                 "'Inferred', and 'InferredAlt'. 'Discrete' expects exactly "
                 "one, 'Inferred' and 'InferredAlt' expect at least one "
                 "separation function, whereas 'Continuous' expects none.")
      ->transform(
          CLI::CheckedTransformer(vss_model_type_map(), CLI::ignore_case))
      ->capture_default_str()
      ->group("Model Parameters");
  app.add_option("-j,--separation-functions",
                 settings.separation_function_types,
                 "Separation functions used by the VSS model type. Can be "
                 "passed multiple times. Currently supports 'Uniform' and "
                 "'Chebyshev'.")
      ->transform(
          CLI::CheckedTransformer(separation_function_map(), CLI::ignore_case))
      ->multi_option_policy(CLI::MultiOptionPolicy::TakeAll)
      ->group("Model Parameters");
  app.add_flag("--only-stop-at-vss", settings.only_stop_at_vss,
               "If this flag is set, trains are only allowed to stop at VSS "
               "borders.")
      ->group("Model Parameters");
  app.add_flag("--use-pwl", settings.use_pwl,
               "If this flag is set, the braking distances are approximated "
               "by piecewise linear functions with a fixed maximal error. "
               "Otherwise (by default) they are modelled as quadratic "
               "functions using Gurobi's ability to solve these by spatial "
               "branching. Only relevant if braking curves are included.")
      ->group("Model Parameters");
  app.add_flag("!--no-schedule-cuts,--use-schedule-cuts",
               settings.use_schedule_cuts,
               "The formulation is strengthened using cuts implied by the "
               "schedule by default. If this flag is negated "
               "(--no-schedule-cuts), these cuts are omitted.")
      ->group("Model Parameters");

  auto* iterative_approach_flag =
      app.add_flag("-x,--iterative-approach", settings.iterative_approach,
                   "If this flag is set, the number of VSS per edge is "
                   "iteratively increased until optimality is proven instead "
                   "of solving the full model at once.")
          ->group("Solver Parameters");
  app.add_option("-q,--optimality-strategy", settings.optimality_strategy,
                 "Optimality strategy to use. Currently supports 'Optimal', "
                 "'TradeOff', and 'Feasible'.")
      ->transform(
          CLI::CheckedTransformer(optimality_strategy_map(), CLI::ignore_case))
      ->capture_default_str()
      ->group("Solver Parameters");
  app.add_option("--iterative-update-strategy",
                 settings.iterative_update_strategy,
                 "Strategy used to update the number of VSS per edge within "
                 "the iterative approach. Currently supports 'Fixed' "
                 "(absolute number) and 'Relative' (fraction of the "
                 "theoretically possible number).")
      ->transform(
          CLI::CheckedTransformer(update_strategy_map(), CLI::ignore_case))
      ->needs(iterative_approach_flag)
      ->capture_default_str()
      ->group("Solver Parameters");
  app.add_option("--iterative-initial-value", settings.iterative_initial_value,
                 "Initial number of VSS per edge ('Fixed' update strategy, "
                 "has to be an integer) or initial fraction of the "
                 "theoretically possible number ('Relative' update strategy, "
                 "has to be within (0,1]) used by the iterative approach.")
      ->check(CLI::PositiveNumber)
      ->needs(iterative_approach_flag)
      ->capture_default_str()
      ->group("Solver Parameters");
  app.add_option("--iterative-update-value", settings.iterative_update_value,
                 "Value by which the number of VSS per edge is increased in "
                 "every iteration. Has to be greater than 1 for the 'Fixed' "
                 "and within (0,1) for the 'Relative' update strategy.")
      ->check(CLI::PositiveNumber)
      ->needs(iterative_approach_flag)
      ->capture_default_str()
      ->group("Solver Parameters");
  app.add_flag("!--no-iterative-cuts,--iterative-cuts",
               settings.iterative_include_cuts,
               "Cuts excluding the already explored search space are added in "
               "every iteration of the iterative approach by default. If this "
               "flag is negated (--no-iterative-cuts), these cuts are "
               "omitted.")
      ->needs(iterative_approach_flag)
      ->group("Solver Parameters");

  add_solving_options(app, settings.solving);
  add_export_options(app, settings.exporting,
                     {.lp_model                    = true,
                      .postprocess                 = true,
                      .lp_model_solution_extension = ".sol"});
}

void cda_rail::cli::finalize_vss_mip_settings(VssMipSettings& settings) {
  if (settings.exporting.generate_identifier) {
    settings.exporting.parameter_identifier =
        generate_vss_mip_identifier(settings);
  }
}

std::string
cda_rail::cli::generate_vss_mip_identifier(const VssMipSettings& settings) {
  std::string identifier = concatenate_string_views(
      {format_double(settings.delta_t), "_",
       bool_to_str(settings.train_dynamics), "_",
       bool_to_str(settings.braking_curves), "_",
       key_by_value(vss_model_type_map(), settings.vss_model_type)});
  for (const auto& separation_function : settings.separation_functions()) {
    identifier = concatenate_string_views(
        {identifier, "_", separation_function.get_name()});
  }
  identifier = concatenate_string_views(
      {identifier,
       "_",
       bool_to_str(settings.only_stop_at_vss),
       "_",
       bool_to_str(settings.use_pwl),
       "_",
       bool_to_str(settings.use_schedule_cuts),
       "_",
       bool_to_str(settings.iterative_approach),
       "_",
       key_by_value(optimality_strategy_map(), settings.optimality_strategy),
       "_",
       key_by_value(update_strategy_map(), settings.iterative_update_strategy),
       "_",
       format_double(settings.iterative_initial_value),
       "_",
       format_double(settings.iterative_update_value),
       "_",
       bool_to_str(settings.iterative_include_cuts),
       "_",
       bool_to_str(settings.exporting.postprocess),
       "_",
       std::to_string(settings.solving.time_limit)});
  // The settings that only exist for one of the two solvers are appended
  // last, so that the identifiers of the settings both of them share keep the
  // very same structure.
  return settings.use_moving_block_information()
             ? concatenate_string_views(
                   {identifier, "_", bool_to_str(settings.fix_stop_positions),
                    "_", bool_to_str(settings.fix_exact_positions), "_",
                    bool_to_str(settings.fix_exact_velocities), "_",
                    bool_to_str(settings.hint_approximate_positions), "_",
                    bool_to_str(settings.fix_order_on_edges)})
             : concatenate_string_views(
                   {identifier, "_", bool_to_str(settings.fix_routes)});
}

void cda_rail::cli::log_vss_mip_settings(const VssMipSettings& settings,
                                         const std::string& working_directory) {
  PLOGD << "Moving Block Information";
  PLOGD << "  Use moving block information: "
        << yes_no(settings.use_moving_block_information());
  if (settings.use_moving_block_information()) {
    PLOGD << "  Moving block working directory: "
          << settings.moving_block_working_directory.value_or(
                 working_directory);
    PLOGD << "  Moving block solution subdirectory: "
          << settings.moving_block_solution_subdirectory;
    PLOGD << "  Moving block parameter identifier: "
          << settings.moving_block_parameter_identifier.value_or("-");
    PLOGD << "  Fix stop positions: " << yes_no(settings.fix_stop_positions);
    PLOGD << "  Fix exact positions: " << yes_no(settings.fix_exact_positions);
    PLOGD << "  Fix exact velocities: "
          << yes_no(settings.fix_exact_velocities);
    PLOGD << "  Hint approximate positions: "
          << yes_no(settings.hint_approximate_positions);
    PLOGD << "  Fix order on edges: " << yes_no(settings.fix_order_on_edges);
  }
  PLOGD << "Model Settings";
  PLOGD << "  Time discretization (delta_t): " << settings.delta_t;
  if (!settings.use_moving_block_information()) {
    PLOGD << "  Fix routes: " << yes_no(settings.fix_routes);
  }
  PLOGD << "  Train dynamics: " << yes_no(settings.train_dynamics);
  PLOGD << "  Braking curves: " << yes_no(settings.braking_curves);
  PLOGD << "  VSS model type: "
        << key_by_value(vss_model_type_map(), settings.vss_model_type);
  for (const auto& separation_function : settings.separation_functions()) {
    PLOGD << "  Separation function: " << separation_function.get_name();
  }
  PLOGD << "  Only stop at VSS: " << yes_no(settings.only_stop_at_vss);
  PLOGD << "  Use PWL: " << yes_no(settings.use_pwl);
  PLOGD << "  Use schedule cuts: " << yes_no(settings.use_schedule_cuts);
  PLOGD << "Solver Settings";
  PLOGD << "  Iterative approach: " << yes_no(settings.iterative_approach);
  PLOGD << "  Optimality strategy: "
        << key_by_value(optimality_strategy_map(),
                        settings.optimality_strategy);
  if (settings.iterative_approach) {
    PLOGD << "  Iterative update strategy: "
          << key_by_value(update_strategy_map(),
                          settings.iterative_update_strategy);
    PLOGD << "  Iterative initial value: " << settings.iterative_initial_value;
    PLOGD << "  Iterative update value: " << settings.iterative_update_value;
    PLOGD << "  Iterative include cuts: "
          << yes_no(settings.iterative_include_cuts);
  }
  log_solving_settings(settings.solving);
  log_export_settings(settings.exporting, working_directory);
}

cda_rail::solver::mip_based::ModelDetailVSSGen
cda_rail::cli::vss_mip_model_detail(const VssMipSettings& settings) {
  return {.delta_t        = settings.delta_t,
          .fix_routes     = settings.fix_routes,
          .train_dynamics = settings.train_dynamics,
          .braking_curves = settings.braking_curves};
}

cda_rail::solver::mip_based::ModelDetailMBInformation
cda_rail::cli::vss_mip_model_detail_mb(const VssMipSettings& settings) {
  return {.delta_t                    = settings.delta_t,
          .train_dynamics             = settings.train_dynamics,
          .braking_curves             = settings.braking_curves,
          .fix_stop_positions         = settings.fix_stop_positions,
          .fix_exact_positions        = settings.fix_exact_positions,
          .fix_exact_velocities       = settings.fix_exact_velocities,
          .hint_approximate_positions = settings.hint_approximate_positions,
          .fix_order_on_edges         = settings.fix_order_on_edges};
}

cda_rail::solver::mip_based::ModelSettingsVSSGen
cda_rail::cli::vss_mip_model_settings(const VssMipSettings& settings) {
  return {.model_type        = vss::Model(settings.vss_model_type,
                                          settings.separation_functions(),
                                          settings.only_stop_at_vss),
          .use_pwl           = settings.use_pwl,
          .use_schedule_cuts = settings.use_schedule_cuts};
}

cda_rail::solver::mip_based::SolverStrategyVSSGen
cda_rail::cli::vss_mip_solver_strategy(const VssMipSettings& settings) {
  return {.iterative_approach  = settings.iterative_approach,
          .optimality_strategy = settings.optimality_strategy,
          .update_strategy     = settings.iterative_update_strategy,
          .initial_value       = settings.iterative_initial_value,
          .update_value        = settings.iterative_update_value,
          .include_cuts        = settings.iterative_include_cuts};
}

cda_rail::solver::mip_based::SolutionSettingsVSSGen
cda_rail::cli::vss_mip_solution_settings(const VssMipSettings& settings,
                                         const std::string& working_directory) {
  solver::mip_based::SolutionSettingsVSSGen solution_settings;
  static_cast<solver::GeneralSolutionSettings&>(solution_settings) =
      general_solution_settings(settings.exporting, working_directory);
  solution_settings.export_lp_model = settings.exporting.export_lp_model;
  solution_settings.model_name      = settings.exporting.model_name;
  solution_settings.postprocess     = settings.exporting.postprocess;
  return solution_settings;
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)
