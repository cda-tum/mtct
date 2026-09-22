#include "options_mb_mip.hpp"

#include "CLI/CLI.hpp"
#include "Definitions.hpp"
#include "Formatting.hpp"
#include "StringHelper.hpp"
#include "options_common.hpp"
#include "solver/mip-based/GenPOMovingBlockMIPSolver.hpp"
#include "solver/mip-based/GeneralMIPSolver.hpp"

#include <map>
#include <plog/Log.h>
#include <string>

// Settings that also exist in the A* solver use the very same short and long
// options there:
//   -l (late entry), -t (time limit), -v (debug), -o -i -b -e -p -g (export).
// The remaining letters are assigned to the MIP specific settings; since the
// two solvers barely share any parameters, a letter may well have a different
// meaning for the A* solver. Since all lowercase letters are used up,
// --max-station-delay, --max-delay, --export-lp-model and --model-name are
// long option only.

// The reinterpret_cast warnings are false positives stemming from the plog
// macros.
// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast)

namespace {
using cda_rail::solver::mip_based::LazyConstraintSelectionStrategy;
using cda_rail::solver::mip_based::LazyTrainSelectionStrategy;

const std::map<std::string, cda_rail::VelocityRefinementStrategy>&
velocity_refinement_strategy_map() {
  static const std::map<std::string, cda_rail::VelocityRefinementStrategy>
      VELOCITY_REFINEMENT_STRATEGY_MAP{
          {"None", cda_rail::VelocityRefinementStrategy::None},
          {"MinOneStep", cda_rail::VelocityRefinementStrategy::MinOneStep}};
  return VELOCITY_REFINEMENT_STRATEGY_MAP;
}

const std::map<std::string, LazyConstraintSelectionStrategy>&
lazy_constraint_selection_strategy_map() {
  static const std::map<std::string, LazyConstraintSelectionStrategy>
      LAZY_CONSTRAINT_SELECTION_STRATEGY_MAP{
          {"OnlyViolated", LazyConstraintSelectionStrategy::OnlyViolated},
          {"OnlyFirstFound", LazyConstraintSelectionStrategy::OnlyFirstFound},
          {"AllChecked", LazyConstraintSelectionStrategy::AllChecked}};
  return LAZY_CONSTRAINT_SELECTION_STRATEGY_MAP;
}

const std::map<std::string, LazyTrainSelectionStrategy>&
lazy_train_selection_strategy_map() {
  static const std::map<std::string, LazyTrainSelectionStrategy>
      LAZY_TRAIN_SELECTION_STRATEGY_MAP{
          {"OnlyAdjacent", LazyTrainSelectionStrategy::OnlyAdjacent},
          {"All", LazyTrainSelectionStrategy::All}};
  return LAZY_TRAIN_SELECTION_STRATEGY_MAP;
}
} // namespace

void cda_rail::cli::add_mb_mip_options(CLI::App& app, MbMipSettings& settings) {
  app.add_flag("-f,--fix-routes", settings.fix_routes,
               "If this flag is set, the routes given by the instance are "
               "fixed. Otherwise (by default) the routes are optimized as "
               "well.")
      ->group("Model Parameters");
  app.add_option("-m,--max-velocity-delta", settings.max_velocity_delta,
                 "Maximal velocity difference (in m/s) between two "
                 "consecutive velocity extensions of a train.")
      ->check(CLI::PositiveNumber)
      ->capture_default_str()
      ->group("Model Parameters");
  app.add_option("-r,--velocity-refinement-strategy",
                 settings.velocity_refinement_strategy,
                 "Strategy used to refine the velocity extensions. Currently "
                 "supports 'None' and 'MinOneStep'.")
      ->transform(CLI::CheckedTransformer(velocity_refinement_strategy_map(),
                                          CLI::ignore_case))
      ->capture_default_str()
      ->group("Model Parameters");
  app.add_flag("-y,--simplify-headway-constraints",
               settings.simplify_headway_constraints,
               "If this flag is set, simplified (weaker) headway constraints "
               "are used instead of the full ones.")
      ->group("Model Parameters");
  app.add_flag("-q,--strengthen-vertex-headway-constraints",
               settings.strengthen_vertex_headway_constraints,
               "If this flag is set, the vertex headway constraints are "
               "strengthened.")
      ->group("Model Parameters");
  app.add_flag("-l,--allow-late-entry", settings.late_entry_possible,
               "Allow late entry (delays) in the solution (default without "
               "flag is false)")
      ->group("Model Parameters");
  app.add_flag("!--no-minimum-time-bounds,--use-minimum-time-bounds",
               settings.use_minimum_time_bounds,
               "Every timing variable is bounded by the minimal running time "
               "the train needs to reach the corresponding event by default. "
               "If this flag is negated (--no-minimum-time-bounds), these "
               "bounds are omitted, which weakens the LP relaxation "
               "considerably and is only useful to measure their effect.")
      ->group("Model Parameters");
  auto* max_exit_delay_opt =
      app.add_option("-x,--max-exit-delay", settings.max_exit_delay,
                     "Maximal delay (in seconds) with which a train is allowed "
                     "to leave the network compared to its scheduled exit "
                     "time.")
          ->check(CLI::NonNegativeNumber)
          ->capture_default_str()
          ->group("Model Parameters");
  auto* max_station_delay_opt =
      app.add_option("--max-station-delay", settings.max_station_delay,
                     "Maximal delay (in seconds) with which a train is allowed "
                     "to be serviced at a station compared to its scheduled "
                     "service time.")
          ->check(CLI::NonNegativeNumber)
          ->capture_default_str()
          ->group("Model Parameters");
  app.add_option("--max-delay", settings.max_delay,
                 "Maximal delay (in seconds) used for both the exit and the "
                 "station delay at once. Mutually exclusive with the "
                 "individual --max-exit-delay and --max-station-delay "
                 "settings.")
      ->check(CLI::NonNegativeNumber)
      ->excludes(max_exit_delay_opt)
      ->excludes(max_station_delay_opt)
      ->group("Model Parameters");

  app.add_flag("-c,--use-indicator-constraints",
               settings.use_indicator_constraints,
               "If this flag is set, indicator constraints are used instead "
               "of big-M formulations where possible.")
      ->group("Solver Parameters");
  app.add_flag("!-z,!--no-lazy-constraints,--use-lazy-constraints",
               settings.use_lazy_constraints,
               "Headway constraints are separated lazily by default. If this "
               "flag is negated (-z or --no-lazy-constraints), they are added "
               "to the model upfront and all remaining lazy settings are "
               "ignored.")
      ->group("Solver Parameters");
  app.add_flag("-w,--include-reverse-headways",
               settings.include_reverse_headways,
               "If this flag is set, headways on reverse edges are separated "
               "as well. Only possible together with lazy constraints using "
               "the 'AllChecked' lazy constraint selection strategy.")
      ->group("Solver Parameters");
  app.add_flag("-u,--include-higher-velocities-in-edge-expr",
               settings.include_higher_velocities_in_edge_expr,
               "If this flag is set, higher velocities are included in the "
               "edge expressions of the lazy headway constraints.")
      ->group("Solver Parameters");
  app.add_option("-j,--lazy-constraint-selection-strategy",
                 settings.lazy_constraint_selection_strategy,
                 "Strategy deciding which violated lazy constraints are "
                 "added. Currently supports 'OnlyViolated', 'OnlyFirstFound', "
                 "and 'AllChecked'.")
      ->transform(CLI::CheckedTransformer(
          lazy_constraint_selection_strategy_map(), CLI::ignore_case))
      ->capture_default_str()
      ->group("Solver Parameters");
  app.add_option("-k,--lazy-train-selection-strategy",
                 settings.lazy_train_selection_strategy,
                 "Strategy deciding which train pairs are checked when "
                 "separating lazy constraints. Currently supports "
                 "'OnlyAdjacent' and 'All'.")
      ->transform(CLI::CheckedTransformer(lazy_train_selection_strategy_map(),
                                          CLI::ignore_case))
      ->capture_default_str()
      ->group("Solver Parameters");
  app.add_option("-a,--abs-mip-gap", settings.abs_mip_gap,
                 "Absolute MIP gap used as termination criterion of the "
                 "solver.")
      ->check(CLI::NonNegativeNumber)
      ->capture_default_str()
      ->group("Solver Parameters");

  add_solving_options(app, settings.solving);
  add_export_options(app, settings.exporting, {.lp_model = true});
}

void cda_rail::cli::finalize_mb_mip_settings(CLI::App&      app,
                                             MbMipSettings& settings) {
  if (app.count("--max-delay") > 0) {
    settings.max_exit_delay    = settings.max_delay;
    settings.max_station_delay = settings.max_delay;
  }
  if (settings.exporting.generate_identifier) {
    settings.exporting.parameter_identifier =
        generate_mb_mip_identifier(settings);
  }
}

std::string
cda_rail::cli::generate_mb_mip_identifier(const MbMipSettings& settings) {
  return concatenate_string_views(
      {bool_to_str(settings.fix_routes),
       "_",
       format_double(settings.max_velocity_delta),
       "_",
       solver::mip_based::velocity_refinement_strategy_to_string(
           settings.velocity_refinement_strategy),
       "_",
       bool_to_str(settings.simplify_headway_constraints),
       "_",
       bool_to_str(settings.strengthen_vertex_headway_constraints),
       "_",
       bool_to_str(settings.late_entry_possible),
       "_",
       bool_to_str(settings.use_minimum_time_bounds),
       "_",
       format_double(settings.max_exit_delay),
       "_",
       format_double(settings.max_station_delay),
       "_",
       bool_to_str(settings.use_indicator_constraints),
       "_",
       bool_to_str(settings.use_lazy_constraints),
       "_",
       bool_to_str(settings.include_reverse_headways),
       "_",
       bool_to_str(settings.include_higher_velocities_in_edge_expr),
       "_",
       solver::mip_based::lazy_constraint_selection_strategy_to_string(
           settings.lazy_constraint_selection_strategy),
       "_",
       solver::mip_based::lazy_train_selection_strategy_to_string(
           settings.lazy_train_selection_strategy),
       "_",
       format_double(settings.abs_mip_gap),
       "_",
       std::to_string(settings.solving.time_limit)});
}

void cda_rail::cli::log_mb_mip_settings(const MbMipSettings& settings,
                                        const std::string& working_directory) {
  PLOGD << "Model Settings";
  PLOGD << "  Fix routes: " << yes_no(settings.fix_routes);
  PLOGD << "  Maximal velocity delta: " << settings.max_velocity_delta;
  PLOGD << "  Velocity refinement strategy: "
        << solver::mip_based::velocity_refinement_strategy_to_string(
               settings.velocity_refinement_strategy);
  PLOGD << "  Simplify headway constraints: "
        << yes_no(settings.simplify_headway_constraints);
  PLOGD << "  Strengthen vertex headway constraints: "
        << yes_no(settings.strengthen_vertex_headway_constraints);
  PLOGD << "  Allow late entry: " << yes_no(settings.late_entry_possible);
  PLOGD << "  Use minimum time bounds: "
        << yes_no(settings.use_minimum_time_bounds);
  PLOGD << "  Maximal exit delay: " << settings.max_exit_delay;
  PLOGD << "  Maximal station delay: " << settings.max_station_delay;
  PLOGD << "Solver Settings";
  PLOGD << "  Use indicator constraints: "
        << yes_no(settings.use_indicator_constraints);
  PLOGD << "  Use lazy constraints: " << yes_no(settings.use_lazy_constraints);
  PLOGD << "  Include reverse headways: "
        << yes_no(settings.include_reverse_headways);
  PLOGD << "  Include higher velocities in edge expressions: "
        << yes_no(settings.include_higher_velocities_in_edge_expr);
  PLOGD << "  Lazy constraint selection strategy: "
        << solver::mip_based::lazy_constraint_selection_strategy_to_string(
               settings.lazy_constraint_selection_strategy);
  PLOGD << "  Lazy train selection strategy: "
        << solver::mip_based::lazy_train_selection_strategy_to_string(
               settings.lazy_train_selection_strategy);
  PLOGD << "  Absolute MIP gap: " << settings.abs_mip_gap;
  log_solving_settings(settings.solving);
  log_export_settings(settings.exporting, working_directory);
}

cda_rail::solver::mip_based::ModelDetailMovingBlock
cda_rail::cli::mb_mip_model_detail(const MbMipSettings& settings) {
  return {.fix_routes                   = settings.fix_routes,
          .max_velocity_delta           = settings.max_velocity_delta,
          .velocity_refinement_strategy = settings.velocity_refinement_strategy,
          .simplify_headway_constraints = settings.simplify_headway_constraints,
          .strengthen_vertex_headway_constraints =
              settings.strengthen_vertex_headway_constraints,
          .allow_late_entry        = settings.late_entry_possible,
          .use_minimum_time_bounds = settings.use_minimum_time_bounds,
          .max_exit_delay          = settings.max_exit_delay,
          .max_station_delay       = settings.max_station_delay};
}

cda_rail::solver::mip_based::SolverStrategyMovingBlock
cda_rail::cli::mb_mip_solver_strategy(const MbMipSettings& settings) {
  return {.use_indicator_constraints = settings.use_indicator_constraints,
          .use_lazy_constraints      = settings.use_lazy_constraints,
          .include_reverse_headways  = settings.include_reverse_headways,
          .include_higher_velocities_in_edge_expr =
              settings.include_higher_velocities_in_edge_expr,
          .lazy_constraint_selection_strategy =
              settings.lazy_constraint_selection_strategy,
          .lazy_train_selection_strategy =
              settings.lazy_train_selection_strategy,
          .abs_mip_gap = settings.abs_mip_gap};
}

cda_rail::solver::mip_based::SolutionSettingsMovingBlock
cda_rail::cli::mb_mip_solution_settings(const MbMipSettings& settings,
                                        const std::string& working_directory) {
  solver::mip_based::SolutionSettingsMovingBlock solution_settings;
  static_cast<solver::GeneralSolutionSettings&>(solution_settings) =
      general_solution_settings(settings.exporting, working_directory);
  solution_settings.export_lp_model = settings.exporting.export_lp_model;
  solution_settings.model_name      = settings.exporting.model_name;
  return solution_settings;
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)
