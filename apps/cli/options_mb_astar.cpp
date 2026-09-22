#include "options_mb_astar.hpp"

#include "CLI/CLI.hpp"
#include "Formatting.hpp"
#include "StringHelper.hpp"
#include "options_common.hpp"
#include "simulator/GreedyHeuristic.hpp"
#include "solver/astar-based/GenPOMovingBlockAStarSolver.hpp"

#include <limits>
#include <map>
#include <plog/Log.h>
#include <string>

// The reinterpret_cast warnings are false positives stemming from the plog
// macros.
// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast)

namespace {
using cda_rail::simulator::RemainingTimeHeuristicType;
using cda_rail::solver::astar_based::NextStateStrategy;

const std::map<std::string, NextStateStrategy>& next_state_strategy_map() {
  static const std::map<std::string, NextStateStrategy> map{
      {"SingleEdge", NextStateStrategy::SingleEdge},
      {"NextTTD", NextStateStrategy::NextTTD},
      {"NextRelevantTTD", NextStateStrategy::NextRelevantTTD}};
  return map;
}

const std::map<std::string, RemainingTimeHeuristicType>&
remaining_time_heuristic_type_map() {
  static const std::map<std::string, RemainingTimeHeuristicType> map{
      {"Zero", RemainingTimeHeuristicType::Zero},
      {"Simple", RemainingTimeHeuristicType::Simple}};
  return map;
}
} // namespace

void cda_rail::cli::add_mb_astar_options(CLI::App&        app,
                                         MbAStarSettings& settings) {
  app.add_option("-c,--dt,--timestep", settings.dt,
                 "Time step (dt) used in the simulation")
      ->check(CLI::PositiveNumber)
      ->capture_default_str()
      ->group("Model Parameters");
  app.add_flag("-l,--allow-late-entry", settings.late_entry_possible,
               "Allow late entry (delays) in the solution (default without "
               "flag is false)")
      ->group("Model Parameters");
  app.add_flag("!-f,!--speed-limit-only-on-train-front,--limit-speed-by-"
               "leaving-edges",
               settings.limit_speed_by_leaving_edges,
               "If this flag is set, trains only respect the limit of their "
               "front position. Otherwise (by default) any edge's speed limit "
               "any part of the train is on applies, i.e., by default speed "
               "limits of edges the train is leaving (and the front already "
               "left) are also relevant.")
      ->group("Model Parameters");
  app.add_flag("!-y,!--allow-early-exit,--consider-earliest-exit",
               settings.consider_earliest_exit,
               "Allow to leave stations and the network early. By defaults "
               "trains cannot leave before the scheduled time, i.e., the "
               "earliest exit times imposed by the schedule are respected.")
      ->group("Model Parameters");
  app.add_flag("-a,--time-aware-state-transitions",
               settings.time_aware_state_transitions,
               "If this flag is set, use time aware state transitions to "
               "avoid unnecessary state exploration.")
      ->group("Solver Parameters");
  app.add_option("-w,--heuristic-weight", settings.a_star_weight,
                 "Weight of the heuristic to use in the (weighted) A*. Has to "
                 "be >=1. If w is the weight, then A* is an w-approximation.")
      ->check(CLI::Range(1.0, std::numeric_limits<double>::max()))
      ->capture_default_str()
      ->group("Solver Parameters");
  app.add_option("-x,--next-state-strategy", settings.next_state_strategy,
                 "Next state strategy to use in the A* search. Currently "
                 "supports 'SingleEdge', 'NextTTD', and 'NextRelevantTTD'.")
      ->transform(
          CLI::CheckedTransformer(next_state_strategy_map(), CLI::ignore_case))
      ->capture_default_str()
      ->group("Solver Parameters");
  app.add_option("-r,--remaining-time-heuristic-strategy",
                 settings.remaining_time_heuristic_type,
                 "Remaining time heuristic strategy to use in the simulation. "
                 "Currently supports 'Zero' and 'Simple'")
      ->transform(CLI::CheckedTransformer(remaining_time_heuristic_type_map(),
                                          CLI::ignore_case))
      ->capture_default_str()
      ->group("Solver Parameters");

  add_solving_options(app, settings.solving);
  add_export_options(app, settings.exporting, {});
}

void cda_rail::cli::finalize_mb_astar_settings(MbAStarSettings& settings) {
  if (settings.exporting.generate_identifier) {
    settings.exporting.parameter_identifier =
        generate_mb_astar_identifier(settings);
  }
}

std::string
cda_rail::cli::generate_mb_astar_identifier(const MbAStarSettings& settings) {
  return concatenate_string_views(
      {format_double(settings.dt), "_",
       bool_to_str(settings.late_entry_possible), "_",
       bool_to_str(settings.limit_speed_by_leaving_edges), "_",
       bool_to_str(settings.consider_earliest_exit), "_",
       bool_to_str(settings.time_aware_state_transitions), "_",
       format_double(settings.a_star_weight), "_",
       solver::astar_based::next_state_strategy_to_string(
           settings.next_state_strategy),
       "_",
       simulator::remaining_time_heuristic_type_to_string(
           settings.remaining_time_heuristic_type),
       "_", std::to_string(settings.solving.time_limit)});
}

void cda_rail::cli::log_mb_astar_settings(
    const MbAStarSettings& settings, const std::string& working_directory) {
  PLOGD << "Solver Settings";
  PLOGD << "  Time step (dt): " << settings.dt;
  PLOGD << "  Allow late entry: " << yes_no(settings.late_entry_possible);
  PLOGD << "  Limit speed by leaving edges: "
        << yes_no(settings.limit_speed_by_leaving_edges);
  PLOGD << "  Consider earliest exit: "
        << yes_no(settings.consider_earliest_exit);
  PLOGD << "  Time aware state transitions: "
        << yes_no(settings.time_aware_state_transitions);
  PLOGD << "  Heuristic weight (w): " << settings.a_star_weight;
  PLOGD << "  Next state strategy: "
        << solver::astar_based::next_state_strategy_to_string(
               settings.next_state_strategy);
  PLOGD << "  Remaining time heuristic strategy: "
        << simulator::remaining_time_heuristic_type_to_string(
               settings.remaining_time_heuristic_type);
  log_solving_settings(settings.solving);
  log_export_settings(settings.exporting, working_directory);
}

cda_rail::solver::astar_based::ModelDetailMBAStar
cda_rail::cli::mb_astar_model_detail(const MbAStarSettings& settings) {
  return {.dt                  = settings.dt,
          .late_entry_possible = settings.late_entry_possible,
          .limit_speed_by_leaving_edges =
              settings.limit_speed_by_leaving_edges};
}

cda_rail::solver::astar_based::SolverStrategyMBAStar
cda_rail::cli::mb_astar_solver_strategy(const MbAStarSettings& settings) {
  return {.remaining_time_heuristic_type =
              settings.remaining_time_heuristic_type,
          .next_state_strategy          = settings.next_state_strategy,
          .consider_earliest_exit       = settings.consider_earliest_exit,
          .time_aware_state_transitions = settings.time_aware_state_transitions,
          .a_star_weight                = settings.a_star_weight};
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)
