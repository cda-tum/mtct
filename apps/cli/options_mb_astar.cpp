#include "options_mb_astar.hpp"

#include "CLI/CLI.hpp"
#include "Formatting.hpp"
#include "StringHelper.hpp"
#include "options_common.hpp"
#include "simulator/GeneralSimulator.hpp"
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
using cda_rail::simulator::SimulatorType;
using cda_rail::solver::astar_based::NextStateStrategy;

const std::map<std::string, SimulatorType>& simulator_type_map() {
  static const std::map<std::string, SimulatorType> SIMULATOR_TYPE_MAP{
      {"Greedy", SimulatorType::Greedy},
      {"EventBased", SimulatorType::EventBased}};
  return SIMULATOR_TYPE_MAP;
}

const std::map<std::string, NextStateStrategy>& next_state_strategy_map() {
  static const std::map<std::string, NextStateStrategy> NEXT_STATE_STRATEGY_MAP{
      {"SingleEdge", NextStateStrategy::SingleEdge},
      {"NextTTD", NextStateStrategy::NextTTD},
      {"NextRelevantTTD", NextStateStrategy::NextRelevantTTD}};
  return NEXT_STATE_STRATEGY_MAP;
}

const std::map<std::string, RemainingTimeHeuristicType>&
remaining_time_heuristic_type_map() {
  static const std::map<std::string, RemainingTimeHeuristicType>
      REMAINING_TIME_HEURISTIC_TYPE_MAP{
          {"Zero", RemainingTimeHeuristicType::Zero},
          {"Simple", RemainingTimeHeuristicType::Simple}};
  return REMAINING_TIME_HEURISTIC_TYPE_MAP;
}
} // namespace

void cda_rail::cli::add_mb_astar_options(CLI::App&        app,
                                         MbAStarSettings& settings) {
  app.add_option("-c,--dt,--timestep", settings.dt,
                 "Time step (dt) used in the greedy simulation. The "
                 "event-based simulation simulates a train following another "
                 "one again after at least this time.")
      ->check(CLI::PositiveNumber)
      ->capture_default_str()
      ->group("Model Parameters");
  app.add_option("--simulator", settings.simulator_type,
                 "Simulator used to evaluate the states of the search. "
                 "Currently supports 'Greedy' and 'EventBased'.")
      ->transform(
          CLI::CheckedTransformer(simulator_type_map(), CLI::ignore_case))
      ->capture_default_str()
      ->group("Model Parameters");
  app.add_flag("-l,--allow-late-entry", settings.late_entry_possible,
               "If this flag is set, trains may enter the network later than "
               "scheduled. Otherwise (by default) they enter exactly at their "
               "scheduled time.")
      ->group("Model Parameters");
  app.add_flag("!-f,!--speed-limit-only-on-train-front,--limit-speed-by-"
               "leaving-edges",
               settings.limit_speed_by_leaving_edges,
               "The speed limit of every edge any part of the train is on "
               "applies by default, i.e., also the limits of edges the train "
               "is leaving (and the front already left). If this flag is "
               "negated (-f or --speed-limit-only-on-train-front), trains only "
               "respect the limit at their front position.")
      ->group("Model Parameters");
  app.add_flag("!-y,!--allow-early-exit,--consider-earliest-exit",
               settings.consider_earliest_exit,
               "The earliest exit times imposed by the schedule are respected "
               "by default, i.e., trains cannot leave stations and the network "
               "before their scheduled time. If this flag is negated (-y or "
               "--allow-early-exit), they may leave early.")
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
  app.add_option("--completion-interval", settings.completion_interval,
                 "If x > 0, the initial state and every x-th state after it "
                 "that the search expands are completed heuristically to a "
                 "feasible solution, if possible. 0 never completes a state.")
      ->capture_default_str()
      ->group("Solver Parameters");

  add_solving_options(app, settings.solving);
  add_export_options(app, settings.exporting, {});
}

void cda_rail::cli::finalize_mb_astar_settings(MbAStarSettings& settings) {
  if (!settings.exporting.evaluation_time_step.has_value()) {
    settings.exporting.evaluation_time_step = settings.dt;
  }
  if (settings.exporting.generate_identifier) {
    settings.exporting.parameter_identifier =
        hash_identifier(generate_mb_astar_identifier(settings));
  }
}

std::string
cda_rail::cli::generate_mb_astar_identifier(const MbAStarSettings& settings) {
  // The completion interval is only appended if completions are used, so
  // that the identifiers of all other settings stay as they were
  return concatenate_string_views(
      {format_double(settings.dt),
       "_",
       simulator::simulator_type_to_string(settings.simulator_type),
       "_",
       bool_to_str(settings.late_entry_possible),
       "_",
       bool_to_str(settings.limit_speed_by_leaving_edges),
       "_",
       bool_to_str(settings.consider_earliest_exit),
       "_",
       bool_to_str(settings.time_aware_state_transitions),
       "_",
       format_double(settings.a_star_weight),
       "_",
       solver::astar_based::next_state_strategy_to_string(
           settings.next_state_strategy),
       "_",
       simulator::remaining_time_heuristic_type_to_string(
           settings.remaining_time_heuristic_type),
       "_",
       std::to_string(settings.solving.time_limit),
       settings.completion_interval > 0
           ? "_" + std::to_string(settings.completion_interval)
           : ""});
}

void cda_rail::cli::log_mb_astar_settings(
    const MbAStarSettings& settings, const std::string& working_directory) {
  PLOGD << "Solver Settings";
  PLOGD << "  Time step (dt): " << settings.dt;
  PLOGD << "  Simulator: "
        << simulator::simulator_type_to_string(settings.simulator_type);
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
  PLOGD << "  Completion interval: " << settings.completion_interval;
  log_solving_settings(settings.solving);
  log_export_settings(settings.exporting, working_directory);
}

cda_rail::solver::astar_based::ModelDetailMBAStar
cda_rail::cli::mb_astar_model_detail(const MbAStarSettings& settings) {
  return {.dt                  = settings.dt,
          .simulator_type      = settings.simulator_type,
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
          .a_star_weight                = settings.a_star_weight,
          .completion_interval          = settings.completion_interval};
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)
