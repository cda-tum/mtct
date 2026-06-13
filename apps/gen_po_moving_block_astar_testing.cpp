#include "CLI/CLI.hpp"
#include "plog/Init.h"
#include "plog/Logger.h"
#include "plog/Severity.h"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "simulator/GreedyHeuristic.hpp"
#include "solver/astar-based/GenPOMovingBlockAStarSolver.hpp"

#include <cstdlib>
#include <map>
#include <plog/Appenders/ColorConsoleAppender.h>
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Log.h>
#include <string>

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,bugprone-exception-escape)

int main(int argc, char** argv) {
  // Only log to console using std::cerr and std::cout respectively unless
  // initialized differently
  if (plog::get() == nullptr) {
    static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
    plog::init(plog::debug, &console_appender);
  }

  CLI::App app{"Moving Block Optimization using A*"};
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

  // solver parameters
  double dt{6};
  bool   late_entry_possible{false};
  bool   limit_speed_by_leaving_edges{true};
  bool   consider_earliest_exit{true};
  bool   time_aware_state_transitions{false};
  double a_star_weight{1.0};

  double time_limit{-1};
  bool   debug_output{false};

  cda_rail::solver::astar_based::NextStateStrategy next_state_strategy{
      cda_rail::solver::astar_based::NextStateStrategy::SingleEdge};
  cda_rail::simulator::BrakingTimeHeuristicType braking_time_heuristic_type{
      cda_rail::simulator::BrakingTimeHeuristicType::Simple};
  cda_rail::simulator::RemainingTimeHeuristicType remaining_time_heuristic_type{
      cda_rail::simulator::RemainingTimeHeuristicType::Simple};

  // helper maps
  std::map<std::string, cda_rail::solver::astar_based::NextStateStrategy> const
      next_state_strategy_map{
          {"SingleEdge",
           cda_rail::solver::astar_based::NextStateStrategy::SingleEdge},
          {"NextTTD",
           cda_rail::solver::astar_based::NextStateStrategy::NextTTD}};
  std::map<std::string, cda_rail::simulator::BrakingTimeHeuristicType> const
      braking_time_heuristic_type_map{
          {"Simple", cda_rail::simulator::BrakingTimeHeuristicType::Simple}};
  std::map<std::string, cda_rail::simulator::RemainingTimeHeuristicType> const
      remaining_time_heuristic_type_map{
          {"Zero", cda_rail::simulator::RemainingTimeHeuristicType::Zero},
          {"Simple", cda_rail::simulator::RemainingTimeHeuristicType::Simple}};

  app.add_option("--dt,--timestep", dt, "Time step (dt) used in the simulation")
      ->check(CLI::PositiveNumber)
      ->capture_default_str()
      ->group("Model Parameters");
  app.add_flag("--allow-late-entry", late_entry_possible,
               "Allow late entry (delays) in the solution (default without "
               "flag is false)")
      ->group("Model Parameters");
  app.add_flag("!--speed-limit-only-on-train-front",
               limit_speed_by_leaving_edges,
               "If this flag is set, trains only respect the limit of their "
               "front position. Otherwise (by default) any edge's speed limit "
               "any part of the train is on applies.")
      ->group("Model Parameters");
  app.add_flag("!--allow-early-exit", consider_earliest_exit,
               "Allow to leave stations and the network early. By defaults "
               "trains cannot leave before the scheduled time.")
      ->group("Model Parameters");
  app.add_flag("!--time-aware-state-transitions", time_aware_state_transitions,
               "If this flag is set, use time aware state transitions to avoid "
               "unnecessary state exploration.")
      ->group("Solver Parameters");
  app.add_option("-w,--heuristic-weight", a_star_weight,
                 "Weight of the heuristic to use in the (weighted) A*. Has to "
                 "be >=1. If w is the weight, then A* is an w-approximation.")
      ->check(CLI::Range(1.0, std::numeric_limits<double>::max()))
      ->capture_default_str()
      ->group("Solver Parameters");
  app.add_option("--next-state-strategy", next_state_strategy,
                 "Next state strategy to use in the A* search. Currently "
                 "supports 'SingleEdge' and 'NextTTD'.")
      ->transform(
          CLI::CheckedTransformer(next_state_strategy_map, CLI::ignore_case))
      ->capture_default_str()
      ->group("Solver Parameters");
  app.add_option("--braking-time-heuristic-strategy",
                 braking_time_heuristic_type,
                 "Braking time heuristic strategy to use in the simulation. "
                 "Currently only supports 'Simple'")
      ->transform(CLI::CheckedTransformer(braking_time_heuristic_type_map,
                                          CLI::ignore_case))
      ->capture_default_str()
      ->group("Solver Parameters");
  app.add_option("--remaining-time-heuristic-strategy",
                 remaining_time_heuristic_type,
                 "Remaining time heuristic strategy to use in the simulation. "
                 "Currently supports 'Zero' and 'Simple'")
      ->transform(CLI::CheckedTransformer(remaining_time_heuristic_type_map,
                                          CLI::ignore_case))
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
  std::string                solution_subdirectory{};
  std::optional<std::string> parameter_identifier{};
  bool                       generate_identifier{false};

  auto* export_sol_flag =
      app.add_flag(
             "--export-solution",
             [&export_option](int) {
               export_option =
                   cda_rail::solver::GeneralExportOption::ExportSolution;
             },
             "Export the solution.")
          ->group("Export Options");
  auto* export_sol_inst_flag =
      app.add_flag(
             "--export-solution-and-instance",
             [&export_option](int) {
               export_option = cda_rail::solver::GeneralExportOption::
                   ExportSolutionWithInstance;
             },
             "Export the solution and the instance.")
          ->group("Export Options");

  app.add_option("-e,--solution-export-subdirectory", solution_subdirectory,
                 "Subdirectory to export the solution to. Will be created in "
                 "working_directory/solutions/solution_subdirectory/"
                 "instance_name-parameters.")
      ->required()
      ->group("Export Options");
  auto* parameter_identifier_option =
      app.add_option(
             "--parameter-identifier", parameter_identifier,
             "Optional identifier to distinguish different parameterizations "
             "of the same instance. Will be appended to the instance name in "
             "the export path as instance_name-parameter_identifier. If empty, "
             "no parameter identifier will be appended")
          ->capture_default_str()
          ->group("Export Options");
  auto* generate_identifier_flag =
      app.add_flag("--generate-parameter-identifier", generate_identifier,
                   "Whether to automatically generate a parameter identifier "
                   "based on the parameter settings. If set, the parameter "
                   "identifier will be generated as a concatenation of the "
                   "parameter names and values. Otherwise the identifier has "
                   "to be set explicitly if it should not remain empty.")
          ->group("Export Options");

  export_sol_flag->excludes(export_sol_inst_flag);
  generate_identifier_flag->excludes(parameter_identifier_option);

  CLI11_PARSE(app, argc, argv);

#if 0
  if (argc != 11) {
    PLOGE << "Expected 10 arguments, got " << argc - 1;
    std::exit(-1);
  }

  auto              args          = gsl::span<char*>(argv, argc);
  const std::string model_name    = args[1];
  const std::string instance_path = args[2];
  const bool        cast_instance = std::stoi(args[3]) != 0;
  cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver solver =
      cast_instance
          ? cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver(
                cda_rail::instances::GeneralPerformanceOptimizationInstance::
                    cast_from_vss_generation(
                        cda_rail::instances::VSSGenerationTimetable(
                            instance_path)))
          : cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver(
                instance_path);

  const int  dt                           = std::stoi(args[4]);
  const bool allow_delays                 = std::stoi(args[5]) != 0;
  const bool limit_speed_by_leaving_edges = std::stoi(args[6]) != 0;
  const int  next_state_strategy_int      = std::stoi(args[7]);
  const auto next_state_strategy =
      static_cast<cda_rail::solver::astar_based::NextStateStrategy>(
          next_state_strategy_int);
  const int  remaining_time_heuristic_int = std::stoi(args[8]);
  const auto remaining_time_heuristic =
      static_cast<cda_rail::simulator::RemainingTimeHeuristicType>(
          remaining_time_heuristic_int);
  const bool consider_earliest_exit = std::stoi(args[9]) != 0;

  const int timeout = std::stoi(args[10]);

  PLOGI << "The following parameters were passed:";
  PLOGI << "Model name: " << model_name;
  PLOGI << "Instance path: " << instance_path;
  PLOGI << "Time step (dt): " << dt;
  PLOGI << "Allow delays: " << (allow_delays ? "yes" : "no");
  PLOGI << "Limit speed by leaving edges: "
        << (limit_speed_by_leaving_edges ? "yes" : "no");
  switch (next_state_strategy) {
  case cda_rail::solver::astar_based::NextStateStrategy::SingleEdge:
    PLOGI << "Next state strategy: SingleEdge";
    break;
  case cda_rail::solver::astar_based::NextStateStrategy::NextTTD:
    PLOGI << "Next state strategy: NextTTD";
    break;
  }
  switch (remaining_time_heuristic) {
  case cda_rail::simulator::RemainingTimeHeuristicType::Zero:
    PLOGI << "Remaining time heuristic: Zero";
    break;
  case cda_rail::simulator::RemainingTimeHeuristicType::Simple:
    PLOGI << "Remaining time heuristic: Simple";
    break;
  }
  PLOGI << "Consider earliest exit: "
        << (consider_earliest_exit ? "yes" : "no");
  PLOGI << "Timeout: " << timeout;

  // NOLINTNEXTLINE(clang-diagnostic-unused-result)
  solver.solve({.dt                           = dt,
                .late_entry_possible          = allow_delays,
                .late_exit_possible           = allow_delays,
                .late_stop_possible           = allow_delays,
                .limit_speed_by_leaving_edges = limit_speed_by_leaving_edges},
               {.remaining_time_heuristic_type = remaining_time_heuristic,
                .next_state_strategy           = next_state_strategy,
                .consider_earliest_exit        = consider_earliest_exit},
               {.name = model_name}, timeout, true, true);
#endif
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,bugprone-exception-escape)
