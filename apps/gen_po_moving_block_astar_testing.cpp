#include "CLI/CLI.hpp"
#include "plog/Init.h"
#include "plog/Logger.h"
#include "plog/Severity.h"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "simulator/GreedyHeuristic.hpp"
#include "solver/astar-based/GenPOMovingBlockAStarSolver.hpp"

#include <cstdlib>
#include <iomanip>
#include <map>
#include <plog/Appenders/ColorConsoleAppender.h>
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Log.h>
#include <sstream>
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

  int  time_limit{-1};
  bool debug_output{false};

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

  auto get_key_by_value = [](const auto& map, const auto& value) {
    for (const auto& [k, v] : map) {
      if (v == value) {
        return k;
      }
    }
    return std::string{"UNKNOWN"};
  };

  app.add_option("-c,--dt,--timestep", dt,
                 "Time step (dt) used in the simulation")
      ->check(CLI::PositiveNumber)
      ->capture_default_str()
      ->group("Model Parameters");
  app.add_flag("-l,--allow-late-entry", late_entry_possible,
               "Allow late entry (delays) in the solution (default without "
               "flag is false)")
      ->group("Model Parameters");
  app.add_flag("-f,!--speed-limit-only-on-train-front",
               limit_speed_by_leaving_edges,
               "If this flag is set, trains only respect the limit of their "
               "front position. Otherwise (by default) any edge's speed limit "
               "any part of the train is on applies.")
      ->group("Model Parameters");
  app.add_flag("-y,!--allow-early-exit", consider_earliest_exit,
               "Allow to leave stations and the network early. By defaults "
               "trains cannot leave before the scheduled time.")
      ->group("Model Parameters");
  app.add_flag("-a,!--time-aware-state-transitions",
               time_aware_state_transitions,
               "If this flag is set, use time aware state transitions to avoid "
               "unnecessary state exploration.")
      ->group("Solver Parameters");
  app.add_option("-w,--heuristic-weight", a_star_weight,
                 "Weight of the heuristic to use in the (weighted) A*. Has to "
                 "be >=1. If w is the weight, then A* is an w-approximation.")
      ->check(CLI::Range(1.0, std::numeric_limits<double>::max()))
      ->capture_default_str()
      ->group("Solver Parameters");
  app.add_option("-x,--next-state-strategy", next_state_strategy,
                 "Next state strategy to use in the A* search. Currently "
                 "supports 'SingleEdge' and 'NextTTD'.")
      ->transform(
          CLI::CheckedTransformer(next_state_strategy_map, CLI::ignore_case))
      ->capture_default_str()
      ->group("Solver Parameters");
  app.add_option("-b,--braking-time-heuristic-strategy",
                 braking_time_heuristic_type,
                 "Braking time heuristic strategy to use in the simulation. "
                 "Currently only supports 'Simple'")
      ->transform(CLI::CheckedTransformer(braking_time_heuristic_type_map,
                                          CLI::ignore_case))
      ->capture_default_str()
      ->group("Solver Parameters");
  app.add_option("-r,--remaining-time-heuristic-strategy",
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

  auto* solution_subdir_opt =
      app.add_option(
             "-e,--solution-export-subdirectory", solution_subdirectory,
             "Subdirectory to export the solution to. Will be created in "
             "working_directory/solutions/solution_subdirectory/"
             "instance_name-parameters.")
          ->group("Export Options");

  export_sol_flag->needs(solution_subdir_opt);
  export_sol_inst_flag->needs(solution_subdir_opt);

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

  export_sol_flag->excludes(export_sol_inst_flag);
  generate_identifier_flag->excludes(parameter_identifier_option);

  CLI11_PARSE(app, argc, argv);

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
        {format_double(dt), "_", bool_to_str(late_entry_possible), "_",
         bool_to_str(limit_speed_by_leaving_edges), "_",
         bool_to_str(consider_earliest_exit), "_",
         bool_to_str(time_aware_state_transitions), "_",
         format_double(a_star_weight), "_",
         get_key_by_value(next_state_strategy_map, next_state_strategy), "_",
         get_key_by_value(braking_time_heuristic_type_map,
                          braking_time_heuristic_type),
         "_",
         get_key_by_value(remaining_time_heuristic_type_map,
                          remaining_time_heuristic_type),
         "_", std::to_string(time_limit)});
  }

  PLOGD << "The following parameters were passed:";
  PLOGD << "Instance Settings";
  PLOGD << "  Instance name: " << instance_name;
  PLOGD << "  Instance subdirectory: " << instance_subdirectory;
  PLOGD << "  Working directory: " << working_directory;
  PLOGD << "Solver Settings";
  PLOGD << "  Time step (dt): " << dt;
  PLOGD << "  Allow late entry: " << (late_entry_possible ? "yes" : "no");
  PLOGD << "  Limit speed by leaving edges: "
        << (limit_speed_by_leaving_edges ? "yes" : "no");
  PLOGD << "  Consider earliest exit: "
        << (consider_earliest_exit ? "yes" : "no");
  PLOGD << "  Time aware state transitions: "
        << (time_aware_state_transitions ? "yes" : "no");
  PLOGD << "  Heuristic weight (w): " << a_star_weight;
  PLOGD << "  Next state strategy: "
        << get_key_by_value(next_state_strategy_map, next_state_strategy);
  PLOGD << "  Braking time heuristic strategy: "
        << get_key_by_value(braking_time_heuristic_type_map,
                            braking_time_heuristic_type);
  PLOGD << "  Remaining time heuristic strategy: "
        << get_key_by_value(remaining_time_heuristic_type_map,
                            remaining_time_heuristic_type);
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
  PLOGD << "  Solution export subdirectory: " << solution_subdirectory;
  PLOGD << "  Parameter identifier: " << parameter_identifier.value_or("-")
        << (generate_identifier ? " (generated)" : "");

  cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver solver(
      instance_name, instance_subdirectory, working_directory);

  // --------------
  // SOLVE
  // --------------

  PLOGI << "Solving...";

  // NOLINTNEXTLINE(clang-diagnostic-unused-result)
  auto const solution = solver.solve(
      {.dt                           = dt,
       .late_entry_possible          = late_entry_possible,
       .limit_speed_by_leaving_edges = limit_speed_by_leaving_edges},
      {.braking_time_heuristic_type   = braking_time_heuristic_type,
       .remaining_time_heuristic_type = remaining_time_heuristic_type,
       .next_state_strategy           = next_state_strategy,
       .consider_earliest_exit        = consider_earliest_exit,
       .time_aware_state_transitions  = time_aware_state_transitions,
       .a_star_weight                 = a_star_weight},
      {.export_option         = export_option,
       .working_directory     = working_directory,
       .solution_subdirectory = solution_subdirectory,
       .parameter_identifier  = parameter_identifier},
      time_limit, debug_output, true);

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
