#include "plog/Init.h"
#include "plog/Logger.h"
#include "plog/Severity.h"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "probleminstances/VSSGenerationTimetable.hpp"
#include "simulator/GreedyHeuristic.hpp"
#include "solver/astar-based/GenPOMovingBlockAStarSolver.hpp"

#include <cstdlib>
#include <gsl/span>
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
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,bugprone-exception-escape)
