#include "Definitions.hpp"
#include "plog/Init.h"
#include "plog/Logger.h"
#include "plog/Severity.h"
#include "solver/mip-based/GenPOMovingBlockMIPSolver.hpp"

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

  if (argc != 9) {
    PLOGE << "Expected 8 arguments, got " << argc - 1;
    std::exit(-1);
  }

  auto              args          = gsl::span<char*>(argv, argc);
  const std::string model_name    = args[1];
  const std::string instance_path = args[2];
  auto              solver =
      cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver(instance_path);

  const bool use_lazy          = std::stoi(args[3]) != 0;
  const bool reverse_headways  = std::stoi(args[4]) != 0;
  const bool higher_velocities = std::stoi(args[5]) != 0;
  const int  lazy_strategy_int = std::stoi(args[6]);
  const auto lazy_strategy =
      static_cast<cda_rail::solver::mip_based::LazyConstraintSelectionStrategy>(
          lazy_strategy_int);
  const int  train_strategy_int = std::stoi(args[7]);
  const auto train_strategy =
      static_cast<cda_rail::solver::mip_based::LazyTrainSelectionStrategy>(
          train_strategy_int);

  const int timeout = std::stoi(args[8]);

  PLOGI << "The following parameters were passed:";
  PLOGI << "Model name: " << model_name;
  PLOGI << "Instance path: " << instance_path;
  if (use_lazy) {
    PLOGI << "Using lazy constraints";
  }
  if (reverse_headways) {
    PLOGI << "Including reverse headways";
  }
  if (higher_velocities) {
    PLOGI << "Including higher velocities in edge expressions";
  }
  switch (lazy_strategy) {
  case cda_rail::solver::mip_based::LazyConstraintSelectionStrategy::
      OnlyViolated:
    PLOGI << "Lazy constraint selection strategy: OnlyViolated";
    break;
  case cda_rail::solver::mip_based::LazyConstraintSelectionStrategy::
      OnlyFirstFound:
    PLOGI << "Lazy constraint selection strategy: OnlyFirstFound";
    break;
  case cda_rail::solver::mip_based::LazyConstraintSelectionStrategy::AllChecked:
    PLOGI << "Lazy constraint selection strategy: AllChecked";
    break;
  }
  switch (train_strategy) {
  case cda_rail::solver::mip_based::LazyTrainSelectionStrategy::OnlyAdjacent:
    PLOGI << "Lazy train selection strategy: OnlyAdjacent";
    break;
  case cda_rail::solver::mip_based::LazyTrainSelectionStrategy::All:
    PLOGI << "Lazy train selection strategy: All";
    break;
  }
  PLOGI << "Timeout: " << timeout;

  // NOLINTNEXTLINE(clang-diagnostic-unused-result)
  solver.solve({false, 5.55, cda_rail::VelocityRefinementStrategy::None},
               {use_lazy, reverse_headways, higher_velocities, lazy_strategy,
                train_strategy},
               {}, timeout, true);
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,bugprone-exception-escape)
