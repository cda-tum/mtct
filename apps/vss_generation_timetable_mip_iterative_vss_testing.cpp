#include "Definitions.hpp"
#include "VSSModel.hpp"
#include "solver/mip-based/VSSGenTimetableSolver.hpp"

#include <gsl/span>
#include <plog/Appenders/ColorConsoleAppender.h>
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Initializers/ConsoleInitializer.h>
#include <plog/Log.h>

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay)

int main(int argc, char** argv) {
  // Only log to console using std::cerr and std::cout respectively unless
  // initialized differently
  if (plog::get() == nullptr) {
    static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
    plog::init(plog::debug, &console_appender);
  }

  if (argc < 12 || argc > 13) {
    PLOGE << "Expected 11 or 12 arguments, got " << argc - 1;
    std::exit(-1);
  }

  auto args = gsl::span<char*>(argv, argc);

  const std::string                                  model_name    = args[1];
  const std::string                                  instance_path = args[2];
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(instance_path);

  PLOGI << "Instance " << model_name << " loaded at " << instance_path;

  const int  delta_t                 = std::stoi(args[3]);
  const bool fix_routes              = std::stoi(args[4]) != 0;
  const bool include_train_dynamics  = std::stoi(args[5]) != 0;
  const bool include_braking_curves  = std::stoi(args[6]) != 0;
  const bool use_pwl                 = std::stoi(args[7]) != 0;
  const bool use_schedule_cuts       = std::stoi(args[8]) != 0;
  const bool iterate_vss             = std::stoi(args[9]) != 0;
  const int  optimality_strategy_int = std::stoi(args[10]);
  const auto optimality_strategy =
      static_cast<cda_rail::OptimalityStrategy>(optimality_strategy_int);
  const int         timeout     = std::stoi(args[11]);
  const std::string output_path = (argc == 13 ? args[12] : "");

  PLOGI << "The following parameters were passed to the toolkit:";
  PLOGI << "   delta_t: " << delta_t;
  if (fix_routes) {
    PLOGI << "   routes are fixed";
  }
  if (include_train_dynamics) {
    PLOGI << "   acceleration and deceleration are included";
  }
  if (include_braking_curves) {
    PLOGI << "   braking distance is included";
  }
  if (use_pwl) {
    PLOGI << "   piecewise linear functions are used";
  }
  if (use_schedule_cuts) {
    PLOGI << "   schedule cuts are used";
  }
  if (iterate_vss) {
    PLOGI << "   VSS is iterated";
  }
  if (optimality_strategy == cda_rail::OptimalityStrategy::Optimal) {
    PLOGI << "   optimality strategy: optimal";
  } else if (optimality_strategy == cda_rail::OptimalityStrategy::TradeOff) {
    PLOGI << "   optimality strategy: trade-off";
  } else if (optimality_strategy == cda_rail::OptimalityStrategy::Feasible) {
    PLOGI << "   optimality strategy: feasible";
  } else {
    PLOGI << "   optimality strategy: unknown";
  }
  PLOGI << "   timeout: " << timeout << "s";

  const std::string file_name =
      model_name + "_" + std::to_string(delta_t) + "_" +
      std::to_string(static_cast<int>(fix_routes)) + "_" +
      std::to_string(static_cast<int>(include_train_dynamics)) + "_" +
      std::to_string(static_cast<int>(include_braking_curves)) + "_" +
      std::to_string(static_cast<int>(use_pwl)) + "_" +
      std::to_string(static_cast<int>(use_schedule_cuts)) + "_" +
      std::to_string(static_cast<int>(iterate_vss)) + "_" +
      std::to_string(static_cast<int>(optimality_strategy_int)) + "_" +
      std::to_string(timeout);

  cda_rail::vss::Model vss_model(cda_rail::vss::ModelType::Continuous);

  // NOLINTNEXTLINE(clang-diagnostic-unused-result)
  solver.solve(
      {delta_t, fix_routes, include_train_dynamics, include_braking_curves},
      {vss_model, use_pwl, use_schedule_cuts},
      {iterate_vss, optimality_strategy},
      {false, cda_rail::ExportOption::ExportSolution, file_name, output_path},
      timeout, true);
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay)
