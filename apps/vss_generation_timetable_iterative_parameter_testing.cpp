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

  if (argc < 13 || argc > 15) {
    PLOGE << "Expected 12 or 13 or 14 arguments, got " << argc - 1;
    std::exit(-1);
  }

  auto args = gsl::span<char*>(argv, argc);

  const std::string                                  model_name    = args[1];
  const std::string                                  instance_path = args[2];
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(instance_path);

  PLOGI << "Instance " << model_name << " loaded at " << instance_path;

  const int  delta_t                 = std::stoi(args[3]);
  const bool fix_routes              = std::stoi(args[4]) != 0;
  const bool include_braking_curves  = std::stoi(args[5]) != 0;
  const bool iterate_vss             = std::stoi(args[6]) != 0;
  const int  optimality_strategy_int = std::stoi(args[7]);
  const auto optimality_strategy =
      static_cast<cda_rail::OptimalityStrategy>(optimality_strategy_int);
  const int  update_strategy_int = std::stoi(args[8]);
  const auto update_strategy =
      static_cast<cda_rail::solver::mip_based::UpdateStrategy>(
          update_strategy_int);
  const double      initial_vss  = std::stod(args[9]);
  const double      update_value = std::stod(args[10]);
  const bool        include_cuts = std::stoi(args[11]) != 0;
  const int         timeout      = std::stoi(args[12]);
  const std::string output_path  = (argc >= 14 ? args[13] : "");
  const std::string file_name =
      (argc >= 15
           ? args[14]
           : model_name + "_" + std::to_string(delta_t) + "_" +
                 std::to_string(static_cast<int>(fix_routes)) + "_" +
                 std::to_string(static_cast<int>(include_braking_curves)) +
                 "_" + std::to_string(static_cast<int>(iterate_vss)) + "_" +
                 std::to_string(optimality_strategy_int) + "_" +
                 std::to_string(update_strategy_int) + "_" +
                 std::to_string(initial_vss) + "_" +
                 std::to_string(update_value) + "_" +
                 std::to_string(static_cast<int>(include_cuts)) + "_" +
                 std::to_string(timeout));

  PLOGI << "The following parameters were passed to the toolkit:";
  PLOGI << "   delta_t: " << delta_t;
  if (fix_routes) {
    PLOGI << "   routes are fixed";
  }
  if (include_braking_curves) {
    PLOGI << "   braking distance is included";
  }
  if (iterate_vss) {
    PLOGI << "   VSS is iterated to optimality";
    PLOGI << "      using initial value " << initial_vss;
    PLOGI << "      and update value " << update_value;
    if (update_strategy == cda_rail::solver::mip_based::UpdateStrategy::Fixed) {
      PLOGI << "      with fixed update strategy";
    } else if (update_strategy ==
               cda_rail::solver::mip_based::UpdateStrategy::Relative) {
      PLOGI << "      with relative update strategy";
    } else {
      PLOGI << "      with unknown update strategy";
    }
    if (include_cuts) {
      PLOGI << "      and cuts are used";
    }
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
  PLOGI << "   output path: " << output_path;
  PLOGI << "   file name: " << file_name;

  cda_rail::vss::Model vss_model(cda_rail::vss::ModelType::Continuous);

  solver.solve(
      {delta_t, fix_routes, true, include_braking_curves}, {vss_model},
      {iterate_vss, optimality_strategy, update_strategy, initial_vss,
       update_value, include_cuts},
      {false, cda_rail::ExportOption::ExportSolution, file_name, output_path},
      timeout, true);
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay)
