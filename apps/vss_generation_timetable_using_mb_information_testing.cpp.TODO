#include "Definitions.hpp"
#include "VSSModel.hpp"
#include "plog/Init.h"
#include "plog/Logger.h"
#include "plog/Severity.h"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "solver/mip-based/VSSGenTimetableSolver.hpp"

#include <cstdlib>
#include <filesystem>
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

  if (argc < 12 || argc > 13) {
    PLOGE << "Expected 11 or 12 arguments, got " << argc - 1;
    std::exit(-1);
  }

  auto args = gsl::span<char*>(argv, argc);

  const std::string model_name                 = args[1];
  const std::string instance_path              = args[2];
  const int         delta_t                    = std::stoi(args[3]);
  const bool        use_mb_information         = std::stoi(args[4]) != 0;
  const bool        fix_stop_positions         = std::stoi(args[5]) != 0;
  const bool        fix_exact_positions        = std::stoi(args[6]) != 0;
  const bool        fix_exact_velocities       = std::stoi(args[7]) != 0;
  const bool        hint_approximate_positions = std::stoi(args[8]) != 0;
  const bool        fix_order_on_edges         = std::stoi(args[9]) != 0;
  const bool        use_pwl                    = std::stoi(args[10]) != 0;
  const int         timeout                    = std::stoi(args[11]);
  const std::string output_path                = (argc == 13 ? args[12] : "");

  PLOGI << "Solving instance " << model_name
        << " with the following parameters:";
  PLOGI << "   delta_t: " << delta_t;
  if (use_mb_information) {
    PLOGI << "   moving block information is used";
    if (fix_stop_positions) {
      PLOGI << "   stop positions are fixed";
    }
    if (fix_exact_positions) {
      PLOGI << "   exact positions are fixed";
    }
    if (fix_exact_velocities) {
      PLOGI << "   exact velocities are fixed";
    }
    if (hint_approximate_positions) {
      PLOGI << "   approximate positions are hinted";
    }
    if (fix_order_on_edges) {
      PLOGI << "   order on edges is fixed";
    }
  } else {
    PLOGI << "   moving block information is not used";
  }
  if (use_pwl) {
    PLOGI << "   piecewise linear functions are used";
  }
  PLOGI << "   timeout: " << timeout << "s";

  const std::string file_name =
      model_name + "_" + std::to_string(delta_t) + "_" +
      std::to_string(static_cast<int>(use_mb_information)) + "_" +
      std::to_string(static_cast<int>(fix_stop_positions)) + "_" +
      std::to_string(static_cast<int>(fix_exact_positions)) + "_" +
      std::to_string(static_cast<int>(fix_exact_velocities)) + "_" +
      std::to_string(static_cast<int>(hint_approximate_positions)) + "_" +
      std::to_string(static_cast<int>(fix_order_on_edges)) + "_" +
      std::to_string(static_cast<int>(use_pwl)) + "_" + std::to_string(timeout);

  if (use_mb_information) {
    cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
        solver(instance_path);
    PLOGI << "Instance " << model_name << " loaded at " << instance_path;
    // NOLINTNEXTLINE(clang-diagnostic-unused-result)
    solver.solve({delta_t, use_mb_information, true, fix_stop_positions,
                  fix_exact_positions, fix_exact_positions,
                  hint_approximate_positions, fix_order_on_edges},
                 {cda_rail::vss::Model(), use_pwl},
                 {false, cda_rail::OptimalityStrategy::Optimal},
                 {false, cda_rail::ExportOption::ExportSolutionWithInstance,
                  file_name, output_path},
                 timeout, true);
  } else {
    const std::filesystem::path vss_instance_path =
        std::filesystem::path(instance_path) / "instance";
    const cda_rail::instances::GeneralPerformanceOptimizationInstance
               vss_instance_before_parse(vss_instance_path);
    const auto vss_instance =
        vss_instance_before_parse.cast_to_vss_generation(true);
    cda_rail::solver::mip_based::VSSGenTimetableSolver solver(vss_instance);
    PLOGI << "Instance " << model_name << " loaded at " << vss_instance_path;

    // NOLINTNEXTLINE(clang-diagnostic-unused-result)
    solver.solve({delta_t, false, true, true},
                 {cda_rail::vss::Model(), use_pwl},
                 {false, cda_rail::OptimalityStrategy::Optimal},
                 {false, cda_rail::ExportOption::ExportSolutionWithInstance,
                  file_name, output_path},
                 timeout, true);
  }
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,bugprone-exception-escape)
