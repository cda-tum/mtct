#include "Definitions.hpp"
#include "solver/mip-based/GenPOMovingBlockMIPSolver.hpp"

#include <filesystem>
#include <gsl/span>
#include <iostream>
#include <plog/Appenders/ColorConsoleAppender.h>
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Initializers/ConsoleInitializer.h>
#include <plog/Log.h>

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,bugprone-exception-escape)

int main(int argc, char** argv) {
  // Only log to console using std::cerr and std::cout respectively unless
  // initialized differently
  if (plog::get() == nullptr) {
    static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
    plog::init(plog::debug, &console_appender);
  }

  // For every folder in example-networks
  // Get list of folders in example-networks
  std::string path = "./test/example-networks/";
  for (const auto& instance_path : std::filesystem::directory_iterator(path)) {
    PLOGD << "Processing instance: " << instance_path.path().string();

    // Get last folder name from path
    std::string instance_name = instance_path.path().filename().string();

    const auto instance_before_parse =
        cda_rail::instances::VSSGenerationTimetable(instance_path);
    const auto instance =
        cda_rail::instances::GeneralPerformanceOptimizationInstance::
            cast_from_vss_generation(instance_before_parse);
    cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(instance);
    solver.solve(
        {false, 3, cda_rail::VelocityRefinementStrategy::None, false, false},
        {false},
        {cda_rail::ExportOption::ExportSolutionWithInstance, instance_name,
         "./test/example-networks-mb-solutions"},
        -1, true);
  }
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,bugprone-exception-escape)
