#include "CLI/CLI.hpp"
#include "options_common.hpp"
#include "options_vss_mip.hpp"
#include "plog/Init.h"
#include "plog/Logger.h"
#include "plog/Severity.h"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "solver/mip-based/VSSGenTimetableSolver.hpp"

#include <plog/Appenders/ColorConsoleAppender.h>
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Log.h>

// This app covers both VSS generation solvers, namely VSSGenTimetableSolver
// and VSSGenTimetableSolverWithMovingBlockInformation. The latter is used as
// soon as -m (--moving-block-solution-subdirectory) is given; in that case a
// previously obtained moving block solution is loaded in addition to the
// instance itself and the routes of the instance are superseded by the routes
// of that solution.
//
// Every option of this app is defined in apps/cli/options_vss_mip.hpp, so that
// the 'solve vss-mip' subcommand of rail_cli offers the very same settings and
// the two cannot drift apart.

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,bugprone-exception-escape)

int main(int argc, char** argv) {
  // Only log to console using std::cerr and std::cout respectively unless
  // initialized differently
  if (plog::get() == nullptr) {
    static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
    plog::init(plog::debug, &console_appender);
  }

  CLI::App app{"VSS Generation Optimization using a MIP"};
  argv = app.ensure_utf8(argv);
  app.option_defaults()->multi_option_policy(CLI::MultiOptionPolicy::Throw);

  cda_rail::cli::InstanceSettings instance_settings;
  cda_rail::cli::VssMipSettings   settings;
  cda_rail::cli::add_instance_options(app, instance_settings);
  cda_rail::cli::add_vss_mip_options(app, settings);

  CLI11_PARSE(app, argc, argv);

  cda_rail::cli::finalize_vss_mip_settings(settings);

  // ----------------------
  // PRINT SETTINGS
  // ----------------------

  PLOGD << "The following parameters were passed:";
  cda_rail::cli::log_instance_settings(instance_settings);
  cda_rail::cli::log_vss_mip_settings(settings,
                                      instance_settings.working_directory);

  const auto model_settings  = cda_rail::cli::vss_mip_model_settings(settings);
  const auto solver_strategy = cda_rail::cli::vss_mip_solver_strategy(settings);
  const auto solution_settings = cda_rail::cli::vss_mip_solution_settings(
      settings, instance_settings.working_directory);

  // --------------
  // SOLVE
  // --------------

  PLOGI << "Solving...";

  auto const solution = [&]()
      -> cda_rail::instances::SolVSSGeneralPerformanceOptimizationInstance {
    if (settings.use_moving_block_information()) {
      const cda_rail::instances::GeneralPerformanceOptimizationInstance
          instance(instance_settings.instance_name,
                   instance_settings.instance_subdirectory,
                   instance_settings.working_directory);
      cda_rail::instances::SolGeneralPerformanceOptimizationInstance
          moving_block_solution(instance);
      moving_block_solution.load_solution(
          settings.moving_block_working_directory.value_or(
              instance_settings.working_directory),
          settings.moving_block_solution_subdirectory,
          settings.moving_block_parameter_identifier);

      cda_rail::solver::mip_based::
          VSSGenTimetableSolverWithMovingBlockInformation solver(
              moving_block_solution);

      return solver.solve(cda_rail::cli::vss_mip_model_detail_mb(settings),
                          model_settings, solver_strategy, solution_settings,
                          settings.solving.time_limit,
                          settings.solving.debug_output, true);
    }

    cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
        instance_settings.instance_name,
        instance_settings.instance_subdirectory,
        instance_settings.working_directory);

    return solver.solve(cda_rail::cli::vss_mip_model_detail(settings),
                        model_settings, solver_strategy, solution_settings,
                        settings.solving.time_limit,
                        settings.solving.debug_output, true);
  }();

  cda_rail::cli::log_solution_status(solution.get_status(), solution.get_obj());
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,bugprone-exception-escape)
