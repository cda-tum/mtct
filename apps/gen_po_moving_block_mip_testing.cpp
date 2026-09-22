#include "CLI/CLI.hpp"
#include "options_common.hpp"
#include "options_mb_mip.hpp"
#include "plog/Init.h"
#include "plog/Logger.h"
#include "plog/Severity.h"
#include "solver/mip-based/GenPOMovingBlockMIPSolver.hpp"

#include <plog/Appenders/ColorConsoleAppender.h>
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Log.h>

// Every option of this app is defined in apps/cli/options_mb_mip.hpp, so that
// the 'solve mb-mip' subcommand of rail_cli offers the very same settings and
// the two cannot drift apart.

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,bugprone-exception-escape)

int main(int argc, char** argv) {
  // Only log to console using std::cerr and std::cout respectively unless
  // initialized differently
  if (plog::get() == nullptr) {
    static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
    plog::init(plog::debug, &console_appender);
  }

  CLI::App app{"Moving Block Optimization using a MIP"};
  argv = app.ensure_utf8(argv);
  app.option_defaults()->multi_option_policy(CLI::MultiOptionPolicy::Throw);

  cda_rail::cli::InstanceSettings instance_settings;
  cda_rail::cli::MbMipSettings    settings;
  cda_rail::cli::add_instance_options(app, instance_settings);
  cda_rail::cli::add_mb_mip_options(app, settings);

  CLI11_PARSE(app, argc, argv);

  cda_rail::cli::finalize_mb_mip_settings(app, settings);

  // ----------------------
  // PRINT SETTINGS
  // ----------------------

  PLOGD << "The following parameters were passed:";
  cda_rail::cli::log_instance_settings(instance_settings);
  cda_rail::cli::log_mb_mip_settings(settings,
                                     instance_settings.working_directory);

  cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(
      instance_settings.instance_name, instance_settings.instance_subdirectory,
      instance_settings.working_directory);

  // --------------
  // SOLVE
  // --------------

  PLOGI << "Solving...";

  // NOLINTNEXTLINE(clang-diagnostic-unused-result)
  auto const solution = solver.solve(
      cda_rail::cli::mb_mip_model_detail(settings),
      cda_rail::cli::mb_mip_solver_strategy(settings),
      cda_rail::cli::mb_mip_solution_settings(
          settings, instance_settings.working_directory),
      settings.solving.time_limit, settings.solving.debug_output, true);

  cda_rail::cli::log_solution_status(solution.get_status(), solution.get_obj());
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,bugprone-exception-escape)
