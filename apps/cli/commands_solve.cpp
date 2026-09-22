#include "CLI/CLI.hpp"
#include "CommandInterpreter.hpp"
#include "Session.hpp"
#include "options_common.hpp"
#include "options_mb_astar.hpp"
#include "options_mb_mip.hpp"
#include "options_vss_mip.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "solver/astar-based/GenPOMovingBlockAStarSolver.hpp"
#include "solver/mip-based/GenPOMovingBlockMIPSolver.hpp"
#include "solver/mip-based/VSSGenTimetableSolver.hpp"

#include <memory>
#include <plog/Log.h>

// The solvers run on the instance the session holds, including its unsaved
// edits. Only the instance options of the standalone apps are therefore
// missing from the subcommands below; every model, solver and export option is
// the very same one, defined once in the options_*.hpp headers.

// The reinterpret_cast warnings are false positives stemming from the plog
// macros.
// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast)

namespace {
using cda_rail::cli::Session;

// The settings of the three solver subcommands are held by a shared_ptr
// because the callback of a subcommand runs after the function that added it
// has returned, so it cannot capture a local by reference.
void add_mb_mip_command(CLI::App& solve_cmd, Session& session) {
  auto* mb_mip_cmd = solve_cmd.add_subcommand(
      "mb-mip", "Route the trains under moving block control using a MIP");
  auto settings = std::make_shared<cda_rail::cli::MbMipSettings>();
  cda_rail::cli::add_mb_mip_options(*mb_mip_cmd, *settings);

  mb_mip_cmd->callback([&session, mb_mip_cmd, settings]() {
    using namespace cda_rail::cli;
    const auto working_directory = session.working_directory().string();
    finalize_mb_mip_settings(*mb_mip_cmd, *settings);
    log_mb_mip_settings(*settings, working_directory);

    cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(
        session.instance());
    PLOGI << "Solving...";
    const auto solution = solver.solve(
        mb_mip_model_detail(*settings), mb_mip_solver_strategy(*settings),
        mb_mip_solution_settings(*settings, working_directory),
        settings->solving.time_limit, settings->solving.debug_output, true);
    log_solution_status(solution.get_status(), solution.get_obj());
  });
}

void add_mb_astar_command(CLI::App& solve_cmd, Session& session) {
  auto* mb_astar_cmd = solve_cmd.add_subcommand(
      "mb-astar",
      "Route the trains under moving block control using an A* search");
  auto settings = std::make_shared<cda_rail::cli::MbAStarSettings>();
  cda_rail::cli::add_mb_astar_options(*mb_astar_cmd, *settings);

  mb_astar_cmd->callback([&session, settings]() {
    using namespace cda_rail::cli;
    const auto working_directory = session.working_directory().string();
    finalize_mb_astar_settings(*settings);
    log_mb_astar_settings(*settings, working_directory);

    cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver solver(
        session.instance());
    PLOGI << "Solving...";
    const auto solution = solver.solve(
        mb_astar_model_detail(*settings), mb_astar_solver_strategy(*settings),
        general_solution_settings(settings->exporting, working_directory),
        settings->solving.time_limit, settings->solving.debug_output, true);
    log_solution_status(solution.get_status(), solution.get_obj());
  });
}

void add_vss_mip_command(CLI::App& solve_cmd, Session& session) {
  auto* vss_mip_cmd = solve_cmd.add_subcommand(
      "vss-mip", "Generate a minimal VSS layout for the timetable using a MIP");
  auto settings = std::make_shared<cda_rail::cli::VssMipSettings>();
  cda_rail::cli::add_vss_mip_options(*vss_mip_cmd, *settings);

  vss_mip_cmd->callback([&session, settings]() {
    using namespace cda_rail::cli;
    namespace instances = cda_rail::instances;
    namespace mip_based = cda_rail::solver::mip_based;

    const auto working_directory = session.working_directory().string();
    finalize_vss_mip_settings(*settings);
    log_vss_mip_settings(*settings, working_directory);

    const auto model_settings  = vss_mip_model_settings(*settings);
    const auto solver_strategy = vss_mip_solver_strategy(*settings);
    const auto solution_settings =
        vss_mip_solution_settings(*settings, working_directory);

    PLOGI << "Solving...";
    const auto solution =
        [&]() -> instances::SolVSSGeneralPerformanceOptimizationInstance {
      if (settings->use_moving_block_information()) {
        instances::SolGeneralPerformanceOptimizationInstance
            moving_block_solution(session.const_instance());
        moving_block_solution.load_solution(
            settings->moving_block_working_directory.value_or(
                working_directory),
            settings->moving_block_solution_subdirectory,
            settings->moving_block_parameter_identifier);

        mip_based::VSSGenTimetableSolverWithMovingBlockInformation solver(
            moving_block_solution);
        return solver.solve(vss_mip_model_detail_mb(*settings), model_settings,
                            solver_strategy, solution_settings,
                            settings->solving.time_limit,
                            settings->solving.debug_output, true);
      }

      mip_based::VSSGenTimetableSolver solver(session.instance());
      return solver.solve(vss_mip_model_detail(*settings), model_settings,
                          solver_strategy, solution_settings,
                          settings->solving.time_limit,
                          settings->solving.debug_output, true);
    }();
    log_solution_status(solution.get_status(), solution.get_obj());
  });
}
} // namespace

void cda_rail::cli::add_solve_commands(CLI::App& app, Session& session) {
  auto* solve_cmd = app.add_subcommand(
      "solve", "Run one of the solvers on the loaded instance");
  solve_cmd->require_subcommand(1);

  add_mb_mip_command(*solve_cmd, session);
  add_mb_astar_command(*solve_cmd, session);
  add_vss_mip_command(*solve_cmd, session);
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)
