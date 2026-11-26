#include "CLI/CLI.hpp"
#include "CommandInterpreter.hpp"
#include "Formatting.hpp"
#include "Session.hpp"
#include "nlohmann/json.hpp"
#include "nlohmann/json_fwd.hpp"
#include "options_common.hpp"
#include "options_mb_astar.hpp"
#include "options_mb_mip.hpp"
#include "options_vss_mip.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "solver/GeneralSolver.hpp"
#include "solver/astar-based/GenPOMovingBlockAStarSolver.hpp"
#include "solver/mip-based/GenPOMovingBlockMIPSolver.hpp"
#include "solver/mip-based/VSSGenTimetableSolver.hpp"

#include <iostream>
#include <memory>
#include <optional>
#include <plog/Log.h>
#include <utility>

// The solvers run on the instance the session holds, including its unsaved
// edits. Only the instance options of the standalone apps are therefore
// missing from the subcommands below; every model, solver and export option is
// the very same one, defined once in the options_*.hpp headers.

// The reinterpret_cast warnings are false positives stemming from the plog
// macros.
// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast,readability-inconsistent-ifelse-braces)

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

  mb_mip_cmd->callback([&session, mb_mip_cmd, settings] {
    using namespace cda_rail::cli;
    const auto working_directory = session.working_directory().string();
    finalize_mb_mip_settings(*mb_mip_cmd, *settings);
    log_mb_mip_settings(*settings, working_directory);

    cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(
        session.instance());
    const auto solution_settings =
        mb_mip_solution_settings(*settings, working_directory);
    PLOGI << "Solving...";
    const auto solution = solver.solve(
        mb_mip_model_detail(*settings), mb_mip_solver_strategy(*settings),
        solution_settings, settings->solving.time_limit,
        settings->solving.debug_output, true);
    log_solution_status(solution.get_status(), solution.get_obj());
    log_solution_evaluation(solution);
    // The moving block MIP has no time step of its own
    session.set_solution(
        std::make_unique<
            cda_rail::instances::SolGeneralPerformanceOptimizationInstance>(
            solution),
        std::nullopt,
        static_cast<const cda_rail::solver::GeneralSolutionSettings&>(
            solution_settings),
        solver.get_solver_data(),
        hash_identifier(generate_mb_mip_identifier(*settings)));
  });
}

void add_mb_astar_command(CLI::App& solve_cmd, Session& session) {
  auto* mb_astar_cmd = solve_cmd.add_subcommand(
      "mb-astar",
      "Route the trains under moving block control using an A* search");
  auto settings = std::make_shared<cda_rail::cli::MbAStarSettings>();
  cda_rail::cli::add_mb_astar_options(*mb_astar_cmd, *settings);

  mb_astar_cmd->callback([&session, settings] {
    using namespace cda_rail::cli;
    const auto working_directory = session.working_directory().string();
    finalize_mb_astar_settings(*settings);
    log_mb_astar_settings(*settings, working_directory);

    cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver solver(
        session.instance());
    const auto solution_settings =
        general_solution_settings(settings->exporting, working_directory);
    PLOGI << "Solving...";
    const auto solution = solver.solve(
        mb_astar_model_detail(*settings), mb_astar_solver_strategy(*settings),
        solution_settings, settings->solving.time_limit,
        settings->solving.debug_output, true);
    log_solution_status(solution.get_status(), solution.get_obj());
    log_solution_evaluation(solution);
    session.set_solution(
        std::make_unique<
            cda_rail::instances::SolGeneralPerformanceOptimizationInstance>(
            solution),
        settings->dt, solution_settings, solver.get_solver_data(),
        hash_identifier(generate_mb_astar_identifier(*settings)));
  });
}

void add_vss_mip_command(CLI::App& solve_cmd, Session& session) {
  auto* vss_mip_cmd = solve_cmd.add_subcommand(
      "vss-mip", "Generate a minimal VSS layout for the timetable using a MIP");
  auto settings = std::make_shared<cda_rail::cli::VssMipSettings>();
  cda_rail::cli::add_vss_mip_options(*vss_mip_cmd, *settings);

  vss_mip_cmd->callback([&session, settings] {
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
    nlohmann::json solver_data;
    const auto     solution =
        [&] -> instances::SolVSSGeneralPerformanceOptimizationInstance {
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
        auto result = solver.solve(
            vss_mip_model_detail_mb(*settings), model_settings, solver_strategy,
            solution_settings, settings->solving.time_limit,
            settings->solving.debug_output, true);
        solver_data = solver.get_solver_data();
        return result;
      }

      mip_based::VSSGenTimetableSolver solver(session.instance());
      auto                             result = solver.solve(
          vss_mip_model_detail(*settings), model_settings, solver_strategy,
          solution_settings, settings->solving.time_limit,
          settings->solving.debug_output, true);
      solver_data = solver.get_solver_data();
      return result;
    }();
    log_solution_status(solution.get_status(), solution.get_obj());
    log_solution_evaluation(solution);
    session.set_solution(
        std::make_unique<
            instances::SolVSSGeneralPerformanceOptimizationInstance>(solution),
        settings->delta_t,
        static_cast<const cda_rail::solver::GeneralSolutionSettings&>(
            solution_settings),
        std::move(solver_data),
        hash_identifier(generate_vss_mip_identifier(*settings)));
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

void cda_rail::cli::add_solution_commands(CLI::App& app, Session& session) {
  auto* solution_cmd = app.add_subcommand(
      "solution", "Work with the solution of the last solve");
  solution_cmd->require_subcommand(1);

  auto* evaluate_cmd = solution_cmd->add_subcommand(
      "evaluate", "Simulate the decisions of the solution in detail and report "
                  "the resulting delays. If the solution was exported, the "
                  "evaluation is written back to it.");
  auto time_step = std::make_shared<std::optional<double>>();
  evaluate_cmd
      ->add_option("-c,--dt,--timestep", *time_step,
                   "Time step (dt) of the evaluation in seconds. If unset, the "
                   "time step of the solver is used.")
      ->check(CLI::PositiveNumber);
  evaluate_cmd->callback([&session, time_step] {
    const auto path = session.evaluate_solution(*time_step);
    log_solution_evaluation(session.solution());
    if (path.has_value()) {
      std::cout << "Wrote evaluation to " << path->string() << '\n';
    }
  });

  auto* export_cmd = solution_cmd->add_subcommand(
      "export",
      "Export the solution, including solver_data.json, as 'solve' does with "
      "--export-solution. The solution can be exported more than once; "
      "'solution evaluate' writes back to the last export. The MIP model "
      "itself (--export-lp-model) can only be exported by 'solve'.");
  auto export_settings = std::make_shared<cda_rail::cli::ExportSettings>();
  export_cmd
      ->add_flag(
          "-i,--export-solution-and-instance",
          [export_settings](int) {
            export_settings->export_option = cda_rail::solver::
                GeneralExportOption::ExportSolutionWithInstance;
          },
          "Export the instance as well.")
      ->group("Export Options");
  cda_rail::cli::add_export_location_options(*export_cmd, *export_settings)
      .solution_subdirectory->required();
  export_cmd->callback([&session, export_settings] {
    auto solution_settings = cda_rail::cli::general_solution_settings(
        *export_settings, session.working_directory().string());
    if (solution_settings.export_option ==
        cda_rail::solver::GeneralExportOption::NoExport) {
      solution_settings.export_option =
          cda_rail::solver::GeneralExportOption::ExportSolution;
    }
    const auto path = session.export_solution(
        std::move(solution_settings), export_settings->generate_identifier);
    std::cout << "Wrote solution to " << path.string() << '\n';
  });
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,readability-inconsistent-ifelse-braces)
