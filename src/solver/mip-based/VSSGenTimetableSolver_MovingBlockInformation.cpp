#include "CustomExceptions.hpp"
#include "MultiArray.hpp"
#include "gurobi_c++.h"
#include "solver/mip-based/VSSGenTimetableSolver.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <plog/Appenders/ColorConsoleAppender.h>
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Initializers/ConsoleInitializer.h>
#include <plog/Log.h>
#include <utility>

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay)

// NOLINTBEGIN(performance-inefficient-string-concatenation)

cda_rail::instances::SolVSSGenerationTimetable cda_rail::solver::mip_based::
    VSSGenTimetableSolverWithMovingBlockInformation::solve(
        const cda_rail::solver::mip_based::ModelDetailMBInformation&
            model_detail_mb_information,
        const cda_rail::solver::mip_based::ModelSettings&    model_settings,
        const cda_rail::solver::mip_based::SolverStrategy&   solver_strategy,
        const cda_rail::solver::mip_based::SolutionSettings& solution_settings,
        int time_limit, bool debug_input) {
  auto old_instance =
      initialize_variables({model_detail_mb_information.delta_t,
                            model_detail_mb_information.fix_routes,
                            model_detail_mb_information.train_dynamics,
                            model_detail_mb_information.braking_curves},
                           model_settings, solver_strategy, solution_settings,
                           time_limit, debug_input);
  fix_stop_positions  = model_detail_mb_information.fix_stop_positions;
  fix_exact_positions = model_detail_mb_information.fix_exact_positions;
  hint_approximate_positions =
      model_detail_mb_information.hint_approximate_positions;

  std::optional<instances::SolVSSGenerationTimetable> sol_object;

  return sol_object.value();
}

// NOLINTEND(performance-inefficient-string-concatenation)

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay)
