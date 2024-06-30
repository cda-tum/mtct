#include "CustomExceptions.hpp"
#include "MultiArray.hpp"
#include "gurobi_c++.h"
#include "solver/mip-based/VSSGenTimetableSolver.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
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
  /**
   * This function solves the VSS generation problem.
   * It functions the same as the solve function in the parent class, but
   * includes additional information from a previous moving block solution.
   *
   * @param model_detail_mb_information.fix_stop_positions: Whether to fix the
   * positions at which trains stop at a station
   * @param model_detail_mb_information.fix_exact_positions: Whether to fix the
   * exact positions of trains at every vertex / bound them by their minimal and
   * maximal positions
   * @param model_detail_mb_information.hint_approximate_positions: Whether to
   * hint approximate positions of trains at every given time
   *
   * @return The solution object
   */

  if (solver_strategy.iterative_approach) {
    throw cda_rail::exceptions::InvalidInputException(
        "Iterative approach not supported for this model.");
  }

  auto old_instance =
      initialize_variables({model_detail_mb_information.delta_t, true,
                            model_detail_mb_information.train_dynamics,
                            model_detail_mb_information.braking_curves},
                           model_settings, solver_strategy, solution_settings,
                           time_limit, debug_input);
  fix_stop_positions  = model_detail_mb_information.fix_stop_positions;
  fix_exact_positions = model_detail_mb_information.fix_exact_positions;
  hint_approximate_positions =
      model_detail_mb_information.hint_approximate_positions;

  create_variables();
  set_objective();
  create_constraints();
  include_additional_information();

  std::optional<instances::SolVSSGenerationTimetable> sol_object;

  return sol_object.value();
}

void cda_rail::solver::mip_based::
    VSSGenTimetableSolverWithMovingBlockInformation::
        include_additional_information() {
  PLOGD << "Including additional information";
  fix_oder_on_edges();
  if (fix_stop_positions) {
    PLOGD << "Fixing stop positions";
    fix_stop_positions_constraints();
  }
  if (fix_exact_positions) {
    PLOGD << "Fixing exact positions";
    fix_exact_positions_constraints();
  }
  if (hint_approximate_positions) {
    PLOGD << "Hinting approximate positions";
    hint_approximate_positions_constraints();
  }
}

void cda_rail::solver::mip_based::
    VSSGenTimetableSolverWithMovingBlockInformation::
        fix_stop_positions_constraints() {
  // TODO
}

void cda_rail::solver::mip_based::
    VSSGenTimetableSolverWithMovingBlockInformation::
        fix_exact_positions_constraints() {
  // TODO
}

void cda_rail::solver::mip_based::
    VSSGenTimetableSolverWithMovingBlockInformation::
        hint_approximate_positions_constraints() {
  const auto& train_list = instance.get_train_list();
  for (size_t tr = 0; tr < num_tr; ++tr) {
    const auto& tr_obj  = train_list.get_train(tr);
    const auto& tr_name = tr_obj.name;
    const auto& tr_len  = tr_obj.length;
    for (size_t t_steps = train_interval[tr].first;
         t_steps <= train_interval[tr].second + 1; ++t_steps) {
      const auto t = t_steps * dt;
      const auto approx_info =
          moving_block_solution.get_approximate_train_pos_and_vel(tr_name, t);
      if (approx_info.has_value()) {
        const auto& [pos_approx, vel_approx] = approx_info.value();
        const double bl = include_braking_curves ? vel_approx * vel_approx /
                                                       (2 * tr_obj.deceleration)
                                                 : 0;
        vars["v"](tr, t_steps).set(GRB_DoubleAttr_VarHintVal, vel_approx);
        if (t_steps >= train_interval[tr].first + 1) {
          vars["mu"](tr, t_steps - 1)
              .set(GRB_DoubleAttr_VarHintVal, pos_approx + bl);
          if (include_braking_curves) {
            vars["brakelen"](tr, t_steps - 1)
                .set(GRB_DoubleAttr_VarHintVal, bl);
          }
        }
        if (t_steps <= train_interval[tr].second) {
          vars["lda"](tr, t_steps)
              .set(GRB_DoubleAttr_VarHintVal, pos_approx - tr_len);
        }
      }
    }
  }
}

void cda_rail::solver::mip_based::
    VSSGenTimetableSolverWithMovingBlockInformation::fix_oder_on_edges() {
  // TODO
}

// NOLINTEND(performance-inefficient-string-concatenation)

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay)
