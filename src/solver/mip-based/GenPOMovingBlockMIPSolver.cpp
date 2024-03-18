#include "solver/mip-based/GenPOMovingBlockMIPSolver.hpp"

#include "MultiArray.hpp"
#include "gurobi_c++.h"

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay)

cda_rail::instances::SolGeneralPerformanceOptimizationInstance
cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::solve(
    const SolutionSettingsMovingBlock& solution_settings_input, int time_limit,
    bool debug_input) {
  /**
   * Solves initiated performance optimization problem with moving block
   * signaling/routing. Only breakable edges can use moving block. On all
   * others, only one train is allowed (in practice Flankenschutz can be
   * included this way). Trains are only routed if no route is specified.
   *
   * @param time_limit: time limit for the solver in seconds. If -1, no time
   * limit is set.
   * @param debug_input: if true, the debug output is enabled.
   *
   * @return: respective solution object
   */

  this->solve_init_gen_po_mb(time_limit, debug_input);

  if (!instance.n().is_consistent_for_transformation()) {
    PLOGE << "Instance is not consistent for transformation.";
    throw exceptions::ConsistencyException();
  }

  instances::GeneralPerformanceOptimizationInstance old_instance = instance;
  this->instance.discretize_stops();

  num_tr                  = instance.get_train_list().size();
  num_edges               = instance.const_n().number_of_edges();
  num_vertices            = instance.const_n().number_of_vertices();
  max_t                   = instance.max_t();
  this->solution_settings = solution_settings_input;
  this->ttd_sections      = instance.n().unbreakable_sections();
  this->num_ttd           = this->ttd_sections.size();

  PLOGD << "Create variables";
  create_variables();

  this->instance = old_instance;

  return {};
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    create_variables() {
  create_timing_variables();
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    create_timing_variables() {
  vars["t_front_arrival"]   = MultiArray<GRBVar>(num_tr, num_vertices);
  vars["t_front_departure"] = MultiArray<GRBVar>(num_tr, num_vertices);
  vars["t_rear_departure"]  = MultiArray<GRBVar>(num_tr, num_vertices);
  vars["t_ttd_departure"]   = MultiArray<GRBVar>(num_tr, num_ttd);

  for (size_t tr = 0; tr < num_tr; tr++) {
    const double tr_approx_leaving_time =
        instance.get_approximate_leaving_time(tr);
    for (size_t v = 0; v < num_vertices; v++) {
      vars["t_front_arrival"](tr, v) =
          model->addVar(0.0, max_t, 0.0, GRB_CONTINUOUS);
      vars["t_front_departure"](tr, v) = model->addVar(
          0.0, max_t + tr_approx_leaving_time, 0.0, GRB_CONTINUOUS);
      vars["t_rear_departure"](tr, v) = model->addVar(
          0.0, max_t + tr_approx_leaving_time, 0.0, GRB_CONTINUOUS);
    }
    for (size_t ttd = 0; ttd < num_ttd; ttd++) {
      vars["t_ttd_departure"](tr, ttd) = model->addVar(
          0.0, max_t + tr_approx_leaving_time, 0.0, GRB_CONTINUOUS);
    }
  }
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay)
