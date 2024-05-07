#include "EOMHelper.hpp"
#include "MultiArray.hpp"
#include "gurobi_c++.h"
#include "solver/mip-based/GenPOMovingBlockMIPSolver.hpp"
#include "solver/mip-based/GeneralMIPSolver.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,performance-inefficient-string-concatenation)

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::LazyCallback::
    callback() {
  try {
    if (where == GRB_CB_MESSAGE) {
      MessageCallback::callback();
    } else if (where == GRB_CB_MIPSOL) {
      const auto routes = get_routes();
      // TODO: Continue
    }
  } catch (GRBException& e) {
    PLOGE << "Error number: " << e.getErrorCode();
    PLOGE << e.getMessage();
  } catch (...) {
    PLOGE << "Error during callback";
  }
}

std::vector<std::vector<std::pair<size_t, double>>> cda_rail::solver::
    mip_based::GenPOMovingBlockMIPSolver::LazyCallback::get_routes() {
  /**
   * Extract routes from the current solution.
   * At the same time, save the distance from the start for every vertex.
   */

  std::vector<std::vector<std::pair<size_t, double>>> routes;
  routes.reserve(solver->num_tr);
  for (size_t tr = 0; tr < solver->num_tr; tr++) {
    routes.emplace_back();
    assert(routes.size() == tr + 1);
    const auto entry = solver->instance.get_schedule(tr).get_entry();
    auto       edges_to_consider = solver->instance.const_n().out_edges(entry);

    double current_pos = 0;
    routes[tr].emplace_back(entry, current_pos);
    while (!edges_to_consider.empty()) {
      const auto& edge_id = edges_to_consider.back();
      edges_to_consider.pop_back();
      auto& tmp_var = solver->vars["x"](tr, edge_id);
      if (!tmp_var.sameAs(GRBVar()) && getSolution(tmp_var) > 0.5) {
        const auto& edge_object = solver->instance.const_n().get_edge(edge_id);
        current_pos += edge_object.length;
        routes[tr].emplace_back(edge_object.target, current_pos);
        const auto& [old_edge_id, old_edge_pos] =
            solver->instance.const_n().get_old_edge(edge_id);
        edges_to_consider = solver->instance.const_n().out_edges(
            solver->instance.const_n().get_edge(edge_id).target);
      }
    }
  }

  return routes;
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,performance-inefficient-string-concatenation)
