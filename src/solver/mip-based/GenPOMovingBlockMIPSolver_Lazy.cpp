#include "EOMHelper.hpp"
#include "MultiArray.hpp"
#include "gurobi_c++.h"
#include "solver/mip-based/GenPOMovingBlockMIPSolver.hpp"
#include "solver/mip-based/GeneralMIPSolver.hpp"

#include <algorithm>
#include <unordered_map>

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

std::vector<std::vector<size_t>> cda_rail::solver::mip_based::
    GenPOMovingBlockMIPSolver::LazyCallback::get_train_orders_on_ttd() {
  std::vector<std::vector<size_t>> train_orders_on_ttd;
  train_orders_on_ttd.reserve(solver->num_ttd);
  for (size_t ttd = 0; ttd < solver->num_ttd; ttd++) {
    train_orders_on_ttd.emplace_back();
    assert(train_orders_on_ttd.size() == ttd + 1);
    std::unordered_map<size_t, double> train_ttd_times;
    for (size_t tr = 0; tr < solver->num_tr; tr++) {
      GRBVar x_ttd = solver->vars["x_ttd"](tr, ttd);
      GRBVar t_ttd = solver->vars["t_ttd_departure"](tr, ttd);
      if (!x_ttd.sameAs(GRBVar()) && getSolution(x_ttd) > 0.5) {
        train_ttd_times[tr] = getSolution(t_ttd);
        train_orders_on_ttd[ttd].emplace_back(tr);
      }
    }
    if (train_orders_on_ttd[ttd].size() >= 2) {
      std::sort(train_orders_on_ttd[ttd].begin(),
                train_orders_on_ttd[ttd].end(),
                [&train_ttd_times](size_t tr1, size_t tr2) {
                  return train_ttd_times[tr1] < train_ttd_times[tr2];
                });
    }
  }

  return train_orders_on_ttd;
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,performance-inefficient-string-concatenation)
