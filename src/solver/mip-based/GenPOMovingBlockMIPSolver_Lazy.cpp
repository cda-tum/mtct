#include "EOMHelper.hpp"
#include "MultiArray.hpp"
#include "gurobi_c++.h"
#include "solver/mip-based/GenPOMovingBlockMIPSolver.hpp"
#include "solver/mip-based/GeneralMIPSolver.hpp"

#include <algorithm>
#include <unordered_map>
#include <utility>
#include <vector>

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,performance-inefficient-string-concatenation)

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::LazyCallback::
    callback() {
  try {
    if (where == GRB_CB_MESSAGE) {
      MessageCallback::callback();
    } else if (where == GRB_CB_MIPSOL) {
      const auto routes                = get_routes();
      const auto train_velocities      = get_train_velocities(routes);
      const auto train_orders_on_edges = get_train_orders_on_edges(routes);
      const auto train_orders_on_ttd   = get_train_orders_on_ttd();

      // Check edge headways
      for (size_t tr = 0; tr < solver->num_tr; tr++) {
        const auto& tr_object = solver->instance.get_train_list().get_train(tr);
        for (size_t r_v_idx = 0; r_v_idx < routes.at(tr).size(); r_v_idx++) {
          const auto& [v_idx, pos] = routes.at(tr).at(r_v_idx);
          const auto& vel          = train_velocities.at(tr).at(v_idx);
          const auto  bd           = vel * vel / (2 * tr_object.deceleration);
          const auto  ma_pos       = pos + bd;
          if (ma_pos >= routes.at(tr).back().second) {
            // TODO: Exit node is reached by moving authority
          } else {
            // r_ma_idx >= r_v_idx s.th. routes.at(tr).at(r_ma_idx).second <=
            // ma_pos < routes.at(tr).at(r_ma_idx + 1).second which should be
            // unique by design
            size_t r_ma_idx = r_v_idx;
            while (routes.at(tr).at(r_ma_idx + 1).second <= ma_pos) {
              r_ma_idx++;
            }
            const auto& [rel_source, rel_source_pos] =
                routes.at(tr).at(r_ma_idx);
            const auto& [rel_target, rel_target_pos] =
                routes.at(tr).at(r_ma_idx + 1);
            const auto rel_e_idx = solver->instance.const_n().get_edge_index(
                rel_source, rel_target);
            // TODO: Continue checking edge headways
          }
        }
      }
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

std::vector<std::pair<std::vector<size_t>, std::vector<size_t>>>
cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::LazyCallback::
    get_train_orders_on_edges(
        const std::vector<std::vector<std::pair<size_t, double>>>& routes) {
  std::vector<std::pair<std::vector<size_t>, std::vector<size_t>>>
      train_orders_on_edges;
  train_orders_on_edges.reserve(solver->num_edges);
  for (size_t edge_id = 0; edge_id < solver->num_edges; edge_id++) {
    train_orders_on_edges.emplace_back();
    assert(train_orders_on_edges.size() == edge_id + 1);
    const auto& edge_object = solver->instance.const_n().get_edge(edge_id);
    std::unordered_map<size_t, double> train_edge_times_source;
    std::unordered_map<size_t, double> train_edge_times_target;
    for (size_t tr = 0; tr < solver->num_tr; tr++) {
      for (size_t i = 0; i < routes[tr].size() - 1; i++) {
        if (routes[tr][i].first == edge_object.source &&
            routes[tr][i + 1].first == edge_object.target) {
          GRBVar t_source =
              solver->vars["t_front_departure"](tr, edge_object.source);
          GRBVar t_target =
              solver->vars["t_rear_departure"](tr, edge_object.target);
          // Assume they exist by choice of routes
          train_edge_times_source[tr] = getSolution(t_source);
          train_edge_times_target[tr] = getSolution(t_target);
          train_orders_on_edges[edge_id].first.emplace_back(tr);
          train_orders_on_edges[edge_id].second.emplace_back(tr);
        }
      }
    }
    if (train_orders_on_edges[edge_id].first.size() >= 2) {
      std::sort(train_orders_on_edges[edge_id].first.begin(),
                train_orders_on_edges[edge_id].first.end(),
                [&train_edge_times_source](size_t tr1, size_t tr2) {
                  return train_edge_times_source[tr1] <
                         train_edge_times_source[tr2];
                });
    }
    if (train_orders_on_edges[edge_id].second.size() >= 2) {
      std::sort(train_orders_on_edges[edge_id].second.begin(),
                train_orders_on_edges[edge_id].second.end(),
                [&train_edge_times_target](size_t tr1, size_t tr2) {
                  return train_edge_times_target[tr1] <
                         train_edge_times_target[tr2];
                });
    }
  }

  return train_orders_on_edges;
}

std::vector<std::unordered_map<size_t, double>> cda_rail::solver::mip_based::
    GenPOMovingBlockMIPSolver::LazyCallback::get_train_velocities(
        const std::vector<std::vector<std::pair<size_t, double>>>& routes) {
  std::vector<std::unordered_map<size_t, double>> train_velocities(
      solver->num_tr);
  for (size_t tr = 0; tr < solver->num_tr; tr++) {
    const auto& exit = solver->instance.get_schedule(tr).get_exit();
    for (size_t route_v_idx = 0; route_v_idx < routes[tr].size();
         route_v_idx++) {
      const auto& v_idx = routes[tr][route_v_idx].first;
      const auto  e_idx = (route_v_idx == routes[tr].size() - 1)
                              ? solver->instance.const_n().get_edge_index(
                                   routes[tr][route_v_idx - 1].first, v_idx)
                              : solver->instance.const_n().get_edge_index(
                                   v_idx, routes[tr][route_v_idx + 1].first);
      const auto& edge  = solver->instance.const_n().get_edge(e_idx);
      const auto& source_velocities =
          solver->velocity_extensions.at(tr).at(edge.source);
      const auto& target_velocities =
          solver->velocity_extensions.at(tr).at(edge.target);

      bool vel_found = false;
      for (size_t i = 0; i < source_velocities.size() && !vel_found; i++) {
        const auto& source_v = source_velocities[i];
        for (size_t j = 0; j < target_velocities.size() && !vel_found; j++) {
          const auto& target_v  = target_velocities[j];
          GRBVar      y_var_tmp = solver->vars["y"](tr, e_idx, i, j);
          if (!y_var_tmp.sameAs(GRBVar()) && getSolution(y_var_tmp) > 0.5) {
            train_velocities[tr][v_idx] =
                edge.source == v_idx ? source_v : target_v;
            vel_found = true;
          }
        }
      }
      if (!vel_found) {
        PLOGE << "No velocity found for train " << tr << " at vertex " << v_idx;
        throw cda_rail::exceptions::ConsistencyException(
            "No velocity found for train " + std::to_string(tr) +
            " at vertex " + std::to_string(v_idx));
      }
    }
  }

  return train_velocities;
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,performance-inefficient-string-concatenation)
