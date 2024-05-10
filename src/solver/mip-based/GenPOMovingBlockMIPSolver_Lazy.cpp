#include "Definitions.hpp"
#include "EOMHelper.hpp"
#include "MultiArray.hpp"
#include "gurobi_c++.h"
#include "solver/mip-based/GenPOMovingBlockMIPSolver.hpp"
#include "solver/mip-based/GeneralMIPSolver.hpp"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdlib>
#include <unordered_map>
#include <unordered_set>
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

      create_lazy_edge_headway_constraints(routes, train_velocities,
                                           train_orders_on_edges);
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

bool cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::LazyCallback::
    create_lazy_edge_headway_constraints(
        const std::vector<std::vector<std::pair<size_t, double>>>& routes,
        const std::vector<std::unordered_map<size_t, double>>& train_velocities,
        const std::vector<std::pair<std::vector<size_t>, std::vector<size_t>>>&
            train_orders_on_edges) {
  // Check edge headways
  bool violated_constraint_found = false;
  bool only_one_constraint =
      solver->solver_strategy.lazy_constraint_selection_strategy ==
      LazyConstraintSelectionStrategy::OnlyFirstFound;
  for (size_t tr = 0; tr < solver->num_tr &&
                      (!only_one_constraint || !violated_constraint_found);
       tr++) {
    const auto& tr_object = solver->instance.get_train_list().get_train(tr);
    const auto  t_bound   = solver->ub_timing_variable(tr);
    for (size_t r_v_idx = 0;
         r_v_idx < routes.at(tr).size() &&
         (!only_one_constraint || !violated_constraint_found);
         r_v_idx++) {
      const auto& [v_idx, pos] = routes.at(tr).at(r_v_idx);
      const auto& vel          = train_velocities.at(tr).at(v_idx);
      const auto  bd           = vel * vel / (2 * tr_object.deceleration);
      const auto  ma_pos       = pos + bd;

      const auto& tr_t_var       = solver->vars["t_front_departure"](tr, v_idx);
      const auto& tr_t_var_value = getSolution(tr_t_var);

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
        const auto& [rel_source, rel_source_pos] = routes.at(tr).at(r_ma_idx);
        const auto& [rel_target, rel_target_pos] =
            routes.at(tr).at(r_ma_idx + 1);
        assert(rel_source_pos <= ma_pos);
        assert(ma_pos < rel_target_pos);
        const auto rel_pos_on_edge = ma_pos - rel_source_pos;

        // Get used path, which is
        // routes.at(tr).at(i) -> routes.at(tr).at(i + 1) for i in [r_v_idx,
        // r_ma_idx]
        const std::vector<size_t> p = [&]() {
          std::vector<size_t> p_tmp;
          p_tmp.reserve(r_ma_idx - r_v_idx + 1);
          for (size_t i = r_v_idx; i <= r_ma_idx; i++) {
            p_tmp.emplace_back(solver->instance.const_n().get_edge_index(
                routes.at(tr).at(i).first, routes.at(tr).at(i + 1).first));
          }
          return p_tmp;
        }();
        const auto& rel_e_idx = p.back();
        const auto& rel_e_obj = solver->instance.const_n().get_edge(rel_e_idx);

        // Create path expression according to route. The first edge must
        // use the specified velocity or faster, since only then the desired
        // headway must hold.
        GRBLinExpr  edge_path_expr = 0;
        const auto& e_1            = p.front();
        const auto& e_1_obj        = solver->instance.const_n().get_edge(e_1);
        const auto  tmp_max_speed =
            std::min(tr_object.max_speed, e_1_obj.max_speed);
        const auto& v_source_velocities =
            solver->velocity_extensions.at(tr).at(e_1_obj.source);
        const auto& v_target_velocities =
            solver->velocity_extensions.at(tr).at(e_1_obj.target);
        for (size_t v_source_index = 0;
             v_source_index < v_source_velocities.size(); v_source_index++) {
          const auto& vel_source = v_source_velocities.at(v_source_index);
          if (vel_source < vel || vel_source > tmp_max_speed) {
            continue;
          }
          for (size_t v_target_index = 0;
               v_target_index < v_target_velocities.size(); v_target_index++) {
            const auto& vel_target = v_target_velocities.at(v_target_index);
            if (vel_target > tmp_max_speed) {
              continue;
            }
            if (cda_rail::possible_by_eom(
                    vel_source, vel_target, tr_object.acceleration,
                    tr_object.deceleration, e_1_obj.length)) {
              edge_path_expr +=
                  solver->vars["y"](tr, e_1, v_source_index, v_target_index);
            }
          }
        }
        for (const auto& e_p : p) {
          if (e_p != e_1) {
            edge_path_expr += solver->vars["x"](tr, e_p);
          }
        }

        // Get other trains that might conflict with the current train on
        // this edge
        const auto& rel_tr_order = train_orders_on_edges.at(rel_e_idx);
        std::unordered_set<size_t> other_trains;
        for (size_t i = 0; i < 2; i++) {
          const auto& tr_order =
              i == 0 ? rel_tr_order.first : rel_tr_order.second;
          const auto tr_index =
              std::find(tr_order.begin(), tr_order.end(), tr) -
              tr_order.begin();
          assert(tr_index != tr_order.end() - tr_order.begin());
          for (size_t tr_other_idx = 0; tr_other_idx < tr_order.size();
               tr_other_idx++) {
            if (tr_other_idx == tr_index) {
              continue;
            }
            if (solver->solver_strategy.lazy_train_selection_strategy ==
                    LazyTrainSelectionStrategy::OnlyAdjacent &&
                std::abs(static_cast<int>(tr_other_idx) -
                         static_cast<int>(tr_index)) > 1) {
              continue;
            }
            if (!solver->solver_strategy.include_reverse_headways &&
                tr_other_idx < tr_index) {
              continue;
            }
            other_trains.insert(tr_order.at(tr_other_idx));
          }
        }
        for (const auto& tr_other_idx : other_trains) {
          const auto& tr_other_object =
              solver->instance.get_train_list().get_train(tr_other_idx);
          const auto& tr_other_source_speed =
              train_velocities.at(tr_other_idx).at(rel_source);
          const auto& tr_other_target_speed =
              train_velocities.at(tr_other_idx).at(rel_target);

          const auto& tr_other_source_var =
              solver->vars["t_rear_departure"](tr_other_idx, rel_source);
          const auto& tr_other_target_var =
              solver->vars["t_front_departure"](tr_other_idx, rel_target);

          const auto& tr_other_max_speed =
              std::min(tr_other_object.max_speed, rel_e_obj.max_speed);

          // Check if this constraint should be added
          bool add_constr =
              (solver->solver_strategy.lazy_constraint_selection_strategy ==
               LazyConstraintSelectionStrategy::AllChecked);
          if (!add_constr &&
              tr_t_var_value <
                  getSolution(tr_other_source_var) +
                      cda_rail::min_travel_time_from_start(
                          tr_other_source_speed, tr_other_target_speed,
                          tr_other_max_speed, tr_other_object.acceleration,
                          tr_other_object.deceleration, rel_e_obj.length,
                          rel_pos_on_edge)) {
            add_constr = true;
          }
          if (!add_constr && rel_pos_on_edge > EPS &&
              tr_t_var_value <
                  getSolution(tr_other_target_var) -
                      cda_rail::max_travel_time_to_end(
                          tr_other_source_speed, tr_other_target_speed, V_MIN,
                          tr_other_object.acceleration,
                          tr_other_object.deceleration, rel_e_obj.length,
                          rel_pos_on_edge, rel_e_obj.breakable)) {
            add_constr = true;
          }

          const auto& t_bound_tmp =
              std::max(t_bound, solver->ub_timing_variable(tr_other_idx));

          if (add_constr) {
            const GRBLinExpr lhs =
                tr_t_var +
                t_bound_tmp * (static_cast<double>(p.size()) - edge_path_expr) +
                t_bound_tmp *
                    (1 - solver->vars["order"](tr, tr_other_idx, p.back()));
            std::vector<GRBLinExpr> rhs;
            if (rel_pos_on_edge < EPS) {
              rhs.emplace_back(tr_other_source_var);
            } else {
              rhs.emplace_back(tr_other_source_var);
              rhs.emplace_back(tr_other_target_var);

              const auto& v_tr_other_source_velocities =
                  solver->velocity_extensions.at(tr_other_idx).at(rel_source);
              const auto& v_tr_other_target_velocities =
                  solver->velocity_extensions.at(tr_other_idx).at(rel_target);

              for (size_t v_tr_other_source_index = 0;
                   v_tr_other_source_index <
                   v_tr_other_source_velocities.size();
                   v_tr_other_source_index++) {
                const auto& vel_tr_other_source =
                    v_tr_other_source_velocities.at(v_tr_other_source_index);
                if (vel_tr_other_source > tr_other_max_speed) {
                  continue;
                }
                for (size_t v_tr_other_target_index = 0;
                     v_tr_other_target_index <
                     v_tr_other_target_velocities.size();
                     v_tr_other_target_index++) {
                  const auto& vel_tr_other_target =
                      v_tr_other_target_velocities.at(v_tr_other_target_index);
                  if (vel_tr_other_target > tr_other_max_speed) {
                    continue;
                  }
                  if (cda_rail::possible_by_eom(
                          vel_tr_other_source, vel_tr_other_target,
                          tr_other_object.acceleration,
                          tr_other_object.deceleration, rel_e_obj.length)) {
                    rhs.at(0) +=
                        solver->vars["y"](tr_other_idx, rel_e_idx,
                                          v_tr_other_source_index,
                                          v_tr_other_target_index) *
                        cda_rail::min_travel_time_from_start(
                            vel_tr_other_source, vel_tr_other_target,
                            tr_other_max_speed, tr_other_object.acceleration,
                            tr_other_object.deceleration, rel_e_obj.length,
                            rel_pos_on_edge);
                    const auto max_travel_time =
                        cda_rail::max_travel_time_to_end(
                            vel_tr_other_source, vel_tr_other_target, V_MIN,
                            tr_other_object.acceleration,
                            tr_other_object.deceleration, rel_e_obj.length,
                            rel_pos_on_edge, rel_e_obj.breakable);
                    rhs.at(1) -=
                        solver->vars["y"](tr_other_idx, rel_e_idx,
                                          v_tr_other_source_index,
                                          v_tr_other_target_index) *
                        (max_travel_time > t_bound_tmp ? t_bound_tmp
                                                       : max_travel_time);
                  }
                }
              }
            }

            for (const auto& rhs_expr : rhs) {
              addLazy(lhs >= rhs_expr);
              if (solver->solution_settings.export_option ==
                      ExportOption::ExportLP ||
                  solver->solution_settings.export_option ==
                      ExportOption::ExportSolutionAndLP ||
                  solver->solution_settings.export_option ==
                      ExportOption::ExportSolutionWithInstanceAndLP) {
                solver->lazy_constraints.emplace_back(lhs >= rhs_expr);
              }
              violated_constraint_found = true;
            }
          }
        }
      }
    }
  }

  return violated_constraint_found;
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,performance-inefficient-string-concatenation)
