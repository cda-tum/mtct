#include "CustomExceptions.hpp"
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
#include <limits>
#include <numeric>
#include <optional>
#include <string>
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

      auto constraint_created =
          create_lazy_vertex_headway_constraints(routes, train_orders_on_edges);
      if (solver->solver_strategy.lazy_constraint_selection_strategy !=
              LazyConstraintSelectionStrategy::OnlyFirstFound ||
          !constraint_created) {
        constraint_created = create_lazy_edge_and_ttd_headway_constraints(
            routes, train_velocities, train_orders_on_edges,
            train_orders_on_ttd);
      }
      if (solver->solver_strategy.lazy_constraint_selection_strategy !=
              LazyConstraintSelectionStrategy::OnlyFirstFound ||
          !constraint_created) {
        create_lazy_reverse_edge_constraints(train_orders_on_edges);
      }
    }
  } catch (GRBException& e) {
    PLOGE << "Error number: " << e.getErrorCode();
    PLOGE << e.getMessage();
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

std::vector<std::pair<std::vector<std::pair<size_t, bool>>,
                      std::vector<std::pair<size_t, bool>>>>
cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::LazyCallback::
    get_train_orders_on_edges(
        const std::vector<std::vector<std::pair<size_t, double>>>& routes) {
  std::vector<std::pair<std::vector<std::pair<size_t, bool>>,
                        std::vector<std::pair<size_t, bool>>>>
      train_orders_on_edges;
  train_orders_on_edges.reserve(solver->num_edges);
  for (size_t edge_id = 0; edge_id < solver->num_edges; edge_id++) {
    train_orders_on_edges.emplace_back();
    assert(train_orders_on_edges.size() == edge_id + 1);
    const auto& edge_object = solver->instance.const_n().get_edge(edge_id);
    std::unordered_map<size_t, double> train_edge_times_source;
    std::unordered_map<size_t, double> train_edge_times_target;
    for (size_t tr = 0; tr < solver->num_tr; tr++) {
      bool edge_found = false;
      for (size_t i = 0; i < routes[tr].size() - 1 && !edge_found; i++) {
        if ((routes[tr][i].first == edge_object.source &&
             routes[tr][i + 1].first == edge_object.target) ||
            (routes[tr][i].first == edge_object.target &&
             routes[tr][i + 1].first == edge_object.source)) {
          GRBVar t_source =
              solver->vars["t_front_departure"](tr, edge_object.source);
          GRBVar t_target =
              solver->vars["t_rear_departure"](tr, edge_object.target);
          // Assume they exist by choice of routes
          train_edge_times_source[tr] = getSolution(t_source);
          train_edge_times_target[tr] = getSolution(t_target);
          train_orders_on_edges[edge_id].first.emplace_back(
              tr, routes[tr][i].first == edge_object.source);
          train_orders_on_edges[edge_id].second.emplace_back(
              tr, routes[tr][i].first == edge_object.source);
          edge_found = true;
        }
      }
    }
    if (train_orders_on_edges[edge_id].first.size() >= 2) {
      std::sort(train_orders_on_edges[edge_id].first.begin(),
                train_orders_on_edges[edge_id].first.end(),
                [&train_edge_times_source](std::pair<size_t, bool> tr1,
                                           std::pair<size_t, bool> tr2) {
                  return train_edge_times_source[tr1.first] <
                         train_edge_times_source[tr2.first];
                });
    }
    if (train_orders_on_edges[edge_id].second.size() >= 2) {
      std::sort(train_orders_on_edges[edge_id].second.begin(),
                train_orders_on_edges[edge_id].second.end(),
                [&train_edge_times_target](std::pair<size_t, bool> tr1,
                                           std::pair<size_t, bool> tr2) {
                  return train_edge_times_target[tr1.first] <
                         train_edge_times_target[tr2.first];
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
    create_lazy_edge_and_ttd_headway_constraints(
        const std::vector<std::vector<std::pair<size_t, double>>>& routes,
        const std::vector<std::unordered_map<size_t, double>>& train_velocities,
        const std::vector<std::pair<std::vector<std::pair<size_t, bool>>,
                                    std::vector<std::pair<size_t, bool>>>>&
                                                train_orders_on_edges,
        const std::vector<std::vector<size_t>>& train_orders_on_ttd) {
  bool       violated_constraint_found = false;
  const bool only_one_constraint =
      solver->solver_strategy.lazy_constraint_selection_strategy ==
      LazyConstraintSelectionStrategy::OnlyFirstFound;
  for (size_t tr = 0; tr < solver->num_tr &&
                      (!only_one_constraint || !violated_constraint_found);
       tr++) {
    const auto& tr_object = solver->instance.get_train_list().get_train(tr);
    const auto  t_bound   = solver->ub_timing_variable(tr);
    const auto& entry     = solver->instance.get_schedule(tr).get_entry();
    // Check every vertex except the last one, because only vertex headway is
    // imposed in that case
    for (size_t r_v_idx = 0;
         r_v_idx < routes.at(tr).size() - 1 &&
         (!only_one_constraint || !violated_constraint_found);
         r_v_idx++) {
      const auto& [v_idx, pos] = routes.at(tr).at(r_v_idx);
      const auto& vel          = train_velocities.at(tr).at(v_idx);
      const auto  bd           = vel * vel / (2 * tr_object.deceleration);
      const auto  ma_pos       = pos + bd;

      const auto& tr_t_var       = solver->vars["t_front_departure"](tr, v_idx);
      const auto& tr_t_var_value = getSolution(tr_t_var);

      if (ma_pos <= routes.at(tr).back().second) {
        // r_ma_idx >= r_v_idx s.th. routes.at(tr).at(r_ma_idx).second <
        // ma_pos <= routes.at(tr).at(r_ma_idx + 1).second which should be
        // unique by design unless bd = 0, then r_ma_idx = r_v_idx
        size_t r_ma_idx = r_v_idx;
        while (routes.at(tr).at(r_ma_idx + 1).second < ma_pos - EPS) {
          r_ma_idx++;
        }
        const auto& [rel_source, rel_source_pos] = routes.at(tr).at(r_ma_idx);
        const auto& [rel_target, rel_target_pos] =
            routes.at(tr).at(r_ma_idx + 1);
        assert(bd == 0 || rel_source_pos < ma_pos - EPS);
        assert(ma_pos <= rel_target_pos);
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
        const GRBLinExpr edge_path_expr = solver->get_edge_path_expr(
            tr, p, vel,
            solver->solver_strategy.include_higher_velocities_in_edge_expr);

        // Get other trains that might conflict with the current train on
        // this edge
        const auto& [rel_tr_order_source, rel_tr_order_target] =
            train_orders_on_edges.at(rel_e_idx);
        std::unordered_set<size_t> other_trains;
        for (size_t i = 0; i < 2; i++) {
          const auto& tr_order =
              i == 0 ? rel_tr_order_source : rel_tr_order_target;
          const auto tr_index = std::find(tr_order.begin(), tr_order.end(),
                                          std::pair<size_t, bool>(tr, true)) -
                                tr_order.begin();
          assert(tr_index != tr_order.end() - tr_order.begin());
          for (size_t tr_other_idx = 0; tr_other_idx < tr_order.size();
               tr_other_idx++) {
            if (tr_other_idx == tr_index) {
              continue;
            }
            if (!tr_order.at(tr_other_idx).second) {
              // The train travels in reverse direction!
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
            other_trains.insert(tr_order.at(tr_other_idx).first);
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
            if (std::abs(rel_e_obj.length - rel_pos_on_edge) < EPS) {
              rhs.emplace_back(tr_other_target_var);
            } else if (rel_pos_on_edge < EPS) {
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

            if (!rhs.empty()) {
              GRBLinExpr order_expr =
                  solver->vars["order"](tr, tr_other_idx, p.back()) +
                  solver->vars["order"](tr_other_idx, tr, p.back());
              GRBLinExpr edge_expr = solver->vars["x"](tr, p.back()) +
                                     solver->vars["x"](tr_other_idx, p.back());
              addLazy(order_expr <= 0.5 * edge_expr);
              addLazy(order_expr >= edge_expr - 1);
              if (solver->solution_settings.export_option ==
                      ExportOption::ExportLP ||
                  solver->solution_settings.export_option ==
                      ExportOption::ExportSolutionAndLP ||
                  solver->solution_settings.export_option ==
                      ExportOption::ExportSolutionWithInstanceAndLP) {
                solver->lazy_constraints.emplace_back(order_expr <=
                                                      0.5 * edge_expr);
                solver->lazy_constraints.emplace_back(order_expr >=
                                                      edge_expr - 1);
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

        // Is there a conflict with TTD constraints
        const auto intersecting_ttd =
            cda_rail::Network::get_intersecting_ttd(p, solver->ttd_sections);
        for (const auto& [ttd_index, e_index] : intersecting_ttd) {
          const auto& p_tmp =
              std::vector<size_t>(p.begin(), p.begin() + e_index);
          const auto p_tmp_len = std::accumulate(
              p_tmp.begin(), p_tmp.end(), 0.0,
              [this](double sum, const auto& edge_index) {
                return sum +
                       solver->instance.const_n().get_edge(edge_index).length;
              });
          GRBLinExpr edge_tmp_path_expr = 0;
          for (const auto& e_tmp : p_tmp) {
            edge_tmp_path_expr += solver->vars["x"](tr, e_tmp);
          }

          const auto obd = bd - p_tmp_len;
          assert(obd >= 0);

          double                t_reduction = 0;
          std::optional<double> t_addition;

          std::optional<size_t> prev_v_idx;
          std::optional<double> prev_pos;
          std::optional<double> prev_vel;
          std::optional<GRBVar> prev_t_var;
          std::optional<double> prev_t_var_value;
          std::optional<size_t> prev_edge_index;

          bool skip = false;
          if (v_idx == entry) {
            t_reduction = vel <= GRB_EPS ? 0 : obd / vel;
          } else {
            assert(r_v_idx >= 1);
            prev_v_idx = routes.at(tr).at(r_v_idx - 1).first;
            prev_pos   = routes.at(tr).at(r_v_idx - 1).second;
            prev_vel   = train_velocities.at(tr).at(prev_v_idx.value());
            const auto& prev_bd = prev_vel.value() * prev_vel.value() /
                                  (2 * tr_object.deceleration);
            const auto& prev_ma_pos = prev_pos.value() + prev_bd;
            prev_edge_index         = solver->instance.const_n().get_edge_index(
                prev_v_idx.value(), v_idx);
            const auto& prev_edge_object =
                solver->instance.const_n().get_edge(prev_edge_index.value());
            prev_t_var =
                solver->vars["t_front_departure"](tr, prev_v_idx.value());
            prev_t_var_value = getSolution(prev_t_var.value());
            const auto& prev_max_speed =
                std::min(prev_edge_object.max_speed, tr_object.max_speed);
            if (prev_ma_pos > pos + p_tmp_len) {
              skip = true;
              // obd is too long and relevant vertex is earlier
            } else {
              t_reduction = cda_rail::min_time_from_rear_to_ma_point(
                  prev_vel.value(), vel, V_MIN, prev_max_speed,
                  tr_object.acceleration, tr_object.deceleration,
                  prev_edge_object.length, obd);
              const auto tmp_max = cda_rail::max_time_from_front_to_ma_point(
                  prev_vel.value(), vel, V_MIN, tr_object.acceleration,
                  tr_object.deceleration, prev_edge_object.length, obd,
                  prev_edge_object.breakable);
              if (tmp_max < std::numeric_limits<double>::infinity()) {
                t_addition = tmp_max;
              }
            }
          }

          if (skip) {
            continue;
          }

          // Get other trains that might conflict with the current train on
          // this TTD section
          const auto& rel_tr_order_ttd = train_orders_on_ttd.at(ttd_index);
          std::unordered_set<size_t> other_trains_ttd;
          const auto                 tr_index =
              std::find(rel_tr_order_ttd.begin(), rel_tr_order_ttd.end(), tr) -
              rel_tr_order_ttd.begin();
          assert(tr_index != rel_tr_order_ttd.end() - rel_tr_order_ttd.begin());
          for (size_t tr_other_idx = 0; tr_other_idx < rel_tr_order_ttd.size();
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
            other_trains_ttd.insert(rel_tr_order_ttd.at(tr_other_idx));
          }

          for (const size_t other_tr : other_trains_ttd) {
            // Check if TTD constraint is violated or not and add if needed
            bool add_constr =
                (solver->solver_strategy.lazy_constraint_selection_strategy ==
                 LazyConstraintSelectionStrategy::AllChecked);
            const auto& other_tr_t_variable =
                solver->vars["t_ttd_departure"](other_tr, ttd_index);
            if (!add_constr && tr_t_var_value - t_reduction <
                                   getSolution(other_tr_t_variable)) {
              add_constr = true;
            }
            if (!add_constr && prev_t_var_value.has_value() &&
                t_addition.has_value() &&
                prev_t_var_value.value() + t_addition.value() <
                    getSolution(other_tr_t_variable)) {
              add_constr = true;
            }

            if (add_constr) {
              const auto& t_bound_tmp =
                  std::max(t_bound, solver->ub_timing_variable(other_tr));
              GRBLinExpr rhs =
                  other_tr_t_variable +
                  t_bound_tmp *
                      (solver->vars["order_ttd"](tr, other_tr, ttd_index) - 1);
              std::vector<GRBLinExpr> lhs;
              if (prev_edge_index.has_value()) {
                assert(prev_vel.has_value());
                const auto vel_idx =
                    std::find(
                        solver->velocity_extensions.at(tr).at(v_idx).begin(),
                        solver->velocity_extensions.at(tr).at(v_idx).end(),
                        vel) -
                    solver->velocity_extensions.at(tr).at(v_idx).begin();
                const auto prev_vel_idx =
                    std::find(solver->velocity_extensions.at(tr)
                                  .at(prev_v_idx.value())
                                  .begin(),
                              solver->velocity_extensions.at(tr)
                                  .at(prev_v_idx.value())
                                  .end(),
                              prev_vel.value()) -
                    solver->velocity_extensions.at(tr)
                        .at(prev_v_idx.value())
                        .begin();
                lhs.emplace_back(
                    tr_t_var - t_reduction +
                    t_bound_tmp *
                        (static_cast<double>(p_tmp.size()) -
                         edge_tmp_path_expr + 1 -
                         solver->vars["y"](tr, prev_edge_index.value(),
                                           prev_vel_idx, vel_idx)));
                if (t_addition.has_value()) {
                  assert(prev_t_var.has_value());
                  lhs.emplace_back(
                      prev_t_var.value() + t_addition.value() +
                      t_bound_tmp *
                          (static_cast<double>(p_tmp.size()) -
                           edge_tmp_path_expr + 1 -
                           solver->vars["y"](tr, prev_edge_index.value(),
                                             prev_vel_idx, vel_idx)));
                }
              } else {
                // Entry node
                assert(v_idx == entry);
                lhs.emplace_back(tr_t_var - t_reduction +
                                 t_bound_tmp *
                                     (static_cast<double>(p_tmp.size()) -
                                      edge_tmp_path_expr));
              }

              for (const auto& lhs_expr : lhs) {
                addLazy(lhs_expr >= rhs);
                if (solver->solution_settings.export_option ==
                        ExportOption::ExportLP ||
                    solver->solution_settings.export_option ==
                        ExportOption::ExportSolutionAndLP ||
                    solver->solution_settings.export_option ==
                        ExportOption::ExportSolutionWithInstanceAndLP) {
                  solver->lazy_constraints.emplace_back(lhs_expr >= rhs);
                }
                violated_constraint_found = true;
              }
            }
          }
        }
      }
    }
  }

  return violated_constraint_found;
}

bool cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::LazyCallback::
    create_lazy_vertex_headway_constraints(
        const std::vector<std::vector<std::pair<size_t, double>>>& routes,
        const std::vector<std::pair<std::vector<std::pair<size_t, bool>>,
                                    std::vector<std::pair<size_t, bool>>>>&
            train_orders_on_edges) {
  bool violated_constraint_found = false;
  bool only_one_constraint =
      solver->solver_strategy.lazy_constraint_selection_strategy ==
      LazyConstraintSelectionStrategy::OnlyFirstFound;
  for (size_t tr = 0; tr < solver->num_tr &&
                      (!only_one_constraint || !violated_constraint_found);
       tr++) {
    const auto tr_t_bound = solver->ub_timing_variable(tr);
    for (size_t r_v_idx = 0;
         r_v_idx < routes.at(tr).size() - 1 &&
         (!only_one_constraint || !violated_constraint_found);
         r_v_idx++) {
      const auto& v_source = routes.at(tr).at(r_v_idx).first;
      const auto& v_target = routes.at(tr).at(r_v_idx + 1).first;
      const auto& edge_index =
          solver->instance.const_n().get_edge_index(v_source, v_target);

      const auto& [rel_tr_order_source, rel_tr_order_target] =
          train_orders_on_edges.at(edge_index);

      for (size_t i = 0;
           i < 2 && (!only_one_constraint || !violated_constraint_found); i++) {
        const auto& rel_tr_oder =
            i == 0 ? rel_tr_order_source : rel_tr_order_target;
        const auto& rel_vertex = i == 0 ? v_source : v_target;
        const auto& rel_vertex_object =
            solver->instance.const_n().get_vertex(rel_vertex);

        const auto tr_idx = std::find(rel_tr_oder.begin(), rel_tr_oder.end(),
                                      std::pair<size_t, bool>(tr, true)) -
                            rel_tr_oder.begin();
        assert(tr_idx < rel_tr_oder.size());
        size_t lb_idx = 0;
        size_t ub_idx = rel_tr_oder.size() - 1;
        if (solver->solver_strategy.lazy_train_selection_strategy ==
            LazyTrainSelectionStrategy::OnlyAdjacent) {
          lb_idx = std::max<int>(static_cast<int>(lb_idx),
                                 static_cast<int>(tr_idx) - 1);
          ub_idx = std::min<int>(static_cast<int>(ub_idx),
                                 static_cast<int>(tr_idx) + 1);
        }
        if (!solver->solver_strategy.include_reverse_headways) {
          lb_idx = std::max<int>(static_cast<int>(lb_idx),
                                 static_cast<int>(tr_idx) + 1);
        }
        if (solver->solver_strategy.lazy_constraint_selection_strategy !=
            LazyConstraintSelectionStrategy::AllChecked) {
          lb_idx = std::max<int>(static_cast<int>(lb_idx),
                                 static_cast<int>(tr_idx) + 1);
        }

        for (size_t edge_order_other_tr_idx = lb_idx;
             edge_order_other_tr_idx <= ub_idx &&
             (!only_one_constraint || !violated_constraint_found);
             edge_order_other_tr_idx++) {
          if (edge_order_other_tr_idx == tr_idx) {
            continue;
          }
          const auto& [other_tr, other_tr_direction] =
              rel_tr_oder.at(edge_order_other_tr_idx);
          if (!other_tr_direction) {
            // The train travels in reverse direction!
            continue;
          }
          auto tr_t_var = solver->vars["t_front_departure"](tr, rel_vertex);
          auto other_tr_t_var =
              solver->vars["t_front_departure"](other_tr, rel_vertex);
          const auto& headway = rel_vertex_object.headway;
          if (solver->solver_strategy.lazy_constraint_selection_strategy ==
                  LazyConstraintSelectionStrategy::AllChecked ||
              getSolution(other_tr_t_var) < getSolution(tr_t_var) + headway) {
            const auto t_bound =
                std::max(tr_t_bound, solver->ub_timing_variable(other_tr)) +
                headway;
            GRBLinExpr lhs =
                other_tr_t_var +
                t_bound * (1 - solver->vars["order"](other_tr, tr, edge_index));
            GRBLinExpr rhs = tr_t_var + headway;
            addLazy(lhs >= rhs);
            if (solver->solution_settings.export_option ==
                    ExportOption::ExportLP ||
                solver->solution_settings.export_option ==
                    ExportOption::ExportSolutionAndLP ||
                solver->solution_settings.export_option ==
                    ExportOption::ExportSolutionWithInstanceAndLP) {
              solver->lazy_constraints.emplace_back(lhs >= rhs);
            }
            violated_constraint_found = true;
          }
        }
      }
    }
  }

  return violated_constraint_found;
}

bool cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::LazyCallback::
    create_lazy_reverse_edge_constraints(
        const std::vector<std::pair<std::vector<std::pair<size_t, bool>>,
                                    std::vector<std::pair<size_t, bool>>>>&
            train_orders_on_edges) {
  bool violated_constraint_found = false;
  bool only_one_constraint =
      solver->solver_strategy.lazy_constraint_selection_strategy ==
      LazyConstraintSelectionStrategy::OnlyFirstFound;

  for (size_t idx = 0; idx < solver->relevant_reverse_edges.size() &&
                       (!only_one_constraint || !violated_constraint_found);
       idx++) {
    const auto& [e1, e2]        = solver->relevant_reverse_edges.at(idx);
    const auto& e_obj           = solver->instance.const_n().get_edge(e1);
    const auto& tr_order_source = train_orders_on_edges.at(e1).first;
    const auto& tr_order_target = train_orders_on_edges.at(e1).second;

    for (size_t i = 0;
         i < 2 && (!only_one_constraint || !violated_constraint_found); i++) {
      const auto& tr_order   = i == 0 ? tr_order_source : tr_order_target;
      const auto& rel_vertex = i == 0 ? e_obj.source : e_obj.target;
      for (size_t tr1_idx = 0;
           tr1_idx < tr_order.size() &&
           (!only_one_constraint || !violated_constraint_found);
           tr1_idx++) {
        const auto& [tr1, tr1_direction] = tr_order.at(tr1_idx);
        const auto& tr1_object =
            solver->instance.get_train_list().get_train(tr1);
        const auto& tr1_name = tr1_object.name;
        const auto& tr1_t_var =
            solver->vars["t_front_departure"](tr1, rel_vertex);
        const auto& tr1_t_var_value = getSolution(tr1_t_var);
        const auto  tr1_t_bound     = solver->ub_timing_variable(tr1);

        size_t lb_idx = 0;
        size_t ub_idx = tr_order.size() - 1;
        if (solver->solver_strategy.lazy_train_selection_strategy ==
            LazyTrainSelectionStrategy::OnlyAdjacent) {
          lb_idx = std::max<int>(static_cast<int>(lb_idx),
                                 static_cast<int>(tr1_idx) - 1);
          ub_idx = std::min<int>(static_cast<int>(ub_idx),
                                 static_cast<int>(tr1_idx) + 1);
        }
        if (!solver->solver_strategy.include_reverse_headways) {
          lb_idx = std::max<int>(static_cast<int>(lb_idx),
                                 static_cast<int>(tr1_idx) + 1);
        }
        if (solver->solver_strategy.lazy_constraint_selection_strategy !=
            LazyConstraintSelectionStrategy::AllChecked) {
          lb_idx = std::max<int>(static_cast<int>(lb_idx),
                                 static_cast<int>(tr1_idx) + 1);
        }

        for (size_t tr2_idx = lb_idx;
             tr2_idx <= ub_idx &&
             (!only_one_constraint || !violated_constraint_found);
             tr2_idx++) {
          if (tr1_idx == tr2_idx) {
            continue;
          }
          const auto& [tr2, tr2_direction] = tr_order.at(tr2_idx);
          if (tr1_direction == tr2_direction) {
            // The trains travel in the same direction!
            continue;
          }
          const auto& tr2_object =
              solver->instance.get_train_list().get_train(tr2);
          const auto& tr2_name = tr2_object.name;
          const auto& tr2_t_var =
              solver->vars["t_rear_departure"](tr2, rel_vertex);
          const auto& tr2_t_var_value = getSolution(tr2_t_var);

          if (solver->solver_strategy.lazy_constraint_selection_strategy ==
                  LazyConstraintSelectionStrategy::AllChecked ||
              tr1_t_var_value < tr2_t_var_value) {
            const auto  tr2_t_bound = solver->ub_timing_variable(tr2);
            const auto  t_bound     = std::max(tr1_t_bound, tr2_t_bound);
            const auto& tr1_edge    = tr1_direction ? e1 : e2;
            const auto& tr2_edge    = tr1_direction ? e1 : e2;

            GRBLinExpr lhs1 = solver->vars["reverse_order"](tr1, tr2, idx) +
                              solver->vars["reverse_order"](tr2, tr1, idx);
            GRBLinExpr rhs1 = solver->vars["x"](tr1, tr1_edge) +
                              solver->vars["x"](tr2, tr2_edge) - 1;

            GRBLinExpr lhs2 =
                tr1_t_var +
                t_bound * (1 - solver->vars["reverse_order"](tr1, tr2, idx));
            GRBLinExpr rhs2 = tr2_t_var;

            addLazy(lhs1 >= rhs1);
            addLazy(lhs1 <= 1);
            addLazy(lhs2 >= rhs2);

            if (solver->solution_settings.export_option ==
                    ExportOption::ExportLP ||
                solver->solution_settings.export_option ==
                    ExportOption::ExportSolutionAndLP ||
                solver->solution_settings.export_option ==
                    ExportOption::ExportSolutionWithInstanceAndLP) {
              solver->lazy_constraints.emplace_back(lhs1 >= rhs1);
              solver->lazy_constraints.emplace_back(lhs1 <= 1);
              solver->lazy_constraints.emplace_back(lhs2 >= rhs2);
            }

            violated_constraint_found = true;
          }
        }
      }
    }
  }
  return violated_constraint_found;
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,performance-inefficient-string-concatenation)
