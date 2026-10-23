#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "VSSModel.hpp"
#include "gurobi_c++.h"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "solver/mip-based/GeneralMIPSolver.hpp"
#include "solver/mip-based/VSSGenTimetableSolver.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <optional>
#include <plog/Log.h>
#include <string>
#include <utility>

using std::size_t;

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,bugprone-unchecked-optional-access)

// NOLINTBEGIN(performance-inefficient-string-concatenation)

cda_rail::instances::SolVSSGeneralPerformanceOptimizationInstance cda_rail::
    solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation::solve(
        const cda_rail::solver::mip_based::ModelDetailMBInformation&
            model_detail_mb_information,
        const cda_rail::solver::mip_based::ModelSettingsVSSGen& model_settings,
        const cda_rail::solver::mip_based::SolverStrategyVSSGen&
            solver_strategy,
        const cda_rail::solver::mip_based::SolutionSettingsVSSGen&
            solution_settings,
        int time_limit, bool debug_input, bool overwrite_severity) {
  if (model_settings.model_type.get_model_type() == vss::ModelType::Discrete) {
    // Not implemented
    throw cda_rail::exceptions::InvalidInputException(
        "Discrete model type is not supported.");
  }

  auto old_instance =
      initialize_variables({model_detail_mb_information.delta_t, true,
                            model_detail_mb_information.train_dynamics,
                            model_detail_mb_information.braking_curves},
                           model_settings, solver_strategy, solution_settings,
                           time_limit, debug_input, overwrite_severity);

  assert(!old_instance.has_value());

  m_fix_orders_on_edges  = model_detail_mb_information.fix_order_on_edges;
  m_fix_stop_positions   = model_detail_mb_information.fix_stop_positions;
  m_fix_exact_positions  = model_detail_mb_information.fix_exact_positions;
  m_fix_exact_velocities = model_detail_mb_information.fix_exact_velocities;
  m_hint_approximate_positions =
      model_detail_mb_information.hint_approximate_positions;

  create_variables();
  set_objective();
  create_constraints();
  include_additional_information();

  set_timeout(time_limit);

  auto sol_object = optimize(old_instance, time_limit);

  export_lp_model_if_applicable(sol_object.value(), solution_settings);

  auto further_data = get_further_data(solution_settings, time_limit);
  further_data.bool_data.insert(
      {{"fix_stop_positions", m_fix_stop_positions},
       {"fix_exact_positions", m_fix_exact_positions},
       {"fix_exact_velocities", m_fix_exact_velocities},
       {"hint_approximate_positions", m_hint_approximate_positions},
       {"fix_order_on_edges", m_fix_orders_on_edges}});
  export_general_solution(sol_object.value(), solution_settings, further_data);

  cleanup();

  return sol_object.value();
}

void cda_rail::solver::mip_based::
    VSSGenTimetableSolverWithMovingBlockInformation::
        include_additional_information() {
  PLOGD << "Including additional information";
  if (m_fix_orders_on_edges) {
    PLOGD << "Fixing orders on edges";
    fix_oder_on_edges();
  }
  if (m_fix_stop_positions) {
    PLOGD << "Fixing stop positions";
    fix_stop_positions_constraints();
  }
  if (m_fix_exact_positions || m_fix_exact_velocities) {
    PLOGD << "Fixing exact positions and/or velocities";
    fix_exact_positions_and_velocities_constraints();
  }
  if (m_hint_approximate_positions) {
    PLOGD << "Hinting approximate positions";
    hint_approximate_positions_constraints();
  }
}

void cda_rail::solver::mip_based::
    VSSGenTimetableSolverWithMovingBlockInformation::
        fix_stop_positions_constraints() {
  const auto& train_list = m_instance.get_const_train_list();
  for (size_t tr = 0; tr < num_tr; ++tr) {
    const auto& tr_obj  = train_list.get_train(tr);
    const auto& tr_name = tr_obj.get_name();
    const auto& tr_len  = tr_obj.get_length();
    for (size_t t_steps = train_interval[tr].first + 1;
         t_steps < train_interval[tr].second; ++t_steps) {
      const auto t = static_cast<double>(t_steps) * dt;
      const auto approx_info =
          m_moving_block_solution.get_approximate_train_pos_and_vel(tr_name, t);
      if (approx_info.has_value()) {
        const auto& [pos_approx, vel_approx] = approx_info.value();
        if (std::abs(vel_approx) < GRB_EPS &&
            std::ranges::any_of(
                m_instance.get_const_schedule(tr_name).get_stops(),
                [&t](const auto& tr_stop) {
                  return tr_stop.get_service_time() <= t &&
                         t <= tr_stop.get_earliest_departure();
                })) {
          // Train is stopping
          m_model->addConstr(m_vars["lda"](tr, t_steps) >=
                                 pos_approx - tr_len - STOP_TOLERANCE,
                             "stop_pos_lb_lda_" + tr_name + "_" +
                                 std::to_string(t));
          m_model->addConstr(m_vars["lda"](tr, t_steps) <= pos_approx - tr_len,
                             "stop_pos_ub_lda_" + tr_name + "_" +
                                 std::to_string(t));
          m_model->addConstr(
              m_vars["mu"](tr, t_steps - 1) >= pos_approx - STOP_TOLERANCE,
              "stop_pos_lb_mu_" + tr_name + "_" + std::to_string(t));
          m_model->addConstr(m_vars["mu"](tr, t_steps - 1) <= pos_approx,
                             "stop_pos_ub_mu_" + tr_name + "_" +
                                 std::to_string(t));
          m_model->addConstr(m_vars["v"](tr, t_steps) == 0,
                             "stop_vel_" + tr_name + "_" + std::to_string(t));
          if (include_braking_curves) {
            m_model->addConstr(m_vars["brakelen"](tr, t_steps - 1) == 0,
                               "stop_brakelen_" + tr_name + "_" +
                                   std::to_string(t));
          }
        }
      }
    }
  }
}

void cda_rail::solver::mip_based::
    VSSGenTimetableSolverWithMovingBlockInformation::
        fix_exact_positions_and_velocities_constraints() {
  const auto& train_list = m_instance.get_const_train_list();
  for (size_t tr = 0; tr < num_tr; tr++) {
    const auto&  tr_obj  = train_list.get_train(tr);
    const auto&  tr_name = tr_obj.get_name();
    const auto&  tr_len  = tr_obj.get_length();
    const double delta_v =
        std::max(tr_obj.get_acceleration(), tr_obj.get_deceleration()) * dt;
    const double delta_pos = tr_obj.get_max_speed() * dt;

    for (size_t t_steps = train_interval[tr].first + 1;
         t_steps <= train_interval[tr].second; t_steps++) {
      const auto t = static_cast<double>(t_steps) * dt;
      const auto bounds =
          m_moving_block_solution.get_exact_pos_and_vel_bounds(tr_name, t);
      const auto& pos_lb = bounds.pos.lb;
      const auto& pos_ub = bounds.pos.ub;
      const auto& vel_lb = bounds.vel.lb;
      const auto& vel_ub = bounds.vel.ub;

      if (m_fix_exact_positions) {
        m_model->addConstr(
            m_vars["lda"](tr, t_steps) >= pos_lb - tr_len - delta_pos,
            "exact_pos_lb_lda_" + tr_name + "_" + std::to_string(t));
        m_model->addConstr(
            m_vars["lda"](tr, t_steps) <= pos_ub - tr_len + delta_pos,
            "exact_pos_ub_lda_" + tr_name + "_" + std::to_string(t));

        GRBLinExpr pos_mu_expr = m_vars["mu"](tr, t_steps - 1);
        if (include_braking_curves) {
          pos_mu_expr -= m_vars["brakelen"](tr, t_steps - 1);
        }
        m_model->addConstr(pos_mu_expr >= pos_lb - delta_pos,
                           "exact_pos_lb_mu_" + tr_name + "_" +
                               std::to_string(t));
        m_model->addConstr(pos_mu_expr <= pos_ub + delta_pos,
                           "exact_pos_ub_mu_" + tr_name + "_" +
                               std::to_string(t));
      }

      if (m_fix_exact_velocities) {
        const auto rel_vel_lb = std::max(vel_lb - delta_v, 0.0);
        const auto rel_vel_ub = vel_ub + delta_v;
        m_model->addConstr(m_vars["v"](tr, t_steps) >= rel_vel_lb,
                           "exact_vel_lb_" + tr_name + "_" + std::to_string(t));
        m_model->addConstr(m_vars["v"](tr, t_steps) <= rel_vel_ub,
                           "exact_vel_ub_" + tr_name + "_" + std::to_string(t));
        if (include_braking_curves) {
          const auto bl_lb =
              rel_vel_lb * rel_vel_lb / (2 * tr_obj.get_deceleration());
          const auto bl_ub =
              rel_vel_ub * rel_vel_ub / (2 * tr_obj.get_deceleration());
          m_model->addConstr(m_vars["brakelen"](tr, t_steps - 1) >= bl_lb,
                             "exact_brakelen_lb_" + tr_name + "_" +
                                 std::to_string(t));
          m_model->addConstr(m_vars["brakelen"](tr, t_steps - 1) <= bl_ub,
                             "exact_brakelen_ub_" + tr_name + "_" +
                                 std::to_string(t));
        }
      }
    }
  }
}

void cda_rail::solver::mip_based::
    VSSGenTimetableSolverWithMovingBlockInformation::
        hint_approximate_positions_constraints() {
  const auto& train_list = m_instance.get_const_train_list();
  for (size_t tr = 0; tr < num_tr; ++tr) {
    const auto& tr_obj  = train_list.get_train(tr);
    const auto& tr_name = tr_obj.get_name();
    const auto& tr_len  = tr_obj.get_length();
    for (size_t t_steps = train_interval[tr].first;
         t_steps <= train_interval[tr].second + 1; ++t_steps) {
      const auto t = static_cast<double>(t_steps) * dt;
      const auto approx_info =
          m_moving_block_solution.get_approximate_train_pos_and_vel(tr_name, t);
      if (approx_info.has_value()) {
        const auto& [pos_approx, vel_approx] = approx_info.value();
        const double bl =
            include_braking_curves
                ? vel_approx * vel_approx / (2 * tr_obj.get_deceleration())
                : 0;
        m_vars["v"](tr, t_steps).set(GRB_DoubleAttr_VarHintVal, vel_approx);
        if (t_steps >= train_interval[tr].first + 1) {
          m_vars["mu"](tr, t_steps - 1)
              .set(GRB_DoubleAttr_VarHintVal, pos_approx + bl);
          if (include_braking_curves) {
            m_vars["brakelen"](tr, t_steps - 1)
                .set(GRB_DoubleAttr_VarHintVal, bl);
          }
        }
        if (t_steps <= train_interval[tr].second) {
          m_vars["lda"](tr, t_steps)
              .set(GRB_DoubleAttr_VarHintVal, pos_approx - tr_len);
        }
      }
    }
  }
}

void cda_rail::solver::mip_based::
    VSSGenTimetableSolverWithMovingBlockInformation::fix_oder_on_edges() {
  // Fixes train order on every breakable edge
  // For this b_front and b_rear are set equal where applicable
  for (size_t i = 0; i < breakable_edges.size(); ++i) {
    const auto& e            = breakable_edges[i];
    const auto& network      = m_instance.get_const_network();
    const auto  vss_number_e = network.max_vss_on_edge(e);
    const auto& edge         = network.get_edge(e);
    const auto& edge_name = "[" + network.get_vertex(edge.source).name + "," +
                            network.get_vertex(edge.target).name + "]";
    const auto  tr_order_on_e = m_moving_block_solution.get_train_order(e);
    for (size_t tr_i = 1; tr_i < tr_order_on_e.size(); tr_i++) {
      const auto& tr_object =
          m_instance.get_const_train_list().get_train(tr_order_on_e.at(tr_i));
      const auto& tr_object_prev = m_instance.get_const_train_list().get_train(
          tr_order_on_e.at(tr_i - 1));
      if (!tr_object_prev.has_tim()) {
        continue;
      }
      for (size_t vss = 0; vss < vss_number_e; ++vss) {
        for (size_t t =
                 std::max(train_interval[tr_order_on_e.at(tr_i)].first,
                          train_interval[tr_order_on_e.at(tr_i - 1)].first);
             t <= train_interval[tr_order_on_e.at(tr_i)].second &&
             t <= train_interval[tr_order_on_e.at(tr_i - 1)].second;
             ++t) {
          m_model->addConstr(
              m_vars["b_front"](tr_order_on_e.at(tr_i), t, i, vss) ==
                  m_vars["b_rear"](tr_order_on_e.at(tr_i - 1), t, i, vss),
              "fix_order_" + tr_object_prev.get_name() + "_" +
                  tr_object.get_name() + "_" +
                  std::to_string(static_cast<double>(t) * dt) + "_" +
                  edge_name + "_" + std::to_string(vss));
        }
      }
    }
  }

  // Additional fix order on every edge: the following train may only enter the
  // edge once the rear of the previous train has passed its start. The front of
  // a train has passed the start of an edge if x_mu of that edge is 1, the rear
  // has passed it if x_lda is 0 for the edge of the route ending there, i.e.,
  // the edge before e or, in the opposite direction, the reverse edge. Together
  // with the separation constraints, this also fixes the order in which the
  // trains leave the edge.
  for (size_t e = 0; e < num_edges; ++e) {
    const auto& network   = m_instance.get_const_network();
    const auto& edge_obj  = network.get_edge(e);
    const auto& edge_name = "[" + network.get_vertex(edge_obj.source).name +
                            "," + network.get_vertex(edge_obj.target).name +
                            "]";
    const auto  tr_order_on_e =
        m_moving_block_solution.get_train_order_with_reverse(e);
    for (size_t tr_i = 1; tr_i < tr_order_on_e.size(); tr_i++) {
      // Fix order on edge e
      const auto& [tr_following, tr_following_direction] =
          tr_order_on_e.at(tr_i);
      if (!tr_following_direction) {
        // Prevent double counting
        continue;
      }
      const auto& tr_following_obj =
          m_instance.get_const_train_list().get_train(tr_following);

      const auto  tr_prev = tr_order_on_e.at(tr_i - 1).train_id;
      const auto& tr_prev_obj =
          m_instance.get_const_train_list().get_train(tr_prev);

      const auto& tr_following_interval = train_interval[tr_following];
      const auto& tr_prev_interval      = train_interval[tr_prev];

      const auto& prev_edges = m_instance.get_const_routes()
                                   .get_route(tr_prev_obj.get_name())
                                   .get_edges();
      const auto  prev_rear_edge =
          std::ranges::find_if(prev_edges, [&](size_t prev_edge) {
            return network.get_edge(prev_edge).target == edge_obj.source;
          });
      if (prev_rear_edge == prev_edges.end()) {
        // e starts at the entry vertex of tr_prev, see common_entry
        continue;
      }

      for (size_t t_idx =
               std::min(tr_prev_interval.first, tr_following_interval.first);
           t_idx <=
           std::max(tr_prev_interval.second, tr_following_interval.second);
           ++t_idx) {
        const auto t = static_cast<double>(t_idx) * dt;
        const bool prev_present =
            t_idx >= tr_prev_interval.first && t_idx <= tr_prev_interval.second;
        const bool following_present = t_idx >= tr_following_interval.first &&
                                       t_idx <= tr_following_interval.second;
        const auto name = "fix_order_entry_" + tr_prev_obj.get_name() + "_" +
                          tr_following_obj.get_name() + "_" +
                          std::to_string(t) + "_" + edge_name;

        // Before tr_prev enters, its rear has not passed the start of e. After
        // tr_following has left, its front has.
        if (prev_present && following_present) {
          m_model->addConstr(
              m_vars["x_mu"](tr_following, t_idx, e) +
                      m_vars["x_lda"](tr_prev, t_idx, *prev_rear_edge) <=
                  1,
              name);
        } else if (following_present && t_idx < tr_prev_interval.first) {
          m_model->addConstr(m_vars["x_mu"](tr_following, t_idx, e) <= 0, name);
        } else if (prev_present && t_idx > tr_following_interval.second) {
          m_model->addConstr(
              m_vars["x_lda"](tr_prev, t_idx, *prev_rear_edge) <= 0, name);
        }
      }
    }
  }
}

void cda_rail::solver::mip_based::
    VSSGenTimetableSolverWithMovingBlockInformation::cleanup() {
  VSSGenTimetableSolver::cleanup();
  m_fix_orders_on_edges        = true;
  m_fix_stop_positions         = true;
  m_fix_exact_positions        = true;
  m_fix_exact_velocities       = true;
  m_hint_approximate_positions = true;
}

// NOLINTEND(performance-inefficient-string-concatenation)

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,bugprone-unchecked-optional-access)
