#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "MultiArray.hpp"
#include "gurobi_c++.h"
#include "gurobi_c.h"
#include "solver/mip-based/VSSGenTimetableSolver.hpp"

#include <cmath>
#include <cstddef>
#include <string>

// NOLINTBEGIN(performance-inefficient-string-concatenation,bugprone-unchecked-optional-access)

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_fixed_routes_variables() {
  /**
   * Creates variables connected to the fixed route version of the problem
   */

  vars["lda"]   = MultiArray<GRBVar>(num_tr, num_t);
  vars["mu"]    = MultiArray<GRBVar>(num_tr, num_t);
  vars["x_lda"] = MultiArray<GRBVar>(num_tr, num_t, num_edges);
  vars["x_mu"]  = MultiArray<GRBVar>(num_tr, num_t, num_edges);

  const auto& train_list = instance.get_train_list();
  for (size_t tr = 0; tr < num_tr; ++tr) {
    const auto tr_name = train_list.get_train(tr).name;
    const auto r_len   = instance.route_length(tr_name);
    const auto tr_len  = train_list.get_train(tr_name).length;
    double     mu_ub   = r_len + tr_len;
    if (this->include_braking_curves) {
      mu_ub += get_max_brakelen(tr);
    }
    for (size_t t_steps = train_interval[tr].first;
         t_steps <= train_interval[tr].second; ++t_steps) {
      auto t = t_steps * dt;
      vars["mu"](tr, t_steps) =
          model->addVar(0, mu_ub, 0, GRB_CONTINUOUS,
                        "mu_" + tr_name + "_" + std::to_string(t));
      vars["lda"](tr, t_steps) =
          model->addVar(-tr_len, r_len, 0, GRB_CONTINUOUS,
                        "lda_" + tr_name + "_" + std::to_string(t));
      for (auto const edge_id :
           instance.edges_used_by_train(tr_name, fix_routes)) {
        const auto& edge = instance.n().get_edge(edge_id);
        const auto& edge_name =
            "[" + instance.n().get_vertex(edge.source).name + "," +
            instance.n().get_vertex(edge.target).name + "]";
        vars["x_lda"](tr, t_steps, edge_id) = model->addVar(
            0, 1, 0, GRB_BINARY,
            "x_lda_" + tr_name + "_" + std::to_string(t) + "_" + edge_name);
        vars["x_mu"](tr, t_steps, edge_id) = model->addVar(
            0, 1, 0, GRB_BINARY,
            "x_mu_" + tr_name + "_" + std::to_string(t) + "_" + edge_name);
      }
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_fixed_routes_constraints() {
  /**
   * These constraints appear only when routes are fixed
   */

  create_fixed_routes_position_constraints();
  create_boundary_fixed_routes_constraints();
  create_fixed_routes_occupation_constraints();
  create_fixed_route_schedule_constraints();
  create_fixed_routes_no_overlap_entry_exit_constraints();
  if (this->use_schedule_cuts) {
    create_fixed_routes_impossibility_cuts();
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_fixed_routes_position_constraints() {
  /**
   * Creates constraints that ensure that the trains move according to their
   * fixed routes.
   */

  auto train_list = instance.get_train_list();
  for (size_t tr = 0; tr < num_tr; ++tr) {
    auto tr_name = train_list.get_train(tr).name;
    auto tr_len  = instance.get_train_list().get_train(tr_name).length;
    for (size_t t = train_interval[tr].first;
         t <= train_interval[tr].second - 1; ++t) {
      // full pos: mu - lda = len + (v(t) + v(t+1))/2 * dt + brakelen (if
      // applicable)
      GRBLinExpr rhs =
          tr_len + (vars["v"](tr, t) + vars["v"](tr, t + 1)) * dt / 2;
      if (this->include_braking_curves) {
        rhs += vars["brakelen"](tr, t);
      }
      model->addConstr(vars["mu"](tr, t) - vars["lda"](tr, t) == rhs,
                       "full_pos_" + tr_name + "_" + std::to_string(t));
      // overlap: mu(t) - lda(t+1) = len + brakelen (if applicable)
      rhs = tr_len;
      if (this->include_braking_curves) {
        rhs += vars["brakelen"](tr, t);
      }
      model->addConstr(vars["mu"](tr, t) - vars["lda"](tr, t + 1) == rhs,
                       "overlap_" + tr_name + "_" + std::to_string(t));
      // mu increasing: mu(t+1) >= mu(t)
      model->addConstr(vars["mu"](tr, t + 1) >= vars["mu"](tr, t),
                       "mu_increasing_" + tr_name + "_" + std::to_string(t));
      // lda increasing: lda(t+1) >= lda(t)
      model->addConstr(vars["lda"](tr, t + 1) >= vars["lda"](tr, t),
                       "lda_increasing_" + tr_name + "_" + std::to_string(t));
    }
    // full pos also holds for t = train_interval[i].second
    auto       t = train_interval[tr].second;
    GRBLinExpr rhs =
        tr_len + (vars["v"](tr, t) + vars["v"](tr, t + 1)) * dt / 2;
    if (this->include_braking_curves) {
      rhs += vars["brakelen"](tr, t);
    }
    model->addConstr(vars["mu"](tr, t) - vars["lda"](tr, t) == rhs,
                     "full_pos_" + tr_name + "_" + std::to_string(t));
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_boundary_fixed_routes_constraints() {
  /**
   * Create boundary conditions for the fixed routes of the trains
   */

  auto train_list = instance.get_train_list();
  for (size_t i = 0; i < num_tr; ++i) {
    auto tr_name = train_list.get_train(i).name;
    auto r_len   = instance.route_length(tr_name);
    auto tr_len  = instance.get_train_list().get_train(tr_name).length;
    // initial_lda: lda(train_interval[i].first) = - tr_len
    model->addConstr(vars["lda"](i, train_interval[i].first) == -tr_len,
                     "initial_lda_" + tr_name);
    // final_mu: mu(train_interval[i].second) = r_len + tr_len + brakelen (if
    // applicable)
    GRBLinExpr rhs = r_len + tr_len;
    if (this->include_braking_curves) {
      rhs += vars["brakelen"](i, train_interval[i].second);
    }
    model->addConstr(vars["mu"](i, train_interval[i].second) == rhs,
                     "final_mu_" + tr_name);
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_fixed_routes_occupation_constraints() {
  /**
   * Create constraints for edge occupation of trains with fixed routes.
   */

  // Iterate over all trains
  const auto& train_list = instance.get_train_list();
  for (size_t tr = 0; tr < train_list.size(); ++tr) {
    const auto  tr_name  = train_list.get_train(tr).name;
    const auto  r_len    = instance.route_length(tr_name);
    const auto& tr_route = instance.get_route(tr_name);
    const auto  r_size   = tr_route.size();
    const auto  tr_len   = instance.get_train_list().get_train(tr_name).length;

    double mu_ub = r_len + tr_len;
    if (this->include_braking_curves) {
      mu_ub += get_max_brakelen(tr);
    }

    // Iterate over all edges
    for (size_t j = 0; j < r_size; ++j) {
      const auto edge_id  = tr_route.get_edge(j);
      const auto edge_pos = instance.route_edge_pos(tr_name, edge_id);
      // Iterate over possible time steps
      for (size_t t = train_interval[tr].first; t <= train_interval[tr].second;
           ++t) {
        // x_mu(tr, t, edge_id) = 1 if, and only if, mu(tr,t) > edge_pos.first
        model->addConstr(mu_ub * vars["x_mu"](tr, t, edge_id) >=
                             (vars["mu"](tr, t) - edge_pos.first),
                         "x_mu_if_" + tr_name + "_" + std::to_string(t) + "_" +
                             std::to_string(edge_id));
        model->addConstr(r_len * vars["x_mu"](tr, t, edge_id) <=
                             r_len + vars["mu"](tr, t) - edge_pos.first,
                         "x_mu_only_if_" + tr_name + "_" + std::to_string(t) +
                             "_" + std::to_string(edge_id));

        // x_lda = 1 if, and only if, lda < edge_pos.second
        model->addConstr((r_len + tr_len) * vars["x_lda"](tr, t, edge_id) >=
                             edge_pos.second - vars["lda"](tr, t),
                         "x_lda_if_" + tr_name + "_" + std::to_string(t) + "_" +
                             std::to_string(edge_id));
        model->addConstr(r_len * vars["x_lda"](tr, t, edge_id) <=
                             r_len + edge_pos.second - vars["lda"](tr, t),
                         "x_lda_only_if_" + tr_name + "_" + std::to_string(t) +
                             "_" + std::to_string(edge_id));

        // x = x_lda AND x_mu
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays)
        GRBVar clause[2];
        clause[0] = vars["x_lda"](tr, t, edge_id);
        clause[1] = vars["x_mu"](tr, t, edge_id);
        // NOLINTBEGIN(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
        model->addGenConstrAnd(vars["x"](tr, t, edge_id), clause, 2,
                               "x_" + tr_name + "_" + std::to_string(t) + "_" +
                                   std::to_string(edge_id));
        // NOLINTEND(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
      }
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_fixed_route_schedule_constraints() {
  /**
   * Constrain lambda and mu for fixed routes in stations.
   */

  // Iterate over all trains
  const auto& train_list = instance.get_train_list();
  for (size_t tr = 0; tr < train_list.size(); ++tr) {
    const auto  tr_name     = train_list.get_train(tr).name;
    const auto& tr_schedule = instance.get_schedule(tr_name);
    for (const auto& tr_stop : tr_schedule.get_stops()) {
      const auto  t0 = tr_stop.arrival() / dt;
      const auto  t1 = std::ceil(static_cast<double>(tr_stop.departure()) / dt);
      const auto& stop_edges = instance.get_station_list()
                                   .get_station(tr_stop.get_station_name())
                                   .tracks;
      const auto& stop_pos = instance.route_edge_pos(tr_name, stop_edges);
      // Other cases follow by increasing of lambda and mu
      model->addConstr(vars["mu"](tr, t0 - 1) >= stop_pos.first,
                       "mu_station_min_" + tr_name + "_" +
                           std::to_string(t0 - 1)); // entering station
      model->addConstr(vars["mu"](tr, t1 - 1) <= stop_pos.second,
                       "mu_station_max_" + tr_name + "_" +
                           std::to_string(t1 - 1)); // last before leaving
      model->addConstr(vars["lda"](tr, t0) >= stop_pos.first,
                       "lda_station_min_" + tr_name + "_" +
                           std::to_string(t0)); // first after entering
      model->addConstr(vars["lda"](tr, t1) <= stop_pos.second,
                       "lda_station_max_" + tr_name + "_" +
                           std::to_string(t1)); // leaving station
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_fixed_routes_impossibility_cuts() {
  /**
   * Cuts off solutions that are not possible in any way.
   */

  // Iterate over all trains
  const auto& train_list = instance.get_train_list();
  for (size_t tr = 0; tr < train_list.size(); ++tr) {
    const auto tr_name = train_list.get_train(tr).name;
    for (size_t t = train_interval[tr].first; t <= train_interval[tr].second;
         ++t) {
      const auto before_after_struct =
          get_temporary_impossibility_struct(tr, t);
      if (!before_after_struct.to_use) {
        continue;
      }

      // Before and after position
      double before_max = NAN;
      double after_min  = NAN;
      if (before_after_struct.t_before <= train_interval[tr].first) {
        before_max = 0;
      } else {
        before_max =
            instance.route_edge_pos(tr_name, before_after_struct.edges_before)
                .second;
      }
      if (before_after_struct.t_after >= train_interval[tr].second) {
        after_min = instance.route_length(tr_name);
      } else {
        after_min =
            instance.route_edge_pos(tr_name, before_after_struct.edges_after)
                .first;
      }

      // Constraint inferred from before position
      auto t_steps        = t - before_after_struct.t_before + 1;
      auto dist_travelled = max_distance_travelled(
          tr, t_steps, before_after_struct.v_before,
          train_list.get_train(tr).acceleration, this->include_braking_curves);
      // mu <= before_max + dist_travelled
      model->addConstr(vars["mu"](tr, t), GRB_LESS_EQUAL,
                       before_max + dist_travelled,
                       "mu_cut_" + tr_name + "_" + std::to_string(t));

      // Constraint inferred from after position
      t_steps = before_after_struct.t_after - t;
      dist_travelled =
          max_distance_travelled(tr, t_steps, before_after_struct.v_after,
                                 train_list.get_train(tr).deceleration, false);
      // lda >= after_min - dist_travelled
      model->addConstr(vars["lda"](tr, t), GRB_GREATER_EQUAL,
                       after_min - dist_travelled,
                       "lda_cut_" + tr_name + "_" + std::to_string(t));
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_non_discretized_fixed_route_constraints() {
  /**
   * Creates non discretized vss constraints if routes are fixed
   */

  for (size_t tr = 0; tr < num_tr; ++tr) {
    const auto& tr_name = instance.get_train_list().get_train(tr).name;
    const auto  r_len   = instance.route_length(tr_name);
    const auto& tr_len  = instance.get_train_list().get_train(tr).length;
    double      mu_ub   = r_len + tr_len;
    if (this->include_braking_curves) {
      mu_ub += get_max_brakelen(tr);
    }
    for (const auto e : instance.edges_used_by_train(tr, this->fix_routes)) {
      const auto& e_index      = breakable_edge_indices[e];
      const auto& e_len        = instance.n().get_edge(e).length;
      const auto  vss_number_e = instance.n().max_vss_on_edge(e);
      const auto  edge_pos     = instance.route_edge_pos(tr_name, e);
      for (size_t t = train_interval[tr].first; t <= train_interval[tr].second;
           ++t) {
        for (size_t vss = 0; vss < vss_number_e; ++vss) {
          // mu(tr, t) - edge_pos.first <= b_pos(e_index, vss) + mu_ub * (1 -
          // b_front(tr, t, e_index, vss))

          // lda(tr, t) - edge_pos.first + (r_len + tr_len + e_len) * (1 -
          // b_rear(tr, t, e_index, vss)) >= b_pos(e_index, vss)
          const auto m1 = mu_ub;
          model->addConstr(vars["mu"](tr, t) - edge_pos.first, GRB_LESS_EQUAL,
                           vars["b_pos"](e_index, vss) +
                               m1 * (1 - vars["b_front"](tr, t, e_index, vss)),
                           "b_pos_front_" + std::to_string(tr) + "_" +
                               std::to_string(t) + "_" + std::to_string(e) +
                               "_" + std::to_string(vss));
          if (instance.get_train_list().get_train(tr).tim) {
            const auto m2 = r_len + tr_len + e_len;
            model->addConstr(vars["lda"](tr, t) - edge_pos.first +
                                 m2 * (1 - vars["b_rear"](tr, t, e_index, vss)),
                             GRB_GREATER_EQUAL, vars["b_pos"](e_index, vss),
                             "b_pos_rear_" + std::to_string(tr) + "_" +
                                 std::to_string(t) + "_" + std::to_string(e) +
                                 "_" + std::to_string(vss));
          }
        }
      }
    }
  }

  if (vss_model.get_only_stop_at_vss()) {
    create_non_discretized_fixed_routes_only_stop_at_vss_constraints();
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_fixed_routes_no_overlap_entry_exit_constraints() {
  /**
   * Create constraints on common entry and exit points.
   */

  const auto train_entry_exit_pairs = common_entry_exit_vertices();

  // If two trains share an entry vertex, then the first train must have left
  // before the second train enters
  for (const auto& tr_list : train_entry_exit_pairs.first) {
    for (size_t i = 0; i < tr_list.size() - 1; ++i) {
      const auto& tr1_entry = train_interval[tr_list[i]].first;
      const auto& tr2_entry = train_interval[tr_list[i + 1]].first;
      if (tr1_entry >= tr2_entry) {
        throw exceptions::ModelCreationException(
            "Something went wrong with trains " + std::to_string(tr_list[i]) +
            " and " + std::to_string(tr_list[i + 1]) + " at common entry");
      }
      for (size_t t = tr2_entry; t < train_interval[tr_list[i]].second; ++t) {
        // lda(tr1, t) >= 0
        model->addConstr(vars["lda"](tr_list[i], t), GRB_GREATER_EQUAL, 0,
                         "common_entry_" + std::to_string(tr_list[i]) + "_" +
                             std::to_string(tr_list[i + 1]) + "_" +
                             std::to_string(t));
      }
    }
  }

  // If two trains share an exit vertex, then the first train must have left
  // before the second train enters
  for (const auto& tr_list : train_entry_exit_pairs.second) {
    for (size_t i = 0; i < tr_list.size() - 1; ++i) {
      const auto& tr1_exit = train_interval[tr_list[i]].second;
      const auto& tr2_exit = train_interval[tr_list[i + 1]].second;
      if (tr1_exit <= tr2_exit) {
        throw exceptions::ModelCreationException(
            "Something went wrong with trains " + std::to_string(tr_list[i]) +
            " and " + std::to_string(tr_list[i + 1]) + " at common exit");
      }
      const auto& tr1_name =
          instance.get_train_list().get_train(tr_list[i]).name;
      const auto& tr1_route_length = instance.route_length(tr1_name);
      for (size_t t = train_interval[tr_list[i]].first; t <= tr2_exit; ++t) {
        // mu(tr1, t) <= tr1_route_length
        model->addConstr(
            vars["mu"](tr_list[i], t), GRB_LESS_EQUAL, tr1_route_length,
            "common_exit_" + std::to_string(tr_list[i]) + "_" +
                std::to_string(tr_list[i + 1]) + "_" + std::to_string(t));
      }
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_non_discretized_fixed_routes_only_stop_at_vss_constraints() {
  // For every breakable edge position exactly b_pos if tight
  for (size_t i = 0; i < breakable_edges.size(); ++i) {
    const auto& e            = breakable_edges[i];
    const auto  vss_number_e = instance.n().max_vss_on_edge(e);
    const auto& edge         = instance.n().get_edge(e);
    const auto& edge_name    = "[" + instance.n().get_vertex(edge.source).name +
                            "," + instance.n().get_vertex(edge.target).name +
                            "]";
    for (size_t vss = 0; vss < vss_number_e; ++vss) {
      for (const auto tr : instance.trains_on_edge(e, this->fix_routes)) {
        const auto& tr_name  = instance.get_train_list().get_train(tr).name;
        const auto  edge_pos = instance.route_edge_pos(tr_name, e);
        const auto  r_len    = instance.route_length(tr_name);
        const auto& tr_len   = instance.get_train_list().get_train(tr).length;
        double      mu_ub    = r_len + tr_len;
        if (this->include_braking_curves) {
          mu_ub += get_max_brakelen(tr);
        }
        for (size_t t = train_interval[tr].first + 2;
             t <= train_interval[tr].second; ++t) {
          model->addConstr(vars["mu"](tr, t - 1) - edge_pos.first,
                           GRB_GREATER_EQUAL,
                           vars["b_pos"](i, vss) - STOP_TOLERANCE -
                               r_len * (1 - vars["b_tight"](tr, t, i, vss)),
                           "tight_vss_border_constraint_1_" + tr_name + "_" +
                               std::to_string(t * dt) + "_" + edge_name + "_" +
                               std::to_string(vss));
          model->addConstr(vars["mu"](tr, t - 1) - edge_pos.first,
                           GRB_LESS_EQUAL,
                           vars["b_pos"](i, vss) +
                               mu_ub * (1 - vars["b_tight"](tr, t, i, vss)),
                           "tight_vss_border_constraint_2_" + tr_name + "_" +
                               std::to_string(t * dt) + "_" + edge_name + "_" +
                               std::to_string(vss));
        }
      }
    }
  }

  // Analog for every edge ending
  for (size_t e = 0; e < num_edges; ++e) {
    const auto& edge      = instance.n().get_edge(e);
    const auto& edge_name = "[" + instance.n().get_vertex(edge.source).name +
                            "," + instance.n().get_vertex(edge.target).name +
                            "]";
    for (const auto tr : instance.trains_on_edge(e, this->fix_routes)) {
      const auto& tr_name  = instance.get_train_list().get_train(tr).name;
      const auto  edge_pos = instance.route_edge_pos(tr_name, e);
      const auto  r_len    = instance.route_length(tr_name);
      for (size_t t = train_interval[tr].first + 2;
           t <= train_interval[tr].second; ++t) {
        model->addConstr(vars["mu"](tr, t - 1), GRB_GREATER_EQUAL,
                         edge_pos.second - STOP_TOLERANCE -
                             r_len * (1 - vars["e_tight"](tr, t, e)),
                         "tight_ttd_border_constraint_" + tr_name + "_" +
                             std::to_string(t * dt) + "_" + edge_name);
      }
    }
  }

  // Cannot stop after route end
  for (size_t tr = 0; tr < num_tr; ++tr) {
    const auto& tr_object    = instance.get_train_list().get_train(tr);
    const auto& tr_name      = tr_object.name;
    const auto  r_len        = instance.route_length(tr_name);
    const auto& tr_len       = tr_object.length;
    const auto  max_brakelen = get_max_brakelen(tr);
    for (size_t t = train_interval[tr].first + 2;
         t <= train_interval[tr].second; ++t) {
      model->addConstr(vars["mu"](tr, t - 1), GRB_LESS_EQUAL,
                       r_len + (tr_len + max_brakelen) * vars["stopped"](tr, t),
                       "len_out_tight_if_stopped_" + tr_name + "_" +
                           std::to_string(t * dt));
    }
  }
}

// NOLINTEND(performance-inefficient-string-concatenation,bugprone-unchecked-optional-access)
